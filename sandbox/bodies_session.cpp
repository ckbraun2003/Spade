// See bodies_session.hpp.

#include "bodies_session.hpp"

#include <chrono>
#include <cmath>
#include <cstdio>
#include <utility>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>

#include "render/raster_cpu.hpp"
#include "unit_meshes.hpp"

namespace spade::sandbox {

namespace {

[[nodiscard]] float ms_since(std::chrono::steady_clock::time_point t0) {
    return std::chrono::duration<float, std::milli>(std::chrono::steady_clock::now() - t0).count();
}

[[nodiscard]] render::Material lambert(float r, float g, float b) {
    render::Material m;
    m.base_color = glm::vec4(r, g, b, 1.0f);
    m.shading = 0u;
    return m;
}

[[nodiscard]] glm::mat4 pose_of(const BodyState& b) {
    return glm::translate(glm::mat4(1.0f), b.pos) * glm::mat4_cast(b.orient);
}

}  // namespace

Result<std::unique_ptr<BodiesSession>> BodiesSession::create() {
    std::unique_ptr<BodiesSession> s(new BodiesSession());
    Result<bodies::BodiesSim> sim = bodies::BodiesSim::create();
    if (!sim) return std::unexpected(Error{sim.error().code, "the bodies would not start: " + sim.error().context});
    s->sim_ = std::make_unique<bodies::BodiesSim>(std::move(*sim));

    // The ground: a plane, drawn by the renderer's SDF path (the world outlives the scene that borrows it).
    WorldBuilder builder;
    builder.name("bodies_ground").environment(Environment{}).capacities(Capacities{1, 1, 1, 1});
    builder.plane(glm::vec3(0.0f, 1.0f, 0.0f), 0.0f);
    Result<WorldDesc> world = builder.build();
    if (!world) return std::unexpected(Error{world.error().code, "the ground would not build: " + world.error().context});
    s->world_ = std::move(*world);
    const Result<render::RenderScene> scene = render::scene_from_world(s->world_, {});
    if (!scene) return std::unexpected(Error{scene.error().code, "the scene would not build: " + scene.error().context});
    s->scene_ = *scene;

    s->box_mesh_ = static_cast<uint32_t>(s->scene_.meshes.size());
    s->scene_.meshes.push_back(make_box_mesh());  // unit: half 0.5
    s->sphere_mesh_ = static_cast<uint32_t>(s->scene_.meshes.size());
    s->scene_.meshes.push_back(make_sphere_mesh());  // radius 0.5
    s->cylinder_mesh_ = static_cast<uint32_t>(s->scene_.meshes.size());
    s->scene_.meshes.push_back(make_cylinder_mesh());  // radius 0.5, half height 0.5, along y
    const auto material = [&](float r, float g, float b) {
        const auto at = static_cast<uint32_t>(s->scene_.materials.size());
        s->scene_.materials.push_back(lambert(r, g, b));
        return at;
    };
    s->sphere_mat_ = material(0.92f, 0.56f, 0.14f);   // orange
    s->cube_mat_ = material(0.29f, 0.56f, 0.89f);     // the theme's blue
    s->capsule_mat_ = material(0.22f, 0.70f, 0.42f);  // green
    s->hull_mat_ = material(0.35f, 0.37f, 0.40f);     // the drone's grey
    s->rotor_mat_ = material(0.85f, 0.15f, 0.10f);    // the nose's red

    // The row spans about 4 m of x: seen from a little above and ahead.
    s->camera_.target = glm::vec3(-0.1f, 0.15f, 0.0f);
    s->camera_.yaw = 0.0f;
    s->camera_.pitch = 0.32f;
    s->camera_.distance = 5.2f;
    s->draw_items();
    return s;
}

void BodiesSession::attach(GlTargetSink& sink) {
    sink_ = &sink;
    gpu_ = start_gpu(scene_);  // the meshes and the palette never change; only the items do
}

BodiesSession::~BodiesSession() {
    if (sink_ != nullptr) sink_->set_status_line({});
}

void BodiesSession::draw_items() {
    scene_.dynamics.clear();
    const auto add = [&](uint32_t mesh, uint32_t mat, const glm::mat4& local_to_world) {
        render::DrawItem item;
        item.mesh_index = mesh;
        item.local_to_world = local_to_world;
        item.material_override = mat;
        scene_.dynamics.push_back(item);
    };
    const glm::mat4 quarter_turn_z = glm::rotate(glm::mat4(1.0f), glm::half_pi<float>(), glm::vec3(0.0f, 0.0f, 1.0f));
    const std::vector<bodies::Piece>& pieces = sim_->pieces();
    for (std::size_t i = 0; i < pieces.size(); ++i) {
        const glm::mat4 pose = pose_of(sim_->state(i));
        switch (pieces[i].kind) {
            case bodies::Kind::sphere:
                add(sphere_mesh_, sphere_mat_, pose * glm::scale(glm::mat4(1.0f), glm::vec3(2.0f * bodies::kSphereRadius)));
                break;
            case bodies::Kind::box:
                add(box_mesh_, cube_mat_, pose * glm::scale(glm::mat4(1.0f), glm::vec3(2.0f * bodies::kCubeHalf)));
                break;
            case bodies::Kind::capsule: {
                // A cylinder along the body's x axis (the cylinder mesh stands along y) and a sphere on each end.
                const float r = 2.0f * bodies::kCapsuleRadius;
                const float len = 2.0f * bodies::kCapsuleHalfLength;
                add(cylinder_mesh_, capsule_mat_, pose * quarter_turn_z * glm::scale(glm::mat4(1.0f), glm::vec3(r, len, r)));
                for (const float end : {-bodies::kCapsuleHalfLength, bodies::kCapsuleHalfLength}) {
                    add(sphere_mesh_, capsule_mat_,
                        pose * glm::translate(glm::mat4(1.0f), glm::vec3(end, 0.0f, 0.0f)) *
                            glm::scale(glm::mat4(1.0f), glm::vec3(r)));
                }
                break;
            }
            case bodies::Kind::hull: {
                // A drone: the body, two crossed arms out to the feet, a foot under each arm tip and a rotor disc over it.
                add(box_mesh_, hull_mat_, pose * glm::scale(glm::mat4(1.0f), glm::vec3(0.16f, 2.0f * bodies::kHullFoot, 0.16f)));
                for (const float turn : {glm::quarter_pi<float>(), -glm::quarter_pi<float>()}) {
                    add(box_mesh_, hull_mat_,
                        pose * glm::rotate(glm::mat4(1.0f), turn, glm::vec3(0.0f, 1.0f, 0.0f)) *
                            glm::scale(glm::mat4(1.0f), glm::vec3(0.45f, 0.02f, 0.02f)));
                }
                for (const glm::vec3 foot : bodies::drone_hull_points()) {
                    if (foot.y > -bodies::kHullFoot + 1e-4f || std::abs(foot.x) < 0.2f) continue;  // the four tips only
                    add(sphere_mesh_, hull_mat_,
                        pose * glm::translate(glm::mat4(1.0f), foot + glm::vec3(0.0f, 0.02f, 0.0f)) *
                            glm::scale(glm::mat4(1.0f), glm::vec3(0.04f)));
                    add(cylinder_mesh_, rotor_mat_,
                        pose * glm::translate(glm::mat4(1.0f), glm::vec3(foot.x, 0.025f, foot.z)) *
                            glm::scale(glm::mat4(1.0f), glm::vec3(0.18f, 0.008f, 0.18f)));
                }
                break;
            }
        }
    }
}

Result<void> BodiesSession::frame(float dt) {
    const auto p0 = std::chrono::steady_clock::now();
    if (const Result<void> stepped = sim_->advance(dt); !stepped) {
        return std::unexpected(Error{stepped.error().code, "the bodies' step failed: " + stepped.error().context});
    }
    const float physics_ms = ms_since(p0);
    draw_items();

    const uint32_t fw = sink_->viewport().width;
    const uint32_t fh = sink_->viewport().height;
    if (fw == 0u || fh == 0u) return {};  // minimised: the sim keeps stepping
    const render::Camera rc = camera_.to_render_camera();
    render::RenderOptions options;
    options.ground_grid = true;
    options.overlays = false;
    const auto r0 = std::chrono::steady_clock::now();
    if (gpu_) {
        sink_->begin_gpu_frame();
        if (const Result<void> drew = gpu_->draw(scene_, rc, options, fw, fh); !drew) {
            return std::unexpected(Error{drew.error().code, "GPU draw failed: " + drew.error().context});
        }
        sink_->present_overlay(ms_since(r0), physics_ms);
    } else {
        sink_->note_physics_ms(physics_ms);
        if (!render_frame(scene_, rc, fw, fh, options, pixels_, *sink_)) {
            return std::unexpected(Error{Code::internal, "the CPU render failed (see above)"});
        }
        sink_->note_render_ms(ms_since(r0));
    }
    return {};
}

}  // namespace spade::sandbox
