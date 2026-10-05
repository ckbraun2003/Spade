// See scene_file_session.hpp.

#include "scene_file_session.hpp"

#include <array>
#include <chrono>
#include <cmath>
#include <string>
#include <utility>

#include <glm/gtc/matrix_transform.hpp>

#include "builder_scene.hpp"   // the unit box, sphere and cylinder meshes
#include "drone_view.hpp"      // detail::lambert(), the drone's part dimensions
#include "gl_target_sink.hpp"
#include "render_gap.hpp"

namespace spade::sandbox {
namespace {

[[nodiscard]] float ms_since(std::chrono::steady_clock::time_point t0) {
    return std::chrono::duration<float, std::milli>(std::chrono::steady_clock::now() - t0).count();
}

// One colour per lane, so a many-lane scene tells its lanes apart.
constexpr std::array<glm::vec3, 4> kLaneColours = {glm::vec3(0.95f, 0.55f, 0.25f), glm::vec3(0.35f, 0.75f, 0.40f),
                                                    glm::vec3(0.30f, 0.55f, 0.95f), glm::vec3(0.75f, 0.40f, 0.85f)};

}  // namespace

spade::Result<std::unique_ptr<SceneFileSession>> SceneFileSession::create(std::vector<Lane> lanes) {
    if (lanes.empty()) {
        return std::unexpected(spade::Error{spade::Code::invalid_argument, "a scene session needs a scene file"});
    }
    std::unique_ptr<SceneFileSession> s(new SceneFileSession());
    // Reserved, so the runs never move once the scene borrows the first one.
    s->runs_.reserve(lanes.size());
    for (const Lane& lane : lanes) {
        spade::Result<spade::scene::ComposedScene> composed = spade::scene::compose_file(lane.scene_file);
        if (!composed) {
            return std::unexpected(spade::Error{composed.error().code, lane.scene_file.filename().string() + ": " +
                                                                           composed.error().context});
        }
        spade::Result<spade::scene::SceneRun> run =
            spade::scene::instantiate(*composed, instance_of(lane.records), kBuiltinStepNs, kBuiltinSubsteps);
        if (!run) {
            return std::unexpected(
                spade::Error{run.error().code, lane.scene_file.filename().string() + ": " + run.error().context});
        }
        s->runs_.push_back(Run{std::move(*composed), std::move(*run), lane.records});
    }

    spade::Result<spade::render::RenderScene> scene = spade::render::scene_from_world(s->runs_[0].composed.world, {});
    if (!scene) {
        return std::unexpected(spade::Error{scene.error().code, "the scene would not draw: " + scene.error().context});
    }
    s->scene_ = std::move(*scene);

    // The parts every body is drawn with: unit meshes, scaled per item.
    s->box_mesh_ = static_cast<uint32_t>(s->scene_.meshes.size());
    s->scene_.meshes.push_back(make_box_mesh());
    s->sphere_mesh_ = static_cast<uint32_t>(s->scene_.meshes.size());
    s->scene_.meshes.push_back(make_sphere_mesh());
    s->disc_mesh_ = static_cast<uint32_t>(s->scene_.meshes.size());
    s->scene_.meshes.push_back(make_cylinder_mesh());
    s->body_material_ = static_cast<uint32_t>(s->scene_.materials.size());
    s->scene_.materials.push_back(detail::lambert(0.35f, 0.37f, 0.40f));
    s->nose_material_ = static_cast<uint32_t>(s->scene_.materials.size());
    s->scene_.materials.push_back(detail::lambert(0.85f, 0.15f, 0.10f));
    s->rotor_material_ = static_cast<uint32_t>(s->scene_.materials.size());
    s->scene_.materials.push_back(detail::lambert(0.12f, 0.12f, 0.14f));
    s->ball_material_ = static_cast<uint32_t>(s->scene_.materials.size());
    for (const glm::vec3& c : kLaneColours) {
        s->scene_.materials.push_back(detail::lambert(c.r, c.g, c.b));
    }
    s->options_.ground_grid = true;
    s->draw_items();
    return s;
}

void SceneFileSession::attach(GlTargetSink& sink) {
    sink_ = &sink;
    gpu_ = start_gpu(scene_);  // after create() bound every mesh, so one upload carries them
}

SceneFileSession::~SceneFileSession() {
    if (sink_ != nullptr) {
        sink_->set_gap_line({});
    }
}

spade::Result<glm::vec3> SceneFileSession::position(std::size_t lane, std::size_t vehicle) const {
    const Run& r = runs_.at(lane);
    auto body = r.run.sim.body(r.run.vehicles.at(vehicle).body);
    if (!body) return std::unexpected(body.error());
    return (*body)->pos;
}

spade::Result<glm::quat> SceneFileSession::orientation(std::size_t lane, std::size_t vehicle) const {
    const Run& r = runs_.at(lane);
    auto body = r.run.sim.body(r.run.vehicles.at(vehicle).body);
    if (!body) return std::unexpected(body.error());
    return (*body)->orient;
}

// Every body as draw items: a body-only model is a sphere of the contact
// radius it collides with; a model with rotors is a body, an arm to each
// rotor hub and a disc of the rotor's radius.
void SceneFileSession::draw_items() {
    scene_.dynamics.clear();
    for (std::size_t lane = 0; lane < runs_.size(); ++lane) {
        const Run& r = runs_[lane];
        const uint32_t ball = ball_material_ + static_cast<uint32_t>(lane % kLaneColours.size());
        for (std::size_t i = 0; i < r.run.vehicles.size(); ++i) {
            auto body = r.run.sim.body(r.run.vehicles[i].body);
            if (!body) continue;
            const glm::mat4 pose = glm::translate(glm::mat4(1.0f), (*body)->pos) * glm::mat4_cast((*body)->orient);
            const spade::vehicles::ModelType& model = r.composed.models.at(r.composed.vehicles.at(i).model);
            if (model.rotors.empty()) {
                render::DrawItem item;
                item.mesh_index = sphere_mesh_;
                item.local_to_world = glm::scale(pose, glm::vec3(2.0f * r.records.contacts.proxy_radius));
                item.material_override = ball;
                scene_.dynamics.push_back(item);
                continue;
            }
            render::DrawItem hull;
            hull.mesh_index = box_mesh_;
            hull.local_to_world = glm::scale(pose, 2.0f * kDroneBodyHalf);
            hull.material_override = body_material_;
            scene_.dynamics.push_back(hull);
            for (std::size_t k = 0; k < model.rotors.size(); ++k) {
                const spade::vehicles::RotorDesc& rotor = model.rotors[k];
                const glm::vec3 flat(rotor.local_pos.x, 0.0f, rotor.local_pos.z);
                const float reach = glm::length(flat);
                // The arm mesh is a unit box along +X; turn it about +Y onto
                // this arm (rotating +X by a about +Y gives (cos a, 0, -sin a)).
                const float a = std::atan2(-flat.z, flat.x);
                render::DrawItem arm;
                arm.mesh_index = box_mesh_;
                arm.local_to_world = glm::scale(pose * glm::translate(glm::mat4(1.0f), 0.5f * flat) *
                                                    glm::rotate(glm::mat4(1.0f), a, glm::vec3(0.0f, 1.0f, 0.0f)),
                                                glm::vec3(reach, 2.0f * kDroneArmHalfSection, 2.0f * kDroneArmHalfSection));
                arm.material_override = k == 0 ? nose_material_ : body_material_;
                scene_.dynamics.push_back(arm);
                render::DrawItem disc;
                disc.mesh_index = disc_mesh_;
                disc.local_to_world = glm::scale(
                    pose * glm::translate(glm::mat4(1.0f), rotor.local_pos + glm::vec3(0.0f, kDroneRotorLift, 0.0f)),
                    glm::vec3(2.0f * rotor.radius, 2.0f * kDroneRotorHalfThickness, 2.0f * rotor.radius));
                disc.material_override = k == 0 ? nose_material_ : rotor_material_;
                scene_.dynamics.push_back(disc);
            }
        }
    }
}

spade::Result<void> SceneFileSession::frame(const FrameInput& in, float dt) {
    camera_.apply(in, dt);

    // Whole built-in steps only; the remainder carries, so the run advances at
    // exactly its own rate whatever the frame time.
    carry_ns_ += static_cast<uint64_t>(std::llround(static_cast<double>(dt) * 1e9));
    const auto p0 = std::chrono::steady_clock::now();
    while (carry_ns_ >= kBuiltinStepNs) {
        carry_ns_ -= kBuiltinStepNs;
        for (Run& r : runs_) {
            if (const spade::Result<void> stepped = r.run.sim.step(1); !stepped) {
                return std::unexpected(spade::Error{stepped.error().code, "a scene step failed at step " +
                                                                              std::to_string(steps_) + ": " +
                                                                              stepped.error().context});
            }
        }
        ++steps_;
    }
    const float physics_ms = ms_since(p0);
    draw_items();

    const uint32_t fw = sink_->framebuffer_width();
    const uint32_t fh = sink_->framebuffer_height();
    if (fw == 0u || fh == 0u) {
        return {};  // minimised: the runs keep stepping
    }
    const spade::render::Camera rc = camera_.to_render_camera();
    // SL10: the HUD names what the active path does not draw.
    sink_->set_gap_line(gpu_ ? render_gap_line(GpuRenderer::unhonoured(options_)) : std::string{});
    const auto r0 = std::chrono::steady_clock::now();
    if (gpu_) {
        sink_->begin_gpu_frame();
        if (const spade::Result<void> drew = gpu_->draw(scene_, rc, options_, fw, fh); !drew) {
            return std::unexpected(spade::Error{drew.error().code, "GPU draw failed: " + drew.error().context});
        }
        sink_->present_overlay(ms_since(r0), physics_ms);
    } else {
        sink_->note_physics_ms(physics_ms);
        const bool ok = render_frame(scene_, rc, fw, fh, options_, pixels_, *sink_);
        sink_->note_render_ms(ms_since(r0));
        if (!ok) {
            return std::unexpected(spade::Error{spade::Code::internal, "the CPU render failed (see above)"});
        }
    }
    return {};
}

}  // namespace spade::sandbox
