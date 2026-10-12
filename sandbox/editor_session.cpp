// The editor in a window (editor plan Task 10). See the header.

#include "editor_session.hpp"

#include <chrono>
#include <cmath>
#include <optional>
#include <string>
#include <utility>

#include <glm/glm.hpp>

#include "builder_scene.hpp"  // the unit meshes, ray_from_screen()
#include "drone_view.hpp"     // detail::lambert()
#include "editor_pick.hpp"
#include "gl_target_sink.hpp"
#include "render_gap.hpp"
#include "viewport_rect.hpp"

namespace spade::sandbox {

namespace {

[[nodiscard]] float ms_since(std::chrono::steady_clock::time_point t0) {
    return std::chrono::duration<float, std::milli>(std::chrono::steady_clock::now() - t0).count();
}

// An asset's box in the world, for the selection overlay: its collider's
// extent, or a sphere around it when it is turned.
[[nodiscard]] render::Aabb asset_box(const scene::SceneAsset& a) {
    glm::vec3 half = editor::detail::asset_half_extents(a);
    const glm::quat r = a.pose.rotation;
    if (std::abs(std::abs(r.w) - 1.0f) > 1e-6f) half = glm::vec3(glm::length(half));
    half *= a.pose.scale;
    return render::Aabb{a.pose.position - half, a.pose.position + half};
}

}  // namespace

Result<std::unique_ptr<EditorSession>> EditorSession::create(editor::OpenedScene opened) {
    Result<std::unique_ptr<editor::EditorWorkspace>> workspace = editor::EditorWorkspace::create(std::move(opened));
    if (!workspace) return std::unexpected(workspace.error());
    std::unique_ptr<EditorSession> s(new EditorSession());
    s->workspace_ = std::move(*workspace);
    if (Result<void> built = s->rebuild_scene(); !built) return std::unexpected(built.error());
    // The editor opens framed on the first vehicle, as F would frame it, or
    // looking at the origin.
    const scene::SceneDesc& scene = s->workspace_->documents().scene.desc();
    s->camera_.target = glm::vec3(0.0f);
    s->camera_.distance = 6.0f;
    for (const scene::SceneVehicle& v : scene.vehicles) {
        for (const vehicles::ModelType& m : scene.models) {
            if (m.name == v.model) {
                s->camera_.frame(v.start.pos, editor::detail::reach_of(m));
                return s;
            }
        }
    }
    return s;
}

void EditorSession::attach(GlTargetSink& sink) {
    sink_ = &sink;
    sink.attach_editor(&workspace_->panels());
    gpu_ = start_gpu(scene_);  // after create() bound every mesh, so one upload carries them
}

EditorSession::~EditorSession() {
    if (sink_ != nullptr) {
        sink_->attach_editor(nullptr);
        sink_->set_gap_line({});
    }
}

// The composed world as a render scene, plus the meshes and materials the
// vehicles and the selection draw with. Only when the composition changed.
Result<void> EditorSession::rebuild_scene() {
    Result<render::RenderScene> built = render::scene_from_world(workspace_->composed().world, {});
    if (!built) {
        return std::unexpected(Error{built.error().code, "the scene would not draw: " + built.error().context});
    }
    scene_ = std::move(*built);
    look_.box_mesh = static_cast<uint32_t>(scene_.meshes.size());
    scene_.meshes.push_back(make_box_mesh());
    look_.sphere_mesh = static_cast<uint32_t>(scene_.meshes.size());
    scene_.meshes.push_back(make_sphere_mesh());
    look_.disc_mesh = static_cast<uint32_t>(scene_.meshes.size());
    scene_.meshes.push_back(make_cylinder_mesh());
    look_.body_material = static_cast<uint32_t>(scene_.materials.size());
    scene_.materials.push_back(detail::lambert(0.35f, 0.37f, 0.40f));
    look_.nose_material = static_cast<uint32_t>(scene_.materials.size());
    scene_.materials.push_back(detail::lambert(0.85f, 0.15f, 0.10f));
    look_.rotor_material = static_cast<uint32_t>(scene_.materials.size());
    scene_.materials.push_back(detail::lambert(0.12f, 0.12f, 0.14f));
    selected_material_ = static_cast<uint32_t>(scene_.materials.size());
    scene_.materials.push_back(detail::lambert(0.29f, 0.56f, 0.89f));  // the theme's accent
    options_.ground_grid = true;
    options_.overlays = true;  // the selection's box
    drawn_composition_ = workspace_->composition();
    if (gpu_) {
        if (Result<void> up = gpu_->upload_scene(scene_); !up) {
            return std::unexpected(Error{up.error().code, "re-upload failed: " + up.error().context});
        }
    }
    return {};
}

// The vehicles where they are (the run's, or their starts), the selected one
// in the accent colour, and a box around the selection.
void EditorSession::draw_items() {
    const editor::EditorWorkspace& w = *workspace_;
    const scene::SceneDesc& s = w.documents().scene.desc();
    const editor::Selection& selected = w.panels().selection;
    scene_.dynamics.clear();
    std::optional<render::Aabb> box;
    for (const scene::SceneVehicle& v : s.vehicles) {
        const vehicles::ModelType* model = nullptr;
        for (const vehicles::ModelType& m : s.models) {
            if (m.name == v.model) model = &m;
        }
        if (model == nullptr) continue;
        glm::vec3 pos = v.start.pos;
        glm::quat orient = v.start.orient;
        if (w.run() != nullptr) {
            if (const std::optional<FrameState> now = w.run()->state_of(v.name)) {
                pos = now->pos;
                orient = now->orient;
            }
        }
        const bool is_selected = selected.kind == editor::Selection::Kind::object && selected.name == v.name;
        VehicleLook look = look_;
        if (is_selected) look.body_material = selected_material_;
        append_vehicle_items(scene_.dynamics, look, *model, pos, orient, 2.0f * w.records().contacts.proxy_radius,
                             is_selected ? selected_material_ : look_.body_material);
        if (is_selected) {
            const glm::vec3 reach(editor::detail::reach_of(*model));
            box = render::Aabb{pos - reach, pos + reach};
        }
    }
    if (selected.kind == editor::Selection::Kind::object && !box) {
        if (const scene::SceneAsset* a = editor::EditorWorkspace::find_asset(s, selected.name)) box = asset_box(*a);
    }
    // No selection: a box of no size, which draws nothing.
    scene_.bounds = box.value_or(render::Aabb{});
}

Result<void> EditorSession::frame(const FrameInput& in, float dt) {
    camera_.apply(in, dt);
    editor::EditorWorkspace& w = *workspace_;

    // A click in the view picks or places, along the ray under the cursor
    // (measured from the view's corner, at the view's size).
    const ViewportRect view = sink_->viewport();
    if (in.left_click && !in.ui_captured_mouse && !view.empty()) {
        const Ray ray = ray_from_screen(camera_.to_render_camera(), in.mouse_x, in.mouse_y, view.width, view.height);
        w.click(ray.origin, ray.direction);
    }
    if (!in.ui_captured_keyboard) {
        if (in.delete_pressed) w.key_delete();
        if (in.duplicate_pressed) w.key_duplicate();
        if (in.frame_pressed) {
            if (const std::optional<editor::Framing> f = w.selection_framing()) camera_.frame(f->centre, f->radius);
        }
    }
    // What the panels queued while the last frame drew them.
    w.apply_panels();

    const auto p0 = std::chrono::steady_clock::now();
    w.advance(dt);
    const float physics_ms = ms_since(p0);

    if (w.composition() != drawn_composition_) {
        if (Result<void> rebuilt = rebuild_scene(); !rebuilt) return rebuilt;
    }
    draw_items();

    if (view.empty()) return {};  // minimised: the run keeps stepping
    const render::Camera rc = camera_.to_render_camera();
    // SL10: the HUD names what the active path does not draw.
    sink_->set_gap_line(gpu_ ? render_gap_line(GpuRenderer::unhonoured(options_)) : std::string{});
    const auto r0 = std::chrono::steady_clock::now();
    if (gpu_) {
        sink_->begin_gpu_frame();
        if (const Result<void> drew = gpu_->draw(scene_, rc, options_, view.width, view.height); !drew) {
            return std::unexpected(Error{drew.error().code, "GPU draw failed: " + drew.error().context});
        }
        sink_->present_overlay(ms_since(r0), physics_ms);
    } else {
        sink_->note_physics_ms(physics_ms);
        if (!render_frame(scene_, rc, view.width, view.height, options_, pixels_, *sink_)) {
            return std::unexpected(Error{Code::internal, "the CPU render failed (see above)"});
        }
        sink_->note_render_ms(ms_since(r0));
    }
    return {};
}

}  // namespace spade::sandbox
