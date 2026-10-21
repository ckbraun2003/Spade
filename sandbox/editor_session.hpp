// The editor in a window (editor plan Task 10): the workspace
// (editor_workspace.hpp), which decides everything, drawn.
//
// Each frame: the camera moves; a click in the view picks or places; Delete
// and Ctrl+D act on the selection; what the panels queued last frame is
// applied; the run steps; and the scene is drawn between the panels, with a
// box around the selection. The render scene is rebuilt only when the
// composition changes, so an idle editor uploads nothing.

#pragma once

#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "core/error.hpp"
#include "editor_files.hpp"
#include "editor_views.hpp"
#include "editor_workspace.hpp"
#include "orbit_camera.hpp"
#include "render/scene.hpp"
#include "render/view_mode.hpp"
#include "render_parts/render_parts.hpp"
#include "scene_sessions.hpp"
#include "vehicle_draw.hpp"

namespace spade::sandbox {

class GlTargetSink;

class EditorSession {
  public:
    // Opens the documents in a workspace and builds what draws them. Needs no
    // window, so a scene that will not compose refuses before one opens.
    [[nodiscard]] static Result<std::unique_ptr<EditorSession>> create(editor::OpenedScene opened);

    // Shows the panels in `sink` and starts the GPU renderer in its context,
    // or announces the CPU fallback. `sink` must outlive this. Call once.
    void attach(GlTargetSink& sink);

    ~EditorSession();
    EditorSession(const EditorSession&) = delete;
    EditorSession& operator=(const EditorSession&) = delete;

    // One frame, `dt` seconds after the last.
    [[nodiscard]] Result<void> frame(const FrameInput& in, float dt);

    [[nodiscard]] editor::EditorWorkspace& workspace() noexcept { return *workspace_; }
    [[nodiscard]] OrbitCamera& camera() noexcept { return camera_; }
    [[nodiscard]] bool gpu_active() const noexcept { return gpu_ != nullptr; }

    // What the last frame did with the view's mode, for the tour's
    // editor.views (Test/Docs' modes_drawn()): applied through apply_view(),
    // drawn by the path that presents, and what that path left unhonoured.
    struct ViewDrawn {
        bool applied = false;
        render::ViewBase base = render::ViewBase::shaded;  // what the frame drew: the chosen base, or shaded in its stead
        bool drawn = false;
        std::string unhonoured;  // GpuRenderer::unhonoured()'s names, ", "-joined; empty when all is honoured
    };
    [[nodiscard]] const ViewDrawn& last_view() const noexcept { return last_view_; }
    // While on, each frame draws the view's background alone: its item lists
    // are emptied after binding (the meshes stay, so nothing re-uploads). The
    // tour's editor.views judges "not blank" against this frame.
    void set_background_only(bool on) noexcept { background_only_ = on; }

  private:
    EditorSession() = default;
    [[nodiscard]] Result<void> rebuild_scene();
    void draw_items();
    // The view's backend, and its inputs read from the scene as apply_view()
    // reads them (editor_views.hpp).
    [[nodiscard]] render::ViewBackend view_backend() const noexcept;
    [[nodiscard]] render::ViewInputs view_inputs();
    // The gizmo's object: the selected asset's pose, or the selected
    // vehicle's start when no run moves it; none for anything else.
    [[nodiscard]] std::optional<editor::GizmoPose> gizmo_target() const;
    // The edit that gives the selection `pose`, its scale and the rest kept.
    [[nodiscard]] std::optional<editor::SceneEdit> pose_edit(const editor::GizmoPose& pose) const;

    std::unique_ptr<editor::EditorWorkspace> workspace_;
    GlTargetSink* sink_ = nullptr;
    render::RenderScene scene_;
    render::RenderOptions options_;
    // N12: the composed world and the meshes and materials vehicles draw with,
    // built once per composition; scene_ is a copy of it each frame, with the
    // vehicles bound in (bind_parts() writes statics).
    render::RenderScene base_;
    std::vector<std::pair<std::string, render_parts::PreparedParts>> prepared_;  // by model name
    bool upload_each_frame_ = false;  // a CSG-root piece marches at its uploaded pose
    editor::ViewState view_;        // what the view shows (N19): editor state, not scene data
    render::ViewCache view_cache_;  // one across views and frames (streamlines traced once per generation)
    VehicleLook look_;
    uint32_t selected_material_ = 0;
    uint64_t drawn_composition_ = 0;
    ViewDrawn last_view_;
    bool background_only_ = false;
    std::unique_ptr<GpuRenderer> gpu_;
    OrbitCamera camera_;
    std::optional<editor::GizmoDrag> drag_;  // the gizmo's drag, from press to release
    editor::GizmoHandle hot_;                // the handle under the cursor, or dragged
    std::vector<uint8_t> pixels_;
};

}  // namespace spade::sandbox
