// The editor's per-view state and its mode menu: N19, the end-goal design §5
// (INT-10). Display-free, so the window only draws what this decides.
//
// A VIEW'S MODE IS EDITOR STATE, NOT SCENE DATA. Each view holds what it
// shows (a render::ViewSpec), the legends its last draw returned, and the
// reason it is not drawing what was chosen. The scene stays what is
// simulated.
//
// THE MENU NEVER ANSWERS FOR ITSELF. Each entry is render::supports()' answer
// for this view's spec with that base, on this backend, with these inputs, so
// a kind the view can't draw shows supports()' phrase verbatim -- the phrase
// the draw path would refuse with -- instead of drawing blank.
//
// M1 offers the bases only (raster modes); the flow layers and Split come with
// the editor's airflow (M2).

#pragma once

#include <algorithm>
#include <cstdint>
#include <string>
#include <utility>
#include <vector>

#include "core/error.hpp"
#include "core/flow_volume.hpp"
#include "render/scene.hpp"
#include "render/target.hpp"
#include "render/view_mode.hpp"

namespace spade::sandbox::editor {

struct ViewState {
    render::ViewSpec spec{};                   // what was chosen; shaded with no layers by default
    std::vector<render::ViewLegend> legends;   // the last draw's, one per layer
    std::string refusal;                       // why the view is not drawing `spec`; empty when it is
};

// One entry of a view's mode menu.
struct ModeEntry {
    render::ViewBase base = render::ViewBase::shaded;
    std::string name;     // render::view_base_name(): stable, for a menu and a script
    bool drawable = false;
    std::string why_not;  // supports()' phrase when not drawable
    bool current = false; // the view's chosen base
};

namespace detail {

[[nodiscard]] inline render::ViewSpec with_base(const render::ViewSpec& spec, render::ViewBase base) {
    render::ViewSpec s = spec;
    s.base = base;
    return s;
}

}  // namespace detail

// What `scene` and the backend have, read the way render::apply_view() reads
// them (view_mode.cpp), so the menu and the draw cannot disagree: an SDF only
// when the scene's program is non-empty, walls when any item is a flow wall.
[[nodiscard]] inline render::ViewInputs inputs_of(const render::RenderScene& scene, const FlowVolume* volume,
                                                  bool gl_flow_ready) {
    const auto wall = [](const render::DrawItem& item) { return item.flow_wall; };
    render::ViewInputs in;
    in.has_volume = volume != nullptr;
    in.has_walls = std::any_of(scene.statics.begin(), scene.statics.end(), wall) ||
                   std::any_of(scene.dynamics.begin(), scene.dynamics.end(), wall);
    in.has_sdf = scene.sdf != nullptr && !scene.sdf->empty();
    in.gl_flow_ready = gl_flow_ready;
    return in;
}

// Every base, in enum order.
[[nodiscard]] inline std::vector<ModeEntry> mode_menu(const ViewState& view, render::ViewBackend backend,
                                                      const render::ViewInputs& inputs) {
    std::vector<ModeEntry> menu;
    menu.reserve(render::kViewBaseCount);
    for (uint32_t i = 0; i < render::kViewBaseCount; ++i) {
        const auto base = static_cast<render::ViewBase>(i);
        const render::ViewSupport s = render::supports(detail::with_base(view.spec, base), backend, inputs);
        menu.push_back(ModeEntry{base, std::string(render::view_base_name(base)), s.drawable, s.why_not,
                                 base == view.spec.base});
    }
    return menu;
}

// Switches the view to `base`, or refuses in supports()' words and keeps what
// the view draws. Either way `view.refusal` is what the panel shows.
[[nodiscard]] inline Result<void> choose_mode(ViewState& view, render::ViewBase base, render::ViewBackend backend,
                                              const render::ViewInputs& inputs) {
    const render::ViewSpec next = detail::with_base(view.spec, base);
    const render::ViewSupport s = render::supports(next, backend, inputs);
    if (!s.drawable) {
        view.refusal = "this view can't draw " + std::string(render::view_base_name(base)) + ": " + s.why_not;
        return std::unexpected(Error{Code::invalid_argument, view.refusal});
    }
    view.spec = next;
    view.refusal.clear();
    view.legends.clear();
    return {};
}

// Builds the view into `scene` and `options` for this frame through
// render::apply_view(), its streamlines through `cache` (one cache across
// views and frames). A chosen spec this backend can no longer draw (the
// backend or the inputs changed under it) draws shaded with the reason in
// `view.refusal`; the choice is kept, and drawn again once it can be.
[[nodiscard]] inline Result<void> apply_to(ViewState& view, const FlowVolume* volume, render::ViewBackend backend,
                                           const render::ViewInputs& inputs, render::RenderScene& scene,
                                           render::RenderOptions& options, render::ViewCache& cache) {
    const render::ViewSupport s = render::supports(view.spec, backend, inputs);
    if (!s.drawable) {
        view.refusal = "drawing shaded: this view can't draw " + std::string(render::view_base_name(view.spec.base)) +
                       ": " + s.why_not;
        const Result<std::vector<render::ViewLegend>> shaded =
            render::apply_view(render::ViewSpec{}, volume, scene, options, cache);
        if (!shaded) return std::unexpected(shaded.error());
        view.legends.clear();
        return {};
    }
    Result<std::vector<render::ViewLegend>> legends = render::apply_view(view.spec, volume, scene, options, cache);
    if (!legends) {
        view.refusal = legends.error().context;
        return std::unexpected(legends.error());
    }
    view.legends = std::move(*legends);
    view.refusal.clear();
    return {};
}

}  // namespace spade::sandbox::editor
