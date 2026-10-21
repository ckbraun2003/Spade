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
#include <limits>
#include <cstdint>
#include <format>
#include <string>
#include <utility>
#include <vector>

#include "core/error.hpp"
#include "core/flow_volume.hpp"
#include <glm/geometric.hpp>
#include "render/scene.hpp"
#include "render/target.hpp"
#include "render/view_mode.hpp"

namespace spade::sandbox::editor {

struct ViewState {
    render::ViewSpec spec{};                   // what was chosen; shaded with no layers by default
    std::vector<render::ViewLegend> legends;   // the last draw's, one per layer
    std::string refusal;                       // why the view is not drawing `spec`; empty when it is
    render::BaseLegend base_legend{};          // the last draw's base legend (depth's range, the normals key)
};

// A depth view's range (Rendering's sensor bases, ee79a68): near 0.1 m; far
// twice the diagonal of the box around the origins of what the scene draws,
// never under 1 m, so the whole scene sits inside the colour ramp from any
// orbit. Set by apply_to() each frame, as the scene changes.
inline constexpr float kDepthNearM = 0.1f;
inline constexpr float kDepthFarMinM = 1.0f;

[[nodiscard]] inline render::DepthRange depth_range_for(const render::RenderScene& scene) {
    glm::vec3 lo(std::numeric_limits<float>::max());
    glm::vec3 hi(std::numeric_limits<float>::lowest());
    bool any = false;
    for (const auto* items : {&scene.statics, &scene.dynamics}) {
        for (const render::DrawItem& item : *items) {
            const glm::vec3 at(item.local_to_world[3]);
            lo = glm::min(lo, at);
            hi = glm::max(hi, at);
            any = true;
        }
    }
    render::DepthRange r;
    r.near_m = kDepthNearM;
    r.far_m = std::max(kDepthFarMinM, any ? 2.0f * glm::length(hi - lo) : 0.0f);
    return r;
}

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
    view.spec.depth = depth_range_for(scene);
    const render::ViewSupport s = render::supports(view.spec, backend, inputs);
    if (!s.drawable) {
        view.refusal = "drawing shaded: this view can't draw " + std::string(render::view_base_name(view.spec.base)) +
                       ": " + s.why_not;
        const Result<std::vector<render::ViewLegend>> shaded =
            render::apply_view(render::ViewSpec{}, volume, scene, options, cache);
        if (!shaded) return std::unexpected(shaded.error());
        view.legends.clear();
        view.base_legend = render::base_legend(render::ViewSpec{}, scene);
        return {};
    }
    Result<std::vector<render::ViewLegend>> legends = render::apply_view(view.spec, volume, scene, options, cache);
    if (!legends) {
        view.refusal = legends.error().context;
        return std::unexpected(legends.error());
    }
    view.legends = std::move(*legends);
    view.base_legend = render::base_legend(view.spec, scene);
    view.refusal.clear();
    return {};
}

// A flow layer's legend as the toolbar shows it: the quantity and the range
// its colours span ("C_p -1.25 to 0.8", "speed |u| 0 to 9.96 m/s"), and
// what the colour may claim (ViewLegend::note), verbatim; empty when the
// quantity says it all.
struct LegendLine {
    std::string scale;
    std::string note;
};

[[nodiscard]] inline LegendLine legend_line(const render::ViewLegend& legend) {
    LegendLine line;
    line.scale = std::format("{} {:.3g} to {:.3g}", legend.quantity, legend.range.min, legend.range.max);
    if (!legend.unit.empty()) line.scale += " " + legend.unit;
    line.note = legend.note;
    return line;
}

}  // namespace spade::sandbox::editor
