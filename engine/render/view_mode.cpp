#include "render/view_mode.hpp"

namespace spade::render {

// RED: the interface only; the tests fail until the green step.
std::string_view view_base_name(ViewBase) noexcept { return {}; }
std::string_view flow_view_name(FlowView) noexcept { return {}; }
std::optional<ViewBase> view_base_from_name(std::string_view) noexcept { return std::nullopt; }
std::optional<FlowView> flow_view_from_name(std::string_view) noexcept { return std::nullopt; }

ViewSupport supports(const ViewSpec&, ViewBackend, const ViewInputs&) { return ViewSupport{false, "not built yet"}; }

Result<std::vector<ViewLegend>> apply_view(const ViewSpec&, const FlowVolume*, RenderScene&, RenderOptions&) {
    return std::unexpected(Error{Code::unavailable, "apply_view: not built yet"});
}

}  // namespace spade::render
