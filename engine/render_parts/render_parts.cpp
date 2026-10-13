#include "render_parts/render_parts.hpp"

namespace spade::render_parts {

// RED: the interface only; the tests fail until the green step.
Result<PreparedParts> prepare_parts(const vehicles::ModelType& model) {
    return std::unexpected(Error{Code::unavailable, "prepare_parts: not built yet (model '" + model.name + "')"});
}

Result<void> bind_parts(const PreparedParts&, const vehicles::PosedParts&, const PartsLook&, render::RenderScene&) {
    return std::unexpected(Error{Code::unavailable, "bind_parts: not built yet"});
}

}  // namespace spade::render_parts
