#include "transform.hpp"

namespace spade::partimport {

std::string Axes::text() const { return {}; }

Expected<Axes> parse_axes(std::string_view) { return std::unexpected(Problem{"internal", "not built"}); }
Expected<double> parse_scale(std::string_view) { return std::unexpected(Problem{"internal", "not built"}); }
glm::dvec3 apply_axes(const Axes&, const glm::dvec3& p) { return p; }
Expected<std::shared_ptr<const TriangleMesh>> transform_mesh(const std::shared_ptr<const TriangleMesh>&, const Axes&, double) {
    return std::unexpected(Problem{"internal", "not built"});
}
std::optional<UnitGuess> guess_unit(const glm::dvec3&) { return std::nullopt; }

}  // namespace spade::partimport
