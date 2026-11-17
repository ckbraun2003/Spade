#include "transform.hpp"

#include <algorithm>
#include <charconv>
#include <cmath>
#include <format>
#include <system_error>
#include <vector>

namespace spade::partimport {

namespace {

[[nodiscard]] std::string_view trimmed(std::string_view s) {
    while (!s.empty() && (s.front() == ' ' || s.front() == '\t')) s.remove_prefix(1);
    while (!s.empty() && (s.back() == ' ' || s.back() == '\t')) s.remove_suffix(1);
    return s;
}

[[nodiscard]] Problem axes_problem(std::string_view text, const std::string& why) {
    return Problem{"axes_invalid", "--axes '" + std::string(text) + "': " + why};
}

}  // namespace

std::string Axes::text() const {
    std::string out;
    for (int i = 0; i < 3; ++i) {
        if (i != 0) out.push_back(',');
        if (sign[i] < 0) out.push_back('-');
        out.push_back("xyz"[source[i]]);
    }
    return out;
}

Expected<Axes> parse_axes(std::string_view text) {
    std::vector<std::string_view> items;
    std::size_t start = 0;
    for (;;) {
        const std::size_t comma = text.find(',', start);
        items.push_back(trimmed(text.substr(start, comma == std::string_view::npos ? std::string_view::npos : comma - start)));
        if (comma == std::string_view::npos) break;
        start = comma + 1;
    }
    if (items.size() != 3) return std::unexpected(axes_problem(text, "three comma-separated axes are needed, for example -y,x,z"));
    Axes axes;
    std::array<bool, 3> used{false, false, false};
    for (int i = 0; i < 3; ++i) {
        std::string_view item = items[static_cast<std::size_t>(i)];
        int sign = 1;
        if (!item.empty() && item.front() == '-') {
            sign = -1;
            item.remove_prefix(1);
        }
        int axis = -1;
        if (item.size() == 1) {
            const char c = item.front();
            if (c == 'x' || c == 'X') axis = 0;
            if (c == 'y' || c == 'Y') axis = 1;
            if (c == 'z' || c == 'Z') axis = 2;
        }
        if (axis < 0) return std::unexpected(axes_problem(text, "'" + std::string(items[static_cast<std::size_t>(i)]) + "' is not x, y or z with an optional minus"));
        if (used[static_cast<std::size_t>(axis)]) return std::unexpected(axes_problem(text, std::string("the axis ") + "xyz"[axis] + " is used twice"));
        used[static_cast<std::size_t>(axis)] = true;
        axes.source[static_cast<std::size_t>(i)] = axis;
        axes.sign[static_cast<std::size_t>(i)] = sign;
    }
    // The determinant: the permutation's parity times the signs.
    int inversions = 0;
    for (int i = 0; i < 3; ++i) {
        for (int j = i + 1; j < 3; ++j) {
            if (axes.source[static_cast<std::size_t>(i)] > axes.source[static_cast<std::size_t>(j)]) ++inversions;
        }
    }
    const int det = (inversions % 2 == 0 ? 1 : -1) * axes.sign[0] * axes.sign[1] * axes.sign[2];
    if (det < 0) {
        return std::unexpected(Problem{"axes_mirror", "--axes '" + std::string(text) + "' is a mirror (determinant -1), which reverses the part"});
    }
    return axes;
}

Expected<double> parse_scale(std::string_view text) {
    const std::string_view t = trimmed(text);
    double v = 0.0;
    const auto r = std::from_chars(t.data(), t.data() + t.size(), v);
    if (t.empty() || r.ec != std::errc() || r.ptr != t.data() + t.size() || !std::isfinite(v) || !(v > 0.0)) {
        return std::unexpected(Problem{"scale_invalid", "--scale '" + std::string(text) + "' is not a finite number above zero"});
    }
    return v;
}

glm::dvec3 apply_axes(const Axes& axes, const glm::dvec3& p) {
    glm::dvec3 out;
    for (int i = 0; i < 3; ++i) {
        double v = p[axes.source[static_cast<std::size_t>(i)]];
        if (axes.sign[static_cast<std::size_t>(i)] < 0) v = -v;
        out[i] = v == 0.0 ? 0.0 : v;  // never a negative zero: the sidecar's bits are the same for a mirrored zero
    }
    return out;
}

Expected<std::shared_ptr<const TriangleMesh>> transform_mesh(const std::shared_ptr<const TriangleMesh>& mesh, const Axes& axes,
                                                            double scale) {
    if (axes.identity() && scale == 1.0) return mesh;
    MeshInput in;
    in.unit = MeshUnit::metre;
    in.positions.reserve(mesh->positions().size());
    for (const glm::vec3& p : mesh->positions()) {
        const glm::dvec3 q = apply_axes(axes, glm::dvec3(p));
        in.positions.push_back(scale == 1.0 ? q : q * scale);
    }
    in.indices.assign(mesh->indices().begin(), mesh->indices().end());
    auto built = make_mesh(in, "the part frame");
    if (!built.has_value()) return std::unexpected(Problem{"mesh_refused", built.error().context});
    return *built;
}

std::optional<UnitGuess> guess_unit(const glm::dvec3& box_sides) {
    const double longest = std::max({box_sides.x, box_sides.y, box_sides.z});
    if (!std::isfinite(box_sides.x) || !std::isfinite(box_sides.y) || !std::isfinite(box_sides.z) || !(longest > 0.0)) return std::nullopt;
    if (longest >= 5.0 && longest < 5000.0) {
        return UnitGuess{"mm", std::format("the longest side is {} in the file's own numbers; as millimetres that is {} m, a part-sized object", longest, longest / 1000.0)};
    }
    if (longest >= 0.005 && longest < 5.0) {
        return UnitGuess{"m", std::format("the longest side is {} in the file's own numbers; as metres that is a part-sized object", longest)};
    }
    return std::nullopt;
}

}  // namespace spade::partimport
