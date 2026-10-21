// world/mesh_import/weld.cpp -- see world/detail/weld.hpp.
//
// Vertices are sorted by (x, y, z, index) under a total order on each
// coordinate's bits, -0 first made +0, so equal positions are adjacent and
// each run's first entry is its lowest index. A run merges only entries that
// compare equal by value, so a NaN stays alone. Survivors are then numbered
// as the index stream first names them. No unordered container, no tolerance.
#include "world/detail/weld.hpp"

#include <algorithm>
#include <array>
#include <bit>
#include <cstddef>
#include <limits>
#include <numeric>
#include <string>
#include <vector>

namespace spade::detail {
namespace {

// A coordinate's bits, ordered as the values are: negative values flipped,
// positive ones offset past them. -0 is made +0 first, so the two share a key.
[[nodiscard]] uint64_t order_key(double v) noexcept {
    if (v == 0.0) v = 0.0;
    const uint64_t bits = std::bit_cast<uint64_t>(v);
    constexpr uint64_t kSign = uint64_t{1} << 63;
    return (bits & kSign) != 0 ? ~bits : (bits | kSign);
}

[[nodiscard]] std::array<uint64_t, 3> key_of(const glm::dvec3& p) noexcept {
    return {order_key(p.x), order_key(p.y), order_key(p.z)};
}

}  // namespace

MeshInput weld_exact(std::span<const glm::dvec3> positions, std::span<const uint32_t> indices, MeshUnit unit) {
    const std::size_t n = positions.size();
    std::vector<std::array<uint64_t, 3>> keys(n);
    for (std::size_t v = 0; v < n; ++v) keys[v] = key_of(positions[v]);

    std::vector<uint32_t> order(n);
    std::iota(order.begin(), order.end(), uint32_t{0});
    std::sort(order.begin(), order.end(),
              [&keys](uint32_t a, uint32_t b) { return keys[a] != keys[b] ? keys[a] < keys[b] : a < b; });

    // Each vertex's representative: the lowest index of its run of equal
    // positions.
    std::vector<uint32_t> rep(n);
    for (std::size_t k = 0; k < n; ++k) {
        const uint32_t v = order[k];
        rep[v] = v;
        if (k > 0) {
            const uint32_t prev = order[k - 1];
            if (keys[prev] == keys[v] && positions[prev] == positions[v]) rep[v] = rep[prev];
        }
    }

    // Compaction, in first-use order.
    constexpr uint32_t kUnnumbered = std::numeric_limits<uint32_t>::max();
    std::vector<uint32_t> number(n, kUnnumbered);
    MeshInput out;
    out.unit = unit;
    out.indices.reserve(indices.size());
    for (const uint32_t i : indices) {
        const uint32_t r = rep[i];
        if (number[r] == kUnnumbered) {
            number[r] = static_cast<uint32_t>(out.positions.size());
            out.positions.push_back(positions[r]);
        }
        out.indices.push_back(number[r]);
    }
    return out;
}

std::string unit_problem(MeshUnit unit) {
    if (unit == MeshUnit::unspecified) {
        return "the unit is unspecified (the file carries none: declare the one it was exported in)";
    }
    if (mesh_unit_scale(unit) == 0.0) {
        return "unit " + std::to_string(static_cast<uint32_t>(unit)) + " is not a MeshUnit";
    }
    return {};
}

}  // namespace spade::detail
