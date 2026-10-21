#include "physics/airflow/reduce.hpp"

#include <algorithm>
#include <cstddef>

namespace spade::physics::airflow {

float fixed_tile_sum(std::span<const float> x, uint32_t nx, uint32_t ny, uint32_t nz) noexcept {
    return fixed_tile_sum_of<float>(nx, ny, nz, [x](uint32_t, uint32_t, uint32_t, std::size_t c) { return x[c]; });
}

float fixed_tile_dot(std::span<const float> a, std::span<const float> b, uint32_t nx, uint32_t ny,
                     uint32_t nz) noexcept {
    return fixed_tile_sum_of<float>(nx, ny, nz,
                                    [a, b](uint32_t, uint32_t, uint32_t, std::size_t c) { return a[c] * b[c]; });
}

void remove_mean(std::span<float> x, uint32_t nx, uint32_t ny, uint32_t nz) noexcept {
    if (x.empty()) return;
    const auto inv_count = static_cast<float>(1.0 / static_cast<double>(x.size()));  // the host's reciprocal
    const float mean = fixed_tile_sum(x, nx, ny, nz) * inv_count;
    for (float& v : x) v -= mean;
}

}  // namespace spade::physics::airflow
