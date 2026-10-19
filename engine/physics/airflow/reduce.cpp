#include "physics/airflow/reduce.hpp"

#include <algorithm>
#include <cstddef>

namespace spade::physics::airflow {

float fixed_tile_sum(std::span<const float> x, uint32_t nx, uint32_t ny, uint32_t nz) noexcept {
    float total = 0.0f;
    for (uint32_t tk = 0; tk < nz; tk += kReduceTile) {
        const uint32_t k_end = std::min(tk + kReduceTile, nz);
        for (uint32_t tj = 0; tj < ny; tj += kReduceTile) {
            const uint32_t j_end = std::min(tj + kReduceTile, ny);
            for (uint32_t ti = 0; ti < nx; ti += kReduceTile) {
                const uint32_t i_end = std::min(ti + kReduceTile, nx);
                float tile = 0.0f;
                for (uint32_t k = tk; k < k_end; ++k) {
                    for (uint32_t j = tj; j < j_end; ++j) {
                        for (uint32_t i = ti; i < i_end; ++i) {
                            tile += x[i + std::size_t{nx} * (j + std::size_t{ny} * k)];
                        }
                    }
                }
                total += tile;
            }
        }
    }
    return total;
}

float fixed_tile_dot(std::span<const float> a, std::span<const float> b, uint32_t nx, uint32_t ny,
                     uint32_t nz) noexcept {
    float total = 0.0f;
    for (uint32_t tk = 0; tk < nz; tk += kReduceTile) {
        const uint32_t k_end = std::min(tk + kReduceTile, nz);
        for (uint32_t tj = 0; tj < ny; tj += kReduceTile) {
            const uint32_t j_end = std::min(tj + kReduceTile, ny);
            for (uint32_t ti = 0; ti < nx; ti += kReduceTile) {
                const uint32_t i_end = std::min(ti + kReduceTile, nx);
                float tile = 0.0f;
                for (uint32_t k = tk; k < k_end; ++k) {
                    for (uint32_t j = tj; j < j_end; ++j) {
                        for (uint32_t i = ti; i < i_end; ++i) {
                            const std::size_t c = i + std::size_t{nx} * (j + std::size_t{ny} * k);
                            tile += a[c] * b[c];
                        }
                    }
                }
                total += tile;
            }
        }
    }
    return total;
}

void remove_mean(std::span<float> x, uint32_t nx, uint32_t ny, uint32_t nz) noexcept {
    if (x.empty()) return;
    const auto inv_count = static_cast<float>(1.0 / static_cast<double>(x.size()));  // the host's reciprocal
    const float mean = fixed_tile_sum(x, nx, ny, nz) * inv_count;
    for (float& v : x) v -= mean;
}

}  // namespace spade::physics::airflow
