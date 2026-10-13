#include "lambda2_fill.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <vector>

#include "core/fp16.hpp"
#include "core/lambda2.hpp"

namespace flow_movie {

void fill_lambda2(spade::FlowVolume& volume) {
    spade::FlowVolumeHeader& h = volume.header;
    const std::size_t cells = std::size_t{h.nx} * h.ny * h.nz;
    // The velocity decoded once: three floats per cell.
    std::vector<float> vel(cells * 3u);
    for (std::size_t c = 0; c < cells; ++c) {
        for (uint32_t a = 0; a < 3u; ++a) {
            vel[c * 3u + a] = spade::fp16::to_f32(volume.v[c * spade::kFlowHalvesPerCell + a]);
        }
    }
    const uint32_t n[3] = {h.nx, h.ny, h.nz};
    const auto index = [&](uint32_t i, uint32_t j, uint32_t k) { return spade::flow_cell_index(h, i, j, k); };
    bool seen = false;
    float lo = 0.0f, hi = 0.0f;
    for (uint32_t k = 0; k < h.nz; ++k) {
        for (uint32_t j = 0; j < h.ny; ++j) {
            for (uint32_t i = 0; i < h.nx; ++i) {
                const uint32_t at[3] = {i, j, k};
                spade::VelocityGradient g{};
                bool finite = true;
                for (uint32_t a = 0; a < 3u; ++a) {  // d/dx_a: central inside, one-sided at a face
                    uint32_t lo_at[3] = {i, j, k}, hi_at[3] = {i, j, k};
                    if (at[a] > 0u) lo_at[a] = at[a] - 1u;
                    if (at[a] + 1u < n[a]) hi_at[a] = at[a] + 1u;
                    const uint32_t span = hi_at[a] - lo_at[a];
                    if (span == 0u) continue;  // one cell along this axis: no gradient
                    const std::size_t m = index(lo_at[0], lo_at[1], lo_at[2]);
                    const std::size_t p = index(hi_at[0], hi_at[1], hi_at[2]);
                    for (uint32_t c = 0; c < 3u; ++c) {
                        const float d = vel[p * 3u + c] - vel[m * 3u + c];
                        g[c][a] = d / (static_cast<float>(span) * h.cell);
                        finite = finite && std::isfinite(d);
                    }
                }
                const std::size_t cell = index(i, j, k);
                uint16_t& out = volume.s[cell * spade::kFlowHalvesPerCell + (spade::kFlowLambda2Hat % 4u)];
                const bool was_no_data = out == spade::fp16::kNoDataBits;
                const spade::fp16::Half half = spade::fp16::from_f32(finite ? spade::lambda2_hat(g)
                                                                            : std::numeric_limits<float>::quiet_NaN());
                out = half.bits;
                if (half.no_data && !was_no_data) ++h.no_data;
                if (!half.no_data && was_no_data) --h.no_data;
                if (half.no_data) continue;
                const float x = spade::fp16::to_f32(half.bits);
                lo = seen ? std::min(lo, x) : x;
                hi = seen ? std::max(hi, x) : x;
                seen = true;
            }
        }
    }
    h.channel_min[spade::kFlowLambda2Hat] = lo;
    h.channel_max[spade::kFlowLambda2Hat] = hi;
}

}  // namespace flow_movie
