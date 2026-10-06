#include "render/flow_render.hpp"

#include <algorithm>
#include <cmath>
#include <string>

#include "core/fp32_math.hpp"

namespace spade::render {

Result<void> validate_flow_layer(const FlowLayer& layer) {
    const auto refuse = [](const std::string& why) {
        return std::unexpected(Error{Code::invalid_argument, "flow layer: " + why});
    };
    if (layer.volume == nullptr) {
        return refuse("it has no volume");
    }
    if (Result<void> valid = validate_flow_volume(*layer.volume); !valid) {
        return refuse(valid.error().context);
    }
    const std::size_t n = layer.transfers.size();
    if (n == 0u || n > kMaxFlowTransfers) {
        return refuse("it has " + std::to_string(n) + " transfer functions, not 1 to " +
                      std::to_string(kMaxFlowTransfers));
    }
    for (std::size_t i = 0; i < n; ++i) {
        if (Result<void> valid = validate_transfer_function(layer.transfers[i]); !valid) {
            return refuse("transfer function " + std::to_string(i) + ": " + valid.error().context);
        }
    }
    return {};
}

PreparedFlowLayer prepare_flow_layer(const FlowLayer& layer, const Lighting* lighting) {
    (void)lighting;  // RED: iso-surfaces are not marched yet
    const FlowVolumeHeader& h = layer.volume->header;
    PreparedFlowLayer p;
    p.sampler = make_flow_sampler(*layer.volume);
    for (const TransferFunction& tf : layer.transfers) {
        p.transfers.push_back(prepare_transfer(tf, h));
    }
    const glm::vec3 extent(static_cast<float>(h.nx), static_cast<float>(h.ny), static_cast<float>(h.nz));
    p.box = Aabb{h.origin, h.origin + extent * h.cell};
    p.step = 0.5f * h.cell;
    p.inv_step = 1.0f / p.step;
    return p;
}

FlowRayResult march_flow_ray(const PreparedFlowLayer& layer, glm::vec3 origin, glm::vec3 dir, float t_start,
                             float t_end) {
    FlowRayResult r;
    if (!(t_start < t_end)) {
        return r;
    }
    // Step i starts at t_start + i h, by multiplication, so no rounding
    // accumulates along the ray; the last step takes what is left.
    const float length = t_end - t_start;
    const float full = std::floor(length * layer.inv_step);
    const auto n_full = static_cast<uint32_t>(full);
    const float last = std::max(0.0f, length - full * layer.step);
    const uint32_t n = n_full + (last > 0.0f ? 1u : 0u);
    for (uint32_t i = 0; i < n; ++i) {
        const float t0 = t_start + static_cast<float>(i) * layer.step;
        const float dt = i < n_full ? layer.step : last;
        const glm::vec3 p = origin + dir * (t0 + 0.5f * dt);
        float sigma = 0.0f;
        glm::vec3 weighted(0.0f);
        bool no_data = false;
        for (const PreparedTransfer& transfer : layer.transfers) {
            const FlowValue v = sample_quantity(layer.sampler, transfer.tf.quantity, p);
            if (v.no_data) {
                no_data = true;
                break;
            }
            const TransferSample s = apply_transfer(transfer, v.value);
            sigma += s.sigma;
            weighted += s.sigma * s.colour;
        }
        if (no_data) {
            r.crossed_no_data = true;  // clear: this step adds nothing
            continue;
        }
        if (!(sigma > 0.0f)) {
            continue;
        }
        const float tau = math::exp32(-sigma * dt);
        r.colour += r.transmittance * (1.0f - tau) * (weighted / sigma);
        r.transmittance *= tau;
        if (r.transmittance < kFlowEarlyExitTransmittance) {
            break;
        }
    }
    return r;
}

}  // namespace spade::render
