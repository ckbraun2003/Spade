#include "render/transfer_function.hpp"

#include <algorithm>
#include <cmath>
#include <string>

// The colour tables live in colour_maps.cpp, generated, with their provenance.

namespace spade::render {
namespace {

[[nodiscard]] uint32_t channel_of(FlowQuantity q) {
    switch (q) {
        case FlowQuantity::u:
            return kFlowU;
        case FlowQuantity::v:
            return kFlowV;
        case FlowQuantity::w:
            return kFlowW;
        case FlowQuantity::dye:
            return kFlowDye;
        case FlowQuantity::pressure:
            return kFlowPressure;
        case FlowQuantity::vorticity:
            return kFlowVorticity;
        case FlowQuantity::q_hat:
            return kFlowQHat;
        case FlowQuantity::lambda2_hat:
            return kFlowLambda2Hat;
        case FlowQuantity::speed:
            break;
    }
    return kFlowU;  // speed reads three channels; resolve_range() handles it apart
}

[[nodiscard]] float largest_magnitude(const FlowVolumeHeader& h, uint32_t channel) {
    return std::max(std::fabs(h.channel_min[channel]), std::fabs(h.channel_max[channel]));
}

}  // namespace

glm::vec3 colour_map_at(ColourMapId id, float t) {
    const auto& table = colour_map_table(id);
    const float x = std::clamp(t, 0.0f, 1.0f) * static_cast<float>(kColourMapEntries - 1);
    if (!(x < static_cast<float>(kColourMapEntries - 1))) return table.back();  // also a NaN t
    const auto i = static_cast<std::size_t>(x);
    const float u = x - static_cast<float>(i);
    const glm::vec3& a = table[i];
    const glm::vec3& b = table[i + 1];
    return glm::vec3(a.x + (b.x - a.x) * u, a.y + (b.y - a.y) * u, a.z + (b.z - a.z) * u);
}

Result<void> validate_transfer_function(const TransferFunction& tf) {
    const auto refuse = [](const std::string& why) {
        return std::unexpected(Error{Code::invalid_argument, "transfer function: " + why});
    };
    if (!tf.auto_range &&
        !(std::isfinite(tf.range_min) && std::isfinite(tf.range_max) && tf.range_min < tf.range_max)) {
        return refuse("its fixed range is not finite with min < max");
    }
    const std::size_t n = tf.extinction.size();
    if (n < 2u || n > kMaxExtinctionPoints) {
        return refuse("it has " + std::to_string(n) + " extinction points, not 2 to " +
                      std::to_string(kMaxExtinctionPoints));
    }
    for (std::size_t i = 0; i < n; ++i) {
        const ExtinctionPoint& p = tf.extinction[i];
        if (!std::isfinite(p.value) || (i > 0u && !(p.value > tf.extinction[i - 1u].value))) {
            return refuse("extinction point " + std::to_string(i) + "'s value is not finite and strictly ascending");
        }
        if (!(std::isfinite(p.sigma) && p.sigma >= 0.0f)) {
            return refuse("extinction point " + std::to_string(i) + "'s sigma is not finite and >= 0");
        }
    }
    if (tf.constant_colour) {
        const glm::vec3& c = *tf.constant_colour;
        for (const float x : {c.x, c.y, c.z}) {
            if (!(x >= 0.0f && x <= 1.0f)) return refuse("its constant colour is outside [0, 1]");
        }
    }
    return {};
}

ResolvedRange resolve_range(const TransferFunction& tf, const FlowVolumeHeader& header) {
    if (!tf.auto_range) return ResolvedRange{tf.range_min, tf.range_max, false};
    float lo = 0.0f;
    float hi = 0.0f;
    if (tf.quantity == FlowQuantity::speed) {
        float sum = 0.0f;
        for (const uint32_t c : {kFlowU, kFlowV, kFlowW}) {
            const float m = largest_magnitude(header, c);
            sum += m * m;
        }
        hi = std::sqrt(sum);
    } else {
        const uint32_t c = channel_of(tf.quantity);
        lo = header.channel_min[c];
        hi = header.channel_max[c];
    }
    if (tf.colours == ColourMapId::cool_warm && !tf.constant_colour) {
        const float m = std::max(std::fabs(lo), std::fabs(hi));
        lo = -m;
        hi = m;
    }
    return ResolvedRange{lo, hi, true};
}

PreparedTransfer prepare_transfer(const TransferFunction& tf, const FlowVolumeHeader& header) {
    PreparedTransfer p;
    p.tf = tf;
    p.range = resolve_range(tf, header);
    const float span = p.range.max - p.range.min;
    p.inv_span = span > 0.0f && std::isfinite(span) ? 1.0f / span : 0.0f;
    for (std::size_t i = 0; i + 1u < tf.extinction.size() && i < p.extinction_inv_width.size(); ++i) {
        p.extinction_inv_width[i] = 1.0f / (tf.extinction[i + 1u].value - tf.extinction[i].value);
    }
    return p;
}

TransferSample apply_transfer(const PreparedTransfer& prepared, float value) {
    const TransferFunction& tf = prepared.tf;
    TransferSample out;
    out.colour = tf.constant_colour ? *tf.constant_colour
                                    : colour_map_at(tf.colours, (value - prepared.range.min) * prepared.inv_span);
    const std::vector<ExtinctionPoint>& e = tf.extinction;
    if (!(value > e.front().value)) {
        out.sigma = e.front().sigma;
    } else if (!(value < e.back().value)) {
        out.sigma = e.back().sigma;
    } else {
        std::size_t i = 0;
        while (!(value < e[i + 1u].value)) ++i;
        const float u = (value - e[i].value) * prepared.extinction_inv_width[i];
        out.sigma = e[i].sigma + (e[i + 1u].sigma - e[i].sigma) * u;
    }
    return out;
}

}  // namespace spade::render
