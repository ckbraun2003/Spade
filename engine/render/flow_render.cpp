#include "render/flow_render.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <optional>
#include <string>

#include "core/fp32_math.hpp"
#include "render/scene.hpp"  // Lighting, Material, shade_vertex_color()

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
    const std::size_t m = layer.isos.size();
    if (n > kMaxFlowTransfers || m > kMaxIsoSurfaces || n + m == 0u) {
        return refuse("it has " + std::to_string(n) + " transfer functions and " + std::to_string(m) +
                      " iso-surfaces; each may be 0 to " + std::to_string(kMaxFlowTransfers) +
                      ", and one must be non-empty");
    }
    for (std::size_t i = 0; i < n; ++i) {
        if (Result<void> valid = validate_transfer_function(layer.transfers[i]); !valid) {
            return refuse("transfer function " + std::to_string(i) + ": " + valid.error().context);
        }
    }
    for (std::size_t i = 0; i < m; ++i) {
        const IsoSurface& iso = layer.isos[i];
        const std::string name = "iso-surface " + std::to_string(i);
        if (!std::isfinite(iso.value)) {
            return refuse(name + "'s level is not finite");
        }
        if (!(iso.opacity > 0.0f && iso.opacity <= 1.0f)) {
            return refuse(name + "'s opacity is outside (0, 1]");
        }
        for (const float c : {iso.colour.x, iso.colour.y, iso.colour.z}) {
            if (!(c >= 0.0f && c <= 1.0f)) return refuse(name + "'s colour is outside [0, 1]");
        }
        if (iso.colour_by) {
            if (Result<void> valid = validate_transfer_function(*iso.colour_by); !valid) {
                return refuse(name + "'s colour_by: " + valid.error().context);
            }
        }
    }
    return {};
}

PreparedFlowLayer prepare_flow_layer(const FlowLayer& layer, const Lighting* lighting) {
    const FlowVolumeHeader& h = layer.volume->header;
    PreparedFlowLayer p;
    p.sampler = make_flow_sampler(*layer.volume);
    for (const TransferFunction& tf : layer.transfers) {
        p.transfers.push_back(prepare_transfer(tf, h));
    }
    for (const IsoSurface& iso : layer.isos) {
        PreparedIso& prepared = p.isos.emplace_back();
        prepared.quantity = iso.quantity;
        prepared.value = iso.value;
        prepared.colour = iso.colour;
        prepared.opacity = iso.opacity;
        if (iso.colour_by.has_value()) {
            prepared.coloured_by = true;
            prepared.colour_by = prepare_transfer(iso.colour_by.value(), h);
        }
    }
    p.lighting = lighting;
    const glm::vec3 extent(static_cast<float>(h.nx), static_cast<float>(h.ny), static_cast<float>(h.nz));
    p.box = Aabb{h.origin, h.origin + extent * h.cell};
    p.step = 0.5f * h.cell;
    p.inv_step = 1.0f / p.step;
    return p;
}

namespace {

struct IsoHit {
    float t = 0.0f;
    std::size_t k = 0;
};

// Iso-surface k's offset from its level at t, or nullopt for no-data.
[[nodiscard]] std::optional<float> iso_offset(const PreparedFlowLayer& layer, std::size_t k, glm::vec3 origin,
                                              glm::vec3 dir, float t) {
    const FlowValue v = sample_quantity(layer.sampler, layer.isos[k].quantity, origin + dir * t);
    if (v.no_data) return std::nullopt;
    return v.value - layer.isos[k].value;
}

// The crossing in [ta, tb], whose offsets differ in sign: kIsoBisections
// halvings on the trilinear field, then linear interpolation in the last
// bracket. A no-data midpoint ends the refinement where it stands.
[[nodiscard]] float refine_crossing(const PreparedFlowLayer& layer, std::size_t k, glm::vec3 origin, glm::vec3 dir,
                                    float ta, float fa, float tb, float fb) {
    for (uint32_t i = 0; i < kIsoBisections; ++i) {
        const float tm = 0.5f * (ta + tb);
        const std::optional<float> fm = iso_offset(layer, k, origin, dir, tm);
        if (!fm) break;
        if ((fa < 0.0f) != (*fm < 0.0f)) {
            tb = tm;
            fb = *fm;
        } else {
            ta = tm;
            fa = *fm;
        }
    }
    const float span = fa - fb;
    return span != 0.0f ? ta + (tb - ta) * (fa / span) : 0.5f * (ta + tb);
}

// The colour an iso-surface shows at hit point q: its own, or colour_by's
// map at q; lit by the sun about the field's gradient, turned toward the
// eye, when the layer has lighting.
[[nodiscard]] glm::vec3 shade_iso(const PreparedFlowLayer& layer, const PreparedIso& iso, glm::vec3 q, glm::vec3 dir,
                                  FlowRayResult& r) {
    glm::vec3 base = iso.colour;
    if (iso.coloured_by) {
        const FlowValue v = sample_quantity(layer.sampler, iso.colour_by.tf.quantity, q);
        if (v.no_data) {
            r.crossed_no_data = true;  // the surface keeps its own colour there
        } else {
            base = apply_transfer(iso.colour_by, v.value).colour;
        }
    }
    if (layer.lighting == nullptr) {
        return base;
    }
    // Central differences one cell apart (two half-cell steps).
    const float h = layer.step;
    glm::vec3 grad(0.0f);
    bool ok = true;
    for (int a = 0; a < 3 && ok; ++a) {
        glm::vec3 e(0.0f);
        e[a] = h;
        const FlowValue hi = sample_quantity(layer.sampler, iso.quantity, q + e);
        const FlowValue lo = sample_quantity(layer.sampler, iso.quantity, q - e);
        ok = !hi.no_data && !lo.no_data;
        grad[a] = hi.value - lo.value;
    }
    const float len = std::sqrt(grad.x * grad.x + grad.y * grad.y + grad.z * grad.z);
    glm::vec3 normal = ok && len > 0.0f ? grad / len : -dir;
    if (normal.x * dir.x + normal.y * dir.y + normal.z * dir.z > 0.0f) {
        normal = -normal;  // two-sided: the side facing the eye
    }
    Material material;
    material.base_color = glm::vec4(base, 1.0f);
    return shade_vertex_color(material, *layer.lighting, normal).combined;
}

}  // namespace

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
    // Each iso-surface's previous sample: its offset from the level and t.
    const std::size_t n_isos = std::min(layer.isos.size(), kMaxIsoSurfaces);  // validated; bounds the arrays
    std::array<float, kMaxIsoSurfaces> prev_f{};
    std::array<float, kMaxIsoSurfaces> prev_t{};
    std::array<bool, kMaxIsoSurfaces> prev_valid{};
    for (uint32_t i = 0; i < n; ++i) {
        const float t0 = t_start + static_cast<float>(i) * layer.step;
        const float dt = i < n_full ? layer.step : last;
        const float tm = t0 + 0.5f * dt;
        const glm::vec3 p = origin + dir * tm;

        // Iso-surfaces crossed since the previous sample, nearest first.
        std::array<IsoHit, kMaxIsoSurfaces> hits{};
        std::size_t n_hits = 0;
        for (std::size_t k = 0; k < n_isos; ++k) {
            const FlowValue v = sample_quantity(layer.sampler, layer.isos[k].quantity, p);
            if (v.no_data) {
                r.crossed_no_data = true;
                prev_valid[k] = false;  // never a crossing through no-data
                continue;
            }
            const float f = v.value - layer.isos[k].value;
            if (prev_valid[k] && (prev_f[k] < 0.0f) != (f < 0.0f)) {
                hits[n_hits++] = IsoHit{refine_crossing(layer, k, origin, dir, prev_t[k], prev_f[k], tm, f), k};
            }
            prev_f[k] = f;
            prev_t[k] = tm;
            prev_valid[k] = true;
        }
        // Insertion sort: at most four, and std::sort's unrolled insertion
        // trips gcc 13's -O2 array-bounds analysis on a four-slot array.
        for (std::size_t a = 1; a < n_hits; ++a) {
            for (std::size_t b = a; b > 0 && hits[b].t < hits[b - 1].t; --b) {
                std::swap(hits[b], hits[b - 1]);
            }
        }
        for (std::size_t h = 0; h < n_hits; ++h) {
            const PreparedIso& iso = layer.isos[hits[h].k];
            const glm::vec3 shaded = shade_iso(layer, iso, origin + dir * hits[h].t, dir, r);
            r.colour += r.transmittance * iso.opacity * shaded;
            if (iso.opacity >= 1.0f) {
                r.transmittance = 0.0f;
                r.opaque_t = hits[h].t;
                return r;
            }
            r.transmittance *= 1.0f - iso.opacity;
        }

        if (layer.transfers.empty()) {
            continue;
        }
        float sigma = 0.0f;
        glm::vec3 weighted(0.0f);
        bool no_data = false;
        for (const PreparedTransfer& transfer : layer.transfers) {
            const FlowValue v = sample_quantity(layer.sampler, transfer.tf.quantity, p);
            if (v.no_data) {
                no_data = true;
                break;
            }
            const TransferSample ts = apply_transfer(transfer, v.value);
            sigma += ts.sigma;
            weighted += ts.sigma * ts.colour;
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
