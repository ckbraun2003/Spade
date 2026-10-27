#include "arm_pressure.hpp"

#include <cmath>
#include <limits>
#include <optional>
#include <string>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include "core/fp16.hpp"
#include "vehicles/airflow_solids.hpp"
#include "vehicles/part_pose.hpp"

namespace flow_movie {
namespace {

constexpr float kScan = 0.0005f;   // m: the step along each arm and up through its top
constexpr float kAbove = 0.0005f;  // m over the top where another part's cover is tested
constexpr float kMaxRise = 0.05f;  // m: no arm is thicker than this

}  // namespace

double published_pressure_at(const spade::FlowVolume& volume, glm::vec3 world) {
    const spade::FlowVolumeHeader& h = volume.header;
    const glm::dvec3 q = glm::dvec3(world - h.origin) / static_cast<double>(h.cell) - 0.5;
    const glm::ivec3 b(static_cast<int>(std::floor(q.x)), static_cast<int>(std::floor(q.y)),
                       static_cast<int>(std::floor(q.z)));
    const glm::dvec3 t = q - glm::dvec3(b);
    double p = 0.0;
    double weight = 0.0;
    for (int n = 0; n < 8; ++n) {
        const glm::ivec3 c = b + glm::ivec3(n & 1, (n >> 1) & 1, (n >> 2) & 1);
        if (c.x < 0 || c.y < 0 || c.z < 0 || c.x >= static_cast<int>(h.nx) || c.y >= static_cast<int>(h.ny) ||
            c.z >= static_cast<int>(h.nz)) {
            continue;
        }
        const double w = ((n & 1) != 0 ? t.x : 1.0 - t.x) * ((n & 2) != 0 ? t.y : 1.0 - t.y) *
                         ((n & 4) != 0 ? t.z : 1.0 - t.z);
        const float v = spade::fp16::to_f32(
            volume.s[spade::flow_cell_index(h, static_cast<uint32_t>(c.x), static_cast<uint32_t>(c.y),
                                            static_cast<uint32_t>(c.z)) *
                     spade::kFlowHalvesPerCell]);
        if (!std::isfinite(v) || w <= 0.0) continue;
        p += w * v;
        weight += w;
    }
    return weight > 0.0 ? p / weight : std::numeric_limits<double>::quiet_NaN();
}

spade::Result<std::vector<ArmPressure>> arm_pressures(const spade::vehicles::ModelType& model,
                                                      const spade::BodyState& body, const spade::FlowVolume& volume,
                                                      std::vector<ArmSample>& samples) {
    namespace sv = spade::vehicles;
    const std::vector<float> still(model.rotors.size(), 0.0f);
    auto posed = sv::posed_parts(model, body, still, still, sv::PartFilter::rigid_only);
    if (!posed) return std::unexpected(posed.error());
    // Which posed part each model part is, before AirflowSolids takes them.
    std::vector<uint32_t> posed_index(model.parts.size(), ~0u);
    for (uint32_t i = 0; i < posed->parts.size(); ++i) posed_index[posed->parts[i].part] = i;
    const uint32_t posed_count = static_cast<uint32_t>(posed->parts.size());
    const sv::AirflowSolids solids(model, std::move(*posed), volume.header.origin);
    const auto sampler = solids.sampler();
    const glm::quat q_body = glm::normalize(body.orient);
    const glm::vec3 up = q_body * glm::vec3(0.0f, 1.0f, 0.0f);
    const auto world = [&](glm::vec3 p_body) { return body.pos + q_body * p_body; };
    const auto inside = [&](uint32_t part, glm::vec3 w) {
        return sampler.part(sampler.ctx, part, w - volume.header.origin).distance < 0.0f;
    };
    const float cell = volume.header.cell;

    std::vector<ArmPressure> rows;
    for (uint32_t k = 0; k < model.rotors.size(); ++k) {
        const glm::vec3 hub = model.rotors[k].local_pos;
        const glm::vec3 hub_flat(hub.x, 0.0f, hub.z);
        const float reach = glm::length(hub_flat);
        if (!(reach > 0.0f)) {
            return std::unexpected(spade::Error{spade::Code::invalid_argument,
                                                "arm_pressures: rotor " + std::to_string(k) + " sits on the centre"});
        }
        const glm::vec3 dir = hub_flat / reach;
        // Its arm: a body part, not a motor, mounted between the centre and
        // the hub, nearest halfway.
        std::optional<uint32_t> arm;
        float best = 0.0f;
        for (uint32_t i = 0; i < model.parts.size(); ++i) {
            const sv::PartDesc& d = model.parts[i];
            if (d.attach != sv::PartAttach::body || d.role == "motor" || posed_index[i] == ~0u) continue;
            const glm::vec3 m(d.mount_pos.x, 0.0f, d.mount_pos.z);
            const float along = glm::dot(m, dir);
            const float off = glm::length(m - along * dir);
            if (along < 0.2f * reach || along > 0.8f * reach || off > 0.25f * reach) continue;
            const float miss = std::abs(along - 0.5f * reach);
            if (!arm || miss < best) {
                arm = i;
                best = miss;
            }
        }
        if (!arm) {
            return std::unexpected(spade::Error{spade::Code::invalid_argument,
                                                "arm_pressures: no arm found for rotor " + std::to_string(k) +
                                                    ": no body part, not a motor, mounted between the centre and "
                                                    "its hub"});
        }
        const uint32_t a = posed_index[*arm];
        const float arm_y = model.parts[*arm].mount_pos.y;
        // The arm's top over the point r along it, world; none where r is
        // not in the arm.
        const auto top_at = [&](float r) -> std::optional<glm::vec3> {
            glm::vec3 p = world(dir * r + glm::vec3(0.0f, arm_y, 0.0f));
            if (!inside(a, p)) return std::nullopt;
            for (float rise = 0.0f; rise < kMaxRise; rise += kScan) {
                if (!inside(a, p + up * (rise + kScan))) return p + up * (rise + kScan);
            }
            return std::nullopt;
        };
        // Bare: the arm, under the disc, with no other part over its top.
        const auto bare = [&](float r) {
            if (reach - r > model.rotors[k].radius) return false;
            const std::optional<glm::vec3> top = top_at(r);
            if (!top) return false;
            for (uint32_t j = 0; j < posed_count; ++j) {
                if (j != a && inside(j, *top + up * kAbove)) return false;
            }
            return true;
        };
        // The bare run nearest the hub, scanned inward.
        ArmPressure row{.rotor = k, .span_from = 0.0f, .span_to = 0.0f, .samples = 0,
                        .mean = std::numeric_limits<double>::quiet_NaN()};
        bool in_run = false;
        for (float r = reach; r > 0.0f; r -= kScan) {
            if (bare(r)) {
                if (!in_run) row.span_to = r;
                row.span_from = r;
                in_run = true;
            } else if (in_run) {
                break;
            }
        }
        if (in_run && row.span_to > row.span_from) {
            const float span = row.span_to - row.span_from;
            const uint32_t n = std::max(1u, static_cast<uint32_t>(std::lround(span / cell)));
            double sum = 0.0;
            for (uint32_t j = 0; j < n; ++j) {
                const float r = row.span_from + (static_cast<float>(j) + 0.5f) * span / static_cast<float>(n);
                const std::optional<glm::vec3> top = top_at(r);
                if (!top) continue;
                ArmSample s{.rotor = k, .from_centre = r, .at = *top + up * cell, .pressure = 0.0};
                s.pressure = published_pressure_at(volume, s.at);
                if (std::isfinite(s.pressure)) {
                    sum += s.pressure;
                    ++row.samples;
                }
                samples.push_back(s);
            }
            if (row.samples > 0u) row.mean = sum / static_cast<double>(row.samples);
        }
        rows.push_back(row);
    }
    return rows;
}

}  // namespace flow_movie
