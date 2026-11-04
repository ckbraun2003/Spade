#include "physics/contact_sets.hpp"

#include <algorithm>
#include <array>

#include "vehicles/model_part.hpp"
#include "vehicles/part_pose.hpp"

namespace spade::physics {

namespace {

// cos and sin of k * 45 degrees, k = 0..7, as constants (no libm, L10).
constexpr float kHalfRoot2 = 0.70710677f;
constexpr std::array<float, 8> kRimCos = {1.0f, kHalfRoot2, 0.0f, -kHalfRoot2, -1.0f, -kHalfRoot2, 0.0f, kHalfRoot2};
constexpr std::array<float, 8> kRimSin = {0.0f, kHalfRoot2, 1.0f, kHalfRoot2, 0.0f, -kHalfRoot2, -1.0f, -kHalfRoot2};

[[nodiscard]] bool has_body_part(const vehicles::ModelType& model) {
    return std::any_of(model.parts.begin(), model.parts.end(),
                       [](const vehicles::PartDesc& p) { return p.attach == vehicles::PartAttach::body; });
}

}  // namespace

std::vector<glm::vec3> contact_points(const vehicles::ModelType& model) {
    std::vector<glm::vec3> out;
    if (!has_body_part(model)) return out;
    // The body frame: the identity pose puts the centre of mass at the origin
    // and the principal axes on the world's.
    BodyState identity{};
    identity.orient = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
    const std::vector<float> still(model.rotors.size(), 0.0f);
    const Result<vehicles::PosedParts> posed =
        vehicles::posed_parts(model, identity, still, still, vehicles::PartFilter::rigid_only);
    if (posed) {
        for (const vehicles::PosedPart& p : posed->parts) {
            const Result<Aabb> box = vehicles::part_bounds(model.parts[p.part]);
            if (!box) continue;
            for (uint32_t c = 0; c < 8u; ++c) {
                const glm::vec3 corner((c & 1u) != 0u ? box->max.x : box->min.x,
                                       (c & 2u) != 0u ? box->max.y : box->min.y,
                                       (c & 4u) != 0u ? box->max.z : box->min.z);
                out.push_back(p.origin + p.orient * corner);
            }
        }
    }
    for (const vehicles::RotorDesc& r : model.rotors) {
        const glm::vec3 x_hub = r.local_orient * glm::vec3(1.0f, 0.0f, 0.0f);
        const glm::vec3 z_hub = r.local_orient * glm::vec3(0.0f, 0.0f, 1.0f);
        for (uint32_t k = 0; k < 8u; ++k) {
            out.push_back(r.local_pos + r.radius * (kRimCos[k] * x_hub + kRimSin[k] * z_hub));
        }
    }
    return out;
}

void build_contact_sets(std::span<const vehicles::ModelType> models, std::vector<float>& out) {
    std::vector<std::vector<glm::vec3>> sets;
    sets.reserve(models.size());
    bool any = false;
    for (const vehicles::ModelType& m : models) {
        sets.push_back(contact_points(m));
        any = any || !sets.back().empty();
    }
    if (!any) return;  // no set: the table is empty, a part-less registry's identity master's
    uint32_t first = static_cast<uint32_t>(models.size());
    for (const std::vector<glm::vec3>& s : sets) {
        const auto count = static_cast<uint32_t>(s.size());
        const float f = count == 0u ? 0.0f : static_cast<float>(first);
        out.insert(out.end(), {f, static_cast<float>(count), 0.0f, 0.0f});
        first += count;
    }
    for (const std::vector<glm::vec3>& s : sets) {
        for (const glm::vec3& p : s) out.insert(out.end(), {p.x, p.y, p.z, 0.0f});
    }
}

std::optional<float> lowest_contact_point(const vehicles::ModelType& model, const BodyState& body) {
    const std::vector<glm::vec3> points = contact_points(model);
    std::optional<float> low;
    for (const glm::vec3& p : points) {
        const float y = contact_point_world(body, p).y;
        low = low ? std::min(*low, y) : y;
    }
    return low;
}

}  // namespace spade::physics
