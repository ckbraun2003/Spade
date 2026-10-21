#include "vehicles/airflow_solids.hpp"

#include <utility>

#include "vehicles/model_part.hpp"

// ---------------------------------------------------------------------------
// The adapter (see airflow_solids.hpp). Moved from the tests'
// airflow_vehicle_sampler.hpp, unchanged in behaviour, so a live run and the
// tests share one (TD-9).
// ---------------------------------------------------------------------------

namespace spade::vehicles {

namespace {

namespace air = physics::airflow;

[[nodiscard]] air::SolidQuery to_query(const PartSample& s, uint32_t index) noexcept {
    if (s.owner == kNoPart) return {};
    return {s.distance, s.gradient, index, s.velocity};
}

}  // namespace

AirflowSolids::AirflowSolids(const ModelType& model, PosedParts posed, glm::vec3 origin)
    : model_(&model), posed_(std::move(posed)), origin_(origin) {
    solid_of_.assign(model.parts.size(), air::kNoSolidPart);
    for (uint32_t i = 0; i < posed_.parts.size(); ++i) {
        solid_of_[posed_.parts[i].part] = i;
        PosedParts one = posed_;
        one.parts = {posed_.parts[i]};
        single_.push_back(std::move(one));
    }
}

air::SolidQuery AirflowSolids::nearest(const void* ctx, glm::vec3 x) noexcept {
    const auto& a = *static_cast<const AirflowSolids*>(ctx);
    const PartSample s = sample_parts(*a.model_, a.posed_, a.origin_ + x);
    return to_query(s, s.owner == kNoPart ? 0u : a.solid_of_[s.owner]);
}

air::SolidQuery AirflowSolids::part(const void* ctx, uint32_t p, glm::vec3 x) noexcept {
    const auto& a = *static_cast<const AirflowSolids*>(ctx);
    return to_query(sample_parts(*a.model_, a.single_[p], a.origin_ + x), p);
}

air::SolidSampler AirflowSolids::sampler() const noexcept {
    return {this, &AirflowSolids::nearest, &AirflowSolids::part};
}

Result<std::vector<air::SolidPart>> AirflowSolids::parts() const {
    std::vector<air::SolidPart> out;
    for (const PosedPart& pp : posed_.parts) {
        const PartDesc& d = model_->parts[pp.part];
        const Result<Aabb> b = part_bounds(d);
        if (!b) return std::unexpected(b.error());
        air::SolidPart sp;
        sp.name = d.name;
        sp.porous = d.surface.kind == PartSurface::Kind::porous;
        sp.min_feature = d.min_feature;
        sp.extent = b->max - b->min;
        glm::vec3 lo(1.0e30f);
        glm::vec3 hi(-1.0e30f);
        for (int c = 0; c < 8; ++c) {
            const glm::vec3 corner((c & 1) ? b->max.x : b->min.x, (c & 2) ? b->max.y : b->min.y,
                                   (c & 4) ? b->max.z : b->min.z);
            const glm::vec3 x = pp.origin + pp.orient * corner - origin_;
            lo = glm::min(lo, x);
            hi = glm::max(hi, x);
        }
        sp.box_min = lo;
        sp.box_max = hi;
        out.push_back(sp);
    }
    return out;
}

}  // namespace spade::vehicles
