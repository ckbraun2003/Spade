#pragma once
// vehicles::sample_parts() as the airflow core's SolidSampler (the solids
// design, section 3, the layering): the tests' adapter, and the shape of the
// D1 demo's and plan C's. The grid frame is the world shifted by `origin`, the
// grid's minimum corner. Each part's box is vehicles::part_bounds()'s (its part
// frame), posed by its part frame's world pose (TD-9).
#include <cstdint>
#include <string>
#include <utility>
#include <vector>

#include <glm/glm.hpp>

#include "physics/airflow/solids.hpp"
#include "vehicles/model_part.hpp"
#include "vehicles/model_type.hpp"
#include "vehicles/part_pose.hpp"

namespace airflow_test {

struct VehicleSampler {
    const spade::vehicles::ModelType* model = nullptr;
    spade::vehicles::PosedParts posed;
    glm::vec3 origin{0.0f};                           // the grid's minimum corner in the world, m
    std::vector<spade::vehicles::PosedParts> single;  // one per posed part, for the per-part query
    std::vector<uint32_t> solid_of;                   // ModelType part index -> posed (SolidPart) index

    VehicleSampler(const spade::vehicles::ModelType& m, spade::vehicles::PosedParts p, glm::vec3 o)
        : model(&m), posed(std::move(p)), origin(o) {
        solid_of.assign(m.parts.size(), spade::physics::airflow::kNoSolidPart);
        for (uint32_t i = 0; i < posed.parts.size(); ++i) {
            solid_of[posed.parts[i].part] = i;
            spade::vehicles::PosedParts one = posed;
            one.parts = {posed.parts[i]};
            single.push_back(std::move(one));
        }
    }

    [[nodiscard]] static spade::physics::airflow::SolidQuery to_query(const spade::vehicles::PartSample& s,
                                                                      uint32_t index) noexcept {
        if (s.owner == spade::vehicles::kNoPart) return {};
        return {s.distance, s.gradient, index, s.velocity};
    }

    static spade::physics::airflow::SolidQuery nearest(const void* ctx, glm::vec3 x) noexcept {
        const auto& v = *static_cast<const VehicleSampler*>(ctx);
        const spade::vehicles::PartSample s = spade::vehicles::sample_parts(*v.model, v.posed, v.origin + x);
        return to_query(s, s.owner == spade::vehicles::kNoPart ? 0u : v.solid_of[s.owner]);
    }

    static spade::physics::airflow::SolidQuery part(const void* ctx, uint32_t p, glm::vec3 x) noexcept {
        const auto& v = *static_cast<const VehicleSampler*>(ctx);
        return to_query(spade::vehicles::sample_parts(*v.model, v.single[p], v.origin + x), p);
    }

    [[nodiscard]] spade::physics::airflow::SolidSampler sampler() const {
        return {this, &VehicleSampler::nearest, &VehicleSampler::part};
    }

    // The posed parts as the grid sees them: part_bounds()'s part-frame box,
    // its eight corners posed into the grid's frame. A part part_bounds()
    // refuses is returned as the refusal (the caller fails by name).
    [[nodiscard]] spade::Result<std::vector<spade::physics::airflow::SolidPart>> parts() const {
        std::vector<spade::physics::airflow::SolidPart> out;
        for (const spade::vehicles::PosedPart& pp : posed.parts) {
            const spade::vehicles::PartDesc& d = model->parts[pp.part];
            const spade::Result<spade::Aabb> b = spade::vehicles::part_bounds(d);
            if (!b) return std::unexpected(b.error());
            spade::physics::airflow::SolidPart sp;
            sp.name = d.name;
            sp.porous = d.surface.kind == spade::vehicles::PartSurface::Kind::porous;
            sp.min_feature = d.min_feature;
            sp.extent = b->max - b->min;
            glm::vec3 lo(1.0e30f);
            glm::vec3 hi(-1.0e30f);
            for (int c = 0; c < 8; ++c) {
                const glm::vec3 corner((c & 1) ? b->max.x : b->min.x, (c & 2) ? b->max.y : b->min.y,
                                       (c & 4) ? b->max.z : b->min.z);
                const glm::vec3 x = pp.origin + pp.orient * corner - origin;
                lo = glm::min(lo, x);
                hi = glm::max(hi, x);
            }
            sp.box_min = lo;
            sp.box_max = hi;
            out.push_back(sp);
        }
        return out;
    }
};

}  // namespace airflow_test
