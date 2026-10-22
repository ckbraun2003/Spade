#pragma once
#include <cstdint>
#include <string>
#include <string_view>
#include <vector>

#include <glm/glm.hpp>

#include "core/error.hpp"
#include "physics/airflow/solids.hpp"
#include "vehicles/model_type.hpp"
#include "vehicles/part_pose.hpp"

// ---------------------------------------------------------------------------
// A MODEL'S POSED PARTS AS AIRFLOW SOLIDS (the solids design, section 3, the
// layering). The adapter a live run hands set_solids() (physics/airflow/
// fluid_step.hpp) when the pose changes, and the one the tests use:
//
//   parts()    each posed part as the grid sees it: its name, porosity and
//              min_feature, and part_bounds()'s part-frame box -- its extent
//              for the classifier, its eight corners posed into the grid's
//              frame for the grid box (TD-9: one box rule);
//   sampler()  the nearest part, or one named part, through sample_parts()
//              (distance, gradient, the part's wall velocity), at a point in
//              the grid's frame: the world shifted by `origin`, the grid's
//              minimum corner.
//
// Part indices are the posed list's (SolidPart order), not the ModelType's.
// The sampler points at this object: it must outlive the set_solids() or
// build_solids() call that uses it, and is neither copied nor moved. A part
// part_bounds() refuses makes parts() return that refusal (it names the part).
//
// DRAWN ONLY (N15; Physics' ruling, 2026-10-22): a part whose role is
// kDrawnOnlyRole is drawn but is not a solid. It is left out of parts() and
// the sampler, as if it were not posed, and drawn_only() names it, in posed
// order, for the run line. Marking a part is the user's modelling choice; the
// solver never drops a part of its own accord. The role is the one mark, so
// PartDesc gains no field and model_identity() (which never folds parts)
// does not move.
// ---------------------------------------------------------------------------
namespace spade::vehicles {

inline constexpr std::string_view kDrawnOnlyRole = "imported-visual";

class AirflowSolids {
public:
    AirflowSolids(const ModelType& model, PosedParts posed, glm::vec3 origin);
    AirflowSolids(const AirflowSolids&) = delete;
    AirflowSolids& operator=(const AirflowSolids&) = delete;

    [[nodiscard]] Result<std::vector<physics::airflow::SolidPart>> parts() const;
    [[nodiscard]] physics::airflow::SolidSampler sampler() const noexcept;
    // The posed parts marked kDrawnOnlyRole, by name, in posed order.
    [[nodiscard]] const std::vector<std::string>& drawn_only() const noexcept { return drawn_only_; }

private:
    static physics::airflow::SolidQuery nearest(const void* ctx, glm::vec3 x) noexcept;
    static physics::airflow::SolidQuery part(const void* ctx, uint32_t p, glm::vec3 x) noexcept;

    const ModelType* model_ = nullptr;
    PosedParts posed_;
    glm::vec3 origin_{0.0f};               // the grid's minimum corner in the world, m
    std::vector<PosedParts> single_;       // one per posed part, for the per-part query
    std::vector<uint32_t> solid_of_;       // ModelType part index -> posed (SolidPart) index
    std::vector<std::string> drawn_only_;  // the posed parts left out, by name
};

}  // namespace spade::vehicles
