// ---------------------------------------------------------------------------
// vehicles/part_pose.cpp -- see part_pose.hpp.
//
// RED STUB (the parts and meshes plan, Task A2 Step 2): every function
// returns its empty or default answer, so the tests fail on behaviour.
// ---------------------------------------------------------------------------
#include "vehicles/part_pose.hpp"

namespace spade::vehicles {

Result<PosedParts> posed_parts(const ModelType& /*model*/, const BodyState& /*body*/,
                               std::span<const float> /*rotor_omega*/, std::span<const float> /*psi*/,
                               PartFilter /*filter*/) {
    return PosedParts{};
}

Result<SdfProgram> posed_program(const ModelType& /*model*/, const PosedParts& /*posed*/) {
    return SdfProgram{};
}

glm::vec3 part_wall_velocity(const PosedParts& /*posed*/, std::size_t /*i*/, glm::vec3 /*x*/) noexcept {
    return glm::vec3(0.0f);
}

SdfSample part_sample(const PartDesc& /*part*/, glm::vec3 /*p_part*/) noexcept {
    return SdfSample{};
}

PartSample sample_parts(const ModelType& /*model*/, const PosedParts& /*posed*/, glm::vec3 /*x*/) noexcept {
    return PartSample{};
}

}  // namespace spade::vehicles
