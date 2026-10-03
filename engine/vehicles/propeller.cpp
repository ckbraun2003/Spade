#include "vehicles/propeller.hpp"

// STUB: every function returns 0 so the tests run red for the right reason.
// The implementation replaces this file in the next commit.

namespace spade::vehicles {

Result<PropellerTable> propeller_resample_table(std::span<const std::array<double, 2>>,
                                                std::span<const std::array<double, 2>>, uint32_t count) {
    PropellerTable t;
    t.j_max = 1.0f;
    t.ct.assign(count < 2 ? 2 : count, 0.0f);
    t.cq.assign(count < 2 ? 2 : count, 0.0f);
    return t;
}
float propeller_advance_ratio(float, float, float) noexcept { return 0.0f; }
double propeller_advance_ratio(double, double, double) noexcept { return 0.0; }
float propeller_coefficient(std::span<const float>, float, float, float, uint32_t&) noexcept { return 0.0f; }
double propeller_coefficient(std::span<const float>, double, double, double, uint32_t&) noexcept { return 0.0; }
float propeller_thrust(float, float, float, float) noexcept { return 0.0f; }
double propeller_thrust(double, double, double, double) noexcept { return 0.0; }
float propeller_torque(float, float, float, float) noexcept { return 0.0f; }
double propeller_torque(double, double, double, double) noexcept { return 0.0; }

}  // namespace spade::vehicles
