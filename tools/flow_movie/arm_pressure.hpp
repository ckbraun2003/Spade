// D1's arm-top pressure for Physics' record (docs/design/test-docs/
// 2026-10-21-d1-hover-checks.md section 2: the sign of C_p on each arm's top,
// under its disc). Each rotor's arm is sampled where it is bare -- from where
// it leaves the body, or the disc's inner edge if that is farther out, to the
// motor's edge (Physics, 2026-10-21) -- one cell above the arm's own top, in
// the published pressure as published_pressure_at() reads it
// (tests/test_airflow_solids_step.cpp): trilinear in the cell-centre
// pressure, P6's first solid layer included, the weights renormalised over
// the corners that hold a value. Read from a cached frame's FlowVolume, so
// to fp16; Physics compares means against the inflow's pressure.
#pragma once

#include <cstdint>
#include <vector>

#include <glm/vec3.hpp>

#include "core/error.hpp"
#include "core/flow_volume.hpp"
#include "state/layout.hpp"  // BodyState
#include "vehicles/model_type.hpp"

namespace flow_movie {

struct ArmSample {
    uint32_t rotor = 0;
    float from_centre = 0.0f;  // m, along the arm from the body's centre
    glm::vec3 at{0.0f};        // world, m: one cell above the arm's top
    double pressure = 0.0;     // Pa, gauge; NaN where no corner holds a value
};

struct ArmPressure {
    uint32_t rotor = 0;
    float span_from = 0.0f;  // m from the body's centre: the bare arm under the disc
    float span_to = 0.0f;
    uint32_t samples = 0;    // those with a value
    double mean = 0.0;       // Pa, over them; NaN with none
};

// The published pressure at `world` (m), as above; NaN where no corner of
// the eight holds a value or the point is outside the volume.
[[nodiscard]] double published_pressure_at(const spade::FlowVolume& volume, glm::vec3 world);

// One row per rotor, in rotor order, and its samples appended to `samples`:
// about one a cell along the bare span. The arm of rotor k is the
// body-attached part, not a motor, whose mount lies between the body's centre
// and the hub (the reference quad's parts[1..4]); a model without one is
// refused, naming the rotor. `body` is the frame's held body (FrameLog).
[[nodiscard]] spade::Result<std::vector<ArmPressure>> arm_pressures(const spade::vehicles::ModelType& model,
                                                                    const spade::BodyState& body,
                                                                    const spade::FlowVolume& volume,
                                                                    std::vector<ArmSample>& samples);

}  // namespace flow_movie
