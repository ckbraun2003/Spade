// tests/reference_airframe.hpp -- the reference quad's AirframeSpec: one
// fixture (TD-9) for the airframe compile's tests (test_airframe_compile.cpp)
// and the parts drawing's (test_render_parts.cpp). Moved here unchanged from
// test_airframe_compile.cpp; the airframe compile's pinned identities read it.

#pragma once

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include "vehicles/airframe_compile.hpp"

namespace spade::vehicles::fixtures {

inline constexpr double kArm = 0.11;

[[nodiscard]] inline PartInertia make_part(PartShape shape, double mass, glm::dvec3 pos, glm::dvec3 size,
                                           glm::dquat orient = glm::dquat(1.0, 0.0, 0.0, 0.0)) {
    PartInertia p;
    p.shape = shape;
    p.mass = mass;
    p.position = pos;
    p.size = size;
    p.orientation = orient;
    return p;
}

// A 5-inch plus-layout quad (Y up, nose +X), 0.526 kg, in its design frame.
[[nodiscard]] inline AirframeSpec reference_quad() {
    AirframeSpec s;
    s.name = "ref_quad";
    s.param_schema_id = 1;
    s.visual_ref = "meshes/ref_quad";
    s.parts.push_back(make_part(PartShape::box, 0.12, glm::dvec3(0.0), glm::dvec3(0.12, 0.03, 0.12)));
    // 90-degree turns as exact literals, not glm::angleAxis: its libm cos and
    // sin would put a platform's rounding into the pinned identity's input.
    const double h = 0.7071067811865476;  // sqrt(1/2)
    const glm::dquat along_x(h, 0.0, 0.0, h);  // about +Z: the tube's +Y onto -X
    const glm::dquat along_z(h, h, 0.0, 0.0);  // about +X: the tube's +Y onto +Z
    const glm::dvec3 tube(0.006, kArm, 0.0);
    s.parts.push_back(make_part(PartShape::tube, 0.015, glm::dvec3(+kArm / 2, 0.0, 0.0), tube, along_x));
    s.parts.push_back(make_part(PartShape::tube, 0.015, glm::dvec3(0.0, 0.0, +kArm / 2), tube, along_z));
    s.parts.push_back(make_part(PartShape::tube, 0.015, glm::dvec3(-kArm / 2, 0.0, 0.0), tube, along_x));
    s.parts.push_back(make_part(PartShape::tube, 0.015, glm::dvec3(0.0, 0.0, -kArm / 2), tube, along_z));

    s.motor.kv = 2450.0;
    s.motor.resistance = 0.07;
    s.motor.no_load_current = 1.2;
    s.motor.no_load_voltage = 10.0;
    s.motor.current_max = 45.0;
    s.motor.pole_pairs = 7.0;
    s.motor.rotor_inertia = 1.2e-5;
    s.motor.stator = "2207";
    s.motor.mass = 0.033;
    s.motor.shape = BlockShape{PartShape::cylinder, glm::dvec3(0.014, 0.018, 0.0), glm::dmat3(0.0)};

    s.prop.diameter = 0.1524;
    s.prop.pitch = 0.1016;
    s.prop.blades = 3;
    s.prop.ct_table = {{0.0, 0.12}, {0.8, 0.04}};
    s.prop.cq_table = {{0.0, 0.008}, {0.8, 0.004}};
    s.prop.mass = 0.006;

    s.esc.current_burst = 55.0;
    s.esc.current_total = 160.0;
    s.esc.channels = 4;
    s.esc.on_resistance = 0.005;
    s.esc.mass = 0.01;
    s.esc.mount.position = glm::dvec3(0.0, 0.01, 0.0);

    s.battery.cells_series = 4;
    s.battery.cells_parallel = 1;
    s.battery.cell_capacity = 1.5;
    s.battery.cell_resistance = 0.005;
    s.battery.cell_voltage_cutoff = 3.0;
    s.battery.ocv_table = {{0.0, 3.3}, {0.2, 3.6}, {0.5, 3.75}, {0.8, 3.95}, {1.0, 4.2}};
    s.battery.c_rating = 100.0;
    s.battery.mass = 0.18;
    s.battery.mount.position = glm::dvec3(0.0, -0.03, 0.0);
    s.battery.shape = BlockShape{PartShape::box, glm::dvec3(0.07, 0.035, 0.035), glm::dmat3(0.0)};

    const double spins[4] = {1.0, -1.0, 1.0, -1.0};
    const glm::dvec3 hubs[4] = {glm::dvec3(kArm, 0.02, 0.0), glm::dvec3(0.0, 0.02, kArm),
                                glm::dvec3(-kArm, 0.02, 0.0), glm::dvec3(0.0, 0.02, -kArm)};
    for (int k = 0; k < 4; ++k) {
        RotorSpec r;
        r.hub.position = hubs[k];
        r.spin_dir = spins[k];
        s.rotors.push_back(r);
    }
    ImuSpec imu;
    imu.mount.position = glm::dvec3(0.0, 0.005, 0.0);
    s.imus.push_back(imu);
    s.proxy_radius = 0.15;
    return s;
}

}  // namespace spade::vehicles::fixtures
