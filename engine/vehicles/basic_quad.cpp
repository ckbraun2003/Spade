#include "vehicles/basic_quad.hpp"

#include <array>
#include <charconv>
#include <cmath>
#include <cstddef>
#include <optional>
#include <span>
#include <string>
#include <utility>
#include <vector>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

namespace spade::vehicles {
namespace {

[[nodiscard]] Error invalid(std::string context) { return Error{Code::invalid_argument, std::move(context)}; }

[[nodiscard]] bool positive(double x) noexcept { return std::isfinite(x) && x > 0.0; }

// A number in a message: the shortest form that reads back to it.
[[nodiscard]] std::string text(double x) {
    char buf[32];
    const std::to_chars_result r = std::to_chars(buf, buf + sizeof(buf), x);
    return std::string(buf, r.ptr);
}

constexpr std::array<const char*, 3> kAxes = {"x", "y", "z"};

// The turn taking +Y onto the horizontal unit vector (dx, 0, dz): about
// Y x d = (dz, 0, -dx) by 90 degrees, so q = (1, dz, 0, -dx) / sqrt(2). Only
// sqrt (TD-3).
[[nodiscard]] glm::dquat y_onto(double dx, double dz) {
    const double s = 1.0 / std::sqrt(2.0);
    return glm::dquat(s, dz * s, 0.0, -dx * s);
}

[[nodiscard]] PartInertia record(std::string name, std::string role, double mass, glm::dvec3 position,
                                 glm::dquat orientation, PartShape shape, glm::dvec3 size) {
    PartInertia p;
    p.mass = mass;
    p.position = position;
    p.orientation = orientation;
    p.shape = shape;
    p.size = size;
    p.geometry.name = std::move(name);
    p.geometry.role = std::move(role);
    return p;
}

// The parts' mass as the compile gathers it: the records, then a motor and a
// propeller per rotor, the ESC and the battery (vehicles/airframe_compile.hpp,
// PARTS). A block of zero mass adds nothing either way.
[[nodiscard]] double parts_mass(const AirframeSpec& s) {
    double m = 0.0;
    for (const PartInertia& p : s.parts) m += p.mass;
    for (std::size_t k = 0; k < s.rotors.size(); ++k) {
        m += s.motor.mass;
        m += s.prop.mass;
    }
    m += s.esc.mass;
    m += s.battery.mass;
    return m;
}

}  // namespace

MotorBlock BasicQuadParams::default_motor() {
    MotorBlock m;
    m.kv = 1700.0;
    m.resistance = 0.06;
    m.no_load_current = 1.0;
    m.no_load_voltage = 10.0;
    m.current_max = 40.0;
    m.pole_pairs = 7.0;
    m.inductance = 0.0;
    m.rotor_inertia = 1.0e-5;
    m.stator = "2306";
    m.mass = 0.032;
    m.shape = BlockShape{PartShape::cylinder, glm::dvec3(0.0145, 0.019, 0.0), glm::dmat3(0.0)};
    return m;
}

PropBlock BasicQuadParams::default_prop() {
    PropBlock p;
    p.diameter = 0.1524;
    p.pitch = 0.0889;
    p.blades = 3;
    p.inertia = 0.0;
    p.ct_static = 0.12;
    p.cq_static = 0.008;
    p.mass = 0.0064;
    return p;
}

EscBlock BasicQuadParams::default_esc() {
    EscBlock e;
    e.current_continuous = 45.0;
    e.current_burst = 55.0;
    e.current_total = 0.0;
    e.channels = 4;
    e.on_resistance = 0.003;
    e.braking = true;
    e.protocol = "DShot600";
    e.mass = 0.012;
    return e;
}

BatteryBlock BasicQuadParams::default_battery() {
    BatteryBlock b;
    b.cells_series = 6;
    b.cells_parallel = 1;
    b.cell_capacity = 1.3;
    b.cell_resistance = 0.004;
    b.cell_voltage_nominal = 3.7;
    b.cell_voltage_full = 4.2;
    b.cell_voltage_cutoff = 3.3;
    b.ocv_table = {{0.0, 3.3}, {0.2, 3.6}, {0.5, 3.75}, {0.8, 3.95}, {1.0, 4.2}};
    b.c_rating = 75.0;
    b.polarization_resistance = 0.0;
    b.polarization_capacitance = 0.0;
    b.mass = 0.24;
    // On top of the body, sunk 2 mm into it (overlap, never a gap: Physics'
    // solids rule).
    b.mount.position = glm::dvec3(0.0, 0.0305, 0.0);
    b.shape = BlockShape{PartShape::box, glm::dvec3(0.075, 0.03, 0.035), glm::dmat3(0.0)};
    return b;
}

Result<AirframeSpec> basic_quad_airframe(const BasicQuadParams& p) {
    if (!positive(p.half_span_x)) {
        return std::unexpected(invalid("half_span_x must be finite and above 0, found " + text(p.half_span_x)));
    }
    if (!positive(p.half_span_z)) {
        return std::unexpected(invalid("half_span_z must be finite and above 0, found " + text(p.half_span_z)));
    }
    if (p.mass && !std::isfinite(*p.mass)) return std::unexpected(invalid("mass target is not finite"));
    if (p.inertia) {
        for (int a = 0; a < 3; ++a) {
            if (!std::isfinite((*p.inertia)[a])) {
                return std::unexpected(invalid(std::string("inertia.") + kAxes[a] + " target is not finite"));
            }
        }
        if (!p.mass) {
            return std::unexpected(invalid("inertia: a fitted inertia needs a mass target (a massless remainder "
                                           "is no body)"));
        }
    }

    AirframeSpec s;
    s.name = p.name;
    s.param_schema_id = 1;
    s.motor = p.motor;
    s.prop = p.prop;
    s.esc = p.esc;
    s.battery = p.battery;

    if (p.kappa) {
        if (!positive(*p.kappa)) {
            return std::unexpected(invalid("kappa must be finite and above 0, found " + text(*p.kappa)));
        }
        if (!p.prop.ct_table.empty() || !p.prop.cq_table.empty()) {
            return std::unexpected(invalid("kappa sets prop.cq_static, which the compile reads only without "
                                           "tables: give the propeller no ct_table and cq_table, or no kappa"));
        }
        if (!positive(p.prop.diameter)) {
            return std::unexpected(invalid("kappa needs a propeller diameter above 0, found " +
                                           text(p.prop.diameter)));
        }
        s.prop.cq_static = *p.kappa * p.prop.ct_static / p.prop.diameter;
    }

    // THE FRAME: front-right, front-left, rear-left, rear-right.
    const double x = p.half_span_x;
    const double z = p.half_span_z;
    const std::array<glm::dvec2, 4> corners = {glm::dvec2(x, z), glm::dvec2(x, -z), glm::dvec2(-x, -z),
                                               glm::dvec2(-x, z)};
    const std::array<double, 4> spins = {1.0, -1.0, 1.0, -1.0};
    const double arm_length = std::sqrt(x * x + z * z);

    s.parts.push_back(record("body", "body", p.body_mass, glm::dvec3(0.0), glm::dquat(1.0, 0.0, 0.0, 0.0),
                             PartShape::box, p.body_size));
    for (std::size_t k = 0; k < corners.size(); ++k) {
        const glm::dvec2 c = corners[k];
        s.parts.push_back(record("arm[" + std::to_string(k) + "]", "arm", p.arm_mass_per_metre * arm_length,
                                 glm::dvec3(c.x / 2, 0.0, c.y / 2), y_onto(c.x / arm_length, c.y / arm_length),
                                 PartShape::tube, glm::dvec3(p.arm_radius, arm_length, 0.0)));
    }
    for (std::size_t k = 0; k < corners.size(); ++k) {
        RotorSpec r;
        r.hub.position = glm::dvec3(corners[k].x, p.hub_height, corners[k].y);
        r.spin_dir = spins[k];
        s.rotors.push_back(r);
    }
    s.imus.push_back(ImuSpec{});
    s.proxy_radius = arm_length + 0.5 * p.prop.diameter;

    if (!p.mass) return s;

    // THE FIT. The parts are measured by the compile itself (TD-9): its
    // composite inertia about their centre of mass, in design axes, in double.
    const Result<CompiledAirframe> parts = compile_airframe(s);
    if (!parts) return std::unexpected(parts.error());
    const double own_mass = parts_mass(s);
    const double rest_mass = *p.mass - own_mass;
    if (!(rest_mass > 0.0)) {
        return std::unexpected(invalid("mass: the fitted " + text(*p.mass) + " kg is not above the parts' own " +
                                       text(own_mass) + " kg"));
    }
    PartInertia rest = record("remainder", "remainder", rest_mass, glm::dvec3(parts->inertia.center_of_mass),
                              glm::dquat(1.0, 0.0, 0.0, 0.0), PartShape::point, glm::dvec3(0.0));
    rest.geometry.source = GeometrySource::none;
    if (p.inertia) {
        const glm::dmat3& own = parts->inertia.inertia_design;
        for (int a = 0; a < 3; ++a) {
            if ((*p.inertia)[a] < own[a][a]) {
                return std::unexpected(invalid(std::string("inertia.") + kAxes[a] + ": the fitted " +
                                               text((*p.inertia)[a]) + " kg m^2 is below the parts' own " +
                                               text(own[a][a]) + " kg m^2"));
            }
        }
        glm::dmat3 tensor = -own;
        for (int a = 0; a < 3; ++a) tensor[a][a] += (*p.inertia)[a];
        rest.shape = PartShape::tensor;
        rest.tensor = tensor;
        const std::vector<PartIssue> issues = composite_inertia_issues(std::span<const PartInertia>(&rest, 1));
        if (!issues.empty()) {
            return std::unexpected(invalid("inertia: the remainder the fit needs, diag(" + text((*p.inertia)[0]) +
                                           ", " + text((*p.inertia)[1]) + ", " + text((*p.inertia)[2]) +
                                           ") less the parts' own diag(" + text(own[0][0]) + ", " +
                                           text(own[1][1]) + ", " + text(own[2][2]) + "), is not a body: it " +
                                           issues.front().message));
        }
    }
    s.parts.push_back(std::move(rest));
    return s;
}

std::vector<BasicQuadAssumption> basic_quad_assumptions() {
    return {
        {"half_span_x", "Kat's competition-700 layout, +-0.075 m along the nose"},
        {"half_span_z", "Kat's competition-700 layout, +-0.10 m lateral"},
        {"hub_height", "the propeller plane 22 mm above the arms, a motor's height"},
        {"body_size", "a 90 x 35 x 60 mm centre stack"},
        {"body_mass", "a typical 6-inch frame's plates, flight controller and wiring"},
        {"arm_radius", "a 12 mm arm, three cells across at 4 mm"},
        {"arm_mass_per_metre", "a typical carbon arm"},
        {"motor.kv", "a 2306 1700 KV motor, typical on 6S"},
        {"motor.resistance", "typical of a 2306 motor"},
        {"motor.no_load_current", "typical of a 2306 motor"},
        {"motor.no_load_voltage", "the usual no-load test voltage"},
        {"motor.current_max", "typical of a 2306 motor"},
        {"motor.pole_pairs", "12N14P"},
        {"motor.inductance", "dropped (0)"},
        {"motor.rotor_inertia", "typical of a 2306 bell and shaft"},
        {"motor.mass", "typical of a 2306 motor"},
        {"motor.shape", "a 29 x 19 mm bell"},
        {"prop.diameter", "a 6-inch propeller"},
        {"prop.pitch", "3.5 inches"},
        {"prop.blades", "a tri-blade"},
        {"prop.inertia", "estimated from the mass (0)"},
        {"prop.ct_static", "typical of a 6-inch tri-blade, until L4a's UIUC-derived table"},
        {"prop.cq_static", "typical of a 6-inch tri-blade, until L4a's UIUC-derived table"},
        {"prop.mass", "a typical 6x3.5x3 propeller"},
        {"esc.current_continuous", "a 45 A 4-in-1 ESC"},
        {"esc.current_burst", "a 45 A 4-in-1 ESC"},
        {"esc.current_total", "no board total given (0)"},
        {"esc.channels", "one 4-in-1 board"},
        {"esc.on_resistance", "typical of a 4-in-1 ESC"},
        {"esc.braking", "active braking, as most FPV firmware runs"},
        {"esc.mass", "a typical 4-in-1 ESC"},
        {"esc.mount", "in the body, at its centre"},
        {"battery.cells_series", "a 6S pack"},
        {"battery.cells_parallel", "one cell per series step"},
        {"battery.cell_capacity", "a 1300 mAh pack"},
        {"battery.cell_resistance", "typical of a 1300 mAh cell"},
        {"battery.cell_voltage_nominal", "LiPo nominal"},
        {"battery.cell_voltage_full", "LiPo full"},
        {"battery.cell_voltage_cutoff", "a usual LiPo cutoff"},
        {"battery.ocv_table", "the chain's current LiPo shape (ruling 8: no open table found)"},
        {"battery.c_rating", "typical of a 1300 mAh FPV pack"},
        {"battery.polarization_resistance", "not modelled (0)"},
        {"battery.polarization_capacitance", "not modelled (0)"},
        {"battery.mass", "typical of a 6S 1300 mAh pack"},
        {"battery.mount", "on top of the body, sunk 2 mm into it"},
        {"battery.shape", "a 75 x 30 x 35 mm pack"},
    };
}

std::vector<BasicQuadAdvisory> basic_quad_advisories(const AirframeSpec& spec) {
    (void)spec;
    return {};
}

}  // namespace spade::vehicles
