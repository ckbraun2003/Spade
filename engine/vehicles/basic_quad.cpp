#include "vehicles/basic_quad.hpp"

#include <array>
#include <charconv>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include "world/mesh.hpp"         // mass_properties
#include "world/mesh_shapes.hpp"  // THE LOOK's generators

namespace spade::vehicles {
namespace {

[[nodiscard]] Error invalid(std::string context) { return Error{Code::invalid_argument, std::move(context)}; }

[[nodiscard]] bool positive(double x) noexcept { return std::isfinite(x) && x > 0.0; }

// A length in a message: millimetres, to a tenth.
[[nodiscard]] std::string mm(double metres) {
    char buf[32];
    const std::to_chars_result r =
        std::to_chars(buf, buf + sizeof(buf), metres * 1000.0, std::chars_format::fixed, 1);
    return std::string(buf, r.ptr);
}

// A number in a message: the shortest form that reads back to it.
[[nodiscard]] std::string text(double x) {
    char buf[32];
    const std::to_chars_result r = std::to_chars(buf, buf + sizeof(buf), x);
    return std::string(buf, r.ptr);
}

constexpr std::array<const char*, 3> kAxes = {"x", "y", "z"};

// The tri-blade stand-in's line (L4a's grade note; the lead, 2026-10-22), for
// each of default_prop()'s coefficients.
constexpr std::string_view kTriBlade =
    "tri-blade tables: mit_5x4 (three-blade, 5 in), scaled to 6 in per Physics' rule";

// A fit target that is not finite, by name (THE FIT): the same refusal from
// basic_quad_airframe() and fit_remainder().
[[nodiscard]] std::optional<Error> non_finite_target(const std::optional<double>& mass,
                                                     const std::optional<glm::dvec3>& inertia) {
    if (mass && !std::isfinite(*mass)) return invalid("mass target is not finite");
    if (inertia) {
        for (int a = 0; a < 3; ++a) {
            if (!std::isfinite((*inertia)[a])) {
                return invalid(std::string("inertia.") + kAxes[a] + " target is not finite");
            }
        }
    }
    return std::nullopt;
}

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
    m.rotor_inertia = 2.7e-6;
    m.stator = "2306";
    m.mass = 0.032;
    m.shape = BlockShape{PartShape::cylinder, glm::dvec3(0.0145, 0.019, 0.0), glm::dmat3(0.0)};
    return m;
}

PropBlock BasicQuadParams::default_prop() {
    // The library's triblade_6inch_scaled (L4a). The static point and tables
    // are copied from tests/library_props.gen.hpp as its shortest round-trip
    // literals, so they are its numbers bit for bit (tests/test_basic_quad.cpp
    // pins them); the static point is each table's J = 0 row.
    PropBlock p;
    p.diameter = 0.1524;
    p.pitch = 0.0889;
    p.blades = 3;
    p.ct_static = 0.18147428896905915;
    p.cq_static = 0.018923916207378596;
    p.ct_table = {{0.0, 0.18147428896905915}, {0.05, 0.18201184989635935}, {0.1, 0.18182731382170575},
                  {0.15, 0.17596891268979323}, {0.2, 0.16972798370663342}, {0.25, 0.1640415085147342},
                  {0.3, 0.1535107600936337}, {0.35, 0.14236731200453}, {0.4, 0.12904313886769417},
                  {0.45, 0.11588920053226577}, {0.5, 0.10039877116740561}, {0.55, 0.08448299501009322},
                  {0.6, 0.068280012546626}, {0.65, 0.052129378582626514}, {0.7, 0.03411875906336438},
                  {0.75, 0.0140625953772972}, {0.8, -0.005762060217429924}, {0.85, -0.025496199246994394},
                  {0.9, -0.04710815759519209}};
    p.cq_table = {{0.0, 0.018923916207378596}, {0.05, 0.01895885170425953}, {0.1, 0.018930795123769308},
                  {0.15, 0.018403663821918506}, {0.2, 0.017847137026031582}, {0.25, 0.017446704115154192},
                  {0.3, 0.016640316889998744}, {0.35, 0.015808383667437115}, {0.4, 0.014774322066910418},
                  {0.45, 0.013791407670878986}, {0.5, 0.012623069679092559}, {0.55, 0.011468158165227036},
                  {0.6, 0.010305967325430116}, {0.65, 0.009179534056486645}, {0.7, 0.007910990775304116},
                  {0.75, 0.006412361581935726}, {0.8, 0.004945488492696708}, {0.85, 0.003424586492792064},
                  {0.9, 0.0020542645321587113}};
    p.mass = 0.0064;
    // The compile's own estimate, prop_inertia()'s m D^2 / 12, written out as
    // the part file writes it, so it is that double exactly (the lead,
    // 2026-10-22: a 0 read as no inertia).
    p.inertia = p.mass * p.diameter * p.diameter / 12.0;
    return p;
}

EscBlock BasicQuadParams::default_esc() {
    // The library's basic_4in1_45a.
    EscBlock e;
    e.current_continuous = 45.0;
    e.current_burst = 55.0;
    e.current_total = 0.0;
    e.channels = 4;
    e.on_resistance = 0.003;
    e.braking = true;
    e.protocol = "DShot600";
    e.mass = 0.012;
    // In the body, at its centre, inside its 90 x 35 x 60 mm box: a 30.5 mm-
    // mount board, at least 8 mm thick (the solids rule).
    e.shape = BlockShape{PartShape::box, glm::dvec3(0.036, 0.008, 0.036), glm::dmat3(0.0)};
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
    if (const std::optional<Error> e = non_finite_target(p.mass, p.inertia)) return std::unexpected(*e);
    if (p.inertia && !p.mass) {
        return std::unexpected(invalid("inertia: a fitted inertia needs a mass target (a massless remainder "
                                       "is no body)"));
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
        if (!positive(p.prop.diameter)) {
            return std::unexpected(invalid("kappa needs a propeller diameter above 0, found " +
                                           text(p.prop.diameter)));
        }
        if (p.prop.ct_table.empty() && !p.prop.cq_table.empty()) {
            return std::unexpected(invalid("kappa derives prop.cq_table from the ct_table, and this propeller has a "
                                           "cq_table with no ct_table to scale"));
        }
        // C_Q = kappa C_T / D, exact (T = C_T rho n^2 D^4, Q = C_Q rho n^2 D^5):
        // the static value, and the torque table at the thrust table's J points.
        s.prop.cq_static = *p.kappa * p.prop.ct_static / p.prop.diameter;
        s.prop.cq_table.clear();
        for (const std::array<double, 2>& row : p.prop.ct_table) {
            s.prop.cq_table.push_back({row[0], *p.kappa * row[1] / p.prop.diameter});
        }
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
    if (p.look) {
        if (const Result<void> worn = wear_look(s, *p.look); !worn) return std::unexpected(worn.error());
    }

    if (!p.mass) return s;

    // THE FIT, by the one rule every spec's remainder uses.
    Result<PartInertia> rest = fit_remainder(s, *p.mass, p.inertia);
    if (!rest) return std::unexpected(rest.error());
    s.parts.push_back(std::move(*rest));
    return s;
}

// THE LOOK (basic_quad.hpp), on the records themselves: the body record, the
// four arm records and the motors as generated meshes, each mass kept. A body
// or arm record becomes a mesh record at its own pose, of density mass / V;
// the motors take the bell as AirframeSpec::motor_mesh, of the motor's mass.
// Everything is checked and generated before the spec is touched.
Result<void> wear_look(AirframeSpec& s, const BasicQuadLook& look) {
    const auto layout = [](std::size_t k, const char* why) {
        return std::unexpected(invalid("look: parts[" + std::to_string(k) + "] " + why +
                                       ": the look wears basic_quad_airframe()'s layout, the body box, then four "
                                       "arm tubes of one size, and nothing after them"));
    };
    if (s.parts.size() > 5) return layout(5, "is one record too many");
    if (s.parts.size() < 5) return layout(s.parts.size(), "is missing");
    if (s.parts[0].shape != PartShape::box) return layout(0, "is not a box");
    for (std::size_t k = 1; k < 5; ++k) {
        if (s.parts[k].shape != PartShape::tube) return layout(k, "is not a tube");
        if (s.parts[k].size != s.parts[1].size) return layout(k, "is not parts[1]'s size");
    }
    if (look.segments < 1 || look.segments > shapes::kMaxBoxSegments) {
        return std::unexpected(invalid("look.segments must be in [1, " + std::to_string(shapes::kMaxBoxSegments) +
                                       "], found " + std::to_string(look.segments)));
    }
    const uint32_t around = 8u * look.segments;  // a full circle at the rounded box's spacing
    const auto refused = [](const char* what, const Error& e) {
        return std::unexpected(Error{e.code, std::string("look: ") + what + ": " + e.context});
    };
    const Result<shapes::GeneratedMesh> body =
        shapes::rounded_box(s.parts[0].size, look.body_edge_radius, look.segments);
    if (!body) return refused("the body", body.error());
    const Result<shapes::GeneratedMesh> arm = shapes::capped_tube(s.parts[1].size.x, s.parts[1].size.y, around);
    if (!arm) return refused("the arms", arm.error());
    if (!s.motor.shape || s.motor.shape->shape != PartShape::cylinder) {
        return std::unexpected(invalid("look: a bell is sized from motor.shape, a cylinder (radius, length), and "
                                       "this motor's shape is not one"));
    }
    const Result<shapes::GeneratedMesh> bell =
        shapes::motor_bell(s.motor.shape->size.x, s.motor.shape->size.y, look.bell_chamfer, around);
    if (!bell) return refused("the bells", bell.error());

    const auto wear_mesh = [](PartInertia& r, const shapes::GeneratedMesh& g) {
        r.geometry.source = GeometrySource::mesh;
        r.geometry.mesh = g.mesh;
        r.geometry.density = r.mass / g.mesh->mass_properties().volume;
        r.geometry.min_feature = g.min_feature;
        r.mass = 0.0;
        r.shape = PartShape::point;
        r.size = glm::dvec3(0.0);
    };
    wear_mesh(s.parts[0], *body);
    for (std::size_t k = 1; k < 5; ++k) wear_mesh(s.parts[k], *arm);
    s.motor_mesh = MotorMeshSpec{.mesh = bell->mesh, .min_feature = bell->min_feature};
    return {};
}

Result<PartInertia> fit_remainder(const AirframeSpec& spec, double mass, const std::optional<glm::dvec3>& inertia) {
    if (const std::optional<Error> e = non_finite_target(mass, inertia)) return std::unexpected(*e);

    // The parts are measured by the compile's own composite (TD-9), without
    // the fit or the drag rule: their mass in double, as the composite sums
    // it, and their inertia about their centre of mass, in design axes.
    const Result<AirframeComposite> parts = airframe_composite(spec);
    if (!parts) return std::unexpected(parts.error());
    const double own_mass = parts->mass;
    const double rest_mass = mass - own_mass;
    if (!(rest_mass > 0.0)) {
        return std::unexpected(invalid("mass: the fitted " + text(mass) + " kg is not above the parts' own " +
                                       text(own_mass) + " kg"));
    }
    PartInertia rest = record("remainder", "remainder", rest_mass, glm::dvec3(parts->inertia.center_of_mass),
                              glm::dquat(1.0, 0.0, 0.0, 0.0), PartShape::point, glm::dvec3(0.0));
    rest.geometry.source = GeometrySource::none;
    if (inertia) {
        const glm::dmat3& own = parts->inertia.inertia_design;
        for (int a = 0; a < 3; ++a) {
            if ((*inertia)[a] < own[a][a]) {
                return std::unexpected(invalid(std::string("inertia.") + kAxes[a] + ": the fitted " +
                                               text((*inertia)[a]) + " kg m^2 is below the parts' own " +
                                               text(own[a][a]) + " kg m^2"));
            }
        }
        glm::dmat3 tensor = -own;
        for (int a = 0; a < 3; ++a) tensor[a][a] += (*inertia)[a];
        rest.shape = PartShape::tensor;
        rest.tensor = tensor;
        const std::vector<PartIssue> issues = composite_inertia_issues(std::span<const PartInertia>(&rest, 1));
        if (!issues.empty()) {
            return std::unexpected(invalid("inertia: the remainder the fit needs, diag(" + text((*inertia)[0]) +
                                           ", " + text((*inertia)[1]) + ", " + text((*inertia)[2]) +
                                           ") less the parts' own diag(" + text(own[0][0]) + ", " +
                                           text(own[1][1]) + ", " + text(own[2][2]) + "), is not a body: it " +
                                           issues.front().message));
        }
    }
    return rest;
}

std::vector<BasicQuadAssumption> basic_quad_assumptions() {
    return {
        {"half_span_x", "a generic 6-inch layout: same-side hubs 160 mm apart, 7.6 mm clear of the discs"},
        {"half_span_z", "+-0.10 m lateral, as Kat's competition-700"},
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
        {"motor.inductance", "not modelled (Physics, 2026-10-22)"},
        {"motor.rotor_inertia", "the same bell estimate as the 2207s (Physics, 2026-10-22)"},
        {"motor.mass", "typical of a 2306 motor"},
        {"motor.shape", "a 29 x 19 mm bell"},
        {"prop.diameter", "a 6-inch propeller"},
        {"prop.pitch", "3.5 inches"},
        {"prop.blades", "a tri-blade"},
        {"prop.inertia", "m D^2/12, the engine's own estimate (prop_inertia), written out"},
        {"prop.ct_static", kTriBlade},
        {"prop.cq_static", kTriBlade},
        {"prop.ct_table", kTriBlade},
        {"prop.cq_table", kTriBlade},
        {"prop.mass", "a typical 6x3.5x3 propeller"},
        {"esc.current_continuous", "a 45 A 4-in-1 ESC"},
        {"esc.current_burst", "a 45 A 4-in-1 ESC"},
        {"esc.current_total", "no board total given (0)"},
        {"esc.channels", "one 4-in-1 board"},
        {"esc.on_resistance", "typical of a 4-in-1 ESC"},
        {"esc.braking", "active braking, as most FPV firmware runs"},
        {"esc.mass", "a typical 4-in-1 ESC"},
        {"esc.mount", "in the body, at its centre"},
        {"esc.shape", "a 36 x 8 x 36 mm board on a 30.5 mm mount, at least 8 mm thick (solids rule)"},
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
        {"rotors", "the shared wake is not modelled: a rotor's thrust does not respond to its neighbour's "
                   "induced inflow (Physics, 2026-10-21)"},
    };
}

std::vector<BasicQuadAdvisory> basic_quad_advisories(const AirframeSpec& spec) {
    std::vector<BasicQuadAdvisory> out;
    const double diameter = spec.prop.diameter;
    for (std::size_t i = 0; i < spec.rotors.size(); ++i) {
        // Rotor i's spin axis: its hub's +Y (THE FRAME), normalized with sqrt
        // only. A zero orientation, which the compile refuses, gives none.
        const glm::dvec3 turned = spec.rotors[i].hub.orientation * glm::dvec3(0.0, 1.0, 0.0);
        const double length = std::sqrt(glm::dot(turned, turned));
        if (!(length > 0.0)) continue;
        const glm::dvec3 axis = turned / length;
        for (std::size_t j = i + 1; j < spec.rotors.size(); ++j) {
            // Measured in rotor i's disc plane: the spin axis projected out.
            const glm::dvec3 gap = spec.rotors[j].hub.position - spec.rotors[i].hub.position;
            const double axial = glm::dot(gap, axis);
            const glm::dvec3 planar = gap - axial * axis;
            const double apart = std::sqrt(glm::dot(planar, planar));
            if (!(apart < diameter)) continue;
            const std::string a = "rotors[" + std::to_string(i) + "]";
            const std::string b = "rotors[" + std::to_string(j) + "]";
            std::string message = a + " and " + b + ": their discs overlap by " + mm(diameter - apart) +
                                  " mm (hubs " + mm(apart) + " mm apart in the disc plane";
            if (axial != 0.0) message += ", " + mm(axial < 0.0 ? -axial : axial) + " mm apart along the spin axis";
            message += ", under the " + mm(diameter) +
                       " mm diameter). Each disc keeps its own thrust and torque; what the model leaves out is "
                       "the shared wake: a rotor's thrust does not respond to its neighbour's induced inflow.";
            out.push_back({a + ", " + b, std::move(message)});
        }
    }
    return out;
}

}  // namespace spade::vehicles
