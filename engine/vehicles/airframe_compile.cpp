#include "vehicles/airframe_compile.hpp"

#include <cmath>
#include <span>
#include <string>
#include <utility>

#include "vehicles/battery.hpp"
#include "vehicles/motor.hpp"
#include "vehicles/propeller.hpp"
#include "vehicles/propulsion_flags.hpp"

namespace spade::vehicles {
namespace {

constexpr uint32_t kTablePoints = 33;  // resampled grid for propeller and OCV tables
constexpr double kInvFourPiSq = 0.025330295910584444;  // 1 / (4 pi^2)
constexpr double kDragCoefficient = 1.0;  // best-effort bluff-body C_d for the drag estimate

[[nodiscard]] bool is_finite(double x) noexcept { return std::isfinite(x); }
[[nodiscard]] bool positive(double x) noexcept { return std::isfinite(x) && x > 0.0; }
[[nodiscard]] bool non_negative(double x) noexcept { return std::isfinite(x) && x >= 0.0; }

[[nodiscard]] bool finite_vec(const glm::dvec3& v) noexcept {
    return is_finite(v.x) && is_finite(v.y) && is_finite(v.z);
}

[[nodiscard]] bool usable_pose(const MountPose& m) noexcept {
    const glm::dquat& q = m.orientation;
    const double n = q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z;
    return finite_vec(m.position) && is_finite(n) && n > 0.0;
}

// The collector every check writes into.
struct Issues {
    std::vector<AirframeIssue> list;
    void add(std::string kind, std::size_t index, std::string field, std::string message) {
        list.push_back(AirframeIssue{Code::invalid_argument, std::move(kind), index, std::move(field),
                                     std::move(message)});
    }
};

void check_pose(Issues& out, const char* kind, std::size_t index, const char* field, const MountPose& m) {
    if (!usable_pose(m)) out.add(kind, index, field, "is not finite, or its orientation is zero");
}

void check_blocks(const AirframeSpec& s, Issues& out) {
    if (s.name.empty()) out.add("airframe", 0, "name", "is empty");
    if (s.version < 1) out.add("airframe", 0, "version", "must be at least 1");
    if (!non_negative(s.proxy_radius)) out.add("airframe", 0, "proxy_radius", "must be finite and >= 0");
    if (!positive(s.air_density)) out.add("airframe", 0, "air_density", "must be positive");
    if (!positive(s.gravity)) out.add("airframe", 0, "gravity", "must be positive");
    if (!(is_finite(s.state_of_charge) && s.state_of_charge >= 0.0 && s.state_of_charge <= 1.0)) {
        out.add("airframe", 0, "state_of_charge", "must lie in [0, 1]");
    }

    const MotorBlock& m = s.motor;
    if (!positive(m.kv)) out.add("motor", 0, "kv", "must be positive (rpm/V)");
    if (!positive(m.resistance)) out.add("motor", 0, "resistance", "must be positive");
    if (!non_negative(m.no_load_current)) out.add("motor", 0, "no_load_current", "must be >= 0");
    if (!positive(m.current_max)) out.add("motor", 0, "current_max", "must be positive");
    if (!non_negative(m.pole_pairs)) out.add("motor", 0, "pole_pairs", "must be >= 0");
    if (!non_negative(m.inductance)) out.add("motor", 0, "inductance", "must be >= 0");
    if (!non_negative(m.rotor_inertia)) out.add("motor", 0, "rotor_inertia", "must be >= 0");
    if (!non_negative(m.mass)) out.add("motor", 0, "mass", "must be >= 0");

    const PropBlock& p = s.prop;
    if (!positive(p.diameter)) out.add("prop", 0, "diameter", "must be positive");
    if (!non_negative(p.inertia)) out.add("prop", 0, "inertia", "must be >= 0");
    if (!non_negative(p.mass)) out.add("prop", 0, "mass", "must be >= 0");
    if (p.ct_table.empty() != p.cq_table.empty()) {
        out.add("prop", 0, "ct_table", "C_T and C_Q tables must both be given, or neither");
    } else if (p.ct_table.empty()) {
        if (!positive(p.ct_static)) out.add("prop", 0, "ct_static", "must be positive when no table is given");
        if (!non_negative(p.cq_static)) out.add("prop", 0, "cq_static", "must be >= 0 when no table is given");
    } else if (const Result<PropellerTable> t = propeller_resample_table(p.ct_table, p.cq_table, kTablePoints); !t) {
        out.add("prop", 0, "ct_table", t.error().context);
    }

    const EscBlock& e = s.esc;
    if (!non_negative(e.current_burst)) out.add("esc", 0, "current_burst", "must be >= 0");
    if (!non_negative(e.current_total)) out.add("esc", 0, "current_total", "must be >= 0");
    if (!non_negative(e.on_resistance)) out.add("esc", 0, "on_resistance", "must be >= 0");
    if (!non_negative(e.mass)) out.add("esc", 0, "mass", "must be >= 0");
    check_pose(out, "esc", 0, "mount", e.mount);

    const BatteryBlock& b = s.battery;
    if (b.cells_series < 1) out.add("battery", 0, "cells_series", "must be at least 1");
    if (b.cells_parallel < 1) out.add("battery", 0, "cells_parallel", "must be at least 1");
    if (!positive(b.cell_capacity)) out.add("battery", 0, "cell_capacity", "must be positive");
    if (!non_negative(b.cell_resistance)) out.add("battery", 0, "cell_resistance", "must be >= 0");
    if (!non_negative(b.cell_voltage_cutoff)) out.add("battery", 0, "cell_voltage_cutoff", "must be >= 0");
    if (!non_negative(b.c_rating)) out.add("battery", 0, "c_rating", "must be >= 0");
    if (!non_negative(b.polarization_resistance)) {
        out.add("battery", 0, "polarization_resistance", "must be >= 0");
    }
    if (!non_negative(b.mass)) out.add("battery", 0, "mass", "must be >= 0");
    if (const Result<std::vector<float>> t = battery_resample_ocv(b.ocv_table, kTablePoints); !t) {
        out.add("battery", 0, "ocv_table", t.error().context);
    }
    check_pose(out, "battery", 0, "mount", b.mount);

    if (s.rotors.empty()) out.add("rotors", 0, "", "an airframe needs at least one rotor");
    for (std::size_t k = 0; k < s.rotors.size(); ++k) {
        check_pose(out, "rotors", k, "hub", s.rotors[k].hub);
        const double spin = s.rotors[k].spin_dir;
        if (!(spin == 1.0 || spin == -1.0 || spin == 0.0)) out.add("rotors", k, "spin_dir", "must be +1, -1 or 0");
    }
    for (std::size_t k = 0; k < s.drag.size(); ++k) {
        const DragSpec& d = s.drag[k];
        if (d.mode != physics::drag_mode::quadratic && d.mode != physics::drag_mode::componentwise) {
            out.add("drag", k, "mode", "is neither quadratic nor componentwise");
        }
        if (!non_negative(d.area)) out.add("drag", k, "area", "must be >= 0");
        if (!finite_vec(d.coeffs) || d.coeffs.x < 0.0 || d.coeffs.y < 0.0 || d.coeffs.z < 0.0) {
            out.add("drag", k, "coeffs", "must be finite and >= 0");
        }
        check_pose(out, "drag", k, "mount", d.mount);
    }
    for (std::size_t k = 0; k < s.imus.size(); ++k) {
        const ImuSpec& i = s.imus[k];
        if (i.rate_divider < 1) out.add("imus", k, "rate_divider", "must be at least 1");
        if (!non_negative(i.sigma_a) || !non_negative(i.sigma_g) || !non_negative(i.sigma_ba) ||
            !non_negative(i.sigma_bg)) {
            out.add("imus", k, "sigma", "every sigma must be finite and >= 0");
        }
        check_pose(out, "imus", k, "mount", i.mount);
    }
}

// The propeller's polar inertia: given, or a thin rod of its mass across its
// diameter (best-effort).
[[nodiscard]] double prop_inertia(const PropBlock& p) noexcept {
    return p.inertia > 0.0 ? p.inertia : p.mass * p.diameter * p.diameter / 12.0;
}

[[nodiscard]] PartInertia block_part(double mass, const MountPose& pose, const std::optional<BlockShape>& shape,
                                     const BlockShape& fallback) {
    const BlockShape& s = shape ? *shape : fallback;
    PartInertia p;
    p.mass = mass;
    p.position = pose.position;
    p.orientation = pose.orientation;
    p.shape = s.shape;
    p.size = s.size;
    p.tensor = s.tensor;
    return p;
}

// Every mass: the spec's parts, then a motor and a propeller at each hub, then
// the ESC and the battery. `origin` records, for each, its kind and index, so
// a part issue can name what the caller wrote. A block of zero mass adds no
// part.
struct PartList {
    std::vector<PartInertia> parts;
    std::vector<std::pair<std::string, std::size_t>> origin;
    std::vector<bool> drag_source;  // false for propellers: their aero is the rotor's
};

[[nodiscard]] PartList gather_parts(const AirframeSpec& s) {
    PartList out;
    const auto push = [&](PartInertia p, std::string kind, std::size_t index, bool drag) {
        out.parts.push_back(std::move(p));
        out.origin.emplace_back(std::move(kind), index);
        out.drag_source.push_back(drag);
    };
    for (std::size_t k = 0; k < s.parts.size(); ++k) push(s.parts[k], "parts", k, true);
    const BlockShape point{};
    // A thin disc across the diameter, or a point when the diameter is
    // unusable, which is then reported once, as prop.diameter.
    BlockShape disc;
    if (positive(s.prop.diameter)) {
        disc.shape = PartShape::cylinder;
        disc.size = glm::dvec3(0.5 * s.prop.diameter, 0.0, 0.0);
    }
    for (std::size_t k = 0; k < s.rotors.size(); ++k) {
        if (s.motor.mass > 0.0) push(block_part(s.motor.mass, s.rotors[k].hub, s.motor.shape, point), "motor", k, true);
        if (s.prop.mass > 0.0) push(block_part(s.prop.mass, s.rotors[k].hub, s.prop.shape, disc), "prop", k, false);
    }
    if (s.esc.mass > 0.0) push(block_part(s.esc.mass, s.esc.mount, s.esc.shape, point), "esc", 0, true);
    if (s.battery.mass > 0.0) {
        push(block_part(s.battery.mass, s.battery.mount, s.battery.shape, point), "battery", 0, true);
    }
    return out;
}

// The local projected areas of one shape, onto the planes normal to its local
// x, y and z: what a flow along that axis sees.
[[nodiscard]] glm::dvec3 local_areas(const PartInertia& p) noexcept {
    constexpr double kPi = 3.14159265358979323846;
    switch (p.shape) {
        case PartShape::sphere: {
            const double a = kPi * p.size.x * p.size.x;
            return glm::dvec3(a, a, a);
        }
        case PartShape::box:
            return glm::dvec3(p.size.y * p.size.z, p.size.x * p.size.z, p.size.x * p.size.y);
        case PartShape::cylinder:
        case PartShape::tube: {
            const double side = 2.0 * p.size.x * p.size.y;
            return glm::dvec3(side, kPi * p.size.x * p.size.x, side);
        }
        case PartShape::point:
        case PartShape::tensor:
        default:
            return glm::dvec3(0.0);  // no shape to see
    }
}

[[nodiscard]] glm::vec3 to_float(const glm::dvec3& v) noexcept {
    return glm::vec3(static_cast<float>(v.x), static_cast<float>(v.y), static_cast<float>(v.z));
}

[[nodiscard]] glm::quat to_float(const glm::dquat& q) noexcept {
    return glm::quat(static_cast<float>(q.w), static_cast<float>(q.x), static_cast<float>(q.y),
                     static_cast<float>(q.z));
}

struct Built {
    Issues issues;
    std::optional<CompiledAirframe> result;
};

[[nodiscard]] Result<PropulsionChain> chain_from(const AirframeSpec& s) {
    const MotorBlock& m = s.motor;
    const PropBlock& p = s.prop;
    const EscBlock& e = s.esc;
    const BatteryBlock& b = s.battery;
    PropulsionChain c;
    c.kv = motor_kv_si(m.kv);
    c.resistance = m.resistance + e.on_resistance;
    c.no_load_current = m.no_load_current;
    c.current_max = e.current_burst > 0.0 && e.current_burst < m.current_max ? e.current_burst : m.current_max;
    c.current_min = e.braking ? -c.current_max : 0.0;
    c.esc_current_total_max = e.current_total;
    c.pole_pairs = m.pole_pairs;
    c.inductance = m.inductance;
    c.diameter = p.diameter;
    Result<PropellerTable> table =
        p.ct_table.empty()
            ? propeller_resample_table(std::vector<std::array<double, 2>>{{0.0, p.ct_static}, {1.0, p.ct_static}},
                                       std::vector<std::array<double, 2>>{{0.0, p.cq_static}, {1.0, p.cq_static}},
                                       kTablePoints)
            : propeller_resample_table(p.ct_table, p.cq_table, kTablePoints);
    if (!table) return std::unexpected(table.error());
    c.table = std::move(*table);
    c.cells_series = b.cells_series;
    c.cells_parallel = b.cells_parallel;
    c.cell_resistance = b.cell_resistance;
    c.cell_cutoff_voltage = b.cell_voltage_cutoff;
    c.pack_polarization_resistance = b.polarization_resistance;
    c.pack_current_max = b.c_rating * b.cell_capacity * static_cast<double>(b.cells_parallel);
    Result<std::vector<float>> ocv = battery_resample_ocv(b.ocv_table, kTablePoints);
    if (!ocv) return std::unexpected(ocv.error());
    c.ocv_table = std::move(*ocv);
    c.motors_on_bus = static_cast<uint32_t>(s.rotors.size());
    return c;
}

[[nodiscard]] Built build(const AirframeSpec& s) {
    Built out;
    check_blocks(s, out.issues);

    const PartList parts = gather_parts(s);
    for (const PartIssue& pi : composite_inertia_issues(parts.parts)) {
        const auto& [kind, index] = parts.origin[pi.index];
        out.issues.add(kind, index, pi.field, pi.message);
    }
    if (!out.issues.list.empty()) return out;

    // --- the body -------------------------------------------------------------
    const Result<CompositeInertia> inertia = composite_inertia(parts.parts);
    if (!inertia) {
        out.issues.add("airframe", 0, "parts", inertia.error().context);
        return out;
    }
    const glm::quat q_bd_f = canonical_design_rotation(inertia->design_to_body);
    const glm::dquat q_bd(q_bd_f.w, q_bd_f.x, q_bd_f.y, q_bd_f.z);
    const glm::dmat3 r_bd = glm::mat3_cast(q_bd);
    const glm::dvec3 c(inertia->center_of_mass);
    const auto to_body_pos = [&](const glm::dvec3& p_d) { return to_float(r_bd * (p_d - c)); };
    const auto to_body_rot = [&](const glm::dquat& q_d) { return to_float(q_bd * glm::normalize(q_d)); };

    // --- the propulsion chain and the fit ------------------------------------
    Result<PropulsionChain> chain = chain_from(s);
    if (!chain) {
        out.issues.add("airframe", 0, "propulsion", chain.error().context);
        return out;
    }
    AirframeFit fit;
    const double weight = static_cast<double>(inertia->mass) * s.gravity;
    fit.hover_thrust = weight / static_cast<double>(s.rotors.size());
    if (s.rotors.size() <= kMaxModelRotors) {
        const SteadyPoint hover = steady_state_for_thrust(*chain, fit.hover_thrust, s.air_density, 0.0, s.state_of_charge);
        fit.solver_flags = hover.flags;
        if ((hover.flags & (propulsion_flags::unreachable | propulsion_flags::no_bracket)) != 0u) {
            out.issues.add("airframe", 0, "propulsion",
                           "the chain cannot hover this airframe: it needs " + std::to_string(fit.hover_thrust) +
                               " N per rotor and gives " + std::to_string(hover.thrust) + " N at full duty");
            return out;
        }
        fit.hover_duty = hover.duty;
        fit.hover_omega = hover.omega;
    }
    uint32_t table_flags = 0;
    const double ct0 = propeller_coefficient(chain->table.ct, static_cast<double>(chain->table.j_min),
                                             static_cast<double>(chain->table.j_max), 0.0, table_flags);
    const double cq0 = propeller_coefficient(chain->table.cq, static_cast<double>(chain->table.j_min),
                                             static_cast<double>(chain->table.j_max), 0.0, table_flags);
    const double d2 = s.prop.diameter * s.prop.diameter;
    fit.thrust_coeff = ct0 * s.air_density * d2 * d2 * kInvFourPiSq;
    fit.torque_coeff = cq0 * s.air_density * d2 * d2 * s.prop.diameter * kInvFourPiSq;
    fit.rotor_inertia = s.motor.rotor_inertia + prop_inertia(s.prop);
    fit.rotor_inertia_source =
        (s.prop.inertia > 0.0 || s.prop.mass == 0.0) ? Provenance::given : Provenance::estimated;
    fit.tau = steady_state_time_constant(*chain, fit.hover_duty, s.air_density, 0.0, s.state_of_charge,
                                         fit.rotor_inertia);
    if (s.rotors.size() <= kMaxModelRotors && !(fit.tau > 0.0)) {
        out.issues.add("airframe", 0, "propulsion",
                       "the chain's time constant at hover cannot be fitted (is the rotor inertia zero?)");
        return out;
    }

    // --- the model ------------------------------------------------------------
    ModelType model;
    model.name = s.name;
    model.param_schema_id = s.param_schema_id;
    model.visual_ref = s.visual_ref;
    model.version = s.version;
    model.design_to_principal = q_bd_f;
    model.com_offset = inertia->center_of_mass;
    model.body.mass = inertia->mass;
    model.body.inertia_diag = inertia->principal_moments;
    model.proxy_radius = static_cast<float>(s.proxy_radius);

    for (const RotorSpec& r : s.rotors) {
        RotorDesc d;
        d.local_pos = to_body_pos(r.hub.position);
        d.local_orient = to_body_rot(r.hub.orientation);
        d.spin_dir = static_cast<float>(r.spin_dir);
        d.tau = static_cast<float>(fit.tau);
        d.radius = static_cast<float>(0.5 * s.prop.diameter);
        d.thrust_coeff = static_cast<float>(fit.thrust_coeff);
        d.torque_coeff = static_cast<float>(fit.torque_coeff);
        model.rotors.push_back(d);
    }

    if (!s.drag.empty()) {
        fit.drag = Provenance::given;
        for (const DragSpec& g : s.drag) {
            DragBodyDesc d;
            d.mode = g.mode;
            d.area = static_cast<float>(g.area);
            d.coeffs = to_float(g.coeffs);
            d.local_pos = to_body_pos(g.mount.position);
            d.local_orient = to_body_rot(g.mount.orientation);
            model.drag_bodies.push_back(d);
        }
    } else {
        // Best-effort: half rho C_d times each body axis's projected area, summed
        // over the parts. A rotated part projects as sum_j |M_ij| A_j: exact
        // for a box at any angle and for an axis-aligned part, an overestimate
        // for a tilted cylinder; parts that shield each other are counted
        // twice. Propellers are left out: their aerodynamics are the rotor's.
        fit.drag = Provenance::estimated;
        glm::dvec3 area(0.0);
        for (std::size_t k = 0; k < parts.parts.size(); ++k) {
            if (!parts.drag_source[k]) continue;
            const PartInertia& p = parts.parts[k];
            const glm::dmat3 to_body = r_bd * glm::mat3_cast(glm::normalize(p.orientation));
            const glm::dvec3 local = local_areas(p);
            for (int i = 0; i < 3; ++i) {
                // glm is column-major: to_body[j][i] is row i, column j.
                area[i] += std::fabs(to_body[0][i]) * local.x + std::fabs(to_body[1][i]) * local.y +
                           std::fabs(to_body[2][i]) * local.z;
            }
        }
        if (area.x > 0.0 || area.y > 0.0 || area.z > 0.0) {
            DragBodyDesc d;
            d.mode = physics::drag_mode::componentwise;
            d.coeffs = to_float(0.5 * s.air_density * kDragCoefficient * area);
            model.drag_bodies.push_back(d);
        }
    }

    for (const ImuSpec& i : s.imus) {
        ImuMountDesc d;
        d.mount_pos = to_body_pos(i.mount.position);
        d.mount_orient = to_body_rot(i.mount.orientation);
        d.rate_divider = i.rate_divider;
        d.sigma_a = static_cast<float>(i.sigma_a);
        d.sigma_g = static_cast<float>(i.sigma_g);
        d.sigma_ba = static_cast<float>(i.sigma_ba);
        d.sigma_bg = static_cast<float>(i.sigma_bg);
        model.imu_mounts.push_back(d);
    }

    if (const Result<void> v = model.validate(); !v) {
        out.issues.add("model", 0, "", v.error().context);
        return out;
    }
    out.result = CompiledAirframe{std::move(model), *inertia, fit};
    return out;
}

}  // namespace

Result<PropulsionChain> airframe_propulsion_chain(const AirframeSpec& spec) {
    Issues issues;
    check_blocks(spec, issues);
    std::string message;
    for (const AirframeIssue& i : issues.list) {
        if (i.kind != "motor" && i.kind != "prop" && i.kind != "esc" && i.kind != "battery" && i.kind != "rotors") {
            continue;
        }
        message += " " + i.kind + "[" + std::to_string(i.index) + "]." + i.field + ": " + i.message + ";";
    }
    if (!message.empty()) return std::unexpected(Error{Code::invalid_argument, "airframe_propulsion_chain:" + message});
    return chain_from(spec);
}

std::vector<AirframeIssue> check_airframe(const AirframeSpec& spec) { return build(spec).issues.list; }

Result<CompiledAirframe> compile_airframe(const AirframeSpec& spec) {
    Built b = build(spec);
    if (!b.issues.list.empty() || !b.result) {
        std::string message = "compile_airframe:";
        for (const AirframeIssue& i : b.issues.list) {
            message += " " + i.kind + "[" + std::to_string(i.index) + "]";
            if (!i.field.empty()) message += "." + i.field;
            message += ": " + i.message + ";";
        }
        return std::unexpected(Error{Code::invalid_argument, message});
    }
    return std::move(*b.result);
}

}  // namespace spade::vehicles
