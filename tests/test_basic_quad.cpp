// The basic parametric quad (Task L3 of docs/design/core/plans/2026-10-21-
// default-drone-library-plan.md; vehicles/basic_quad.hpp): an X-layout quad's
// AirframeSpec from a handful of numbers, with an optional fitted mass,
// inertia and kappa.

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <iterator>
#include <initializer_list>
#include <string>
#include <string_view>
#include <vector>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include "core/error.hpp"
#include "vehicles/airframe_compile.hpp"
#include "vehicles/basic_quad.hpp"

namespace {

using spade::Code;
using spade::vehicles::AirframeSpec;
using spade::vehicles::BasicQuadParams;
using spade::vehicles::CompiledAirframe;
using spade::vehicles::GeometrySource;
using spade::vehicles::PartShape;

[[nodiscard]] uint32_t bits(float v) { return std::bit_cast<uint32_t>(v); }
[[nodiscard]] uint64_t bits(double v) { return std::bit_cast<uint64_t>(v); }

[[nodiscard]] bool contains(std::string_view haystack, std::string_view needle) {
    return haystack.find(needle) != std::string_view::npos;
}

template <typename T>
void expect_refused(const spade::Result<T>& r, std::initializer_list<std::string_view> needles) {
    ASSERT_FALSE(r.has_value());
    EXPECT_EQ(r.error().code, Code::invalid_argument) << r.error().context;
    for (const std::string_view needle : needles) {
        EXPECT_TRUE(contains(r.error().context, needle)) << "missing '" << needle << "' in: " << r.error().context;
    }
}

// The competition drone's fit (Kat's competition-700.airframe.json, read
// 2026-10-21): 0.700 kg, and its (Ixx, Iyy, Izz) of (0.0025, 0.0021, 0.0043)
// in Kat's FLU body frame, which is (x roll, y yaw, z pitch) =
// (0.0025, 0.0043, 0.0021) in Spade's Y-up design axes.
constexpr double kCompetitionMass = 0.700;
const glm::dvec3 kCompetitionInertia(0.0025, 0.0043, 0.0021);

// Kat's derived layout, +-0.075 x +-0.10 m, whose same-side discs overlap
// (the lead's ruling: the competition fit keeps it; the defaults do not).
[[nodiscard]] BasicQuadParams competition() {
    BasicQuadParams p;
    p.half_span_x = 0.075;
    p.mass = kCompetitionMass;
    p.inertia = kCompetitionInertia;
    return p;
}

}  // namespace

TEST(BasicQuad, TheDefaultsCompile) {
    const spade::Result<AirframeSpec> spec = spade::vehicles::basic_quad_airframe(BasicQuadParams{});
    ASSERT_TRUE(spec.has_value()) << spec.error().context;
    EXPECT_EQ(spec->name, "basic_quad");

    // The body and four arms; no remainder without a fit target.
    ASSERT_EQ(spec->parts.size(), 5u);
    EXPECT_EQ(spec->parts[0].geometry.name, "body");
    EXPECT_EQ(spec->parts[0].shape, PartShape::box);
    EXPECT_EQ(spec->parts[0].mass, 0.12);

    // The hubs: front-right, front-left, rear-left, rear-right; the diagonals
    // spin alike.
    ASSERT_EQ(spec->rotors.size(), 4u);
    const glm::dvec3 hubs[4] = {{0.080, 0.022, 0.10}, {0.080, 0.022, -0.10}, {-0.080, 0.022, -0.10},
                                {-0.080, 0.022, 0.10}};
    const double spins[4] = {1.0, -1.0, 1.0, -1.0};
    for (std::size_t k = 0; k < 4; ++k) {
        EXPECT_EQ(spec->rotors[k].hub.position, hubs[k]) << k;
        EXPECT_EQ(spec->rotors[k].spin_dir, spins[k]) << k;
        EXPECT_FALSE(spec->rotors[k].pusher) << k;
    }

    // A valid generic 6-inch quad: same-side hubs clear the discs by at least
    // 5 mm (the lead's ruling), so nothing is advised.
    EXPECT_GE(2.0 * 0.080 - spec->prop.diameter, 0.005);
    EXPECT_TRUE(spade::vehicles::basic_quad_advisories(*spec).empty());

    // Each arm runs from the origin to its hub, centred at its midpoint,
    // turned from +Y onto the arm.
    const double arm_length = std::sqrt(0.080 * 0.080 + 0.10 * 0.10);
    for (std::size_t k = 0; k < 4; ++k) {
        const spade::vehicles::PartInertia& arm = spec->parts[1 + k];
        EXPECT_EQ(arm.geometry.name, "arm[" + std::to_string(k) + "]");
        EXPECT_EQ(arm.shape, PartShape::tube);
        EXPECT_EQ(arm.size.x, 0.006);
        EXPECT_EQ(arm.size.y, arm_length);
        EXPECT_EQ(arm.mass, 0.15 * arm_length);
        EXPECT_EQ(arm.position, glm::dvec3(hubs[k].x / 2, 0.0, hubs[k].z / 2));
        const glm::dvec3 along = arm.orientation * glm::dvec3(0.0, 1.0, 0.0);
        EXPECT_NEAR(along.x, hubs[k].x / arm_length, 1e-15) << k;
        EXPECT_NEAR(along.y, 0.0, 1e-15) << k;
        EXPECT_NEAR(along.z, hubs[k].z / arm_length, 1e-15) << k;
    }
    EXPECT_EQ(spec->imus.size(), 1u);
    EXPECT_EQ(spec->proxy_radius, arm_length + 0.5 * spec->prop.diameter);
    EXPECT_TRUE(spec->drag.empty());

    const spade::Result<CompiledAirframe> compiled = spade::vehicles::compile_airframe(*spec);
    ASSERT_TRUE(compiled.has_value()) << compiled.error().context;
    // Symmetric about x and z: the design axes are the principal axes.
    EXPECT_EQ(compiled->inertia.design_to_body, glm::quat(1.0f, 0.0f, 0.0f, 0.0f));

    // Every numeric default is listed as assumed, in declaration order,
    // with a note (the fields listed here from the struct, not the code).
    const std::vector<std::string_view> expected = {
        "half_span_x", "half_span_z", "hub_height", "body_size", "body_mass", "arm_radius",
        "arm_mass_per_metre",
        "motor.kv", "motor.resistance", "motor.no_load_current", "motor.no_load_voltage", "motor.current_max",
        "motor.pole_pairs", "motor.inductance", "motor.rotor_inertia", "motor.mass", "motor.shape",
        "prop.diameter", "prop.pitch", "prop.blades", "prop.inertia", "prop.ct_static", "prop.cq_static",
        "prop.mass",
        "esc.current_continuous", "esc.current_burst", "esc.current_total", "esc.channels", "esc.on_resistance",
        "esc.braking", "esc.mass", "esc.mount",
        "battery.cells_series", "battery.cells_parallel", "battery.cell_capacity", "battery.cell_resistance",
        "battery.cell_voltage_nominal", "battery.cell_voltage_full", "battery.cell_voltage_cutoff",
        "battery.ocv_table", "battery.c_rating", "battery.polarization_resistance",
        "battery.polarization_capacitance", "battery.mass", "battery.mount", "battery.shape",
        // Last, what the model assumes of every basic quad (Physics, via the lead).
        "rotors",
    };
    const std::vector<spade::vehicles::BasicQuadAssumption> assumed = spade::vehicles::basic_quad_assumptions();
    ASSERT_EQ(assumed.size(), expected.size());
    for (std::size_t i = 0; i < expected.size(); ++i) {
        EXPECT_EQ(assumed[i].field, expected[i]) << i;
        EXPECT_FALSE(assumed[i].note.empty()) << assumed[i].field;
    }
    EXPECT_TRUE(contains(assumed.back().note, "shared wake")) << assumed.back().note;
    EXPECT_TRUE(contains(assumed.back().note, "neighbour's induced inflow")) << assumed.back().note;
}

TEST(BasicQuad, TheCompiledBodyMeetsAFittedMassAndInertiaExactly) {
    const spade::Result<AirframeSpec> spec = spade::vehicles::basic_quad_airframe(competition());
    ASSERT_TRUE(spec.has_value()) << spec.error().context;

    // The remainder: one mass-only record after the arms.
    ASSERT_EQ(spec->parts.size(), 6u);
    const spade::vehicles::PartInertia& rest = spec->parts.back();
    EXPECT_EQ(rest.geometry.name, "remainder");
    EXPECT_EQ(rest.geometry.source, GeometrySource::none);
    EXPECT_EQ(rest.shape, PartShape::tensor);
    EXPECT_GT(rest.mass, 0.0);

    // The triangle inequality's margin, printed so a later default change
    // shows its cost before it becomes a refusal (the lead's ruling): the
    // remainder's smaller two moments less its largest. Its off-diagonals are
    // rounding, so its diagonal is its principal moments.
    double m[3] = {rest.tensor[0][0], rest.tensor[1][1], rest.tensor[2][2]};
    std::sort(std::begin(m), std::end(m));
    const double margin = m[0] + m[1] - m[2];
    std::printf("[ margin   ] competition remainder: I1 + I2 - I3 = %.3e kg m^2 (%.1f%% of its largest moment)\n",
                margin, 100.0 * margin / m[2]);
    EXPECT_GT(margin, 0.0);

    const spade::Result<CompiledAirframe> compiled = spade::vehicles::compile_airframe(*spec);
    ASSERT_TRUE(compiled.has_value()) << compiled.error().context;
    EXPECT_EQ(bits(compiled->inertia.mass), bits(static_cast<float>(kCompetitionMass)));
    for (int a = 0; a < 3; ++a) {
        EXPECT_EQ(bits(compiled->inertia.principal_moments[a]), bits(static_cast<float>(kCompetitionInertia[a])))
            << "axis " << a << ": " << compiled->inertia.principal_moments[a];
    }
    EXPECT_EQ(compiled->inertia.design_to_body, glm::quat(1.0f, 0.0f, 0.0f, 0.0f));

    // A mass target alone: a point remainder, the parts' inertia kept.
    BasicQuadParams mass_only;
    mass_only.mass = kCompetitionMass;
    const spade::Result<AirframeSpec> heavier = spade::vehicles::basic_quad_airframe(mass_only);
    ASSERT_TRUE(heavier.has_value()) << heavier.error().context;
    ASSERT_EQ(heavier->parts.size(), 6u);
    EXPECT_EQ(heavier->parts.back().shape, PartShape::point);
    const spade::Result<CompiledAirframe> heavier_compiled = spade::vehicles::compile_airframe(*heavier);
    ASSERT_TRUE(heavier_compiled.has_value()) << heavier_compiled.error().context;
    EXPECT_EQ(bits(heavier_compiled->inertia.mass), bits(static_cast<float>(kCompetitionMass)));
}

TEST(BasicQuad, AFittedInertiaBelowThePartsIsRefusedNamingTheAxis) {
    // The parts alone are about (0.00203, 0.00313, 0.00138) kg m^2.
    BasicQuadParams p = competition();
    p.inertia = glm::dvec3(0.0025, 0.0043, 0.0010);
    expect_refused(spade::vehicles::basic_quad_airframe(p), {"inertia.z", "0.001", "below the parts'"});
    p.inertia = glm::dvec3(0.0015, 0.0043, 0.0021);
    expect_refused(spade::vehicles::basic_quad_airframe(p), {"inertia.x", "0.0015", "below the parts'"});
    p.inertia = glm::dvec3(0.0025, 0.0030, 0.0021);
    expect_refused(spade::vehicles::basic_quad_airframe(p), {"inertia.y", "0.003", "below the parts'"});

    // The mass, likewise: the parts weigh about 0.6006 kg.
    BasicQuadParams light = competition();
    light.mass = 0.5;
    expect_refused(spade::vehicles::basic_quad_airframe(light), {"mass", "0.5", "parts' own"});
}

TEST(BasicQuad, ARemainderThatIsNotABodyIsRefusedByName) {
    // Every axis above the parts', yet the remainder's yaw exceeds its roll
    // plus pitch: no body has that tensor.
    BasicQuadParams p = competition();
    p.inertia = glm::dvec3(0.0025, 0.0049, 0.0021);
    // It names the target and the parts' own value, as every fit refusal does.
    expect_refused(spade::vehicles::basic_quad_airframe(p),
                   {"inertia", "remainder", "triangle inequality", "diag(0.0025, 0.0049, 0.0021)",
                    "the parts' own diag(0.00203"});

    // An inertia target needs a mass target: a massless remainder is no body.
    BasicQuadParams no_mass;
    no_mass.inertia = kCompetitionInertia;
    expect_refused(spade::vehicles::basic_quad_airframe(no_mass), {"inertia", "needs a mass target"});

    // Non-finite targets are refused by name.
    BasicQuadParams nan_mass;
    nan_mass.mass = std::nan("");
    expect_refused(spade::vehicles::basic_quad_airframe(nan_mass), {"mass", "not finite"});
}

TEST(BasicQuad, AFittedKappaSetsCqStaticAndScalesTheTables) {
    BasicQuadParams p;
    p.kappa = 0.022;
    const spade::Result<AirframeSpec> spec = spade::vehicles::basic_quad_airframe(p);
    ASSERT_TRUE(spec.has_value()) << spec.error().context;
    EXPECT_EQ(bits(spec->prop.cq_static), bits(0.022 * p.prop.ct_static / p.prop.diameter));
    EXPECT_EQ(bits(spec->prop.ct_static), bits(p.prop.ct_static));
    EXPECT_TRUE(spade::vehicles::compile_airframe(*spec).has_value());

    // With tables, the thrust table stays and the torque table is derived at
    // its J points, C_Q(J) = kappa C_T(J) / D (the lead's ruling; Physics
    // confirmed it exact): the fitted kappa replaces the measured C_Q.
    BasicQuadParams tables = p;
    tables.prop.ct_table = {{0.0, 0.12}, {0.4, 0.09}, {0.8, 0.04}};
    tables.prop.cq_table = {{0.0, 0.008}, {0.8, 0.004}};
    const spade::Result<AirframeSpec> scaled = spade::vehicles::basic_quad_airframe(tables);
    ASSERT_TRUE(scaled.has_value()) << scaled.error().context;
    ASSERT_EQ(scaled->prop.ct_table.size(), 3u);
    ASSERT_EQ(scaled->prop.cq_table.size(), 3u);
    for (std::size_t i = 0; i < 3; ++i) {
        EXPECT_EQ(bits(scaled->prop.ct_table[i][0]), bits(tables.prop.ct_table[i][0])) << i;
        EXPECT_EQ(bits(scaled->prop.ct_table[i][1]), bits(tables.prop.ct_table[i][1])) << i;
        EXPECT_EQ(bits(scaled->prop.cq_table[i][0]), bits(tables.prop.ct_table[i][0])) << i;
        EXPECT_EQ(bits(scaled->prop.cq_table[i][1]), bits(0.022 * tables.prop.ct_table[i][1] / tables.prop.diameter))
            << i;
    }
    EXPECT_TRUE(spade::vehicles::compile_airframe(*scaled).has_value());

    // A torque table with no thrust table to scale from is refused, not
    // silently replaced by an empty one.
    BasicQuadParams torque_only = p;
    torque_only.prop.cq_table = {{0.0, 0.008}, {0.8, 0.004}};
    expect_refused(spade::vehicles::basic_quad_airframe(torque_only), {"kappa", "prop.cq_table", "ct_table"});

    // A thrust table alone: kappa fills in the torque table from it.
    BasicQuadParams thrust_only = p;
    thrust_only.prop.ct_table = {{0.0, 0.12}, {0.8, 0.04}};
    const spade::Result<AirframeSpec> filled = spade::vehicles::basic_quad_airframe(thrust_only);
    ASSERT_TRUE(filled.has_value()) << filled.error().context;
    ASSERT_EQ(filled->prop.cq_table.size(), 2u);
    for (std::size_t i = 0; i < 2; ++i) {
        const std::array<double, 2>& ct = thrust_only.prop.ct_table[i];
        EXPECT_EQ(bits(filled->prop.cq_table[i][0]), bits(ct[0])) << i;
        EXPECT_EQ(bits(filled->prop.cq_table[i][1]), bits(0.022 * ct[1] / thrust_only.prop.diameter)) << i;
    }
    EXPECT_TRUE(spade::vehicles::compile_airframe(*filled).has_value());

    BasicQuadParams negative = p;
    negative.kappa = -0.022;
    expect_refused(spade::vehicles::basic_quad_airframe(negative), {"kappa", "above 0"});
}

// Overlapping actuator discs are sound (Physics, 2026-10-21): each keeps its
// own thrust and torque. So a disc pair closer than the diameter is advised,
// never refused, with what the model leaves out.
TEST(BasicQuad, RotorDiscsCloserThanTheirDiameterAreAdvisedNotRefused) {
    const spade::Result<AirframeSpec> spec = spade::vehicles::basic_quad_airframe(competition());
    ASSERT_TRUE(spec.has_value()) << spec.error().context;
    const std::vector<spade::vehicles::BasicQuadAdvisory> advised = spade::vehicles::basic_quad_advisories(*spec);
    // Front-right with rear-right, and front-left with rear-left: 150 mm
    // apart under a 152.4 mm propeller.
    ASSERT_EQ(advised.size(), 2u);
    EXPECT_EQ(advised[0].field, "rotors[0], rotors[3]");
    EXPECT_EQ(advised[1].field, "rotors[1], rotors[2]");
    for (const spade::vehicles::BasicQuadAdvisory& a : advised) {
        for (const std::string_view needle :
             {"overlap by 2.4 mm", "150.0 mm apart", "152.4 mm", "own thrust and torque", "shared wake"}) {
            EXPECT_TRUE(contains(a.message, needle)) << "missing '" << needle << "' in: " << a.message;
        }
    }
    EXPECT_TRUE(contains(advised[0].message, "rotors[0] and rotors[3]")) << advised[0].message;
    // The spec still compiles: an advisory is not a refusal.
    EXPECT_TRUE(spade::vehicles::compile_airframe(*spec).has_value());

    // Measured in the disc plane, the spin axis projected out: one hub raised
    // 10 mm still overlaps its neighbour by 2.4 mm (a 3D distance would say
    // 2.1), and the advisory names the axial offset.
    AirframeSpec raised = *spec;
    raised.rotors[3].hub.position.y += 0.010;
    const std::vector<spade::vehicles::BasicQuadAdvisory> lifted = spade::vehicles::basic_quad_advisories(raised);
    ASSERT_EQ(lifted.size(), 2u);
    EXPECT_TRUE(contains(lifted[0].message, "overlap by 2.4 mm")) << lifted[0].message;
    EXPECT_TRUE(contains(lifted[0].message, "150.0 mm apart in the disc plane")) << lifted[0].message;
    EXPECT_TRUE(contains(lifted[0].message, "10.0 mm apart along the spin axis")) << lifted[0].message;
    EXPECT_FALSE(contains(lifted[1].message, "along the spin axis")) << lifted[1].message;
}

TEST(BasicQuad, AHalfSpanNotAboveZeroIsRefused) {
    BasicQuadParams p;
    p.half_span_z = 0.0;
    expect_refused(spade::vehicles::basic_quad_airframe(p), {"half_span_z", "above 0"});
    p.half_span_z = 0.10;
    p.half_span_x = std::nan("");
    expect_refused(spade::vehicles::basic_quad_airframe(p), {"half_span_x"});
}
