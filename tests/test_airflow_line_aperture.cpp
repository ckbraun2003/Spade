// ---------------------------------------------------------------------------
// test_airflow_line_aperture.cpp -- a GUARD on the actuator line's overlap with solid cells (fix 2 of the O158/O159 list, O167; measured and
// resolved 2026-11-22: the aperture-aware kernel was NOT built).
//
// The Gaussian kernel (eps 2 dx, support 6 cells) is blind to the solid field: a rotor whose kernel reaches a wall spreads part of its force
// onto CLOSED faces, where the projection drops it, and its elements read the wall's zero velocity as air. Measured with the film's own
// propeller (library uiuc_mit_5x4, polar fitted through the line) on the CPU line, with the solid field built by the solver's own
// build_solids() (so a thin wall is classified as the solver classifies it):
//
//   FPV motor top (OpenFrame-5F motor[0]: radius 14.65 mm, top 3.97 mm under the disc), 9,298 rpm: 0.00% of |increment| on closed faces,
//                                                                                                 0.00% of the axial impulse.
//   duct stand-in, 1.5 mm wall (a SHEET at 1.25 mm), 75 mm prop, 78 mm bore:                     0.03% and about 0.00%.
//   the same duct with a 3 mm wall (a wall, 2.4 dx):                                             0.18% and 0.24% (0.48% counting the
//                                                                                                 partly closed faces).
//   an element's felt air in the outer tenth of the span, next to the 3 mm wall:                 2.1% low (-2.938 against -3 m/s).
//
// So the loss is far under the line's own +-2.5% polar uncertainty, and the earlier "about 4% (FPV), 2% (X500)" figure, a GEOMETRIC bound on
// the kernel's overlap, is withdrawn (it ignored that the load sits at 0.5 to 0.85 of the radius). The ruling (the lead, relaying Oversight):
// no kernel change; THIS TEST STAYS AS THE GUARD. It fails at 1% of the axial impulse (or 1% of |increment|), so a change of kernel,
// resolution or geometry that makes the overlap matter fails loudly; it is to be re-run on the cinewhoop's real duct geometry before O159's
// solve, and the aperture-aware kernel (weights times theta, normalised over the open faces; CPU, then the three device kernels and their
// pin rows) is built only if that crosses the threshold.
//
// The duct is a STAND-IN (published dimensions, not any maker's frame): every size but the propeller's diameter is assumed or secondhand
// (docs/design/physics/plans, the cinewhoop table). The motor top is the OpenFrame-5F model's own.
// ---------------------------------------------------------------------------

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iterator>
#include <optional>
#include <span>
#include <string>
#include <vector>

#include <glm/glm.hpp>

#include "physics/airflow/blade.hpp"
#include "physics/airflow/line.hpp"
#include "physics/airflow/mac_grid.hpp"
#include "physics/airflow/solids.hpp"
#include "scene/part_file.hpp"
#include "scene/prop_line.hpp"

namespace {

namespace af = spade::physics::airflow;

constexpr float kRho = 1.225f;
constexpr float kDx = 0.00125f;

struct Prop {
    af::BladeShape blade;
    af::PolarTable polar;
};

// The film's propeller: the library's uiuc_mit_5x4, its polar fitted through the line.
[[nodiscard]] std::optional<Prop> film_prop() {
    const std::filesystem::path file = std::filesystem::path(SPADE_ASSETS_DIR) / "library/parts/uiuc_mit_5x4.prop.yaml";
    std::ifstream in(file, std::ios::binary);
    if (!in.good()) return std::nullopt;
    const std::string text((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
    const auto part = spade::scene::prop_from_yaml(text);
    if (!part) return std::nullopt;
    const auto prop = spade::scene::line_propeller(*part);
    if (!prop) return std::nullopt;
    return Prop{prop->blade, prop->polar};
}

// The same blade scaled to a tip radius (every length by the ratio, the angles and the polar as they are: a stand-in 75 mm propeller).
[[nodiscard]] af::BladeShape scaled(af::BladeShape b, float radius) {
    const float k = radius / b.radius;
    b.radius = radius;
    for (af::BladeStation& s : b.stations) {
        s.r *= k;
        s.chord *= k;
    }
    return b;
}

[[nodiscard]] af::MacGrid grid_of(uint32_t n) {
    af::MacGrid g;
    g.shape = {n, n, n, kDx};
    g.u.assign(std::size_t{n + 1u} * n * n, 0.0f);
    g.v.assign(std::size_t{n} * (n + 1u) * n, 0.0f);
    g.w.assign(std::size_t{n} * n * (n + 1u), 0.0f);
    g.p.assign(std::size_t{n} * n * n, 0.0f);
    return g;
}

// The solid field the SOLVER builds for one part: through build_solids(), so a thin part is classified as the solver classifies it (under
// 2 dx a SHEET: one layer of closed faces, the faces its mid-surface crosses; a wall otherwise) and the faces' open fractions are its own.
struct OnePart {
    std::function<float(glm::vec3)> phi;
};

[[nodiscard]] af::SolidQuery query_of(const OnePart& part, glm::vec3 x) {
    af::SolidQuery q;
    q.distance = part.phi(x);
    const float e = 2.0e-5f;
    const glm::vec3 g(part.phi(x + glm::vec3(e, 0, 0)) - part.phi(x - glm::vec3(e, 0, 0)),
                      part.phi(x + glm::vec3(0, e, 0)) - part.phi(x - glm::vec3(0, e, 0)),
                      part.phi(x + glm::vec3(0, 0, e)) - part.phi(x - glm::vec3(0, 0, e)));
    const float n = std::sqrt(g.x * g.x + g.y * g.y + g.z * g.z);
    q.gradient = n > 0.0f ? g / n : glm::vec3(0.0f, 1.0f, 0.0f);
    q.part = 0;
    return q;
}

[[nodiscard]] af::SolidField solver_field(const af::GridShape& shape, const std::function<float(glm::vec3)>& phi, af::SolidPart part) {
    OnePart one{phi};
    const af::SolidSampler sampler{
        &one, [](const void* c, glm::vec3 x) noexcept { return query_of(*static_cast<const OnePart*>(c), x); },
        [](const void* c, uint32_t, glm::vec3 x) noexcept { return query_of(*static_cast<const OnePart*>(c), x); }};
    af::SolidField f;
    const auto made = af::make_solid_field(shape);
    if (!made) return f;
    f = *made;
    const af::SolidPart parts[1] = {std::move(part)};
    const auto built = af::build_solids(f, shape, {false, false, false}, std::span<const af::SolidPart>(parts, 1), sampler);
    EXPECT_TRUE(built.has_value()) << (built ? "" : built.error().context);
    return f;
}

[[nodiscard]] af::SolidPart part_of(const char* name, glm::vec3 lo, glm::vec3 hi, float min_feature) {
    af::SolidPart p;
    p.name = name;
    p.min_feature = min_feature;
    p.extent = hi - lo;
    p.box_min = lo;
    p.box_max = hi;
    return p;
}

struct Increments {
    std::vector<float> du, dv, dw;
    explicit Increments(const af::MacGrid& g) : du(g.u.size(), 0.0f), dv(g.v.size(), 0.0f), dw(g.w.size(), 0.0f) {}
    [[nodiscard]] const std::vector<float>& of(uint32_t c) const { return c == 0 ? du : c == 1 ? dv : dw; }
};

struct Landed {
    double total = 0.0;      // sum of |increment| over every face
    double in_closed = 0.0;  // the part on closed faces (theta == 0)
    double blocked = 0.0;    // sum |increment| (1 - theta): what the partly closed faces also withhold
    double axial = 0.0;      // the signed sum of the axis component (v, the rotor's axis is +Y)
    double axial_closed = 0.0;
    [[nodiscard]] double closed_share() const { return total > 0.0 ? in_closed / total : 0.0; }
    [[nodiscard]] double axial_closed_share() const { return axial != 0.0 ? axial_closed / axial : 0.0; }
};

[[nodiscard]] Landed landed(const af::GridShape& shape, const Increments& inc, const af::SolidField& field) {
    Landed out;
    for (uint32_t c = 0; c < 3; ++c) {
        af::for_each_sample(shape, c, [&](std::size_t idx, glm::vec3) {
            const double a = static_cast<double>(inc.of(c)[idx]);
            const float theta = field.theta[c][idx];
            out.total += std::fabs(a);
            out.blocked += std::fabs(a) * (1.0 - static_cast<double>(theta));
            if (af::closed(theta)) out.in_closed += std::fabs(a);
            if (c == 1) {
                out.axial += a;
                if (af::closed(theta)) out.axial_closed += a;
            }
        });
    }
    return out;
}

[[nodiscard]] af::LineRotor rotor_in(const Prop& p, const af::BladeShape& blade, glm::vec3 centre, float omega) {
    af::LineRotor r;
    r.centre = centre;
    r.blade = &blade;
    r.polar = &p.polar;
    r.omega = omega;
    return r;
}

// The solid shell of a finite duct about +Y through `c`: inner radius r_in, wall t, half-length half, as a signed distance (negative inside).
[[nodiscard]] float duct_phi(glm::vec3 x, glm::vec3 c, float r_in, float t, float half) {
    const float r = std::sqrt((x.x - c.x) * (x.x - c.x) + (x.z - c.z) * (x.z - c.z));
    const float radial = std::fabs(r - (r_in + 0.5f * t)) - 0.5f * t;
    const float axial = std::fabs(x.y - c.y) - half;
    return std::max(radial, axial);
}

// A solid cylinder about +Y: radius r, its top face at y = top, its bottom at y = bottom.
[[nodiscard]] float post_phi(glm::vec3 x, glm::vec3 c, float r, float top, float bottom) {
    const float rr = std::sqrt((x.x - c.x) * (x.x - c.x) + (x.z - c.z) * (x.z - c.z));
    return std::max(rr - r, std::max(x.y - top, bottom - x.y));
}

}  // namespace

// THE DUCT. A 75 mm propeller (the film's blade scaled, a stand-in) in a duct 78 mm across: the tip's kernel (6 cells, 7.5 mm) reaches the
// wall. How much of the line's force lands on closed faces, as a sheet (1.5 mm) and as a wall (3 mm). The guard: under 1%.
TEST(AirflowLineAperture, ADuctWallTakesUnderOnePercentOfTheForce) {
    const auto prop = film_prop();
    ASSERT_TRUE(prop.has_value());
    const af::BladeShape blade = scaled(prop->blade, 0.0375f);
    const glm::vec3 centre(0.075f);
    const af::LineRotor r = rotor_in(*prop, blade, centre, 2000.0f);
    const af::MacGrid g = grid_of(120);
    Increments spread(g);
    ASSERT_TRUE(af::spread_line(r, g, kRho, 0.000025f, 1, spread.du, spread.dv, spread.dw).has_value());
    for (const float wall : {0.0015f, 0.003f}) {
        const glm::vec3 lo = centre - glm::vec3(0.039f + wall, 0.015f, 0.039f + wall);
        const glm::vec3 hi = centre + glm::vec3(0.039f + wall, 0.015f, 0.039f + wall);
        const af::SolidField duct =
            solver_field(g.shape, [&](glm::vec3 x) { return duct_phi(x, centre, 0.039f, wall, 0.015f); }, part_of("duct", lo, hi, wall));
        const Landed l = landed(g.shape, spread, duct);
        std::printf("[ aperture ] duct, %.1f mm wall (%s): %.2f%% of |increment| on closed faces, %.2f%% of the axial impulse, %.2f%% withheld counting the "
                    "partly closed\n",
                    static_cast<double>(wall) * 1000.0, wall < 2.0f * kDx ? "a sheet" : "a wall", 100.0 * l.closed_share(),
                    100.0 * l.axial_closed_share(), 100.0 * l.blocked / l.total);
        EXPECT_LT(std::fabs(l.axial_closed_share()), 0.01) << wall << " m wall: the axial impulse lost to closed faces";
        EXPECT_LT(l.closed_share(), 0.01) << wall << " m wall: |increment| on closed faces";
        EXPECT_LT(l.blocked / l.total, 0.01) << wall << " m wall: |increment| withheld counting the partly closed faces";
    }
}

// THE FPV'S MOTOR TOP. The OpenFrame-5F's motor is a cylinder of radius 14.65 mm whose top is 3.97 mm under the disc plane; the blade root
// (the library propeller's first station) is inside its radius, in reach of its top. The same guard.
TEST(AirflowLineAperture, TheMotorTopTakesUnderOnePercentOfTheForce) {
    const auto prop = film_prop();
    ASSERT_TRUE(prop.has_value());
    const glm::vec3 centre(0.075f);
    const af::LineRotor r = rotor_in(*prop, prop->blade, centre, 973.704f);
    const af::MacGrid g = grid_of(120);
    const float top = centre.y - 0.00397f;
    const af::SolidField motor = solver_field(
        g.shape, [&](glm::vec3 x) { return post_phi(x, centre, 0.01465f, top, top - 0.0306f); },
        part_of("motor", glm::vec3(centre.x - 0.01465f, top - 0.0306f, centre.z - 0.01465f),
                glm::vec3(centre.x + 0.01465f, top, centre.z + 0.01465f), 0.0f));
    Increments spread(g);
    ASSERT_TRUE(af::spread_line(r, g, kRho, 0.000025f, 1, spread.du, spread.dv, spread.dw).has_value());
    const Landed l = landed(g.shape, spread, motor);
    std::printf("[ aperture ] FPV motor top (R 63.5 mm film prop at 9,298 rpm, motor 14.65 mm radius, top 3.97 mm under the disc): %.2f%% of |increment| "
                "on closed faces, %.2f%% of the axial impulse; %.2f%% withheld counting the partly closed\n",
                100.0 * l.closed_share(), 100.0 * l.axial_closed_share(), 100.0 * l.blocked / l.total);
    EXPECT_LT(std::fabs(l.axial_closed_share()), 0.01) << "the axial impulse lost to closed faces";
    EXPECT_LT(l.closed_share(), 0.01);
    EXPECT_LT(l.blocked / l.total, 0.01);
}

// THE AIR AN ELEMENT FEELS. Open faces carry a steady 3 m/s along -Y; closed faces hold the wall's 0. An element whose kernel reaches the duct
// wall reads a mean that the wall's zeros pull down. The guard: the outer tenth of the span reads within 5% of the open air (measured 2.1% low
// next to a 3 mm wall, 2026-11-22).
TEST(AirflowLineAperture, AnElementNearTheWallReadsWithinFivePercentOfTheOpenAir) {
    const auto prop = film_prop();
    ASSERT_TRUE(prop.has_value());
    const af::BladeShape blade = scaled(prop->blade, 0.0375f);
    const glm::vec3 centre(0.075f);
    const af::LineRotor r = rotor_in(*prop, blade, centre, 2000.0f);
    af::MacGrid g = grid_of(120);
    const af::SolidField duct =
        solver_field(g.shape, [&](glm::vec3 x) { return duct_phi(x, centre, 0.039f, 0.003f, 0.015f); },
                     part_of("duct", centre - glm::vec3(0.042f, 0.015f, 0.042f), centre + glm::vec3(0.042f, 0.015f, 0.042f), 0.003f));
    af::for_each_sample(g.shape, 1, [&](std::size_t idx, glm::vec3) { g.v[idx] = af::closed(duct.theta[1][idx]) ? 0.0f : -3.0f; });
    double sum = 0.0;
    int n = 0;
    for (const af::LineElement& e : af::line_elements(r, kDx, 0.000025f, 1)) {
        if (e.r < 0.9f * blade.radius) continue;
        sum += static_cast<double>(af::line_sample(g, e.pos).y);
        ++n;
    }
    ASSERT_GT(n, 0);
    std::printf("[ aperture ] the outer tenth of the span (%d elements) reads a mean axial air of %.3f m/s next to a 3 mm wall (open air: -3)\n", n,
                sum / n);
    EXPECT_NEAR(sum / n, -3.0, 0.15);
}
