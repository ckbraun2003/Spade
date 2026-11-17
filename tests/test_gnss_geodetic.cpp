// SP-3 of the geo plan (L85, docs/design/core/2026-11-17-geo-g1-design.md): a GNSS poll that also reports WGS84 coordinates when the
// receiver's world has an anchor (Simulation::poll_gnss_geodetic).
//
// What is pinned: with noise off a receiver at world (0, 0, 0) reports the anchor, and one 100 m east and 100 m north reports the
// first-order coordinates within 1 cm (V2); with noise on, each coordinate triple comes from the SAME stored noisy position as its fix
// (REQ-G19); nothing is drawn from the noise stream (REQ-G01b), so an anchored world's fixes are byte-identical to a plain world's; a
// plain world refuses with `unavailable` rather than report zeros; the ring and poll_gnss are untouched.
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <optional>
#include <vector>

#include <glm/vec3.hpp>

#include "core/error.hpp"
#include "core/geodesy.hpp"
#include "sensors/gnss.hpp"
#include "sensors/rings.hpp"
#include "sim/simulation.hpp"
#include "sim/world_set.hpp"
#include "world/builder.hpp"

namespace {

namespace geo = spade::geo;
using spade::GnssSensorRef;
using spade::GnssSensorSpawn;
using spade::Simulation;
using spade::sensors::GnssFix;

constexpr double kPi = 3.14159265358979323846;
const geo::Geodetic kAnchor{40.76, -113.9, 1286.5};

[[nodiscard]] spade::WorldSetDesc world_set(std::optional<geo::Geodetic> anchor) {
    spade::Environment env;
    env.gravity = glm::vec3(0.0f);  // a body left at rest stays where it was put
    env.wind = glm::vec3(0.0f);
    env.air_density = 1.225f;
    spade::Capacities caps;
    caps.bodies = 4;
    caps.force_elements = 1;
    caps.sensors = 2;
    caps.contacts = 1;
    const spade::Result<spade::WorldDesc> built = spade::WorldBuilder().name("geo").environment(env).capacities(caps).build();
    EXPECT_TRUE(built.has_value()) << (built ? "" : built.error().context);
    spade::WorldInstanceDesc inst;
    inst.world = *built;
    inst.world.anchor = anchor;
    inst.seed = 0x5EEDu;
    inst.turbulence = spade::dryden_params(spade::TurbulenceLevel::none);
    inst.grid.cell_size = 1.0f;
    return spade::WorldSetDesc{{inst}};
}

struct Rig {
    Simulation sim;
    GnssSensorRef ref;
};

[[nodiscard]] Rig make_rig(std::optional<geo::Geodetic> anchor, glm::vec3 at, const GnssSensorSpawn& receiver) {
    spade::Result<Simulation> sim = Simulation::create(world_set(anchor), 1'000'000, 1);
    EXPECT_TRUE(sim.has_value()) << (sim ? "" : sim.error().context);
    spade::BodySpawn body;
    body.pos = at;
    body.mass = 1.0f;
    body.inv_inertia_diag = glm::vec3(1.0f);
    const spade::Result<spade::BodyRef> b = sim->spawn(0, body);
    EXPECT_TRUE(b.has_value()) << (b ? "" : b.error().context);
    const spade::Result<GnssSensorRef> ref = sim->add_gnss_sensor(*b, receiver);
    EXPECT_TRUE(ref.has_value()) << (ref ? "" : ref.error().context);
    EXPECT_TRUE(sim->flush_structural().has_value());
    return Rig{std::move(*sim), *ref};
}

[[nodiscard]] GnssSensorSpawn quiet_receiver() {
    GnssSensorSpawn g;
    g.rate_divider = 1;  // a fix every substep
    return g;           // every sigma 0, no bias
}

[[nodiscard]] GnssSensorSpawn noisy_receiver() {
    GnssSensorSpawn g;
    g.rate_divider = 1;
    g.sigma_h = 1.5f;
    g.sigma_v = 2.5f;
    g.sigma_vel = 0.125f;
    g.bias_tau_s = 60.0f;
    g.sigma_bias = 0.8f;
    return g;
}

}  // namespace

TEST(GnssGeodetic, ANoiselessReceiverAtTheOriginReportsTheAnchor) {
    Rig rig = make_rig(kAnchor, glm::vec3(0.0f), quiet_receiver());
    ASSERT_TRUE(rig.sim.step(3).has_value());
    std::vector<GnssFix> fixes(spade::sensors::kRingDepth);
    std::vector<geo::Geodetic> geodetic(fixes.size());
    const spade::Result<spade::GnssPoll> poll = rig.sim.poll_gnss_geodetic(rig.ref, 0, fixes, geodetic);
    ASSERT_TRUE(poll.has_value()) << poll.error().context;
    ASSERT_FALSE(poll->samples.empty());
    for (std::size_t i = 0; i < poll->samples.size(); ++i) {
        EXPECT_EQ(poll->samples[i].position, glm::vec3(0.0f));
        EXPECT_NEAR(geodetic[i].lat_deg, kAnchor.lat_deg, 1e-9) << i;
        EXPECT_NEAR(geodetic[i].lon_deg, kAnchor.lon_deg, 1e-9) << i;
        EXPECT_NEAR(geodetic[i].height_m, kAnchor.height_m, 1e-6) << i;
    }
}

TEST(GnssGeodetic, AReceiver100mEastAnd100mNorthReportsTheExpectedCoordinatesWithin1cm) {
    // World x east, z south: 100 m north is z = -100.
    Rig rig = make_rig(kAnchor, glm::vec3(100.0f, 0.0f, -100.0f), quiet_receiver());
    ASSERT_TRUE(rig.sim.step(2).has_value());
    std::vector<GnssFix> fixes(spade::sensors::kRingDepth);
    std::vector<geo::Geodetic> geodetic(fixes.size());
    const spade::Result<spade::GnssPoll> poll = rig.sim.poll_gnss_geodetic(rig.ref, 0, fixes, geodetic);
    ASSERT_TRUE(poll.has_value()) << poll.error().context;
    ASSERT_FALSE(poll->samples.empty());

    // The first-order formulas, written out independently of core/geodesy: the meridian and prime-vertical radii at the anchor.
    const double phi = kAnchor.lat_deg * (kPi / 180.0);
    const double s = std::sin(phi);
    const double c = std::cos(phi);
    const double w = 1.0 - geo::kWgs84E2 * s * s;
    const double nu = geo::kWgs84A / std::sqrt(w);
    const double m = geo::kWgs84A * (1.0 - geo::kWgs84E2) / (w * std::sqrt(w));
    const double lat = kAnchor.lat_deg + (100.0 / (m + kAnchor.height_m)) * (180.0 / kPi);
    const double lon = kAnchor.lon_deg + (100.0 / ((nu + kAnchor.height_m) * c)) * (180.0 / kPi);
    const double tol_lat = 0.01 / m * (180.0 / kPi);
    const double tol_lon = 0.01 / (nu * c) * (180.0 / kPi);
    const geo::Geodetic got = geodetic[0];
    EXPECT_NEAR(got.lat_deg, lat, tol_lat) << "1 cm of latitude is " << tol_lat << " degrees";
    EXPECT_NEAR(got.lon_deg, lon, tol_lon) << "1 cm of longitude is " << tol_lon << " degrees";
    EXPECT_NEAR(got.height_m, kAnchor.height_m, 0.01) << "the plane is 1.6 mm above the ellipsoid 141 m out";
    EXPECT_GT(got.height_m, kAnchor.height_m) << "the tangent plane leaves the ellipsoid";
}

TEST(GnssGeodetic, EachTripleComesFromTheSameNoisyPositionAsItsFix) {
    Rig rig = make_rig(kAnchor, glm::vec3(10.0f, 20.0f, -30.0f), noisy_receiver());
    ASSERT_TRUE(rig.sim.step(8).has_value());
    std::vector<GnssFix> fixes(spade::sensors::kRingDepth);
    std::vector<geo::Geodetic> geodetic(fixes.size());
    const spade::Result<spade::GnssPoll> poll = rig.sim.poll_gnss_geodetic(rig.ref, 0, fixes, geodetic);
    ASSERT_TRUE(poll.has_value()) << poll.error().context;
    ASSERT_GE(poll->samples.size(), 4u);
    const geo::LocalFrame frame = geo::make_local_frame(kAnchor);
    float spread = 0.0f;
    for (std::size_t i = 0; i < poll->samples.size(); ++i) {
        const glm::vec3 p = poll->samples[i].position;
        const glm::dvec3 back = geo::geodetic_to_world(frame, geodetic[i]);
        EXPECT_NEAR(back.x, static_cast<double>(p.x), 1e-6) << i;
        EXPECT_NEAR(back.y, static_cast<double>(p.y), 1e-6) << i;
        EXPECT_NEAR(back.z, static_cast<double>(p.z), 1e-6) << i;
        spread = std::max(spread, std::abs(p.x - 10.0f));
    }
    EXPECT_GT(spread, 0.1f) << "the noise is on: the fixes are not the true position";
}

TEST(GnssGeodetic, NothingIsDrawnFromTheNoiseStream) {
    Rig anchored = make_rig(kAnchor, glm::vec3(10.0f, 20.0f, -30.0f), noisy_receiver());
    Rig plain = make_rig(std::nullopt, glm::vec3(10.0f, 20.0f, -30.0f), noisy_receiver());
    ASSERT_TRUE(anchored.sim.step(6).has_value());
    ASSERT_TRUE(plain.sim.step(6).has_value());

    // The stream's bytes before and after a geodetic poll.
    const auto stream_bytes = [](const Simulation& sim, GnssSensorRef ref) {
        const spade::Result<const spade::sensors::GnssSensorRow*> row = sim.gnss_sensor(ref);
        EXPECT_TRUE(row.has_value());
        std::vector<unsigned char> bytes(sizeof((*row)->noise));
        std::memcpy(bytes.data(), &(*row)->noise, bytes.size());
        return bytes;
    };
    const std::vector<unsigned char> before = stream_bytes(anchored.sim, anchored.ref);
    std::vector<GnssFix> a(spade::sensors::kRingDepth);
    std::vector<geo::Geodetic> geodetic(a.size());
    const spade::Result<spade::GnssPoll> pa = anchored.sim.poll_gnss_geodetic(anchored.ref, 0, a, geodetic);
    ASSERT_TRUE(pa.has_value()) << pa.error().context;
    EXPECT_EQ(stream_bytes(anchored.sim, anchored.ref), before);

    // An anchor does not change the noise: the same world without one produces the same fixes, byte for byte.
    std::vector<GnssFix> p(spade::sensors::kRingDepth);
    const spade::Result<spade::GnssPoll> pp = plain.sim.poll_gnss(plain.ref, 0, p);
    ASSERT_TRUE(pp.has_value());
    ASSERT_EQ(pa->samples.size(), pp->samples.size());
    ASSERT_FALSE(pp->samples.empty());
    EXPECT_EQ(std::memcmp(pa->samples.data(), pp->samples.data(), pa->samples.size() * sizeof(GnssFix)), 0);
}

TEST(GnssGeodetic, PollGnssAndPollGnssGeodeticReturnTheSameFixes) {
    Rig rig = make_rig(kAnchor, glm::vec3(1.0f, 2.0f, 3.0f), noisy_receiver());
    ASSERT_TRUE(rig.sim.step(5).has_value());
    std::vector<GnssFix> a(spade::sensors::kRingDepth);
    std::vector<GnssFix> b(spade::sensors::kRingDepth);
    std::vector<geo::Geodetic> geodetic(b.size());
    const spade::Result<spade::GnssPoll> plain = rig.sim.poll_gnss(rig.ref, 0, a);
    const spade::Result<spade::GnssPoll> geo_poll = rig.sim.poll_gnss_geodetic(rig.ref, 0, b, geodetic);
    ASSERT_TRUE(plain.has_value() && geo_poll.has_value());
    ASSERT_EQ(plain->samples.size(), geo_poll->samples.size());
    EXPECT_EQ(plain->next_since, geo_poll->next_since);
    EXPECT_EQ(plain->dropped, geo_poll->dropped);
    EXPECT_EQ(std::memcmp(plain->samples.data(), geo_poll->samples.data(), plain->samples.size() * sizeof(GnssFix)), 0);
}

TEST(GnssGeodetic, APlainWorldRefusesAsUnavailableAndASmallBufferAsInvalid) {
    Rig plain = make_rig(std::nullopt, glm::vec3(0.0f), quiet_receiver());
    ASSERT_TRUE(plain.sim.step(2).has_value());
    std::vector<GnssFix> fixes(spade::sensors::kRingDepth);
    std::vector<geo::Geodetic> geodetic(fixes.size());
    const spade::Result<spade::GnssPoll> none = plain.sim.poll_gnss_geodetic(plain.ref, 0, fixes, geodetic);
    ASSERT_FALSE(none.has_value());
    EXPECT_EQ(none.error().code, spade::Code::unavailable);
    EXPECT_NE(none.error().context.find("no anchor"), std::string::npos) << none.error().context;

    Rig anchored = make_rig(kAnchor, glm::vec3(0.0f), quiet_receiver());
    ASSERT_TRUE(anchored.sim.step(2).has_value());
    std::vector<geo::Geodetic> small(2);
    const spade::Result<spade::GnssPoll> bad = anchored.sim.poll_gnss_geodetic(anchored.ref, 0, fixes, small);
    ASSERT_FALSE(bad.has_value());
    EXPECT_EQ(bad.error().code, spade::Code::invalid_argument);
}
