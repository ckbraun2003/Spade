// ---------------------------------------------------------------------------
// test_baro.cpp -- the barometer (sensors/baro.hpp, the opt-in module `baro`, Phase 1 of the flight-computer work, O144): its layout, the ISA truth function
// against the double-precision law, the sensor in a world (height, mount, origin altitude, the emission clock, noise and its seeds, the turn-on bias and its
// reseed), the refusals, snapshot and restore, and what the module is not (not in the standard set; absent on Vulkan). This test uses libm for its reference
// values: tests may, engine/ may not.
// ---------------------------------------------------------------------------

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <optional>
#include <string>
#include <vector>

#include <glm/glm.hpp>

#include "sensors/baro.hpp"
#include "sim/module.hpp"
#include "sim/simulation.hpp"

namespace {

using spade::BodyRef;
using spade::BodySpawn;
using spade::Capacities;
using spade::Environment;
using spade::RowRef;
using spade::Simulation;
using spade::TurbulenceLevel;
using spade::WorldBuilder;
using spade::WorldInstanceDesc;
using spade::WorldSetDesc;
using spade::sensors::BaroSample;
using spade::sensors::BaroSensorRow;
using spade::sensors::BaroSensorSpawn;

// The ISA in double: the reference the float truth function is held to.
double isa_pressure(double h) { return 101325.0 * std::pow(1.0 - 0.0065 * h / 288.15, 0.034163195 / 0.0065); }
double isa_temperature(double h) { return 288.15 - 0.0065 * h; }

#define ASSERT_OK(expr)                                                                                  \
    do {                                                                                                 \
        const auto& r_ = (expr);                                                                         \
        ASSERT_TRUE(r_.has_value()) << #expr << " failed: " << (r_ ? "" : r_.error().context);          \
    } while (false)

[[nodiscard]] spade::modules::ModuleSet standard_plus_baro() {
    spade::modules::ModuleSet set = spade::modules::standard_modules();
    set.push_back(spade::modules::baro_module());
    return set;
}

// A world with no gravity, so a body placed at a height stays there.
[[nodiscard]] spade::Result<WorldSetDesc> still_world(uint64_t seed) {
    Environment env;
    env.gravity = glm::vec3(0.0f);
    env.wind = glm::vec3(0.0f);
    env.air_density = 1.225f;
    Capacities caps;
    caps.bodies = 4;
    caps.force_elements = 1;
    caps.sensors = 2;
    caps.contacts = 1;
    const spade::Result<spade::WorldDesc> world = WorldBuilder().name("still").environment(env).capacities(caps).build();
    if (!world) return std::unexpected(world.error());
    spade::physics::ContactParams contacts;
    contacts.restitution_e = 0.0f;
    contacts.friction_mu = 0.0f;
    contacts.proxy_radius = 0.0f;
    spade::physics::GridParams grid;
    grid.cell_size = 1.0f;
    WorldInstanceDesc instance;
    instance.world = *world;
    instance.seed = seed;
    instance.turbulence = spade::dryden_params(TurbulenceLevel::none);
    instance.contacts = contacts;
    instance.grid = grid;
    return WorldSetDesc{{instance}};
}

struct Rig {
    Simulation sim;
    BodyRef body;
    RowRef row;
    spade::sensors::SampleIndex since = 0;  // the newest index run() has returned
};

// dt 2 ms, 2 substeps (h = 1 ms); a body at height `y`; one barometer with `spawn`.
[[nodiscard]] std::optional<Rig> make_rig(const BaroSensorSpawn& spawn, float y = 0.0f, uint64_t seed = 0x5EEDu,
                                          std::string* why = nullptr) {
    const spade::Result<WorldSetDesc> set = still_world(seed);
    if (!set) return std::nullopt;
    spade::Result<Simulation> sim = Simulation::create(*set, 2'000'000, 2, {}, standard_plus_baro());
    if (!sim) {
        if (why) *why = sim.error().context;
        return std::nullopt;
    }
    BodySpawn b;
    b.pos = glm::vec3(0.0f, y, 0.0f);
    const spade::Result<BodyRef> body = sim->spawn(0, b);
    if (!body) return std::nullopt;
    const spade::Result<RowRef> row = sim->attach_row(*body, "baro_sensors", spawn);
    if (!row) {
        if (why) *why = row.error().context;
        return std::nullopt;
    }
    if (!sim->flush_structural()) return std::nullopt;
    return Rig{std::move(*sim), *body, *row, 0};
}

// Every sample newer than `since`, in index order.
[[nodiscard]] std::vector<BaroSample> poll(const Simulation& sim, const RowRef& row, spade::sensors::SampleIndex since) {
    const auto rows = sim.module_rows<BaroSensorRow>("baro_sensors", row.world_index);
    const auto ring = sim.module_rows<BaroSample>("baro_ring", row.world_index);
    if (!rows || !ring) return {};
    std::vector<BaroSample> out(spade::sensors::kRingDepth);
    const auto window = std::span<const BaroSample>(*ring).subspan(std::size_t{row.slot} * spade::sensors::kRingDepth,
                                                                     spade::sensors::kRingDepth);
    const auto r = spade::sensors::ring_poll<BaroSample>(window, (*rows)[row.slot].last_index, since, out);
    out.resize(r.samples.size());
    return out;
}

// Step `steps` ticks, one at a time, collecting every sample (the ring is only 64 deep).
[[nodiscard]] std::vector<BaroSample> run(Rig& rig, uint64_t steps) {
    std::vector<BaroSample> all;
    for (uint64_t t = 0; t < steps; ++t) {
        EXPECT_TRUE(rig.sim.step(1).has_value());
        for (const BaroSample& s : poll(rig.sim, rig.row, rig.since)) {
            all.push_back(s);
            rig.since = s.index;
        }
    }
    return all;
}

}  // namespace

// --- layout and the function -------------------------------------------------

TEST(BaroLayout, TheRowAndTheSampleAreTheStd430RowsTheyClaim) {
    EXPECT_EQ(sizeof(BaroSensorRow), 112u);
    EXPECT_EQ(sizeof(BaroSample), 32u);
    EXPECT_EQ(offsetof(BaroSensorRow, noise), 80u);
    EXPECT_EQ(offsetof(BaroSensorRow, last_index), 96u);
    EXPECT_EQ(offsetof(BaroSample, index), 16u);
    EXPECT_EQ(spade::sensors::sensor_kind::baro, 4u);
    EXPECT_TRUE(spade::sensors::sensor_kind::is_live(spade::sensors::sensor_kind::baro));
}

TEST(BaroTruth, MatchesTheIsaInDoubleWithinATenthOfAPascal) {
    for (const float h : {0.0f, 100.0f, 500.0f, 3000.0f, 10000.0f, -500.0f}) {
        const spade::sensors::BaroTruth t = spade::sensors::baro_truth(h, 101325.0f, 288.15f, 0.0065f);
        EXPECT_NEAR(t.pressure_pa, isa_pressure(h), 0.1) << "h = " << h;
        EXPECT_NEAR(t.temperature_k, isa_temperature(h), 1e-3) << "h = " << h;
    }
}

TEST(BaroTruth, ThePressureAltitudeInvertsIt) {
    for (const float h : {0.0f, 100.0f, 500.0f, 3000.0f, 10000.0f, -500.0f}) {
        const spade::sensors::BaroTruth t = spade::sensors::baro_truth(h, 101325.0f, 288.15f, 0.0065f);
        EXPECT_NEAR(spade::sensors::baro_pressure_altitude(t.pressure_pa, 101325.0f, 288.15f, 0.0065f), h, 0.05) << "h = " << h;
    }
}

TEST(BaroTruth, IsTotalAtTheEdges) {
    // far above any troposphere the temperature is floored and nothing is NaN; a pressure of zero reads as 1 Pa
    const spade::sensors::BaroTruth t = spade::sensors::baro_truth(60000.0f, 101325.0f, 288.15f, 0.0065f);
    EXPECT_TRUE(std::isfinite(t.pressure_pa) && std::isfinite(t.temperature_k));
    EXPECT_TRUE(std::isfinite(spade::sensors::baro_pressure_altitude(0.0f, 101325.0f, 288.15f, 0.0065f)));
}

// --- the sensor in a world ---------------------------------------------------

TEST(Baro, ReadsTheIsaAtItsHeight) {
    BaroSensorSpawn s;
    std::string why;
    std::optional<Rig> rig = make_rig(s, 1000.0f, 0x5EEDu, &why);
    ASSERT_TRUE(rig.has_value()) << why;
    const std::vector<BaroSample> got = run(*rig, 1);
    ASSERT_EQ(got.size(), 2u) << "two substeps, divider 1: two samples in one step";
    EXPECT_EQ(got[0].index, 1u);
    EXPECT_EQ(got[0].tick, 0u);
    EXPECT_NEAR(got[0].pressure_pa, isa_pressure(1000.0), 0.1);
    EXPECT_NEAR(got[0].temperature_c, isa_temperature(1000.0) - 273.15, 0.01);
    EXPECT_NEAR(got[0].pressure_alt_m, 1000.0f, 0.1);
}

TEST(Baro, TheMountOffsetAndTheWorldOriginAltitudeBothMoveTheHeight) {
    BaroSensorSpawn mounted;
    mounted.mount_pos = glm::vec3(0.0f, 10.0f, 0.0f);
    std::optional<Rig> a = make_rig(mounted, 0.0f);
    ASSERT_TRUE(a.has_value());
    const std::vector<BaroSample> ga = run(*a, 1);
    ASSERT_FALSE(ga.empty());
    EXPECT_NEAR(ga[0].pressure_pa, isa_pressure(10.0), 0.1);

    BaroSensorSpawn lifted;
    lifted.world_origin_altitude_m = 500.0f;
    std::optional<Rig> b = make_rig(lifted, 0.0f);
    ASSERT_TRUE(b.has_value());
    const std::vector<BaroSample> gb = run(*b, 1);
    ASSERT_FALSE(gb.empty());
    EXPECT_NEAR(gb[0].pressure_pa, isa_pressure(500.0), 0.1);
}

TEST(Baro, ATemperatureCoefficientShiftsThePressureByItsProduct) {
    BaroSensorSpawn s;
    s.temp_coeff_pa_per_k = 2.0f;
    std::optional<Rig> rig = make_rig(s, 1000.0f);
    ASSERT_TRUE(rig.has_value());
    const std::vector<BaroSample> got = run(*rig, 1);
    ASSERT_FALSE(got.empty());
    const double expected = isa_pressure(1000.0) + 2.0 * (isa_temperature(1000.0) - 288.15);
    EXPECT_NEAR(got[0].pressure_pa, expected, 0.1);
}

TEST(Baro, EmitsOnItsOwnClockWithMonotonicIndicesAndTickStamps) {
    BaroSensorSpawn s;
    s.rate_divider = 3u;  // h = 1 ms substeps: a sample every 3 substeps; substeps 3, 6, 9, 12 fall in steps 1, 2, 4, 5
    std::optional<Rig> rig = make_rig(s, 0.0f);
    ASSERT_TRUE(rig.has_value());
    const std::vector<BaroSample> got = run(*rig, 6);
    ASSERT_EQ(got.size(), 4u);
    const uint64_t ticks[] = {1, 2, 4, 5};
    for (std::size_t i = 0; i < got.size(); ++i) {
        EXPECT_EQ(got[i].index, i + 1) << i;
        EXPECT_EQ(got[i].tick, ticks[i]) << i;
    }
}

TEST(Baro, WhiteNoiseHasItsSigmaAndTheMeanIsTheTruth) {
    BaroSensorSpawn s;
    s.sigma_p_pa = 20.0f;
    s.sigma_t_k = 0.5f;
    std::optional<Rig> rig = make_rig(s, 1000.0f);
    ASSERT_TRUE(rig.has_value());
    const std::vector<BaroSample> got = run(*rig, 2500);  // 5000 samples
    ASSERT_GE(got.size(), 4000u);
    double mean_p = 0.0, mean_t = 0.0;
    for (const BaroSample& g : got) {
        mean_p += g.pressure_pa;
        mean_t += g.temperature_c;
    }
    mean_p /= static_cast<double>(got.size());
    mean_t /= static_cast<double>(got.size());
    double var_p = 0.0, var_t = 0.0;
    for (const BaroSample& g : got) {
        var_p += (g.pressure_pa - mean_p) * (g.pressure_pa - mean_p);
        var_t += (g.temperature_c - mean_t) * (g.temperature_c - mean_t);
    }
    EXPECT_NEAR(std::sqrt(var_p / static_cast<double>(got.size())), 20.0, 1.0);
    EXPECT_NEAR(std::sqrt(var_t / static_cast<double>(got.size())), 0.5, 0.025);
    EXPECT_NEAR(mean_p, isa_pressure(1000.0), 1.5);
    EXPECT_NEAR(mean_t, isa_temperature(1000.0) - 273.15, 0.05);
}

TEST(Baro, TheSameSeedGivesTheSameBytesAndAnotherSeedDoesNot) {
    BaroSensorSpawn s;
    s.sigma_p_pa = 5.0f;
    s.sigma_bias_pa = 30.0f;
    s.bias_tau_s = 60.0f;
    std::optional<Rig> a = make_rig(s, 500.0f, 7u);
    std::optional<Rig> b = make_rig(s, 500.0f, 7u);
    std::optional<Rig> c = make_rig(s, 500.0f, 8u);
    ASSERT_TRUE(a && b && c);
    const std::vector<BaroSample> ga = run(*a, 50);
    const std::vector<BaroSample> gb = run(*b, 50);
    const std::vector<BaroSample> gc = run(*c, 50);
    ASSERT_EQ(ga.size(), gb.size());
    ASSERT_FALSE(ga.empty());
    EXPECT_EQ(std::memcmp(ga.data(), gb.data(), ga.size() * sizeof(BaroSample)), 0);
    EXPECT_NE(std::memcmp(ga.data(), gc.data(), ga.size() * sizeof(BaroSample)), 0);
}

TEST(Baro, TheTurnOnBiasIsAConstantOffsetPerSeedAndIsRedrawnAtEveryReseed) {
    BaroSensorSpawn s;
    s.sigma_bias_pa = 50.0f;
    s.bias_tau_s = 1.0e6f;  // a constant offset: the turn-on value, no drift
    std::optional<Rig> rig = make_rig(s, 1000.0f, 11u);
    ASSERT_TRUE(rig.has_value());
    const double truth = isa_pressure(1000.0);
    const std::vector<BaroSample> first = run(*rig, 3);
    ASSERT_GE(first.size(), 3u);
    const double offset = first[0].pressure_pa - truth;
    EXPECT_NE(offset, 0.0) << "a turn-on bias was drawn";
    for (const BaroSample& g : first) EXPECT_NEAR(g.pressure_pa - truth, offset, 0.02) << "constant over the run";

    ASSERT_OK(rig->sim.reseed(99u));
    const spade::sensors::SampleIndex since = first.back().index;
    ASSERT_OK(rig->sim.step(1));
    const std::vector<BaroSample> second = poll(rig->sim, rig->row, since);
    ASSERT_FALSE(second.empty());
    EXPECT_GT(std::abs((second[0].pressure_pa - truth) - offset), 1.0) << "another seed, another offset";

    // reseeding with the first seed's scene seed twice gives one offset (the draw is a function of the seed alone)
    ASSERT_OK(rig->sim.reseed(99u));
    const spade::sensors::SampleIndex since2 = second.back().index;
    ASSERT_OK(rig->sim.step(1));
    const std::vector<BaroSample> third = poll(rig->sim, rig->row, since2);
    ASSERT_FALSE(third.empty());
    EXPECT_NEAR(third[0].pressure_pa, second[0].pressure_pa, 0.02);
}

TEST(Baro, NoBiasMeansNoOffsetEvenWithATurnOnSigmaOfZero) {
    BaroSensorSpawn s;  // sigma_bias 0
    std::optional<Rig> rig = make_rig(s, 1000.0f);
    ASSERT_TRUE(rig.has_value());
    const std::vector<BaroSample> got = run(*rig, 2);
    ASSERT_FALSE(got.empty());
    for (const BaroSample& g : got) EXPECT_NEAR(g.pressure_pa, isa_pressure(1000.0), 0.1);
}

// --- refusals ----------------------------------------------------------------

TEST(BaroRefusals, EachBadFieldIsRefusedByName) {
    struct Case {
        const char* what;
        BaroSensorSpawn s;
    };
    std::vector<Case> cases;
    auto add = [&](const char* what, auto edit) {
        BaroSensorSpawn s;
        edit(s);
        cases.push_back({what, s});
    };
    const float nan = std::numeric_limits<float>::quiet_NaN();
    add("rate_divider 0", [](BaroSensorSpawn& s) { s.rate_divider = 0; });
    add("a NaN mount", [&](BaroSensorSpawn& s) { s.mount_pos.x = nan; });
    add("a NaN sigma_p", [&](BaroSensorSpawn& s) { s.sigma_p_pa = nan; });
    add("a negative sigma_p", [](BaroSensorSpawn& s) { s.sigma_p_pa = -1.0f; });
    add("a negative sigma_t", [](BaroSensorSpawn& s) { s.sigma_t_k = -1.0f; });
    add("a negative sigma_bias", [](BaroSensorSpawn& s) { s.sigma_bias_pa = -1.0f; });
    add("a NaN tau", [&](BaroSensorSpawn& s) { s.bias_tau_s = nan; });
    add("a zero sea-level pressure", [](BaroSensorSpawn& s) { s.sea_level_pressure_pa = 0.0f; });
    add("a zero sea-level temperature", [](BaroSensorSpawn& s) { s.sea_level_temperature_k = 0.0f; });
    add("a zero lapse rate", [](BaroSensorSpawn& s) { s.lapse_k_per_m = 0.0f; });
    add("a NaN temp coefficient", [&](BaroSensorSpawn& s) { s.temp_coeff_pa_per_k = nan; });
    add("a mount height above the troposphere", [](BaroSensorSpawn& s) { s.world_origin_altitude_m = 12000.0f; });
    add("a mount height far below the datum", [](BaroSensorSpawn& s) { s.mount_pos.y = -2000.0f; });
    for (const Case& c : cases) {
        std::string why;
        const std::optional<Rig> rig = make_rig(c.s, 0.0f, 0x5EEDu, &why);
        EXPECT_FALSE(rig.has_value()) << c.what << " was accepted";
        EXPECT_FALSE(why.empty()) << c.what << " has no message";
    }
}

// --- state: snapshot and restore, despawn ------------------------------------

TEST(Baro, ARestoredSnapshotContinuesWithTheSameBytes) {
    BaroSensorSpawn s;
    s.sigma_p_pa = 5.0f;
    s.sigma_bias_pa = 30.0f;
    s.bias_tau_s = 60.0f;
    std::optional<Rig> rig = make_rig(s, 500.0f, 21u);
    ASSERT_TRUE(rig.has_value());
    (void)run(*rig, 10);
    const spade::Result<spade::SnapshotBlob> blob = rig->sim.snapshot();
    ASSERT_OK(blob);
    const spade::sensors::SampleIndex mark = poll(rig->sim, rig->row, 0).back().index;
    const std::vector<BaroSample> after = run(*rig, 10);
    ASSERT_OK(rig->sim.restore(*blob));
    std::vector<BaroSample> replay;
    spade::sensors::SampleIndex since = mark;
    for (int t = 0; t < 10; ++t) {
        ASSERT_OK(rig->sim.step(1));
        for (const BaroSample& g : poll(rig->sim, rig->row, since)) {
            replay.push_back(g);
            since = g.index;
        }
    }
    ASSERT_EQ(after.size(), replay.size());
    ASSERT_FALSE(after.empty());
    EXPECT_EQ(std::memcmp(after.data(), replay.data(), after.size() * sizeof(BaroSample)), 0);
}

TEST(Baro, DespawningTheBodyFreesTheSensorAndClearsItsRing) {
    BaroSensorSpawn s;
    std::optional<Rig> rig = make_rig(s, 500.0f);
    ASSERT_TRUE(rig.has_value());
    (void)run(*rig, 2);
    ASSERT_OK(rig->sim.despawn(rig->body));
    ASSERT_OK(rig->sim.step(0));
    const auto rows = rig->sim.module_rows<BaroSensorRow>("baro_sensors", 0);
    ASSERT_OK(rows);
    EXPECT_EQ((*rows)[rig->row.slot].kind, 0u) << "inert again";
    const auto ring = rig->sim.module_rows<BaroSample>("baro_ring", 0);
    ASSERT_OK(ring);
    for (uint32_t i = 0; i < spade::sensors::kRingDepth; ++i) {
        EXPECT_EQ((*ring)[std::size_t{rig->row.slot} * spade::sensors::kRingDepth + i].index, 0u);
    }
}

// --- what the module is not --------------------------------------------------

TEST(BaroModule, IsNotInTheStandardSetAndIsCpuOnly) {
    for (const spade::modules::ModuleDesc& m : spade::modules::standard_modules()) EXPECT_NE(m.name, "baro");
    const spade::modules::ModuleDesc m = spade::modules::baro_module();
    EXPECT_EQ(m.name, "baro");
    EXPECT_EQ(m.grade.cpu, spade::Grade::reference);
    EXPECT_EQ(m.grade.vulkan, spade::Grade::absent) << "a Vulkan world that holds the module is refused (L6)";
}

TEST(BaroModule, ASetWithoutItRefusesToAttachABarometer) {
    spade::Result<WorldSetDesc> set = still_world(1);
    ASSERT_OK(set);
    spade::Result<Simulation> sim = Simulation::create(*set, 2'000'000, 2);
    ASSERT_OK(sim);
    const spade::Result<BodyRef> body = sim->spawn(0, BodySpawn{});
    ASSERT_OK(body);
    const spade::Result<RowRef> row = sim->attach_row(*body, "baro_sensors", BaroSensorSpawn{});
    EXPECT_FALSE(row.has_value()) << "the array is not in the set";
}
