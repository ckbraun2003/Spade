// test_fc_host_boot.cpp -- the lockstep host's boot phase (tools/fc_host/lockstep.hpp): while a flight computer boots the world is HELD. boot_readings() reads
// the vehicle at its spawn state and steps nothing; its small noise comes from a stream of its own, a pure function of (BootConfig::seed, boot_step), so
// the length of the boot is invisible to the world and to the trace from tick 0 on (the 3-vs-300 test).
#include <gtest/gtest.h>

#include <cmath>

#include "fc_host/lockstep.hpp"
#include "fc_host_test_rig.hpp"
#include "testing/replay.hpp"

namespace {

using namespace fc_host_test;

constexpr float kG = 9.80665f;

[[nodiscard]] BootConfig no_noise() {
    BootConfig c;
    c.sigma_accel = 0.0f;
    c.sigma_gyro = 0.0f;
    c.sigma_pressure_pa = 0.0f;
    c.sigma_mag_ut = 0.0f;
    return c;
}

[[nodiscard]] uint32_t rd_u32(const std::vector<std::byte>& b, std::size_t at) {
    uint32_t v = 0;
    for (int i = 3; i >= 0; --i) v = (v << 8) | static_cast<uint32_t>(std::to_integer<uint8_t>(b[at + static_cast<std::size_t>(i)]));
    return v;
}
[[nodiscard]] uint64_t rd_u64(const std::vector<std::byte>& b, std::size_t at) { return rd_u32(b, at) | (static_cast<uint64_t>(rd_u32(b, at + 4)) << 32); }

TEST(FcHostBoot, TheHeldVehicleReadsGravityOnTheAccelerometerAndNoRotation) {
    std::optional<Rig> rig = make_rig(8, 40, 20, kDt, 1, -kG);
    ASSERT_TRUE(rig.has_value());
    const std::vector<SensorBinding> sensors = all_bindings(*rig);
    spade::Result<LockstepHost> host = LockstepHost::open(rig->sim, rig->vehicle, sensors);
    ASSERT_OK(host);
    const spade::Result<BootReadings> r = host->boot_readings(0, no_noise());
    ASSERT_OK(r);
    EXPECT_EQ(r->boot_step, 0u);
    EXPECT_NEAR(r->accel_body.x, 0.0f, 1e-5f);
    EXPECT_NEAR(r->accel_body.y, kG, 1e-4f) << "a level body at rest: specific force is +g up, in Spade's y-up world";
    EXPECT_NEAR(r->accel_body.z, 0.0f, 1e-5f);
    EXPECT_EQ(r->gyro_body, glm::vec3(0.0f));
    EXPECT_TRUE(r->has_baro);
    EXPECT_NEAR(r->pressure_pa, 101325.0f * std::pow(1.0f - 0.0065f * 10.0f / 288.15f, 5.2559f), 1.0f) << "the ISA pressure at 10 m, about 101205 Pa";
    EXPECT_TRUE(r->has_mag);
    EXPECT_NEAR(r->mag_body_ut.x, 20.0f, 1e-3f) << "level, so the body axes are the world axes";
    EXPECT_NEAR(r->mag_body_ut.y, -40.0f, 1e-3f);
    EXPECT_NEAR(r->mag_body_ut.z, 5.0f, 1e-3f);
    EXPECT_EQ(r->position, glm::vec3(0.0f, 10.0f, 0.0f));
    EXPECT_EQ(r->velocity, glm::vec3(0.0f));
}

TEST(FcHostBoot, AHostWithoutABaroOrMagBindingSaysSo) {
    std::optional<Rig> rig = make_rig(8, 40, 20, kDt, 1, -kG);
    ASSERT_TRUE(rig.has_value());
    const std::vector<SensorBinding> sensors = {SensorBinding::of(rig->vehicle.imu_sensors[0])};
    spade::Result<LockstepHost> host = LockstepHost::open(rig->sim, rig->vehicle, sensors);
    ASSERT_OK(host);
    const spade::Result<BootReadings> r = host->boot_readings(0, no_noise());
    ASSERT_OK(r);
    EXPECT_FALSE(r->has_baro);
    EXPECT_FALSE(r->has_mag);
}

TEST(FcHostBoot, ReadingTheBootStepsDoesNotMoveTheWorld) {
    std::optional<Rig> rig = make_rig(8, 40, 20, kDt, 1, -kG);
    ASSERT_TRUE(rig.has_value());
    const std::vector<SensorBinding> sensors = all_bindings(*rig);
    spade::Result<LockstepHost> host = LockstepHost::open(rig->sim, rig->vehicle, sensors);
    ASSERT_OK(host);
    const uint64_t before = spade::testing::state_digest(rig->sim);
    for (uint64_t k = 0; k < 50; ++k) ASSERT_OK(host->boot_readings(k));
    EXPECT_EQ(host->tick(), 0u);
    EXPECT_EQ(host->sim_time_ns(), 0u);
    EXPECT_EQ(spade::testing::state_digest(rig->sim), before);
    EXPECT_EQ(host->trace_ticks(), 0u);
    ASSERT_OK(host->take_sensors());
}

TEST(FcHostBoot, TheNoiseIsAPureFunctionOfTheSeedAndTheBootStep) {
    std::optional<Rig> rig = make_rig(8, 40, 20, kDt, 1, -kG);
    ASSERT_TRUE(rig.has_value());
    const std::vector<SensorBinding> sensors = all_bindings(*rig);
    spade::Result<LockstepHost> host = LockstepHost::open(rig->sim, rig->vehicle, sensors);
    ASSERT_OK(host);
    const BootConfig config;
    const spade::Result<BootReadings> a = host->boot_readings(7, config);
    ASSERT_OK(a);
    // asking again, or in another order, gives the same numbers: nothing is consumed
    ASSERT_OK(host->boot_readings(3, config));
    const spade::Result<BootReadings> again = host->boot_readings(7, config);
    ASSERT_OK(again);
    EXPECT_EQ(a->accel_body, again->accel_body);
    EXPECT_EQ(a->gyro_body, again->gyro_body);
    EXPECT_EQ(a->pressure_pa, again->pressure_pa);
    EXPECT_EQ(a->mag_body_ut, again->mag_body_ut);
    // another step differs, another seed differs, and the noise is small (inside 8 sigma) and not nothing
    const spade::Result<BootReadings> next = host->boot_readings(8, config);
    ASSERT_OK(next);
    EXPECT_NE(a->accel_body, next->accel_body);
    BootConfig other = config;
    other.seed ^= 1;
    const spade::Result<BootReadings> reseeded = host->boot_readings(7, other);
    ASSERT_OK(reseeded);
    EXPECT_NE(a->accel_body, reseeded->accel_body);
    const spade::Result<BootReadings> exact = host->boot_readings(7, no_noise());
    ASSERT_OK(exact);
    EXPECT_NE(a->accel_body, exact->accel_body);
    EXPECT_LT(glm::length(a->accel_body - exact->accel_body), 8.0f * config.sigma_accel * 1.8f);
    EXPECT_LT(glm::length(a->gyro_body), 8.0f * config.sigma_gyro * 1.8f);
    EXPECT_LT(std::fabs(a->pressure_pa - exact->pressure_pa), 8.0f * config.sigma_pressure_pa);
    EXPECT_EQ(exact->gyro_body, glm::vec3(0.0f)) << "sigma 0 is exactly the held state";
}

TEST(FcHostBoot, ABootOfThreeStepsAndABootOfThreeHundredLeaveTheSameWorld) {
    std::string hash[2];
    uint64_t digest[2];
    std::vector<std::byte> file[2];
    const uint64_t boots[2] = {3, 300};
    for (int run = 0; run < 2; ++run) {
        std::optional<Rig> rig = make_rig(8, 40, 20, kDt, 1, -kG);
        ASSERT_TRUE(rig.has_value());
        const std::vector<SensorBinding> sensors = all_bindings(*rig);
        spade::Result<LockstepHost> host = LockstepHost::open(rig->sim, rig->vehicle, sensors);
        ASSERT_OK(host);
        for (uint64_t k = 0; k < boots[run]; ++k) ASSERT_OK(host->boot_readings(k));
        ASSERT_OK(host->note_boot(boots[run], boots[run] * 4000, BootConfig{}));
        const float cmd[4] = {500.0f, 500.0f, 500.0f, 500.0f};
        ASSERT_OK(host->command(cmd));
        ASSERT_OK(host->advance_to(40 * kDt));
        hash[run] = host->trace_hash();
        digest[run] = spade::testing::state_digest(rig->sim);
        const spade::Result<std::vector<std::byte>> f = host->trace_bytes();
        ASSERT_OK(f);
        file[run] = *f;
        ASSERT_OK(host->take_sensors());
    }
    EXPECT_EQ(hash[0], hash[1]);
    EXPECT_EQ(digest[0], digest[1]);
    ASSERT_EQ(file[0].size(), file[1].size());
    // the files differ in the boot fields of the header and in the trailer that covers them, and nowhere else
    std::size_t differing = 0;
    for (std::size_t i = 0; i < file[0].size(); ++i) {
        if (file[0][i] == file[1][i]) continue;
        ++differing;
        const bool in_boot_fields = i >= 160 && i < 184;
        const bool in_trailer = i >= file[0].size() - 64;
        EXPECT_TRUE(in_boot_fields || in_trailer) << "byte " << i << " differs";
    }
    EXPECT_GT(differing, 0u);
}

TEST(FcHostBoot, NoteBootWritesTheBootFieldsInTheTraceHeader) {
    std::optional<Rig> rig = make_rig(8, 40, 20, kDt, 1, -kG);
    ASSERT_TRUE(rig.has_value());
    const std::vector<SensorBinding> sensors = all_bindings(*rig);
    spade::Result<LockstepHost> host = LockstepHost::open(rig->sim, rig->vehicle, sensors);
    ASSERT_OK(host);
    BootConfig config;
    config.seed = 0x1122334455667788ULL;
    ASSERT_OK(host->note_boot(42, 168'000, config));
    ASSERT_OK(host->advance_to(8 * kDt));
    const spade::Result<std::vector<std::byte>> f = host->trace_bytes();
    ASSERT_OK(f);
    EXPECT_EQ(rd_u64(*f, 160), 42u);
    EXPECT_EQ(rd_u64(*f, 168), 168'000u);
    EXPECT_EQ(rd_u64(*f, 176), 0x1122334455667788ULL);
    for (std::size_t i = 184; i < 192; ++i) EXPECT_EQ(std::to_integer<uint8_t>((*f)[i]), 0u) << "header byte " << i << " stays reserved zero";
}

TEST(FcHostBoot, TheWorldIsHeldOnlyBeforeTickZero) {
    std::optional<Rig> rig = make_rig(8, 40, 20, kDt, 1, -kG);
    ASSERT_TRUE(rig.has_value());
    const std::vector<SensorBinding> sensors = all_bindings(*rig);
    spade::Result<LockstepHost> host = LockstepHost::open(rig->sim, rig->vehicle, sensors);
    ASSERT_OK(host);
    ASSERT_OK(host->advance_to(kDt));
    const spade::Result<BootReadings> r = host->boot_readings(0);
    ASSERT_FALSE(r.has_value());
    EXPECT_EQ(r.error().code, spade::Code::invalid_argument);
    EXPECT_NE(r.error().context.find("tick"), std::string::npos) << r.error().context;
    const spade::Result<void> n = host->note_boot(3, 12'000, BootConfig{});
    ASSERT_FALSE(n.has_value());
    EXPECT_EQ(n.error().code, spade::Code::invalid_argument);
}

TEST(FcHostBoot, ANonFiniteOrNegativeSigmaIsRefused) {
    std::optional<Rig> rig = make_rig(8, 40, 20, kDt, 1, -kG);
    ASSERT_TRUE(rig.has_value());
    const std::vector<SensorBinding> sensors = all_bindings(*rig);
    spade::Result<LockstepHost> host = LockstepHost::open(rig->sim, rig->vehicle, sensors);
    ASSERT_OK(host);
    BootConfig bad;
    bad.sigma_gyro = -1.0f;
    const spade::Result<BootReadings> r = host->boot_readings(0, bad);
    ASSERT_FALSE(r.has_value());
    EXPECT_NE(r.error().context.find("sigma"), std::string::npos) << r.error().context;
    bad = BootConfig{};
    bad.sigma_accel = std::numeric_limits<float>::quiet_NaN();
    EXPECT_FALSE(host->note_boot(1, 4000, bad).has_value());
}

}  // namespace
