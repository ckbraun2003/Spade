// ---------------------------------------------------------------------------
// test_fc_link_exchange.cpp -- O140 phase 1: the seam between Core's LockstepHost and an autopilot link (fc_link/autopilot_link.hpp). A real
// LockstepHost over the fc_host test rig, a scripted transport in place of PX4.
//
//   FcLinkExchange.OnePeriodSendsTheHostsImuSampleInFrdAndTheRepliesOutputsDriveTheRotors   one conversion site: HIL_SENSOR = the host's IMU sample in the
//                                                                                          autopilot's frame; the four outputs, through the chain, are the
//                                                                                          rotor command the host logs
//   FcLinkExchange.TheBootHoldsTheWorldAndTheHostIsToldHowLongItWas                         booting replies: the host's tick does not move, boot_readings go out;
//                                                                                          note_boot gets the steps and the end time; the clock then runs from there
//   FcLinkExchange.ABootOfThreeStepsAndOneOfThirtyGiveTheSameTrace                          the trace_hash does not depend on the boot's length
//   FcLinkExchange.ATransportFailureIsReturnedAndNothingIsCommanded                         a refused exchange is the caller's error; the host takes no command
// ---------------------------------------------------------------------------

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <cstring>
#include <memory>
#include <string>
#include <cstdint>
#include <filesystem>
#include <optional>
#include <vector>

#include "fc_host_test_rig.hpp"
#include "fc_link/autopilot_link.hpp"
#include "fc_link/autopilot_map.hpp"
#include "scene/model_file.hpp"
#include "sensors/gnss.hpp"

namespace {

using namespace fc_host_test;
namespace fcl = spade::fc_link;
namespace mav = spade::fc_link::mavlink;

constexpr uint32_t kImuDivider = 8;  // 8 ticks of 500 us: a 4 ms period

struct Seen {
    std::vector<mav::HilSensor> sensors;
};

[[nodiscard]] spade::scene::ModelFile x500() {
    const auto m = spade::scene::load_model_file(std::filesystem::path(SPADE_REPO_DIR) / "tools/flow_movie/data/x500/x500.model.yaml");
    EXPECT_TRUE(m.has_value());
    return *m;
}

// PX4's outputs for the four motors in its order, not booting.
[[nodiscard]] fcl::ActuatorReply reply_of(std::array<float, 4> motors, bool booting = false) {
    fcl::ActuatorReply r;
    for (std::size_t i = 0; i < 4; ++i) r.controls[i] = booting ? 0.0f : motors[i];
    r.armed = !booting;
    r.lockstep = true;
    r.booting = booting;
    return r;
}

TEST(FcLinkExchange, OnePeriodSendsTheHostsImuSampleInFrdAndTheRepliesOutputsDriveTheRotors) {
    const spade::scene::ModelFile model = x500();
    ASSERT_TRUE(model.model.propulsion.has_value());
    std::optional<Rig> a = make_rig(kImuDivider, 40, 20, kDt, 1, -9.81f);
    std::optional<Rig> b = make_rig(kImuDivider, 40, 20, kDt, 1, -9.81f);
    ASSERT_TRUE(a.has_value() && b.has_value());
    const std::vector<SensorBinding> imu_a{SensorBinding::of(a->vehicle.imu_sensors[0])};
    const std::vector<SensorBinding> imu_b{SensorBinding::of(b->vehicle.imu_sensors[0])};
    spade::Result<LockstepHost> host_a = LockstepHost::open(a->sim, a->vehicle, imu_a);
    spade::Result<LockstepHost> host_b = LockstepHost::open(b->sim, b->vehicle, imu_b);
    ASSERT_TRUE(host_a.has_value() && host_b.has_value());

    const std::array<float, 4> motors{0.2f, 0.4f, 0.6f, 0.8f};
    auto seen = std::make_shared<Seen>();
    fcl::ExchangeConfig config;
    config.chain = &*model.model.propulsion;
    auto link = fcl::AutopilotLink::create(
        [seen, motors](const mav::HilSensor& s, const std::optional<mav::HilGps>&) -> spade::Result<fcl::ActuatorReply> {
            seen->sensors.push_back(s);
            return reply_of(motors);
        },
        config);
    ASSERT_TRUE(link.has_value());

    // The reference: the same host stepped by hand, converted by the library's own functions.
    const std::array<double, 4> duties{0.2, 0.4, 0.6, 0.8};
    const auto omega = fcl::autopilot_duties_to_rotor_commands(*model.model.propulsion, duties, config.density, config.soc);
    ASSERT_TRUE(omega.has_value());
    ASSERT_TRUE(host_b->command(*omega).has_value());
    ASSERT_TRUE(host_b->advance_to(kImuDivider * kDt).has_value());
    const auto taken = host_b->take_sensors();
    ASSERT_TRUE(taken.has_value());
    ASSERT_EQ(taken->size(), 1u);
    spade::sensors::ImuSample imu{};
    std::memcpy(&imu, (*taken)[0].bytes.data(), sizeof(imu));
    const glm::dvec3 accel = spade::fc_link::frames::body_to_frd(config.frame, glm::dvec3(imu.accel));
    const glm::dvec3 gyro = spade::fc_link::frames::body_to_frd(config.frame, glm::dvec3(imu.gyro));

    // Exchange 1 is the autopilot's first message, a boot message: its reply is not `booting`, so boot ends at once (zero steps) and the reply's
    // outputs are the command before tick 0. Exchange 2 runs one IMU period of the world.
    const spade::Result<fcl::ExchangeResult> first = link->exchange(*host_a);
    ASSERT_TRUE(first.has_value()) << first.error().context;
    EXPECT_FALSE(first->booting);
    EXPECT_EQ(host_a->tick(), 0u);
    ASSERT_TRUE(host_a->boot_note().has_value());
    EXPECT_EQ(host_a->boot_note()->steps, 0u);
    EXPECT_EQ(host_a->boot_note()->end_time_usec, kImuDivider * kDt / 1000u);
    const spade::Result<fcl::ExchangeResult> done = link->exchange(*host_a);
    ASSERT_TRUE(done.has_value()) << done.error().context;
    EXPECT_FALSE(done->booting);
    EXPECT_EQ(done->tick, kImuDivider);
    EXPECT_EQ(host_a->tick(), kImuDivider);
    ASSERT_EQ(seen->sensors.size(), 2u);
    const mav::HilSensor& m = seen->sensors[1];
    EXPECT_EQ(m.time_usec, 2u * kImuDivider * kDt / 1000u) << "the boot offset plus the host's sim time, in microseconds";
    EXPECT_FLOAT_EQ(m.xacc, static_cast<float>(accel.x));
    EXPECT_FLOAT_EQ(m.yacc, static_cast<float>(accel.y));
    EXPECT_FLOAT_EQ(m.zacc, static_cast<float>(accel.z));
    EXPECT_FLOAT_EQ(m.xgyro, static_cast<float>(gyro.x));
    EXPECT_FLOAT_EQ(m.ygyro, static_cast<float>(gyro.y));
    EXPECT_FLOAT_EQ(m.zgyro, static_cast<float>(gyro.z));
    EXPECT_EQ(m.fields_updated, 0x003Fu) << "accel and gyro only: the IMU is all this slice sends";
    EXPECT_EQ(host_a->commands_bytes(), host_b->commands_bytes()) << "the reply's outputs, through the chain, are the command the host logs";
}

TEST(FcLinkExchange, TheBootHoldsTheWorldAndTheHostIsToldHowLongItWas) {
    const spade::scene::ModelFile model = x500();
    ASSERT_TRUE(model.model.propulsion.has_value());
    std::optional<Rig> rig = make_rig(kImuDivider, 40, 20, kDt, 1, -9.81f);
    ASSERT_TRUE(rig.has_value());
    const std::vector<SensorBinding> imu{SensorBinding::of(rig->vehicle.imu_sensors[0])};
    spade::Result<LockstepHost> host = LockstepHost::open(rig->sim, rig->vehicle, imu);
    ASSERT_TRUE(host.has_value());

    auto seen = std::make_shared<Seen>();
    auto calls = std::make_shared<int>(0);
    fcl::ExchangeConfig config;
    config.chain = &*model.model.propulsion;
    auto link = fcl::AutopilotLink::create(
        [seen, calls](const mav::HilSensor& s, const std::optional<mav::HilGps>&) -> spade::Result<fcl::ActuatorReply> {
            seen->sensors.push_back(s);
            ++*calls;
            return *calls <= 3 ? reply_of({0, 0, 0, 0}, true) : reply_of({0.5f, 0.5f, 0.5f, 0.5f});
        },
        config);
    ASSERT_TRUE(link.has_value());

    for (int i = 1; i <= 3; ++i) {
        const auto r = link->exchange(*host);
        ASSERT_TRUE(r.has_value()) << r.error().context;
        EXPECT_TRUE(r->booting) << "call " << i;
        EXPECT_EQ(host->tick(), 0u) << "the world is held while PX4 boots";
        EXPECT_EQ(seen->sensors.back().time_usec, static_cast<uint64_t>(i) * 4000u);
    }
    const auto fourth = link->exchange(*host);  // PX4 answers this one: boot is over
    ASSERT_TRUE(fourth.has_value()) << fourth.error().context;
    EXPECT_FALSE(fourth->booting);
    EXPECT_EQ(host->tick(), 0u) << "the exchange that ends boot still sent a boot message: the world has not moved";
    ASSERT_TRUE(host->boot_note().has_value());
    EXPECT_EQ(host->boot_note()->steps, 3u);
    EXPECT_EQ(host->boot_note()->end_time_usec, 16000u);

    const auto fifth = link->exchange(*host);
    ASSERT_TRUE(fifth.has_value()) << fifth.error().context;
    EXPECT_EQ(host->tick(), kImuDivider) << "now the world runs one IMU period";
    EXPECT_EQ(seen->sensors.back().time_usec, 16000u + kImuDivider * kDt / 1000u) << "the autopilot's clock continues from the end of boot";
}

// The boot's noise stream is the caller's: ExchangeConfig::boot reaches both the boot readings and the note the host keeps of the boot (its config is in the run
// record). With the accelerometer's sigma at zero the held boot messages repeat their accelerometer and gyro readings exactly; the note carries the sigmas given.
TEST(FcLinkExchange, TheBootMessagesNoiseAndTheBootNoteComeFromTheConfigsBootConfig) {
    const spade::scene::ModelFile model = x500();
    ASSERT_TRUE(model.model.propulsion.has_value());
    std::optional<Rig> rig = make_rig(kImuDivider, 40, 20, kDt, 1, -9.81f);
    ASSERT_TRUE(rig.has_value());
    const std::vector<SensorBinding> imu{SensorBinding::of(rig->vehicle.imu_sensors[0])};
    spade::Result<LockstepHost> host = LockstepHost::open(rig->sim, rig->vehicle, imu);
    ASSERT_TRUE(host.has_value());

    auto seen = std::make_shared<Seen>();
    auto calls = std::make_shared<int>(0);
    fcl::ExchangeConfig config;
    config.chain = &*model.model.propulsion;
    config.boot.sigma_accel = 0.0f;
    config.boot.sigma_gyro = 0.0f;
    config.boot.sigma_pressure_pa = 2.5f;
    config.boot.sigma_mag_ut = 0.7f;
    auto link = fcl::AutopilotLink::create(
        [seen, calls](const mav::HilSensor& s, const std::optional<mav::HilGps>&) -> spade::Result<fcl::ActuatorReply> {
            seen->sensors.push_back(s);
            ++*calls;
            return *calls <= 3 ? reply_of({0, 0, 0, 0}, true) : reply_of({0.5f, 0.5f, 0.5f, 0.5f});
        },
        config);
    ASSERT_TRUE(link.has_value());
    for (int i = 0; i < 4; ++i) ASSERT_TRUE(link->exchange(*host).has_value());  // three booting replies, then the one that ends the boot

    ASSERT_GE(seen->sensors.size(), 4u);
    for (std::size_t i = 1; i < 4; ++i) {
        EXPECT_EQ(seen->sensors[i].xacc, seen->sensors[0].xacc) << "boot message " << i << ": no accelerometer noise was asked for";
        EXPECT_EQ(seen->sensors[i].zacc, seen->sensors[0].zacc);
        EXPECT_EQ(seen->sensors[i].xgyro, seen->sensors[0].xgyro);
    }
    ASSERT_TRUE(host->boot_note().has_value());
    EXPECT_EQ(host->boot_note()->config.sigma_pressure_pa, 2.5f);
    EXPECT_EQ(host->boot_note()->config.sigma_mag_ut, 0.7f);
    EXPECT_EQ(host->boot_note()->config.sigma_accel, 0.0f);
}

[[nodiscard]] std::string trace_after_boot_of(int boot_steps) {
    const spade::scene::ModelFile model = x500();
    std::optional<Rig> rig = make_rig(kImuDivider, 40, 20, kDt, 1, -9.81f);
    if (!rig || !model.model.propulsion) return "setup failed";
    const std::vector<SensorBinding> imu{SensorBinding::of(rig->vehicle.imu_sensors[0])};
    spade::Result<LockstepHost> host = LockstepHost::open(rig->sim, rig->vehicle, imu);
    if (!host) return "open failed";
    auto calls = std::make_shared<int>(0);
    fcl::ExchangeConfig config;
    config.chain = &*model.model.propulsion;
    auto link = fcl::AutopilotLink::create(
        [calls, boot_steps](const mav::HilSensor&, const std::optional<mav::HilGps>&) -> spade::Result<fcl::ActuatorReply> {
            ++*calls;
            return *calls <= boot_steps ? reply_of({0, 0, 0, 0}, true) : reply_of({0.5f, 0.5f, 0.5f, 0.5f});
        },
        config);
    if (!link) return "create failed";
    for (int i = 0; i < boot_steps + 1 + 10; ++i) {  // the boot, the exchange that ends it, then ten real periods
        if (!link->exchange(*host)) return "exchange failed";
    }
    return host->trace_hash();
}

TEST(FcLinkExchange, ABootOfThreeStepsAndOneOfThirtyGiveTheSameTrace) {
    const std::string three = trace_after_boot_of(3);
    const std::string thirty = trace_after_boot_of(30);
    EXPECT_EQ(three.size(), 64u) << three;
    EXPECT_EQ(three, thirty) << "boot must not leak into the run";
}

TEST(FcLinkExchange, ATransportFailureIsReturnedAndNothingIsCommanded) {
    const spade::scene::ModelFile model = x500();
    std::optional<Rig> rig = make_rig(kImuDivider, 40, 20, kDt, 1, -9.81f);
    ASSERT_TRUE(rig.has_value() && model.model.propulsion.has_value());
    const std::vector<SensorBinding> imu{SensorBinding::of(rig->vehicle.imu_sensors[0])};
    spade::Result<LockstepHost> host = LockstepHost::open(rig->sim, rig->vehicle, imu);
    ASSERT_TRUE(host.has_value());
    fcl::ExchangeConfig config;
    config.chain = &*model.model.propulsion;
    auto link = fcl::AutopilotLink::create(
        [](const mav::HilSensor&, const std::optional<mav::HilGps>&) -> spade::Result<fcl::ActuatorReply> {
            return std::unexpected(spade::Error{spade::Code::io_error, "px4_no_reply: test"});
        },
        config);
    ASSERT_TRUE(link.has_value());
    const auto r = link->exchange(*host);
    ASSERT_FALSE(r.has_value());
    EXPECT_NE(r.error().context.find("px4_no_reply"), std::string::npos) << r.error().context << " (the transport's refusal comes back as it was)";
    EXPECT_TRUE(host->commands_bytes().size() <= 24u) << "no command was logged (an empty commands.bin is its header only)";
}

// Slice 2: the barometer and magnetometer groups of HIL_SENSOR. A group is marked in fields_updated only when its sample is NEW in this period
// (PX4 treats an unmarked group as not updated), and a boot message carries the held state's pressure and field. The rig's dividers: IMU 8, mag 20,
// baro 40 ticks, so over five 8-tick periods the mag sample is due in period 3 (tick 19, taken after tick 24) and both in period 5 (tick 39).
constexpr uint32_t kImuBits = 0x003Fu;
constexpr uint32_t kMagBits = 0x01C0u;   // xmag, ymag, zmag
constexpr uint32_t kBaroBits = 0x1A00u;  // abs_pressure, pressure_alt, temperature (no differential pressure)

TEST(FcLinkExchange, BaroAndMagAreMarkedWhenTheirSampleIsNewAndConvertedToHilUnits) {
    const spade::scene::ModelFile model = x500();
    ASSERT_TRUE(model.model.propulsion.has_value());
    std::optional<Rig> a = make_rig(kImuDivider, 40, 20, kDt, 1, -9.81f);
    std::optional<Rig> b = make_rig(kImuDivider, 40, 20, kDt, 1, -9.81f);
    ASSERT_TRUE(a.has_value() && b.has_value());
    const std::vector<SensorBinding> sensors_a = all_bindings(*a);  // IMU 0, barometer 1, magnetometer 2
    const std::vector<SensorBinding> sensors_b = all_bindings(*b);
    spade::Result<LockstepHost> host_a = LockstepHost::open(a->sim, a->vehicle, sensors_a);
    spade::Result<LockstepHost> host_b = LockstepHost::open(b->sim, b->vehicle, sensors_b);
    ASSERT_TRUE(host_a.has_value() && host_b.has_value());

    auto seen = std::make_shared<Seen>();
    fcl::ExchangeConfig config;
    config.chain = &*model.model.propulsion;
    config.baro_binding = 1;
    config.mag_binding = 2;
    auto link = fcl::AutopilotLink::create(
        [seen](const mav::HilSensor& s, const std::optional<mav::HilGps>&) -> spade::Result<fcl::ActuatorReply> {
            seen->sensors.push_back(s);
            return reply_of({0.5f, 0.5f, 0.5f, 0.5f});
        },
        config);
    ASSERT_TRUE(link.has_value());

    ASSERT_TRUE(link->exchange(*host_a).has_value());  // the boot message (zero boot steps)
    const std::array<double, 4> duties{0.5, 0.5, 0.5, 0.5};  // what every reply asks, so the reference host flies the same
    const auto omega = fcl::autopilot_duties_to_rotor_commands(*model.model.propulsion, duties, config.density, config.soc);
    ASSERT_TRUE(omega.has_value());
    ASSERT_TRUE(host_b->command(*omega).has_value());
    std::vector<spade::fc_host::Sample> baro_b, mag_b;
    std::vector<uint32_t> expected_bits;
    for (int period = 1; period <= 5; ++period) {
        ASSERT_TRUE(link->exchange(*host_a).has_value());
        ASSERT_TRUE(host_b->advance_to(static_cast<uint64_t>(period) * kImuDivider * kDt).has_value());
        const auto taken = host_b->take_sensors();
        ASSERT_TRUE(taken.has_value());
        uint32_t bits = kImuBits;
        for (const spade::fc_host::Sample& s : *taken) {
            if (s.kind == spade::fc_host::SensorKind::baro) { baro_b.push_back(s); bits |= kBaroBits; }
            if (s.kind == spade::fc_host::SensorKind::mag) { mag_b.push_back(s); bits |= kMagBits; }
        }
        expected_bits.push_back(bits);
    }
    ASSERT_EQ(seen->sensors.size(), 6u);  // the boot message and five periods
    for (std::size_t i = 0; i < 5; ++i) {
        EXPECT_EQ(seen->sensors[i + 1].fields_updated, expected_bits[i]) << "period " << (i + 1);
    }
    EXPECT_EQ(expected_bits[2], kImuBits | kMagBits) << "the rig's mag sample is due in period 3";
    EXPECT_EQ(expected_bits[4], kImuBits | kMagBits | kBaroBits) << "and both in period 5";
    ASSERT_EQ(baro_b.size(), 1u);
    ASSERT_EQ(mag_b.size(), 2u);
    spade::sensors::BaroSample baro{};
    std::memcpy(&baro, baro_b[0].bytes.data(), sizeof(baro));
    spade::sensors::MagSample mag{};
    std::memcpy(&mag, mag_b[1].bytes.data(), sizeof(mag));
    const mav::HilSensor& last = seen->sensors[5];
    EXPECT_FLOAT_EQ(last.abs_pressure, baro.pressure_pa / 100.0f) << "hPa";
    EXPECT_FLOAT_EQ(last.temperature, baro.temperature_c);
    EXPECT_FLOAT_EQ(last.pressure_alt, baro.pressure_alt_m);
    const glm::dvec3 field = spade::fc_link::frames::body_to_frd(config.frame, glm::dvec3(mag.field_body_ut)) / 100.0;  // uT to gauss
    EXPECT_FLOAT_EQ(last.xmag, static_cast<float>(field.x));
    EXPECT_FLOAT_EQ(last.ymag, static_cast<float>(field.y));
    EXPECT_FLOAT_EQ(last.zmag, static_cast<float>(field.z));
}

TEST(FcLinkExchange, ABootMessageCarriesTheHeldPressureAndField) {
    const spade::scene::ModelFile model = x500();
    ASSERT_TRUE(model.model.propulsion.has_value());
    std::optional<Rig> rig = make_rig(kImuDivider, 40, 20, kDt, 1, -9.81f);
    ASSERT_TRUE(rig.has_value());
    const std::vector<SensorBinding> sensors = all_bindings(*rig);
    spade::Result<LockstepHost> host = LockstepHost::open(rig->sim, rig->vehicle, sensors);
    ASSERT_TRUE(host.has_value());
    const auto held = host->boot_readings(1);
    ASSERT_TRUE(held.has_value());
    ASSERT_TRUE(held->has_baro && held->has_mag);

    auto seen = std::make_shared<Seen>();
    fcl::ExchangeConfig config;
    config.chain = &*model.model.propulsion;
    config.baro_binding = 1;
    config.mag_binding = 2;
    auto link = fcl::AutopilotLink::create(
        [seen](const mav::HilSensor& s, const std::optional<mav::HilGps>&) -> spade::Result<fcl::ActuatorReply> {
            seen->sensors.push_back(s);
            return reply_of({0, 0, 0, 0}, true);
        },
        config);
    ASSERT_TRUE(link.has_value());
    ASSERT_TRUE(link->exchange(*host).has_value());
    ASSERT_EQ(seen->sensors.size(), 1u);
    const mav::HilSensor& m = seen->sensors[0];
    EXPECT_EQ(m.fields_updated, kImuBits | kMagBits | kBaroBits) << "every group is fresh in a boot message";
    EXPECT_FLOAT_EQ(m.abs_pressure, held->pressure_pa / 100.0f);
    EXPECT_FLOAT_EQ(m.temperature, held->temperature_c);
    const glm::dvec3 field = spade::fc_link::frames::body_to_frd(config.frame, glm::dvec3(held->mag_body_ut)) / 100.0;
    EXPECT_FLOAT_EQ(m.xmag, static_cast<float>(field.x));
    EXPECT_FLOAT_EQ(m.ymag, static_cast<float>(field.y));
    EXPECT_FLOAT_EQ(m.zmag, static_cast<float>(field.z));
}

// Slice 3: HIL_GPS. A GNSS fix is sent in the exchange in which it is NEW (every other exchange passes none); its position goes world -> NED about
// the config's home point -> geodetic through the config's local frame (frames.hpp); the velocity goes to NED in cm/s with the ground speed and the
// course. A boot message carries the held position as a still fix, every 200 ms of link time.
struct GpsRig {
    std::optional<Rig> rig;
    spade::GnssSensorRef gnss;
};

[[nodiscard]] std::optional<GpsRig> make_gps_rig() {
    GpsRig out;
    out.rig = make_rig(kImuDivider, 40, 20, kDt, 1, -9.81f);
    if (!out.rig) return std::nullopt;
    spade::GnssSensorSpawn g;
    g.rate_divider = 40;  // zero noise: the fix is the truth, so the test can compute it
    const spade::Result<spade::GnssSensorRef> ref = out.rig->sim.add_gnss_sensor(out.rig->vehicle.body, g);
    if (!ref || !out.rig->sim.flush_structural()) return std::nullopt;
    out.gnss = *ref;
    return out;
}

[[nodiscard]] std::vector<SensorBinding> gps_bindings(const GpsRig& r) {
    return {SensorBinding::of(r.rig->vehicle.imu_sensors[0]), SensorBinding::of(r.gnss)};
}

[[nodiscard]] mav::HilGps expected_gps(const fcl::ExchangeConfig& config, const glm::dvec3& position_world, const glm::dvec3& velocity_world,
                                       uint64_t time_usec) {
    const glm::dvec3 ned = fcl::frames::position_to_ned(config.frame, position_world, config.home_world);
    const spade::geo::Geodetic g = fcl::frames::ned_to_geodetic(*config.geo_frame, ned);
    const glm::dvec3 v = fcl::frames::world_to_ned(config.frame, velocity_world);
    mav::HilGps out;
    out.time_usec = time_usec;
    out.lat = static_cast<int32_t>(std::llround(g.lat_deg * 1.0e7));
    out.lon = static_cast<int32_t>(std::llround(g.lon_deg * 1.0e7));
    out.alt = static_cast<int32_t>(std::llround(g.height_m * 1000.0));
    out.vn = static_cast<int16_t>(std::llround(v.x * 100.0));
    out.ve = static_cast<int16_t>(std::llround(v.y * 100.0));
    out.vd = static_cast<int16_t>(std::llround(v.z * 100.0));
    const double speed = std::hypot(v.x, v.y);
    out.vel = static_cast<uint16_t>(std::llround(speed * 100.0));
    out.cog = 65535;  // below 5 cm/s the course is not defined
    if (speed >= 0.05) {
        double deg = std::atan2(v.y, v.x) * 180.0 / 3.14159265358979323846;
        if (deg < 0.0) deg += 360.0;
        out.cog = static_cast<uint16_t>(std::llround(deg * 100.0) % 36000);
    }
    out.eph = 100;  // HDOP and VDOP 1.0: Spade's receiver model carries sigmas, not DOPs
    out.epv = 100;
    out.fix_type = 3;
    out.satellites_visible = 10;
    return out;
}

TEST(FcLinkExchange, AGnssFixBecomesHilGpsInTheExchangeWhereItIsNew) {
    const spade::scene::ModelFile model = x500();
    ASSERT_TRUE(model.model.propulsion.has_value());
    std::optional<GpsRig> a = make_gps_rig();
    std::optional<GpsRig> b = make_gps_rig();
    ASSERT_TRUE(a.has_value() && b.has_value());
    const std::vector<SensorBinding> sa = gps_bindings(*a);
    const std::vector<SensorBinding> sb = gps_bindings(*b);
    spade::Result<LockstepHost> host_a = LockstepHost::open(a->rig->sim, a->rig->vehicle, sa);
    spade::Result<LockstepHost> host_b = LockstepHost::open(b->rig->sim, b->rig->vehicle, sb);
    ASSERT_TRUE(host_a.has_value() && host_b.has_value());

    fcl::ExchangeConfig config;
    config.chain = &*model.model.propulsion;
    config.gnss_binding = 1;
    config.geo_frame = spade::geo::make_local_frame(spade::geo::Geodetic{40.76, -113.90, 1267.0});
    config.home_world = glm::dvec3(0.0);
    auto gps_seen = std::make_shared<std::vector<std::optional<mav::HilGps>>>();
    auto sensor_seen = std::make_shared<Seen>();
    auto link = fcl::AutopilotLink::create(
        [gps_seen, sensor_seen](const mav::HilSensor& s, const std::optional<mav::HilGps>& g) -> spade::Result<fcl::ActuatorReply> {
            sensor_seen->sensors.push_back(s);
            gps_seen->push_back(g);
            return reply_of({0.5f, 0.5f, 0.5f, 0.5f});
        },
        config);
    ASSERT_TRUE(link.has_value());

    ASSERT_TRUE(link->exchange(*host_a).has_value());  // the boot message (zero boot steps)
    const std::array<double, 4> duties{0.5, 0.5, 0.5, 0.5};
    const auto omega = fcl::autopilot_duties_to_rotor_commands(*model.model.propulsion, duties, config.density, config.soc);
    ASSERT_TRUE(omega.has_value());
    ASSERT_TRUE(host_b->command(*omega).has_value());
    std::optional<spade::fc_host::Sample> fix_b;
    for (int period = 1; period <= 5; ++period) {
        ASSERT_TRUE(link->exchange(*host_a).has_value());
        ASSERT_TRUE(host_b->advance_to(static_cast<uint64_t>(period) * kImuDivider * kDt).has_value());
        const auto taken = host_b->take_sensors();
        ASSERT_TRUE(taken.has_value());
        for (const spade::fc_host::Sample& s : *taken) {
            if (s.kind == spade::fc_host::SensorKind::gnss) fix_b = s;
        }
    }
    ASSERT_EQ(gps_seen->size(), 6u);  // the boot message and five periods
    for (std::size_t i = 1; i <= 4; ++i) EXPECT_FALSE((*gps_seen)[i].has_value()) << "period " << i << ": no new fix, no HIL_GPS";
    ASSERT_TRUE((*gps_seen)[5].has_value()) << "the receiver's divider is 40 ticks: its fix is new in period 5";
    ASSERT_TRUE(fix_b.has_value());
    spade::sensors::GnssFix fix{};
    std::memcpy(&fix, fix_b->bytes.data(), sizeof(fix));
    const mav::HilGps want = expected_gps(config, glm::dvec3(fix.position), glm::dvec3(fix.velocity), sensor_seen->sensors[5].time_usec);
    EXPECT_EQ(*(*gps_seen)[5], want);
}

TEST(FcLinkExchange, ABootMessageCarriesTheHeldPositionAsAStillFixEveryTwoHundredMilliseconds) {
    const spade::scene::ModelFile model = x500();
    ASSERT_TRUE(model.model.propulsion.has_value());
    std::optional<GpsRig> r = make_gps_rig();
    ASSERT_TRUE(r.has_value());
    const std::vector<SensorBinding> sensors = gps_bindings(*r);
    spade::Result<LockstepHost> host = LockstepHost::open(r->rig->sim, r->rig->vehicle, sensors);
    ASSERT_TRUE(host.has_value());
    fcl::ExchangeConfig config;
    config.chain = &*model.model.propulsion;
    config.gnss_binding = 1;
    config.geo_frame = spade::geo::make_local_frame(spade::geo::Geodetic{40.76, -113.90, 1267.0});
    auto seen = std::make_shared<std::vector<std::optional<mav::HilGps>>>();
    auto times = std::make_shared<std::vector<uint64_t>>();
    auto link = fcl::AutopilotLink::create(
        [seen, times](const mav::HilSensor& s, const std::optional<mav::HilGps>& g) -> spade::Result<fcl::ActuatorReply> {
            seen->push_back(g);
            times->push_back(s.time_usec);
            return reply_of({0, 0, 0, 0}, true);
        },
        config);
    ASSERT_TRUE(link.has_value());
    const auto held = host->boot_readings(1);
    ASSERT_TRUE(held.has_value());
    for (int i = 0; i < 51; ++i) ASSERT_TRUE(link->exchange(*host).has_value());  // 51 boot messages of 4 ms: 204 ms
    ASSERT_EQ(seen->size(), 51u);
    ASSERT_TRUE((*seen)[0].has_value()) << "the first boot message carries a fix";
    EXPECT_EQ(*(*seen)[0], expected_gps(config, glm::dvec3(held->position), glm::dvec3(held->velocity), (*times)[0]));
    for (std::size_t i = 1; i < 50; ++i) EXPECT_FALSE((*seen)[i].has_value()) << "boot message " << (i + 1);
    EXPECT_TRUE((*seen)[50].has_value()) << "and one every 50 boot messages (200 ms of link time)";
}

}  // namespace
