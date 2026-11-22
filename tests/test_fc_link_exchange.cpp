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
    ASSERT_TRUE(host_b->advance_to(kImuDivider * kDt).has_value());
    const auto taken = host_b->take_sensors();
    ASSERT_TRUE(taken.has_value());
    ASSERT_EQ(taken->size(), 1u);
    spade::sensors::ImuSample imu{};
    std::memcpy(&imu, (*taken)[0].bytes.data(), sizeof(imu));
    const glm::dvec3 accel = spade::fc_link::frames::body_to_frd(config.frame, glm::dvec3(imu.accel));
    const glm::dvec3 gyro = spade::fc_link::frames::body_to_frd(config.frame, glm::dvec3(imu.gyro));
    const std::array<double, 4> duties{0.2, 0.4, 0.6, 0.8};
    const auto omega = fcl::autopilot_duties_to_rotor_commands(*model.model.propulsion, duties, config.density, config.soc);
    ASSERT_TRUE(omega.has_value());
    ASSERT_TRUE(host_b->command(*omega).has_value());

    const spade::Result<fcl::ExchangeResult> done = link->exchange(*host_a);
    ASSERT_TRUE(done.has_value()) << done.error().context;
    EXPECT_FALSE(done->booting);
    EXPECT_EQ(done->tick, kImuDivider);
    EXPECT_EQ(host_a->tick(), kImuDivider);
    ASSERT_EQ(seen->sensors.size(), 1u);
    const mav::HilSensor& m = seen->sensors[0];
    EXPECT_EQ(m.time_usec, kImuDivider * kDt / 1000u) << "the host's sim time in microseconds (no boot offset)";
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

}  // namespace
