// ---------------------------------------------------------------------------
// test_fc_link_run.cpp -- FC-F1, the part that needs no autopilot: a run DRIVEN THROUGH the exchange seam (fc_link/autopilot_link.hpp) leaves the run record
// (fc_host/run_record.hpp) and the record passes the checker (tests/fc_run_folder_check.hpp, written apart from the writer). The peer is a scripted
// transport; the live PX4 run is the same loop with the real link (docs/design/test-docs/plans/2026-11-21-fc-tests.md, FC-F1).
//
//   FcLinkRun.ARunDrivenThroughTheExchangeLeavesARecordTheCheckerPasses          written to a folder, read back, checked: K1-K3, K5-K7, K9, K10 pass
//   FcLinkRun.TheRecordSaysTheBootAndTheTicksTheExchangeSawAndTheClockAddsUp     boot_steps, boot_end_time_usec and trace_ticks are the loop's own counts;
//                                                                                the last HIL_SENSOR time is the boot end plus the ticks run
//   FcLinkRun.TwoRunsOfTheSameScriptLeaveTheSameBytesAndTheBootLeavesNoMarkOnTheTrace
//   FcLinkRun.ARecordOfAnExchangeRunFailsTheCheckerWhenAFileIsChanged            one flipped bit in trace.bin or commands.bin: K5 FAIL
// ---------------------------------------------------------------------------

#include <gtest/gtest.h>

#include <array>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "fc_host/run_record.hpp"
#include "fc_host_test_rig.hpp"
#include "fc_link/autopilot_link.hpp"
#include "fc_run_folder_check.hpp"
#include "scene/model_file.hpp"
#include "unique_temp.hpp"

namespace {

using namespace fc_host_test;
namespace fcl = spade::fc_link;
namespace mav = spade::fc_link::mavlink;
using spade::fc_host::RunIdentity;
using spade::fc_host::RunMeters;
using spade::fc_host::RunRecord;

constexpr uint32_t kImuDivider = 8;      // 8 ticks of 500 us: the 4 ms link step
constexpr uint64_t kLinkStepUs = 4000;

[[nodiscard]] RunIdentity identity() {
    RunIdentity id;
    id.run_id = "fc-link-run-0001";
    id.mode = "measured";
    id.box = "test-box";
    id.tree = "0123456789ab";
    id.spade_build_id = "msvc-release";
    id.spade_commit = std::string(40, 'a');
    id.engine_config_hash = std::string(64, '1');
    id.world_file_hash = std::string(64, '2');
    id.model_hash = std::string(64, '3');
    id.link_kind = "scripted";
    id.link_version = "test";
    id.software_identity = "scripted-reply";
    id.package_hash = std::string(64, '4');
    id.grade_world = "reference";
    id.grade_mode = "best-effort";
    id.grade_link = "banded";
    id.profile_board_yaml = "board: fixture\ncpu_mhz: 480\n";
    return id;
}

[[nodiscard]] RunMeters meters() {
    RunMeters m;
    m.memory_peak_bytes = 123456;
    return m;
}

[[nodiscard]] spade::fc::Files as_files(const RunRecord& record) {
    spade::fc::Files files;
    for (const auto& [name, bytes] : record.files) files[name] = std::string(reinterpret_cast<const char*>(bytes.data()), bytes.size());
    return files;
}

struct Driven {
    RunRecord record;
    uint64_t exchanges = 0;       // transport calls: the boot, the exchange that ends it, then one per period
    uint64_t last_time_usec = 0;  // the time_usec of the last HIL_SENSOR the transport saw
    uint64_t tick = 0;            // the host's tick at the end
};

// The loop a flight run makes: `boot_replies` booting replies, the exchange whose reply ends the boot, then `periods` periods of the world, every reply
// a different set of motor outputs so the commands the host logs are not constant.
[[nodiscard]] std::optional<Driven> drive(uint64_t boot_replies, uint64_t periods) {
    const auto model = spade::scene::load_model_file(std::filesystem::path(SPADE_REPO_DIR) / "tools/flow_movie/data/x500/x500.model.yaml");
    if (!model || !model->model.propulsion) {
        ADD_FAILURE() << "the x500 model did not load";
        return std::nullopt;
    }
    std::optional<Rig> rig = make_rig(kImuDivider, 40, 20, kDt, 1, -9.81f);
    if (!rig) {
        ADD_FAILURE() << "the rig did not build";
        return std::nullopt;
    }
    const std::vector<SensorBinding> sensors = all_bindings(*rig);
    spade::Result<LockstepHost> host = LockstepHost::open(rig->sim, rig->vehicle, sensors);
    if (!host) {
        ADD_FAILURE() << "the host did not open: " << host.error().context;
        return std::nullopt;
    }
    Driven out;
    fcl::ExchangeConfig config;
    config.chain = &*model->model.propulsion;
    config.baro_binding = 1;
    config.mag_binding = 2;
    auto calls = std::make_shared<uint64_t>(0);
    auto last = std::make_shared<uint64_t>(0);
    auto link = fcl::AutopilotLink::create(
        [calls, last, boot_replies](const mav::HilSensor& s, const std::optional<mav::HilGps>&) -> spade::Result<fcl::ActuatorReply> {
            ++*calls;
            *last = s.time_usec;
            fcl::ActuatorReply r;
            const bool booting = *calls <= boot_replies;
            const float base = 0.5f + 0.02f * static_cast<float>(*calls % 5);
            for (std::size_t i = 0; i < 4; ++i) r.controls[i] = booting ? 0.0f : base + 0.01f * static_cast<float>(i);
            r.armed = !booting;
            r.lockstep = true;
            r.booting = booting;
            return r;
        },
        config);
    if (!link) {
        ADD_FAILURE() << "the link did not open: " << link.error().context;
        return std::nullopt;
    }
    for (uint64_t i = 0; i < boot_replies + 1 + periods; ++i) {
        const auto r = link->exchange(*host);
        if (!r) {
            ADD_FAILURE() << "exchange " << i << ": " << r.error().context;
            return std::nullopt;
        }
    }
    const spade::Result<RunRecord> record = spade::fc_host::build_run_record(*host, identity(), meters());
    if (!record) {
        ADD_FAILURE() << "the record was refused: " << record.error().context;
        return std::nullopt;
    }
    out.record = *record;
    out.exchanges = *calls;
    out.last_time_usec = *last;
    out.tick = host->tick();
    return out;
}

TEST(FcLinkRun, ARunDrivenThroughTheExchangeLeavesARecordTheCheckerPasses) {
    const std::optional<Driven> run = drive(3, 25);
    ASSERT_TRUE(run.has_value());
    const spade::test::TempDir dir("fc_link_run");
    const std::filesystem::path folder = dir.path() / "run";
    ASSERT_TRUE(spade::fc_host::write_run_folder(run->record, folder).has_value());
    const spade::fc::Report report = spade::fc::check_run_folder(spade::fc::load_run_folder(folder));
    EXPECT_TRUE(report.ok()) << report.text();
    for (const char* id : {"K1", "K2", "K3", "K5", "K6", "K7", "K9", "K10"}) EXPECT_NE(report.find(id, spade::fc::Verdict::pass), nullptr) << id << "\n" << report.text();
}

TEST(FcLinkRun, TheRecordSaysTheBootAndTheTicksTheExchangeSawAndTheClockAddsUp) {
    constexpr uint64_t kBoot = 3, kPeriods = 25;
    const std::optional<Driven> run = drive(kBoot, kPeriods);
    ASSERT_TRUE(run.has_value());
    EXPECT_EQ(run->exchanges, kBoot + 1 + kPeriods);
    EXPECT_EQ(run->tick, kPeriods * kImuDivider);
    std::string err;
    const auto yaml = spade::fc::read_flat_yaml(as_files(run->record).at("run.yaml"), &err);
    ASSERT_TRUE(err.empty()) << err;
    EXPECT_EQ(yaml.at("boot_steps"), std::to_string(kBoot));
    EXPECT_EQ(yaml.at("boot_end_time_usec"), std::to_string((kBoot + 1) * kLinkStepUs)) << "the boot's steps and the message that ends it, 4 ms each";
    EXPECT_EQ(yaml.at("trace_ticks"), std::to_string(kPeriods * kImuDivider));
    EXPECT_EQ(yaml.at("sensor_periods_ticks"), "[8, 40, 20]");
    EXPECT_EQ(yaml.at("dt_ns"), "500000");
    EXPECT_EQ(run->last_time_usec, (kBoot + 1 + kPeriods) * kLinkStepUs) << "the lockstep's own arithmetic: (boot steps + steps answered) x the link step";
    EXPECT_EQ(run->last_time_usec, std::stoull(yaml.at("boot_end_time_usec")) + std::stoull(yaml.at("trace_ticks")) * 500u) << "boot end plus the ticks run, 500 us each";
}

TEST(FcLinkRun, TwoRunsOfTheSameScriptLeaveTheSameBytesAndTheBootLeavesNoMarkOnTheTrace) {
    const std::optional<Driven> a = drive(3, 12);
    const std::optional<Driven> b = drive(3, 12);
    ASSERT_TRUE(a.has_value() && b.has_value());
    EXPECT_TRUE(a->record.files == b->record.files) << "the same script gives the same record, file for file";
    const std::optional<Driven> longer = drive(30, 12);
    ASSERT_TRUE(longer.has_value());
    EXPECT_TRUE(a->record.files.at("trace.bin") == longer->record.files.at("trace.bin")) << "a boot of 3 steps and one of 30 leave the same trace";
    EXPECT_TRUE(a->record.files.at("commands.bin") == longer->record.files.at("commands.bin")) << "and the same commands";
}

TEST(FcLinkRun, ARecordOfAnExchangeRunFailsTheCheckerWhenAFileIsChanged) {
    const std::optional<Driven> run = drive(3, 12);
    ASSERT_TRUE(run.has_value());
    for (const char* name : {"trace.bin", "commands.bin"}) {
        spade::fc::Files files = as_files(run->record);
        std::string& bytes = files.at(name);
        ASSERT_GT(bytes.size(), 64u) << name;
        bytes[bytes.size() / 2] = static_cast<char>(bytes[bytes.size() / 2] ^ 0x01);
        const spade::fc::Report report = spade::fc::check_run_folder(spade::fc::RunFolder{files});
        EXPECT_FALSE(report.ok()) << name;
        EXPECT_NE(report.find("K5", spade::fc::Verdict::fail), nullptr) << name << ": the manifest's hash no longer matches\n" << report.text();
    }
}

}  // namespace
