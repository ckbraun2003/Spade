// test_fc_host_run_record.cpp -- the run folder the host leaves (tools/fc_host/run_record.hpp). The checker it is held to is tests/fc_run_folder_check.hpp,
// written apart from the writer (TD-4): its own reader, its own file list, its own SHA-256 comparison.
#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <string>

#include "fc_host/lockstep.hpp"
#include "fc_host/run_record.hpp"
#include "fc_host_test_rig.hpp"
#include "fc_run_folder_check.hpp"
#include "unique_temp.hpp"
#include "world/sha256.hpp"

namespace {

using namespace fc_host_test;
using spade::fc_host::RunIdentity;
using spade::fc_host::RunMeters;
using spade::fc_host::RunRecord;
using spade::fc_host::StageLatency;

constexpr float kCmd[4] = {500.0f, 520.0f, 500.0f, 480.0f};

[[nodiscard]] RunIdentity good_identity() {
    RunIdentity id;
    id.run_id = "fc-host-test-0001";
    id.mode = "measured";
    id.box = "test-box";
    id.tree = "0123456789ab";
    id.spade_build_id = "msvc-release";
    id.spade_commit = std::string(40, 'a');
    id.engine_config_hash = std::string(64, '1');
    id.world_file_hash = std::string(64, '2');
    id.model_hash = std::string(64, '3');
    id.link_kind = "fake";
    id.link_version = "test";
    id.software_identity = "fixture-package";
    id.package_hash = std::string(64, '4');
    id.grade_world = "reference";
    id.grade_mode = "best-effort";
    id.grade_link = "banded";
    id.profile_board_yaml = "board: fixture\ncpu_mhz: 480\n";
    return id;
}

[[nodiscard]] RunMeters good_meters() {
    RunMeters m;
    m.overruns = 0;
    m.late_samples = 1;
    m.skipped_samples = 0;
    m.memory_peak_bytes = 123456;
    StageLatency chain;
    chain.name = "chain";
    chain.min_ns = 1000.0;
    chain.mean_ns = 1500.5;
    chain.p99_ns = 2000.0;
    chain.max_ns = 2100.0;
    m.stages.push_back(chain);
    return m;
}

// A host that has (optionally) noted a 3-step boot, run 3 sensor cycles with commands, and taken its samples.
[[nodiscard]] bool run_host(std::optional<Rig>& rig, spade::Result<LockstepHost>& host, bool note_boot) {
    rig = make_rig(8);
    if (!rig) return false;
    const std::vector<SensorBinding> sensors = all_bindings(*rig);
    host = LockstepHost::open(rig->sim, rig->vehicle, sensors);
    if (!host) return false;
    if (note_boot && !host->note_boot(3, 12'000, BootConfig{})) return false;
    for (int cycle = 1; cycle <= 3; ++cycle) {
        const float cmd[4] = {kCmd[0] + static_cast<float>(cycle), kCmd[1], kCmd[2] - static_cast<float>(cycle), kCmd[3]};
        if (!host->command(cmd) || !host->advance_to(static_cast<uint64_t>(cycle) * 8 * kDt) || !host->take_sensors()) return false;
    }
    return true;
}

[[nodiscard]] spade::fc::Files as_files(const RunRecord& record) {
    spade::fc::Files files;
    for (const auto& [name, bytes] : record.files) files[name] = std::string(reinterpret_cast<const char*>(bytes.data()), bytes.size());
    return files;
}

[[nodiscard]] std::map<std::string, std::string> yaml_of(const RunRecord& record, const char* name) {
    std::string err;
    const spade::fc::Files files = as_files(record);
    std::map<std::string, std::string> out = spade::fc::read_flat_yaml(files.at(name), &err);
    EXPECT_TRUE(err.empty()) << name << ": " << err;
    return out;
}

TEST(FcHostRunRecord, TheRecordPassesEveryCheckTheCheckerImplements) {
    std::optional<Rig> rig;
    spade::Result<LockstepHost> host = std::unexpected(spade::Error{spade::Code::internal, "unset"});
    ASSERT_TRUE(run_host(rig, host, true));
    const spade::Result<RunRecord> record = spade::fc_host::build_run_record(*host, good_identity(), good_meters());
    ASSERT_OK(record);
    const spade::fc::Report report = spade::fc::check_run_folder(spade::fc::RunFolder{as_files(*record)});
    EXPECT_TRUE(report.ok()) << report.text();
    for (const char* id : {"K1", "K2", "K3", "K5", "K6", "K7", "K9", "K10"}) EXPECT_NE(report.find(id, spade::fc::Verdict::pass), nullptr) << id << "\n" << report.text();
}

TEST(FcHostRunRecord, RunYamlHoldsWhatTheHostKnowsAndTheGradesAreTheWeakest) {
    std::optional<Rig> rig;
    spade::Result<LockstepHost> host = std::unexpected(spade::Error{spade::Code::internal, "unset"});
    ASSERT_TRUE(run_host(rig, host, true));
    const spade::Result<RunRecord> record = spade::fc_host::build_run_record(*host, good_identity(), good_meters());
    ASSERT_OK(record);
    const auto run = yaml_of(*record, "run.yaml");
    EXPECT_EQ(run.at("mode"), "measured");
    EXPECT_EQ(run.at("box"), "test-box");
    EXPECT_EQ(run.at("dt_ns"), "500000");
    EXPECT_EQ(run.at("substeps"), "1");
    EXPECT_EQ(run.at("sensor_periods_ticks"), "[8, 40, 20]");
    EXPECT_EQ(run.at("trace_ticks"), "24");
    EXPECT_EQ(run.at("trace_hash"), host->trace_hash());
    EXPECT_EQ(run.at("fc_compute_not_modelled"), "true");
    EXPECT_EQ(run.at("grade_run"), "best-effort") << "the weakest of reference, best-effort and banded";
    EXPECT_EQ(run.at("profile_hash"), spade::sha256_hex(good_identity().profile_board_yaml));
    EXPECT_EQ(run.at("command_latency_ticks"), "0");
    const uint64_t world_seed = (*rig->sim.world_params(0))->seed;
    EXPECT_EQ(run.at("seeds"), "[" + std::to_string(world_seed) + ", " + std::to_string(BootConfig{}.seed) + "]");
    EXPECT_EQ(run.at("boot_steps"), "3");
    EXPECT_EQ(run.at("boot_end_time_usec"), "12000");
    EXPECT_EQ(run.at("boot_sigma_accel"), "0.02");
    EXPECT_EQ(run.at("boot_sigma_gyro"), "0.002");
    EXPECT_EQ(run.at("boot_sigma_pressure_pa"), "1");
    EXPECT_EQ(run.at("boot_sigma_mag_ut"), "0.1");
}

TEST(FcHostRunRecord, ARunWithNoBootSaysNothingOfOne) {
    std::optional<Rig> rig;
    spade::Result<LockstepHost> host = std::unexpected(spade::Error{spade::Code::internal, "unset"});
    ASSERT_TRUE(run_host(rig, host, false));
    const spade::Result<RunRecord> record = spade::fc_host::build_run_record(*host, good_identity(), good_meters());
    ASSERT_OK(record);
    const auto run = yaml_of(*record, "run.yaml");
    EXPECT_EQ(run.count("boot_steps"), 0u);
    EXPECT_EQ(run.count("boot_sigma_accel"), 0u);
    EXPECT_EQ(run.at("seeds"), "[" + std::to_string((*rig->sim.world_params(0))->seed) + "]");
}

TEST(FcHostRunRecord, TheFilesAreTheHostsOwnBytesAndTheManifestHashesEachOne) {
    std::optional<Rig> rig;
    spade::Result<LockstepHost> host = std::unexpected(spade::Error{spade::Code::internal, "unset"});
    ASSERT_TRUE(run_host(rig, host, true));
    const spade::Result<RunRecord> record = spade::fc_host::build_run_record(*host, good_identity(), good_meters());
    ASSERT_OK(record);
    const spade::Result<std::vector<std::byte>> trace = host->trace_bytes();
    ASSERT_OK(trace);
    EXPECT_TRUE(record->files.at("trace.bin") == *trace);
    EXPECT_TRUE(record->files.at("commands.bin") == host->commands_bytes());
    const spade::fc::Files files = as_files(*record);
    EXPECT_EQ(files.at("profile.board.yaml"), good_identity().profile_board_yaml);
    const auto manifest = yaml_of(*record, "manifest.yaml");
    std::size_t listed = 0;
    for (const auto& [key, value] : manifest) {
        if (key.rfind("files.", 0) != 0) continue;
        ++listed;
        EXPECT_EQ(value, spade::sha256_hex(files.at(key.substr(6)))) << key;
    }
    EXPECT_EQ(listed, 5u) << "run.yaml, trace.bin, commands.bin, meters.yaml, profile.board.yaml: every file but the manifest";
    EXPECT_EQ(manifest.count("files.manifest.yaml"), 0u);
}

TEST(FcHostRunRecord, MetersYamlHoldsTheCountsAndTheStageLatencies) {
    std::optional<Rig> rig;
    spade::Result<LockstepHost> host = std::unexpected(spade::Error{spade::Code::internal, "unset"});
    ASSERT_TRUE(run_host(rig, host, true));
    const spade::Result<RunRecord> record = spade::fc_host::build_run_record(*host, good_identity(), good_meters());
    ASSERT_OK(record);
    const auto meters = yaml_of(*record, "meters.yaml");
    EXPECT_EQ(meters.at("overruns"), "0");
    EXPECT_EQ(meters.at("late_samples"), "1");
    EXPECT_EQ(meters.at("skipped_samples"), "0");
    EXPECT_EQ(meters.at("memory_peak_bytes"), "123456");
    EXPECT_EQ(meters.at("stages.chain.latency_ns.min"), "1000");
    EXPECT_EQ(meters.at("stages.chain.latency_ns.mean"), "1500.5");
    EXPECT_EQ(meters.at("stages.chain.latency_ns.p99"), "2000");
    EXPECT_EQ(meters.at("stages.chain.latency_ns.max"), "2100");
}

TEST(FcHostRunRecord, ARecordThatCannotBeWrittenTruthfullyIsRefusedNamingTheField) {
    std::optional<Rig> rig;
    spade::Result<LockstepHost> host = std::unexpected(spade::Error{spade::Code::internal, "unset"});
    ASSERT_TRUE(run_host(rig, host, true));
    const auto refused = [&](const RunIdentity& id, const RunMeters& meters, const char* needle) {
        const spade::Result<RunRecord> r = spade::fc_host::build_run_record(*host, id, meters);
        ASSERT_FALSE(r.has_value()) << needle;
        EXPECT_EQ(r.error().code, spade::Code::invalid_argument);
        EXPECT_NE(r.error().context.find(needle), std::string::npos) << needle << " not in: " << r.error().context;
    };
    const auto with = [](auto&& edit) {
        RunIdentity id = good_identity();
        edit(id);
        return id;
    };
    const auto with_meters = [](auto&& edit) {
        RunMeters m = good_meters();
        edit(m);
        return m;
    };
    refused(with([](RunIdentity& i) { i.mode = "budget"; }), good_meters(), "phase 2");
    refused(with([](RunIdentity& i) { i.mode = "weird"; }), good_meters(), "mode");
    refused(with([](RunIdentity& i) { i.run_id.clear(); }), good_meters(), "run_id");
    refused(with([](RunIdentity& i) { i.software_identity = "a\nb: c"; }), good_meters(), "software_identity");
    refused(with([](RunIdentity& i) { i.package_hash = "abc"; }), good_meters(), "package_hash");
    refused(with([](RunIdentity& i) { i.grade_world = "excellent"; }), good_meters(), "grade_world");
    refused(with([](RunIdentity& i) { i.box.clear(); }), good_meters(), "box");
    refused(with([](RunIdentity& i) { i.tree.clear(); }), good_meters(), "tree");
    refused(with([](RunIdentity& i) { i.profile_board_yaml.clear(); }), good_meters(), "profile_board_yaml");
    refused(good_identity(), with_meters([](RunMeters& m) { m.stages.clear(); }), "stage");
    refused(good_identity(), with_meters([](RunMeters& m) { m.stages[0].p99_ns = std::numeric_limits<double>::quiet_NaN(); }), "latency");
    refused(good_identity(), with_meters([](RunMeters& m) { m.stages[0].min_ns = 5000.0; }), "latency");
    refused(good_identity(), with_meters([](RunMeters& m) { m.stages[0].name = "a.b"; }), "stage name");
    refused(good_identity(), with_meters([](RunMeters& m) { m.memory_peak_bytes.reset(); }), "memory_peak_bytes");

    // a user's folder in a value is refused, naming the field and never quoting the path
    RunIdentity leaky = good_identity();
    leaky.link_version = "C:\\Users\\someone\\px4";
    const spade::Result<RunRecord> r = spade::fc_host::build_run_record(*host, leaky, good_meters());
    ASSERT_FALSE(r.has_value());
    EXPECT_NE(r.error().context.find("link_version"), std::string::npos) << r.error().context;
    EXPECT_EQ(r.error().context.find("someone"), std::string::npos) << "the path is never quoted back";
}

TEST(FcHostRunRecord, WritingTheFolderLeavesTheSameBytesAndNeverOverwritesARecord) {
    std::optional<Rig> rig;
    spade::Result<LockstepHost> host = std::unexpected(spade::Error{spade::Code::internal, "unset"});
    ASSERT_TRUE(run_host(rig, host, true));
    const spade::Result<RunRecord> record = spade::fc_host::build_run_record(*host, good_identity(), good_meters());
    ASSERT_OK(record);
    const spade::test::TempDir dir("fc_host_run_record");
    const std::filesystem::path target = dir.path() / "run-0001";
    ASSERT_OK(spade::fc_host::write_run_folder(*record, target));
    const spade::fc::RunFolder loaded = spade::fc::load_run_folder(target);
    EXPECT_EQ(loaded.files, as_files(*record));
    const spade::fc::Report report = spade::fc::check_run_folder(loaded);
    EXPECT_TRUE(report.ok()) << report.text();

    // a second write into the same folder is refused and changes nothing
    RunRecord other = *record;
    other.files["run.yaml"].push_back(std::byte{'x'});
    const spade::Result<void> again = spade::fc_host::write_run_folder(other, target);
    ASSERT_FALSE(again.has_value());
    EXPECT_EQ(again.error().code, spade::Code::invalid_argument);
    EXPECT_NE(again.error().context.find("not empty"), std::string::npos) << again.error().context;
    EXPECT_EQ(spade::fc::load_run_folder(target).files, as_files(*record));
}

}  // namespace
