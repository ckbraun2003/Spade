// The runner's run record for the flight film (tests/fc_flight_record.hpp), built offline from the real flight world and held to the checker the host's own
// record is held to (tests/fc_run_folder_check.hpp). Nothing here needs PX4.
#include <gtest/gtest.h>

#include <array>
#include <filesystem>
#include <string>

#include "fc_flight_record.hpp"
#include "fc_flight_world.hpp"
#include "fc_run_folder_check.hpp"

namespace {

using namespace fc_flight_test;
using spade::fc_host::RunRecord;
using spade::modules::ConfigurationEntry;

bool is_hex64(const std::string& s) {
    if (s.size() != 64) return false;
    for (char c : s)
        if (!((c >= '0' && c <= '9') || (c >= 'a' && c <= 'f'))) return false;
    return true;
}

TEST(FcFlightRecord, TheEngineConfigDigestIs64HexAndChangesWithAnyOneInput) {
    const std::vector<ConfigurationEntry> mods = {{"rigid_contact", 1, 16, 0x1111}, {"baro", 2, 8, 0x2222}, {"mag", 1, 8, 0x3333}};
    const std::string base = engine_config_digest(0xABCDEF, mods);
    EXPECT_TRUE(is_hex64(base)) << base;
    EXPECT_EQ(base, engine_config_digest(0xABCDEF, mods)) << "the same inputs give the same digest";
    EXPECT_NE(base, engine_config_digest(0xABCDEE, mods)) << "the replay config hash is in it";
    auto changed = mods;
    changed[1].hash = 0x2223;
    EXPECT_NE(base, engine_config_digest(0xABCDEF, changed)) << "one module's configuration hash is in it";
    changed = mods;
    changed[2].version = 2;
    EXPECT_NE(base, engine_config_digest(0xABCDEF, changed)) << "a module's version is in it";
    changed = mods;
    std::swap(changed[0], changed[1]);
    EXPECT_NE(base, engine_config_digest(0xABCDEF, changed)) << "the set order is in it";
}

struct LiveRun {
    std::optional<FlightWorld> world;
    spade::Result<spade::fc_host::LockstepHost> host = std::unexpected(spade::Error{spade::Code::internal, "unset"});
};

bool run_a_little(LiveRun& r) {
    r.world = make_flight_world();
    if (!r.world) return false;
    r.host = spade::fc_host::LockstepHost::open(r.world->sim, r.world->vehicle, r.world->bindings);
    if (!r.host) return false;
    const std::array<float, 4> off{0.0f, 0.0f, 0.0f, 0.0f};
    for (int cycle = 1; cycle <= 5; ++cycle) {
        if (!r.host->command(off) || !r.host->advance_to(static_cast<uint64_t>(cycle) * kImuDivider * kDtNs) || !r.host->take_sensors()) return false;
    }
    return true;
}

FlightRecordInputs good_inputs() {
    FlightRecordInputs in;
    in.run_id = "fc-record-test-0001";
    in.package_hash = "sha256:" + std::string(64, '4');  // as `docker image inspect --format {{.Id}}` prints it
    in.dockerfile = std::filesystem::path(SPADE_REPO_DIR) / "tools/fc_link/px4/Dockerfile";
    in.link_version = "v1.17.0";
    in.model_file = std::filesystem::path(SPADE_REPO_DIR) / "tools/flow_movie/data/x500/x500.model.yaml";
    in.world_file = std::filesystem::path(SPADE_REPO_DIR) / "tests/fc_flight_world.hpp";
    in.profile_board_yaml = "board: px4_sitl_10090\nlink: docker_forward\n";
    in.wait_ns = {1.5e6, 2.0e6, 1.7e6, 9.0e6, 1.6e6};
    in.gap_ns = {0.2e6, 0.3e6, 0.2e6, 0.4e6, 0.2e6};
    return in;
}

spade::fc::Files as_files(const RunRecord& record) {
    spade::fc::Files files;
    for (const auto& [name, bytes] : record.files) files[name] = std::string(reinterpret_cast<const char*>(bytes.data()), bytes.size());
    return files;
}

TEST(FcFlightRecord, ARecordOfTheFlightWorldPassesTheChecksTheCheckerImplements) {
    LiveRun r;
    ASSERT_TRUE(run_a_little(r));
    const auto record = build_flight_record(*r.host, r.world->sim, good_inputs());
    ASSERT_TRUE(record.has_value()) << record.error().context;
    const spade::fc::Report report = spade::fc::check_run_folder(spade::fc::RunFolder{as_files(*record)});
    EXPECT_TRUE(report.ok()) << report.text();
    for (const char* id : {"K1", "K2", "K3", "K5", "K6", "K7", "K9", "K10"}) EXPECT_NE(report.find(id, spade::fc::Verdict::pass), nullptr) << id << "\n" << report.text();
}

TEST(FcFlightRecord, TheRecordSaysWhatTheRunWasAndGradesItAsALiveAutopilotRun) {
    LiveRun r;
    ASSERT_TRUE(run_a_little(r));
    const auto record = build_flight_record(*r.host, r.world->sim, good_inputs());
    ASSERT_TRUE(record.has_value()) << record.error().context;
    std::string err;
    const auto run = spade::fc::read_flat_yaml(as_files(*record).at("run.yaml"), &err);
    ASSERT_TRUE(err.empty()) << err;
    EXPECT_EQ(run.at("run_id"), "fc-record-test-0001");
    EXPECT_EQ(run.at("mode"), "measured");
    EXPECT_EQ(run.at("box"), "windows-docker-desktop");
    EXPECT_EQ(run.at("package_hash"), std::string(64, '4'));
    const std::string identity = run.at("software_identity");
    EXPECT_EQ(identity.rfind("image id of the local build of PX4 v1.17.0 on px4io/px4-dev@sha256:", 0), 0u) << identity;
    EXPECT_NE(identity.find("; recipe sha256 "), std::string::npos) << "the recipe's hash rides beside the identity: " << identity;
    EXPECT_EQ(identity.find("Users"), std::string::npos);
    EXPECT_EQ(run.at("link_kind"), "px4");
    EXPECT_EQ(run.at("grade_world"), std::string(spade::grade_name(r.world->sim.grade().grade))) << "the engine's own grade for the world's module set";
    EXPECT_EQ(run.at("grade_mode"), "best-effort");
    EXPECT_EQ(run.at("grade_link"), "best-effort");
    EXPECT_EQ(run.at("grade_run"), "best-effort") << "the weakest of the three";
    EXPECT_TRUE(is_hex64(run.at("engine_config_hash")));
    EXPECT_TRUE(is_hex64(run.at("world_file_hash")));
    EXPECT_TRUE(is_hex64(run.at("model_hash")));
    EXPECT_NE(run.at("world_file_hash"), run.at("model_hash"));
}

TEST(FcFlightRecord, TheMetersHoldTheMeasuredPeakMemoryAndBothStages) {
    LiveRun r;
    ASSERT_TRUE(run_a_little(r));
    const auto record = build_flight_record(*r.host, r.world->sim, good_inputs());
    ASSERT_TRUE(record.has_value()) << record.error().context;
    const std::string meters = as_files(*record).at("meters.yaml");
    EXPECT_NE(meters.find("memory_peak_bytes"), std::string::npos) << meters;
    EXPECT_NE(meters.find("px4_wait"), std::string::npos) << meters;
    EXPECT_NE(meters.find("host_gap"), std::string::npos) << meters;
}

TEST(FcFlightRecord, WhatTheRunLacksRefusesTheRecordNamingIt) {
    LiveRun r;
    ASSERT_TRUE(run_a_little(r));
    const auto refused = [&](FlightRecordInputs in, const char* needle) {
        const auto record = build_flight_record(*r.host, r.world->sim, in);
        ASSERT_FALSE(record.has_value()) << needle;
        EXPECT_NE(record.error().context.find(needle), std::string::npos) << record.error().context;
    };
    FlightRecordInputs in = good_inputs();
    in.package_hash.clear();
    refused(in, "package_hash");
    in = good_inputs();
    in.package_hash = "abc";
    refused(in, "package_hash");
    in = good_inputs();
    in.dockerfile = "no-such-dockerfile";
    refused(in, "recipe");
    in = good_inputs();
    in.dockerfile = std::filesystem::path(SPADE_REPO_DIR) / "README.md";
    refused(in, "FROM");
    in = good_inputs();
    in.wait_ns.clear();
    refused(in, "px4_wait");
    in = good_inputs();
    in.gap_ns.clear();
    refused(in, "host_gap");
    in = good_inputs();
    in.model_file = "no-such-model-file.yaml";
    refused(in, "model_hash");
}

}  // namespace
