#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <nlohmann/json.hpp>

#include "../tools/part_import/process.hpp"
#include "../tools/part_import/report.hpp"
#include "../tools/part_import/run.hpp"
#include "part_import_fixtures.hpp"
#include "unique_temp.hpp"
#include "world/mesh_file.hpp"
#include "world/sha256.hpp"

// spade_part_import's STEP route (L88 step 4; design note sections 4 and 5), against tests/fake_step_convert.py: the same modes as the real
// step_convert.py (--probe, --inspect, --emit-3mf) with two cubes, so the route (which converter, which arguments, what is stored, what the
// codes are, cancel) is tested without OpenCASCADE. The real converter's new modes are NOT run by any test here. A machine with no python
// skips each test with the reason named.

namespace {

namespace fs = std::filesystem;
using nlohmann::json;
using namespace spade;
using namespace spade::partimport;
using namespace spade::partimporttest;

[[nodiscard]] bool have_python() {
    ProcessOptions o;
    o.argv = {"python", "-c", "pass"};
    const ProcessResult r = run_process(o);
    return r.started && r.exit == 0;
}

#define NEED_PYTHON()                                                                          \
    if (!have_python()) {                                                                      \
        GTEST_SKIP() << "no python on PATH: the STEP route is tested with tests/fake_step_convert.py"; \
    }                                                                                          \
    static_assert(true)

void set_env(const char* name, const char* value) {
#if defined(_WIN32)
    _putenv_s(name, value);
#else
    if (*value == '\0') {
        unsetenv(name);
    } else {
        setenv(name, value, 1);
    }
#endif
}

// Sets an environment variable for one test and clears it after.
struct EnvGuard {
    const char* name;
    EnvGuard(const char* n, const std::string& v) : name(n) { set_env(name, v.c_str()); }
    ~EnvGuard() { set_env(name, ""); }
};

struct Out {
    int exit = -1;
    std::vector<json> lines;
    json result;
    [[nodiscard]] std::string code() const { return result.value("code", std::string("?")); }
};

struct StepRig {
    test::TempDir tmp{"part_import_step"};
    std::atomic<bool> cancel{false};
    RunEnv env;
    fs::path fake = fs::path(SPADE_TESTS_DIR) / "fake_step_convert.py";

    StepRig() {
        env.getenv = [](const char*) -> const char* { return nullptr; };
        env.exe_dir = tmp.path() / "exe";
        env.cancelled = [this] { return cancel.load(); };
        env.token = "s1";
    }
    [[nodiscard]] fs::path store() const { return tmp.path() / "store"; }
    [[nodiscard]] fs::path step_file() const {
        const fs::path p = tmp.path() / "in" / "part.step";
        write_file(p, "ISO-10303-21; fake\n");
        return p;
    }
    [[nodiscard]] std::vector<std::string> with_converter(std::vector<std::string> args) const {
        args.insert(args.end(), {"--step-python", "python", "--step-convert", fake.string()});
        return args;
    }
    Out run(const std::vector<std::string>& args) {
        std::ostringstream out, err;
        Out r;
        r.exit = run_part_import(args, env, out, err);
        std::istringstream in(out.str());
        for (std::string l; std::getline(in, l);) {
            json j = json::parse(l, nullptr, false);
            EXPECT_FALSE(j.is_discarded()) << "not JSON: " << l;
            r.lines.push_back(j);
        }
        if (!r.lines.empty()) r.result = r.lines.back();
        return r;
    }
    [[nodiscard]] int converter_runs(const fs::path& log) const {
        std::ifstream in(log);
        int n = 0;
        for (std::string l; std::getline(in, l);) ++n;
        return n;
    }
};

[[nodiscard]] std::vector<double> numbers(const json& a) {
    std::vector<double> v;
    for (const json& x : a) v.push_back(x.get<double>());
    return v;
}

// ---- probe ---------------------------------------------------------------------------------------------------------------------

TEST(PartImportStep, ProbeReportsTheConverterThatWasGiven) {
    NEED_PYTHON();
    StepRig rig;
    const Out r = rig.run(rig.with_converter({"--probe"}));
    ASSERT_EQ(r.exit, 0) << r.result.dump();
    EXPECT_TRUE(r.result.value("ok", false));
    EXPECT_EQ(r.result.value("mode", std::string()), "probe");
    const json& step = r.result.at("step");
    EXPECT_TRUE(step.value("available", false));
    EXPECT_EQ(step.value("occt", std::string()), "7.8.1-fake");
    EXPECT_FALSE(step.value("python", std::string()).empty());
    EXPECT_EQ(step.value("converter", std::string()), rig.fake.string());
}

TEST(PartImportStep, ProbeWithNothingFoundSaysWhereItLookedAndHowToInstall) {
    StepRig rig;
    const Out r = rig.run({"--probe"});
    EXPECT_EQ(r.exit, 3);
    EXPECT_EQ(r.code(), "step_unavailable");
    EXPECT_FALSE(r.result.at("step").value("available", true));
    EXPECT_FALSE(r.result.at("step").at("searched").empty());
    EXPECT_NE(r.result.at("step").value("install", std::string()).find("pip install"), std::string::npos);
    EXPECT_NE(r.result.at("step").value("install", std::string()).find(".venv-step"), std::string::npos);
}

TEST(PartImportStep, ProbeWithoutOpenCascadeIsStepUnavailableAndSaysSo) {
    NEED_PYTHON();
    const EnvGuard mode("FAKE_STEP_MODE", "noocp");
    StepRig rig;
    const Out r = rig.run(rig.with_converter({"--probe"}));
    EXPECT_EQ(r.exit, 3);
    EXPECT_EQ(r.code(), "step_unavailable");
    EXPECT_FALSE(r.result.at("step").value("available", true));
    EXPECT_NE(r.result.at("step").value("reason", std::string()).find("not installed"), std::string::npos);
}

TEST(PartImportStep, TheEnvironmentNamesThePythonAndTheConverterWhenNoOptionDoes) {
    NEED_PYTHON();
    StepRig rig;
    rig.env.getenv = [&](const char* name) -> const char* {
        static std::string convert;
        convert = rig.fake.string();
        if (std::string(name) == "SPADE_STEP_PYTHON") return "python";
        if (std::string(name) == "SPADE_STEP_CONVERT") return convert.c_str();
        return nullptr;
    };
    const Out r = rig.run({"--probe"});
    EXPECT_EQ(r.exit, 0) << r.result.dump();
    EXPECT_TRUE(r.result.at("step").value("available", false));
}

// ---- inspect -------------------------------------------------------------------------------------------------------------------

TEST(PartImportStep, AFastInspectListsTheSolidsWithTheirBoxAndBrepVolumeAndChecksNoGate) {
    NEED_PYTHON();
    StepRig rig;
    const fs::path in = rig.step_file();
    const Out r = rig.run(rig.with_converter({"--inspect", in.string()}));
    ASSERT_EQ(r.exit, 0) << r.result.dump();
    EXPECT_EQ(r.result.value("format", std::string()), "step");
    EXPECT_EQ(r.result.value("gate", std::string()), "not_checked");
    EXPECT_EQ(r.result.value("depth", std::string()), "fast");
    EXPECT_EQ(r.result.at("unit").value("declared", std::string()), "mm");
    ASSERT_EQ(r.result.at("objects").size(), 2u);
    const json& arm = r.result.at("objects")[0];
    const json& hub = r.result.at("objects")[1];
    EXPECT_EQ(arm.value("name", std::string()), "arm");
    EXPECT_EQ(hub.value("name", std::string()), "hub");
    EXPECT_FALSE(arm.contains("triangles"));
    EXPECT_NEAR(arm.value("volume", 0.0), 1e-6, 1e-12);
    EXPECT_NEAR(hub.value("volume", 0.0), 8e-6, 1e-11);
    EXPECT_NEAR(numbers(hub.at("bbox").at("min"))[0], 0.040, 1e-9);
    EXPECT_NEAR(numbers(hub.at("bbox").at("max"))[0], 0.060, 1e-9);
    EXPECT_EQ(r.result.value("measures_in", std::string()), "m");
}

TEST(PartImportStep, APartWithNoSolidIsListedAsSkippedWithItsCodeAndTheRestIsRead) {
    NEED_PYTHON();
    const EnvGuard mode("FAKE_STEP_MODE", "sheet");
    StepRig rig;
    const Out fast = rig.run(rig.with_converter({"--inspect", rig.step_file().string()}));
    EXPECT_EQ(fast.exit, 0) << fast.result.dump();
    const auto& objects = fast.result.at("objects");
    ASSERT_EQ(objects.size(), 3u) << fast.result.dump();  // arm, hub and the skipped cover
    EXPECT_EQ(objects[0].value("name", std::string()), "arm");
    EXPECT_EQ(objects[2].value("name", std::string()), "cover");
    EXPECT_EQ(objects[2].value("code", std::string()), "no_solid");
    EXPECT_FALSE(objects[2].value("ok", true));
    EXPECT_FALSE(objects[2].value("message", std::string()).empty());
    const Out deep = rig.run(rig.with_converter({"--inspect", rig.step_file().string(), "--deep", "--deflection", "0.1mm"}));
    EXPECT_EQ(deep.exit, 0) << deep.result.dump();
    ASSERT_TRUE(deep.result.contains("skipped")) << deep.result.dump();
    ASSERT_EQ(deep.result.at("skipped").size(), 1u);
    EXPECT_EQ(deep.result.at("skipped")[0].value("name", std::string()), "cover");
    EXPECT_EQ(deep.result.at("skipped")[0].value("code", std::string()), "no_solid");
    EXPECT_EQ(deep.result.at("objects").size(), 2u);  // the solids only: the skipped one is not an object of the gate
}

TEST(PartImportStep, ADeepInspectNeedsADeflectionThenTessellatesAndRunsTheGate) {
    NEED_PYTHON();
    StepRig rig;
    const fs::path in = rig.step_file();
    Out r = rig.run(rig.with_converter({"--inspect", in.string(), "--deep"}));
    EXPECT_EQ(r.exit, 2);
    EXPECT_EQ(r.code(), "deflection_required");
    r = rig.run(rig.with_converter({"--inspect", in.string(), "--deep", "--deflection", "0.1mm"}));
    ASSERT_EQ(r.exit, 0) << r.result.dump();
    EXPECT_EQ(r.result.value("gate", std::string()), "passed");
    EXPECT_EQ(r.result.value("depth", std::string()), "deep");
    ASSERT_EQ(r.result.at("objects").size(), 2u);
    EXPECT_EQ(r.result.at("objects")[0].value("triangles", 0), 12);
    EXPECT_NEAR(r.result.at("objects")[1].value("volume", 0.0), 8e-6, 1e-11);
    EXPECT_EQ(r.result.at("cross_check").size(), 2u);
    EXPECT_FALSE(fs::exists(rig.store()));
}

// ---- import --------------------------------------------------------------------------------------------------------------------

TEST(PartImportStep, AnImportStoresTheSourceAndOneSidecarPerSolidWithTheDeflectionAsTolerance) {
    NEED_PYTHON();
    StepRig rig;
    const fs::path in = rig.step_file();
    const Out r = rig.run(rig.with_converter({"--import", in.string(), "--store", rig.store().string(), "--deflection", "0.1mm"}));
    ASSERT_EQ(r.exit, 0) << r.result.dump();
    EXPECT_FALSE(r.result.value("cached", true));
    EXPECT_EQ(r.result.value("converter", std::string()), "step-1");
    const std::string sha = sha256_hex(read_file(in));
    EXPECT_EQ(r.result.value("sha256", std::string()), sha);
    EXPECT_TRUE(fs::is_regular_file(rig.store() / sha / "part.step"));
    ASSERT_EQ(r.result.at("objects").size(), 2u);
    for (const json& o : r.result.at("objects")) {
        const auto loaded = load_mesh_file(fs::path(o.value("sidecar", std::string())));
        ASSERT_TRUE(loaded.has_value()) << loaded.error().context;
        EXPECT_EQ(loaded->provenance.unit, MeshUnit::millimetre);
        EXPECT_NEAR(loaded->provenance.tolerance, 1e-4, 1e-12);
        EXPECT_NE(loaded->provenance.source.find("part.step"), std::string::npos);
        EXPECT_NE(loaded->provenance.source.find(o.value("name", std::string())), std::string::npos);
    }
    EXPECT_TRUE(r.result.at("objects")[0].contains("cross_check"));
}

TEST(PartImportStep, ASecondImportIsACacheHitThatDoesNotRunTheConverterAndANewDeflectionIsAMiss) {
    NEED_PYTHON();
    StepRig rig;
    const fs::path log = rig.tmp.path() / "calls.log";
    const EnvGuard guard("FAKE_STEP_LOG", log.string());
    const fs::path in = rig.step_file();
    const auto run = [&](const char* deflection) {
        return rig.run(rig.with_converter({"--import", in.string(), "--store", rig.store().string(), "--deflection", deflection}));
    };
    const Out first = run("0.1mm");
    ASSERT_EQ(first.exit, 0) << first.result.dump();
    EXPECT_EQ(rig.converter_runs(log), 1);
    const Out again = run("0.1mm");
    ASSERT_EQ(again.exit, 0) << again.result.dump();
    EXPECT_TRUE(again.result.value("cached", false));
    EXPECT_EQ(rig.converter_runs(log), 1) << "a cache hit ran the converter";
    const Out other = run("0.05mm");
    ASSERT_EQ(other.exit, 0) << other.result.dump();
    EXPECT_FALSE(other.result.value("cached", true));
    EXPECT_NE(other.result.value("key", std::string()), first.result.value("key", std::string()));
    EXPECT_EQ(rig.converter_runs(log), 2);
}

TEST(PartImportStep, AnObjectOfTheStepIsPassedAsASolidAndOnlyItIsStored) {
    NEED_PYTHON();
    StepRig rig;
    const fs::path log = rig.tmp.path() / "calls.log";
    const EnvGuard guard("FAKE_STEP_LOG", log.string());
    const Out r = rig.run(rig.with_converter(
        {"--import", rig.step_file().string(), "--store", rig.store().string(), "--deflection", "0.1mm", "--object", "hub"}));
    ASSERT_EQ(r.exit, 0) << r.result.dump();
    ASSERT_EQ(r.result.at("objects").size(), 1u);
    EXPECT_EQ(r.result.at("objects")[0].value("name", std::string()), "hub");
    std::ifstream in(log);
    std::string line;
    std::getline(in, line);
    EXPECT_NE(line.find("\"--solid\", \"hub\""), std::string::npos) << line;
    EXPECT_NE(line.find("\"--deflection\", \"0.1mm\""), std::string::npos) << line;
}

TEST(PartImportStep, AFlaggedCrossCheckIsWrittenAllTheSameWithExitSix) {
    NEED_PYTHON();
    const EnvGuard mode("FAKE_STEP_MODE", "flag");
    StepRig rig;
    const fs::path log = rig.tmp.path() / "calls.log";
    const EnvGuard guard("FAKE_STEP_LOG", log.string());
    const Out r = rig.run(rig.with_converter({"--import", rig.step_file().string(), "--store", rig.store().string(), "--deflection", "0.1mm"}));
    EXPECT_EQ(r.exit, 6);
    EXPECT_FALSE(r.result.value("ok", true));
    EXPECT_EQ(r.code(), "cross_check_flagged");
    EXPECT_NE(r.result.value("message", std::string()).find("arm"), std::string::npos);
    ASSERT_EQ(r.result.at("objects").size(), 2u);
    EXPECT_TRUE(fs::is_regular_file(fs::path(r.result.at("objects")[0].value("sidecar", std::string()))));
    EXPECT_EQ(r.result.at("objects")[0].at("cross_check").value("flagged", false), true);
    // the next run is a cache hit and is flagged the same: the flag is kept with the entry, and the converter does not run again
    const Out again = rig.run(rig.with_converter({"--import", rig.step_file().string(), "--store", rig.store().string(), "--deflection", "0.1mm"}));
    EXPECT_EQ(again.exit, 6) << again.result.dump();
    EXPECT_TRUE(again.result.value("cached", false));
    EXPECT_FALSE(again.result.value("ok", true));
    EXPECT_EQ(again.code(), "cross_check_flagged");
    EXPECT_NE(again.result.value("message", std::string()).find("arm"), std::string::npos);
    ASSERT_EQ(again.result.at("objects").size(), 2u);
    EXPECT_EQ(again.result.at("objects")[0].at("cross_check").value("flagged", false), true);
    EXPECT_EQ(again.result.at("objects")[1].at("cross_check").value("flagged", true), false);
    EXPECT_EQ(rig.converter_runs(log), 1) << "a cache hit ran the converter";
}

TEST(PartImportStep, TheConvertersOwnRefusalsAndFailuresKeepTheirMeaningAndStoreNothing) {
    NEED_PYTHON();
    struct Case {
        const char* mode;
        int exit;
        const char* code;
        const char* in_message;
    };
    const Case cases[] = {
        {"refuse", 4, "mesh_refused", "not closed"},
        {"unreadable", 4, "file_invalid", "not a STEP file"},
        {"noocp", 3, "step_unavailable", "not installed"},
        {"garbage", 3, "converter_failed", "JSON"},
    };
    for (const Case& c : cases) {
        const EnvGuard mode("FAKE_STEP_MODE", c.mode);
        StepRig rig;
        const Out r =
            rig.run(rig.with_converter({"--import", rig.step_file().string(), "--store", rig.store().string(), "--deflection", "0.1mm"}));
        EXPECT_EQ(r.exit, c.exit) << c.mode << ": " << r.result.dump();
        EXPECT_EQ(r.code(), c.code) << c.mode;
        EXPECT_NE(r.result.value("message", std::string()).find(c.in_message), std::string::npos) << c.mode << ": " << r.result.dump();
        int files = 0;
        if (fs::exists(rig.store())) {
            for (const auto& e : fs::recursive_directory_iterator(rig.store())) files += e.is_regular_file() ? 1 : 0;
        }
        EXPECT_EQ(files, 0) << c.mode;
    }
}

TEST(PartImportStep, ACancelWhileTheConverterRunsKillsItAndLeavesNothingBehind) {
    NEED_PYTHON();
    const EnvGuard mode("FAKE_STEP_MODE", "slow");
    StepRig rig;
    const fs::path in = rig.step_file();
    std::thread later([&] {
        std::this_thread::sleep_for(std::chrono::milliseconds(1500));
        rig.cancel = true;
    });
    const auto t0 = std::chrono::steady_clock::now();
    const Out r = rig.run(rig.with_converter({"--import", in.string(), "--store", rig.store().string(), "--deflection", "0.1mm"}));
    later.join();
    EXPECT_EQ(r.exit, 5) << r.result.dump();
    EXPECT_EQ(r.code(), "cancelled");
    EXPECT_LT(std::chrono::steady_clock::now() - t0, std::chrono::seconds(30));
    int files = 0;
    if (fs::exists(rig.store())) {
        for (const auto& e : fs::recursive_directory_iterator(rig.store())) files += e.is_regular_file() ? 1 : 0;
    }
    EXPECT_EQ(files, 0);
    EXPECT_EQ(r.lines.back().value("event", std::string()), "result");
}

}  // namespace
