#include <gtest/gtest.h>

#include <chrono>
#include <filesystem>
#include <map>
#include <set>
#include <sstream>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include "../tools/part_import/options.hpp"
#include "../tools/part_import/report.hpp"
#include "../tools/part_import/run.hpp"
#include "part_import_fixtures.hpp"
#include "unique_temp.hpp"
#include "world/mesh_file.hpp"
#include "world/sha256.hpp"

// spade_part_import, in-process on a temporary store (L88; docs/design/interface/plans/2026-11-17-part-import-design.md): --inspect and
// --import for the mesh formats, the options, every usage code, the JSON lines, the cache, cancel.

namespace {

namespace fs = std::filesystem;
using nlohmann::json;
using namespace spade;
using namespace spade::partimport;
using namespace spade::partimporttest;

struct Out {
    int exit = -1;
    std::vector<json> lines;
    json result;
    std::string out, err;
    [[nodiscard]] bool ok() const { return result.value("ok", false); }
    [[nodiscard]] std::string code() const { return result.value("code", std::string("?")); }
};

struct Rig {
    test::TempDir tmp{"part_import_run"};
    std::map<std::string, std::string> vars;
    bool cancel = false;
    RunEnv env;

    Rig() {
        env.getenv = [this](const char* name) -> const char* {
            const auto it = vars.find(name);
            return it == vars.end() ? nullptr : it->second.c_str();
        };
        env.exe_dir = tmp.path() / "exe";
        env.cancelled = [this] { return cancel; };
        env.token = "t1";
    }
    [[nodiscard]] fs::path store() const { return tmp.path() / "store"; }
    [[nodiscard]] fs::path put(const std::string& name, std::string_view bytes) const {
        const fs::path p = tmp.path() / "in" / name;
        write_file(p, bytes);
        return p;
    }
    Out run(std::vector<std::string> args) {
        std::ostringstream out, err;
        Out r;
        r.exit = run_part_import(args, env, out, err);
        r.out = out.str();
        r.err = err.str();
        std::istringstream in(r.out);
        for (std::string l; std::getline(in, l);) {
            json j = json::parse(l, nullptr, false);
            EXPECT_FALSE(j.is_discarded()) << "not JSON: " << l;
            r.lines.push_back(j);
        }
        if (!r.lines.empty()) r.result = r.lines.back();
        return r;
    }
};

[[nodiscard]] std::vector<std::string> A(std::initializer_list<std::string> a) { return {a}; }

// Every line is a stage or the one result, and the result is last.
void expect_json_lines_discipline(const Out& r) {
    ASSERT_FALSE(r.lines.empty()) << r.out;
    int results = 0;
    for (const json& j : r.lines) {
        ASSERT_TRUE(j.is_object());
        const std::string ev = j.value("event", std::string());
        EXPECT_TRUE(ev == "stage" || ev == "progress" || ev == "result") << j.dump();
        if (ev == "result") ++results;
    }
    EXPECT_EQ(results, 1);
    EXPECT_EQ(r.lines.back().value("event", std::string()), "result");
    EXPECT_EQ(r.result.value("exit", -1), r.exit);
}

// ---- usage ---------------------------------------------------------------------------------------------------------------------

TEST(PartImportRun, ACommandLineThatIsNoModeIsAUsageProblemWithOnlyTheResultLineOnStandardOut) {
    Rig rig;
    const std::vector<std::vector<std::string>> bad = {A({}),
                                                       A({"--inspect"}),
                                                       A({"--inspect", "a.stl", "--import", "b.stl"}),
                                                       A({"--bogus"}),
                                                       A({"--inspect", "a.stl", "--unit"}),
                                                       A({"--probe", "extra"})};
    for (const auto& args : bad) {
        const Out r = rig.run(args);
        EXPECT_EQ(r.exit, 2) << r.out;
        EXPECT_FALSE(r.ok());
        EXPECT_EQ(r.code(), "usage");
        EXPECT_FALSE(r.result.value("message", std::string()).empty());
        EXPECT_FALSE(r.result.value("advice", std::string()).empty());
        EXPECT_EQ(r.lines.size(), 1u) << r.out;
        EXPECT_NE(r.err.find("usage: spade_part_import"), std::string::npos) << r.err;
        expect_json_lines_discipline(r);
    }
}

TEST(PartImportRun, HelpPrintsTheUsageOnStandardErrorAndExitsZero) {
    Rig rig;
    const Out r = rig.run(A({"--help"}));
    EXPECT_EQ(r.exit, 0);
    EXPECT_TRUE(r.ok());
    EXPECT_EQ(r.result.value("mode", std::string()), "help");
    EXPECT_NE(r.err.find("--inspect"), std::string::npos);
    EXPECT_NE(r.err.find("--import"), std::string::npos);
    EXPECT_NE(r.err.find("--store"), std::string::npos);
    EXPECT_NE(r.err.find("--axes"), std::string::npos);
}

TEST(PartImportRun, EveryUsageCodeHasItsExitCodeAndNothingIsWritten) {
    struct Case {
        const char* what;
        const char* file;
        std::string bytes;
        std::vector<std::string> args;
        const char* code;
    };
    const std::string stl = cube_stl(10);
    const std::string glb = cube_glb();
    const Case cases[] = {
        {"no store", "cube.stl", stl, A({"--unit", "mm"}), "store_required"},
        {"stl without a unit", "cube.stl", stl, A({"--store", "S"}), "unit_required"},
        {"a unit for a glb", "cube.glb", glb, A({"--store", "S", "--unit", "mm"}), "unit_not_allowed"},
        {"a mirror", "cube.glb", glb, A({"--store", "S", "--axes", "y,x,z"}), "axes_mirror"},
        {"axes twice the same", "cube.glb", glb, A({"--store", "S", "--axes", "x,x,z"}), "axes_invalid"},
        {"a scale of zero", "cube.glb", glb, A({"--store", "S", "--scale", "0"}), "scale_invalid"},
        {"a unit that is no unit", "cube.stl", stl, A({"--store", "S", "--unit", "parsec"}), "usage"},
        {"a step without a deflection", "cube.step", "ISO-10303-21;", A({"--store", "S"}), "deflection_required"},
        {"an unknown name", "cube.glb", glb, A({"--store", "S", "--object", "nothing"}), "object_not_found"},
    };
    for (const Case& c : cases) {
        Rig rig;
        const fs::path in = rig.put(c.file, c.bytes);
        std::vector<std::string> args = {"--import", in.string()};
        for (std::string a : c.args) {
            if (a == "S") a = rig.store().string();
            args.push_back(a);
        }
        const Out r = rig.run(args);
        EXPECT_EQ(r.result.value("code", std::string()), c.code) << c.what << ": " << r.out;
        EXPECT_EQ(r.exit, code_info(c.code).exit) << c.what;
        EXPECT_FALSE(r.result.value("message", std::string()).empty()) << c.what;
        EXPECT_FALSE(r.result.value("advice", std::string()).empty()) << c.what;
        EXPECT_FALSE(fs::exists(rig.store())) << c.what << ": the store was created";
        expect_json_lines_discipline(r);
    }
}

TEST(PartImportRun, TheEnvironmentVariableIsOnlyTheDefaultForTheStore) {
    Rig rig;
    const fs::path in = rig.put("cube.stl", cube_stl(10));
    const fs::path from_env = rig.tmp.path() / "env-store";
    rig.vars["SPADE_PART_STORE"] = from_env.string();
    Out r = rig.run(A({"--import", in.string(), "--unit", "mm"}));
    ASSERT_EQ(r.exit, 0) << r.out;
    EXPECT_TRUE(fs::exists(from_env));
    EXPECT_EQ(fs::path(r.result.value("store", std::string())), from_env);
    // --store wins
    r = rig.run(A({"--import", in.string(), "--unit", "mm", "--store", rig.store().string()}));
    ASSERT_EQ(r.exit, 0) << r.out;
    EXPECT_TRUE(fs::exists(rig.store()));
}

TEST(PartImportRun, AStoreInsideAGitWorkTreeIsRefused) {
    Rig rig;
    const fs::path in = rig.put("cube.stl", cube_stl(10));
    fs::create_directories(rig.tmp.path() / "work" / ".git");
    const Out r = rig.run(A({"--import", in.string(), "--unit", "mm", "--store", (rig.tmp.path() / "work" / "store").string()}));
    EXPECT_EQ(r.exit, 2);
    EXPECT_EQ(r.code(), "store_in_git_tree");
    EXPECT_FALSE(fs::exists(rig.tmp.path() / "work" / "store"));
}

TEST(PartImportRun, AFileThatIsMissingOrOfAnUnknownTypeOrNotSelfContainedIsNamed) {
    Rig rig;
    Out r = rig.run(A({"--inspect", (rig.tmp.path() / "in" / "absent.stl").string(), "--unit", "mm"}));
    EXPECT_EQ(r.exit, 1);
    EXPECT_EQ(r.code(), "file_unreadable");
    r = rig.run(A({"--inspect", rig.put("part.xyz", "bytes").string()}));
    EXPECT_EQ(r.exit, 3);
    EXPECT_EQ(r.code(), "format_unknown");
    r = rig.run(A({"--import", rig.put("ext.gltf", gltf_with_external_buffer()).string(), "--store", rig.store().string()}));
    EXPECT_EQ(r.exit, 3);
    EXPECT_EQ(r.code(), "external_buffers");
    EXPECT_FALSE(fs::exists(rig.store()));
}

TEST(PartImportRun, AStepFileWithNoConverterIsStepUnavailable) {
    Rig rig;
    const Out r = rig.run(A({"--inspect", rig.put("part.step", "ISO-10303-21;").string()}));
    EXPECT_EQ(r.exit, 3);
    EXPECT_EQ(r.code(), "step_unavailable");
    EXPECT_FALSE(r.result.value("advice", std::string()).empty());
}

// ---- inspect -------------------------------------------------------------------------------------------------------------------

[[nodiscard]] std::vector<double> numbers(const json& a) {
    std::vector<double> v;
    for (const json& x : a) v.push_back(x.get<double>());
    return v;
}

TEST(PartImportInspect, EachFormatGivesItsObjectsWithBoxVolumeAndHash) {
    struct Case {
        const char* file;
        std::string bytes;
        std::vector<std::string> extra;
        const char* format;
        std::vector<std::pair<std::string, double>> objects;  // name, edge length in metres
    };
    const Case cases[] = {
        {"cube.stl", cube_stl(10), {"--unit", "mm"}, "stl", {{"cube", 0.010}}},
        {"cube.obj", cube_obj(10), {"--unit", "mm"}, "obj", {{"cube", 0.010}}},
        {"cube.glb", cube_glb(2.0f, "frame"), {}, "glb", {{"frame", 2.0}}},
        {"two.3mf", three_mf({cube_object(1, "arm left", 10.0), cube_object(2, "hub", 20.0, 40.0)}), {}, "3mf",
         {{"arm left", 0.010}, {"hub", 0.020}}},
    };
    for (const Case& c : cases) {
        Rig rig;
        const fs::path in = rig.put(c.file, c.bytes);
        std::vector<std::string> args = {"--inspect", in.string()};
        args.insert(args.end(), c.extra.begin(), c.extra.end());
        const Out r = rig.run(args);
        ASSERT_EQ(r.exit, 0) << c.file << ": " << r.out;
        expect_json_lines_discipline(r);
        EXPECT_TRUE(r.ok());
        EXPECT_EQ(r.result.value("mode", std::string()), "inspect");
        EXPECT_EQ(r.result.value("format", std::string()), c.format);
        EXPECT_EQ(r.result.value("sha256", std::string()), sha256_hex(c.bytes));
        EXPECT_EQ(r.result.value("bytes", 0u), c.bytes.size());
        EXPECT_EQ(r.result.value("gate", std::string()), "passed");
        ASSERT_EQ(r.result.at("objects").size(), c.objects.size()) << c.file;
        for (std::size_t i = 0; i < c.objects.size(); ++i) {
            const json& o = r.result.at("objects")[i];
            EXPECT_EQ(o.value("name", std::string()), c.objects[i].first);
            EXPECT_TRUE(o.value("ok", false));
            EXPECT_EQ(o.value("triangles", 0), 12);
            EXPECT_EQ(o.value("vertices", 0), 8);
            EXPECT_EQ(o.value("shells", 0), 1);
            EXPECT_EQ(o.value("content_hash", std::string()).size(), 16u);
            const double edge = c.objects[i].second;
            const auto lo = numbers(o.at("bbox").at("min")), hi = numbers(o.at("bbox").at("max"));
            ASSERT_EQ(lo.size(), 3u);
            ASSERT_EQ(hi.size(), 3u);
            for (int k = 0; k < 3; ++k) EXPECT_NEAR(hi[k] - lo[k], edge, edge * 1e-6) << c.file << " axis " << k;
            EXPECT_NEAR(o.value("volume", 0.0), edge * edge * edge, edge * edge * edge * 1e-5) << c.file;
        }
    }
}

TEST(PartImportInspect, TheUnitIsDeclaredOrGivenOrGuessedAndMeasuresSayWhichUnitTheyAreIn) {
    Rig rig;
    // a 3MF declares its own: millimetre
    Out r = rig.run(A({"--inspect", rig.put("a.3mf", three_mf({cube_object(1, "a", 10.0)})).string()}));
    ASSERT_EQ(r.exit, 0) << r.out;
    EXPECT_EQ(r.result.at("unit").value("declared", std::string()), "mm");
    EXPECT_EQ(r.result.at("measures_in").get<std::string>(), "m");
    // an STL with --unit: used, not declared
    r = rig.run(A({"--inspect", rig.put("b.stl", cube_stl(100)).string(), "--unit", "mm"}));
    EXPECT_TRUE(r.result.at("unit").at("declared").is_null());
    EXPECT_EQ(r.result.at("unit").value("used", std::string()), "mm");
    EXPECT_EQ(r.result.at("measures_in").get<std::string>(), "m");
    // an STL with none: the file's own numbers, and a guess
    r = rig.run(A({"--inspect", rig.put("c.stl", cube_stl(100)).string()}));
    ASSERT_EQ(r.exit, 0) << r.out;
    EXPECT_TRUE(r.result.at("unit").at("used").is_null());
    EXPECT_EQ(r.result.at("measures_in").get<std::string>(), "file");
    EXPECT_EQ(r.result.at("unit").at("guess").value("unit", std::string()), "mm");
    EXPECT_FALSE(r.result.at("unit").at("guess").value("basis", std::string()).empty());
    EXPECT_NEAR(numbers(r.result.at("objects")[0].at("bbox").at("max"))[0], 100.0, 1e-4);
    r = rig.run(A({"--inspect", rig.put("d.stl", cube_stl(1)).string()}));
    EXPECT_EQ(r.result.at("unit").at("guess").value("unit", std::string()), "m");
    r = rig.run(A({"--inspect", rig.put("e.stl", cube_stl(0.001f)).string()}));
    EXPECT_TRUE(r.result.at("unit").at("guess").is_null());
}

TEST(PartImportInspect, WritesNothingAnywhere) {
    Rig rig;
    const fs::path in = rig.put("cube.stl", cube_stl(10));
    const auto listing = [&] {
        std::set<std::string> files;
        for (const auto& e : fs::recursive_directory_iterator(rig.tmp.path())) files.insert(e.path().string());
        return files;
    };
    const auto before = listing();
    const Out r = rig.run(A({"--inspect", in.string(), "--unit", "mm", "--store", rig.store().string()}));
    EXPECT_EQ(r.exit, 0) << r.out;
    EXPECT_EQ(listing(), before);
    EXPECT_FALSE(fs::exists(rig.store()));
}

TEST(PartImportInspect, AnOpenMeshIsRefusedWithItsMessageAndAdvice) {
    Rig rig;
    const Out r = rig.run(A({"--inspect", rig.put("open.obj", cube_obj(1, 11)).string(), "--unit", "m"}));
    EXPECT_EQ(r.exit, 4);
    EXPECT_FALSE(r.ok());
    EXPECT_EQ(r.result.value("gate", std::string()), "refused");
    EXPECT_FALSE(r.result.value("message", std::string()).empty());
    EXPECT_FALSE(r.result.value("advice", std::string()).empty());
    expect_json_lines_discipline(r);
}

TEST(PartImportInspect, AxesAndScaleShowThePartFrame) {
    Rig rig;
    const Out r = rig.run(A({"--inspect", rig.put("cube.glb", cube_glb(2.0f)).string(), "--axes", "-y,x,z", "--scale", "0.5"}));
    ASSERT_EQ(r.exit, 0) << r.out;
    EXPECT_EQ(r.result.value("frame", std::string()), "part");
    const json& box = r.result.at("objects")[0].at("bbox");
    EXPECT_EQ(numbers(box.at("min")), (std::vector<double>{-1.0, 0.0, 0.0}));
    EXPECT_EQ(numbers(box.at("max")), (std::vector<double>{0.0, 1.0, 1.0}));
}

// ---- import --------------------------------------------------------------------------------------------------------------------

TEST(PartImportImport, AnStlGoesToTheStoreAsASidecarWithItsProvenanceAndTheSourceBesideIt) {
    Rig rig;
    const std::string bytes = cube_stl(10);
    const fs::path in = rig.put("cube.stl", bytes);
    const Out r = rig.run(A({"--import", in.string(), "--unit", "mm", "--store", rig.store().string()}));
    ASSERT_EQ(r.exit, 0) << r.out;
    expect_json_lines_discipline(r);
    EXPECT_TRUE(r.ok());
    EXPECT_EQ(r.result.value("mode", std::string()), "import");
    EXPECT_FALSE(r.result.value("cached", true));
    EXPECT_EQ(r.result.value("sha256", std::string()), sha256_hex(bytes));
    EXPECT_EQ(r.result.value("converter", std::string()), kMeshConverterVersion);
    const fs::path entry = r.result.value("entry", std::string());
    EXPECT_EQ(entry.parent_path(), rig.store() / sha256_hex(bytes));
    EXPECT_EQ(entry.filename().string(), r.result.value("key", std::string()));
    EXPECT_EQ(read_file(rig.store() / sha256_hex(bytes) / "cube.stl"), bytes);
    EXPECT_TRUE(fs::is_regular_file(entry / "import.json"));
    ASSERT_EQ(r.result.at("objects").size(), 1u);
    const json& o = r.result.at("objects")[0];
    EXPECT_EQ(o.value("name", std::string()), "cube");
    const auto loaded = load_mesh_file(fs::path(o.value("sidecar", std::string())));
    ASSERT_TRUE(loaded.has_value()) << loaded.error().context;
    EXPECT_EQ(loaded->provenance.unit, MeshUnit::millimetre);
    EXPECT_EQ(loaded->provenance.source, "cube.stl: cube");
    char hex[17];
    std::snprintf(hex, sizeof hex, "%016llx", static_cast<unsigned long long>(loaded->mesh->content_hash()));
    EXPECT_EQ(o.value("content_hash", std::string()), hex);
    // the sidecar is also a store file of its own, named by store_ref
    ASSERT_TRUE(o.contains("store_ref")) << o.dump();
    const fs::path addressed = rig.store() / o.at("store_ref").value("sha256", std::string()) / o.at("store_ref").value("file", std::string());
    EXPECT_EQ(o.at("store_ref").value("sha256", std::string()), sha256_hex(read_file(fs::path(o.value("sidecar", std::string())))));
    EXPECT_TRUE(load_mesh_file(addressed).has_value());
    std::set<std::string> stages;
    for (const json& l : r.lines) {
        if (l.value("event", std::string()) == "stage") stages.insert(l.value("stage", std::string()));
    }
    EXPECT_EQ(stages, (std::set<std::string>{"read", "check", "write"}));
}

TEST(PartImportImport, ASecondRunIsACacheHitAndEachOptionThatChangesASidecarIsAMiss) {
    Rig rig;
    const fs::path in = rig.put("cube.glb", cube_glb());
    const auto run = [&](std::vector<std::string> extra) {
        std::vector<std::string> args = {"--import", in.string(), "--store", rig.store().string()};
        args.insert(args.end(), extra.begin(), extra.end());
        return rig.run(args);
    };
    const Out first = run({});
    ASSERT_EQ(first.exit, 0) << first.out;
    EXPECT_FALSE(first.result.value("cached", true));
    const Out again = run({});
    ASSERT_EQ(again.exit, 0) << again.out;
    EXPECT_TRUE(again.result.value("cached", false));
    EXPECT_EQ(again.result.at("objects")[0].at("store_ref"), first.result.at("objects")[0].at("store_ref"));
    EXPECT_EQ(again.result.value("key", std::string()), first.result.value("key", std::string()));
    EXPECT_EQ(again.result.at("objects")[0].value("content_hash", std::string()), first.result.at("objects")[0].value("content_hash", std::string()));
    std::set<std::string> keys = {first.result.value("key", std::string())};
    for (const std::vector<std::string>& extra : {std::vector<std::string>{"--scale", "2"}, {"--axes", "-y,x,z"}, {"--object", "cube"}}) {
        const Out r = run(extra);
        ASSERT_EQ(r.exit, 0) << r.out;
        EXPECT_FALSE(r.result.value("cached", true)) << extra[0];
        EXPECT_TRUE(keys.insert(r.result.value("key", std::string())).second) << extra[0] << " did not change the key";
    }
    int entries = 0;
    for (const auto& e : fs::directory_iterator(rig.store() / sha256_hex(cube_glb()))) entries += e.is_directory() ? 1 : 0;
    EXPECT_EQ(entries, 4);
}

TEST(PartImportImport, AxesAndScaleAreInTheStoredMesh) {
    Rig rig;
    const Out r = rig.run(A({"--import", rig.put("cube.glb", cube_glb(2.0f)).string(), "--store", rig.store().string(), "--axes", "-y,x,z",
                             "--scale", "0.5"}));
    ASSERT_EQ(r.exit, 0) << r.out;
    const auto loaded = load_mesh_file(fs::path(r.result.at("objects")[0].value("sidecar", std::string())));
    ASSERT_TRUE(loaded.has_value()) << loaded.error().context;
    glm::vec3 lo(1e9f), hi(-1e9f);
    for (const glm::vec3& p : loaded->mesh->positions()) {
        lo = glm::min(lo, p);
        hi = glm::max(hi, p);
    }
    EXPECT_EQ(lo, (glm::vec3{-1, 0, 0}));
    EXPECT_EQ(hi, (glm::vec3{0, 1, 1}));
}

TEST(PartImportImport, AnObjectOfAFileWithSeveralCanBePicked) {
    Rig rig;
    const std::string bytes = three_mf({cube_object(1, "arm left", 10.0), cube_object(2, "hub", 20.0, 40.0)});
    const fs::path in = rig.put("two.3mf", bytes);
    Out r = rig.run(A({"--import", in.string(), "--store", rig.store().string(), "--object", "hub"}));
    ASSERT_EQ(r.exit, 0) << r.out;
    ASSERT_EQ(r.result.at("objects").size(), 1u);
    EXPECT_EQ(r.result.at("objects")[0].value("name", std::string()), "hub");
    EXPECT_FALSE(fs::exists(fs::path(r.result.value("entry", std::string())) / "arm left.spmesh"));
    const auto loaded = load_mesh_file(fs::path(r.result.at("objects")[0].value("sidecar", std::string())));
    ASSERT_TRUE(loaded.has_value());
    EXPECT_EQ(loaded->provenance.unit, MeshUnit::millimetre);  // the model's own
    // an unknown name lists the ones the file has
    r = rig.run(A({"--import", in.string(), "--store", rig.store().string(), "--object", "nothing"}));
    EXPECT_EQ(r.exit, 2);
    EXPECT_EQ(r.code(), "object_not_found");
    EXPECT_NE(r.result.value("message", std::string()).find("arm left"), std::string::npos);
    EXPECT_NE(r.result.value("message", std::string()).find("hub"), std::string::npos);
}

TEST(PartImportImport, ARefusedFileStoresNothingNotEvenTheSource) {
    Rig rig;
    const Out r = rig.run(A({"--import", rig.put("open.obj", cube_obj(1, 11)).string(), "--unit", "m", "--store", rig.store().string()}));
    EXPECT_EQ(r.exit, 4);
    EXPECT_FALSE(r.ok());
    int files = 0;
    if (fs::exists(rig.store())) {
        for (const auto& e : fs::recursive_directory_iterator(rig.store())) files += e.is_regular_file() ? 1 : 0;
    }
    EXPECT_EQ(files, 0);
}

TEST(PartImportImport, ACancelBeforeTheWorkIsExitFiveWithNothingStored) {
    Rig rig;
    rig.cancel = true;
    const fs::path in = rig.put("cube.glb", cube_glb());
    Out r = rig.run(A({"--import", in.string(), "--store", rig.store().string()}));
    EXPECT_EQ(r.exit, 5);
    EXPECT_EQ(r.code(), "cancelled");
    EXPECT_FALSE(fs::exists(rig.store() / sha256_hex(cube_glb())));
    r = rig.run(A({"--inspect", in.string()}));
    EXPECT_EQ(r.exit, 5);
    EXPECT_EQ(r.code(), "cancelled");
    expect_json_lines_discipline(r);
}

TEST(PartImportImport, ALeftoverTemporaryFromAKilledRunIsRemovedAtTheNextImport) {
    Rig rig;
    const std::string bytes = cube_glb();
    const fs::path sha_dir = rig.store() / sha256_hex(bytes);
    write_file(sha_dir / ".tmp-9999-oldkey" / "x.spmesh", "half");
    fs::last_write_time(sha_dir / ".tmp-9999-oldkey", fs::file_time_type::clock::now() - std::chrono::hours(5));
    const Out r = rig.run(A({"--import", rig.put("cube.glb", bytes).string(), "--store", rig.store().string()}));
    ASSERT_EQ(r.exit, 0) << r.out;
    EXPECT_FALSE(fs::exists(sha_dir / ".tmp-9999-oldkey"));
}

}  // namespace
