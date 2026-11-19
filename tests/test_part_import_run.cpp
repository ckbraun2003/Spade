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
    std::string text;  // lines of out that are not JSON (only --help has any), when the caller said they may be
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
    Out run(std::vector<std::string> args, bool text_ok = false) {
        std::ostringstream out, err;
        Out r;
        r.exit = run_part_import(args, env, out, err);
        r.out = out.str();
        r.err = err.str();
        std::istringstream in(r.out);
        for (std::string l; std::getline(in, l);) {
            json j = json::parse(l, nullptr, false);
            if (j.is_discarded() && text_ok) {
                r.text += l + "\n";
                continue;
            }
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

// --help is a request, not a usage problem: plain usage text on standard output and exit 0, with no JSON lines at all (the one exception to
// "standard output is JSON lines"; a machine never calls --help) and nothing on standard error, so a shell that wraps stderr as an error
// record sees none. A usage ERROR keeps standard error, its JSON result line and exit 2.
TEST(PartImportRun, HelpPrintsPlainUsageOnStandardOutputWithNoJsonAndExitsZero) {
    Rig rig;
    for (const char* flag : {"--help", "-h"}) {
        const Out r = rig.run(A({flag}), true);
        EXPECT_EQ(r.exit, 0) << flag;
        EXPECT_TRUE(r.err.empty()) << flag << ": " << r.err;
        EXPECT_TRUE(r.lines.empty()) << flag << " printed a JSON line: " << r.out;
        for (const char* word : {"usage: spade_part_import", "--inspect", "--import", "--store", "--axes", "--object"}) {
            EXPECT_NE(r.text.find(word), std::string::npos) << flag << " lacks " << word << " on standard output";
        }
        EXPECT_EQ(r.text.find('{'), std::string::npos) << flag << ": no JSON in the help";
    }
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

TEST(PartImportRun, AnUnsupportedExtensionNamesItsFormatWhereKnownAndSaysWhatToConvertTo) {
    Rig rig;
    const std::pair<const char*, const char*> known[] = {
        {"frame.dae", "COLLADA"}, {"FRAME.DAE", "COLLADA"}, {"a.fbx", "FBX"},    {"a.blend", "Blender"}, {"a.ply", "PLY"},
        {"a.3ds", "3ds"},         {"a.iges", "IGES"},       {"a.igs", "IGES"},   {"a.skp", "SketchUp"},  {"a.usdz", "USD"},
        {"a.x3d", "X3D"},         {"a.dxf", "DXF"},         {"a.amf", "AMF"},    {"a.off", "OFF"},
    };
    for (const auto& [file, name] : known) {
        const Out r = rig.run(A({"--inspect", (rig.tmp.path() / "in" / file).string()}));
        EXPECT_EQ(r.exit, 3) << file;
        EXPECT_EQ(r.code(), "format_unknown") << file;
        const std::string message = r.result.value("message", std::string());
        EXPECT_NE(message.find(name), std::string::npos) << file << ": " << message;
        EXPECT_NE(message.find("convert"), std::string::npos) << file << ": " << message;
        for (const char* target : {"glb", "OBJ", "STL", "3MF", "STEP"}) {
            EXPECT_NE(message.find(target), std::string::npos) << file << " should say " << target << ": " << message;
        }
    }
    // an extension nobody here knows still says what is read, and to convert
    const Out unknown = rig.run(A({"--inspect", (rig.tmp.path() / "in" / "a.xyz").string()}));
    EXPECT_EQ(unknown.exit, 3);
    EXPECT_EQ(unknown.code(), "format_unknown");
    EXPECT_NE(unknown.result.value("message", std::string()).find("convert"), std::string::npos) << unknown.out;
    EXPECT_NE(unknown.result.value("message", std::string()).find(".xyz"), std::string::npos) << unknown.out;
    // no extension at all
    const Out bare = rig.run(A({"--inspect", (rig.tmp.path() / "in" / "noextension").string()}));
    EXPECT_EQ(bare.code(), "format_unknown");
    // a supported one is not touched by this
    const Out ok = rig.run(A({"--inspect", rig.put("cube.stl", cube_stl(1.0f)).string(), "--unit", "m"}));
    EXPECT_EQ(ok.exit, 0) << ok.out;
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

TEST(PartImportInspect, EveryObjectOfAFileGetsItsOwnGateResult) {
    Rig rig;
    const std::string bytes = three_mf({cube_object(1, "a", 10.0), cube_object(2, "b", 10.0, 40.0, 11), cube_object(3, "c", 10.0, 80.0)});
    const Out r = rig.run(A({"--inspect", rig.put("three.3mf", bytes).string()}));
    EXPECT_EQ(r.exit, 4) << r.out;
    EXPECT_FALSE(r.ok());
    EXPECT_EQ(r.result.value("gate", std::string()), "refused");
    EXPECT_EQ(r.code(), "not_closed");
    ASSERT_EQ(r.result.at("objects").size(), 3u) << r.out;
    const std::vector<bool> expected_ok = {true, false, true};
    for (std::size_t i = 0; i < 3; ++i) {
        const json& o = r.result.at("objects")[i];
        EXPECT_EQ(o.value("ok", !expected_ok[i]), expected_ok[i]) << o.dump();
        if (expected_ok[i]) {
            EXPECT_EQ(o.value("triangles", 0), 12);
        } else {
            EXPECT_EQ(o.value("code", std::string()), "not_closed");
            EXPECT_FALSE(o.value("message", std::string()).empty());
            EXPECT_FALSE(o.contains("triangles"));
        }
    }
    EXPECT_NE(r.result.value("message", std::string()).find("b"), std::string::npos);
    expect_json_lines_discipline(r);
    // an OBJ whose FIRST object is the open one
    std::string obj;
    uint32_t base = 0;
    for (const auto& [name, faces] : {std::pair<const char*, std::size_t>{"first", 11}, {"second", 12}}) {
        obj += std::string("o ") + name + "\n";
        for (uint32_t v = 0; v < 8; ++v) {
            const glm::vec3 c = corner(v, 1.0f);
            obj += "v " + std::to_string(c.x + static_cast<float>(base)) + " " + std::to_string(c.y) + " " + std::to_string(c.z) + "\n";
        }
        for (std::size_t i = 0; i < faces; ++i) {
            obj += "f " + std::to_string(kCube[i][0] + 1 + base) + " " + std::to_string(kCube[i][1] + 1 + base) + " " +
                   std::to_string(kCube[i][2] + 1 + base) + "\n";
        }
        base += 8;
    }
    const Out o = rig.run(A({"--inspect", rig.put("two.obj", obj).string(), "--unit", "m"}));
    EXPECT_EQ(o.exit, 4) << o.out;
    ASSERT_EQ(o.result.at("objects").size(), 2u);
    EXPECT_FALSE(o.result.at("objects")[0].value("ok", true));
    EXPECT_TRUE(o.result.at("objects")[1].value("ok", false));
}

TEST(PartImportImport, TheGateIsRunOnThePickedObjectsOnlyAndARefusalIsItsKindWithNothingStored) {
    Rig rig;
    const std::string bytes = three_mf({cube_object(1, "a", 10.0), cube_object(2, "b", 10.0, 40.0, 11), cube_object(3, "c", 10.0, 80.0)});
    const fs::path in = rig.put("three.3mf", bytes);
    // all of it: refused, naming the object, and nothing stored
    Out r = rig.run(A({"--import", in.string(), "--store", rig.store().string()}));
    EXPECT_EQ(r.exit, 4) << r.out;
    EXPECT_EQ(r.code(), "not_closed");
    EXPECT_NE(r.result.value("message", std::string()).find("b"), std::string::npos);
    EXPECT_FALSE(fs::exists(rig.store() / sha256_hex(bytes)));
    // the refused one picked: the same
    r = rig.run(A({"--import", in.string(), "--store", rig.store().string(), "--object", "b"}));
    EXPECT_EQ(r.exit, 4) << r.out;
    EXPECT_EQ(r.code(), "not_closed");
    // a good one picked: stored, the open sibling does not stop it
    r = rig.run(A({"--import", in.string(), "--store", rig.store().string(), "--object", "c"}));
    ASSERT_EQ(r.exit, 0) << r.out;
    ASSERT_EQ(r.result.at("objects").size(), 1u);
    EXPECT_EQ(r.result.at("objects")[0].value("name", std::string()), "c");
}

// A cube turned inside out, then an open one: the file's code is the first refused object's kind, and every object has its own.
TEST(PartImportInspect, EachRefusedObjectHasItsOwnKindAndTheFileTakesTheFirst) {
    Rig rig;
    std::string obj;
    uint32_t base = 0;
    for (const auto& [name, faces] : {std::pair<const char*, std::size_t>{"turned", 12}, {"open", 11}}) {
        obj += std::string("o ") + name + "\n";
        for (uint32_t v = 0; v < 8; ++v) {
            const glm::vec3 c = corner(v, 1.0f);
            obj += "v " + std::to_string(c.x + static_cast<float>(base)) + " " + std::to_string(c.y) + " " + std::to_string(c.z) + "\n";
        }
        for (std::size_t i = 0; i < faces; ++i) {
            const bool turn = std::string(name) == "turned";
            obj += "f " + std::to_string(kCube[i][0] + 1 + base) + " " + std::to_string(kCube[i][turn ? 2 : 1] + 1 + base) + " " +
                   std::to_string(kCube[i][turn ? 1 : 2] + 1 + base) + "\n";
        }
        base += 8;
    }
    const fs::path in = rig.put("kinds.obj", obj);
    const Out r = rig.run(A({"--inspect", in.string(), "--unit", "m"}));
    EXPECT_EQ(r.exit, 4) << r.out;
    EXPECT_EQ(r.code(), "inside_out");
    ASSERT_EQ(r.result.at("objects").size(), 2u) << r.out;
    EXPECT_EQ(r.result.at("objects")[0].value("code", std::string()), "inside_out");
    EXPECT_EQ(r.result.at("objects")[1].value("code", std::string()), "not_closed");
    EXPECT_NE(r.result.value("message", std::string()).find("1 more"), std::string::npos) << r.out;
    // the import is refused with the same code and stores nothing
    const Out i = rig.run(A({"--import", in.string(), "--unit", "m", "--store", rig.store().string()}));
    EXPECT_EQ(i.exit, 4) << i.out;
    EXPECT_EQ(i.code(), "inside_out");
    EXPECT_FALSE(fs::exists(rig.store() / sha256_hex(obj)));
    // the open one alone is its own kind
    const Out o = rig.run(A({"--import", in.string(), "--unit", "m", "--store", rig.store().string(), "--object", "open"}));
    EXPECT_EQ(o.code(), "not_closed");
}

TEST(PartImportImport, SeveralObjectsCanBePickedAndAreStoredAsOneEntryWhateverTheOrderOfThePicks) {
    Rig rig;
    const std::string bytes = three_mf({cube_object(1, "a", 10.0), cube_object(2, "b", 10.0, 40.0, 11), cube_object(3, "c", 10.0, 80.0)});
    const fs::path in = rig.put("three.3mf", bytes);
    // b is the open one: picking a and c leaves it alone
    Out r = rig.run(A({"--import", in.string(), "--store", rig.store().string(), "--object", "a", "--object", "c"}));
    ASSERT_EQ(r.exit, 0) << r.out;
    EXPECT_FALSE(r.result.value("cached", true));
    ASSERT_EQ(r.result.at("objects").size(), 2u);
    EXPECT_EQ(r.result.at("objects")[0].value("name", std::string()), "a");
    EXPECT_EQ(r.result.at("objects")[1].value("name", std::string()), "c");
    // the same picks in the other order, and a name twice, are the same entry: a cache hit
    r = rig.run(A({"--import", in.string(), "--store", rig.store().string(), "--object", "c", "--object", "a", "--object", "c"}));
    ASSERT_EQ(r.exit, 0) << r.out;
    EXPECT_TRUE(r.result.value("cached", false));
    EXPECT_EQ(r.result.at("objects").size(), 2u);
    // one pick is its own entry, not a hit of the pair
    r = rig.run(A({"--import", in.string(), "--store", rig.store().string(), "--object", "a"}));
    ASSERT_EQ(r.exit, 0) << r.out;
    EXPECT_FALSE(r.result.value("cached", true));
    EXPECT_EQ(r.result.at("objects").size(), 1u);
    // a refused one among the picks refuses the import; an unknown name is object_not_found
    r = rig.run(A({"--import", in.string(), "--store", rig.store().string(), "--object", "a", "--object", "b"}));
    EXPECT_EQ(r.exit, 4) << r.out;
    EXPECT_EQ(r.code(), "not_closed");
    r = rig.run(A({"--import", in.string(), "--store", rig.store().string(), "--object", "a", "--object", "nothing"}));
    EXPECT_EQ(r.code(), "object_not_found");
    EXPECT_NE(r.result.value("message", std::string()).find("nothing"), std::string::npos);
    // an inspect takes the same list
    r = rig.run(A({"--inspect", in.string(), "--object", "a", "--object", "c"}));
    EXPECT_EQ(r.exit, 0) << r.out;
    EXPECT_EQ(r.result.at("objects").size(), 2u);
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
