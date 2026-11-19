// ---------------------------------------------------------------------------
// test_part_spec.cpp -- a part's flight spec beside its blocks (L103, L107; scene/part_spec.hpp): a blocks part (part_version 2) may carry the v1 part
// file's body, a motor, prop, esc or battery block with its `sources`, under a `spec:` key. Both readers share one file: the blocks reader ignores `spec`
// and the part-file reader reads it as the v1 file it is.
// ---------------------------------------------------------------------------
#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <iterator>
#include <string>
#include <variant>

#include "scene/part_blocks.hpp"
#include "scene/part_file.hpp"
#include "scene/part_spec.hpp"

namespace {

namespace fs = std::filesystem;
namespace sc = spade::scene;

std::string slurp(const fs::path& path) {
    std::ifstream in(path, std::ios::binary);
    return std::string(std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>());
}

// The v1 text of a library part, as the body of a `spec:` key: its `part_version` line dropped, every other line indented two spaces.
std::string as_spec(const std::string& v1_text) {
    std::string out = "spec:\n";
    std::size_t at = 0;
    while (at < v1_text.size()) {
        std::size_t end = v1_text.find('\n', at);
        if (end == std::string::npos) end = v1_text.size();
        std::string line = v1_text.substr(at, end - at);
        if (!line.empty() && line.back() == '\r') line.pop_back();
        at = end + 1;
        if (line.rfind("part_version:", 0) == 0 || line.empty() || line[0] == '#') continue;
        out += "  " + line + "\n";
    }
    return out;
}

const fs::path kLibrary = fs::path(SPADE_ASSETS_DIR) / "library" / "parts";
const std::string kBlocks = "part_version: 2\nid: \"part.motor.t\"\nmass_properties: {mass: 0.055}\nsources:\n  mass_properties.mass: {grade: assumed, note: \"a stand-in mass\"}\n";

TEST(PartSpec, APartWithNoSpecHasNone) {
    const auto r = sc::part_spec_from_text(kBlocks);
    ASSERT_TRUE(r.has_value()) << r.error().context;
    EXPECT_FALSE(r->has_value());
}

TEST(PartSpec, AMotorSpecBesideTheBlocksReadsAsTheV1PartItIs) {
    const std::string v1 = slurp(kLibrary / "iris_920kv.motor.yaml");
    ASSERT_FALSE(v1.empty());
    const auto want = sc::motor_from_yaml(v1);
    ASSERT_TRUE(want.has_value()) << want.error().context;
    const std::string text = kBlocks + as_spec(v1);
    const auto got = sc::part_spec_from_text(text);
    ASSERT_TRUE(got.has_value()) << got.error().context;
    ASSERT_TRUE(got->has_value());
    ASSERT_EQ(sc::part_kind(**got), sc::PartKind::motor);
    const sc::MotorPart& motor = std::get<sc::MotorPart>(**got);
    EXPECT_EQ(motor.block.kv, want->block.kv);
    EXPECT_EQ(motor.block.resistance, want->block.resistance);
    EXPECT_EQ(motor.block.rotor_inertia, want->block.rotor_inertia);
    EXPECT_EQ(motor.sources.size(), want->sources.size());
    // The blocks reader still reads the same file, and ignores the spec.
    const auto blocks = sc::read_part_blocks(text);
    ASSERT_TRUE(blocks.has_value()) << blocks.error().context;
    EXPECT_DOUBLE_EQ(blocks->mass_properties->mass, 0.055);
}

TEST(PartSpec, ASpecOfEachKindIsReadAsThatKind) {
    struct Case {
        const char* file;
        sc::PartKind kind;
    };
    for (const Case& c : {Case{"iris_920kv.motor.yaml", sc::PartKind::motor}, Case{"uiuc_apce_10x5.prop.yaml", sc::PartKind::prop},
                          Case{"iris_4x20a.esc.yaml", sc::PartKind::esc}, Case{"phantom3_4s_4480.battery.yaml", sc::PartKind::battery}}) {
        const std::string v1 = slurp(kLibrary / c.file);
        ASSERT_FALSE(v1.empty()) << c.file;
        const auto got = sc::part_spec_from_text(kBlocks + as_spec(v1));
        ASSERT_TRUE(got.has_value()) << c.file << ": " << got.error().context;
        ASSERT_TRUE(got->has_value()) << c.file;
        EXPECT_EQ(sc::part_kind(**got), c.kind) << c.file;
    }
}

TEST(PartSpec, ARefusalNamesTheSpec) {
    const std::string v1 = slurp(kLibrary / "iris_920kv.motor.yaml");
    // not a mapping
    const auto flat = sc::part_spec_from_text(kBlocks + "spec: 3\n");
    ASSERT_FALSE(flat.has_value());
    EXPECT_NE(flat.error().context.find("spec"), std::string::npos) << flat.error().context;
    // no kind block in it
    const auto empty = sc::part_spec_from_text(kBlocks + "spec:\n  sources: {}\n");
    ASSERT_FALSE(empty.has_value());
    EXPECT_NE(empty.error().context.find("spec"), std::string::npos) << empty.error().context;
    // two kinds in it
    const auto two = sc::part_spec_from_text(kBlocks + as_spec(v1) + "  prop: {diameter: 0.2}\n");
    ASSERT_FALSE(two.has_value());
    EXPECT_NE(two.error().context.find("spec"), std::string::npos) << two.error().context;
    // a v1 rule broken inside it is the v1 reader's, still named as the spec's
    std::string bad = as_spec(v1);
    const std::size_t kv = bad.find("kv:");
    ASSERT_NE(kv, std::string::npos);
    bad.replace(kv, bad.find('\n', kv) - kv, "kv: abc");
    const auto broken = sc::part_spec_from_text(kBlocks + bad);
    ASSERT_FALSE(broken.has_value());
    EXPECT_NE(broken.error().context.find("spec"), std::string::npos) << broken.error().context;
}

}  // namespace
