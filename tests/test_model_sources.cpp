// A library drone's sources sidecar (Task L5 of docs/design/core/plans/
// 2026-10-21-default-drone-library-plan.md; scene/model_sources.hpp): the
// model and its parts pinned by hash, every input with its value and its
// citation, the checks, and check_model_sources() against a library.
//
// What is pinned here:
//   - THE HASH: FNV-1a 64 with CRLF read as LF, tools/library_props'
//     hash_file;
//   - THE TEXT: a full sidecar round-trips byte for byte and value for value;
//     unknown, missing and duplicate keys are refused at every level by the
//     dotted path with the line and column; another version is
//     schema_mismatch, read first; the citations' registry-free rules (a
//     source unless assumed, a note on derived and assumed) in the reader and
//     the writer;
//   - THE CHECK, on a library in a temp folder: it passes when every hash
//     matches; a changed model is refused naming model.hash; a stale or
//     unlisted part naming parts.<kind>; a citation of an unlisted source by
//     its field;
//   - THE LEAD'S RULING (2026-10-22): a project's model entry with no sidecar
//     still loads; only the default library's tests require one.

#include <gtest/gtest.h>

#include <array>
#include <bit>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <limits>
#include <optional>
#include <string>
#include <string_view>
#include <system_error>
#include <utility>
#include <variant>
#include <vector>

#include "core/error.hpp"
#include "core/path_text.hpp"
#include "core/rng.hpp"
#include "scene/library.hpp"
#include "scene/library_sources.hpp"
#include "scene/model_file.hpp"
#include "scene/model_sources.hpp"
#include "scene/part_file.hpp"
#include "unique_temp.hpp"

namespace {

namespace fs = std::filesystem;

using spade::Code;
using spade::scene::check_model_sources;
using spade::scene::Grade;
using spade::scene::InputValue;
using spade::scene::KatProvenance;
using spade::scene::Library;
using spade::scene::load_model_sources;
using spade::scene::ModelCheck;
using spade::scene::ModelFile;
using spade::scene::ModelInput;
using spade::scene::ModelSources;
using spade::scene::model_sources_from_yaml;
using spade::scene::model_sources_to_yaml;
using spade::scene::MotorPart;
using spade::scene::PartKind;
using spade::scene::PinnedFile;
using spade::scene::PinnedPart;
using spade::scene::pinned_file_hash;
using spade::scene::SourceRef;
using spade::scene::SourceRegistry;

template <class T>
[[nodiscard]] testing::AssertionResult IsOk(const char* expr, const spade::Result<T>& r) {
    if (r) return testing::AssertionSuccess();
    return testing::AssertionFailure() << expr << " failed: [" << static_cast<int>(r.error().code) << "] "
                                       << r.error().context;
}
#define ASSERT_OK(expr) ASSERT_PRED_FORMAT1(IsOk, expr)
#define EXPECT_OK(expr) EXPECT_PRED_FORMAT1(IsOk, expr)

// The refusal's context, or a marker when the call unexpectedly succeeded.
template <class T>
[[nodiscard]] std::string why(const spade::Result<T>& r) {
    return r ? std::string("<succeeded>") : r.error().context;
}

template <class T>
[[nodiscard]] int code_of(const spade::Result<T>& r) {
    return r ? -1 : static_cast<int>(r.error().code);
}

[[nodiscard]] constexpr int code(Code c) { return static_cast<int>(c); }

[[nodiscard]] uint64_t bits(double v) { return std::bit_cast<uint64_t>(v); }

// `text` with its one occurrence of `from` replaced by `to`; an anchor that is
// absent or repeated fails the calling test instead of mutating the wrong place.
[[nodiscard]] std::string with(const std::string& text, const std::string& from, const std::string& to) {
    const std::size_t at = text.find(from);
    EXPECT_NE(at, std::string::npos) << "anchor not found: " << from;
    EXPECT_EQ(text.find(from, at + 1), std::string::npos) << "anchor not unique: " << from;
    if (at == std::string::npos) return text;
    std::string out = text;
    out.replace(at, from.size(), to);
    return out;
}

void write_text(const fs::path& path, std::string_view text) {
    std::error_code ec;
    fs::create_directories(path.parent_path(), ec);
    std::ofstream out(path, std::ios::binary | std::ios::trunc);
    out.write(text.data(), static_cast<std::streamsize>(text.size()));
}

[[nodiscard]] bool read_bytes(const fs::path& path, std::string& out) {
    std::ifstream in(path, std::ios::binary);
    if (!in) return false;
    out.assign(std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>());
    return true;
}

// The spelling a sidecar writes a hash in: 16 lowercase hex digits, no 0x.
[[nodiscard]] std::string hex16(uint64_t value) {
    static constexpr char kHex[] = "0123456789abcdef";
    std::string out;
    for (int shift = 60; shift >= 0; shift -= 4) out += kHex[(value >> shift) & 0xfu];
    return out;
}

// ---------------------------------------------------------------------------
// A full sidecar, as a struct and as its canonical text
// ---------------------------------------------------------------------------

// The generated-file comment the writer puts first.
constexpr std::string_view kHeader =
    "# =============================================================================\n"
    "# Spade model sources, schema version 1: where each input of one model file\n"
    "# came from, and the files it was built from, each pinned by its hash.\n"
    "# GENERATED by spade::scene::model_sources_to_yaml() beside the model; never\n"
    "# edit it by hand. check_model_sources() refuses it once its model or a part\n"
    "# changes. The format is engine/scene/model_sources.hpp's.\n"
    "# =============================================================================\n";

// full_sources()'s text after kHeader and a blank line, so its first line,
// the version, is line 9. A long line is split across two raw strings.
constexpr std::string_view kFullBody = R"YAML(model_sources_version: 1
model: {file: "drones/quad.model.yaml", hash: "0123456789abcdef"}
built_by: "vehicles::basic_quad_airframe()"
parts:
  prop: {file: "parts/p6.prop.yaml", hash: "00000000000000ff"}
  motor: {file: "parts/m2306.motor.yaml", hash: "fedcba9876543210"}
inputs:
  "mass": {value: 0.7, source: "kat_700", grade: fitted, )YAML"
                                       R"YAML(kat: {source: "tuned", confidence: "medium", date: "2026-07-27"}}
  "body_size": {value: [0.09, 0.035, 0.06], grade: assumed, note: "the centre box"}
  "inertia": {value: [0.0025, 0.0043, 0.0021], grade: assumed, note: "NeuroBEM's", )YAML"
                                       R"YAML(kat: {source: "assumed", confidence: "low"}}
  "rotors[0].hub.position": {value: [0.08, 0.022, -0.1], source: "nasa_ames_2016", grade: measured}
  "motor.rotor_inertia": {value: 2.7e-06, grade: assumed, note: "the bell estimate"}
  "kappa": {value: 0.022, source: "kat_700", grade: derived, note: "kappa C_T / D"}
not_carried:
  - "thrust_curve"
  - "motor.tau"
checks:
  - {test: "HoversAtTheCurvesHoverPoint", reference: 9.712, unit: "m/s^2", source: "kat_700", grade: fitted, )YAML"
                                       R"YAML(band: [-0.02, 0.02], report_only: false}
  - {test: "FullDutyThrust", reference: 9.188, unit: "N", grade: assumed, note: "Kat's maximum", )YAML"
                                       R"YAML(report_only: true}
assumptions:
  - "kappa applies only to a prop without a table"
caveat: "fitted to the VQ1 simulator, not hardware"
)YAML";

[[nodiscard]] std::string full_text() { return std::string(kHeader) + "\n" + std::string(kFullBody); }

[[nodiscard]] ModelSources full_sources() {
    ModelSources s;
    s.model = PinnedFile{"drones/quad.model.yaml", 0x0123456789abcdefull};
    s.built_by = "vehicles::basic_quad_airframe()";
    s.parts = {PinnedPart{PartKind::prop, "parts/p6.prop.yaml", 0xffull},
               PinnedPart{PartKind::motor, "parts/m2306.motor.yaml", 0xfedcba9876543210ull}};
    s.inputs = {
        ModelInput{"mass", 0.7, SourceRef{"kat_700", Grade::fitted, ""},
                   KatProvenance{"tuned", "medium", "2026-07-27"}},
        ModelInput{"body_size", std::vector<double>{0.09, 0.035, 0.06},
                   SourceRef{"", Grade::assumed, "the centre box"}, std::nullopt},
        ModelInput{"inertia", std::vector<double>{0.0025, 0.0043, 0.0021}, SourceRef{"", Grade::assumed, "NeuroBEM's"},
                   KatProvenance{"assumed", "low", ""}},
        ModelInput{"rotors[0].hub.position", std::vector<double>{0.08, 0.022, -0.1},
                   SourceRef{"nasa_ames_2016", Grade::measured, ""}, std::nullopt},
        ModelInput{"motor.rotor_inertia", 2.7e-06, SourceRef{"", Grade::assumed, "the bell estimate"}, std::nullopt},
        ModelInput{"kappa", 0.022, SourceRef{"kat_700", Grade::derived, "kappa C_T / D"}, std::nullopt},
    };
    s.not_carried = {"thrust_curve", "motor.tau"};
    s.checks = {
        ModelCheck{"HoversAtTheCurvesHoverPoint", 9.712, "m/s^2", SourceRef{"kat_700", Grade::fitted, ""},
                   std::array<double, 2>{-0.02, 0.02}, false},
        ModelCheck{"FullDutyThrust", 9.188, "N", SourceRef{"", Grade::assumed, "Kat's maximum"}, std::nullopt, true},
    };
    s.assumptions = {"kappa applies only to a prop without a table"};
    s.caveat = "fitted to the VQ1 simulator, not hardware";
    return s;
}

void expect_same_ref(const SourceRef& a, const SourceRef& b, const std::string& what) {
    EXPECT_EQ(a.source, b.source) << what;
    EXPECT_EQ(a.grade, b.grade) << what;
    EXPECT_EQ(a.note, b.note) << what;
}

void expect_same_value(const InputValue& a, const InputValue& b, const std::string& what) {
    ASSERT_EQ(a.index(), b.index()) << what << ": a number and a vector are two kinds of value";
    if (const double* x = std::get_if<double>(&a)) {
        EXPECT_EQ(bits(*x), bits(std::get<double>(b))) << what;
        return;
    }
    const std::vector<double>& x = std::get<std::vector<double>>(a);
    const std::vector<double>& y = std::get<std::vector<double>>(b);
    ASSERT_EQ(x.size(), y.size()) << what;
    for (std::size_t i = 0; i < x.size(); ++i) {
        EXPECT_EQ(bits(x[i]), bits(y[i])) << what << "[" << i << "]";
    }
}

// Field for field, every double bit for bit.
void expect_same(const ModelSources& a, const ModelSources& b) {
    EXPECT_EQ(a.model.file, b.model.file);
    EXPECT_EQ(a.model.hash, b.model.hash);
    EXPECT_EQ(a.built_by, b.built_by);
    ASSERT_EQ(a.parts.size(), b.parts.size());
    for (std::size_t i = 0; i < a.parts.size(); ++i) {
        EXPECT_EQ(a.parts[i].kind, b.parts[i].kind) << "parts[" << i << "]";
        EXPECT_EQ(a.parts[i].file, b.parts[i].file) << "parts[" << i << "]";
        EXPECT_EQ(a.parts[i].hash, b.parts[i].hash) << "parts[" << i << "]";
    }
    ASSERT_EQ(a.inputs.size(), b.inputs.size());
    for (std::size_t i = 0; i < a.inputs.size(); ++i) {
        const ModelInput& x = a.inputs[i];
        const ModelInput& y = b.inputs[i];
        const std::string what = "inputs[" + std::to_string(i) + "] " + x.path;
        EXPECT_EQ(x.path, y.path) << what;
        expect_same_value(x.value, y.value, what);
        expect_same_ref(x.ref, y.ref, what);
        ASSERT_EQ(x.kat.has_value(), y.kat.has_value()) << what;
        if (x.kat) {
            EXPECT_EQ(x.kat->source, y.kat->source) << what;
            EXPECT_EQ(x.kat->confidence, y.kat->confidence) << what;
            EXPECT_EQ(x.kat->date, y.kat->date) << what;
        }
    }
    EXPECT_EQ(a.not_carried, b.not_carried);
    ASSERT_EQ(a.checks.size(), b.checks.size());
    for (std::size_t i = 0; i < a.checks.size(); ++i) {
        const ModelCheck& x = a.checks[i];
        const ModelCheck& y = b.checks[i];
        const std::string what = "checks[" + std::to_string(i) + "]";
        EXPECT_EQ(x.test, y.test) << what;
        EXPECT_EQ(bits(x.reference), bits(y.reference)) << what;
        EXPECT_EQ(x.unit, y.unit) << what;
        expect_same_ref(x.ref, y.ref, what);
        ASSERT_EQ(x.band.has_value(), y.band.has_value()) << what;
        if (x.band) {
            EXPECT_EQ(bits((*x.band)[0]), bits((*y.band)[0])) << what;
            EXPECT_EQ(bits((*x.band)[1]), bits((*y.band)[1])) << what;
        }
        EXPECT_EQ(x.report_only, y.report_only) << what;
    }
    EXPECT_EQ(a.assumptions, b.assumptions);
    EXPECT_EQ(a.caveat, b.caveat);
}

// A refusal of the reader's, at `line` and, when it is not 0, `column`: a key
// inside a flow mapping is pinned to its line only.
struct Refusal {
    std::string what;
    std::string text;
    std::string message;  // the context less its " (line L, column C)"
    int line = 0;
    int column = 0;
};

void expect_refused_at(const Refusal& c) {
    SCOPED_TRACE(c.what);
    const spade::Result<ModelSources> r = model_sources_from_yaml(c.text);
    ASSERT_FALSE(r.has_value());
    EXPECT_EQ(r.error().code, Code::invalid_argument) << r.error().context;
    const std::string mark = " (line " + std::to_string(c.line) + ", column ";
    if (c.column != 0) {
        EXPECT_EQ(r.error().context, c.message + mark + std::to_string(c.column) + ")");
    } else {
        EXPECT_TRUE(r.error().context.starts_with(c.message + mark) && r.error().context.ends_with(")"))
            << r.error().context;
    }
}

}  // namespace

// ===========================================================================
// The hash and the text
// ===========================================================================

// tools/library_props' fnv1a64: FNV-1a 64, CRLF read as LF (its
// TheHashIsFnv1a64OfLfText pins the same two values).
TEST(ModelSources, ThePinnedHashIsFnv1a64WithCrlfReadAsLf) {
    EXPECT_EQ(pinned_file_hash(""), 0xCBF29CE484222325ull);
    EXPECT_EQ(pinned_file_hash("a"), 0xAF63DC4C8601EC8Cull);
    EXPECT_EQ(pinned_file_hash("a\nb\n"), spade::rng::fnv1a64("a\nb\n"));
    EXPECT_EQ(pinned_file_hash("a\r\nb\r\n"), pinned_file_hash("a\nb\n"));
    // Only a CR that an LF follows is dropped.
    EXPECT_NE(pinned_file_hash("a\rb"), pinned_file_hash("ab"));
    EXPECT_EQ(pinned_file_hash("a\r\r\n"), spade::rng::fnv1a64("a\r\n"));
}

TEST(ModelSources, AFullSidecarRoundTripsByteForByte) {
    const std::string text = full_text();
    const spade::Result<ModelSources> read = model_sources_from_yaml(text);
    ASSERT_OK(read);
    const spade::Result<std::string> written = model_sources_to_yaml(*read);
    ASSERT_OK(written);
    EXPECT_EQ(*written, text);
}

TEST(ModelSources, AFullSidecarReadsValueForValueAndWritesItsText) {
    const spade::Result<ModelSources> read = model_sources_from_yaml(full_text());
    ASSERT_OK(read);
    expect_same(*read, full_sources());

    const spade::Result<std::string> written = model_sources_to_yaml(full_sources());
    ASSERT_OK(written);
    EXPECT_EQ(*written, full_text());

    // One number in a sequence stays a vector.
    const spade::Result<ModelSources> one =
        model_sources_from_yaml(with(full_text(), "{value: 2.7e-06, ", "{value: [2.7e-06], "));
    ASSERT_OK(one);
    ASSERT_EQ(one->inputs.size(), 6u);
    expect_same_value(one->inputs[4].value, InputValue{std::vector<double>{2.7e-06}}, "inputs[4]");
}

TEST(ModelSources, AnEmptySidecarRoundTrips) {
    const std::string text = std::string(kHeader) +
                             "\n"
                             "model_sources_version: 1\n"
                             "model: {file: \"drones/quad.model.yaml\", hash: \"0000000000000000\"}\n"
                             "built_by: \"by hand\"\n"
                             "parts: {}\n"
                             "inputs: {}\n"
                             "not_carried: []\n"
                             "checks: []\n"
                             "assumptions: []\n";
    const spade::Result<ModelSources> read = model_sources_from_yaml(text);
    ASSERT_OK(read);
    EXPECT_TRUE(read->parts.empty());
    EXPECT_TRUE(read->inputs.empty());
    EXPECT_TRUE(read->checks.empty());
    EXPECT_TRUE(read->caveat.empty());
    const spade::Result<std::string> written = model_sources_to_yaml(*read);
    ASSERT_OK(written);
    EXPECT_EQ(*written, text);
}

TEST(ModelSources, UnknownMissingAndDuplicateKeysAreRefusedWithTheirLineAndColumn) {
    const std::string base = full_text();
    const std::string schema = " -- model sources schema v1 does not define it";
    const std::vector<Refusal> cases = {
        // The top level (line 9 is the version, 10 the model, 11 built_by).
        {"an unknown top-level key", with(base, "built_by:", "maker: \"x\"\nbuilt_by:"),
         "unknown key 'maker' in model sources file" + schema, 11, 1},
        {"no version", with(base, "model_sources_version: 1\n", ""),
         "missing required key 'model_sources_version' in model sources file", 9, 1},
        {"no built_by", with(base, "built_by: \"vehicles::basic_quad_airframe()\"\n", ""),
         "missing required key 'built_by' in model sources file", 9, 1},
        {"a duplicate top-level key",
         with(base, "built_by:", "model: {file: \"x\", hash: \"0000000000000000\"}\nbuilt_by:"),
         "duplicate key 'model' in model sources file", 11, 1},
        // The model.
        {"an unknown model key", with(base, "hash: \"0123456789abcdef\"}", "hash: \"0123456789abcdef\", size: 3}"),
         "unknown key 'size' in model" + schema, 10, 0},
        {"a model with no hash", with(base, ", hash: \"0123456789abcdef\"}", "}"),
         "missing required key 'hash' in model", 10, 0},
        // The parts (13 the prop, 14 the motor).
        {"an unknown part kind",
         with(base, "  motor: {", "  frame: {file: \"x\", hash: \"0000000000000000\"}\n  motor: {"),
         "unknown key 'frame' in parts" + schema, 14, 3},
        {"a part kind twice", with(base, "  motor: {", "  prop: {file: \"x\", hash: \"0000000000000000\"}\n  motor: {"),
         "duplicate key 'prop' in parts", 14, 3},
        // The inputs (16 mass to 21 kappa).
        {"an unknown input key", with(base, "grade: measured}", "grade: measured, maker: \"x\"}"),
         "unknown key 'maker' in inputs.rotors[0].hub.position" + schema, 19, 0},
        {"an input with no grade",
         with(base, ", grade: assumed, note: \"the centre box\"}", ", note: \"the centre box\"}"),
         "missing required key 'grade' in inputs.body_size", 17, 0},
        {"an input with no value", with(base, "{value: 2.7e-06, ", "{"),
         "missing required key 'value' in inputs.motor.rotor_inertia", 20, 0},
        {"an input twice",
         with(base, "  \"kappa\":", "  \"mass\": {value: 1, grade: assumed, note: \"x\"}\n  \"kappa\":"),
         "duplicate key 'mass' in inputs", 21, 3},
        {"a duplicate input key", with(base, "grade: measured}", "grade: measured, grade: fitted}"),
         "duplicate key 'grade' in inputs.rotors[0].hub.position", 19, 0},
        {"an unknown kat key", with(base, "confidence: \"low\"}", "confidence: \"low\", by: \"x\"}"),
         "unknown key 'by' in inputs.inertia.kat" + schema, 18, 0},
        {"a kat with no confidence", with(base, "{source: \"assumed\", confidence: \"low\"}", "{source: \"assumed\"}"),
         "missing required key 'confidence' in inputs.inertia.kat", 18, 0},
        // The checks (26 and 27).
        {"an unknown check key", with(base, "report_only: true}", "report_only: true, owner: \"x\"}"),
         "unknown key 'owner' in checks[1]" + schema, 27, 0},
        {"a check with no unit", with(base, "unit: \"N\", ", ""), "missing required key 'unit' in checks[1]", 27, 0},
        {"a check with no report_only", with(base, ", report_only: false}", "}"),
         "missing required key 'report_only' in checks[0]", 26, 0},
        {"a duplicate check key", with(base, "report_only: true}", "report_only: true, report_only: false}"),
         "duplicate key 'report_only' in checks[1]", 27, 0},
        // A hash is 16 lowercase hex digits.
        {"an upper-case hash", with(base, "0123456789abcdef", "0123456789ABCDEF"),
         "model.hash: '0123456789ABCDEF' must be 16 lowercase hex digits, the FNV-1a 64 with no 0x", 10, 0},
    };
    for (const Refusal& c : cases) expect_refused_at(c);
}

TEST(ModelSources, AnotherVersionIsASchemaMismatchReadFirst) {
    const std::string two = with(full_text(), "model_sources_version: 1\n", "model_sources_version: 2\n");
    const spade::Result<ModelSources> r = model_sources_from_yaml(two);
    EXPECT_EQ(code_of(r), code(Code::schema_mismatch));
    EXPECT_EQ(why(r), "model sources file is schema version 2, this build reads version 1 only (line 9, column 24)");

    // Before any other key: a later version's keys are not called unknown.
    const spade::Result<ModelSources> later =
        model_sources_from_yaml(with(two, "built_by:", "maker: \"x\"\nbuilt_by:"));
    EXPECT_EQ(code_of(later), code(Code::schema_mismatch)) << why(later);
}

// check_source_ref()'s registry-free rules, in the reader and the writer.
TEST(ModelSources, ACitationNeedsASourceUnlessAssumedAndANoteWhenDerived) {
    const std::string base = full_text();
    const std::vector<Refusal> cases = {
        {"a measured input with no source",
         with(base, "source: \"nasa_ames_2016\", grade: measured}", "grade: measured}"),
         "inputs.rotors[0].hub.position is graded measured but cites no source; only an assumed value may cite none",
         19, 0},
        {"a derived input with no note", with(base, "grade: derived, note: \"kappa C_T / D\"}", "grade: derived}"),
         "inputs.kappa is graded derived and needs a note saying which code derived it", 21, 0},
        {"an assumed input with no note",
         with(base, ", grade: assumed, note: \"the bell estimate\"}", ", grade: assumed}"),
         "inputs.motor.rotor_inertia is graded assumed and needs a note saying why", 20, 0},
        {"a fitted check with no source",
         with(base, "unit: \"m/s^2\", source: \"kat_700\", grade: fitted", "unit: \"m/s^2\", grade: fitted"),
         "checks[0] is graded fitted but cites no source; only an assumed value may cite none", 26, 0},
        {"an empty source", with(base, "source: \"nasa_ames_2016\"", "source: \"\""),
         "inputs.rotors[0].hub.position.source must not be empty: leave the key out for an assumed value that "
         "cites none",
         19, 0},
    };
    for (const Refusal& c : cases) expect_refused_at(c);

    // An assumed value may cite none: body_size does, and reads (above).
    ModelSources no_source = full_sources();
    no_source.inputs[3].ref.source.clear();
    const spade::Result<std::string> measured = model_sources_to_yaml(no_source);
    EXPECT_EQ(code_of(measured), code(Code::invalid_argument));
    EXPECT_EQ(why(measured),
              "inputs.rotors[0].hub.position is graded measured but cites no source; only an assumed value may "
              "cite none");

    ModelSources no_note = full_sources();
    no_note.inputs[5].ref.note.clear();
    const spade::Result<std::string> derived = model_sources_to_yaml(no_note);
    EXPECT_EQ(code_of(derived), code(Code::invalid_argument));
    EXPECT_EQ(why(derived), "inputs.kappa is graded derived and needs a note saying which code derived it");
}

TEST(ModelSources, TheWriterRefusesWhatTheReaderWould) {
    const auto refusal = [](const ModelSources& s) { return why(model_sources_to_yaml(s)); };

    ModelSources upper = full_sources();
    upper.inputs[0].path = "Mass";
    EXPECT_TRUE(refusal(upper).starts_with("inputs key 'Mass' is not a field path")) << refusal(upper);
    for (const char* bad : {"", "a..b", "rotors[].x", "rotors[0", "0abc", "a.", "hub-x"}) {
        ModelSources s = full_sources();
        s.inputs[0].path = bad;
        EXPECT_TRUE(refusal(s).starts_with("inputs key '" + std::string(bad) + "' is not a field path"))
            << bad << ": " << refusal(s);
    }
    ModelSources indexed = full_sources();
    indexed.inputs[0].path = "imus[0].rate_divider[12]";
    EXPECT_OK(model_sources_to_yaml(indexed));

    ModelSources twice = full_sources();
    twice.inputs[1].path = "mass";
    EXPECT_EQ(refusal(twice), "inputs has 'mass' twice: each input is listed once");

    ModelSources empty_vector = full_sources();
    empty_vector.inputs[1].value = std::vector<double>{};
    EXPECT_EQ(refusal(empty_vector),
              "inputs.body_size.value is an empty sequence: a vector input has at least one number");

    ModelSources infinite = full_sources();
    infinite.inputs[0].value = std::numeric_limits<double>::infinity();
    EXPECT_TRUE(refusal(infinite).starts_with("inputs.mass.value is not finite")) << refusal(infinite);

    ModelSources bad_date = full_sources();
    bad_date.inputs[0].kat->date = "2026-7-27";
    EXPECT_TRUE(refusal(bad_date).starts_with("inputs.mass.kat.date '2026-7-27' must be a date, YYYY-MM-DD"))
        << refusal(bad_date);

    ModelSources swapped = full_sources();
    std::swap(swapped.parts[0], swapped.parts[1]);
    EXPECT_EQ(refusal(swapped),
              "parts lists the prop after the motor: the order is prop, motor, esc, battery, each kind at most once");

    ModelSources inverted = full_sources();
    inverted.checks[0].band = std::array<double, 2>{0.02, -0.02};
    EXPECT_EQ(refusal(inverted), "checks[0].band [0.02, -0.02] has lo above hi");

    ModelSources unbanded = full_sources();
    unbanded.checks[0].band.reset();
    EXPECT_EQ(refusal(unbanded), "checks[0] has no band but is not report_only: a banded check needs [lo, hi]");
    // A report-only check may record a band.
    ModelSources recorded = full_sources();
    recorded.checks[1].band = std::array<double, 2>{-0.3, 0.3};
    EXPECT_OK(model_sources_to_yaml(recorded));

    ModelSources repeated = full_sources();
    repeated.not_carried.push_back("thrust_curve");
    EXPECT_EQ(refusal(repeated), "not_carried[2] 'thrust_curve' is not_carried[0] again: each is listed once");

    ModelSources blank = full_sources();
    blank.assumptions.push_back("");
    EXPECT_EQ(refusal(blank), "assumptions[1] must not be empty");

    ModelSources no_model = full_sources();
    no_model.model.file.clear();
    EXPECT_EQ(refusal(no_model), "model.file must not be empty");
}

TEST(ModelSources, LoadModelSourcesReadsTheFileAndNamesIt) {
    const spade::test::TempDir dir("model-sources-load");
    const fs::path file = dir.path() / "quad.sources.yaml";
    write_text(file, full_text());
    const spade::Result<ModelSources> loaded = load_model_sources(file);
    ASSERT_OK(loaded);
    expect_same(*loaded, full_sources());

    const fs::path absent = dir.path() / "absent.sources.yaml";
    const spade::Result<ModelSources> none = load_model_sources(absent);
    EXPECT_EQ(code_of(none), code(Code::io_error));
    EXPECT_EQ(why(none), "cannot open model sources file '" + spade::path_text(absent) + "'");

    const fs::path newer = dir.path() / "newer.sources.yaml";
    write_text(newer, with(full_text(), "model_sources_version: 1\n", "model_sources_version: 2\n"));
    const spade::Result<ModelSources> version = load_model_sources(newer);
    EXPECT_EQ(code_of(version), code(Code::schema_mismatch));
    EXPECT_TRUE(why(version).starts_with(spade::path_text(newer) + ": model sources file is schema version 2"))
        << why(version);
}

// ===========================================================================
// check_model_sources(), on a library in a temp folder
// ===========================================================================

namespace {

constexpr std::string_view kRegistry = R"YAML(sources_version: 1
sources:
  - {id: uiuc_propdb, citation: "UIUC", retrieved: 2026-10-21, license: facts_cite, license_evidence: "facts"}
  - {id: kat_700, citation: "Kat", retrieved: 2026-10-21, license: facts_cite, license_evidence: "facts"}
  - {id: apc_data_files, citation: "APC", retrieved: 2026-10-21, license: not_usable, license_evidence: "no"}
)YAML";

constexpr std::string_view kIndex = R"YAML(library_version: 2
entries:
  - {name: Quad, category: drones, file: drones/quad.model.yaml}
  - {name: M2312, category: motors, file: parts/m2312.motor.yaml}
)YAML";

// A model the model file accepts, of `mass`.
[[nodiscard]] std::string a_model_text(float mass) {
    ModelFile file;
    file.model.name = "quad";
    file.model.body.mass = mass;
    file.model.proxy_radius = 0.2f;
    const spade::Result<std::string> text = spade::scene::model_to_yaml(file);
    EXPECT_TRUE(text.has_value()) << why(text);
    return text.value_or("");
}

// A motor part of `kv`, citing `kv_source` for it and assuming the rest
// (tests/test_library.cpp's).
[[nodiscard]] std::string a_motor_text(double kv, std::string_view kv_source) {
    MotorPart m;
    m.block.kv = kv;
    m.block.resistance = 0.1;
    m.block.no_load_current = 0.5;
    m.block.current_max = 20.0;
    m.block.mass = 0.055;
    for (const char* f : {"kv", "resistance", "no_load_current", "no_load_voltage", "current_max", "pole_pairs",
                          "inductance", "rotor_inertia", "mass"}) {
        m.sources.push_back({f, std::string_view(f) == "kv" ? SourceRef{std::string(kv_source), Grade::measured, ""}
                                                            : SourceRef{"", Grade::assumed, "typical"}});
    }
    const spade::Result<std::string> text = spade::scene::motor_to_yaml(m);
    EXPECT_TRUE(text.has_value()) << why(text);
    return text.value_or("");
}

// `text` with every LF made CRLF, as a converting checkout makes it.
[[nodiscard]] std::string crlf(std::string_view text) {
    std::string out;
    for (const char c : text) {
        if (c == '\n') out += '\r';
        out += c;
    }
    return out;
}

[[nodiscard]] fs::path model_path(const fs::path& dir) { return dir / "drones" / "quad.model.yaml"; }
[[nodiscard]] fs::path motor_path(const fs::path& dir) { return dir / "parts" / "m2312.motor.yaml"; }

// The index, its registry, one model and one motor part.
void lay_out_library(const fs::path& dir) {
    write_text(dir / "sources.yaml", kRegistry);
    write_text(dir / "library.yaml", kIndex);
    write_text(model_path(dir), a_model_text(1.5f));
    write_text(motor_path(dir), a_motor_text(960.0, "uiuc_propdb"));
}

[[nodiscard]] uint64_t hash_of(const fs::path& path) {
    std::string bytes;
    EXPECT_TRUE(read_bytes(path, bytes)) << spade::path_text(path);
    return pinned_file_hash(bytes);
}

// lay_out_library()'s sidecar, pinned to the files as they are now.
[[nodiscard]] ModelSources sidecar_of(const fs::path& dir) {
    ModelSources s;
    s.model = PinnedFile{"drones/quad.model.yaml", hash_of(model_path(dir))};
    s.built_by = "a test";
    s.parts = {PinnedPart{PartKind::motor, "parts/m2312.motor.yaml", hash_of(motor_path(dir))}};
    s.inputs = {
        ModelInput{"mass", 1.5, SourceRef{"kat_700", Grade::fitted, ""},
                   KatProvenance{"tuned", "medium", "2026-07-27"}},
        ModelInput{"body_size", std::vector<double>{0.09, 0.035, 0.06}, SourceRef{"", Grade::assumed, "a box"},
                   std::nullopt},
    };
    s.checks = {ModelCheck{"Hovers", 9.80665, "m/s^2", SourceRef{"uiuc_propdb", Grade::measured, ""},
                           std::array<double, 2>{-0.05, 0.05}, false}};
    return s;
}

struct LoadedLibrary {
    Library library{};
    SourceRegistry registry{};
};

[[nodiscard]] LoadedLibrary load(const fs::path& dir) {
    LoadedLibrary out;
    spade::Result<Library> library = spade::scene::load_library(dir / "library.yaml");
    EXPECT_TRUE(library.has_value()) << why(library);
    if (library) out.library = std::move(*library);
    spade::Result<SourceRegistry> registry = spade::scene::load_library_sources(out.library);
    EXPECT_TRUE(registry.has_value()) << why(registry);
    if (registry) out.registry = std::move(*registry);
    return out;
}

}  // namespace

TEST(ModelSourcesCheck, PassesWhenEveryHashMatches) {
    const spade::test::TempDir dir("model-sources-pass");
    lay_out_library(dir.path());
    const LoadedLibrary l = load(dir.path());
    const ModelSources s = sidecar_of(dir.path());
    EXPECT_OK(check_model_sources(l.library, l.registry, s));

    // A checkout that turns the model's LFs into CRLFs keeps its hash.
    write_text(model_path(dir.path()), crlf(a_model_text(1.5f)));
    EXPECT_OK(check_model_sources(l.library, l.registry, s));

    // The model alone: no parts, inputs or checks.
    ModelSources bare;
    bare.model = s.model;
    bare.built_by = "a test";
    EXPECT_OK(check_model_sources(l.library, l.registry, bare));
}

TEST(ModelSourcesCheck, AChangedModelIsRefusedNamingModelHash) {
    const spade::test::TempDir dir("model-sources-model");
    lay_out_library(dir.path());
    const LoadedLibrary l = load(dir.path());
    const ModelSources s = sidecar_of(dir.path());

    // The model must be a model entry of the index.
    ModelSources unlisted = s;
    unlisted.model.file = "drones/other.model.yaml";
    const spade::Result<void> other = check_model_sources(l.library, l.registry, unlisted);
    EXPECT_EQ(code_of(other), code(Code::invalid_argument));
    EXPECT_EQ(why(other), "model.file 'drones/other.model.yaml' is not a model of the library's index");
    ModelSources a_part = s;
    a_part.model.file = "parts/m2312.motor.yaml";
    EXPECT_EQ(why(check_model_sources(l.library, l.registry, a_part)),
              "model.file 'parts/m2312.motor.yaml' is not a model of the library's index");

    // Regenerated without its sidecar.
    write_text(model_path(dir.path()), a_model_text(1.6f));
    const spade::Result<void> changed = check_model_sources(l.library, l.registry, s);
    EXPECT_EQ(code_of(changed), code(Code::invalid_argument));
    EXPECT_TRUE(why(changed).starts_with("model.hash is " + hex16(s.model.hash) + ", but 'drones/quad.model.yaml' "
                                         "hashes to " + hex16(hash_of(model_path(dir.path())))))
        << why(changed);

    // Gone from the disk.
    fs::remove(model_path(dir.path()));
    const spade::Result<void> gone = check_model_sources(l.library, l.registry, s);
    EXPECT_EQ(code_of(gone), code(Code::io_error));
    EXPECT_TRUE(why(gone).starts_with("model.file 'drones/quad.model.yaml': cannot open '")) << why(gone);
}

TEST(ModelSourcesCheck, AStaleOrUnlistedPartIsRefusedNamingItsKind) {
    const spade::test::TempDir dir("model-sources-parts");
    lay_out_library(dir.path());
    const LoadedLibrary l = load(dir.path());
    const ModelSources s = sidecar_of(dir.path());

    // Not in the index, or not of its kind there.
    ModelSources unlisted = s;
    unlisted.parts[0].file = "parts/other.motor.yaml";
    const spade::Result<void> other = check_model_sources(l.library, l.registry, unlisted);
    EXPECT_EQ(code_of(other), code(Code::invalid_argument));
    EXPECT_EQ(why(other), "parts.motor.file 'parts/other.motor.yaml' is not a motor part of the library's index");
    ModelSources wrong_kind = s;
    wrong_kind.parts[0].kind = PartKind::prop;
    EXPECT_EQ(why(check_model_sources(l.library, l.registry, wrong_kind)),
              "parts.prop.file 'parts/m2312.motor.yaml' is not a prop part of the library's index");

    // Stale: the part changed after the sidecar was written.
    write_text(motor_path(dir.path()), a_motor_text(961.0, "uiuc_propdb"));
    const spade::Result<void> stale = check_model_sources(l.library, l.registry, s);
    EXPECT_EQ(code_of(stale), code(Code::invalid_argument));
    EXPECT_TRUE(why(stale).starts_with("parts.motor.hash is " + hex16(s.parts[0].hash) +
                                       ", but 'parts/m2312.motor.yaml' hashes to "))
        << why(stale);

    // Pinned, but it does not load: its own citation is refused, with its code.
    write_text(motor_path(dir.path()), a_motor_text(960.0, "apc_data_files"));
    ModelSources repinned = s;
    repinned.parts[0].hash = hash_of(motor_path(dir.path()));
    const spade::Result<void> refused = check_model_sources(l.library, l.registry, repinned);
    EXPECT_EQ(code_of(refused), code(Code::invalid_argument));
    EXPECT_TRUE(why(refused).starts_with("parts.motor: library entry 'M2312': ")) << why(refused);
    EXPECT_NE(why(refused).find("'apc_data_files'"), std::string::npos) << why(refused);
}

TEST(ModelSourcesCheck, ACitationOfAnUnlistedSourceIsRefusedByItsField) {
    const spade::test::TempDir dir("model-sources-cite");
    lay_out_library(dir.path());
    const LoadedLibrary l = load(dir.path());
    const ModelSources s = sidecar_of(dir.path());

    ModelSources input = s;
    input.inputs[0].ref.source = "nobody";
    const spade::Result<void> unlisted = check_model_sources(l.library, l.registry, input);
    EXPECT_EQ(code_of(unlisted), code(Code::invalid_argument));
    EXPECT_EQ(why(unlisted), "inputs.mass cites source 'nobody', which the sources registry does not list");

    ModelSources ruled_out = s;
    ruled_out.inputs[0].ref.source = "apc_data_files";
    EXPECT_TRUE(why(check_model_sources(l.library, l.registry, ruled_out))
                    .starts_with("inputs.mass cites source 'apc_data_files', whose license is not_usable"));

    ModelSources check = s;
    check.checks[0].ref.source = "nobody";
    EXPECT_EQ(why(check_model_sources(l.library, l.registry, check)),
              "checks[0] cites source 'nobody', which the sources registry does not list");
}

// The lead's ruling (2026-10-22): a project's library, written by Save to
// library, has model entries with no sidecar, and they keep loading. Only the
// default library's tests require one. A guard: it passes before and after.
TEST(ModelSourcesCheck, AProjectsModelEntryWithNoSidecarStillLoads) {
    const spade::test::TempDir dir("model-sources-project");
    write_text(model_path(dir.path()), a_model_text(1.5f));
    write_text(dir.path() / "library.yaml",
               "library_version: 1\nentries:\n  - {name: Quad, category: drones, file: drones/quad.model.yaml}\n");
    ASSERT_FALSE(fs::exists(dir.path() / "drones" / "quad.sources.yaml"));
    const spade::Result<Library> library = spade::scene::load_library(dir.path() / "library.yaml");
    ASSERT_OK(library);
    EXPECT_OK(spade::scene::check_library_contents(*library));
}
