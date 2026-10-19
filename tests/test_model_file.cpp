// The model file (scene/model_file.hpp): one vehicle model plus, optionally,
// its propulsion chain -- what the editor adds a drone from, what Spade's
// model library holds, what an analysis case's ModelRef.file names, and what
// Kat's builder writes.
//
// What is pinned here:
//   - CANONICAL TEXT. The reference quad and its chain survive the trip with
//     their three identities (model, parts, propulsion) and their bytes; a
//     tiny model with a fixed chain matches a pinned text, which the gcc leg
//     reproduces (MSVC = gcc).
//   - ONE ENCODING (TD-9). The model block is the scene's per-model block, key
//     for key, two spaces shallower: the scene and the model file share one
//     emitter, one parser and one set of per-model rules.
//   - ONE VALIDATOR, the scene's model rules plus the chain's one rule
//     (vehicles::validate_propulsion_chain), run by the reader and the writer.
//   - THE VERSION GATE, and unknown, missing and duplicate keys refused at
//     every level, blamed on "model schema v1".
//   - MESH PARTS by sidecar, relative to the model file; the writer never
//     writes one.
//   - DOUBLES (world/detail/yaml_text.hpp's format_double and parse_double):
//     bit for bit, -0 included; a non-finite or out-of-range one refused.

#include <gtest/gtest.h>

#include <cfloat>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <functional>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include <glm/gtc/quaternion.hpp>
#include <yaml-cpp/yaml.h>

#include "core/error.hpp"
#include "core/path_text.hpp"
#include "reference_airframe.hpp"
#include "scene/model_file.hpp"
#include "scene/scene_file.hpp"
#include "unique_temp.hpp"
#include "vehicles/airframe_compile.hpp"
#include "vehicles/model_identity.hpp"
#include "vehicles/model_part.hpp"
#include "vehicles/propulsion_steady.hpp"
#include "world/detail/yaml_text.hpp"
#include "world/mesh.hpp"
#include "world/mesh_file.hpp"
#include "world/sdf.hpp"

namespace {

using spade::scene::ModelFile;
using spade::vehicles::ModelType;
using spade::vehicles::PartDesc;
using spade::vehicles::PropulsionChain;

template <class T>
[[nodiscard]] testing::AssertionResult IsOk(const char* expr, const spade::Result<T>& r) {
    if (r) return testing::AssertionSuccess();
    return testing::AssertionFailure() << expr << " failed: [" << static_cast<int>(r.error().code) << "] "
                                       << r.error().context;
}
#define ASSERT_OK(expr) ASSERT_PRED_FORMAT1(IsOk, expr)
#define EXPECT_OK(expr) EXPECT_PRED_FORMAT1(IsOk, expr)

[[nodiscard]] bool contains(const std::string& haystack, const std::string& needle) {
    return haystack.find(needle) != std::string::npos;
}

// The refusal's context, or a marker when the call unexpectedly succeeded.
template <class T>
[[nodiscard]] std::string why(const spade::Result<T>& r) {
    return r ? std::string("<succeeded>") : r.error().context;
}

[[nodiscard]] std::string first_difference(const std::string& expected, const std::string& actual) {
    std::size_t i = 0;
    while (i < expected.size() && i < actual.size() && expected[i] == actual[i]) ++i;
    if (i == expected.size() && i == actual.size()) return "identical";
    std::size_t line = 1;
    for (std::size_t j = 0; j < i; ++j) {
        if (expected[j] == '\n') ++line;
    }
    return "first difference at byte " + std::to_string(i) + ", line " + std::to_string(line);
}

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

void write_text(const std::filesystem::path& path, const std::string& text) {
    std::ofstream out(path, std::ios::binary | std::ios::trunc);
    out.write(text.data(), static_cast<std::streamsize>(text.size()));
}

[[nodiscard]] uint64_t bits(double v) {
    uint64_t b = 0;
    std::memcpy(&b, &v, sizeof b);
    return b;
}

// ---------------------------------------------------------------------------
// The reference quad (tests/reference_airframe.hpp), compiled: a real model
// with parts, and the chain its blocks describe.
// ---------------------------------------------------------------------------
[[nodiscard]] std::optional<ModelFile> reference_model_file() {
    const auto compiled = spade::vehicles::compile_airframe(spade::vehicles::fixtures::reference_quad());
    EXPECT_TRUE(compiled.has_value()) << why(compiled);
    const auto chain = spade::vehicles::airframe_propulsion_chain(spade::vehicles::fixtures::reference_quad());
    EXPECT_TRUE(chain.has_value()) << why(chain);
    if (!compiled || !chain) return std::nullopt;
    return ModelFile{compiled->model, *chain};
}

// ---------------------------------------------------------------------------
// The tiny model and the fixed chain of the pinned text. Every value is a
// literal (no libm, TD-3). The model has one rotor and one SDF part, so the
// text shows each nested list at the model block's depth. The chain's
// doubles are spelt both ways: fixed (256.56340004316644, the 2450 rpm/V kv)
// and with an exponent, negative (1.5e-05) and positive (1.2e+06, a pack
// limit high enough to need one), and current_min is -0.
// ---------------------------------------------------------------------------
[[nodiscard]] ModelType tiny_model() {
    ModelType m{};
    m.name = "tiny";
    m.visual_ref = "meshes/tiny";
    m.body.mass = 0.25f;
    m.body.inertia_diag = {0.5f, 0.25f, 0.5f};
    m.proxy_radius = 0.125f;
    spade::vehicles::RotorDesc r{};
    r.local_pos = {0.0f, 0.0625f, 0.0f};
    r.spin_dir = 1.0f;
    r.tau = 0.03f;
    r.radius = 0.0635f;
    r.thrust_coeff = 4.1e-6f;
    r.torque_coeff = 6.2e-8f;
    m.rotors = {r};
    PartDesc frame{};
    frame.name = "frame";
    frame.role = "frame";
    frame.shape.transforms.push_back(spade::SdfTransform{});
    spade::SdfNode box{};
    box.kind = static_cast<uint32_t>(spade::SdfPrim::box);
    box.op = static_cast<uint32_t>(spade::SdfOp::none);
    box.params = {0.5f, 0.125f, 0.5f, 0.0f};
    frame.shape.nodes.push_back(box);
    frame.min_feature = 0.25f;
    m.parts = {frame};
    return m;
}

[[nodiscard]] PropulsionChain fixed_chain() {
    PropulsionChain c;
    c.kv = 256.56340004316644;  // 2450 rpm/V
    c.resistance = 0.075;
    c.no_load_current = 1.2;
    c.current_min = -0.0;
    c.current_max = 50.0;
    c.esc_current_total_max = 160.0;
    c.pole_pairs = 7.0;
    c.inductance = 1.5e-5;
    c.diameter = 0.1524;
    c.table.j_min = 0.0f;
    c.table.j_max = 0.8f;
    c.table.ct = {0.12f, 0.04f};
    c.table.cq = {0.008f, 0.004f};
    c.cells_series = 4;
    c.cells_parallel = 1;
    c.cell_resistance = 0.005;
    c.cell_cutoff_voltage = 3.0;
    c.pack_polarization_resistance = 0.002;
    c.pack_current_max = 1.2e6;
    c.ocv_table = {3.3f, 4.2f};
    c.motors_on_bus = 1;
    return c;
}

// THE PINNED CANONICAL TEXT of ModelFile{tiny_model(), fixed_chain()}. Written
// by hand from the format (model_file.hpp), every number spelt independently:
// a float as %.9g of its binary32, a double as its shortest round-trip digits
// placed by printf's %g rule (fixed for an exponent in [-4, 6), else
// scientific), each exponent signed with at least two digits (a Python port,
// not the C++). The gcc leg must reproduce it byte for byte.
constexpr std::string_view kPinnedText = R"YAML(# =============================================================================
# Spade model file -- schema version 1.
#
# WHAT THIS IS: one vehicle model, and optionally its propulsion chain -- what
# the editor adds a drone from, what Spade's model library holds, and what an
# analysis case names.
#
# GENERATED by spade::scene::model_to_yaml(). Hand editing is supported: every
# field is validated on load, and re-saving normalizes the file back to this
# canonical form (fixed key order, fixed number spelling, these comments).
#
# UNITS are SI -- metres, kilograms, seconds, radians, volts, amperes, ohms,
# henries -- in a right-handed, Y-up frame. FLOATS carry 9 significant digits
# and DOUBLES the fewest that read back to the same value, so every value
# survives a load and a save bit for bit. Quaternions are [w, x, y, z].
# =============================================================================

model_version: 1

# The model: a scene file's per-model block, key for key. `design_to_principal`
# turns the design frame into the principal-axis body frame; `com_offset` is
# the centre of mass from the design origin, in the design frame. Rotor, drag
# and IMU mounts are in the body frame; a part's mount is too, or, on a rotor,
# in that rotor's hub frame at azimuth 0. A mesh part names its .spmesh
# sidecar by a path relative to this file and FNV-1a 64 of the mesh's content.
model:
  name: "tiny"
  version: 1
  param_schema_id: 0
  visual_ref: "meshes/tiny"
  design_to_principal: [1, 0, 0, 0]
  com_offset: [0, 0, 0]
  body: {mass: 0.25, inertia_diag: [0.5, 0.25, 0.5]}
  proxy_radius: 0.125
  rotors:
    - {local_pos: [0, 0.0625, 0], local_orient: [1, 0, 0, 0], spin_dir: 1, tau: 0.0299999993, radius: 0.063500002, thrust_coeff: 4.09999984e-06, torque_coeff: 6.1999998e-08}
  drag_bodies: []
  imu_mounts: []
  parts:
    - name: "frame"
      role: "frame"
      attach: "body"
      rotor: 0
      mount: {position: [0, 0, 0], orientation: [1, 0, 0, 0]}
      surface: {kind: "wall", inv_permeability: 0, inertial_coeff: 0}
      min_feature: 0.25
      shape:
        sdf:
          transforms:
            # [0]
            - world_to_local: [1, 0, 0, 0,
                               0, 1, 0, 0,
                               0, 0, 1, 0,
                               0, 0, 0, 1]
              scale: 1
          nodes:
            - {prim: box, transform: 0, params: [0.5, 0.125, 0.5, 0]}

# The propulsion chain: every field of vehicles::PropulsionChain, by its own
# name. kv is in rad/(s V), not rpm/V; battery fields are per cell unless they
# say pack; the propeller's tables are on a uniform grid of advance ratio from
# j_min to j_max, and ocv_table on a uniform state-of-charge grid.
propulsion:
  kv: 256.56340004316644
  resistance: 0.075
  no_load_current: 1.2
  current_min: -0
  current_max: 50
  esc_current_total_max: 160
  pole_pairs: 7
  inductance: 1.5e-05
  diameter: 0.1524
  table:
    j_min: 0
    j_max: 0.800000012
    ct: [0.119999997, 0.0399999991]
    cq: [0.00800000038, 0.00400000019]
  cells_series: 4
  cells_parallel: 1
  cell_resistance: 0.005
  cell_cutoff_voltage: 3
  pack_polarization_resistance: 0.002
  pack_current_max: 1.2e+06
  ocv_table: [3.29999995, 4.19999981]
  motors_on_bus: 1
)YAML";

// ---------------------------------------------------------------------------
// A mesh part (scene file's CORE-6 tests have the same shapes).
// ---------------------------------------------------------------------------
using MeshPtr = std::shared_ptr<const spade::TriangleMesh>;

// A tetrahedron with three legs of `leg` metres, counter-clockwise from outside.
[[nodiscard]] MeshPtr tetra_mesh(double leg) {
    spade::MeshInput m;
    m.unit = spade::MeshUnit::metre;
    m.positions = {{0.0, 0.0, 0.0}, {leg, 0.0, 0.0}, {0.0, leg, 0.0}, {0.0, 0.0, leg}};
    m.indices = {0, 2, 1, 0, 1, 3, 0, 3, 2, 1, 2, 3};
    const auto r = spade::make_mesh(m, "tetra");
    EXPECT_TRUE(r.has_value()) << why(r);
    return r ? *r : nullptr;
}

[[nodiscard]] PartDesc mesh_part(MeshPtr mesh, std::string file) {
    PartDesc p{};
    p.name = "hull";
    p.role = "body";
    p.mount_pos = {0.0f, 0.05f, 0.0f};
    p.mesh = std::move(mesh);
    p.mesh_file = std::move(file);
    p.min_feature = 0.004f;
    return p;
}

constexpr const char* kSidecar = "meshes/tetra.spmesh";

}  // namespace

// ===========================================================================
// Canonical text
// ===========================================================================

TEST(ModelFile, TheReferenceQuadRoundTripsWithItsIdentitiesAndItsBytes) {
    const std::optional<ModelFile> file = reference_model_file();
    ASSERT_TRUE(file.has_value());
    ASSERT_TRUE(file->propulsion.has_value());
    ASSERT_FALSE(file->model.parts.empty()) << "the compiled quad has parts, so parts_identity is not vacuous";

    const spade::Result<std::string> text = spade::scene::model_to_yaml(*file);
    ASSERT_OK(text);
    const spade::Result<ModelFile> back = spade::scene::model_from_yaml(*text);
    ASSERT_OK(back);

    EXPECT_EQ(spade::vehicles::model_identity(back->model), spade::vehicles::model_identity(file->model));
    EXPECT_EQ(spade::vehicles::parts_identity(back->model), spade::vehicles::parts_identity(file->model));
    EXPECT_NE(spade::vehicles::parts_identity(back->model), spade::vehicles::kNoPartsIdentity);
    ASSERT_TRUE(back->propulsion.has_value());
    EXPECT_EQ(spade::vehicles::propulsion_identity(*back->propulsion),
              spade::vehicles::propulsion_identity(*file->propulsion));
    EXPECT_EQ(back->model.visual_ref, file->model.visual_ref) << "render data, in no identity, still survives";

    // Write what was read: the same bytes.
    const spade::Result<std::string> again = spade::scene::model_to_yaml(*back);
    ASSERT_OK(again);
    EXPECT_TRUE(*again == *text) << first_difference(*text, *again);
}

TEST(ModelFile, NoPropulsionBlockRoundTripsAsAbsent) {
    const std::optional<ModelFile> reference = reference_model_file();
    ASSERT_TRUE(reference.has_value());
    const ModelFile bare{reference->model, std::nullopt};

    const spade::Result<std::string> text = spade::scene::model_to_yaml(bare);
    ASSERT_OK(text);
    EXPECT_FALSE(contains(*text, "\npropulsion:")) << *text;
    EXPECT_FALSE(contains(*text, "# The propulsion chain:")) << "no block, and no comment for one: " << *text;
    const spade::Result<ModelFile> back = spade::scene::model_from_yaml(*text);
    ASSERT_OK(back);
    EXPECT_FALSE(back->propulsion.has_value());
    EXPECT_EQ(spade::vehicles::model_identity(back->model), spade::vehicles::model_identity(bare.model));
    const spade::Result<std::string> again = spade::scene::model_to_yaml(*back);
    ASSERT_OK(again);
    EXPECT_TRUE(*again == *text) << first_difference(*text, *again);

    // The same model with its chain is the same text plus the block.
    const spade::Result<std::string> dressed = spade::scene::model_to_yaml(*reference);
    ASSERT_OK(dressed);
    EXPECT_TRUE(dressed->starts_with(*text)) << "the propulsion block comes last";
}

TEST(ModelFile, TheCanonicalTextIsPinned) {
    const ModelFile file{tiny_model(), fixed_chain()};
    const spade::Result<std::string> text = spade::scene::model_to_yaml(file);
    ASSERT_OK(text);
    const std::string pinned(kPinnedText);
    EXPECT_TRUE(*text == pinned) << first_difference(pinned, *text) << "\n" << *text;

    // The pinned text reads back to the same file, every bit.
    const spade::Result<ModelFile> back = spade::scene::model_from_yaml(pinned);
    ASSERT_OK(back);
    EXPECT_EQ(spade::vehicles::model_identity(back->model), spade::vehicles::model_identity(file.model));
    EXPECT_EQ(spade::vehicles::parts_identity(back->model), spade::vehicles::parts_identity(file.model));
    ASSERT_TRUE(back->propulsion.has_value());
    EXPECT_EQ(spade::vehicles::propulsion_identity(*back->propulsion), spade::vehicles::propulsion_identity(*file.propulsion));
    EXPECT_EQ(bits(back->propulsion->current_min), bits(-0.0)) << "-0 stays -0";
}

TEST(ModelFile, HandWrittenTextNormalizesToTheCanonicalForm) {
    // No comments, a flow mapping with its keys in another order, a '+', and
    // other spellings of the same values.
    const std::string pinned(kPinnedText);
    std::string hand = pinned.substr(pinned.find("model_version: 1"));
    hand = with(hand, "model_version: 1", "model_version: 01");
    hand = with(hand, "kv: 256.56340004316644", "kv: +2.5656340004316644e2");
    hand = with(hand, "inductance: 1.5e-05", "inductance: 0.000015");
    hand = with(hand, "cells_series: 4", "cells_series: +4");
    hand = with(hand,
                "\n  table:\n    j_min: 0\n    j_max: 0.800000012\n    ct: [0.119999997, 0.0399999991]\n"
                "    cq: [0.00800000038, 0.00400000019]\n",
                "\n  table: {cq: [0.008, 0.004], ct: [0.12, 0.04], j_max: 0.8, j_min: 0}\n");
    ASSERT_NE(hand, pinned);
    const spade::Result<ModelFile> read = spade::scene::model_from_yaml(hand);
    ASSERT_OK(read);
    const spade::Result<std::string> text = spade::scene::model_to_yaml(*read);
    ASSERT_OK(text);
    EXPECT_TRUE(*text == std::string(kPinnedText)) << first_difference(std::string(kPinnedText), *text);
}

TEST(ModelFile, TheModelBlockIsTheScenesPerModelBlock) {
    // TD-9: the scene's `models:` item, as a mapping two spaces shallower, is
    // the model file's `model:` block, byte for byte.
    const std::optional<ModelFile> reference = reference_model_file();
    ASSERT_TRUE(reference.has_value());
    spade::scene::SceneDesc scene{};
    scene.name = "s";
    scene.world.file = "w.world.yaml";
    scene.models = {reference->model};
    const spade::Result<std::string> scene_text = spade::scene::scene_to_yaml(scene);
    ASSERT_OK(scene_text);
    const std::size_t item = scene_text->find("\nmodels:\n  - name: ");
    ASSERT_NE(item, std::string::npos) << *scene_text;
    const std::size_t item_begin = item + std::string_view("\nmodels:\n").size();
    const std::size_t item_end = scene_text->find("\n\n", item_begin);
    ASSERT_NE(item_end, std::string::npos);
    std::string shallower;
    std::size_t at = item_begin;
    while (at <= item_end) {
        const std::size_t eol = scene_text->find('\n', at);
        std::string line = scene_text->substr(at, eol - at);
        if (at == item_begin) {
            ASSERT_TRUE(line.starts_with("  - ")) << line;
            line = "  " + line.substr(4);
        } else {
            ASSERT_TRUE(line.starts_with("  ")) << line;
            line = line.substr(2);
        }
        shallower += line + "\n";
        at = eol + 1;
    }

    const spade::Result<std::string> model_text = spade::scene::model_to_yaml(ModelFile{reference->model, std::nullopt});
    ASSERT_OK(model_text);
    const std::size_t block = model_text->find("\nmodel:\n");
    ASSERT_NE(block, std::string::npos) << *model_text;
    const std::string model_block = model_text->substr(block + std::string_view("\nmodel:\n").size());
    EXPECT_TRUE(model_block == shallower) << first_difference(shallower, model_block) << "\n" << model_block << "\nvs\n"
                                          << shallower;
}

// ===========================================================================
// Doubles: world/detail/yaml_text.hpp's format_double and parse_double
// ===========================================================================

TEST(YamlTextDouble, EveryDoubleSurvivesTheTripBitForBit) {
    struct Case {
        double value;
        const char* spelling;
    };
    const Case cases[] = {
        {-0.0, "-0"},
        {0.0, "0"},
        {0.1, "0.1"},
        {1.0 / 3.0, "0.3333333333333333"},
        {std::numeric_limits<double>::denorm_min(), "5e-324"},
        {2.225073858507201e-308, "2.225073858507201e-308"},  // the largest subnormal
        {DBL_MIN, "2.2250738585072014e-308"},
        {DBL_MAX, "1.7976931348623157e+308"},
        {-DBL_MAX, "-1.7976931348623157e+308"},
        {256.56340004316644, "256.56340004316644"},
        {123456.0, "123456"},
        {1.0e6, "1e+06"},
        {1.5e-5, "1.5e-05"},
        {0.0001, "0.0001"},
    };
    for (const Case& c : cases) {
        SCOPED_TRACE(c.spelling);
        const std::string text = spade::yaml_text::format_double(c.value);
        EXPECT_EQ(text, c.spelling);
        const YAML::Node node = YAML::Load(text);
        const spade::Result<double> back = spade::yaml_text::parse_double(node, "x");
        ASSERT_OK(back);
        EXPECT_EQ(bits(*back), bits(c.value)) << "bit for bit";
    }
}

TEST(YamlTextDouble, AcceptsALeadingPlusAsParseFloatDoes) {
    const spade::Result<double> plus = spade::yaml_text::parse_double(YAML::Load("+1.5"), "x");
    ASSERT_OK(plus);
    EXPECT_EQ(*plus, 1.5);
}

TEST(YamlTextDouble, RefusesWhatIsNotAFiniteDoubleByName) {
    const std::pair<const char*, const char*> cases[] = {
        {"nan", "x: 'nan' is not finite"},
        {"inf", "x: 'inf' is not finite"},
        {"-inf", "x: '-inf' is not finite"},
        {"1e999", "x: '1e999' is outside the range of a 64-bit float"},
        {"-1e999", "x: '-1e999' is outside the range of a 64-bit float"},
        {"abc", "x: 'abc' is not a decimal number"},
        {"1.5x", "x: '1.5x' is not a decimal number"},
        {"", "x: '' is not a decimal number"},
    };
    for (const auto& [text, needle] : cases) {
        SCOPED_TRACE(text);
        const YAML::Node node = YAML::Load(std::string("\"") + text + "\"");
        const spade::Result<double> r = spade::yaml_text::parse_double(node, "x");
        ASSERT_FALSE(r);
        EXPECT_EQ(r.error().code, spade::Code::invalid_argument);
        EXPECT_TRUE(contains(r.error().context, needle)) << r.error().context;
    }
    const spade::Result<double> mapping = spade::yaml_text::parse_double(YAML::Load("{a: 1}"), "x");
    EXPECT_TRUE(contains(why(mapping), "x must be a scalar")) << why(mapping);
}

// ===========================================================================
// The version gate and the keys
// ===========================================================================

TEST(ModelFile, AnotherModelVersionIsASchemaMismatch) {
    for (const char* version : {"2", "0"}) {
        SCOPED_TRACE(version);
        const spade::Result<ModelFile> r = spade::scene::model_from_yaml(
            with(std::string(kPinnedText), "\nmodel_version: 1\n", std::string("\nmodel_version: ") + version + "\n"));
        ASSERT_FALSE(r);
        EXPECT_EQ(r.error().code, spade::Code::schema_mismatch);
        EXPECT_TRUE(contains(r.error().context, std::string("model file is schema version ") + version +
                                                    ", this build reads version 1 only"))
            << r.error().context;
    }
    // A version-2 file's keys are not reported as unknown: the version comes first.
    const spade::Result<ModelFile> newer = spade::scene::model_from_yaml(
        with(with(std::string(kPinnedText), "\nmodel_version: 1\n", "\nmodel_version: 2\n"), "\npropulsion:\n",
             "\nthrottle_curve: [0, 1]\npropulsion:\n"));
    ASSERT_FALSE(newer);
    EXPECT_EQ(newer.error().code, spade::Code::schema_mismatch);

    const spade::Result<ModelFile> word = spade::scene::model_from_yaml(
        with(std::string(kPinnedText), "\nmodel_version: 1\n", "\nmodel_version: one\n"));
    ASSERT_FALSE(word);
    EXPECT_EQ(word.error().code, spade::Code::invalid_argument);
    EXPECT_TRUE(contains(word.error().context, "model_version: 'one' is not a non-negative decimal integer"))
        << word.error().context;

    const spade::Result<ModelFile> list = spade::scene::model_from_yaml("[1, 2]\n");
    EXPECT_TRUE(contains(why(list), "a model file's top level must be a mapping with a 'model_version' key"))
        << why(list);
}

TEST(ModelFile, UnknownMissingAndDuplicateKeysAreRefusedAtEveryLevel) {
    const std::string base(kPinnedText);
    ASSERT_OK(spade::scene::model_from_yaml(base));
    struct Case {
        std::string what;
        std::string from;
        std::string to;
        std::string needle;
    };
    const std::vector<Case> cases = {
        // The top level.
        {"a missing model", "\nmodel:\n  name: \"tiny\"\n", "\nmodel_block:\n  name: \"tiny\"\n",
         "unknown key 'model_block' in model file -- model schema v1 does not define it (line "},
        {"an unknown top-level key", "\nmodel_version: 1\n", "\nmodel_version: 1\nbogus: 1\n",
         "unknown key 'bogus' in model file -- model schema v1 does not define it (line "},
        {"a duplicate top-level key", "\nmodel_version: 1\n", "\nmodel_version: 1\nmodel_version: 1\n",
         "duplicate key 'model_version' in model file"},
        // The model block: the scene's key set, blamed on the model's schema.
        {"an unknown key in the model", "\n  proxy_radius: 0.125\n", "\n  proxy_radius: 0.125\n  bogus: 1\n",
         "unknown key 'bogus' in model -- model schema v1 does not define it (line "},
        {"a missing key in the model", "\n  proxy_radius: 0.125\n", "\n",
         "missing required key 'proxy_radius' in model (line "},
        {"an unknown key in a part", "\n      min_feature: 0.25\n", "\n      min_feature: 0.25\n      bogus: 1\n",
         "unknown key 'bogus' in model.parts[0] -- model schema v1 does not define it (line "},
        {"an unknown key in a rotor", "{local_pos: [0, 0.0625, 0], ", "{bogus: 1, local_pos: [0, 0.0625, 0], ",
         "unknown key 'bogus' in model.rotors[0] -- model schema v1 does not define it (line "},
        // The propulsion block: every field, by name.
        {"an unknown key in propulsion", "\n  motors_on_bus: 1\n", "\n  motors_on_bus: 1\n  bogus: 1\n",
         "unknown key 'bogus' in propulsion -- model schema v1 does not define it (line "},
        {"a missing key in propulsion", "\n  kv: 256.56340004316644\n", "\n",
         "missing required key 'kv' in propulsion (line "},
        {"a duplicate key in propulsion", "\n  pole_pairs: 7\n", "\n  pole_pairs: 7\n  pole_pairs: 7\n",
         "duplicate key 'pole_pairs' in propulsion"},
        {"an unknown key in propulsion.table", "\n    j_min: 0\n", "\n    j_min: 0\n    bogus: 1\n",
         "unknown key 'bogus' in propulsion.table -- model schema v1 does not define it (line "},
        {"a missing key in propulsion.table", "\n    cq: [0.00800000038, 0.00400000019]\n", "\n",
         "missing required key 'cq' in propulsion.table (line "},
        // Values, by their dotted paths.
        {"a double that is not a number", "\n  resistance: 0.075\n", "\n  resistance: low\n",
         "propulsion.resistance: 'low' is not a decimal number (line "},
        {"a non-finite double", "\n  inductance: 1.5e-05\n", "\n  inductance: .nan\n",
         "propulsion.inductance: '.nan' is not a decimal number (line "},
        {"a double out of range", "\n  diameter: 0.1524\n", "\n  diameter: 1e400\n",
         "propulsion.diameter: '1e400' is outside the range of a 64-bit float (line "},
        {"a negative count", "\n  cells_series: 4\n", "\n  cells_series: -4\n",
         "propulsion.cells_series: '-4' is not a non-negative decimal integer (line "},
        {"a table value that is not a number", "ct: [0.119999997, 0.0399999991]", "ct: [0.119999997, x]",
         "propulsion.table.ct[1]: 'x' is not a decimal number (line "},
        {"a table that is not a sequence", "ocv_table: [3.29999995, 4.19999981]", "ocv_table: 3.3",
         "propulsion.ocv_table must be a sequence (line "},
    };
    for (const Case& c : cases) {
        SCOPED_TRACE(c.what);
        const spade::Result<ModelFile> r = spade::scene::model_from_yaml(with(base, c.from, c.to));
        ASSERT_FALSE(r) << "accepted";
        EXPECT_EQ(r.error().code, spade::Code::invalid_argument);
        EXPECT_TRUE(contains(r.error().context, c.needle)) << r.error().context;
        EXPECT_FALSE(contains(r.error().context, "scene schema")) << "a model file blames its own schema";
    }

    // The model key is required, and propulsion is not.
    const std::size_t model_at = base.find("\n# The model:");
    const std::size_t propulsion_at = base.find("\n# The propulsion chain:");
    ASSERT_NE(model_at, std::string::npos);
    ASSERT_NE(propulsion_at, std::string::npos);
    const std::string no_model = base.substr(0, model_at) + base.substr(propulsion_at);
    const spade::Result<ModelFile> missing = spade::scene::model_from_yaml(no_model);
    EXPECT_TRUE(contains(why(missing), "missing required key 'model' in model file")) << why(missing);
    const spade::Result<ModelFile> no_chain = spade::scene::model_from_yaml(base.substr(0, propulsion_at));
    ASSERT_OK(no_chain);
    EXPECT_FALSE(no_chain->propulsion.has_value());

    // A propulsion block that is not a mapping.
    const spade::Result<ModelFile> scalar =
        spade::scene::model_from_yaml(base.substr(0, propulsion_at) + "\npropulsion: 3\n");
    EXPECT_TRUE(contains(why(scalar), "propulsion must be a mapping (line ")) << why(scalar);
}

// ===========================================================================
// One validator
// ===========================================================================

TEST(ModelFile, ABadChainIsRefusedNamingTheField) {
    struct Case {
        std::string what;
        std::function<void(PropulsionChain&)> edit;
        std::string from;
        std::string to;
        std::string field;
    };
    const std::vector<Case> cases = {
        {"kv 0", [](PropulsionChain& c) { c.kv = 0.0; }, "\n  kv: 256.56340004316644\n", "\n  kv: 0\n", "kv"},
        {"a ct and cq size mismatch", [](PropulsionChain& c) { c.table.cq.push_back(0.002f); },
         "cq: [0.00800000038, 0.00400000019]", "cq: [0.00800000038, 0.00400000019, 0.002]", "table.cq"},
        {"motors_on_bus 0", [](PropulsionChain& c) { c.motors_on_bus = 0; }, "\n  motors_on_bus: 1\n",
         "\n  motors_on_bus: 0\n", "motors_on_bus"},
        {"motors_on_bus 65", [](PropulsionChain& c) { c.motors_on_bus = 65; }, "\n  motors_on_bus: 1\n",
         "\n  motors_on_bus: 65\n", "motors_on_bus"},
    };
    for (const Case& c : cases) {
        SCOPED_TRACE(c.what);
        ModelFile file{tiny_model(), fixed_chain()};
        c.edit(*file.propulsion);
        const spade::Result<void> chain = spade::vehicles::validate_propulsion_chain(*file.propulsion);
        ASSERT_FALSE(chain);
        EXPECT_EQ(chain.error().code, spade::Code::invalid_argument);
        EXPECT_TRUE(contains(chain.error().context, c.field)) << chain.error().context;

        // The file's validator, its writer and its reader: the chain's own message.
        const spade::Result<void> validated = spade::scene::validate_model_file(file);
        ASSERT_FALSE(validated);
        EXPECT_EQ(validated.error().code, spade::Code::invalid_argument);
        EXPECT_EQ(validated.error().context, chain.error().context);
        const spade::Result<std::string> written = spade::scene::model_to_yaml(file);
        ASSERT_FALSE(written);
        EXPECT_EQ(written.error().context, chain.error().context);
        const spade::Result<ModelFile> read = spade::scene::model_from_yaml(with(std::string(kPinnedText), c.from, c.to));
        ASSERT_FALSE(read);
        EXPECT_EQ(read.error().code, spade::Code::invalid_argument);
        EXPECT_EQ(read.error().context, chain.error().context);
    }
}

TEST(ModelFile, ABadModelIsRefusedWithTheScenesMessage) {
    // The per-model rules are the scene's, in one function (TD-9): the same
    // message, word for word, except where it names the file or the schema.
    struct Case {
        std::string what;
        std::function<void(ModelType&)> edit;
        bool names_the_file;  // the scene's message names "scene file" or "scene schema v2"
    };
    const std::vector<Case> cases = {
        {"no mass (ModelType::validate)", [](ModelType& m) { m.body.mass = 0.0f; }, false},
        {"no name (ModelType::validate)", [](ModelType& m) { m.name.clear(); }, false},
        {"a part past the rotors (ModelType::validate)",
         [](ModelType& m) {
             m.parts[0].attach = spade::vehicles::PartAttach::rotor;
             m.parts[0].rotor = 3;
         },
         false},
        {"a design frame that is not a unit quaternion",
         [](ModelType& m) { m.design_to_principal = glm::quat(2.0f, 0.0f, 0.0f, 0.0f); }, false},
        {"a sidecar name with no mesh", [](ModelType& m) { m.parts[0].mesh_file = "meshes/x.spmesh"; }, false},
        {"a sidecar name with a backslash",
         [](ModelType& m) {
             m.parts[0] = mesh_part(tetra_mesh(0.25), "meshes\\tetra.spmesh");
         },
         false},
        {"an absolute sidecar name", [](ModelType& m) { m.parts[0] = mesh_part(tetra_mesh(0.25), "C:/tetra.spmesh"); },
         true},
        {"a part's shape with padding", [](ModelType& m) { m.parts[0].shape.nodes[0]._pad = 1; }, true},
    };
    for (const Case& c : cases) {
        SCOPED_TRACE(c.what);
        ModelFile file{tiny_model(), fixed_chain()};
        c.edit(file.model);
        spade::scene::SceneDesc scene{};
        scene.name = "s";
        scene.world.file = "w.world.yaml";
        scene.models = {file.model};
        const spade::Result<void> in_scene = spade::scene::validate_scene(scene);
        ASSERT_FALSE(in_scene) << "the scene refuses it";
        const spade::Result<void> in_file = spade::scene::validate_model_file(file);
        ASSERT_FALSE(in_file);
        EXPECT_EQ(in_file.error().code, in_scene.error().code);
        std::string expected = in_scene.error().context;
        if (c.names_the_file) {
            for (const auto& [from, to] : {std::pair<std::string, std::string>{"scene file", "model file"},
                                           {"scene schema v2", "model schema v1"}}) {
                const std::size_t at = expected.find(from);
                if (at != std::string::npos) {
                    expected.replace(at, from.size(), to);
                }
            }
            EXPECT_NE(expected, in_scene.error().context) << "the scene's message names its file or schema";
        }
        EXPECT_EQ(in_file.error().context, expected);
        const spade::Result<std::string> written = spade::scene::model_to_yaml(file);
        ASSERT_FALSE(written);
        EXPECT_EQ(written.error().context, expected) << "the writer validates first";
    }
}

// ===========================================================================
// Mesh parts: a sidecar relative to the model file
// ===========================================================================

TEST(ModelFile, AMeshPartLoadsFromItsSidecar) {
    const spade::test::TempDir dir("model-mesh-sidecar");
    const MeshPtr tetra = tetra_mesh(0.25);
    ASSERT_NE(tetra, nullptr);
    std::filesystem::create_directories(dir.path() / "meshes");

    ModelFile file{tiny_model(), fixed_chain()};
    file.model.parts.push_back(mesh_part(tetra, kSidecar));
    const spade::Result<std::string> text = spade::scene::model_to_yaml(file);
    ASSERT_OK(text);
    EXPECT_TRUE(contains(*text, "      shape:\n        mesh: {file: \"meshes/tetra.spmesh\", hash: \"0x"))
        << "a mesh part names its sidecar and its hash: " << *text;
    EXPECT_FALSE(std::filesystem::exists(dir.path() / kSidecar)) << "the writer never writes a sidecar";
    ASSERT_OK(spade::save_mesh_file(dir.path() / kSidecar, *tetra));
    const std::filesystem::path path = dir.path() / "drone.model.yaml";
    write_text(path, *text);

    const spade::Result<ModelFile> loaded = spade::scene::load_model_file(path);
    ASSERT_OK(loaded);
    ASSERT_EQ(loaded->model.parts.size(), 2u);
    const PartDesc& hull = loaded->model.parts[1];
    EXPECT_EQ(hull.mesh_file, kSidecar);
    ASSERT_NE(hull.mesh, nullptr);
    EXPECT_EQ(hull.mesh->content_hash(), tetra->content_hash());
    EXPECT_EQ(spade::vehicles::parts_identity(loaded->model), spade::vehicles::parts_identity(file.model));
    EXPECT_EQ(spade::vehicles::model_identity(loaded->model), spade::vehicles::model_identity(file.model));

    // Write what was read: the same bytes; and the text with its directory
    // reads the same file.
    const spade::Result<std::string> again = spade::scene::model_to_yaml(*loaded);
    ASSERT_OK(again);
    EXPECT_TRUE(*again == *text) << first_difference(*text, *again);
    const spade::Result<ModelFile> via_text = spade::scene::model_from_yaml(*text, dir.path());
    ASSERT_OK(via_text);
    EXPECT_EQ(spade::vehicles::parts_identity(via_text->model), spade::vehicles::parts_identity(file.model));

    // Text alone cannot resolve a sidecar.
    const spade::Result<ModelFile> text_only = spade::scene::model_from_yaml(*text);
    ASSERT_FALSE(text_only);
    EXPECT_EQ(text_only.error().code, spade::Code::invalid_argument);
    EXPECT_TRUE(contains(why(text_only), "model.parts[1].shape.mesh: a sidecar's path is relative to the model "
                                         "file, and this text came with no directory: read a model with mesh parts "
                                         "with load_model_file, or give a base directory"))
        << why(text_only);

    // The writer names the sidecar and never writes it, so it needs its name.
    ModelFile unnamed = *loaded;
    unnamed.model.parts[1].mesh_file.clear();
    ASSERT_OK(spade::scene::validate_model_file(unnamed)) << "in memory, a mesh part needs no file";
    const spade::Result<std::string> written = spade::scene::model_to_yaml(unnamed);
    ASSERT_FALSE(written);
    EXPECT_TRUE(contains(why(written), "model 'tiny' part 'hull': a mesh part needs mesh_file, its sidecar's path "
                                       "relative to the model file"))
        << why(written);

    // A missing sidecar is io_error, naming it, after the file's path.
    std::filesystem::remove(dir.path() / kSidecar);
    const spade::Result<ModelFile> missing = spade::scene::load_model_file(path);
    ASSERT_FALSE(missing);
    EXPECT_EQ(missing.error().code, spade::Code::io_error);
    EXPECT_TRUE(contains(why(missing), "tetra.spmesh")) << why(missing);
    EXPECT_TRUE(contains(why(missing), spade::path_text(path) + ": ")) << why(missing);
}

TEST(ModelFile, LoadModelFileNamesThePath) {
    const spade::test::TempDir dir("model-load");
    const std::filesystem::path absent = dir.path() / "absent.model.yaml";
    const spade::Result<ModelFile> missing = spade::scene::load_model_file(absent);
    ASSERT_FALSE(missing);
    EXPECT_EQ(missing.error().code, spade::Code::io_error);
    EXPECT_TRUE(contains(why(missing), "cannot open model file '" + spade::path_text(absent) + "'")) << why(missing);

    // A parse diagnostic comes back prefixed with the path.
    const std::filesystem::path bad = dir.path() / "bad.model.yaml";
    write_text(bad, with(std::string(kPinnedText), "\n  resistance: 0.075\n", "\n  resistance: low\n"));
    const spade::Result<ModelFile> parsed = spade::scene::load_model_file(bad);
    ASSERT_FALSE(parsed);
    EXPECT_EQ(parsed.error().code, spade::Code::invalid_argument);
    EXPECT_TRUE(parsed.error().context.starts_with(spade::path_text(bad) + ": propulsion.resistance: 'low'"))
        << parsed.error().context;

    // A YAML syntax error too.
    write_text(bad, "model_version: [1\n");
    const spade::Result<ModelFile> syntax = spade::scene::load_model_file(bad);
    ASSERT_FALSE(syntax);
    EXPECT_EQ(syntax.error().code, spade::Code::invalid_argument);
    EXPECT_TRUE(syntax.error().context.starts_with(spade::path_text(bad) + ": ")) << syntax.error().context;

    // And a good one loads.
    const std::filesystem::path good = dir.path() / "good.model.yaml";
    write_text(good, std::string(kPinnedText));
    const spade::Result<ModelFile> loaded = spade::scene::load_model_file(good);
    ASSERT_OK(loaded);
    EXPECT_EQ(loaded->model.name, "tiny");
}
