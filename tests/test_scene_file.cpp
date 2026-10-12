// The scene file (scene/scene_file.hpp): Core's schema for a world reference
// plus the objects placed in it (docs/design/interface/plans/
// 2026-10-03-scene-file-draft.md, SCN-001..009; Core's drone-builder plan,
// Task D).
//
// What is pinned here, and what is not:
//   - CANONICAL TEXT (SCN-002). A committed scene that exercises every section
//     reloads and rewrites to the same bytes, and the writer reproduces it
//     from the same SceneDesc built in code.
//   - ONE VALIDATOR (SCN-003, SCN-004, SCN-009). Everything the scene alone
//     decides is refused by validate_scene(), which the reader and the writer
//     both call; unknown, missing and duplicate keys are refused at every
//     level of the text.
//   - THE WORLD HASH'S DEFINITION (SCN-001): FNV-1a 64 over the world file's
//     canonical text of the loaded world, so it survives a load and save and
//     tells two worlds apart.
// The refusals that need the world -- a hash mismatch and a scene material
// reusing a world material's name -- are compose()'s, tested beside it
// (tests/test_scene_compose.cpp; Interface and Core agreed the split,
// 2026-10-03).

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iterator>
#include <limits>
#include <string>
#include <string_view>
#include <vector>

#include <glm/gtc/quaternion.hpp>

#include "core/error.hpp"
#include "core/rng.hpp"
#include "scene/scene_file.hpp"
#include "world/builder.hpp"
#include "world/sdf.hpp"
#include "world/world_file.hpp"

namespace {

using spade::MaterialDesc;
using spade::SdfNode;
using spade::SdfOp;
using spade::SdfPrim;
using spade::SdfTransform;
using spade::WorldDesc;
using spade::scene::SceneAsset;
using spade::scene::SceneDesc;
using spade::scene::SceneVehicle;

template <class T>
[[nodiscard]] testing::AssertionResult IsOk(const char* expr, const spade::Result<T>& r) {
    if (r) return testing::AssertionSuccess();
    return testing::AssertionFailure() << expr << " failed: [" << static_cast<int>(r.error().code)
                                       << "] " << r.error().context;
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

[[nodiscard]] std::filesystem::path golden(const std::string& relative) {
    return std::filesystem::path(SPADE_GOLDEN_DIR) / relative;
}

[[nodiscard]] bool read_file(const std::filesystem::path& path, std::string& out) {
    std::ifstream in(path, std::ios::binary);
    if (!in) return false;
    out.assign(std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>());
    return true;
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

// TD-1: a committed scene carries its provenance in its own first lines --
// comments of this form, which the writer never produces. They are required,
// and set aside before the byte comparison.
constexpr std::string_view kProvenance = "# provenance: ";

// The offset of the first byte after the leading provenance lines.
[[nodiscard]] std::size_t provenance_end(const std::string& text) {
    std::size_t at = 0;
    while (text.compare(at, kProvenance.size(), kProvenance) == 0) {
        const std::size_t eol = text.find('\n', at);
        if (eol == std::string::npos) return text.size();
        at = eol + 1;
    }
    return at;
}

// Rotations about +Y, (w, x, y, z) = (cos(a/2), 0, sin(a/2), 0), as exact float
// constants. No libm call may feed a golden, because libm differs across
// platforms (BitPortability.NoLibmTranscendentalInEngineOrGoldenTestSource, TD-3).
const glm::quat kYawPlus90(0.707106769f, 0.0f, 0.707106769f, 0.0f);
const glm::quat kYawMinus15(0.991444886f, 0.0f, -0.1305262f, 0.0f);
const glm::quat kYawPlus30(0.965925813f, 0.0f, 0.258819044f, 0.0f);
const glm::quat kYawMinus45(0.923879504f, 0.0f, -0.382683456f, 0.0f);

[[nodiscard]] SdfNode prim(SdfPrim kind, uint32_t transform, glm::vec4 params) {
    SdfNode n{};
    n.kind = static_cast<uint32_t>(kind);
    n.op = static_cast<uint32_t>(SdfOp::none);
    n.transform = transform;
    n.params = params;
    return n;
}

[[nodiscard]] SdfNode op(SdfOp which) {
    SdfNode n{};
    n.op = static_cast<uint32_t>(which);
    return n;
}

// A translation by `offset`, pre-inverted as SdfTransform stores it.
[[nodiscard]] SdfTransform moved_by(glm::vec3 offset) {
    SdfTransform t{};
    t.world_to_local[3] = glm::vec4(-offset, 1.0f);
    return t;
}

[[nodiscard]] spade::vehicles::RotorDesc rotor(glm::vec3 at, float spin) {
    spade::vehicles::RotorDesc r{};
    r.local_pos = at;
    r.spin_dir = spin;
    r.tau = 0.03f;
    r.radius = 0.0635f;
    r.thrust_coeff = 4.1e-6f;
    r.torque_coeff = 6.2e-8f;
    return r;
}

// ---------------------------------------------------------------------------
// The committed scene: every section, every optional half, a real design
// frame, an unused model and a non-zero spare. Its world is the committed
// gate world, pinned by its real hash.
// ---------------------------------------------------------------------------
[[nodiscard]] SceneDesc gate_run_scene() {
    SceneDesc s{};
    s.name = "gate_run";
    s.world.file = "../worlds/gate.world.yaml";
    const spade::Result<WorldDesc> world = spade::load_world_file(golden("worlds/gate.world.yaml"));
    if (world) {
        const spade::Result<uint64_t> hash = spade::scene::world_hash(*world);
        if (hash) s.world.hash = *hash;
    }

    s.materials.push_back(MaterialDesc{"gate_orange", {0.95f, 0.45f, 0.10f, 1.0f},
                                       spade::MaterialShading::lambert});
    s.materials.push_back(MaterialDesc{"marker_white", {1.0f, 1.0f, 1.0f, 1.0f},
                                       spade::MaterialShading::emissive});

    spade::vehicles::ModelType kat{};
    kat.name = "kat_5in";
    kat.version = 3;
    kat.param_schema_id = 1;
    kat.visual_ref = "kat/5in_frame";
    kat.design_to_principal = kYawPlus90;
    kat.com_offset = {0.0f, -0.012f, 0.004f};
    kat.body.mass = 0.62f;
    kat.body.inertia_diag = {0.0021f, 0.0038f, 0.0023f};
    kat.proxy_radius = 0.12f;
    kat.rotors = {rotor({0.08f, 0.0f, 0.08f}, 1.0f), rotor({-0.08f, 0.0f, 0.08f}, -1.0f),
                  rotor({-0.08f, 0.0f, -0.08f}, 1.0f), rotor({0.08f, 0.0f, -0.08f}, -1.0f)};
    spade::vehicles::DragBodyDesc drag{};
    drag.mode = spade::physics::drag_mode::quadratic;
    drag.area = 0.012f;
    drag.coeffs = {1.1f, 0.0f, 0.0f};
    drag.local_orient = kYawMinus15;
    kat.drag_bodies = {drag};
    spade::vehicles::ImuMountDesc imu{};
    imu.mount_pos = {0.0f, 0.01f, 0.0f};
    imu.rate_divider = 2;
    imu.sigma_a = 0.02f;
    imu.sigma_g = 0.001f;
    imu.sigma_ba = 1e-5f;
    imu.sigma_bg = 1e-6f;
    kat.imu_mounts = {imu};
    s.models.push_back(kat);

    // Registered and never spawned, as when the editor stages one.
    spade::vehicles::ModelType spotter{};
    spotter.name = "spotter";
    spotter.body.mass = 2.5f;
    spotter.proxy_radius = 0.3f;
    s.models.push_back(spotter);

    SceneAsset gate_1{};
    gate_1.name = "gate_1";
    gate_1.pose.position = {0.0f, 1.8f, 0.0f};
    gate_1.collider.transforms = {SdfTransform{}};
    gate_1.collider.nodes = {prim(SdfPrim::torus, 0, {0.75f, 0.05f, 0.0f, 0.0f})};
    gate_1.collider_materials = {"gate_orange"};
    gate_1.visual = {"gates/ring_1500", "gate_orange"};
    s.assets.push_back(gate_1);

    SceneAsset gate_2{};
    gate_2.name = "gate_2";
    gate_2.pose.position = {4.0f, 1.8f, -10.0f};
    gate_2.pose.rotation = kYawPlus30;
    gate_2.pose.scale = 1.25f;
    gate_2.collider.transforms = {SdfTransform{}, moved_by({0.5f, 0.0f, 0.0f})};
    gate_2.collider.nodes = {prim(SdfPrim::box, 0, {0.1f, 0.9f, 0.1f, 0.0f}),
                             prim(SdfPrim::sphere, 1, {0.3f, 0.0f, 0.0f, 0.0f}), op(SdfOp::union_)};
    gate_2.visual = {"gates/frame_2000", "marker_white"};
    s.assets.push_back(gate_2);

    // Collision only: no visual.
    SceneAsset pylon{};
    pylon.name = "pylon";
    pylon.pose.position = {-3.0f, 0.0f, -6.0f};
    pylon.collider.transforms = {SdfTransform{}};
    pylon.collider.nodes = {prim(SdfPrim::capsule, 0, {0.2f, 1.0f, 0.0f, 0.0f})};
    s.assets.push_back(pylon);

    // Visual only: no collision.
    SceneAsset banner{};
    banner.name = "banner";
    banner.pose.position = {0.0f, 3.5f, -12.0f};
    banner.visual = {"decor/banner", "marker_white"};
    s.assets.push_back(banner);

    SceneVehicle quad_1{};
    quad_1.name = "quad_1";
    quad_1.model = "kat_5in";
    quad_1.start.pos = {0.0f, 0.3f, -8.0f};
    s.vehicles.push_back(quad_1);

    SceneVehicle quad_2{};
    quad_2.name = "quad_2";
    quad_2.model = "kat_5in";
    quad_2.start.pos = {1.5f, 0.3f, -8.0f};
    quad_2.start.orient = kYawMinus45;
    quad_2.start.vel = {0.0f, 0.0f, -2.0f};
    quad_2.start.omega_body = {0.0f, 0.25f, 0.0f};
    quad_2.start.rotor_omega = 1200.0f;
    s.vehicles.push_back(quad_2);

    s.spare.bodies = 1;
    return s;
}

// The smallest scene with one of everything, as hand-authored text. Every
// mapping a key test mutates has a key/value pair that occurs nowhere else.
constexpr const char* kMinimalScene = R"YAML(scene_version: 1
name: "s"
world: {file: "w.world.yaml", hash: "0x0000000000000001"}
materials: [{name: "m", base_color: [1, 1, 1, 1], shading: "lambert"}]
models:
  - name: "q"
    version: 1
    param_schema_id: 0
    visual_ref: ""
    design_to_principal: [1, 0, 0, 0]
    com_offset: [0, 0, 0]
    body: {mass: 1, inertia_diag: [1, 1, 1]}
    proxy_radius: 0.1
    rotors: [{local_pos: [0, 0, 0], local_orient: [1, 0, 0, 0], spin_dir: 1, tau: 0, radius: 0.1, thrust_coeff: 1e-06, torque_coeff: 0}]
    drag_bodies: [{mode: "quadratic", area: 0.01, coeffs: [1, 0, 0], local_pos: [0, 0, 0], local_orient: [1, 0, 0, 0]}]
    imu_mounts: [{mount_pos: [0, 0, 0], mount_orient: [1, 0, 0, 0], rate_divider: 1, sigma_a: 0, sigma_g: 0, sigma_ba: 0, sigma_bg: 0}]
assets:
  - name: "a"
    pose: {position: [3, 0, 0], orientation: [1, 0, 0, 0], scale: 2}
    collider: {sdf: {node_materials: ["m"], transforms: [{scale: 1.5, world_to_local: [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1]}], nodes: [{prim: sphere, transform: 0, params: [1, 0, 0, 0]}]}}
    visual: {mesh_ref: "r", material: "m"}
vehicles:
  - name: "v"
    model: "q"
    start: {position: [0, 0, 0], orientation: [1, 0, 0, 0], velocity: [0, 0, 0], omega_body: [0, 0, 0], rotor_omega: 0}
spare: {bodies: 0, force_elements: 0, sensors: 0, contacts: 0}
)YAML";

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

// ===========================================================================
// SCN-002: canonical text
// ===========================================================================

TEST(SceneFile, AScenesTextRoundTripsByteForByte) {
    const SceneDesc scene = gate_run_scene();
    ASSERT_NE(scene.world.hash, 0u) << "the committed gate world failed to load or hash";
    const spade::Result<std::string> text = spade::scene::scene_to_yaml(scene);
    ASSERT_OK(text);

    // The committed scene. A MISSING file is written and the test fails, so a
    // human reads it and commits it -- the world corpus's regeneration rule
    // (test_world_file.cpp, section 7).
    const std::filesystem::path path = golden("scenes/gate_run.scene.yaml");
    std::string committed;
    if (!read_file(path, committed)) {
        std::filesystem::create_directories(path.parent_path());
        std::ofstream out(path, std::ios::binary | std::ios::trunc);
        const std::string stub = std::string(kProvenance) +
                                 "GENERATED by SceneFile.AScenesTextRoundTripsByteForByte. Replace this line "
                                 "with when, from which commit, and why (TD-1).\n";
        out.write(stub.data(), static_cast<std::streamsize>(stub.size()));
        out.write(text->data(), static_cast<std::streamsize>(text->size()));
        FAIL() << path << " did not exist and has been GENERATED. Read it, write its provenance, then commit it.";
    }
    const std::size_t body = provenance_end(committed);
    ASSERT_GT(body, 0u) << path << " carries no '" << kProvenance << "' lines (TD-1)";
    const std::string canonical = committed.substr(body);
    EXPECT_TRUE(canonical == *text) << first_difference(canonical, *text);

    // Load, then write: the same bytes.
    const spade::Result<SceneDesc> loaded = spade::scene::load_scene_file(path);
    ASSERT_OK(loaded);
    const spade::Result<std::string> again = spade::scene::scene_to_yaml(*loaded);
    ASSERT_OK(again);
    EXPECT_TRUE(canonical == *again) << first_difference(canonical, *again);

    // And the loaded content is the built content, bit for bit where it is POD.
    ASSERT_EQ(loaded->assets.size(), scene.assets.size());
    for (std::size_t i = 0; i < scene.assets.size(); ++i) {
        const auto& a = scene.assets[i].collider;
        const auto& b = loaded->assets[i].collider;
        ASSERT_EQ(a.transforms.size(), b.transforms.size());
        ASSERT_EQ(a.nodes.size(), b.nodes.size());
        if (!a.transforms.empty()) {
            EXPECT_EQ(std::memcmp(a.transforms.data(), b.transforms.data(),
                                  a.transforms.size() * sizeof(SdfTransform)),
                      0);
        }
        if (!a.nodes.empty()) {
            EXPECT_EQ(std::memcmp(a.nodes.data(), b.nodes.data(), a.nodes.size() * sizeof(SdfNode)), 0);
        }
        EXPECT_TRUE(b.node_materials.empty());
        EXPECT_EQ(loaded->assets[i].collider_materials, scene.assets[i].collider_materials);
    }
    ASSERT_EQ(loaded->vehicles.size(), 2u);
    const spade::VehicleSpawn& built = scene.vehicles[1].start;
    const spade::VehicleSpawn& parsed = loaded->vehicles[1].start;
    EXPECT_EQ(std::memcmp(&built.pos, &parsed.pos, sizeof built.pos), 0);
    EXPECT_EQ(std::memcmp(&built.orient, &parsed.orient, sizeof built.orient), 0);
    EXPECT_EQ(std::memcmp(&built.vel, &parsed.vel, sizeof built.vel), 0);
    EXPECT_EQ(std::memcmp(&built.omega_body, &parsed.omega_body, sizeof built.omega_body), 0);
    EXPECT_EQ(std::memcmp(&built.rotor_omega, &parsed.rotor_omega, sizeof built.rotor_omega), 0);
    EXPECT_EQ(loaded->world.hash, scene.world.hash);
    EXPECT_EQ(loaded->world.file, "../worlds/gate.world.yaml");
}

TEST(SceneFile, HandWrittenTextNormalizesToTheCanonicalForm) {
    const spade::Result<SceneDesc> once = spade::scene::scene_from_yaml(kMinimalScene);
    ASSERT_OK(once);
    const spade::Result<std::string> text = spade::scene::scene_to_yaml(*once);
    ASSERT_OK(text);
    const spade::Result<SceneDesc> twice = spade::scene::scene_from_yaml(*text);
    ASSERT_OK(twice);
    const spade::Result<std::string> again = spade::scene::scene_to_yaml(*twice);
    ASSERT_OK(again);
    EXPECT_TRUE(*text == *again) << first_difference(*text, *again);
}

TEST(SceneFile, SpareIsAlwaysWritten) {
    const std::string full = kMinimalScene;
    const std::string spare_line = "spare: {bodies: 0, force_elements: 0, sensors: 0, contacts: 0}\n";

    // Absent: every count is 0, and the writer still spells all four.
    const spade::Result<SceneDesc> absent = spade::scene::scene_from_yaml(with(full, spare_line, ""));
    ASSERT_OK(absent);
    EXPECT_EQ(absent->spare.bodies, 0u);
    EXPECT_EQ(absent->spare.force_elements, 0u);
    EXPECT_EQ(absent->spare.sensors, 0u);
    EXPECT_EQ(absent->spare.contacts, 0u);
    const spade::Result<std::string> text = spade::scene::scene_to_yaml(*absent);
    ASSERT_OK(text);
    EXPECT_TRUE(contains(*text, "\n" + spare_line)) << *text;

    // Partial: the named count is read, the rest default to 0.
    const spade::Result<SceneDesc> partial =
        spade::scene::scene_from_yaml(with(full, spare_line, "spare: {sensors: 3}\n"));
    ASSERT_OK(partial);
    EXPECT_EQ(partial->spare.sensors, 3u);
    EXPECT_EQ(partial->spare.bodies, 0u);
    const spade::Result<std::string> partial_text = spade::scene::scene_to_yaml(*partial);
    ASSERT_OK(partial_text);
    EXPECT_TRUE(contains(*partial_text, "spare: {bodies: 0, force_elements: 0, sensors: 3, contacts: 0}\n"))
        << *partial_text;
}

// ===========================================================================
// SCN-003: unknown, missing and duplicate keys, at every level
// ===========================================================================

struct KeyCase {
    std::string level;
    std::string needle;     // one key and its value, exactly as the text spells it
    std::string unknown;    // the needle with a key the schema does not define beside it
    std::string missing;    // the text that replaces the needle to drop that key
    std::string duplicate;  // the needle twice
};

// A key inside a flow mapping: "key: value, ".
[[nodiscard]] KeyCase flow(std::string level, std::string needle) {
    return {std::move(level), needle, "bogus: 1, " + needle, "", needle + needle};
}

// A key on its own line of a block mapping, at `indent`.
[[nodiscard]] KeyCase block(std::string level, std::string line, const std::string& indent) {
    return {std::move(level), line, line + indent + "bogus: 1\n", "", line + line};
}

TEST(SceneFile, UnknownMissingAndDuplicateKeysAreRefusedAtEveryLevel) {
    const std::string base = kMinimalScene;
    ASSERT_OK(spade::scene::scene_from_yaml(base));

    const std::string sdf = R"({sdf: {node_materials: ["m"], transforms: [{scale: 1.5, world_to_local: [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1]}], nodes: [{prim: sphere, transform: 0, params: [1, 0, 0, 0]}]}})";
    const std::string sdf_body = sdf.substr(1, sdf.size() - 2) + ", ";  // "sdf: {...}, "
    const std::vector<KeyCase> cases = {
        block("scene", "name: \"s\"\n", ""),
        flow("scene.world", "file: \"w.world.yaml\", "),
        flow("scene.materials[0]", "base_color: [1, 1, 1, 1], "),
        block("scene.models[0]", "    version: 1\n", "    "),
        flow("scene.models[0].body", "mass: 1, "),
        flow("scene.models[0].rotors[0]", "spin_dir: 1, "),
        flow("scene.models[0].drag_bodies[0]", "area: 0.01, "),
        flow("scene.models[0].imu_mounts[0]", "sigma_a: 0, "),
        block("scene.assets[0]", "    visual: {mesh_ref: \"r\", material: \"m\"}\n", "    "),
        flow("scene.assets[0].pose", "position: [3, 0, 0], "),
        {"scene.assets[0].collider", sdf, "{bogus: 1, " + sdf.substr(1), "{}",
         "{" + sdf_body + sdf.substr(1)},
        flow("scene.assets[0].collider.sdf", "node_materials: [\"m\"], "),
        flow("scene.assets[0].collider.sdf.transforms[0]", "scale: 1.5, "),
        flow("scene.assets[0].collider.sdf.nodes[0]", "transform: 0, "),
        flow("scene.assets[0].visual", "mesh_ref: \"r\", "),
        block("scene.vehicles[0]", "    model: \"q\"\n", "    "),
        flow("scene.vehicles[0].start", "velocity: [0, 0, 0], "),
    };

    for (const KeyCase& c : cases) {
        SCOPED_TRACE(c.level);
        const spade::Result<SceneDesc> unknown = spade::scene::scene_from_yaml(with(base, c.needle, c.unknown));
        EXPECT_TRUE(contains(why(unknown), "unknown key 'bogus' in " + c.level)) << why(unknown);
        EXPECT_TRUE(contains(why(unknown), "scene schema v1")) << why(unknown);

        const spade::Result<SceneDesc> missing = spade::scene::scene_from_yaml(with(base, c.needle, c.missing));
        EXPECT_TRUE(contains(why(missing), "missing required key")) << why(missing);
        EXPECT_TRUE(contains(why(missing), c.level)) << why(missing);

        const spade::Result<SceneDesc> duplicate =
            spade::scene::scene_from_yaml(with(base, c.needle, c.duplicate));
        EXPECT_TRUE(contains(why(duplicate), "duplicate key")) << why(duplicate);
        EXPECT_TRUE(contains(why(duplicate), c.level)) << why(duplicate);
    }

    // spare's keys are optional, but still accounted for.
    const std::string spare = "bodies: 0, ";
    const spade::Result<SceneDesc> unknown =
        spade::scene::scene_from_yaml(with(base, spare, "bogus: 1, " + spare));
    EXPECT_TRUE(contains(why(unknown), "unknown key 'bogus' in scene.spare")) << why(unknown);
    const spade::Result<SceneDesc> duplicate =
        spade::scene::scene_from_yaml(with(base, spare, spare + spare));
    EXPECT_TRUE(contains(why(duplicate), "duplicate key 'bodies' in scene.spare")) << why(duplicate);
}

TEST(SceneFile, AnotherSceneVersionIsASchemaMismatch) {
    // This build reads versions 1 and 2 (2 carries parts, CORE-7). Above and
    // below that range is "upgrade", never "fix your file".
    for (const std::string version : {"3", "0"}) {
        SCOPED_TRACE(version);
        const spade::Result<SceneDesc> r = spade::scene::scene_from_yaml(
            with(kMinimalScene, "scene_version: 1\n", "scene_version: " + version + "\n"));
        ASSERT_FALSE(r);
        EXPECT_EQ(r.error().code, spade::Code::schema_mismatch);
        EXPECT_TRUE(contains(r.error().context, "scene file is schema version " + version)) << r.error().context;
        EXPECT_TRUE(contains(r.error().context, "reads versions 1 and 2")) << r.error().context;
    }
}

TEST(SceneFile, TheTextIsReadAsTheSchemaSpellsIt) {
    const std::string base = kMinimalScene;
    const auto refused = [&](const std::string& from, const std::string& to, const std::string& needle) {
        const spade::Result<SceneDesc> r = spade::scene::scene_from_yaml(with(base, from, to));
        EXPECT_FALSE(r) << "accepted: " << to;
        EXPECT_TRUE(contains(why(r), needle)) << why(r);
    };
    // The world hash is "0x" and 16 hex digits.
    refused("\"0x0000000000000001\"", "\"0x1\"", "16 hex digits");
    refused("\"0x0000000000000001\"", "1", "16 hex digits");
    // Drag modes and shadings are names.
    refused("mode: \"quadratic\"", "mode: \"cubic\"", "unknown drag mode 'cubic'");
    refused("shading: \"lambert\"", "shading: \"glossy\"", "unknown material shading 'glossy'");
    // A collider's node materials are names; a name the scene validates.
    refused("node_materials: [\"m\"]", "node_materials: [\"\"]", "node material");
    // Not a number.
    refused("mass: 1, ", "mass: one, ", "is not a decimal number");
}

// ===========================================================================
// SCN-003/004/009: the one validator, called by the reader and the writer
// ===========================================================================

TEST(SceneFile, TheReaderAndTheWriterBothValidate) {
    // Through the reader: a vehicle naming a model the scene does not declare.
    const spade::Result<SceneDesc> read =
        spade::scene::scene_from_yaml(with(kMinimalScene, "    model: \"q\"\n", "    model: \"zz\"\n"));
    EXPECT_TRUE(contains(why(read), "unknown model 'zz'")) << why(read);

    // Through the writer: the same scene built in memory.
    spade::Result<SceneDesc> scene = spade::scene::scene_from_yaml(kMinimalScene);
    ASSERT_OK(scene);
    scene->vehicles[0].model = "zz";
    const spade::Result<std::string> written = spade::scene::scene_to_yaml(*scene);
    EXPECT_TRUE(contains(why(written), "unknown model 'zz'")) << why(written);
    EXPECT_TRUE(contains(why(spade::scene::validate_scene(*scene)), "unknown model 'zz'"));
}

TEST(SceneFile, NamesAreUniqueAcrossAssetsAndVehicles) {
    const spade::Result<SceneDesc> base = spade::scene::scene_from_yaml(kMinimalScene);
    ASSERT_OK(base);
    ASSERT_OK(spade::scene::validate_scene(*base));

    {  // a vehicle named as an asset
        SceneDesc s = *base;
        s.vehicles[0].name = "a";
        EXPECT_TRUE(contains(why(spade::scene::validate_scene(s)), "'a'")) << why(spade::scene::validate_scene(s));
        EXPECT_TRUE(contains(why(spade::scene::validate_scene(s)), "assets and vehicles"));
    }
    {  // two assets of one name
        SceneDesc s = *base;
        s.assets.push_back(s.assets[0]);
        EXPECT_TRUE(contains(why(spade::scene::validate_scene(s)), "'a'"));
    }
    {  // two vehicles of one name
        SceneDesc s = *base;
        s.vehicles.push_back(s.vehicles[0]);
        EXPECT_TRUE(contains(why(spade::scene::validate_scene(s)), "'v'"));
    }
    {  // two materials of one name (SCN-004)
        SceneDesc s = *base;
        s.materials.push_back(s.materials[0]);
        EXPECT_TRUE(contains(why(spade::scene::validate_scene(s)), "duplicate material name 'm'"));
    }
    {  // two models of one name
        SceneDesc s = *base;
        s.models.push_back(s.models[0]);
        EXPECT_TRUE(contains(why(spade::scene::validate_scene(s)), "duplicate model name 'q'"));
    }
    {  // a material and a model may share a name: they are different namespaces
        SceneDesc s = *base;
        s.models[0].name = "m";
        s.vehicles[0].model = "m";
        EXPECT_OK(spade::scene::validate_scene(s));
    }
}

struct Refusal {
    std::string what;
    std::function<void(SceneDesc&)> edit;
    std::string needle;
};

TEST(SceneFile, TheValidatorRefusesWhatTheSceneAloneDecides) {
    const spade::Result<SceneDesc> base = spade::scene::scene_from_yaml(kMinimalScene);
    ASSERT_OK(base);
    constexpr float kNaN = std::numeric_limits<float>::quiet_NaN();

    const std::vector<Refusal> cases = {
        {"an empty scene name", [](SceneDesc& s) { s.name.clear(); }, "scene name"},
        {"no world file", [](SceneDesc& s) { s.world.file.clear(); }, "world.file"},
        {"a backslash in the world path", [](SceneDesc& s) { s.world.file = "worlds\\w.world.yaml"; },
         "forward slashes"},
        {"an absolute world path", [](SceneDesc& s) { s.world.file = "/srv/w.world.yaml"; }, "relative"},
        {"a drive-letter world path", [](SceneDesc& s) { s.world.file = "C:/w.world.yaml"; }, "relative"},
        {"an empty material name", [](SceneDesc& s) { s.materials[0].name.clear(); }, "material 0"},
        {"a non-finite material colour", [=](SceneDesc& s) { s.materials[0].base_color.x = kNaN; },
         "base_color"},
        {"a model version of 0", [](SceneDesc& s) { s.models[0].version = 0; }, "version must be >= 1"},
        {"a non-unit design rotation",
         [](SceneDesc& s) { s.models[0].design_to_principal = glm::quat(2.0f, 0.0f, 0.0f, 0.0f); },
         "design_to_principal must be a unit quaternion"},
        {"a model the model type refuses", [](SceneDesc& s) { s.models[0].body.mass = 0.0f; }, "body.mass"},
        {"an empty asset name", [](SceneDesc& s) { s.assets[0].name.clear(); }, "asset 0"},
        {"an asset scale of 0", [](SceneDesc& s) { s.assets[0].pose.scale = 0.0f; }, "scale"},
        {"a non-unit asset orientation",
         [](SceneDesc& s) { s.assets[0].pose.rotation = glm::quat(0.5f, 0.0f, 0.0f, 0.0f); },
         "unit quaternion"},
        {"a non-finite asset position", [=](SceneDesc& s) { s.assets[0].pose.position.y = kNaN; }, "finite"},
        {"a malformed collider", [](SceneDesc& s) { s.assets[0].collider.nodes[0].transform = 7; },
         "collider"},
        // What the SDF text encoding cannot carry: a scene must be expressible
        // as its file, or a save and reload composes to another config_hash.
        {"an operator node carrying a primitive kind",
         [](SceneDesc& s) {
             spade::SdfNode join = op(SdfOp::union_);
             join.kind = 3;
             s.assets[0].collider.nodes = {s.assets[0].collider.nodes[0], s.assets[0].collider.nodes[0], join};
             s.assets[0].collider_materials.clear();
         },
         "non-zero primitive kind"},
        {"a node with non-zero padding", [](SceneDesc& s) { s.assets[0].collider.nodes[0]._pad = 1; },
         "non-zero padding"},
        {"a transform with non-zero padding",
         [](SceneDesc& s) { s.assets[0].collider.transforms[0]._pad[0] = 1.0f; }, "non-zero padding"},
        {"collider material indices", [](SceneDesc& s) { s.assets[0].collider.node_materials = {0}; },
         "by name"},
        {"too few collider material names", [](SceneDesc& s) { s.assets[0].collider_materials.push_back("m"); },
         "collider_materials"},
        {"an empty collider material name", [](SceneDesc& s) { s.assets[0].collider_materials[0].clear(); },
         "node material"},
        {"collider transforms with no nodes",
         [](SceneDesc& s) {
             s.assets[0].collider.nodes.clear();
             s.assets[0].collider_materials.clear();
         },
         "no nodes"},
        {"a visual material without a mesh", [](SceneDesc& s) { s.assets[0].visual.mesh_ref.clear(); },
         "visual"},
        {"a mesh without a visual material", [](SceneDesc& s) { s.assets[0].visual.material.clear(); },
         "visual"},
        {"an asset with neither half",
         [](SceneDesc& s) {
             s.assets[0].collider = {};
             s.assets[0].collider_materials.clear();
             s.assets[0].visual = {};
         },
         "neither"},
        {"an empty vehicle name", [](SceneDesc& s) { s.vehicles[0].name.clear(); }, "vehicle 0"},
        {"an unknown model", [](SceneDesc& s) { s.vehicles[0].model = "zz"; }, "unknown model 'zz'"},
        {"a non-unit start orientation",
         [](SceneDesc& s) { s.vehicles[0].start.orient = glm::quat(0.0f, 0.0f, 0.0f, 0.0f); },
         "unit quaternion"},
        {"a non-finite start velocity", [=](SceneDesc& s) { s.vehicles[0].start.vel.z = kNaN; }, "finite"},
        {"a negative rotor speed", [](SceneDesc& s) { s.vehicles[0].start.rotor_omega = -1.0f; },
         "rotor_omega"},
    };

    for (const Refusal& c : cases) {
        SCOPED_TRACE(c.what);
        SceneDesc s = *base;
        c.edit(s);
        const spade::Result<void> r = spade::scene::validate_scene(s);
        ASSERT_FALSE(r);
        EXPECT_EQ(r.error().code, spade::Code::invalid_argument);
        EXPECT_TRUE(contains(r.error().context, c.needle)) << r.error().context;
    }
}

TEST(SceneFile, AnAssetMayCarryEitherHalfAlone) {
    const spade::Result<SceneDesc> base = spade::scene::scene_from_yaml(kMinimalScene);
    ASSERT_OK(base);
    {  // collision only
        SceneDesc s = *base;
        s.assets[0].visual = {};
        ASSERT_OK(spade::scene::validate_scene(s));
        const spade::Result<std::string> text = spade::scene::scene_to_yaml(s);
        ASSERT_OK(text);
        EXPECT_TRUE(contains(*text, "visual: {mesh_ref: \"\", material: \"\"}")) << *text;
        ASSERT_OK(spade::scene::scene_from_yaml(*text));
    }
    {  // visual only
        SceneDesc s = *base;
        s.assets[0].collider = {};
        s.assets[0].collider_materials.clear();
        ASSERT_OK(spade::scene::validate_scene(s));
        const spade::Result<std::string> text = spade::scene::scene_to_yaml(s);
        ASSERT_OK(text);
        EXPECT_TRUE(contains(*text, "transforms: []")) << *text;
        EXPECT_TRUE(contains(*text, "nodes: []")) << *text;
        EXPECT_TRUE(contains(*text, "node_materials: []")) << *text;
        ASSERT_OK(spade::scene::scene_from_yaml(*text));
    }
}

// ===========================================================================
// CORE-7: a model's parts, and scene version 2 (the parts and meshes plan,
// Task A3). Version 2 is written ONLY when a model has parts, so every scene
// without them -- gate_run's golden and every committed scene -- is still
// version 1, byte for byte (AScenesTextRoundTripsByteForByte and
// EditorFiles.EveryCommittedSceneSavesBackByteForByte, unedited, pin that).
// ===========================================================================

using spade::vehicles::PartAttach;
using spade::vehicles::PartDesc;
using spade::vehicles::PartSurface;

// Rotations about +Y, as exact float constants (TD-3: no libm here).
const glm::quat kYawPlus45(0.923879504f, 0.0f, 0.382683456f, 0.0f);

// A one-transform, one-primitive shape.
[[nodiscard]] spade::SdfProgram solid(SdfPrim kind, glm::vec4 params) {
    spade::SdfProgram p;
    p.transforms.push_back(SdfTransform{});
    p.nodes.push_back(prim(kind, 0, params));
    return p;
}

// A cylinder arm, mounted out along the diagonal and yawed 45 degrees.
[[nodiscard]] PartDesc arm_part() {
    PartDesc p{};
    p.name = "arm_fl";
    p.role = "arm";
    p.mount_pos = {0.064f, 0.0f, 0.064f};
    p.mount_orient = kYawPlus45;
    p.shape = solid(SdfPrim::cylinder, {0.006f, 0.07f, 0.0f, 0.0f});
    p.min_feature = 0.012f;
    return p;
}

// kMinimalScene's model with one part, as a version-2 file. Every mapping a
// key test mutates has a key/value pair that occurs nowhere else in the text.
constexpr const char* kImuLine =
    "    imu_mounts: [{mount_pos: [0, 0, 0], mount_orient: [1, 0, 0, 0], rate_divider: 1, sigma_a: 0, sigma_g: 0, "
    "sigma_ba: 0, sigma_bg: 0}]\n";
constexpr const char* kPartShape =
    R"({sdf: {transforms: [{world_to_local: [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1], scale: 2}], nodes: [{params: [0.75, 0.125, 0.125, 0], prim: box, transform: 0}]}})";
[[nodiscard]] std::string parts_block() {
    return std::string("    parts:\n"
                       "      - name: \"arm\"\n"
                       "        role: \"strut\"\n"
                       "        attach: \"body\"\n"
                       "        rotor: 0\n"
                       "        mount: {position: [0.5, 0, 0], orientation: [1, 0, 0, 0]}\n"
                       "        surface: {kind: \"wall\", inv_permeability: 0, inertial_coeff: 0}\n"
                       "        min_feature: 0.25\n"
                       "        shape: ") +
           kPartShape + "\n";
}
[[nodiscard]] std::string minimal_parts_scene() {
    return with(with(kMinimalScene, "scene_version: 1\n", "scene_version: 2\n"), kImuLine,
                std::string(kImuLine) + parts_block());
}

TEST(SceneFile, ASceneWithPartsIsVersion2AndOneWithoutIsVersion1) {
    // An old file still loads, and is still written as version 1: a scene
    // with no parts anywhere is the text it always was.
    const spade::Result<SceneDesc> plain = spade::scene::scene_from_yaml(kMinimalScene);
    ASSERT_OK(plain);
    const spade::Result<std::string> plain_text = spade::scene::scene_to_yaml(*plain);
    ASSERT_OK(plain_text);
    EXPECT_TRUE(contains(*plain_text, "\nscene_version: 1\n")) << *plain_text;
    EXPECT_TRUE(contains(*plain_text, "# Spade scene file -- schema version 1.\n"));
    EXPECT_FALSE(contains(*plain_text, "parts:")) << *plain_text;

    SceneDesc dressed = *plain;
    dressed.models.at(0).parts.push_back(arm_part());
    const spade::Result<std::string> text = spade::scene::scene_to_yaml(dressed);
    ASSERT_OK(text);
    EXPECT_TRUE(contains(*text, "\nscene_version: 2\n")) << *text;
    EXPECT_TRUE(contains(*text, "# Spade scene file -- schema version 2.\n"));
    EXPECT_TRUE(contains(*text, "\n    parts:\n      - name: \"arm_fl\"\n        role: \"arm\"\n")) << *text;
    const spade::Result<SceneDesc> back = spade::scene::scene_from_yaml(*text);
    ASSERT_OK(back);
    ASSERT_EQ(back->models.at(0).parts.size(), 1u);
    EXPECT_EQ(back->models.at(0).parts[0].name, "arm_fl");
    const spade::Result<std::string> again = spade::scene::scene_to_yaml(*back);
    ASSERT_OK(again);
    EXPECT_TRUE(*again == *text) << "byte for byte (SCN-002): " << first_difference(*text, *again);
}

TEST(SceneFile, PartsRoundTripBitForBit) {
    const spade::Result<SceneDesc> base = spade::scene::scene_from_yaml(kMinimalScene);
    ASSERT_OK(base);
    SceneDesc s = *base;
    s.models[0].rotors.push_back(s.models[0].rotors[0]);  // two rotors; a part rides the second

    // A rigid wall: a three-node CSG shape, its second primitive under a
    // rotated, translated, scaled transform; a mount with a negative zero.
    PartDesc hull{};
    hull.name = "hull";
    hull.role = "body";
    hull.mount_pos = {0.1f, -0.0f, -0.3f};
    hull.mount_orient = kYawPlus30;
    SdfTransform turned{};
    const float inv_scale = 1.0f / 1.25f;
    turned.world_to_local[0] = glm::vec4(0.866025388f * inv_scale, 0.0f, -0.5f * inv_scale, 0.0f);
    turned.world_to_local[1] = glm::vec4(0.0f, inv_scale, 0.0f, 0.0f);
    turned.world_to_local[2] = glm::vec4(0.5f * inv_scale, 0.0f, 0.866025388f * inv_scale, 0.0f);
    turned.world_to_local[3] = glm::vec4(-0.1f / 3.0f, 0.025f, 1.0f / 7.0f, 1.0f);
    turned.scale = 1.25f;
    hull.shape.transforms = {SdfTransform{}, turned};
    hull.shape.nodes = {prim(SdfPrim::box, 0, {0.07f, 0.02f, 0.05f, 0.0f}),
                        prim(SdfPrim::capsule, 1, {0.011f, 0.033f, 0.0f, 0.0f}), op(SdfOp::smooth_union)};
    hull.shape.nodes[2].params.x = 0.0035f;
    hull.min_feature = 0.012f;

    // A porous part on rotor 1, both coefficients set.
    PartDesc duct{};
    duct.name = "duct";
    duct.role = "guard";
    duct.attach = PartAttach::rotor;
    duct.rotor = 1;
    duct.mount_pos = {0.0f, 0.004f, 0.0f};
    duct.mount_orient = kYawMinus15;
    duct.shape = solid(SdfPrim::torus, {0.0635f, 0.0045f, 0.0f, 0.0f});
    duct.surface = {.kind = PartSurface::Kind::porous, .inv_permeability = 3.3e6f, .inertial_coeff = 17.25f};
    duct.min_feature = 1.0e-3f;

    // A rotor wall on rotor 0, and a porous body part with one coefficient.
    PartDesc blade{};
    blade.name = "blade";
    blade.role = "blade";
    blade.attach = PartAttach::rotor;
    blade.mount_pos = {0.03f, 0.0f, 0.0f};
    blade.shape = solid(SdfPrim::box, {0.03f, 0.0008f, 0.006f, 0.0f});
    blade.min_feature = 0.0016f;
    PartDesc screen{};
    screen.name = "screen";
    screen.role = "filter";
    screen.mount_orient = kYawPlus90;
    screen.shape = solid(SdfPrim::sphere, {0.021f, 0.0f, 0.0f, 0.0f});
    screen.surface = {.kind = PartSurface::Kind::porous, .inv_permeability = 0.0f, .inertial_coeff = 0.7f};
    screen.min_feature = 2.0e-4f;

    s.models[0].parts = {hull, duct, blade, screen};
    const spade::Result<std::string> text = spade::scene::scene_to_yaml(s);
    ASSERT_OK(text);
    const spade::Result<SceneDesc> back = spade::scene::scene_from_yaml(*text);
    ASSERT_OK(back);
    const spade::Result<std::string> again = spade::scene::scene_to_yaml(*back);
    ASSERT_OK(again);
    EXPECT_TRUE(*again == *text) << first_difference(*text, *again);

    const std::vector<PartDesc>& built = s.models[0].parts;
    const std::vector<PartDesc>& read = back->models[0].parts;
    ASSERT_EQ(read.size(), built.size());
    for (std::size_t i = 0; i < built.size(); ++i) {
        SCOPED_TRACE(built[i].name);
        const PartDesc& a = built[i];
        const PartDesc& b = read[i];
        EXPECT_EQ(b.name, a.name);
        EXPECT_EQ(b.role, a.role);
        EXPECT_EQ(b.attach, a.attach);
        EXPECT_EQ(b.rotor, a.rotor);
        EXPECT_EQ(std::memcmp(&a.mount_pos, &b.mount_pos, sizeof a.mount_pos), 0);
        EXPECT_EQ(std::memcmp(&a.mount_orient, &b.mount_orient, sizeof a.mount_orient), 0);
        EXPECT_EQ(b.surface.kind, a.surface.kind);
        EXPECT_EQ(std::memcmp(&a.surface.inv_permeability, &b.surface.inv_permeability, sizeof(float)), 0);
        EXPECT_EQ(std::memcmp(&a.surface.inertial_coeff, &b.surface.inertial_coeff, sizeof(float)), 0);
        EXPECT_EQ(std::memcmp(&a.min_feature, &b.min_feature, sizeof(float)), 0);
        ASSERT_EQ(b.shape.transforms.size(), a.shape.transforms.size());
        ASSERT_EQ(b.shape.nodes.size(), a.shape.nodes.size());
        EXPECT_EQ(std::memcmp(a.shape.transforms.data(), b.shape.transforms.data(),
                              a.shape.transforms.size() * sizeof(SdfTransform)),
                  0);
        EXPECT_EQ(std::memcmp(a.shape.nodes.data(), b.shape.nodes.data(), a.shape.nodes.size() * sizeof(SdfNode)), 0);
        EXPECT_TRUE(b.shape.node_materials.empty());
    }
    EXPECT_TRUE(std::signbit(read[0].mount_pos.y)) << "a negative zero is a bit pattern too";
}

TEST(SceneFile, AVersion1FileMayNotCarryParts) {
    // parts in a version-1 file: an unknown key, blamed on version 1, at its line.
    const std::string v1 = with(kMinimalScene, kImuLine, std::string(kImuLine) + parts_block());
    const spade::Result<SceneDesc> refused = spade::scene::scene_from_yaml(v1);
    ASSERT_FALSE(refused);
    EXPECT_EQ(refused.error().code, spade::Code::invalid_argument);
    EXPECT_TRUE(contains(why(refused), "unknown key 'parts' in scene.models[0] -- scene schema v1 does not define it"))
        << why(refused);
    EXPECT_TRUE(contains(why(refused), "(line 17, column 5)")) << why(refused);

    // The same text as version 2 reads.
    const spade::Result<SceneDesc> v2 = spade::scene::scene_from_yaml(minimal_parts_scene());
    ASSERT_OK(v2);
    ASSERT_EQ(v2->models[0].parts.size(), 1u);
    EXPECT_EQ(v2->models[0].parts[0].role, "strut");

    // A version-2 file with no parts reads, and is written back as version 1:
    // the version is a function of the content.
    const spade::Result<SceneDesc> bare =
        spade::scene::scene_from_yaml(with(kMinimalScene, "scene_version: 1\n", "scene_version: 2\n"));
    ASSERT_OK(bare);
    const spade::Result<std::string> text = spade::scene::scene_to_yaml(*bare);
    ASSERT_OK(text);
    EXPECT_TRUE(contains(*text, "\nscene_version: 1\n")) << *text;
    const spade::Result<SceneDesc> plain = spade::scene::scene_from_yaml(kMinimalScene);
    ASSERT_OK(plain);
    const spade::Result<std::string> plain_text = spade::scene::scene_to_yaml(*plain);
    ASSERT_OK(plain_text);
    EXPECT_TRUE(*text == *plain_text) << first_difference(*plain_text, *text);
}

TEST(SceneFile, APartsShapeMustBeRepresentable) {
    const spade::Result<SceneDesc> base = spade::scene::scene_from_yaml(kMinimalScene);
    ASSERT_OK(base);
    const std::vector<Refusal> cases = {
        {"a node with non-zero padding", [](SceneDesc& s) { s.models[0].parts[0].shape.nodes[0]._pad = 1; },
         "node 0 has non-zero padding"},
        {"a transform with non-zero padding",
         [](SceneDesc& s) { s.models[0].parts[0].shape.transforms[0]._pad[2] = 1.0f; },
         "transform 0 has non-zero padding"},
        {"an operator node carrying a primitive kind",
         [](SceneDesc& s) {
             spade::SdfProgram& shape = s.models[0].parts[0].shape;
             spade::SdfNode join = op(SdfOp::union_);
             join.kind = 2;
             shape.nodes = {shape.nodes[0], shape.nodes[0], join};
         },
         "operator node 2 carries a non-zero primitive kind"},
    };
    for (const Refusal& c : cases) {
        SCOPED_TRACE(c.what);
        SceneDesc s = *base;
        s.models[0].parts.push_back(arm_part());
        c.edit(s);
        ASSERT_OK(s.models[0].validate()) << "the model alone accepts it: the text encoding is the scene's rule";
        const spade::Result<void> validated = spade::scene::validate_scene(s);
        ASSERT_FALSE(validated);
        EXPECT_EQ(validated.error().code, spade::Code::invalid_argument);
        EXPECT_TRUE(contains(why(validated), "model 'q' part 'arm_fl' shape " + c.needle)) << why(validated);
        const spade::Result<std::string> written = spade::scene::scene_to_yaml(s);
        ASSERT_FALSE(written);
        EXPECT_TRUE(contains(why(written), "model 'q' part 'arm_fl' shape " + c.needle)) << why(written);
    }

    // node_materials is render data, which a part never carries: under a
    // part's sdf it is an unknown key, at its line.
    const spade::Result<SceneDesc> r = spade::scene::scene_from_yaml(
        with(minimal_parts_scene(), "{sdf: {transforms: [{world_to_local",
             "{sdf: {node_materials: [\"m\"], transforms: [{world_to_local"));
    EXPECT_TRUE(contains(why(r), "unknown key 'node_materials' in scene.models[0].parts[0].shape.sdf -- scene "
                                 "schema v2 does not define it (line "))
        << why(r);
}

TEST(SceneFile, APartsKeysAreRefusedAtEveryLevel) {
    const std::string base = minimal_parts_scene();
    ASSERT_OK(spade::scene::scene_from_yaml(base));

    const std::string shape = kPartShape;
    const std::string shape_body = shape.substr(1, shape.size() - 2) + ", ";  // "sdf: {...}, "
    const std::vector<KeyCase> cases = {
        block("scene.models[0].parts[0]", "        role: \"strut\"\n", "        "),
        flow("scene.models[0].parts[0].mount", "position: [0.5, 0, 0], "),
        flow("scene.models[0].parts[0].surface", "inv_permeability: 0, "),
        {"scene.models[0].parts[0].shape", shape, "{bogus: 1, " + shape.substr(1), "{}",
         "{" + shape_body + shape.substr(1)},
        flow("scene.models[0].parts[0].shape.sdf",
             "transforms: [{world_to_local: [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1], scale: 2}], "),
        flow("scene.models[0].parts[0].shape.sdf.transforms[0]",
             "world_to_local: [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1], "),
        flow("scene.models[0].parts[0].shape.sdf.nodes[0]", "params: [0.75, 0.125, 0.125, 0], "),
    };
    for (const KeyCase& c : cases) {
        SCOPED_TRACE(c.level);
        const spade::Result<SceneDesc> unknown = spade::scene::scene_from_yaml(with(base, c.needle, c.unknown));
        EXPECT_TRUE(contains(why(unknown), "unknown key 'bogus' in " + c.level + " -- scene schema v2"))
            << why(unknown);
        EXPECT_TRUE(contains(why(unknown), "(line ")) << why(unknown);

        const spade::Result<SceneDesc> missing = spade::scene::scene_from_yaml(with(base, c.needle, c.missing));
        EXPECT_TRUE(contains(why(missing), "missing required key")) << why(missing);
        EXPECT_TRUE(contains(why(missing), c.level)) << why(missing);
        EXPECT_TRUE(contains(why(missing), "(line ")) << why(missing);

        const spade::Result<SceneDesc> duplicate =
            spade::scene::scene_from_yaml(with(base, c.needle, c.duplicate));
        EXPECT_TRUE(contains(why(duplicate), "duplicate key")) << why(duplicate);
        EXPECT_TRUE(contains(why(duplicate), c.level)) << why(duplicate);
    }

    // The two enumerations are names, refused at their line.
    const auto refused = [&](const std::string& from, const std::string& to, const std::string& needle) {
        const spade::Result<SceneDesc> r = spade::scene::scene_from_yaml(with(base, from, to));
        EXPECT_FALSE(r) << "accepted: " << to;
        EXPECT_TRUE(contains(why(r), needle)) << why(r);
        EXPECT_TRUE(contains(why(r), "(line ")) << why(r);
    };
    refused("attach: \"body\"", "attach: \"wheel\"", "scene.models[0].parts[0].attach: unknown part attachment 'wheel'");
    refused("kind: \"wall\"", "kind: \"solid\"", "scene.models[0].parts[0].surface.kind: unknown part surface 'solid'");
    refused("min_feature: 0.25", "min_feature: tiny", "scene.models[0].parts[0].min_feature: 'tiny' is not a decimal");

    // `parts` is a list. An empty one is no parts, and is written as none.
    const std::string v2 = with(kMinimalScene, "scene_version: 1\n", "scene_version: 2\n");
    const spade::Result<SceneDesc> mapping =
        spade::scene::scene_from_yaml(with(v2, kImuLine, std::string(kImuLine) + "    parts: {}\n"));
    EXPECT_TRUE(contains(why(mapping), "scene.models[0].parts must be a sequence (line 17")) << why(mapping);
    const spade::Result<SceneDesc> empty =
        spade::scene::scene_from_yaml(with(v2, kImuLine, std::string(kImuLine) + "    parts: []\n"));
    ASSERT_OK(empty);
    EXPECT_TRUE(empty->models[0].parts.empty());
}

TEST(SceneFile, APartTheModelRefusesIsRefusedWithTheModelsOwnMessage) {
    // The part rules are ModelType::issues()'s (A1), run by validate_scene()
    // through ModelType::validate(): the scene repeats none of them (TD-9).
    const spade::Result<SceneDesc> base = spade::scene::scene_from_yaml(minimal_parts_scene());
    ASSERT_OK(base);
    struct Case {
        std::string what;
        std::string from;
        std::string to;
        std::function<void(PartDesc&)> edit;
        std::string needle;
    };
    const std::vector<Case> cases = {
        {"a rotor past the model's one rotor", "attach: \"body\"\n        rotor: 0\n",
         "attach: \"rotor\"\n        rotor: 1\n",
         [](PartDesc& p) {
             p.attach = PartAttach::rotor;
             p.rotor = 1;
         },
         "part 0 'arm': rotor 1 is out of range"},
        {"a coefficient on a wall", "inv_permeability: 0, ", "inv_permeability: 2, ",
         [](PartDesc& p) { p.surface.inv_permeability = 2.0f; }, "part 0 'arm': surface.inv_permeability must be 0"},
        {"no smallest feature", "min_feature: 0.25", "min_feature: 0",
         [](PartDesc& p) { p.min_feature = 0.0f; }, "part 0 'arm': min_feature must be finite and > 0"},
        {"an unbounded shape", "params: [0.75, 0.125, 0.125, 0], prim: box", "params: [0, 1, 0, 0], prim: plane",
         [](PartDesc& p) { p.shape.nodes[0] = prim(SdfPrim::plane, 0, {0.0f, 1.0f, 0.0f, 0.0f}); },
         "part 0 'arm': shape is unbounded"},
    };
    for (const Case& c : cases) {
        SCOPED_TRACE(c.what);
        SceneDesc s = *base;
        c.edit(s.models[0].parts[0]);
        const spade::Result<void> model = s.models[0].validate();
        ASSERT_FALSE(model);
        EXPECT_TRUE(contains(model.error().context, c.needle)) << model.error().context;
        const spade::Result<void> scene = spade::scene::validate_scene(s);
        ASSERT_FALSE(scene);
        EXPECT_EQ(scene.error().code, model.error().code);
        EXPECT_EQ(scene.error().context, model.error().context) << "the model's message, word for word";

        // And through the reader, from the text.
        const spade::Result<SceneDesc> read = spade::scene::scene_from_yaml(with(minimal_parts_scene(), c.from, c.to));
        ASSERT_FALSE(read);
        EXPECT_EQ(read.error().context, model.error().context) << "the model's message, word for word";
    }
}

TEST(SceneFile, PartsTextMatchesTheGolden) {
    // tests/golden/scenes/parts.scene.yaml is HAND-WRITTEN from the plan's
    // schema (TD-4), not generated by the writer, so this test never writes
    // it: a missing file is a failure. It is final once a fresh Docker gcc
    // leg reproduces it (TD-12).
    const std::filesystem::path path = golden("scenes/parts.scene.yaml");
    std::string committed;
    ASSERT_TRUE(read_file(path, committed)) << path << " is missing";
    const std::size_t body = provenance_end(committed);
    ASSERT_GT(body, 0u) << path << " carries no '" << kProvenance << "' lines (TD-1)";
    const std::string canonical = committed.substr(body);

    const spade::Result<SceneDesc> loaded = spade::scene::load_scene_file(path);
    ASSERT_OK(loaded);
    const spade::Result<std::string> text = spade::scene::scene_to_yaml(*loaded);
    ASSERT_OK(text);
    EXPECT_TRUE(canonical == *text) << first_difference(canonical, *text);

    // What the text says, read by hand from it.
    ASSERT_EQ(loaded->models.size(), 2u);
    EXPECT_TRUE(loaded->models[1].parts.empty()) << "a model with no parts writes none, even in version 2";
    const std::vector<PartDesc>& parts = loaded->models[0].parts;
    ASSERT_EQ(parts.size(), 4u);
    EXPECT_EQ(parts[0].name, "frame");
    EXPECT_EQ(parts[1].mount_orient, kYawMinus45);
    EXPECT_EQ(parts[1].shape.transforms.at(0).world_to_local[1][0], -1.0f);
    EXPECT_EQ(parts[2].surface.kind, PartSurface::Kind::porous);
    EXPECT_EQ(parts[2].surface.inv_permeability, 1.0e6f);
    EXPECT_EQ(parts[2].shape.nodes.at(2).op, static_cast<uint32_t>(SdfOp::subtract));
    EXPECT_EQ(parts[3].attach, PartAttach::rotor);
    EXPECT_EQ(parts[3].rotor, 1u);
    EXPECT_EQ(parts[3].shape.transforms.at(0).scale, 0.5f);
    EXPECT_EQ(parts[3].min_feature, 0.00390625f);
}

// ===========================================================================
// SCN-001: the world hash
// ===========================================================================

TEST(SceneWorldHash, IsFnv1a64OverTheWorldsCanonicalText) {
    const spade::Result<WorldDesc> world = spade::load_world_file(golden("worlds/gate.world.yaml"));
    ASSERT_OK(world);
    const spade::Result<std::string> text = spade::world_to_yaml(*world);
    ASSERT_OK(text);
    const spade::Result<uint64_t> hash = spade::scene::world_hash(*world);
    ASSERT_OK(hash);
    EXPECT_EQ(*hash, spade::rng::fnv1a64(*text));
}

TEST(SceneWorldHash, ALoadAndSaveKeepsIt) {
    std::string committed;
    ASSERT_TRUE(read_file(golden("worlds/gate.world.yaml"), committed));
    const spade::Result<WorldDesc> once = spade::world_from_yaml(committed);
    ASSERT_OK(once);
    const spade::Result<std::string> written = spade::world_to_yaml(*once);
    ASSERT_OK(written);
    const spade::Result<WorldDesc> twice = spade::world_from_yaml(*written);
    ASSERT_OK(twice);
    const spade::Result<uint64_t> a = spade::scene::world_hash(*once);
    const spade::Result<uint64_t> b = spade::scene::world_hash(*twice);
    ASSERT_OK(a);
    ASSERT_OK(b);
    EXPECT_EQ(*a, *b);

    // A hand edit that changes no content -- a comment, a blank line -- keeps it,
    // because the hash is over write(load(file)), not the file's bytes.
    const spade::Result<WorldDesc> edited =
        spade::world_from_yaml("# a note a person added\n\n" + committed + "\n# and another\n");
    ASSERT_OK(edited);
    const spade::Result<uint64_t> c = spade::scene::world_hash(*edited);
    ASSERT_OK(c);
    EXPECT_EQ(*a, *c);
}

TEST(SceneWorldHash, TwoDifferentWorldsHashDifferently) {
    const spade::Result<WorldDesc> gate = spade::load_world_file(golden("worlds/gate.world.yaml"));
    const spade::Result<WorldDesc> maximal = spade::load_world_file(golden("worlds/maximal.world.yaml"));
    ASSERT_OK(gate);
    ASSERT_OK(maximal);
    const spade::Result<uint64_t> g = spade::scene::world_hash(*gate);
    const spade::Result<uint64_t> m = spade::scene::world_hash(*maximal);
    ASSERT_OK(g);
    ASSERT_OK(m);
    EXPECT_NE(*g, *m);

    // One ulp of gravity is another world.
    WorldDesc nudged = *gate;
    nudged.environment.gravity.y = std::nextafter(nudged.environment.gravity.y, 0.0f);
    const spade::Result<uint64_t> n = spade::scene::world_hash(nudged);
    ASSERT_OK(n);
    EXPECT_NE(*g, *n);
}

}  // namespace
