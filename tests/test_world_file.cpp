// ---------------------------------------------------------------------------
// The versioned YAML world file (engine design D7) -- writer, loader, and the
// shared validation both of them run.
//
// WHAT THIS FILE HAS TO PROVE, and why each claim needs its own shape of test:
//
//   BIT-EXACTNESS. A world that goes to text and back is the SAME world, not a
//   close one. Checked by memcmp over the SDF's two POD arrays, because that
//   is the only comparison a rounded float cannot pass by accident.
//
//   BEHAVIOURAL EQUIVALENCE. Bit-equal descriptions are worth having because
//   they produce bit-equal RUNS; the digest test runs the same scenario on the
//   original and the loaded world and demands one number.
//
//   CANONICAL EMISSION. The writer is a pure function of the WorldDesc, so
//   yaml -> desc -> yaml is the identity on text. That is what lets
//   tests/golden/worlds/*.world.yaml be committed data compared byte for byte.
//
//   LOCALE IMMUNITY. Under a comma-decimal global locale -- the state an
//   embedding application can put this process in without asking -- every one
//   of the above still holds. This is the test that would fail if anything in
//   the conversion path reached for an iostream, a strtof or a snprintf.
//
//   REJECTION. A loader that accepts a broken file is worse than no loader.
//   One adversarial case per rule, each asserting the DIAGNOSTIC and not just
//   the failure, because "it returned an error" is not the contract -- "it
//   said which key, on which line" is.
// ---------------------------------------------------------------------------

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <clocale>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <locale>
#include <string>
#include <string_view>
#include <system_error>
#include <utility>
#include <vector>

#include <glm/gtc/quaternion.hpp>
#include <glm/vec2.hpp>
#include <glm/vec3.hpp>

#include "sim/simulation.hpp"
#include "sim/world_set.hpp"
#include "testing/replay.hpp"
#include "world/builder.hpp"
#include "world/world_file.hpp"
#include "world/world_ref.hpp"

namespace {

using spade::Capacities;
using spade::Environment;
using spade::LightingDesc;
using spade::MaterialDesc;
using spade::MaterialShading;
using spade::PropDesc;
using spade::SdfPose;
using spade::SpawnPoint;
using spade::WorldBuilder;
using spade::WorldDesc;

// Same GoogleTest plumbing as tests/test_determinism.cpp: a failed Result
// prints its code and context, which is the difference between "it failed" and
// "world.sdf.nodes[3].params[1]: 'nan' is not finite (line 41, column 38)".
template <class T>
[[nodiscard]] testing::AssertionResult IsOk(const char* expr, const spade::Result<T>& r) {
    if (r) return testing::AssertionSuccess();
    return testing::AssertionFailure() << expr << " failed: [" << static_cast<int>(r.error().code)
                                       << "] " << r.error().context;
}

#define ASSERT_OK(expr) ASSERT_PRED_FORMAT1(IsOk, expr)
#define EXPECT_OK(expr) EXPECT_PRED_FORMAT1(IsOk, expr)

template <class T>
[[nodiscard]] int code_of(const spade::Result<T>& r) {
    return r ? -1 : static_cast<int>(r.error().code);
}

[[nodiscard]] constexpr int code(spade::Code c) { return static_cast<int>(c); }

// The context of a Result that is EXPECTED to have failed. Empty when it
// succeeded, so a wrongly-passing negative test reads as a missing message
// rather than reaching into an inactive union member.
template <class T>
[[nodiscard]] std::string why(const spade::Result<T>& r) {
    return r ? std::string{} : r.error().context;
}

// ---------------------------------------------------------------------------
// Text helpers
// ---------------------------------------------------------------------------

[[nodiscard]] bool contains(std::string_view haystack, std::string_view needle) {
    return haystack.find(needle) != std::string_view::npos;
}

[[nodiscard]] std::string replace_first(std::string text, std::string_view from,
                                        std::string_view to) {
    const std::size_t at = text.find(from);
    EXPECT_NE(at, std::string::npos) << "test fixture is stale: '" << from << "' not in the world text";
    if (at == std::string::npos) return text;
    text.replace(at, from.size(), to);
    return text;
}

// A one-line report of where two texts first differ. gtest's own string diff on
// a 4 KB YAML document is unreadable; this points at the byte.
[[nodiscard]] std::string first_difference(const std::string& expected,
                                           const std::string& actual) {
    std::size_t i = 0;
    while (i < expected.size() && i < actual.size() && expected[i] == actual[i]) ++i;
    if (i == expected.size() && i == actual.size()) return "identical";
    std::size_t line = 1;
    std::size_t line_begin = 0;
    for (std::size_t j = 0; j < i; ++j) {
        if (expected[j] == '\n') {
            ++line;
            line_begin = j + 1;
        }
    }
    const auto excerpt = [&](const std::string& s) {
        if (line_begin >= s.size()) return std::string("<end of text>");
        const std::size_t end = std::min(s.find('\n', line_begin), s.size());
        return s.substr(line_begin, end - line_begin);
    };
    return "first difference at byte " + std::to_string(i) + " (line " + std::to_string(line) +
           ")\n  committed: " + excerpt(expected) + "\n  regenerated: " + excerpt(actual);
}

// ---------------------------------------------------------------------------
// The maximal world: every primitive, every operator, poses with a rotation
// and a scale != 1, several spawns, several visuals.
//
// NO TRANSCENDENTAL BUILDS ANY POSE HERE. The rotations are exact unit
// quaternions or ones glm::normalize() (a sqrt, which IEEE mandates correctly
// rounded) makes unit -- never glm::angleAxis(), whose sin/cos are libm and
// therefore not guaranteed identical on the two compilers this suite runs on.
// The bytes these poses produce end up in committed .world.yaml files that CI
// compares byte for byte against a Linux/gcc rebuild, so "almost certainly the
// same" is not the standard.
// ---------------------------------------------------------------------------

[[nodiscard]] SdfPose pose(glm::vec3 position, glm::quat rotation, float scale) {
    SdfPose p;
    p.position = position;
    p.rotation = rotation;
    p.scale = scale;
    return p;
}

[[nodiscard]] WorldDesc maximal_world() {
    Environment env;
    env.gravity = glm::vec3(0.100000001f, -9.80665016f, -0.200000003f);
    env.wind = glm::vec3(1.5f, 0.0f, -0.25f);
    env.air_density = 1.19000006f;
    env.temperature_k = 291.5f;
    env.seed = 0xFEEDFACECAFEBEEFULL;  // exercises the full 64-bit range

    Capacities caps;
    caps.bodies = 8;
    caps.force_elements = 4;
    caps.sensors = 2;
    caps.contacts = 16;

    // Schema v2's own showcase alongside the SDF program below: three
    // materials spanning all three MaterialShading values, non-default
    // lighting, and two render-only props.
    LightingDesc lighting;
    lighting.sun_direction = glm::vec3(0.400000006f, 0.699999988f, -0.300000012f);
    lighting.sun_color = glm::vec3(1.0f, 0.949999988f, 0.899999976f);
    lighting.sun_intensity = 2.5f;
    lighting.ambient_color = glm::vec3(0.150000006f, 0.170000002f, 0.200000003f);
    lighting.sky_zenith = glm::vec3(0.200000003f, 0.400000006f, 0.699999988f);
    lighting.sky_horizon = glm::vec3(0.850000024f, 0.870000005f, 0.899999976f);

    MaterialDesc asphalt;
    asphalt.name = "asphalt";
    asphalt.base_color = glm::vec4(0.200000003f, 0.200000003f, 0.210000008f, 1.0f);
    asphalt.shading = MaterialShading::lambert;

    MaterialDesc beacon;
    beacon.name = "beacon";
    beacon.base_color = glm::vec4(1.0f, 0.300000012f, 0.100000001f, 1.0f);
    beacon.shading = MaterialShading::emissive;

    MaterialDesc decal;
    decal.name = "decal";
    decal.base_color = glm::vec4(0.899999976f, 0.899999976f, 0.899999976f, 1.0f);
    decal.shading = MaterialShading::unlit;

    // Postfix, 7 primitives and 6 operators, reducing to exactly one value:
    //   plane sphere UNION box INTERSECT cylinder capsule SUBTRACT SMOOTH_UNION
    //   torus heightfield UNION UNION
    // Peak evaluation depth 3.
    const spade::Result<WorldDesc> world =
        WorldBuilder()
            .name("maximal")
            .environment(env)
            .capacities(caps)
            .lighting(lighting)
            .material(asphalt)  // index 0 -- the palette's default
            .material(beacon)   // index 1
            .material(decal)    // index 2
            .spawn("start", glm::vec3(0.0f, 1.25f, -6.0f))
            .spawn("mid", glm::vec3(-2.5f, 3.0f, 0.5f),
                   glm::normalize(glm::quat(0.5f, 0.5f, -0.5f, 0.5f)))
            .spawn("finish", glm::vec3(4.0f, 0.75f, 7.25f),
                   glm::normalize(glm::quat(0.600000024f, 0.0f, 0.800000012f, 0.0f)))
            .visual("mesh:track/gate_ring")
            .visual("material:track/asphalt")
            .visual("mesh:props/banner")
            .plane(glm::vec3(0.0f, 1.0f, 0.0f), 0.0f)
            .sphere(2.5f, pose(glm::vec3(1.25f, -0.5f, 2.0f),
                               glm::normalize(glm::quat(0.600000024f, 0.800000012f, 0.0f, 0.0f)),
                               1.75f))
            .material_for_last_node(1)  // the sphere is the beacon
            .union_()
            .box(glm::vec3(0.75f, 1.5f, 0.25f),
                 pose(glm::vec3(-3.0f, 0.75f, 0.25f), glm::quat(0.5f, 0.5f, 0.5f, 0.5f), 0.400000006f))
            .intersect()
            .cylinder(0.600000024f, 1.20000005f, pose(glm::vec3(2.0f, 0.0f, -1.0f),
                                                      glm::quat(1.0f, 0.0f, 0.0f, 0.0f), 1.0f))
            .capsule(0.300000012f, 0.899999976f,
                     pose(glm::vec3(0.0f, 2.0f, 0.0f),
                          glm::normalize(glm::quat(0.0f, 1.0f, 0.0f, 0.0f)), 2.25f))
            .subtract()
            .smooth_union(0.349999994f)
            .torus(1.5f, 0.150000006f,
                   pose(glm::vec3(0.0f, 1.79999995f, 0.0f),
                        glm::normalize(glm::quat(0.0f, 0.0f, 0.707106781f, 0.707106781f)), 1.0f))
            .material_for_last_node(2)  // the torus is the decal
            .heightfield(0.400000006f, glm::vec2(0.699999988f, 1.29999995f), -1.0f,
                         pose(glm::vec3(0.0f, -4.0f, 0.0f), glm::quat(1.0f, 0.0f, 0.0f, 0.0f),
                              3.0f))
            .union_()
            .prop("mesh:props/tower",
                  pose(glm::vec3(-5.0f, 0.0f, -5.0f),
                       glm::normalize(glm::quat(0.899999976f, 0.0f, 0.400000006f, 0.0f)), 1.20000005f),
                  1)
            .prop("mesh:props/flag", pose(glm::vec3(5.0f, 0.5f, -5.0f),
                                          glm::quat(1.0f, 0.0f, 0.0f, 0.0f), 0.800000012f),
                  2)
            .union_()
            .build();
    EXPECT_OK(world);
    return world ? *world : WorldDesc{};
}

// ---------------------------------------------------------------------------
// Field-by-field comparison. The SDF halves go through memcmp -- these are
// std430-shaped PODs with no implicit padding (static_asserted in sdf.hpp), so
// a byte comparison is exactly the right instrument and no float tolerance can
// creep into it.
// ---------------------------------------------------------------------------
void expect_same_world(const WorldDesc& expected, const WorldDesc& actual) {
    EXPECT_EQ(expected.name, actual.name);

    ASSERT_EQ(expected.sdf.nodes.size(), actual.sdf.nodes.size());
    if (!expected.sdf.nodes.empty()) {
        EXPECT_EQ(0, std::memcmp(expected.sdf.nodes.data(), actual.sdf.nodes.data(),
                                 expected.sdf.nodes.size() * sizeof(spade::SdfNode)))
            << "SDF node bytes differ -- the round trip is not bit-exact";
    }
    ASSERT_EQ(expected.sdf.transforms.size(), actual.sdf.transforms.size());
    if (!expected.sdf.transforms.empty()) {
        EXPECT_EQ(0, std::memcmp(expected.sdf.transforms.data(), actual.sdf.transforms.data(),
                                 expected.sdf.transforms.size() * sizeof(spade::SdfTransform)))
            << "SDF transform bytes differ -- the round trip is not bit-exact";
    }
    EXPECT_EQ(expected.sdf.node_materials, actual.sdf.node_materials)
        << "node_materials differ -- the round trip is not bit-exact";

    ASSERT_EQ(expected.spawns.size(), actual.spawns.size());
    for (std::size_t i = 0; i < expected.spawns.size(); ++i) {
        const SpawnPoint& e = expected.spawns[i];
        const SpawnPoint& a = actual.spawns[i];
        EXPECT_EQ(e.name, a.name) << "spawn " << i;
        EXPECT_EQ(e.position.x, a.position.x) << "spawn " << i;
        EXPECT_EQ(e.position.y, a.position.y) << "spawn " << i;
        EXPECT_EQ(e.position.z, a.position.z) << "spawn " << i;
        EXPECT_EQ(e.orientation.w, a.orientation.w) << "spawn " << i;
        EXPECT_EQ(e.orientation.x, a.orientation.x) << "spawn " << i;
        EXPECT_EQ(e.orientation.y, a.orientation.y) << "spawn " << i;
        EXPECT_EQ(e.orientation.z, a.orientation.z) << "spawn " << i;
    }

    EXPECT_EQ(expected.environment.gravity.x, actual.environment.gravity.x);
    EXPECT_EQ(expected.environment.gravity.y, actual.environment.gravity.y);
    EXPECT_EQ(expected.environment.gravity.z, actual.environment.gravity.z);
    EXPECT_EQ(expected.environment.wind.x, actual.environment.wind.x);
    EXPECT_EQ(expected.environment.wind.y, actual.environment.wind.y);
    EXPECT_EQ(expected.environment.wind.z, actual.environment.wind.z);
    EXPECT_EQ(expected.environment.air_density, actual.environment.air_density);
    EXPECT_EQ(expected.environment.temperature_k, actual.environment.temperature_k);
    EXPECT_EQ(expected.environment.seed, actual.environment.seed);

    EXPECT_EQ(expected.capacities.bodies, actual.capacities.bodies);
    EXPECT_EQ(expected.capacities.force_elements, actual.capacities.force_elements);
    EXPECT_EQ(expected.capacities.sensors, actual.capacities.sensors);
    EXPECT_EQ(expected.capacities.contacts, actual.capacities.contacts);

    EXPECT_EQ(expected.visual_refs, actual.visual_refs);

    ASSERT_EQ(expected.materials.size(), actual.materials.size());
    for (std::size_t i = 0; i < expected.materials.size(); ++i) {
        const MaterialDesc& e = expected.materials[i];
        const MaterialDesc& a = actual.materials[i];
        EXPECT_EQ(e.name, a.name) << "material " << i;
        EXPECT_EQ(e.base_color.x, a.base_color.x) << "material " << i;
        EXPECT_EQ(e.base_color.y, a.base_color.y) << "material " << i;
        EXPECT_EQ(e.base_color.z, a.base_color.z) << "material " << i;
        EXPECT_EQ(e.base_color.w, a.base_color.w) << "material " << i;
        EXPECT_EQ(e.shading, a.shading) << "material " << i;
    }

    EXPECT_EQ(expected.lighting.sun_direction.x, actual.lighting.sun_direction.x);
    EXPECT_EQ(expected.lighting.sun_direction.y, actual.lighting.sun_direction.y);
    EXPECT_EQ(expected.lighting.sun_direction.z, actual.lighting.sun_direction.z);
    EXPECT_EQ(expected.lighting.sun_color.x, actual.lighting.sun_color.x);
    EXPECT_EQ(expected.lighting.sun_color.y, actual.lighting.sun_color.y);
    EXPECT_EQ(expected.lighting.sun_color.z, actual.lighting.sun_color.z);
    EXPECT_EQ(expected.lighting.sun_intensity, actual.lighting.sun_intensity);
    EXPECT_EQ(expected.lighting.ambient_color.x, actual.lighting.ambient_color.x);
    EXPECT_EQ(expected.lighting.ambient_color.y, actual.lighting.ambient_color.y);
    EXPECT_EQ(expected.lighting.ambient_color.z, actual.lighting.ambient_color.z);
    EXPECT_EQ(expected.lighting.sky_zenith.x, actual.lighting.sky_zenith.x);
    EXPECT_EQ(expected.lighting.sky_zenith.y, actual.lighting.sky_zenith.y);
    EXPECT_EQ(expected.lighting.sky_zenith.z, actual.lighting.sky_zenith.z);
    EXPECT_EQ(expected.lighting.sky_horizon.x, actual.lighting.sky_horizon.x);
    EXPECT_EQ(expected.lighting.sky_horizon.y, actual.lighting.sky_horizon.y);
    EXPECT_EQ(expected.lighting.sky_horizon.z, actual.lighting.sky_horizon.z);

    ASSERT_EQ(expected.props.size(), actual.props.size());
    for (std::size_t i = 0; i < expected.props.size(); ++i) {
        const PropDesc& e = expected.props[i];
        const PropDesc& a = actual.props[i];
        EXPECT_EQ(e.mesh_ref, a.mesh_ref) << "prop " << i;
        EXPECT_EQ(e.pose.position.x, a.pose.position.x) << "prop " << i;
        EXPECT_EQ(e.pose.position.y, a.pose.position.y) << "prop " << i;
        EXPECT_EQ(e.pose.position.z, a.pose.position.z) << "prop " << i;
        EXPECT_EQ(e.pose.rotation.w, a.pose.rotation.w) << "prop " << i;
        EXPECT_EQ(e.pose.rotation.x, a.pose.rotation.x) << "prop " << i;
        EXPECT_EQ(e.pose.rotation.y, a.pose.rotation.y) << "prop " << i;
        EXPECT_EQ(e.pose.rotation.z, a.pose.rotation.z) << "prop " << i;
        EXPECT_EQ(e.pose.scale, a.pose.scale) << "prop " << i;
        EXPECT_EQ(e.material, a.material) << "prop " << i;
    }
}

// ---------------------------------------------------------------------------
// A hand-written minimal world. It is the adversarial tests' base document --
// each case is one small edit of it -- and it doubles as the proof that this
// schema is HAND AUTHORABLE: nothing here came out of the writer.
// ---------------------------------------------------------------------------
constexpr const char* kMinimalWorld = R"YAML(world_version: 1
name: "minimal"
environment:
  gravity: [0, -9.80665016, 0]
  wind: [0, 0, 0]
  air_density: 1.22500002
  temperature_k: 288.149994
  seed: 0
capacities:
  bodies: 1
  force_elements: 1
  sensors: 1
  contacts: 1
spawns: []
sdf:
  transforms:
    - world_to_local: [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1]
      scale: 1
  nodes:
    - {prim: plane, transform: 0, params: [0, 1, 0, 0]}
visuals: []
)YAML";

// ---------------------------------------------------------------------------
// The same minimal world, spelled at schema v2: the three new sections at
// hand-authorable, non-default values (proving v2 -- not just v1's upgrade
// path -- is hand authorable too). Base document for the v2-specific
// adversarial tests below.
// ---------------------------------------------------------------------------
constexpr const char* kMinimalWorldV2 = R"YAML(world_version: 2
name: "minimal"
environment:
  gravity: [0, -9.80665016, 0]
  wind: [0, 0, 0]
  air_density: 1.22500002
  temperature_k: 288.149994
  seed: 0
capacities:
  bodies: 1
  force_elements: 1
  sensors: 1
  contacts: 1
spawns: []
sdf:
  transforms:
    - world_to_local: [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1]
      scale: 1
  nodes:
    - {prim: plane, transform: 0, params: [0, 1, 0, 0]}
  node_materials: []
visuals: []
materials:
  - {name: "default", base_color: [1, 1, 1, 1], shading: "lambert"}
lighting:
  sun_direction: [0, 1, 0]
  sun_color: [1, 1, 1]
  sun_intensity: 1
  ambient_color: [0.100000001, 0.100000001, 0.100000001]
  sky_zenith: [0.300000012, 0.5, 0.800000012]
  sky_horizon: [0.800000012, 0.850000024, 0.899999976]
props: []
)YAML";

// ---------------------------------------------------------------------------
// The committed corpus: the five determinism scenarios' worlds, the gate, and
// the maximal world.
//
// THE FIRST FIVE ARE THE DETERMINISM CORPUS'S WORLDS, and since S5 Task 7 they
// are the ONLY spelling of them: tests/golden/scenarios/*.scenario.yaml name
// these files by path, and the builder lambdas that used to construct the same
// worlds in C++ are gone. So what the builders below pin is no longer "two
// spellings agree" -- it is that spade::world_to_yaml() still reproduces the
// committed TEXT byte for byte, on every compiler CI runs, which is what makes
// the files reviewable artifacts rather than opaque blobs.
//
// The gate comes from engine/tools/viewer/scenes.cpp's build_gate_world().
//
// maximal.world.yaml is the odd one out and earns its place twice over. It is
// the round-trip test's own world, so committing it turns an in-memory
// property into a REVIEWABLE ARTIFACT: every key the schema has, all seven
// primitives, all four operators including smooth_union's blend radius, poses
// with a rotation and a scale != 1, three named spawns, three visual
// references, exponent-form floats and a full-range u64 seed, all in one
// document a reader can check by eye. The five worlds above are each honest
// about their own scenario and between them show almost none of that -- four
// of them have no spawns and no visuals at all -- so without this file the
// schema would be signed off having never been seen populated.
// ---------------------------------------------------------------------------

// tests/test_determinism.cpp's default_environment(): gravity, wind and
// air_density set explicitly to the values Environment{} already defaults to,
// temperature and seed left at their defaults.
[[nodiscard]] Environment default_environment() {
    Environment env;
    env.gravity = glm::vec3(0.0f, -9.80665f, 0.0f);
    env.wind = glm::vec3(0.0f);
    env.air_density = 1.225f;
    return env;
}

[[nodiscard]] Capacities scenario_capacities(uint32_t bodies, uint32_t elements) {
    Capacities caps;
    caps.bodies = bodies;
    caps.force_elements = elements;
    caps.sensors = 1;
    caps.contacts = 1;
    return caps;
}

[[nodiscard]] WorldDesc gate_world() {
    // The viewer builds the ring pose with glm::angleAxis(radians(90), +X).
    // Spelled here as the exact quaternion that is, (sqrt(1/2), sqrt(1/2), 0,
    // 0), for the reason in maximal_world()'s note: this world's bytes are
    // committed and compared across compilers, and std::sin/std::cos are not
    // part of that contract. std::sqrt is -- IEEE-754 mandates it correctly
    // rounded -- so this form is portable where angleAxis is merely likely.
    const float h = std::sqrt(0.5f);
    SdfPose ring;
    ring.position = glm::vec3(0.0f, 1.8f, 0.0f);
    ring.rotation = glm::quat(h, h, 0.0f, 0.0f);

    const glm::vec3 post_half(0.15f, 1.5f, 0.15f);
    SdfPose left_post;
    left_post.position = glm::vec3(-1.8f, 1.5f, 0.0f);
    SdfPose right_post;
    right_post.position = glm::vec3(1.8f, 1.5f, 0.0f);

    Capacities caps;
    caps.bodies = 5;
    caps.force_elements = 1;
    caps.sensors = 1;
    caps.contacts = 1;

    const spade::Result<WorldDesc> world = WorldBuilder()
                                               .name("gate_world")
                                               .environment(Environment{})
                                               .capacities(caps)
                                               .plane(glm::vec3(0.0f, 1.0f, 0.0f), 0.0f)
                                               .torus(1.5f, 0.15f, ring)
                                               .box(post_half, left_post)
                                               .union_()
                                               .box(post_half, right_post)
                                               .union_()
                                               .union_()
                                               .build();
    EXPECT_OK(world);
    return world ? *world : WorldDesc{};
}

struct CorpusWorld {
    const char* file;  // tests/golden/worlds/<file>.world.yaml
    WorldDesc desc;
};

[[nodiscard]] std::vector<CorpusWorld> corpus_worlds() {
    // ballistic: no geometry at all -- an empty node list with the builder's
    // identity transform still in the table. The degenerate case, on purpose.
    const spade::Result<WorldDesc> ballistic = WorldBuilder()
                                                   .name("void")
                                                   .environment(default_environment())
                                                   .capacities(scenario_capacities(4, 4))
                                                   .build();
    EXPECT_OK(ballistic);

    const spade::Result<WorldDesc> bounce = WorldBuilder()
                                                .name("ground")
                                                .environment(default_environment())
                                                .capacities(scenario_capacities(4, 2))
                                                .plane(glm::vec3(0.0f, 1.0f, 0.0f), 0.0f)
                                                .build();
    EXPECT_OK(bounce);

    SdfPose cavity;
    cavity.position = glm::vec3(0.0f, 0.4f, 0.0f);
    const spade::Result<WorldDesc> shower = WorldBuilder()
                                                .name("bowl")
                                                .environment(default_environment())
                                                .capacities(scenario_capacities(128, 4))
                                                .sphere(4.0f)
                                                .sphere(3.8f, cavity)
                                                .subtract()
                                                .build();
    EXPECT_OK(shower);

    const spade::Result<WorldDesc> two_world = WorldBuilder()
                                                   .name("ground")
                                                   .environment(default_environment())
                                                   .capacities(scenario_capacities(16, 4))
                                                   .plane(glm::vec3(0.0f, 1.0f, 0.0f), 0.0f)
                                                   .build();
    EXPECT_OK(two_world);

    // quad_hover (S5 Task 7): an EMPTY world sized for ONE QUADROTOR. Five
    // force elements -- make_quadrotor() declares four rotors plus the
    // airframe's one bluff-body drag element, and spec S3 counts both against
    // the world's single Capacities::force_elements budget -- and one sensor
    // for its IMU mount. No other world in this corpus has room for a vehicle;
    // the four above were sized for bare bodies.
    //
    // NO GEOMETRY, deliberately: an empty SDF program evaluates to
    // kSdfEmptyDistance, so the rotor's ground-effect factor is exactly 1 and
    // vehicles::hover_command()'s trim is exact rather than approximate. The
    // four worlds above cover contact physics; this one covers the vehicle.
    Capacities quad_caps;
    quad_caps.bodies = 1;
    quad_caps.force_elements = 5;
    quad_caps.sensors = 1;
    quad_caps.contacts = 1;
    const spade::Result<WorldDesc> quad_hover = WorldBuilder()
                                                    .name("quad_void")
                                                    .environment(default_environment())
                                                    .capacities(quad_caps)
                                                    .build();
    EXPECT_OK(quad_hover);

    std::vector<CorpusWorld> worlds;
    worlds.push_back({"ballistic", ballistic ? *ballistic : WorldDesc{}});
    worlds.push_back({"bounce", bounce ? *bounce : WorldDesc{}});
    worlds.push_back({"shower", shower ? *shower : WorldDesc{}});
    worlds.push_back({"two_world_isolation", two_world ? *two_world : WorldDesc{}});
    worlds.push_back({"quad_hover", quad_hover ? *quad_hover : WorldDesc{}});
    worlds.push_back({"gate", gate_world()});
    // The schema's own showcase -- see the note above. Nothing else in the
    // corpus carries a spawn, a visual reference, a scaled pose or an operator
    // with a parameter.
    worlds.push_back({"maximal", maximal_world()});
    return worlds;
}

[[nodiscard]] std::string world_golden_path(const std::string& name) {
    return std::string(SPADE_GOLDEN_DIR) + "/worlds/" + name + ".world.yaml";
}

[[nodiscard]] bool read_file(const std::string& path, std::string& out) {
    std::ifstream in(path, std::ios::binary);
    if (!in) return false;
    out.assign(std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>());
    return true;
}

}  // namespace

// ===========================================================================
// 1. Round trip
// ===========================================================================

TEST(WorldFile, MaximalWorldRoundTripsBitExactly) {
    const WorldDesc original = maximal_world();
    ASSERT_FALSE(original.sdf.nodes.empty());

    const spade::Result<std::string> text = spade::world_to_yaml(original);
    ASSERT_OK(text);

    const spade::Result<WorldDesc> loaded = spade::world_from_yaml(*text);
    ASSERT_OK(loaded);

    expect_same_world(original, *loaded);
}

TEST(WorldFile, MaximalWorldCoversEveryPrimitiveAndEveryOperator) {
    // Guards the round-trip test above from quietly shrinking: it is only a
    // proof about "every primitive, every operator" while it still contains
    // every primitive and every operator.
    const WorldDesc world = maximal_world();
    std::array<bool, spade::kSdfPrimCount> prims{};
    std::array<bool, spade::kSdfOpCount> ops{};
    for (const spade::SdfNode& node : world.sdf.nodes) {
        if (node.op == static_cast<uint32_t>(spade::SdfOp::none)) {
            ASSERT_LT(node.kind, spade::kSdfPrimCount);
            prims[node.kind] = true;
        } else {
            ASSERT_LT(node.op, spade::kSdfOpCount);
            ops[node.op] = true;
        }
    }
    for (uint32_t i = 0; i < spade::kSdfPrimCount; ++i) EXPECT_TRUE(prims[i]) << "primitive " << i;
    for (uint32_t i = 1; i < spade::kSdfOpCount; ++i) EXPECT_TRUE(ops[i]) << "operator " << i;
    EXPECT_FALSE(ops[0]) << "SdfOp::none is not an operator and must never appear as one";

    // Non-identity poses with a scale != 1 really are in there (the whole
    // reason the transform table is interesting).
    ASSERT_GT(world.sdf.transforms.size(), 1u);
    bool scaled = false;
    for (const spade::SdfTransform& t : world.sdf.transforms) {
        if (t.scale != 1.0f) scaled = true;
    }
    EXPECT_TRUE(scaled);
    EXPECT_EQ(world.spawns.size(), 3u);
    EXPECT_EQ(world.visual_refs.size(), 3u);

    // Schema v2's own fields: a palette wider than the auto-inserted single
    // default, all three MaterialShading values represented, non-default
    // lighting, at least one node actually using a non-default material, and
    // at least one prop.
    ASSERT_EQ(world.materials.size(), 3u);
    std::array<bool, spade::kMaterialShadingCount> shadings{};
    for (const MaterialDesc& m : world.materials) {
        ASSERT_LT(static_cast<uint32_t>(m.shading), spade::kMaterialShadingCount);
        shadings[static_cast<uint32_t>(m.shading)] = true;
    }
    for (uint32_t i = 0; i < spade::kMaterialShadingCount; ++i) EXPECT_TRUE(shadings[i]) << "shading " << i;
    EXPECT_NE(world.lighting.sun_intensity, LightingDesc{}.sun_intensity);
    ASSERT_EQ(world.sdf.node_materials.size(), world.sdf.nodes.size());
    EXPECT_TRUE(std::any_of(world.sdf.node_materials.begin(), world.sdf.node_materials.end(),
                            [](uint32_t m) { return m != 0; }));
    EXPECT_EQ(world.props.size(), 2u);
}

TEST(WorldFile, EmissionIsCanonical) {
    // yaml -> desc -> yaml is the identity on text. Without this the committed
    // corpus files could not be compared byte for byte, because "the same
    // world" would have more than one spelling.
    const spade::Result<std::string> once = spade::world_to_yaml(maximal_world());
    ASSERT_OK(once);
    const spade::Result<WorldDesc> reparsed = spade::world_from_yaml(*once);
    ASSERT_OK(reparsed);
    const spade::Result<std::string> twice = spade::world_to_yaml(*reparsed);
    ASSERT_OK(twice);
    EXPECT_EQ(*once, *twice) << first_difference(*once, *twice);

    // ...and a third pass, because an emitter that is idempotent only after the
    // first normalization is not canonical.
    const spade::Result<WorldDesc> rereparsed = spade::world_from_yaml(*twice);
    ASSERT_OK(rereparsed);
    const spade::Result<std::string> thrice = spade::world_to_yaml(*rereparsed);
    ASSERT_OK(thrice);
    EXPECT_EQ(*once, *thrice);
}

TEST(WorldFile, SavesAndLoadsThroughTheFilesystem) {
    const WorldDesc original = maximal_world();
    const std::filesystem::path path =
        std::filesystem::temp_directory_path() / "spade_test_maximal.world.yaml";
    std::filesystem::remove(path);

    ASSERT_OK(spade::save_world_file(original, path));
    const spade::Result<WorldDesc> loaded = spade::load_world_file(path);
    ASSERT_OK(loaded);
    expect_same_world(original, *loaded);

    // The bytes on disk are exactly world_to_yaml()'s -- LF endings included,
    // which is why save_world_file opens in binary mode.
    const spade::Result<std::string> text = spade::world_to_yaml(original);
    ASSERT_OK(text);
    std::string on_disk;
    ASSERT_TRUE(read_file(path.string(), on_disk));
    EXPECT_EQ(*text, on_disk);
    EXPECT_EQ(on_disk.find('\r'), std::string::npos) << "world files are LF-only";

    std::filesystem::remove(path);
}

TEST(WorldFile, MissingFileIsAnIoErrorNamingThePath) {
    const std::filesystem::path path =
        std::filesystem::temp_directory_path() / "spade_test_no_such.world.yaml";
    std::filesystem::remove(path);
    const spade::Result<WorldDesc> loaded = spade::load_world_file(path);
    EXPECT_EQ(code_of(loaded), code(spade::Code::io_error));
    EXPECT_TRUE(contains(why(loaded), "spade_test_no_such")) << why(loaded);
}

// ===========================================================================
// 2. Digest equivalence -- the loaded world RUNS the same
// ===========================================================================

TEST(WorldFile, LoadedWorldProducesAnIdenticalStateDigest) {
    const WorldDesc original = maximal_world();
    const spade::Result<std::string> text = spade::world_to_yaml(original);
    ASSERT_OK(text);
    const spade::Result<WorldDesc> loaded = spade::world_from_yaml(*text);
    ASSERT_OK(loaded);

    // One scenario description, parameterized only by which WorldDesc it
    // builds -- so anything that differs between the two runs came from the
    // world, which is the whole point.
    const auto scenario_for = [](const WorldDesc& world) {
        spade::testing::Scenario s;
        s.name = "world_file_digest";
        s.dt_ns = 2'000'000;  // 2 ms step
        s.substeps = 2;       // 1 ms substep
        s.steps = 150;

        s.build = [world]() -> spade::Result<spade::WorldSetDesc> {
            spade::WorldInstanceDesc instance;
            instance.world = world;
            instance.seed = 0x5DF11E5EEDULL;
            instance.turbulence = spade::dryden_params(spade::TurbulenceLevel::moderate);
            instance.contacts.restitution_e = 0.35f;
            instance.contacts.friction_mu = 0.25f;
            instance.contacts.proxy_radius = 0.12f;
            instance.grid.cell_size = 0.3f;
            return spade::WorldSetDesc{{instance}};
        };
        s.setup = [](spade::Simulation& sim) -> spade::Result<void> {
            for (uint32_t i = 0; i < 6; ++i) {
                spade::BodySpawn body;
                body.pos = glm::vec3(-1.0f + 0.4f * static_cast<float>(i), 6.0f,
                                     0.5f * static_cast<float>(i % 3u));
                body.vel = glm::vec3(0.25f, 0.0f, -0.1f);
                body.omega_body = glm::vec3(0.1f, -0.2f, 0.05f);
                body.mass = 0.4f;
                body.inv_inertia_diag = glm::vec3(250.0f);
                const spade::Result<spade::BodyRef> ref = sim.spawn(0, body);
                if (!ref) return std::unexpected(ref.error());
            }
            return {};
        };
        return s;
    };

    const spade::Result<uint64_t> from_builder =
        spade::testing::run_scenario(scenario_for(original));
    ASSERT_OK(from_builder);
    const spade::Result<uint64_t> from_file = spade::testing::run_scenario(scenario_for(*loaded));
    ASSERT_OK(from_file);

    EXPECT_EQ(*from_builder, *from_file)
        << "a world loaded from YAML did not run identically to the world it was written from";

    // AND THE EQUALITY ABOVE IS SENSITIVE TO THE SDF, which is not something
    // to assume. A one-ULP change to a single params lane must move the
    // digest; if it did not, the test would keep passing over a loader that
    // had stopped reading params at all.
    //
    // BE PRECISE ABOUT WHICH CHANNEL CARRIES THAT SENSITIVITY, because the
    // two are not equally strong. The GUARANTEED one is replay_config: its
    // config_hash folds the entire WorldSetDesc -- the SdfNode bytes included
    // -- into registered state, so ANY change to the description moves
    // state_digest whether or not it changes a single trajectory. The
    // incidental one is the contact dynamics, which for this particular
    // perturbation (a sphere radius) also move the bodies. So this assertion
    // proves the digest sees the SDF DESCRIPTION; it is deliberately NOT
    // offered as proof that it sees the SDF's physical effect.
    WorldDesc mutated = original;
    ASSERT_GT(mutated.sdf.nodes.size(), 1u);
    ASSERT_EQ(mutated.sdf.nodes[1].kind, static_cast<uint32_t>(spade::SdfPrim::sphere));
    mutated.sdf.nodes[1].params.x = std::nextafter(mutated.sdf.nodes[1].params.x, 1000.0f);
    ASSERT_NE(mutated.sdf.nodes[1].params.x, original.sdf.nodes[1].params.x);
    ASSERT_OK(spade::validate_world_desc(mutated));

    const spade::Result<uint64_t> from_mutant =
        spade::testing::run_scenario(scenario_for(mutated));
    ASSERT_OK(from_mutant);
    EXPECT_NE(*from_builder, *from_mutant)
        << "one ULP of one SDF parameter left the digest unchanged -- this test is no longer "
           "measuring what it claims to";
}

// ===========================================================================
// 3. Locale immunity
// ===========================================================================

namespace {

// Restores both locales -- the C++ global one (which iostreams imbue) and the
// C one (which strtof/printf read) -- however the test leaves.
class LocaleGuard {
public:
    LocaleGuard() {
        const char* current = std::setlocale(LC_ALL, nullptr);
        c_locale_ = current != nullptr ? current : "C";
    }
    ~LocaleGuard() {
        std::locale::global(cxx_locale_);
        std::setlocale(LC_ALL, c_locale_.c_str());
    }
    LocaleGuard(const LocaleGuard&) = delete;
    LocaleGuard& operator=(const LocaleGuard&) = delete;

    // Installs the first comma-decimal locale this host actually has. Returns
    // false when none is available (a bare Linux container usually generates
    // no locales at all), which weakens the test but does not invalidate it.
    [[nodiscard]] static bool install_comma_decimal() {
        for (const char* name : {"de_DE.UTF-8", "de-DE", "German_Germany.1252", "fr_FR.UTF-8",
                                 "fr-FR", "French_France.1252"}) {
            try {
                std::locale::global(std::locale(name));
            } catch (const std::exception&) {
                continue;
            }
            std::setlocale(LC_ALL, name);
            return true;
        }
        return false;
    }

private:
    std::locale cxx_locale_{};
    std::string c_locale_;
};

}  // namespace

TEST(WorldFile, FloatTextIsImmuneToTheGlobalLocale) {
    LocaleGuard guard;
    const bool hostile = LocaleGuard::install_comma_decimal();

    // 0.1f is the canonical locale-hostile value: it is the shortest decimal a
    // comma-decimal locale mangles ("0,1"), and its 9-digit form
    // (0.100000001) is also the shortest proof that 9 digits are being used --
    // a 6-digit "%g" would write "0.1" and lose bits.
    // The two exponent-form values are EXACT POWERS OF TWO (2^-24 and 2^40), so
    // their 9-digit decimal expansions are facts rather than guesses -- which is
    // what makes it fair to assert on the exact emitted text below.
    Environment env;
    env.gravity = glm::vec3(0.100000001f, -0.100000001f, 1.00000001e-07f);
    env.wind = glm::vec3(5.96046448e-08f, 1099511627776.0f, -1.17549435e-38f);
    env.air_density = 0.100000001f;
    env.temperature_k = 3.14159274f;
    env.seed = 18446744073709551615ULL;  // 2^64 - 1

    Capacities caps;
    caps.bodies = 1;
    caps.force_elements = 1;
    caps.sensors = 1;
    caps.contacts = 1;

    const spade::Result<WorldDesc> original =
        WorldBuilder()
            .name("locale")
            .environment(env)
            .capacities(caps)
            .sphere(0.100000001f)
            .box(glm::vec3(3.14159274f, 1.00000001e-07f, 5.96046448e-08f))
            .union_()
            .build();
    ASSERT_OK(original);

    const spade::Result<std::string> text = spade::world_to_yaml(*original);
    ASSERT_OK(text);

    // The decimal separator is a POINT and the mantissa carries 9 digits,
    // whatever the process locale says. A snprintf("%.9g") or an ostream here
    // would have written "0,100000001" and this line would be the failure.
    EXPECT_TRUE(contains(*text, "0.100000001")) << "hostile locale installed: " << hostile;
    EXPECT_FALSE(contains(*text, "0,100000001"));
    // The exponent spelling is pinned too: sign always present, at least two
    // digits, which is what keeps the committed corpus byte-identical across
    // standard-library implementations.
    EXPECT_TRUE(contains(*text, "5.96046448e-08"));   // 2^-24
    EXPECT_TRUE(contains(*text, "1.09951163e+12"));   // 2^40
    // And the u64 seed went out as a plain decimal integer -- full 64-bit
    // range, and no thousands grouping, which a locale-aware integer
    // formatter would have inserted.
    EXPECT_TRUE(contains(*text, "18446744073709551615"));
    EXPECT_FALSE(contains(*text, "18,446,744"));
    EXPECT_FALSE(contains(*text, "18.446.744"));

    const spade::Result<WorldDesc> loaded = spade::world_from_yaml(*text);
    ASSERT_OK(loaded);
    expect_same_world(*original, *loaded);

    // yaml-cpp's own as<float>() is the trap this code avoids; if the parse
    // path ever regressed to it, this is the assertion that would catch it,
    // because under the installed locale "0.100000001" would come back as 0.
    EXPECT_EQ(loaded->environment.air_density, 0.100000001f);
}

// ===========================================================================
// 4. The schema is hand authorable
// ===========================================================================

TEST(WorldFile, MinimalHandWrittenDocumentLoads) {
    const spade::Result<WorldDesc> world = spade::world_from_yaml(kMinimalWorld);
    ASSERT_OK(world);
    EXPECT_EQ(world->name, "minimal");
    EXPECT_EQ(world->sdf.nodes.size(), 1u);
    EXPECT_EQ(world->sdf.transforms.size(), 1u);
    EXPECT_EQ(world->capacities.bodies, 1u);
    EXPECT_TRUE(world->spawns.empty());
    EXPECT_TRUE(world->visual_refs.empty());
    EXPECT_EQ(world->environment.gravity.y, -9.80665016f);
}

// Step 1's upgrade-path claim, pinned directly: a v1 file (no materials/
// lighting/props keys at all) still loads, and its WorldDesc gains schema
// v2's three sections at their defaults -- one default material, default
// lighting, no props. Same fixture MinimalHandWrittenDocumentLoads uses
// above, so this is strictly an ADDITIONAL claim about it, not a new fixture.
TEST(WorldFile, V1FileUpgradesWithDefaultMaterialsLightingAndProps) {
    const spade::Result<WorldDesc> world = spade::world_from_yaml(kMinimalWorld);
    ASSERT_OK(world);
    ASSERT_EQ(world->materials.size(), 1u);
    EXPECT_EQ(world->materials[0].name, "default");
    EXPECT_EQ(world->materials[0].base_color, glm::vec4(1.0f, 1.0f, 1.0f, 1.0f));
    EXPECT_EQ(world->materials[0].shading, MaterialShading::lambert);
    EXPECT_EQ(world->lighting.sun_direction, LightingDesc{}.sun_direction);
    EXPECT_EQ(world->lighting.sun_intensity, LightingDesc{}.sun_intensity);
    EXPECT_TRUE(world->props.empty());
    EXPECT_TRUE(world->sdf.node_materials.empty());
}

// v2 is hand authorable too, not just v1's upgrade path -- kMinimalWorldV2
// spells the three new sections directly, at values that are NOT the
// defaults (props: [] aside, which is the only legal empty state for it).
TEST(WorldFile, HandAuthoredV2DocumentLoads) {
    const spade::Result<WorldDesc> world = spade::world_from_yaml(kMinimalWorldV2);
    ASSERT_OK(world);
    EXPECT_EQ(world->name, "minimal");
    ASSERT_EQ(world->materials.size(), 1u);
    EXPECT_EQ(world->materials[0].name, "default");
    EXPECT_EQ(world->materials[0].shading, MaterialShading::lambert);
    EXPECT_EQ(world->lighting.sun_direction, glm::vec3(0.0f, 1.0f, 0.0f));
    EXPECT_TRUE(world->props.empty());
}

// A v2 hand-authored document round-trips byte-exact through the writer too
// (Step 1's other claim) -- world_from_yaml -> world_to_yaml -> world_from_yaml
// produces the SAME WorldDesc, mirroring MaximalWorldRoundTripsBitExactly's
// shape but starting from hand-written text instead of the builder.
TEST(WorldFile, HandAuthoredV2DocumentRoundTripsThroughTheWriter) {
    const spade::Result<WorldDesc> once = spade::world_from_yaml(kMinimalWorldV2);
    ASSERT_OK(once);
    const spade::Result<std::string> text = spade::world_to_yaml(*once);
    ASSERT_OK(text);
    const spade::Result<WorldDesc> twice = spade::world_from_yaml(*text);
    ASSERT_OK(twice);
    expect_same_world(*once, *twice);
    EXPECT_TRUE(contains(*text, "world_version: 2")) << *text;
}

// ===========================================================================
// 5. Adversarial -- every rejection rule, and its diagnostic
// ===========================================================================

TEST(WorldFileRejects, TruncatedDocument) {
    const spade::Result<std::string> text = spade::world_to_yaml(maximal_world());
    ASSERT_OK(text);
    // Cut inside a flow sequence, which leaves a bracket open: the shape a
    // half-written or half-copied file actually has.
    const std::size_t at = text->find("world_to_local: [");
    ASSERT_NE(at, std::string::npos);
    const spade::Result<WorldDesc> world = spade::world_from_yaml(text->substr(0, at + 24));
    EXPECT_EQ(code_of(world), code(spade::Code::invalid_argument));
    EXPECT_TRUE(contains(why(world), "line")) << why(world);
}

TEST(WorldFileRejects, EmptyDocument) {
    const spade::Result<WorldDesc> world = spade::world_from_yaml("");
    EXPECT_EQ(code_of(world), code(spade::Code::invalid_argument));
    EXPECT_TRUE(contains(why(world), "mapping")) << why(world);
}

// A VERSION MISMATCH IS schema_mismatch, NOT invalid_argument, and these tests
// assert the CODE and not merely the failure because the distinction is the
// whole point: every other rejection in this suite means "fix your file",
// while this one means "this build cannot read this artifact" -- which a
// caller may answer by upgrading instead. state/snapshot.cpp says the same
// sentence about a versioned blob with the same code; one engine, one answer.
//
// version 3 (not 2) IS THE MISMATCH HERE, schema v2's own upgrade path
// (kWorldFileMinReadVersion) is what made "world_version: 2" a version this
// build reads -- that is exactly what WorldFile.HandAuthoredV2DocumentLoads
// and the v1-upgrade tests below pin. Version 3 is a genuinely unsupported
// future version, the same role 2 played before this task.
TEST(WorldFileRejects, WrongSchemaVersionNamingBoth) {
    const spade::Result<WorldDesc> world =
        spade::world_from_yaml(replace_first(kMinimalWorld, "world_version: 1", "world_version: 3"));
    EXPECT_EQ(code_of(world), code(spade::Code::schema_mismatch));
    EXPECT_TRUE(contains(why(world), "version 3")) << why(world);
    EXPECT_TRUE(contains(why(world), "versions 1-2")) << why(world);
    EXPECT_TRUE(contains(why(world), "line")) << why(world);
}

TEST(WorldFileRejects, AFutureVersionIsReportedAsAVersionNotAsUnknownKeys) {
    // A v3 file is full of keys v1/v2 do not know. "unknown key 'thermals'" is
    // a true but useless answer; the version check has to come first.
    const std::string v3 =
        replace_first(replace_first(kMinimalWorld, "world_version: 1", "world_version: 3"),
                      "name: \"minimal\"", "name: \"minimal\"\nthermals: {model: \"bubble\"}");
    const spade::Result<WorldDesc> world = spade::world_from_yaml(v3);
    EXPECT_EQ(code_of(world), code(spade::Code::schema_mismatch));
    EXPECT_TRUE(contains(why(world), "schema version 3")) << why(world);
    EXPECT_FALSE(contains(why(world), "unknown key")) << why(world);
}

TEST(WorldFileRejects, AVersionMismatchKeepsItsCodeThroughTheFileLayer) {
    // load_world_file() re-wraps the parse error to prefix the path. The CODE
    // has to survive that re-wrap, or the distinction above is invisible to
    // everyone who reads files rather than strings.
    const std::filesystem::path path =
        std::filesystem::temp_directory_path() / "spade_test_v3.world.yaml";
    {
        std::ofstream out(path, std::ios::binary | std::ios::trunc);
        ASSERT_TRUE(out.good());
        const std::string v3 = replace_first(kMinimalWorld, "world_version: 1", "world_version: 3");
        out.write(v3.data(), static_cast<std::streamsize>(v3.size()));
    }
    const spade::Result<WorldDesc> world = spade::load_world_file(path);
    EXPECT_EQ(code_of(world), code(spade::Code::schema_mismatch));
    EXPECT_TRUE(contains(why(world), "spade_test_v3.world.yaml")) << why(world);
    std::filesystem::remove(path);
}

TEST(WorldFileRejects, UnknownKeyAtTheTopLevel) {
    const spade::Result<WorldDesc> world = spade::world_from_yaml(
        replace_first(kMinimalWorld, "name: \"minimal\"", "name: \"minimal\"\nrestitution: 0.5"));
    EXPECT_EQ(code_of(world), code(spade::Code::invalid_argument));
    EXPECT_TRUE(contains(why(world), "unknown key 'restitution'")) << why(world);
    EXPECT_TRUE(contains(why(world), "in world")) << why(world);
}

TEST(WorldFileRejects, UnknownKeyNestedInsideAMapping) {
    const spade::Result<WorldDesc> world = spade::world_from_yaml(
        replace_first(kMinimalWorld, "  seed: 0", "  seed: 0\n  humidity: 0.4"));
    EXPECT_EQ(code_of(world), code(spade::Code::invalid_argument));
    EXPECT_TRUE(contains(why(world), "unknown key 'humidity'")) << why(world);
    EXPECT_TRUE(contains(why(world), "world.environment")) << why(world);
}

TEST(WorldFileRejects, UnknownKeyInsideAnSdfNode) {
    const spade::Result<WorldDesc> world = spade::world_from_yaml(replace_first(
        kMinimalWorld, "{prim: plane, transform: 0", "{prim: plane, blend: 0.2, transform: 0"));
    EXPECT_EQ(code_of(world), code(spade::Code::invalid_argument));
    EXPECT_TRUE(contains(why(world), "unknown key 'blend'")) << why(world);
    EXPECT_TRUE(contains(why(world), "world.sdf.nodes[0]")) << why(world);
}

TEST(WorldFileRejects, UnknownKeyInsideASpawnEntry) {
    const spade::Result<WorldDesc> world = spade::world_from_yaml(replace_first(
        kMinimalWorld, "spawns: []",
        "spawns:\n  - {name: \"start\", position: [0, 1, 0], orientation: [1, 0, 0, 0], "
        "heading_deg: 90}"));
    EXPECT_EQ(code_of(world), code(spade::Code::invalid_argument));
    EXPECT_TRUE(contains(why(world), "unknown key 'heading_deg'")) << why(world);
    EXPECT_TRUE(contains(why(world), "world.spawns[0]")) << why(world);
}

TEST(WorldFileRejects, UnknownKeyInsideAnSdfTransform) {
    const spade::Result<WorldDesc> world =
        spade::world_from_yaml(replace_first(kMinimalWorld, "      scale: 1", "      scale: 1\n      shear: 0.1"));
    EXPECT_EQ(code_of(world), code(spade::Code::invalid_argument));
    EXPECT_TRUE(contains(why(world), "unknown key 'shear'")) << why(world);
    EXPECT_TRUE(contains(why(world), "world.sdf.transforms[0]")) << why(world);
}

TEST(WorldFileRejects, DuplicateKey) {
    const spade::Result<WorldDesc> world =
        spade::world_from_yaml(replace_first(kMinimalWorld, "  seed: 0", "  seed: 0\n  seed: 7"));
    EXPECT_EQ(code_of(world), code(spade::Code::invalid_argument));
    EXPECT_TRUE(contains(why(world), "duplicate key 'seed'")) << why(world);
}

TEST(WorldFileRejects, MissingRequiredKey) {
    const spade::Result<WorldDesc> world =
        spade::world_from_yaml(replace_first(kMinimalWorld, "  seed: 0\n", ""));
    EXPECT_EQ(code_of(world), code(spade::Code::invalid_argument));
    EXPECT_TRUE(contains(why(world), "missing required key 'seed'")) << why(world);
}

TEST(WorldFileRejects, InfinityInFloatText) {
    const spade::Result<WorldDesc> world = spade::world_from_yaml(
        replace_first(kMinimalWorld, "air_density: 1.22500002", "air_density: inf"));
    EXPECT_EQ(code_of(world), code(spade::Code::invalid_argument));
    EXPECT_TRUE(contains(why(world), "not finite")) << why(world);
    EXPECT_TRUE(contains(why(world), "world.environment.air_density")) << why(world);
}

TEST(WorldFileRejects, NanInFloatText) {
    const spade::Result<WorldDesc> world = spade::world_from_yaml(
        replace_first(kMinimalWorld, "params: [0, 1, 0, 0]", "params: [0, 1, 0, nan]"));
    EXPECT_EQ(code_of(world), code(spade::Code::invalid_argument));
    EXPECT_TRUE(contains(why(world), "not finite")) << why(world);
    EXPECT_TRUE(contains(why(world), "params[3]")) << why(world);
}

TEST(WorldFileRejects, YamlsOwnInfinitySpelling) {
    // ".inf" is YAML 1.1's infinity. It is not a decimal number, so it is
    // rejected on the way in rather than becoming one.
    const spade::Result<WorldDesc> world = spade::world_from_yaml(
        replace_first(kMinimalWorld, "air_density: 1.22500002", "air_density: .inf"));
    EXPECT_EQ(code_of(world), code(spade::Code::invalid_argument));
    EXPECT_TRUE(contains(why(world), "not a decimal number")) << why(world);
}

TEST(WorldFileRejects, NonNumericFloatText) {
    const spade::Result<WorldDesc> world = spade::world_from_yaml(
        replace_first(kMinimalWorld, "air_density: 1.22500002", "air_density: dense"));
    EXPECT_EQ(code_of(world), code(spade::Code::invalid_argument));
    EXPECT_TRUE(contains(why(world), "not a decimal number")) << why(world);
}

TEST(WorldFileRejects, TransformIndexOutOfRange) {
    // Caught by the SHARED validation (SdfProgram::validate via
    // validate_world_desc), not by a loader-local check -- which is the point
    // of the extraction.
    const spade::Result<WorldDesc> world = spade::world_from_yaml(
        replace_first(kMinimalWorld, "transform: 0", "transform: 7"));
    EXPECT_EQ(code_of(world), code(spade::Code::invalid_argument));
    EXPECT_TRUE(contains(why(world), "SDF transform index out of range")) << why(world);
}

TEST(WorldFileRejects, OperatorNodeNonZeroTransform) {
    // The op-node tightening (S6 hygiene batch): an operator's `transform` is
    // unused (world_file.cpp's own emission comment: "on an `op` node it is
    // unused") but nothing enforced that on the READ side until now --
    // validated here, mirroring this codebase's other reserved-lane-must-be-
    // zero checks (ContactParams/GridParams's _r0.._r2). This is the proof
    // the new check actually fires rather than being dead validation code:
    // sdf.cpp's operator branch checks `node.transform` BEFORE it checks the
    // stack depth, so a single, stack-empty op node -- which would otherwise
    // hit PostfixUnderflow's "needs two operands" rejection just below --
    // hits THIS rejection first when its transform is nonzero.
    const spade::Result<WorldDesc> world = spade::world_from_yaml(replace_first(
        kMinimalWorld, "    - {prim: plane, transform: 0, params: [0, 1, 0, 0]}",
        "    - {op: union, transform: 5, params: [0, 0, 0, 0]}"));
    EXPECT_EQ(code_of(world), code(spade::Code::invalid_argument));
    EXPECT_TRUE(contains(why(world), "SDF operator node's transform index must be 0"))
        << why(world);
}

TEST(WorldFileRejects, PostfixUnderflow) {
    // An operator with nothing to pop. The message is SdfProgram::validate()'s
    // own wording, which is the proof that the loader runs the SAME validation
    // WorldBuilder::build() runs rather than a parallel one of its own.
    const spade::Result<WorldDesc> world = spade::world_from_yaml(replace_first(
        kMinimalWorld, "    - {prim: plane, transform: 0, params: [0, 1, 0, 0]}",
        "    - {op: union, transform: 0, params: [0, 0, 0, 0]}"));
    EXPECT_EQ(code_of(world), code(spade::Code::invalid_argument));
    EXPECT_TRUE(contains(why(world), "SDF operator needs two operands on the stack"))
        << why(world);
}

TEST(WorldFileRejects, ProgramThatDoesNotReduceToOneValue) {
    const spade::Result<WorldDesc> world = spade::world_from_yaml(replace_first(
        kMinimalWorld, "    - {prim: plane, transform: 0, params: [0, 1, 0, 0]}",
        "    - {prim: plane, transform: 0, params: [0, 1, 0, 0]}\n"
        "    - {prim: sphere, transform: 0, params: [1, 0, 0, 0]}"));
    EXPECT_EQ(code_of(world), code(spade::Code::invalid_argument));
    EXPECT_TRUE(contains(why(world), "must reduce to exactly one value")) << why(world);
}

TEST(WorldFileRejects, DuplicateSpawnNames) {
    const spade::Result<WorldDesc> world = spade::world_from_yaml(replace_first(
        kMinimalWorld, "spawns: []",
        "spawns:\n"
        "  - {name: \"start\", position: [0, 1, 0], orientation: [1, 0, 0, 0]}\n"
        "  - {name: \"start\", position: [2, 1, 0], orientation: [1, 0, 0, 0]}"));
    EXPECT_EQ(code_of(world), code(spade::Code::invalid_argument));
    EXPECT_TRUE(contains(why(world), "duplicate spawn point name 'start'")) << why(world);
}

TEST(WorldFileRejects, DegenerateSpawnOrientation) {
    const spade::Result<WorldDesc> world = spade::world_from_yaml(replace_first(
        kMinimalWorld, "spawns: []",
        "spawns:\n  - {name: \"start\", position: [0, 1, 0], orientation: [0, 0, 0, 0]}"));
    EXPECT_EQ(code_of(world), code(spade::Code::invalid_argument));
    EXPECT_TRUE(contains(why(world), "unit quaternion")) << why(world);
}

TEST(WorldFileRejects, ZeroCapacity) {
    const spade::Result<WorldDesc> world =
        spade::world_from_yaml(replace_first(kMinimalWorld, "  bodies: 1", "  bodies: 0"));
    EXPECT_EQ(code_of(world), code(spade::Code::invalid_argument));
    EXPECT_TRUE(contains(why(world), "'bodies' must be > 0")) << why(world);
}

TEST(WorldFileRejects, UnknownPrimitiveName) {
    const spade::Result<WorldDesc> world =
        spade::world_from_yaml(replace_first(kMinimalWorld, "prim: plane", "prim: dodecahedron"));
    EXPECT_EQ(code_of(world), code(spade::Code::invalid_argument));
    EXPECT_TRUE(contains(why(world), "unknown SDF primitive 'dodecahedron'")) << why(world);
}

TEST(WorldFileRejects, OpNoneIsNotAnOperator) {
    const spade::Result<WorldDesc> world =
        spade::world_from_yaml(replace_first(kMinimalWorld, "prim: plane", "op: none"));
    EXPECT_EQ(code_of(world), code(spade::Code::invalid_argument));
    EXPECT_TRUE(contains(why(world), "'none' is not an operator")) << why(world);
}

TEST(WorldFileRejects, NodeWithBothPrimAndOp) {
    const spade::Result<WorldDesc> world = spade::world_from_yaml(
        replace_first(kMinimalWorld, "{prim: plane,", "{prim: plane, op: union,"));
    EXPECT_EQ(code_of(world), code(spade::Code::invalid_argument));
    EXPECT_TRUE(contains(why(world), "exactly one of 'prim'")) << why(world);
}

TEST(WorldFileRejects, WrongLengthFloatSequence) {
    const spade::Result<WorldDesc> world = spade::world_from_yaml(
        replace_first(kMinimalWorld, "gravity: [0, -9.80665016, 0]", "gravity: [0, -9.80665016]"));
    EXPECT_EQ(code_of(world), code(spade::Code::invalid_argument));
    EXPECT_TRUE(contains(why(world), "exactly 3 numbers, found 2")) << why(world);
}

TEST(WorldFileRejects, EmptyVisualReference) {
    const spade::Result<WorldDesc> world =
        spade::world_from_yaml(replace_first(kMinimalWorld, "visuals: []", "visuals:\n  - \"\""));
    EXPECT_EQ(code_of(world), code(spade::Code::invalid_argument));
    EXPECT_TRUE(contains(why(world), "visual reference 0 is empty")) << why(world);
}

// ---------------------------------------------------------------------------
// Schema v2's own sections (materials, lighting, props, sdf.node_materials).
// Step 1: "Unknown keys are still errors at every level, including inside the
// new sections. node_materials of the wrong length is invalid_argument. A
// props entry with an empty mesh_ref is rejected."
// ---------------------------------------------------------------------------

TEST(WorldFileRejects, V1DocumentCannotCarryASchemaV2Section) {
    // v1 never defined `materials` -- a v1-labeled document that includes it
    // is malformed (bump world_version to 2, or drop the key), not silently
    // upgraded. Proves the top-level allowlist really is version-gated, not
    // just widened once and for all.
    const spade::Result<WorldDesc> world = spade::world_from_yaml(
        replace_first(kMinimalWorld, "visuals: []", "visuals: []\nmaterials: []"));
    EXPECT_EQ(code_of(world), code(spade::Code::invalid_argument));
    EXPECT_TRUE(contains(why(world), "unknown key 'materials'")) << why(world);
    EXPECT_TRUE(contains(why(world), "schema v1")) << why(world);
}

TEST(WorldFileRejects, V1DocumentSdfSectionCannotCarryNodeMaterials) {
    // Same version-gating, one level down: v1's `sdf:` block never carried
    // node_materials either (PA-2 postdates it).
    const spade::Result<WorldDesc> world = spade::world_from_yaml(replace_first(
        kMinimalWorld, "  nodes:\n    - {prim: plane, transform: 0, params: [0, 1, 0, 0]}",
        "  nodes:\n    - {prim: plane, transform: 0, params: [0, 1, 0, 0]}\n  node_materials: []"));
    EXPECT_EQ(code_of(world), code(spade::Code::invalid_argument));
    EXPECT_TRUE(contains(why(world), "unknown key 'node_materials'")) << why(world);
}

TEST(WorldFileRejects, NodeMaterialsWrongLengthViaYamlIsInvalid) {
    const spade::Result<WorldDesc> world = spade::world_from_yaml(
        replace_first(kMinimalWorldV2, "node_materials: []", "node_materials: [0, 0]"));
    EXPECT_EQ(code_of(world), code(spade::Code::invalid_argument));
    EXPECT_TRUE(contains(why(world), "node_materials")) << why(world);
}

TEST(WorldFileRejects, UnknownKeyInsideAMaterialsEntry) {
    const spade::Result<WorldDesc> world = spade::world_from_yaml(replace_first(
        kMinimalWorldV2, "shading: \"lambert\"}", "shading: \"lambert\", glossy: 0.5}"));
    EXPECT_EQ(code_of(world), code(spade::Code::invalid_argument));
    EXPECT_TRUE(contains(why(world), "unknown key 'glossy'")) << why(world);
    EXPECT_TRUE(contains(why(world), "world.materials[0]")) << why(world);
}

TEST(WorldFileRejects, UnknownMaterialShadingNameIsInvalid) {
    const spade::Result<WorldDesc> world =
        spade::world_from_yaml(replace_first(kMinimalWorldV2, "shading: \"lambert\"", "shading: \"shiny\""));
    EXPECT_EQ(code_of(world), code(spade::Code::invalid_argument));
    EXPECT_TRUE(contains(why(world), "unknown material shading 'shiny'")) << why(world);
}

TEST(WorldFileRejects, UnknownKeyInsideLighting) {
    const spade::Result<WorldDesc> world = spade::world_from_yaml(
        replace_first(kMinimalWorldV2, "  sun_intensity: 1\n", "  sun_intensity: 1\n  haze: 0.2\n"));
    EXPECT_EQ(code_of(world), code(spade::Code::invalid_argument));
    EXPECT_TRUE(contains(why(world), "unknown key 'haze'")) << why(world);
    EXPECT_TRUE(contains(why(world), "world.lighting")) << why(world);
}

TEST(WorldFileRejects, UnknownKeyInsideAPropsEntry) {
    const spade::Result<WorldDesc> world = spade::world_from_yaml(replace_first(
        kMinimalWorldV2, "props: []",
        "props:\n  - {mesh_ref: \"mesh:a\", position: [0, 0, 0], orientation: [1, 0, 0, 0], "
        "scale: 1, material: 0, cast_shadow: true}"));
    EXPECT_EQ(code_of(world), code(spade::Code::invalid_argument));
    EXPECT_TRUE(contains(why(world), "unknown key 'cast_shadow'")) << why(world);
    EXPECT_TRUE(contains(why(world), "world.props[0]")) << why(world);
}

TEST(WorldFileRejects, PropWithEmptyMeshRefIsInvalid) {
    const spade::Result<WorldDesc> world = spade::world_from_yaml(replace_first(
        kMinimalWorldV2, "props: []",
        "props:\n  - {mesh_ref: \"\", position: [0, 0, 0], orientation: [1, 0, 0, 0], scale: 1, "
        "material: 0}"));
    EXPECT_EQ(code_of(world), code(spade::Code::invalid_argument));
    EXPECT_TRUE(contains(why(world), "empty mesh_ref")) << why(world);
}

TEST(WorldFileRejects, PropMaterialIndexOutOfRangeViaYamlIsInvalid) {
    const spade::Result<WorldDesc> world = spade::world_from_yaml(replace_first(
        kMinimalWorldV2, "props: []",
        "props:\n  - {mesh_ref: \"mesh:a\", position: [0, 0, 0], orientation: [1, 0, 0, 0], "
        "scale: 1, material: 9}"));
    EXPECT_EQ(code_of(world), code(spade::Code::invalid_argument));
    EXPECT_TRUE(contains(why(world), "material index out of range")) << why(world);
}

TEST(WorldFileRejects, ErrorsFromLoadCarryTheFilePath) {
    const std::filesystem::path path =
        std::filesystem::temp_directory_path() / "spade_test_broken.world.yaml";
    {
        std::ofstream out(path, std::ios::binary | std::ios::trunc);
        ASSERT_TRUE(out.good());
        const std::string broken = replace_first(kMinimalWorld, "  seed: 0", "  seed: 0\n  humidity: 0.4");
        out.write(broken.data(), static_cast<std::streamsize>(broken.size()));
    }
    const spade::Result<WorldDesc> world = spade::load_world_file(path);
    EXPECT_EQ(code_of(world), code(spade::Code::invalid_argument));
    EXPECT_TRUE(contains(why(world), "spade_test_broken.world.yaml")) << why(world);
    EXPECT_TRUE(contains(why(world), "unknown key 'humidity'")) << why(world);
    EXPECT_TRUE(contains(why(world), "line")) << why(world);
    std::filesystem::remove(path);
}

// The writer refuses anything it cannot write back exactly. Both fields it
// declines to carry are dead ones (std430 padding, and an operator node's
// unused `kind`), so this can never fire on a builder-made or file-loaded
// world -- it fires on a hand-assembled SdfProgram, which is precisely the
// case that could otherwise lose bytes silently.
TEST(WorldFileRejects, WritingANodeSchemaV1CannotRepresent) {
    WorldDesc world = maximal_world();
    ASSERT_FALSE(world.sdf.nodes.empty());
    EXPECT_OK(spade::world_to_yaml(world));

    WorldDesc padded = world;
    padded.sdf.nodes[0]._pad = 1;
    const spade::Result<std::string> a = spade::world_to_yaml(padded);
    EXPECT_EQ(code_of(a), code(spade::Code::invalid_argument));
    EXPECT_TRUE(contains(why(a), "non-zero padding")) << why(a);

    WorldDesc kinded = world;
    std::size_t op_index = 0;
    while (op_index < kinded.sdf.nodes.size() &&
           kinded.sdf.nodes[op_index].op == static_cast<uint32_t>(spade::SdfOp::none)) {
        ++op_index;
    }
    ASSERT_LT(op_index, kinded.sdf.nodes.size());
    kinded.sdf.nodes[op_index].kind = 3;
    const spade::Result<std::string> b = spade::world_to_yaml(kinded);
    EXPECT_EQ(code_of(b), code(spade::Code::invalid_argument));
    EXPECT_TRUE(contains(why(b), "non-zero primitive kind")) << why(b);
}

// PA-2 companion (schema v2, S7a task W1). The test above already exercises
// this on maximal_world(), which now carries a real materials palette and a
// fully-populated node_materials array -- this test just says so explicitly,
// so the property "the _pad/kind refusals hold even with the new v2 payload
// present" is a named fact rather than something a reader has to notice.
// _pad sits right next to where a material index might have gone (sdf.hpp's
// own note); this is the test that would fail if it ever did.
TEST(WorldFileRejects, WritingANodeSchemaV2CannotRepresentEitherWithMaterialsPresent) {
    WorldDesc world = maximal_world();
    ASSERT_FALSE(world.materials.empty());
    ASSERT_FALSE(world.sdf.node_materials.empty());
    EXPECT_OK(spade::world_to_yaml(world));

    WorldDesc padded = world;
    padded.sdf.nodes[0]._pad = 1;
    const spade::Result<std::string> a = spade::world_to_yaml(padded);
    EXPECT_EQ(code_of(a), code(spade::Code::invalid_argument));
    EXPECT_TRUE(contains(why(a), "non-zero padding")) << why(a);
}

TEST(WorldFileRejects, WritingAnInvalidWorld) {
    // Nothing invalid is ever written -- world_to_yaml runs the same
    // validation the loader does, ahead of emitting a byte.
    WorldDesc world = maximal_world();
    world.capacities.bodies = 0;
    const spade::Result<std::string> text = spade::world_to_yaml(world);
    EXPECT_EQ(code_of(text), code(spade::Code::invalid_argument));
    EXPECT_TRUE(contains(why(text), "world_to_yaml")) << why(text);
    EXPECT_TRUE(contains(why(text), "'bodies' must be > 0")) << why(text);
}

// ===========================================================================
// 6. validate_world_desc is the SAME function on both paths
// ===========================================================================

TEST(WorldFile, BuilderAndLoaderShareOneValidation) {
    // A world the builder rejects, spelled as a file, must be rejected by the
    // loader with the IDENTICAL error -- same code, same context. If the two
    // paths ever grew separate checks, one of these strings would drift.
    const spade::Result<WorldDesc> built = WorldBuilder()
                                               .name("no_capacity")
                                               .environment(Environment{})
                                               .capacities(Capacities{0, 1, 1, 1})
                                               .build();
    const spade::Result<WorldDesc> parsed =
        spade::world_from_yaml(replace_first(kMinimalWorld, "  bodies: 1", "  bodies: 0"));

    ASSERT_FALSE(built.has_value());
    ASSERT_FALSE(parsed.has_value());
    EXPECT_EQ(built.error().code, parsed.error().code);
    EXPECT_EQ(built.error().context, parsed.error().context);
}

TEST(WorldFile, ValidateWorldDescReturnsTheSdfDepth) {
    const WorldDesc world = maximal_world();
    const spade::Result<uint32_t> depth = spade::validate_world_desc(world);
    ASSERT_OK(depth);
    const spade::Result<uint32_t> from_sdf = world.sdf.validate();
    ASSERT_OK(from_sdf);
    EXPECT_EQ(*depth, *from_sdf);
    EXPECT_EQ(*depth, 3u) << "maximal_world's postfix peaks at three live operands";
}

TEST(WorldFile, VisualRefsAreRenderOnlyAndValidatedNonEmpty) {
    const spade::Result<WorldDesc> empty_ref = WorldBuilder()
                                                   .name("v")
                                                   .environment(Environment{})
                                                   .capacities(Capacities{1, 1, 1, 1})
                                                   .visual("")
                                                   .build();
    EXPECT_EQ(code_of(empty_ref), code(spade::Code::invalid_argument));
    EXPECT_TRUE(contains(why(empty_ref), "visual reference")) << why(empty_ref);

    const spade::Result<WorldDesc> ok = WorldBuilder()
                                            .name("v")
                                            .environment(Environment{})
                                            .capacities(Capacities{1, 1, 1, 1})
                                            .visual("mesh:a")
                                            .visual("mesh:b")
                                            .build();
    ASSERT_OK(ok);
    ASSERT_EQ(ok->visual_refs.size(), 2u);
    EXPECT_EQ(ok->visual_refs[0], "mesh:a");
    EXPECT_EQ(ok->visual_refs[1], "mesh:b");
}

// ===========================================================================
// 7. The committed corpus -- tests/golden/worlds/*.world.yaml
//
// REGENERATION, deliberately made a two-step act rather than a convenience:
// a MISSING file is written and the test fails, so a human reads the new file
// and commits it; an EXISTING file is only ever compared. To regenerate one,
// delete it and re-run -- which leaves the change in the diff where a reviewer
// will see it, instead of quietly under a passing test.
//
// gate.world.yaml is USER CHECKPOINT No.1's reading sample.
// ===========================================================================

TEST(WorldFileCorpus, CommittedFilesMatchTheWriter) {
    for (const CorpusWorld& world : corpus_worlds()) {
        SCOPED_TRACE(world.file);
        const spade::Result<std::string> text = spade::world_to_yaml(world.desc);
        ASSERT_OK(text);

        const std::string path = world_golden_path(world.file);
        std::string committed;
        if (!read_file(path, committed)) {
            std::error_code ec;
            std::filesystem::create_directories(std::filesystem::path(path).parent_path(), ec);
            std::ofstream out(path, std::ios::binary | std::ios::trunc);
            if (out) {
                out.write(text->data(), static_cast<std::streamsize>(text->size()));
            }
            ADD_FAILURE() << path
                          << " did not exist and has been GENERATED. Read it, then commit it.";
            continue;
        }

        // EXPECT_TRUE on the comparison rather than EXPECT_EQ on the strings:
        // gtest would print both 4 KB documents in full and bury the one line
        // that actually moved.
        EXPECT_TRUE(committed == *text)
            << first_difference(committed, *text) << "\n"
            << "The writer no longer reproduces the committed world file. If the change is "
               "intended, delete " << path << " and re-run this test to regenerate it, then "
               "review the diff.";
    }
}

TEST(WorldFileCorpus, CommittedFilesLoadBackToTheirWorlds) {
    for (const CorpusWorld& world : corpus_worlds()) {
        SCOPED_TRACE(world.file);
        const spade::Result<WorldDesc> loaded =
            spade::load_world_file(world_golden_path(world.file));
        ASSERT_OK(loaded);
        expect_same_world(world.desc, *loaded);
    }
}

TEST(WorldFileCorpus, GateWorldHasTheGateAssemblysStructure) {
    // WHAT THIS PINS, EXACTLY, AND WHAT IT DOES NOT.
    //
    // It pins the STRUCTURE of THIS FILE's gate_world() -- the postfix
    // sequence, the transform count, the capacities -- so the world the user
    // approves at the checkpoint cannot be quietly reshaped by a later edit to
    // this test file.
    //
    // It does NOT compare against tools/viewer/scenes.cpp's
    // build_gate_world(), and cannot: spade_viewer is a SPADE_BUILD_V1-gated
    // executable, not a library, and is not linked into spade_tests.
    //
    // UNIFIED AS OF S5 T9 TICKET A: build_gate_world() now spells the ring
    // rotation the same way gate_world() does below -- the exact quaternion
    // (sqrt(1/2), sqrt(1/2), 0, 0) -- rather than glm::angleAxis(radians(90),
    // +X), whose sinf used to land 0.038 ulp from the rounding midpoint (close
    // enough to agree bit-for-bit on this box, but not guaranteed to on a
    // different libm). The two call sites are still separate transcriptions
    // -- nothing here compares them -- but they are no longer free to drift:
    // both read the identical literal.
    const WorldDesc gate = gate_world();
    EXPECT_EQ(gate.name, "gate_world");
    ASSERT_EQ(gate.sdf.nodes.size(), 7u);  // plane torus box union box union union
    EXPECT_EQ(gate.sdf.nodes[0].kind, static_cast<uint32_t>(spade::SdfPrim::plane));
    EXPECT_EQ(gate.sdf.nodes[1].kind, static_cast<uint32_t>(spade::SdfPrim::torus));
    EXPECT_EQ(gate.sdf.nodes[2].kind, static_cast<uint32_t>(spade::SdfPrim::box));
    EXPECT_EQ(gate.sdf.nodes[3].op, static_cast<uint32_t>(spade::SdfOp::union_));
    EXPECT_EQ(gate.sdf.nodes[4].kind, static_cast<uint32_t>(spade::SdfPrim::box));
    EXPECT_EQ(gate.sdf.nodes[5].op, static_cast<uint32_t>(spade::SdfOp::union_));
    EXPECT_EQ(gate.sdf.nodes[6].op, static_cast<uint32_t>(spade::SdfOp::union_));
    ASSERT_EQ(gate.sdf.transforms.size(), 4u);  // identity + ring + two posts
    EXPECT_EQ(gate.capacities.bodies, 5u);
}

// ===========================================================================
// 8. C5 -- world_ref + world_set_from: worlds loadable by path or handle, and
// the training-fleet constructor built on top of them.
// ===========================================================================

namespace {

// A representative instance prototype: real turbulence and dynamics
// parameters, no `.world` set -- world_set_from() is the thing under test for
// how that field gets filled in.
[[nodiscard]] spade::WorldInstanceDesc fleet_prototype() {
    spade::WorldInstanceDesc prototype;
    prototype.turbulence = spade::dryden_params(spade::TurbulenceLevel::light);
    prototype.contacts.restitution_e = 0.3f;
    prototype.contacts.friction_mu = 0.25f;
    prototype.contacts.proxy_radius = 0.1f;
    prototype.grid.cell_size = 0.5f;
    return prototype;
}

}  // namespace

TEST(WorldSetFrom, PathRefAndDescRefProduceIdenticalWorldSets) {
    const spade::WorldRef path_ref = std::filesystem::path(world_golden_path("gate"));
    const spade::WorldRef desc_ref = gate_world();
    const spade::WorldInstanceDesc prototype = fleet_prototype();
    constexpr uint32_t kCount = 4;
    constexpr uint64_t kSceneSeed = 0xC5C5FEEDULL;

    const spade::Result<spade::WorldSetDesc> from_path =
        spade::world_set_from(path_ref, kCount, kSceneSeed, prototype);
    ASSERT_OK(from_path);
    const spade::Result<spade::WorldSetDesc> from_desc =
        spade::world_set_from(desc_ref, kCount, kSceneSeed, prototype);
    ASSERT_OK(from_desc);

    ASSERT_EQ(from_path->worlds.size(), kCount);
    ASSERT_EQ(from_desc->worlds.size(), kCount);
    for (std::size_t i = 0; i < kCount; ++i) {
        SCOPED_TRACE(i);
        expect_same_world(from_path->worlds[i].world, from_desc->worlds[i].world);
        EXPECT_EQ(from_path->worlds[i].seed, from_desc->worlds[i].seed);
        EXPECT_EQ(0, std::memcmp(&from_path->worlds[i].turbulence, &from_desc->worlds[i].turbulence,
                                 sizeof(spade::DrydenParams)));
        EXPECT_EQ(0, std::memcmp(&from_path->worlds[i].contacts, &from_desc->worlds[i].contacts,
                                 sizeof(spade::physics::ContactParams)));
        EXPECT_EQ(0, std::memcmp(&from_path->worlds[i].grid, &from_desc->worlds[i].grid,
                                 sizeof(spade::physics::GridParams)));
    }

    // A good corroborating assertion on top of the structural one above (Task
    // 3's fold covers the entire desc, per-world seeds included).
    EXPECT_EQ(spade::config_hash(*from_path), spade::config_hash(*from_desc));

    // AND THE SEEDS REALLY ARE replicate()'s, not a re-derived formula: rebuild
    // the same fleet through replicate() directly and compare seed-for-seed.
    spade::WorldInstanceDesc expected_prototype = prototype;
    expected_prototype.world = gate_world();
    const spade::WorldSetDesc expected = spade::replicate(expected_prototype, kCount, kSceneSeed);
    ASSERT_EQ(expected.worlds.size(), kCount);
    for (std::size_t i = 0; i < kCount; ++i) {
        EXPECT_EQ(expected.worlds[i].seed, from_path->worlds[i].seed) << i;
    }
}

TEST(WorldSetFrom, MissingFileErrorCarriesThePath) {
    const std::filesystem::path path =
        std::filesystem::temp_directory_path() / "spade_test_world_set_from_missing.world.yaml";
    std::filesystem::remove(path);

    const spade::Result<spade::WorldSetDesc> result =
        spade::world_set_from(spade::WorldRef{path}, 3, 1, fleet_prototype());
    ASSERT_FALSE(result.has_value());
    EXPECT_EQ(result.error().code, spade::Code::io_error);
    EXPECT_TRUE(contains(result.error().context, "spade_test_world_set_from_missing"))
        << result.error().context;
}

TEST(WorldSetFrom, ZeroCountIsRejected) {
    const spade::Result<spade::WorldSetDesc> result =
        spade::world_set_from(spade::WorldRef{gate_world()}, 0, 1, fleet_prototype());
    EXPECT_EQ(code_of(result), code(spade::Code::invalid_argument));
    EXPECT_TRUE(contains(why(result), "count")) << why(result);

    // count == 0 is rejected BEFORE resolving -- a missing/broken path never
    // gets read for a fleet the caller is about to throw away.
    const std::filesystem::path path =
        std::filesystem::temp_directory_path() / "spade_test_world_set_from_zero_count.world.yaml";
    std::filesystem::remove(path);
    const spade::Result<spade::WorldSetDesc> path_result =
        spade::world_set_from(spade::WorldRef{path}, 0, 1, fleet_prototype());
    EXPECT_EQ(code_of(path_result), code(spade::Code::invalid_argument));
    EXPECT_TRUE(contains(why(path_result), "count")) << why(path_result);
}

TEST(WorldSetFrom, FourWorldFleetFromGateWorldCreatesAndStepsASimulation) {
    const std::filesystem::path path = std::filesystem::path(world_golden_path("gate"));
    const spade::Result<spade::WorldSetDesc> set =
        spade::world_set_from(spade::WorldRef{path}, 4, 0xFEED123ULL, fleet_prototype());
    ASSERT_OK(set);
    ASSERT_EQ(set->worlds.size(), 4u);

    spade::Result<spade::Simulation> sim = spade::Simulation::create(*set, 2'000'000, 2);
    ASSERT_OK(sim);
    EXPECT_EQ(sim->world_count(), 4u);
    EXPECT_OK(sim->step(10));
}

// I5 (S5 final-review fix wave). Before this fix, resolve_world()'s desc
// alternative handed back its WorldDesc unvalidated: the path alternative
// always ran validate_world_desc() (via load_world_file()), but a caller
// who built or mutated a WorldDesc some OTHER way -- exactly the case the
// desc alternative exists to accept -- could smuggle an invalid world past
// resolve_world() entirely. This is that gap, closed: gate_world() starts
// with no spawns (asserted below, or this test would not actually introduce
// the duplicate), so appending two spawn points sharing a name produces a
// WorldDesc that never passed through WorldBuilder::build() or a YAML
// loader in this state. Both resolve_world() directly and world_set_from()
// built on top of it must reject it with the SAME diagnostic
// WorldFileRejects.DuplicateSpawnNames pins for the file path -- proving the
// two arms of WorldRef are, once again, truly ONE validation.
TEST(WorldSetFrom, DescRefWithDuplicateSpawnNamesIsRejectedLikeAFileWould) {
    WorldDesc invalid = gate_world();
    ASSERT_TRUE(invalid.spawns.empty()) << "gate_world() must start with no spawns, or this test "
                                            "does not actually introduce the duplicate";
    SpawnPoint start;
    start.name = "start";
    start.position = glm::vec3(0.0f, 1.0f, 0.0f);
    invalid.spawns.push_back(start);
    invalid.spawns.push_back(start);  // same name -- the duplicate

    const spade::Result<WorldDesc> resolved = spade::resolve_world(spade::WorldRef{invalid});
    ASSERT_FALSE(resolved.has_value());
    EXPECT_EQ(code_of(resolved), code(spade::Code::invalid_argument));
    EXPECT_TRUE(contains(why(resolved), "duplicate spawn point name 'start'")) << why(resolved);

    const spade::Result<spade::WorldSetDesc> from_desc =
        spade::world_set_from(spade::WorldRef{invalid}, 2, 1, fleet_prototype());
    ASSERT_FALSE(from_desc.has_value());
    EXPECT_EQ(code_of(from_desc), code(spade::Code::invalid_argument));
    EXPECT_TRUE(contains(why(from_desc), "duplicate spawn point name 'start'")) << why(from_desc);
}
