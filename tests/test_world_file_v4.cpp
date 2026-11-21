// test_world_file_v4.cpp -- the world file's schema version 4 (docs/design/core/2026-11-20-world-file-v4-design.md, Phase 3.4): the four camera and
// keypoint capacity keys and `sdf.node_labels`, and the helper that resolves the label of a ray hit. v4 is v3 plus optional keys, written only when used, so
// the writer's rule (the lowest version that holds the world) keeps every existing world at version 2 or 3, byte for byte.

#include <gtest/gtest.h>

#include <cstddef>
#include <cstdint>
#include <string>

#include <glm/vec3.hpp>

#include "core/geodesy.hpp"
#include "set_body_state_support.hpp"
#include "sim/world_set.hpp"
#include "world/builder.hpp"
#include "world/sdf.hpp"
#include "world/world_file.hpp"

namespace {

using spade::Capacities;
using spade::SdfPose;
using spade::WorldBuilder;
using spade::WorldDesc;

constexpr const char* kDressingMesh = "track/banner";

[[nodiscard]] Capacities caps_with(uint32_t cameras = 0, uint32_t camera_pixels = 0, uint32_t keypoint_groups = 0, uint32_t keypoints_per_group = 0) {
    Capacities caps;
    caps.bodies = 2;
    caps.force_elements = 4;
    caps.sensors = 2;
    caps.contacts = 1;
    caps.cameras = cameras;
    caps.camera_pixels = camera_pixels;
    caps.keypoint_groups = keypoint_groups;
    caps.keypoints_per_group = keypoints_per_group;
    return caps;
}

// A floor plane (node 0), room for bodies; each case adds what it needs.
[[nodiscard]] WorldBuilder base_builder(const Capacities& caps = caps_with()) {
    WorldBuilder b;
    b.name("v4").capacities(caps).plane(glm::vec3(0.0f, 1.0f, 0.0f), 0.0f);
    return b;
}

[[nodiscard]] SdfPose at(float x, float y, float z) {
    SdfPose pose;
    pose.position = glm::vec3(x, y, z);
    return pose;
}

[[nodiscard]] WorldDesc build_ok(const WorldBuilder& b) {
    spade::Result<WorldDesc> w = b.build();
    EXPECT_TRUE(w.has_value()) << w.error().context;
    return w.has_value() ? *w : WorldDesc{};
}

[[nodiscard]] std::string yaml_ok(const WorldDesc& w) {
    const spade::Result<std::string> text = spade::world_to_yaml(w);
    EXPECT_TRUE(text.has_value()) << text.error().context;
    return text.has_value() ? *text : std::string();
}

[[nodiscard]] std::string replace_once(std::string text, const std::string& from, const std::string& to) {
    const std::size_t at_pos = text.find(from);
    EXPECT_NE(at_pos, std::string::npos) << "missing: " << from;
    if (at_pos != std::string::npos) text.replace(at_pos, from.size(), to);
    return text;
}

// The labelled world: the floor plane (node 0) is unlabelled, the box (node 1) carries label 5, and the union (node 2) is unlabelled.
[[nodiscard]] WorldDesc labelled_world() {
    WorldBuilder b = base_builder();
    b.box(glm::vec3(1.0f), SdfPose{}).label_for_last_node(5).union_();
    return build_ok(b);
}

[[nodiscard]] WorldDesc camera_world() { return build_ok(base_builder(caps_with(2, 1'228'800, 5, 4))); }

}  // namespace

TEST(WorldFileV4, ACameraWorldIsWrittenAsVersion4AndRoundTrips) {
    const WorldDesc world = camera_world();
    const std::string text = yaml_ok(world);
    EXPECT_NE(text.find("world_version: 4\n"), std::string::npos) << text;
    EXPECT_NE(text.find("  cameras: 2\n"), std::string::npos);
    EXPECT_NE(text.find("  camera_pixels: 1228800\n"), std::string::npos);
    EXPECT_NE(text.find("  keypoint_groups: 5\n"), std::string::npos);
    EXPECT_NE(text.find("  keypoints_per_group: 4\n"), std::string::npos);
    const spade::Result<WorldDesc> loaded = spade::world_from_yaml(text);
    ASSERT_TRUE(loaded.has_value()) << loaded.error().context;
    EXPECT_EQ(loaded->capacities.cameras, 2u);
    EXPECT_EQ(loaded->capacities.camera_pixels, 1'228'800u);
    EXPECT_EQ(loaded->capacities.keypoint_groups, 5u);
    EXPECT_EQ(loaded->capacities.keypoints_per_group, 4u);
    EXPECT_EQ(yaml_ok(*loaded), text) << "emission is canonical";
}

TEST(WorldFileV4, ALabelledWorldIsWrittenAsVersion4AndRoundTrips) {
    const WorldDesc world = labelled_world();
    ASSERT_EQ(world.sdf.node_labels.size(), world.sdf.nodes.size());
    const std::string text = yaml_ok(world);
    EXPECT_NE(text.find("world_version: 4\n"), std::string::npos) << text;
    EXPECT_NE(text.find("node_labels: [0, 5, 0]"), std::string::npos) << text;
    EXPECT_EQ(text.find("cameras"), std::string::npos) << "no camera capacity, so none of those keys";
    const spade::Result<WorldDesc> loaded = spade::world_from_yaml(text);
    ASSERT_TRUE(loaded.has_value()) << loaded.error().context;
    ASSERT_EQ(loaded->sdf.node_labels.size(), 3u);
    EXPECT_EQ(loaded->sdf.node_labels[0], 0u);
    EXPECT_EQ(loaded->sdf.node_labels[1], 5u);
    EXPECT_EQ(yaml_ok(*loaded), text) << "emission is canonical";
}

TEST(WorldFileV4, AWorldWithNothingNewKeepsItsVersionAndItsKeys) {
    WorldBuilder b = base_builder();
    b.box(glm::vec3(1.0f), SdfPose{}).union_().prop(kDressingMesh, SdfPose{}, 0);
    const WorldDesc plain_world = build_ok(b);
    const std::string plain = yaml_ok(plain_world);
    EXPECT_NE(plain.find("world_version: 2\n"), std::string::npos);
    EXPECT_EQ(plain.find("label"), std::string::npos) << "an unlabelled world writes no label key";
    EXPECT_EQ(plain.find("camera"), std::string::npos);

    WorldDesc anchored = plain_world;
    anchored.anchor = spade::geo::Geodetic{47.0, 8.0, 400.0};
    const std::string v3 = yaml_ok(anchored);
    EXPECT_NE(v3.find("world_version: 3\n"), std::string::npos) << "an anchor alone is still version 3";
    EXPECT_EQ(v3.find("label"), std::string::npos);

    WorldDesc both = labelled_world();
    both.anchor = spade::geo::Geodetic{47.0, 8.0, 400.0};
    const std::string v4 = yaml_ok(both);
    EXPECT_NE(v4.find("world_version: 4\n"), std::string::npos);
    EXPECT_NE(v4.find("anchor:"), std::string::npos) << "version 4 keeps everything version 3 holds";
    const spade::Result<WorldDesc> loaded = spade::world_from_yaml(v4);
    ASSERT_TRUE(loaded.has_value()) << loaded.error().context;
    EXPECT_TRUE(loaded->anchor.has_value());
}

TEST(WorldFileV4, ACapacityKeyMayBeStatedAsZeroOrOmitted) {
    const std::string text = yaml_ok(labelled_world());
    const std::string stated = replace_once(text, "  contacts: 1\n",
                                            "  contacts: 1\n  cameras: 0\n  camera_pixels: 0\n  keypoint_groups: 0\n  keypoints_per_group: 0\n");
    const spade::Result<WorldDesc> loaded = spade::world_from_yaml(stated);
    ASSERT_TRUE(loaded.has_value()) << loaded.error().context;
    EXPECT_EQ(loaded->capacities.cameras, 0u);
    EXPECT_TRUE(spade::world_from_yaml(text).has_value()) << "and omitting them is what the writer does";
}

TEST(WorldFileV4, AV4DocumentWithNothingNewInItStillLoadsAndIsRewrittenAtTheLowestVersion) {
    const std::string plain = yaml_ok(build_ok(base_builder()));
    const std::string relabelled = replace_once(plain, "world_version: 2", "world_version: 4");
    const spade::Result<WorldDesc> loaded = spade::world_from_yaml(relabelled);
    ASSERT_TRUE(loaded.has_value()) << loaded.error().context;
    EXPECT_NE(yaml_ok(*loaded).find("world_version: 2\n"), std::string::npos) << "the lowest-version rule is the writer's";
}

TEST(WorldFileV4, AV4KeyInAVersion3DocumentIsRefusedNamingIt) {
    const auto as_v3 = [](const WorldDesc& w) { return replace_once(yaml_ok(w), "world_version: 4", "world_version: 3"); };
    const spade::Result<WorldDesc> node_labels = spade::world_from_yaml(as_v3(labelled_world()));
    ASSERT_FALSE(node_labels.has_value());
    EXPECT_NE(node_labels.error().context.find("node_labels"), std::string::npos) << node_labels.error().context;
    const spade::Result<WorldDesc> caps = spade::world_from_yaml(as_v3(camera_world()));
    ASSERT_FALSE(caps.has_value());
    EXPECT_NE(caps.error().context.find("cameras"), std::string::npos) << caps.error().context;
}

TEST(WorldFileV4, ALabelIsAUint16AndNodeLabelsMatchTheNodes) {
    const std::string text = yaml_ok(labelled_world());
    const spade::Result<WorldDesc> too_big = spade::world_from_yaml(replace_once(text, "node_labels: [0, 5, 0]", "node_labels: [0, 70000, 0]"));
    ASSERT_FALSE(too_big.has_value());
    EXPECT_NE(too_big.error().context.find("node_labels"), std::string::npos) << too_big.error().context;
    const spade::Result<WorldDesc> short_list = spade::world_from_yaml(replace_once(text, "node_labels: [0, 5, 0]", "node_labels: [5]"));
    ASSERT_FALSE(short_list.has_value());
    EXPECT_NE(short_list.error().context.find("node_labels"), std::string::npos) << short_list.error().context;
}

TEST(WorldFileV4, TheBuilderKeepsNodeLabelsInLockstepWithTheNodes) {
    WorldBuilder b = base_builder();
    b.label_for_last_node(1).box(glm::vec3(1.0f), SdfPose{}).sphere(0.5f, SdfPose{}).union_().union_();
    const WorldDesc w = build_ok(b);
    ASSERT_EQ(w.sdf.nodes.size(), 5u);
    ASSERT_EQ(w.sdf.node_labels.size(), 5u) << "nodes pushed after the first label_for_last_node get label 0, as node_materials does";
    EXPECT_EQ(w.sdf.node_labels[0], 1u);
    EXPECT_EQ(w.sdf.node_labels[4], 0u);
    WorldBuilder before_any;
    before_any.label_for_last_node(1);
    EXPECT_FALSE(before_any.build().has_value()) << "no node was pushed yet";
    WorldDesc mismatched = w;
    mismatched.sdf.node_labels.pop_back();
    const spade::Result<uint32_t> valid = spade::validate_world_desc(mismatched);
    ASSERT_FALSE(valid.has_value());
    EXPECT_NE(valid.error().context.find("node_labels"), std::string::npos) << valid.error().context;
}

TEST(WorldFileV4, LabelsAreRenderOnlyAndStayOutOfTheConfigHash) {
    spade::Result<spade::WorldSetDesc> set = spade::test::floor_world_set(2);
    ASSERT_TRUE(set.has_value()) << set.error().context;
    spade::WorldSetDesc labelled = *set;
    labelled.worlds[0].world.sdf.node_labels = {4};
    EXPECT_EQ(spade::config_hash(labelled), spade::config_hash(*set)) << "two worlds that differ only in labels hash alike";
}

// ---- the label of a hit (Rendering's P1.2 reads it through sdf_hit_label) ---------------------------------------------------------------------------

// A gate as KAT composes it: a torus unioned with two boxes, every node (the operators too) labelled 7.
TEST(SdfHitLabel, AComposedObjectReadsItsOneLabelAnywhereOnIt) {
    WorldBuilder b;
    b.name("gate").capacities(caps_with());
    b.torus(1.0f, 0.2f, at(0.0f, 0.0f, 0.0f)).label_for_last_node(7);
    b.box(glm::vec3(0.2f), at(3.0f, 0.0f, 0.0f)).label_for_last_node(7);
    b.union_().label_for_last_node(7);
    b.box(glm::vec3(0.2f), at(-3.0f, 0.0f, 0.0f)).label_for_last_node(7);
    b.union_().label_for_last_node(7);
    const WorldDesc w = build_ok(b);
    ASSERT_EQ(w.sdf.node_labels.size(), w.sdf.nodes.size());
    EXPECT_EQ(spade::sdf_hit_label(w.sdf, glm::vec3(1.0f, 0.0f, 0.0f)), 7u) << "on the torus ring";
    EXPECT_EQ(spade::sdf_hit_label(w.sdf, glm::vec3(-1.0f, 0.0f, 0.0f)), 7u) << "the far side of the ring";
    EXPECT_EQ(spade::sdf_hit_label(w.sdf, glm::vec3(3.0f, 0.2f, 0.0f)), 7u) << "on the first box";
    EXPECT_EQ(spade::sdf_hit_label(w.sdf, glm::vec3(-3.0f, 0.2f, 0.0f)), 7u) << "on the second box";
}

// Two touching objects: each side of the seam reads its own label, the one of the node that decides the surface there.
TEST(SdfHitLabel, TwoTouchingObjectsEachReadTheirOwnLabel) {
    WorldBuilder b;
    b.name("pair").capacities(caps_with());
    b.box(glm::vec3(0.5f), at(-0.5f, 0.0f, 0.0f)).label_for_last_node(3);
    b.box(glm::vec3(0.5f), at(0.5f, 0.0f, 0.0f)).label_for_last_node(4);
    b.union_();
    const WorldDesc w = build_ok(b);
    EXPECT_EQ(spade::sdf_hit_label(w.sdf, glm::vec3(-0.25f, 0.5f, 0.0f)), 3u) << "the top of the first box";
    EXPECT_EQ(spade::sdf_hit_label(w.sdf, glm::vec3(0.25f, 0.5f, 0.0f)), 4u) << "the top of the second";
    EXPECT_EQ(spade::sdf_hit_label(w.sdf, glm::vec3(-1.0f, 0.0f, 0.0f)), 3u) << "the outer face of the first";
    EXPECT_EQ(spade::sdf_hit_label(w.sdf, glm::vec3(1.0f, 0.0f, 0.0f)), 4u) << "the outer face of the second";
}

TEST(SdfHitLabel, AProgramWithNoLabelsAnswersZero) {
    WorldBuilder b;
    b.name("plain").capacities(caps_with());
    b.box(glm::vec3(0.5f), SdfPose{});
    const WorldDesc w = build_ok(b);
    EXPECT_TRUE(w.sdf.node_labels.empty());
    EXPECT_EQ(spade::sdf_hit_label(w.sdf, glm::vec3(0.5f, 0.0f, 0.0f)), 0u);
    EXPECT_EQ(spade::sdf_hit_label(spade::SdfProgram{}, glm::vec3(0.0f)), 0u) << "and an empty program has no hit";
}
