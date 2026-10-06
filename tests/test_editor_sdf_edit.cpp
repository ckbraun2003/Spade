// The editor's terrain edits over the public SdfProgram (editor plan Task 15;
// EDT-015). Core reviews this helper.
//
// A program is parsed into a tree from its root. Its PIECES are the subtrees a
// chain of plain unions joins at the top; a smooth union, an intersection or a
// subtraction is one piece, whole, and a root that is not a union is the only
// piece. Every edit is checked against what WorldBuilder itself builds, byte
// for byte through world_to_yaml(), which also proves the result is
// representable in a world file. Exact no-ops return the input unchanged.

#include <gtest/gtest.h>

#include <array>
#include <string>
#include <vector>

#include <glm/gtc/quaternion.hpp>

#include "../sandbox/editor_sdf_edit.hpp"
#include "editor_test_support.hpp"
#include "world/builder.hpp"
#include "world/world_file.hpp"

namespace {

namespace ed = spade::sandbox::editor;
namespace support = spade::sandbox::editor::test;
using spade::Result;
using spade::SdfOp;
using spade::SdfPose;
using spade::SdfPrim;
using spade::SdfProgram;
using spade::WorldBuilder;

[[nodiscard]] WorldBuilder builder() {
    WorldBuilder b;
    b.name("terrain").capacities(spade::Capacities{1, 1, 1, 1});
    return b;
}

[[nodiscard]] SdfProgram built(WorldBuilder& b) {
    const Result<spade::WorldDesc> w = b.build();
    EXPECT_TRUE(w.has_value()) << (w ? "" : w.error().context);
    return w ? w->sdf : SdfProgram{};
}

// The world file's canonical text of a program: equal text, equal bytes. A
// program the world file refuses (not representable, not valid) fails the
// test here, so two refusals never compare equal.
[[nodiscard]] std::string text_of(const SdfProgram& p) {
    spade::WorldDesc w = builder().plane(glm::vec3(0, 1, 0), 0.0f).build().value();
    w.sdf = p;
    const Result<std::string> t = spade::world_to_yaml(w);
    EXPECT_TRUE(t.has_value()) << (t ? "" : t.error().context);
    return t ? *t : std::string("<refused>");
}

[[nodiscard]] SdfPose pose(glm::vec3 at, float angle = 0.0f, float scale = 1.0f) {
    SdfPose p;
    p.position = at;
    p.rotation = glm::angleAxis(angle, glm::normalize(glm::vec3(0.2f, 1.0f, 0.1f)));
    p.scale = scale;
    return p;
}

const SdfPose kBoxAt = pose(glm::vec3(2.0f, 0.5f, -1.0f), 0.4f);
const SdfPose kBallAt = pose(glm::vec3(-3.0f, 1.0f, 0.0f));

// The points programs are compared at.
[[nodiscard]] std::vector<glm::vec3> samples() {
    std::vector<glm::vec3> out;
    for (float x = -4.0f; x <= 4.0f; x += 1.25f) {
        for (float y = -1.0f; y <= 3.0f; y += 1.0f) {
            for (float z = -3.0f; z <= 3.0f; z += 1.5f) out.emplace_back(x, y, z);
        }
    }
    return out;
}

void expect_same_field(const SdfProgram& a, const SdfProgram& b) {
    for (const glm::vec3& p : samples()) {
        EXPECT_EQ(spade::eval(a, p), spade::eval(b, p)) << p.x << " " << p.y << " " << p.z;
    }
}

// ground ∪ box ∪ (sphere ∩ cylinder) ∪ (capsule ⊕ torus)
[[nodiscard]] SdfProgram four_pieces() {
    WorldBuilder b = builder();
    b.plane(glm::vec3(0, 1, 0), 0.0f)
        .box(glm::vec3(0.5f), kBoxAt)
        .union_()
        .sphere(1.0f, kBallAt)
        .cylinder(0.5f, 1.0f, kBallAt)
        .intersect()
        .union_()
        .capsule(0.3f, 0.6f, pose(glm::vec3(0.0f, 2.0f, 2.0f)))
        .torus(0.8f, 0.2f, pose(glm::vec3(0.5f, 2.0f, 2.0f)))
        .smooth_union(0.25f)
        .union_();
    return built(b);
}

}  // namespace

TEST(EditorSdfEdit, PiecesFollowTheTopLevelUnionChain) {
    const Result<std::vector<ed::SdfPiece>> pieces = ed::union_pieces(four_pieces());
    ASSERT_TRUE(pieces.has_value()) << pieces.error().context;
    ASSERT_EQ(pieces->size(), 4u);
    EXPECT_EQ((*pieces)[0].op, static_cast<uint32_t>(SdfOp::none));        // the ground
    EXPECT_EQ((*pieces)[1].op, static_cast<uint32_t>(SdfOp::none));        // the box
    EXPECT_EQ((*pieces)[2].op, static_cast<uint32_t>(SdfOp::intersect));   // sphere ∩ cylinder, whole
    EXPECT_EQ((*pieces)[3].op, static_cast<uint32_t>(SdfOp::smooth_union));  // not split: not associative
    EXPECT_EQ((*pieces)[2].last - (*pieces)[2].first, 2u);
}

TEST(EditorSdfEdit, ARootThatIsNotAUnionIsTheOnlyPiece) {
    WorldBuilder b = builder();
    b.plane(glm::vec3(0, 1, 0), 0.0f).box(glm::vec3(1.0f), kBoxAt).subtract();
    const Result<std::vector<ed::SdfPiece>> pieces = ed::union_pieces(built(b));
    ASSERT_TRUE(pieces.has_value());
    ASSERT_EQ(pieces->size(), 1u);
    EXPECT_EQ((*pieces)[0].op, static_cast<uint32_t>(SdfOp::subtract));
    EXPECT_TRUE(ed::union_pieces(SdfProgram{}).value().empty());
}

TEST(EditorSdfEdit, AddingAPrimitiveIsWhatTheBuilderBuilds) {
    WorldBuilder before = builder();
    before.plane(glm::vec3(0, 1, 0), 0.0f).sphere(1.0f, kBallAt).union_();
    WorldBuilder after = builder();
    after.plane(glm::vec3(0, 1, 0), 0.0f).sphere(1.0f, kBallAt).union_().box(glm::vec3(0.5f, 0.25f, 1.0f), kBoxAt).union_();
    const Result<SdfProgram> added =
        ed::add_primitive(built(before), SdfPrim::box, glm::vec4(0.5f, 0.25f, 1.0f, 0.0f), kBoxAt);
    ASSERT_TRUE(added.has_value()) << added.error().context;
    EXPECT_EQ(text_of(*added), text_of(built(after)));
    // At the identity pose the builder reuses transform 0, and so does the helper.
    WorldBuilder at_origin = builder();
    at_origin.plane(glm::vec3(0, 1, 0), 0.0f).sphere(0.5f).union_();
    WorldBuilder ground = builder();
    ground.plane(glm::vec3(0, 1, 0), 0.0f);
    const Result<SdfProgram> at_identity = ed::add_primitive(built(ground), SdfPrim::sphere, glm::vec4(0.5f, 0, 0, 0), SdfPose{});
    ASSERT_TRUE(at_identity.has_value());
    EXPECT_EQ(text_of(*at_identity), text_of(built(at_origin)));
}

TEST(EditorSdfEdit, AddThenRemoveRestoresTheBytesOfEveryAssetWorld) {
    for (const auto& entry : std::filesystem::directory_iterator(std::filesystem::path(SPADE_ASSETS_DIR) / "worlds")) {
        if (!entry.path().string().ends_with(".world.yaml")) continue;
        const Result<spade::WorldDesc> w = spade::load_world_file(entry.path());
        ASSERT_TRUE(w.has_value()) << entry.path();
        const Result<SdfProgram> added = ed::add_primitive(w->sdf, SdfPrim::box, glm::vec4(0.5f, 0.5f, 0.5f, 0.0f), kBoxAt);
        ASSERT_TRUE(added.has_value()) << entry.path() << ": " << added.error().context;
        const std::size_t last = ed::union_pieces(*added).value().size() - 1;
        const Result<SdfProgram> removed = ed::remove_piece(*added, last);
        ASSERT_TRUE(removed.has_value()) << entry.path() << ": " << removed.error().context;
        EXPECT_EQ(text_of(*removed), text_of(w->sdf)) << entry.path();
    }
}

TEST(EditorSdfEdit, RemovingAPieceLeavesWhatTheBuilderBuildsWithoutIt) {
    WorldBuilder abc = builder();
    abc.plane(glm::vec3(0, 1, 0), 0.0f).box(glm::vec3(0.5f), kBoxAt).union_().sphere(1.0f, kBallAt).union_();
    WorldBuilder ac = builder();
    ac.plane(glm::vec3(0, 1, 0), 0.0f).sphere(1.0f, kBallAt).union_();
    const Result<SdfProgram> removed = ed::remove_piece(built(abc), 1);
    ASSERT_TRUE(removed.has_value()) << removed.error().context;
    EXPECT_EQ(text_of(*removed), text_of(built(ac)));
}

TEST(EditorSdfEdit, ACompoundPieceIsRemovedWhole) {
    const Result<SdfProgram> removed = ed::remove_piece(four_pieces(), 2);
    ASSERT_TRUE(removed.has_value()) << removed.error().context;
    WorldBuilder expected = builder();
    expected.plane(glm::vec3(0, 1, 0), 0.0f)
        .box(glm::vec3(0.5f), kBoxAt)
        .union_()
        .capsule(0.3f, 0.6f, pose(glm::vec3(0.0f, 2.0f, 2.0f)))
        .torus(0.8f, 0.2f, pose(glm::vec3(0.5f, 2.0f, 2.0f)))
        .smooth_union(0.25f)
        .union_();
    EXPECT_EQ(text_of(*removed), text_of(built(expected)));
}

TEST(EditorSdfEdit, TheOnlyPieceAndAPieceBeyondTheEndAreRefused) {
    WorldBuilder ground = builder();
    ground.plane(glm::vec3(0, 1, 0), 0.0f);
    const Result<SdfProgram> only = ed::remove_piece(built(ground), 0);
    ASSERT_FALSE(only.has_value());
    EXPECT_NE(only.error().context.find("only piece"), std::string::npos) << only.error().context;
    const Result<SdfProgram> beyond = ed::remove_piece(four_pieces(), 4);
    ASSERT_FALSE(beyond.has_value());
    EXPECT_NE(beyond.error().context.find("4 pieces"), std::string::npos) << beyond.error().context;
}

TEST(EditorSdfEdit, ReposingAPrimitivePieceIsWhatTheBuilderBuildsThere) {
    WorldBuilder before = builder();
    before.plane(glm::vec3(0, 1, 0), 0.0f).box(glm::vec3(0.5f), kBoxAt).union_();
    const SdfPose moved = pose(glm::vec3(-1.0f, 0.75f, 3.0f), 1.1f, 1.5f);
    WorldBuilder after = builder();
    after.plane(glm::vec3(0, 1, 0), 0.0f).box(glm::vec3(0.5f), moved).union_();
    const Result<SdfProgram> reposed = ed::repose_piece(built(before), 1, moved);
    ASSERT_TRUE(reposed.has_value()) << reposed.error().context;
    EXPECT_EQ(text_of(*reposed), text_of(built(after)));
}

TEST(EditorSdfEdit, ReposingAPieceThatSharesItsTransformMovesItAlone) {
    // Two primitives at the identity pose share transform 0.
    WorldBuilder shared = builder();
    shared.sphere(0.5f).box(glm::vec3(0.25f)).union_();
    const Result<SdfProgram> reposed = ed::repose_piece(built(shared), 1, kBoxAt);
    ASSERT_TRUE(reposed.has_value()) << reposed.error().context;
    WorldBuilder expected = builder();
    expected.sphere(0.5f).box(glm::vec3(0.25f), kBoxAt).union_();
    expect_same_field(*reposed, built(expected));
    (void)text_of(*reposed);  // representable: text_of() fails the test otherwise
}

TEST(EditorSdfEdit, ReposingAPieceOfSeveralPosesIsRefusedNamingItsOperator) {
    const Result<SdfProgram> r = ed::repose_piece(four_pieces(), 3, kBoxAt);
    ASSERT_FALSE(r.has_value());
    EXPECT_NE(r.error().context.find("smooth_union"), std::string::npos) << r.error().context;
}

TEST(EditorSdfEdit, ExactNoOpsReturnTheInputUnchanged) {
    WorldBuilder b = builder();
    b.plane(glm::vec3(0, 1, 0), 0.0f).box(glm::vec3(0.5f), kBoxAt).union_();
    const SdfProgram p = built(b);
    const Result<SdfProgram> same_pose = ed::repose_piece(p, 1, kBoxAt);
    ASSERT_TRUE(same_pose.has_value());
    EXPECT_EQ(text_of(*same_pose), text_of(p));
    const Result<SdfProgram> same_params = ed::set_node_params(p, 1, p.nodes[1].params);
    ASSERT_TRUE(same_params.has_value());
    EXPECT_EQ(text_of(*same_params), text_of(p));
}

TEST(EditorSdfEdit, NodeParamsAreSetAndValidated) {
    WorldBuilder b = builder();
    b.plane(glm::vec3(0, 1, 0), 0.0f).box(glm::vec3(0.5f), kBoxAt).union_();
    WorldBuilder bigger = builder();
    bigger.plane(glm::vec3(0, 1, 0), 0.0f).box(glm::vec3(1.0f, 0.5f, 0.5f), kBoxAt).union_();
    const Result<SdfProgram> set = ed::set_node_params(built(b), 1, glm::vec4(1.0f, 0.5f, 0.5f, 0.0f));
    ASSERT_TRUE(set.has_value()) << set.error().context;
    EXPECT_EQ(text_of(*set), text_of(built(bigger)));
    EXPECT_FALSE(ed::set_node_params(built(b), 1, glm::vec4(-1.0f, 0.5f, 0.5f, 0.0f)).has_value());
    const Result<SdfProgram> beyond = ed::set_node_params(built(b), 3, glm::vec4(1.0f));
    ASSERT_FALSE(beyond.has_value());
    EXPECT_NE(beyond.error().context.find("3 nodes"), std::string::npos) << beyond.error().context;
}

TEST(EditorSdfEdit, NodeMaterialsStayParallelToTheNodes) {
    WorldBuilder b = builder();
    spade::MaterialDesc red;
    red.name = "red";
    b.material(red).plane(glm::vec3(0, 1, 0), 0.0f).box(glm::vec3(0.5f), kBoxAt).material_for_last_node(1).union_();
    const SdfProgram p = built(b);
    ASSERT_EQ(p.node_materials.size(), p.nodes.size());
    const Result<SdfProgram> added = ed::add_primitive(p, SdfPrim::sphere, glm::vec4(0.5f, 0, 0, 0), kBallAt, 1);
    ASSERT_TRUE(added.has_value()) << added.error().context;
    ASSERT_EQ(added->node_materials.size(), added->nodes.size());
    EXPECT_EQ(added->node_materials[added->nodes.size() - 2], 1u);  // the sphere
    const Result<SdfProgram> removed = ed::remove_piece(*added, 1);  // the box
    ASSERT_TRUE(removed.has_value()) << removed.error().context;
    ASSERT_EQ(removed->node_materials.size(), removed->nodes.size());
    EXPECT_EQ(removed->node_materials[1], 1u);  // the sphere kept its material
}
