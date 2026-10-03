// spade::scene's asset-pose helper (scene/compose.hpp): an asset's collider
// transform posed by the asset. Plan:
// docs/design/interface/plans/2026-10-03-scene-composer-plan.md, with Core's
// review. It needs none of the scene-file types, so it lands first; the
// composer's own tests follow with Core's schema and transform_of().

#include <gtest/gtest.h>

#include <cmath>
#include <cstring>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>

#include "scene/compose.hpp"
#include "world/sdf.hpp"

namespace {

using spade::SdfTransform;

// The bytes config_hash folds: two transforms are "the same" only if these are.
bool same_bytes(const SdfTransform& a, const SdfTransform& b) {
    return std::memcmp(&a, &b, sizeof(SdfTransform)) == 0;
}

// A pose as WorldBuilder stores it: world_to_local = transpose(R) / s with the
// translation -(m * position), the arithmetic builder.cpp uses. At the identity
// pose that translation is -(m * 0) = -0 in every lane: an identity with
// negative zeros, exactly what the short-cuts must still recognise.
SdfTransform builder_style(const glm::vec3& position, const glm::quat& rotation, float scale) {
    const glm::mat3 m = glm::transpose(glm::mat3_cast(rotation / glm::length(rotation))) / scale;
    const glm::vec3 c = -(m * position);
    SdfTransform t{};
    t.world_to_local = glm::mat4(m);
    t.world_to_local[3] = glm::vec4(c, 1.0f);
    t.scale = scale;
    return t;
}

const glm::quat kTilt = glm::angleAxis(0.7f, glm::normalize(glm::vec3(0.3f, 1.0f, -0.4f)));

}  // namespace

// The reason for the short-cuts: a 4x4 product with the identity is not
// bitwise identity-preserving, because (-0)*1 + (+0) is +0. Rotation matrices
// carry -0 entries, and config_hash folds bytes. If glm ever stopped doing
// this, the short-cuts would still be right, but this test says why they exist.
TEST(SceneComposeTransform, AProductWithTheIdentityCanFlipNegativeZero) {
    const SdfTransform posed = builder_style(glm::vec3(0.0f), kTilt, 1.0f);  // -0 translation
    const glm::mat4 product = posed.world_to_local * glm::mat4(1.0f);
    const glm::mat4 product2 = glm::mat4(1.0f) * posed.world_to_local;
    const bool flipped = std::memcmp(&product, &posed.world_to_local, sizeof(glm::mat4)) != 0 ||
                         std::memcmp(&product2, &posed.world_to_local, sizeof(glm::mat4)) != 0;
    EXPECT_TRUE(flipped) << "the plain product preserved every byte; the short-cuts' stated reason no longer holds";
}

// Short-cut (a): an asset at the identity pose leaves its collider's
// transform bitwise unchanged, negative zeros included.
TEST(SceneComposeTransform, AnIdentityAssetPoseCopiesTheColliderBitwise) {
    const SdfTransform collider = builder_style(glm::vec3(0.0f, 1.8f, 0.0f), kTilt, 1.5f);
    const SdfTransform identity_pose = builder_style(glm::vec3(0.0f), glm::quat(1.0f, 0.0f, 0.0f, 0.0f), 1.0f);
    EXPECT_TRUE(same_bytes(spade::scene::compose_transform(collider, identity_pose), collider));
}

// Short-cut (b): a collider at the identity takes the asset's pose verbatim,
// so a one-primitive asset gets exactly the bytes WorldBuilder would give that
// primitive at the asset's pose.
TEST(SceneComposeTransform, AnIdentityColliderTakesTheAssetPoseVerbatim) {
    const SdfTransform identity_collider = builder_style(glm::vec3(0.0f), glm::quat(1.0f, 0.0f, 0.0f, 0.0f), 1.0f);
    const SdfTransform asset = builder_style(glm::vec3(2.0f, 0.5f, -3.0f), kTilt, 0.8f);
    EXPECT_TRUE(same_bytes(spade::scene::compose_transform(identity_collider, asset), asset));
}

// The general case: a world point goes into the asset's frame, then into the
// collider's. The composed transform does both in one step, and the distance
// scales multiply.
TEST(SceneComposeTransform, TheProductTakesAWorldPointThroughBoth) {
    const SdfTransform collider = builder_style(glm::vec3(0.4f, 0.0f, 0.1f), glm::angleAxis(0.3f, glm::vec3(1, 0, 0)), 1.5f);
    const SdfTransform asset = builder_style(glm::vec3(2.0f, 0.5f, -3.0f), kTilt, 0.8f);
    const SdfTransform composed = spade::scene::compose_transform(collider, asset);
    EXPECT_EQ(composed.scale, collider.scale * asset.scale);
    for (const glm::vec3 p : {glm::vec3(0.0f), glm::vec3(1.0f, 2.0f, 3.0f), glm::vec3(-4.0f, 0.5f, 2.5f)}) {
        const glm::vec4 two_steps = collider.world_to_local * (asset.world_to_local * glm::vec4(p, 1.0f));
        const glm::vec4 one_step = composed.world_to_local * glm::vec4(p, 1.0f);
        EXPECT_NEAR(glm::length(glm::vec3(two_steps - one_step)), 0.0f, 1e-5f) << "point " << p.x << "," << p.y << "," << p.z;
    }
}

// Determinism: the same inputs give the same bytes, every time.
TEST(SceneComposeTransform, ComposingTwiceGivesTheSameBytes) {
    const SdfTransform collider = builder_style(glm::vec3(0.4f, 0.0f, 0.1f), glm::angleAxis(0.3f, glm::vec3(1, 0, 0)), 1.5f);
    const SdfTransform asset = builder_style(glm::vec3(2.0f, 0.5f, -3.0f), kTilt, 0.8f);
    EXPECT_TRUE(same_bytes(spade::scene::compose_transform(collider, asset),
                           spade::scene::compose_transform(collider, asset)));
}
