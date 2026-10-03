// ---------------------------------------------------------------------------
// scene/compose.cpp -- see compose.hpp.
//
// Compiled under spade_fp_strict, like every digest-feeding target: the
// composed transforms land in the SDF program, which config_hash folds, so gcc
// must not contract these products into FMA.
// ---------------------------------------------------------------------------
#include "scene/compose.hpp"

namespace spade::scene {
namespace {

// Value comparison on purpose: -0 == 0, so the builder's identity pose, which
// stores its translation as -0, counts as the identity.
bool is_identity(const SdfTransform& t) {
    return t.world_to_local == glm::mat4(1.0f) && t.scale == 1.0f;
}

}  // namespace

SdfTransform compose_transform(const SdfTransform& collider, const SdfTransform& asset_pose) {
    if (is_identity(asset_pose)) {
        return collider;  // short-cut (a)
    }
    if (is_identity(collider)) {
        return asset_pose;  // short-cut (b)
    }
    SdfTransform out{};
    out.world_to_local = collider.world_to_local * asset_pose.world_to_local;
    out.scale = collider.scale * asset_pose.scale;
    return out;
}

}  // namespace spade::scene
