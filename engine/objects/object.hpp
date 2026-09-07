// object.hpp -- object identity and placement (24th spec SL3).
//
// The composition half of the ECS that engine design section 4 ratified and
// never built. Its identity half already exists (core/ids.hpp's generational
// Handle, and spade_state's registered body_generation); this module supplies
// the objects those handles name.
//
// SL3, AND WHY IT IS THE LOAD-BEARING RULE OF THIS MODULE
// ---------------------------------------------------------------------------
// The object graph is NOT registered state. It is composition and identity,
// reconstructible from a description. Components attached to an object hold
// HANDLES INTO THE SoA ARENAS spade_state already owns -- they never own
// simulation data themselves. That is what lets a structural addition this
// large cost nothing in determinism: GPU buffer order is driven by slot
// assignment, never by pool iteration order, so `kSnapshotVersion`, the
// registry walk, and every CPU<->GPU parity band stay byte-unchanged.
//
// Concretely, for anyone extending `Object` below: a field belongs here only
// if it survives "could this be rebuilt from a saved description, with no
// reference to how the simulation happens to be running right now?" Position
// and orientation qualify -- they are authored placement. A velocity, a
// contact set, or an accumulated force does not: that is state, it lives in a
// spade_state arena, and a component references the slot holding it.

#pragma once

#include <cstdint>
#include <string>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include "core/ids.hpp"

namespace spade::objects {

// Phantom tag: keeps object handles from being interchangeable with body,
// world or any other handle at the type level. Never instantiated, which is
// why it is declared and not defined.
struct ObjectTag;

using ObjectId = Handle<ObjectTag>;

// An object is identity plus placement, and nothing else -- see the SL3 note
// above before adding a field.
struct Object {
    std::string name;
    ObjectId parent{};  // null == root

    glm::vec3 position{0.0f};
    // (w, x, y, z) -- glm's quat CONSTRUCTOR order, which is not its storage
    // order. Spelling identity as {1,0,0,0} here rather than relying on
    // glm::quat's default keeps that explicit at the one place it is easiest
    // to get backwards.
    glm::quat orientation{1.0f, 0.0f, 0.0f, 0.0f};
    glm::vec3 scale{1.0f};
};

}  // namespace spade::objects
