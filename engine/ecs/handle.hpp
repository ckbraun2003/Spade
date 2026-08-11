#pragma once

#include "core/ids.hpp"

namespace spade {

// The identity handle for ECS entities: an index into Registry's entity
// slot array plus a generation counter, built directly on Task 1's generic
// Handle<Tag> (see core/ids.hpp) rather than reinventing index+generation
// bookkeeping here. EntityTag is a phantom type -- forward-declared inline,
// never defined or instantiated -- that exists only to keep EntityHandle
// distinct at the type level from other Handle<Tag> instantiations (body
// handles, world handles, ...) declared elsewhere in the engine.
using EntityHandle = Handle<struct EntityTag>;

}  // namespace spade
