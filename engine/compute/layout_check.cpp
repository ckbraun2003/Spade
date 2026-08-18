// ---------------------------------------------------------------------------
// layout_check.cpp -- the translation unit whose ONLY job is to compile the
// generated D9 layout checks, so that a Slang/C++ layout divergence is a BUILD
// FAILURE of spade_compute rather than a test failure, a device error, or a
// wrong trajectory nobody can explain.
//
// THERE IS NO CODE HERE ON PURPOSE. layout_check.gen.hpp
// (cmake/SpadeSlang.cmake generates it into the build tree from slangc
// reflection over engine/shaders/shared/layouts.slang) is nothing but
// static_asserts: one per field of every mirrored struct, plus a size and an
// alignment assert per struct. Including it IS the check; there is nothing to
// call and nothing to link.
//
// WHY A COMPILED TU RATHER THAN A HEADER SOME OTHER FILE ALREADY INCLUDES.
// A header only checks what happens to include it, and only in the
// configurations that build that includer. This TU is unconditionally part of
// spade_compute, so the checks run on EVERY build of the Vulkan backend --
// which is exactly the set of builds where a Slang struct can be wrong.
//
// WHY IT PULLS IN HEADERS FROM SEVEN MODULES. The generated header includes
// whichever engine headers declare the mirrored types -- state/layout.hpp,
// core/rng.hpp, physics/forces.hpp, physics/contacts.hpp, physics/grid.hpp,
// vehicles/rotor.hpp, sensors/imu.hpp, world/medium.hpp, sim/simulation.hpp --
// because it must SEE those types to assert their offsets. That is a
// COMPILE-time reach only: every one of them is header-material this TU never
// calls into, so spade_compute gains no link dependency and the module graph
// (spec section 2) is unchanged. It is also not accidental coupling: the whole
// premise of the compute backend is that it mirrors every registered state
// row, so "compute knows the shape of all registered state" is the design, and
// this file is where that knowledge is checked instead of assumed.
//
// bindings.gen.hpp is included alongside it so the binding registry is
// compiled here too -- a generator bug that emitted a malformed constant would
// otherwise only surface when spade_tests included it.
// ---------------------------------------------------------------------------

#include "compute/layout_check.hpp"

#include "bindings.gen.hpp"
#include "layout_check.gen.hpp"

namespace spade::compute {

// ONE EXTERNALLY VISIBLE SYMBOL, and it earns its place twice over.
//
// Mechanically: an object file whose entire content is static_asserts exports
// nothing, and MSVC's librarian then emits LNK4221 ("no public symbols found;
// archive member will be inaccessible") for it. A namespace-scope `constexpr
// bool` would NOT fix that -- constexpr implies const implies internal
// linkage -- so this is a function, which does export.
//
// Behaviourally: it also gives a caller a way to state the dependency. It
// returns the number of struct layouts the generated header checked
// (kMirroredStructCount, itself generated), so a test can assert that the
// count is what it expects rather than trusting that a silently emptied
// generated header still "passed".
std::size_t layout_check_mirrored_struct_count() noexcept {
    return gen::kMirroredStructCount;
}

}  // namespace spade::compute
