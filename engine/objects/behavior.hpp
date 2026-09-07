// behavior.hpp -- the behavior registry (24th spec SL6, Plan A Task 8).
//
// SL6: a behavior is DATA -- a name, a schedule slot, declared read/write sets,
// and a CPU implementation. It does not choose where in the substep it runs;
// it names one of TWO FIXED SLOTS the user ruled, and physics/schedule.hpp's
// "no API to add, remove or reorder [a pass]" stays literally true.
//
// NO WALL CLOCK, AND RNG ONLY VIA DOMAIN-SEPARATED SPLITMIX64. A behavior that
// read a clock would make a replay unreproducible, which is the one thing this
// engine's charter does not trade away. Nothing here offers either, and that
// absence is the mechanism rather than a rule to remember.
//
// GPU ELIGIBILITY IS A REFUSAL, NEVER A SILENT FALLBACK. A behavior with no
// record_gpu half makes the whole registry ineligible for the GPU-authoritative
// path. Degrading such a world to the CPU quietly would put a world into the
// parity corpus whose behavior did not run identically on both backends -- the
// corpus would then be comparing two different experiments and passing.
//
// WHY THIS HEADER ONLY FORWARD-DECLARES SubstepContext
// ---------------------------------------------------------------------------
// physics/schedule.hpp holds SubstepContext, and Task 8 gives that struct a
// `const BehaviorRegistry*`. Including each other's headers would be a cycle.
// It is not needed in either direction: a function POINTER whose parameter is
// `const SubstepContext&`, and a member function taking one, are both
// declarable against an incomplete type, and behavior.cpp never touches a
// member of one -- it only forwards the reference. So this header includes
// nothing from physics/ at all. Same technique, same reason, as schedule.hpp's
// own forward declaration of vehicles::RotorRow, which documents the pattern.

#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <string_view>
#include <vector>

#include "core/error.hpp"

namespace spade::physics {
struct SubstepContext;
}  // namespace spade::physics

namespace spade::objects {

// The two fixed slots of SL6. There is no third, and a behavior does not
// choose a position -- it chooses which of these two it belongs to.
enum class BehaviorSlot : uint32_t { kinematic = 0, force = 1 };

// A PLAIN FUNCTION POINTER PLUS AN OPAQUE PARAMETER POINTER, which is the
// minimum shape a parameterised behavior can have while staying data.
//
// Task 8 shipped this without the second argument, and Task 9 found that no
// parameterised behavior can exist without it: the function cannot be a
// capturing lambda (it is a plain pointer, by SL6's "a behavior is data"), it
// cannot read a global (that would make two Simulations in one process share
// one behavior's configuration), and it cannot find its own registry entry
// (nothing tells it which index it is). So the configuration travels beside
// the function, as the C callback shape has always done it.
//
// THE POINTEE MUST OUTLIVE THE REGISTRY AND MUST NOT CHANGE DURING A STEP.
// It is read inside a substep, on the determinism-critical path; mutating it
// mid-step would make the result depend on when the mutation landed.
using BehaviorFn = void (*)(const physics::SubstepContext&, const void* params) noexcept;

// What a caller hands to register_behavior(). `name` is borrowed only for the
// duration of the call -- the registry copies it (see the Entry note below).
struct BehaviorDesc {
    std::string_view name;  // stable identity; the BehaviorComponent's key
    BehaviorSlot slot = BehaviorSlot::kinematic;
    uint32_t reads = 0;   // ComponentTypeId bitmask, as component_mask() speaks
    uint32_t writes = 0;  // ComponentTypeId bitmask
    BehaviorFn execute_cpu = nullptr;  // REQUIRED
    BehaviorFn record_gpu = nullptr;   // absent => CPU-only (SL6)

    // Passed back to execute_cpu/record_gpu verbatim. NOT owned and NOT
    // copied -- the registry stores the pointer, so the caller keeps the
    // pointee alive for as long as the registry is attached to a Simulation.
    // Null is fine for a behavior that needs no configuration.
    const void* user_data = nullptr;
};

class BehaviorRegistry {
  public:
    // Registration order IS execution order within a slot, and it is stated
    // rather than incidental: two behaviors writing the same accumulator must
    // compose in a defined sequence or the result is not reproducible.
    //
    // Errors: invalid_argument when execute_cpu is null, or when `name` is
    // already registered -- names are BehaviorComponent's serialization key, so
    // a duplicate would make a saved graph ambiguous.
    [[nodiscard]] Result<uint32_t> register_behavior(const BehaviorDesc& desc);

    void run_slot(BehaviorSlot slot, const physics::SubstepContext& ctx) const noexcept;

    // False once ANY registered behavior lacks record_gpu. A world using this
    // registry must then be refused from the GPU path, not silently degraded.
    [[nodiscard]] bool gpu_eligible() const noexcept { return gpu_eligible_; }

    [[nodiscard]] std::size_t size() const noexcept { return entries_.size(); }

    // Empty for an out-of-range index. The view is valid until the next
    // register_behavior().
    [[nodiscard]] std::string_view name_at(uint32_t index) const noexcept;

  private:
    // THE NAME IS OWNED HERE, and that is a correction to the plan rather than
    // a style choice. The plan stored BehaviorDesc values directly and kept the
    // strings in a parallel std::vector<std::string>, repointing each stored
    // `name` view at its owned string. That dangles: growing the vector MOVES
    // its elements, and a short std::string keeps its characters inside the
    // object (SSO), so every view into one points at freed memory the moment a
    // reallocation happens -- and every name in this program's tests and in
    // kinematic_mover is short. Worse, it is a use-after-free that usually
    // still reads the right bytes, so a functional test cannot be relied on to
    // catch it. Owning the string removes the aliasing rather than managing it.
    struct Entry {
        std::string name;
        BehaviorSlot slot = BehaviorSlot::kinematic;
        uint32_t reads = 0;
        uint32_t writes = 0;
        BehaviorFn execute_cpu = nullptr;
        BehaviorFn record_gpu = nullptr;
        const void* user_data = nullptr;
    };

    std::vector<Entry> entries_;
    bool gpu_eligible_ = true;
};

}  // namespace spade::objects
