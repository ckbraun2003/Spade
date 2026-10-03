// sim/module.hpp -- the module API (docs/design/core/plans/2026-10-02-module-api-design.md).
//
// A MODULE is a descriptor: a name, a version and the passes it contributes.
// create() compiles a module set into ONE ordered pass list with
// compile_schedule(). Stages 1-2 of the plan: passes, each naming the built-in
// GPU kernel that matches its CPU function. State, fields, grades and roles join
// the descriptor in later stages.
#pragma once

#include <cstddef>
#include <cstdint>
#include <span>
#include <string>
#include <string_view>
#include <vector>

#include "compute/backend.hpp"
#include "core/error.hpp"

namespace spade::physics {
struct SubstepContext;
}  // namespace spade::physics

namespace spade::modules {

// A pass's CPU half: the same function-pointer type as physics::PassFn,
// spelt here so this header stays light and does not pull in
// physics/schedule.hpp and everything it includes.
using PassFn = void (*)(const physics::SubstepContext&) noexcept;

// The engine model's six phases, run in this order every substep.
enum class Phase : uint8_t { fields = 0, forces = 1, constraints = 2, integrate = 3, sensors = 4, publish = 5 };
inline constexpr std::size_t kPhaseCount = 6;

// How a pass touches a quantity. ACCUMULATE adds to it and never clears it
// (rotors and drag on body.wrench); two accumulators of one quantity need no
// edge between them, two writers do.
enum class Access : uint8_t { read, write, accumulate };

// Where a pass sits inside its phase beyond what hazards and edges decide.
// Behaviors use it: kinematic behaviors first in Fields, force behaviors last
// in Forces. A placed pass still declares its access: placement orders it
// against passes of another placement, and two passes of the SAME placement
// that conflict need an edge like any others.
enum class Placement : uint8_t { first = 0, ordered = 1, last = 2 };

// A quantity is a core quantity (kCoreQuantities) or "<module>.<name>" for a
// module in the set. Anything else is refused, so a misspelling cannot drop a
// hazard.
inline constexpr std::string_view kCoreQuantities[] = {"body.pose", "body.wrench", "body.specific_force",
                                                       "world.params"};

struct QuantityAccess {
    std::string_view quantity;
    Access access = Access::read;
};

struct PassDecl {
    std::string_view name;
    Phase phase = Phase::fields;
    Placement placement = Placement::ordered;
    std::span<const QuantityAccess> access{};
    std::span<const std::string_view> after{};  // "<module>.<pass>"
    PassFn cpu = nullptr;
    // The built-in kernel that does what `cpu` does, on the GPU; `none` runs
    // only on the CPU, and Vulkan refuses the set at create().
    compute::GpuRecipe gpu = compute::GpuRecipe::none;
};

struct ModuleDesc {
    std::string_view name;  // no '.'
    uint32_t version = 1;
    std::span<const PassDecl> passes{};
};

using ModuleSet = std::vector<ModuleDesc>;

// Owned names: a Simulation keeps its CompiledSchedule for life, and the set
// it was compiled from may have been built from temporary strings.
struct CompiledPass {
    std::string module;
    std::string pass;
    Phase phase = Phase::fields;
    PassFn cpu = nullptr;
    compute::GpuRecipe gpu = compute::GpuRecipe::none;
};

struct CompiledSchedule {
    std::vector<CompiledPass> passes;
    uint64_t identity = 0;  // FNV-1a 64 over the set and the compiled order; see compile_schedule()
};

// Orders every pass of `modules`:
//   1. phases in Phase order;
//   2. inside a phase: placement groups (first, ordered, last); a reader after
//      every writer and accumulator of what it reads; `after` edges; two
//      writers, or a writer and an accumulator, of one placement need an edge;
//   3. ties by module-set order, then declaration order.
// invalid_argument for: a module name that is empty, contains '.', or repeats;
// a pass with no name or no CPU function, or declared twice; a GPU recipe
// paired with any CPU function but builtin_cpu_for(recipe); an unknown
// quantity; an edge to a pass no module declares or to a later phase; two
// writers, or a writer and an accumulator, of one quantity with no edge
// between them; a cycle.
//
// IDENTITY, spelt byte by byte so it is the same on every platform: for each
// module in set order, its name, 0x00 and its version as 4 bytes little-endian;
// then 0x01; then for each compiled pass, module, '.', pass, 0x00 and the phase
// as one byte. FNV-1a 64 (core/rng.hpp's constants) over those bytes.
[[nodiscard]] Result<CompiledSchedule> compile_schedule(std::span<const ModuleDesc> modules);

// Today's engine as modules. The default module set of Simulation::create().
[[nodiscard]] ModuleSet standard_modules();

// The CPU function a GPU recipe stands for; nullptr for `none`. A pass that
// names a recipe must carry exactly this function (compile_schedule refuses
// anything else), so the two backends run one experiment.
[[nodiscard]] PassFn builtin_cpu_for(compute::GpuRecipe recipe) noexcept;

// Every compiled pass as "<module>.<pass>" and its recipe, in schedule order --
// the chain the Vulkan step records. Passes with no recipe are included; the
// caller refuses them.
[[nodiscard]] std::vector<compute::GpuPass> gpu_passes(const CompiledSchedule& schedule);

}  // namespace spade::modules
