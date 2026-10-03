# Module API Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Turn Spade's fixed ten-pass schedule into modules placed by a phase scheduler, in six stages that each keep every golden digest unchanged.

**Architecture:** A module is a descriptor (name, version, passes with phase, quantity access and ordering edges). `create()` compiles the module set into one ordered pass list and an identity hash. The CPU step runs that list; later stages derive the GPU chain from it, add fields, move state into modules, add grades and roles, and add the translation lock.

**Tech Stack:** C++23, GoogleTest through ctest, MSVC + Ninja presets, Slang/Vulkan (stage 2 onward).

**Spec:** `2026-10-02-module-api-design.md` (approved by the user 2026-10-02).

## How this plan is shaped

The six stages of spec §14 run in sequence, and each merges on its own. Stage 2 onward builds on interfaces that stage 1 creates, so **stage 1 is written in full below, and stages 2–6 are outlined**: goal, files, done-when, and what each consumes. Each later stage gets its own task-level plan in this directory once the stage before it merges, written against the code as it then stands. Writing their code now would be guessing at stage 1's results.

## Global Constraints

- Every golden digest stays unchanged at every stage, and every parity band holds without widening (spec §1).
- No allocation in a step, and no clock in the step path (`L1`).
- A snapshot restores only into the same module set and schedule (`L2`).
- Refuse, never skip: an unavailable module, backend or grade is refused, never silently dropped (`L6`).
- Builds and tests run only through `scripts\build.ps1` and `scripts\test.ps1`, in the foreground, in a slot the lead hands out. Code lives in `../spade-wt/core`, on branch `core/modules-stage1`.
- Commit only named paths (`git commit <paths>`). Never push.

## Rulings made in this plan

- **Plan Ruling 1, where the module identity lives.** Spec §3 says the module set and schedule go into the configuration hash. The hash stored in `replay_config` is folded into every golden digest, so adding to it would move every golden, which §1 forbids. The identity therefore goes into the **snapshot header** instead: format version 2 gains a `configuration_identity` field, and restore refuses a mismatch. The `L2` behaviour is exactly as specified; only its storage differs. *Cost if wrong:* the snapshot format changes from v1 to v2, and that change is consumer-visible (`../../consumers.md`); KAT passes blobs through without parsing them.
- **Plan Ruling 2, Vulkan in stage 1.** Until stage 2 derives the GPU chain, the recorder still runs its own table. A Vulkan simulation with any module set other than the standard set is therefore refused at `create()`. *Cost if wrong:* developer modules are CPU-only for one stage longer than they need be.
- **Plan Ruling 3, quantity names are checked.** A quantity is either a core quantity (`body.pose`, `body.wrench`, `body.specific_force`, `world.params`) or `<module>.<name>` for a module in the set. Anything else is refused. *Why:* a misspelt quantity would silently drop a hazard.

## Review Focus

1. **A developer module on a Vulkan simulation in stage 1** must be refused with a message naming the module set, never run with the hand-kept GPU table. Pinned in Task 3.
2. **A blob from another module set or another module version** must be refused with a reason, not restored into the wrong schedule. Pinned in Task 4.
3. **A misspelt quantity name** (`body.wrnch`) must be refused at `create()`, not silently ignored as a separate quantity. Pinned in Task 1.
4. **The compiled order must not depend on the platform:** no unordered containers in the compiler, and an identity whose bytes are spelt out. Pinned in Task 1 by a hard-coded identity value, computed independently.
5. **A kinematic behavior now runs before Dryden, where it used to run after.** Neither reads what the other writes, so nothing changes. The existing `KinematicMover.*` tests stay green, and Task 3's full suite is the check.

---

## Stage 1: modules and the scheduler, on the CPU

**Done when:** the standard module set compiles to the order in spec §4, the CPU step runs that compiled list, a CPU-only developer module plugs in, every golden digest is unchanged (full suite on the release preset), and a snapshot carries and checks the configuration identity.

### Task 1: The module types and the schedule compiler

**Files:**
- Create: `engine/sim/module.hpp`
- Create: `engine/sim/module_schedule.cpp`
- Modify: `engine/CMakeLists.txt` (add `sim/module_schedule.cpp` to `spade_sim`, after `physics/schedule.cpp`)
- Test: `tests/test_module_schedule.cpp` (create; add to `spade_tests` in `tests/CMakeLists.txt` after `test_sim_medium.cpp`)

**Interfaces:**
- Consumes: `spade::physics::PassFn` and `spade::physics::SubstepContext` (`physics/schedule.hpp`); `spade::rng::kFnv1aOffsetBasis` and `kFnv1aPrime` (`core/rng.hpp`).
- Produces (namespace `spade::modules`, header `sim/module.hpp`):
  - `enum class Phase : uint8_t { fields, forces, constraints, integrate, sensors, publish }`, numbered 0–5, plus `kPhaseCount = 6`;
  - `enum class Access : uint8_t { read, write, accumulate }`;
  - `enum class Placement : uint8_t { first, ordered, last }`;
  - `struct QuantityAccess { std::string_view quantity; Access access; }`;
  - `struct PassDecl { std::string_view name; Phase phase; Placement placement; std::span<const QuantityAccess> access; std::span<const std::string_view> after; physics::PassFn cpu; }`;
  - `struct ModuleDesc { std::string_view name; uint32_t version; std::span<const PassDecl> passes; }`;
  - `using ModuleSet = std::vector<ModuleDesc>`;
  - `struct CompiledPass { std::string module; std::string pass; Phase phase; physics::PassFn cpu; }`. The names are owned, so a set built from temporary strings cannot leave them dangling inside `Simulation`;
  - `struct CompiledSchedule { std::vector<CompiledPass> passes; uint64_t identity; }`;
  - `Result<CompiledSchedule> compile_schedule(std::span<const ModuleDesc> modules)`.

- [ ] **Step 1: Write the failing tests**

Create `tests/test_module_schedule.cpp`:

```cpp
// test_module_schedule.cpp -- the module API's schedule compiler
// (docs/design/core/plans/2026-10-02-module-api-design.md, section 4).

#include <gtest/gtest.h>

#include <string>
#include <string_view>
#include <vector>

#include "sim/module.hpp"

namespace {

using spade::modules::Access;
using spade::modules::CompiledSchedule;
using spade::modules::compile_schedule;
using spade::modules::ModuleDesc;
using spade::modules::PassDecl;
using spade::modules::Phase;
using spade::modules::Placement;
using spade::modules::QuantityAccess;

void noop(const spade::physics::SubstepContext&) noexcept {}

using Names = std::vector<std::string>;

[[nodiscard]] Names names(const CompiledSchedule& s) {
    Names out;
    for (const auto& p : s.passes) out.push_back(std::string(p.module) + "." + std::string(p.pass));
    return out;
}

[[nodiscard]] spade::Code code_of(const spade::Result<CompiledSchedule>& r) {
    return r.has_value() ? spade::Code::internal : r.error().code;
}

}  // namespace

TEST(ModuleSchedule, PhasesRunInOrderWhateverTheSetOrder) {
    static constexpr PassDecl late[] = {{.name = "late", .phase = Phase::sensors, .cpu = &noop}};
    static constexpr PassDecl early[] = {{.name = "early", .phase = Phase::fields, .cpu = &noop}};
    const ModuleDesc set[] = {{.name = "a", .passes = late}, {.name = "b", .passes = early}};
    const auto s = compile_schedule(set);
    ASSERT_TRUE(s.has_value()) << s.error().context;
    EXPECT_EQ(names(*s), (Names{"b.early", "a.late"}));
}

TEST(ModuleSchedule, AReaderRunsAfterAWriterOfTheSameQuantity) {
    static constexpr QuantityAccess reads[] = {{"w.x", Access::read}};
    static constexpr QuantityAccess writes[] = {{"w.x", Access::write}};
    static constexpr PassDecl reader[] = {{.name = "read", .phase = Phase::forces, .access = reads, .cpu = &noop}};
    static constexpr PassDecl writer[] = {{.name = "write", .phase = Phase::forces, .access = writes, .cpu = &noop}};
    const ModuleDesc set[] = {{.name = "r", .passes = reader}, {.name = "w", .passes = writer}};
    const auto s = compile_schedule(set);
    ASSERT_TRUE(s.has_value()) << s.error().context;
    EXPECT_EQ(names(*s), (Names{"w.write", "r.read"}));
}

TEST(ModuleSchedule, AccumulatorsNeedNoEdgeAndFollowSetOrder) {
    static constexpr QuantityAccess adds[] = {{"body.wrench", Access::accumulate}};
    static constexpr PassDecl p[] = {{.name = "add", .phase = Phase::forces, .access = adds, .cpu = &noop}};
    static constexpr PassDecl q[] = {{.name = "add", .phase = Phase::forces, .access = adds, .cpu = &noop}};
    const ModuleDesc set[] = {{.name = "p", .passes = p}, {.name = "q", .passes = q}};
    const auto s = compile_schedule(set);
    ASSERT_TRUE(s.has_value()) << s.error().context;
    EXPECT_EQ(names(*s), (Names{"p.add", "q.add"}));
}

TEST(ModuleSchedule, AnEdgeOrdersAccumulatorsAgainstSetOrder) {
    static constexpr QuantityAccess adds[] = {{"body.wrench", Access::accumulate}};
    static constexpr std::string_view after_q[] = {"q.add"};
    static constexpr PassDecl p[] = {
        {.name = "add", .phase = Phase::forces, .access = adds, .after = after_q, .cpu = &noop}};
    static constexpr PassDecl q[] = {{.name = "add", .phase = Phase::forces, .access = adds, .cpu = &noop}};
    const ModuleDesc set[] = {{.name = "p", .passes = p}, {.name = "q", .passes = q}};
    const auto s = compile_schedule(set);
    ASSERT_TRUE(s.has_value()) << s.error().context;
    EXPECT_EQ(names(*s), (Names{"q.add", "p.add"}));
}

TEST(ModuleSchedule, TwoWritersWithoutAnEdgeAreRefused) {
    static constexpr QuantityAccess writes[] = {{"body.pose", Access::write}};
    static constexpr PassDecl a[] = {{.name = "fix", .phase = Phase::constraints, .access = writes, .cpu = &noop}};
    static constexpr PassDecl b[] = {{.name = "fix", .phase = Phase::constraints, .access = writes, .cpu = &noop}};
    const ModuleDesc set[] = {{.name = "a", .passes = a}, {.name = "b", .passes = b}};
    EXPECT_EQ(code_of(compile_schedule(set)), spade::Code::invalid_argument);
}

TEST(ModuleSchedule, AWriterAndAnAccumulatorWithoutAnEdgeAreRefused) {
    static constexpr QuantityAccess writes[] = {{"body.wrench", Access::write}};
    static constexpr QuantityAccess adds[] = {{"body.wrench", Access::accumulate}};
    static constexpr PassDecl a[] = {{.name = "set", .phase = Phase::forces, .access = writes, .cpu = &noop}};
    static constexpr PassDecl b[] = {{.name = "add", .phase = Phase::forces, .access = adds, .cpu = &noop}};
    const ModuleDesc set[] = {{.name = "a", .passes = a}, {.name = "b", .passes = b}};
    EXPECT_EQ(code_of(compile_schedule(set)), spade::Code::invalid_argument);
}

TEST(ModuleSchedule, TwoWritersWithAnEdgeCompileInEdgeOrder) {
    static constexpr QuantityAccess writes[] = {{"body.pose", Access::write}};
    static constexpr std::string_view after_b[] = {"b.fix"};
    static constexpr PassDecl a[] = {
        {.name = "fix", .phase = Phase::constraints, .access = writes, .after = after_b, .cpu = &noop}};
    static constexpr PassDecl b[] = {{.name = "fix", .phase = Phase::constraints, .access = writes, .cpu = &noop}};
    const ModuleDesc set[] = {{.name = "a", .passes = a}, {.name = "b", .passes = b}};
    const auto s = compile_schedule(set);
    ASSERT_TRUE(s.has_value()) << s.error().context;
    EXPECT_EQ(names(*s), (Names{"b.fix", "a.fix"}));
}

TEST(ModuleSchedule, ACycleIsRefused) {
    static constexpr std::string_view after_b[] = {"b.two"};
    static constexpr std::string_view after_a[] = {"a.one"};
    static constexpr PassDecl a[] = {{.name = "one", .phase = Phase::forces, .after = after_b, .cpu = &noop}};
    static constexpr PassDecl b[] = {{.name = "two", .phase = Phase::forces, .after = after_a, .cpu = &noop}};
    const ModuleDesc set[] = {{.name = "a", .passes = a}, {.name = "b", .passes = b}};
    EXPECT_EQ(code_of(compile_schedule(set)), spade::Code::invalid_argument);
}

TEST(ModuleSchedule, EdgesMustNameAPassInThisOrAnEarlierPhase) {
    static constexpr std::string_view to_nobody[] = {"nobody.here"};
    static constexpr std::string_view to_late[] = {"b.late"};
    static constexpr std::string_view to_early[] = {"b.early"};
    static constexpr PassDecl dangling[] = {{.name = "x", .phase = Phase::forces, .after = to_nobody, .cpu = &noop}};
    static constexpr PassDecl into_later[] = {{.name = "x", .phase = Phase::forces, .after = to_late, .cpu = &noop}};
    static constexpr PassDecl into_earlier[] = {{.name = "x", .phase = Phase::forces, .after = to_early, .cpu = &noop}};
    static constexpr PassDecl b[] = {{.name = "late", .phase = Phase::sensors, .cpu = &noop},
                                     {.name = "early", .phase = Phase::fields, .cpu = &noop}};
    const ModuleDesc s1[] = {{.name = "a", .passes = dangling}, {.name = "b", .passes = b}};
    const ModuleDesc s2[] = {{.name = "a", .passes = into_later}, {.name = "b", .passes = b}};
    const ModuleDesc s3[] = {{.name = "a", .passes = into_earlier}, {.name = "b", .passes = b}};
    EXPECT_EQ(code_of(compile_schedule(s1)), spade::Code::invalid_argument);
    EXPECT_EQ(code_of(compile_schedule(s2)), spade::Code::invalid_argument);
    EXPECT_TRUE(compile_schedule(s3).has_value()) << "an edge to an earlier phase is already satisfied";
}

TEST(ModuleSchedule, FirstAndLastPlacementBracketTheOrderedPasses) {
    static constexpr PassDecl m[] = {
        {.name = "mid", .phase = Phase::forces, .cpu = &noop},
        {.name = "end", .phase = Phase::forces, .placement = Placement::last, .cpu = &noop},
        {.name = "top", .phase = Phase::forces, .placement = Placement::first, .cpu = &noop},
    };
    const ModuleDesc set[] = {{.name = "m", .passes = m}};
    const auto s = compile_schedule(set);
    ASSERT_TRUE(s.has_value()) << s.error().context;
    EXPECT_EQ(names(*s), (Names{"m.top", "m.mid", "m.end"}));
}

TEST(ModuleSchedule, TwoConflictingPassesOfOnePlacementAreRefused) {
    static constexpr QuantityAccess writes[] = {{"body.pose", Access::write}};
    static constexpr PassDecl a[] = {
        {.name = "pin", .phase = Phase::fields, .placement = Placement::first, .access = writes, .cpu = &noop}};
    static constexpr PassDecl b[] = {
        {.name = "pin", .phase = Phase::fields, .placement = Placement::first, .access = writes, .cpu = &noop}};
    const ModuleDesc set[] = {{.name = "a", .passes = a}, {.name = "b", .passes = b}};
    EXPECT_EQ(code_of(compile_schedule(set)), spade::Code::invalid_argument);
}

TEST(ModuleSchedule, PlacementOrdersAPlacedWriterBeforeAnOrderedWriter) {
    static constexpr QuantityAccess writes[] = {{"body.pose", Access::write}};
    static constexpr PassDecl ordered[] = {{.name = "move", .phase = Phase::fields, .access = writes, .cpu = &noop}};
    static constexpr PassDecl placed[] = {
        {.name = "pin", .phase = Phase::fields, .placement = Placement::first, .access = writes, .cpu = &noop}};
    const ModuleDesc set[] = {{.name = "o", .passes = ordered}, {.name = "p", .passes = placed}};
    const auto s = compile_schedule(set);
    ASSERT_TRUE(s.has_value()) << s.error().context;
    EXPECT_EQ(names(*s), (Names{"p.pin", "o.move"}));
}

TEST(ModuleSchedule, APlacementThatContradictsAHazardIsRefused) {
    // A first-placed READER and an ordered WRITER of one quantity: the hazard
    // wants the writer first, the placement wants the reader first.
    static constexpr QuantityAccess reads[] = {{"body.pose", Access::read}};
    static constexpr QuantityAccess writes[] = {{"body.pose", Access::write}};
    static constexpr PassDecl r[] = {
        {.name = "look", .phase = Phase::fields, .placement = Placement::first, .access = reads, .cpu = &noop}};
    static constexpr PassDecl w[] = {{.name = "move", .phase = Phase::fields, .access = writes, .cpu = &noop}};
    const ModuleDesc set[] = {{.name = "r", .passes = r}, {.name = "w", .passes = w}};
    EXPECT_EQ(code_of(compile_schedule(set)), spade::Code::invalid_argument);
}

TEST(ModuleSchedule, UnknownQuantitiesAreRefused) {
    static constexpr QuantityAccess misspelt[] = {{"body.wrnch", Access::accumulate}};
    static constexpr QuantityAccess no_owner[] = {{"ghost.x", Access::read}};
    static constexpr PassDecl a[] = {{.name = "x", .phase = Phase::forces, .access = misspelt, .cpu = &noop}};
    static constexpr PassDecl b[] = {{.name = "x", .phase = Phase::forces, .access = no_owner, .cpu = &noop}};
    const ModuleDesc s1[] = {{.name = "a", .passes = a}};
    const ModuleDesc s2[] = {{.name = "b", .passes = b}};
    EXPECT_EQ(code_of(compile_schedule(s1)), spade::Code::invalid_argument);
    EXPECT_EQ(code_of(compile_schedule(s2)), spade::Code::invalid_argument);
}

TEST(ModuleSchedule, MalformedSetsAreRefused) {
    static constexpr PassDecl ok[] = {{.name = "x", .phase = Phase::forces, .cpu = &noop}};
    static constexpr PassDecl no_cpu[] = {{.name = "x", .phase = Phase::forces}};
    const ModuleDesc twice[] = {{.name = "a", .passes = ok}, {.name = "a", .passes = ok}};
    const ModuleDesc dotted[] = {{.name = "a.b", .passes = ok}};
    const ModuleDesc null_fn[] = {{.name = "a", .passes = no_cpu}};
    EXPECT_EQ(code_of(compile_schedule(twice)), spade::Code::invalid_argument);
    EXPECT_EQ(code_of(compile_schedule(dotted)), spade::Code::invalid_argument);
    EXPECT_EQ(code_of(compile_schedule(null_fn)), spade::Code::invalid_argument);
}

// The identity is spelt byte by byte (module name, 0x00, version as 4 bytes
// little-endian; then 0x01; then per compiled pass: module, '.', pass, 0x00,
// phase byte), FNV-1a 64 over the result. The constant below was computed by
// an independent Python implementation of that spelling, so a platform- or
// container-dependent compiler fails here.
TEST(ModuleSchedule, IdentityIsSpeltOutAndChangesWithVersionOrOrder) {
    static constexpr PassDecl one[] = {{.name = "one", .phase = Phase::forces, .cpu = &noop}};
    static constexpr PassDecl two[] = {{.name = "two", .phase = Phase::fields, .cpu = &noop}};
    const ModuleDesc set[] = {{.name = "alpha", .version = 1, .passes = one},
                              {.name = "beta", .version = 2, .passes = two}};
    const auto s = compile_schedule(set);
    ASSERT_TRUE(s.has_value()) << s.error().context;
    EXPECT_EQ(s->identity, 0x2a62d4b78a25f25aULL);

    const ModuleDesc bumped[] = {{.name = "alpha", .version = 1, .passes = one},
                                 {.name = "beta", .version = 3, .passes = two}};
    const ModuleDesc swapped[] = {{.name = "beta", .version = 2, .passes = two},
                                  {.name = "alpha", .version = 1, .passes = one}};
    EXPECT_NE(compile_schedule(bumped)->identity, s->identity);
    EXPECT_NE(compile_schedule(swapped)->identity, s->identity);
}
```

Register it in `tests/CMakeLists.txt`, in the `add_executable(spade_tests ...)` list:

```cmake
    test_sim_medium.cpp
    test_module_schedule.cpp
```

- [ ] **Step 2: Ask the lead for a build slot; build and run the new tests**

Run: `scripts\build.ps1 -Target spade_tests`
Expected: FAIL to compile, with `sim/module.hpp` not found.

- [ ] **Step 3: Write `engine/sim/module.hpp`**

```cpp
// sim/module.hpp -- the module API (docs/design/core/plans/2026-10-02-module-api-design.md).
//
// A MODULE is a descriptor: a name, a version and the passes it contributes.
// create() compiles a module set into ONE ordered pass list with
// compile_schedule(). Stage 1 of the plan: passes only. State, fields, grades,
// roles and GPU recordings join the descriptor in later stages.
#pragma once

#include <cstddef>
#include <cstdint>
#include <span>
#include <string>
#include <string_view>
#include <vector>

#include "core/error.hpp"
#include "physics/schedule.hpp"

namespace spade::modules {

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
    physics::PassFn cpu = nullptr;
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
    physics::PassFn cpu = nullptr;
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
// a pass with no name or no CPU function, or declared twice; an unknown
// quantity; an edge to a pass no module declares or to a later phase; two
// writers, or a writer and an accumulator, of one quantity with no edge
// between them; a cycle.
//
// IDENTITY, spelt byte by byte so it is the same on every platform: for each
// module in set order, its name, 0x00 and its version as 4 bytes little-endian;
// then 0x01; then for each compiled pass, module, '.', pass, 0x00 and the phase
// as one byte. FNV-1a 64 (core/rng.hpp's constants) over those bytes.
[[nodiscard]] Result<CompiledSchedule> compile_schedule(std::span<const ModuleDesc> modules);

}  // namespace spade::modules
```

- [ ] **Step 4: Write `engine/sim/module_schedule.cpp`**

```cpp
#include "sim/module.hpp"

#include <map>
#include <string>
#include <tuple>
#include <utility>

#include "core/rng.hpp"

namespace spade::modules {
namespace {

[[nodiscard]] Error invalid(std::string context) { return Error{Code::invalid_argument, std::move(context)}; }

struct Node {
    std::size_t module = 0;  // index in the set
    std::size_t decl = 0;    // index in that module's passes
    const PassDecl* pass = nullptr;
    std::string full;        // "<module>.<pass>"
};

// Two passes touching one quantity need an explicit edge when both write, or
// when one writes and the other accumulates.
[[nodiscard]] bool needs_edge(Access a, Access b) noexcept {
    return (a == Access::write && b != Access::read) || (b == Access::write && a != Access::read);
}

[[nodiscard]] bool known_quantity(std::string_view q, std::span<const ModuleDesc> modules) noexcept {
    for (const std::string_view core : kCoreQuantities) {
        if (q == core) return true;
    }
    const std::size_t dot = q.find('.');
    if (dot == std::string_view::npos || dot == 0 || dot + 1 == q.size()) return false;
    const std::string_view owner = q.substr(0, dot);
    for (const ModuleDesc& m : modules) {
        if (m.name == owner) return true;
    }
    return false;
}

[[nodiscard]] uint64_t fold_byte(uint64_t h, uint8_t b) noexcept {
    h ^= b;
    return h * rng::kFnv1aPrime;
}

[[nodiscard]] uint64_t fold_str(uint64_t h, std::string_view s) noexcept {
    for (const char c : s) h = fold_byte(h, static_cast<uint8_t>(c));
    return h;
}

[[nodiscard]] uint64_t fold_u32_le(uint64_t h, uint32_t v) noexcept {
    for (int i = 0; i < 4; ++i) h = fold_byte(h, static_cast<uint8_t>(v >> (8 * i)));
    return h;
}

}  // namespace

Result<CompiledSchedule> compile_schedule(std::span<const ModuleDesc> modules) {
    std::vector<Node> nodes;
    std::map<std::string, std::size_t, std::less<>> by_name;

    for (std::size_t m = 0; m < modules.size(); ++m) {
        const ModuleDesc& mod = modules[m];
        if (mod.name.empty() || mod.name.find('.') != std::string_view::npos) {
            return std::unexpected(invalid("module " + std::to_string(m) + ": a module needs a name with no '.'"));
        }
        for (std::size_t k = 0; k < m; ++k) {
            if (modules[k].name == mod.name) {
                return std::unexpected(invalid("module '" + std::string(mod.name) + "' appears twice in the set"));
            }
        }
        for (std::size_t d = 0; d < mod.passes.size(); ++d) {
            const PassDecl& p = mod.passes[d];
            std::string full = std::string(mod.name) + "." + std::string(p.name);
            if (p.name.empty() || p.cpu == nullptr) {
                return std::unexpected(invalid(full + ": a pass needs a name and a CPU function"));
            }
            if (static_cast<std::size_t>(p.phase) >= kPhaseCount) {
                return std::unexpected(invalid(full + ": unknown phase"));
            }
            if (by_name.contains(full)) {
                return std::unexpected(invalid(full + ": declared twice"));
            }
            for (const QuantityAccess& qa : p.access) {
                if (!known_quantity(qa.quantity, modules)) {
                    return std::unexpected(invalid(full + ": unknown quantity '" + std::string(qa.quantity) +
                                                   "' (a core quantity, or <module>.<name> for a module in the set)"));
                }
            }
            by_name.emplace(full, nodes.size());
            nodes.push_back(Node{m, d, &p, std::move(full)});
        }
    }

    const std::size_t n = nodes.size();
    std::vector<std::vector<std::size_t>> preds(n);              // preds[i]: must run before i
    std::vector<std::vector<bool>> edge_path(n, std::vector<bool>(n, false));  // by explicit edges only

    for (std::size_t i = 0; i < n; ++i) {
        for (const std::string_view target : nodes[i].pass->after) {
            const auto it = by_name.find(target);
            if (it == by_name.end()) {
                return std::unexpected(invalid(nodes[i].full + ": edge to '" + std::string(target) +
                                               "', which no module in the set declares"));
            }
            const Node& t = nodes[it->second];
            if (t.pass->phase > nodes[i].pass->phase) {
                return std::unexpected(invalid(nodes[i].full + ": edge to '" + t.full + "', which runs in a later phase"));
            }
            if (t.pass->phase == nodes[i].pass->phase) {
                preds[i].push_back(it->second);
                edge_path[it->second][i] = true;
            }
        }
    }
    for (std::size_t k = 0; k < n; ++k) {
        for (std::size_t i = 0; i < n; ++i) {
            if (!edge_path[i][k]) continue;
            for (std::size_t j = 0; j < n; ++j) {
                if (edge_path[k][j]) edge_path[i][j] = true;
            }
        }
    }

    for (std::size_t i = 0; i < n; ++i) {
        for (std::size_t j = i + 1; j < n; ++j) {
            const PassDecl& a = *nodes[i].pass;
            const PassDecl& b = *nodes[j].pass;
            if (a.phase != b.phase) continue;
            if (a.placement != b.placement) {
                if (a.placement < b.placement) {
                    preds[j].push_back(i);
                } else {
                    preds[i].push_back(j);
                }
            }
            for (const QuantityAccess& qa : a.access) {
                for (const QuantityAccess& qb : b.access) {
                    if (qa.quantity != qb.quantity) continue;
                    if (needs_edge(qa.access, qb.access)) {
                        // A different placement already orders the pair.
                        if (a.placement == b.placement && !edge_path[i][j] && !edge_path[j][i]) {
                            return std::unexpected(invalid(nodes[i].full + " and " + nodes[j].full + " both write '" +
                                                           std::string(qa.quantity) +
                                                           "' with no edge between them"));
                        }
                    } else if (qa.access == Access::read && qb.access != Access::read) {
                        preds[i].push_back(j);
                    } else if (qb.access == Access::read && qa.access != Access::read) {
                        preds[j].push_back(i);
                    }
                }
            }
        }
    }

    std::vector<std::size_t> indegree(n, 0);
    std::vector<std::vector<std::size_t>> succs(n);
    for (std::size_t i = 0; i < n; ++i) {
        for (const std::size_t p : preds[i]) {
            succs[p].push_back(i);
            ++indegree[i];
        }
    }
    const auto key = [&](std::size_t i) {
        return std::tuple(static_cast<int>(nodes[i].pass->phase), nodes[i].module, nodes[i].decl);
    };

    CompiledSchedule out;
    out.passes.reserve(n);
    std::vector<bool> done(n, false);
    for (std::size_t emitted = 0; emitted < n; ++emitted) {
        std::size_t best = n;
        for (std::size_t i = 0; i < n; ++i) {
            if (!done[i] && indegree[i] == 0 && (best == n || key(i) < key(best))) best = i;
        }
        if (best == n) {
            std::string names;
            for (std::size_t i = 0; i < n; ++i) {
                if (!done[i]) names += " " + nodes[i].full;
            }
            return std::unexpected(invalid("these passes form a cycle:" + names));
        }
        done[best] = true;
        for (const std::size_t s : succs[best]) --indegree[s];
        const Node& nd = nodes[best];
        out.passes.push_back(CompiledPass{std::string(modules[nd.module].name), std::string(nd.pass->name),
                                          nd.pass->phase, nd.pass->cpu});
    }

    uint64_t h = rng::kFnv1aOffsetBasis;
    for (const ModuleDesc& m : modules) {
        h = fold_str(h, m.name);
        h = fold_byte(h, 0);
        h = fold_u32_le(h, m.version);
    }
    h = fold_byte(h, 1);
    for (const CompiledPass& p : out.passes) {
        h = fold_str(h, p.module);
        h = fold_byte(h, '.');
        h = fold_str(h, p.pass);
        h = fold_byte(h, 0);
        h = fold_byte(h, static_cast<uint8_t>(p.phase));
    }
    out.identity = h;
    return out;
}

}  // namespace spade::modules
```

Add the source to `spade_sim` in `engine/CMakeLists.txt`:

```cmake
add_library(spade_sim STATIC
    physics/schedule.cpp
    sim/module_schedule.cpp
```

- [ ] **Step 5: Build and run the new tests**

Run: `scripts\build.ps1 -Target spade_tests`, then `scripts\test.ps1 -Filter ModuleSchedule`
Expected: 15 tests, all PASS.

- [ ] **Step 6: Commit**

```bash
git commit engine/sim/module.hpp engine/sim/module_schedule.cpp engine/CMakeLists.txt tests/test_module_schedule.cpp tests/CMakeLists.txt -m "feat(core): module descriptors and the schedule compiler"
```

### Task 2: Today's passes as the standard module set

**Files:**
- Modify: `engine/physics/schedule.hpp`, `engine/physics/schedule.cpp`. Add `pass_rotor_forces`, `pass_drag`, `pass_sensor_imu` and `pass_sensor_gnss`, which split the two combined passes.
- Create: `engine/sim/standard_modules.cpp`
- Modify: `engine/sim/module.hpp` (declare `standard_modules()`), `engine/CMakeLists.txt`
- Test: `tests/test_module_schedule.cpp`

**Interfaces:**
- Consumes: Task 1's types and `compile_schedule`.
- Produces:
  - `spade::modules::ModuleSet standard_modules()`, in this set order: drag, dryden, imu, rotor, gnss, behaviors, static_contact, dynamic_contact, integrate;
  - `void physics::pass_rotor_forces(const SubstepContext&) noexcept`;
  - `void physics::pass_drag(const SubstepContext&) noexcept`;
  - `void physics::pass_sensor_imu(const SubstepContext&) noexcept`;
  - `void physics::pass_sensor_gnss(const SubstepContext&) noexcept`.

- [ ] **Step 1: Write the failing tests** (append to `tests/test_module_schedule.cpp`)

```cpp
// Spec section 4's table: the standard set compiles to today's order, with the
// two documented changes (kinematic behaviors before Dryden; the empty
// Gravity and Publish passes gone).
TEST(StandardModules, CompileToTodaysOrder) {
    const spade::modules::ModuleSet set = spade::modules::standard_modules();
    const auto s = compile_schedule(set);
    ASSERT_TRUE(s.has_value()) << s.error().context;
    EXPECT_EQ(names(*s), (Names{"behaviors.kinematic", "dryden.advance", "rotor.forces", "drag.forces",
                                "behaviors.force", "static_contact.resolve", "dynamic_contact.resolve",
                                "integrate.integrate", "imu.synthesize", "gnss.synthesize"}));
}

// Without drag's edge, set order (drag before rotor: the golden walk's order)
// would run drag first and move every golden with a rotor. The edge is
// load-bearing, so dropping it must change both the order and the identity.
TEST(StandardModules, DroppingDragsEdgeChangesTheOrderAndTheIdentity) {
    spade::modules::ModuleSet set = spade::modules::standard_modules();
    const auto with_edge = compile_schedule(set);
    ASSERT_TRUE(with_edge.has_value()) << with_edge.error().context;

    static constexpr QuantityAccess drag_access[] = {{"body.pose", Access::read},
                                                     {"body.wrench", Access::accumulate},
                                                     {"dryden.state", Access::read}};
    static constexpr PassDecl drag_no_edge[] = {
        {.name = "forces", .phase = Phase::forces, .access = drag_access, .cpu = &spade::physics::pass_drag}};
    ASSERT_EQ(set[0].name, "drag");
    set[0].passes = drag_no_edge;

    const auto without = compile_schedule(set);
    ASSERT_TRUE(without.has_value()) << without.error().context;
    EXPECT_EQ(names(*without)[2], "drag.forces");
    EXPECT_EQ(names(*without)[3], "rotor.forces");
    EXPECT_NE(without->identity, with_edge->identity);
}
```

- [ ] **Step 2: Build; expect a compile failure** (`standard_modules` and `pass_drag` undeclared).

Run: `scripts\build.ps1 -Target spade_tests`
Expected: FAIL to compile.

- [ ] **Step 3: Split the combined passes** in `engine/physics/schedule.cpp`, beside `pass_force_elements` and `pass_sensor_synthesis`, which stay until Task 3 removes them:

```cpp
// The ForceElements chain, split into its two modules' passes. Each world's
// bodies still receive rotors' wrench before drag's: the loops run world by
// world, and worlds share no body, so splitting the per-world call pair into
// two all-world loops changes no accumulation order.
void pass_rotor_forces(const SubstepContext& ctx) noexcept {
    for (const WorldSubstepView& w : ctx.worlds) {
        const DrydenMedium medium(*w.dryden, *w.dryden_params);
        vehicles::apply_rotors(w.bodies, w.rotors, *w.sdf, medium, *w.params, ctx.h);
    }
}

void pass_drag(const SubstepContext& ctx) noexcept {
    for (const WorldSubstepView& w : ctx.worlds) {
        const DrydenMedium medium(*w.dryden, *w.dryden_params);
        apply_drag(w.bodies, w.drag_elements, medium, *w.params, ctx.h);
    }
}

// SensorSynthesis, split the same way. IMU and GNSS read bodies as const and
// write disjoint arrays from separately tagged streams, so they commute.
void pass_sensor_imu(const SubstepContext& ctx) noexcept {
    for (const WorldSubstepView& w : ctx.worlds) {
        sensors::synthesize_imu(w.bodies, w.imu_sensors, w.imu_ring, ctx.tick.value);
    }
}

void pass_sensor_gnss(const SubstepContext& ctx) noexcept {
    for (const WorldSubstepView& w : ctx.worlds) {
        sensors::synthesize_gnss(w.bodies, w.gnss_sensors, w.gnss_ring, ctx.tick.value);
    }
}
```

Declare all four in `engine/physics/schedule.hpp`, after `pass_force_elements`:

```cpp
void pass_rotor_forces(const SubstepContext&) noexcept;
void pass_drag(const SubstepContext&) noexcept;
void pass_sensor_imu(const SubstepContext&) noexcept;
void pass_sensor_gnss(const SubstepContext&) noexcept;
```

- [ ] **Step 4: Write `engine/sim/standard_modules.cpp`**

```cpp
// The standard module set: today's engine as modules (plan stage 1).
#include "sim/module.hpp"

namespace spade::modules {
namespace {

constexpr QuantityAccess kDrydenAccess[] = {{"dryden.state", Access::write}};
constexpr PassDecl kDrydenPasses[] = {
    {.name = "advance", .phase = Phase::fields, .access = kDrydenAccess, .cpu = &physics::pass_medium_update}};

constexpr QuantityAccess kRotorAccess[] = {{"body.pose", Access::read},
                                           {"body.wrench", Access::accumulate},
                                           {"rotor.state", Access::write},
                                           {"dryden.state", Access::read}};
constexpr PassDecl kRotorPasses[] = {
    {.name = "forces", .phase = Phase::forces, .access = kRotorAccess, .cpu = &physics::pass_rotor_forces}};

// Rotors then drag is an fp32 accumulation order the goldens pin.
constexpr QuantityAccess kDragAccess[] = {{"body.pose", Access::read},
                                          {"body.wrench", Access::accumulate},
                                          {"dryden.state", Access::read}};
constexpr std::string_view kDragAfter[] = {"rotor.forces"};
constexpr PassDecl kDragPasses[] = {{.name = "forces",
                                     .phase = Phase::forces,
                                     .access = kDragAccess,
                                     .after = kDragAfter,
                                     .cpu = &physics::pass_drag}};

constexpr QuantityAccess kImuAccess[] = {{"body.pose", Access::read},
                                         {"body.specific_force", Access::read},
                                         {"imu.state", Access::write}};
constexpr PassDecl kImuPasses[] = {
    {.name = "synthesize", .phase = Phase::sensors, .access = kImuAccess, .cpu = &physics::pass_sensor_imu}};

constexpr QuantityAccess kGnssAccess[] = {{"body.pose", Access::read}, {"gnss.state", Access::write}};
constexpr PassDecl kGnssPasses[] = {
    {.name = "synthesize", .phase = Phase::sensors, .access = kGnssAccess, .cpu = &physics::pass_sensor_gnss}};

// Kinematic behaviors write poses before any field is sampled (Q2), so they
// run first in Fields; force behaviors add after every built-in force. Both
// declare what they touch, so the compiler checks them like any other pass.
constexpr QuantityAccess kKinematicAccess[] = {{"body.pose", Access::write}};
constexpr QuantityAccess kForceBehaviorAccess[] = {{"body.wrench", Access::accumulate}};
constexpr PassDecl kBehaviorPasses[] = {
    {.name = "kinematic", .phase = Phase::fields, .placement = Placement::first, .access = kKinematicAccess,
     .cpu = &physics::pass_behaviors_kinematic},
    {.name = "force", .phase = Phase::forces, .placement = Placement::last, .access = kForceBehaviorAccess,
     .cpu = &physics::pass_behaviors_force},
};

constexpr QuantityAccess kContactAccess[] = {{"body.pose", Access::write}};
constexpr PassDecl kStaticPasses[] = {{.name = "resolve",
                                       .phase = Phase::constraints,
                                       .access = kContactAccess,
                                       .cpu = &physics::pass_collision_static}};
// Both contact passes correct pos and vel: dynamic runs on static's result.
constexpr std::string_view kDynamicAfter[] = {"static_contact.resolve"};
constexpr PassDecl kDynamicPasses[] = {{.name = "resolve",
                                        .phase = Phase::constraints,
                                        .access = kContactAccess,
                                        .after = kDynamicAfter,
                                        .cpu = &physics::pass_collision_dynamic}};

constexpr QuantityAccess kIntegrateAccess[] = {{"body.pose", Access::write},
                                               {"body.wrench", Access::write},
                                               {"body.specific_force", Access::write},
                                               {"world.params", Access::read}};
constexpr PassDecl kIntegratePasses[] = {
    {.name = "integrate", .phase = Phase::integrate, .access = kIntegrateAccess, .cpu = &physics::pass_integrate}};

}  // namespace

ModuleSet standard_modules() {
    // Set order is today's registration order for the modules that own state
    // (drag, dryden, imu, rotor, gnss: the golden walk, which stage 4 hands to
    // the modules), then the stateless ones.
    return {
        {.name = "drag", .passes = kDragPasses},
        {.name = "dryden", .passes = kDrydenPasses},
        {.name = "imu", .passes = kImuPasses},
        {.name = "rotor", .passes = kRotorPasses},
        {.name = "gnss", .passes = kGnssPasses},
        {.name = "behaviors", .passes = kBehaviorPasses},
        {.name = "static_contact", .passes = kStaticPasses},
        {.name = "dynamic_contact", .passes = kDynamicPasses},
        {.name = "integrate", .passes = kIntegratePasses},
    };
}

}  // namespace spade::modules
```

In `engine/sim/module.hpp`, after `compile_schedule`:

```cpp
// Today's engine as modules. The default module set of Simulation::create().
[[nodiscard]] ModuleSet standard_modules();
```

Add `sim/standard_modules.cpp` to `spade_sim` in `engine/CMakeLists.txt`, after `sim/module_schedule.cpp`.

- [ ] **Step 5: Build and run**

Run: `scripts\build.ps1 -Target spade_tests`, then `scripts\test.ps1 -Filter "ModuleSchedule|StandardModules"`
Expected: 17 tests, all PASS.

- [ ] **Step 6: Commit**

```bash
git commit engine/physics/schedule.hpp engine/physics/schedule.cpp engine/sim/standard_modules.cpp engine/sim/module.hpp engine/CMakeLists.txt tests/test_module_schedule.cpp -m "feat(core): today's passes as the standard module set"
```

### Task 3: The CPU step runs the compiled schedule

**Files:**
- Modify: `engine/sim/simulation.hpp`, `engine/sim/simulation.cpp`
- Modify: `engine/physics/schedule.hpp`, `engine/physics/schedule.cpp`. Remove `Pass`, `kSchedule`, `kSubstepPassCount`, `substep_schedule`, `run_substep`, `pass_gravity`, `pass_publish`, `pass_force_elements` and `pass_sensor_synthesis`, together with their comments.
- Modify: `tests/test_determinism.cpp`. Replace `Schedule.IsSpecSectionThreeInOrderWithSL6sTwoBehaviorSlots` and `Schedule.RemovingTheBehaviorSlotsLeavesSpecSectionThreeExactly`.
- Test: `tests/test_module_schedule.cpp`

**Interfaces:**
- Consumes: `compile_schedule`, `standard_modules` (Tasks 1–2).
- Produces:
  - `static Result<Simulation> Simulation::create(const WorldSetDesc&, uint64_t dt_ns, uint32_t substeps, const compute::BackendDesc& backend = {}, const modules::ModuleSet& modules = modules::standard_modules())`;
  - `const modules::CompiledSchedule& Simulation::schedule() const noexcept`.

- [ ] **Step 1: Write the failing tests** (append to `tests/test_module_schedule.cpp`, with these includes added at the top: `"sim/simulation.hpp"`, `"sim/world_set.hpp"`, `"world/builder.hpp"`, `"world/medium.hpp"`, `"compute/backend.hpp"`)

```cpp
namespace {

// A developer module: one pass that holds body 0 of every world up against
// gravity by adding m*(-g) to force_acc. Mass 2 (a power of two) makes the
// cancellation in Integrate exact.
void hover_pass(const spade::physics::SubstepContext& ctx) noexcept {
    for (const spade::physics::WorldSubstepView& w : ctx.worlds) {
        spade::BodyState& b = w.bodies[0];
        b.force_acc += -b.mass * w.params->gravity;
    }
}
constexpr QuantityAccess kHoverAccess[] = {{"body.wrench", Access::accumulate}};
constexpr PassDecl kHoverPasses[] = {
    {.name = "lift", .phase = Phase::forces, .access = kHoverAccess, .cpu = &hover_pass}};

[[nodiscard]] spade::WorldSetDesc one_body_world() {
    auto world = spade::WorldBuilder()
                     .name("m")
                     .environment(spade::Environment{})
                     .capacities(spade::Capacities{1, 1, 1, 1})
                     .build();
    spade::WorldInstanceDesc inst;
    inst.world = *world;
    inst.turbulence = spade::dryden_params(spade::TurbulenceLevel::none);
    return spade::WorldSetDesc{{inst}};
}

}  // namespace

TEST(ModuleSimulation, TheDefaultSetIsTheStandardSet) {
    auto sim = spade::Simulation::create(one_body_world(), 2'000'000, 2);
    ASSERT_TRUE(sim.has_value()) << sim.error().context;
    const auto standard = compile_schedule(spade::modules::standard_modules());
    EXPECT_EQ(sim->schedule().identity, standard->identity);
}

TEST(ModuleSimulation, ADeveloperModulePlugsInOnTheCpu) {
    spade::modules::ModuleSet set = spade::modules::standard_modules();
    set.push_back({.name = "hover", .passes = kHoverPasses});
    auto held = spade::Simulation::create(one_body_world(), 2'000'000, 2, {}, set);
    auto falling = spade::Simulation::create(one_body_world(), 2'000'000, 2);
    ASSERT_TRUE(held.has_value()) << held.error().context;
    ASSERT_TRUE(falling.has_value()) << falling.error().context;
    spade::BodySpawn body;
    body.mass = 2.0f;
    const auto held_ref = held->spawn(0, body);
    const auto falling_ref = falling->spawn(0, body);
    ASSERT_TRUE(held_ref.has_value() && falling_ref.has_value());
    ASSERT_TRUE(held->step(10).has_value());
    ASSERT_TRUE(falling->step(10).has_value());
    const auto h = held->body(*held_ref);
    const auto f = falling->body(*falling_ref);
    ASSERT_TRUE(h.has_value() && f.has_value());
    EXPECT_EQ((*h)->vel, glm::vec3(0.0f)) << "the module's lift cancels gravity exactly";
    EXPECT_LT((*f)->vel.y, 0.0f) << "without the module the body falls";
}

TEST(ModuleSimulation, ScheduleNamesOutliveTheStringsThatNamedThem) {
    std::string name = "hover";
    spade::modules::ModuleSet set = spade::modules::standard_modules();
    set.push_back({.name = name, .passes = kHoverPasses});
    auto sim = spade::Simulation::create(one_body_world(), 2'000'000, 2, {}, set);
    ASSERT_TRUE(sim.has_value()) << sim.error().context;
    name.assign("xxxxx");  // a view into `name` would now read "xxxxx"
    bool found = false;
    for (const auto& pass : sim->schedule().passes) found = found || pass.module == "hover";
    EXPECT_TRUE(found) << "the compiled schedule must own its module names";
}

TEST(ModuleSimulation, ANonStandardSetOnVulkanIsRefusedUntilStage2) {
    spade::modules::ModuleSet set = spade::modules::standard_modules();
    set.push_back({.name = "hover", .passes = kHoverPasses});
    const auto sim = spade::Simulation::create(one_body_world(), 2'000'000, 2,
                                               spade::compute::BackendDesc{.kind = spade::compute::BackendKind::vulkan},
                                               set);
    ASSERT_FALSE(sim.has_value());
    EXPECT_EQ(sim.error().code, spade::Code::unavailable);
    EXPECT_NE(sim.error().context.find("module set"), std::string::npos) << sim.error().context;
}

TEST(ModuleSimulation, AnInvalidSetIsRefusedAtCreate) {
    static constexpr QuantityAccess misspelt[] = {{"body.wrnch", Access::accumulate}};
    static constexpr PassDecl bad[] = {{.name = "x", .phase = Phase::forces, .access = misspelt, .cpu = &noop}};
    spade::modules::ModuleSet set = spade::modules::standard_modules();
    set.push_back({.name = "bad", .passes = bad});
    const auto sim = spade::Simulation::create(one_body_world(), 2'000'000, 2, {}, set);
    ASSERT_FALSE(sim.has_value());
    EXPECT_EQ(sim.error().code, spade::Code::invalid_argument);
}
```

In `tests/test_determinism.cpp`, replace the two `Schedule.*` tests with one that pins the CPU order to the GPU recorder's slots. The recorder keeps its own table until stage 2:

```cpp
// Until plan stage 2 derives the GPU chain from the schedule, the recorder
// keeps its own slot table (compute/vulkan/step_recorder.cpp, kPassPipeline).
// This pins the compiled standard set, minus the CPU-only behavior passes, to
// that table's dispatch order.
TEST(Schedule, TheCompiledStandardSetFollowsTheGpuRecordersOrder) {
    const auto s = spade::modules::compile_schedule(spade::modules::standard_modules());
    ASSERT_TRUE(s.has_value()) << s.error().context;
    std::vector<std::string> cpu;
    for (const auto& p : s->passes) {
        if (p.module == "behaviors") continue;
        cpu.push_back(std::string(p.module) + "." + std::string(p.pass));
    }
    // MediumUpdate; ForceElements = rotors then drag; CollisionStatic;
    // CollisionDynamic; Integrate; SensorSynthesis = imu then gnss.
    const std::vector<std::string> gpu = {"dryden.advance",         "rotor.forces",       "drag.forces",
                                          "static_contact.resolve", "dynamic_contact.resolve",
                                          "integrate.integrate",    "imu.synthesize",     "gnss.synthesize"};
    EXPECT_EQ(cpu, gpu);
}
```

Add `#include "sim/module.hpp"` to `tests/test_determinism.cpp`.

- [ ] **Step 2: Build; expect a compile failure** (`schedule()` and the five-argument `create` are not declared).

Run: `scripts\build.ps1 -Target spade_tests`
Expected: FAIL to compile.

- [ ] **Step 3: Implement**

In `engine/sim/simulation.hpp`, add `#include "sim/module.hpp"`, then:
- change the `create` declaration to take `const modules::ModuleSet& modules = modules::standard_modules()` as its fifth parameter;
- add the accessor `[[nodiscard]] const modules::CompiledSchedule& schedule() const noexcept { return schedule_; }`;
- add the private member `modules::CompiledSchedule schedule_;`.

In `Simulation::create()` (`engine/sim/simulation.cpp`), right after the `dt_ns` and `substeps` argument checks and before any arena allocation:

```cpp
    Result<modules::CompiledSchedule> compiled = modules::compile_schedule(modules);
    if (!compiled) return std::unexpected(compiled.error());
    if (backend.kind == compute::BackendKind::vulkan) {
        // Plan stage 1: the GPU recorder still runs its own pass table, which
        // is the standard set's. Any other set would run a different
        // experiment on the GPU than on the CPU, silently (L6).
        static const uint64_t kStandardIdentity =
            modules::compile_schedule(modules::standard_modules())->identity;
        if (compiled->identity != kStandardIdentity) {
            return std::unexpected(Error{Code::unavailable,
                                         "a module set other than the standard set runs only on the CPU until "
                                         "the GPU chain is derived from the schedule (module-API plan, stage 2)"});
        }
    }
```

After the `Simulation` object is constructed in `create()`, store the result: `sim.schedule_ = std::move(*compiled);`.

In `Simulation::step()`'s CPU loop, replace `physics::run_substep(ctx);` with:

```cpp
            for (const modules::CompiledPass& pass : schedule_.passes) {
                pass.cpu(ctx);
            }
```

Update the comment above it that cites `physics::run_substep`, and the one that cites `pass_publish()`. The tick note should point at this loop instead.

In `engine/physics/schedule.{hpp,cpp}`, delete `Pass`, `kSchedule`, `kSubstepPassCount`, `substep_schedule()`, `run_substep()`, `pass_gravity`, `pass_publish`, `pass_force_elements` and `pass_sensor_synthesis`, together with their comments. Gravity stays applied inside Integrate (`engine A9`); say so in one line at `pass_integrate`'s declaration. Keep `PassFn`, `SubstepContext`, `WorldSubstepView` and the remaining `pass_*` functions.

- [ ] **Step 4: Build and run the focused tests**

Run: `scripts\build.ps1 -Target spade_tests`, then `scripts\test.ps1 -Filter "ModuleSchedule|StandardModules|ModuleSimulation|Schedule"`
Expected: all PASS.

- [ ] **Step 5: Run the full suite. Every golden must stay put.**

Run: `scripts\test.ps1`
Expected: 0 failed, and the same skips as master (the KAT agreement cases when run in a worktree). If any `ScenarioCorpus.*` or `GpuParity.*` test fails, stop: a digest or band moved, and the cause must be found before anything else.

- [ ] **Step 6: Commit**

```bash
git commit engine/sim/simulation.hpp engine/sim/simulation.cpp engine/physics/schedule.hpp engine/physics/schedule.cpp tests/test_module_schedule.cpp tests/test_determinism.cpp -m "feat(core): the CPU step runs the compiled module schedule"
```

### Task 4: Snapshots carry the configuration identity (`L2`)

**Files:**
- Modify: `engine/state/snapshot.hpp`, `engine/state/snapshot.cpp`
- Modify: `engine/sim/simulation.cpp` (`snapshot()`, `restore()`)
- Test: `tests/test_module_schedule.cpp`, `tests/test_snapshot.cpp`

**Interfaces:**
- Consumes: `Simulation::schedule()` (Task 3).
- Produces:
  - `SnapshotHeader::configuration_identity` (`uint64_t`, at offset 32; the header is now 40 bytes);
  - `kSnapshotVersion = 2`;
  - `Result<SnapshotBlob> save(const StateRegistry&, Tick, uint64_t configuration_identity = 0)`, and the same extra parameter on the `ArenaSet` overload;
  - `uint64_t SnapshotBlob::configuration_identity() const noexcept`.

- [ ] **Step 1: Write the failing tests** (append to `tests/test_module_schedule.cpp`)

```cpp
TEST(ModuleSnapshot, ABlobCarriesTheScheduleIdentity) {
    auto sim = spade::Simulation::create(one_body_world(), 2'000'000, 2);
    ASSERT_TRUE(sim.has_value()) << sim.error().context;
    const auto blob = sim->snapshot();
    ASSERT_TRUE(blob.has_value()) << blob.error().context;
    EXPECT_EQ(blob->configuration_identity(), sim->schedule().identity);
}

TEST(ModuleSnapshot, RestoreIntoAnotherModuleSetIsRefused) {
    spade::modules::ModuleSet with_hover = spade::modules::standard_modules();
    with_hover.push_back({.name = "hover", .passes = kHoverPasses});
    auto source = spade::Simulation::create(one_body_world(), 2'000'000, 2, {}, with_hover);
    auto target = spade::Simulation::create(one_body_world(), 2'000'000, 2);
    ASSERT_TRUE(source.has_value() && target.has_value());
    const auto blob = source->snapshot();
    ASSERT_TRUE(blob.has_value()) << blob.error().context;
    const auto restored = target->restore(*blob);
    ASSERT_FALSE(restored.has_value());
    EXPECT_EQ(restored.error().code, spade::Code::invalid_argument);
    EXPECT_NE(restored.error().context.find("module set"), std::string::npos) << restored.error().context;
}

TEST(ModuleSnapshot, RestoreUnderAnotherModuleVersionIsRefused) {
    spade::modules::ModuleSet a = spade::modules::standard_modules();
    spade::modules::ModuleSet b = spade::modules::standard_modules();
    a.push_back({.name = "hover", .version = 1, .passes = kHoverPasses});
    b.push_back({.name = "hover", .version = 2, .passes = kHoverPasses});
    auto source = spade::Simulation::create(one_body_world(), 2'000'000, 2, {}, a);
    auto target = spade::Simulation::create(one_body_world(), 2'000'000, 2, {}, b);
    ASSERT_TRUE(source.has_value() && target.has_value());
    const auto blob = source->snapshot();
    ASSERT_TRUE(blob.has_value());
    EXPECT_FALSE(target->restore(*blob).has_value());
}

TEST(ModuleSnapshot, RestoreIntoTheSameSetSucceeds) {
    auto source = spade::Simulation::create(one_body_world(), 2'000'000, 2);
    auto target = spade::Simulation::create(one_body_world(), 2'000'000, 2);
    ASSERT_TRUE(source.has_value() && target.has_value());
    ASSERT_TRUE(source->step(3).has_value());
    const auto blob = source->snapshot();
    ASSERT_TRUE(blob.has_value());
    EXPECT_TRUE(target->restore(*blob).has_value());
}
```

In `tests/test_snapshot.cpp`, add the round-trip of the new field at the state layer:

```cpp
// A blob written before the module API is format version 1. It is refused
// with the version message, never misread as a 40-byte header. (A v2 blob with
// its version field set back to 1 stands in for one.)
TEST(SnapshotFormat, AVersionOneBlobIsRefusedWithTheVersionMessage) {
    spade::StateRegistry registry;
    const auto blob = spade::save(registry, spade::Tick{1});
    ASSERT_TRUE(blob.has_value()) << blob.error().context;
    std::vector<std::byte> bytes(blob->bytes().begin(), blob->bytes().end());
    const uint32_t v1 = 1;
    std::memcpy(bytes.data() + offsetof(spade::SnapshotHeader, version), &v1, sizeof(v1));
    const auto reread = spade::SnapshotBlob::from_bytes(std::move(bytes));
    ASSERT_FALSE(reread.has_value());
    EXPECT_EQ(reread.error().code, spade::Code::schema_mismatch);
    EXPECT_NE(reread.error().context.find("format version 1"), std::string::npos) << reread.error().context;
}

TEST(SnapshotFormat, TheConfigurationIdentityRoundTripsThroughTheHeader) {
    spade::StateRegistry registry;
    const auto blob = spade::save(registry, spade::Tick{7}, 0x1234'5678'9ABC'DEF0ULL);
    ASSERT_TRUE(blob.has_value()) << blob.error().context;
    EXPECT_EQ(blob->version(), 2u);
    EXPECT_EQ(blob->configuration_identity(), 0x1234'5678'9ABC'DEF0ULL);
    const auto reread = spade::SnapshotBlob::from_bytes(
        std::vector<std::byte>(blob->bytes().begin(), blob->bytes().end()));
    ASSERT_TRUE(reread.has_value()) << reread.error().context;
    EXPECT_EQ(reread->configuration_identity(), 0x1234'5678'9ABC'DEF0ULL);
}
```

- [ ] **Step 2: Build; expect a compile failure** (`configuration_identity` is not declared).

- [ ] **Step 3: Implement**

In `engine/state/snapshot.hpp`:
- set `kSnapshotVersion` to `2`;
- append `uint64_t configuration_identity;` to `SnapshotHeader`, after `array_count`, with the comment `// v2: the module set and schedule that produced the state (Simulation's CompiledSchedule::identity); 0 for a bare registry save`;
- change the size assert to 40, add `static_assert(offsetof(SnapshotHeader, configuration_identity) == 32);`, and add the field to the no-implicit-padding sum;
- add `[[nodiscard]] uint64_t configuration_identity() const noexcept { return header().configuration_identity; }` to `SnapshotBlob`;
- add `uint64_t configuration_identity = 0` as a trailing parameter to both `save` overloads;
- update the header's doc comment to say the header is 40 bytes from version 2.

In `engine/state/snapshot.cpp`, pass the parameter through, and set `.configuration_identity = configuration_identity` in the header write. Check every place that assumes 32 bytes (`grep -n "32" engine/state/snapshot.cpp`) and use `sizeof(SnapshotHeader)` instead.

In `engine/sim/simulation.cpp`:
- `snapshot()` becomes `return save(arenas_, tick_, schedule_.identity);`;
- in `restore()`, immediately before `if (Result<void> config = check_replay_config(blob); !config)`, refuse a mismatch:

```cpp
    if (blob.configuration_identity() != schedule_.identity) {
        return std::unexpected(invalid("restore: blob was taken under a different module set or schedule "
                                       "(identity " + std::to_string(blob.configuration_identity()) +
                                       ", this simulation runs " + std::to_string(schedule_.identity) +
                                       "); restoring it here could replay different physics"));
    }
```

- [ ] **Step 4: Run the focused tests, then the full suite**

Run: `scripts\test.ps1 -Filter "ModuleSnapshot|SnapshotFormat|Snapshot"`, then `scripts\test.ps1`
Expected: all PASS, with every golden unchanged. The state digest does not read the header.

- [ ] **Step 5: Commit**

```bash
git commit engine/state/snapshot.hpp engine/state/snapshot.cpp engine/sim/simulation.cpp tests/test_module_schedule.cpp tests/test_snapshot.cpp -m "feat(core): snapshots carry the module set's identity, and restore refuses another (L2)" -m "Snapshot format v1 -> v2. A v1 blob is refused with the version message, so KAT's session ring cannot restore a blob written before this change; the lead updates consumers.md at merge."
```

### Task 5: Docs follow the code

**Files:**
- Modify: `docs/design/core/07-status.md` (main tree, after the merge)
- Modify: `docs/design/core/02-state-and-snapshot.md` (snapshot format v2)

- [ ] **Step 1:** In `02-state-and-snapshot.md`'s "Snapshot and restore" section, change the format line to read: the header carries the format version (2), schema hash, tick, world count, array count and the configuration identity (the module set and its compiled order); restore refuses another identity.
- [ ] **Step 2:** In `07-status.md`, change the "Scheduler phases" row to: **built for the CPU (plan stage 1)**, with passes from the standard module set compiled into six phases; the GPU recorder still keeps its own table until stage 2. Change the "Modules declare…" row to: **passes only** so far. Add the stage 1 merge commit. Under "Deferred on purpose", add: `replay_config` does not yet carry the configuration identity, which lives in the snapshot header (plan Ruling 1). It can move into `replay_config` with the next deliberate golden regeneration.
- [ ] **Step 3:** Tell the lead that the snapshot format is now v2, for `../../consumers.md` (Plan Ruling 1).
- [ ] **Step 4: Commit** (main tree, path-scoped): `git commit docs/design/core/02-state-and-snapshot.md docs/design/core/07-status.md -m "docs(core): status and snapshot format follow module-API stage 1"`

---

## Stages 2–6 (outlined; each gets its own plan when the stage before it merges)

### Stage 2: the GPU chain derived from the schedule
- **Goal:** `StepRecorder` walks the compiled schedule, and each pass with a GPU recording appends its dispatches. `kPassPipeline`, `kPassGrid` and the slot tally are deleted.
- **Files:** `sim/module.hpp` (`PassDecl::gpu`, a function that appends dispatches to the recorder's command list); `compute/vulkan/step_recorder.*`; `compute/backend.hpp` (`PassDurationsNs` becomes durations keyed by pass name); the bench's name mapping; `sim/standard_modules.cpp` (the built-ins' GPU recordings).
- **Refusals:** a pass with no GPU recording, on Vulkan, refuses `create()` unless its module declares it inert there. This replaces stage 1's whole-set refusal. Behaviors stay refused (`CORE-1`).
- **Done when:** every GPU parity band is unchanged; the recorded dispatch order equals the schedule's GPU passes; the `CORE-2` barrier test holds; `Schedule.TheCompiledStandardSetFollowsTheGpuRecordersOrder` is deleted, because nothing is left to follow.
- **Consumes:** `CompiledSchedule`, `PassDecl`.

### Stage 3: fields and sample buffers
- **Goal:** a field registry (`gravity`, `density`, `wind`); providers (environment constants, and Dryden for `wind`); per-world sample buffers written in Fields after the kinematic behaviors; readers call `sample(field, point)`.
- **Files:** `sim/fields.*` (new); `sim/module.hpp` (field declarations); `physics/schedule.cpp`, `vehicles/rotor.cpp`, `physics/forces.cpp` and `physics/integrator.cpp` read samples; a GPU sample kernel for Dryden; `Simulation::sample_medium` calls the provider's CPU sample function on the current state.
- **Done when:** every golden and band is unchanged; stored samples equal today's inline values bitwise; `sample_medium` before the first step returns the mean wind plus the initial gust.
- **Consumes:** stages 1–2.

### Stage 4: modules own their state (`CORE-4`)
- **Goal:** modules declare their arrays, body-attached rows (`init`/`free` run from the structural queue, and despawn frees them) and seeded streams (`reseed()` walks the declarations). The IMU/GNSS function pairs collapse. `SubstepContext` gains module views.
- **Files:** `sim/module.hpp`, `sim/simulation.*`, `sensors/*`, `vehicles/*`, `compute/vulkan/state_mirror.*` (array shapes come from the declarations, not a hand list).
- **Done when:** the walk is identical (`kWalkEntryCount` 22, `replay_config` at 16, through the legacy marker); every golden is unchanged; the IMU, GNSS and reseed suites pass through the generic path; the public `add_*`/`poll_*` calls keep their signatures.
- **Consumes:** stages 1–3.

### Stage 5: grades, roles, component availability
- **Goal:** each module declares a grade per backend (`absent` included, and a Vulkan `reference` is refused); `create()` takes an optional minimum grade; roles, starting with `dynamic_contact`; the component-type table gains availability, with `fluid` reserved ("SPH, `PHY-4`").
- **Done when:** the refusal tests of spec §13 pass (minimum grade, `absent` on Vulkan, a role filled twice or not at all, `L2` under another role choice); every golden is unchanged.
- **Consumes:** stages 1–4. Physics and Rendering supply the built-ins' grade declarations.

### Stage 6: the translation lock
- **Goal:** a built-in `lock` module (not in the standard set). One row per body slot holds the anchor and a per-axis mask. Integrate selects per locked axis after today's arithmetic. `lock_translation` and `unlock` are queued, and are refused with `unavailable` when the module is not in the set. On the GPU, a fixed binding, and Integrate records with the select off when the module is absent.
- **Done when:** the new held-quad golden passes; full-lock IMU reads `−Rᵀg` from a non-power-of-two mass; the y-free rail passes; the contact-against-a-locked-body test passes; Vulkan `pos` and `vel` are bit-exact; every existing golden is unchanged. Interface then replaces the drone box's behaviors and lifts its Vulkan refusal.
- **Consumes:** stages 1–5.
