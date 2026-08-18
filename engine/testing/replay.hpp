#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <functional>
#include <span>
#include <string>
#include <string_view>
#include <type_traits>
#include <utility>

#include "compute/backend.hpp"
#include "core/error.hpp"
#include "core/time.hpp"
#include "sim/simulation.hpp"
#include "state/arenas.hpp"
#include "state/registry.hpp"
#include "state/snapshot.hpp"

// ---------------------------------------------------------------------------
// THE DETERMINISM REPLAY HARNESS (charter P2/P4; engine design spec §4: "the
// determinism-replay corpus is snapshot pairs + input scripts").
//
// TEST SUPPORT, NOT ENGINE API. This header lives under engine/testing/ and is
// deliberately NOT installed, NOT exported, and NOT compiled into any shipped
// target: it is header-only and included by spade/tests/ only. It uses
// std::function and std::string freely for that reason -- it is allowed to be
// convenient in ways the engine is not.
//
// ---------------------------------------------------------------------------
// WHAT A DIGEST IS, EXACTLY
//
// A digest is a 64-bit FNV-1a over the whole of a Simulation's authoritative
// state, folded in the registry's walk order, with the tick folded in first. It
// exists because "did these two runs produce the same state?" needs a single
// comparable number that (a) covers everything, including the state a test
// author did not think to look at, and (b) is cheap enough to record for a
// scenario that runs a hundred thousand substeps.
//
// COVERAGE IS STRUCTURAL, NOT CURATED. The walk is StateRegistry's, and
// registration is the only way to obtain arena storage, so a digest covers
// every authoritative byte in the engine BY CONSTRUCTION. An array added by a
// future task joins the digest with no change to this file -- and changes every
// golden, which is the correct and intended alarm.
//
// THE MIXING FUNCTION IS state/snapshot.hpp's fnv1a64(), reused rather than
// re-derived, so a digest and a snapshot blob's schema hash are hashing bytes
// the same way. Nothing here defends against a crafted collision; the digest
// answers "did anything change", and when it says yes the blobs themselves are
// there to diff.
//
// LAYOUT AND SHAPE ARE FOLDED IN TOO -- each array's name (length prefixed, so
// no two name/payload boundaries can be confused), element size and per-world
// capacity precede its bytes. Two runs whose arrays hold the same values under
// different names or extents are not the same run.
//
// HOST BYTE ORDER, like the snapshot format itself and for the same reason: a
// digest is a same-host replay artifact, and the payloads it folds are raw
// std430 state images whose float layout is already a host/ABI fact.
//
// ---------------------------------------------------------------------------
// PER-WORLD DIGESTS, AND THE TWO NORMALIZATIONS THEY APPLY
//
// world_digest(sim, w) folds only world w's partition of each array. It is what
// makes BATCHING INVARIANCE checkable: "a world's trajectory does not depend on
// what else is in the world set, or on where in the set it sits".
//
// That claim forces exactly two normalizations, and both are worth being
// explicit about rather than hiding. Every per-world array's contents are
// independent of the world's INDEX -- WorldParams carries no index, BodyState
// carries none, DragBodyRow::body_slot is world-local, DrydenState is seeded
// from the world's own seed -- with two exceptions:
//
//   1. THE SLOT->WORLD MAPS store the world id itself, so world 2 of a
//      four-world set stores 2 where the same world run alone stores 0. That is
//      a fact about the world's ADDRESS, not about its state. A per-world
//      digest therefore folds those arrays' LIVENESS rather than their values:
//      each entry contributes 1 if the slot is allocated and 0 if it is free.
//      The information that matters (which slots hold bodies) is preserved
//      exactly; the information that is purely positional is dropped.
//
//   2. replay_config (sim/simulation.hpp, ticket M-1) is a WHOLE-SET identity
//      stored once per world: its config_hash covers the entire WorldSetDesc --
//      every world in it, and the world count. So the same world run alone and
//      run inside a four-world set legitimately hold DIFFERENT bytes there,
//      because they are in different sets, which is the one thing a batching-
//      invariance comparison must not be sensitive to. It is SKIPPED entirely
//      rather than normalized: unlike a slot->world map it carries no per-world
//      information at all, so there is nothing left to fold once the set-wide
//      part is removed. (Its slot->world map is NOT skipped -- the array is
//      direct-indexed, so that map is uniformly "free" in every world of every
//      set, and folds identically on both sides by construction.)
//
// state_digest(), which covers the whole set and has no such comparison to
// support, applies NEITHER normalization and folds the raw bytes of everything
// -- including replay_config, which is exactly how a golden digest comes to
// pin the configuration a scenario ran under.
//
// ---------------------------------------------------------------------------
// A SCENARIO is a world set builder plus a per-tick input script plus a step
// count. Nothing in it reads a clock, a file, or an environment variable, and
// the builder takes no arguments -- a scenario is a pure, reproducible
// description, which is what lets its digest be committed to a file.
// ---------------------------------------------------------------------------

namespace spade::testing {

namespace detail {

// Folds an object's byte representation. Used only for scalars and for spans of
// POD state, all of which are documented to have no implicit padding.
template <class T>
[[nodiscard]] inline uint64_t fold_value(uint64_t seed, const T& value) noexcept {
    static_assert(std::is_trivially_copyable_v<T>, "digest folds byte representations");
    std::byte bytes[sizeof(T)];
    std::memcpy(bytes, &value, sizeof(T));
    return fnv1a64(std::span<const std::byte>(bytes, sizeof(T)), seed);
}

[[nodiscard]] inline uint64_t fold_bytes(uint64_t seed, std::span<const std::byte> bytes) noexcept {
    return fnv1a64(bytes, seed);
}

[[nodiscard]] inline uint64_t fold_name(uint64_t seed, const std::string& name) noexcept {
    seed = fold_value(seed, static_cast<uint64_t>(name.size()));
    return fold_bytes(seed, std::as_bytes(std::span<const char>(name.data(), name.size())));
}

// True for the auxiliary maps ArenaSet registers alongside every array (see
// kSlotToWorldSuffix). These are the only per-world arrays whose CONTENTS
// encode the world index, hence the only ones a per-world digest normalizes.
[[nodiscard]] inline bool is_slot_to_world_map(const std::string& name) noexcept {
    if (name.size() < kSlotToWorldSuffix.size()) return false;
    return std::string_view(name).substr(name.size() - kSlotToWorldSuffix.size()) == kSlotToWorldSuffix;
}

}  // namespace detail

// ---------------------------------------------------------------------------
// The whole world set's state, at the current tick.
// ---------------------------------------------------------------------------
[[nodiscard]] inline uint64_t state_digest(const Simulation& sim) noexcept {
    uint64_t seed = kFnv1a64Offset;
    seed = detail::fold_value(seed, sim.tick().value);
    sim.arenas().registry().for_each_array([&seed](const RegisteredArray& array) {
        seed = detail::fold_name(seed, array.name);
        seed = detail::fold_value(seed, array.elem_size);
        seed = detail::fold_value(seed, array.world_count);
        seed = detail::fold_value(seed, array.capacity_per_world);
        seed = detail::fold_bytes(seed, std::span<const std::byte>(array.data, array.byte_size()));
    });
    return seed;
}

// ---------------------------------------------------------------------------
// One world's state, comparable across differently-shaped world sets. See the
// normalization note in the header comment.
//
// The tick is folded (two runs of the same world at different ticks are
// different), the world INDEX is not (that is the whole point).
// ---------------------------------------------------------------------------
[[nodiscard]] inline uint64_t world_digest(const Simulation& sim, uint32_t world) noexcept {
    uint64_t seed = kFnv1a64Offset;
    seed = detail::fold_value(seed, sim.tick().value);
    sim.arenas().registry().for_each_array([&seed, world](const RegisteredArray& array) {
        if (world >= array.world_count) return;
        // Normalization 2: a whole-set identity is not this world's state. See
        // the header note -- folding it would make every batching-invariance
        // comparison in the suite fail for a reason that has nothing to do with
        // the physics they exist to check.
        if (array.name == kReplayConfigArray) return;
        seed = detail::fold_name(seed, array.name);
        seed = detail::fold_value(seed, array.elem_size);
        seed = detail::fold_value(seed, array.capacity_per_world);

        const std::size_t stride = array.elem_size;
        const std::size_t begin = static_cast<std::size_t>(world) * array.capacity_per_world * stride;
        const std::size_t length = static_cast<std::size_t>(array.capacity_per_world) * stride;

        if (detail::is_slot_to_world_map(array.name)) {
            // Liveness, not the world id -- the one positional value in the
            // state. Folded as one byte per slot so the digest still changes
            // when a slot's occupancy changes.
            const uint32_t* entries =
                reinterpret_cast<const uint32_t*>(array.data + begin);  // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)
            for (uint32_t i = 0; i < array.capacity_per_world; ++i) {
                const uint8_t live = entries[i] == kInvalidWorld ? uint8_t{0} : uint8_t{1};
                seed = detail::fold_value(seed, live);
            }
            return;
        }

        seed = detail::fold_bytes(seed, std::span<const std::byte>(array.data + begin, length));
    });
    return seed;
}

// ---------------------------------------------------------------------------
// A scenario: a reproducible description of a run.
//
// `build` constructs the world set -- from either of two producers,
// depending on how the Scenario was made. A hand-built Scenario (this file's
// own callers in tests/test_determinism.cpp and tests/test_m1b_bar.cpp)
// builds its worlds with WorldBuilder in code, no world file involved. A
// Scenario loaded from the golden corpus (testing/scenario_file.hpp's
// load_scenario_file(), S5 Task 7) instead builds `build` as a closure over
// worlds already resolved from a world file via load_world_file() -- the
// corpus scenario names the world file, load_scenario_file() reads it once at
// load time, and `build` just returns the result. Either way this struct only
// ever sees the finished WorldSetDesc; it has no opinion on which producer
// built it. `setup` runs once, after create(), for the initial spawns.
// `input` runs once per step, BEFORE that
// step, and is handed the tick that is about to be executed -- which is what
// makes an input script REPLAYABLE from an arbitrary resume point: replaying
// ticks [k, N) applies exactly the inputs the uninterrupted run applied there.
//
// Both hooks may be empty.
// ---------------------------------------------------------------------------
struct Scenario {
    std::string name;
    uint64_t dt_ns = 1'000'000;  // 1 ms step
    uint32_t substeps = 1;
    uint64_t steps = 0;

    std::function<Result<WorldSetDesc>()> build;
    std::function<Result<void>(Simulation&)> setup;
    std::function<Result<void>(Simulation&, Tick)> input;
};

// Creates the Simulation for a scenario and runs its setup, leaving it at tick
// 0 with the structural queue flushed (so the state is snapshot-legal and
// digest-complete before the first step).
//
// `backend` (S6 Task 5) -- DEFAULTED, threading Simulation::create()'s own
// new trailing parameter through unchanged for every pre-existing caller.
// Its one reason to exist: the A7 cpu-leg backend-knob invariance test
// (tests/test_gpu_state_mirror.cpp) needs to run the SAME scenario twice,
// once under the implicit backend={} every other caller in this tree still
// uses and once under an EXPLICIT BackendDesc{.kind = BackendKind::cpu}, and
// compare digests -- which is impossible without a way to pass one in.
[[nodiscard]] inline Result<Simulation> start_scenario(const Scenario& scenario,
                                                        const compute::BackendDesc& backend = {}) {
    Result<WorldSetDesc> desc = scenario.build();
    if (!desc) return std::unexpected(desc.error());

    Result<Simulation> sim = Simulation::create(*desc, scenario.dt_ns, scenario.substeps, backend);
    if (!sim) return std::unexpected(sim.error());

    if (scenario.setup) {
        if (Result<void> r = scenario.setup(*sim); !r) return std::unexpected(r.error());
    }
    if (Result<void> r = sim->flush_structural(); !r) return std::unexpected(r.error());
    return sim;
}

// Advances `sim` from its current tick to `until_tick`, applying the scenario's
// input script before each step. This is the resume primitive: calling it on a
// freshly restored Simulation is exactly what the replay guarantee tests.
[[nodiscard]] inline Result<void> advance_scenario(const Scenario& scenario, Simulation& sim,
                                                   uint64_t until_tick) {
    while (sim.tick().value < until_tick) {
        if (scenario.input) {
            if (Result<void> r = scenario.input(sim, sim.tick()); !r) return r;
        }
        if (Result<void> r = sim.step(1); !r) return r;
    }
    return {};
}

// Runs a scenario end to end and returns its whole-set digest. `backend`:
// see start_scenario()'s doc comment.
[[nodiscard]] inline Result<uint64_t> run_scenario(const Scenario& scenario,
                                                    const compute::BackendDesc& backend = {}) {
    Result<Simulation> sim = start_scenario(scenario, backend);
    if (!sim) return std::unexpected(sim.error());
    if (Result<void> r = advance_scenario(scenario, *sim, scenario.steps); !r) {
        return std::unexpected(r.error());
    }
    return state_digest(*sim);
}

}  // namespace spade::testing
