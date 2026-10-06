# Airflow Solver Core Implementation Plan — the CPU reference and V1

> **DRAFT IN PROGRESS** (2026-10-05): Tasks 1–8 written; Tasks 9–12 (the actuator source, the reference data, V1's validation, the docs) and the self-review follow. Not yet sent to the lead.

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** The real-time tier's incompressible Navier–Stokes solver as a standalone CPU reference library, validated on V1's cases (the lid-driven cavity against Ghia et al., the Taylor–Green vortex as the numerics' own dissipation, and an actuator disc's far wake against momentum theory).

**Architecture:**
- A library of pure functions in `engine/physics/airflow/`, in fp32, over a MAC grid: boundaries, the divergence and pressure operator, a geometric multigrid V-cycle, the projection, semi-Lagrangian MacCormack advection, implicit viscosity with no-slip walls, the fluid step that composes them, and the actuator-disc source.
- It includes nothing from `sim/` and touches no engine state: no module, no arena, no GPU. It is the CPU reference the GPU twin mirrors expression for expression (plan B), and the solver the `airflow` module will wrap once Core's stage 7 exists (plan C).
- Every loop has a fixed order, every Gauss–Seidel sweep is red–black (order-free within a colour), and every scatter is written as a gather, so the GPU twin can reproduce each per-cell expression.

**Tech Stack:** C++23, glm, GoogleTest through ctest, MSVC + Ninja presets, the gcc-13 Docker leg.

**Spec:** `2026-10-05-airflow-design.md`: §1.5 (the discretisation), §2 (boundaries, including the wall and periodic faces), §3.1 (the actuator source), §4.5 (determinism), §4.6 (tests and gate time) and §11 (fidelity and validation, V1). Decision: `PHY-9` (`../00-decisions.md`).

**What this plan is, and is not.** The spec covers several subsystems; this plan is the first: the solver core on the CPU and V1. It produces working, tested software on its own. Two plans follow, and neither blocks this one:
- **Plan B, the GPU twin and the spike:** Slang kernels for each operator here, banded against this reference per kernel (`TD-14`), and the timings at 64³ and 128³ (spec §9, M1). It needs a persistent-buffer kernel chain, which is Core's machinery, so it is written with Core once Task 5 here has fixed the projection's expressions. It may run in parallel with Tasks 6–11.
- **Plan C, the `airflow` module:** the engine integration (point samples, regions, the recordings, device residency) waits on Core's stage 7, which follows stages 5 and 6 (`PHY-9`).

## Global Constraints

- **fp32 throughout**, in the order the code states; no reassociation. The project's existing compiler flags (no fused multiply-add contraction); add none.
- **No libm transcendental in `engine/`** (`TD-3`): `std::sqrt` is allowed (correctly rounded on both toolchains); `std::floor`, `std::isfinite`, `std::min`/`max` are exact. Test files that are not golden-feeding (they do not use `SPADE_GOLDEN_DIR`) may compute reference values in `double` with `std::sin` and the like; the BitPortability scan reads only engine sources and golden-feeding tests.
- **Reciprocals on the host** (spec §4.5): `1/dx`, `h/(ρ dx)`, `ρ dx²/h` and the like are computed once per call and multiplied, never divided per cell, except where a formula below divides on purpose (the Gauss–Seidel update's division by the diagonal, which the GPU twin repeats).
- **Determinism:** loops run `k` outermost, `i` innermost; a stencil's six faces are visited in the order −x, +x, −y, +y, −z, +z; sums accumulate in that order. A periodic axis has an even size or size 1, so red–black colouring stays order-free. No unordered container, no atomics, no reduction feeds back into stepping.
- **No allocation in a step.** Everything `fluid_step` touches is sized by `make_grid` and `make_fluid_scratch`.
- **Refuse, never skip** (`L6`): setup functions return `Result<T>` and refuse by name (`invalid_argument`, or `capacity_exceeded` for size). Per-step functions are `noexcept` and take checked inputs.
- **Layering:** `engine/physics/airflow/` includes only `core/` headers, glm and the standard library.
- `-Wall -Wextra -Wpedantic` clean under gcc, warning-free under MSVC.
- **Reference data** for each validation case is committed with its provenance (the source, the table or figure, the transcription or digitization method) and reviewed by Test/Docs like a golden **before** the case's first run (spec §11.1; the lead, 2026-10-05). A tolerance is fixed in this plan, before the first run, and changes only with a named cause.
- **Where the code lives:** `../spade-wt/physics`, branch `physics/airflow-core`, cut from master when the lead approves this plan.
- **The task gate**, at the end of every task, all in the foreground:
  1. `scripts\build.ps1 -Preset release -ParallelLevel 8` and `-Preset debug -ParallelLevel 8`;
  2. `scripts\test.ps1 -Preset release` and `-Preset debug`: 0 failed, skips named, counts reported with tree and commit (`TD-7`, `TD-8`);
  3. `bash scripts/gcc-check.sh <every changed C++ file>`: all `gcc ok`;
  4. Tasks 8 and 12 also run `scripts\docker-leg.ps1 -Memory 8g -Jobs 8` at the task's head (Task 8 adds a golden: a fresh `-NoSeed` leg, `TD-12`).
- **Running one suite during a task:** `scripts\build.ps1 -Preset release -Target spade_tests -ParallelLevel 8`, then `build-ninja\release\bin\spade_tests.exe --gtest_filter=Suite.*`.

## Review Focus

The input classes the spec implies that no case below would otherwise meet, most likely first. Each has its test in the task that owns the code.

1. **A velocity that sends a backtrace far outside the grid** (a gust spike, a bad command): sampling clamps or wraps, and the step stays finite; no `NaN` index cast. Test: Task 6, `AirflowAdvect.AVelocityFarPastTheGridStaysFinite`.
2. **An actuator disc whose kernel support crosses the grid's edge** (a vehicle near the box wall): refused by name at setup, never silently short of momentum. Test: Task 9, `AirflowActuator.ADiscWhoseSupportLeavesTheGridIsRefused`.
3. **A box closed on every side** (an enclosed room): the pressure equation is singular (Neumann everywhere); the solve must stay finite and the divergence must still fall. Test: Task 5, `AirflowProjection.AClosedBoxStaysFiniteAndItsDivergenceFalls`.
4. **The ambient turning around between steps** (a gust reversing): an open face changes from inflow to outlet, and the next projection must treat it as an outlet. Test: Task 5, `AirflowProjection.AFaceFollowsTheAmbientEachStep`.
5. **A grid one cell wide on an axis** (a quasi-2D slab with walls, not periodic): boundary copies read no index past the array. Test: Task 2, `AirflowBoundaries.AOneCellWideAxisReadsNothingOutOfRange`.

---

### Task 1: The grid and its domain configuration

**Files:**
- Create: `engine/physics/airflow/mac_grid.hpp`, `engine/physics/airflow/mac_grid.cpp`
- Modify: `engine/CMakeLists.txt` (the `spade_physics` source list, after `physics/forces.cpp`)
- Create: `tests/test_airflow_grid.cpp`
- Modify: `tests/CMakeLists.txt` (the main source list, after `test_forces.cpp`)

**Interfaces:**
- Produces: `spade::physics::airflow::{Side, FaceBc, DomainBc, GridShape, MacGrid}`, `u_index`, `v_index`, `w_index`, `cell_index`, `make_grid(const GridShape&, const DomainBc&) -> Result<MacGrid>`, and `for_each_sample(const GridShape&, uint32_t component, F)`, which every later task uses.

- [ ] **Step 1: Write the failing tests**

`tests/test_airflow_grid.cpp`:

```cpp
// The airflow solver core's grid (physics plan 2026-10-05-airflow-solver-core-plan.md,
// Task 1): a MAC grid, its staggered arrays and the domain faces' configuration.
#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <vector>

#include "physics/airflow/mac_grid.hpp"

namespace {

using spade::Code;
using spade::physics::airflow::DomainBc;
using spade::physics::airflow::FaceBc;
using spade::physics::airflow::GridShape;
using spade::physics::airflow::make_grid;

TEST(AirflowGrid, SizesItsArraysByTheStaggeredLayout) {
    const auto g = make_grid(GridShape{4, 3, 2, 0.1f}, DomainBc{});
    ASSERT_TRUE(g.has_value()) << g.error().context;
    EXPECT_EQ(g->u.size(), 5u * 3u * 2u) << "x-faces: (nx + 1) ny nz";
    EXPECT_EQ(g->v.size(), 4u * 4u * 2u) << "y-faces: nx (ny + 1) nz";
    EXPECT_EQ(g->w.size(), 4u * 3u * 3u) << "z-faces: nx ny (nz + 1)";
    EXPECT_EQ(g->p.size(), 24u);
    for (float x : g->u) EXPECT_EQ(x, 0.0f);
    for (float x : g->p) EXPECT_EQ(x, 0.0f);
}

TEST(AirflowGrid, IndexesEveryArrayWithIInnermost) {
    const GridShape s{4, 3, 2, 0.1f};
    using namespace spade::physics::airflow;
    EXPECT_EQ(u_index(s, 4, 2, 1), 4u + 5u * (2u + 3u * 1u));
    EXPECT_EQ(v_index(s, 3, 3, 1), 3u + 4u * (3u + 4u * 1u));
    EXPECT_EQ(w_index(s, 3, 2, 2), 3u + 4u * (2u + 3u * 2u));
    EXPECT_EQ(cell_index(s, 3, 2, 1), 3u + 4u * (2u + 3u * 1u));
}

TEST(AirflowGrid, VisitsEachComponentsSamplesAtTheirPositions) {
    const GridShape s{2, 1, 1, 0.5f};
    std::vector<float> xs;
    spade::physics::airflow::for_each_sample(s, 0, [&](std::size_t idx, glm::vec3 x) {
        EXPECT_EQ(idx, xs.size());
        xs.push_back(x.x);
        EXPECT_EQ(x.y, 0.25f) << "a u-face sits mid-cell in y";
        EXPECT_EQ(x.z, 0.25f);
    });
    EXPECT_EQ(xs, (std::vector<float>{0.0f, 0.5f, 1.0f})) << "u-faces at i dx, i = 0..nx";
}

TEST(AirflowGrid, RefusesAShapeItCannotStep) {
    const auto refused = [](GridShape s, DomainBc bc, const char* what) {
        const auto g = make_grid(s, bc);
        if (g.has_value()) return testing::AssertionFailure() << "accepted";
        if (g.error().code != Code::invalid_argument) return testing::AssertionFailure() << "code";
        if (g.error().context.find(what) == std::string::npos) {
            return testing::AssertionFailure() << g.error().context;
        }
        return testing::AssertionSuccess();
    };
    EXPECT_TRUE(refused({0, 4, 4, 0.1f}, {}, "dimension"));
    EXPECT_TRUE(refused({4, 4, 4, 0.0f}, {}, "dx"));
    EXPECT_TRUE(refused({4, 4, 4, std::numeric_limits<float>::quiet_NaN()}, {}, "dx"));
    DomainBc unpaired;
    unpaired.face[0] = FaceBc::periodic;
    EXPECT_TRUE(refused({4, 4, 4, 0.1f}, unpaired, "x+"));
    DomainBc odd;
    odd.face[0] = odd.face[1] = FaceBc::periodic;
    EXPECT_TRUE(refused({3, 4, 4, 0.1f}, odd, "even"));
    DomainBc slab;
    slab.face[4] = slab.face[5] = FaceBc::periodic;
    EXPECT_TRUE(make_grid({4, 4, 1, 0.1f}, slab).has_value()) << "a periodic axis of size 1 is a quasi-2D slab";
}

}  // namespace
```

- [ ] **Step 2: Register the test and confirm it fails to compile**

Add `test_airflow_grid.cpp` after `test_forces.cpp` in `tests/CMakeLists.txt`'s main list. Build: `scripts\build.ps1 -Preset release -Target spade_tests -ParallelLevel 8`.
Expected: FAIL, `physics/airflow/mac_grid.hpp: No such file or directory`.

- [ ] **Step 3: Write the header**

`engine/physics/airflow/mac_grid.hpp`:

```cpp
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

#include <glm/glm.hpp>

#include "core/error.hpp"

// ---------------------------------------------------------------------------
// THE AIRFLOW SOLVER'S GRID (physics/plans/2026-10-05-airflow-design.md §1.5,
// §2). A MAC (staggered) grid of cubic cells: each velocity component lives on
// the faces normal to its axis, the pressure at the cell centres. A domain is
// [0, nx dx] x [0, ny dx] x [0, nz dx], in metres, in the grid's own frame.
//
// The six domain faces are indexed 2 * axis + (0 for the minus face, 1 for the
// plus face) everywhere in this library.
// ---------------------------------------------------------------------------
namespace spade::physics::airflow {

enum Side : uint32_t { kXMinus = 0, kXPlus = 1, kYMinus = 2, kYPlus = 3, kZMinus = 4, kZPlus = 5 };

enum class FaceBc : uint8_t {
    open,      // the far field: an inflow where the ambient enters, otherwise a pressure outlet (p = 0)
    wall,      // no slip, at the face's wall velocity (a moving lid is a wall with a tangential velocity)
    periodic,  // wraps to the opposite face; both faces of an axis say so, or neither
};

struct DomainBc {
    std::array<FaceBc, 6> face{FaceBc::open, FaceBc::open, FaceBc::open,
                               FaceBc::open, FaceBc::open, FaceBc::open};
    std::array<glm::vec3, 6> wall_velocity{};  // per face, read where face[s] == wall, m/s
    glm::vec3 ambient{0.0f};                   // the far-field velocity at open faces, m/s
};

struct GridShape {
    uint32_t nx = 0;
    uint32_t ny = 0;
    uint32_t nz = 0;
    float dx = 0.0f;  // the cell size, metres
};

struct MacGrid {
    GridShape shape{};
    std::vector<float> u;  // x-faces, (nx + 1) ny nz, at (i dx, (j + 1/2) dx, (k + 1/2) dx)
    std::vector<float> v;  // y-faces, nx (ny + 1) nz
    std::vector<float> w;  // z-faces, nx ny (nz + 1)
    std::vector<float> p;  // cells, nx ny nz: gauge pressure, Pa
};

[[nodiscard]] inline std::size_t u_index(const GridShape& g, uint32_t i, uint32_t j, uint32_t k) noexcept {
    return i + std::size_t{g.nx + 1u} * (j + std::size_t{g.ny} * k);
}
[[nodiscard]] inline std::size_t v_index(const GridShape& g, uint32_t i, uint32_t j, uint32_t k) noexcept {
    return i + std::size_t{g.nx} * (j + std::size_t{g.ny + 1u} * k);
}
[[nodiscard]] inline std::size_t w_index(const GridShape& g, uint32_t i, uint32_t j, uint32_t k) noexcept {
    return i + std::size_t{g.nx} * (j + std::size_t{g.ny} * k);
}
[[nodiscard]] inline std::size_t cell_index(const GridShape& g, uint32_t i, uint32_t j, uint32_t k) noexcept {
    return i + std::size_t{g.nx} * (j + std::size_t{g.ny} * k);
}

// Every sample of velocity component `component` (0 u, 1 v, 2 w), in array
// order (k outermost, i innermost), with its position: on its own axis at
// index * dx, on the other two at (index + 1/2) dx.
template <class F>
void for_each_sample(const GridShape& s, uint32_t component, F&& f) {
    const uint32_t cx = component == 0 ? s.nx + 1u : s.nx;
    const uint32_t cy = component == 1 ? s.ny + 1u : s.ny;
    const uint32_t cz = component == 2 ? s.nz + 1u : s.nz;
    const float ox = component == 0 ? 0.0f : 0.5f;
    const float oy = component == 1 ? 0.0f : 0.5f;
    const float oz = component == 2 ? 0.0f : 0.5f;
    std::size_t idx = 0;
    for (uint32_t k = 0; k < cz; ++k) {
        for (uint32_t j = 0; j < cy; ++j) {
            for (uint32_t i = 0; i < cx; ++i, ++idx) {
                f(idx, glm::vec3((static_cast<float>(i) + ox) * s.dx, (static_cast<float>(j) + oy) * s.dx,
                                 (static_cast<float>(k) + oz) * s.dx));
            }
        }
    }
}

// A grid at rest: every face and cell 0. Refused (invalid_argument, naming
// what): a zero dimension; dx not positive and finite; a periodic face whose
// opposite face is not periodic; a periodic axis of odd size other than 1,
// since red-black colouring needs an even ring. capacity_exceeded past 2^31
// cells.
[[nodiscard]] Result<MacGrid> make_grid(const GridShape& shape, const DomainBc& bc);

}  // namespace spade::physics::airflow
```

- [ ] **Step 4: Write the source**

`engine/physics/airflow/mac_grid.cpp`:

```cpp
#include "physics/airflow/mac_grid.hpp"

#include <cmath>
#include <string>

namespace spade::physics::airflow {

namespace {
constexpr const char* kSideNames[6] = {"x-", "x+", "y-", "y+", "z-", "z+"};
}  // namespace

Result<MacGrid> make_grid(const GridShape& shape, const DomainBc& bc) {
    if (shape.nx == 0 || shape.ny == 0 || shape.nz == 0) {
        return std::unexpected(Error{Code::invalid_argument, "airflow grid: every dimension must be at least 1"});
    }
    if (!(shape.dx > 0.0f) || !std::isfinite(shape.dx)) {
        return std::unexpected(Error{Code::invalid_argument, "airflow grid: dx must be positive and finite"});
    }
    if (uint64_t{shape.nx} * shape.ny * shape.nz > (uint64_t{1} << 31)) {
        return std::unexpected(Error{Code::capacity_exceeded, "airflow grid: more than 2^31 cells"});
    }
    const uint32_t size[3] = {shape.nx, shape.ny, shape.nz};
    for (uint32_t axis = 0; axis < 3; ++axis) {
        const bool lo = bc.face[2 * axis] == FaceBc::periodic;
        const bool hi = bc.face[2 * axis + 1] == FaceBc::periodic;
        if (lo != hi) {
            return std::unexpected(Error{Code::invalid_argument,
                                         std::string("airflow grid: face ") + kSideNames[2 * axis + (lo ? 1 : 0)] +
                                             " must be periodic like its opposite face"});
        }
        if (lo && size[axis] != 1 && size[axis] % 2 != 0) {
            return std::unexpected(Error{Code::invalid_argument,
                                         std::string("airflow grid: periodic axis ") + "xyz"[axis] +
                                             " must have an even size or size 1 (red-black colouring)"});
        }
    }
    MacGrid g;
    g.shape = shape;
    g.u.assign(std::size_t{shape.nx + 1u} * shape.ny * shape.nz, 0.0f);
    g.v.assign(std::size_t{shape.nx} * (shape.ny + 1u) * shape.nz, 0.0f);
    g.w.assign(std::size_t{shape.nx} * shape.ny * (shape.nz + 1u), 0.0f);
    g.p.assign(std::size_t{shape.nx} * shape.ny * shape.nz, 0.0f);
    return g;
}

}  // namespace spade::physics::airflow
```

Add `physics/airflow/mac_grid.cpp` after `physics/forces.cpp` in `engine/CMakeLists.txt`'s `add_library(spade_physics STATIC ...)`.

- [ ] **Step 5: Run the tests and confirm they pass**

Run: `build-ninja\release\bin\spade_tests.exe --gtest_filter=AirflowGrid.*`
Expected: 4 tests PASS.

- [ ] **Step 6: Commit**

```bash
git add engine/physics/airflow/mac_grid.hpp engine/physics/airflow/mac_grid.cpp engine/CMakeLists.txt tests/test_airflow_grid.cpp tests/CMakeLists.txt
git commit -m "feat(physics): the airflow solver's MAC grid and domain configuration (airflow core, Task 1)"
```

(Every commit message in this plan ends with the two attribution lines the session's system reminder gives.)

---

### Task 2: Velocity boundaries

**Files:**
- Create: `engine/physics/airflow/boundaries.hpp`, `engine/physics/airflow/boundaries.cpp`
- Modify: `engine/CMakeLists.txt` (after `physics/airflow/mac_grid.cpp`)
- Modify: `tests/test_airflow_grid.cpp` (a new suite, `AirflowBoundaries`)

**Interfaces:**
- Consumes: Task 1's types.
- Produces: `enum class FaceKind { fixed_velocity, pressure_outlet, periodic }`, `face_kind(const DomainBc&, Side) -> FaceKind`, `face_kinds(const DomainBc&) -> std::array<FaceKind, 6>`, `fixed_normal_velocity(const DomainBc&, Side) -> float`, `apply_velocity_boundaries(MacGrid&, const DomainBc&) noexcept`.

- [ ] **Step 1: Write the failing tests**

Append to `tests/test_airflow_grid.cpp`, inside the anonymous namespace, with `#include "physics/airflow/boundaries.hpp"` added at the top:

```cpp
using spade::physics::airflow::FaceKind;
using spade::physics::airflow::MacGrid;

[[nodiscard]] MacGrid filled(spade::physics::airflow::GridShape s, const DomainBc& bc, float value) {
    MacGrid g = *make_grid(s, bc);
    for (float& x : g.u) x = value;
    for (float& x : g.v) x = value;
    for (float& x : g.w) x = value;
    return g;
}

TEST(AirflowBoundaries, AWallSetsItsNormalVelocity) {
    DomainBc bc;
    bc.face.fill(FaceBc::wall);
    bc.wall_velocity[spade::physics::airflow::kYPlus] = glm::vec3(1.0f, 0.0f, 0.0f);  // a lid: tangential only
    bc.wall_velocity[spade::physics::airflow::kXMinus] = glm::vec3(0.25f, 0.0f, 0.0f);  // a piston
    MacGrid g = filled({4, 4, 4, 0.1f}, bc, 7.0f);
    spade::physics::airflow::apply_velocity_boundaries(g, bc);
    const auto& s = g.shape;
    using namespace spade::physics::airflow;
    for (uint32_t k = 0; k < 4; ++k) {
        for (uint32_t i = 0; i < 4; ++i) {
            EXPECT_EQ(g.v[v_index(s, i, 4, k)], 0.0f) << "the lid moves along x, so its normal velocity is 0";
            EXPECT_EQ(g.v[v_index(s, i, 0, k)], 0.0f);
            EXPECT_EQ(g.v[v_index(s, i, 2, k)], 7.0f) << "an interior face is untouched";
        }
        for (uint32_t j = 0; j < 4; ++j) EXPECT_EQ(g.u[u_index(s, 0, j, k)], 0.25f) << "the piston's normal velocity";
    }
}

TEST(AirflowBoundaries, TheAmbientEntersThroughAnOpenFaceAndLeavesThroughAnOutlet) {
    DomainBc bc;
    bc.ambient = glm::vec3(2.0f, 0.0f, 0.0f);
    EXPECT_EQ(face_kind(bc, spade::physics::airflow::kXMinus), FaceKind::fixed_velocity) << "inflow";
    EXPECT_EQ(face_kind(bc, spade::physics::airflow::kXPlus), FaceKind::pressure_outlet);
    EXPECT_EQ(face_kind(bc, spade::physics::airflow::kYMinus), FaceKind::pressure_outlet) << "still across it";
    MacGrid g = filled({4, 2, 2, 0.1f}, bc, 3.0f);
    using namespace spade::physics::airflow;
    g.u[u_index(g.shape, 3, 1, 1)] = 5.0f;
    apply_velocity_boundaries(g, bc);
    EXPECT_EQ(g.u[u_index(g.shape, 0, 1, 1)], 2.0f) << "the inflow takes the ambient";
    EXPECT_EQ(g.u[u_index(g.shape, 4, 1, 1)], 5.0f) << "the outlet copies its neighbour";
}

TEST(AirflowBoundaries, AnOutletRefusesBackflow) {
    DomainBc bc;  // still air: every open face is an outlet
    MacGrid g = filled({4, 2, 2, 0.1f}, bc, 0.0f);
    using namespace spade::physics::airflow;
    g.u[u_index(g.shape, 1, 0, 0)] = 1.5f;   // flowing into the box through x-
    g.u[u_index(g.shape, 3, 0, 0)] = -1.5f;  // flowing into the box through x+
    apply_velocity_boundaries(g, bc);
    EXPECT_EQ(g.u[u_index(g.shape, 0, 0, 0)], 0.0f) << "backflow takes the ambient's normal velocity";
    EXPECT_EQ(g.u[u_index(g.shape, 4, 0, 0)], 0.0f);
}

TEST(AirflowBoundaries, APeriodicAxisCopiesItsFirstFaceToItsLast) {
    DomainBc bc;
    bc.face.fill(FaceBc::periodic);
    MacGrid g = filled({4, 2, 2, 0.1f}, bc, 0.0f);
    using namespace spade::physics::airflow;
    g.u[u_index(g.shape, 0, 1, 1)] = 4.0f;
    g.u[u_index(g.shape, 4, 1, 1)] = -9.0f;
    apply_velocity_boundaries(g, bc);
    EXPECT_EQ(g.u[u_index(g.shape, 4, 1, 1)], 4.0f);
}

// Review Focus 5. With nx = 1 the x- outlet reads face 1 and the x+ outlet
// reads face nx - 1 = 0: both in range. Still air, so both are outlets.
TEST(AirflowBoundaries, AOneCellWideAxisReadsNothingOutOfRange) {
    DomainBc bc;
    MacGrid g = filled({1, 3, 1, 0.1f}, bc, 0.0f);
    using namespace spade::physics::airflow;
    ASSERT_EQ(g.u.size(), 2u * 3u * 1u);
    g.u[u_index(g.shape, 1, 2, 0)] = 0.5f;
    apply_velocity_boundaries(g, bc);
    // x- copies face 1 (0.5): flow in +x at the minus face enters the box,
    // so it is refused and takes the ambient's 0. x+ then copies face 0 (0).
    EXPECT_EQ(g.u[u_index(g.shape, 0, 2, 0)], 0.0f);
    EXPECT_EQ(g.u[u_index(g.shape, 1, 2, 0)], 0.0f);
}
```

- [ ] **Step 2: Run them and confirm they fail to compile**

Expected: FAIL, `physics/airflow/boundaries.hpp: No such file or directory`.

- [ ] **Step 3: Write the header**

`engine/physics/airflow/boundaries.hpp`:

```cpp
#pragma once

#include <array>
#include <cstdint>

#include "physics/airflow/mac_grid.hpp"

// ---------------------------------------------------------------------------
// THE DOMAIN FACES' VELOCITY CONDITIONS (spec §2). Each fluid step a domain
// face is one of three kinds:
//   * fixed_velocity: a wall, or an open face the ambient flows in through.
//     Its normal velocity is prescribed; the pressure is Neumann there.
//   * pressure_outlet: an open face the ambient does not flow in through
//     (still air included). p = 0 on the face; the normal velocity is copied
//     from the neighbouring face, then corrected by the projection; flow into
//     the box through it is refused (it takes the ambient's normal velocity).
//   * periodic: the last face copies the first.
// Tangential velocities have no stored boundary value: the viscous step and
// the sampler supply them (viscous.hpp, advect.hpp).
// ---------------------------------------------------------------------------
namespace spade::physics::airflow {

enum class FaceKind : uint8_t { fixed_velocity, pressure_outlet, periodic };

[[nodiscard]] FaceKind face_kind(const DomainBc& bc, Side side) noexcept;
[[nodiscard]] std::array<FaceKind, 6> face_kinds(const DomainBc& bc) noexcept;
[[nodiscard]] float fixed_normal_velocity(const DomainBc& bc, Side side) noexcept;

// Writes every domain face's normal velocity, axis by axis, x then y then z.
void apply_velocity_boundaries(MacGrid& g, const DomainBc& bc) noexcept;

}  // namespace spade::physics::airflow
```

- [ ] **Step 4: Write the source**

`engine/physics/airflow/boundaries.cpp`:

```cpp
#include "physics/airflow/boundaries.hpp"

namespace spade::physics::airflow {

FaceKind face_kind(const DomainBc& bc, Side side) noexcept {
    const FaceBc f = bc.face[side];
    if (f == FaceBc::periodic) return FaceKind::periodic;
    if (f == FaceBc::wall) return FaceKind::fixed_velocity;
    const auto axis = static_cast<glm::length_t>(side / 2u);
    const float inward = (side % 2u) == 0u ? bc.ambient[axis] : -bc.ambient[axis];
    return inward > 0.0f ? FaceKind::fixed_velocity : FaceKind::pressure_outlet;
}

std::array<FaceKind, 6> face_kinds(const DomainBc& bc) noexcept {
    std::array<FaceKind, 6> out{};
    for (uint32_t s = 0; s < 6; ++s) out[s] = face_kind(bc, static_cast<Side>(s));
    return out;
}

float fixed_normal_velocity(const DomainBc& bc, Side side) noexcept {
    const auto axis = static_cast<glm::length_t>(side / 2u);
    return bc.face[side] == FaceBc::wall ? bc.wall_velocity[side][axis] : bc.ambient[axis];
}

namespace {

// One axis' normal faces. `at(q, a, b)` addresses the component's array with q
// along the axis (0..n) and (a, b) across it.
template <class At>
void apply_axis(const DomainBc& bc, uint32_t axis, uint32_t n, uint32_t m1, uint32_t m2, At at) noexcept {
    const auto lo = static_cast<Side>(2u * axis);
    const auto hi = static_cast<Side>(2u * axis + 1u);
    const FaceKind klo = face_kind(bc, lo);
    const FaceKind khi = face_kind(bc, hi);
    const float amb = bc.ambient[static_cast<glm::length_t>(axis)];
    for (uint32_t b = 0; b < m2; ++b) {
        for (uint32_t a = 0; a < m1; ++a) {
            if (klo == FaceKind::periodic) {
                at(n, a, b) = at(0, a, b);
                continue;
            }
            if (klo == FaceKind::fixed_velocity) {
                at(0, a, b) = fixed_normal_velocity(bc, lo);
            } else {
                float copy = at(1, a, b);
                if (copy > 0.0f) copy = amb;  // into the box through the minus face: refused
                at(0, a, b) = copy;
            }
            if (khi == FaceKind::fixed_velocity) {
                at(n, a, b) = fixed_normal_velocity(bc, hi);
            } else {
                float copy = at(n - 1u, a, b);
                if (copy < 0.0f) copy = amb;  // into the box through the plus face: refused
                at(n, a, b) = copy;
            }
        }
    }
}

}  // namespace

void apply_velocity_boundaries(MacGrid& g, const DomainBc& bc) noexcept {
    const GridShape& s = g.shape;
    apply_axis(bc, 0, s.nx, s.ny, s.nz,
               [&](uint32_t q, uint32_t a, uint32_t b) -> float& { return g.u[u_index(s, q, a, b)]; });
    apply_axis(bc, 1, s.ny, s.nx, s.nz,
               [&](uint32_t q, uint32_t a, uint32_t b) -> float& { return g.v[v_index(s, a, q, b)]; });
    apply_axis(bc, 2, s.nz, s.nx, s.ny,
               [&](uint32_t q, uint32_t a, uint32_t b) -> float& { return g.w[w_index(s, a, b, q)]; });
}

}  // namespace spade::physics::airflow
```

Add `physics/airflow/boundaries.cpp` to `spade_physics`.

- [ ] **Step 5: Run the tests and confirm they pass**

Run: `build-ninja\release\bin\spade_tests.exe --gtest_filter=AirflowBoundaries.*:AirflowGrid.*`
Expected: 9 tests PASS.

- [ ] **Step 6: Commit**

```bash
git add engine/physics/airflow/boundaries.hpp engine/physics/airflow/boundaries.cpp engine/CMakeLists.txt tests/test_airflow_grid.cpp
git commit -m "feat(physics): the airflow domain faces' velocity conditions (airflow core, Task 2)"
```

---

### Task 3: The divergence and the pressure operator

**Files:**
- Create: `engine/physics/airflow/pressure.hpp`, `engine/physics/airflow/pressure.cpp`
- Modify: `engine/CMakeLists.txt`
- Create: `tests/test_airflow_pressure.cpp`; modify `tests/CMakeLists.txt`

**Interfaces:**
- Consumes: Tasks 1–2.
- Produces: `struct PoissonLevel { uint32_t nx, ny, nz; std::array<FaceKind, 6> kind; std::vector<float> p, b, r; }`, `make_level(uint32_t, uint32_t, uint32_t, const std::array<FaceKind, 6>&) -> PoissonLevel`, `divergence(const MacGrid&, std::span<float>) noexcept`, `smooth_one_cell(PoissonLevel&, uint32_t, uint32_t, uint32_t) noexcept`, `smooth_red_black(PoissonLevel&, uint32_t sweeps) noexcept`, `residual(PoissonLevel&) noexcept`.

**The equation**, scaled by `dx²`: for each cell, `Σ_faces (p_nb − p) = b`. A face to another cell, or wrapped across a periodic axis of size > 1, contributes `p_nb − p`. A pressure outlet contributes `−2p` (the ghost is `−p`, so `p` = 0 on the face itself). A fixed-velocity face, or a periodic axis of size 1, contributes nothing. The Gauss–Seidel update is `p = (sum − b) / diag`, where `sum` adds the real neighbours and `diag` counts 1 per real neighbour and 2 per outlet face; a cell with `diag` = 0 is left alone.

- [ ] **Step 1: Write the failing tests**

`tests/test_airflow_pressure.cpp`:

```cpp
// The airflow solver core's pressure operator, multigrid and projection
// (airflow core plan, Tasks 3-5).
#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <vector>

#include "physics/airflow/boundaries.hpp"
#include "physics/airflow/pressure.hpp"

namespace {

using namespace spade::physics::airflow;

// A fixed pseudo-random sequence in [-1, 1), so every run sees the same field.
struct Lcg {
    uint32_t state = 12345u;
    float next() noexcept {
        state = state * 1664525u + 1013904223u;
        return static_cast<float>(state >> 8) * (2.0f / 16777216.0f) - 1.0f;
    }
};

[[nodiscard]] std::array<FaceKind, 6> all(FaceKind k) {
    std::array<FaceKind, 6> out{};
    out.fill(k);
    return out;
}

[[nodiscard]] float max_abs(const std::vector<float>& v) {
    float m = 0.0f;
    for (float x : v) m = std::max(m, std::fabs(x));
    return m;
}

TEST(AirflowPressure, AUniformFlowHasExactlyZeroDivergence) {
    MacGrid g = *make_grid({4, 4, 4, 0.1f}, DomainBc{});
    std::fill(g.u.begin(), g.u.end(), 1.5f);
    std::fill(g.v.begin(), g.v.end(), -0.25f);
    std::fill(g.w.begin(), g.w.end(), 3.0f);
    std::vector<float> div(g.p.size(), 7.0f);
    divergence(g, div);
    for (float d : div) EXPECT_EQ(d, 0.0f);
}

TEST(AirflowPressure, ALinearFlowsDivergenceIsItsSlope) {
    MacGrid g = *make_grid({4, 4, 4, 0.5f}, DomainBc{});
    for_each_sample(g.shape, 0, [&](std::size_t idx, glm::vec3 x) { g.u[idx] = 2.0f * x.x; });
    std::vector<float> div(g.p.size(), 0.0f);
    divergence(g, div);
    for (float d : div) EXPECT_NEAR(d, 2.0f, 1.0e-6f);
}

// b set from a known p through the same arithmetic, so the residual is
// exactly 0: the expression is deterministic, which the GPU twin relies on.
TEST(AirflowPressure, AManufacturedSolutionLeavesAZeroResidual) {
    PoissonLevel L = make_level(4, 4, 4, all(FaceKind::pressure_outlet));
    Lcg rng;
    for (float& x : L.p) x = rng.next();
    std::fill(L.b.begin(), L.b.end(), 0.0f);
    residual(L);  // r = 0 - (sum - diag p)
    for (std::size_t c = 0; c < L.b.size(); ++c) L.b[c] = -L.r[c];
    residual(L);
    for (float r : L.r) EXPECT_EQ(r, 0.0f);
}

TEST(AirflowPressure, SmoothingReducesTheResidual) {
    PoissonLevel L = make_level(16, 16, 16, all(FaceKind::pressure_outlet));
    Lcg rng;
    for (float& x : L.b) x = rng.next();
    residual(L);
    const float r0 = max_abs(L.r);
    smooth_red_black(L, 10);
    residual(L);
    EXPECT_LT(max_abs(L.r), 0.5f * r0);
}

// Red-black is order-free within a colour: sweeping the cells in reverse order
// gives the same bits. That is the property that lets the GPU update every
// cell of a colour at once.
TEST(AirflowPressure, AColourSweepDoesNotDependOnItsCellOrder) {
    std::array<FaceKind, 6> kinds = all(FaceKind::periodic);
    kinds[kXMinus] = kinds[kXPlus] = FaceKind::pressure_outlet;
    PoissonLevel forward = make_level(8, 6, 4, kinds);
    Lcg rng;
    for (float& x : forward.b) x = rng.next();
    for (float& x : forward.p) x = rng.next();
    PoissonLevel reverse = forward;
    smooth_red_black(forward, 3);
    for (uint32_t sweep = 0; sweep < 3; ++sweep) {
        for (uint32_t colour = 0; colour < 2; ++colour) {
            for (uint32_t k = reverse.nz; k-- > 0;) {
                for (uint32_t j = reverse.ny; j-- > 0;) {
                    for (uint32_t i = reverse.nx; i-- > 0;) {
                        if (((i + j + k) & 1u) == colour) smooth_one_cell(reverse, i, j, k);
                    }
                }
            }
        }
    }
    EXPECT_EQ(forward.p, reverse.p);
}

}  // namespace
```

- [ ] **Step 2: Register the test file and confirm it fails to compile**

Add `test_airflow_pressure.cpp` after `test_airflow_grid.cpp`. Expected: FAIL, `physics/airflow/pressure.hpp: No such file or directory`.

- [ ] **Step 3: Write the header**

`engine/physics/airflow/pressure.hpp`:

```cpp
#pragma once

#include <array>
#include <cstdint>
#include <span>
#include <vector>

#include "physics/airflow/boundaries.hpp"
#include "physics/airflow/mac_grid.hpp"

// ---------------------------------------------------------------------------
// THE PRESSURE EQUATION (spec §1.5, "Project"), on one grid level, scaled by
// dx^2: for each cell, the sum over its six faces of (p_nb - p) = b. A face to
// another cell (or across a periodic axis of size > 1) contributes p_nb - p; a
// pressure outlet contributes -2 p (ghost -p: p = 0 on the face); a
// fixed-velocity face or a periodic axis of size 1 contributes nothing. The
// faces are visited -x, +x, -y, +y, -z, +z, and the GPU twin keeps that order.
// ---------------------------------------------------------------------------
namespace spade::physics::airflow {

struct PoissonLevel {
    uint32_t nx = 0;
    uint32_t ny = 0;
    uint32_t nz = 0;
    std::array<FaceKind, 6> kind{};
    std::vector<float> p;  // the unknown
    std::vector<float> b;  // the right side
    std::vector<float> r;  // the residual b - (sum - diag p), after residual()
};

[[nodiscard]] PoissonLevel make_level(uint32_t nx, uint32_t ny, uint32_t nz, const std::array<FaceKind, 6>& kind);

// The discrete divergence of every cell, 1/s:
// ((u_{i+1} - u_i) + (v_{j+1} - v_j) + (w_{k+1} - w_k)) * (1 / dx).
void divergence(const MacGrid& g, std::span<float> out) noexcept;

// One Gauss-Seidel update of cell (i, j, k): p = (sum - b) / diag.
void smooth_one_cell(PoissonLevel& level, uint32_t i, uint32_t j, uint32_t k) noexcept;

// `sweeps` red-black sweeps: every cell with (i + j + k) even, then every odd one.
void smooth_red_black(PoissonLevel& level, uint32_t sweeps) noexcept;

// level.r = b - (sum - diag p) for every cell.
void residual(PoissonLevel& level) noexcept;

}  // namespace spade::physics::airflow
```

- [ ] **Step 4: Write the source**

`engine/physics/airflow/pressure.cpp`:

```cpp
#include "physics/airflow/pressure.hpp"

namespace spade::physics::airflow {

namespace {

struct Stencil {
    float sum = 0.0f;
    float diag = 0.0f;
};

[[nodiscard]] inline std::size_t at(const PoissonLevel& L, uint32_t i, uint32_t j, uint32_t k) noexcept {
    return i + std::size_t{L.nx} * (j + std::size_t{L.ny} * k);
}

// The neighbour sum and the diagonal of cell (i, j, k), faces -x, +x, -y, +y, -z, +z.
[[nodiscard]] Stencil stencil(const PoissonLevel& L, uint32_t i, uint32_t j, uint32_t k) noexcept {
    Stencil s;
    const uint32_t n[3] = {L.nx, L.ny, L.nz};
    const uint32_t c[3] = {i, j, k};
    for (uint32_t axis = 0; axis < 3; ++axis) {
        for (uint32_t dir = 0; dir < 2; ++dir) {
            const bool edge = dir == 0 ? c[axis] == 0 : c[axis] + 1u == n[axis];
            uint32_t nb[3] = {i, j, k};
            if (!edge) {
                nb[axis] = dir == 0 ? c[axis] - 1u : c[axis] + 1u;
                s.sum += L.p[at(L, nb[0], nb[1], nb[2])];
                s.diag += 1.0f;
                continue;
            }
            switch (L.kind[2u * axis + dir]) {
                case FaceKind::periodic:
                    if (n[axis] > 1) {
                        nb[axis] = dir == 0 ? n[axis] - 1u : 0u;
                        s.sum += L.p[at(L, nb[0], nb[1], nb[2])];
                        s.diag += 1.0f;
                    }
                    break;
                case FaceKind::pressure_outlet:
                    s.diag += 2.0f;
                    break;
                case FaceKind::fixed_velocity:
                    break;
            }
        }
    }
    return s;
}

}  // namespace

PoissonLevel make_level(uint32_t nx, uint32_t ny, uint32_t nz, const std::array<FaceKind, 6>& kind) {
    PoissonLevel L;
    L.nx = nx;
    L.ny = ny;
    L.nz = nz;
    L.kind = kind;
    const std::size_t cells = std::size_t{nx} * ny * nz;
    L.p.assign(cells, 0.0f);
    L.b.assign(cells, 0.0f);
    L.r.assign(cells, 0.0f);
    return L;
}

void divergence(const MacGrid& g, std::span<float> out) noexcept {
    const GridShape& s = g.shape;
    const float inv_dx = 1.0f / s.dx;
    for (uint32_t k = 0; k < s.nz; ++k) {
        for (uint32_t j = 0; j < s.ny; ++j) {
            for (uint32_t i = 0; i < s.nx; ++i) {
                const float du = g.u[u_index(s, i + 1u, j, k)] - g.u[u_index(s, i, j, k)];
                const float dv = g.v[v_index(s, i, j + 1u, k)] - g.v[v_index(s, i, j, k)];
                const float dw = g.w[w_index(s, i, j, k + 1u)] - g.w[w_index(s, i, j, k)];
                out[cell_index(s, i, j, k)] = ((du + dv) + dw) * inv_dx;
            }
        }
    }
}

void smooth_one_cell(PoissonLevel& L, uint32_t i, uint32_t j, uint32_t k) noexcept {
    const Stencil st = stencil(L, i, j, k);
    if (st.diag == 0.0f) return;
    const std::size_t c = at(L, i, j, k);
    L.p[c] = (st.sum - L.b[c]) / st.diag;
}

void smooth_red_black(PoissonLevel& L, uint32_t sweeps) noexcept {
    for (uint32_t sweep = 0; sweep < sweeps; ++sweep) {
        for (uint32_t colour = 0; colour < 2; ++colour) {
            for (uint32_t k = 0; k < L.nz; ++k) {
                for (uint32_t j = 0; j < L.ny; ++j) {
                    for (uint32_t i = 0; i < L.nx; ++i) {
                        if (((i + j + k) & 1u) == colour) smooth_one_cell(L, i, j, k);
                    }
                }
            }
        }
    }
}

void residual(PoissonLevel& L) noexcept {
    for (uint32_t k = 0; k < L.nz; ++k) {
        for (uint32_t j = 0; j < L.ny; ++j) {
            for (uint32_t i = 0; i < L.nx; ++i) {
                const Stencil st = stencil(L, i, j, k);
                const std::size_t c = at(L, i, j, k);
                L.r[c] = L.b[c] - (st.sum - st.diag * L.p[c]);
            }
        }
    }
}

}  // namespace spade::physics::airflow
```

Add `physics/airflow/pressure.cpp` to `spade_physics`.

- [ ] **Step 5: Run the tests and confirm they pass**

Run: `build-ninja\release\bin\spade_tests.exe --gtest_filter=AirflowPressure.*`
Expected: 5 tests PASS. If `AColourSweepDoesNotDependOnItsCellOrder` fails, a periodic wrap joined two cells of one colour: check that `make_grid`'s even-size rule reached this level.

- [ ] **Step 6: Commit**

```bash
git add engine/physics/airflow/pressure.hpp engine/physics/airflow/pressure.cpp engine/CMakeLists.txt tests/test_airflow_pressure.cpp tests/CMakeLists.txt
git commit -m "feat(physics): the airflow pressure operator, red-black Gauss-Seidel and the residual (airflow core, Task 3)"
```

---

### Task 4: The multigrid V-cycle

**Files:**
- Create: `engine/physics/airflow/multigrid.hpp`, `engine/physics/airflow/multigrid.cpp`
- Modify: `engine/CMakeLists.txt`; `tests/test_airflow_pressure.cpp` (a new suite, `AirflowMultigrid`)

**Interfaces:**
- Consumes: Task 3.
- Produces: `struct Multigrid { std::vector<PoissonLevel> levels; }` (`levels[0]` the finest), `make_multigrid(uint32_t, uint32_t, uint32_t) -> Multigrid`, `set_kinds(Multigrid&, const std::array<FaceKind, 6>&) noexcept`, `restrict_residual(const PoissonLevel& fine, PoissonLevel& coarse) noexcept`, `prolong_add(const PoissonLevel& coarse, PoissonLevel& fine) noexcept`, `v_cycle(Multigrid&) noexcept`, and `kPreSweeps` = 2, `kPostSweeps` = 2, `kCoarsestSweeps` = 32.

**The design** (spec §1.5, "Pressure solver"):
- **Coarsening.** An axis of size 1 takes no part. A level is coarsened when every participating axis is even and at least 4, halving each; otherwise it is the coarsest. 128 × 128 × 1 gives 128, 64, 32, 16, 8, 4, 2 (each × 1).
- **Restriction** averages a coarse cell's children (8 in 3D, 4 in a slab) and scales by 4, because the equation is scaled by `dx²` and the coarse `dx` is twice the fine: `b_c = sum × (4 / children)`.
- **Prolongation** is cell-centred trilinear: along a coarsened axis, an even fine index takes ¾ of coarse `I = i/2` and ¼ of `I − 1`, an odd one ¾ of `I` and ¼ of `I + 1`. Past the edge a periodic axis wraps and any other clamps.
- **The cycle:** 2 red–black sweeps, the residual, restriction, a zero guess on the coarse level, recursion, prolongation added, 2 sweeps. The coarsest level takes 32 sweeps.

- [ ] **Step 1: Write the failing tests**

Append to `tests/test_airflow_pressure.cpp`'s anonymous namespace, with `#include "physics/airflow/multigrid.hpp"` at the top:

```cpp
TEST(AirflowMultigrid, HalvesEveryParticipatingAxisUntilOneIsTooSmall) {
    const Multigrid slab = make_multigrid(128, 128, 1);
    ASSERT_EQ(slab.levels.size(), 7u);
    EXPECT_EQ(slab.levels.back().nx, 2u);
    EXPECT_EQ(slab.levels.back().nz, 1u) << "a size-1 axis takes no part";
    const Multigrid box = make_multigrid(64, 64, 128);
    ASSERT_EQ(box.levels.size(), 6u);
    EXPECT_EQ(box.levels.back().nx, 2u);
    EXPECT_EQ(box.levels.back().nz, 4u);
    EXPECT_EQ(make_multigrid(6, 6, 6).levels.size(), 2u) << "6 -> 3, and 3 is odd";
}

TEST(AirflowMultigrid, OneVCycleCutsTheResidualAtLeastThreefold) {
    for (const std::array<uint32_t, 3>& n : {std::array<uint32_t, 3>{32, 32, 32}, std::array<uint32_t, 3>{64, 64, 1}}) {
        Multigrid mg = make_multigrid(n[0], n[1], n[2]);
        std::array<FaceKind, 6> kinds = all(FaceKind::pressure_outlet);
        kinds[kYMinus] = kinds[kYPlus] = FaceKind::fixed_velocity;
        if (n[2] == 1u) kinds[kZMinus] = kinds[kZPlus] = FaceKind::periodic;
        set_kinds(mg, kinds);
        Lcg rng;
        for (float& x : mg.levels[0].b) x = rng.next();
        residual(mg.levels[0]);
        const float r0 = max_abs(mg.levels[0].r);
        v_cycle(mg);
        residual(mg.levels[0]);
        const float r1 = max_abs(mg.levels[0].r);
        std::printf("V(2,2) on %ux%ux%u: residual %.3e -> %.3e, factor %.1f\n", n[0], n[1], n[2],
                    static_cast<double>(r0), static_cast<double>(r1), static_cast<double>(r0 / r1));
        EXPECT_LT(r1, r0 / 3.0f) << n[0] << "x" << n[1] << "x" << n[2];
    }
}

TEST(AirflowMultigrid, TwoRunsAreBitIdentical) {
    const auto run = [] {
        Multigrid mg = make_multigrid(16, 16, 16);
        set_kinds(mg, all(FaceKind::pressure_outlet));
        Lcg rng;
        for (float& x : mg.levels[0].b) x = rng.next();
        for (int n = 0; n < 3; ++n) v_cycle(mg);
        return mg.levels[0].p;
    };
    EXPECT_EQ(run(), run());
}
```

- [ ] **Step 2: Run them and confirm they fail to compile**

Expected: FAIL, `physics/airflow/multigrid.hpp: No such file or directory`.

- [ ] **Step 3: Write the header**

`engine/physics/airflow/multigrid.hpp`:

```cpp
#pragma once

#include <array>
#include <cstdint>
#include <vector>

#include "physics/airflow/pressure.hpp"

// ---------------------------------------------------------------------------
// GEOMETRIC MULTIGRID for the pressure equation (spec §1.5): V(2,2) cycles of
// red-black Gauss-Seidel, averaging restriction, cell-centred trilinear
// prolongation, and a coarsest level solved by 32 sweeps. A fixed number of
// cycles per projection; nothing stops on a residual.
// ---------------------------------------------------------------------------
namespace spade::physics::airflow {

inline constexpr uint32_t kPreSweeps = 2;
inline constexpr uint32_t kPostSweeps = 2;
inline constexpr uint32_t kCoarsestSweeps = 32;

struct Multigrid {
    std::vector<PoissonLevel> levels;  // [0] the finest
};

// The hierarchy for an nx x ny x nz grid. Face kinds are set by set_kinds().
[[nodiscard]] Multigrid make_multigrid(uint32_t nx, uint32_t ny, uint32_t nz);
void set_kinds(Multigrid& mg, const std::array<FaceKind, 6>& kind) noexcept;

// coarse.b = (sum of the children's residuals) * (4 / children).
void restrict_residual(const PoissonLevel& fine, PoissonLevel& coarse) noexcept;
// fine.p += the trilinear interpolation of coarse.p.
void prolong_add(const PoissonLevel& coarse, PoissonLevel& fine) noexcept;

// One V(2,2) cycle on levels[0]: its p is the initial guess, its b is set.
void v_cycle(Multigrid& mg) noexcept;

}  // namespace spade::physics::airflow
```

- [ ] **Step 4: Write the source**

`engine/physics/airflow/multigrid.cpp`:

```cpp
#include "physics/airflow/multigrid.hpp"

#include <algorithm>

namespace spade::physics::airflow {

namespace {

[[nodiscard]] inline std::size_t at(const PoissonLevel& L, uint32_t i, uint32_t j, uint32_t k) noexcept {
    return i + std::size_t{L.nx} * (j + std::size_t{L.ny} * k);
}

// Along one axis: the two coarse indices a fine index reads, and their weights.
struct Taps {
    uint32_t i0;
    uint32_t i1;
    float w0;
    float w1;
};

[[nodiscard]] Taps taps(uint32_t fine, uint32_t nf, uint32_t nc, bool periodic) noexcept {
    if (nf == nc) return {fine, fine, 1.0f, 0.0f};  // an axis of size 1 is not coarsened
    const uint32_t I = fine / 2u;
    if (fine % 2u == 0u) {
        const uint32_t other = I == 0u ? (periodic ? nc - 1u : 0u) : I - 1u;
        return {I, other, 0.75f, 0.25f};
    }
    const uint32_t other = I + 1u == nc ? (periodic ? 0u : I) : I + 1u;
    return {I, other, 0.75f, 0.25f};
}

void cycle(Multigrid& mg, std::size_t l) noexcept {
    PoissonLevel& f = mg.levels[l];
    if (l + 1u == mg.levels.size()) {
        smooth_red_black(f, kCoarsestSweeps);
        return;
    }
    smooth_red_black(f, kPreSweeps);
    residual(f);
    PoissonLevel& c = mg.levels[l + 1u];
    restrict_residual(f, c);
    std::fill(c.p.begin(), c.p.end(), 0.0f);
    cycle(mg, l + 1u);
    prolong_add(c, f);
    smooth_red_black(f, kPostSweeps);
}

}  // namespace

Multigrid make_multigrid(uint32_t nx, uint32_t ny, uint32_t nz) {
    Multigrid mg;
    mg.levels.push_back(make_level(nx, ny, nz, {}));
    for (;;) {
        const uint32_t n[3] = {mg.levels.back().nx, mg.levels.back().ny, mg.levels.back().nz};
        bool participates = false;
        bool coarsens = true;
        for (uint32_t a : n) {
            if (a == 1u) continue;
            participates = true;
            if (a < 4u || a % 2u != 0u) coarsens = false;
        }
        if (!participates || !coarsens) break;
        mg.levels.push_back(make_level(n[0] == 1u ? 1u : n[0] / 2u, n[1] == 1u ? 1u : n[1] / 2u,
                                       n[2] == 1u ? 1u : n[2] / 2u, {}));
    }
    return mg;
}

void set_kinds(Multigrid& mg, const std::array<FaceKind, 6>& kind) noexcept {
    for (PoissonLevel& L : mg.levels) L.kind = kind;
}

void restrict_residual(const PoissonLevel& f, PoissonLevel& c) noexcept {
    const uint32_t sx = f.nx == c.nx ? 1u : 2u;
    const uint32_t sy = f.ny == c.ny ? 1u : 2u;
    const uint32_t sz = f.nz == c.nz ? 1u : 2u;
    const float scale = 4.0f / static_cast<float>(sx * sy * sz);
    for (uint32_t K = 0; K < c.nz; ++K) {
        for (uint32_t J = 0; J < c.ny; ++J) {
            for (uint32_t I = 0; I < c.nx; ++I) {
                float sum = 0.0f;
                for (uint32_t dk = 0; dk < sz; ++dk) {
                    for (uint32_t dj = 0; dj < sy; ++dj) {
                        for (uint32_t di = 0; di < sx; ++di) {
                            sum += f.r[at(f, I * sx + di, J * sy + dj, K * sz + dk)];
                        }
                    }
                }
                c.b[at(c, I, J, K)] = sum * scale;
            }
        }
    }
}

void prolong_add(const PoissonLevel& c, PoissonLevel& f) noexcept {
    const bool px = f.kind[kXMinus] == FaceKind::periodic;
    const bool py = f.kind[kYMinus] == FaceKind::periodic;
    const bool pz = f.kind[kZMinus] == FaceKind::periodic;
    const auto q = [&](uint32_t a, uint32_t b, uint32_t d) { return c.p[at(c, a, b, d)]; };
    for (uint32_t k = 0; k < f.nz; ++k) {
        const Taps tz = taps(k, f.nz, c.nz, pz);
        for (uint32_t j = 0; j < f.ny; ++j) {
            const Taps ty = taps(j, f.ny, c.ny, py);
            for (uint32_t i = 0; i < f.nx; ++i) {
                const Taps tx = taps(i, f.nx, c.nx, px);
                const float z0 = ty.w0 * (tx.w0 * q(tx.i0, ty.i0, tz.i0) + tx.w1 * q(tx.i1, ty.i0, tz.i0)) +
                                 ty.w1 * (tx.w0 * q(tx.i0, ty.i1, tz.i0) + tx.w1 * q(tx.i1, ty.i1, tz.i0));
                const float z1 = ty.w0 * (tx.w0 * q(tx.i0, ty.i0, tz.i1) + tx.w1 * q(tx.i1, ty.i0, tz.i1)) +
                                 ty.w1 * (tx.w0 * q(tx.i0, ty.i1, tz.i1) + tx.w1 * q(tx.i1, ty.i1, tz.i1));
                f.p[at(f, i, j, k)] += tz.w0 * z0 + tz.w1 * z1;
            }
        }
    }
}

void v_cycle(Multigrid& mg) noexcept {
    if (!mg.levels.empty()) cycle(mg, 0);
}

}  // namespace spade::physics::airflow
```

Add `physics/airflow/multigrid.cpp` to `spade_physics`.

- [ ] **Step 5: Run the tests and confirm they pass**

Run: `build-ninja\release\bin\spade_tests.exe --gtest_filter=AirflowMultigrid.*:AirflowPressure.*`
Expected: 8 tests PASS. The printed factors are the measurement the spec asks for (§1.5: "about 10× on a plain Poisson problem"); quote them in the task's report. A factor under 3 is a defect in restriction or prolongation, not a tolerance to loosen.

- [ ] **Step 6: Commit**

```bash
git add engine/physics/airflow/multigrid.hpp engine/physics/airflow/multigrid.cpp engine/CMakeLists.txt tests/test_airflow_pressure.cpp
git commit -m "feat(physics): the airflow pressure solve's multigrid V-cycle (airflow core, Task 4)"
```

---

### Task 5: The projection

**Files:**
- Create: `engine/physics/airflow/projection.hpp`, `engine/physics/airflow/projection.cpp`
- Modify: `engine/CMakeLists.txt`; `tests/test_airflow_pressure.cpp` (a new suite, `AirflowProjection`)

**Interfaces:**
- Consumes: Tasks 1–4.
- Produces: `struct ProjectionScratch { Multigrid mg; std::vector<float> div; }`, `make_projection_scratch(const GridShape&) -> ProjectionScratch`, `project(MacGrid&, const DomainBc&, float rho, float h, uint32_t n_v, ProjectionScratch&) noexcept`.

**What it does** (spec §1.5, step 6): `b = (ρ dx² / h) · div(u*)`; `n_v` V-cycles from the grid's own `p` (the warm start); then every face that is not fixed takes `u −= (h / (ρ dx)) (p_right − p_left)`. An outlet face's outside value is the ghost `−p`; a periodic axis' faces 0 and `n` wrap and stay equal. Fixed-velocity faces are untouched. The face kinds are recomputed from the domain configuration on every call, so an ambient that turns around changes the next projection.

- [ ] **Step 1: Write the failing tests**

Append to `tests/test_airflow_pressure.cpp`, with `#include "physics/airflow/projection.hpp"`:

```cpp
[[nodiscard]] float max_divergence(const MacGrid& g) {
    std::vector<float> div(g.p.size(), 0.0f);
    divergence(g, div);
    return max_abs(div);
}

void randomise(MacGrid& g, uint32_t seed) {
    Lcg rng{seed};
    for (float& x : g.u) x = rng.next();
    for (float& x : g.v) x = rng.next();
    for (float& x : g.w) x = rng.next();
}

TEST(AirflowProjection, AUniformFlowPassesThroughUntouched) {
    DomainBc bc;
    bc.face.fill(FaceBc::periodic);
    MacGrid g = *make_grid({8, 8, 8, 0.1f}, bc);
    std::fill(g.u.begin(), g.u.end(), 1.5f);
    std::fill(g.v.begin(), g.v.end(), -0.5f);
    const MacGrid before = g;
    ProjectionScratch s = make_projection_scratch(g.shape);
    project(g, bc, 1.225f, 0.004f, 2, s);
    EXPECT_EQ(g.u, before.u);
    EXPECT_EQ(g.v, before.v);
    EXPECT_EQ(g.w, before.w);
    for (float p : g.p) EXPECT_EQ(p, 0.0f);
}

TEST(AirflowProjection, MoreCyclesLeaveLessDivergence) {
    DomainBc bc;  // still air: every face an outlet
    float previous = 0.0f;
    for (uint32_t n_v : {1u, 2u, 4u, 8u}) {
        MacGrid g = *make_grid({32, 32, 32, 0.05f}, bc);
        randomise(g, 7u);
        apply_velocity_boundaries(g, bc);
        const float before = max_divergence(g);
        ProjectionScratch s = make_projection_scratch(g.shape);
        project(g, bc, 1.225f, 0.004f, n_v, s);
        const float after = max_divergence(g);
        std::printf("n_v %u: max |div| %.3e -> %.3e\n", n_v, static_cast<double>(before), static_cast<double>(after));
        if (n_v > 1u) EXPECT_LT(after, previous) << n_v;
        previous = after;
        if (n_v == 8u) EXPECT_LT(after, 1.0e-4f * before);
    }
}

// Review Focus 3: Neumann on every face, so p is fixed only up to a constant.
TEST(AirflowProjection, AClosedBoxStaysFiniteAndItsDivergenceFalls) {
    DomainBc bc;
    bc.face.fill(FaceBc::wall);
    MacGrid g = *make_grid({16, 16, 16, 0.05f}, bc);
    randomise(g, 11u);
    apply_velocity_boundaries(g, bc);
    const float before = max_divergence(g);
    ProjectionScratch s = make_projection_scratch(g.shape);
    for (int n = 0; n < 20; ++n) project(g, bc, 1.225f, 0.004f, 2, s);
    for (float x : g.p) ASSERT_TRUE(std::isfinite(x));
    EXPECT_LT(max_divergence(g), 1.0e-3f * before);
}

// Review Focus 4.
TEST(AirflowProjection, AFaceFollowsTheAmbientEachStep) {
    DomainBc bc;
    bc.ambient = glm::vec3(1.0f, 0.0f, 0.0f);  // x- is an inflow, fixed
    MacGrid g = *make_grid({8, 4, 4, 0.1f}, bc);
    randomise(g, 3u);
    apply_velocity_boundaries(g, bc);
    ProjectionScratch s = make_projection_scratch(g.shape);
    project(g, bc, 1.225f, 0.004f, 2, s);
    EXPECT_EQ(g.u[u_index(g.shape, 0, 2, 2)], 1.0f) << "an inflow face is fixed";
    bc.ambient = glm::vec3(-1.0f, 0.0f, 0.0f);  // now x- is an outlet
    g.u[u_index(g.shape, 0, 2, 2)] = 0.0f;
    project(g, bc, 1.225f, 0.004f, 2, s);
    EXPECT_NE(g.u[u_index(g.shape, 0, 2, 2)], 0.0f) << "an outlet face is corrected by the projection";
}
```

- [ ] **Step 2: Run them and confirm they fail to compile**

Expected: FAIL, `physics/airflow/projection.hpp: No such file or directory`.

- [ ] **Step 3: Write the header**

`engine/physics/airflow/projection.hpp`:

```cpp
#pragma once

#include <cstdint>
#include <vector>

#include "physics/airflow/multigrid.hpp"

// ---------------------------------------------------------------------------
// THE PROJECTION (spec §1.5, step 6). Solves (dx^2-scaled) sum (p_nb - p) =
// (rho dx^2 / h) div(u*) by n_v V-cycles from the grid's own p, then subtracts
// (h / (rho dx)) (p_right - p_left) from every face that is not fixed. The
// face kinds come from `bc` on every call.
// ---------------------------------------------------------------------------
namespace spade::physics::airflow {

struct ProjectionScratch {
    Multigrid mg;
    std::vector<float> div;
};

[[nodiscard]] ProjectionScratch make_projection_scratch(const GridShape& shape);

void project(MacGrid& g, const DomainBc& bc, float rho, float h, uint32_t n_v, ProjectionScratch& s) noexcept;

}  // namespace spade::physics::airflow
```

- [ ] **Step 4: Write the source**

`engine/physics/airflow/projection.cpp`:

```cpp
#include "physics/airflow/projection.hpp"

#include <algorithm>

namespace spade::physics::airflow {

namespace {

// One axis' normal faces: q along the axis (0..n), (a, b) across it. `vel`
// addresses the component, `cell(q, a, b)` the pressure cell at q.
template <class Vel, class Cell>
void subtract_gradient(uint32_t n, uint32_t m1, uint32_t m2, FaceKind lo, FaceKind hi, float g_coef,
                       const std::vector<float>& p, Vel vel, Cell cell) noexcept {
    for (uint32_t b = 0; b < m2; ++b) {
        for (uint32_t a = 0; a < m1; ++a) {
            for (uint32_t q = 1; q < n; ++q) {
                vel(q, a, b) -= g_coef * (p[cell(q, a, b)] - p[cell(q - 1u, a, b)]);
            }
            const float p_first = p[cell(0, a, b)];
            const float p_last = p[cell(n - 1u, a, b)];
            if (lo == FaceKind::periodic) {
                if (n > 1u) vel(0, a, b) -= g_coef * (p_first - p_last);
                vel(n, a, b) = vel(0, a, b);
                continue;
            }
            if (lo == FaceKind::pressure_outlet) vel(0, a, b) -= g_coef * (p_first - (-p_first));
            if (hi == FaceKind::pressure_outlet) vel(n, a, b) -= g_coef * ((-p_last) - p_last);
        }
    }
}

}  // namespace

ProjectionScratch make_projection_scratch(const GridShape& shape) {
    ProjectionScratch s;
    s.mg = make_multigrid(shape.nx, shape.ny, shape.nz);
    s.div.assign(std::size_t{shape.nx} * shape.ny * shape.nz, 0.0f);
    return s;
}

void project(MacGrid& g, const DomainBc& bc, float rho, float h, uint32_t n_v, ProjectionScratch& s) noexcept {
    const GridShape& sh = g.shape;
    const std::array<FaceKind, 6> kinds = face_kinds(bc);
    set_kinds(s.mg, kinds);
    divergence(g, s.div);
    PoissonLevel& f = s.mg.levels[0];
    const float b_coef = rho * sh.dx * sh.dx / h;
    for (std::size_t c = 0; c < f.b.size(); ++c) f.b[c] = b_coef * s.div[c];
    std::copy(g.p.begin(), g.p.end(), f.p.begin());
    for (uint32_t n = 0; n < n_v; ++n) v_cycle(s.mg);
    std::copy(f.p.begin(), f.p.end(), g.p.begin());

    const float g_coef = h / (rho * sh.dx);
    subtract_gradient(
        sh.nx, sh.ny, sh.nz, kinds[kXMinus], kinds[kXPlus], g_coef, g.p,
        [&](uint32_t q, uint32_t a, uint32_t b) -> float& { return g.u[u_index(sh, q, a, b)]; },
        [&](uint32_t q, uint32_t a, uint32_t b) { return cell_index(sh, q, a, b); });
    subtract_gradient(
        sh.ny, sh.nx, sh.nz, kinds[kYMinus], kinds[kYPlus], g_coef, g.p,
        [&](uint32_t q, uint32_t a, uint32_t b) -> float& { return g.v[v_index(sh, a, q, b)]; },
        [&](uint32_t q, uint32_t a, uint32_t b) { return cell_index(sh, a, q, b); });
    subtract_gradient(
        sh.nz, sh.nx, sh.ny, kinds[kZMinus], kinds[kZPlus], g_coef, g.p,
        [&](uint32_t q, uint32_t a, uint32_t b) -> float& { return g.w[w_index(sh, a, b, q)]; },
        [&](uint32_t q, uint32_t a, uint32_t b) { return cell_index(sh, a, b, q); });
}

}  // namespace spade::physics::airflow
```

Add `physics/airflow/projection.cpp` to `spade_physics`.

- [ ] **Step 5: Run the tests and confirm they pass**

Run: `build-ninja\release\bin\spade_tests.exe --gtest_filter=AirflowProjection.*:AirflowMultigrid.*:AirflowPressure.*`
Expected: 12 tests PASS. Quote the printed `n_v` table in the report: it is the first measurement of the spec's "residual each `n_V` leaves".

- [ ] **Step 6: Commit, and hand the expressions to plan B**

```bash
git add engine/physics/airflow/projection.hpp engine/physics/airflow/projection.cpp engine/CMakeLists.txt tests/test_airflow_pressure.cpp
git commit -m "feat(physics): the airflow projection (airflow core, Task 5)"
```

Tell the lead that Task 5 is in: the pressure path's expressions are now fixed, so plan B (the GPU twin, with Core's kernel chain) can be written against them.

---

### Task 6: Advection — semi-Lagrangian with the MacCormack correction

**Files:**
- Create: `engine/physics/airflow/advect.hpp`, `engine/physics/airflow/advect.cpp`
- Modify: `engine/CMakeLists.txt`
- Create: `tests/test_airflow_advect.cpp`; modify `tests/CMakeLists.txt`

**Interfaces:**
- Consumes: Tasks 1–2.
- Produces: `struct Sample { float value, lo, hi; }`, `sample_component(const GridShape&, std::span<const float>, uint32_t component, glm::vec3 x, const DomainBc&) noexcept -> Sample`, `velocity_at(const MacGrid&, glm::vec3, const DomainBc&) noexcept -> glm::vec3`, `struct AdvectScratch { std::array<std::vector<float>, 3> hat, lo, hi; }`, `make_advect_scratch(const GridShape&) -> AdvectScratch`, `advect_velocity(const MacGrid& in, MacGrid& out, const DomainBc&, float h, AdvectScratch&) noexcept`.

**What it does** (spec §1.5, step 4): for every face, an RK2 backtrace through `in`'s velocity (`mid = x − ½h·u(x)`, `x_d = x − h·u(mid)`), the forward estimate `φ̂ = φ(x_d)`, the backward estimate `φ̃ = φ̂(x_f)` from the forward trace (`x_f = x + h·u(x + ½h·u(x))`), and `φ = φ̂ + ½(φ_old − φ̃)`, clamped to the range of the eight values `φ̂` interpolated. Interpolation is trilinear in the lerp form `a + t(b − a)`, x then y then z, so a uniform field is reproduced bit for bit. A periodic axis wraps; any other clamps to the samples' extent, and a `NaN` coordinate clamps to 0 rather than reaching an integer cast.

- [ ] **Step 1: Write the failing tests**

`tests/test_airflow_advect.cpp`:

```cpp
// The airflow solver core's advection (airflow core plan, Task 6). Not a
// golden-feeding file: it computes reference values in double with std::sin.
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <numbers>
#include <vector>

#include "physics/airflow/advect.hpp"
#include "physics/airflow/boundaries.hpp"

namespace {

using namespace spade::physics::airflow;

struct Lcg {
    uint32_t state = 12345u;
    float next() noexcept {
        state = state * 1664525u + 1013904223u;
        return static_cast<float>(state >> 8) * (2.0f / 16777216.0f) - 1.0f;
    }
};

[[nodiscard]] DomainBc periodic_box() {
    DomainBc bc;
    bc.face.fill(FaceBc::periodic);
    return bc;
}

// dx a power of two, so x * (1 / dx) is exact and a sample at a stored
// position reads exactly the stored value.
TEST(AirflowAdvect, SamplingReadsAStoredValueAtItsOwnPosition) {
    const DomainBc bc = periodic_box();
    MacGrid g = *make_grid({4, 4, 4, 0.25f}, bc);
    Lcg rng;
    for (float& x : g.v) x = rng.next();
    apply_velocity_boundaries(g, bc);  // face ny is face 0 on a periodic axis
    for_each_sample(g.shape, 1, [&](std::size_t idx, glm::vec3 x) {
        EXPECT_EQ(sample_component(g.shape, g.v, 1, x, bc).value, g.v[idx]) << idx;
    });
}

TEST(AirflowAdvect, SamplingWrapsAPeriodicAxisAndClampsAnOtherAxis) {
    DomainBc bc = periodic_box();
    bc.face[kYMinus] = bc.face[kYPlus] = FaceBc::wall;
    MacGrid g = *make_grid({4, 4, 1, 0.25f}, bc);
    Lcg rng;
    for (float& x : g.u) x = rng.next();
    for (uint32_t j = 0; j < 4; ++j) g.u[u_index(g.shape, 4, j, 0)] = g.u[u_index(g.shape, 0, j, 0)];
    const glm::vec3 inside(0.25f, 0.375f, 0.125f);
    EXPECT_EQ(sample_component(g.shape, g.u, 0, inside + glm::vec3(1.0f, 0.0f, 0.0f), bc).value,
              sample_component(g.shape, g.u, 0, inside, bc).value)
        << "x is periodic with period nx dx = 1";
    EXPECT_EQ(sample_component(g.shape, g.u, 0, glm::vec3(0.25f, -5.0f, 0.125f), bc).value,
              sample_component(g.shape, g.u, 0, glm::vec3(0.25f, 0.125f, 0.125f), bc).value)
        << "y clamps to the first row of samples";
}

TEST(AirflowAdvect, AUniformFlowIsReproducedBitForBit) {
    const DomainBc bc = periodic_box();
    MacGrid in = *make_grid({8, 8, 8, 0.125f}, bc);
    std::fill(in.u.begin(), in.u.end(), 1.0f);
    std::fill(in.v.begin(), in.v.end(), 2.0f);
    std::fill(in.w.begin(), in.w.end(), -3.0f);
    MacGrid out = in;
    AdvectScratch s = make_advect_scratch(in.shape);
    advect_velocity(in, out, bc, 0.01f, s);
    EXPECT_EQ(out.u, in.u);
    EXPECT_EQ(out.v, in.v);
    EXPECT_EQ(out.w, in.w);
}

TEST(AirflowAdvect, TheClampMakesNoNewExtrema) {
    const DomainBc bc = periodic_box();
    MacGrid in = *make_grid({8, 8, 8, 0.125f}, bc);
    Lcg rng;
    for (float& x : in.u) x = rng.next();
    for (float& x : in.v) x = rng.next();
    for (float& x : in.w) x = rng.next();
    MacGrid out = in;
    AdvectScratch s = make_advect_scratch(in.shape);
    advect_velocity(in, out, bc, 0.05f, s);
    const auto range = [](const std::vector<float>& v) { return std::minmax_element(v.begin(), v.end()); };
    for (const auto& [a, b] : {std::pair{&in.u, &out.u}, std::pair{&in.v, &out.v}, std::pair{&in.w, &out.w}}) {
        EXPECT_GE(*range(*b).first, *range(*a).first);
        EXPECT_LE(*range(*b).second, *range(*a).second);
    }
}

// A sine in v carried once across a periodic box by a uniform u of 1 m/s at
// CFL 0.5 comes back to within 5% of its amplitude.
TEST(AirflowAdvect, ASmoothWaveCrossesThePeriodicBoxAndReturns) {
    const DomainBc bc = periodic_box();
    MacGrid g = *make_grid({64, 4, 4, 1.0f / 64.0f}, bc);
    std::fill(g.u.begin(), g.u.end(), 1.0f);
    const double amplitude = 1.0e-3;
    for_each_sample(g.shape, 1, [&](std::size_t idx, glm::vec3 x) {
        g.v[idx] = static_cast<float>(amplitude * std::sin(2.0 * std::numbers::pi * x.x));
    });
    const std::vector<float> start = g.v;
    MacGrid out = g;
    AdvectScratch s = make_advect_scratch(g.shape);
    const float h = 0.5f / 64.0f;
    for (int step = 0; step < 128; ++step) {
        advect_velocity(g, out, bc, h, s);
        std::swap(g.u, out.u);
        std::swap(g.v, out.v);
        std::swap(g.w, out.w);
    }
    float worst = 0.0f;
    for (std::size_t i = 0; i < start.size(); ++i) worst = std::max(worst, std::fabs(g.v[i] - start[i]));
    std::printf("one crossing at CFL 0.5, 64 cells per wavelength: max error %.3e of amplitude %.1e\n",
                static_cast<double>(worst), amplitude);
    EXPECT_LT(worst, 0.05f * static_cast<float>(amplitude));
}

// Review Focus 1.
TEST(AirflowAdvect, AVelocityFarPastTheGridStaysFinite) {
    for (bool periodic : {true, false}) {
        DomainBc bc = periodic ? periodic_box() : DomainBc{};
        MacGrid in = *make_grid({8, 8, 8, 0.125f}, bc);
        Lcg rng;
        for (float& x : in.u) x = 1.0e6f * rng.next();
        for (float& x : in.v) x = 1.0e6f * rng.next();
        for (float& x : in.w) x = 1.0e6f * rng.next();
        MacGrid out = in;
        AdvectScratch s = make_advect_scratch(in.shape);
        advect_velocity(in, out, bc, 0.01f, s);
        for (float x : out.u) ASSERT_TRUE(std::isfinite(x)) << (periodic ? "periodic" : "clamped");
        for (float x : out.v) ASSERT_TRUE(std::isfinite(x));
        for (float x : out.w) ASSERT_TRUE(std::isfinite(x));
    }
}

}  // namespace
```

- [ ] **Step 2: Register the test file and confirm it fails to compile**

Add `test_airflow_advect.cpp` after `test_airflow_pressure.cpp`. Expected: FAIL, `physics/airflow/advect.hpp: No such file or directory`.

- [ ] **Step 3: Write the header**

`engine/physics/airflow/advect.hpp`:

```cpp
#pragma once

#include <array>
#include <cstdint>
#include <span>
#include <vector>

#include <glm/glm.hpp>

#include "physics/airflow/mac_grid.hpp"

// ---------------------------------------------------------------------------
// ADVECTION (spec §1.5, step 4): semi-Lagrangian, an RK2 backtrace, and the
// MacCormack correction clamped to the forward stencil's range.
// ---------------------------------------------------------------------------
namespace spade::physics::airflow {

struct Sample {
    float value = 0.0f;
    float lo = 0.0f;  // the smallest of the eight values interpolated
    float hi = 0.0f;  // the largest
};

// Trilinear interpolation of one velocity component's array at a point
// (metres, in the grid's frame), in the lerp form a + t (b - a), x then y
// then z. A periodic axis wraps; any other clamps to the samples' extent.
[[nodiscard]] Sample sample_component(const GridShape& s, std::span<const float> field, uint32_t component,
                                      glm::vec3 x, const DomainBc& bc) noexcept;
[[nodiscard]] glm::vec3 velocity_at(const MacGrid& g, glm::vec3 x, const DomainBc& bc) noexcept;

struct AdvectScratch {
    std::array<std::vector<float>, 3> hat;  // the forward estimate, per component
    std::array<std::vector<float>, 3> lo;   // its stencil's range, for the clamp
    std::array<std::vector<float>, 3> hi;
};

[[nodiscard]] AdvectScratch make_advect_scratch(const GridShape& shape);

// Writes every face of `out` (shaped like `in`); reads only `in`.
void advect_velocity(const MacGrid& in, MacGrid& out, const DomainBc& bc, float h, AdvectScratch& s) noexcept;

}  // namespace spade::physics::airflow
```

- [ ] **Step 4: Write the source**

`engine/physics/airflow/advect.cpp`:

```cpp
#include "physics/airflow/advect.hpp"

#include <algorithm>
#include <cmath>

namespace spade::physics::airflow {

namespace {

struct Axis {
    uint32_t i0;
    uint32_t i1;
    float t;
};

// A fractional sample coordinate f along an axis of n cells holding `count`
// samples (n + 1 on the component's own axis, n on the others).
[[nodiscard]] Axis locate(float f, uint32_t n, uint32_t count, bool periodic) noexcept {
    if (periodic) {
        if (n == 1u) return {0u, 0u, 0.0f};
        const float nf = static_cast<float>(n);
        float w = f - nf * std::floor(f / nf);
        if (!(w >= 0.0f) || !(w < nf)) w = 0.0f;  // NaN, or rounding onto the period's end
        const auto i0 = static_cast<uint32_t>(w);
        return {i0, i0 + 1u == n ? 0u : i0 + 1u, w - static_cast<float>(i0)};
    }
    if (count == 1u) return {0u, 0u, 0.0f};
    const float top = static_cast<float>(count - 1u);
    const float c = f > top ? top : (f >= 0.0f ? f : 0.0f);  // NaN clamps to 0
    uint32_t i0 = static_cast<uint32_t>(c);
    if (i0 + 1u >= count) i0 = count - 2u;
    return {i0, i0 + 1u, c - static_cast<float>(i0)};
}

[[nodiscard]] inline float lerp(float a, float b, float t) noexcept { return a + t * (b - a); }

}  // namespace

Sample sample_component(const GridShape& s, std::span<const float> field, uint32_t c, glm::vec3 x,
                        const DomainBc& bc) noexcept {
    const uint32_t n[3] = {s.nx, s.ny, s.nz};
    const float inv_dx = 1.0f / s.dx;
    Axis ax[3];
    for (uint32_t a = 0; a < 3; ++a) {
        const bool own = a == c;
        const float f = x[static_cast<glm::length_t>(a)] * inv_dx - (own ? 0.0f : 0.5f);
        ax[a] = locate(f, n[a], own ? n[a] + 1u : n[a], bc.face[2u * a] == FaceBc::periodic);
    }
    const std::size_t sx = c == 0u ? s.nx + 1u : s.nx;
    const std::size_t sy = c == 1u ? s.ny + 1u : s.ny;
    const auto at = [&](uint32_t i, uint32_t j, uint32_t k) { return field[i + sx * (j + sy * k)]; };
    const float a000 = at(ax[0].i0, ax[1].i0, ax[2].i0);
    const float a100 = at(ax[0].i1, ax[1].i0, ax[2].i0);
    const float a010 = at(ax[0].i0, ax[1].i1, ax[2].i0);
    const float a110 = at(ax[0].i1, ax[1].i1, ax[2].i0);
    const float a001 = at(ax[0].i0, ax[1].i0, ax[2].i1);
    const float a101 = at(ax[0].i1, ax[1].i0, ax[2].i1);
    const float a011 = at(ax[0].i0, ax[1].i1, ax[2].i1);
    const float a111 = at(ax[0].i1, ax[1].i1, ax[2].i1);
    const float c0 = lerp(lerp(a000, a100, ax[0].t), lerp(a010, a110, ax[0].t), ax[1].t);
    const float c1 = lerp(lerp(a001, a101, ax[0].t), lerp(a011, a111, ax[0].t), ax[1].t);
    Sample out{lerp(c0, c1, ax[2].t), a000, a000};
    for (const float v : {a100, a010, a110, a001, a101, a011, a111}) {
        out.lo = std::min(out.lo, v);
        out.hi = std::max(out.hi, v);
    }
    return out;
}

glm::vec3 velocity_at(const MacGrid& g, glm::vec3 x, const DomainBc& bc) noexcept {
    return glm::vec3(sample_component(g.shape, g.u, 0, x, bc).value, sample_component(g.shape, g.v, 1, x, bc).value,
                     sample_component(g.shape, g.w, 2, x, bc).value);
}

AdvectScratch make_advect_scratch(const GridShape& shape) {
    AdvectScratch s;
    const std::size_t sizes[3] = {std::size_t{shape.nx + 1u} * shape.ny * shape.nz,
                                  std::size_t{shape.nx} * (shape.ny + 1u) * shape.nz,
                                  std::size_t{shape.nx} * shape.ny * (shape.nz + 1u)};
    for (uint32_t c = 0; c < 3; ++c) {
        s.hat[c].assign(sizes[c], 0.0f);
        s.lo[c].assign(sizes[c], 0.0f);
        s.hi[c].assign(sizes[c], 0.0f);
    }
    return s;
}

void advect_velocity(const MacGrid& in, MacGrid& out, const DomainBc& bc, float h, AdvectScratch& s) noexcept {
    const float half_h = 0.5f * h;
    const auto back_trace = [&](glm::vec3 x) {
        const glm::vec3 mid = x - half_h * velocity_at(in, x, bc);
        return x - h * velocity_at(in, mid, bc);
    };
    const auto forward_trace = [&](glm::vec3 x) {
        const glm::vec3 mid = x + half_h * velocity_at(in, x, bc);
        return x + h * velocity_at(in, mid, bc);
    };
    const std::vector<float>* src[3] = {&in.u, &in.v, &in.w};
    std::vector<float>* dst[3] = {&out.u, &out.v, &out.w};
    // 1. The forward semi-Lagrangian estimate and its stencil's range.
    for (uint32_t c = 0; c < 3; ++c) {
        for_each_sample(in.shape, c, [&](std::size_t idx, glm::vec3 x) {
            const Sample f = sample_component(in.shape, *src[c], c, back_trace(x), bc);
            s.hat[c][idx] = f.value;
            s.lo[c][idx] = f.lo;
            s.hi[c][idx] = f.hi;
        });
    }
    // 2. The backward estimate from the forward one, and the clamped correction.
    for (uint32_t c = 0; c < 3; ++c) {
        for_each_sample(in.shape, c, [&](std::size_t idx, glm::vec3 x) {
            const float back = sample_component(in.shape, s.hat[c], c, forward_trace(x), bc).value;
            float v = s.hat[c][idx] + 0.5f * ((*src[c])[idx] - back);
            v = v < s.lo[c][idx] ? s.lo[c][idx] : (v > s.hi[c][idx] ? s.hi[c][idx] : v);
            (*dst[c])[idx] = v;
        });
    }
}

}  // namespace spade::physics::airflow
```

Add `physics/airflow/advect.cpp` to `spade_physics`.

- [ ] **Step 5: Run the tests and confirm they pass**

Run: `build-ninja\release\bin\spade_tests.exe --gtest_filter=AirflowAdvect.*`
Expected: 6 tests PASS. Quote the printed crossing error in the report.

- [ ] **Step 6: Commit**

```bash
git add engine/physics/airflow/advect.hpp engine/physics/airflow/advect.cpp engine/CMakeLists.txt tests/test_airflow_advect.cpp tests/CMakeLists.txt
git commit -m "feat(physics): airflow advection, semi-Lagrangian with a clamped MacCormack correction (airflow core, Task 6)"
```

---

### Task 7: Implicit viscosity and no-slip walls

**Files:**
- Create: `engine/physics/airflow/viscous.hpp`, `engine/physics/airflow/viscous.cpp`
- Modify: `engine/CMakeLists.txt`; `tests/test_airflow_advect.cpp` (a new suite, `AirflowViscous`)

**Interfaces:**
- Consumes: Tasks 1–2.
- Produces: `diffuse(MacGrid&, const DomainBc&, float nu, float h, uint32_t sweeps, std::vector<float>& rhs) noexcept`, where `rhs` holds at least the largest component's size.

**What it does** (spec §11.3, item 1; the molecular viscous term lands here, in M3, because V1's cavity needs it): backward Euler, `(1 + α Σ) x − α Σ x_nb = u*` with `α = ν h / dx²`, by `sweeps` red–black Gauss–Seidel sweeps from `x = u*`. Unconditionally stable, so no step-size refusal is needed; with `ν` = 0 it returns at once and changes nothing. The neighbours, in the order −x, +x, −y, +y, −z, +z:
- **Along the component's own axis** the neighbour faces always exist (a non-periodic axis updates faces 1 … n−1 only; its two boundary faces are held by `apply_velocity_boundaries`) or wrap (a periodic axis updates faces 0 … n−1, and face n copies face 0 after the sweeps).
- **Across, at a domain face:** periodic wraps; a **wall** gives the no-slip ghost `2U − x` (so the average on the wall is the wall's velocity), which adds `2α` to the diagonal and `2α U` to the right side; an **inflow** does the same with the ambient's velocity; an **outlet** gives the zero-gradient ghost, which adds nothing.

- [ ] **Step 1: Write the failing tests**

Append to `tests/test_airflow_advect.cpp`'s anonymous namespace, with `#include "physics/airflow/viscous.hpp"`:

```cpp
[[nodiscard]] std::vector<float> rhs_for(const MacGrid& g) {
    return std::vector<float>(std::max({g.u.size(), g.v.size(), g.w.size()}), 0.0f);
}

TEST(AirflowViscous, WithoutViscosityNothingChanges) {
    const DomainBc bc = periodic_box();
    MacGrid g = *make_grid({8, 8, 8, 0.125f}, bc);
    Lcg rng;
    for (float& x : g.u) x = rng.next();
    const MacGrid before = g;
    std::vector<float> rhs = rhs_for(g);
    diffuse(g, bc, 0.0f, 0.01f, 8, rhs);
    EXPECT_EQ(g.u, before.u);
}

// Plane Couette flow: a wall at rest at y = 0, a wall moving at 1 m/s along x
// at y = 1. The discrete steady state with the no-slip ghost is exactly
// linear, u = y.
TEST(AirflowViscous, CouetteFlowSettlesToALinearProfile) {
    DomainBc bc = periodic_box();
    bc.face[kYMinus] = bc.face[kYPlus] = FaceBc::wall;
    bc.wall_velocity[kYPlus] = glm::vec3(1.0f, 0.0f, 0.0f);
    MacGrid g = *make_grid({4, 16, 1, 1.0f / 16.0f}, bc);
    std::vector<float> rhs = rhs_for(g);
    for (int step = 0; step < 200; ++step) diffuse(g, bc, 1.0f, 1.0f, 50, rhs);
    for (uint32_t j = 0; j < 16; ++j) {
        const float expected = (static_cast<float>(j) + 0.5f) / 16.0f;
        EXPECT_NEAR(g.u[u_index(g.shape, 1, j, 0)], expected, 1.0e-4f) << j;
    }
}

// One converged implicit step damps a sine mode by exactly the discrete
// backward-Euler factor 1 / (1 + 4 alpha sin^2(pi dx / L)).
TEST(AirflowViscous, ASineModeDecaysAtTheImplicitRate) {
    const DomainBc bc = periodic_box();
    const uint32_t n = 32;
    MacGrid g = *make_grid({4, n, 1, 1.0f / static_cast<float>(n)}, bc);
    for_each_sample(g.shape, 0, [&](std::size_t idx, glm::vec3 x) {
        g.u[idx] = static_cast<float>(std::sin(2.0 * std::numbers::pi * x.y));
    });
    const std::vector<float> start = g.u;
    const float nu = 0.5f;
    const float h = 1.0f / static_cast<float>(n * n);  // alpha = nu h / dx^2 = 0.5
    std::vector<float> rhs = rhs_for(g);
    diffuse(g, bc, nu, h, 400, rhs);
    const double s = std::sin(std::numbers::pi / n);
    const double factor = 1.0 / (1.0 + 4.0 * 0.5 * s * s);
    for (std::size_t i = 0; i < start.size(); ++i) {
        EXPECT_NEAR(g.u[i], static_cast<float>(factor * start[i]), 1.0e-5f) << i;
    }
}

TEST(AirflowViscous, AMovingWallDragsTheFluidBesideIt) {
    DomainBc bc = periodic_box();
    bc.face[kYMinus] = bc.face[kYPlus] = FaceBc::wall;
    bc.wall_velocity[kYPlus] = glm::vec3(1.0f, 0.0f, 0.0f);
    MacGrid g = *make_grid({4, 8, 1, 0.125f}, bc);
    std::vector<float> rhs = rhs_for(g);
    diffuse(g, bc, 0.01f, 0.01f, 8, rhs);
    EXPECT_GT(g.u[u_index(g.shape, 1, 7, 0)], 0.0f) << "the row beside the lid";
    EXPECT_GT(g.u[u_index(g.shape, 1, 7, 0)], g.u[u_index(g.shape, 1, 6, 0)]);
}
```

- [ ] **Step 2: Run them and confirm they fail to compile**

Expected: FAIL, `physics/airflow/viscous.hpp: No such file or directory`.

- [ ] **Step 3: Write the header**

`engine/physics/airflow/viscous.hpp`:

```cpp
#pragma once

#include <cstdint>
#include <vector>

#include "physics/airflow/mac_grid.hpp"

// ---------------------------------------------------------------------------
// VISCOSITY (spec §11.3, item 1): implicit (backward Euler) diffusion of every
// velocity component, (1 + a sum) x - a sum x_nb = u*, a = nu h / dx^2, by a
// fixed number of red-black Gauss-Seidel sweeps from u*. A wall gives its
// tangential neighbour the no-slip ghost 2 U - x; an inflow the same with the
// ambient; an outlet the zero-gradient ghost; a periodic face its wrap.
// nu = 0 returns at once. `rhs` holds at least the largest component's size.
// ---------------------------------------------------------------------------
namespace spade::physics::airflow {

void diffuse(MacGrid& g, const DomainBc& bc, float nu, float h, uint32_t sweeps, std::vector<float>& rhs) noexcept;

}  // namespace spade::physics::airflow
```

- [ ] **Step 4: Write the source**

`engine/physics/airflow/viscous.cpp`:

```cpp
#include "physics/airflow/viscous.hpp"

#include <algorithm>

#include "physics/airflow/boundaries.hpp"

namespace spade::physics::airflow {

namespace {

void diffuse_component(std::vector<float>& x, uint32_t c, const GridShape& s, const DomainBc& bc, float alpha,
                       uint32_t sweeps, std::vector<float>& rhs) noexcept {
    const uint32_t n[3] = {s.nx, s.ny, s.nz};
    const uint32_t cnt[3] = {c == 0u ? s.nx + 1u : s.nx, c == 1u ? s.ny + 1u : s.ny, c == 2u ? s.nz + 1u : s.nz};
    const auto at = [&](uint32_t i, uint32_t j, uint32_t k) {
        return i + std::size_t{cnt[0]} * (j + std::size_t{cnt[1]} * k);
    };
    const bool periodic[3] = {bc.face[kXMinus] == FaceBc::periodic, bc.face[kYMinus] == FaceBc::periodic,
                              bc.face[kZMinus] == FaceBc::periodic};
    const auto cl = static_cast<glm::length_t>(c);
    std::copy(x.begin(), x.end(), rhs.begin());
    for (uint32_t sweep = 0; sweep < sweeps; ++sweep) {
        for (uint32_t colour = 0; colour < 2; ++colour) {
            for (uint32_t k = 0; k < cnt[2]; ++k) {
                for (uint32_t j = 0; j < cnt[1]; ++j) {
                    for (uint32_t i = 0; i < cnt[0]; ++i) {
                        if (((i + j + k) & 1u) != colour) continue;
                        const uint32_t idx[3] = {i, j, k};
                        const uint32_t own = idx[c];
                        // The periodic alias face, or a boundary face the velocity conditions hold.
                        if (periodic[c] ? own == n[c] : (own == 0u || own == n[c])) continue;
                        float sum = 0.0f;
                        float diag = 1.0f;
                        float extra = 0.0f;
                        for (uint32_t a = 0; a < 3; ++a) {
                            for (uint32_t dir = 0; dir < 2; ++dir) {
                                uint32_t nb[3] = {i, j, k};
                                if (a == c) {
                                    if (periodic[c] && n[c] == 1u) continue;  // its own neighbour
                                    if (dir == 0) {
                                        nb[a] = own == 0u ? n[c] - 1u : own - 1u;
                                    } else {
                                        nb[a] = (periodic[c] && own + 1u == n[c]) ? 0u : own + 1u;
                                    }
                                    sum += alpha * x[at(nb[0], nb[1], nb[2])];
                                    diag += alpha;
                                    continue;
                                }
                                const bool edge = dir == 0 ? idx[a] == 0u : idx[a] + 1u == n[a];
                                if (!edge) {
                                    nb[a] = dir == 0 ? idx[a] - 1u : idx[a] + 1u;
                                    sum += alpha * x[at(nb[0], nb[1], nb[2])];
                                    diag += alpha;
                                    continue;
                                }
                                const auto side = static_cast<Side>(2u * a + dir);
                                switch (bc.face[side]) {
                                    case FaceBc::periodic:
                                        if (n[a] > 1u) {
                                            nb[a] = dir == 0 ? n[a] - 1u : 0u;
                                            sum += alpha * x[at(nb[0], nb[1], nb[2])];
                                            diag += alpha;
                                        }
                                        break;
                                    case FaceBc::wall:  // no slip: the ghost 2 U - x
                                        diag += 2.0f * alpha;
                                        extra += 2.0f * alpha * bc.wall_velocity[side][cl];
                                        break;
                                    case FaceBc::open:
                                        if (face_kind(bc, side) == FaceKind::fixed_velocity) {  // an inflow
                                            diag += 2.0f * alpha;
                                            extra += 2.0f * alpha * bc.ambient[cl];
                                        }  // an outlet: the zero-gradient ghost, no term
                                        break;
                                }
                            }
                        }
                        const std::size_t self = at(i, j, k);
                        x[self] = ((rhs[self] + sum) + extra) / diag;
                    }
                }
            }
        }
    }
    if (periodic[c]) {  // face n is face 0
        for (uint32_t k = 0; k < cnt[2]; ++k) {
            for (uint32_t j = 0; j < cnt[1]; ++j) {
                for (uint32_t i = 0; i < cnt[0]; ++i) {
                    const uint32_t idx[3] = {i, j, k};
                    if (idx[c] != n[c]) continue;
                    uint32_t first[3] = {i, j, k};
                    first[c] = 0u;
                    x[at(i, j, k)] = x[at(first[0], first[1], first[2])];
                }
            }
        }
    }
}

}  // namespace

void diffuse(MacGrid& g, const DomainBc& bc, float nu, float h, uint32_t sweeps, std::vector<float>& rhs) noexcept {
    if (nu == 0.0f) return;
    const float alpha = nu * h / (g.shape.dx * g.shape.dx);
    diffuse_component(g.u, 0, g.shape, bc, alpha, sweeps, rhs);
    diffuse_component(g.v, 1, g.shape, bc, alpha, sweeps, rhs);
    diffuse_component(g.w, 2, g.shape, bc, alpha, sweeps, rhs);
}

}  // namespace spade::physics::airflow
```

Add `physics/airflow/viscous.cpp` to `spade_physics`.

- [ ] **Step 5: Run the tests and confirm they pass**

Run: `build-ninja\release\bin\spade_tests.exe --gtest_filter=AirflowViscous.*:AirflowAdvect.*`
Expected: 10 tests PASS.

- [ ] **Step 6: Commit**

```bash
git add engine/physics/airflow/viscous.hpp engine/physics/airflow/viscous.cpp engine/CMakeLists.txt tests/test_airflow_advect.cpp
git commit -m "feat(physics): airflow implicit viscosity with no-slip walls (airflow core, Task 7)"
```

---

### Task 8: The fluid step, and the solver core's golden

**Files:**
- Create: `engine/physics/airflow/fluid_step.hpp`, `engine/physics/airflow/fluid_step.cpp`
- Modify: `engine/CMakeLists.txt`
- Create: `tests/test_airflow_step.cpp`; modify `tests/CMakeLists.txt`

**Interfaces:**
- Consumes: Tasks 1–7.
- Produces: `struct FluidConfig { float rho = 1.225f; float nu = 0.0f; float h = 0.0f; uint32_t n_v = 2; uint32_t visc_sweeps = 8; }`, `struct FluidScratch { MacGrid next; AdvectScratch advect; ProjectionScratch projection; std::vector<float> rhs, du, dv, dw; }`, `make_fluid_scratch(const MacGrid&, const FluidConfig&) -> Result<FluidScratch>`, `fluid_step(MacGrid&, const DomainBc&, const FluidConfig&, FluidScratch&) noexcept`, and `digest(const MacGrid&) -> uint64_t` (test-side helper in `test_airflow_step.cpp`, reused by Task 11).

**One fluid step** (spec §1.5): the velocity conditions; advection into the scratch grid, swapped in; the sources (`du`, `dv`, `dw`, velocity increments a source wrote, Task 9) added and cleared; the velocity conditions; if `ν` > 0, the viscous solve and the conditions again; the projection. Nothing runs after the projection: an outlet's projected face must keep its correction, and the projection keeps periodic faces equal itself. `p` stays in the grid as the next step's warm start.

**The golden** (`TD-1`, `TD-12`): a 16³ lid-driven cavity, 64 steps, folded by FNV-1a over the bytes of `u`, `v`, `w`, `p`. It pins the solver core's arithmetic across MSVC and gcc before any engine integration. Its digest is captured from the first MSVC release run, must equal on MSVC debug, and is final only when a fresh `-NoSeed` Docker leg reproduces it.

- [ ] **Step 1: Write the failing tests**

`tests/test_airflow_step.cpp`:

```cpp
// The airflow solver core's fluid step and its golden (airflow core plan,
// Task 8).
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <limits>
#include <vector>

#include "physics/airflow/fluid_step.hpp"

namespace {

using namespace spade::physics::airflow;

struct Lcg {
    uint32_t state = 12345u;
    float next() noexcept {
        state = state * 1664525u + 1013904223u;
        return static_cast<float>(state >> 8) * (2.0f / 16777216.0f) - 1.0f;
    }
};

[[nodiscard]] uint64_t digest(const MacGrid& g) {
    uint64_t h = 1469598103934665603ull;
    const auto fold = [&](const std::vector<float>& v) {
        const auto* bytes = reinterpret_cast<const unsigned char*>(v.data());
        for (std::size_t i = 0; i < v.size() * sizeof(float); ++i) {
            h ^= bytes[i];
            h *= 1099511628211ull;
        }
    };
    fold(g.u);
    fold(g.v);
    fold(g.w);
    fold(g.p);
    return h;
}

[[nodiscard]] DomainBc cavity_bc() {
    DomainBc bc;
    bc.face.fill(FaceBc::wall);
    bc.wall_velocity[kYPlus] = glm::vec3(1.0f, 0.0f, 0.0f);
    return bc;
}

TEST(AirflowStep, TheConfigurationIsChecked) {
    const MacGrid g = *make_grid({4, 4, 4, 0.25f}, DomainBc{});
    const auto refused = [&](FluidConfig cfg, const char* what) {
        const auto s = make_fluid_scratch(g, cfg);
        if (s.has_value()) return testing::AssertionFailure() << "accepted";
        if (s.error().context.find(what) == std::string::npos) return testing::AssertionFailure() << s.error().context;
        return testing::AssertionSuccess();
    };
    EXPECT_TRUE(refused({.rho = 0.0f, .h = 0.01f}, "rho"));
    EXPECT_TRUE(refused({.h = 0.0f}, "h"));
    EXPECT_TRUE(refused({.h = std::numeric_limits<float>::infinity()}, "h"));
    EXPECT_TRUE(refused({.nu = -1.0f, .h = 0.01f}, "nu"));
    EXPECT_TRUE(refused({.h = 0.01f, .n_v = 0}, "n_v"));
    EXPECT_TRUE(refused({.nu = 0.01f, .h = 0.01f, .visc_sweeps = 0}, "visc_sweeps"));
    EXPECT_TRUE(make_fluid_scratch(g, {.h = 0.01f}).has_value());
}

TEST(AirflowStep, AUniformFlowThroughAPeriodicBoxIsExact) {
    DomainBc bc;
    bc.face.fill(FaceBc::periodic);
    MacGrid g = *make_grid({8, 8, 8, 0.125f}, bc);
    std::fill(g.u.begin(), g.u.end(), 1.0f);
    std::fill(g.v.begin(), g.v.end(), 0.5f);
    const MacGrid start = g;
    // The lean (inviscid) mode: the implicit viscous solve's fp32 rounding may
    // move a uniform field by an ulp, so viscosity is checked in AirflowViscous.
    const FluidConfig cfg{.h = 0.01f};
    FluidScratch s = *make_fluid_scratch(g, cfg);
    for (int n = 0; n < 10; ++n) fluid_step(g, bc, cfg, s);
    EXPECT_EQ(g.u, start.u);
    EXPECT_EQ(g.v, start.v);
    EXPECT_EQ(g.w, start.w);
    for (float p : g.p) EXPECT_EQ(p, 0.0f);
}

TEST(AirflowStep, TwoRunsAreBitIdentical) {
    const auto run = [] {
        const DomainBc bc = cavity_bc();
        MacGrid g = *make_grid({12, 12, 12, 1.0f / 12.0f}, bc);
        Lcg rng;
        for (float& x : g.u) x = 0.1f * rng.next();
        for (float& x : g.v) x = 0.1f * rng.next();
        for (float& x : g.w) x = 0.1f * rng.next();
        const FluidConfig cfg{.rho = 1.0f, .nu = 0.01f, .h = 0.5f / 12.0f};
        FluidScratch s = *make_fluid_scratch(g, cfg);
        for (int n = 0; n < 32; ++n) fluid_step(g, bc, cfg, s);
        return digest(g);
    };
    EXPECT_EQ(run(), run());
}

// THE SOLVER CORE'S GOLDEN (TD-1, TD-12). A 16^3 lid-driven cavity at Re 100,
// 64 steps at CFL 0.5, folded by FNV-1a over u, v, w and p.
//   generated  <date of the first run> by this test, MSVC release, at <commit>
//   cross-check MSVC debug agrees; the gcc-13 Docker leg (TD-12): <the leg>
// A change to any value here is a golden regeneration under TD-1, with its
// reason in the commit.
constexpr uint64_t kSolverCoreDigest = 0x0000000000000000ull;  // Step 4 records the first run's value

TEST(AirflowStep, TheSolverCoresDigestIsPinned) {
    const DomainBc bc = cavity_bc();
    MacGrid g = *make_grid({16, 16, 16, 1.0f / 16.0f}, bc);
    const FluidConfig cfg{.rho = 1.0f, .nu = 0.01f, .h = 0.5f / 16.0f, .n_v = 2, .visc_sweeps = 8};
    FluidScratch s = *make_fluid_scratch(g, cfg);
    for (int n = 0; n < 64; ++n) fluid_step(g, bc, cfg, s);
    for (float x : g.u) ASSERT_TRUE(std::isfinite(x));
    const uint64_t d = digest(g);
    std::printf("airflow solver core digest: 0x%016llx\n", static_cast<unsigned long long>(d));
    EXPECT_EQ(d, kSolverCoreDigest);
}

}  // namespace
```

- [ ] **Step 2: Register the test file and confirm it fails to compile**

Add `test_airflow_step.cpp` after `test_airflow_advect.cpp`. Expected: FAIL, `physics/airflow/fluid_step.hpp: No such file or directory`.

- [ ] **Step 3: Write the header and the source**

`engine/physics/airflow/fluid_step.hpp`:

```cpp
#pragma once

#include <cstdint>
#include <vector>

#include "core/error.hpp"
#include "physics/airflow/advect.hpp"
#include "physics/airflow/boundaries.hpp"
#include "physics/airflow/projection.hpp"
#include "physics/airflow/viscous.hpp"

// ---------------------------------------------------------------------------
// ONE FLUID STEP of the real-time tier (spec §1.5): conditions, advection,
// sources, conditions, viscosity (if nu > 0) and conditions, projection.
// Allocation-free: everything it touches is sized by make_fluid_scratch().
// ---------------------------------------------------------------------------
namespace spade::physics::airflow {

struct FluidConfig {
    float rho = 1.225f;        // kg/m^3
    float nu = 0.0f;           // m^2/s, molecular; 0 is the inviscid lean mode
    float h = 0.0f;            // the fluid step, s
    uint32_t n_v = 2;          // V-cycles per projection
    uint32_t visc_sweeps = 8;  // red-black sweeps per viscous solve
};

struct FluidScratch {
    MacGrid next;                    // advection's output, swapped in
    AdvectScratch advect;
    ProjectionScratch projection;
    std::vector<float> rhs;          // the viscous solve's right side
    std::vector<float> du, dv, dw;   // velocity increments a source wrote this step; cleared when added
};

// Refused (invalid_argument, naming the field): rho or h not positive and
// finite; nu negative or not finite; n_v = 0; visc_sweeps = 0 with nu > 0.
[[nodiscard]] Result<FluidScratch> make_fluid_scratch(const MacGrid& g, const FluidConfig& cfg);

void fluid_step(MacGrid& g, const DomainBc& bc, const FluidConfig& cfg, FluidScratch& s) noexcept;

}  // namespace spade::physics::airflow
```

`engine/physics/airflow/fluid_step.cpp`:

```cpp
#include "physics/airflow/fluid_step.hpp"

#include <algorithm>
#include <cmath>
#include <string>

namespace spade::physics::airflow {

namespace {

void add_and_clear(std::vector<float>& vel, std::vector<float>& d) noexcept {
    for (std::size_t i = 0; i < vel.size(); ++i) {
        vel[i] += d[i];
        d[i] = 0.0f;
    }
}

[[nodiscard]] bool positive_finite(float x) noexcept { return x > 0.0f && std::isfinite(x); }

}  // namespace

Result<FluidScratch> make_fluid_scratch(const MacGrid& g, const FluidConfig& cfg) {
    const auto refuse = [](const char* what) {
        return std::unexpected(Error{Code::invalid_argument, std::string("airflow fluid step: ") + what});
    };
    if (!positive_finite(cfg.rho)) return refuse("rho must be positive and finite");
    if (!positive_finite(cfg.h)) return refuse("h must be positive and finite");
    if (!(cfg.nu >= 0.0f) || !std::isfinite(cfg.nu)) return refuse("nu must be non-negative and finite");
    if (cfg.n_v == 0u) return refuse("n_v must be at least 1");
    if (cfg.nu > 0.0f && cfg.visc_sweeps == 0u) return refuse("visc_sweeps must be at least 1 when nu > 0");
    FluidScratch s;
    s.next = g;
    s.advect = make_advect_scratch(g.shape);
    s.projection = make_projection_scratch(g.shape);
    s.rhs.assign(std::max({g.u.size(), g.v.size(), g.w.size()}), 0.0f);
    s.du.assign(g.u.size(), 0.0f);
    s.dv.assign(g.v.size(), 0.0f);
    s.dw.assign(g.w.size(), 0.0f);
    return s;
}

void fluid_step(MacGrid& g, const DomainBc& bc, const FluidConfig& cfg, FluidScratch& s) noexcept {
    apply_velocity_boundaries(g, bc);
    advect_velocity(g, s.next, bc, cfg.h, s.advect);
    g.u.swap(s.next.u);
    g.v.swap(s.next.v);
    g.w.swap(s.next.w);
    add_and_clear(g.u, s.du);
    add_and_clear(g.v, s.dv);
    add_and_clear(g.w, s.dw);
    apply_velocity_boundaries(g, bc);
    if (cfg.nu > 0.0f) {
        diffuse(g, bc, cfg.nu, cfg.h, cfg.visc_sweeps, s.rhs);
        apply_velocity_boundaries(g, bc);
    }
    project(g, bc, cfg.rho, cfg.h, cfg.n_v, s.projection);
}

}  // namespace spade::physics::airflow
```

Add `physics/airflow/fluid_step.cpp` to `spade_physics`.

- [ ] **Step 4: Run, capture the golden, run again**

Run: `build-ninja\release\bin\spade_tests.exe --gtest_filter=AirflowStep.*`
Expected: 3 PASS, and `TheSolverCoresDigestIsPinned` FAILS against the zero placeholder, printing `airflow solver core digest: 0x…`. Copy that value into `kSolverCoreDigest`, fill the provenance block's date and commit, rebuild, and run again in **both** presets: 4 PASS in each. A debug–release mismatch is a determinism defect (an expression the optimiser is reordering), to be found and fixed, never pinned per preset.

- [ ] **Step 5: The gate, with a fresh leg**

Run the task gate (both presets, gcc check on every changed file), then `scripts\docker-leg.ps1 -NoSeed -Memory 8g -Jobs 8` at the task's head. Expected: PASS, with `AirflowStep.TheSolverCoresDigestIsPinned` passing on gcc-13. Write the leg's directory into the provenance block (`TD-12`) and commit that.

- [ ] **Step 6: Commit**

```bash
git add engine/physics/airflow/fluid_step.hpp engine/physics/airflow/fluid_step.cpp engine/CMakeLists.txt tests/test_airflow_step.cpp tests/CMakeLists.txt
git commit -m "feat(physics): the airflow fluid step and the solver core's golden (airflow core, Task 8; TD-1)"
```

---

