# Airflow Solids P2 Implementation Plan — walls, sheets, the solver with solids

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Immersed solid walls and labelled sheets in plan A's CPU airflow reference: every part reaches the grid through one sampler, the projection holds the walls with face apertures under CG preconditioned by a symmetric V-cycle, each closed fluid region keeps its own gauge, the viscous solve holds no-slip on the same surface, and every part the grid cannot resolve is announced, sheeted or refused by name.

**Architecture:**
- **Solids as data.** `engine/physics/airflow/solids.{hpp,cpp}` builds a `SolidField`: per velocity sample, θ, the wall velocity, φ and the viscous wall links. It is built from a `SolidSampler` callback and a per-part `SolidPart` list. The airflow core never includes `vehicles/`, which links it. The tests, and later the D1 demo and plan C's module, adapt `vehicles::sample_parts()` and Core's `part_bounds()` to it.
- **The solids path is separate.** Every changed function keeps plan A's code untouched when no solid exists, so both goldens stay pinned. With solids:
  - the pressure levels carry face weights;
  - the projection solves by `solve_pcg()`;
  - the closed regions keep their own means (`regions.{hpp,cpp}`);
  - the viscous solve applies the wall links.
- **Host calls and step calls.** `set_solids()` is a host call that allocates. `fluid_step()` stays allocation-free.

**Tech Stack:** C++23, glm, GoogleTest through ctest, MSVC + Ninja presets, the gcc-13 Docker leg.

**Spec:** `2026-10-13-airflow-solids-design.md` (approved 2026-10-13; amended b0fdfcc, 9942694, b4ec550): §2–§4, §8, §10–§11. Also spec `2026-10-05-airflow-design.md` §1.5 (amended: CG with solids) and §2, and Test/Docs' `../../test-docs/2026-10-13-solids-tolerances.md` (S1–S5). Decisions: `PHY-11`.

**What this plan is, and is not.**
- **It is P2:** walls, sheets, the measured safeguard, the solver with solids, the fluid regions, the viscous wall stencil, and design tests 1–3, 5–7 and 9–11, each in its P2 part.
- **It is not:**
  - P3: the force and moment, `C_p`, test 1's added mass, test 3's force, and test 8 with Publish's one-sided differences;
  - porous parts (test 4: `classify_part()` refuses them by name until then);
  - P2b: the drone box's boundaries and the floor-height refusal.
- **When it merges:** after Core's `part_bounds()` merges (the lead, 2026-10-13). Task 10 switches the tests' adapter to it.

## Global Constraints

- **fp32 throughout**, in the order the code states; no reassociation. The project's compiler flags (no FMA contraction); add none.
- **No libm transcendental in `engine/`** (`TD-3`): `std::sqrt`, `std::floor`, `std::fabs`, `std::isfinite` and `std::min`/`max` only. The tests' analytic shapes use `std::sqrt` only.
- **Plan A's paths are untouched when no solid exists:** `kSolverCoreDigest` and the nine-config golden stay pinned (design test 6, `TD-1`).
- **Determinism:**
  - loops run `k` outermost and `i` innermost, components in the order u, v, w, and a stencil's faces in the order −x, +x, −y, +y, −z, +z;
  - red–black sweeps stay order-free within a colour;
  - every reduction that feeds stepping uses `reduce.hpp`'s fixed tiles, and every max is order-free;
  - the region labelling is a flood fill in array order.
- **No allocation in a step.** `make_solid_field()`, `set_solids()` and `label_regions()` are host calls and may allocate. `fluid_step()`, `project()` and `diffuse()` must not.
- **Refuse, never skip** (`L6`): every part the grid cannot hold is refused by name, or sheeted or announced on the run line. A closed region with a net prescribed flux is a fault naming its cell and faces. A pressure solve that leaves a relative residual above 10⁻³ is announced.
- **Layering:** `engine/physics/airflow/` includes only `core/` headers, glm and the standard library. Tests may include `vehicles/`. No bounds computation goes into `engine/` (TD-9; the lead): the tests compute their shapes' bounds until Core's `part_bounds()` lands.
- **Tolerances against references are Test/Docs'**: S1–S5 in `test-docs/2026-10-13-solids-tolerances.md`, cited in each record's `tolerance:` line. Implementation bands (S3's convergence, test 7's apertures) are `TD-2`, measured and pinned, with the measurement in the test's comment.
- **Mutations.** Each check that could pass on a wrong solver has a named mutation. It is applied by hand once, the test is seen to fail, the mutation is removed, and the result goes in the ledger (the lead's finding 4).
- **Where the code lives:** `../spade-wt/physics`, branch `physics/airflow-solids`, cut from master.
- **The task gate**, at the end of every task, in the foreground, each heavy chunk only with ≥ 6 GB free memory and ≥ 6 GB free commit:
  1. `scripts\build.ps1 -Preset release -Target spade_tests` and `-Preset debug -Target spade_tests`;
  2. ctest, release and debug, serially in `-I` chunks under 10 minutes, with `--output-on-failure`: 0 failed, with the skips named;
  3. `bash scripts/gcc-check.sh --compile <every changed .cpp>`, and a plain check on every changed header;
  4. Tasks 6 and 10 also run a Docker leg at the task's head. A leg passes only when its `summary.txt` reads `docker leg: PASS`.
- **Running one suite during a task:** `scripts\build.ps1 -Preset release -Target spade_tests -BuildDir build-ninja/release`, then `build-ninja\release\bin\spade_tests.exe --gtest_filter=Suite.*`. Heavy new tests are timed under ctest in debug, against its 60 s limit.

## Review Focus

The input classes the design implies that no task's main tests meet, most likely first. Each has its test in the task that owns the code.

1. **The drone turning:** `set_solids()` is called every step with a new pose. A rebuild must leave no aperture, link or region from the last pose, and must not grow memory. Test: Task 6, `AirflowSolidsStep.ARebuildLeavesNothingOfTheLastPose`.
2. **A sampler that misbehaves** (a mesh query returning NaN, a part index past the list, a non-finite velocity): refused by name at build, never written into θ. Test: Task 1, `AirflowSolids.ABadSamplerIsRefusedByName`.
3. **A part posed wholly outside the grid** (a bad mount, a unit slip): refused by name, never silently empty. A part reaching past the grid's edge is clipped, since validation walls fill to the edge; keeping the drone inside its box is P2b's check. Tests: Task 1, `AirflowSolids.APartWhollyOutsideTheGridIsRefused` and `AirflowSolids.APartReachingPastTheGridIsClipped`.
4. **Every part removed again** (a scene unloading its drone): the step returns to plan A's path, bit for bit. Test: Task 6, `AirflowSolidsStep.NoPartsIsPlanABitForBit`.
5. **A pocket sealed by a sheet** (a hollow canopy, a closed frame): a closed region of a few cells keeps its own gauge and does not drift over many steps. Test: Task 6, `AirflowSolidsStep.ASealedPocketKeepsItsGaugeOverManySteps`.

## Files

| File | Responsibility | Task |
|---|---|---|
| `engine/physics/airflow/solids.hpp/.cpp` (new) | `SolidQuery`, `SolidSampler`, `SolidPart`, `classify_part()`, `SolidField`, `make_solid_field()`, `build_solids()`, the measured safeguard, `apply_solid_velocities()` | 1, 2, 7, 8 |
| `engine/physics/airflow/pressure.hpp/.cpp` | `PoissonLevel::w` (face weights), the weighted stencil, `apply_operator()`, reversed colour order | 3 |
| `engine/physics/airflow/multigrid.hpp/.cpp` | `set_weights()`, `clear_weights()`, `restrict_full_weighting()`, `precondition()` (the symmetric V(2,2)) | 3, 4 |
| `engine/physics/airflow/reduce.hpp/.cpp` | `fixed_tile_dot()` | 4 |
| `engine/physics/airflow/pcg.hpp/.cpp` (new) | `PcgScratch`, `solve_pcg()` | 4 |
| `engine/physics/airflow/regions.hpp/.cpp` (new) | `FluidRegions`, `label_regions()`, `check_regions()`, `remove_region_means()`, `describe_regions()` | 5 |
| `engine/physics/airflow/projection.hpp/.cpp` | the solids path: the aperture divergence, `solve_pcg`, closed faces take the wall velocity, `ProjectionReport` | 6 |
| `engine/physics/airflow/fluid_step.hpp/.cpp` | `FluidConfig::interior`, `set_solids()`, `clear_solids()`, the solids path of the step, the residual on `FluidHealth`, `ill_posed_region` | 6, 7 |
| `engine/physics/airflow/viscous.hpp/.cpp` | the wall links and fixed samples | 8 |
| `engine/CMakeLists.txt` | the three new sources in `spade_physics` | 1, 4, 5 |
| `tests/airflow_solid_shapes.hpp` (new) | analytic test shapes, their sampler and their bounds (tests only, TD-9) | 1 |
| `tests/test_airflow_solids.cpp` (new) | the field, the classifier, the safeguard, the links | 1, 2, 8 |
| `tests/test_airflow_solids_solve.cpp` (new) | the weighted operator, the preconditioner, PCG, the regions | 3, 4, 5 |
| `tests/test_airflow_solids_step.cpp` (new) | S3, S1's P2 checks, S2, rebuilds, no-solid bit identity, the posed drone | 6, 7, 8, 9 |
| `tests/CMakeLists.txt` | the three new test files | 1, 3, 6 |

---
### Task 1: The solid field — the classifier, walls and sheets

**Files:**
- Create: `engine/physics/airflow/solids.hpp`, `engine/physics/airflow/solids.cpp`
- Modify: `engine/CMakeLists.txt` (the `spade_physics` sources: `physics/airflow/solids.cpp` after `physics/airflow/actuator.cpp`)
- Create: `tests/airflow_solid_shapes.hpp`, `tests/test_airflow_solids.cpp`
- Modify: `tests/CMakeLists.txt` (`test_airflow_solids.cpp` after `test_airflow_publish.cpp`)

**Interfaces:**
- Consumes: `GridShape`, `Result`, `Error`, `Code` (plan A).
- Produces, for every later task:
  - `kNoSolidPart`, `kNoWallLinks`
  - `struct SolidQuery { float distance; glm::vec3 gradient; uint32_t part; glm::vec3 velocity; }`
  - `struct SolidSampler { const void* ctx; SolidQuery (*nearest)(const void*, glm::vec3) noexcept; SolidQuery (*part)(const void*, uint32_t, glm::vec3) noexcept; }`
  - `struct SolidPart { std::string name; bool porous; float min_feature; glm::vec3 extent; glm::vec3 box_min, box_max; }`
  - `enum class SolidRole : uint8_t { wall, sheet }`, `struct SolidClass { SolidRole role; float thickness; }`
  - `Result<SolidClass> classify_part(const SolidPart&, float dx)`
  - `struct WallLinks { std::array<float, 6> inv_f; std::array<float, 6> wall; }`
  - `struct SolidRange { std::array<uint32_t, 3> lo, hi; bool empty; }`
  - `struct SolidField { bool any; std::array<std::vector<float>, 3> theta, wall, phi; std::array<std::vector<uint32_t>, 3> link; std::vector<WallLinks> links; std::array<std::vector<uint32_t>, 3> fill1, fill2; std::array<SolidRange, 3> range; std::vector<SolidClass> classes; std::string run_line; }`
  - `Result<SolidField> make_solid_field(const GridShape&)`
  - `Result<void> build_solids(SolidField&, const GridShape&, std::array<bool, 3> periodic, std::span<const SolidPart>, const SolidSampler&)`
  - `[[nodiscard]] inline bool closed(float theta)` (θ == 0)

- [ ] **Step 1: The test shapes (tests only; TD-9)**

Create `tests/airflow_solid_shapes.hpp`:

```cpp
#pragma once
// Analytic solids for the airflow solids tests: exact signed distances (or,
// for a wedge, a 1-Lipschitz bound), their gradients, rigid velocities, and
// their bounding boxes. Bounds live here, in the tests, until Core's
// vehicles::part_bounds() lands (TD-9; the lead, 2026-10-13).
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

#include <glm/glm.hpp>

#include "physics/airflow/solids.hpp"

namespace airflow_test {

using spade::physics::airflow::kNoSolidPart;
using spade::physics::airflow::SolidPart;
using spade::physics::airflow::SolidQuery;
using spade::physics::airflow::SolidSampler;

enum class Kind { sphere, box, cylinder_y, outside_cylinder_y, wedge_y };

// One shape in the grid's frame. `to_local` turns a point's offset from the
// centre into the shape's own axes (a pure rotation; identity by default).
struct Shape {
    std::string name = "part";
    Kind kind = Kind::sphere;
    glm::vec3 centre{0.0f};
    glm::vec3 half{0.0f};          // box: half extents; wedge: (tip-to-back length, half height in y, 0)
    float radius = 0.0f;           // sphere, cylinder: radius; wedge: half-angle, radians
    glm::mat3 to_local{1.0f};      // local = to_local * (x - centre)
    glm::vec3 omega{0.0f};         // rigid rotation about `centre`, rad/s
    float min_feature = 0.0f;      // declared, m
    bool porous = false;
};

struct Field {
    float d;
    glm::vec3 g;  // grid frame, unit
};

// The box's exact distance and gradient in local coordinates.
[[nodiscard]] inline Field box_field(glm::vec3 p, glm::vec3 half) {
    const glm::vec3 q = glm::abs(p) - half;
    const glm::vec3 out = glm::max(q, glm::vec3(0.0f));
    const float out_len = std::sqrt(glm::dot(out, out));
    if (out_len > 0.0f) {
        glm::vec3 g = out / out_len;
        for (int a = 0; a < 3; ++a) g[a] = p[a] < 0.0f ? -g[a] : g[a];
        return {out_len, g};
    }
    int a = 0;
    if (q.y > q[a]) a = 1;
    if (q.z > q[a]) a = 2;
    glm::vec3 g(0.0f);
    g[a] = p[a] < 0.0f ? -1.0f : 1.0f;
    return {q[a], g};
}

[[nodiscard]] inline Field shape_field(const Shape& s, glm::vec3 x) {
    const glm::vec3 p = s.to_local * (x - s.centre);
    const glm::mat3 to_grid = glm::transpose(s.to_local);
    Field f{};
    switch (s.kind) {
        case Kind::sphere: {
            const float r = std::sqrt(glm::dot(p, p));
            f = {r - s.radius, r > 0.0f ? p / r : glm::vec3(1.0f, 0.0f, 0.0f)};
            break;
        }
        case Kind::box:
            f = box_field(p, s.half);
            break;
        case Kind::cylinder_y: {  // infinite along local y
            const float r = std::sqrt(p.x * p.x + p.z * p.z);
            f = {r - s.radius, r > 0.0f ? glm::vec3(p.x / r, 0.0f, p.z / r) : glm::vec3(1.0f, 0.0f, 0.0f)};
            break;
        }
        case Kind::outside_cylinder_y: {  // solid where r > radius (a fixed tube's wall)
            const float r = std::sqrt(p.x * p.x + p.z * p.z);
            f = {s.radius - r, r > 0.0f ? glm::vec3(-p.x / r, 0.0f, -p.z / r) : glm::vec3(1.0f, 0.0f, 0.0f)};
            break;
        }
        case Kind::wedge_y: {
            // A wedge along local +x from its tip at the centre: |z| <= x tan(a),
            // 0 <= x <= L, |y| <= H, the intersection of half-spaces. The max
            // of their signed plane distances is a 1-Lipschitz bound (exact
            // inside and on the faces), which sphere tracing needs.
            const float ca = std::cos(s.radius);  // test-only libm: not engine code
            const float sa = std::sin(s.radius);
            const Field planes[5] = {
                {p.z * ca - p.x * sa, glm::vec3(-sa, 0.0f, ca)},   // the +z face
                {-p.z * ca - p.x * sa, glm::vec3(-sa, 0.0f, -ca)}, // the -z face
                {p.x - s.half.x, glm::vec3(1.0f, 0.0f, 0.0f)},     // the back
                {p.y - s.half.y, glm::vec3(0.0f, 1.0f, 0.0f)},
                {-p.y - s.half.y, glm::vec3(0.0f, -1.0f, 0.0f)},
            };
            f = planes[0];
            for (const Field& pl : planes) {
                if (pl.d > f.d) f = pl;
            }
            break;
        }
    }
    f.g = to_grid * f.g;
    return f;
}

[[nodiscard]] inline glm::vec3 shape_velocity(const Shape& s, glm::vec3 x) {
    return glm::cross(s.omega, x - s.centre);
}

// The shape's grid-frame bounding box and part-frame extent.
[[nodiscard]] inline SolidPart part_of(const Shape& s) {
    glm::vec3 local_half(0.0f);
    switch (s.kind) {
        case Kind::sphere: local_half = glm::vec3(s.radius); break;
        case Kind::box: local_half = s.half; break;
        case Kind::cylinder_y: local_half = glm::vec3(s.radius, 1.0e3f, s.radius); break;
        case Kind::outside_cylinder_y: local_half = glm::vec3(1.0e3f); break;
        case Kind::wedge_y: local_half = glm::vec3(s.half.x, s.half.y, s.half.x * std::tan(s.radius)); break;
    }
    SolidPart p;
    p.name = s.name;
    p.porous = s.porous;
    p.min_feature = s.min_feature;
    p.extent = 2.0f * local_half;
    // The rotated box's eight corners (the wedge's box from its tip).
    const glm::mat3 to_grid = glm::transpose(s.to_local);
    const glm::vec3 lo_local = s.kind == Kind::wedge_y ? glm::vec3(0.0f, -local_half.y, -local_half.z) : -local_half;
    const glm::vec3 hi_local = s.kind == Kind::wedge_y ? glm::vec3(s.half.x, local_half.y, local_half.z) : local_half;
    if (s.kind == Kind::wedge_y) p.extent = hi_local - lo_local;
    glm::vec3 lo(std::numeric_limits<float>::max());
    glm::vec3 hi(-std::numeric_limits<float>::max());
    for (int c = 0; c < 8; ++c) {
        const glm::vec3 corner((c & 1) ? hi_local.x : lo_local.x, (c & 2) ? hi_local.y : lo_local.y,
                               (c & 4) ? hi_local.z : lo_local.z);
        const glm::vec3 x = s.centre + to_grid * corner;
        lo = glm::min(lo, x);
        hi = glm::max(hi, x);
    }
    p.box_min = lo;
    p.box_max = hi;
    return p;
}

// A scene of shapes, sampled as sample_parts() does: the smallest distance
// wins, a tie to the lower index.
struct Scene {
    std::vector<Shape> shapes;

    [[nodiscard]] std::vector<SolidPart> parts() const {
        std::vector<SolidPart> out;
        for (const Shape& s : shapes) out.push_back(part_of(s));
        return out;
    }

    static SolidQuery nearest(const void* ctx, glm::vec3 x) noexcept {
        const auto& scene = *static_cast<const Scene*>(ctx);
        SolidQuery q;
        for (uint32_t i = 0; i < scene.shapes.size(); ++i) {
            const Field f = shape_field(scene.shapes[i], x);
            if (q.part == kNoSolidPart || f.d < q.distance) {
                q = SolidQuery{f.d, f.g, i, shape_velocity(scene.shapes[i], x)};
            }
        }
        return q;
    }

    static SolidQuery one(const void* ctx, uint32_t part, glm::vec3 x) noexcept {
        const auto& scene = *static_cast<const Scene*>(ctx);
        const Field f = shape_field(scene.shapes[part], x);
        return SolidQuery{f.d, f.g, part, shape_velocity(scene.shapes[part], x)};
    }

    [[nodiscard]] SolidSampler sampler() const { return SolidSampler{this, &Scene::nearest, &Scene::one}; }
};

}  // namespace airflow_test
```

- [ ] **Step 2: Write the failing tests**

Create `tests/test_airflow_solids.cpp`:

```cpp
// The solid field (the solids design, sections 2-4; P2's Tasks 1, 2 and 8):
// apertures from one sampler, labelled sheets, the classifier, the measured
// safeguard and the viscous wall links. Shapes and bounds are analytic
// (tests/airflow_solid_shapes.hpp); expectations are worked from the
// geometry, not read from the code (TD-4).
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

#include "airflow_solid_shapes.hpp"
#include "physics/airflow/solids.hpp"

namespace {

using namespace spade::physics::airflow;
using airflow_test::Kind;
using airflow_test::Scene;
using airflow_test::Shape;

[[nodiscard]] SolidField built(const GridShape& shape, const Scene& scene, std::array<bool, 3> periodic = {}) {
    SolidField f = *make_solid_field(shape);
    const std::vector<SolidPart> parts = scene.parts();
    const Result<void> ok = build_solids(f, shape, periodic, parts, scene.sampler());
    EXPECT_TRUE(ok.has_value()) << (ok ? "" : ok.error().context);
    return f;
}

[[nodiscard]] float theta_at(const SolidField& f, const GridShape& s, uint32_t c, uint32_t i, uint32_t j, uint32_t k) {
    const std::size_t idx = c == 0 ? u_index(s, i, j, k) : (c == 1 ? v_index(s, i, j, k) : w_index(s, i, j, k));
    return f.theta[c][idx];
}

TEST(AirflowSolids, TheClassifierSortsWallsSheetsAndWires) {
    const float dx = 0.01f;
    SolidPart wall{.name = "hull", .porous = false, .min_feature = 0.03f, .extent = {0.10f, 0.05f, 0.04f}};
    const auto w = classify_part(wall, dx);
    ASSERT_TRUE(w.has_value());
    EXPECT_EQ(w->role, SolidRole::wall);

    SolidPart plate{.name = "frame_top", .porous = false, .min_feature = 0.002f, .extent = {0.10f, 0.002f, 0.10f}};
    const auto p = classify_part(plate, dx);
    ASSERT_TRUE(p.has_value());
    EXPECT_EQ(p->role, SolidRole::sheet);
    EXPECT_EQ(p->thickness, 0.002f);

    // Declared thick, but its bounds are a 4 mm plate: the extent catches it.
    SolidPart misdeclared{.name = "board", .porous = false, .min_feature = 0.05f, .extent = {0.10f, 0.004f, 0.10f}};
    const auto m = classify_part(misdeclared, dx);
    ASSERT_TRUE(m.has_value());
    EXPECT_EQ(m->role, SolidRole::sheet);
    EXPECT_EQ(m->thickness, 0.004f);

    SolidPart wire{.name = "antenna", .porous = false, .min_feature = 0.002f, .extent = {0.002f, 0.003f, 0.20f}};
    const auto r = classify_part(wire, dx);
    ASSERT_FALSE(r.has_value());
    EXPECT_NE(r.error().context.find("antenna"), std::string::npos) << r.error().context;
    EXPECT_NE(r.error().context.find("needs dx <= 1.0 mm"), std::string::npos) << r.error().context;

    SolidPart screen{.name = "screen", .porous = true, .min_feature = 0.03f, .extent = {0.1f, 0.1f, 0.1f}};
    const auto s = classify_part(screen, dx);
    ASSERT_FALSE(s.has_value());
    EXPECT_NE(s.error().context.find("porous"), std::string::npos) << s.error().context;
}

// A sphere of radius 4 cells centred on a cell corner: every sampled face takes
// the ramp's aperture clamp(0.5 + phi / dx), mirror-symmetric on all three
// axes, and the faces far from it stay open.
TEST(AirflowSolids, ASphereGivesTheRampsApertureAndIsMirrorSymmetric) {
    const GridShape shape{16, 16, 16, 1.0f};
    Scene scene;
    scene.shapes.push_back(Shape{.name = "ball", .kind = Kind::sphere, .centre = {8.0f, 8.0f, 8.0f}, .radius = 4.0f,
                                 .min_feature = 8.0f});
    const SolidField f = built(shape, scene);
    EXPECT_TRUE(f.any);
    uint32_t closed_faces = 0;
    for (uint32_t c = 0; c < 3; ++c) {
        for_each_sample(shape, c, [&](std::size_t idx, glm::vec3 x) {
            const glm::vec3 d = x - glm::vec3(8.0f);
            const float phi = std::sqrt(glm::dot(d, d)) - 4.0f;
            const float expected = std::min(std::max(0.5f + phi, 0.0f), 1.0f);
            EXPECT_EQ(f.theta[c][idx], expected) << c << " " << idx;
            if (f.theta[c][idx] == 0.0f) ++closed_faces;
            EXPECT_EQ(f.wall[c][idx], 0.0f);
        });
    }
    EXPECT_GT(closed_faces, 0u);
    for (uint32_t k = 0; k < 16; ++k) {
        for (uint32_t j = 0; j < 16; ++j) {
            for (uint32_t i = 0; i <= 16; ++i) {
                EXPECT_EQ(theta_at(f, shape, 0, i, j, k), theta_at(f, shape, 0, 16 - i, j, k));
                EXPECT_EQ(theta_at(f, shape, 0, i, j, k), theta_at(f, shape, 0, i, 15 - j, k));
            }
        }
    }
}

// A sheet 0.25 dx thick across a periodic channel closes exactly one plane of
// u faces: the one it lies on, or, on a cell-centre plane, the lower of the
// two (the half-open rule), and no v or w face.
TEST(AirflowSolids, ASheetClosesExactlyOneFacePlane) {
    const GridShape shape{32, 8, 8, 1.0f};
    for (const float at : {16.0f, 17.0f, 16.5f}) {
        Scene scene;
        scene.shapes.push_back(Shape{.name = "plate", .kind = Kind::box, .centre = {at, 4.0f, 4.0f},
                                     .half = {0.125f, 20.0f, 20.0f}, .min_feature = 0.25f});
        SolidField f = *make_solid_field(shape);
        std::vector<SolidPart> parts = scene.parts();
        // The plate spans the whole section: its box is clipped to the grid
        // across the periodic y and z axes, as a caller's posed box would be.
        parts[0].box_min = glm::vec3(at - 0.125f, 0.0f, 0.0f);
        parts[0].box_max = glm::vec3(at + 0.125f, 8.0f, 8.0f);
        parts[0].extent = glm::vec3(0.25f, 8.0f, 8.0f);
        ASSERT_TRUE(build_solids(f, shape, {true, true, true}, parts, scene.sampler()).has_value());
        const uint32_t plane = at == 17.0f ? 17u : 16u;
        for (uint32_t k = 0; k < 8; ++k) {
            for (uint32_t j = 0; j < 8; ++j) {
                for (uint32_t i = 0; i <= 32; ++i) {
                    EXPECT_EQ(theta_at(f, shape, 0, i, j, k), i == plane ? 0.0f : 1.0f) << at << ": " << i;
                }
            }
        }
        for (uint32_t c = 1; c < 3; ++c) {
            for (const float t : f.theta[c]) EXPECT_EQ(t, 1.0f);
        }
        EXPECT_NE(f.run_line.find("plate: sheet (250.0 mm < 2 dx = 2000.0 mm)"), std::string::npos) << f.run_line;
    }
}

TEST(AirflowSolids, APartWhollyOutsideTheGridIsRefused) {
    const GridShape shape{16, 16, 16, 1.0f};
    Scene scene;
    scene.shapes.push_back(Shape{.name = "stray", .kind = Kind::sphere, .centre = {40.0f, 8.0f, 8.0f}, .radius = 2.0f,
                                 .min_feature = 4.0f});
    SolidField f = *make_solid_field(shape);
    const std::vector<SolidPart> parts = scene.parts();
    const Result<void> r = build_solids(f, shape, {}, parts, scene.sampler());
    ASSERT_FALSE(r.has_value());
    EXPECT_NE(r.error().context.find("stray"), std::string::npos) << r.error().context;
    EXPECT_NE(r.error().context.find("wholly outside the grid"), std::string::npos) << r.error().context;
    EXPECT_FALSE(f.any) << "nothing written";
}

// A part reaching past the grid is clipped, not refused: a wall filling the
// domain's edge (S2's tube) is ordinary validation geometry.
TEST(AirflowSolids, APartReachingPastTheGridIsClipped) {
    const GridShape shape{16, 16, 16, 1.0f};
    Scene scene;
    scene.shapes.push_back(Shape{.name = "slab", .kind = Kind::box, .centre = {8.0f, 8.0f, 0.0f},
                                 .half = {20.0f, 20.0f, 3.0f}, .min_feature = 6.0f});
    const SolidField f = built(shape, scene);
    EXPECT_TRUE(f.any);
    EXPECT_EQ(theta_at(f, shape, 2, 8, 8, 1), 0.0f) << "z = 1: 2 dx inside the slab";
    EXPECT_EQ(theta_at(f, shape, 2, 8, 8, 6), 1.0f) << "z = 6: 3 dx outside it";
}

TEST(AirflowSolids, ABadSamplerIsRefusedByName) {
    const GridShape shape{16, 16, 16, 1.0f};
    Scene scene;
    scene.shapes.push_back(Shape{.name = "ball", .kind = Kind::sphere, .centre = {8.0f, 8.0f, 8.0f}, .radius = 3.0f,
                                 .min_feature = 6.0f});
    const std::vector<SolidPart> parts = scene.parts();
    const auto nan_distance = [](const void*, glm::vec3) noexcept {
        return SolidQuery{std::numeric_limits<float>::quiet_NaN(), glm::vec3(1.0f, 0.0f, 0.0f), 0u, glm::vec3(0.0f)};
    };
    const auto past_the_list = [](const void*, glm::vec3) noexcept {
        return SolidQuery{-1.0f, glm::vec3(1.0f, 0.0f, 0.0f), 7u, glm::vec3(0.0f)};
    };
    const auto wild_velocity = [](const void*, glm::vec3) noexcept {
        return SolidQuery{-1.0f, glm::vec3(1.0f, 0.0f, 0.0f), 0u, glm::vec3(std::numeric_limits<float>::infinity())};
    };
    const struct {
        SolidQuery (*fn)(const void*, glm::vec3) noexcept;
        const char* says;
    } cases[] = {{nan_distance, "non-finite distance"}, {past_the_list, "part 7"}, {wild_velocity, "non-finite velocity"}};
    for (const auto& c : cases) {
        SolidField f = *make_solid_field(shape);
        const Result<void> r = build_solids(f, shape, {}, parts, SolidSampler{&scene, c.fn, &Scene::one});
        ASSERT_FALSE(r.has_value()) << c.says;
        EXPECT_NE(r.error().context.find(c.says), std::string::npos) << r.error().context;
        EXPECT_FALSE(f.any);
        for (uint32_t comp = 0; comp < 3; ++comp) {
            for (const float t : f.theta[comp]) ASSERT_EQ(t, 1.0f) << "a refused build leaves every face open";
        }
    }
}

TEST(AirflowSolids, NoPartsLeavesEveryFaceOpen) {
    const GridShape shape{8, 8, 8, 1.0f};
    SolidField f = *make_solid_field(shape);
    const Scene scene;
    ASSERT_TRUE(build_solids(f, shape, {}, {}, scene.sampler()).has_value());
    EXPECT_FALSE(f.any);
    for (uint32_t c = 0; c < 3; ++c) {
        for (const float t : f.theta[c]) EXPECT_EQ(t, 1.0f);
    }
    EXPECT_EQ(f.run_line, "solids: none");
}

}  // namespace
```

- [ ] **Step 3: Run the tests to see them fail**

Add `test_airflow_solids.cpp` to `tests/CMakeLists.txt` after `test_airflow_publish.cpp`. Then run `scripts\build.ps1 -Preset release -Target spade_tests -BuildDir build-ninja/release`.
Expected: a build failure, `cannot open include file 'physics/airflow/solids.hpp'`.

- [ ] **Step 4: Write the implementation**

Create `engine/physics/airflow/solids.hpp`:

```cpp
#pragma once

#include <array>
#include <cstdint>
#include <limits>
#include <span>
#include <string>
#include <vector>

#include <glm/glm.hpp>

#include "core/error.hpp"
#include "physics/airflow/mac_grid.hpp"

// ---------------------------------------------------------------------------
// SOLIDS IN THE GRID (2026-10-13-airflow-solids-design.md, sections 2-4; spec
// section 2). Every part reaches the grid through a sampler the caller
// provides: vehicles::sample_parts() adapted, in the tests, the D1 demo and
// plan C's module. The airflow core includes nothing from vehicles/, which
// links it, and computes no bounds (TD-9): each part arrives with its posed box
// and its part-frame extent (Core's part_bounds()).
//
// Per velocity sample (u, v, w arrays, MacGrid's indexing):
//   theta  the face's open fraction. A wall part: clamp(0.5 + phi / dx, 0, 1)
//          (Batty, Bertails & Bridson 2007). A sheet: 0 where its mid-surface
//          crosses the segment between the face's two cell centres, half-open
//          (-dx/2, dx/2] along the face's axis, so a sheet on a cell-centre
//          plane closes exactly one face; otherwise 1.
//   wall   the owning part's rigid velocity, this component, at the sample.
//   phi    a wall part's signed distance, negative inside; a sheet's
//          (never negative) distance to its mid-surface; max off solids.
//   link   the viscous solve's wall crossings (Task 8), or kNoWallLinks.
// Only samples within the parts' posed boxes grown by 2 dx are sampled; every
// other face stays open.
// ---------------------------------------------------------------------------
namespace spade::physics::airflow {

inline constexpr uint32_t kNoSolidPart = std::numeric_limits<uint32_t>::max();
inline constexpr uint32_t kNoWallLinks = std::numeric_limits<uint32_t>::max();

// What the geometry reports at a point of the grid's frame (metres).
struct SolidQuery {
    float distance = std::numeric_limits<float>::max();  // m, negative inside; max: no part
    glm::vec3 gradient{0.0f};                             // unit, out of the solid
    uint32_t part = kNoSolidPart;                         // index into the SolidPart list
    glm::vec3 velocity{0.0f};                             // the owner's rigid velocity at the point, m/s
};

// A non-owning view of the geometry. `nearest` is sample_parts()'s rule: the
// smallest distance wins, a tie to the lower part. `part` evaluates one part
// alone (the measured safeguard traces each part by itself).
struct SolidSampler {
    const void* ctx = nullptr;
    SolidQuery (*nearest)(const void* ctx, glm::vec3 x) noexcept = nullptr;
    SolidQuery (*part)(const void* ctx, uint32_t part, glm::vec3 x) noexcept = nullptr;
};

// One part as the grid sees it.
struct SolidPart {
    std::string name{};
    bool porous = false;
    float min_feature = 0.0f;  // m, declared (CORE-8 D5); 0: undeclared
    glm::vec3 extent{0.0f};    // m, its part-frame bounding box's size (vehicles::part_bounds())
    glm::vec3 box_min{0.0f};   // its posed bounding box in the grid's frame, m
    glm::vec3 box_max{0.0f};
};

enum class SolidRole : uint8_t { wall, sheet };

struct SolidClass {
    SolidRole role = SolidRole::wall;
    float thickness = 0.0f;  // a sheet's, m
};

// THE CLASSIFIER (design section 4; TD-9: Interface's resolve() calls this,
// not a rule of its own). A part is thin when min(declared min_feature, its
// smallest extent) < 2 dx; a thin part whose middle extent is also below 2 dx
// is refused (a wire or a fastener), any other thin part is a sheet of that
// thickness. Refused (invalid_argument, naming the part): a porous part (the
// porous task, design section 5); a non-finite or negative min_feature or
// extent; a wire, with the dx it needs.
[[nodiscard]] Result<SolidClass> classify_part(const SolidPart& part, float dx);

// The viscous solve's wall crossings at one sample (Task 8): per neighbour,
// -x, +x, -y, +y, -z, +z, 1 / f for a link the wall crosses at the fraction f
// of its length (else 0), and the wall's velocity there.
struct WallLinks {
    std::array<float, 6> inv_f{};
    std::array<float, 6> wall{};
};

// One component's sampled box, inclusive sample indices per axis.
struct SolidRange {
    std::array<uint32_t, 3> lo{};
    std::array<uint32_t, 3> hi{};
    bool empty = true;
};

struct SolidField {
    bool any = false;  // false: no part, every face open (plan A's paths)
    std::array<std::vector<float>, 3> theta;
    std::array<std::vector<float>, 3> wall;
    std::array<std::vector<float>, 3> phi;
    std::array<std::vector<uint32_t>, 3> link;
    std::vector<WallLinks> links;
    std::array<std::vector<uint32_t>, 3> fill1;  // the free-slip fill's first layer (Task 7)
    std::array<std::vector<uint32_t>, 3> fill2;  // and its second
    std::array<SolidRange, 3> range;             // per component, the sampled box
    std::vector<SolidClass> classes;             // per part
    std::string run_line;
};

[[nodiscard]] inline bool closed(float theta) noexcept { return theta == 0.0f; }

// Every array sized for `shape`, every face open. capacity_exceeded past
// kGridByteCap (16 bytes per face), invalid_argument for a shape make_grid()
// refuses.
[[nodiscard]] Result<SolidField> make_solid_field(const GridShape& shape);

// Rebuilds `field` from `parts` through `sample` (a host call; it allocates
// the link list and the fill layers). Each part is classified, every part's
// box grown by 2 dx is clipped to the grid, and every sample in the union of
// those boxes is sampled. On a periodic axis the last face plane copies the
// first. Refused (invalid_argument, naming what), with `field` left with every
// face open: classify_part()'s refusals; a part wholly outside the grid; a
// sampler that reports a non-finite distance, gradient or velocity, or
// a part past the list.
[[nodiscard]] Result<void> build_solids(SolidField& field, const GridShape& shape, std::array<bool, 3> periodic,
                                        std::span<const SolidPart> parts, const SolidSampler& sample);

}  // namespace spade::physics::airflow
```

Create `engine/physics/airflow/solids.cpp`:

```cpp
#include "physics/airflow/solids.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace spade::physics::airflow {

namespace {

[[nodiscard]] std::string mm(float metres) {
    char buf[32];
    std::snprintf(buf, sizeof buf, "%.1f mm", static_cast<double>(metres) * 1000.0);
    return buf;
}

[[nodiscard]] std::size_t sample_count(const GridShape& s, uint32_t c) noexcept {
    return std::size_t{c == 0 ? s.nx + 1u : s.nx} * (c == 1 ? s.ny + 1u : s.ny) * (c == 2 ? s.nz + 1u : s.nz);
}

[[nodiscard]] std::array<uint32_t, 3> sample_dims(const GridShape& s, uint32_t c) noexcept {
    return {c == 0 ? s.nx + 1u : s.nx, c == 1 ? s.ny + 1u : s.ny, c == 2 ? s.nz + 1u : s.nz};
}

void open_everything(SolidField& f) {
    for (uint32_t c = 0; c < 3; ++c) {
        std::fill(f.theta[c].begin(), f.theta[c].end(), 1.0f);
        std::fill(f.wall[c].begin(), f.wall[c].end(), 0.0f);
        std::fill(f.phi[c].begin(), f.phi[c].end(), std::numeric_limits<float>::max());
        std::fill(f.link[c].begin(), f.link[c].end(), kNoWallLinks);
        f.fill1[c].clear();
        f.fill2[c].clear();
        f.range[c] = SolidRange{};
    }
    f.links.clear();
    f.classes.clear();
    f.any = false;
}

// Where a sheet's mid-surface crosses the line through a sample along an axis,
// as a signed offset from the sample: the sheet lies against the gradient,
// which points away from it. False when the line runs along the sheet.
[[nodiscard]] bool sheet_offset(float d_mid, float g_axis, float& s) noexcept {
    if (g_axis == 0.0f) return false;
    s = (g_axis > 0.0f ? -d_mid : d_mid) / std::fabs(g_axis);
    return true;
}

[[nodiscard]] bool finite3(glm::vec3 v) noexcept {
    return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
}

}  // namespace

Result<SolidClass> classify_part(const SolidPart& p, float dx) {
    const auto refuse = [&](const std::string& why) -> Result<SolidClass> {
        return std::unexpected(Error{Code::invalid_argument, "airflow solids: part '" + p.name + "' " + why});
    };
    if (!(dx > 0.0f) || !std::isfinite(dx)) {
        return std::unexpected(Error{Code::invalid_argument, "airflow solids: dx must be positive and finite"});
    }
    if (p.porous) return refuse("is porous; porous parts arrive with the porous task (the solids design, section 5)");
    if (!(p.min_feature >= 0.0f) || !std::isfinite(p.min_feature)) {
        return refuse("has a min_feature that is not finite and non-negative");
    }
    std::array<float, 3> e{p.extent.x, p.extent.y, p.extent.z};
    for (const float v : e) {
        if (!(v >= 0.0f) || !std::isfinite(v)) return refuse("has a bounding box that is not finite and non-negative");
    }
    std::sort(e.begin(), e.end());
    const float feature = p.min_feature > 0.0f ? std::min(p.min_feature, e[0]) : e[0];
    const float two_dx = 2.0f * dx;
    if (feature >= two_dx) return SolidClass{SolidRole::wall, 0.0f};
    if (e[1] < two_dx) {
        return refuse("is below 2 dx = " + mm(two_dx) + " in two extents (smallest feature " + mm(feature) +
                      ", bounding box " + mm(e[0]) + " x " + mm(e[1]) + " x " + mm(e[2]) +
                      "): a wire or a fastener is neither a wall nor a sheet; it needs dx <= " + mm(0.5f * feature) +
                      " (refused, never dropped)");
    }
    return SolidClass{SolidRole::sheet, feature};
}

Result<SolidField> make_solid_field(const GridShape& s) {
    if (s.nx == 0 || s.ny == 0 || s.nz == 0 || !(s.dx > 0.0f) || !std::isfinite(s.dx)) {
        return std::unexpected(Error{Code::invalid_argument, "airflow solids: a grid shape make_grid() refuses"});
    }
    uint64_t faces = 0;
    for (uint32_t c = 0; c < 3; ++c) faces += sample_count(s, c);
    if (const uint64_t bytes = faces * 16u; bytes > kGridByteCap) {
        return std::unexpected(Error{Code::capacity_exceeded, "airflow solids: " + std::to_string(bytes) +
                                                                   " bytes of solid field, past the cap of " +
                                                                   std::to_string(kGridByteCap)});
    }
    SolidField f;
    for (uint32_t c = 0; c < 3; ++c) {
        const std::size_t n = sample_count(s, c);
        f.theta[c].assign(n, 1.0f);
        f.wall[c].assign(n, 0.0f);
        f.phi[c].assign(n, std::numeric_limits<float>::max());
        f.link[c].assign(n, kNoWallLinks);
    }
    f.run_line = "solids: none";
    return f;
}

Result<void> build_solids(SolidField& f, const GridShape& s, std::array<bool, 3> periodic,
                          std::span<const SolidPart> parts, const SolidSampler& sample) {
    open_everything(f);
    f.run_line = "solids: none";
    if (parts.empty()) return {};
    const auto refuse = [&](const std::string& why) -> Result<void> {
        open_everything(f);
        f.run_line = "solids: none";
        return std::unexpected(Error{Code::invalid_argument, "airflow solids: " + why});
    };
    if (sample.nearest == nullptr || sample.part == nullptr) return refuse("the sampler is empty");

    // 1. Classify every part, and check each grown box against the grid.
    std::vector<SolidClass> classes;
    classes.reserve(parts.size());
    const float n_metres[3] = {static_cast<float>(s.nx) * s.dx, static_cast<float>(s.ny) * s.dx,
                               static_cast<float>(s.nz) * s.dx};
    const float grow = 2.0f * s.dx;
    glm::vec3 lo(std::numeric_limits<float>::max());
    glm::vec3 hi(-std::numeric_limits<float>::max());
    for (const SolidPart& p : parts) {
        Result<SolidClass> c = classify_part(p, s.dx);
        if (!c) {
            open_everything(f);
            f.run_line = "solids: none";
            return std::unexpected(c.error());
        }
        classes.push_back(*c);
        // A part reaching past the grid is clipped to it: validation geometry
        // fills to the domain's edge (S2's tube), and the drone box's own
        // placement check is P2b's. A part wholly outside it is refused.
        bool meets = true;
        for (uint32_t a = 0; a < 3; ++a) {
            const float a0 = p.box_min[static_cast<glm::length_t>(a)] - grow;
            const float a1 = p.box_max[static_cast<glm::length_t>(a)] + grow;
            if (!std::isfinite(a0) || !std::isfinite(a1)) return refuse("part '" + p.name + "' has a non-finite box");
            if (a1 < 0.0f || a0 > n_metres[a]) meets = false;
        }
        if (!meets) return refuse("part '" + p.name + "' lies wholly outside the grid");
        lo = glm::min(lo, p.box_min - glm::vec3(grow));
        hi = glm::max(hi, p.box_max + glm::vec3(grow));
    }

    // 2. Each component's sampled index range.
    const float inv_dx = 1.0f / s.dx;
    std::array<SolidRange, 3> range{};
    for (uint32_t c = 0; c < 3; ++c) {
        const std::array<uint32_t, 3> dims = sample_dims(s, c);
        range[c].empty = false;
        for (uint32_t a = 0; a < 3; ++a) {
            const float off = a == c ? 0.0f : 0.5f;
            const float q0 = -std::floor(-(lo[static_cast<glm::length_t>(a)] * inv_dx - off));  // ceil
            const float q1 = std::floor(hi[static_cast<glm::length_t>(a)] * inv_dx - off);
            const float top = static_cast<float>(dims[a] - 1u);
            const float c0 = std::max(q0, 0.0f);
            const float c1 = std::min(q1, top);
            if (c1 < c0) {
                range[c].empty = true;
                break;
            }
            range[c].lo[a] = static_cast<uint32_t>(c0);
            range[c].hi[a] = static_cast<uint32_t>(c1);
        }
    }

    // 3. Pass 1: theta, the wall velocity and phi at every sample in range.
    for (uint32_t c = 0; c < 3; ++c) {
        if (range[c].empty) continue;
        const std::array<uint32_t, 3> dims = sample_dims(s, c);
        const glm::vec3 off(c == 0 ? 0.0f : 0.5f, c == 1 ? 0.0f : 0.5f, c == 2 ? 0.0f : 0.5f);
        for (uint32_t k = range[c].lo[2]; k <= range[c].hi[2]; ++k) {
            for (uint32_t j = range[c].lo[1]; j <= range[c].hi[1]; ++j) {
                for (uint32_t i = range[c].lo[0]; i <= range[c].hi[0]; ++i) {
                    const std::size_t idx = i + std::size_t{dims[0]} * (j + std::size_t{dims[1]} * k);
                    const glm::vec3 x = (glm::vec3(static_cast<float>(i), static_cast<float>(j),
                                                   static_cast<float>(k)) +
                                         off) *
                                        s.dx;
                    const SolidQuery q = sample.nearest(sample.ctx, x);
                    if (q.part == kNoSolidPart && q.distance == std::numeric_limits<float>::max()) continue;
                    if (!std::isfinite(q.distance) || !finite3(q.gradient)) {
                        return refuse("the sampler reported a non-finite distance or gradient at (" +
                                      std::to_string(x.x) + ", " + std::to_string(x.y) + ", " + std::to_string(x.z) +
                                      ") m");
                    }
                    if (q.part >= parts.size()) {
                        return refuse("the sampler reported part " + std::to_string(q.part) + " of " +
                                      std::to_string(parts.size()));
                    }
                    if (!finite3(q.velocity)) {
                        return refuse("the sampler reported a non-finite velocity for part '" + parts[q.part].name +
                                      "'");
                    }
                    const SolidClass& cl = classes[q.part];
                    f.wall[c][idx] = q.velocity[static_cast<glm::length_t>(c)];
                    if (cl.role == SolidRole::wall) {
                        f.phi[c][idx] = q.distance;
                        f.theta[c][idx] = std::min(std::max(0.5f + q.distance * inv_dx, 0.0f), 1.0f);
                    } else {
                        const float d_mid = std::max(q.distance + 0.5f * cl.thickness, 0.0f);
                        f.phi[c][idx] = d_mid;
                        float off_c = 0.0f;
                        const bool crosses = sheet_offset(d_mid, q.gradient[static_cast<glm::length_t>(c)], off_c) &&
                                             off_c > -0.5f * s.dx && off_c <= 0.5f * s.dx;
                        f.theta[c][idx] = crosses ? 0.0f : 1.0f;
                    }
                }
            }
        }
    }

    // 4. A periodic axis: the last face plane is the first.
    for (uint32_t c = 0; c < 3; ++c) {
        if (!periodic[c]) continue;
        const std::array<uint32_t, 3> dims = sample_dims(s, c);
        const uint32_t n = dims[c] - 1u;
        for (uint32_t k = 0; k < dims[2]; ++k) {
            for (uint32_t j = 0; j < dims[1]; ++j) {
                for (uint32_t i = 0; i < dims[0]; ++i) {
                    const uint32_t q[3] = {i, j, k};
                    if (q[c] != n) continue;
                    uint32_t first[3] = {i, j, k};
                    first[c] = 0u;
                    const std::size_t to = i + std::size_t{dims[0]} * (j + std::size_t{dims[1]} * k);
                    const std::size_t from = first[0] + std::size_t{dims[0]} * (first[1] + std::size_t{dims[1]} * first[2]);
                    f.theta[c][to] = f.theta[c][from];
                    f.wall[c][to] = f.wall[c][from];
                    f.phi[c][to] = f.phi[c][from];
                }
            }
        }
    }

    // 5. The measured safeguard (Task 2), the free-slip layers (Task 7) and
    //    the wall links (Task 8) follow here.

    // 6. The run line.
    uint32_t walls = 0;
    uint32_t sheets = 0;
    std::string sheet_text;
    for (std::size_t p = 0; p < parts.size(); ++p) {
        if (classes[p].role == SolidRole::wall) {
            ++walls;
            continue;
        }
        ++sheets;
        sheet_text += "; " + parts[p].name + ": sheet (" + mm(classes[p].thickness) + " < 2 dx = " + mm(2.0f * s.dx) + ")";
    }
    f.run_line = "solids: " + std::to_string(parts.size()) + " parts, " + std::to_string(walls) + " walls, " +
                 std::to_string(sheets) + " sheets" + sheet_text;
    f.classes = std::move(classes);
    f.range = range;
    f.any = true;
    return {};
}

}  // namespace spade::physics::airflow
```

Add `physics/airflow/solids.cpp` to `spade_physics` in `engine/CMakeLists.txt`, after `physics/airflow/actuator.cpp`.

- [ ] **Step 5: Run the tests to see them pass**

Run: `scripts\build.ps1 -Preset release -Target spade_tests -BuildDir build-ninja/release`, then `build-ninja\release\bin\spade_tests.exe --gtest_filter=AirflowSolids.*`
Expected: 7 tests PASS.

- [ ] **Step 6: The task gate, then commit**

The gate (Global Constraints) at the task's head. Then:

```bash
git add engine/physics/airflow/solids.hpp engine/physics/airflow/solids.cpp engine/CMakeLists.txt tests/airflow_solid_shapes.hpp tests/test_airflow_solids.cpp tests/CMakeLists.txt
git commit -m "feat(physics): the solid field -- classify_part(), apertures for walls (the ramp) and labelled sheets (half-open closure), from one sampler (P2 Task 1)"
```

---
### Task 2: The measured safeguard — blobs, leaking runs, edges and measured sheets

**Files:**
- Modify: `engine/physics/airflow/solids.cpp` (pass 1 becomes a function; the safeguard after it)
- Modify: `engine/physics/airflow/solids.hpp` (the header comment's safeguard paragraph)
- Modify: `tests/airflow_solid_shapes.hpp` (a two-box union), `tests/test_airflow_solids.cpp`

**Interfaces:**
- Consumes: Task 1's `SolidField`, `build_solids()`, `SolidSampler::part`.
- Produces: no new names. `build_solids()` now refuses leaking parts, announces edge leaks and sheets measured sheets on the run line (`f.classes[p]` is the final class).

The rule (design §4, as ruled 2026-10-13), for each part classified as a wall:
- **(A) A blob:** the part closes no face (no sample it wins has θ = 0).
- **(B) Leaking runs:** along each axis, a run joins two cell centres outside the part (φ > 0) with k ≥ 0 inside between them.
  - For k = 0 it must pass the screen `φ₁ + φ₂ ≤ dx` and a sphere-traced hit.
  - Sphere traces from both ends give the entry and exit, with their normals and the chord between them.
  - **Thin crossing:** `n_in · n_out ≤ −0.5` and `τ = chord · |n_in · e| < 2 dx`.
  - **Leaking run:** a thin crossing whose k + 1 faces are all open. Its area is its smallest θ times dx².
- **The outcome:**
  - **At an edge** (announced): not a blob, every leaking run's cells within 4 cells (Chebyshev) of a cell with a closed face, and total area ≤ 2% of the part's largest bounds face.
  - **Sheet-like** (sheeted, announced): a blob with at least one thin crossing, and every crossing's τ < 2 dx. Its thickness is the largest thin τ.
  - **Otherwise** refused by name.
  - After any part is sheeted, pass 1 runs again.

- [ ] **Step 1: The two-box union in the test shapes**

In `tests/airflow_solid_shapes.hpp`:
- add `two_boxes` to `Kind`;
- add two fields to `Shape`, after `half`: `glm::vec3 centre2{0.0f}; glm::vec3 half2{0.0f};  // two_boxes: the second box, axis-aligned`;
- add this case to `shape_field()`'s switch:

```cpp
        case Kind::two_boxes: {  // the union of two axis-aligned boxes: the smaller distance wins
            const Field a = box_field(p, s.half);
            const Field b = box_field(x - s.centre2, s.half2);
            f = b.d < a.d ? b : a;
            break;
        }
```

In `part_of()`, give `two_boxes` the union of the two boxes, with no rotation. Add before the corner loop:

```cpp
    if (s.kind == Kind::two_boxes) {
        p.box_min = glm::min(s.centre - s.half, s.centre2 - s.half2);
        p.box_max = glm::max(s.centre + s.half, s.centre2 + s.half2);
        p.extent = p.box_max - p.box_min;
        return p;
    }
```

- [ ] **Step 2: Write the failing tests**

Append to `tests/test_airflow_solids.cpp`, before the closing `}  // namespace`:

```cpp
// --- The measured safeguard (design section 4; P2 Task 2) -----------------

[[nodiscard]] Result<void> build_scene(SolidField& f, const GridShape& shape, const std::vector<SolidPart>& parts,
                                       const Scene& scene) {
    return build_solids(f, shape, {}, parts, scene.sampler());
}

// A 12-cell hub with a fin 0.3 dx thick on its +z face, centred on the face
// plane x = 16, 2 cells tall: every x line through the fin passes centres
// 0.35 dx outside on both sides (the screen's 0.7 dx), and the face between
// them has theta = 0.35. One part, declared thick.
[[nodiscard]] Shape hub_with_fin(float fin_length, float fin_half_height) {
    return Shape{.name = "hub_fin", .kind = Kind::two_boxes, .centre = {16.0f, 16.0f, 16.0f},
                 .half = {6.0f, 6.0f, 6.0f}, .centre2 = {16.0f, 16.0f, 22.0f + 0.5f * fin_length},
                 .half2 = {0.15f, fin_half_height, 0.5f * fin_length}, .min_feature = 12.0f};
}

// A fin 8 cells long: its leaking runs reach 8 cells past the hub's closed
// faces, beyond the 4-cell edge reach. Refused. Under the screen mutation
// phi1 + phi2 < dx/2 the fin's runs (0.7 dx) are skipped and this fails.
TEST(AirflowSolidsSafeguard, ALongFinOnAHubIsRefused) {
    const GridShape shape{32, 32, 36, 1.0f};
    Scene scene;
    scene.shapes.push_back(hub_with_fin(8.0f, 3.0f));
    SolidField f = *make_solid_field(shape);
    const Result<void> r = build_scene(f, shape, scene.parts(), scene);
    ASSERT_FALSE(r.has_value());
    EXPECT_NE(r.error().context.find("hub_fin"), std::string::npos) << r.error().context;
    EXPECT_NE(r.error().context.find("without meeting a closed face"), std::string::npos) << r.error().context;
    EXPECT_NE(r.error().context.find("declare min_feature or refine dx"), std::string::npos) << r.error().context;
}

// The same hub with a fin 3 cells long and 2 tall: six leaking runs, each
// within 3 cells of the hub's closed faces, open area 6 x 0.35 dx^2 = 2.1 dx^2
// against 2% of the bounds' largest face (12 x 15 = 180 dx^2: 3.6). Announced.
TEST(AirflowSolidsSafeguard, AShortFinOnAHubIsAnnouncedWithItsOpenArea) {
    const GridShape shape{32, 32, 32, 1.0f};
    Scene scene;
    scene.shapes.push_back(hub_with_fin(3.0f, 1.0f));
    SolidField f = *make_solid_field(shape);
    ASSERT_TRUE(build_scene(f, shape, scene.parts(), scene).has_value());
    EXPECT_EQ(f.classes[0].role, SolidRole::wall);
    EXPECT_NE(f.run_line.find("hub_fin: leaks at an edge (6 runs, open area 2.100 dx^2, 1.17% of its largest face)"),
              std::string::npos)
        << f.run_line;
}

// A fin 1.5 dx thick centred on the cell-centre plane x = 16.5: the centre in
// the fin is inside (k = 1), its two faces have theta = 0.25. Found and refused.
TEST(AirflowSolidsSafeguard, AFinOneAndAHalfCellsThickIsFound) {
    const GridShape shape{32, 32, 36, 1.0f};
    Scene scene;
    Shape s = hub_with_fin(8.0f, 3.0f);
    s.centre2.x = 16.5f;
    s.half2.x = 0.75f;
    scene.shapes.push_back(s);
    SolidField f = *make_solid_field(shape);
    const Result<void> r = build_scene(f, shape, scene.parts(), scene);
    ASSERT_FALSE(r.has_value());
    EXPECT_NE(r.error().context.find("without meeting a closed face"), std::string::npos) << r.error().context;
}

// A sphere 2 dx across, centred on a cell corner: the nearest face centres
// are 0.71 dx from its centre (theta 0.21), so it closes no face. Every line
// through it passes 0.71 R off centre, where n_in . n_out = 2 rho^2 - 1 = 0:
// no thin crossing, so not sheet-like either. Refused.
TEST(AirflowSolidsSafeguard, ATwoDxSphereAtItsWorstAlignmentIsRefused) {
    const GridShape shape{16, 16, 16, 1.0f};
    Scene scene;
    scene.shapes.push_back(Shape{.name = "bead", .kind = Kind::sphere, .centre = {8.0f, 8.0f, 8.0f}, .radius = 1.0f,
                                 .min_feature = 2.0f});
    SolidField f = *make_solid_field(shape);
    const Result<void> r = build_scene(f, shape, scene.parts(), scene);
    ASSERT_FALSE(r.has_value());
    EXPECT_NE(r.error().context.find("bead"), std::string::npos) << r.error().context;
    EXPECT_NE(r.error().context.find("closes no face"), std::string::npos) << r.error().context;
}

// A 4 dx box turned 45 degrees about y. Lines along x and z enter and leave
// through adjacent faces (n_in . n_out = 0); lines along y oppose but run the
// box's full 4 dx (tau = 4 dx). Not flagged.
TEST(AirflowSolidsSafeguard, ABoxTurnedFortyFiveDegreesIsNotFlagged) {
    const GridShape shape{32, 32, 32, 1.0f};
    const float h = 0.70710678f;
    Scene scene;
    scene.shapes.push_back(Shape{.name = "turned", .kind = Kind::box, .centre = {16.0f, 16.0f, 16.0f},
                                 .half = {2.0f, 2.0f, 2.0f},
                                 .to_local = glm::mat3(h, 0.0f, -h, 0.0f, 1.0f, 0.0f, h, 0.0f, h),
                                 .min_feature = 4.0f});
    SolidField f = *make_solid_field(shape);
    ASSERT_TRUE(build_scene(f, shape, scene.parts(), scene).has_value());
    EXPECT_EQ(f.classes[0].role, SolidRole::wall);
    EXPECT_EQ(f.run_line.find("leaks"), std::string::npos) << f.run_line;
}

// A 30-degree wedge (half-angle 15 degrees), 64 cells of chord, 8 of span:
// its tip leaks within the 4-cell reach of the closed faces that begin 2-4
// cells back, and the leak's open area is far under 2% of its planform
// (64 x 34.3 dx^2). Announced, with the count and the area.
TEST(AirflowSolidsSafeguard, AThirtyDegreeWedgesTipIsAnnounced) {
    const GridShape shape{80, 12, 40, 1.0f};
    Scene scene;
    scene.shapes.push_back(Shape{.name = "wedge", .kind = Kind::wedge_y, .centre = {8.0f, 6.0f, 20.0f},
                                 .half = {64.0f, 4.0f, 0.0f}, .radius = 0.26179939f, .min_feature = 4.0f});
    SolidField f = *make_solid_field(shape);
    const Result<void> r = build_scene(f, shape, scene.parts(), scene);
    ASSERT_TRUE(r.has_value()) << r.error().context;
    EXPECT_EQ(f.classes[0].role, SolidRole::wall);
    EXPECT_NE(f.run_line.find("wedge: leaks at an edge ("), std::string::npos) << f.run_line;
    std::printf("measured (TD-2, pin on the first run): %s\n", f.run_line.c_str());
}

// A plate 0.5 dx thick, declared 4 dx and turned 30 degrees about z inside
// its own frame, so its bounds look thick (10.6 x 6.4 x 12 dx) and the
// classifier calls it a wall. It closes no face, and every line crossing it
// is thin (tau = 0.5 dx): sheet-like. Sheeted with its measured thickness,
// announced, and never "at an edge".
TEST(AirflowSolidsSafeguard, ATiltedThinPlateDeclaredThickIsSheeted) {
    const GridShape shape{32, 32, 32, 1.0f};
    const float c = 0.8660254f;
    const float s = 0.5f;
    Scene scene;
    scene.shapes.push_back(Shape{.name = "tilted", .kind = Kind::box, .centre = {16.0f, 16.0f, 16.0f},
                                 .half = {6.0f, 0.25f, 6.0f},
                                 .to_local = glm::mat3(c, -s, 0.0f, s, c, 0.0f, 0.0f, 0.0f, 1.0f),
                                 .min_feature = 4.0f});
    std::vector<SolidPart> parts = scene.parts();
    parts[0].extent = parts[0].box_max - parts[0].box_min;  // a mesh tilted in its own frame
    SolidField f = *make_solid_field(shape);
    ASSERT_TRUE(build_scene(f, shape, parts, scene).has_value());
    EXPECT_EQ(f.classes[0].role, SolidRole::sheet);
    EXPECT_NEAR(f.classes[0].thickness, 0.5f, 0.05f);
    EXPECT_NE(f.run_line.find("tilted: sheet (measured"), std::string::npos) << f.run_line;
    EXPECT_EQ(f.run_line.find("at an edge"), std::string::npos) << f.run_line;
    uint32_t closed_faces = 0;
    for (uint32_t comp = 0; comp < 3; ++comp) {
        for (const float t : f.theta[comp]) closed_faces += t == 0.0f ? 1u : 0u;
    }
    EXPECT_GT(closed_faces, 0u) << "as a sheet it closes the faces it crosses";
}
```

The percentage in `AShortFinOnAHubIsAnnouncedWithItsOpenArea` is 2.1 / 180 = 1.17%. The `to_local` matrices are glm column-major: the 45° matrix is a rotation about y, the 30° one about z. Each test's comment says which.

- [ ] **Step 3: Run the tests to see them fail**

Run: build `spade_tests`, then `--gtest_filter=AirflowSolidsSafeguard.*`.
Expected: `ALongFinOnAHubIsRefused`, `AFinOneAndAHalfCellsThickIsFound`, `ATwoDxSphereAtItsWorstAlignmentIsRefused` and `ATiltedThinPlateDeclaredThickIsSheeted` FAIL, because the build succeeds or classes stay wall. `AShortFin…` and `AThirtyDegree…` FAIL on the run line. `ABoxTurned…` passes, as it should before and after.

- [ ] **Step 4: Write the implementation**

In `solids.cpp`, move Task 1's steps 3 and 4 (pass 1 and the periodic copy) into a function, and record which parts close a face:

```cpp
// Pass 1 (theta, the wall velocity and phi in range) and the periodic copy.
// closes[p] is set when a sample part p wins has theta = 0.
[[nodiscard]] Result<void> sample_pass(SolidField& f, const GridShape& s, std::array<bool, 3> periodic,
                                       std::span<const SolidPart> parts, const std::vector<SolidClass>& classes,
                                       const SolidSampler& sample, const std::array<SolidRange, 3>& range,
                                       std::vector<uint8_t>& closes);
```

Its body is Task 1's steps 3 and 4 verbatim. Make three changes:
- every `return refuse(...)` becomes `return std::unexpected(Error{Code::invalid_argument, "airflow solids: " + ...})`, since the caller opens the field;
- the field is reset to open first, for every sample in range: θ = 1, wall = 0, φ = max;
- after each θ is written, one line is added:

```cpp
                    if (f.theta[c][idx] == 0.0f) closes[q.part] = 1u;
```

`build_solids()` calls it where steps 3 and 4 were, and on failure opens the field and returns the error.

Then add, in the anonymous namespace:

```cpp
// The 4-cell reach and the 2% bound (design section 4).
constexpr int kEdgeReachCells = 4;
constexpr float kEdgeAreaFraction = 0.02f;
constexpr int kTraceSteps = 32;
constexpr float kTraceHit = 1.0e-4f;  // times dx

struct Trace {
    float t = 0.0f;           // from the start, m
    glm::vec3 normal{0.0f};   // the part's gradient at the last point
    bool hit = false;
    bool finite = true;
};

// Sphere-traces part `p` from `from` along `dir` (unit), up to `length`: t
// advances by phi, a hit is phi <= kTraceHit dx, at most kTraceSteps steps.
[[nodiscard]] Trace trace(const SolidSampler& sample, uint32_t p, glm::vec3 from, glm::vec3 dir, float length,
                          float dx) noexcept {
    Trace out;
    const float eps = kTraceHit * dx;
    SolidQuery q = sample.part(sample.ctx, p, from);
    for (int step = 0; step < kTraceSteps; ++step) {
        if (!std::isfinite(q.distance) || !finite3(q.gradient)) {
            out.finite = false;
            return out;
        }
        out.normal = q.gradient;
        if (q.distance <= eps) {
            out.hit = true;
            return out;
        }
        out.t += q.distance;
        if (out.t >= length) {
            out.t = length;
            return out;
        }
        q = sample.part(sample.ctx, p, from + out.t * dir);
    }
    out.hit = std::isfinite(q.distance) && q.distance <= eps;
    if (out.hit) out.normal = q.gradient;
    return out;
}

struct Leaks {
    uint32_t runs = 0;          // leaking runs
    uint32_t off_edge = 0;      // of them, outside the edge reach
    float area = 0.0f;          // their open area, m^2
    glm::vec3 first{0.0f};      // the first one's entry point
    uint32_t thin = 0;          // thin crossings
    float tau_max = 0.0f;       // the largest crossing thickness estimate, m
    float thin_tau_max = 0.0f;  // the largest thin crossing's, m
};

// Cells with a closed face, grown by kEdgeReachCells (Chebyshev).
[[nodiscard]] std::vector<uint8_t> near_closed(const SolidField& f, const GridShape& s) {
    const uint32_t n[3] = {s.nx, s.ny, s.nz};
    const std::size_t cells = std::size_t{s.nx} * s.ny * s.nz;
    std::vector<uint8_t> mask(cells, 0u);
    for (uint32_t k = 0; k < s.nz; ++k) {
        for (uint32_t j = 0; j < s.ny; ++j) {
            for (uint32_t i = 0; i < s.nx; ++i) {
                const bool any_closed = closed(f.theta[0][u_index(s, i, j, k)]) ||
                                        closed(f.theta[0][u_index(s, i + 1u, j, k)]) ||
                                        closed(f.theta[1][v_index(s, i, j, k)]) ||
                                        closed(f.theta[1][v_index(s, i, j + 1u, k)]) ||
                                        closed(f.theta[2][w_index(s, i, j, k)]) ||
                                        closed(f.theta[2][w_index(s, i, j, k + 1u)]);
                mask[cell_index(s, i, j, k)] = any_closed ? 1u : 0u;
            }
        }
    }
    std::vector<uint8_t> tmp(cells, 0u);
    const std::size_t stride[3] = {1u, s.nx, std::size_t{s.nx} * s.ny};
    for (uint32_t a = 0; a < 3; ++a) {  // a separable max filter, axis by axis
        for (uint32_t k = 0; k < s.nz; ++k) {
            for (uint32_t j = 0; j < s.ny; ++j) {
                for (uint32_t i = 0; i < s.nx; ++i) {
                    const uint32_t q[3] = {i, j, k};
                    const std::size_t c = cell_index(s, i, j, k);
                    uint8_t v = 0u;
                    for (int d = -kEdgeReachCells; d <= kEdgeReachCells && v == 0u; ++d) {
                        const int64_t at = int64_t{q[a]} + d;
                        if (at < 0 || at >= int64_t{n[a]}) continue;
                        v = mask[static_cast<std::size_t>(int64_t(c) + d * int64_t(stride[a]))];
                    }
                    tmp[c] = v;
                }
            }
        }
        mask.swap(tmp);
    }
    return mask;
}

// Part p's cell box: its posed box grown by 2 dx, as inclusive cell indices.
struct CellBox {
    std::array<uint32_t, 3> lo{};
    std::array<uint32_t, 3> hi{};
};

[[nodiscard]] CellBox cell_box(const GridShape& s, const SolidPart& p) noexcept {
    const uint32_t n[3] = {s.nx, s.ny, s.nz};
    const float inv_dx = 1.0f / s.dx;
    CellBox b;
    for (uint32_t a = 0; a < 3; ++a) {
        const auto al = static_cast<glm::length_t>(a);
        const float q0 = -std::floor(-((p.box_min[al] - 2.0f * s.dx) * inv_dx - 0.5f));
        const float q1 = std::floor((p.box_max[al] + 2.0f * s.dx) * inv_dx - 0.5f);
        b.lo[a] = static_cast<uint32_t>(std::min(std::max(q0, 0.0f), static_cast<float>(n[a] - 1u)));
        b.hi[a] = static_cast<uint32_t>(std::min(std::max(q1, 0.0f), static_cast<float>(n[a] - 1u)));
    }
    return b;
}

// Part p's leaks: every run along every axis through its cell box.
[[nodiscard]] Result<Leaks> find_leaks(const SolidField& f, const GridShape& s, const SolidSampler& sample,
                                       uint32_t p, const SolidPart& part, const std::vector<uint8_t>& near,
                                       std::vector<float>& cache) {
    Leaks out;
    const CellBox b = cell_box(s, part);
    const uint32_t ext[3] = {b.hi[0] - b.lo[0] + 1u, b.hi[1] - b.lo[1] + 1u, b.hi[2] - b.lo[2] + 1u};
    cache.assign(std::size_t{ext[0]} * ext[1] * ext[2], 0.0f);
    const auto centre = [&](uint32_t i, uint32_t j, uint32_t k) {
        return (glm::vec3(static_cast<float>(i), static_cast<float>(j), static_cast<float>(k)) + 0.5f) * s.dx;
    };
    const auto at = [&](uint32_t i, uint32_t j, uint32_t k) {
        return (i - b.lo[0]) + std::size_t{ext[0]} * ((j - b.lo[1]) + std::size_t{ext[1]} * (k - b.lo[2]));
    };
    for (uint32_t k = b.lo[2]; k <= b.hi[2]; ++k) {
        for (uint32_t j = b.lo[1]; j <= b.hi[1]; ++j) {
            for (uint32_t i = b.lo[0]; i <= b.hi[0]; ++i) {
                const SolidQuery q = sample.part(sample.ctx, p, centre(i, j, k));
                if (!std::isfinite(q.distance)) {
                    return std::unexpected(Error{Code::invalid_argument, "airflow solids: the sampler reported a "
                                                                         "non-finite distance for part '" +
                                                                             part.name + "'"});
                }
                cache[at(i, j, k)] = q.distance;
            }
        }
    }
    const float dx = s.dx;
    for (uint32_t a = 0; a < 3; ++a) {
        const uint32_t a1 = (a + 1u) % 3u;
        const uint32_t a2 = (a + 2u) % 3u;
        const uint32_t outer = a1 > a2 ? a1 : a2;  // k-like, then j-like, then along the line
        const uint32_t inner = a1 > a2 ? a2 : a1;
        glm::vec3 e(0.0f);
        e[static_cast<glm::length_t>(a)] = 1.0f;
        for (uint32_t u = b.lo[outer]; u <= b.hi[outer]; ++u) {
            for (uint32_t v = b.lo[inner]; v <= b.hi[inner]; ++v) {
                int64_t o1 = -1;
                for (uint32_t q = b.lo[a]; q <= b.hi[a]; ++q) {
                    uint32_t ijk[3];
                    ijk[a] = q;
                    ijk[outer] = u;
                    ijk[inner] = v;
                    const float phi = cache[at(ijk[0], ijk[1], ijk[2])];
                    if (!(phi > 0.0f)) continue;  // inside: the run goes on
                    if (o1 >= 0) {
                        // A run from o1 to q.
                        uint32_t c1[3] = {ijk[0], ijk[1], ijk[2]};
                        c1[a] = static_cast<uint32_t>(o1);
                        const uint32_t k_inside = q - static_cast<uint32_t>(o1) - 1u;
                        const float phi1 = cache[at(c1[0], c1[1], c1[2])];
                        const float length = static_cast<float>(q - static_cast<uint32_t>(o1)) * dx;
                        const glm::vec3 x1 = centre(c1[0], c1[1], c1[2]);
                        const glm::vec3 x2 = centre(ijk[0], ijk[1], ijk[2]);
                        if (k_inside > 0u || phi1 + phi <= dx) {
                            const Trace in = trace(sample, p, x1, e, length, dx);
                            const Trace out_t = trace(sample, p, x2, -e, length, dx);
                            if (!in.finite || !out_t.finite) {
                                return std::unexpected(Error{Code::invalid_argument,
                                                             "airflow solids: the sampler reported a non-finite "
                                                             "distance for part '" + part.name + "'"});
                            }
                            if (k_inside > 0u || in.hit) {
                                const float chord = std::max(length - in.t - out_t.t, 0.0f);
                                const float tau = chord * std::fabs(in.normal[static_cast<glm::length_t>(a)]);
                                out.tau_max = std::max(out.tau_max, tau);
                                const bool thin = glm::dot(in.normal, out_t.normal) <= -0.5f && tau < 2.0f * dx;
                                if (thin) {
                                    ++out.thin;
                                    out.thin_tau_max = std::max(out.thin_tau_max, tau);
                                    float smallest = 1.0f;
                                    bool all_open = true;
                                    for (uint32_t face = static_cast<uint32_t>(o1) + 1u; face <= q; ++face) {
                                        uint32_t fi[3] = {ijk[0], ijk[1], ijk[2]};
                                        fi[a] = face;
                                        const std::size_t idx = a == 0 ? u_index(s, fi[0], fi[1], fi[2])
                                                                : (a == 1 ? v_index(s, fi[0], fi[1], fi[2])
                                                                          : w_index(s, fi[0], fi[1], fi[2]));
                                        const float th = f.theta[a][idx];
                                        if (closed(th)) {
                                            all_open = false;
                                            break;
                                        }
                                        smallest = std::min(smallest, th);
                                    }
                                    if (all_open) {
                                        if (out.runs == 0u) out.first = x1 + in.t * e;
                                        ++out.runs;
                                        out.area += smallest * dx * dx;
                                        for (uint32_t cq = static_cast<uint32_t>(o1); cq <= q; ++cq) {
                                            uint32_t ci[3] = {ijk[0], ijk[1], ijk[2]};
                                            ci[a] = cq;
                                            if (near[cell_index(s, ci[0], ci[1], ci[2])] == 0u) {
                                                ++out.off_edge;
                                                break;
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }
                    o1 = q;
                }
            }
        }
    }
    return out;
}

[[nodiscard]] std::string fmt(const char* format, double v) {
    char buf[64];
    std::snprintf(buf, sizeof buf, format, v);
    return buf;
}
```

In `build_solids()`, between the first `sample_pass()` and the run line:

```cpp
    // 5. The measured safeguard (design section 4).
    std::string notes;
    {
        const std::vector<uint8_t> near = near_closed(f, s);
        std::vector<float> cache;
        bool resheeted = false;
        for (uint32_t p = 0; p < parts.size(); ++p) {
            if (classes[p].role != SolidRole::wall) continue;
            Result<Leaks> found = find_leaks(f, s, sample, p, parts[p], near, cache);
            if (!found) {
                open_everything(f);
                f.run_line = "solids: none";
                return std::unexpected(found.error());
            }
            const Leaks& lk = *found;
            const bool blob = closes[p] == 0u;
            if (!blob && lk.runs == 0u) continue;  // resolved
            std::array<float, 3> e{parts[p].extent.x, parts[p].extent.y, parts[p].extent.z};
            std::sort(e.begin(), e.end());
            const float largest_face = e[1] * e[2];
            if (!blob && lk.off_edge == 0u && lk.area <= kEdgeAreaFraction * largest_face) {
                notes += "; " + parts[p].name + ": leaks at an edge (" + std::to_string(lk.runs) + " runs, open area " +
                         fmt("%.3f", static_cast<double>(lk.area / (s.dx * s.dx))) + " dx^2, " +
                         fmt("%.2f", 100.0 * static_cast<double>(lk.area / largest_face)) + "% of its largest face)";
                continue;
            }
            if (blob && lk.thin > 0u && lk.tau_max < 2.0f * s.dx) {
                classes[p] = SolidClass{SolidRole::sheet, lk.thin_tau_max};
                resheeted = true;
                notes += "; " + parts[p].name + ": sheet (measured " + mm(lk.thin_tau_max) + " across " +
                         std::to_string(lk.thin) + " thin crossings, no closed face; declared min_feature " +
                         mm(parts[p].min_feature) + ")";
                continue;
            }
            const std::string where = blob ? "it closes no face at dx = " + mm(s.dx) +
                                                 " (no face centre lies dx/2 inside it)"
                                           : "a grid line crosses it at (" + fmt("%.4f", lk.first.x) + ", " +
                                                 fmt("%.4f", lk.first.y) + ", " + fmt("%.4f", lk.first.z) +
                                                 ") m without meeting a closed face (" + std::to_string(lk.runs) +
                                                 " leaking runs, " + std::to_string(lk.off_edge) +
                                                 " beyond the edge reach)";
            open_everything(f);
            f.run_line = "solids: none";
            return std::unexpected(Error{Code::invalid_argument, "airflow solids: part '" + parts[p].name +
                                                                     "' leaks: " + where +
                                                                     "; declare min_feature or refine dx"});
        }
        if (resheeted) {
            std::fill(closes.begin(), closes.end(), 0u);
            if (Result<void> again = sample_pass(f, s, periodic, parts, classes, sample, range, closes); !again) {
                open_everything(f);
                f.run_line = "solids: none";
                return std::unexpected(again.error());
            }
        }
    }
```

In the run line (Task 1's step 6), a sheet entry the safeguard made is already in `notes`. Skip its classifier text for parts whose note starts the sheet entry: keep a `std::vector<uint8_t> measured(parts.size(), 0)` set where `resheeted` is set, and in the sheet loop write the classifier's text only when `measured[p] == 0`. Append `notes` last:

```cpp
    f.run_line = "solids: " + std::to_string(parts.size()) + " parts, " + std::to_string(walls) + " walls, " +
                 std::to_string(sheets) + " sheets" + sheet_text + notes;
```

In `solids.hpp`, extend the header comment with the rule above, in the design's words.

- [ ] **Step 5: Run the tests to see them pass**

Run: `--gtest_filter=AirflowSolids*`
Expected: all PASS. `AThirtyDegreeWedgesTipIsAnnounced` prints its measured count and area. Pin them in its comment and as an `EXPECT_NE(f.run_line.find("wedge: leaks at an edge (<n> runs, open area <a> dx^2"), …)` (TD-2).

- [ ] **Step 6: The mutation, by hand**

In `find_leaks()`, change `phi1 + phi <= dx` to `phi1 + phi < 0.5f * dx`, rebuild, and run `AirflowSolidsSafeguard.ALongFinOnAHubIsRefused`. It must FAIL: the fin's 0.7 dx runs are screened out and the part passes as a wall. Restore the line. Record it in the ledger: the mutation, the failing assertion and its line.

- [ ] **Step 7: The task gate, then commit**

```bash
git add engine/physics/airflow/solids.hpp engine/physics/airflow/solids.cpp tests/airflow_solid_shapes.hpp tests/test_airflow_solids.cpp
git commit -m "feat(physics): the measured safeguard -- blobs and leaking runs (Lipschitz screen, traced entry and exit, opposing normals under 2 dx), edge leaks announced, measured sheets, the rest refused by name (P2 Task 2)"
```

---
### Task 3: The weighted pressure operator and its coarse weights

**Files:**
- Modify: `engine/physics/airflow/pressure.hpp`, `engine/physics/airflow/pressure.cpp`
- Modify: `engine/physics/airflow/multigrid.hpp`, `engine/physics/airflow/multigrid.cpp`
- Create: `tests/test_airflow_solids_solve.cpp`
- Modify: `tests/CMakeLists.txt` (`test_airflow_solids_solve.cpp` after `test_airflow_solids.cpp`)

**Interfaces:**
- Consumes: plan A's `PoissonLevel`, `Multigrid`, `stencil()`, `smooth_red_black()`, `residual()`.
- Produces:
  - `PoissonLevel::w`: `std::array<std::vector<float>, 3>`, the faces normal to x, y and z, indexed like `u_index`/`v_index`/`w_index` on the level's shape; empty means plan A's stencil.
  - `void smooth_red_black(PoissonLevel&, uint32_t sweeps, bool black_first = false) noexcept`
  - `void apply_operator(const PoissonLevel&, std::span<const float> x, std::span<float> out) noexcept`: out = sum − diag·x
  - `void set_weights(Multigrid&, const std::array<std::span<const float>, 3>& fine)`
  - `void clear_weights(Multigrid&) noexcept`

- [ ] **Step 1: Write the failing tests**

Create `tests/test_airflow_solids_solve.cpp`:

```cpp
// The pressure solve with solids (the solids design, section 3; spec 1.5 as
// amended 2026-10-13): face weights, their coarse means, the symmetric
// V-cycle, CG, and the fluid regions (P2 Tasks 3-5).
#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <span>
#include <vector>

#include "airflow_solid_shapes.hpp"
#include "physics/airflow/multigrid.hpp"
#include "physics/airflow/pressure.hpp"
#include "physics/airflow/solids.hpp"

namespace {

using namespace spade::physics::airflow;

struct Lcg {
    uint32_t state = 2463534242u;
    float next() noexcept {
        state = state * 1664525u + 1013904223u;
        return static_cast<float>(state >> 8) * (2.0f / 16777216.0f) - 1.0f;
    }
};

[[nodiscard]] std::array<std::vector<float>, 3> ones(uint32_t nx, uint32_t ny, uint32_t nz) {
    return {std::vector<float>(std::size_t{nx + 1u} * ny * nz, 1.0f),
            std::vector<float>(std::size_t{nx} * (ny + 1u) * nz, 1.0f),
            std::vector<float>(std::size_t{nx} * ny * (nz + 1u), 1.0f)};
}

[[nodiscard]] bool same_bits(const std::vector<float>& a, const std::vector<float>& b) {
    return a.size() == b.size() && std::memcmp(a.data(), b.data(), a.size() * sizeof(float)) == 0;
}

// Weights of exactly 1 are plan A's stencil bit for bit: 1.0f * p is p, and
// the six weights (and an outlet's 2 * 1) sum as plan A's integers do. Every
// face kind is present: x periodic, y outlets, z fixed.
TEST(AirflowSolidsSolve, WeightsOfOneAreBitIdentical) {
    const std::array<FaceKind, 6> kinds{FaceKind::periodic,        FaceKind::periodic,       FaceKind::pressure_outlet,
                                        FaceKind::pressure_outlet, FaceKind::fixed_velocity, FaceKind::fixed_velocity};
    PoissonLevel plain = make_level(8, 6, 5, kinds);
    Lcg rng;
    for (float& v : plain.p) v = rng.next();
    for (float& v : plain.b) v = rng.next();
    PoissonLevel weighted = plain;
    weighted.w = ones(8, 6, 5);
    smooth_red_black(plain, 3);
    smooth_red_black(weighted, 3);
    EXPECT_TRUE(same_bits(plain.p, weighted.p));
    residual(plain);
    residual(weighted);
    EXPECT_TRUE(same_bits(plain.r, weighted.r));
    std::vector<float> out_plain(plain.p.size());
    std::vector<float> out_weighted(plain.p.size());
    apply_operator(plain, plain.p, out_plain);
    apply_operator(weighted, weighted.p, out_weighted);
    EXPECT_TRUE(same_bits(out_plain, out_weighted));
    for (std::size_t c = 0; c < plain.r.size(); ++c) {
        EXPECT_EQ(plain.r[c], plain.b[c] - out_plain[c]) << "apply_operator is the residual's operator";
    }
}

// A closed face (weight 0) decouples the two cells on either side: with p = 0
// on the left half and 1 on the right, the operator is 0 everywhere but at the
// open faces between the halves, and there are none.
TEST(AirflowSolidsSolve, AClosedFaceDecouplesItsCells) {
    const std::array<FaceKind, 6> walls{FaceKind::fixed_velocity, FaceKind::fixed_velocity, FaceKind::fixed_velocity,
                                        FaceKind::fixed_velocity, FaceKind::fixed_velocity, FaceKind::fixed_velocity};
    PoissonLevel L = make_level(4, 2, 2, walls);
    L.w = ones(4, 2, 2);
    for (uint32_t k = 0; k < 2; ++k) {
        for (uint32_t j = 0; j < 2; ++j) L.w[0][2u + 5u * (j + 2u * k)] = 0.0f;  // the u faces at i = 2
    }
    for (uint32_t k = 0; k < 2; ++k) {
        for (uint32_t j = 0; j < 2; ++j) {
            for (uint32_t i = 0; i < 4; ++i) L.p[i + 4u * (j + 2u * k)] = i < 2 ? 0.0f : 1.0f;
        }
    }
    std::vector<float> out(L.p.size());
    apply_operator(L, L.p, out);
    for (const float v : out) EXPECT_EQ(v, 0.0f);
}

// A coarse face's weight is the mean of the fine faces on its plane that it
// covers: 2 x 2 of them.
TEST(AirflowSolidsSolve, CoarseWeightsAreTheMeanOfTheFacesTheyCover) {
    Multigrid mg = make_multigrid(8, 8, 8);
    std::array<std::vector<float>, 3> fine = ones(8, 8, 8);
    for (uint32_t c = 0; c < 3; ++c) {
        for (std::size_t q = 0; q < fine[c].size(); ++q) fine[c][q] = static_cast<float>((q * 7u + c) % 5u) * 0.25f;
    }
    set_weights(mg, {fine[0], fine[1], fine[2]});
    ASSERT_GE(mg.levels.size(), 2u);
    const PoissonLevel& coarse = mg.levels[1];
    ASSERT_EQ(coarse.w[0].size(), std::size_t{5} * 4 * 4);
    for (uint32_t K = 0; K < 4; ++K) {
        for (uint32_t J = 0; J < 4; ++J) {
            for (uint32_t I = 0; I <= 4; ++I) {
                const auto f = [&](uint32_t i, uint32_t j, uint32_t k) { return fine[0][i + 9u * (j + 8u * k)]; };
                float sum = 0.0f;
                sum += f(2u * I, 2u * J, 2u * K);
                sum += f(2u * I, 2u * J + 1u, 2u * K);
                sum += f(2u * I, 2u * J, 2u * K + 1u);
                sum += f(2u * I, 2u * J + 1u, 2u * K + 1u);
                EXPECT_EQ(coarse.w[0][I + 5u * (J + 4u * K)], sum * 0.25f) << I << "," << J << "," << K;
            }
        }
    }
    clear_weights(mg);
    for (const PoissonLevel& L : mg.levels) EXPECT_TRUE(L.w[0].empty());
}

}  // namespace
```

`CoarseWeightsAreTheMeanOfTheFacesTheyCover` fixes the summation order as the implementation's, (j, k) = (0,0), (1,0), (0,1), (1,1), with the second transverse axis outer. Keep them in that order on both sides.

- [ ] **Step 2: Run the tests to see them fail**

Expected: a build failure on `PoissonLevel::w`, `apply_operator`, `set_weights` and `clear_weights`.

- [ ] **Step 3: Write the implementation**

`pressure.hpp`: add to `PoissonLevel`, after `r`:

```cpp
    // Face weights (the solids design, section 3): w[a] holds the faces normal
    // to axis a, indexed as a MAC grid of this level's shape indexes u, v and
    // w. Each multiplies its neighbour term and its part of the diagonal. On a
    // periodic axis the last face plane mirrors the first. Empty: every weight
    // 1, and plan A's stencil and fast paths run, bit for bit.
    std::array<std::vector<float>, 3> w;
```

and change and add these declarations:

```cpp
// `sweeps` red-black sweeps: every cell with (i + j + k) even, then every odd
// one; black_first runs the odd colour first (the symmetric cycle's
// post-smoothing, Task 4).
void smooth_red_black(PoissonLevel& level, uint32_t sweeps, bool black_first = false) noexcept;

// out = sum - diag x, the level's operator applied to `x` (both sized like p).
void apply_operator(const PoissonLevel& level, std::span<const float> x, std::span<float> out) noexcept;
```

`pressure.cpp`:
- `stencil()` takes the values it reads: `stencil(const PoissonLevel& L, const float* p, uint32_t i, uint32_t j, uint32_t k)`. Its body reads `p[...]` where it read `L.p[...]`, and every existing caller passes `L.p.data()`.
- Add the weighted stencil beside it:

```cpp
[[nodiscard]] inline std::size_t face_at(const PoissonLevel& L, uint32_t axis, uint32_t i, uint32_t j,
                                         uint32_t k) noexcept {
    const std::size_t cx = axis == 0 ? L.nx + 1u : L.nx;
    const std::size_t cy = axis == 1 ? L.ny + 1u : L.ny;
    return i + cx * (j + cy * std::size_t{k});
}

// stencil() with face weights, the faces in the same order. A cell's low face
// on axis a has index c[a], its high face c[a] + 1; on a periodic axis the
// high face of the last cell is face n, which mirrors face 0.
[[nodiscard]] Stencil weighted_stencil(const PoissonLevel& L, const float* p, uint32_t i, uint32_t j,
                                       uint32_t k) noexcept {
    Stencil s;
    const uint32_t n[3] = {L.nx, L.ny, L.nz};
    const uint32_t c[3] = {i, j, k};
    for (uint32_t axis = 0; axis < 3; ++axis) {
        for (uint32_t dir = 0; dir < 2; ++dir) {
            uint32_t f[3] = {i, j, k};
            f[axis] = c[axis] + dir;
            const float w = L.w[axis][face_at(L, axis, f[0], f[1], f[2])];
            const bool edge = dir == 0 ? c[axis] == 0 : c[axis] + 1u == n[axis];
            uint32_t nb[3] = {i, j, k};
            if (!edge) {
                nb[axis] = dir == 0 ? c[axis] - 1u : c[axis] + 1u;
                s.sum += w * p[at(L, nb[0], nb[1], nb[2])];
                s.diag += w;
                continue;
            }
            switch (L.kind[2u * axis + dir]) {
                case FaceKind::periodic:
                    if (n[axis] > 1) {
                        nb[axis] = dir == 0 ? n[axis] - 1u : 0u;
                        s.sum += w * p[at(L, nb[0], nb[1], nb[2])];
                        s.diag += w;
                    }
                    break;
                case FaceKind::pressure_outlet:
                    s.diag += 2.0f * w;
                    break;
                case FaceKind::fixed_velocity:
                    break;
            }
        }
    }
    return s;
}

[[nodiscard]] inline Stencil any_stencil(const PoissonLevel& L, const float* p, uint32_t i, uint32_t j,
                                         uint32_t k) noexcept {
    return L.w[0].empty() ? stencil(L, p, i, j, k) : weighted_stencil(L, p, i, j, k);
}
```

`smooth_one_cell()` uses `any_stencil(L, L.p.data(), i, j, k)`.

`smooth_red_black()` takes `black_first`. Its colour loop becomes `for (uint32_t step = 0; step < 2; ++step) { const uint32_t colour = black_first ? 1u - step : step; ... }`. With weights, a level takes no fast path: `const bool fast = L.w[0].empty();`, and the fast branch is `if (fast && row_inside && inside(i, L.nx))`. With `black_first == false` and no weights, the operations and their order are plan A's.

`residual()`: the fast branch gains the same `fast &&`, and the general branch uses `any_stencil(L, L.p.data(), i, j, k)`.

Add:

```cpp
void apply_operator(const PoissonLevel& L, std::span<const float> x, std::span<float> out) noexcept {
    for (uint32_t k = 0; k < L.nz; ++k) {
        for (uint32_t j = 0; j < L.ny; ++j) {
            for (uint32_t i = 0; i < L.nx; ++i) {
                const Stencil st = any_stencil(L, x.data(), i, j, k);
                const std::size_t c = at(L, i, j, k);
                out[c] = st.sum - st.diag * x[c];
            }
        }
    }
}
```

`apply_operator()` uses the general stencil even without weights. Its bits equal `residual()`'s operator: the fast path adds the same terms in the same order (plan A's proof), which `WeightsOfOneAreBitIdentical` checks.

`multigrid.hpp`: add

```cpp
// The face weights of every level (the solids design, section 3; a host call:
// it allocates). Level 0 copies `fine`, the MAC grid's u, v and w apertures.
// Each coarser face is the mean of the faces of the level above on its own
// plane that it covers (2 x 2, or 2 or 1 across an axis that is not
// coarsened), summed with the second transverse axis outer. A coarse face
// covers only every second fine plane: a sheet on another plane is invisible
// to it (finding 2), which is why the solids path solves by CG (pcg.hpp).
void set_weights(Multigrid& mg, const std::array<std::span<const float>, 3>& fine);
// Every level back to plan A's stencil.
void clear_weights(Multigrid& mg) noexcept;
```

`multigrid.cpp`: add

```cpp
void set_weights(Multigrid& mg, const std::array<std::span<const float>, 3>& fine) {
    for (uint32_t a = 0; a < 3; ++a) mg.levels[0].w[a].assign(fine[a].begin(), fine[a].end());
    for (std::size_t l = 1; l < mg.levels.size(); ++l) {
        const PoissonLevel& f = mg.levels[l - 1];
        PoissonLevel& c = mg.levels[l];
        const uint32_t nf[3] = {f.nx, f.ny, f.nz};
        const uint32_t nc[3] = {c.nx, c.ny, c.nz};
        for (uint32_t a = 0; a < 3; ++a) {
            const uint32_t t1 = a == 0 ? 1u : 0u;  // the transverse axes, t1 < t2
            const uint32_t t2 = a == 2 ? 1u : 2u;
            const uint32_t r[3] = {nf[0] == nc[0] ? 1u : 2u, nf[1] == nc[1] ? 1u : 2u, nf[2] == nc[2] ? 1u : 2u};
            const uint32_t fdim[3] = {a == 0 ? nf[0] + 1u : nf[0], a == 1 ? nf[1] + 1u : nf[1], a == 2 ? nf[2] + 1u : nf[2]};
            const uint32_t cdim[3] = {a == 0 ? nc[0] + 1u : nc[0], a == 1 ? nc[1] + 1u : nc[1], a == 2 ? nc[2] + 1u : nc[2]};
            const float inv = 1.0f / static_cast<float>(r[t1] * r[t2]);
            c.w[a].assign(std::size_t{cdim[0]} * cdim[1] * cdim[2], 0.0f);
            for (uint32_t K = 0; K < cdim[2]; ++K) {
                for (uint32_t J = 0; J < cdim[1]; ++J) {
                    for (uint32_t I = 0; I < cdim[0]; ++I) {
                        const uint32_t C[3] = {I, J, K};
                        float sum = 0.0f;
                        for (uint32_t d2 = 0; d2 < r[t2]; ++d2) {
                            for (uint32_t d1 = 0; d1 < r[t1]; ++d1) {
                                uint32_t F[3];
                                F[a] = C[a] * r[a];
                                F[t1] = C[t1] * r[t1] + d1;
                                F[t2] = C[t2] * r[t2] + d2;
                                sum += f.w[a][F[0] + std::size_t{fdim[0]} * (F[1] + std::size_t{fdim[1]} * F[2])];
                            }
                        }
                        c.w[a][I + std::size_t{cdim[0]} * (J + std::size_t{cdim[1]} * K)] = sum * inv;
                    }
                }
            }
        }
    }
}

void clear_weights(Multigrid& mg) noexcept {
    for (PoissonLevel& L : mg.levels) {
        for (std::vector<float>& w : L.w) w.clear();
    }
}
```

A coarse face plane `C[a]` maps to fine plane `C[a] * r[a]`, which is in range: the last coarse plane `nc[a]` maps to `nf[a]`. On a periodic axis that is the fine mirror of plane 0, so the coarse level's last plane mirrors its first too.

- [ ] **Step 4: Run the tests to see them pass**

Run: `--gtest_filter=AirflowSolidsSolve.*:AirflowPressure*:AirflowStep.TheSolverCoresDigestIsPinned:AirflowConfigs.*`
Expected: all PASS. Both goldens are unchanged, as plan A's paths are.

- [ ] **Step 5: The task gate, then commit**

```bash
git add engine/physics/airflow/pressure.hpp engine/physics/airflow/pressure.cpp engine/physics/airflow/multigrid.hpp engine/physics/airflow/multigrid.cpp tests/test_airflow_solids_solve.cpp tests/CMakeLists.txt
git commit -m "feat(physics): face weights in the pressure operator and their coarse means; apply_operator(); weights of 1 bit-identical to plan A (P2 Task 3)"
```

---
### Task 4: The symmetric V-cycle and CG (spec §1.5 as amended; design tests 3's convergence check and 10)

**Files:**
- Modify: `engine/physics/airflow/reduce.hpp`, `engine/physics/airflow/reduce.cpp` (`fixed_tile_dot`)
- Modify: `engine/physics/airflow/multigrid.hpp`, `engine/physics/airflow/multigrid.cpp` (`restrict_full_weighting`, `precondition`)
- Create: `engine/physics/airflow/pcg.hpp`, `engine/physics/airflow/pcg.cpp`
- Modify: `engine/CMakeLists.txt` (`physics/airflow/pcg.cpp` after `physics/airflow/multigrid.cpp`)
- Modify: `tests/test_airflow_solids_solve.cpp`

**Interfaces:**
- Consumes: Task 3's weights, `apply_operator()`, and `smooth_red_black(…, black_first)`.
- Produces:
  - `float fixed_tile_dot(std::span<const float> a, std::span<const float> b, uint32_t nx, uint32_t ny, uint32_t nz) noexcept`
  - `void restrict_full_weighting(const PoissonLevel& fine, PoissonLevel& coarse) noexcept`
  - `inline constexpr uint32_t kCoarsestSymSweeps = 16;` and `void precondition(Multigrid&) noexcept`: levels[0].b in, levels[0].p out
  - `struct PcgGauge { void (*remove)(void* ctx, std::span<float> cells) noexcept; void* ctx; const uint8_t* solid; }`
  - `struct PcgScratch { std::vector<float> x, b, r, z, d, q; }`, `PcgScratch make_pcg_scratch(std::size_t cells)`
  - `float solve_pcg(Multigrid&, PcgScratch&, uint32_t iterations, const PcgGauge&) noexcept`: returns max|b − A x| / max|b| over the cells not marked solid, or 0 when b is 0

- [ ] **Step 1: Write the failing tests**

Add `#include "physics/airflow/pcg.hpp"` and `#include "physics/airflow/reduce.hpp"` to `tests/test_airflow_solids_solve.cpp`, and append before `}  // namespace`:

```cpp
// --- The symmetric V-cycle and CG (P2 Task 4) -------------------------------

// S3's channel: 32 x 8 x 8 cells, every axis periodic, dx 1, and a sheet
// 0.25 dx thick across the whole section, its mid-plane on fine face plane
// `plane` (16: every level sees it; 17: no coarse level does).
[[nodiscard]] std::array<std::vector<float>, 3> sheet_weights(uint32_t plane) {
    const GridShape shape{32, 8, 8, 1.0f};
    airflow_test::Scene scene;
    scene.shapes.push_back(airflow_test::Shape{.name = "plate", .kind = airflow_test::Kind::box,
                                               .centre = {static_cast<float>(plane), 4.0f, 4.0f},
                                               .half = {0.125f, 20.0f, 20.0f}, .min_feature = 0.25f});
    std::vector<SolidPart> parts = scene.parts();
    parts[0].box_min = glm::vec3(static_cast<float>(plane) - 0.125f, 0.0f, 0.0f);
    parts[0].box_max = glm::vec3(static_cast<float>(plane) + 0.125f, 8.0f, 8.0f);
    parts[0].extent = glm::vec3(0.25f, 8.0f, 8.0f);
    SolidField f = *make_solid_field(shape);
    EXPECT_TRUE(build_solids(f, shape, {true, true, true}, parts, scene.sampler()).has_value());
    return f.theta;
}

[[nodiscard]] Multigrid sheet_multigrid(uint32_t plane) {
    const std::array<bool, 3> periodic{true, true, true};
    Multigrid mg = make_multigrid(32, 8, 8, periodic);
    set_kinds(mg, {FaceKind::periodic, FaceKind::periodic, FaceKind::periodic, FaceKind::periodic,
                   FaceKind::periodic, FaceKind::periodic});
    const std::array<std::vector<float>, 3> w = sheet_weights(plane);
    set_weights(mg, {w[0], w[1], w[2]});
    return mg;
}

void remove_mean_gauge(void* ctx, std::span<float> cells) noexcept {
    (void)ctx;
    remove_mean(cells, 32, 8, 8);
}

[[nodiscard]] std::vector<float> apply_m(Multigrid& mg, const std::vector<float>& r) {
    std::copy(r.begin(), r.end(), mg.levels[0].b.begin());
    precondition(mg);
    return mg.levels[0].p;
}

[[nodiscard]] double dot64(const std::vector<float>& a, const std::vector<float>& b) {
    double s = 0.0;
    for (std::size_t i = 0; i < a.size(); ++i) s += static_cast<double>(a[i]) * static_cast<double>(b[i]);
    return s;
}

// Design test 10 (the lead's condition): CG needs a symmetric preconditioner,
// <M r1, r2> = <r1, M r2>, to fp32 rounding, on S3's grid with its sheet on
// an odd plane. Mutation: plan A's restriction (the children's sum) in the
// cycle, which the port measured at 1e-1 of asymmetry.
TEST(AirflowSolidsSolve, ThePreconditionerIsSymmetric) {
    Multigrid mg = sheet_multigrid(17);
    Lcg rng;
    std::vector<float> r1(32u * 8u * 8u);
    std::vector<float> r2(r1.size());
    for (float& v : r1) v = rng.next();
    for (float& v : r2) v = rng.next();
    remove_mean(r1, 32, 8, 8);
    remove_mean(r2, 32, 8, 8);
    const std::vector<float> m1 = apply_m(mg, r1);
    const std::vector<float> m2 = apply_m(mg, r2);
    const double a = dot64(m1, r2);
    const double b = dot64(r1, m2);
    EXPECT_NEAR(a, b, 1.0e-4 * std::fabs(a)) << a << " vs " << b;
    EXPECT_LT(dot64(m1, r1), 0.0) << "negative definite on mean-free vectors, as the operator (sum - diag p) is";
}

// The right side of one projection of a uniform u = 1 blocked by the sheet:
// the aperture divergence theta_{i+1} - theta_i, exact integers.
[[nodiscard]] std::vector<float> blocked_flow_rhs(const std::array<std::vector<float>, 3>& w) {
    std::vector<float> b(32u * 8u * 8u);
    for (uint32_t k = 0; k < 8; ++k) {
        for (uint32_t j = 0; j < 8; ++j) {
            for (uint32_t i = 0; i < 32; ++i) {
                b[i + 32u * (j + 8u * k)] = w[0][(i + 1u) + 33u * (j + 8u * k)] - w[0][i + 33u * (j + 8u * k)];
            }
        }
    }
    return b;
}

// Design test 3's convergence check (the lead's finding 2): from a cold start,
// on both parities, CG preconditioned by the symmetric V-cycle brings the
// relative residual to 1e-3 within 8 iterations. An implementation band
// (TD-2), not a Test/Docs tolerance: the port (Physics' ledger,
// .superpowers/sdd/airflow-solids/mg_sym_port.py) measured 1-2e-3 at 4
// iterations and 1e-8 at 8, in fp64; pinned at 8. Mutation: plan A's plain
// V-cycles on the odd plane, which the port leaves at 0.6 after 8.
TEST(AirflowSolidsSolve, ASheetOnEitherParityConvergesInEightIterations) {
    for (const uint32_t plane : {16u, 17u}) {
        Multigrid mg = sheet_multigrid(plane);
        const std::array<std::vector<float>, 3> w = sheet_weights(plane);
        PcgScratch s = make_pcg_scratch(32u * 8u * 8u);
        s.b = blocked_flow_rhs(w);
        std::fill(s.x.begin(), s.x.end(), 0.0f);
        const float rel = solve_pcg(mg, s, 8, PcgGauge{&remove_mean_gauge, nullptr, nullptr});
        EXPECT_LE(rel, 1.0e-3f) << "plane " << plane;
        std::printf("plane %u: relative residual after 8 = %.3e\n", plane, static_cast<double>(rel));
    }
}

TEST(AirflowSolidsSolve, TheFixedTileDotIsTheTileSumOfProducts) {
    std::vector<float> a(9u * 7u * 5u);
    std::vector<float> b(a.size());
    std::vector<float> ab(a.size());
    Lcg rng;
    for (std::size_t i = 0; i < a.size(); ++i) {
        a[i] = rng.next();
        b[i] = rng.next();
        ab[i] = a[i] * b[i];
    }
    const float d = fixed_tile_dot(a, b, 9, 7, 5);
    const float s = fixed_tile_sum(ab, 9, 7, 5);
    EXPECT_EQ(std::memcmp(&d, &s, sizeof d), 0) << d << " vs " << s;
}
```

- [ ] **Step 2: Run the tests to see them fail**

Expected: a build failure on `pcg.hpp`, `precondition` and `fixed_tile_dot`.

- [ ] **Step 3: Write the implementation**

`reduce.hpp`, add:

```cpp
// The sum of a[i] * b[i] in fixed_tile_sum()'s order, each product rounded to
// fp32 first: CG's dot products (pcg.hpp), so the GPU twin sums them alike.
[[nodiscard]] float fixed_tile_dot(std::span<const float> a, std::span<const float> b, uint32_t nx, uint32_t ny,
                                   uint32_t nz) noexcept;
```

`reduce.cpp`: `fixed_tile_dot` is `fixed_tile_sum`'s body with `tile += a[c] * b[c];`, where `c` is the same index.

`multigrid.hpp`, add:

```cpp
// The coarsest level's sweeps each way in the symmetric cycle.
inline constexpr uint32_t kCoarsestSymSweeps = 16;

// coarse.b = (4 / children) * P^T fine.r, with P prolong_add()'s trilinear
// interpolation: the restriction is the interpolation's transpose (full
// weighting), so the cycle below is symmetric. Gathered per coarse cell from
// the fine cells within one of its children, in (k, j, i) order.
void restrict_full_weighting(const PoissonLevel& fine, PoissonLevel& coarse) noexcept;

// The symmetric V(2,2) cycle, CG's preconditioner (spec 1.5 as amended
// 2026-10-13): from p = 0 on every level, red then black twice, restrict by
// full weighting, recurse, prolong, black then red twice; the coarsest level
// kCoarsestSymSweeps sweeps red-first and as many black-first. levels[0].b is
// the input, levels[0].p the output, and every level's p, b and r are
// scratch. A symmetric operator (design test 10).
void precondition(Multigrid& mg) noexcept;
```

`multigrid.cpp`, add after `restrict_residual`:

```cpp
namespace {

// Along one axis, the fine indices whose interpolation reads coarse index I,
// with the weight they read it by: taps()'s transpose. A coarsened axis:
// 2I - 1 .. 2I + 2, wrapped on a periodic axis, dropped past an edge, a weight
// 0 dropped too; an axis not coarsened: I alone, weight 1.
struct Reach {
    uint32_t n = 0;
    uint32_t idx[4] = {};
    float w[4] = {};
};

[[nodiscard]] Reach reach(uint32_t I, uint32_t nf, uint32_t nc, bool periodic) noexcept {
    Reach r;
    if (nf == nc) {
        r.n = 1;
        r.idx[0] = I;
        r.w[0] = 1.0f;
        return r;
    }
    for (int d = -1; d <= 2; ++d) {
        int64_t i = 2 * int64_t{I} + d;
        if (i < 0 || i >= int64_t{nf}) {
            if (!periodic) continue;
            i = (i + int64_t{nf}) % int64_t{nf};
        }
        const Taps t = taps(static_cast<uint32_t>(i), nf, nc, periodic);
        float w = 0.0f;
        if (t.i0 == I) w += t.w0;
        if (t.i1 == I) w += t.w1;
        if (w == 0.0f) continue;
        r.idx[r.n] = static_cast<uint32_t>(i);
        r.w[r.n] = w;
        ++r.n;
    }
    return r;
}

void cycle_sym(Multigrid& mg, std::size_t l) noexcept {
    PoissonLevel& f = mg.levels[l];
    if (l + 1u == mg.levels.size()) {
        smooth_red_black(f, kCoarsestSymSweeps, false);
        smooth_red_black(f, kCoarsestSymSweeps, true);
        return;
    }
    smooth_red_black(f, kPreSweeps, false);
    residual(f);
    PoissonLevel& c = mg.levels[l + 1u];
    restrict_full_weighting(f, c);
    std::fill(c.p.begin(), c.p.end(), 0.0f);
    cycle_sym(mg, l + 1u);
    prolong_add(c, f);
    smooth_red_black(f, kPostSweeps, true);
}

}  // namespace

void restrict_full_weighting(const PoissonLevel& f, PoissonLevel& c) noexcept {
    const bool px = f.kind[kXMinus] == FaceKind::periodic;
    const bool py = f.kind[kYMinus] == FaceKind::periodic;
    const bool pz = f.kind[kZMinus] == FaceKind::periodic;
    const uint32_t sx = f.nx == c.nx ? 1u : 2u;
    const uint32_t sy = f.ny == c.ny ? 1u : 2u;
    const uint32_t sz = f.nz == c.nz ? 1u : 2u;
    const float scale = 4.0f / static_cast<float>(sx * sy * sz);
    for (uint32_t K = 0; K < c.nz; ++K) {
        const Reach rz = reach(K, f.nz, c.nz, pz);
        for (uint32_t J = 0; J < c.ny; ++J) {
            const Reach ry = reach(J, f.ny, c.ny, py);
            for (uint32_t I = 0; I < c.nx; ++I) {
                const Reach rx = reach(I, f.nx, c.nx, px);
                float sum = 0.0f;
                for (uint32_t a = 0; a < rz.n; ++a) {
                    for (uint32_t b = 0; b < ry.n; ++b) {
                        const float wzy = rz.w[a] * ry.w[b];
                        for (uint32_t d = 0; d < rx.n; ++d) {
                            sum += (wzy * rx.w[d]) * f.r[at(f, rx.idx[d], ry.idx[b], rz.idx[a])];
                        }
                    }
                }
                c.b[at(c, I, J, K)] = sum * scale;
            }
        }
    }
}

void precondition(Multigrid& mg) noexcept {
    if (mg.levels.empty()) return;
    std::fill(mg.levels[0].p.begin(), mg.levels[0].p.end(), 0.0f);
    cycle_sym(mg, 0);
}
```

`taps()` lives in the same anonymous namespace earlier in the file; keep `reach()` after it. `kPreSweeps` and `kPostSweeps` are plan A's (both 2). The fine level's periodic flags come from its `kind`, as `prolong_add()` reads them.

Create `engine/physics/airflow/pcg.hpp`:

```cpp
#pragma once

#include <cstddef>
#include <cstdint>
#include <span>
#include <vector>

#include "physics/airflow/multigrid.hpp"

// ---------------------------------------------------------------------------
// THE PRESSURE SOLVE WITH SOLIDS (spec 1.5 as amended by the lead, 2026-10-13;
// the solids design, section 3): conjugate gradients on levels[0]'s weighted
// operator, preconditioned by one symmetric V(2,2) cycle (precondition()).
// McAdams, Sifakis and Teran (2010), "A parallel multigrid Poisson solver for
// fluids simulation on large grids". A fixed iteration count, nothing stopping
// on a residual. The dot products are reduce.hpp's fixed-tile sums, so the GPU
// twin sums them in the same order. The gauge removes each closed region's
// mean from r, z and x (regions.hpp); cells marked solid hold 0 in z and x.
// ---------------------------------------------------------------------------
namespace spade::physics::airflow {

struct PcgGauge {
    void (*remove)(void* ctx, std::span<float> cells) noexcept = nullptr;  // null: no closed region
    void* ctx = nullptr;
    const uint8_t* solid = nullptr;  // per cell, nonzero: in no fluid region; null: none
};

struct PcgScratch {
    std::vector<float> x;  // the unknown: the caller's initial guess in, the solution out
    std::vector<float> b;  // the right side
    std::vector<float> r, z, d, q;
};

[[nodiscard]] PcgScratch make_pcg_scratch(std::size_t cells);

// `iterations` CG iterations on mg.levels[0]'s operator from s.x. Returns the
// final relative residual max|b - A x| / max|b| over the cells not marked
// solid (0 when b is 0 there); a max is order-free, so both backends report
// the same value. An exact zero in a step's denominator (a solved system)
// ends the iterations early, the only early exit.
float solve_pcg(Multigrid& mg, PcgScratch& s, uint32_t iterations, const PcgGauge& gauge) noexcept;

}  // namespace spade::physics::airflow
```

Create `engine/physics/airflow/pcg.cpp`:

```cpp
#include "physics/airflow/pcg.hpp"

#include <algorithm>
#include <cmath>

#include "physics/airflow/reduce.hpp"

namespace spade::physics::airflow {

PcgScratch make_pcg_scratch(std::size_t cells) {
    PcgScratch s;
    s.x.assign(cells, 0.0f);
    s.b.assign(cells, 0.0f);
    s.r.assign(cells, 0.0f);
    s.z.assign(cells, 0.0f);
    s.d.assign(cells, 0.0f);
    s.q.assign(cells, 0.0f);
    return s;
}

float solve_pcg(Multigrid& mg, PcgScratch& s, uint32_t iterations, const PcgGauge& g) noexcept {
    PoissonLevel& L = mg.levels[0];
    const std::size_t n = s.x.size();
    const auto mask = [&](std::vector<float>& v) {
        if (g.solid == nullptr) return;
        for (std::size_t i = 0; i < n; ++i) {
            if (g.solid[i] != 0u) v[i] = 0.0f;
        }
    };
    const auto gauge = [&](std::vector<float>& v) {
        if (g.remove != nullptr) g.remove(g.ctx, v);
    };
    const auto dot = [&](const std::vector<float>& a, const std::vector<float>& b) {
        return fixed_tile_dot(a, b, L.nx, L.ny, L.nz);
    };
    const auto precond = [&]() {
        std::copy(s.r.begin(), s.r.end(), L.b.begin());
        precondition(mg);
        std::copy(L.p.begin(), L.p.end(), s.z.begin());
        mask(s.z);
        gauge(s.z);
    };
    mask(s.x);
    apply_operator(L, s.x, s.q);
    for (std::size_t i = 0; i < n; ++i) s.r[i] = s.b[i] - s.q[i];
    mask(s.r);
    gauge(s.r);
    precond();
    std::copy(s.z.begin(), s.z.end(), s.d.begin());
    float rz = dot(s.r, s.z);
    for (uint32_t it = 0; it < iterations; ++it) {
        apply_operator(L, s.d, s.q);
        const float dq = dot(s.d, s.q);
        if (dq == 0.0f || rz == 0.0f) break;  // solved exactly: nothing left to reduce
        const float alpha = rz / dq;
        for (std::size_t i = 0; i < n; ++i) {
            s.x[i] += alpha * s.d[i];
            s.r[i] -= alpha * s.q[i];
        }
        gauge(s.r);
        precond();
        const float rz_next = dot(s.r, s.z);
        const float beta = rz_next / rz;
        for (std::size_t i = 0; i < n; ++i) s.d[i] = s.z[i] + beta * s.d[i];
        rz = rz_next;
    }
    gauge(s.x);
    apply_operator(L, s.x, s.q);
    float r_max = 0.0f;
    float b_max = 0.0f;
    for (std::size_t i = 0; i < n; ++i) {
        if (g.solid != nullptr && g.solid[i] != 0u) continue;
        r_max = std::max(r_max, std::fabs(s.b[i] - s.q[i]));
        b_max = std::max(b_max, std::fabs(s.b[i]));
    }
    return b_max > 0.0f ? r_max / b_max : 0.0f;
}

}  // namespace spade::physics::airflow
```

A note on sign: the operator is `sum − diag·x`, which is negative semi-definite. The V-cycle solves `A z = r` for it, so M approximates A⁻¹, also negative. Then `r·z < 0` and `d·Ad < 0`, so α and β come out positive, exactly as for the positive form. Nothing is negated.

- [ ] **Step 4: Run the tests to see them pass**

Run: `--gtest_filter=AirflowSolidsSolve.*`
Expected: PASS. The convergence test prints both planes' residuals. Record them in the ledger and in the test's comment (TD-2).

- [ ] **Step 5: The mutations, by hand**

1. In `cycle_sym()`, call `restrict_residual(f, c)` in place of `restrict_full_weighting(f, c)`. `ThePreconditionerIsSymmetric` must FAIL. Restore.
2. In `ASheetOnEitherParityConvergesInEightIterations`, replace `solve_pcg` with eight `v_cycle(mg)` calls on `mg.levels[0]` with `b` set, each followed by `remove_mean` of p, then compute the same relative residual. Plane 17 must exceed 1e-3 (the port: 0.6). Restore.

Record both in the ledger.

- [ ] **Step 6: The task gate, then commit**

```bash
git add engine/physics/airflow/reduce.hpp engine/physics/airflow/reduce.cpp engine/physics/airflow/multigrid.hpp engine/physics/airflow/multigrid.cpp engine/physics/airflow/pcg.hpp engine/physics/airflow/pcg.cpp engine/CMakeLists.txt tests/test_airflow_solids_solve.cpp
git commit -m "feat(physics): CG preconditioned by a symmetric V(2,2) cycle (full-weighting restriction, red-black forward then reversed), fixed-tile dot products; symmetric to rounding, both sheet parities within 1e-3 in 8 iterations (P2 Task 4; spec 1.5 as amended)"
```

---
### Task 5: Fluid regions — labels, closed-region gauges, the net-flux refusal (design tests 5 and 9)

**Files:**
- Create: `engine/physics/airflow/regions.hpp`, `engine/physics/airflow/regions.cpp`
- Modify: `engine/CMakeLists.txt` (`physics/airflow/regions.cpp` after `physics/airflow/pcg.cpp`)
- Modify: `tests/test_airflow_solids_solve.cpp`

**Interfaces:**
- Consumes: Task 1's `SolidField` (θ), plan A's `face_kind()`, `fixed_normal_velocity()`, `kReduceTile`.
- Produces:
  - `kNoRegion`
  - `struct FluidRegions { std::vector<uint32_t> label; std::vector<uint8_t> solid; uint32_t count; std::vector<uint32_t> cells, first; std::vector<uint8_t> closed, faces; … }`
  - `void label_regions(FluidRegions&, const GridShape&, std::array<bool, 3> periodic, const SolidField&)`
  - `struct RegionFault { uint32_t region; uint32_t cell; uint8_t faces; }`
  - `RegionFault check_regions(FluidRegions&, const GridShape&, const DomainBc&, const SolidField&) noexcept`
  - `void remove_region_means(std::span<float> x, FluidRegions&, const GridShape&) noexcept`
  - `std::string describe_regions(const FluidRegions&)`

- [ ] **Step 1: Write the failing tests**

Add `#include "physics/airflow/boundaries.hpp"`, `#include "physics/airflow/fluid_step.hpp"` and `#include "physics/airflow/regions.hpp"` to `tests/test_airflow_solids_solve.cpp`, and append before `}  // namespace`:

```cpp
// --- Fluid regions (P2 Task 5; design tests 5 and 9) -------------------------

// A 16 x 8 x 8 box, a sheet 0.25 dx thick across it on face plane x = 5: two
// regions, 5 x 64 and 11 x 64 cells.
[[nodiscard]] SolidField cut_box(const GridShape& shape) {
    airflow_test::Scene scene;
    scene.shapes.push_back(airflow_test::Shape{.name = "divider", .kind = airflow_test::Kind::box,
                                               .centre = {5.0f, 4.0f, 4.0f}, .half = {0.125f, 20.0f, 20.0f},
                                               .min_feature = 0.25f});
    SolidField f = *make_solid_field(shape);
    std::vector<SolidPart> parts = scene.parts();
    parts[0].extent = glm::vec3(0.25f, 8.0f, 8.0f);
    EXPECT_TRUE(build_solids(f, shape, {}, parts, scene.sampler()).has_value());
    return f;
}

[[nodiscard]] DomainBc walls() {
    DomainBc bc;
    bc.face.fill(FaceBc::wall);
    return bc;
}

TEST(AirflowRegions, ASheetCutsAClosedBoxIntoTwoRegions) {
    const GridShape shape{16, 8, 8, 1.0f};
    const SolidField f = cut_box(shape);
    FluidRegions r;
    label_regions(r, shape, {}, f);
    ASSERT_EQ(r.count, 2u);
    EXPECT_EQ(r.cells[0], 5u * 64u);
    EXPECT_EQ(r.cells[1], 11u * 64u);
    EXPECT_EQ(r.first[0], 0u);
    EXPECT_EQ(r.first[1], 5u);
    EXPECT_EQ(r.label[cell_index(shape, 4, 3, 3)], 0u);
    EXPECT_EQ(r.label[cell_index(shape, 5, 3, 3)], 1u);
    const RegionFault fault = check_regions(r, shape, walls(), f);
    EXPECT_EQ(fault.region, kNoRegion);
    EXPECT_EQ(describe_regions(r), "fluid regions: 2 (0 with an outlet, 2 closed)");
}

// Design test 9's gauge: each closed region loses its own mean. Mutation: one
// mean over all the fluid (reduce.hpp's remove_mean), which leaves each
// region's mean wherever the values put it.
TEST(AirflowRegions, EachClosedRegionLosesItsOwnMean) {
    const GridShape shape{16, 8, 8, 1.0f};
    const SolidField f = cut_box(shape);
    FluidRegions r;
    label_regions(r, shape, {}, f);
    ASSERT_EQ(check_regions(r, shape, walls(), f).region, kNoRegion);
    std::vector<float> x(cell_index(shape, 15, 7, 7) + 1u);
    Lcg rng;
    for (std::size_t c = 0; c < x.size(); ++c) x[c] = rng.next() + (r.label[c] == 0u ? 3.0f : -1.0f);
    remove_region_means(x, r, shape);
    double mean[2] = {0.0, 0.0};
    for (std::size_t c = 0; c < x.size(); ++c) mean[r.label[c]] += static_cast<double>(x[c]);
    // fp32 rounding of a few hundred values of order 4: about 3e-7.
    EXPECT_NEAR(mean[0] / r.cells[0], 0.0, 1.0e-5);
    EXPECT_NEAR(mean[1] / r.cells[1], 0.0, 1.0e-5);
}

// Design test 5's region refusal: the -x wall moves in at 1 m/s and the +x
// wall out at 1 m/s, so the whole domain balances (check_domain() passes),
// but the sheet leaves the left region taking the inflow alone. Refused,
// naming the region's first cell and its face.
TEST(AirflowRegions, AClosedRegionWithANetPrescribedFluxIsRefused) {
    const GridShape shape{16, 8, 8, 1.0f};
    const SolidField f = cut_box(shape);
    DomainBc bc = walls();
    bc.wall_velocity[kXMinus] = glm::vec3(1.0f, 0.0f, 0.0f);
    bc.wall_velocity[kXPlus] = glm::vec3(1.0f, 0.0f, 0.0f);
    ASSERT_TRUE(check_domain(bc, shape).has_value()) << "the domain as a whole balances";
    FluidRegions r;
    label_regions(r, shape, {}, f);
    const RegionFault fault = check_regions(r, shape, bc, f);
    EXPECT_EQ(fault.region, 0u);
    EXPECT_EQ(fault.cell, 0u);
    EXPECT_EQ(fault.faces, 1u << kXMinus);
}

// An outlet pins its region: open faces in still air are outlets, so neither
// side of the sheet is closed.
TEST(AirflowRegions, AnOutletPinsItsRegion) {
    const GridShape shape{16, 8, 8, 1.0f};
    const SolidField f = cut_box(shape);
    FluidRegions r;
    label_regions(r, shape, {}, f);
    ASSERT_EQ(check_regions(r, shape, DomainBc{}, f).region, kNoRegion);
    EXPECT_EQ(r.closed[0], 0u);
    EXPECT_EQ(r.closed[1], 0u);
    EXPECT_EQ(describe_regions(r), "fluid regions: 2 (2 with an outlet, 0 closed)");
}
```

`cut_box()`'s plate reaches past the grid on y and z. It is clipped there (Task 1), which is what seals the two regions against the walls.

- [ ] **Step 2: Run the tests to see them fail**

Expected: a build failure on `regions.hpp`.

- [ ] **Step 3: Write the implementation**

Create `engine/physics/airflow/regions.hpp`:

```cpp
#pragma once

#include <array>
#include <cstdint>
#include <limits>
#include <span>
#include <string>
#include <vector>

#include "physics/airflow/boundaries.hpp"
#include "physics/airflow/mac_grid.hpp"
#include "physics/airflow/solids.hpp"

// ---------------------------------------------------------------------------
// FLUID REGIONS (the solids design, section 3; the lead's finding 3). A sheet
// or a solid can cut the fluid into separate regions, each closed one with
// its own null space:
//   * a cell is fluid when any of its six faces is open (theta > 0; a domain
//     face counts), and in no region (solid) otherwise;
//   * regions are the connected sets of fluid cells through the open faces
//     between them, across a periodic seam too, labelled by a flood fill from
//     each unlabelled fluid cell in array order, faces -x, +x, -y, +y, -z, +z:
//     deterministic;
//   * each step, a region an open pressure-outlet face reaches is pinned; any
//     other is closed, keeps its own mean (remove_region_means), and must take
//     no net prescribed flux through the domain faces it touches -- walls and
//     inflows, theta-weighted, summed in double over their samples, within
//     1e-6 of their magnitudes (check_domain()'s rule, per region). Immersed
//     rigid walls need no check: a rigid motion's flux through any closed
//     surface of cell faces is exactly 0 (the face-centre rule is exact for a
//     linear field), and the gauge takes the rounding.
// ---------------------------------------------------------------------------
namespace spade::physics::airflow {

inline constexpr uint32_t kNoRegion = std::numeric_limits<uint32_t>::max();

struct FluidRegions {
    std::vector<uint32_t> label;   // per cell: its region, or kNoRegion (solid)
    std::vector<uint8_t> solid;    // per cell: 1 where label is kNoRegion (CG's mask)
    uint32_t count = 0;
    std::vector<uint32_t> cells;   // per region: its cell count
    std::vector<uint32_t> first;   // per region: its first cell, array order
    std::vector<uint8_t> closed;   // per region, this step: no open outlet reaches it
    std::vector<uint8_t> faces;    // per region, this step: a bit per domain side with a prescribed flux into it
    std::vector<double> flux;      // per region, this step: the net outward prescribed flux, m^3/s
    std::vector<double> magnitude;
    std::vector<float> total;      // per region: the masked sums, then the means
    std::vector<float> tile;       // per region: one tile's partial sum
    std::vector<uint8_t> mark;     // per region: met in the current tile
    std::vector<uint32_t> touched; // the regions the current tile met, in first-met order
};

// Labels the fluid cells (a host call: it allocates).
void label_regions(FluidRegions& r, const GridShape& s, std::array<bool, 3> periodic, const SolidField& f);

struct RegionFault {
    uint32_t region = kNoRegion;  // kNoRegion: every closed region balances
    uint32_t cell = 0;            // the region's first cell
    uint8_t faces = 0;            // a bit per domain side whose prescribed flux enters it
};

// This step's status, allocation-free: closed[], and the first closed region
// whose prescribed flux does not balance.
[[nodiscard]] RegionFault check_regions(FluidRegions& r, const GridShape& s, const DomainBc& bc,
                                        const SolidField& f) noexcept;

// x -= each closed region's mean over its own cells. A region's sum is
// reduce.hpp's fixed tiles restricted to its cells: each tile's cells of that
// region in (k, j, i) order from 0, the tiles' partial sums in (tk, tj, ti)
// order, all in fp32; the mean is the sum times the host's reciprocal of its
// cell count, subtracted in array order. Pinned regions are left alone.
void remove_region_means(std::span<float> x, FluidRegions& r, const GridShape& s) noexcept;

// "fluid regions: N (K with an outlet, M closed)", from the last check_regions().
[[nodiscard]] std::string describe_regions(const FluidRegions& r);

}  // namespace spade::physics::airflow
```

Create `engine/physics/airflow/regions.cpp`:

```cpp
#include "physics/airflow/regions.hpp"

#include <algorithm>
#include <cmath>

#include "physics/airflow/reduce.hpp"

namespace spade::physics::airflow {

namespace {

// The face of cell (i, j, k) on side `side` (2 axis + 0 low, 1 high): its
// component and index.
struct FaceRef {
    uint32_t c;
    std::size_t idx;
};

[[nodiscard]] FaceRef face_of(const GridShape& s, uint32_t i, uint32_t j, uint32_t k, uint32_t side) noexcept {
    const uint32_t a = side / 2u;
    const uint32_t up = side % 2u;
    switch (a) {
        case 0: return {0u, u_index(s, i + up, j, k)};
        case 1: return {1u, v_index(s, i, j + up, k)};
        default: return {2u, w_index(s, i, j, k + up)};
    }
}

}  // namespace

void label_regions(FluidRegions& r, const GridShape& s, std::array<bool, 3> periodic, const SolidField& f) {
    const std::size_t cells = std::size_t{s.nx} * s.ny * s.nz;
    r.label.assign(cells, kNoRegion);
    r.solid.assign(cells, 1u);
    r.cells.clear();
    r.first.clear();
    r.count = 0;
    const uint32_t n[3] = {s.nx, s.ny, s.nz};
    const auto fluid = [&](uint32_t i, uint32_t j, uint32_t k) {
        for (uint32_t side = 0; side < 6; ++side) {
            const FaceRef fr = face_of(s, i, j, k, side);
            if (f.theta[fr.c][fr.idx] > 0.0f) return true;
        }
        return false;
    };
    std::vector<uint32_t> queue;
    queue.reserve(cells);
    for (uint32_t k = 0; k < s.nz; ++k) {
        for (uint32_t j = 0; j < s.ny; ++j) {
            for (uint32_t i = 0; i < s.nx; ++i) {
                const std::size_t start = cell_index(s, i, j, k);
                if (r.label[start] != kNoRegion || !fluid(i, j, k)) continue;
                const uint32_t id = r.count++;
                r.cells.push_back(0u);
                r.first.push_back(static_cast<uint32_t>(start));
                queue.clear();
                queue.push_back(static_cast<uint32_t>(start));
                r.label[start] = id;
                for (std::size_t head = 0; head < queue.size(); ++head) {
                    const uint32_t c = queue[head];
                    ++r.cells[id];
                    r.solid[c] = 0u;
                    const uint32_t ci = c % s.nx;
                    const uint32_t cj = (c / s.nx) % s.ny;
                    const uint32_t ck = c / (s.nx * s.ny);
                    const uint32_t q[3] = {ci, cj, ck};
                    for (uint32_t side = 0; side < 6; ++side) {
                        const uint32_t a = side / 2u;
                        const bool up = side % 2u == 1u;
                        const FaceRef fr = face_of(s, ci, cj, ck, side);
                        if (!(f.theta[fr.c][fr.idx] > 0.0f)) continue;
                        uint32_t nb[3] = {ci, cj, ck};
                        if (up ? q[a] + 1u < n[a] : q[a] > 0u) {
                            nb[a] = up ? q[a] + 1u : q[a] - 1u;
                        } else if (periodic[a] && n[a] > 1u) {
                            nb[a] = up ? 0u : n[a] - 1u;
                        } else {
                            continue;  // a domain face: no neighbour
                        }
                        const std::size_t to = cell_index(s, nb[0], nb[1], nb[2]);
                        if (r.label[to] != kNoRegion) continue;
                        r.label[to] = id;
                        queue.push_back(static_cast<uint32_t>(to));
                    }
                }
            }
        }
    }
    r.closed.assign(r.count, 1u);
    r.faces.assign(r.count, 0u);
    r.flux.assign(r.count, 0.0);
    r.magnitude.assign(r.count, 0.0);
    r.total.assign(r.count, 0.0f);
    r.tile.assign(r.count, 0.0f);
    r.mark.assign(r.count, 0u);
    r.touched.assign(std::size_t{kReduceTile} * kReduceTile * kReduceTile, 0u);
}
```

A neighbour reached across a face whose θ > 0 is itself fluid, since that face is one of its own. So the fill never labels a solid cell.

```cpp
RegionFault check_regions(FluidRegions& r, const GridShape& s, const DomainBc& bc, const SolidField& f) noexcept {
    std::fill(r.closed.begin(), r.closed.end(), 1u);
    std::fill(r.faces.begin(), r.faces.end(), 0u);
    std::fill(r.flux.begin(), r.flux.end(), 0.0);
    std::fill(r.magnitude.begin(), r.magnitude.end(), 0.0);
    const uint32_t n[3] = {s.nx, s.ny, s.nz};
    const double dx2 = static_cast<double>(s.dx) * static_cast<double>(s.dx);
    for (uint32_t side = 0; side < 6; ++side) {
        const FaceKind kind = face_kind(bc, static_cast<Side>(side));
        if (kind == FaceKind::periodic) continue;
        const uint32_t a = side / 2u;
        const bool up = side % 2u == 1u;
        const uint32_t t1 = a == 0 ? 1u : 0u;
        const uint32_t t2 = a == 2 ? 1u : 2u;
        const double normal = kind == FaceKind::fixed_velocity ? fixed_normal_velocity(bc, static_cast<Side>(side)) : 0.0;
        for (uint32_t v = 0; v < n[t2]; ++v) {
            for (uint32_t u = 0; u < n[t1]; ++u) {
                uint32_t q[3];
                q[a] = up ? n[a] - 1u : 0u;
                q[t1] = u;
                q[t2] = v;
                const FaceRef fr = face_of(s, q[0], q[1], q[2], side);
                const float theta = f.theta[fr.c][fr.idx];
                if (!(theta > 0.0f)) continue;
                const uint32_t id = r.label[cell_index(s, q[0], q[1], q[2])];
                if (id == kNoRegion) continue;
                if (kind == FaceKind::pressure_outlet) {
                    r.closed[id] = 0u;
                    continue;
                }
                if (normal == 0.0) continue;
                const double outward = (up ? normal : -normal) * static_cast<double>(theta) * dx2;
                r.flux[id] += outward;
                r.magnitude[id] += std::fabs(outward);
                r.faces[id] = static_cast<uint8_t>(r.faces[id] | (1u << side));
            }
        }
    }
    for (uint32_t id = 0; id < r.count; ++id) {
        if (r.closed[id] != 0u && std::fabs(r.flux[id]) > 1.0e-6 * r.magnitude[id]) {
            return RegionFault{id, r.first[id], r.faces[id]};
        }
    }
    return {};
}

void remove_region_means(std::span<float> x, FluidRegions& r, const GridShape& s) noexcept {
    std::fill(r.total.begin(), r.total.end(), 0.0f);
    for (uint32_t tk = 0; tk < s.nz; tk += kReduceTile) {
        const uint32_t k_end = std::min(tk + kReduceTile, s.nz);
        for (uint32_t tj = 0; tj < s.ny; tj += kReduceTile) {
            const uint32_t j_end = std::min(tj + kReduceTile, s.ny);
            for (uint32_t ti = 0; ti < s.nx; ti += kReduceTile) {
                const uint32_t i_end = std::min(ti + kReduceTile, s.nx);
                std::size_t met = 0;
                for (uint32_t k = tk; k < k_end; ++k) {
                    for (uint32_t j = tj; j < j_end; ++j) {
                        for (uint32_t i = ti; i < i_end; ++i) {
                            const std::size_t c = cell_index(s, i, j, k);
                            const uint32_t id = r.label[c];
                            if (id == kNoRegion || r.closed[id] == 0u) continue;
                            if (r.mark[id] == 0u) {
                                r.mark[id] = 1u;
                                r.touched[met++] = id;
                            }
                            r.tile[id] += x[c];
                        }
                    }
                }
                for (std::size_t m = 0; m < met; ++m) {
                    const uint32_t id = r.touched[m];
                    r.total[id] += r.tile[id];
                    r.tile[id] = 0.0f;
                    r.mark[id] = 0u;
                }
            }
        }
    }
    for (uint32_t id = 0; id < r.count; ++id) {
        if (r.closed[id] == 0u) continue;
        r.total[id] *= static_cast<float>(1.0 / static_cast<double>(r.cells[id]));  // the host's reciprocal
    }
    for (std::size_t c = 0; c < x.size(); ++c) {
        const uint32_t id = r.label[c];
        if (id == kNoRegion || r.closed[id] == 0u) continue;
        x[c] -= r.total[id];
    }
}

std::string describe_regions(const FluidRegions& r) {
    uint32_t closed = 0;
    for (const uint8_t c : r.closed) closed += c != 0u ? 1u : 0u;
    return "fluid regions: " + std::to_string(r.count) + " (" + std::to_string(r.count - closed) +
           " with an outlet, " + std::to_string(closed) + " closed)";
}

}  // namespace spade::physics::airflow
```

`check_regions()` sums in double, in the faces' fixed order (sides 0–5, then the outer transverse axis, then the inner), as plan A's `unbalanced_faces()` does. It is a check, so it feeds no stepping value.

- [ ] **Step 4: Run the tests to see them pass**

Run: `--gtest_filter=AirflowRegions.*`
Expected: 4 PASS.

- [ ] **Step 5: The mutation, by hand**

In `EachClosedRegionLosesItsOwnMean`, replace `remove_region_means(x, r, shape)` with `remove_mean(x, 16, 8, 8)`. It must FAIL: the overall mean is 0.25, so the left region keeps a mean near 2.75 and the right one near −1.25. Restore, and record it in the ledger.

- [ ] **Step 6: The task gate, then commit**

```bash
git add engine/physics/airflow/regions.hpp engine/physics/airflow/regions.cpp engine/CMakeLists.txt tests/test_airflow_solids_solve.cpp
git commit -m "feat(physics): fluid regions -- a deterministic flood fill over open faces, a gauge per closed region by masked fixed-tile sums, and the per-region net-flux refusal (P2 Task 5; design tests 5 and 9)"
```

---
### Task 6: The projection with solids, `set_solids()`, and the step's solids path (design tests 3 and 6; S3)

**Files:**
- Modify: `engine/physics/airflow/projection.hpp`, `engine/physics/airflow/projection.cpp`
- Modify: `engine/physics/airflow/solids.hpp`, `engine/physics/airflow/solids.cpp` (`SolidInterior`, `apply_solid_velocities` for the rigid interior)
- Modify: `engine/physics/airflow/fluid_step.hpp`, `engine/physics/airflow/fluid_step.cpp`
- Modify: `tests/airflow_solid_shapes.hpp` (a box shell)
- Create: `tests/test_airflow_solids_step.cpp`
- Modify: `tests/CMakeLists.txt` (`test_airflow_solids_step.cpp` after `test_airflow_solids_solve.cpp`)

**Interfaces:**
- Consumes: Tasks 1–5.
- Produces:
  - `struct ProjectionScratch { Multigrid mg; std::vector<float> div; PcgScratch pcg; }`
  - `struct ProjectionReport { float residual; }`
  - `ProjectionReport project(MacGrid&, const DomainBc&, float rho, float h, uint32_t n_v, ProjectionScratch&, const SolidField* solids = nullptr, FluidRegions* regions = nullptr) noexcept`
  - `enum class SolidInterior : uint8_t { rigid, extrapolate }`; `void apply_solid_velocities(MacGrid&, const SolidField&, SolidInterior) noexcept`
  - `FluidConfig::interior`, `FluidFault::ill_posed_region`, `FluidHealth::pressure_residual`, `FluidHealth::unconverged`, `kResidualAnnounce = 1e-3f`
  - `FluidScratch::solids`, `FluidScratch::regions`
  - `Result<void> set_solids(FluidScratch&, const MacGrid&, const DomainBc&, std::span<const SolidPart>, const SolidSampler&)`

- [ ] **Step 1: A box shell in the test shapes**

In `tests/airflow_solid_shapes.hpp`, add `box_shell` to `Kind`. It is a closed hollow box, outer half extents `half` and wall thickness `radius`:

```cpp
        case Kind::box_shell: {  // the outer box minus the inner: max(outer, -inner)
            const Field outer = box_field(p, s.half);
            const Field inner = box_field(p, s.half - glm::vec3(s.radius));
            f = outer.d >= -inner.d ? outer : Field{-inner.d, -inner.g};
            break;
        }
```

Its `part_of()` local half is `s.half`, like a box.

- [ ] **Step 2: Write the failing tests**

Create `tests/test_airflow_solids_step.cpp`:

```cpp
// The fluid step with solids (the solids design; P2 Tasks 6-9): S3's sheet
// across a channel, S1's P2 checks, S2's Couette flow, rebuilds, plan A's
// bits without solids, and the posed drone. Bands are Test/Docs'
// (test-docs/2026-10-13-solids-tolerances.md) or measured and pinned (TD-2),
// as each says.
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#include "airflow_solid_shapes.hpp"
#include "physics/airflow/fluid_step.hpp"

namespace {

using namespace spade::physics::airflow;
using airflow_test::Kind;
using airflow_test::Scene;
using airflow_test::Shape;

struct Lcg {
    uint32_t state = 12345u;
    float next() noexcept {
        state = state * 1664525u + 1013904223u;
        return static_cast<float>(state >> 8) * (2.0f / 16777216.0f) - 1.0f;
    }
};

[[nodiscard]] bool same_bits(const std::vector<float>& a, const std::vector<float>& b) {
    return a.size() == b.size() && std::memcmp(a.data(), b.data(), a.size() * sizeof(float)) == 0;
}

[[nodiscard]] DomainBc all_periodic() {
    DomainBc bc;
    bc.face.fill(FaceBc::periodic);
    return bc;
}

// S3's channel: L = 32 dx = 1 m, an 8 x 8 section, every axis periodic, and a
// sheet 0.25 dx thick across the section with its mid-plane on face plane
// `plane`.
[[nodiscard]] std::vector<SolidPart> s3_sheet(Scene& scene, uint32_t plane, float dx) {
    const float x = static_cast<float>(plane) * dx;
    scene.shapes.push_back(Shape{.name = "sheet", .kind = Kind::box, .centre = {x, 4.0f * dx, 4.0f * dx},
                                 .half = {0.125f * dx, 20.0f * dx, 20.0f * dx}, .min_feature = 0.25f * dx});
    std::vector<SolidPart> parts = scene.parts();
    parts[0].extent = glm::vec3(0.25f * dx, 8.0f * dx, 8.0f * dx);
    return parts;
}

// Design test 3 / S3 (Test/Docs' tolerances): one projection of u = U blocked
// by the sheet, from a cold start, n_v = 32 CG iterations, on an even and an
// odd plane. Exact in 1D: u = 0 everywhere after it, and p rises along x by
// rho U / h per metre, dropping by rho U (L - dx) / h across the sheet
// between the cell centres either side.
TEST(AirflowSolidsStep, S3ASheetAcrossAChannelStopsItsFlow) {
    const float dx = 1.0f / 32.0f;
    const GridShape shape{32, 8, 8, dx};
    const DomainBc bc = all_periodic();
    const float rho = 1.225f;
    const float h = 0.001f;
    const float U = 1.0f;
    for (const uint32_t plane : {16u, 17u}) {
        MacGrid g = *make_grid(shape, bc);
        std::fill(g.u.begin(), g.u.end(), U);
        const FluidConfig cfg{.rho = rho, .nu = 0.0f, .h = h, .n_v = 32};
        FluidScratch s = *make_fluid_scratch(g, bc, cfg);
        Scene scene;
        const std::vector<SolidPart> parts = s3_sheet(scene, plane, dx);
        ASSERT_TRUE(set_solids(s, g, bc, parts, scene.sampler()).has_value());
        apply_solid_velocities(g, s.solids, SolidInterior::rigid);
        ASSERT_EQ(check_regions(s.regions, shape, bc, s.solids).region, kNoRegion);
        const ProjectionReport rep = project(g, bc, rho, h, cfg.n_v, s.projection, &s.solids, &s.regions);

        float u_max = 0.0f;
        for (const std::vector<float>* a : {&g.u, &g.v, &g.w}) {
            for (const float v : *a) u_max = std::max(u_max, std::fabs(v));
        }
        bool closed_hold_the_wall = true;
        for (uint32_t k = 0; k < 8; ++k) {
            for (uint32_t j = 0; j < 8; ++j) {
                closed_hold_the_wall = closed_hold_the_wall && g.u[u_index(shape, plane, j, k)] == 0.0f;
            }
        }
        const double step = static_cast<double>(g.p[cell_index(shape, plane - 1u, 3, 3)]) -
                            static_cast<double>(g.p[cell_index(shape, plane, 3, 3)]);
        const double expected = static_cast<double>(rho) * U * (1.0 - dx) / h;
        std::printf("=== validation record: S3 sheet across a channel, plane %u (%s) ===\n"
                    "config: 32x8x8 periodic, dx 1/32 m, a sheet 0.25 dx thick, rho %.3f, U %.1f, h %.4f, one "
                    "projection, n_v 32 CG iterations from a cold start\n"
                    "measure: every closed face carries the wall velocity = %s\n"
                    "measure: max |u| / U after the projection = %.3e\n"
                    "reference: 0 (exact in 1D)\n"
                    "tolerance: <= 1e-3 (test-docs/2026-10-13-solids-tolerances.md, fixed before the first run)\n"
                    "verdict: %s\n"
                    "measure: the pressure step dp h / (rho U (L - dx)) = %.6f\n"
                    "reference: 1 (exact: the stencil is exact for a linear field)\n"
                    "tolerance: 1 +/- 1e-3 (test-docs/2026-10-13-solids-tolerances.md, fixed before the first run)\n"
                    "verdict: %s\n"
                    "solve: relative residual %.3e\n"
                    "%s\n",
                    plane, plane % 2u == 0u ? "even" : "odd", static_cast<double>(rho), static_cast<double>(U),
                    static_cast<double>(h), closed_hold_the_wall ? "yes" : "NO",
                    static_cast<double>(u_max / U), u_max / U <= 1.0e-3f ? "within" : "OUTSIDE", step / expected,
                    std::fabs(step / expected - 1.0) <= 1.0e-3 ? "within" : "OUTSIDE",
                    static_cast<double>(rep.residual), s.solids.run_line.c_str());
        EXPECT_TRUE(closed_hold_the_wall) << plane;
        EXPECT_LE(u_max / U, 1.0e-3f) << plane;
        EXPECT_NEAR(step / expected, 1.0, 1.0e-3) << plane;
        EXPECT_NE(s.solids.run_line.find("sheet: sheet (7.8 mm < 2 dx = 62.5 mm)"), std::string::npos)
            << s.solids.run_line;
    }
}

// With n_v = 1 on the odd plane the solve cannot converge: the step reports
// its relative residual and announces it (L6), and does not refuse.
TEST(AirflowSolidsStep, AnUnconvergedSolveIsAnnounced) {
    const float dx = 1.0f / 32.0f;
    const GridShape shape{32, 8, 8, dx};
    const DomainBc bc = all_periodic();
    MacGrid g = *make_grid(shape, bc);
    std::fill(g.u.begin(), g.u.end(), 1.0f);
    const FluidConfig cfg{.rho = 1.225f, .nu = 0.0f, .h = 0.001f, .n_v = 1};
    FluidScratch s = *make_fluid_scratch(g, bc, cfg);
    Scene scene;
    ASSERT_TRUE(set_solids(s, g, bc, s3_sheet(scene, 17, dx), scene.sampler()).has_value());
    const FluidHealth health = fluid_step(g, bc, cfg, s);
    EXPECT_EQ(health.fault, FluidFault::none);
    EXPECT_GT(health.pressure_residual, kResidualAnnounce);
    EXPECT_TRUE(health.unconverged);
    EXPECT_NE(describe(health, shape).find("announced"), std::string::npos) << describe(health, shape);
}

[[nodiscard]] uint64_t digest(const MacGrid& g) {
    uint64_t h = 1469598103934665603ull;
    for (const std::vector<float>* v : {&g.u, &g.v, &g.w, &g.p}) {
        const auto* bytes = reinterpret_cast<const unsigned char*>(v->data());
        for (std::size_t i = 0; i < v->size() * sizeof(float); ++i) {
            h ^= bytes[i];
            h *= 1099511628211ull;
        }
    }
    return h;
}

// Design test 6: with no parts -- never set, set empty, or set and then
// cleared -- four steps of a walled box with a moving lid are plan A's bits.
TEST(AirflowSolidsStep, NoPartsIsPlanABitForBit) {
    const GridShape shape{12, 10, 8, 0.1f};
    DomainBc bc;
    bc.face.fill(FaceBc::wall);
    bc.wall_velocity[kYPlus] = glm::vec3(1.0f, 0.0f, 0.0f);
    const FluidConfig cfg{.rho = 1.225f, .nu = 0.01f, .h = 0.01f, .n_v = 2};
    const auto run = [&](int how) {
        MacGrid g = *make_grid(shape, bc);
        Lcg rng;
        for (std::vector<float>* v : {&g.u, &g.v, &g.w}) {
            for (float& x : *v) x = 0.1f * rng.next();
        }
        FluidScratch s = *make_fluid_scratch(g, bc, cfg);
        Scene scene;
        if (how >= 1) EXPECT_TRUE(set_solids(s, g, bc, {}, scene.sampler()).has_value());
        if (how == 2) {
            scene.shapes.push_back(Shape{.name = "ball", .kind = Kind::sphere, .centre = {0.6f, 0.5f, 0.4f},
                                         .radius = 0.2f, .min_feature = 0.4f});
            const std::vector<SolidPart> parts = scene.parts();
            EXPECT_TRUE(set_solids(s, g, bc, parts, scene.sampler()).has_value());
            EXPECT_TRUE(s.solids.any);
            EXPECT_TRUE(set_solids(s, g, bc, {}, scene.sampler()).has_value());
        }
        for (int step = 0; step < 4; ++step) EXPECT_EQ(fluid_step(g, bc, cfg, s).fault, FluidFault::none);
        return digest(g);
    };
    const uint64_t plain = run(0);
    EXPECT_EQ(run(1), plain);
    EXPECT_EQ(run(2), plain);
}

// Review focus 1: a rebuild at a new pose leaves nothing of the last one and
// grows no array: the scratch rebuilt from pose A to pose B equals a fresh
// scratch built at B, bit for bit, with the same capacities as after A.
TEST(AirflowSolidsStep, ARebuildLeavesNothingOfTheLastPose) {
    const GridShape shape{24, 24, 24, 0.05f};
    const DomainBc bc;
    const FluidConfig cfg{.rho = 1.225f, .nu = 0.0f, .h = 0.002f, .n_v = 4};
    MacGrid g = *make_grid(shape, bc);
    const auto pose = [](float x) {
        Scene scene;
        scene.shapes.push_back(Shape{.name = "body", .kind = Kind::box, .centre = {x, 0.6f, 0.6f},
                                     .half = {0.15f, 0.1f, 0.2f}, .omega = {0.0f, 2.0f, 0.0f}, .min_feature = 0.2f});
        return scene;
    };
    FluidScratch moved = *make_fluid_scratch(g, bc, cfg);
    const Scene a = pose(0.45f);
    ASSERT_TRUE(set_solids(moved, g, bc, a.parts(), a.sampler()).has_value());
    const std::size_t cap_theta = moved.solids.theta[0].capacity();
    const std::size_t cap_label = moved.regions.label.capacity();
    const std::size_t cap_x = moved.projection.pcg.x.capacity();
    const Scene b = pose(0.75f);
    ASSERT_TRUE(set_solids(moved, g, bc, b.parts(), b.sampler()).has_value());
    FluidScratch fresh = *make_fluid_scratch(g, bc, cfg);
    ASSERT_TRUE(set_solids(fresh, g, bc, b.parts(), b.sampler()).has_value());
    for (uint32_t c = 0; c < 3; ++c) {
        EXPECT_TRUE(same_bits(moved.solids.theta[c], fresh.solids.theta[c])) << c;
        EXPECT_TRUE(same_bits(moved.solids.wall[c], fresh.solids.wall[c])) << c;
        EXPECT_TRUE(same_bits(moved.solids.phi[c], fresh.solids.phi[c])) << c;
        EXPECT_EQ(moved.solids.link[c], fresh.solids.link[c]) << c;
    }
    EXPECT_EQ(moved.regions.label, fresh.regions.label);
    for (std::size_t l = 0; l < fresh.projection.mg.levels.size(); ++l) {
        for (uint32_t c = 0; c < 3; ++c) {
            EXPECT_TRUE(same_bits(moved.projection.mg.levels[l].w[c], fresh.projection.mg.levels[l].w[c])) << l;
        }
    }
    EXPECT_EQ(moved.solids.theta[0].capacity(), cap_theta);
    EXPECT_EQ(moved.regions.label.capacity(), cap_label);
    EXPECT_EQ(moved.projection.pcg.x.capacity(), cap_x);
}

// Review focus 5: a closed box shell (a sheet, by its 1 mm wall) seals a
// pocket of cells in an open domain. The pocket is its own closed region and
// keeps its gauge -- its mean pressure 0 -- step after step, while the outside
// stays pinned by the outlets.
TEST(AirflowSolidsStep, ASealedPocketKeepsItsGaugeOverManySteps) {
    const GridShape shape{24, 24, 24, 0.05f};
    const DomainBc bc;
    const FluidConfig cfg{.rho = 1.225f, .nu = 0.0f, .h = 0.002f, .n_v = 8};
    MacGrid g = *make_grid(shape, bc);
    Lcg rng;
    for (std::vector<float>* v : {&g.u, &g.v, &g.w}) {
        for (float& x : *v) x = rng.next();
    }
    FluidScratch s = *make_fluid_scratch(g, bc, cfg);
    Scene scene;
    scene.shapes.push_back(Shape{.name = "canopy", .kind = Kind::box_shell, .centre = {0.6f, 0.6f, 0.6f},
                                 .half = {0.22f, 0.22f, 0.22f}, .radius = 0.001f, .min_feature = 0.001f});
    ASSERT_TRUE(set_solids(s, g, bc, scene.parts(), scene.sampler()).has_value());
    ASSERT_EQ(s.regions.count, 2u) << "outside and the pocket";
    for (int step = 0; step < 40; ++step) {
        const FluidHealth health = fluid_step(g, bc, cfg, s);
        ASSERT_EQ(health.fault, FluidFault::none) << describe(health, shape);
        uint32_t pocket = kNoRegion;
        for (uint32_t r = 0; r < s.regions.count; ++r) {
            if (s.regions.closed[r] != 0u) pocket = r;
        }
        ASSERT_NE(pocket, kNoRegion);
        double sum = 0.0;
        float p_max = 0.0f;
        for (std::size_t c = 0; c < g.p.size(); ++c) {
            if (s.regions.label[c] != pocket) continue;
            sum += static_cast<double>(g.p[c]);
            p_max = std::max(p_max, std::fabs(g.p[c]));
        }
        EXPECT_LE(std::fabs(sum / s.regions.cells[pocket]), 1.0e-5 * std::max(p_max, 1.0f)) << "step " << step;
    }
    EXPECT_EQ(describe_regions(s.regions), "fluid regions: 2 (1 with an outlet, 1 closed)");
}

}  // namespace
```

In `S3ASheetAcrossAChannelStopsItsFlow`, the sheet's thickness at dx = 1/32 m is 0.25 dx = 7.8 mm, and 2 dx = 62.5 mm. That gives the run line's text.

- [ ] **Step 3: Run the tests to see them fail**

Expected: a build failure on `set_solids`, `apply_solid_velocities`, `ProjectionReport`, the new `project()` arguments and `FluidHealth`'s new fields.

- [ ] **Step 4: Write the implementation**

`solids.hpp`, add:

```cpp
// What a closed sample (theta = 0) holds between projections (the solids
// design, section 3, as approved 2026-10-13): `rigid`, the owner's rigid
// velocity, so a backtrace into a solid reads the rigid field (exact deep in
// it: trilinear interpolation reproduces a linear field); `extrapolate`, the
// free-slip lean mode, two layers filled from the open side (Task 7).
enum class SolidInterior : uint8_t { rigid, extrapolate };

// Writes the closed samples per `mode`. Allocation-free.
void apply_solid_velocities(MacGrid& g, const SolidField& f, SolidInterior mode) noexcept;
```

`solids.cpp`, add:

```cpp
void apply_solid_velocities(MacGrid& g, const SolidField& f, SolidInterior mode) noexcept {
    if (!f.any) return;
    std::vector<float>* vel[3] = {&g.u, &g.v, &g.w};
    for (uint32_t c = 0; c < 3; ++c) {
        if (f.range[c].empty) continue;
        const std::array<uint32_t, 3> dims = sample_dims(g.shape, c);
        const SolidRange& rg = f.range[c];
        for (uint32_t k = rg.lo[2]; k <= rg.hi[2]; ++k) {
            for (uint32_t j = rg.lo[1]; j <= rg.hi[1]; ++j) {
                for (uint32_t i = rg.lo[0]; i <= rg.hi[0]; ++i) {
                    const std::size_t idx = i + std::size_t{dims[0]} * (j + std::size_t{dims[1]} * k);
                    if (closed(f.theta[c][idx])) (*vel[c])[idx] = f.wall[c][idx];
                }
            }
        }
    }
    (void)mode;  // extrapolate: Task 7
}
```

On a periodic axis the copied last plane can lie outside `range`. Pass 1 copied its θ and wall, and the step's `apply_velocity_boundaries()` mirrors the velocity there.

`projection.hpp`:
- includes `pcg.hpp`, `regions.hpp` and `solids.hpp`;
- `ProjectionScratch` gains `PcgScratch pcg;  // sized by set_solids(); empty without solids`;
- `project()` becomes:

```cpp
struct ProjectionReport {
    float residual = 0.0f;  // with solids: max|r| / max|b| after the solve (pcg.hpp); 0 on plan A's path
};

// WITH SOLIDS (solids->any, the solids design, section 3): the right side is
// (rho dx^2 / h) times the aperture divergence of theta u* + (1 - theta)
// u_wall, 0 in solid cells; each closed region's mean is removed; n_v
// iterations of solve_pcg() from the grid's p, the regions' gauge and solid
// mask with it; then every face with theta > 0 subtracts the pressure
// gradient as plan A does, and every face with theta = 0 takes the wall's
// velocity. The multigrid's weights are set by set_solids() (fluid_step.hpp),
// the regions' status by check_regions(). Without solids: plan A, bit for bit.
ProjectionReport project(MacGrid& g, const DomainBc& bc, float rho, float h, uint32_t n_v, ProjectionScratch& s,
                         const SolidField* solids = nullptr, FluidRegions* regions = nullptr) noexcept;
```

`projection.cpp`: plan A's body stays as it is, ending in `return {};`. At its top:

```cpp
    if (solids != nullptr && solids->any && regions != nullptr) {
        return project_solids(g, bc, rho, h, n_v, s, *solids, *regions);
    }
```

In the anonymous namespace:

```cpp
struct GaugeCtx {
    FluidRegions* regions;
    const GridShape* shape;
};

void gauge_regions(void* ctx, std::span<float> x) noexcept {
    const auto* c = static_cast<const GaugeCtx*>(ctx);
    remove_region_means(x, *c->regions, *c->shape);
}

// subtract_gradient() with apertures: a closed face (theta = 0) takes the
// wall's velocity; an open one subtracts as plan A does. A fixed-velocity
// domain face is left as the boundaries wrote it.
template <class Vel, class Cell, class Face>
void subtract_gradient_solids(uint32_t n, uint32_t m1, uint32_t m2, FaceKind lo, FaceKind hi, float g_coef,
                              const std::vector<float>& p, Vel vel, Cell cell, Face face, const std::vector<float>& theta,
                              const std::vector<float>& wall) noexcept {
    for (uint32_t b = 0; b < m2; ++b) {
        for (uint32_t a = 0; a < m1; ++a) {
            for (uint32_t q = 1; q < n; ++q) {
                const std::size_t f = face(q, a, b);
                if (closed(theta[f])) {
                    vel(q, a, b) = wall[f];
                } else {
                    vel(q, a, b) -= g_coef * (p[cell(q, a, b)] - p[cell(q - 1u, a, b)]);
                }
            }
            const float p_first = p[cell(0, a, b)];
            const float p_last = p[cell(n - 1u, a, b)];
            const std::size_t f0 = face(0, a, b);
            const std::size_t fn = face(n, a, b);
            if (lo == FaceKind::periodic) {
                if (closed(theta[f0])) {
                    vel(0, a, b) = wall[f0];
                } else if (n > 1u) {
                    vel(0, a, b) -= g_coef * (p_first - p_last);
                }
                vel(n, a, b) = vel(0, a, b);
                continue;
            }
            if (lo == FaceKind::pressure_outlet) {
                if (closed(theta[f0])) {
                    vel(0, a, b) = wall[f0];
                } else {
                    vel(0, a, b) -= g_coef * (p_first - (-p_first));
                }
            }
            if (hi == FaceKind::pressure_outlet) {
                if (closed(theta[fn])) {
                    vel(n, a, b) = wall[fn];
                } else {
                    vel(n, a, b) -= g_coef * ((-p_last) - p_last);
                }
            }
        }
    }
}

ProjectionReport project_solids(MacGrid& g, const DomainBc& bc, float rho, float h, uint32_t n_v,
                                ProjectionScratch& s, const SolidField& f, FluidRegions& regions) noexcept {
    const GridShape& sh = g.shape;
    const std::array<FaceKind, 6> kinds = face_kinds(bc);
    set_kinds(s.mg, kinds);
    const float inv_dx = 1.0f / sh.dx;
    const float b_coef = rho * sh.dx * sh.dx / h;
    const auto flux = [&](uint32_t c, std::size_t idx, const std::vector<float>& vel) {
        const float t = f.theta[c][idx];
        return t * vel[idx] + (1.0f - t) * f.wall[c][idx];
    };
    for (uint32_t k = 0; k < sh.nz; ++k) {
        for (uint32_t j = 0; j < sh.ny; ++j) {
            for (uint32_t i = 0; i < sh.nx; ++i) {
                const std::size_t c = cell_index(sh, i, j, k);
                if (regions.label[c] == kNoRegion) {
                    s.pcg.b[c] = 0.0f;
                    continue;
                }
                const float du = flux(0, u_index(sh, i + 1u, j, k), g.u) - flux(0, u_index(sh, i, j, k), g.u);
                const float dv = flux(1, v_index(sh, i, j + 1u, k), g.v) - flux(1, v_index(sh, i, j, k), g.v);
                const float dw = flux(2, w_index(sh, i, j, k + 1u), g.w) - flux(2, w_index(sh, i, j, k), g.w);
                s.pcg.b[c] = b_coef * (((du + dv) + dw) * inv_dx);
            }
        }
    }
    remove_region_means(s.pcg.b, regions, sh);
    std::copy(g.p.begin(), g.p.end(), s.pcg.x.begin());
    GaugeCtx ctx{&regions, &sh};
    const float residual = solve_pcg(s.mg, s.pcg, n_v, PcgGauge{&gauge_regions, &ctx, regions.solid.data()});
    std::copy(s.pcg.x.begin(), s.pcg.x.end(), g.p.begin());

    const float g_coef = h / (rho * sh.dx);
    subtract_gradient_solids(
        sh.nx, sh.ny, sh.nz, kinds[kXMinus], kinds[kXPlus], g_coef, g.p,
        [&](uint32_t q, uint32_t a, uint32_t b) -> float& { return g.u[u_index(sh, q, a, b)]; },
        [&](uint32_t q, uint32_t a, uint32_t b) { return cell_index(sh, q, a, b); },
        [&](uint32_t q, uint32_t a, uint32_t b) { return u_index(sh, q, a, b); }, f.theta[0], f.wall[0]);
    subtract_gradient_solids(
        sh.ny, sh.nx, sh.nz, kinds[kYMinus], kinds[kYPlus], g_coef, g.p,
        [&](uint32_t q, uint32_t a, uint32_t b) -> float& { return g.v[v_index(sh, a, q, b)]; },
        [&](uint32_t q, uint32_t a, uint32_t b) { return cell_index(sh, a, q, b); },
        [&](uint32_t q, uint32_t a, uint32_t b) { return v_index(sh, a, q, b); }, f.theta[1], f.wall[1]);
    subtract_gradient_solids(
        sh.nz, sh.nx, sh.ny, kinds[kZMinus], kinds[kZPlus], g_coef, g.p,
        [&](uint32_t q, uint32_t a, uint32_t b) -> float& { return g.w[w_index(sh, a, b, q)]; },
        [&](uint32_t q, uint32_t a, uint32_t b) { return cell_index(sh, a, b, q); },
        [&](uint32_t q, uint32_t a, uint32_t b) { return w_index(sh, a, b, q); }, f.theta[2], f.wall[2]);
    return {residual};
}
```

`fluid_step.hpp`:
- includes `regions.hpp` and `solids.hpp`;
- `FluidConfig` gains `SolidInterior interior = SolidInterior::rigid;  // what closed samples hold (solids.hpp)`;
- `FluidFault` gains `ill_posed_region,  // a closed fluid region whose prescribed domain fluxes do not balance (regions.hpp)`;
- `FluidHealth` gains:

```cpp
    float pressure_residual = 0.0f;  // with solids: this step's max|r| / max|b| (pcg.hpp)
    bool unconverged = false;        // pressure_residual > kResidualAnnounce: announced (L6), not refused
```

For `ill_posed_region`, `index` holds the region's first cell and `faces` its faces.
- Add the declarations:

```cpp
// A pressure solve leaving more than this relative residual is announced (the
// lead's condition, spec 1.5 as amended): a first value, revised with the
// iteration count chosen from S1, S3 and the dense-parts case.
inline constexpr float kResidualAnnounce = 1.0e-3f;
```

- `FluidScratch` gains `SolidField solids;  // set_solids(); empty: plan A's paths` and `FluidRegions regions;`;
- add:

```cpp
// Sets the scratch's solids from `parts` (a host call: on first use it
// allocates the solid field, and every call relabels the regions and resets
// the multigrid's weights and CG's vectors to their sizes). No parts: plan A's
// paths, bit for bit. Refused as build_solids() refuses, leaving no solid.
[[nodiscard]] Result<void> set_solids(FluidScratch& s, const MacGrid& g, const DomainBc& bc,
                                      std::span<const SolidPart> parts, const SolidSampler& sample);
```

`fluid_step.cpp`:

```cpp
Result<void> set_solids(FluidScratch& s, const MacGrid& g, const DomainBc& bc, std::span<const SolidPart> parts,
                        const SolidSampler& sample) {
    if (s.solids.theta[0].empty()) {
        Result<SolidField> made = make_solid_field(g.shape);
        if (!made) return std::unexpected(made.error());
        s.solids = std::move(*made);
    }
    const std::array<bool, 3> periodic = periodic_axes(bc);
    if (Result<void> ok = build_solids(s.solids, g.shape, periodic, parts, sample); !ok) {
        clear_weights(s.projection.mg);
        return ok;
    }
    if (!s.solids.any) {
        clear_weights(s.projection.mg);
        return {};
    }
    label_regions(s.regions, g.shape, periodic, s.solids);
    set_weights(s.projection.mg, {s.solids.theta[0], s.solids.theta[1], s.solids.theta[2]});
    if (s.projection.pcg.x.size() != g.p.size()) s.projection.pcg = make_pcg_scratch(g.p.size());
    return {};
}
```

`describe()` gains:

```cpp
        case FluidFault::ill_posed_region: {
            const uint64_t c = health.index;
            return "airflow fluid step: a closed fluid region (its first cell (" + std::to_string(c % s.nx) + ", " +
                   std::to_string((c / s.nx) % s.ny) + ", " + std::to_string(c / (uint64_t{s.nx} * s.ny)) +
                   ")) has no outlet and takes a net prescribed flux through faces " + face_list(health.faces);
        }
```

Its `none` case returns, when `health.unconverged`:

```cpp
            if (health.unconverged) {
                return "airflow fluid step: healthy, but the pressure solve left a relative residual of " +
                       std::to_string(health.pressure_residual) + ", above " + std::to_string(kResidualAnnounce) +
                       " (announced, L6)";
            }
            return "airflow fluid step: healthy";
```

`fluid_step()` becomes:

```cpp
FluidHealth fluid_step(MacGrid& g, const DomainBc& bc, const FluidConfig& cfg, FluidScratch& s) noexcept {
    if (s.health.fault != FluidFault::none) return s.health;
    const bool solid = s.solids.any;
    if (solid) {
        if (const RegionFault rf = check_regions(s.regions, g.shape, bc, s.solids); rf.region != kNoRegion) {
            s.health = FluidHealth{.fault = FluidFault::ill_posed_region, .faces = rf.faces, .index = rf.cell};
            return s.health;
        }
    } else if (const uint8_t faces = unbalanced_faces(bc, g.shape); faces != 0u) {
        s.health = FluidHealth{.fault = FluidFault::ill_posed_domain, .faces = faces};
        return s.health;
    }
    apply_velocity_boundaries(g, bc);
    if (solid) apply_solid_velocities(g, s.solids, cfg.interior);
    advect_velocity(g, s.next, bc, cfg.h, s.advect);
    g.u.swap(s.next.u);
    g.v.swap(s.next.v);
    g.w.swap(s.next.w);
    add_and_clear(g.u, s.du);
    add_and_clear(g.v, s.dv);
    add_and_clear(g.w, s.dw);
    apply_velocity_boundaries(g, bc);
    if (solid) apply_solid_velocities(g, s.solids, cfg.interior);
    if (cfg.nu > 0.0f) {
        diffuse(g, bc, cfg.nu, cfg.h, cfg.visc_sweeps, s.rhs);
        apply_velocity_boundaries(g, bc);
    }
    const ProjectionReport rep =
        project(g, bc, cfg.rho, cfg.h, cfg.n_v, s.projection, solid ? &s.solids : nullptr, solid ? &s.regions : nullptr);
    s.health = scan(g);
    if (solid && s.health.fault == FluidFault::none) {
        s.health.pressure_residual = rep.residual;
        s.health.unconverged = rep.residual > kResidualAnnounce;
    }
    return s.health;
}
```

Without solids, every call and its order is plan A's.

- [ ] **Step 5: Run the tests to see them pass**

Run: `--gtest_filter=AirflowSolidsStep.*:AirflowStep.*:AirflowConfigs.*`
Expected: all PASS, and both goldens are unchanged. S3 prints its two record blocks. Paste them into `tests/data/airflow/records/s3-sheet.md` at Task 10, with commit and date lines after each opening line.

- [ ] **Step 6: The task gate, a Docker leg at the head, then commit**

```bash
git add engine/physics/airflow/projection.hpp engine/physics/airflow/projection.cpp engine/physics/airflow/solids.hpp engine/physics/airflow/solids.cpp engine/physics/airflow/fluid_step.hpp engine/physics/airflow/fluid_step.cpp tests/airflow_solid_shapes.hpp tests/test_airflow_solids_step.cpp tests/CMakeLists.txt
git commit -m "feat(physics): the aperture projection under CG with the regions' gauges, set_solids(), rigid-filled closed samples and the step's solids path; the residual reported and announced past 1e-3 (P2 Task 6; S3 within both parities; plan A bit for bit without solids)"
```

---
### Task 7: The free-slip interior and S1's P2 checks (design test 1, P2 part)

**Files:**
- Modify: `engine/physics/airflow/solids.hpp`, `engine/physics/airflow/solids.cpp` (`SolidField::layer`, the layers in `build_solids()`, `extrapolate` in `apply_solid_velocities()`)
- Modify: `engine/physics/airflow/fluid_step.cpp` (`make_fluid_scratch()` refuses the free-slip interior with ν > 0)
- Modify: `tests/test_airflow_solids.cpp`, `tests/test_airflow_solids_step.cpp`

**Interfaces:**
- Consumes: Task 6's `apply_solid_velocities()` and `SolidInterior`.
- Produces: `SolidField::layer` (`std::array<std::vector<uint8_t>, 3>`: 0 for none, 1 and 2 for the free-slip layers). `fill1` and `fill2` are filled, and `extrapolate` is implemented.

- [ ] **Step 1: Write the failing tests**

Append to `tests/test_airflow_solids.cpp`, before `}  // namespace`:

```cpp
// The free-slip interior (design section 3, approved 2026-10-13): a closed
// sample next to an open one of its component takes their mean; a closed
// sample next to that first layer takes the mean of its first-layer
// neighbours; deeper ones keep the rigid velocity. A wall filling x >= 8.
TEST(AirflowSolids, TheFreeSlipFillCopiesTheOpenSideIntoTwoLayers) {
    const GridShape shape{16, 8, 8, 1.0f};
    Scene scene;
    scene.shapes.push_back(Shape{.name = "wall", .kind = Kind::box, .centre = {16.0f, 4.0f, 4.0f},
                                 .half = {8.0f, 20.0f, 20.0f}, .min_feature = 16.0f});
    const SolidField f = built(shape, scene);
    MacGrid g = *make_grid(shape, DomainBc{});
    for_each_sample(shape, 0, [&](std::size_t idx, glm::vec3 x) { g.u[idx] = 1.0f + 0.1f * x.y; });
    for_each_sample(shape, 1, [&](std::size_t idx, glm::vec3 x) { g.v[idx] = 0.3f + 0.1f * x.z; });
    apply_solid_velocities(g, f, SolidInterior::extrapolate);
    for (uint32_t k = 0; k < 8; ++k) {
        for (uint32_t j = 0; j < 8; ++j) {
            // u: x = 8 is open (theta 0.5), x = 9 the first layer, x = 10 the second.
            EXPECT_EQ(g.u[u_index(shape, 9, j, k)], g.u[u_index(shape, 8, j, k)]) << j << "," << k;
            EXPECT_EQ(g.u[u_index(shape, 10, j, k)], g.u[u_index(shape, 9, j, k)]);
            EXPECT_EQ(g.u[u_index(shape, 11, j, k)], 0.0f) << "deeper: the rigid (still) wall";
        }
        for (uint32_t j = 0; j <= 8; ++j) {
            // v: x = 7.5 is open, x = 8.5 the first layer, x = 9.5 the second.
            EXPECT_EQ(g.v[v_index(shape, 8, j, k)], g.v[v_index(shape, 7, j, k)]);
            EXPECT_EQ(g.v[v_index(shape, 9, j, k)], g.v[v_index(shape, 8, j, k)]);
            EXPECT_EQ(g.v[v_index(shape, 10, j, k)], 0.0f);
        }
    }
}
```

Append to `tests/test_airflow_solids_step.cpp`, before `}  // namespace`:

```cpp
// The free-slip interior is the inviscid lean mode: with nu > 0 it is refused.
TEST(AirflowSolidsStep, TheFreeSlipInteriorIsRefusedWithViscosity) {
    const GridShape shape{8, 8, 8, 0.1f};
    const MacGrid g = *make_grid(shape, DomainBc{});
    const FluidConfig cfg{.rho = 1.225f, .nu = 1.0e-5f, .h = 0.001f, .n_v = 2, .interior = SolidInterior::extrapolate};
    const auto s = make_fluid_scratch(g, DomainBc{}, cfg);
    ASSERT_FALSE(s.has_value());
    EXPECT_NE(s.error().context.find("free-slip"), std::string::npos) << s.error().context;
}

// Design test 1, its P2 checks (S1's configuration, Test/Docs): a sphere
// R = 6 dx in a uniform stream, free slip, inviscid, a box of side 8R with the
// sphere centred on a cell corner, inflow on -x and the other faces open,
// U h / dx = 0.1, n_v = 16. After step 1 (from a cold start):
//   - every closed face carries the wall velocity, 0, exactly;
//   - the solve's final relative residual is at most 1e-4 (Test/Docs: an
//     unconverged solve must show as a miss, not bias C_M; printed);
//   - the aperture divergence left is at most 2e-4 of the right side's, the
//     residual's own measure carried through the velocity update.
// C_p at step 3 and the added mass at step 1 read P3 and join it.
TEST(AirflowSolidsStep, S1TheSpheresStepOneConvergesAndHoldsItsWall) {
    const float dx = 0.01f;
    const GridShape shape{48, 48, 48, dx};
    DomainBc bc;
    bc.ambient = glm::vec3(1.0f, 0.0f, 0.0f);
    const float U = 1.0f;
    const FluidConfig cfg{.rho = 1.225f, .nu = 0.0f, .h = 0.1f * dx / U, .n_v = 16,
                          .interior = SolidInterior::extrapolate};
    MacGrid g = *make_grid(shape, bc);
    std::fill(g.u.begin(), g.u.end(), U);
    FluidScratch s = *make_fluid_scratch(g, bc, cfg);
    Scene scene;
    scene.shapes.push_back(Shape{.name = "sphere", .kind = Kind::sphere, .centre = {0.24f, 0.24f, 0.24f},
                                 .radius = 6.0f * dx, .min_feature = 12.0f * dx});
    ASSERT_TRUE(set_solids(s, g, bc, scene.parts(), scene.sampler()).has_value());
    const FluidHealth health = fluid_step(g, bc, cfg, s);
    ASSERT_EQ(health.fault, FluidFault::none) << describe(health, shape);

    bool walls_hold = true;
    const std::vector<float>* vel[3] = {&g.u, &g.v, &g.w};
    for (uint32_t c = 0; c < 3; ++c) {
        for (std::size_t i = 0; i < vel[c]->size(); ++i) {
            if (s.solids.theta[c][i] == 0.0f) walls_hold = walls_hold && (*vel[c])[i] == 0.0f;
        }
    }
    const float b_coef = cfg.rho * dx * dx / cfg.h;
    float div_max = 0.0f;
    float b_max = 0.0f;
    const auto flux = [&](uint32_t c, std::size_t idx) {
        const float t = s.solids.theta[c][idx];
        return t * (*vel[c])[idx] + (1.0f - t) * s.solids.wall[c][idx];
    };
    for (uint32_t k = 0; k < 48; ++k) {
        for (uint32_t j = 0; j < 48; ++j) {
            for (uint32_t i = 0; i < 48; ++i) {
                const std::size_t c = cell_index(shape, i, j, k);
                if (s.regions.label[c] == kNoRegion) continue;
                const float div = ((flux(0, u_index(shape, i + 1u, j, k)) - flux(0, u_index(shape, i, j, k))) +
                                   (flux(1, v_index(shape, i, j + 1u, k)) - flux(1, v_index(shape, i, j, k))) +
                                   (flux(2, w_index(shape, i, j, k + 1u)) - flux(2, w_index(shape, i, j, k)))) /
                                  dx;
                div_max = std::max(div_max, std::fabs(div));
                b_max = std::max(b_max, std::fabs(s.projection.pcg.b[c]));
            }
        }
    }
    const float div_ratio = div_max * b_coef / b_max;
    std::printf("=== validation record: S1 sphere in a uniform stream, R/dx 6, P2 checks ===\n"
                "config: 48^3 at dx 1 cm, R 0.06 m centred on a cell corner, free slip, inviscid, inflow U 1 m/s on "
                "-x, other faces open, U h/dx 0.1, n_v 16 CG iterations, step 1 from a cold start\n"
                "measure: every closed face carries the wall velocity = %s\n"
                "measure: the solve's final relative residual = %.3e\n"
                "reference: a converged solve\n"
                "tolerance: <= 1e-4 (test-docs/2026-10-13-solids-tolerances.md, S1)\n"
                "verdict: %s\n"
                "measure: the aperture divergence left, over the right side's = %.3e\n"
                "tolerance: <= 2e-4 (twice the residual's band: the update's rounding)\n"
                "verdict: %s\n"
                "%s\n",
                walls_hold ? "yes" : "NO", static_cast<double>(health.pressure_residual),
                health.pressure_residual <= 1.0e-4f ? "within" : "OUTSIDE", static_cast<double>(div_ratio),
                div_ratio <= 2.0e-4f ? "within" : "OUTSIDE", s.solids.run_line.c_str());
    EXPECT_TRUE(walls_hold);
    EXPECT_LE(health.pressure_residual, 1.0e-4f);
    EXPECT_LE(div_ratio, 2.0e-4f);
}
```

- [ ] **Step 2: Run the tests to see them fail**

Expected:
- `TheFreeSlipFillCopiesTheOpenSideIntoTwoLayers` FAILS: the layers keep the rigid 0.
- `TheFreeSlipInteriorIsRefusedWithViscosity` FAILS: the scratch is made.
- `S1TheSpheresStepOneConvergesAndHoldsItsWall` may pass or fail on its numbers. Time it under ctest in debug against the 60 s limit, and record the time.

- [ ] **Step 3: Write the implementation**

`solids.hpp`: add to `SolidField`, after `fill2`:

```cpp
    std::array<std::vector<uint8_t>, 3> layer;  // per sample: 0, or the free-slip layer 1 or 2
```

`make_solid_field()` assigns `f.layer[c].assign(n, 0u)`, and `open_everything()` fills it with 0.

`solids.cpp`: in `build_solids()`, after the safeguard (and its second sample pass), before the run line:

```cpp
    // The free-slip layers (Task 7): closed samples with an open neighbour of
    // their own component, then closed samples with a first-layer neighbour.
    for (uint32_t c = 0; c < 3; ++c) {
        if (range[c].empty) continue;
        const std::array<uint32_t, 3> dims = sample_dims(s, c);
        const std::size_t stride[3] = {1u, dims[0], std::size_t{dims[0]} * dims[1]};
        for (uint32_t layer = 1; layer <= 2; ++layer) {
            for (uint32_t k = range[c].lo[2]; k <= range[c].hi[2]; ++k) {
                for (uint32_t j = range[c].lo[1]; j <= range[c].hi[1]; ++j) {
                    for (uint32_t i = range[c].lo[0]; i <= range[c].hi[0]; ++i) {
                        const std::size_t idx = i + std::size_t{dims[0]} * (j + std::size_t{dims[1]} * k);
                        if (!closed(f.theta[c][idx]) || f.layer[c][idx] != 0u) continue;
                        const uint32_t q[3] = {i, j, k};
                        bool next_to = false;
                        for (uint32_t a = 0; a < 3 && !next_to; ++a) {
                            for (int d = -1; d <= 1; d += 2) {
                                if ((d < 0 && q[a] == 0u) || (d > 0 && q[a] + 1u == dims[a])) continue;
                                const std::size_t nb = d < 0 ? idx - stride[a] : idx + stride[a];
                                if (layer == 1u ? !closed(f.theta[c][nb]) : f.layer[c][nb] == 1u) {
                                    next_to = true;
                                    break;
                                }
                            }
                        }
                        if (!next_to) continue;
                        (layer == 1u ? f.fill1[c] : f.fill2[c]).push_back(static_cast<uint32_t>(idx));
                    }
                }
            }
            for (const uint32_t idx : (layer == 1u ? f.fill1[c] : f.fill2[c])) f.layer[c][idx] = static_cast<uint8_t>(layer);
        }
    }
```

Each layer is found completely before it is marked. So layer 2 sees only the finished layer 1, and the lists are in array order.

`apply_solid_velocities()`: after the rigid loop, replace `(void)mode;` with:

```cpp
    if (mode != SolidInterior::extrapolate) return;
    for (uint32_t c = 0; c < 3; ++c) {
        const std::array<uint32_t, 3> dims = sample_dims(g.shape, c);
        const std::size_t stride[3] = {1u, dims[0], std::size_t{dims[0]} * dims[1]};
        std::vector<float>& v = *vel[c];
        for (uint32_t layer = 1; layer <= 2; ++layer) {
            for (const uint32_t idx : (layer == 1u ? f.fill1[c] : f.fill2[c])) {
                const uint32_t q[3] = {static_cast<uint32_t>(idx % dims[0]),
                                       static_cast<uint32_t>((idx / dims[0]) % dims[1]),
                                       static_cast<uint32_t>(idx / (std::size_t{dims[0]} * dims[1]))};
                float sum = 0.0f;
                uint32_t count = 0;
                for (uint32_t a = 0; a < 3; ++a) {
                    for (int d = -1; d <= 1; d += 2) {
                        if ((d < 0 && q[a] == 0u) || (d > 0 && q[a] + 1u == dims[a])) continue;
                        const std::size_t nb = d < 0 ? idx - stride[a] : idx + stride[a];
                        const bool source = layer == 1u ? !closed(f.theta[c][nb]) : f.layer[c][nb] == 1u;
                        if (!source) continue;
                        sum += v[nb];
                        ++count;
                    }
                }
                v[idx] = sum / static_cast<float>(count);  // a deliberate per-sample division: count is 1 to 6
            }
        }
    }
```

Each listed sample has at least one source, by construction, so `count >= 1`. Layer 1 reads only open samples, which this pass does not write. Layer 2 reads layer 1's new values. The passes are deterministic Jacobi passes in list order.

`fluid_step.cpp`, in `make_fluid_scratch()` after the `visc_sweeps` check:

```cpp
    if (cfg.interior == SolidInterior::extrapolate && cfg.nu > 0.0f) {
        return refuse("the free-slip interior (SolidInterior::extrapolate) is the inviscid lean mode: nu must be 0");
    }
```

and its comment's refusal list gains that line.

- [ ] **Step 4: Run the tests to see them pass**

Run: `--gtest_filter=AirflowSolids*`
Expected: PASS. S1 prints its record block, which Task 10 pastes into `tests/data/airflow/records/s1-sphere.md`. Time `S1TheSpheresStepOneConvergesAndHoldsItsWall` under ctest in debug. If it is past 30 s, move it behind `SPADE_AIRFLOW_VALIDATION` as V1's long cases are, and say so in the record.

- [ ] **Step 5: The task gate, then commit**

```bash
git add engine/physics/airflow/solids.hpp engine/physics/airflow/solids.cpp engine/physics/airflow/fluid_step.cpp tests/test_airflow_solids.cpp tests/test_airflow_solids_step.cpp
git commit -m "feat(physics): the free-slip interior -- two Jacobi layers from the open side, refused with viscosity; S1's step 1 converges within 1e-4 and its closed faces hold the wall (P2 Task 7)"
```

---
### Task 8: The viscous wall stencil (design tests 2 and 11; S2)

**Files:**
- Modify: `engine/physics/airflow/solids.cpp` (the sheet links in the sample pass; the wall links after the layers)
- Modify: `engine/physics/airflow/viscous.hpp`, `engine/physics/airflow/viscous.cpp` (`diffuse()` takes the solid field)
- Modify: `engine/physics/airflow/fluid_step.cpp` (`diffuse()` gets `&s.solids` on the solids path)
- Modify: `tests/test_airflow_solids.cpp`, `tests/test_airflow_solids_step.cpp`

**Interfaces:**
- Consumes: Task 1's `SolidField::link`, `links` and `WallLinks`; Task 6's step.
- Produces:
  - `void diffuse(MacGrid&, const DomainBc&, float nu, float h, uint32_t sweeps, std::vector<float>& rhs, const SolidField* solids = nullptr) noexcept`
  - `kMinCrossing = 0.01f`

The rule (design §3, as approved), for each velocity component's solve:
- **Fixed samples:** a sample with θ = 0 or φ < 0 holds its wall velocity and is not solved.
- **Wall links:** a solved sample whose neighbour has φ < 0 links to the wall at `f = φ_s / (φ_s − φ_n)`, clamped to at least 0.01. Its term becomes `(α/f)(u_w − u)`, with `u_w = wall_s + f (wall_n − wall_s)`. That is exact for one rigid owner, whose velocity is linear in position.
- **Sheet links:** a sheet sample's link toward the sheet crosses its mid-surface at `d_mid / |g_a|` along the axis. It takes the same term, with the sheet's velocity.

- [ ] **Step 1: Write the failing tests**

Append to `tests/test_airflow_solids.cpp`, before `}  // namespace`:

```cpp
// Design test 11 (the lead's condition): a wall a hair from a velocity sample.
// The box's face lies about 1e-6 dx beyond the u sample at x = 4, so its link
// to the inside neighbour would cross at f ~ 1e-6; the floor holds it at 0.01,
// so the term is 100 alpha, not 1e6 alpha, and the solve stays finite and
// within the values it blends. Mutation: no clamp.
TEST(AirflowSolids, TheCrossingFractionsFloorHoldsTheStencil) {
    const GridShape shape{8, 8, 8, 1.0f};
    Scene scene;
    scene.shapes.push_back(Shape{.name = "slab", .kind = Kind::box, .centre = {12.000001f, 4.0f, 4.0f},
                                 .half = {8.0f, 20.0f, 20.0f}, .omega = {0.0f, 0.0f, 1.0f}, .min_feature = 8.0f});
    const SolidField f = built(shape, scene);
    const std::size_t s = u_index(shape, 4, 3, 3);
    ASSERT_GE(f.phi[0][s], 0.0f);
    ASSERT_LT(f.phi[0][u_index(shape, 5, 3, 3)], 0.0f);
    ASSERT_NE(f.link[0][s], kNoWallLinks);
    const WallLinks& wl = f.links[f.link[0][s]];
    EXPECT_EQ(wl.inv_f[1], 1.0f / 0.01f) << "+x: the floor";
    EXPECT_EQ(wl.inv_f[0], 0.0f) << "-x: no wall";

    MacGrid g = *make_grid(shape, DomainBc{});
    for_each_sample(shape, 0, [&](std::size_t idx, glm::vec3 x) { g.u[idx] = 0.5f + 0.05f * x.z; });
    float lo = 0.0f;
    float hi = 0.0f;
    for (const float v : g.u) {
        lo = std::min(lo, v);
        hi = std::max(hi, v);
    }
    for (const float v : f.wall[0]) {
        lo = std::min(lo, v);
        hi = std::max(hi, v);
    }
    std::vector<float> rhs(g.u.size());
    diffuse(g, DomainBc{}, 1.0f, 1.0f, 8, rhs, &f);
    EXPECT_TRUE(std::isfinite(g.u[s]));
    EXPECT_GE(g.u[s], lo);
    EXPECT_LE(g.u[s], hi);
}

// A sheet links the samples either side of it: the v samples at x = 15.5 and
// 16.5 either side of a sheet on face plane 16 each cross it halfway along
// their link.
TEST(AirflowSolids, ASheetLinksTheSamplesEitherSide) {
    const GridShape shape{32, 8, 8, 1.0f};
    Scene scene;
    scene.shapes.push_back(Shape{.name = "plate", .kind = Kind::box, .centre = {16.0f, 4.0f, 4.0f},
                                 .half = {0.125f, 20.0f, 20.0f}, .min_feature = 0.25f});
    std::vector<SolidPart> parts = scene.parts();
    parts[0].extent = glm::vec3(0.25f, 8.0f, 8.0f);
    SolidField f = *make_solid_field(shape);
    ASSERT_TRUE(build_solids(f, shape, {}, parts, scene.sampler()).has_value());
    const uint32_t left = f.link[1][v_index(shape, 15, 3, 3)];
    const uint32_t right = f.link[1][v_index(shape, 16, 3, 3)];
    ASSERT_NE(left, kNoWallLinks);
    ASSERT_NE(right, kNoWallLinks);
    EXPECT_EQ(f.links[left].inv_f[1], 2.0f) << "+x, at f = 0.5";
    EXPECT_EQ(f.links[right].inv_f[0], 2.0f) << "-x, at f = 0.5";
    EXPECT_EQ(f.link[1][v_index(shape, 13, 3, 3)], kNoWallLinks) << "two cells off: no link";
}
```

Append to `tests/test_airflow_solids_step.cpp`, before `}  // namespace`:

```cpp
[[nodiscard]] bool validation_enabled() {
    const char* v = std::getenv("SPADE_AIRFLOW_VALIDATION");
    return v != nullptr && std::string(v) == "1";
}

struct CouetteResult {
    double error;        // max |u_theta - exact| / (Omega R1)
    bool walls_hold;     // every closed face carries the rigid velocity, bit for bit
    double circulation;  // the mean u_theta over the measured cells, m/s
};

// S2 (Test/Docs): circular Couette flow. The cylinder R1 turns at Omega about
// y inside a fixed tube R2 = 2 R1, in a one-cell slab periodic in y, both
// no-slip; Re = Omega R1 (R2 - R1) / nu = 10, Omega R1 h / dx = 0.5, run to
// T = 2 (R2 - R1)^2 / nu. Exact: u_theta = A r + B / r, A = -Omega / 3,
// B = 4 Omega R1^2 / 3 (Landau & Lifshitz, Fluid Mechanics, section 18).
[[nodiscard]] CouetteResult couette(uint32_t r1_cells, uint32_t n_v) {
    const float dx = 1.0e-3f;
    const float r1 = static_cast<float>(r1_cells) * dx;
    const float r2 = 2.0f * r1;
    const uint32_t n = 2u * (2u * r1_cells) + 4u;
    const GridShape shape{n, 1, n, dx};
    DomainBc bc;
    bc.face.fill(FaceBc::wall);
    bc.face[kYMinus] = FaceBc::periodic;
    bc.face[kYPlus] = FaceBc::periodic;
    const float omega = 0.1f / r1;  // Omega R1 = 0.1 m/s
    const float nu = 0.1f * (r2 - r1) / 10.0f;
    const float h = 0.5f * dx / 0.1f;
    const FluidConfig cfg{.rho = 1.0f, .nu = nu, .h = h, .n_v = n_v, .visc_sweeps = 8};
    MacGrid g = *make_grid(shape, bc);
    FluidScratch s = *make_fluid_scratch(g, bc, cfg);
    const glm::vec3 centre(0.5f * static_cast<float>(n) * dx, 0.5f * dx, 0.5f * static_cast<float>(n) * dx);
    Scene scene;
    scene.shapes.push_back(Shape{.name = "cylinder", .kind = Kind::cylinder_y, .centre = centre, .radius = r1,
                                 .omega = {0.0f, omega, 0.0f}, .min_feature = 2.0f * r1});
    scene.shapes.push_back(Shape{.name = "tube", .kind = Kind::outside_cylinder_y, .centre = centre, .radius = r2,
                                 .min_feature = 4.0f * dx});
    EXPECT_TRUE(set_solids(s, g, bc, scene.parts(), scene.sampler()).has_value());
    const auto steps = static_cast<uint32_t>(2.0f * (r2 - r1) * (r2 - r1) / nu / h + 0.5f);
    for (uint32_t k = 0; k < steps; ++k) {
        const FluidHealth health = fluid_step(g, bc, cfg, s);
        EXPECT_EQ(health.fault, FluidFault::none) << describe(health, shape);
        if (health.fault != FluidFault::none) break;
    }
    CouetteResult out{0.0, true, 0.0};
    const std::vector<float>* vel[3] = {&g.u, &g.v, &g.w};
    for (uint32_t c = 0; c < 3; ++c) {
        for (std::size_t i = 0; i < vel[c]->size(); ++i) {
            if (s.solids.theta[c][i] == 0.0f) out.walls_hold = out.walls_hold && (*vel[c])[i] == s.solids.wall[c][i];
        }
    }
    const double A = -static_cast<double>(omega) / 3.0;
    const double B = 4.0 * static_cast<double>(omega) * r1 * r1 / 3.0;
    double sum = 0.0;
    uint32_t count = 0;
    for (uint32_t k = 0; k < n; ++k) {
        for (uint32_t i = 0; i < n; ++i) {
            const double rx = (static_cast<double>(i) + 0.5) * dx - centre.x;
            const double rz = (static_cast<double>(k) + 0.5) * dx - centre.z;
            const double r = std::sqrt(rx * rx + rz * rz);
            if (r < r1 + dx || r > r2 - dx) continue;
            const double uc = 0.5 * (static_cast<double>(g.u[u_index(shape, i, 0, k)]) + g.u[u_index(shape, i + 1u, 0, k)]);
            const double wc = 0.5 * (static_cast<double>(g.w[w_index(shape, i, 0, k)]) + g.w[w_index(shape, i, 0, k + 1u)]);
            const double ut = (uc * rz - wc * rx) / r;
            out.error = std::max(out.error, std::fabs(ut - (A * r + B / r)) / 0.1);
            sum += ut;
            ++count;
        }
    }
    out.circulation = sum / count;
    return out;
}

// Design test 2 / S2 at R1 = 16 dx (the gate case; R1 = 32 dx and the order
// check are opt-in). Tolerance <= 0.11 (Test/Docs). The exact checks: closed
// faces carry the rigid velocity bit for bit, and the air turns with the
// cylinder. Mutation: the rigid velocity at closed faces only (the links'
// terms ignored), which Test/Docs expects to miss by up to a factor of two.
TEST(AirflowSolidsStep, S2CircularCouetteFlow) {
    const CouetteResult r = couette(16, 4);
    std::printf("=== validation record: S2 circular Couette flow, R1/dx 16 ===\n"
                "config: a 68x1x68 slab (y periodic), dx 1 mm, R1 16 mm turning at Omega R1 = 0.1 m/s inside a "
                "fixed tube R2 = 2 R1, both no-slip, Re 10, Omega R1 h/dx 0.5, T = 2 (R2 - R1)^2 / nu, n_v 4, "
                "visc_sweeps 8\n"
                "measure: max |u_theta - u_theta,exact| / (Omega R1) over R1 + dx <= r <= R2 - dx = %.4f\n"
                "reference: A r + B / r (Landau & Lifshitz, Fluid Mechanics, section 18)\n"
                "tolerance: <= 0.11 (test-docs/2026-10-13-solids-tolerances.md, fixed before the first run)\n"
                "verdict: %s\n"
                "measure: closed faces carry the rigid velocity = %s; mean u_theta = %.5f m/s (the rotation's sense: "
                "positive)\n",
                r.error, r.error <= 0.11 ? "within" : "OUTSIDE", r.walls_hold ? "yes" : "NO", r.circulation);
    EXPECT_LE(r.error, 0.11);
    EXPECT_TRUE(r.walls_hold);
    EXPECT_GT(r.circulation, 0.0);
}

TEST(AirflowSolidsStep, S2CircularCouetteFlowAt32CellsAndItsOrder) {
    if (!validation_enabled()) GTEST_SKIP() << "set SPADE_AIRFLOW_VALIDATION=1 to run S2 at R1/dx 32 and its order";
    const CouetteResult coarse = couette(16, 4);
    const CouetteResult fine = couette(32, 4);
    std::printf("=== validation record: S2 circular Couette flow, R1/dx 32, and the order check ===\n"
                "measure: max |u_theta - u_theta,exact| / (Omega R1) at R1/dx 32 = %.4f\n"
                "tolerance: <= 0.054 (test-docs/2026-10-13-solids-tolerances.md, fixed before the first run)\n"
                "verdict: %s\n"
                "measure: e(32) / e(16) = %.3f\n"
                "tolerance: <= 0.7 (an observed order of at least 0.5; reported only if e(16) < 0.011)\n"
                "verdict: %s\n",
                fine.error, fine.error <= 0.054 ? "within" : "OUTSIDE", fine.error / coarse.error,
                coarse.error < 0.011 ? "report only" : (fine.error <= 0.7 * coarse.error ? "within" : "OUTSIDE"));
    EXPECT_LE(fine.error, 0.054);
    if (coarse.error >= 0.011) EXPECT_LE(fine.error, 0.7 * coarse.error);
}
```

Add `#include <cstdlib>` to the file's includes.

- [ ] **Step 2: Run the tests to see them fail**

Expected: a build failure, since `diffuse()` takes no solid field. Once the declaration exists with an empty body for the new path, the link and Couette tests FAIL: no links are built.

- [ ] **Step 3: Write the implementation**

`solids.hpp`, add:

```cpp
// The viscous wall links' floor on the crossing fraction (design test 11): a
// wall a hair from a sample would make 1 / f, and the stencil's diagonal,
// unbounded.
inline constexpr float kMinCrossing = 0.01f;
```

`solids.cpp`:
- In `sample_pass()`, reset the links first: `f.links.clear();`, and `f.link[c][idx] = kNoWallLinks` for every sample in range alongside θ, wall and φ.
- In the sheet branch, after θ, add the sheet's links:

```cpp
                        // The sheet's links (Task 8): along each axis the
                        // mid-surface lies against the gradient, d_mid / |g_a| away.
                        for (uint32_t a = 0; a < 3; ++a) {
                            const float g_a = q.gradient[static_cast<glm::length_t>(a)];
                            if (g_a == 0.0f) continue;
                            const float dist = d_mid / std::fabs(g_a);
                            if (dist > s.dx) continue;
                            const uint32_t dir = g_a > 0.0f ? 0u : 1u;  // the sheet's side: -a when the gradient points +a
                            const uint32_t qa = a == 0 ? i : (a == 1 ? j : k);
                            if (dir == 0u ? qa == 0u : qa + 1u == dims[a]) continue;  // no neighbour there
                            const float frac = std::max(dist / s.dx, kMinCrossing);
                            if (f.link[c][idx] == kNoWallLinks) {
                                f.link[c][idx] = static_cast<uint32_t>(f.links.size());
                                f.links.push_back(WallLinks{});
                            }
                            WallLinks& wl = f.links[f.link[c][idx]];
                            wl.inv_f[2u * a + dir] = 1.0f / frac;
                            wl.wall[2u * a + dir] = q.velocity[static_cast<glm::length_t>(c)];
                        }
```

- In `build_solids()`, after the free-slip layers, add the wall links:

```cpp
    // The wall links (Task 8): a solved sample (theta > 0, phi >= 0) next to
    // a sample inside a wall (phi < 0) crosses the wall at f = phi_s / (phi_s
    // - phi_n) of the link (Gibou, Fedkiw, Cheng and Kang 2002), floored at
    // kMinCrossing; the wall's velocity there blends the two samples' rigid
    // velocities, exact for one rigid owner. A sheet link already nearer wins.
    for (uint32_t c = 0; c < 3; ++c) {
        if (range[c].empty) continue;
        const std::array<uint32_t, 3> dims = sample_dims(s, c);
        const std::size_t stride[3] = {1u, dims[0], std::size_t{dims[0]} * dims[1]};
        for (uint32_t k = range[c].lo[2]; k <= range[c].hi[2]; ++k) {
            for (uint32_t j = range[c].lo[1]; j <= range[c].hi[1]; ++j) {
                for (uint32_t i = range[c].lo[0]; i <= range[c].hi[0]; ++i) {
                    const std::size_t idx = i + std::size_t{dims[0]} * (j + std::size_t{dims[1]} * k);
                    const float phi_s = f.phi[c][idx];
                    if (closed(f.theta[c][idx]) || phi_s < 0.0f) continue;  // fixed: not solved
                    const uint32_t q[3] = {i, j, k};
                    for (uint32_t a = 0; a < 3; ++a) {
                        for (uint32_t dir = 0; dir < 2; ++dir) {
                            if (dir == 0u ? q[a] == 0u : q[a] + 1u == dims[a]) continue;
                            const std::size_t nb = dir == 0u ? idx - stride[a] : idx + stride[a];
                            const float phi_n = f.phi[c][nb];
                            if (!(phi_n < 0.0f)) continue;
                            const float frac = std::max(phi_s / (phi_s - phi_n), kMinCrossing);
                            const float inv = 1.0f / frac;
                            if (f.link[c][idx] == kNoWallLinks) {
                                f.link[c][idx] = static_cast<uint32_t>(f.links.size());
                                f.links.push_back(WallLinks{});
                            }
                            WallLinks& wl = f.links[f.link[c][idx]];
                            if (wl.inv_f[2u * a + dir] >= inv) continue;
                            wl.inv_f[2u * a + dir] = inv;
                            wl.wall[2u * a + dir] = f.wall[c][idx] + frac * (f.wall[c][nb] - f.wall[c][idx]);
                        }
                    }
                }
            }
        }
    }
```

A wall winner's φ outside the ramp is its true distance, and a sheet's is its non-negative mid-surface distance. So `phi_n < 0` marks only samples inside walls.

`viscous.hpp`: the declaration becomes:

```cpp
// WITH SOLIDS (the solids design, section 3): every sample with theta = 0 or
// phi < 0 holds its wall velocity and is not solved; a solved sample's link
// that crosses a wall or a sheet at the fraction f of its length takes
// (alpha / f) (u_wall - x) in place of alpha (x_nb - x) (solids.hpp's links;
// the symmetric ghost-fluid Dirichlet condition of Gibou, Fedkiw, Cheng and
// Kang 2002, on phi = 0, where the apertures put theta = 1/2). Without:
// plan A, bit for bit.
void diffuse(MacGrid& g, const DomainBc& bc, float nu, float h, uint32_t sweeps, std::vector<float>& rhs,
             const SolidField* solids = nullptr) noexcept;
```

It includes `physics/airflow/solids.hpp`.

`viscous.cpp`: `diffuse_component()` takes `const SolidField* solids` and passes it from `diffuse()`. At its top, after the periodic flags:

```cpp
    const bool walls = solids != nullptr && solids->any;
    const std::vector<float>* theta = walls ? &solids->theta[c] : nullptr;
    const std::vector<float>* phi = walls ? &solids->phi[c] : nullptr;
    const std::vector<float>* wall_v = walls ? &solids->wall[c] : nullptr;
    const std::vector<uint32_t>* link = walls ? &solids->link[c] : nullptr;
    const auto fixed = [&](std::size_t idx) { return closed((*theta)[idx]) || (*phi)[idx] < 0.0f; };
    if (walls) {
        for (std::size_t idx = 0; idx < x.size(); ++idx) {
            if (fixed(idx)) x[idx] = (*wall_v)[idx];
        }
    }
```

This goes before `std::copy(x.begin(), x.end(), rhs.begin());`.

In the sweep, at the top of the per-sample body:

```cpp
                        const std::size_t here = at(i, j, k);
                        const WallLinks* wl = nullptr;
                        if (walls) {
                            if (fixed(here)) continue;
                            if ((*link)[here] != kNoWallLinks) wl = &solids->links[(*link)[here]];
                        }
```

The fast path's condition becomes `if (wl == nullptr && row_inside && in_range(0, i))`. In the general body's per-direction loop, first:

```cpp
                                if (wl != nullptr && wl->inv_f[2u * a + dir] > 0.0f) {
                                    const float w = alpha * wl->inv_f[2u * a + dir];
                                    diag += w;
                                    extra += w * wl->wall[2u * a + dir];
                                    continue;
                                }
```

This runs before the `a == c` branch. Without solids, `walls` is false and `wl` is null, so every operation and its order is plan A's.

`fluid_step.cpp`: `diffuse(g, bc, cfg.nu, cfg.h, cfg.visc_sweeps, s.rhs, solid ? &s.solids : nullptr);`

- [ ] **Step 4: Run the tests to see them pass**

Run: `--gtest_filter=AirflowSolids*:AirflowStep.*:AirflowConfigs.*`
Expected: PASS, with both goldens unchanged. S2 prints its record. Time `S2CircularCouetteFlow` under ctest in debug. If it is past 30 s, run it opt-in and keep a 400-step version in the gate.

- [ ] **Step 5: The mutations, by hand**

1. In `build_solids()`'s wall links, drop the floor (`frac = phi_s / (phi_s - phi_n)`). `TheCrossingFractionsFloorHoldsTheStencil` must FAIL on `inv_f[1]`. Restore.
2. In `diffuse_component()`, skip the link override (`wl = nullptr` always). This is the rigid velocity at closed faces only. Run `S2CircularCouetteFlow` and record the error it measures.
   - If it exceeds 0.11, the check caught it.
   - If it does not, S2 cannot tell the two treatments apart at R1/dx 16. Say so in the ledger and to Test/Docs, and add a discriminating check before the task closes: planar Couette between a still and a moving plate, the plates off the faces by a quarter cell, where the closed-faces-only wall sits up to a cell from the true one and the linear profile is exact.

Restore. Record both results.

- [ ] **Step 6: The task gate, then commit**

```bash
git add engine/physics/airflow/solids.hpp engine/physics/airflow/solids.cpp engine/physics/airflow/viscous.hpp engine/physics/airflow/viscous.cpp engine/physics/airflow/fluid_step.cpp tests/test_airflow_solids.cpp tests/test_airflow_solids_step.cpp
git commit -m "feat(physics): no-slip at immersed walls and sheets in the viscous solve -- symmetric ghost-fluid links on phi = 0, the crossing floored at 0.01; S2 circular Couette within 0.11 (P2 Task 8)"
```

---
### Task 9: The posed drone through `vehicles::sample_parts()` (design test 7) and the iteration count (the lead's condition)

**Files:**
- Create: `tests/airflow_vehicle_sampler.hpp` (tests only: it adapts `vehicles::sample_parts()` to `SolidSampler`, and computes a part's bounds until Task 10)
- Modify: `tests/test_airflow_solids_step.cpp`
- Modify: `docs/design/physics/plans/2026-10-13-airflow-solids-design.md` (§3: the count and its measurements; a docs commit on master)

**Interfaces:**
- Consumes:
  - `vehicles::posed_parts()`, `vehicles::sample_parts()`, `PosedParts`, `ModelType`, `PartDesc`;
  - `world/mesh.hpp`'s `make_mesh()`;
  - `compile_airframe()` (A4).
- Produces, in the tests: `struct VehicleSampler { … SolidSampler sampler() const; std::vector<SolidPart> parts() const; }`, built from a model, a body state and the grid's world origin.

- [ ] **Step 1: The adapter**

Create `tests/airflow_vehicle_sampler.hpp`:

```cpp
#pragma once
// vehicles::sample_parts() as the airflow core's SolidSampler (the solids
// design, section 3, the layering): the tests' adapter, and the shape of the
// D1 demo's and plan C's. The grid frame is the world shifted by `origin`.
// Bounds: until Core's part_bounds() lands, each part's part-frame box comes
// from `local_half` given by the caller (tests only, TD-9; Task 10 switches).
#include <cstdint>
#include <string>
#include <vector>

#include <glm/glm.hpp>

#include "physics/airflow/solids.hpp"
#include "vehicles/model_type.hpp"
#include "vehicles/part_pose.hpp"

namespace airflow_test {

struct VehicleSampler {
    const spade::vehicles::ModelType* model = nullptr;
    spade::vehicles::PosedParts posed;
    glm::vec3 origin{0.0f};                            // the grid's minimum corner in the world, m
    std::vector<spade::vehicles::PosedParts> single;   // one per posed part, for the per-part query
    std::vector<uint32_t> solid_of;                    // ModelType part index -> posed (SolidPart) index

    VehicleSampler(const spade::vehicles::ModelType& m, spade::vehicles::PosedParts p, glm::vec3 o)
        : model(&m), posed(std::move(p)), origin(o) {
        solid_of.assign(m.parts.size(), spade::physics::airflow::kNoSolidPart);
        for (uint32_t i = 0; i < posed.parts.size(); ++i) {
            solid_of[posed.parts[i].part] = i;
            spade::vehicles::PosedParts one = posed;
            one.parts = {posed.parts[i]};
            single.push_back(std::move(one));
        }
    }

    static spade::physics::airflow::SolidQuery to_query(const VehicleSampler& v, const spade::vehicles::PartSample& s,
                                                         uint32_t index) noexcept {
        if (s.owner == spade::vehicles::kNoPart) return {};
        return {s.distance, s.gradient, index, s.velocity};
    }

    static spade::physics::airflow::SolidQuery nearest(const void* ctx, glm::vec3 x) noexcept {
        const auto& v = *static_cast<const VehicleSampler*>(ctx);
        const spade::vehicles::PartSample s = spade::vehicles::sample_parts(*v.model, v.posed, v.origin + x);
        return to_query(v, s, s.owner == spade::vehicles::kNoPart ? 0u : v.solid_of[s.owner]);
    }

    static spade::physics::airflow::SolidQuery part(const void* ctx, uint32_t p, glm::vec3 x) noexcept {
        const auto& v = *static_cast<const VehicleSampler*>(ctx);
        const spade::vehicles::PartSample s = spade::vehicles::sample_parts(*v.model, v.single[p], v.origin + x);
        return to_query(v, s, p);
    }

    [[nodiscard]] spade::physics::airflow::SolidSampler sampler() const {
        return {this, &VehicleSampler::nearest, &VehicleSampler::part};
    }

    // The posed parts as the grid sees them, from each part's part-frame half
    // extents (tests only until Core's part_bounds(): Task 10).
    [[nodiscard]] std::vector<spade::physics::airflow::SolidPart> parts(const std::vector<glm::vec3>& local_half) const {
        std::vector<spade::physics::airflow::SolidPart> out;
        for (uint32_t i = 0; i < posed.parts.size(); ++i) {
            const spade::vehicles::PosedPart& pp = posed.parts[i];
            const spade::vehicles::PartDesc& d = model->parts[pp.part];
            spade::physics::airflow::SolidPart sp;
            sp.name = d.name;
            sp.porous = d.surface.kind == spade::vehicles::PartSurface::Kind::porous;
            sp.min_feature = d.min_feature;
            const glm::vec3 h = local_half[pp.part];
            sp.extent = 2.0f * h;
            glm::vec3 lo(1e30f), hi(-1e30f);
            for (int c = 0; c < 8; ++c) {
                const glm::vec3 corner((c & 1) ? h.x : -h.x, (c & 2) ? h.y : -h.y, (c & 4) ? h.z : -h.z);
                const glm::vec3 x = pp.origin + pp.orient * corner - origin;
                lo = glm::min(lo, x);
                hi = glm::max(hi, x);
            }
            sp.box_min = lo;
            sp.box_max = hi;
            out.push_back(sp);
        }
        return out;
    }
};

}  // namespace airflow_test
```

- [ ] **Step 2: Write the failing test (design test 7)**

Append to `tests/test_airflow_solids_step.cpp` (with `#include "airflow_vehicle_sampler.hpp"`, `"vehicles/airframe_compile.hpp"` and `"world/mesh.hpp"`).
- **The model:** the compiled reference quad's body box, its first part (A4's analytic solid from the airframe compile), and the same box as a mesh part at the same mount (B4).
- **The pose and the grid:** both posed by one tilted, turning body at the D1 nest's dx = 6 mm, in a 48³ grid centred on it.
- **The check:** their θ, wall and φ must agree within the distance's fp32 rounding. That is an implementation comparison (TD-2), so its band is measured on the first run and pinned, with the measurement in the comment.
- **Helpers:** the mesh comes from `test_part_pose.cpp`'s `box_mesh()` pattern. Copy its twelve-triangle cube, here in a local helper, since that file's helpers are in its own anonymous namespace.

```cpp
TEST(AirflowSolidsStep, ThePosedBodyGivesTheSameAperturesFromA4AndB4) {
    // ... compile_airframe(reference_quad spec trimmed to its body box) for A4's part;
    //     a mesh part of the same half extents (0.06, 0.015, 0.06) at the same mount for B4;
    //     posed_parts() of each with body_at(pos, tilt, vel, omega_body) and PartFilter::rigid_only;
    //     VehicleSampler for each, origin = pos - 24 dx;
    //     build_solids() of each into a 48^3 field at dx 0.006;
    //     max |theta_A - theta_B|, |wall_A - wall_B|, |phi_A - phi_B| printed, then pinned:
    //     EXPECT_LE(theta_diff, <measured, rounded up to one figure>) // TD-2
}
```

When writing it, take the reference quad's body box record verbatim from `tests/test_airframe_compile.cpp`: `make_part(PartShape::box, 0.12, dvec3(0), dvec3(0.12, 0.03, 0.12))`. Compile a spec holding the reference quad's full drive train, since `check_airframe()` needs it, and take `model.parts[0]`. The B4 model is the same `ModelType` with `parts[0]` replaced by a mesh part of half extents (0.06, 0.015, 0.06) at `parts[0]`'s mount. Pose with `body_at(glm::vec3(0.0f), glm::angleAxis(0.4f, glm::normalize(glm::vec3(0.3f, 1.0f, -0.2f))), glm::vec3(0.0f), glm::vec3(0.0f, 2.0f, 0.0f))`, so that wall velocities are nonzero.

- [ ] **Step 3: Run it, measure, pin, commit**

Run it, read its printed differences, pin each bound at the measured value rounded up to one significant figure, and run it again to see it pass. Then run the gate.

```bash
git add tests/airflow_vehicle_sampler.hpp tests/test_airflow_solids_step.cpp
git commit -m "test(physics): the posed body's apertures agree from A4's analytic box and B4's mesh, within the distance's rounding (P2 Task 9; design test 7, TD-2)"
```

- [ ] **Step 4: The iteration count (the lead's condition), measured and recorded**

With a throwaway driver (not committed), print the relative residual after n = 2, 4, 8, 12, 16 CG iterations for:
- S1's step 1 (cold) and its step 3 (warm);
- S3 on both parities (cold);
- a dense-parts case: every rigid part of the compiled reference quad, through `VehicleSampler`, at dx = 6 mm, in still air with open faces. Use the part bounds from the shapes' declared extents until Task 10. Take step 1 (cold) and step 10 (warm), with a fixed uniform downwash of 5 m/s through the box as the right side's source.

Choose the default as the smallest n whose warm residual is ≤ 10⁻⁴ on all three and whose cold residual is ≤ 10⁻³. Set the announcement threshold at ten times the warm residual it leaves, no lower than 10⁻⁴. Record the table, the choice and the threshold in the design's §3, replacing "first 10⁻³". The D1 demo's configuration uses that n_v. Commit the design on master as docs.

---

### Task 10: Core's `part_bounds()`, the records, the documents and the final gate

Runs when Core's `part_bounds()` is on master (the lead: P2 merges after it, calling it).

**Files:**
- Modify: `tests/airflow_vehicle_sampler.hpp` (`parts()` reads `vehicles::part_bounds()` and drops `local_half`)
- Modify: `tests/test_airflow_solids_step.cpp` (Task 9's callers)
- Create: `tests/data/airflow/records/s1-sphere.md`, `s2-couette.md`, `s3-sheet.md`
- Modify: `docs/design/physics/plans/2026-10-13-airflow-solids-design.md` (status: P2 built), `docs/design/test-docs/04-verification.md`'s airflow rows (with Test/Docs), `tasks/todo-physics.md`

- [ ] **Step 1: Switch the adapter to `part_bounds()`**

Rebase the branch on master. In `VehicleSampler::parts()`, replace the `local_half` argument with Core's call:
- `sp.extent` = the box's size;
- the eight corners come from its min and max;
- a refused bound (a part `part_bounds()` refuses) is passed on by name as a test failure.

Task 9's test drops its `local_half` table. Run `AirflowSolidsStep.*`, expecting PASS with the pinned bands unchanged. Any band that moves gets its cause named.

- [ ] **Step 2: The records**

Paste each printed block (S1, S2 at 16 and, opt-in, at 32 with the order check, S3 on both parities) into its record file. Use V1's form: an opening `# S<n> … -- validation records` line and paragraph, then each block in a `text` fence with `commit:` and `date:` lines after its opening line. Send them to Test/Docs for review, as V1's were.

- [ ] **Step 3: The documents**

- The design's status line: P2 built, with the branch and head.
- §3: the iteration count from Task 9.
- `00-decisions.md`'s `PHY-11` gains one line on P2's mutations, with a pointer to the ledger.
- Tell Interface that `classify_part()` exists, with its signature, for their `resolve()` (TD-9).
- Tell Rendering that P2's closed faces and regions are in for R2.5. `PublishInput`'s closed-face flag comes with P3's test 8.

- [ ] **Step 4: The final gate**

The task gate in full, release and debug. Then:
- `gcc-check.sh --compile` on every changed `.cpp`, and plainly on every header;
- the BitPortability and M1B scans;
- a Docker leg at the head that passes only on `summary.txt`'s PASS line. No golden moved (design test 6), so it need not be `-NoSeed`.

Then report READY FOR REVIEW to the lead, with:
- the commits;
- the mutations' ledger;
- the records;
- the iteration-count table.

---

## Self-review (done when the plan was written)

1. **The design against the tasks:**
   - §2, sampler and pose: Tasks 1 and 9.
   - §3:
     - apertures: Task 1;
     - the projection: Task 6;
     - the solver with solids: Tasks 3 and 4;
     - regions: Task 5;
     - the viscous wall: Task 8;
     - inside a solid: Tasks 6 and 7;
     - the layering: Tasks 1 and 9.
   - §4: classifier and sheets in Task 1, the safeguard in Task 2.
   - §8:
     - tests 1 (P2 part) in Task 7, 2 in Task 8, 3 in Tasks 4 and 6, 5 in Tasks 1, 2 and 5;
     - test 6 in Task 6, 7 in Task 9, 9 in Task 5, 10 in Task 4, 11 in Task 8;
     - tests 4 (porous), 8 (Publish) and P3's halves of 1 and 3 are out of scope, as the header says.
   - The lead's conditions:
     - the count: Task 9;
     - the residual on the run line and announced: Task 6;
     - symmetry: Task 4;
     - the GPU band: plan B's, named in spec §1.5;
     - f ≥ 0.01: Task 8;
     - the hub-and-fin boundary and the wedge: Task 2.
2. **Placeholders:** Task 9's test body is described rather than written, because it copies `test_part_pose.cpp`'s and `test_airframe_compile.cpp`'s helpers verbatim (named there) and pins a band measured on its first run (TD-2). Every other step carries its code.
3. **Types:**
   - `SolidQuery`, `SolidSampler`, `SolidPart`, `SolidClass`, `SolidField`, `WallLinks` and `SolidRange` are fixed in Task 1, and `layer` is added in Task 7;
   - `PcgGauge` and `PcgScratch` in Task 4;
   - `FluidRegions` and `RegionFault` in Task 5;
   - `ProjectionReport` in Task 6.
   Later tasks use exactly these names.
4. **Review focus:** each of the five has its test in the task named.
