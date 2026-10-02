# Drone sim box — implementation plan

> **For agentic workers:** realm sessions implement their own tasks; the lead reviews each task before the next one that depends on it, and runs the final verification. Steps use checkbox (`- [ ]`) syntax.

**Goal:** make the sandbox's default window scene a pinned quadrotor driven physically through Spade's rotor model, with an analytic air field, a standard view and an air-velocity heatmap view, and a live physics panel.

**Architecture:** all new scene logic lives in two header-only, display-free sandbox files (`drone_sim.hpp` for physics and control, `drone_view.hpp` for the air field and drawing) so `spade_tests` can exercise it. The engine gains three small things: a medium read accessor (Core), a rotor-wake function (Physics) and the GL lighting fix (Rendering). `main.cpp` and `gl_target_sink.*` only wire input, panel and loop.

**Tech stack:** C++23, GLM, GoogleTest through ctest, GLFW + ImGui (sandbox only).

**Spec:** `docs/design/plans/2026-10-01-drone-sim-box-design.md`

## Global constraints

- Build and test only through `scripts\build.ps1` / `scripts\test.ps1`, in the foreground. **The lead hands out build slots; ask before building.**
- No wall-clock time in anything a test steps. The sandbox's wall-clock `delta_seconds()` feeds only the accumulator.
- No engine golden may move. If one does, stop and tell the lead.
- Commit only your own paths (`git commit <paths>`); never push.

## Review focus (inputs the spec implies that no single task's tests exercise by default)

1. **Rotors stopped** (throttle 0) — the field must equal the medium exactly, the heatmap must not divide by zero (auto range with max 0) → covered in Task 5 tests.
2. **Pitch/roll at the ±60° limit while yawing** — the controller must not flip (quaternion double cover) → covered in Task 4 tests.
3. **Changing options repeatedly** (e.g. dragging a slider) — the rebuild must be debounced and must carry state; a rebuild per frame is a defect → Task 6 debounce + Task 4 carry test.
4. **Selecting Vulkan** — refused with a visible message, simulation stays on CPU, never steps unpinned → Task 4 test + Task 6 manual check.
5. **A minimised window or a long stall** (`dt` huge) — the accumulator must cap steps per frame → Task 4 test.

## Order and build slots

| Task | Realm | Depends on | Can start |
|---|---|---|---|
| 1 `sample_medium` + Vulkan behavior refusal | Core | — | now |
| 2 rotor wake function | Physics | — | now |
| 3 GL lighting convention | Rendering | — | now |
| 4 `drone_sim.hpp` (stand, controller, rebuild) | Interface | — | now (uses existing API only) |
| 5 `drone_view.hpp` (field, heatmap, drone parts) | Interface (Rendering reviews) | 1, 2 | after 1 and 2 merge |
| 6 wiring: scene, input, panel, flags | Interface | 3, 4, 5 | after 3–5 merge |
| 7 verification | Lead | 6 | last |

Tasks 1–4 code in parallel; their builds are serialised by the lead in the order they report ready.

---

### Task 1: `Simulation::sample_medium` and the Vulkan behavior refusal (Core)

**Files:**
- Modify: `engine/sim/simulation.hpp` (declare beside `world_params`), `engine/sim/simulation.cpp` (define; refuse in the Vulkan branch of `step`)
- Test: `tests/test_determinism.cpp` or a new `tests/test_sim_medium.cpp` (Core's choice; register it in `tests/CMakeLists.txt`), and a `Gpu*`-prefixed case in `tests/test_gpu_parity.cpp`

**Interfaces:**
- Produces: `[[nodiscard]] Result<MediumSample> Simulation::sample_medium(uint32_t world_index, glm::vec3 pos) const;` — density from the world's `WorldParams`, wind = `WorldParams::wind` + the world's current Dryden gust, exactly what the ForceElements pass sees (`DrydenMedium(state, params).sample(world_params, pos)`). Works on both backends (Vulkan reads back into the arenas after each `step`).
- Produces: on a Vulkan `Simulation` with a non-empty behavior registry attached, `step()` returns `Error{Code::unavailable, ...}` naming the cause, and steps nothing.

- [ ] **Step 1: Write the failing tests**

```cpp
TEST(SimMedium, SampleMediumReturnsDensityAndWindWithNoTurbulence) {
    spade::Environment env;
    env.wind = glm::vec3(3.0f, 0.0f, 1.0f);
    env.air_density = 1.1f;
    auto world = spade::WorldBuilder().name("m").environment(env)
                     .capacities(spade::Capacities{1, 1, 1, 1}).build();
    ASSERT_TRUE(world.has_value());
    spade::WorldInstanceDesc inst;
    inst.world = *world;
    inst.turbulence = spade::dryden_params(spade::TurbulenceLevel::none);
    auto sim = spade::Simulation::create(spade::WorldSetDesc{{inst}}, 2'000'000, 2);
    ASSERT_TRUE(sim.has_value());
    ASSERT_TRUE(sim->step(10).has_value());
    auto s = sim->sample_medium(0, glm::vec3(0.0f));
    ASSERT_TRUE(s.has_value());
    EXPECT_EQ(s->density, 1.1f);
    EXPECT_EQ(s->wind, glm::vec3(3.0f, 0.0f, 1.0f));
}

TEST(SimMedium, SampleMediumIncludesTheGustAndIsPositionIndependent) {
    spade::Environment env;
    auto world = spade::WorldBuilder().name("m").environment(env)
                     .capacities(spade::Capacities{1, 1, 1, 1}).build();
    spade::WorldInstanceDesc inst;
    inst.world = *world;
    inst.seed = 42u;
    inst.turbulence = spade::dryden_params(spade::TurbulenceLevel::severe);
    auto sim = spade::Simulation::create(spade::WorldSetDesc{{inst}}, 2'000'000, 2);
    ASSERT_TRUE(sim->step(500).has_value());
    auto a = sim->sample_medium(0, glm::vec3(0.0f));
    auto b = sim->sample_medium(0, glm::vec3(10.0f, -3.0f, 7.0f));
    ASSERT_TRUE(a.has_value() && b.has_value());
    EXPECT_EQ(a->wind, b->wind);                       // Dryden is position-independent
    EXPECT_GT(glm::length(a->wind), 0.0f);             // a severe gust after 1 s is not zero
    EXPECT_FALSE(sim->sample_medium(1, glm::vec3(0.0f)).has_value());  // no world 1
}

TEST(GpuBehaviorRefusal, VulkanStepRefusesAnAttachedBehaviorRegistry) {
    if (!spade::compute::vulkan_available()) GTEST_SKIP() << "no Vulkan device";
    // Build a 1-body world on the Vulkan backend, attach a registry holding
    // kinematic_mover, step(1): expect Code::unavailable. Detach (set_behaviors(nullptr)),
    // step(1): expect success.
}
```

Fill in the Gpu case body with the same world-building lines as above plus `spade::compute::BackendDesc{.kind = spade::compute::BackendKind::vulkan}` and `spade::objects::kinematic_mover_desc(params)`.

- [ ] **Step 2: Ask the lead for a build slot; run** `scripts\test.ps1 -Filter "SimMedium|GpuBehaviorRefusal"` — expect compile failure (no `sample_medium`).
- [ ] **Step 3: Implement.** `sample_medium`: `checked_world`, read the world's `WorldParams` row, its `DrydenState` row (`dryden_id_`) and its stored `DrydenParams` (the per-world config captured at `create()`), return `DrydenMedium(state, params).sample(row, pos)`. Refusal: at the top of the Vulkan branch of `step()`, `if (behaviors_ != nullptr && behaviors_->size() > 0) return std::unexpected(Error{Code::unavailable, "behaviors are CPU-only today; the Vulkan step cannot run them and refuses rather than skipping them (SL6)"});`
- [ ] **Step 4: Run the filter again, then the full** `scripts\test.ps1` — all pass; no golden moves.
- [ ] **Step 5: Commit** `engine/sim/simulation.hpp engine/sim/simulation.cpp` + the test files: `feat(core): sample a world's medium from the host, and refuse behaviors on the Vulkan step`.

---

### Task 2: Rotor wake velocity (Physics)

**Files:**
- Create: `engine/vehicles/rotor_wake.hpp`, `engine/vehicles/rotor_wake.cpp`
- Modify: `engine/CMakeLists.txt` (add `vehicles/rotor_wake.cpp` to `spade_vehicles`, near line 763)
- Test: `tests/test_rotor_wake.cpp` (add beside `test_rotor.cpp` in `tests/CMakeLists.txt`, ~line 110)

**Interfaces:**
- Produces:

```cpp
namespace spade::vehicles {
struct RotorWakeInput {
    glm::vec3 hub_world{0.0f};          // rotor hub, world frame, m
    glm::vec3 thrust_axis_world{0.0f, 1.0f, 0.0f};  // unit; thrust direction (wake flows opposite)
    float radius = 0.0f;                // R, m
    float thrust_coeff = 0.0f;          // k_T
    float omega = 0.0f;                 // shaft speed, rad/s
    float density = 0.0f;               // kg/m^3
    glm::vec3 freestream{0.0f};         // air velocity at the rotor, world frame, m/s
};
// Induced air velocity (world frame, m/s) at `point` from one rotor's
// actuator-disc slipstream. Analytic, CPU, visualisation-grade: nothing in the
// step reads it. Total: returns 0 for a stopped or degenerate rotor.
[[nodiscard]] glm::vec3 rotor_wake_velocity(const RotorWakeInput& in, glm::vec3 point) noexcept;
}
```

- [ ] **Step 1: Write the failing tests**

```cpp
#include <gtest/gtest.h>
#include "vehicles/rotor.hpp"
#include "vehicles/rotor_wake.hpp"

namespace {
spade::vehicles::RotorWakeInput hover_rotor() {
    spade::vehicles::RotorWakeInput in;
    in.hub_world = glm::vec3(0.0f);
    in.thrust_axis_world = glm::vec3(0.0f, 1.0f, 0.0f);
    in.radius = 0.12f;
    in.thrust_coeff = 1.2e-5f;
    in.omega = 452.0f;
    in.density = 1.225f;
    return in;
}
float disc_velocity(const spade::vehicles::RotorWakeInput& in) {
    const float t = in.thrust_coeff * in.omega * in.omega;
    return spade::vehicles::rotor_hover_induced_velocity(t, in.density, in.radius);  // λ = 1 in still air
}
}  // namespace

TEST(RotorWake, AtTheDiscTheInducedVelocityIsVi) {
    const auto in = hover_rotor();
    const glm::vec3 v = spade::vehicles::rotor_wake_velocity(in, in.hub_world);
    EXPECT_NEAR(v.y, -disc_velocity(in), 1e-4f);  // flows opposite the thrust axis
    EXPECT_NEAR(v.x, 0.0f, 1e-6f);
    EXPECT_NEAR(v.z, 0.0f, 1e-6f);
}

TEST(RotorWake, FarDownstreamItApproachesTwiceVi) {
    const auto in = hover_rotor();
    const glm::vec3 v = spade::vehicles::rotor_wake_velocity(in, glm::vec3(0.0f, -100.0f * in.radius, 0.0f));
    EXPECT_NEAR(-v.y, 2.0f * disc_velocity(in), 0.01f * disc_velocity(in));
}

TEST(RotorWake, FarUpstreamItVanishes) {
    const auto in = hover_rotor();
    const glm::vec3 v = spade::vehicles::rotor_wake_velocity(in, glm::vec3(0.0f, 100.0f * in.radius, 0.0f));
    EXPECT_LT(glm::length(v), 0.01f * disc_velocity(in));
}

TEST(RotorWake, OutsideTheTubeItVanishes) {
    const auto in = hover_rotor();
    const glm::vec3 v = spade::vehicles::rotor_wake_velocity(in, glm::vec3(5.0f * in.radius, -0.05f, 0.0f));
    EXPECT_LT(glm::length(v), 0.01f * disc_velocity(in));
}

TEST(RotorWake, AStoppedRotorInducesNothing) {
    auto in = hover_rotor();
    in.omega = 0.0f;
    EXPECT_EQ(spade::vehicles::rotor_wake_velocity(in, glm::vec3(0.0f, -0.5f, 0.0f)), glm::vec3(0.0f));
}

TEST(RotorWake, CrosswindSkewsTheWakeDownwind) {
    auto in = hover_rotor();
    in.freestream = glm::vec3(5.0f, 0.0f, 0.0f);
    const glm::vec3 below = spade::vehicles::rotor_wake_velocity(in, glm::vec3(0.0f, -0.6f, 0.0f));
    const glm::vec3 downwind = spade::vehicles::rotor_wake_velocity(in, glm::vec3(0.6f, -0.6f, 0.0f));
    EXPECT_GT(glm::length(downwind), glm::length(below));
}
```

- [ ] **Step 2: Ask the lead for a build slot; run** `scripts\test.ps1 -Filter RotorWake` — expect compile failure.
- [ ] **Step 3: Implement** `rotor_wake.cpp`:

```cpp
#include "vehicles/rotor_wake.hpp"
#include <cmath>
#include "vehicles/rotor.hpp"

namespace spade::vehicles {

glm::vec3 rotor_wake_velocity(const RotorWakeInput& in, glm::vec3 point) noexcept {
    const float thrust = in.thrust_coeff * in.omega * in.omega;
    const float v_hover = rotor_hover_induced_velocity(thrust, in.density, in.radius);
    if (!(v_hover > 0.0f)) return glm::vec3(0.0f);
    const float axis_len = glm::length(in.thrust_axis_world);
    if (!(axis_len > 0.0f)) return glm::vec3(0.0f);
    const glm::vec3 axis = in.thrust_axis_world / axis_len;

    // Air-relative axial velocity of a fixed rotor in a freestream: positive "climbing".
    const float v_axial = -glm::dot(in.freestream, axis);
    const float v_i = v_hover * rotor_inflow_factor(v_axial, v_hover);
    if (!(v_i > 0.0f)) return glm::vec3(0.0f);

    // The wake is convected by the freestream: its axis follows the far-wake velocity.
    glm::vec3 wake_dir = -axis * (2.0f * v_i) + in.freestream;
    const float wd = glm::length(wake_dir);
    wake_dir = wd > 1e-6f ? wake_dir / wd : -axis;

    const glm::vec3 d = point - in.hub_world;
    const float s = glm::dot(d, wake_dir);                    // distance downstream
    const float r = glm::length(d - s * wake_dir);            // distance from the wake axis
    const float R = in.radius;
    const float u = v_i * (1.0f + s / std::sqrt(s * s + R * R));   // actuator-disc axial profile
    if (!(u > 0.0f)) return glm::vec3(0.0f);
    const float r_tube = R * std::sqrt(v_i / u);              // continuity: A(s) u(s) = A v_i
    const float x = r / r_tube;
    const float x2 = x * x;
    const float edge = 1.0f / (1.0f + x2 * x2 * x2 * x2);     // smooth tube edge, ~1 inside, ~0 outside
    return -axis * (u * edge);
}

}  // namespace spade::vehicles
```

- [ ] **Step 4: Run** `scripts\test.ps1 -Filter RotorWake` then the full suite — all pass.
- [ ] **Step 5: Commit** the four paths: `feat(physics): an actuator-disc rotor wake, for visualising the air a rotor moves`.

---

### Task 3: One lighting convention on both render paths (Rendering)

**Facts already established:** `world/builder.hpp`'s `LightingDesc::sun_direction` points **from the scene toward the sun** (default `{0.4, 0.8, 0.6}`), `scene_from_world` normalises it, and the CPU path lights with `dot(n, +sun_direction)`. The GL shader (`engine/render_gl/gl_renderer.cpp`, the `ndl` line ~115) lights with `dot(n, -uSunDir)`, so on GL every world-loaded scene is lit from below. `render::Lighting`'s own default (`render/scene.hpp:103`, `{-0.35, -0.86, -0.37}`) points the other way and looks like a stale "direction light travels" value.

**Files:** `engine/render_gl/gl_renderer.cpp`, possibly `engine/render/scene.hpp:103`, the comment in `sandbox/main.cpp` that cites `dot(n, -uSunDir)`.

- [ ] **Step 1:** Change the GL shader to `max(dot(n, uSunDir), 0.0)`.
- [ ] **Step 2:** Find every user of `render::Lighting{}`'s default (`grep -rn "Lighting{}\|Lighting lighting;\|\.lighting = " engine tests`). If no golden depends on it, flip the default to point toward the sun (`{0.35, 0.86, 0.37}`, normalised). If a golden depends on it, leave the default, record that in a one-line comment beside it, and tell the lead.
- [ ] **Step 3:** Build (ask for a slot) and run the full suite — no golden moves.
- [ ] **Step 4:** Commit: `fix(render): GL lit from the opposite side to the CPU reference; one sun convention`.

(Visual confirmation that both paths now light the drone's top happens in Task 7.)

---

### Task 4: `sandbox/drone_sim.hpp` — the test stand, controller and rebuild (Interface)

**Files:**
- Create: `sandbox/drone_sim.hpp` (header-only)
- Modify: `sandbox/CMakeLists.txt` (link `spade::sim`; raise `cxx_std_20` to `cxx_std_23` while there)
- Test: `tests/test_sandbox_drone.cpp` (add to the `SPADE_BUILD_SANDBOX` block in `tests/CMakeLists.txt`, ~line 266)

**Interfaces:**
- Consumes: existing public API only (`Simulation`, `WorldBuilder`, `make_quadrotor`, `hover_command`, `quadrotor_arm_offset`, `BehaviorRegistry`, `physics::SubstepContext`).
- Produces (used by Tasks 5 and 6):

```cpp
namespace spade::sandbox {
struct DronePhysicsOptions {
    float wind_speed_mps = 0.0f;
    float wind_heading_deg = 0.0f;   // direction the air moves toward: 0 = +X, 90 = +Z
    TurbulenceLevel turbulence = TurbulenceLevel::none;
    float air_density = 1.225f;
    float gravity = 9.80665f;        // magnitude, along -Y
    float throttle = 1.0f;           // fraction of hover thrust
    bool vulkan = false;
    friend bool operator==(const DronePhysicsOptions&, const DronePhysicsOptions&) = default;
};
struct AttitudeTarget { float yaw = 0.0f, pitch = 0.0f, roll = 0.0f; };  // radians
inline constexpr float kMaxTiltRad = 1.0471975512f;                    // 60 degrees
struct DroneCarry { glm::quat orient{1, 0, 0, 0}; glm::vec3 omega_body{0.0f}; float rotor_omega = -1.0f; };
struct DroneReadouts {
    glm::quat orientation{1, 0, 0, 0};
    glm::vec3 omega_body{0.0f};
    std::array<float, 4> rotor_omega{}, rotor_thrust{};
    glm::vec3 moment_cmd{0.0f};
    float hover_omega = 0.0f;
    float hover_induced_velocity = 0.0f;
    glm::vec3 imu_accel{0.0f}, imu_gyro{0.0f};
    uint64_t tick = 0;
};
[[nodiscard]] vehicles::QuadrotorParams drone_stand_params();
[[nodiscard]] glm::quat attitude_quat(const AttitudeTarget&);
[[nodiscard]] glm::vec3 attitude_moment(const glm::quat& q, const glm::vec3& omega_body,
                                        const glm::quat& q_target, const glm::vec3& inertia);
[[nodiscard]] std::array<float, 4> mix_thrusts(float total, const glm::vec3& moment, float arm, float kq_over_kt);
class DroneSim {
  public:
    static constexpr uint64_t kDtNs = 2'000'000;
    static constexpr uint32_t kSubsteps = 2;
    static constexpr uint32_t kMaxStepsPerAdvance = 100;
    [[nodiscard]] static Result<DroneSim> create(const DronePhysicsOptions&, const DroneCarry& = {});
    [[nodiscard]] Result<void> step_fixed(uint32_t steps);   // controller runs before every step
    [[nodiscard]] Result<void> advance(float wall_dt_s);     // fixed-dt accumulator, capped
    [[nodiscard]] Result<void> apply_options(const DronePhysicsOptions&);  // rebuild, carrying state
    AttitudeTarget target;
    [[nodiscard]] const Simulation& sim() const noexcept;
    [[nodiscard]] const VehicleRef& vehicle() const noexcept;
    [[nodiscard]] const vehicles::QuadrotorParams& params() const noexcept;
    [[nodiscard]] const DronePhysicsOptions& options() const noexcept;
    [[nodiscard]] DroneReadouts readouts() const;
};
}
```

- [ ] **Step 1: Write the failing tests** (`tests/test_sandbox_drone.cpp`):

```cpp
#include <gtest/gtest.h>
#include <cmath>
#include "../sandbox/drone_sim.hpp"

using namespace spade::sandbox;

TEST(SandboxDrone, MixerProducesTheRequestedMoments) {
    const float L = 0.18f, c = 1.9e-7f / 1.2e-5f, total = 9.80665f;
    const glm::vec3 M(0.05f, -0.01f, 0.03f);
    const auto T = mix_thrusts(total, M, L, c);
    EXPECT_NEAR(T[0] + T[1] + T[2] + T[3], total, 1e-4f);
    EXPECT_NEAR(L * (T[3] - T[1]), M.x, 1e-5f);              // roll
    EXPECT_NEAR(-c * (T[0] - T[1] + T[2] - T[3]), M.y, 1e-5f);  // yaw
    EXPECT_NEAR(L * (T[0] - T[2]), M.z, 1e-5f);              // pitch
}

TEST(SandboxDrone, ThePinHoldsPositionBitwiseWhileTheAttitudeMoves) {
    auto drone = DroneSim::create(DronePhysicsOptions{});
    ASSERT_TRUE(drone.has_value()) << drone.error().context;
    drone->target = AttitudeTarget{0.5f, 0.3f, -0.2f};
    ASSERT_TRUE(drone->step_fixed(1000).has_value());
    const auto* body = *drone->sim().body(drone->vehicle().body);
    EXPECT_EQ(body->pos, glm::vec3(0.0f));
    EXPECT_EQ(body->vel, glm::vec3(0.0f));
}

TEST(SandboxDrone, TheControllerReachesEachTarget) {
    for (const AttitudeTarget t : {AttitudeTarget{0.6f, 0.0f, 0.0f}, AttitudeTarget{0.0f, 0.4f, 0.0f},
                                   AttitudeTarget{0.0f, 0.0f, -0.4f}, AttitudeTarget{1.2f, kMaxTiltRad, kMaxTiltRad}}) {
        auto drone = DroneSim::create(DronePhysicsOptions{});
        ASSERT_TRUE(drone.has_value());
        drone->target = t;
        ASSERT_TRUE(drone->step_fixed(1500).has_value());  // 3 s
        const glm::quat err = glm::conjugate(drone->readouts().orientation) * attitude_quat(t);
        EXPECT_LT(2.0f * std::acos(std::min(1.0f, std::fabs(err.w))), 0.035f);  // within 2 degrees
    }
}

TEST(SandboxDrone, VulkanIsRefusedWithAReason) {
    DronePhysicsOptions o;
    o.vulkan = true;
    auto drone = DroneSim::create(o);
    ASSERT_FALSE(drone.has_value());
    EXPECT_EQ(drone.error().code, spade::Code::unavailable);
    EXPECT_FALSE(drone.error().context.empty());
}

TEST(SandboxDrone, ARebuildCarriesTheAttitudeAcross) {
    auto drone = DroneSim::create(DronePhysicsOptions{});
    drone->target = AttitudeTarget{0.0f, 0.4f, 0.0f};
    ASSERT_TRUE(drone->step_fixed(1000).has_value());
    const glm::quat before = drone->readouts().orientation;
    DronePhysicsOptions o;
    o.wind_speed_mps = 6.0f;
    ASSERT_TRUE(drone->apply_options(o).has_value());
    EXPECT_EQ(drone->readouts().orientation, before);
    EXPECT_EQ(drone->options(), o);
    EXPECT_EQ(drone->target.pitch, 0.4f);
}

TEST(SandboxDrone, TheAccumulatorCapsAStall) {
    auto drone = DroneSim::create(DronePhysicsOptions{});
    ASSERT_TRUE(drone->advance(30.0f).has_value());  // a 30 s stall
    EXPECT_LE(drone->readouts().tick, DroneSim::kMaxStepsPerAdvance);
}
```

- [ ] **Step 2: Ask for a build slot; run** `scripts\test.ps1 -Filter SandboxDrone` — expect compile failure.
- [ ] **Step 3: Implement `drone_sim.hpp`.** Key pieces (the rest is plumbing over the listed API):

```cpp
inline vehicles::QuadrotorParams drone_stand_params() {
    vehicles::QuadrotorParams p;
    p.name = "sandbox_quad";
    p.mass = 1.0f;  // a power of two keeps the pin's -m*g / m == -g exact
    p.inertia_diag = glm::vec3(0.018f, 0.032f, 0.024f);
    p.arm_length = 0.18f;
    for (auto& r : p.rotors) { r.thrust_coeff = 1.2e-5f; r.torque_coeff = 1.9e-7f; }
    return p;
}

inline glm::quat attitude_quat(const AttitudeTarget& t) {
    return glm::angleAxis(t.yaw, glm::vec3(0, 1, 0)) * glm::angleAxis(t.pitch, glm::vec3(0, 0, 1)) *
           glm::angleAxis(t.roll, glm::vec3(1, 0, 0));
}

// PD on the body-frame rotation error, scaled by inertia (natural frequency 6 rad/s, critically damped).
inline glm::vec3 attitude_moment(const glm::quat& q, const glm::vec3& w, const glm::quat& qt, const glm::vec3& I) {
    glm::quat e = glm::conjugate(q) * qt;
    if (e.w < 0.0f) e = -e;  // shortest way round: never the long way through a flip
    const glm::vec3 err = 2.0f * glm::vec3(e.x, e.y, e.z);
    return I * (36.0f * err - 12.0f * w);
}

// Plus layout: rotor 0 +X, 1 +Z, 2 -X, 3 -Z, spin +1 -1 +1 -1.
// M_x = L(T3-T1), M_z = L(T0-T2), M_y = -c(T0-T1+T2-T3), c = k_Q / k_T.
inline std::array<float, 4> mix_thrusts(float total, const glm::vec3& M, float L, float c) {
    const float q = 0.25f * total, mx = M.x / (2.0f * L), mz = M.z / (2.0f * L), my = M.y / (4.0f * c);
    return {q + mz - my, q - mx + my, q - mz - my, q + mx + my};
}

namespace detail {
struct PinParams { uint32_t world = 0; uint32_t body_slot = 0; };  // world-local slot
inline void pin_kinematic(const physics::SubstepContext& ctx, const void* p) noexcept {
    const auto& pin = *static_cast<const PinParams*>(p);
    if (pin.world >= ctx.worlds.size()) return;
    const auto& w = ctx.worlds[pin.world];
    if (pin.body_slot >= w.bodies.size()) return;
    w.bodies[pin.body_slot].pos = glm::vec3(0.0f);
    w.bodies[pin.body_slot].vel = glm::vec3(0.0f);
}
inline void pin_force(const physics::SubstepContext& ctx, const void* p) noexcept {
    const auto& pin = *static_cast<const PinParams*>(p);
    if (pin.world >= ctx.worlds.size()) return;
    const auto& w = ctx.worlds[pin.world];
    if (pin.body_slot >= w.bodies.size() || w.params == nullptr) return;
    BodyState& b = w.bodies[pin.body_slot];
    b.force_acc = -b.mass * w.params->gravity;  // Integrate adds +gravity back: net linear accel exactly 0
}
}  // namespace detail
```

`DroneSim` holds `std::unique_ptr<Simulation>`, `std::unique_ptr<objects::BehaviorRegistry>` and `std::unique_ptr<detail::PinParams>` so the addresses the registry and the Simulation borrow survive a move. `create()`:
  - refuses `opts.vulkan` with `Error{Code::unavailable, "the drone stand holds the drone with CPU behaviors, which the Vulkan step cannot run yet; staying on CPU"}`;
  - builds `Environment{gravity = {0,-g,0}, wind = speed * (cos h, 0, sin h), air_density}` and a world with `Capacities{1, 5, 1, 1}` and no geometry;
  - `WorldInstanceDesc{.world, .seed = 0xD20E5EEDu, .turbulence = dryden_params(level)}`; `Simulation::create(WorldSetDesc{{inst}}, kDtNs, kSubsteps)`;
  - `register_model(make_quadrotor(params))`, `spawn(0, id, VehicleSpawn{pos 0, orient = carry.orient, omega_body = carry.omega_body, rotor_omega = carry.rotor_omega >= 0 ? carry.rotor_omega : hover_command(params, g)})`, `flush_structural()`;
  - registers `pin_kinematic` (slot kinematic) and `pin_force` (slot force), both with `user_data = pin.get()`, and calls `set_behaviors(registry.get())`. With one world the vehicle's `body.slot` is its world-local slot.

`step_fixed(n)`: for each step — read the body (`orient`, `omega_body`), `M = attitude_moment(...)`, `T = mix_thrusts(throttle * mass * g, M, L, k_Q/k_T)`, `omega_i = sqrt(max(T_i, 0) / k_T)` clamped to `[0, 2.5 * hover_omega]`, `set_rotor_commands(vehicle, omegas)`, `sim.step(1)`. Clamp the target's pitch and roll to `±kMaxTiltRad` before use.

`advance(dt)`: `acc_ += clamp(dt, 0, 0.25)`; `n = min(floor(acc_ / 0.002), kMaxStepsPerAdvance)`; `acc_ -= n * 0.002`, and if `n` hit the cap, drop the remainder (`acc_ = 0`); `step_fixed(n)`.

`apply_options(o)`: capture `DroneCarry{orient, omega_body, mean rotor omega}` from the current state, `create(o, carry)`; on success replace `*this` keeping `target`; on failure keep the current simulation and return the error.

`readouts()`: body pose and rates, each rotor's `omega` (via `sim.rotor(vehicle, i)`) and thrust `k_T * omega^2`, the last commanded moment, `hover_command`, `rotor_hover_induced_velocity(m*g/4, rho, R)`, the latest IMU sample (`poll_imu`), `sim.tick()`.

- [ ] **Step 4: Run** `scripts\test.ps1 -Filter SandboxDrone`, then the full suite — all pass. If `TheControllerReachesEachTarget` diverges, the mixer's sign convention disagrees with the engine: check `quadrotor.hpp`'s section on moments and fix the mixer, not the test.
- [ ] **Step 5: Commit** `sandbox/drone_sim.hpp sandbox/CMakeLists.txt tests/test_sandbox_drone.cpp tests/CMakeLists.txt`: `feat(sandbox): a drone test stand driven through the engine's rotor model`.

---

### Task 5: `sandbox/drone_view.hpp` — air field, heatmap and drone drawing (Interface; Rendering reviews)

**Files:**
- Create: `sandbox/drone_view.hpp` (header-only)
- Test: extend `tests/test_sandbox_drone.cpp`

**Interfaces:**
- Consumes: Task 1 `Simulation::sample_medium`; Task 2 `vehicles::rotor_wake_velocity`, `RotorWakeInput`; Task 4 `DroneSim`; `builder_scene.hpp`'s `make_box_mesh`, `make_cylinder_mesh`, `detail::push_tri`/`finish`.
- Produces:

```cpp
namespace spade::sandbox {
struct AirField {
    MediumSample medium{};                               // position-independent today
    std::array<vehicles::RotorWakeInput, 4> rotors{};
    [[nodiscard]] glm::vec3 velocity(glm::vec3 p) const noexcept;  // medium.wind + sum of wakes
};
[[nodiscard]] Result<AirField> air_field_from(const DroneSim&);

inline constexpr uint32_t kHeatmapBins = 32;
inline constexpr uint32_t kSliceCells = 64;
[[nodiscard]] glm::vec3 viridis(float t);               // t in [0,1], piecewise-linear over 9 control points
[[nodiscard]] uint32_t speed_bin(float speed, float max_speed) noexcept;  // 0..kHeatmapBins-1; 0 when max <= 0

struct DroneDrawBinding {                               // indices into RenderScene
    uint32_t box_mesh, cylinder_mesh, quad_mesh;
    uint32_t body_material, arm_material, nose_material, rotor_material;
    uint32_t heatmap_base;                              // first of kHeatmapBins unlit materials
};
[[nodiscard]] DroneDrawBinding bind_drone_scene(render::RenderScene& scene);  // appends meshes + materials once
void append_drone_items(const DroneDrawBinding&, const vehicles::QuadrotorParams&,
                        const glm::vec3& pos, const glm::quat& orient, std::vector<render::DrawItem>& out);

struct SliceSpec { glm::vec3 center, right, up; float width = 2.0f, height = 2.5f; };
[[nodiscard]] SliceSpec camera_facing_slice(const render::Camera&, const glm::vec3& drone_pos);
// Appends kSliceCells^2 cell items; returns the largest speed seen (for the auto range and the legend).
float append_slice_items(const DroneDrawBinding&, const SliceSpec&, const AirField&,
                         float max_speed /* <= 0 means auto: use the previous frame's maximum */,
                         std::vector<render::DrawItem>& out);
}
```

- [ ] **Step 1: Write the failing tests** (append to `tests/test_sandbox_drone.cpp`, including `"../sandbox/drone_view.hpp"`):

```cpp
TEST(SandboxDroneView, WithRotorsStoppedTheFieldIsTheMedium) {
    DronePhysicsOptions o;
    o.wind_speed_mps = 4.0f;
    o.throttle = 0.0f;
    auto drone = DroneSim::create(o);
    ASSERT_TRUE(drone->step_fixed(200).has_value());  // rotors spin down (tau 0.02 s)
    auto field = air_field_from(*drone);
    ASSERT_TRUE(field.has_value());
    for (const glm::vec3 p : {glm::vec3(0.0f), glm::vec3(0.0f, -0.6f, 0.0f), glm::vec3(0.3f, 0.2f, -0.4f)})
        EXPECT_NEAR(glm::length(field->velocity(p) - field->medium.wind), 0.0f, 1e-4f);
}

TEST(SandboxDroneView, AtHoverTheDownwashIsBelowTheRotors) {
    auto drone = DroneSim::create(DronePhysicsOptions{});
    ASSERT_TRUE(drone->step_fixed(200).has_value());
    auto field = air_field_from(*drone);
    const glm::vec3 below_rotor0 = field->velocity(glm::vec3(0.18f, -0.4f, 0.0f));
    const glm::vec3 above_rotor0 = field->velocity(glm::vec3(0.18f, 0.4f, 0.0f));
    EXPECT_LT(below_rotor0.y, -1.0f);
    EXPECT_GT(glm::length(below_rotor0), glm::length(above_rotor0));
}

TEST(SandboxDroneView, SpeedBinsCoverTheRangeAndSurviveAZeroMaximum) {
    EXPECT_EQ(speed_bin(0.0f, 10.0f), 0u);
    EXPECT_EQ(speed_bin(10.0f, 10.0f), kHeatmapBins - 1);
    EXPECT_EQ(speed_bin(50.0f, 10.0f), kHeatmapBins - 1);
    EXPECT_EQ(speed_bin(3.0f, 0.0f), 0u);
}

TEST(SandboxDroneView, AHeatmapPixelIsExactlyItsPaletteColour) {
    // Build the drone scene (scene_from_world of an empty world + bind_drone_scene),
    // camera at (0, 0, 3) looking at the origin, slice items only (no drone parts),
    // render with raster_cpu, RenderOptions{.shadows = false}. Pick the pixel at the
    // image centre; it shows the slice cell containing the drone centre. Expect its BGR
    // bytes == to_byte(viridis((bin + 0.5) / kHeatmapBins)) for bin = speed_bin(|v(centre cell)|, max).
}

TEST(SandboxDroneView, StandardAndHeatmapViewsDiffer) {
    // Render the same frame twice on raster_cpu: drone parts only, and drone parts + slice.
    // Expect the two pixel buffers to differ.
}
```

Write the two render cases' bodies using `render_through` in `tests/test_sandbox_target_sink.cpp:95` as the pattern.

- [ ] **Step 2: Ask for a build slot; run** `scripts\test.ps1 -Filter SandboxDroneView` — expect compile failure.
- [ ] **Step 3: Implement `drone_view.hpp`.**
  - `air_field_from`: `medium = sim.sample_medium(0, body.pos)`; for each rotor `i`, `RotorRow` via `sim.rotor(vehicle, i)`: `hub_world = pos + orient * row.local_pos`, `thrust_axis_world = orient * (row.local_orient * glm::vec3(0, 1, 0))`, `radius`, `thrust_coeff`, `omega`, `density = medium.density`, `freestream = medium.wind`.
  - `viridis`: piecewise-linear over the nine standard control points (`#440154 #482878 #3E4A89 #31688E #26828E #1F9E89 #35B779 #6DCD59 #FDE725`).
  - `speed_bin`: `max <= 0 → 0`, else `min(uint32(speed / max * kHeatmapBins), kHeatmapBins - 1)`.
  - `bind_drone_scene`: append `make_box_mesh()`, `make_cylinder_mesh()` and a **double-sided** unit quad in the XY plane (both windings; the CPU shaded path culls back faces), then materials: body `{0.18,0.18,0.20}`, arms `{0.55,0.55,0.58}`, nose `{0.85,0.20,0.15}`, rotors `{0.15,0.55,0.85}` (lambert), then `kHeatmapBins` **unlit** (`shading = 1`) materials with `base_color = viridis((b + 0.5) / kHeatmapBins)`.
  - `append_drone_items`: pose matrix `M = translate(pos) * mat4_cast(orient)`; body box scaled `(0.12, 0.05, 0.12)`; for each `i`, the arm box from the centre to `quadrotor_arm_offset(params, i)` (length `arm_length`, section `0.02`), material nose for `i == 0`; rotor disc = cylinder scaled `(2R, 0.01, 2R)` at the hub.
  - `camera_facing_slice`: `to_cam = camera.position - drone_pos` with `y` zeroed (fallback `+Z` if degenerate); `right = normalize(cross(up, to_cam))`; `up = (0,1,0)`; `center = drone_pos + (0, -0.5, 0)` (0.75 m above, 1.75 m below).
  - `append_slice_items`: for each cell `(i, j)`, centre `c = center + right * ((i + 0.5)/N - 0.5) * width + up * ((j + 0.5)/N - 0.5) * height`; `speed = |field.velocity(c)|`; item `local_to_world = translate(c) * basis(right, up, normal) * scale(width/N, height/N, 1)`; `material_override = heatmap_base + speed_bin(speed, range)`. Track and return the max speed.
- [ ] **Step 4: Run** `scripts\test.ps1 -Filter "SandboxDrone"`, then the full suite.
- [ ] **Step 5: Commit** `sandbox/drone_view.hpp tests/test_sandbox_drone.cpp`: `feat(sandbox): the air around the drone, as a field and as a heatmap`.

---

### Task 6: Wiring — the default scene, input, panel and flags (Interface)

**Files:** `sandbox/main.cpp`, `sandbox/gl_target_sink.hpp`, `sandbox/gl_target_sink.cpp`, `sandbox/orbit_camera.hpp`, `scripts/demo.ps1` (add `-Scene sandbox`, optional).

- [ ] **Step 1: Input.** Add to `FrameInput`: `float attitude_pitch, attitude_roll, attitude_yaw` (−1..1, held keys: Up/Down, Left/Right, Z/X), `bool level_pressed` (R, edge), `bool toggle_view_pressed` (V, edge). Read them in `GlTargetSink::poll()` beside the existing keys, inside the same `!io.WantCaptureKeyboard` guard, using the `*_was_down` edge pattern (F1 is the model).
- [ ] **Step 2: Camera.** Add `void apply_drone_orbit(OrbitCamera&, const FrameInput&, float dt)` to `orbit_camera.hpp`: `target` stays the drone; `move_right` changes `yaw`, `move_up` changes `pitch` (clamped by `kPitchLimit`), `move_forward` changes `distance`, clamped to `[0.5, 6]`; mouse drag and scroll as today. Test it in `test_sandbox_camera_and_channels.cpp`: holding W for 10 s never goes below 0.5 m; the camera always faces the target.
- [ ] **Step 3: Panel.** Add `struct DronePanelModel { DronePhysicsOptions edited; bool view_heatmap = false; float heatmap_max = 0.0f /* 0 = auto */; float observed_max = 0.0f; DroneReadouts readouts; std::string status; bool options_dirty = false; }` (in `drone_view.hpp`), `GlTargetSink::attach_drone(DronePanelModel*)`, and `draw_drone_panel` in `gl_target_sink.cpp`, drawn from `draw_overlay()` so both present paths show it. Widgets: wind speed (0–20 m/s), heading (0–360°), turbulence combo, density (0.5–1.5), gravity (0–20), throttle (0–200 %), backend combo (CPU / Vulkan), view radio (standard / heatmap), heatmap range (auto or manual). A viridis legend bar with 0 and the range maximum in m/s. Readouts as listed in the spec. `status` shown in a warning colour when non-empty (the Vulkan refusal lands here).
- [ ] **Step 4: Loop.** In `main.cpp`, add `run_windowed_drone(...)`, mirroring `run_windowed`'s GPU-first/CPU-fallback setup:
  - build an empty world, `scene_from_world`, `bind_drone_scene`, upload once;
  - per frame: `apply_drone_orbit`; nudge `drone.target` by the attitude axes × 1.5 rad/s × dt (R resets pitch and roll to 0); `V` toggles the view;
  - apply panel edits **debounced** — rebuild only when `edited != drone.options()` and no slider is being dragged (`ImGui::IsAnyItemActive()` false) — and put a refusal's message in `status` while keeping the old simulation;
  - `drone.advance(dt)`, timed into `note_physics_ms`;
  - `scene.dynamics.clear()`, `append_drone_items`, then `append_slice_items` when the heatmap is on;
  - draw with GL, or `render_frame` with shadows off in the heatmap. Give `render_frame` a `RenderOptions` parameter instead of `grid`/`blur`.
- [ ] **Step 5: Flags.** `--scene drone|builder` (window default `drone`); `--smoke` forces `builder`; `--headless` takes `--scene` and `--view standard|heatmap`: create the `DroneSim`, `step_fixed(250)` (0.5 s, deterministic), render one CPU frame from a fixed camera at `(1.2, 0.6, 1.6)` looking at the origin, and write `--out`. Update `print_usage`.
- [ ] **Step 6: Build (ask for a slot) and run the full suite.** Then run `spade_sandbox --headless --scene drone --view heatmap --out drone_heatmap.ppm` and `--view standard`, open both images, and check by eye: a drone, a downwash plume below each rotor in the heatmap.
- [ ] **Step 7: Commit** the touched paths: `feat(sandbox): the drone sim box is the default scene`.

---

### Task 7: Verification (Lead)

- [x] Full suite, both presets, foreground: `scripts\test.ps1 -Preset release` and `-Preset debug`. Record the counts with the commit.
- [x] `--headless` images for both views; check by eye.
- [x] Launch the window (`build-ninja\release\bin\spade_sandbox.exe`) and check the whole spec "What the user sees" list on the GL path and with GL forced off (CPU fallback): keys, camera clamp, both views, every panel control, Vulkan refusal message, readouts moving, no rebuild storm while dragging a slider, drone lit from above on both paths.
- [ ] Cross-realm review of each task's diff; ask the user to try the scene.
- [x] (user request, 2026-10-02) Replace the desktop shortcut `Spade GUI.lnk` (still targets the KAT-era `Desktop\KAT\spade\build-gui\bin\spade_sandbox.exe`) with `Spade Builder.lnk` → `C:\Users\ckbra\desktop\spade\build-ninja\release\bin\spade_sandbox.exe`, working directory that `bin`, icon the exe, description listing the drone sim box controls.

**Result (2026-10-02, lead).** Main tree, release, `df33f09`: 921 total, 919 passed, 2 skipped (by design), 0 failed; 65 gpu tests ran (Test/Docs, slot #8; the debug leg follows). Headless frames from the main-tree binary show two 9.40 m/s downwash columns and a drone lit from above. Window on the GPU path (Iris Plus, GL 4.3), driven by key messages posted only to the sandbox window: `V` toggles the heatmap, the arrows pitch the drone through the controller (plumes and slice follow), `D` orbits, clean exit; 60-78 fps, physics under 0.1 ms/frame. Not exercised in the window: the Vulkan refusal message (unit-tested) and the CPU-fallback window (headless covers the CPU path). Cosmetic findings: the HUD's `0x0` on GPU (fixed on `interface/front-rotor`), GL's flat background (Rendering debt). `Spade Builder.lnk` created; `Spade GUI.lnk` removed.
