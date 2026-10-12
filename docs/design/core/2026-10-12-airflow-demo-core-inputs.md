# Airflow demonstration: Core's inputs

**Owner:** Core. **Status:** research draft for the lead. Read-only: nothing edited, built or committed.
**Base:** master `1fa9632`. Also read: `physics/airflow-core` at `a80420f` (plan A Task 6), `core/module-grades` at `9bd4c66` (stage 5 Task 1, WIP).
**The goal** (Cameron, 2026-10-12, `backlog.md:65`): a full 3D view of a drone and the air around it, with propeller flow, vortices, surface pressure, and the CFD analyses behind it.

## Summary

- Every rotor and body quantity a one-way demo needs is readable today through public API, cheaply, on both backends. Thrust is the one exception: it is not stored, so the host recomputes it.
- No vehicle carries a solid shape. The solver cannot see the drone as a body. That matters only for true surface pressure.
- The goal needs one-way coupling first. One-way can run outside `Simulation`, with host reads each frame. It needs none of stages 5, 6 or 7.
- The spec's own path to the sandbox scene (M3) assumes the airflow module, so it does wait on stage 7.
- Keep the code order 5 → 6 → 7. Send stage 7's design to Cameron now, in parallel. Stage 6 is not on the demo's critical path.

## 1. Per-frame reads

### How the sandbox reads state today

`sandbox/drone_view.hpp:64-89` (`air_field_from`) already does this every frame for the analytic wake:
- `sim.body(ref)` for the pose;
- `sim.sample_medium(0, pos)` for wind and density;
- `sim.rotor(vehicle, i)` for each rotor row;
- hub = `pos + orient * local_pos`; axis = `orient * (local_orient * kRotorLocalThrustAxis)`.

A demo harness reads the same things and builds the solver's `ActuatorDisc` (plan A Task 9: centre, axis, radius, thrust) instead of a `RotorWakeInput`.

### (a) Rotor states

| Quantity | Status | Where it comes from |
|---|---|---|
| Hub position, world | **Exists** | `body()->pos + body()->orient * rotor()->local_pos` (`layout.hpp:113,121`; `rotor.hpp:541`). Use `body()`, not `vehicle_state()`: `local_pos` is from the centre of mass in the principal frame, and `vehicle_state()` is the design frame |
| Thrust axis, world | **Exists** | `orient * (local_orient * kRotorLocalThrustAxis)` (`rotor.hpp:429,544`). The disc pushes air along minus this |
| Spin speed | **Exists** | `RotorRow::omega` (`rotor.hpp:546`): the lagged speed the last substep used |
| Spin direction | **Exists** | `RotorRow::spin_dir`, +1 or −1 about the thrust axis (`rotor.hpp:542`) |
| Disc radius | **Exists** | `RotorRow::radius` (`rotor.hpp:539`) |
| Shaft torque (for swirl) | **Exists, exact** | `Q = torque_coeff * omega²` (`rotor.hpp:549`; `rotor.cpp:338-340`). No correction is applied to torque |
| Thrust | **Gap** | Not stored. `apply_rotors` computes `T = k_T ω² · f_inflow · f_ground` and adds it straight into `force_acc` (`rotor.cpp:338-380`) |
| Wind and density at the hub | **Exists** | `sample_medium(world, p)` (`simulation.hpp:1503`; `simulation.cpp:2953-2961`). Dryden only |
| Hub velocity | **Exists** | `vel + orient * cross(omega_body, local_pos)` (`layout.hpp:122,124`) |
| The vehicle's rotor count and slots | **Exists** | `VehicleRef::rotor_count`, from `spawn()` or `vehicle_ref_at()` (`simulation.hpp:1488`) |

**Thrust, the smallest fix.**
- For a one-way demo, no Core change. The host recomputes thrust with public functions: `rotor_hover_induced_velocity`, `rotor_inflow_factor`, `rotor_ground_factor` (`rotor.hpp:614-636`) and `spade::eval` on the world SDF.
- The static part `k_T ω²` is exact. The inflow and ground factors come from the end-of-step pose, one substep after the one the step used. Near hover both factors are close to 1, so the error is small. It is display-grade, not the applied value.
- The world SDF is private in `Simulation` (`simulation.hpp:1609-1610`). The harness already holds it: it built the `WorldDesc`, or `ComposedScene::world`. No getter is needed.
- The exact value needs the per-rotor output the spec already plans (§1.8: `rotor.forces` stores T and Q to unregistered scratch). Today scratch has no host read (stage 4 Task 7b ruling), and on Vulkan scratch is not read back. So the exact read is a Publish copy: stage 7 and plan C.

**Cost.**
- `body()`, `rotor()`, `vehicle_state()` and `sample_medium()` are O(1). Each returns a pointer or a small value. None allocates, except to build an error message.
- CPU: always current.
- Vulkan: `step(n)` already reads the whole walk back into the arenas (`simulation.cpp:1139`). The getters then read host memory. No extra readback call is needed.
- Two Vulkan caveats. `set_rotor_commands` marks the mirror dirty (`simulation.cpp:2294`), so a host control loop re-uploads the whole walk each step. That is small for one drone. And a `Simulation` with behaviors attached refuses Vulkan (`simulation.cpp:1116-1121`, `CORE-1`). The drone box's test stand is behaviors, so it cannot run there; a free-flying drone under a host loop can (spec §6).
- For one drone, the `Simulation` can simply run on the CPU. The GPU then runs only the solver.

### (b) The drone's geometry as a solid boundary

**Gap. A vehicle has no shape the solver can use.**
- `ModelType` has `proxy_radius`, a sphere for contacts (`model_type.hpp:233`), and `visual_ref`, a mesh name "NOT read by the engine" (`model_type.hpp:212`).
- Static scene assets do carry an `SdfProgram` collider in the asset's frame (`scene_file.hpp:66-79`). `compose()` joins it into the static world SDF. Vehicles have no counterpart (`SceneVehicle`, `scene_file.hpp:83-87`).
- The airframe compile's part shapes (box, cylinder) feed the inertia only (`airframe_compile.hpp:83-90`). They do not survive into the `ModelType`.
- The sandbox drone is drawn from `QuadrotorParams`: a box body, four box arms, four cylinder discs (`drone_view.hpp:164-201`, `append_drone_items` at `:170`). Each maps to one SDF primitive.
- An `SdfProgram` is static. Its transforms are stored pre-inverted, world to local (`sdf.hpp:123-134`). It has no per-body pose. A posed query is easy, though: `eval(program, conj(q) * (p - origin))`.

**The smallest fix.**
- For the demo: none in Core. The harness builds the drone's SDF in its design frame from the same parameters it draws from. It evaluates it at the pose from `vehicle_state()`.
- For Kat, the editor and scene files: a `ModelType::collider`, an `SdfProgram` in the design frame, like `SceneAsset::collider`.
  - Public: the installed `model_type.hpp`, so one `consumers.md` line.
  - While stepping never reads it, it stays out of `model_identity`, as `visual_ref` does (`model_identity.cpp:50-80`). Every pinned identity holds.
  - Core's note: the day a step reads it (stage 7's in-Simulation solver using it as a boundary), it must be folded into `model_identity`, under Task 7's rule for table builders. Fold it only when it is non-empty, so every existing identity and stored blob still holds.
  - The scene file writes it only when it is non-empty, so the `gate_run` golden holds.
  - No per-step cost. The model/asset YAML plan, already in Core's interleave, is its natural home.

**What a porous drone means for "surface pressure".**
- The real-time tier treats vehicles as porous: momentum sources only, never solids (spec §2, "Moving bodies"; Q4 recommendation C). A solid moving body is placed in the LBM tier, later.
- With a porous drone, Rendering's surface colouring (vis plan §5) shows the air's pressure at the drone's pixels. That is not a wall pressure. The air flows through the drone.
- True surface pressure needs the drone as a moving solid in the grid: apertures from the posed body SDF each fluid step, and solid faces at `v + ω × r`. One-way is enough: the drone blocks the air, the pressure is integrated for display, and nothing pushes back. No one owns this for the real-time tier (§4 below).

### (c) Body pose and velocity

**Exists.**
- `body()`: `pos` (centre of mass, world), `orient`, `vel` (world), `omega_body` (body frame) (`simulation.hpp:1386`; `layout.hpp:113-124`).
- `vehicle_state()`: the same in the design frame (`simulation.hpp:1395`; `simulation.cpp:2711-2718`). Use it with a design-frame collider.
- A wall point's velocity is `vel + orient * cross(omega_body, r_b)`, with `r_b = conj(orient) * (p - pos)`.
- Both backends, same costs as (a). Nothing is missing for a one-way moving boundary.

### The solver-to-renderer side

**Exists.** `core/flow_volume.hpp` and `core/fp16.hpp` are on master. The volume is a Publish output, outside the walk, digest and snapshot (`flow_volume.hpp:9-12`). Plan A Task 8b fills it on the CPU. A GPU writer is "plan C's" (`flow_volume.hpp:23-26`).

## 2. One-way versus two-way

**The goal needs one-way first.**
- The analyses are the validation cases C0–C8 (spec §11.2). They run in the standalone solver. Plan A's V1 includes nothing from `sim/` (plan A, line 11).
- The picture is the drone driving the air: rotor sources, the wake, its rotation, ground outwash, the vortex structures. The drone's motion does not need the air's feedback to look or be right at hover and slow flight.
- Cameron already chose this order: one-way first, then probe-and-subtract (`PHY-9`; spec §8 Q3, A).
- Two-way adds fidelity, not new pictures: wake ingestion, grid-resolved ground effect, drag pushback. It is M4, and it needs the module inside the step, so stage 7.

**What Physics' plans assume.**
- The spec's M3, the first sandbox scene, is the airflow module inside `Simulation` (plan C). Plan C "waits on Core's stage 7" (plan A, line 20).
- M3 also has "drag reads local air" (§9). That is the air acting on the drone, through point samples: stage 7 again.
- So the spec's path to the demo runs through stage 7, even for one-way rotor sources.
- A host-driven one-way path is not in the spec or plan A: `Simulation` on the host, the solver in Physics' chain runner, host reads each frame, a `FlowVolume` to the renderer. It is the path that frees the demo from stages 5–7.
- One condition to settle. The chain runner was approved as a test and spike harness only: not installed, no `Simulation` integration (`tasks/todo-physics.md`, the 2026-10-05 decision). A sandbox demo may link it; that is not "into `Simulation`". But if the demonstration must be shipped engine API, it is plan C, after stage 7. The lead or Cameron should say which it is.

## 3. The order of stages 5–7

**Recommendation: keep the code order 5 → 6 → 7. Send stage 7's design to Cameron now.**

- **No stage is on the one-way demo's critical path.** The runner is standalone. The host reads exist. The `Simulation` can run on the CPU.
- **Stage 6 (the lock) is not on it at all.** It moves the drone box's behavior test stand to Vulkan. The demo's drone flies free under a host loop (spec §6). Stage 6 matters to airflow only through its fixed-binding pattern, which the module (stage 7) reuses.
- **Stage 5** is in progress (Task 1 and its poison follow-up, WIP `9bd4c66`). Physics' `contact_dv` follow-up waits on Task 1. Stopping it now gains nothing.
- **Reordering 7 before 6 buys little.** By Core's estimates (`core/2026-10-05-airflow-spec-core-answers.md:181-191`):
  - order A, 4 → 5 → 6 → 7: stages 5 and 6 are about 10–12 tasks, 1.5–2 working days; stage 7 about 8 tasks, 1–1.5 days; Core's path about 2.5–3.5 days;
  - order B, 7 first: about 1.5–2 days, 1–1.5 days sooner; stage 6 then reworks stage 7's binding pattern (about half a day), and airflow on Vulkan has no grade until stage 5;
  - the net saving is about 0.5–1 day, and only on the two-way path.
- **The solver is the longer pole.** Physics estimated 3–5 days (spec Q11). Plan A has Tasks 1–5 merged and Task 6 on a branch, of 12. Plan B has not started; its chain-runner design note is still owed to Core.
- **Calendar time has run past the estimates.** Stage 5 Task 1 began 2026-10-06 and is not merged on 2026-10-12, with pauses included. Treat the day counts as effort, not dates.
- **Stage 7's design can go to Cameron now.** Section 15 requires his sign-off before any code. The approved interleave placed the design beside stage 6 code. Moving it to now costs no code and conflicts with nothing. Write it together with the chain-runner review, so the runner's buffers take stage 7's residency shape (guards G1 and G2). Signed early, stage 7 code can start the moment stage 6 merges.
- **One adjustment to consider.** The interleave puts model/asset YAML code before stage 7 code. If the two-way module matters sooner, stage 7 code could follow stage 6 directly. No estimate for the YAML code is on record. If Cameron wants a solid drone, the model collider belongs in that YAML work.

## 4. Blockers that nobody owns yet

1. **The demo harness itself.** Something must step the `Simulation`, read the rotors, feed the GPU runner, publish the volume and render it, live and recorded. Spec §6 gives Interface the `airflow` scene, but on the module (M3, after stage 7). The host-driven one-way harness has no owner.
2. **Surface pressure on a real surface.** It needs the drone as a moving solid in the real-time tier, one-way. The spec puts solids in the LBM tier, later (§2, Q4). Plan A defers even static solids to V2. No vehicle carries a collider (1b). No one owns it.
3. **Swirl in the actuator source.** Spec §3.1 spreads the shaft torque as swirl. Plan A Task 9's `ActuatorDisc` carries thrust only, with no torque or spin direction. Without it the wake does not rotate, which is much of "propeller flow". Physics' plan, but not in any task.
4. **The volume on the GPU.** `flow_volume.hpp` names plan C as the GPU writer. A runner-based demo needs that pack (f16 on the device, or a full fp32 readback and a CPU pack) before plan C exists. Neither plan B nor Rendering's plan holds it.
5. **What "vortices" will mean.** An actuator disc has no blades, so it makes no blade tip vortices. The demo will show the slipstream's shear layer, the vortex ring and the ground outwash. Blade-resolved tip vortices need moving solids in the LBM tier, after C8 (spec §11.2). Physics should say this to Cameron before the demo, not after.
6. **Owned, but on the critical path:** Physics' chain-runner design note. Core must approve it before any runner code. It was due around plan A Task 5, which has merged.
