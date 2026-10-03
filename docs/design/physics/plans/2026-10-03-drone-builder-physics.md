# Drone builder — physics

**Owner:** Physics. **Status:** draft for the lead's review, 2026-10-03. It holds Spade's physics sections of the joint drone-builder spec with Kat, which the user confirmed on 2026-10-03.
**Source:** the lead's brief of 2026-10-03. The joint spec's own text is not on this machine, so where this draft needs a fact from it, it names the gap (§13).
**Depends on:** Core's module API (`../../core/plans/2026-10-02-module-api-design.md`). Stateful rows need its stage 4. Grade declarations need its stage 5. The propeller tier is per-rotor data, not a role (§13).

After reading this page, a reader can build the four models a drone builder needs from Spade, say which module-API stage each waits on, and check each one against a real airframe.

## 1. What the builder needs from Physics

A drone builder assembles an airframe from parts: frame, arms, motors, ESCs, props, a battery and avionics. Physics supplies four things:

1. **A motor and ESC model.** It turns a throttle command into shaft speed, current and torque.
2. **A battery model.** It supplies the bus voltage, sags under load and runs down.
3. **A propeller tier.** It turns shaft speed and inflow into thrust and torque from measured coefficients.
4. **A composite-inertia utility.** It turns the parts' masses and poses into one rigid body.

A steady-state solver joins the first three for one motor, propeller and battery. The builder uses it to show hover throttle, current and thrust margin without stepping a simulation.

Four rulings from the owners shape every model:
- Every part has a definite mount pose.
- A motor takes a command input. Nothing in the chain has gains to tune.
- No part is ever damaged. A limit clamps the quantity and publishes an over-limit flag.
- Each model exposes the pure functions the step calls, so a test can check each one alone.

## 2. Where each piece lives, and when it can be built

| Piece | Form | Waits on | CPU grade | Vulkan grade |
|---|---|---|---|---|
| Pure functions of all four models | Header functions (`vehicles/`) | Nothing | unit-tested | mirrored by the kernels |
| Composite-inertia utility | Host function, not in the step | Nothing | reference, by closed forms | not applicable |
| Steady-state solver | Host function, not in the step | Nothing | reference, by closed forms | not applicable |
| Motor and ESC | A row per rotor, body-attached; runs in the rotor pass | Stage 4 | reference, with a golden | banded |
| Battery | A row per vehicle, body-attached; runs in the rotor pass | Stage 4 | reference, with a golden | banded |
| Propeller tier (coefficient tables) | A variant of the rotor module, chosen per rotor beside momentum theory | Stage 4 (its per-rotor data) | reference, with a golden | banded |

The first three rows need no module API. They can be built and tested now, which de-risks the stateful work.

## 3. Motor and ESC

**Model.** An average-value brushless DC motor behind an ESC. The ESC applies a duty `d` in [0, 1] to the bus voltage. Electrical inductance is neglected: its time constant is microseconds, far below a substep.

- Speed constant: `Kv` (rad/s per V) = `KV` (rpm/V) × π/30. Torque constant: `Kt = 1/Kv` (N·m/A). Back-EMF constant `Ke` equals `Kt` in SI.
- Resistance: `R` = motor winding resistance + ESC on-resistance (Ω).
- Current: `I = (d·V_bus − Ke·ω) / R` (A). The battery sees `d·I` (lossless switching).
- Motor torque: `Q_m = Kt·(I − I0)` (N·m). `I0` is the no-load current. It stands in for friction and iron loss, and it never turns the shaft backwards.
- Shaft: `J_r·dω/dt = Q_m − Q_load(ω)`. `J_r` is the polar inertia of the motor bell plus the propeller. `Q_load` comes from the propeller tier (§5), or is `k_Q·ω²` with today's momentum-theory tier.

**Phase inductance (recommended by §11's fit).** At speed, the winding's reactance `X = p·ω·L` stops being small next to `R`. `p` is the pole-pair count and `L` the phase inductance. With the drive voltage in phase with the back-EMF (no timing advance), only the in-phase part of the current makes torque:
- `I_q = (d·V_bus − Ke·ω)·R / (R² + X²)`.

So the model is the resistive one with `R` replaced by `R_eff(ω) = R + X²/R`. That keeps the bus solve closed-form, because `ω` is held at its start-of-substep value. It also makes `τ_m` speed-dependent, so `α` is computed per substep with `core/fp32_math`'s `exp`, as the rotor lag does today. Timing advance, which real ESCs use, reduces the effect. It is a part property with a published range, not a fit freedom.

**Limits.**
- Current clamps to `[I_lo, I_max]`. `I_max` is the smaller of the motor's and the ESC's rating. `I_lo` is 0 for an ESC without active braking and `−I_max` with it.
- The duty clamps to [0, 1]. A non-finite command reads as 0.
- Shaft speed never goes below 0. The ESC is one-directional.
- Each clamp sets a flag in the motor row.

**Integration.** The electrical part is linear in ω with time constant `τ_m = J_r·R·Kv²`. The load torque is held at its value at the start of the substep. The update is then exact for the linear part:
- `ω_∞ = Kv·(d·V_bus − R·(I0 + Q_load/Kt))`, the speed the motor would settle at under that load;
- `ω ← ω_∞ + (ω − ω_∞)·α`, with `α = exp(−h/τ_m)`.

`α` is the rotor lag's own factor, computed with `core/fp32_math`'s `exp32` (`TD-3`), the GNSS coefficients' pattern. It is computed once at spawn when `R_eff` is constant, and per substep with the inductance term. *Corrected 2026-10-03: this paragraph first said "in double", which would put libm's `exp` on a path that feeds state.*

When the current is at a limit, the motor torque is constant. The update is then `ω ← ω + h·(Q_m − Q_load)/J_r`.

**Reaction on the airframe.** The body receives the motor's torque, `−spin·Q_m` about the thrust axis. Today's rotor applies the aerodynamic torque `k_Q·ω²` instead. The two agree in steady state. They differ during spin-up, which is exactly the yaw reaction a builder needs to see.

**What it replaces.** A rotor with a motor row ignores `omega_cmd` and its first-order lag (`vehicles/rotor.hpp` §5). A rotor without one behaves exactly as today.

## 4. Battery

**Model.** A Thevenin pack: an open-circuit source, a series resistance and one RC branch for slow sag.

- Pack: `S` cells in series and `P` in parallel. Capacity is `P × C_cell` (Ah). The series resistance `R0` is `S·R_cell/P` (Ω).
- Open-circuit voltage: `V_oc = S × OCV(SoC)`. `OCV` is a per-cell table on a uniform state-of-charge grid. Authored points are resampled onto that grid at spawn, in double, and rounded once.
- Sag: an instant drop `R0·I_b`, plus a polarization voltage `V_1` in an RC branch (`R1`, `C1`). `V_1` relaxes with `β = exp(−h/(R1·C1))`, computed once at spawn with `exp32`.
- Terminal voltage: `V_t = V_oc − V_1 − R0·I_b`.
- Charge: `SoC ← SoC − I_b·h/(3600·capacity)`, exact for a current held over the substep.

**The bus solve.** All motors on one vehicle share one bus. For the motors not at a current limit, the bus voltage has a closed form:

`V_bus = (V_oc − V_1 + R0·Σ dᵢ·Keᵢ·ωᵢ/Rᵢ − R0·Σ_fixed dⱼ·Iⱼ) / (1 + R0·Σ dᵢ²/Rᵢ)`

Index `i` runs over the motors not at a limit, and `j` over the motors held at one. The sums run in rotor declaration order. A motor whose current reaches a limit moves to the fixed sum, and the solve repeats. It repeats at most once per motor.

**Limits.**
- Battery current clamps at its rating (C-rate × capacity).
- Terminal voltage never drops below the cutoff (`S` × cell cutoff voltage).
- Both clamps act as the ESC's protection does on a real drone: one common duty scale for the whole vehicle. The plan fixes how that scale is solved. It must be closed-form or have a fixed iteration count, so both backends run the same operations.
- State of charge clamps at 0. The source then holds `OCV(0)`.
- Each clamp sets a flag in the battery row.

## 5. Propeller tier

**Model.** Thrust and torque from measured coefficients, by advance ratio.

- Advance ratio: `J = V_ax/(n·D)`, where `n = ω/2π` (rev/s) and `V_ax` is the axial inflow speed today's rotor pass already computes.
- Thrust: `T = C_T(J)·ρ·n²·D⁴`. Torque: `Q = C_Q(J)·ρ·n²·D⁵`. With `n = 0`, both are 0.
- `C_T` and `C_Q` are tables on a uniform `J` grid, linearly interpolated. A uniform grid makes the lookup index arithmetic, which keeps it identical on both backends.
- Ground effect multiplies thrust as today (Cheeseman–Bennett, `vehicles/rotor.hpp` §4).
- `D` sets the scale. Pitch and blade count identify the propeller. They feed a default table only when no measured table exists.

**Range.** Outside the table's `J` range, the end value holds and the rotor row sets an out-of-table flag. That includes descent (`J < 0`), where most measured tables stop. Today's momentum-theory tier keeps the vortex-ring and windmill-brake branches. The coefficient tier does not model them.

**What it fixes.** Torque now follows inflow. Today's `Q = k_Q·ω²` ignores it (`07-status.md`, debt).

## 6. Composite inertia

**Inputs.** Each part has:
- a mass, which is positive and finite;
- a mount pose (position and orientation) in the airframe's design frame;
- either a primitive shape or an explicit inertia tensor about its own centre of mass, in its own frame.

The primitives are a point mass, a solid sphere, a solid box, a solid cylinder and a thin-walled tube along local +Y (an arm).

**Method.** All sums run in double, in part declaration order:
- total mass `M = Σ mᵢ`;
- centre of mass `c = Σ mᵢ·pᵢ / M`;
- inertia about `c`: `I = Σ (Rᵢ·Iᵢ·Rᵢᵀ + mᵢ·(|dᵢ|²·E − dᵢ·dᵢᵀ))`, with `dᵢ = pᵢ − c` and `E` the identity.

The engine stores a diagonal body-frame inertia. So the utility diagonalizes `I` with a cyclic Jacobi method in double. That method makes no rotation for a matrix that is already diagonal, so a symmetric airframe keeps its design axes exactly. Each principal axis takes the label of the design axis it is nearest to, signed to keep a right-handed frame.

**Outputs.** The total mass, the centre of mass, the principal moments and the design-to-body rotation. Each is rounded to fp32 once, at the end. The template expresses every mount pose in that body frame.

**The rotation stays internal (Kat, 2026-10-03; replaces 92e3b71).** A flight package sees only the design frame, which is its flight controller's frame. The body frame is the engine's own. Three places carry the rotation, and nothing outside them sees it:

| Where | What it does | Owner |
|---|---|---|
| The compile (model type) | Expresses every mount in the body frame: rotors, drag, IMU, GNSS. It stores the design-to-body rotation and the centre-of-mass offset on the model type | Physics (`engine D4` layer) |
| `Simulation::spawn(world, model, VehicleSpawn)` | Takes the pose, velocity and rates of the design origin in the design frame, and converts them to the body's | Core |
| A vehicle-state read (new, for example `vehicle_state(VehicleRef)`) | Returns the design origin's position, orientation, velocity and rates, in the design frame | Core |

The conversions are rigid-body identities. `q_bd` (matrix `R_bd`) maps design-frame vectors to body-frame vectors. `c` is the centre of mass, measured from the design origin in the design frame.
- orientation: `q_wd = q_wb ⊗ q_bd`;
- position: `p_d = p_com − R_wd·c`;
- velocity: `v_d = v_com + ω_w × (p_d − p_com)`, with `ω_w` the rates in the world frame;
- rates: `ω_d = R_bdᵀ·ω_b`.

Spawn applies the inverse of each.

Some outputs need no conversion:
- `set_rotor_commands` and the duty call address rotors by declaration index, which has no frame;
- the IMU reports in its mount frame, whose pose the compile already re-expressed;
- the GNSS reports a world-frame position.

`body(BodyRef)` and `world_bodies()` stay raw body-frame reads, documented as the engine's frame. A flight package must not use them for a vehicle.

**Validation.** A tensor must be symmetric and positive semidefinite, and it must satisfy the triangle inequality (`I₁ + I₂ ≥ I₃`). The utility refuses a part that fails, and it checks the composite the same way.

## 7. Steady-state solver

**Purpose.** It solves one motor, propeller and battery chain at equilibrium, with `k` identical motors sharing the battery. It is a host function in double and is never in the step.

**Forward mode.** Given duty, air density, axial speed and battery state, it finds the `ω` where `Q_m(ω) = Q_load(ω)`. Motor torque falls with `ω`. For measured propellers the load rises with `ω`, so the root is unique. Bisection on `[0, Kv·d·V_oc]` finds it to a fixed tolerance, in a fixed iteration order. The solver checks that the bracket changes sign, and it flags a table that breaks the rising-load assumption. It returns `ω`, thrust, torque, current, bus voltage, electrical and shaft power, efficiency and the limit flags.

**Inverse mode.** Given a target thrust per motor (hover is weight/`k`), it finds the duty by bisection on [0, 1]. A target above full-duty thrust returns full duty with an unreachable flag.

**Its check against the step.** The stepped chain, held at constant duty, settles to the solver's `ω` within a bound the tests derive from the integration method.

## 8. Operation order

Op order is the parity contract (`engine D2`). The rotor pass, in Forces, runs these steps per vehicle, rotors in declaration order:

1. Mount geometry (today's step 1).
2. Axial inflow `V_ax` (today's step 4, moved earlier, because the load torque needs it).
3. Load torque `Q_load(ω_n)` from the propeller tier.
4. Duty: clamp, and set the flag.
5. Bus solve (§4), including the current and battery clamps.
6. Each motor's current and torque `Q_m`.
7. Shaft update to `ω_{n+1}` (§3).
8. Thrust from the tier at `ω_{n+1}`, times the ground factor (today's steps 3, 5 and 6).
9. Wrench: thrust along the axis, `cross(r, T·axis)`, and the reaction `−spin·Q_m` (today's step 7).
10. Battery state: `SoC`, `V_1`, and the published `V_t` and `I_b`.

A vehicle with no motor rows runs today's order unchanged, so no golden moves.

**Division sites.** These are listed because Vulkan's division is device-dependent:
- `1/Kt` and `β` are precomputed at spawn, and so is `α` when `R_eff` is constant;
- the step keeps one division per vehicle (the bus solve) and two per rotor (`J`, and the table position);
- with phase inductance, it adds one per motor (`1/R_eff`).

Both are banded on Vulkan. Transcendentals on the step path come only from `core/fp32_math`.

## 9. Requirements

IDs are `DBP-nn`. "Builder parameters" means the values a part's datasheet or the builder supplies.

**General**
- DBP-01: Each model MUST expose its step arithmetic as pure, total, `noexcept` functions in `vehicles/`.
- DBP-02: Each such function MUST return a defined, finite value for zero, negative, non-finite and overflowing inputs, and a test MUST pin each case.
- DBP-03: Every part MUST have a definite mount pose in the design frame.
- DBP-04: A model MUST NOT represent damage to a part.
- DBP-05: Each limit MUST clamp its quantity and MUST set a named flag in registered state.
- DBP-06: The flags and the electrical quantities (current, bus voltage, terminal voltage, state of charge) MUST be readable through the host API and copied at Publish.
- DBP-07: A model MUST NOT require a gain, a tuning constant or a controller to run.
- DBP-08: A vehicle without the new rows MUST step exactly as today, and every existing golden MUST stay unchanged.

**Motor and ESC**
- DBP-10: The motor MUST take a per-rotor duty command in [0, 1].
- DBP-11: The motor MUST take `KV`, winding resistance, no-load current, current limit and polar inertia as builder parameters.
- DBP-12: The ESC MUST take its on-resistance, its current rating and its braking mode as builder parameters.
- DBP-13: The shaft update MUST be the exponential update of §3, with `α` computed once at spawn for a constant impedance and per substep, with `core/fp32_math`, for a speed-dependent one.
- DBP-14: The body MUST receive the motor's torque as the rotor's reaction.
- DBP-15: The shaft speed MUST NOT go below 0.
- DBP-16: The model SHOULD estimate the propeller's share of the polar inertia from its mass and diameter when the builder gives none.
- DBP-17: The model MAY take a maximum shaft speed, clamped and flagged like the current.
- DBP-18: The motor SHOULD model phase inductance through `R_eff(ω)` (§3), with pole pairs and phase inductance as builder parameters.
- DBP-19: An ESC throttle map MAY be added only as one parameter with the firmware's published range, never as a free table.

**Battery**
- DBP-20: The battery MUST take series count, parallel count, cell capacity, cell resistance, cell cutoff voltage, current rating and an OCV table as builder parameters.
- DBP-21: The bus voltage MUST come from the closed-form solve of §4, with sums in rotor declaration order.
- DBP-22: The battery current MUST NOT exceed its rating, and the terminal voltage MUST NOT fall below cutoff.
- DBP-23: Both battery clamps MUST act through one common duty scale per vehicle.
- DBP-24: The battery SHOULD model polarization with one RC branch, with `β` computed at spawn.
- DBP-25: Temperature effects MAY be added when the medium carries temperature (`engine D6`).
- DBP-26: A 4-in-1 ESC's total current MUST clamp through the same common duty scale as DBP-23.
- DBP-27: Part blocks MUST use the field names and units of §14.2.

**Propeller tier**
- DBP-30: The tier MUST compute thrust and torque from `C_T(J)` and `C_Q(J)` tables.
- DBP-31: Tables MUST be resampled at spawn onto a uniform `J` grid, in double, and rounded once.
- DBP-32: Outside the table's range, the end value MUST hold and an out-of-table flag MUST be set.
- DBP-33: Thrust MUST keep today's ground-effect factor.
- DBP-34: The tier MAY generate a default table from diameter, pitch and blade count, graded best-effort.

**Composite inertia**
- DBP-40: The utility MUST accept the primitives of §6 and explicit tensors.
- DBP-41: All sums MUST run in double, in part declaration order, with one fp32 rounding per output.
- DBP-42: The utility MUST refuse a part with non-positive mass, a non-finite value or an invalid tensor.
- DBP-43: The utility MUST return principal moments and the design-to-body rotation, with the axis labelling of §6.
- DBP-44: The compile MUST express every mount in the body frame.
- DBP-45: The design-to-body rotation and the centre-of-mass offset MUST NOT appear outside the model type, spawn and the vehicle-state read.
- DBP-46: Spawn MUST take a vehicle's pose, velocity and rates in the design frame.
- DBP-47: The vehicle-state read MUST return pose, velocity and rates in the design frame.

**Steady-state solver**
- DBP-50: The solver MUST provide the forward and inverse modes of §7.
- DBP-51: The solver MUST be deterministic and total, and it MUST report an infeasible target with a flag.
- DBP-52: A test MUST show the stepped chain settling to the solver's answer within a derived bound.

**Grades**
- DBP-60: Each stepped model MUST be reference grade on the CPU, with a corpus golden.
- DBP-61: Each stepped model MUST be banded on Vulkan, with a measured band (`TD-2`), or it MUST declare Vulkan `absent`.
- DBP-62: The noise-free models MUST NOT cite `CORE-3`; their bands MUST be measured on their own.

**Validation (§11)**
- DBP-70: The fit MUST keep each parameter within its catalog part's published range.
- DBP-71: A parameter that ends at a range limit MUST be reported as a finding.
- DBP-72: A published range MUST NOT be widened to improve a fit.
- DBP-73: The fit MUST be deterministic: double precision, a fixed start, a fixed iteration cap and a fixed parameter order.
- DBP-74: Validation data MUST NOT enter the fit.
- DBP-75: The band MUST be `k·u_c` with `k = 2`.
- DBP-76: `k` and the uncertainties MUST NOT be raised to make a point pass.
- DBP-77: The run MUST record the items of §11.6, step 10.
- DBP-78: The held-out split MUST be fixed and recorded before the first fit runs.
- DBP-79: The fit MUST NOT pool files made with different thrust tables.
- DBP-80: The fit MUST fit only identifiable combinations, and every other parameter MUST take its catalog value and be reported as not tested.
- DBP-81: A command latency MUST enter the comparison as a delay that Kat states, and MUST NOT be fitted into the motor.

## 10. Grades and goldens

| Model | CPU | Vulkan | Evidence |
|---|---|---|---|
| Motor and ESC | reference | banded | a drone-builder golden: a quad on duty commands, spin-up and a current-limited step |
| Battery | reference | banded | the same golden, run long enough to show sag and a cutoff clamp |
| Propeller tier | reference | banded | the same golden, flown with the coefficient tier |
| Composite inertia | reference by closed forms | not in the step | box, cylinder and tube against textbook moments; parallel-axis cases; a rotated part |
| Steady-state solver | reference by closed forms | not in the step | the no-load speed `Kv·V`, the stall current `V/R`, and the stepped chain's settled `ω` |

A golden here is a determinism record (`L4`). It says nothing about whether the model matches a real drone. §11 covers that.

## 11. Checking against a research airframe

The check is a validation suite, separate from the goldens. It runs on `competition-700` first, then `standard_x250` (Kat, 2026-10-03).

**What it validates against.** Every number here comes from the AI Grand Prix VQ1 simulator, a black box, not from hardware. The check therefore shows agreement with that simulator. It says nothing about a physical drone.

### 11.1 Who owns what

The user ruled on 2026-10-03: "Spade pushes, Kat pulls."
- **Kat owns the fit and the validation run.** Both run on the Kat machine, against Spade's published pure functions (DBP-01). The flight logs never leave that machine.
- **Physics owns the models**, the answer on the ESC map (§11.4) and the judgment of the tolerances (§11.7).
- §11.6 states Physics's requirements on the fit. Kat applies them.
- The pure functions are built after the user approves the joint spec, before module-API stage 4.

### 11.2 The data

Kat has system data only: the simulator's `airframe.json`, and flight logs from the same simulator. For the first airframe:

| Quantity | Value | Provenance | Use |
|---|---|---|---|
| Thrust curve | 12 points | the simulator | fit target |
| Airspeed loss | `f(s) = clamp(1 − k·s^p, 0.35, 1)`, `k = 0.00028`, `p = 2`, `s` = world speed | legacy `quadrotor.py:137-143`; applied to static thrust, at fixed throttle | held-out check, axial flight only (§11.7) |
| Drag | 0.028 N·s²/m², per body axis, `F_i = −c·|v_i|·v_i` | legacy `quadrotor.py:60-64`, `:179` | Spade's componentwise drag element (`physics/forces.hpp`, kg/m, the same unit); set, not fitted |
| Mass | 0.7 kg | `airframe.json` | from the parts (Kat's decision); compared, not fitted |
| Inertia | [0.0025, 0.0021, 0.0043] kg·m² | NeuroBEM, not measured | from the parts; reported beside the composite's |
| Motor time constant | 0.033 s | NeuroBEM, not measured | compared with the measured step response (§11.7) |
| `kappa` | 0.022 m, `k_Q/k_T` | the simulator; about twice the catalog's 9–12 mm | from the parts; the yaw mismatch is a known finding |

There is no component data: no `KV`, `R`, `I0`, `L`, cell data, `C_T` or `C_Q`. The legacy yaml labels `kappa` dimensionless; that label is wrong.

The motor time constant and `kappa` are exactly the parameters of today's momentum-theory rotor: `tau` and `k_Q/k_T` (`vehicles/rotor.hpp`). So that rotor, fed these values directly, is a baseline in the black box's own model class.

**Commands.** The fit uses `cmd_norm`, the time-aligned collective command in the deploy probe logs. Per-axis rate commands are not confirmed.

**Two thrust tables.** Six of the eight probe files were made with an older thrust table. Only the `inplane_sweep` files match today's. A fit must never pool files from two tables, because that fits two black boxes as one. Each file is used only against the table that produced it, and the run records which.

**The split (Kat's, fixed by whole files).**
- FIT, build A: `thrust_staircase`, `force_direction`, `axial_thrust`, `inplane_sweep`, `drag_speed`, `drag_lateral`, `drag_aoa`, `high_tilt_aero`, `axis_sweep`, `axis_sign_probe`.
- HOLDOUT-A: `thrust_step`, `axial_thrust2`, `inplane_sweep2`, `axis_roll_probe`, and the 14 flights from 07-30 after 02:08.
- HOLDOUT-B (August): drift only. It is reported, never asserted.
- 11 flights are excluded, each with its reason in the data pack.

### 11.3 Kat's fit so far, and what it shows

- With datasheet `R`, no catalog set fits: the RMS error is 3.6–42 m/s².
- With `R_eff` at 3–7 times `R` and an rpm cap, RMS reaches 0.4–0.9 m/s². The 0.264–0.55 throttle band still errs by 2.8–5.1%.
- The best set is a 6-inch tri-blade prop, a 2207 2450 KV motor and 4S, with `R_eff` about 3 times `R` and a cap near 18.3 krpm.

A constant `R_eff` and a hard cap are fit freedoms with no part behind them. They are also the signature of phase inductance (§3), which no catalog part yet carries:
- `R_eff(ω) = R + (p·ω·L)²/R` grows with the square of speed. A constant multiplier then gives too much droop at low throttle and too little at high, which is where the 2.8–5.1% error sits.
- It flattens thrust at high speed, which is the soft form of an rpm cap.
- With assumed but typical 2207 values (`R` 50–70 mΩ, `L` 10–20 µH, 7 pole pairs), `R_eff/R` is about 2–8 at 9 krpm and 5–30 at 18.3 krpm. Kat's fitted 3–7 sits inside that range. These inputs are assumptions until the catalog gives `L`.

### 11.4 Kat's question: an ESC map or an inductance term?

**Answer: the inductance term first. Add an ESC map only if that fails, and then as one bounded parameter, never a free table.**
1. Inductance is a part property with a datasheet value and a range. It explains both freedoms Kat had to add, so it should replace them rather than join them.
2. It keeps the solve closed-form: `R` becomes `R_eff(ω)`, and no new state is added (§3).
3. A free throttle-to-output map can match any thrust curve. Fitting one turns the 12-point curve into a copy of itself, so the curve stops being evidence.
4. A real ESC or flight controller does apply a throttle map: thrust linearization. If one is needed, it is a one-parameter curve with the firmware's published range, set from the part's configuration, not fitted freely.
5. The test that settles it: Kat refits with `L` in its published range, datasheet `R`, no `R_eff` multiplier and no cap. If that meets the tolerances of §11.7, the question is closed.

A 1% fit is not required. Kat's own proposed tolerances are 3% and 5%.

### 11.5 What the data cannot identify

Kat's list is right. Each group below enters the data only as a product or a sum, so the data fixes the combination and not its parts:
- `k_T` and `ω`, because no log has rpm;
- mass and `k_T`, because thrust enters as acceleration;
- `KV` and `V_bus`;
- motor `R` and battery `R`, with collective commands only;
- `J_r` and `R_eff`, which enter only through the time constant;
- `k_Q`, inertia, sag and blade count, which no fitted file excites.

Physics adds `L` and `k_T`: without rpm, the scale of `ω`, and so of `p·ω·L`, rides on `k_T`.

The rule that follows: fit only identifiable combinations. Every other parameter takes its catalog value and is reported as "not tested by this data". It is also left out of the covariance, which is otherwise singular in those directions.

### 11.6 Physics's requirements on the fit

Kat runs these steps. DBP-70 to DBP-78 state them as requirements.
1. **Split.** Fix the split before the first fit, and record it.
2. **Parts.** Choose parts from the catalog (Kat's first 5-inch catalog, with sources). Each parameter carries its published range.
3. **Fit.** Fit the identifiable combinations, within their ranges, to the FIT files of one thrust table. The fit is weighted least squares in double, with each residual divided by its point's uncertainty. A bounded Levenberg–Marquardt solve starts from the catalog's nominal values, with a fixed iteration cap and a fixed parameter order, so it is deterministic.
4. **Range limits.** A parameter that ends at a limit of its published range is a finding. Report it; do not widen the range.
5. **Uncertainty.** `u_meas` comes from the data's own scatter, measured on segments chosen before any residual is seen. For `airframe.json` values it is half the last printed digit, unless Kat states more. `u_param` comes from the fit's covariance, `(Aᵀ·W·A)⁻¹` with `A` the fit's Jacobian, propagated by central differences in double. `u_c = sqrt(u_meas² + u_param²)`.
6. **Validation, translational.** The HOLDOUT-A files never enter the fit. For each sample, the model predicts acceleration from `cmd_norm`, the logged orientation and the logged velocity. The logged velocity, differentiated over a window fixed in advance, gives the comparison.
7. **Validation, rotational.** This waits until per-axis rate commands are confirmed in a log.
8. **Tolerance.** The band is `k·u_c`, with `k = 2` (about 95%).
9. **No widening.** A point outside its band is a model finding. Never raise `k` or an uncertainty to make a point pass (`TD-2`).
10. **Recording.** The run records:
    - the airframe, the data pack's version, each file's thrust table, and the split;
    - each catalog part with its source;
    - each fitted value with its range and covariance, and each pinned parameter;
    - `k` and every point's residual and `u_c`.

### 11.7 Kat's proposed tolerances, judged

A tolerance is accepted when it comes from the data's own scatter. One read off the best fit's residuals is circular and is rejected (`TD-4`). The thrust bands pass that test: the current best fit errs 2.8–5.1% in the 0.264–0.55 band, so a ±3% band there would fail it today.

| Proposed | Judgment |
|---|---|
| Thrust ±3% at throttle 0.264–0.55, ±5% at 0.65–1.0 | Accept, once each band is shown to be 2·u_c from scatter, not chosen |
| Hover throttle 0.264 ± 0.005 | Accept, if ±0.005 is the logs' hover scatter |
| t63 50 ± 15 ms | Centre on the measured median, 46 ms, and take the band from the 40–61 ms spread. It is a weak discriminator, about ±30% |
| `f(26.3)` = 0.80 ± 0.04, `f(33.3)` = 0.69 ± 0.07 | Accept, for axial flight only. The centres are the law's own values, 0.806 and 0.690. The model's loss comes from `C_T(J)`, the axial component alone, so it predicts no loss in edgewise flight. Edgewise points are a scope finding (§12), not a failure of the axial model |
| Step ratio 1.00 ± 0.08 | Accept, if from scatter |

**The time constant.** The simulator's own lag is 0.033 s (from NeuroBEM), but the measured t63 is about 46 ms. If part of that difference is command latency, the latency belongs to the simulator's link, not to the motor. It enters the comparison as a fixed command delay that Kat states. It is never fitted into the motor.

**The fixed-throttle loss.** At 18.3 m/s, Kat's 6-inch prop gives `C_T/C_T0` = 0.67 at fixed rpm, but the simulator gives `f` = 0.906. At fixed duty, the motor speeds up as the prop unloads, so the chain recovers much of the loss. The motor-and-prop chain predicts that recovery without being fitted to it, so it is a strong held-out check.

## 12. Not modelled

- Motor inductance, commutation, timing and ESC desync.
- Motor and battery heating, and temperature effects on resistance and capacity.
- Propeller in-plane flow: H-force, inflow skew and blade flapping.
- Vortex-ring and windmill-brake states in the coefficient tier (§5).
- Rotor gyroscopic coupling to body rates. The polar inertia `J_r` exists now, so this is a short step later.
- Damage of any kind (DBP-04).

## 13. Open questions

1. **Tier selection. Decided by the lead (2026-10-03): per-rotor data.** Momentum theory and the coefficient table are two variants of one rotor module. A role is for a tier that changes passes or state, such as BEMT.
2. **The command API.** A duty command needs a host call beside `set_rotor_commands`. That is Core's and Interface's.
3. **The joint spec's facts. Answered by Kat (2026-10-03), and folded into §11.**
   - The airframes are `competition-700`, then `standard_x250`.
   - Drag is componentwise, in N·s²/m². `kappa` is `k_Q/k_T`, in metres.
   - The deploy probe logs carry a time-aligned collective command.
   - The source is the AI Grand Prix VQ1 simulator, not hardware.

   Kat's data pack summary then answered the airspeed-loss law, the command column (`cmd_norm`) and the split (§11.2).

   Still open:
   - the catalog's phase inductance and pole pairs for each motor, which the inductance refit needs (§11.4);
   - whether the simulator's link adds a command delay, and how long (§11.7);
   - whether any log carries per-axis rate commands, which rotational validation needs.
4. **Ownership of the fit. Decided by the user (2026-10-03): "Spade pushes, Kat pulls."** Kat runs the fit and the validation on the Kat machine (§11.1).

## 14. Answers to Kat's spec points

### 14.1 D-2: the ESC map or the inductance term

The answer is in §11.4: the inductance term first. An ESC map comes only if that fails, as one bounded parameter.

### 14.2 D-4: field names for the part blocks

Names are snake_case. Units are SI, except `kv`, which keeps the datasheet's rpm/V. Each part also carries `mass` (kg) and a mount pose, `mount_pos` (m) and `mount_orient` (unit quaternion, w x y z), in the design frame (DBP-03).

Today's names stay as they are: `thrust_coeff`, `torque_coeff`, `radius`, `tau` and `inertia_diag`. The momentum-theory variant and the lag still read them. A part block compiles into them where the table says so.

**`motor`**

| Field | Unit | Meaning |
|---|---|---|
| `kv` | rpm/V | speed constant, as the datasheet gives it; compiled to rad/(s·V) |
| `resistance` | Ω | winding resistance, line to line, as datasheets give it |
| `no_load_current` | A | no-load current |
| `no_load_voltage` | V | the voltage `no_load_current` was measured at |
| `current_max` | A | continuous current rating |
| `pole_pairs` | count | magnet poles divided by 2 |
| `inductance` | H | line to line; optional (DBP-18) |
| `rotor_inertia` | kg·m² | bell and shaft, about the shaft |
| `spin_dir` | +1 or −1 | today's `spin_dirs` entry for this rotor |
| `stator` | text | identity only, for example "2207" |

**`prop`**

| Field | Unit | Meaning |
|---|---|---|
| `diameter` | m | compiles to today's `radius` = `diameter`/2 |
| `pitch` | m | identity, and an input to a default table (DBP-34) |
| `blades` | count | identity, and an input to a default table |
| `inertia` | kg·m² | about the shaft; optional (DBP-16) |
| `ct_static`, `cq_static` | none | `C_T` and `C_Q` at `J` = 0 |
| `ct_table`, `cq_table` | list of `[J, value]` | measured tables, resampled at spawn (DBP-31) |

The coefficients use the UIUC convention: `T = C_T·ρ·n²·D⁴` and `Q = C_Q·ρ·n²·D⁵`, with `n` in rev/s. A source that gives `C_P` converts with `C_Q = C_P/(2π)` when the part is authored.

For the momentum-theory variant, the static coefficients compile into today's names:
- `thrust_coeff = ct_static·ρ_ref·D⁴/(4π²)`;
- `torque_coeff = cq_static·ρ_ref·D⁵/(4π²)`.

Here `ρ_ref` is 1.225 kg/m³. That variant's static thrust does not scale with density today, which is its known limit.

**`esc`**

| Field | Unit | Meaning |
|---|---|---|
| `current_continuous` | A | continuous rating per channel; above it, a flag is set |
| `current_burst` | A | burst rating per channel; the current clamps here |
| `current_total` | A | total for a 4-in-1 across its channels; the sum clamps here, through the common duty scale (§4) |
| `channels` | count | 1, or 4 for a 4-in-1 |
| `on_resistance` | Ω | adds to the motor's `resistance` |
| `braking` | true or false | active braking; sets `I_lo` (§3) |
| `protocol` | text | identity only, for example "DShot600" |
| `throttle_map` | none | optional, one parameter with the firmware's range (DBP-19) |

The motor's clamp is the smaller of `current_max` and `current_burst`. The model keeps no thermal state, so a burst rating has no time limit here. The flag above `current_continuous` is how a builder sees it.

**`battery`**

| Field | Unit | Meaning |
|---|---|---|
| `cells_series` | count | `S` |
| `cells_parallel` | count | `P` |
| `cell_capacity` | Ah | per cell |
| `cell_resistance` | Ω | internal resistance per cell |
| `cell_voltage_nominal` | V | identity |
| `cell_voltage_full` | V | the top of `ocv_table` |
| `cell_voltage_cutoff` | V | the terminal-voltage clamp per cell (§4) |
| `ocv_table` | list of `[SoC, V]` | per cell, resampled at spawn |
| `c_rating` | 1/h | continuous discharge rating; the current clamp is `c_rating × capacity` |
| `polarization_resistance`, `polarization_capacitance` | Ω, F | `R1` and `C1`, optional (DBP-24) |

### 14.3 Drag: part primitives or a fitted value?

**Answer: the lead's suggestion, with one exception.** A fitted airframe value, with the shapes as the prior, is right for an airframe that has its own flight data. The exception is an airframe whose simulator states its drag exactly.
1. **The simulator states it.** `competition-700` gives `F_i = −0.028·|v_i|·v_i` exactly. Set that value; do not fit it. Then use the drag files (`drag_speed`, `drag_lateral`, `drag_aoa`) as a known-answer test of the fit itself. A fit that does not recover 0.028 within `u_c` has a defect in the fit, not in the drag.
2. **The builder has only parts.** Sum the parts' projected areas per body axis, with a drag coefficient per primitive. This gives a per-axis componentwise drag element, graded best-effort. Drag does not add across parts in reality, because parts shield each other. So it is a prior, never a claim.
3. **The airframe has its own data.** Fit the per-axis coefficients. The primitive sum is the starting point, with a range the builder states.
