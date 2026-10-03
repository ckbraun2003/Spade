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

**Limits.**
- Current clamps to `[I_lo, I_max]`. `I_max` is the smaller of the motor's and the ESC's rating. `I_lo` is 0 for an ESC without active braking and `−I_max` with it.
- The duty clamps to [0, 1]. A non-finite command reads as 0.
- Shaft speed never goes below 0. The ESC is one-directional.
- Each clamp sets a flag in the motor row.

**Integration.** The electrical part is linear in ω with time constant `τ_m = J_r·R·Kv²`. The load torque is held at its value at the start of the substep. The update is then exact for the linear part:
- `ω_∞ = Kv·(d·V_bus − R·(I0 + Q_load/Kt))`, the speed the motor would settle at under that load;
- `ω ← ω_∞ + (ω − ω_∞)·α`, with `α = exp(−h/τ_m)`.

`α` depends only on fixed parameters and the fixed substep. It is computed once at spawn, in double, and rounded once, the GNSS coefficients' pattern. The step path then only multiplies.

When the current is at a limit, the motor torque is constant. The update is then `ω ← ω + h·(Q_m − Q_load)/J_r`.

**Reaction on the airframe.** The body receives the motor's torque, `−spin·Q_m` about the thrust axis. Today's rotor applies the aerodynamic torque `k_Q·ω²` instead. The two agree in steady state. They differ during spin-up, which is exactly the yaw reaction a builder needs to see.

**What it replaces.** A rotor with a motor row ignores `omega_cmd` and its first-order lag (`vehicles/rotor.hpp` §5). A rotor without one behaves exactly as today.

## 4. Battery

**Model.** A Thevenin pack: an open-circuit source, a series resistance and one RC branch for slow sag.

- Pack: `S` cells in series and `P` in parallel. Capacity is `P × C_cell` (Ah). The series resistance `R0` is `S·R_cell/P` (Ω).
- Open-circuit voltage: `V_oc = S × OCV(SoC)`. `OCV` is a per-cell table on a uniform state-of-charge grid. Authored points are resampled onto that grid at spawn, in double, and rounded once.
- Sag: an instant drop `R0·I_b`, plus a polarization voltage `V_1` in an RC branch (`R1`, `C1`). `V_1` relaxes with `β = exp(−h/(R1·C1))`, which is precomputed at spawn.
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

**The rotation is published for the template.** When the principal axes differ from the design axes, every consumer of the design frame must apply it. That includes a flight package's commands and the IMU mount.

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
- `1/R`, `1/Kt`, `α` and `β` are precomputed at spawn;
- the step keeps one division per vehicle (the bus solve) and one per rotor (`J`).

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
- DBP-13: The shaft update MUST be the exponential update of §3, with `α` computed at spawn.
- DBP-14: The body MUST receive the motor's torque as the rotor's reaction.
- DBP-15: The shaft speed MUST NOT go below 0.
- DBP-16: The model SHOULD estimate the propeller's share of the polar inertia from its mass and diameter when the builder gives none.
- DBP-17: The model MAY take a maximum shaft speed, clamped and flagged like the current.

**Battery**
- DBP-20: The battery MUST take series count, parallel count, cell capacity, cell resistance, cell cutoff voltage, current rating and an OCV table as builder parameters.
- DBP-21: The bus voltage MUST come from the closed-form solve of §4, with sums in rotor declaration order.
- DBP-22: The battery current MUST NOT exceed its rating, and the terminal voltage MUST NOT fall below cutoff.
- DBP-23: Both battery clamps MUST act through one common duty scale per vehicle.
- DBP-24: The battery SHOULD model polarization with one RC branch, with `β` computed at spawn.
- DBP-25: Temperature effects MAY be added when the medium carries temperature (`engine D6`).

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

**Steady-state solver**
- DBP-50: The solver MUST provide the forward and inverse modes of §7.
- DBP-51: The solver MUST be deterministic and total, and it MUST report an infeasible target with a flag.
- DBP-52: A test MUST show the stepped chain settling to the solver's answer within a derived bound.

**Grades**
- DBP-60: Each stepped model MUST be reference grade on the CPU, with a corpus golden.
- DBP-61: Each stepped model MUST be banded on Vulkan, with a measured band (`TD-2`), or it MUST declare Vulkan `absent`.
- DBP-62: The noise-free models MUST NOT cite `CORE-3`; their bands MUST be measured on their own.

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

The research airframe is the one the joint spec names. Its check is a validation suite, separate from the goldens.

1. **Data.** Use independent component data for parameters: motor `KV`, `R`, `I0`; battery cells; propeller `C_T` and `C_Q`. Use measured system data for checks: thrust-stand sweeps of thrust, torque, current and RPM against throttle, hover throttle and current, and voltage sag under a load step.
2. **No fitting to the checked data.** Parameters come only from component data. The system data is held out.
3. **Model values.** For each checked point, compute the model value with the steady-state solver, in double.
4. **Tolerance.** The band is `k·u_c`, with `k = 2` (about 95%).
   - `u_c = sqrt(u_meas² + u_param²)`.
   - `u_meas` is the measurement's stated uncertainty, or the instrument's stated accuracy.
   - `u_param` is the model's sensitivity to each parameter's stated uncertainty. Compute it by central differences in double, one parameter at a time, combined root-sum-square.
5. **Recording.** The suite records the airframe, the data source, every parameter with its uncertainty, and each point's `u_c`.
6. **No widening.** A point outside its band is a model finding. Never raise `k` or an uncertainty to make it pass (`TD-2`). A point with no stated uncertainty is reported, not asserted.

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
3. **The joint spec's facts.** This draft assumes the joint spec names the research airframe and the builder's part catalog. Physics needs both before the validation suite and the defaults can be written.
