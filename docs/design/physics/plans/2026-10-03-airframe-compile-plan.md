# Drone builder: the airframe compile, plan

**Owner:** Physics. **Status:** plan, 2026-10-03. The lead approved building this now, before module-API stage 4 (`2026-10-03-propulsion-rows-plan.md` section 5; DBP-44; Kat's DBE-013).
**Branch:** `physics/airframe-compile`, cut from Core's `core/design-frame`, which adds `ModelType::design_to_principal` and `com_offset`. The branch rebases onto master once Core's branch merges.

After this lands, Kat's compiler can turn a part-built airframe into a `ModelType` that spawns and flies today. Every mount is converted inside Spade, and the rotor is a FITTED momentum-theory model that stands in for the motor, battery and propeller chain until the stateful rows land.

## The function

`vehicles/airframe_compile.hpp`, installed with `vehicles/`:

`Result<CompiledAirframe> compile_airframe(const AirframeSpec&)`

It takes an `AirframeSpec` (question 3, below), all in the design frame:
- every mass as a `PartInertia`;
- each rotor's hub position, its orientation (thrust along local +Y) and its spin direction;
- one propulsion chain shared by every rotor (`PropulsionChain`), and the rotor's polar inertia;
- the drag elements and the IMU mounts;
- the proxy radius;
- the fit's operating point: air density, gravity, state of charge.

It returns:
- the `ModelType`;
- the `CompositeInertia`;
- a fit report: hover duty, hover speed, `k_T`, `k_Q`, `tau` and the solver's flags.

## What it does

1. **Body.** `composite_inertia()` gives the mass, the principal moments, `design_to_principal` (`q_bd`) and `com_offset` (`c`).
2. **Mounts.** Every mount moves to the body frame: `r_b = R_bd·(p_d − c)`, and `q_b = q_bd ⊗ q_d`. That covers rotor hubs and thrust axes, drag elements and IMUs. So an IMU authored with the identity orientation reads in design axes.
3. **Rotor fit, momentum variant.** This is graded best-effort and approximate.
   - `radius = D/2`.
   - `k_T = C_T(0)·ρ·D⁴/(4π²)` and `k_Q = C_Q(0)·ρ·D⁵/(4π²)`, at the operating density.
   - `tau` is the chain's small-signal time constant at hover: `J_r` divided by the slope `d(Q_load − Q_motor)/dω` at the hover root, from a central difference in double on the steady-state solver's balance.
   - The hover duty and speed come from the solver's inverse mode, at weight divided by the rotor count.
4. **Refusals** (`invalid_argument`), each naming its cause:
   - no rotors;
   - an invalid part (from `composite_inertia`);
   - a chain that cannot hover the airframe (the solver flags it unreachable);
   - any field `ModelType::validate()` rejects.

A small addition to `propulsion_steady.hpp` exposes the time constant: `steady_state_time_constant(chain, duty, density, v_axial, soc, rotor_inertia)`.

## Kat's four questions, answered (2026-10-03)

**1. Determinism.** The same `AirframeSpec` gives a bit-identical `ModelType` on every machine.
- Every step is double arithmetic in a fixed order, with one float rounding per output.
- The only non-arithmetic call is `sqrt`, which IEEE 754 rounds correctly. No libm transcendental is on the path (`TD-3`).
- The solver's bisections and the time constant's central difference have fixed iteration counts and fixed steps.
- A test pins the identity of a reference airframe's compiled model. The MSVC gate and the Docker gcc leg must both reproduce it, so the claim is measured, not argued.

That needs two things from Core, both asked for:
- a public `model_identity(const ModelType&)`, the per-model fold behind the snapshot's model-registry identity (Core's task C), which Kat hashes;
- the canonical sign rule for `design_to_principal` (the first non-zero of w, x, y, z positive) moved beside `ModelType`. Then the compile and `register_model` apply one rule (`TD-9`), and a compiled model's identity equals its registered one.

**2. Provenance.** The fit report tags each derived quantity:
- `from_parts`: mass, centre of mass, principal moments, design rotation, every mount;
- `fitted`: the momentum rotor's `k_T`, `k_Q` and `tau`;
- `estimated`: drag, when the spec gives none. It is the parts' projected areas per body axis times a drag coefficient per shape, graded best-effort (spec section 14.3);
- `given`: drag when supplied, the proxy radius, and sensor noise.

Kat's builder shows the tags.

**3. Input.** `AirframeSpec` is a Spade struct that Kat fills. Spade never reads Kat's part files (D-3). It is made of:
- part blocks with the spec's section 14.2 field names and units (`motor`, `prop`, `esc`, `battery`), each with its mass, its mount pose and an optional shape;
- airframe parts (frame, arms, avionics) as `PartInertia`.

The compile turns every block's mass into a part, so the inertia counts everything once.

**4. Refusals, all at once.**
- `check_airframe(const AirframeSpec&)` returns every problem as a list of `{code, part kind, index, field, message}`. It is empty for a valid spec.
- `compile_airframe` calls it first. On any problem, its `Result` error carries the whole list, joined, so Spade's `Result` convention holds. Kat calls `check_airframe` for the structured list.
- So that each check stays in one place (`TD-9`), `composite_inertia` gains a form that collects every part's problems. `compile_airframe` and `check_airframe` both use it.

## What it is not

The fitted rotor matches the chain at hover, and for small changes around it. Away from hover, it departs:
- current limiting;
- battery sag;
- inductance's speed dependence;
- the propeller's inflow dependence.

The header says so plainly. Kat flies with it until the propulsion rows land, then switches to driven rotors.

## Tests (`tests/test_airframe_compile.cpp`)

- A symmetric quad keeps its axes exactly, and its mounts move by `−c` only.
- An asymmetric airframe's mounts round-trip: `R_bdᵀ·r_b + c = p_d`, within float.
- An IMU authored in design axes has `mount_orient = q_bd`.
- `k_T` and `k_Q` match the closed form from the table's `J = 0` values.
- At the hover speed, the fitted static thrust `k_T·ω²` equals weight divided by the rotor count.
- **The time constant against a measurement:** step the duty 1% from hover through the float step functions, and the 63% rise time matches the fitted `tau` within 5%.
- An airframe the chain cannot lift is refused.
- The compiled `ModelType` passes `validate()`.

The new C++ files get the gcc check before review.

## Slot

One, about 20 min: build `spade_vehicles` and `spade_tests`, run the new suite, then the full suite. Nothing in the step changes, so every golden stays where it is.
