# Drone builder: pure functions, plan

**Owner:** Physics. **Status:** plan, 2026-10-03, for the lead's review. The user approved the joint spec on 2026-10-03, and Physics builds first.
**Spec:** `2026-10-03-drone-builder-physics.md` (DBP-01 to DBP-52).
**Branch:** `physics/builder-functions` in `../spade-wt/physics`.

After this plan lands, Kat can fit against Spade's motor, battery and propeller functions, the composite-inertia utility and the steady-state solver from the installed headers. Nothing here needs the module API, and nothing here runs in the step yet.

## Scope

| In | Out (and why) |
|---|---|
| Motor and ESC pure functions, with the inductance term (DBP-01, 02, 10–19) | Stateful motor and battery rows, flags in registered state, the duty host call (DBP-05, 06, 10's command path): module-API stage 4 |
| Battery pure functions and the bus solve, with both battery clamps (DBP-20–26) | GPU kernels, goldens and bands (DBP-60–62): they follow the rows |
| Propeller coefficient tier (DBP-30–33) | A default table from pitch and blades (DBP-34, MAY) |
| Composite-inertia utility (DBP-40–43) | The frame seam: compile, spawn, vehicle-state read (DBP-44–47): the model type is stage 4; spawn and the read are Core's |
| Steady-state solver, forward and inverse (DBP-50–52) | The momentum-theory variant in the solver: the fit uses the coefficient tier |

## Public names

Stable names in `spade::vehicles`, because Kat's fit links against them. Each header is installed with `vehicles/`.

| Header | Functions and types |
|---|---|
| `vehicles/motor.hpp` | `motor_kv_si`, `motor_effective_resistance`, `motor_current`, `motor_torque`, `motor_time_constant`, `motor_alpha`, `motor_speed_step` |
| `vehicles/battery.hpp` | `battery_ocv`, `battery_resample_ocv`, `battery_rc_beta`, `battery_step`, `bus_solve` with `BusMotor` and `BusResult` |
| `vehicles/propeller.hpp` | `PropellerTable`, `propeller_resample_table`, `propeller_advance_ratio`, `propeller_coefficient`, `propeller_thrust`, `propeller_torque` |
| `vehicles/composite_inertia.hpp` | `PartInertia`, `PartShape`, `CompositeInertia`, `composite_inertia`, and one function per primitive's own inertia |
| `vehicles/propulsion_steady.hpp` | `PropulsionChain`, `SteadyPoint`, `steady_state_at_duty`, `steady_state_for_thrust` |
| `vehicles/propulsion_flags.hpp` | `propulsion_flags::` bits: duty clamped, current limited, speed clamped, out of table, battery current limited, battery cutoff, battery empty, unreachable, no bracket |

## Rules the code follows

1. **One source, two precisions.** The algebraic functions have `float` and `double` overloads. Both overloads call one internal template in the `.cpp`, so both compile under `spade_vehicles`' strict floating-point flags. The step will call `float`; the solver and Kat's fit call `double`.
2. **No libm transcendental on a path that feeds state (`TD-3`).** `motor_alpha` and `battery_rc_beta` use `core/fp32_math`'s `exp32`, as the GNSS coefficients do. The composite utility uses only `+ − × ÷` and `sqrt`, which IEEE 754 rounds correctly, so its double results are portable. *This corrects the spec's §3 and §4, which say "in double" for `α` and `β`; the spec edit rides on this branch.*
3. **Total (DBP-02).** Every function returns a defined, finite value for zero, negative, non-finite and overflowing inputs. A test pins each case.
4. **Fixed op order.** Sums run in declaration order. The bus solve's clamp loop runs at most once per motor. The battery's common duty scale is a bisection with a fixed 24 iterations, so both backends will run the same operations.
5. **Independent expectations (`TD-4`).** Each test computes its reference from the closed form in double, never by calling the function under test.

## Tasks

Each task commits its tests first, against stubs that return wrong values, so the run shows red for the right reason. The implementation then turns it green.

1. **Motor** (`motor.*`, `test_motor.cpp`):
   - no-load speed `Kv·V`, stall current `V/R`, and torque at stall;
   - `R_eff(ω)` against `R + (pωL)²/R`, and equal to `R` with `L` = 0;
   - the exponential step against the closed-form ODE solution;
   - the current-limited branch's linear step;
   - speed never below 0, and friction never reverses the shaft;
   - the degenerate inputs.
2. **Battery and bus** (`battery.*`, `test_battery.cpp`):
   - OCV lookup and resampling against linear interpolation in double;
   - the RC step against its closed form, and `SoC` bookkeeping;
   - the bus solve against hand-solved two- and four-motor cases;
   - a motor moving to the fixed sum;
   - the battery current and cutoff clamps;
   - `SoC` clamps at 0.
3. **Propeller** (`propeller.*`, `test_propeller.cpp`):
   - `J` against `V/(nD)`, and `n` = 0 giving 0 thrust and torque;
   - uniform-grid lookup against interpolation in double;
   - end values held and flagged outside the range;
   - resampling of an authored, non-uniform table.
4. **Composite inertia** (`composite_inertia.*`, `test_composite_inertia.cpp`):
   - each primitive against textbook moments;
   - the parallel-axis theorem;
   - a rotated part;
   - a symmetric airframe keeps its axes exactly, with an identity rotation;
   - an asymmetric one gives a right-handed rotation that diagonalizes `I`;
   - invalid parts refused;
   - one fp32 rounding per output.
5. **Steady-state solver** (`propulsion_steady.*`, `test_propulsion_steady.cpp`):
   - the forward mode against a hand-built chain;
   - the inverse mode's hover duty;
   - the unreachable and no-bracket flags;
   - **DBP-52**: a test loop drives the `float` step functions at constant duty and settles within a bound derived from `α` and the substep.
6. **Docs:** the spec's §3 and §4 fix (rule 2); `07-status.md`; `02-responders.md` gains the builder models as built pure functions; `04-verification.md` lists them as unit-tested and not yet in the step.

## Slot

One slot, about 30 min:
1. Build `spade_vehicles` (about 2 min).
2. Build `spade_tests` incrementally: five new test files plus the CMake changes.
3. Run the five suites on the stubs (red), then on the implementation (green).
4. Run the full suite once.

Nothing in today's step changes, so every golden stays where it is. The full run shows that.
