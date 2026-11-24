# K2: KAT's plant on Spade's motors (design note, no code)

Date: 2026-11-24 (box clock). Owner: Physics. Status: the seam is **ruled (L-S41)**; everything below it is a proposal for the lead's review. Sources: the SDK's KAT inventory at KAT 2582f5f85 (master 4f3d26134 plus the idle commit), its plant header `dronesim/include/kat/rate_thrust_plant.h`, and Spade at master 3628eec9 (`engine/vehicles/rotor.hpp`, `rotor.cpp`, `engine/physics/forces.cpp`, `integrator.cpp`). Decisions behind it: D223 to D225, O244, L-S40, L-S41. The rule of the move: KAT's flights must come out the same on Spade's model before KAT's copy is deleted (D224); moving KAT to Spade's richer model afterwards is a separate, measured step that goes to Cameron.

## 1. What moves, what stays (L-S41)

**KAT keeps:** the rate loop with its gyroscopic feedforward (`rate_thrust_plant.h:770-780`), the mix u = M^T t, `desaturate`, and the rate-thrust lane's curve `acc_of_norm`, which lives in airframe data so there is one copy.

**Spade gets, per motor:** one command entry of three kinds, `ROTOR_SPEED` (exists: `Simulation::set_rotor_commands`), `THRUST_N` (the rate-thrust lane, after `desaturate`) and `THROTTLE` 0..1 (the mixer lane, `kat.actuator_command.v1`). Each maps in one line to the same speed target; then, in this order: **delay, lag, idle floor (armed), omega_max clamp**. No speed scale crosses the seam: k_T, omega_max, tau and the delay are airframe data (KAT's converter assumes 3000 rad/s, which is wrong for every airframe: the real omega_max are 2400 competition-700, 2340 swift and x250, 1032 x500, 1452 pliska, 2501 crazyflie). On KAT's own path the clamps are no-ops (t is already in [t_idle, T_eff]); they bite only on raw commands.

## 2. The motor entry, stage by stage

| stage | the rule | parity configuration | state (snapshot-exact) | note |
|---|---|---|---|---|
| target | `ROTOR_SPEED`: w. `THRUST_N`: w = sqrt(t / k_T). `THROTTLE`: t = the airframe's throttle-to-thrust table (piecewise linear, saturating), then as `THRUST_N` | k_T = T_eff / omega_max^2, T_eff = thrust_scale * thrust_per_motor, so w = omega_max sqrt(t / T_eff) exactly as KAT (:1060, :1067-1068) | none | the table is airframe data (the mixer lane's per-motor curve, `actuator_wire.cpp:696-704`); its file format is Core's, reviewed there |
| delay | the command reaches the lag `command_delay_ticks` whole **physics ticks** late (a ring) | KAT's FIFO counts base ticks (100 Hz: {0, 1}; 1 kHz: {0, 10}) | the ring: one new registered array, ticks x motors floats, plus a head index | 0 by default and free then (no ring touched, bit-identical); with substeps > 1 Spade quantises to whole base ticks, not substeps; set per episode by KAT's domain randomisation as data |
| lag | exact: w += (1 - exp(-h/tau)) (w_target - w); tau up, `tau_down` down (exists, `RotorRow` row 4) | one tau for both directions (KAT's :1057, :1063); **tau 0 means exact follow** | omega (exists) and `omega_cmd` (exists) | see section 7: the exact form is independent of substeps, so the lag matches KAT at any substep count if the target is constant over the tick |
| prime | the first valid command sets w = w_target with no lag (KAT's priming :1062-1070) | on | one new flag per vehicle ("primed"), in the registered state | MotorState{} at each episode start: a respawn clears it |
| idle | w >= w_idle = sqrt(t_idle / k_T) **only when armed**; t_idle = thrust_scale * idle_thrust_n (competition-700 0.170205 N, swift 0.02646875 N, others 0) | on where the airframe has it | one new per-vehicle `armed` flag, registered | ARMED reproduces KAT's failsafe-at-idle exactly: KAT's plant gives t_idle for a valid command (:921), and its supervisor's failsafe sends valid = 1 with thrust 0 (`supervisor.cpp:103-110`), so today the rotors hold idle in a failsafe; changing that is KAT's choice, not ours |
| clamp | w <= omega_max, then w >= 0 | on | none | omega_max rides in the reserved `RotorRow` row-4 lanes, no layout change |

Three details to settle in the first RED, each a bit-level question:
1. **tau 0 must be bit-clean.** `rotor_lag_alpha` returns 1 for tau <= 0 and the pass computes `w += (w_cmd - w) * alpha`, but `x + (c - x) * 1.0f` is not always `c` in float (the subtraction and addition can leave one ulp). Parity with a rotor that follows instantly (every airframe but swift-quad) needs `w = w_cmd` assigned when alpha == 1, and a test that the bits equal the command over a sweep. This is a change to the existing pass, so the existing rotor tests and the GPU twin's pins move only where a lag-off row exists.
2. **Where the idle floor sits against the delay.** The order is the ruled one, delay then lag then idle then clamp: the idle floor bounds the lag's target, not its output, so a lagged rotor can pass through speeds below idle only on a disarm (armed false drops the floor). KAT does the same (the floor is on t_ok before the lag, :911-933 then :1057).
3. **Armed has two sources today** (Spade's `armed_duty` in the propulsion chain, `vehicles/propulsion_steady.hpp`, and KAT's `valid` flag). K2's `armed` is the vehicle's own registered flag set by the command; the analysis path keeps using its own.

## 3. The two L-S40 switches (general, per vehicle)

1. **Ground effect on or off.** `f_ground` (`rotor.cpp`, the image-source form clamped at 1.25) reads the world SDF at the rotor. KAT has none, so a KAT airframe turns it off and `f_ground = 1` exactly. The switch is one flag per vehicle (a reserved lane or a model field), default **on** (Spade's behaviour is unchanged for every existing vehicle); the GPU rotor kernel (`engine/shaders/kernels/rotors.slang`) reads the same flag.
2. **Inflow-loss model selector.** `per_rotor_momentum` (Spade's, the default: `f_inflow = lambda(v_axial / v_h)` per rotor) or `collective_airspeed_loss`: one factor for the whole vehicle, f(s) = clamp(1 - k s^exp, floor, 1), s = |v_com - wind| (the vehicle's world speed relative to the air, as KAT's :711-717; the legacy source is `quadrotor.py:99`), applied to every rotor's thrust. k, exp and floor are airframe data; only competition-700 has k != 0 (k 0.00028, exp 2, floor 0.35). With the selector on `collective_airspeed_loss`, the per-rotor momentum correction is off for that vehicle (the two never multiply).

KAT's airframes select the parity form (ground effect off, collective loss). Moving them to Spade's richer model later is the measured step that goes to Cameron.

## 4. Drag

KAT's form is D_i = -c |v_b,i| v_b,i per BODY axis on v_world - wind, on the whole airframe (`:765`), no rotor drag, no angular damping, and it is **already Spade's `drag_mode::componentwise`**: `apply_drag_element` (`forces.cpp:22-48`) rotates `v_rel = vel - wind` into the body frame, applies `-c_i |v_i| v_i` per body axis, rotates back, and adds the torque `cross(local_pos, F_body)` (zero at the centre of mass). So K2 adds **no drag code**: KAT stops its own drag wrench, the airframe carries a componentwise `DragSpec` with c per axis (competition-700 0.028, crazyflie 0.00102, x250 and swift 0.1, x500 and pliska 0.031), and the parity test proves it. Two facts from Spade Host go in verbatim:

1. A spawned vehicle brings its model's drag bodies (estimated at C_d 1.0 when the airframe has no `DragSpec`), so KAT's wrench on top double-counts; parity needs a zero `DragSpec` until K2.
2. `apply_wrench` sums with the rotors but acts for the next substep only, so a drag wrench must be reissued every substep, and still lags; drag elements do not.

Drag has no state and its CPU/GPU twin exists (`forces_drag.slang`); K2's work on it is the parity test and the airframe data.

## 5. Torque rows and the gyroscopic cancel (checks, not new code)

- **Rows.** KAT's u = M^T t has rows [1, arm_y, -arm_x, -spin kappa] (`thrust_curve.h:202-231`) in the KAT body frame (x forward, y up, z right; thrust on +y). Spade's rotor pass applies force T on the rotor axis at r and a reaction torque -spin_dir * Q(w) along the axis, with the moment cross(r, F) (`rotor.hpp` section 6). With the axis +y and r = (x, 0, z): the moment is (-z t, 0, x t), and the yaw term is -spin * k_Q w^2 = -spin * (k_Q / k_T) t. So the mix's kappa = k_Q / k_T (KAT's converter sets kQ = kappa * kT, `scene_config.cpp:428`), and the identity to prove term by term in a test is: for each airframe, `Spade_torque(t_1..t_4) == M^T t` to float rounding, including the sign of spin. The rows' sign convention (which of arm_x or arm_y is KAT's "z") is checked against the airframe's motor positions, not assumed.
- **The cancel.** The host integrates I w' = torque - w x (I w) (`integrator.cpp:121-171`, explicit Euler on the angular DOF at the pre-update w); KAT's plant adds +w_b x (I w_b) to its torque so the two cancel and the lag law reads w_next = w + dt/(tau + dt) (gain cmd - w) (`rate_thrust_plant.h` preamble, :770-780). Measured on both hosts at P0-3 (KAT `docs/dev/plant-comparison.md`): both apply the term and the trajectories agree to fp32. The term stays in KAT (the rate loop is flight software) and K2 changes nothing in the integrator. **What to confirm in a test:** the cancel is exact only when w is constant across the tick, so at substeps > 1 the residual is about |I_y - I_x| |w| |alpha| h_tick (inertia difference, rate, angular acceleration); the parity runs use substeps 1 for this reason and a substeps > 1 run reports the residual instead of claiming parity.
- **Precision.** The plant is double in KAT (`:282-287`); `kat_host_spade` narrows the wrench to float. Section 7.

## 6. Lanes and the parity configuration

- **Rate-thrust lane** (20 bytes; swift-uzh-truth, test-racing-truth, test-racing): KAT's plant computes t per motor, `desaturate`s, and sends `THRUST_N`; Spade runs delay, lag (swift-quad only, tau 0.0391, `lag_in_plant`), idle, clamp, ground effect off, collective loss on (competition-700).
- **Mixer lane** (36 bytes `kat.actuator_command.v1`; every demo and test package and the SDK corpus): memoryless in KAT today, with no lag, no drag, no idle. The parity configuration is therefore **lag off (tau 0), idle 0, delay 0, zero `DragSpec`, ground effect off, collective loss off**, and the bit-clean tau 0 above is what makes "off" mean off.
- **Wrench lane** (48 bytes): copied as is; not part of K2.
- **Noise:** KAT ships no non-zero sigmas and Spade's sensors refuse them, so noise parity is moot; K2 builds nothing for it (YAGNI) and the note records the decision.

## 7. The parity band, derived (the L-S36 condition)

The band is derived from the maths that still differs once K2's terms are the same on both sides; none of it is read off a measured run.

**What differs after K2:** (a) KAT's fake host computes in double, Spade in float (`host.cpp` narrows the wrench and the mass); (b) nothing else in the maths, as far as the sources read here show (the fake host's angular integrator is taken from the plant header's account and the P0-3 record, not read from its source; the two-host run of section 8 item 6 is what confirms it): both integrate velocity then position semi-implicitly (`bodies.cpp:228-236`, `integrator.cpp` steps 3 and 5), both take the exp-map orientation (Spade: `integrate_orientation`, q (x) exp(0.5 h w), `integrator.cpp` step 6), both use the exact first-order lag (the exponential composes: exp(-h1/tau) exp(-h2/tau) = exp(-(h1+h2)/tau), so the lag is independent of the substep count at a constant target), and both carry w x (I w). So the difference is **float rounding accumulated over N steps**:

- Velocity: each `v += a h` rounds to at most half an ulp of v; the rounding is uniform, so one step's standard deviation is ulp(v)/sqrt(12), and over N steps it grows as sqrt(N) ulp(v)/sqrt(12) (systematic bias is absent: round-to-nearest). At |v| = 10 m/s (ulp 9.5e-7), N = 3,800 (3.8 s at 1 kHz): 1.7e-5 m/s; the worst case, every rounding the same sign, is N ulp/2 = 1.8e-3 m/s and is not the band.
- Position: the velocity error integrated, of order 4e-5 m for the same flight (about two thirds of T times the final velocity error).
- Angular rate and angle: at |w| = 1 rad/s (ulp 1.2e-7): 2e-6 rad/s, angle error of order 4e-6 rad.
- The same figures measured at P0-3 on a 3.8 s flight with drag on both hosts (tree d80a9fa1): Spade minus fake 1.45e-5 m, 1.41e-5 m/s, 3.5e-6 rad; they sit at the random-walk estimates, which is the check that nothing else differs.

**The band in a test is a formula, not a constant:** |dv| <= 4 sqrt(N) ulp(v_max) / sqrt(12), |dx| <= 4 sqrt(N) ulp(x_max)/sqrt(12) + T |dv|, |dw| <= 4 sqrt(N) ulp(w_max)/sqrt(12), with N and the maxima taken from the run, and a run that exceeds it is a finding (the cause named before any loosening). The factor 4 is a four-sigma allowance for one run; it is the one chosen number and is stated beside the formula. Every term that makes a rate-thrust flight differ in a **systematic** way (a lag with different taus up and down, an idle floor, the delay) is configured identically on both sides, so none contributes. The substep case: with substeps > 1 the wrench is re-issued each substep, the lag is unchanged, and the cancel's residual (section 5) is reported.

## 8. Tests, order, effort (estimates, not measurements)

RED first, each term CPU and GPU twin where a twin exists, and a pin for every changed byte of the model rows:

1. tau 0 bit-clean (CPU, GPU twin) and the existing rotor pins still equal elsewhere; **0.5 day**.
2. The motor entry (target, idle armed, clamp) with the parity maths; **2 days**.
3. The delay ring (registered array, snapshot round trip, 0 free and bit-identical); **1.5 days**.
4. Ground-effect flag and the inflow selector (collective loss), CPU and `rotors.slang`; **2 days**.
5. The torque-row identity and the drag parity test, per airframe; **1.5 days**.
6. The two-host parity run (KAT's fake host against Spade, the rate-thrust and mixer lanes, the formula band of section 7) on competition-700 and swift-quad; **3 days**, mostly harness; the KAT side's changes are not mine.
7. The gate: all existing pins equal (SPIR-V pins move only for the rotor kernel's rows, listed before re-recording), `-L gpu`, the Docker leg.

About **10 to 11 working days** for Physics, plus Core's review of the new model-file keys (`command_delay_ticks`, `omega_max`, the thrust table, the two switches) and the SDK's airframe data. The order matches the dependence: 1, 2, 4, 5, 3, 6.

## 9. Decided (lead, 2026-11-24) and what is still open

**Decided:**

1. **The throttle table:** per vehicle, in the model file, one table for all motors (KAT's airframes have one curve). It is `ModelType` data, so Core reviews the format in the RED; it is static data, not registered state.
2. **Substeps:** parity at substeps 1, per the P0-3 record. The 100 Hz KAT jobs are a separate parity row whose band reports N and h (section 7).
3. **tau 0:** `w = w_cmd` is assigned when alpha == 1, in the CPU pass and its GPU twin. **No digest moves for any run with tau > 0**, and it is proved, not asserted: the existing rotor parity rows and every pinned digest that touches a rotor are run unedited at the slice's head and quoted equal; the only rows allowed to change are those of a rotor with tau <= 0, and they are listed in the commit message before any re-recording.
4. **ARMED:** the per-vehicle flag is new registered state (a snapshot carries it).

**Open, and the finding that changes the lead's rule for it: the per-episode door.** The lead's rule was: if KAT starts an episode on Spade by a respawn, the door is spawn-time overrides on `VehicleSpawn` and no setter is built; if by a restore, the values must be registered per-vehicle state set once after the restore. Read from KAT at 2582f5f85 (read-only): **`kathost_reset` is both.** It is `Simulation::restore(genesisBlob)` (the empty-roster snapshot captured at the tail of `kathost_load_world`, `dronesim/spade/host.cpp:3232`), then a replay of every still-live roster entry's **original** spawn call through `spawnBodyFromDesc()` (the same function `kathost_spawn` uses, `host.cpp:29`, `:1758`, replay loop `:3761-3784`), then `reseed(scene_seed)` (`host.cpp:10-30`, `:3696-3712`). The roster is preserved (ids stable, never reused). So an episode start is a RESPAWN from the entry's STORED original descriptor, which means a per-episode draw (the delay ticks, tau and the drag coefficients the SDK's domain randomisation draws, `rollout.cpp:849-857` shows the SDK side resetting its own plant state on the reset epoch) cannot ride a spawn call: the replay re-applies the original descriptor, not today's draw. Three ways to carry a draw, for the lead and Spade Host to choose; I recommend (a) if spawn cost per episode is small:
   - (a) the SDK despawns and respawns the vehicle each episode with the drawn values in the descriptor (no new Spade code; the vehicle's id changes, so the roster-preserved rule is not available for it);
   - (b) the adapter keeps per-entry episode overrides, set by a new ABI call and replayed after the spawn at reset (a new setter on the adapter and on the vehicle: registered state, snapshot-exact; the most code);
   - (c) no per-episode draw on Spade's side (the values are constant per package and KAT's domain randomisation of them stays in its own plant); only possible while the rate-thrust lane runs through KAT's plant for those terms, which is the opposite of the move.
   What I need from Spade Host: the cost of a `kathost_spawn` of a vehicle (a respawn per episode), and whether an override set before `kathost_reset` could be replayed from the roster entry without an ABI change.

## 10. Dates (estimates against the 10 to 11 working days of section 8; GPU and build queue waits are not in them)

K2's code starts after the trim record and the film branch's READY (the trim re-run started 16:07Z and settles in about an hour). Day 1 is the day after the film branch is READY.

| slice | what | RED | GREEN and READY |
|---|---|---|---|
| 1 | tau 0 bit-clean + the motor entry (target, idle armed, clamp) + the delay ring (items 1 to 3 of section 8, 4 days) | day 1 (a first RED with the three behaviours failing, within the first working session) | day 4 |
| 2 | ground-effect flag + the inflow selector, CPU and GPU (2 days) | day 5 | day 6 |
| 3 | torque-row identity + drag parity per airframe (1.5 days) | day 6 | day 7 |
| 4 | the two-host parity run and the formula band (3 days, mostly harness) | day 8 | day 10 |
| 5 | the gate: pins equal, SPIR-V rows listed, `-L gpu`, the Docker leg (1 day) | | day 11 |
