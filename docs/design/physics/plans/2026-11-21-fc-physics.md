# Physics' part of the flight-computer simulation: sensors, frames, actuators, the X500 match, the step (O140, phase 0)

Physics, 2026-11-21. **Docs only; no code.** Branch `physics/fc-physics-docs`. This is Physics' section of the full-simulation plan (the lead's map, `2026-11-21-full-sim-flight-computer`): a real autopilot, PX4 first and then ArduPilot, runs unmodified as software in the loop and flies Spade's X500 in lockstep. Core's section is `docs/design/core/plans/2026-11-21-fc-core.md`. Where the two meet (kind tags, rows, rings, the step) Core owns the plumbing and Physics owns the models; the questions for Core are listed at the end.

**How this was made.** Web pages read on 2026-11-21 under O138a (read only, nothing saved or downloaded by a session): PX4-gazebo-models `x500` and `x500_base` SDF, PX4's `4001_gz_x500`, `SimulatorMavlink.cpp`, `GZMixingInterfaceESC.cpp`, `GZBridge.cpp`, the PX4 simulation and actuator docs, the MAVLink `common` message set, ArduPilot's `SIM_JSON.cpp` and JSON readme, ArduPilot's Quad X motor page, and Holybro's Pixhawk 6C and GPS pages. Grades use the library's words: measured, published, retailer, derived, fitted, assumed. A figure marked **recalled** is from memory of a datasheet that a session could not read this time. It is not a source, and phase 1 must replace it with a cited figure before any gate or card quotes it.

## 1. Sensors

### 1.1 What the autopilots need and Spade lacks today

PX4 over the MAVLink simulator link (PX4 connects to the simulator's TCP port 4560 as the client):

| Needs | Spade today |
|---|---|
| `HIL_SENSOR` (107): accel, gyro (FRD, SI), mag (gauss), `abs_pressure` (hPa), `diff_pressure`, `pressure_alt`, `temperature` (degC), `fields_updated` per-axis bits, `time_usec` | IMU only. **No barometer, no magnetometer, no temperature.** |
| PX4 reads `abs_pressure` x 100 as Pa and the temperature; it does not read `pressure_alt` (per `SimulatorMavlink.cpp`) | no pressure at all |
| `HIL_GPS` (113): `fix_type`, lat/lon degE7, alt mm (MSL, up), `eph`/`epv`, `vn/ve/vd` cm/s, `vel`, `cog`, `satellites_visible`. PX4 divides `eph`/`epv` by 100 and uses them as **metres** of accuracy (its comment: "cm -> m"), though MAVLink calls them HDOP x 100 | a fix has position, velocity and its own sigma_h/sigma_v (`GnssFix`). **No fix state, satellite count, course, or geodetic datum** (the datum is the consumer's, `03-sensors.md`). A GPS-loss scenario needs a fix state. |
| the sim's clock: `HIL_SENSOR` id 0 sets PX4's clock from `time_usec`, and `px4_lockstep_progress` follows. `HIL_GPS` does not touch the clock | the step count times dt gives an exact integer time |
| id 0 accel goes through PX4's FIFO path (scale `g/2048`, a 16 g range) and gyro through 2000 deg/s | the IMU has **no range limit and no quantisation** (`PHY-7`: an impact reads at full size). A landing at 20 g reads 20 g to PX4's filters only if the path allows it. Add a range and a quantum to the row |
| `HIL_ACTUATOR_CONTROLS` back (`flags` bit 0 = lockstep) | `Simulation::set_rotor_commands()` takes rotor speeds in rad/s (section 3) |

ArduPilot over the JSON interface (UDP 9002, ArduPilot sends a 40-byte servo packet: magic 18458, `frame_rate`, `frame_count`, `pwm[16]`; the sim replies with one line of JSON per packet):

| Needs | Spade today |
|---|---|
| mandatory: `timestamp` (s), `imu.gyro` (rad/s), `imu.accel_body` (m/s^2), `position` (N,E,D m), `velocity` (N,E,D m/s), and `attitude` (roll, pitch, yaw rad) or `quaternion` (the quaternion wins if both are sent) | all of it exists, as truth (the state) or as IMU samples. **Position and velocity are truth here, not a GNSS fix.** |
| optional: `battery {voltage, current}`, `rng_1..6`, `airspeed`, `velocity_wind`, `windvane` | the pack's voltage and current are in the chain; rangefinder is not built |
| the compass is **synthesised by ArduPilot** from position and attitude (its code says so); baro and GPS: *not stated in `SIM_JSON.cpp`* (its base classes were not read) | so an ArduPilot flight does not exercise Spade's baro, mag or GNSS models. That is the first reason to build the sensors for PX4 first. |

### 1.2 The shape (Core's plumbing, Physics' models)

Each new sensor follows `03-sensors.md`: a **row** (everything the sensor needs to continue is in it, including its bias state and rng stream and ring cursor), a **kind tag**, a **seeded stream** (`rng::Stream`, domain-tagged), a **ring** (`kRingDepth` 64 samples, tick-stamped, `poll(sensor, since_index)`).

Proposals (Core assigns the numbers and the byte layout; `sensor_kind::count` is 4 today, so baro would take 4 and mag 5):

- **Barometer row:** `body_slot`, `rate_divider`, `phase`, `mount_pos` (the static port, body frame), white sigma (Pa), Gauss-Markov bias state with precomputed retention and drive (as the GNSS row does: no `exp` and no divide in the kernel), temperature offset and coefficient, the rng stream, the ring cursor. Sample: `pressure_pa`, `temperature_c`, `index`, `tick`.
- **Magnetometer row:** `body_slot`, `rate_divider`, `phase`, `mount_orient`, hard-iron (3), soft-iron (3x3), white sigma (T), Gauss-Markov bias, stream, cursor. Sample: the field in the mount frame in tesla, `index`, `tick`. The world field is one environment constant (inclination, declination, intensity) that the anchor or the scene gives; there is no model of the Earth in Spade, and the doc for the field values says which constant it is.
- **Pressure model:** the ISA troposphere, p = p0 (1 - L h / T0)^(g M / (R L)) with p0 = 101,325 Pa, T0 = 288.15 K, L = 0.0065 K/m, exponent 5.25588; h is the port's height above the datum (grade: published, ISA). The temperature is T0 - L h by default. **The engine takes no libm** (`BitPortability*`), so the power is computed with `fp32_math`'s exp and log (banded across backends, the pressure's band stated beside the sensor's), not `pow`; Test/Docs' FC-U3 test holds the reference table. **It is static pressure and noise only: no prop-wash error, no ground effect on the port.** Spade's airflow solver could give the pressure at the port later; that is roadmap, and the card for any run says so.
- **Magnetometer model:** B_world (tesla) rotated to the mount frame by the conjugate of the body orientation, then soft-iron x field + hard-iron + bias + white noise. **Motor-current fields** (several microtesla near the ESCs on a real X500) are a hard-iron that varies with throttle: not modelled in phase 1; named as a limit.

### 1.3 Noise and bias models, for all four sensors

One family, not two. White noise, a bias random walk and a first-order Gauss-Markov bias are the three terms an Allan deviation plot is read as, so a datasheet's noise density and bias instability become the row's parameters and the conversion is stated, not a second model.

| Term | Used by | Per-sample form (as `imu.hpp` and `gnss.hpp` already do) |
|---|---|---|
| white | all | sigma_sample = density x sqrt(f_sensor) (the shipped IMU rule); GNSS and baro/mag take a per-sample sigma |
| bias random walk | IMU | step sigma = rw_density / sqrt(f_sensor), one step per sample (shipped) |
| **turn-on bias** (new) | IMU, baro, mag | one seeded draw per run from N(0, sigma_on), added to the bias state at start; makes two runs of the same seed identical and two seeds different. Needed because PX4 calibrates at start and the initial offset is what its calibration sees |
| first-order Gauss-Markov bias | GNSS (shipped), baro, mag | b' = a b + q n with a = exp(-dt/tau), q = sigma_b sqrt(1 - a^2), both precomputed on the CPU (shipped pattern) |
| range and quantum (new) | IMU, mag | clamp to the sensor's full scale and round to its LSB; **off by default**, on in the PX4 profile because PX4's id-0 path assumes 16 g and 2000 deg/s |

Every stream is domain-tagged and per-sensor, so a row change moves that family's digests only (`03-sensors.md`). Each term is its own test: the variance and the autocorrelation of a long run against the row, on both backends within `CORE-3`'s band.

### 1.4 Rates and defaults (the X500 kit's sensors)

The kit's Pixhawk 6C carries (Holybro technical specification page, published): ICM-42688-P and BMI088 accel/gyro (the page says BMI055 was discontinued), an **MS5611** barometer and an **IST8310** magnetometer on board; the kit's GPS module is a u-blox M9N or M10 class with an IST8310 (Holybro GPS page: M9N 1.5 m CEP, M10 2.0 m CEP, up to 25 Hz; the page does not say which module the X500 V2 kit ships).

| Sensor | Rate | Value | Grade and source |
|---|---|---|---|
| IMU message | 250 Hz (4 ms) | accel/gyro every message | the lead's figure; `SimulatorMavlink.cpp` has a disabled debug block that assumes a 4000 us interval. **Not stated in PX4's docs.** |
| gyro noise density | | 4.9e-5 rad/s/sqrt(Hz) (2.8 mdps/sqrt(Hz)) | **recalled** (ICM-42688-P datasheet), not read |
| accel noise density | | 6.9e-4 m/s^2/sqrt(Hz) (70 ug/sqrt(Hz)) | **recalled** (same) |
| IMU turn-on offsets, bias instability | | gyro 1 deg/s, accel 20 mg initial tolerance recalled; bias instability **assumed** 5 deg/h and 40 ug | recalled / assumed |
| barometer | 50 Hz | white 1.2 Pa (MS5611 highest-oversampling RMS noise, 0.012 mbar), Gauss-Markov bias sigma 5 Pa, tau 300 s, temperature coefficient 1 Pa/K | the part is published; **the noise is recalled; bias and coefficient are assumed**. The datasheet PDF was fetched but its text could not be read |
| magnetometer | 100 Hz | white 0.3 uT (one LSB; the IST8310's resolution is 0.3 uT/LSB), zero-offset +-0.3 uT, hard-iron +-3 uT, max rate 200 Hz | resolution, offset and max rate **published** (Holybro's magnetometer comparison page); the white sigma is **assumed** equal to one LSB; hard-iron **assumed** |
| GNSS | 10 Hz | sigma_h 1.27 m per axis (1.5 m CEP / 1.1774 for a circular Gaussian), sigma_v 1.5 x sigma_h, velocity sigma 0.05 m/s | CEP **published** (M9N); the 1.1774 conversion derived; the vertical ratio taken from PX4's own Gazebo bridge (it scales the down-axis position noise by 1.5 and the velocity noise by 1.2: `GZBridge.cpp`, published); velocity sigma **recalled** (u-blox), the Holybro page does not state one |
| GNSS fix fields | | `fix_type` 3, satellites 12, `eph` = sigma_h x 100 (cm, as PX4 consumes it) | assumed; PX4's own bridge sets fix_type 3 and eph 0.9 / epv 1.78 |

The PX4 bridge's IMU, baro and mag in Gazebo add **no** noise at all (the bridge forwards them). So PX4's own reference simulation is a noise-free IMU with a noisy GNSS; our runs with noise are harder than PX4's reference, and the first phase 1 flight should run with the noise rows at zero to compare like with like.

**Two decisions in the sensor profiles** (`tools/fc_link/sensor_profiles.hpp`, 2026-11-21): (1) the **reference** profile's IMU converter **clamps at PX4's full scale (+-16 g, +-2000 deg/s) but does not quantise**, because PX4's bridge forwards the values with no converter noise, while a real part's saturation is what PX4's decode would also do; (2) the **16-bit converter** (the quantum) is in the **kit** profile only, since it is a datasheet figure (recalled) and the reference is the noise-free comparison.

**Rates in lockstep.** One `HIL_SENSOR` per 4 ms exchange; `fields_updated` sets the baro bits only on a baro tick (every 5th exchange at 50 Hz is 20 ms) and the mag bits on a mag tick (every 10 ms at 100 Hz is every 2.5th exchange, which is **not a whole number of exchanges**: the mag sample is sent on the first exchange after it is due, which delays it by up to 2 ms; or set the mag to 125 Hz or 50 Hz so it divides). GPS is a separate message every 100 ms. PX4's EKF2 needs the IMU at 100 Hz at least and the mag and height sources at 5 Hz at least (a summary of its tuning page, not verbatim).

## 2. Frames

Spade is right-handed with **+Y up**, **+X forward** and **+Z right** (the model files' design frame; the OpenFrame and X500 records use it, and the IMU mount frame of the X500 model is the design frame). PX4 and ArduPilot use NED (world: North, East, Down) and FRD (body: Forward, Right, Down).

**One matrix does all of it.** Take the world's +X as North and +Z as East, so Down is -Y:

M = [[1, 0, 0], [0, 0, 1], [0, -1, 0]] (rows N, E, D from the Spade x, y, z); det M = +1, so it is a proper rotation and **vectors, axial vectors (angular rates) and a quaternion's vector part all transform the same way**:

- vector (position, velocity, specific force, field): (x, y, z) -> (x, z, -y)
- angular rate: (wx, wy, wz) -> (wx, wz, -wy)
- quaternion (w, x, y, z), body to world: (w, x, y, z) -> (w, x, z, -y), because R_ned,frd = M R M^T
- position: N = x - x0, E = z - z0, D = -(y - y0), with the home (x0, y0, z0) the link's choice
- inverse: (n, e, d) -> (n, -d, e) for a vector; quaternion (w, qn, qe, qd) -> (w, qn, -qd, qe)

The yaw of the world is a convention: **North is Spade's +X.** A scene that wants a different heading applies it in the home/origin the link is given, not in M.

**Test vectors** (all exact from the matrix; the tests check them to 1e-6 and then a round trip over random inputs from the seeded stream):

| Case | Spade | NED / FRD |
|---|---|---|
| level at rest, accelerometer | (0, +9.80665, 0) | (0, 0, -9.80665) |
| free fall | (0, 0, 0) | (0, 0, 0) |
| roll rate about the forward axis | gyro (0.1, 0, 0) | (0.1, 0, 0) |
| yaw rate, turning left | gyro (0, 0.2, 0) | (0, 0, -0.2) |
| pitch rate, nose up (about +Z) | gyro (0, 0, 0.3) | (0, 0.3, 0) |
| position 10 forward, 5 up, 3 left | (10, 5, -3) from a home at the ground origin | N 10, E -3, D -5 |
| velocity | (1, 2, 3) | (1, 3, -2) (climbing at 2 m/s) |
| facing west (yaw 90 deg left about +Y) | q (0.70710678, 0, 0.70710678, 0) | q (0.70710678, 0, 0, -0.70710678), heading -90 deg |
| pitch up 30 deg (about +Z) | q (0.96592583, 0, 0, 0.25881905) | q (0.96592583, 0, 0.25881905, 0) |
| roll right 20 deg (about +X) | q (cos 10 deg, sin 10 deg, 0, 0) | q (cos 10 deg, sin 10 deg, 0, 0) |
| field (N, E, D) = (0.2, 0, 0.4) gauss | world (0.2, -0.4, 0) | on a body facing west: FRD (0, 0.2, 0.4) |

Units on the wire: PX4 `HIL_SENSOR` accel m/s^2, gyro rad/s, **mag gauss (1 T = 1e4 gauss)**, pressure hPa (1 Pa = 0.01 hPa), temperature degC; `HIL_GPS` lat/lon degE7, alt mm, velocity cm/s; the quaternion is w,x,y,z (`HIL_STATE_QUATERNION`). ArduPilot's JSON: SI throughout, NED and body in radians; it says NED for position and velocity but does not say FRD for the body axes (`SIM_JSON.cpp` and the readme), so the first ArduPilot test checks it with the level-at-rest vector above.

**Latitude and longitude.** Spade has no datum. The link is given a home (lat0, lon0, alt0, MSL) and uses the WGS84 radii at lat0: dlat = N / M, dlon = E / (N_prime cos lat0). The round trip to 1 mm over 1 km is a test; exact values follow the formula, not a pasted figure. The home's MSL altitude also sets the baro's datum, so the pressure at the home is the ISA pressure at that altitude, or a QNH if the scene gives one.

**Gravity** is -9.80665 m/s^2 in Y in Spade and `CONSTANTS_ONE_G` in PX4 (the same); a test pins it.

## 3. Actuators

### 3.1 Order and spin

Spade's X500 rotors (model file): rotor 0 at (+x, +z) spin +1, rotor 1 (+x, -z) -1, rotor 2 (-x, -z) +1, rotor 3 (-x, +z) -1. With +x forward and +z right that is **0 front-right, 1 front-left, 2 rear-left, 3 rear-right**, spins +1, -1, +1, -1, and `spin_dir` +1 is counter-clockwise seen from above (the OpenFrame record: rotor 0 front-right counter-clockwise; a yaw-torque sign test confirms it for the X500, listed in the phase 1 tests).

PX4's x500 (`4001_gz_x500`, published): `CA_ROTOR0` (+0.174, +0.174) front-right, `KM +0.05`; `CA_ROTOR1` (-0.174, -0.174) rear-left, `+0.05`; `CA_ROTOR2` (+0.174, -0.174) front-left, `-0.05`; `CA_ROTOR3` (-0.174, +0.174) rear-right, `-0.05`, in FRD (the actuator docs: X forward, Y right, Z down, origin at the centre of gravity). The Gazebo model's `x500_base` SDF names its rotors the same way in **FLU** (Y left): rotor_0 at (0.174, -0.174) is front-right, rotor_1 (-0.174, +0.174) rear-left, rotor_2 (0.174, 0.174) front-left, rotor_3 (-0.174, -0.174) rear-right, turning ccw, ccw, cw, cw (the multicopter motor plugin). A first reading of the two files looks like a position mismatch; it is the frame (Y left against Y right), and the pairing is the same. The PX4 docs do not state the sign of KM; the Gazebo model's turning directions show that **positive KM is counter-clockwise**.

ArduPilot's Quad X (the motor page's diagram; the front is inferred from the motor-test text): motor 1 front-right CCW, 2 rear-left CCW, 3 front-left CW, 4 rear-right CW. **The same order as PX4's.** Whether SERVO1..4 are motors 1..4 by default is not stated on the pages read; a phase 5 check.

**The map (PX4 motor index to Spade rotor index): PX4 0 -> Spade 0, PX4 1 -> Spade 2, PX4 2 -> Spade 1, PX4 3 -> Spade 3**; spins agree with no sign change. Where the link and the model file each state their order, a test pins the table so a model that reorders rotors fails loudly (`model_type.hpp`: element order is part of the contract).

### 3.2 Values to rotor commands

`set_rotor_commands()` takes **rotor speed in rad/s**; the propulsion chain (`rotor_omega_for_duty`: duty, battery, ESC, motor, propeller) turns a throttle into the speed that chain reaches in steady state, with the pack's sag.

- **PX4:** `HIL_ACTUATOR_CONTROLS.controls[i]` is `actuator_outputs.output[i]` with no scaling in `SimulatorMavlink.cpp` and only when armed (zeros otherwise). **The range of that output for a motor is not stated in the files read** (the MAVLink text says -1..1 and "depends on the simulated hardware"; PX4's own Gazebo path sends the output unscaled as rotor velocity with a 0..1000 range, `GZMixingInterfaceESC.cpp`). The phase 1 task is to record the value range of an armed hover in a PX4 SITL run and pin the map from it; the proposal is: output o in [0, 1] is the chain's duty, so the pack sag and the motor and ESC limits are in the loop, which is the point of a motor model. A disarmed (zero) output is duty 0, not the idle speed.
- **ArduPilot:** PWM in microseconds, "typically 1000 to 2000". Duty = (pwm - pwm_min) / (pwm_max - pwm_min) with ArduPilot's `MOT_PWM_MIN/MAX` (defaults 1000 and 2000, not read here); spin-min and the thrust expo are ArduPilot's own, upstream of the PWM, so they are not ours to model.
- **The step:** commands are held for the whole period (zero-order hold). In lockstep the reply to exchange k is applied during period k+1: **one sensor period of delay, 4 ms at PX4's rate**, in the loop in addition to the rotor lag. Both the real flight controller and the Gazebo reference have comparable delays; the run record states ours.

## 4. The X500 match

PX4's x500 as published (PX4-gazebo-models `x500` and `x500_base`, PX4 `4001_gz_x500`, read 2026-11-21) against Spade's kit model (`tools/flow_movie/data/x500/`, derived from the build table, see its notes file).

| Quantity | PX4 / Gazebo | Spade X500 | What to do |
|---|---|---|---|
| mass | 2.0 base + 4 x 0.016077 rotors = 2.0643 kg | 1.6326 kg (the build table's 16 entries) | **State.** PX4 has no mass parameter (it estimates hover thrust online; `MPC_THR_HOVER` 0.6 is a start value). Keep ours. For the link check, add a **PX4-match variant** of the model (below) |
| inertia (kg m^2) | 0.021667, 0.021667, 0.040 about the centre | principal 0.02375, 0.02897, 0.02340 (products small) | **State.** Yaw inertia is 41% under PX4's, pitch 34% over. The variant matches PX4's |
| rotor positions | (+-0.174, +-0.174) m; `CA_ROTOR*_PX/PY` the same | (+-0.17689, +-0.17689) m measured (Interface, L98) | **Change PX4's params** for the kit run: `CA_ROTOR*_PX/PY` = +-0.17689 (1.7% wider). Cheap |
| thrust constant | 8.54858e-6 N s^2/rad^2, max 1000 rad/s (34.2 N total), hover about 769 rad/s at 2.0643 kg | k_T = C_T(0) rho D^4 / (2 pi)^2 = 1.2445e-5 (0.09636, 1.225, 0.254; my arithmetic from the model's table), hover 567.1 rad/s at 1.6326 kg | **State**, and **change PX4's params**: `MPC_THR_HOVER` to the chain's hover duty (the case's `trim.hover`) and fit the thrust-curve factor to the chain's duty-to-thrust curve. PX4 re-estimates hover thrust, but a wrong start value is a take-off transient |
| moment ratio Q/T | `momentConstant` 0.016 m | C_Q(0)/C_T(0) x D = 0.0155 m (0.00587/0.09636 x 0.254) | within 3%. **State.** PX4's allocator assumes `KM` 0.05, which does not match the Gazebo model's 0.016; what unit `KM` is in is not stated, so **check in phase 1** whether a PX4 `KM` should follow ours |
| rotor time constant | `timeConstantUp` 0.0125 s, `timeConstantDown` 0.025 s (first order on rotor speed) | tau 0.0761 s, **one value both ways** (the stand-in chain's) | The largest control-relevant difference: PX4's rate loop sees 3 to 6 times more lag than the Gazebo tuning assumes. **Change ours for the variant** (0.0125 up, 0.025 down: needs a `tau_down` in the rotor row, Core's layout, Physics' model) and **state** it for the kit: the Gazebo values are simulation settings, not measured on the kit; the real kit's step response comes in phase 6 |
| rotor drag, rolling moment | 8.06428e-5, 1e-6 | none as such (the rotor model has induced inflow and the body has drag coefficients 0.0222, 0.0348, 0.0224) | **State.** PX4's x500 sets `EKF2_BCOEF_X/Y` 0 (no drag fusion) and `EKF2_MCOEF` 0.12, so the difference is in the flight dynamics, not in PX4's estimator |
| speed limit | `maxRotVelocity` 1000 rad/s | set by the pack's current limit and the chain | the variant uses a high-voltage stand-in or the chain's own; **state** |
| `rotorVelocitySlowdownSim` | 10 (a visual slow-down in Gazebo) | none | no effect on dynamics; ignore |
| battery | PX4 SITL simulates its own (parameters unread) | 4S, `pack_current_max` 67.2 A, sag in the chain | whether PX4 needs the voltage on the link, and which message carries it, is **not known**: an open question |
| ground contact | landing legs in the model | **the kit model carries no legs** (the notes file's `not_carried`) and a contact proxy of radius 0.377 m | section 5.3 |

**The PX4-match variant.** A second model file that sets mass 2.0643 kg, the inertia above, k_T 8.54858e-6, Q/T 0.016, the two time constants and a 1000 rad/s limit, so the same lockstep flight can be flown against PX4's own Gazebo reference and the two compared; if our simulator flying PX4 with PX4's own x500 numbers does not behave like PX4's Gazebo does, the link or a frame is wrong, not the model. Then the kit model, with its own numbers, is the accurate one. This is a generated file like the OpenFrame model (`tests/x500_model.hpp` style), with every number's grade in its notes file.

## 5. The step

### 5.1 The tick

The airflow loops (the lines at 25 us and 0.2 ms) are separate from the rigid-body step and are **not** on the autopilot's path. The link uses the rigid-body step only, with **dt = 500 us, one substep**: it divides PX4's 4 ms (8 ticks), ArduPilot's 2.5 ms at its default 400 Hz (5 ticks), 2 ms and 1 ms; **dt = 1 ms does not divide 2.5 ms**. The sensor periods in ticks: IMU 8, baro at 50 Hz 40, mag at 100 Hz 20, GPS at 10 Hz 200. A convergence check is a phase 1 test: hover and an attitude step at 500 us against 250 us, error in the attitude and the altitude. Until that passes, 500 us is a proposal.

### 5.2 One exchange

1. Spade advances one sensor period (8 ticks) with the current rotor commands.
2. It polls each sensor's ring from its last index (stateless `poll`), builds `HIL_SENSOR` (and `HIL_GPS` when its period has passed), and sends them with `time_usec` = tick x dt in microseconds (an exact integer).
3. PX4 steps its clock from `HIL_SENSOR` id 0, runs, publishes `actuator_outputs_sim`; its sender thread then sends `HIL_ACTUATOR_CONTROLS` with the lockstep flag. **How many controls messages PX4 sends per sensor message is not stated** (it sends at the rate `actuator_outputs_sim` is published); the phase 1 link test counts them and the rule is 1:1 or the link refuses with a named error. The wait has a timeout (PX4's own poll is 100 ms; the link's is longer, and a timeout is a named failure, never a silent step).
4. Spade applies the controls to the next period.

ArduPilot's JSON: ArduPilot sends a servo packet per frame; the sim replies with one JSON line; `frame_rate` is the step the sim **may** take (`SIM_RATE_HZ`, default 400); the code blocks on the reply (polling every 100 ms) and resends after about 1 s without one (the readme says 10 s; the code's condition is 1 s). The same exchange shape, 5 ticks.

### 5.3 Ground contact at take-off and landing

- **Resting contact** already works: the IMU reads +g up (`PHY-7`). The reported velocity at rest still reads about -g x dt (`PHY-8` is not built), which is **-4.9 mm/s at 500 us**: far under PX4's landed-detection thresholds and the GNSS noise, so it is a named limit, not a blocker.
- **The geometry is the open problem.** The kit model has **no landing legs** and a contact proxy radius of 0.377 m (`proxy_radius` is the world-default override, `D-S6-2`). A drone that rests on a 0.377 m sphere stands 0.377 m off the ground. Take-off and landing need contact points where the legs' feet are. Options: (a) the model gives several small contact proxies, at the feet (needs Core: how many proxies a body can have), (b) one proxy of the right radius under the centre (wrong on a slope and in a tilt), (c) a "standing height" in the model and a plane contact. Recommendation (a); the legs' positions come from the kit's mesh (Interface's build table does not carry them today).
- **Ground effect** for the rotors exists (`D5`); it reads the height above the surface. Prop-wash recirculation near the ground is not modelled.
- **Friction and tip-over** on take-off: the existing contact friction applies; a scripted take-off that tips is a test.
- **What PX4 does on the ground** (landed state, auto-disarm after landing) depends on the estimated vertical velocity and acceleration; the phase 1 flight (arm, take off to 5 m, hold 60 s, land) is the check.

## 6. Effort, open questions, and arguments against the picks

**Effort (1 smallest to 5 largest; Physics' parts only).**

| Phase | Physics' work | Effort |
|---|---|---|
| 0 | this record | 1 |
| 1 | barometer and magnetometer rows and kernels, with Core's plumbing and a CPU-GPU band test each | 3 |
| 1 | noise additions (turn-on bias, range and quantum, Gauss-Markov for baro and mag) and the statistical tests | 2 |
| 1 | frame and unit converter (a header with the vectors above as RED tests), geodetic home | 2 |
| 1 | actuator map, the PX4-match variant model and its notes, the asymmetric rotor time constant | 3 |
| 1 | ground contact geometry for take-off and landing (legs, feet proxies) | 4 (it is the least known) |
| 1 | the step convergence test at 500 us | 1 |
| 2 | latency and sensor-to-motor timing meters on the Spade side | 2 |
| 4 | GPS fix states and loss, gust sweeps with the sensors, battery sag scenarios | 3 |
| 5 | ArduPilot: what its JSON needs from the state (truth, not sensors), the battery field, frame check | 2 |
| 6 | calibrating the models to the real kit's logs (thrust, time constant, drag, noise) | 4 |

**Open questions.**

1. **PX4's value range for a motor output in `HIL_ACTUATOR_CONTROLS`** (section 3.2), and how many controls messages per sensor message (5.2): both from a PX4 SITL run, not from the pages.
2. **What the contact proxy is for a multi-foot drone** (Core): can a body carry several?
3. **Kind tags and row layouts** for the baro and the mag (Core assigns).
4. **PX4's battery on the link:** does PX4 SITL need the pack voltage from the simulator, or does it simulate its own battery (so a 4S pack's low-voltage failsafe would fire on its own numbers)?
5. **Which GPS the kit ships** (M9N, M10 or M8N) for the sigma values.
6. **The datasheet figures** marked recalled: the ICM-42688-P noise, the MS5611 noise, the u-blox velocity accuracy. A session that can read the PDFs does this in phase 1.
7. **PX4's own rates** (IMU 250 Hz, baro, mag, GPS) are not in the docs pages read; the link test measures what PX4 accepts.
8. **The home heading:** North = Spade +X; does any scene need a different one? (Proposed answer: no, it goes in the home.)

**Arguments against the picks (reasons, not a refusal).**

- *PX4 through the MAVLink simulator link, first.* It is the right first link: it is documented as the generic simulator API, it is lockstep, and its sensor messages are the ones our sensors map to. The risk is that PX4's own simulation work has moved to Gazebo and uXRCE-DDS, so the MAVLink path is maintained rather than extended; **pin the release** and run the link's 10,000-step test against it at each bump.
- *ArduPilot second.* Its JSON interface takes **truth** for position, velocity and attitude and synthesises the compass itself, so ArduPilot flights do not exercise the sensors we are building. That supports the order, PX4 first, and means phase 5 is cheaper for Physics (effort 2) but tells us less about Spade's sensors. If a customer needs sensor noise through ArduPilot, ArduPilot's own simulation noise parameters apply, not ours; I have not read which it has.
- *The X500 first.* Right choice (PX4's reference), with the mass, inertia and time-constant differences above made visible by the variant rather than hidden. The risk of the kit model alone is that a PX4 tuned for 2.06 kg and 12.5 ms motors is tested on 1.63 kg and 76 ms motors: any instability would be the model, and the variant is how we would know.
- *Budget plus measured modes.* Outside Physics' part; no argument.
- *Lockstep at 4 ms with a 500 us tick.* It makes the autopilot's delay one period (4 ms); a faster sensor rate lowers it. If the EKF delay turns out to matter, 2 ms is available (4 ticks).

## 7. Addendum (Physics, 2026-11-21, later the same day): the pinned PX4 tag, and what batch 48 settled

**7.1 PX4 v1.17.0's x500 airframe is a trapezoid, not the square quoted in 3.1 and section 4.** Interface read `ROMFS/px4fmu_common/init.d-posix/airframes/4001_gz_x500` at the pinned tag v1.17.0 (commit d6f12ad1) in a local shallow clone, and **Physics then read the same file first-hand** from that clone (`git show d6f12ad1:ROMFS/px4fmu_common/init.d-posix/airframes/4001_gz_x500`, read only, 2026-11-21): CA_ROTOR0..3 `PX`/`PY`/`KM` and `MPC_THR_HOVER` 0.60 are as in the table below. In the same file the four simulated motor outputs are `SIM_GZ_EC_FUNC1..4` = 101..104 with `SIM_GZ_EC_MIN` 150 and `SIM_GZ_EC_MAX` 1000, which are Gazebo-only (our link's motors are `PWM_MAIN_FUNC1..4` = 101..104, Interface). The page this document's 3.1 and section 4 read was the `main` branch, which has the square. At the tag, in FRD:

| Motor | `CA_ROTOR*_PX` | `_PY` | `KM` | Corner |
|---|---|---|---|---|
| 0 | +0.13 | +0.22 | +0.05 | front-right, CCW |
| 1 | -0.13 | -0.20 | +0.05 | rear-left, CCW |
| 2 | +0.13 | -0.22 | -0.05 | front-left, CW |
| 3 | -0.13 | +0.20 | -0.05 | rear-right, CW |

Also at the tag: `MPC_THR_HOVER` 0.60, and the motors are `PWM_MAIN_FUNC1..4` = 101..104 on our link (`SIM_GZ_EC_*` are Gazebo-only).

- **The order, spin and sign map is unchanged** (3.1's table and `{0, 2, 1, 3}`): the corners and the `KM` signs are the same; only the magnitudes differ. The map's test compares signs and corners, not magnitudes.
- **The magnitudes in section 4's "rotor positions" row are `main`'s, not the tag's.** The Gazebo model's SDF (+-0.174) does not match the tag's airframe file (the allocator's trapezoid): PX4's own reference flights run an allocator that believes one geometry on a simulated airframe with another.
- **Consequence for the PX4-match variant (agreed with Interface in writing, 2026-11-21).** To reproduce PX4's reference flight the variant's *dynamics* take the SDF's geometry (+-0.174 square, which is what Gazebo flies) and PX4's parameters are left as shipped (the trapezoid). The kit run is different and cleaner: the kit's measured +-0.17689 square (Interface, L98) goes in both the model and `CA_ROTOR*_PX/PY` (Interface's second airframe file, `10091_spade_x500_kit`), and `MPC_THR_HOVER` becomes the chain's hover duty from the case's trim (Physics to send when the kit run is near; whether `KM` should follow our Q/T of 0.0155 is open, as in section 4). Interface's reply, in its words: its `10090_spade_x500_px4` keeps PX4's shipped parameters exactly (the trapezoid `CA_ROTOR*`, `KM` +-0.05, `MPC_THR_HOVER` 0.60, motors on `PWM_MAIN_FUNC` 101..104) so the comparison with PX4's Gazebo reference is like for like; the variant model's dynamics are Physics' (the SDF's +-0.174 square, mass 2.0643 and the rest); `10091_spade_x500_kit` takes the measured +-0.17689 square in `CA_ROTOR*_PX/PY` (FRD) and `MPC_THR_HOVER` from the chain's hover duty, written when Physics sends the number and the `KM` answer.

**7.2 Section 5.3 (ground contact) is superseded by `2026-11-21-ground-contact.md` (merged in batch 48).** What was open there is closed: the route is the existing L68 hull through the model's `collision:` block, with no engine change; twelve points, three to a pad, 16 sweeps; the checks (`tests/test_x500_ground_contact.cpp`) pass 6/6 at the link's 500 us tick: rest height, a clean IMU (+g, spread 1e-6), a drop, the tip-over bracket (about 31 degrees in pitch, 34 in roll) and a 20-degree slope that holds at friction 0.6 and slides at 0.3. Two of its numbers are the named limits this document gave: the rest velocity -g dt (-4.9 mm/s at 500 us) and a slope creep of g sin(slope) dt^2 a step (3.4 mm in 2 s at 20 degrees), both `PHY-8`'s. The feet-only limit stands: a drone tipped past about 31 degrees has nothing else to hold it up; points on the plates and guards come when a scenario tips (phase 4).

**7.3 The actuator map is built** (`tools/fc_link/autopilot_map.*`, branch `physics/fc-link-actuator-map`): `px4_output_to_duty`, `ardupilot_pwm_to_duty` and four duties through `rotor_omega_for_duty` into Spade's rotor order. The PX4 output range on the wire is still not read from a SITL run: the map assumes [0, 1] and refuses outside a 1e-3 slack by name, and phase 1's link test settles it.
