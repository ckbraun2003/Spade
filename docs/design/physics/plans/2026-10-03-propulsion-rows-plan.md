# Drone builder: stateful motor and battery rows, plan

**Owner:** Physics. **Status:** plan, 2026-10-03, for the lead's review. It is built after Core's module-API stage 4 lands. Nothing here is coded yet.
**Spec:** `2026-10-03-drone-builder-physics.md` sections 3, 4, 5 and 8. **Builds on:** the pure functions merged at `1d3112d` (`vehicles/motor.hpp`, `battery.hpp`, `propeller.hpp`, `propulsion_steady.hpp`).

After this plan lands, a spawned vehicle flies on duty commands through the motor, ESC, battery and propeller models in the step, on the CPU first. Its current, bus voltage, state of charge and flags are readable from the host.

## 1. Shape: one optional module

The rows live in a new optional module, `propulsion`, which is not in the standard set. A world without it registers nothing new, so the registry walk and every existing golden stay where they are. This is the pattern Core used for the lock module.

| Piece | Kind | Extent |
|---|---|---|
| `motors` | registered array | one row per rotor slot |
| `batteries` | registered array | one row per body slot |
| `propeller_tables` | configuration buffer of floats | built when models register |
| `ocv_tables` | configuration buffer of floats | built when models register |
| `propulsion.drive` | pass, in Forces, before `rotor.forces` | per world, per battery |

## 2. The rows

Both rows are std430, 16-byte aligned, with every byte a named field, as `RotorRow` is. The byte layout is fixed in the implementation with `static_assert`s.

**`MotorRow`** (one per rotor slot; `enabled` = 0 means the rotor is driven by today's lag):
- `enabled`, `battery_slot` (the vehicle's body slot), `variant` (0 = momentum theory, 1 = coefficient table), `flags` (published);
- `kv`, `resistance`, `no_load_current`, `inertia` (`J_r`);
- `current_min`, `current_max`, `pole_pairs`, `inductance`;
- `duty` (the command; state), `current`, `torque` (`Q_m`) (both published), `alpha` (precomputed when `inductance` is 0, else computed per substep);
- `diameter`, `j_min`, `j_max`, `table_offset`, `table_count` (the propeller table in `propeller_tables`).

**`BatteryRow`** (one per body slot; 64 bytes):
- `enabled`, `cells_series`, `ocv_offset`, `ocv_count`;
- `r0` and `r1` (pack), `beta`, `capacity_ah`;
- `current_max`, `cutoff_voltage`, `soc` and `v1` (state);
- `v_terminal`, `current`, `duty_scale` and `flags` (all published).

State is `duty`, `soc`, `v1` and `RotorRow::omega`. The published fields are recomputed every substep. They are still in the walk, which keeps a restored run bit-identical without recomputing them first.

## 3. The passes

**`propulsion.drive`** runs in Forces, first. For each enabled battery, it takes the world's enabled motors with that `battery_slot`, in rotor-slot order, and runs spec section 8's steps 2 to 7 and 10:
- inflow;
- load torque from the motor's variant;
- duty clamp;
- bus solve (`bus_solve`);
- current and torque;
- shaft update into `RotorRow::omega`;
- battery state.

It is built from the merged pure functions, in their float overloads.

**`rotor.forces`** changes only for driven rotors (`motors[slot].enabled`):
- it skips the lag, because `omega` is already updated;
- it takes thrust from the motor row's variant: the coefficient table, or today's `k_T ω²` with the inflow factor;
- either way, it multiplies by today's ground factor;
- it applies the reaction `−spin·Q_m` from the motor row, not `k_Q ω²`.

Undriven rotors take today's path, operation for operation.

**"Declaration order" becomes "rotor-slot order".** Spec section 4 sums a vehicle's motors in declaration order. A vehicle's rotor slots need not be contiguous after despawns, so the pass sums in rotor-slot order instead. That order is just as deterministic: slot allocation is registered state and is restored by a snapshot. The spec's text changes with this plan.

## 4. What Physics needs from stage 4

Core decides each of these; Physics states the need.

1. **Optional modules register their arrays only when they are in the set**, as the lock does.
2. **Extents** of one row per rotor slot and one per body slot.
3. **Attached rows.** `init` and `free` run from the structural queue. A motor row is freed with its rotor, and a battery row with its body.
4. **Configuration buffers.** Float tables built when models register, mirrored to the GPU, and covered by the model-registry identity (snapshot v3, Core's task C). They hold model data, not state.
5. **A declared cross-module read.** `rotor.forces` reads `propulsion.motors` when the module is present. `propulsion.drive` writes `rotors.omega`, and the scheduler must order it first. An explicit edge, `before: rotor.forces`, makes that order unmissable.
6. **GPU:** fixed bindings that hold empty buffers when the module is absent, so `rotor.forces` records with the select off. That is the lock's pattern.
7. **Host API (Core and Interface):** `set_motor_duty(VehicleRef, span<const float>)`, and reads of each motor's and battery's published fields.

## 5. The model type

`ModelType` gains optional propulsion data, compiled from the builder's part blocks (spec section 14.2):
- one `MotorDesc` per rotor: motor, ESC and propeller, with the table as points;
- one `BatteryDesc`: cells, OCV points and limits.

`register_model` resamples the tables into the configuration buffers. Spawn initializes the rows. Mounts reach the `ModelType` already in the body frame (DBP-44).

**A piece that needs no stage 4: the airframe compile (DBP-44).** A Physics function turns part blocks into a `ModelType`:
- `composite_inertia()` supplies the mass, the inertia, `design_to_principal` and `com_offset`;
- every mount moves to the body frame, `r_b = R_bd·(p_d − c)`;
- the IMU's `mount_orient` becomes `q_bd`, so the IMU reads in design axes.

Until the rows exist, it can emit a momentum-variant model:
- `thrust_coeff` and `torque_coeff` from `ct_static` and `cq_static`;
- `tau` from the chain's small-signal time constant at hover, from the steady-state solver.

Kat could then fly built airframes now, approximately, and exactly once the rows land. Physics recommends building this first. The lead decides.

## 6. Verification

- **Unit:** each pass against hand-built rows. The pass must reproduce the pure functions' answer for the same inputs, bit for bit.
- **The stepped chain settles to the solver.** This is DBP-52 again, now through the step and the registry.
- **A golden, `builder_quad`:** a quadrotor with motor and battery rows flying a duty script that covers:
  - spin-up from rest through the current limit;
  - a step;
  - sag and recovery;
  - a cutoff clamp, run long enough to reach it.

  It is the CPU reference (DBP-60). Its digest is provisional until the Docker gcc leg reproduces it (`TD-12`).
- **Existing goldens:** all unchanged, with the module absent from their sets.
- **GPU:** a `propulsion_drive` kernel and the `rotor.forces` select, measured at zero bands first, then pinned (`TD-2`, DBP-61). The division sites are spec section 8's. Until then, Vulkan declares the module `absent`, and a world that uses it is refused (`L6`).

## 7. Order of work

1. *(optional, now)* The airframe compile, momentum variant (section 5).
2. *(after stage 4)* The rows, `propulsion.drive`, the `rotor.forces` select, `ModelType` data and the duty call, on the CPU, with the `builder_quad` golden. One slot.
3. *(after 2)* The kernels and bands. One slot with the device.
