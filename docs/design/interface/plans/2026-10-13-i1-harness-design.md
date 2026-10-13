# I1: the drone-box airflow harness -- design

**Owner:** Interface. **Status:** **approved by the lead, 2026-10-13**, with the rulings below. LiveRun is coded against Physics' P7a, a disc-only coupler, as soon as its signature arrives. Solids (P2, P2b) and loads (P3) join as they land.
**Base:** master `0d949a8`.
**Answers to:** the demonstration plan's I1 (`../../physics/plans/2026-10-12-airflow-drone-demonstration-plan.md`), Interface's inputs (`../2026-10-12-airflow-demo-interface-inputs.md`), and the CFD spec's live runs and W1 (`2026-10-12-drone-builder-cfd-design.md` §3, §14).

## What changed since the inputs (2026-10-12)

The inputs put the stepping core in the sandbox (`sandbox/airflow_run.hpp`). Since then, INT-5 and INT-6 made the analysis a display-free installed API that Kat's builder hosts, with the sandbox as its reference host. So **the harness's stepping core is the API's `LiveRun`, in `spade::analysis`**, and one core serves four drivers:
1. the sandbox's Airflow panel (the window, D2);
2. the recording of D1, offline, in slow motion;
3. the queued runner's real path, which W2c left refusing ("no airflow solver yet (W1)");
4. Kat's builder, through its facade.

Settings come from a case (`.case.yaml`, INT-5), not from command-line flags. That also removes the inputs' `--height` collision: the sandbox's `--height` already means the window's height.

## The core: `LiveRun`

`engine/analysis/live_run.hpp`. Display-free, with no wall clock (`M1B`) and no threads. A sketch of the names; the signatures follow P7's.

```cpp
class LiveRun {
  public:
    // Builds the drone, the grid and the solids from a resolved case. Refused by name (L6):
    // no step rule, a disc's kernel leaving the grid, the floor too close (P2b).
    static Result<LiveRun> start(const ResolvedCase& resolved, const vehicles::ModelType& model);

    Result<LiveStep> step(const Controls& controls);   // one fluid step, h
    // Steps to the first fluid step at or past the next frame boundary (k/30 s), then publishes.
    Result<LiveFrame> advance_frame(const Controls& controls, FlowVolume& out);

    const LiveState& state() const;              // t, steps, attitude, per-rotor omega, armed
    const LiveLoads& loads() const;              // per rotor: k_T omega^2, the disc's thrust and torque; the body's force and moment (P3)
    std::string run_line() const;                // "actuator disc · 128³, dx 12.5 mm, R/dx 6 · t = 0.250 s"
    const ControlProgram& recorded_controls() const;  // the flight so far, for "queue at higher fidelity"
};
```

**One fluid step**, in order (one-way: the drone never feels the air):
1. **The drone.** A `Simulation` of the case's model, one body, one step of `h`:
   - `set_body_state()` holds it at the box's centre at the commanded attitude, with zero velocity, and with `omega_body` set to the attitude's rate so the state is consistent. The attitude moves toward the controls' angles at a capped rate, so a stick input turns it smoothly. This uses Core's C5 contract, the one Kat's host uses, and it is Cameron's drone-box premise: held at the centre, orientations and thrust states by manual movement.
   - **A held drone's IMU readings are meaningless** (the hold, not the air, sets its motion), so LiveRun exposes none and no view shows them.
   - `set_rotor_commands()` sets each rotor's `omega_cmd` from the throttle, or 0 when disarmed. The motor model spins the rotors up and down, which sheds the start-up ring.
2. **The coupler (P7).** It reads the rotor rows and the pose, and gives the discs (P1, with swirl) or the lines (P4). It poses the parts (`posed_parts()`) and gives them to the solver's solids (P2's `set_solids()`, through a `SolidSampler` over `sample_parts()`).
3. **The fluid.** `fluid_step()` with the case's `DomainBc` (the drone box, already resolved) and `FluidConfig`. A fault refuses the step by name (`describe(health)`) and sticks.
4. **The loads (P3):** the wall pressure's force and moment, and each rotor's disc loads.

**Time is integer nanoseconds.**
- `h` comes from the step rule at `start()`, and the drone's `Simulation` steps at exactly `h` (one drone step per fluid step), so the two clocks never drift.
- Frame `k` ends at the first fluid step at or past `round(k · 10⁹ / 30)` ns: frames are uniform in simulated time, as the C0 records already read.
- The host measures the real-time ratio around `advance_frame()` and only shows it.

**Determinism:** two runs of one case and one control program give the same bits on the CPU path (spec §5's re-run rule). A test pins it.

## The drivers

- **The queued runner.** Without `--dry-run`, the runner steps `LiveRun` with the case's controls, which a queued run holds constant (INT-6, A1). Its history's columns are each rotor's thrust and torque, then the body's force and moment, in SI with units in the header. W2c's refusal goes, and the dry run stays.
- **The recording (D1): one recorder, not two.** Rendering's `tools/flow_movie/` (unmerged, `rendering/flow-movie`) already has the encoder, the captions and the GL stage, and drives its own loop over hard-coded quad discs. **Proposal:** `flow_movie` gains `--case <file.case.yaml>` and drives `LiveRun`. The quad-wake video then comes from the case, solids and all once P2 lands, and the sandbox grows no second encoder. Lead: Q1.
- **The panel (D2)** builds a case through `case_issues()` and `resolve()` (refusals shown verbatim), starts a `LiveRun`, and calls `advance_frame()` from its frame loop. Changing the rotor model, the grid or the floor restarts the run (spec §5). The views bind to `FlowVolume`. I2 (readouts and the validation section, already readable through `validation_report()`) follows R3, which is merged.
- **Kat's host:** the same calls through its facade. `start()` and `advance_frame()` are long: run them off the GUI thread. That's the per-call note I gave Kat's Editor.

## Refusals (L6)

Each is refused by name, before any step where it can be:
- **From `resolve()`, already:** a grid over the byte cap, a wall feature thinner than two finest cells, a changed model or parts.
- **From the step rule:** no stable `h` for the case's speeds.
- **From `start()`:** a disc's kernel support leaving the grid (`check_disc`), and the floor closer than P2b allows.
- **From `step()`:** a fluid fault (non-finite, ill-posed domain or region), naming the step and the cell.

## Tests

- A tiny grid (16³, a few steps) is a CPU test: it runs, it is deterministic bit for bit, and the drone stays at the centre.
- Frame boundaries in simulated time: frame `k` ends at the first step at or past `k/30` s.
- Arm and disarm: omega rises from 0 after arming and falls after disarming; the discs follow it.
- One-way: the drone's state is identical with the fluid on and off.
- The runner's real path writes a history of the right width and units; W2c's dry run still works.
- Each refusal above, by name.
- Kept out of debug ctest's 60 s budget: any test over a few seconds is release-only or chunked (memory note: debug ctest timeout).

## What it needs, and from whom

| Need | From | Status |
|---|---|---|
| P7: the coupler's signature (rotor rows and pose → discs or lines, and the solid sampler) | Physics | not started |
| The step rule (CFL, and the tip's one-cell rule for the line) as a function; `TimeSetup::step` defers to it | Physics | not started |
| P2: `set_solids()` and `SolidSampler` | Physics | plan `dc0ee6d`, Task 1 in progress |
| P2b: the box's placement and the floor-height rule | Physics | designed |
| P3: wall force and moment; publish's apertures | Physics | designed |
| Throttle → `omega_cmd` (per rotor: the motor's limit), and whether arm is `omega_cmd = 0` or the rotor row's `enabled` | Core | to confirm |
| The model: the scene's quad, with parts (`assets/scenes/hover.scene.yaml`); later Kat's model file (Core's model file) | Core | parts merged; model file proposed |
| `flow_movie --case` (Q1) and a per-view drawn-or-refused report for the tour | Rendering | to agree |

## Rulings (the lead, 2026-10-13)

1. **One recorder.** `flow_movie` gains `--case` and drives `LiveRun`. D1's frames draw the solid drone too, agreed with Rendering: LiveRun exposes the posed parts per frame.
2. **The hold** is `set_body_state()` each step at the commanded attitude, with `omega_body` the attitude's rate. The design says a held drone's IMU readings are meaningless and not shown.
3. **One rule for flight → `DomainBc`:** it moves to Physics' P2b, `resolve()` calls it, and `analysis`'s `drone_box_domain()` goes (TD-9).
4. **Layering:** `spade::analysis` links `spade::sim`, with Core reviewing the CMake.
5. **Start now, on Physics' P7a**, not on an Interface-side mapping, which would be a second rotor rule (TD-9). P7a is a small, disc-only, one-way coupler in Physics: rotor rows plus the pose give `ActuatorDisc`s, with the spin-sign rule, lifted from their `quad_case.cpp`. Full P7 (lines, the solid sampler) extends it.

Core takes "throttle → `omega_cmd`, and arm" ahead of B7. Interface sent the need: an installed per-rotor command from the model, a throttle and armed; the idle speed when armed; and the motor's own spin-up and spin-down.

## Order

1. `LiveRun` on P7a's discs, no solids: the drone's hold and commands (Core's throttle and arm), the time and frames, the runner's real path, and the tiny-grid tests.
2. P2's solids and P2b's placement and face rule, as they land; `drone_box_domain()` goes.
3. P3's loads, the history's force and moment columns, and `C_p` through `pressure_coefficient()`.
4. `flow_movie --case` with Rendering: the D1 recording, the solid drone in the frames.
5. The panel and the tour (D2, I2).
