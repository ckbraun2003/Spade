# The simulated flight computer and the autopilot links: phase 0 (O140)

Status: phase 0 design, docs only, for Oversight's review. No code and no download before its OK.
Asked for (O140, 2026-11-21): "Let's get spade the work it needs to be able to create a more accurate simulation with compute parts and the px4/ardupilot connects."
Map: Oversight's `.agents/plans/2026-11-21-full-sim-flight-computer.md`. This page is Spade's spine. The four realm sections carry the detail:

| Section | Realm | What it settles |
|---|---|---|
| [`core/plans/2026-11-21-fc-core.md`](../core/plans/2026-11-21-fc-core.md) | Core | layout, the lockstep rule, the flight computer part (board profiles, budget and measured modes), the run record |
| [`physics/plans/2026-11-21-fc-physics.md`](../physics/plans/2026-11-21-fc-physics.md) | Physics | barometer, magnetometer, noise and bias, frames, motor mapping, the X500 against PX4's, the step, ground contact |
| [`interface/plans/2026-11-21-fc-links.md`](../interface/plans/2026-11-21-fc-links.md) | Interface | the PX4 and ArduPilot links, the MAVLink encoder, Docker, the download list |
| [`test-docs/plans/2026-11-21-fc-tests.md`](../test-docs/plans/2026-11-21-fc-tests.md) | Test/Docs | tests and pass bars per phase, cadence, run-folder checks |

## One page

**What it is.** A real autopilot, unmodified, flies a Spade drone in lockstep. PX4 comes first, ArduPilot second. Spade supplies the sensors and takes the autopilot's motor outputs. A simulated companion computer above the autopilot runs the software under test on a board profile. Every run is recorded. All of it is general: any program can drive it, not only Kat.

**Where it lives.**
- `tools/fc_host` (Core): the lockstep host, board profiles, budget and measured modes, the run record.
- `tools/fc_link` (Interface): the MAVLink encoder and the PX4 and ArduPilot links.
- Both sit outside the engine (L7) and build on its public headers.
- The engine gains only two sensor kinds, a barometer and a magnetometer (Core's plumbing, Physics' models).

**How the lockstep works.**
- No new engine call and no C ABI.
- The host owns the clock, and every sensor period is a whole number of ticks.
- The tick is 500 us: it divides PX4's 4 ms, ArduPilot's 2.5 ms and 2 ms.
- PX4 has no lockstep timeout of its own, so the link carries one (5 s by default, a named refusal).

**Picks.**
- PX4 v1.17.0 (BSD-3), run in Docker (not WSL2), with our own small MAVLink encoder tested against reference frames.
- PX4 has no Gazebo-free X500 target, so Spade adds one X500 airframe file to the PX4 image. PX4's source is otherwise untouched.
- ArduPilot Copter 4.7.3 (GPL-3.0) comes in phase 5, always as a separate program. Its simulator interface takes truth and synthesises its own GPS, barometer and compass, so only Spade's IMU reaches it. That is one reason PX4 goes first.
- Budget mode comes before measured mode:
  - Budget mode is deterministic: integer-nanosecond costs per stage, declared or frozen from one calibration, with overruns counted.
  - Measured mode is a Windows Job Object process with CPU, core and memory caps. It is called "constrained", not "emulated": a PC cannot run at a board's clock.

**What phase 0 found that changes the map.**
1. PX4 likely refuses to arm without a barometer and a compass, so both are phase 1 work, not phase 4. This is to be confirmed on v1.17.0.
2. Ground contact: Spade's drone would stand 0.377 m off the ground on its contact shape. Take-off and landing need foot points, the X500's real legs. This is the least-known item of phase 1.
3. Spade's X500 differs from PX4's own x500 (mass 1.63 against 2.06 kg, inertia, thrust constant, rotor lag). Plan: validate the link on a variant that matches PX4's model, then fly Spade's measured kit model as the accurate one.
4. Spade's rotor lag (76 ms) is derived from stand-in motor inputs, not measured, for every library drone. Phase 1 adds a separate slow-down time constant (default unchanged) and grades the figure honestly. Phase 6 measures it on the real kit.
5. Repeatability is proven by replay: the recorded motor commands reproduce Spade's trace bit for bit without the autopilot. A live re-run is only as repeatable as the autopilot, so that test lists differences rather than demanding identity.
6. Flights are judged on Spade's truth, and the autopilot's estimate must sit within 0.3 m of it. A biased barometer would otherwise hold its own wrong altitude and pass.

**Downloads for phase 1 (none fetched; for Oversight's OK).**

| Item | Source | Licence | Size |
|---|---|---|---|
| PX4-Autopilot, tag v1.17.0 (d6f12ad1), with submodules | github.com/PX4/PX4-Autopilot | BSD-3 (submodules permissive; some not yet read) | a few GB, estimated |
| Build image px4io/px4-dev, pulled by recorded digest | Docker Hub | build environment; PX4's Dockerfile BSD-3 | about 1.9 GB compressed |
| pymavlink, only if the image lacks it (reference frames, offline) | PyPI | LGPLv3, tool only | under 1 MB |

After these, the PX4 build must run with the network off. Anything more it wants becomes a new list item, asked for first.

**Effort (realm estimates, on different scales).**
- Phase 1, PX4 flies the X500 with no Kat:
  - Core: 6 to 8 days;
  - Physics: six parts, the largest ground contact;
  - Interface: 4 of 5;
  - Test/Docs: about 7 branches.
  - The realms work in parallel, so phase 1 is roughly two to three weeks of elapsed work. That is an estimate, and ground contact and the first real PX4 runs are the unknowns.
- Phase 2 (budget mode with a package): about one to two weeks.
- Phase 3 (measured mode): about two weeks, and it waits on KAT's REALTIME runtime.
- Phases 4 and 5: about one to two weeks each.

**For Cameron and Oversight.**
1. The download list above.
2. Pin PX4 v1.17.0 now (recommended), or wait for v1.18 (rc2; it fixes a rare lockstep deadlock).
3. The one X500 airframe file added to the PX4 image.
4. Who signs the run report: KAT's signing, rather than a new Spade one.
5. Whether Kat counts these jobs as Spade's Linux work.
6. A real companion board for phase 6, later.

## Lead's rulings in phase 0 (any can be argued in phase 1)

- Costs v1 are a constant per call: the declared worst case or a frozen p99.
- Caps are measured on this box first.
- No host checkpoint (YAGNI): the autopilot restarts per episode, and time restarts at zero.
- "This PC" is a standalone host profile, not a vehicle part.
- Autopilot tests carry their own ctest label, outside the gate's `spade` set.
- Spade runs on the CPU backend for these tests, on the host at the gate, with the Linux image as a nightly cross-check.
