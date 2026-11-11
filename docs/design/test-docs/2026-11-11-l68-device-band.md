# L68, contact tumbling: the CPU-against-device band, fixed before the first comparison

**Owner:** Test/Docs. **Status:** written 2026-11-11, before any device comparison. Core's proposal (the message of the same day) is taken term by term: kept where a term holds, changed where it does not.
**For:** the device tests of L68 (`docs/design/physics/plans/2026-11-11-l68-contact-tumbling.md`, "GPU plan"): a shaped body dropped onto the world SDF bounces, tumbles and settles on a face, on the CPU and on the device.
**The rule** (`01-verification.md`): a tolerance is fixed before the first run, from the method, never from our solver's error. It is not loosened after a run without a lead decision.

## Why outcomes, and why a CPU ensemble

A tumble is a chain of impacts. Each impact amplifies a rounding-sized difference, so after three or four of them a difference of a few ulps in the SDF walk can change *which corner* hits next. Two things follow.

- **The face can flip** if the scenario sits near the edge between two settling faces.
- **The yaw and the lateral travel are history, not physics.** A cube settled on a face has its yaw set by every impact before; two runs agreeing on the face can differ in yaw by tens of degrees and in position by centimetres. So "the angle between the two final orientations ≤ 2°" and "settled centre within 0.010 m" fail for reasons that are not a defect.

The device is one more rounding-level perturbation of the CPU (per Core's plan its gap is the SDF walk only: the device's divides and square roots are `div32_ftz` and `sqrt32_ftz`). So the band is **the CPU's own sensitivity to a rounding-sized perturbation**, measured on the CPU alone before any device run. The existing static-contact band is not that: it is for a body *at rest* (`tests/test_gpu_contact_pass.cpp`: the device within 1e-4 m of the CPU, moved ≤ 0.010 m, lowest point in [−0.002, 0.005] m).

## The scenario and the ensemble

The scenario is Core's: a 0.2 m cube (half extent 0.1, mass 1 kg, μ 0.5, e 0.3), dropped from 1 m at a generic tilt onto a plane floor, 1 kHz, 6 s.

**The CPU ensemble (run before the first device comparison, CPU only):** nine CPU runs, the nominal and eight perturbed starts: (height, tilt) offsets of (±1e-6 m, 0), (0, ±1e-6 rad) and (±1e-6 m, ±1e-6 rad), 2 + 2 + 4. 1e-6 is about ten times the SDF walk's rounding (an fp32 ulp at 1 m is 1.2e-7), so the ensemble is wider than the device's gap, not narrower.

**The scenario must pass first:** all nine runs settle (|v| and |ω| under Core's threshold for the last half second) on the **same face**. If not, the tilt sits near a basin edge and the scenario is not used for the device: the next tilt in a list written in the test beforehand is tried (20°, 30°, 40° about (1, 2, 3)/√14, in that order), and the record says which was used. A face that is not robust on the CPU cannot be asserted on the device.

## The pinned outcomes

| Outcome | Compared | Band | Built from |
|---|---|---|---|
| Settled face (index of the body axis closest to world up) | device against the CPU | **identical** | Core's (1), kept. It holds because the scenario passed the ensemble's same-face test |
| Settled tilt: the angle between that axis and world up | each run, on its own | **≤ 1°** on the CPU and on the device | Core's REDs already say "up axis within 1°" for the CPU. Replaces Core's (3) for tilt: it is the quantity that does not depend on history |
| Settled height of the centre above the floor | device against the CPU | **within 1e-4 m** of each other, and each in [−0.002, 0.005] m of the half extent | the static-contact band's two numbers (a settled body is a body at rest). The height does not depend on the tumble |
| Settled lateral position (horizontal) | device against the CPU | **max(0.010 m, 2 × the ensemble's largest pairwise horizontal spread), rounded up to 0.01 m** | Core's 0.010 m as the floor; the 2 is for the ensemble's nine samples not reaching the extreme (nine samples cover about 80% of a range's next draw, and I take two for margin) |
| Settled yaw about the vertical: the difference of the two final headings about the vertical (the face's in-plane axis) | device against the CPU | **max(2°, 2 × the ensemble's largest pairwise yaw spread), rounded up to 1°** | Core's 2° as the floor, the same factor of 2 |
| Bounce count (impacts with approach speed above the rest speed) | device against the CPU | **max(1, the ensemble's largest pairwise difference)** | Core's (4), kept as the floor. A rocking cube near the rest speed counts extra small impacts, so the ensemble may widen it |
| CPU against itself, device against itself | two runs each | **memcmp-equal** | Core's (5), kept: the project's rule |

- **The constants:** the test carries the floors now (0.010 m, 2°, 1) and the ensemble's results once Core has run it on the CPU. The ensemble's spread (the pairwise maxima) goes in the record beside each constant, so a reader sees where the band came from.
- **A guard that the sensitivity has not grown:** the test re-runs the CPU ensemble and asserts each spread is at most half its pinned band. A change that makes the tumble more chaotic then fails on the CPU, before it can widen a device band by accident.
- **Both runs must be settled** at 6 s. A run that is not settled is reported, not compared.
- **A different body, shape, tilt, material or substep** needs its own ensemble, not these constants.

## Not covered

- **A tumble's trajectory** is never compared: only the outcomes above, as Core's plan says.
- **The ensemble's size and factor are judgments:** nine runs and the factor of 2. A device comparison that lands within 10% of a band edge is read as a finding and sent to the lead, not accepted or refit.
