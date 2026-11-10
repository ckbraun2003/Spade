# The Smagorinsky film tests' two bands (L52): fixed before the test's first run

**Owner:** Test/Docs. **Status:** written 2026-11-10 from Physics' calibration runs (physics/wake-probe 4f2a2ae), before the opt-in film tests (physics/sgs, `SPADE_SGS_FILM=1`) have run.
**For:** RED 2 (the Smagorinsky setting damps the small scales) and RED 3 (it does not move the thrust's momentum flux).
**The rule** (`01-verification.md`): a tolerance is fixed before the first run, from the reference and the method. It is not loosened after a run without a lead decision.

## The configuration

80×128×80 cells at dx 1 cm, h 0.5 ms, the quad film's discs, inviscid, still air, 8 threads, 900 steps (0.45 s), the CPU reference. Both tests read the field at steps 700, 750, 800, 850 and 900 (t = 0.35 to 0.45 s) and nowhere else.

## The calibration

Physics ran the exact configuration at Cs 0 and Cs 0.17 (117 s and 203 s) and dumped the field at every 50 steps from 500 to 900. Raw: `C:/Users/Cameron/wake-out/calibration.json`. Flux is ρ Σv² dA over the whole horizontal plane 0.30 m under the hubs; R is flux over 4T = 5.158 N.

| Cs | R per dump, steps 500 to 900 | R, five-dump mean (700 to 900) | ke_hi, five-dump mean |
|---|---|---|---|
| 0 | 1.021 1.044 1.000 1.068 0.992 1.004 1.013 1.042 0.994 | 1.009 | 0.0711 |
| 0.17 | 0.996 0.990 1.037 1.033 0.976 1.034 0.985 1.026 1.001 | 1.004 | 0.0102 |

Over all 18 dumps R has mean 1.014 and sd 0.025, from 0.976 to 1.068.

## RED 3: the momentum flux

**R = the mean of flux over steps 700, 750, 800, 850 and 900, divided by 4T = 5.158 N, satisfies 0.90 ≤ R ≤ 1.10 at Cs 0.17, and at Cs 0 as well.** The same band holds at both, so Cs does not move it. A single step is never tested.

| Term | Value | Built from |
|---|---|---|
| The centre | 1 | momentum theory: the flux through a plane below a steady wake equals the thrust. Not taken from the solver |
| The measured mean's offset from 1 | 0.014 | the 18 dumps. Unsteadiness and start-up, small at this window |
| The spread of a mean of five dumps | 0.075 | 3 × the dump sd 0.025. The sd of a five-dump mean lies between 0.025/√5 and 0.025 (the dumps are correlated), and the top is taken |
| The sum | 0.089 | rounded up to two figures: **0.10** |

- **The measured five-dump means** (1.009, 1.004) sit at least 0.09 inside each edge. Neither dump nor mean touches an edge.
- **What it catches:** a thrust or momentum error of 10% or more, such as a wrong disc force or a lost wall. It does not catch a few percent: it is a sanity band on a transient, not an accuracy test.
- **It is for this window only.** Physics' earlier probes at 0.45 to 0.6 s show single dumps from 0.93 to 1.06 and a 1.2 s run's single step at 1.40. The later wake is a different regime, so the test never reads past step 900 and never tests a step.

## RED 2: the small scales

**ke_hi at Cs 0.17 is below half of ke_hi at Cs 0, each the mean of ke_hi over steps 700, 750, 800, 850 and 900** (the same box, the same window). ke_hi is the share of kinetic energy above k = π/(2 dx), on the box ±0.28 m across the discs' centre, from 0.12 m under the hubs to 0.20 m above the bottom, Hann-windowed in each axis.

- **The half is the lead's.** The measured ratio of the means is 0.143, and step 900 alone gives 0.132, so a single step would also hold. The five-dump mean is kept for the same reason as RED 3: one definition for both tests, and no dependence on one step's swing.
- **The test also asserts** that both ke_hi means are finite and that the Cs 0 mean is above zero. Otherwise a field of NaN or of zeros satisfies "below half" (`TD-14`, NaN).

## Not covered

- **A different configuration needs a new calibration:** another grid, dx, h, disc set, thread count, window or Cs. Not a wider band.
- **Run to run.** The runs are the CPU reference at 8 threads. The test's first run records its R and ke_hi means beside 1.009 / 1.004 and 0.0711 / 0.0102, with the thread count.
- **The cost.** About 2 to 4 minutes a run in release, so each case runs opt-in, `SPADE_SGS_FILM=1`, outside both gates.
