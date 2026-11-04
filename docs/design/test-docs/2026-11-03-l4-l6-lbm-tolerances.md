# L4 to L6, the LBM's nests, coupling and resolved rotors: tolerances fixed before the first run

**Owner:** Test/Docs. **Status:** draft for the lead and Physics, 2026-11-03. None of these cases has run. Physics reviews it with L2 and L3's note.
**For:** L4 (nests and the one-way coupling), L5 (Caradonna–Tung) and L6 (the drone's rotors, blade-resolved) of Physics' P5 plan (`../physics/plans/2026-11-03-p5-blade-resolved-plan.md`), in the shape of L1's and L2–L3's notes (`2026-11-03-l1-lbm-tolerances.md`, `2026-11-03-l2-l3-lbm-tolerances.md`).
**The rule** (`01-verification.md`, "Validation against published data"): a tolerance is fixed before the first run, from the reference and the method, never from our solver's error. It is not loosened after a run without a lead decision recorded with its reason. A miss is recorded as a miss.

Lattice units are as in L1's note. Bands are built as in L2–L3's note: term by term, each a worst case, summed and rounded up to two figures.

## What differs from L1 to L3, and why

**1. A nest interface is not exactly conservative.** A uniform lattice conserves mass and momentum to rounding, since streaming and collision each conserve both.
- **At a 2:1 interface** (Lagrava et al. 2012, J. Comput. Phys. 231), the fine grid's incoming populations are interpolated from the coarse grid in space and time. The coarse grid's are restricted from the fine. Neither step conserves exactly.
- **A sum is bounded by the interpolation's own error.** A smooth field of wavenumber k, interpolated at order p from a grid of spacing dx_c, is off by about (k dx_c)^p / 8 for p = 2. That error, carried by the flux through the interface, bounds what an interface can leak.
- **The bands below take p = 2,** the lowest order Lagrava's scheme uses. If Physics names a higher order, the bands stay and the margin grows (open item 1).

**2. The non-equilibrium part must be carried across.** The populations' non-equilibrium part, f_neq, carries the stress. Across a 2:1 interface with acoustic scaling it is rescaled, by the ratio of the levels' relaxation times and steps.
- **A uniform flow has f_neq = 0,** so it cannot catch a wrong rescale.
- **So L4 adds a sheared flow:** planar Couette, whose exact linear profile carries a constant f_neq across the interface.

**3. The one-way coupling imposes the NS grid's velocity, not its pressure.**
- **What the nest takes:** its outermost faces take the NS velocity, interpolated in space and in time between NS steps. The nest's density is its own.
- **Where the mismatch can be exact:** where both tiers hold the same exact solution (uniform flow, planar Couette), the nest must equal the NS field to rounding. There the coupling's mismatch is a band.
- **Where it can't:** in a wake, the tiers differ in physics (the LBM is viscous and weakly compressible; C8's NS run is inviscid), so the mismatch is reported, not banded.
- **f_neq at the coupled faces:** the coupled faces need an f_neq. If they set equilibrium only, the boundary's stress is wrong and a shear slips by O(dx). The Couette band assumes f_neq is rebuilt from the NS field's strain (open item 2).

## L4a. Uniform flow through a nest, coupled to the NS grid

**Reference.** u = U and ρ = ρ0 everywhere. Uniform flow is an exact discrete solution of both tiers.

**Configuration.**
- **The NS box:** periodic, 32³ at dx, holding a uniform U at 30° to every axis, so no component is zero.
- **The LBM nest:** level 0 is a 16³ box at the NS's dx, coupled one way. Level 1 is a 2:1 box of 8³ coarse cells (16³ nodes) at its centre.
- **The flow:** u_lat 0.05 on level 0 (the levels share u_lat under acoustic scaling), ν_lat 1e-3 on level 0, WALE on at C_w 0.5.
- **The run:** long enough for the flow to cross level 0 twice: 2 × 16/0.05 = 640 level-0 steps.
- **If Physics' L4 moves a hub-fixed nest,** the same run with the nest moving at a constant W through the field is the second row. The field must stay U.

| Measure | Tolerance | Built from |
|---|---|---|
| max \|u − U\| / U over every node of both levels, at the last step | ≤ 1e-4 | Rounding only. One population's fp32 rounding is about 6e-8 of ρ0, so a node's velocity is off by about 2e-7 absolute, 4e-6 of U at u_lat 0.05. Sound carries it off and damps it, and 1e-4 is 25 times that |
| max \|ρ/ρ0 − 1\| over the same nodes | ≤ 1e-5 | Rounding only. Uniform flow moves no mass |
| The coupled faces' boundary values against the NS field, interpolated in space and time | exact | The interpolation of a uniform field is exact. A unit test applies the formula to the two NS snapshots and compares bit for bit |

## L4b. Planar Couette through a nest, coupled to the NS grid

**Reference.** The linear profile u_x = S (y − y₀), exact for both tiers' viscous steps.
- **The NS tier holds it to rounding:** the solids note's planar Couette, `AirflowSolidsStep.PlanarCouetteFlowHoldsItsLinearProfile`, 5.6e-8 of U.
- **The LBM holds it:** a linear field has no second derivative, so the second-order lattice's error term vanishes.

**Configuration.**
- **The NS box:** an 8 × 32 × 8 slab, periodic in x and z, its plates S2-planar's: wall links, plates a quarter cell off the face planes, the lower still and the upper at U.
- **The nest:** 8 × 16 × 8 cells centred in the gap. Its y-faces cross the shear, and its x- and z-faces lie along it. Level 1 is 4 × 8 × 4 coarse cells at its centre.
- **The flow:** u_lat 0.05 at U on level 0, ν_lat 0.05 on level 0 (τ 0.65), WALE off. A linear shear has a non-zero WALE operator only through rounding, and switching it off removes it.
- **The run:** start from the exact profile in both tiers, and run to the viscous time across the nest, (16)²/ν = 5120 level-0 steps.

| Measure | Tolerance | Built from |
|---|---|---|
| max \|u − u_exact\| / U over every node of both levels | ≤ 1e-4 | As L4a's: rounding only. A linear field is exact through second-order interpolation in space, and steady in time |
| **The negative control:** the same run with the f_neq rescale at the interface set to 1 | must exceed 1e-4 | A wrong stress across the interface puts a kink in the profile. The test asserts the control misses, as S2's planar Couette asserts its control does |
| **The second control:** the coupled faces at equilibrium only, without f_neq | report only | Shows what open item 2 costs. If Physics' coupling sets equilibrium only, this becomes the main row and the band is rebuilt from the slip |

## L4c. The Taylor–Green mode across a nest: what an interface costs

**Reference.** C0a's 2D mode, its exact decay K = K₀ e^(−4νt/L²) (Taylor 1923). With periodic faces and a mode whose net momentum is zero, mass and momentum are conserved exactly.

**Configuration.**
- **The coarse grid:** C0a's at N 64, u_lat 0.05, ν_lat 0.01, WALE off, ρ from p′, read at n = 2594 steps.
- **The nest:** a 2:1 level of 32 × 32 coarse cells (64 × 64 × 2 nodes) at the box's centre, so its interface cuts the mode's cells at every phase. Pure LBM, with no NS coupling.

| Measure | Tolerance | Built from |
|---|---|---|
| \|ν_eff / ν − 1\|, from K at n | ≤ 0.030 | C0a's band at N 64, 0.027, which the coarse grid sets, since the nest is finer. Plus an interface allowance of 0.003. A static estimate gives 3e-4: the interpolation's (k dx_c)²/8 = 0.0012, over the nest's quarter of the box. The allowance is ten times that, for the fine-to-coarse feedback the static estimate leaves out. That factor is a judgment, stated as one |
| \|Δm / m\|, the total mass's change over the run (fine nodes weighted 1/8) | ≤ 5e-3 | The interface's flux error, at most ρ0 U (k dx_c)²/8 a step on each of its 128 coarse faces, summed over 2594 steps with no cancellation: 4.9e-3 of the box's mass. A correct scheme's signs cancel far below that. A population left unset at the interface leaks about w ρ0 a face a step: about 3 of the box's mass, three orders past the band |
| \|ΔP\| / (ρ0 U N²), the total momentum's change | ≤ 5e-3 | The same bound for momentum: the flux error ρ0 U² (k dx_c)²/8 a face a step |
| The mass's and momentum's changes, signed, every 100 steps | report only | The leak's history, for the record |

## L4d. C8's disc inside a nest

**Reference.** C8's: momentum theory's `v_h (1 + 2/√5)` = 7.5777 m/s for C8's disc (T 1.2315 N, R 0.1 m), as the mean downwash over the nodes within 0.5R of the axis, 2R below (C8 since 2026-10-22, `2026-10-22-c8-spread.md`).

**The plan's "unchanged by a nest".** The NS-only run, 7.8893 m/s (`c8-actuator-disc.md`, the eight runs' mean), is our solver's output, not a reference. So the nest's run is judged against momentum theory with C8's band, and its difference from the NS-only run is printed, report only. With one-way coupling the NS grid's own run is unchanged by construction, so only the nest's answer is new.

**Configuration.**
- **The NS run:** C8's run 0: 64 × 128 × 64 at 1 cm, inviscid, h 1.25 ms, 768 steps.
- **The nest's levels:**
  - Level 0 at the NS's 1 cm, 30 × 40 × 30 cells, from 1R above the disc to 3R below it, ±1.5R across.
  - Level 1 at 5 mm (R/dx 20), 48 × 60 × 48 nodes, from 0.5R above to 2.5R below, ±1.2R across.
- **The flow:** u_lat 0.05 at 8 m/s, C8's far-wake scale, on both levels. That makes level 0's step 62.5 µs, so 20 level-0 steps per NS step: 15 360 level-0 and 30 720 level-1 steps. ν_lat from air's 1.5e-5 m²/s: 9.4e-6 on level 0. WALE on at C_w 0.5.
- **The disc:** both tiers receive it (spec §1.3). The nest gets the same 5 × 32 points and Peskin kernel by Guo's forcing, so its injected momentum is J to rounding, a unit check.
- **The measure's window:** C8's own, steps 385 to 768 in NS steps.

| Measure | Tolerance | Built from |
|---|---|---|
| The mean downwash over level 1's nodes within 0.5R of the axis, 2R below, over C8's window | within 10% of 7.5777 m/s | C8's band (plan A Task 11, spec §11.2), unchanged. The Mach term, Ma² = 0.0075, and the interface terms of L4c are each under a tenth of it |
| The same, minus the NS-only run's 7.8893 m/s | report only | The tiers' difference, not a reference |
| The coupling's mismatch: the rms of \|u_nest − u_NS\| / v_h over level 0's first interior layer | report only | The tiers differ in physics there (viscous against inviscid, weakly compressible against incompressible), so no exact solution bounds it. L4a and L4b band it where one does |
| The nest's momentum budget: the disc's J against the momentum flux through level 0's boundary plus the change inside, over the window | report only | It closes to the interfaces' leak. Its band follows once Physics names the interpolation orders (open item 1), by L4c's method |
| The disc's injected momentum per step in the nest against J | relative 1e-6 | Peskin's kernel is a partition of unity per axis (spec §3.1), so it is exact up to rounding |

## L4e. Exact checks and cost

- **A nest fixed to the world and one fixed to a still hub at the same place, bit-equal.**
- **CPU = GPU to the bit** on every L4 case (the plan's rule).
- **The cost:** ns a cell-step per level, and the interfaces' share of the step, on the GPU. Report only (the plan: "the cost, measured").

## L5. Caradonna–Tung (P9's record): pointed to, and not yet banded

**The data** are P9's:
- `tests/data/airflow/caradonna1981_rotor.hpp`: Table 17's five stations, their C_L, the condition and figure 9's tip vortex path;
- its row in `tests/data/airflow/README.md`;
- the reading's record `2026-11-03-p9-caradonna-review.md`.

**Its bands are not fixed anywhere yet.** P9's plan (`2026-11-03-p9-rotor-data-plan.md`, "What P9 delivers") sets **no tolerance**: "D3's checks fix those before D3's first run". The data file's README row says the same. P5's L5 names "their bands" without giving them. So L5 needs a note of its own before its first run (open item 5). It would cover:
- C_p at the five stations against Table 17;
- C_L against the printed values;
- the tip vortex's descent and contraction against figure 9's digitized path, within its ±4° and ±0.01;
- C_T against the two printed values.

Its compressibility caveat comes from L1's map: at u_lat 0.254 its tip Mach is the flow's own; below that, P9's 10% caveat.

## L6. The drone's rotors, blade-resolved

**What L6 can and cannot claim.** The plan compares L6 with "what the disc tier and the P9-validated CT give". Neither is a reference for a drone's prop:
- **The disc tier** is our own solver.
- **Caradonna–Tung** validates the method on its own rotor, at its own resolution.
- **The drone's props have no complete reference.** UIUC measured their static C_T, C_P and blade geometry (c/R and β), but no airfoil sections. The library plan says "the airfoils are unknown for every one of these props" (P-b, graded `assumed`). A resolved blade with an assumed section can't be validated against the measured C_T to better than the section's own effect. A thickness floor, if Cameron chooses one, changes the blade on purpose.

So L6 is a demonstration with stated consistency checks, not a validation. Its record says so, and every thrust comparison is report only.

**When L6 may carry L5's accuracy.** L5's validated errors transfer to L6 only at L5's configuration: at least 32 cells across the chord, the same wall rule, the same u_lat rule (L1's map) and the same nest levels. The record states whether L6 met each, and quotes L5's achieved errors beside its own numbers. A run below any of them says, in its `config:` line, that L5's validation does not cover it.

**Configuration.** As the plan:
- rotor-attached parts from the builder;
- posed by P11's ψ;
- at the resolution Cameron chooses after L5 (R2, or a stated thickness floor);
- with the body in the NS grid around them;
- at the drone's hover rpm, run for at least 5 revolutions, the measures over the last 3.

| Measure | Tolerance | Built from |
|---|---|---|
| The resolved blade's SDF at the step's ψ against `posed_parts()`'s drawn blade at the same ψ | exact | P11's convention (the demonstration plan, P4/P11): the drawn blade and the resolved blade are one pose. A unit test, bit for bit |
| Each rotor's mean thrust and torque from the blade loads, over C_T and C_P from UIUC's static table at the run's rpm | report only | Why not banded: the airfoil is assumed. For a prop whose table is scaled (the Phantom-class's APC 10x5 at 9.4 in, derived) the ratio says so. With a thickness floor, the floor is named |
| The same over the rigid-body path's k_T ω², the thrust the drone flies with | report only | How the resolved tier differs from the flight model |
| The four rotors' mean thrusts, each against their mean | within 10% | C8q's symmetry band (`tests/data/airflow/README.md`, C8q), carried: a symmetric quad at hover, each rotor's difference set by its neighbours and the body |
| The mean downwash 2R below each hub against momentum theory from that rotor's resolved thrust; the slipstream radius at z = −2R against Landgrebe's 0.78 | report only | A real rotor's slipstream isn't uniform, and Landgrebe's figure is for helicopter rotors. Spec §11.2 compares 0.78 "only in the blade-resolved case" and sets no band. A band needs Landgrebe's data read (fetch F4, below) |
| The thrust against the momentum budget through a control surface in the nest | report only | Closes to the nests' leak, as L4d's. Banded with it |
| CPU = GPU to the bit | exact | The plan's rule |

## The records

The shape is `01-verification.md`'s. Each opening line ends with `LBM`, and `tolerance:` lines cite `test-docs/2026-11-03-l4-l6-lbm-tolerances.md, fixed before the first run`.

| Record | File | Opening line | Measures, in order |
|---|---|---|---|
| L4a, L4b | `l4-nests.md` (new) | `L4a uniform flow through a nest, NS-coupled, LBM` and `L4b planar Couette through a nest, NS-coupled, LBM` | as their tables |
| L4c | `c0-taylor-green.md` | `C0 Taylor-Green, the 2D mode's exact decay, 64x64x1, a 2:1 nest of 32x32, u_lat 0.05, LBM` | ν_eff; Δm; ΔP; the history, report only |
| L4d | `c8-actuator-disc.md` | `C8 actuator-disc far wake, in a 2-level LBM nest, NS-coupled, LBM` | the downwash; the difference; the mismatch; the budget; the injection |
| L6 | `l6-resolved-rotors.md` (new) | `L6 <drone>, blade-resolved, <resolution or floor>, LBM` | as its table, the `config:` line naming which of L5's conditions it meets |

## Where they run

- **The gate:** L4a and L4b (16³ and 8 × 16 × 8 boxes, under 6000 steps) may join it if each times under 30 s in debug, under ctest at `-j 1`. The unit checks are gate tests.
- **Opt-in:** L4c, L4d and L6, with `SPADE_AIRFLOW_VALIDATION=1`, in release. L6 on the GPU.

## Fetches

- **None for L4.** Every reference is exact or C8's existing one. Lagrava et al. (2012) is the method, Physics' to read.
- **F4, for L6's contraction, only if the lead wants it banded.** Landgrebe, A. J. (1971), "An analytical and experimental investigation of helicopter rotor hover performance and wake geometry characteristics", USAAMRDL Technical Report 71-24 (the number from memory, to be confirmed when it is found), the report behind the 1972 J. Am. Helicopter Soc. paper. It would be read from DTIC, if it is public there. Until it is read, the contraction is report only.
- **UIUC's measurement uncertainty** would matter only if an L6 thrust row were banded. None is.

## Open, for Physics before the first run

1. **The interface's interpolation orders,** in space and in time, coarse to fine and fine to coarse. L4c's bounds take p = 2. Name the orders, and L4d's and L6's momentum budgets get bands by the same method.
2. **f_neq at the coupled faces.** Rebuilt from the NS field's strain, or equilibrium only? L4b's band assumes the first.
3. **A hub-fixed nest that moves.** If L4 moves nests with a hub, L4a's second row applies. If nests only sit still, drop it.
4. **The disc in the nest by Guo's forcing,** with the NS tier's points and kernel, so both tiers receive the same source (spec §1.3). Confirm.
5. **L5's bands** (above): not fixed by P9 or P5. Test/Docs can write them in this shape, if the lead asks.
6. **L6's references:** the props' airfoils are unknown, so L6's thrust is report only. A prop with a published section, or one measured for the purpose, would be needed to validate it.
