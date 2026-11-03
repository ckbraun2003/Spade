# P5, blade-resolved rotors (D3): the tier decision, measured, and the plan

**Status:** draft for the lead (2026-11-03). The tier decision is Cameron's (the demonstration plan's P5: "one decision to bring back with measurements").
**Asks this answers:** the demonstration plan (`2026-10-12-airflow-drone-demonstration-plan.md`), D3 and P5. Cameron's geometry push (backlog, 2026-11-03): "D3 starts with P5's plan and its measured NS-with-nests or LBM decision".

## The question

P5 resolves a rotor's blades as a moving solid in a nested fine grid, with a wall model. It can be built in one of two tiers:
- **the real-time NS tier with nests:** plan A/B's solver, which is built, bit-exact on the device, and carries solids;
- **the LBM tier:** the spec's high-fidelity tier (§1.6, §11.5), not yet built. Its bounce-back takes a moving wall link by link.

## Recommendation: the LBM tier

Regularized D3Q27, recursive to third order, with WALE in the relaxation time. It runs in 2:1 nests by acoustic scaling, inside the NS grid, coupled one way (§1.3: the NS grid gives the nest its boundary). The measured reasons:

1. **3 to 4 times less device time a revolution** at the same cells (below). It is about 10 times less if Caradonna–Tung runs at its true tip Mach, which only the weakly compressible tier can represent.
2. **About half the memory a cell:** 112 B against 205 B with solids. Caradonna–Tung's three-level nest fits the 4 GiB cap in LBM (3.2 GiB) and not in NS (5.9 GiB).
3. **Moving walls stay local.** LBM recomputes the links near the blade from its posed SDF each step, inside the blade's box. NS rebuilds the whole solid field each step. Today that is a host function (`set_solids()`, 0.26 s a step at 1.7 M cells on the CPU, M2), against a 30 ms device step, so the NS route needs a GPU builder first.
4. **The bits are easier.** An LBM step is per cell, with no reduction in the stepping. Only the blade loads are summed, in `reduce.hpp`'s fixed order. So CPU = GPU to the bit follows the NS tier's own rules with less to hold.

**What LBM costs:** a new solver tier: its CPU reference and GPU twin, walls (static, then moving), a wall model, nests, and the coupling from the NS grid. The NS route has comparable new work:
- nests as one composite multigrid with refluxing, the hardest single piece of either route;
- a GPU solids builder for moving parts;
- tangential no-slip;
- a wall model.

The engineering is of a size; the run cost and the memory are not.

## The measurements

All on the RTX 3060 Ti, driver 572.83, scratch branch `physics/p5-measure` (never merged), logs in its `.superpowers/sdd/2026-11-03-p5/`.

**M1. The LBM step on this device, through the airflow chain runner** (`airflow_lbm_spike.slang`). One D3Q27 step a dispatch on a periodic box, fp32, two copies with pull streaming. That moves 216 B a cell, the same traffic as the spec's in-place esoteric pull. Mass held to 1e-7, every value finite.

| Collision | 96³ | 128³ | 160³ | Bandwidth |
|---|---|---|---|---|
| BGK | 0.55 ns a cell | 0.55 | 0.54–0.58 | 373–399 GB/s |
| Regularized (2nd order) | 0.58 | 0.57–0.61 | 0.62–0.63 | 343–378 GB/s |
| Recursive-regularized (3rd order) | 0.59–0.72 | 0.58–0.62 | 0.59 | 298–373 GB/s |

That is 77–89% of the card's 448 GB/s: the spec's "80%, unconfirmed" (§1.6, §4.2) is confirmed for this kernel. The collision's arithmetic is nearly free, because the step is bound by memory. Workgroup sizes 64 and 128 differ by under 10%, except the third-order collision at 96³ (23%).

**M2. The NS tier carrying a moving blade, on the CPU reference** (`P5Measure.NsTierWithAMovingBlade`). Two box blades, 3 cells thick and 12 across the chord, on a quad's rotor 0, at 3 mm (120³). They turn so the tip moves one cell a step (30 m/s, h = 0.1 ms), with the solids rebuilt every step by `set_solids()`:

| | Builder a build | Fluid step | CG iterations | Refused |
|---|---|---|---|---|
| Blades still (built once) | 0.232 s | 4.80 s | 8 every step | 0 |
| Blades moving (rebuilt every step) | 0.264 s | 4.99 s | 8 every step | 0 of 24 |

The moving solid costs the CG nothing at this resolution, and the safeguard refuses no pose. What it costs is the builder, which has no device twin. The NS tier's bytes a cell are 104 for the fluid and 205 with solids (`fluid_bytes()`, `solid_bytes()`).

**The NS tier's device step, measured:** D1 with its solids, 7.14 M cells, takes 130 to 138 ms a step with Rendering's gauge tiles (`d1-hover.md`'s GPU block, 2026-11-03). That is **18.2 ns a cell a step**, 29 to 33 times M1's LBM step. The NS step moves about 4.3 KB a cell (the GPU plan's count): mostly the solids CG's nine V-cycle preconditions.

## Device time a revolution

Each tier's step is set by a different limit:
- **NS:** the blade must not jump a cell, so the tip moves at most one cell a step: 2πR/dx steps a revolution. All nest levels share the step (§11.4).
- **LBM:** the step is bound to the lattice's sound speed. At u_lat = 0.1 (Mach 0.17 on the lattice; §1.6's limit) that is 10 times as many steps. Coarser levels step half as often each (acoustic scaling).

The model:
- The innermost nest is a slab of 2.2R × 2.2R × 0.3R around the disc. Each 2:1 level holds the same number of cells.
- LBM carries 30% for walls, nests and the coupling, an estimate the plan's tasks measure.
- NS's builder and refluxing are left out, which favours NS.

| Case | Cells | Memory, NS / LBM | Steps a revolution, NS / LBM | Minutes a revolution, NS / LBM |
|---|---|---|---|---|
| Caradonna–Tung (R 1.143 m, chord 0.1905 m at 32 cells, dx 5.95 mm; tip 149.6 m/s), 3 levels | 30.8 M | 5.87 / 3.22 GiB | 1206 / 12 064 | 11.3 / 2.8 (×4.0) |
| The same at u_lat 0.25 (its true tip Mach 0.44 on the lattice) | 30.8 M | 5.87 / 3.22 GiB | 1206 / 4825 | 11.3 / 1.1 (×10) |
| Caradonna–Tung, 2 levels | 20.6 M | 3.92 / 2.14 GiB | 1206 / 12 064 | 7.5 / 2.4 (×3.1) |
| The Phantom's prop at 0.5 mm (R 0.1194 m, tip 67.8 m/s), slab 0.2R, 2 levels | 26.4 M | 5.02 / 2.75 GiB | 1500 / 15 004 | 12.0 / 3.9 (×3.1) |

A recorded run of a few revolutions is therefore minutes in LBM and tens of minutes in NS, within the cap only in LBM for Caradonna–Tung's three levels.

## What the measurements do not settle (said now)

- **Accuracy is not measured here.** Both tiers are established for resolved rotors. The tier is chosen on cost, memory and moving walls. Caradonna–Tung (P9) then validates whichever is built, and a miss there is reported, not tuned away.
- **LBM's 30% overhead is an estimate.** Tasks L2 and L4 measure the walls' and the nests' cost.
- **u_lat 0.25 needs the recursive-regularized collision to stay stable at lattice Mach 0.44.** L1 measures that before Caradonna–Tung counts on it. Otherwise it runs at 0.1, and NS's incompressible caveat (about 10% at tip Mach 0.44, P9) applies to both.
- **The Phantom's real props are beyond either tier on this card.** Their declared thinnest feature is 0.39 mm. Two cells across it needs 0.2 mm: 206 M cells in the fine slab alone (43 GiB in LBM). D3's second half (the drone's rotors) needs one of:
  - R2's adaptive blocks near the blades (§11.4);
  - a thickness floor for resolved props, drawn and labelled as such (for example 1 mm at 0.5 mm cells).
  That is a decision for Cameron after Caradonna–Tung, not now.

## The plan (the LBM route)

Every task builds a CPU reference first and a GPU twin equal to it to the bit on the device of record. Each passes mutations named in its tests and the gcc check, and lands as a branch for the lead's review. The determinism rules are the airflow tier's: fixed-order reductions, NaN preservation, NoContraction, no libm transcendental, and `core/fp32_math` for any trigonometry.

- **L1. The lattice.**
  - D3Q27 with the recursive-regularized collision to third order and WALE in τ, fp32, one copy streamed in place (esoteric pull), with periodic and open faces: equilibrium inflow, zero-gradient outflow.
  - Validated on C0 (Taylor–Green, its dissipation on record) and C1 (the cavity at Re 100 and 400 against Ghia), as plan A was.
  - The stability map in lattice Mach (0.1 to 0.3) at τ near 0.5, measured, which decides u_lat for Caradonna–Tung.
- **L2. Walls.**
  - Halfway bounce-back from a part's SDF, then interpolated (Bouzidi, the wall's distance from φ), then moving, with the wall-momentum term. Every link's wall is rebuilt from the posed SDF each step inside the part's box.
  - The blade loads by momentum exchange, summed in `reduce.hpp`'s order.
  - Tests:
    - S1's sphere (C_p, against the NS tier's record);
    - a rotating plate's torque against a fine-grid reference;
    - a still part and a part moving with zero velocity bit-equal;
    - the walls' cost, measured.
- **L3. The wall model.** Spalding's law through the slip velocity of Malaspinas and Sagaut (2014). It solves for u_τ with a fixed number of Newton iterations (never to a tolerance) and is tested on C5's turbulent case.
- **L4. Nests and the coupling.**
  - 2:1 levels by acoustic scaling (Lagrava et al. 2012 at the interfaces), fixed to the world or to a rotor's hub.
  - The outermost nest's faces take the NS grid's velocity, interpolated in space and time (one way, §1.3).
  - Tests:
    - a uniform flow through a nest stays uniform to fp32 rounding;
    - the actuator disc's far wake (C8) is unchanged by a nest around the disc;
    - the cost, measured.
- **L5. One rotor resolved: Caradonna–Tung** (P9's record, with Test/Docs).
  - Collective 8°, tip Mach 0.439, at 32 cells across the chord.
  - Its blade pressure at the measured stations, and the tip vortex's position against age, with their bands.
  - Recorded over a few revolutions (offline; the run line names its real-time ratio).
- **L6. The drone's rotors from the builder's parts.** The props are rotor-attached parts, posed by P11's ψ, at the resolution Cameron chooses after L5: R2, or a stated thickness floor. They are recorded with the body in the NS grid around them.

**If Cameron chooses the NS tier with nests instead:**
- N1, a GPU solids builder that moves parts each step (M2's 0.26 s on the host, ported);
- N2, tangential no-slip and the wall model on solid faces;
- N3, nests as one composite multigrid with conservative refluxing (§11.4);
- then L5 and L6 as above.

The cost table above is what that choice accepts.

## P9 (rotor data) beside it

Physics reads Caradonna & Tung (1981, NASA TM 81232) for L5. The record holds the hover condition at tip Mach 0.439 and 8° collective, the blade pressure coefficients at the measured spanwise stations, and the tip vortex's positions, each with its source page and the 10% compressibility caveat. Test/Docs reviews it as it did Ghia. The UIUC propeller data serve the actuator line (P4), not P5. P9 is needed whichever tier is chosen, so it goes ahead now.
