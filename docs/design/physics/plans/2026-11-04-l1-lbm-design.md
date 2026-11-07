# L1, the LBM lattice: design

**Status:** draft for the lead's review (2026-11-04), before any code past the lattice core. **Owner:** Physics.
**For:** L1 of the P5 plan (`2026-11-03-p5-blade-resolved-plan.md`, "The plan (the LBM route)"). Bands, cases and records are Test/Docs' (`../../test-docs/2026-11-03-l1-lbm-tolerances.md`); this file fixes how the solver is built. The collision operator is section 1: the lead asked for it named, since L4d and L5 sit at tau - 1/2 of 1e-5 scale (L26).

## 1. The collision operator

**Named: recursive-regularized (RR) D3Q27 collision to third order, with WALE in the relaxation time.** It is the spec's choice (section 1.6 and 11.3: Malaspinas 2015; Coreixas et al. 2017, Phys. Rev. E 96). Plain BGK cannot carry these cases, so the operator is pinned down exactly here, and the map (section 6) tests whether it does.

**The cases it must carry** (nu_lat = u_lat x cells per length / Re, tau - 1/2 = 3 nu_lat; cell Reynolds = u_lat / nu_lat):

| Case | u_lat | nu_lat | tau - 1/2 | cell Re |
|---|---|---|---|---|
| L2a, S1's sphere | 0.025 | 1.5e-4, 3e-4 | 4.5e-4, 9e-4 | 170, 83 |
| L4d, C8's disc | 0.05 | 9.4e-6 | 2.8e-5 | 5 300 |
| L5, Caradonna-Tung (Re_c 1.95e6, 32 cells a chord) | 0.254 | 4.2e-6 | 1.3e-5 | 60 000 |

**Per node, one step** (fp32, populations stored shifted, f' = f - w_i with rho0 = 1; all sums fixed-order; the shift cancels in every difference f - f_eq):
1. rho' = sum f'_i; rho = 1 + rho'; j = sum c_i f'_i (the rest state has no momentum); u = j / rho.
2. Equilibrium, third order in u (the Hermite series; c_s^2 = 1/3, so the constants are 3, 4.5 and 4.5, all exact in fp32):
   f_eq_i = w_i rho [1 + 3 (u.c) + 4.5 ((u.c)^2 - u^2/3) + 4.5 ((u.c)^3 - u^2 (u.c))]; stored as f_eq_i - w_i.
   On D3Q27 the third-order term has no xxx, yyy or zzz part (H_xxx = c^3 - 3 c_s^2 c = 0 on c in {-1,0,1}); the closed form above already gives that.
3. f_neq_i = f_i - f_eq_i. Second-order moment a2_ab = sum c_a c_b f_neq_i (6 components).
4. Third-order moment by the recursion: a3_abg = u_a a2_bg + u_b a2_ag + u_g a2_ab (7 non-aliased components: xxy-type 6 and xyz).
5. Regularized part: f_reg_i = w_i [4.5 (c.a2.c - tr(a2)/3) + 4.5 H3_i : a3], H3_i,abg = c_a c_b c_g - (c_a d_bg + c_b d_ag + c_g d_ab)/3.
6. Collide: f*_i = f_eq_i + (1 - 1/tau) f_reg_i, with tau = 3 (nu + nu_t) + 1/2 at the node (WALE below). Then stream (section 2).

**WALE** (Nicoud and Ducros 1999), C_w = 0.5 (named before any run, L1 note item 3), filter width 1 (lattice units): the velocity gradient g_ij = du_i/dx_j by central differences of u over the 6 face neighbours (one-sided at a wall or open face; at a halfway wall the ghost is u_g = 2 u_w - u_node); S = sym(g), Sd = sym(g.g) - tr(g.g) I/3; nu_t = (C_w)^2 (Sd:Sd)^(3/2) / ((S:S)^(5/2) + (Sd:Sd)^(5/4)), taken as 0 when the denominator is 0. Powers use sqrt only: x^(3/2) = x sqrt(x), x^(5/2) = x^2 sqrt(x), x^(5/4) = x sqrt(sqrt(x)). WALE needs a rotation, which Pi_neq does not carry, so it needs the neighbours' u (decision D2 below).

**Why this should carry tau - 1/2 of 1e-5, and what is not claimed.**
- BGK goes unstable near tau = 1/2 because its non-hydrodynamic (ghost) moments relax at the same rate as the stress and so are not damped. Regularization rebuilds f from rho, u and the stress alone each step, which sets every ghost moment to zero each step: the dominant mechanism is removed. The third-order recursion keeps the Galilean-correct stress at Mach 0.44, where the second-order form loses accuracy.
- WALE adds nu_t only where the resolved field has grid-scale strain or rotation, so under-resolved gradients are damped and smooth flow runs at nu.
- **Not claimed:** that this holds at u_lat 0.25-0.40, tau - 1/2 = 0 to 1e-5. I have the literature's smooth and periodic results (to be confirmed against Malaspinas 2015 and Coreixas 2017 when the map runs), not a result for these points. The map is the test, and it comes before the walls.

**The ladder, fixed now, used only on the map's evidence** (each rung replaces the collision kernel only; storage, streaming, boundaries, tests and records are untouched; a change is a named cause and a new configuration, the old record kept, TD-2):
- R0. RR3 + WALE, tau unfloored. The map runs this first, including nu_lat = 0.
- R1. R0 with tau >= 1/2 + delta, delta the smallest that holds every map point (a minimum viscosity delta/3, below every case's nu_lat above).
- R2. Hybrid recursive regularization (Jacob, Malaspinas and Sagaut 2018): the stress is a blend sigma Pi_recursive + (1 - sigma) Pi_direct, sigma set from the map.
- R3. Central-moment (cascaded) collision on D3Q27, ghost moments relaxed at 1 (Geier et al. 2006).
- R4. Cumulant (Geier et al. 2015).
The lead decides a rung on the map's records. The bit-equality and test plan below do not change with the rung.

## 2. Lattice, storage and streaming

- **D3Q27 only** (no D3Q19: YAGNI). Weights 8/27, 2/27 (6 faces), 1/54 (12 edges), 1/216 (8 corners), as fp32 literals shared by both twins; velocities in one fixed order with the opposite-direction table.
- **Layout:** one array f'[27][N], direction-major (each direction contiguous), N = nx ny nz, index i + nx (j + ny k); one word of cell flags (fluid, wall, inflow, outflow); the lid and inflow data in a small parameter block. Plus a velocity array u[3][N] for WALE (D2).
- **Esoteric pull** (Lehmann 2022; Geier and Schonherr 2017), one copy, each slot touched by exactly one node a step. With pairs (i, i-bar) and the node at x:
  - even step: read incoming f_i from slot [i][x]; collide; write f*_i to slot [i-bar][x];
  - odd step: read incoming f_i from slot [i-bar][x - c_i]; collide; write f*_j to slot [j][x + c_j].
  Index arithmetic wraps on periodic faces. A link that meets a wall is bounce-back: the node writes the bounced population into the slot it will read the opposite direction from at the next step (plus the moving-wall term 2 w_i rho_w (c_i.u_w) / c_s^2 on the lid). Open faces set the incoming populations by rule (equilibrium inflow, a copy of the interior neighbour at the outflow). The parity makes macroscopic reads parity-aware.
- **Proof of the scheme:** a two-copy pull reference lives in the tests; the in-place scheme must equal it bit for bit over many steps, with walls, on random populations.
- **Memory:** 27 x 4 + 4 (flags) + 12 (u) = 124 B a cell, against the plan's 112 B (which has no u array). Caradonna-Tung's three-level nest was 3.2 GiB at 112 B; at 124 B it is about 3.5 GiB, under the 4 GiB cap.

## 3. Boundaries in L1

Periodic faces; open faces (equilibrium inflow at the ambient, zero-gradient outflow); plain box-face walls by halfway bounce-back, the lid by the moving-wall term (C1). Links into a solid neighbour outside the domain are bounced per link, so edges and corners of the box need no special case. Part walls (SDF links, Bouzidi, moving parts) are L2.

## 4. Step structure and the twin

Two kernels a step, both per node and reduction-free: `lbm_macro` (gather the incoming populations by the parity rule, write u) and `lbm_collide_stream` (gather again, read the 6 neighbours' u, collide, write). The GPU chain is recorded once for each parity (or one chain with the parity as a push word, as the NS chain does with the disc count) and resubmitted. The CPU reference is the same two functions on one thread first; a worker pool can split it by k plane later with no change to the bits (per-node independence, as L18).

**Bit-equality to the device of record**, the NS tier's rules: fixed-order arithmetic, no contraction (NoContraction on the device, contraction off on the CPU), sqrt and division only (correctly rounded on the device of record, TD-14), subnormals flushed on both (the ScopedFlushSubnormals wrapper; the SPIR-V denorm-flush mode), no libm transcendental. The only reductions are the diagnostics (K, mass, momentum), summed in double on the host from the same populations on both paths. New kernels are registered in `kSpirvModules` under the parity profile (a row in `tests/test_slang_layouts.cpp`, Core's file; I ask for the grant when the twin starts). New sources get entries in `engine/CMakeLists.txt` and `tests/CMakeLists.txt`.

## 5. Files

`engine/physics/lbm/`: `lattice.hpp` (the D3Q27 tables), `lbm_grid.hpp/.cpp` (state, shifted populations, flags), `lbm_collide.hpp` (the per-node collision, shared by the step and the unit tests), `lbm_step.cpp` (macro, collide-stream, boundaries), `lbm_diag.hpp/.cpp` (K, mass, momentum, the case samplers). GPU: `spike/lbm/` and `engine/shaders/kernels/lbm_*.slang` beside the airflow ones. Tests: `tests/test_lbm_lattice.cpp`, `_collision`, `_streaming`, `_c0`, `_c1`, `_map`.

## 6. Tests, mutations and order of work

Unit tests (derived values in double): lattice moments to fourth order (sum w = 1, sum w c = 0, sum w c c = c_s^2 I, sum w c^4 isotropic); the equilibrium's moments (rho, rho u, the momentum flux); collision conserves mass and momentum per node; f* = f_eq at tau = 1; the regularized part has no 0th or 1st moment; a Galilean test (a decaying shear wave advected at u0 = 0 and 0.2 decays at the same rate to O(Ma^2)); WALE is exactly 0 in uniform flow and pure shear and equals its closed form in solid rotation; one population circles a periodic box and returns after N steps in all 26 directions; the esoteric scheme against the two-copy pull, bit for bit; a closed box conserves mass; planar Couette through a halfway wall is linear. Mutations named in the tests: the third-order recursion dropped (Galilean test), WALE's rotation dropped (solid-rotation test), the read and write slots swapped (the two-copy test), the opposite-direction table wrong (the closed box), the shift omitted from one sum (the closed-box mass).

Order: (1) this design reviewed; (2) the lattice core and the periodic stream (no collision) with their tests; (3) R0 collision and WALE with their unit tests; (4) **the 35 + 4 map points on the CPU reference, first**, since they decide the rung; (5) C0a, C0b; (6) box walls and C1; (7) the GPU twin, bit-equal on C0 and C1; (8) the records, for Test/Docs' review.

**The map's required rows beyond L1 note's 35 points** (A2): (u_lat, nu_lat) = (0.025; 1.5e-4), (0.025; 3e-4), (0.05; 9.4e-6), (0.254; 4.2e-6). Cost: a 64^3 point to t* 20 is 203.7 / u_lat steps of 2.6e5 nodes; the whole map is about 1e10 node updates, hours on one CPU thread (the pool brings it under an hour).

## 7. For the lead to decide now

- **D1. The operator:** RR3 + WALE as R0, with the ladder R1-R4 and the lead deciding a rung on the map's records.
- **D2. WALE's neighbours.** Options: (W2, recommended) two kernels a step with one u array, 124 B a cell and about 50% more population reads, no lag, simplest to verify; (W1) one fused kernel reading the previous step's u from a ping-pong pair, 136 B a cell (3.9 GiB for the Caradonna-Tung nest, too near the cap) and a one-step lag in nu_t; (W3) Smagorinsky from Pi_neq alone, 112 B and no neighbour reads, but WALE is the decided model and C_w is recorded. I recommend W2; a fused kernel is an optimisation for later.
- **D3. D3Q27 only.**
- **D4. Grants:** entries in `engine/CMakeLists.txt` for the new sources now; the `kSpirvModules` row when the twin starts.

## 8. The map's result: the rung, the coverage table, and the max-|u| rule

**The rung (L33):** R0, RR3 + WALE (C_w 0.5), no fallback. The records are `tests/data/airflow/records/l1-stability-map.md` (41 points on the CPU reference, MSVC, release; every point report only).

**What the map showed** (64^3 periodic Taylor-Green field to t* 20, nu_lat 0 to 1e-3; held = K* finite and at most 1.05 K*(0) at every 5th step):
- u_lat 0.10, 0.15, 0.20, 0.25: every nu_lat held to the horizon, at nu_lat 0 included (WALE's nu_t alone carries tau).
- u_lat 0.275 (Ma 0.476): held in full at all five nu_lat (0 and 1e-5 in b4, 1e-6, 1e-4 and 1e-3 in b5, L33b), so u_held is 0.275.
- u_lat 0.30: nu_lat 0, 1e-6, 1e-5 break at step 320 of 679, 1e-4 at 350, 1e-3 holds. u_lat 0.35: 235 of 582 (nu_lat 0, 1e-6, 1e-5), 245 (1e-4), 1e-3 holds. u_lat 0.40: 170 to 180 of 509 for nu_lat up to 1e-4, 280 at 1e-3. The breaks fall at t* 6.7 to 9.4, the transition's peak, and nu_lat 0 and 1e-6 break at the same step, so what sets the edge is u_lat, not tau - 1/2.
- The four required rows (A2) held in full: L2a (0.025; 1.5e-4) and (0.025; 3e-4), L4d (0.05; 9.4e-6), L5 (0.254; 4.2e-6).

**The coverage table** (what the lattice is stated to hold, and no more):

| Rung | Max u_lat stated | Edge row recorded | Highest u_lat held (the check below) | Min tau - 1/2 tested | Tested on |
|---|---|---|---|---|---|
| R0 | 0.275 (Ma 0.476), any nu_lat down to 0 | 0.254 (Ma 0.440; L5's nominal), nu_lat 4.2e-6, held | 0.275 (all five nu_lat) | 0 (tau = 1/2, WALE's nu_t alone), at u_lat up to 0.275 | 64^3 periodic, one field, t* 20, no walls |

It does not cover walls, nests or a sustained flow (L2, L4, L5 each carry their own stability in their own records), nor any u_lat between 0.275 and 0.30 (the first break is at 0.30).

**L5's u_lat (L33b).** It comes from the tolerances note's rule, fixed before the run (`test-docs/2026-11-03-l1-lbm-tolerances.md`, "The stability map", "The decision"): L5 runs at the smaller of 0.254 and u_held / 1.4, with the margin 1.4 from P9's Table 17 (local speeds 1.32 to 1.40 V_tip), and P9's compressibility caveat applies below 0.254. Here u_held is 0.275 (every nu_lat held there), so **L5 runs at u_lat 0.196 (Ma_lat 0.340)**, and the caveat applies. Had the three further nu_lat at 0.275 broken, u_held would be 0.25 and L5 would run at 0.179. At 0.254 the fluid by the blade would run at about 0.34 to 0.36, past the break at 0.30. The tolerances note is Test/Docs' and stays as it is.

**The max-|u| guard (L33), added for every case, not a replacement for the rule.** Every case's record compares the health line's max lattice |u| with the map's highest held u_lat, 0.275. A case that reaches past it lowers its u_lat, which means more steps, and runs again before its result counts. The check is on the run's own max |u|, not on the nominal u_lat: it matters most for L5, whose blade surface moves at the tip u_lat, so the fluid next to the surface can exceed the nominal value.
