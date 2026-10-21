# P3: wall pressure, force and moment -- implementation plan

> Executed inline by Physics (the lead's order after P2, check_grid and the vehicle adapter, 2026-10-22). Ledger: `.superpowers/sdd/2026-10-22-airflow-p3/progress.md`.

**Goal:** the force and moment the air puts on the solids, for display (one-way; in D4 they replace the body's drag element), with S1's added mass and surface `C_p`, S3's sheet force, and Publish's one-sided gradient at closed faces (S5).

**Design:** `docs/design/physics/plans/2026-10-13-airflow-solids-design.md` §6 (P3), §8 tests 1, 3 and 8, §9 (order); APPROVED by the lead 2026-10-13. **Tolerances:** `docs/design/test-docs/2026-10-13-solids-tolerances.md`, S1, S3 and S5.

## Global constraints

- TD-1 goldens and records name their commit; a record that moves is regenerated with its cause named. TD-2 bands measured and pinned. TD-3 no libm under `engine/`. M1B no wall clock. L6 refuse or announce by name. TD-9 one rule, one source.
- Fixed-order reductions (`reduce.hpp`): every sum feeding a reading is in the tile order, fp32, no fp64.
- Gates (tiered): all targets release, the release CPU suite, gcc `--compile` on changed `.cpp` and plain on headers, a leg (airflow branch). GPU only if GPU code changes (it does not).

## The rule (design §6)

On a cell `c` with any open face, the divergence theorem gives the solid surface inside it the area vector `A_c = Σ_f θ_f dx² n_f`, `n_f` out of the cell. The force on the solids is `F = −Σ_c p_c A_c`, the moment about a point `o` is `M = Σ_c (x_c − o) × (−p_c A_c)`, with `p` the cell's own pressure (Pa) and `x_c` its centre. A full fluid cell and a solid cell give `A_c = 0`. Beside a sheet the closed face gives each side `∓ dx² e`, so the two sides sum to the pressure jump times `dx²`: the design's sheet rule, with no separate path. Solid cells (no open face) are skipped, so their pressure is never read.

## Tasks

### Task 1: one tile order, for any cell value

- `reduce.hpp`: `template <class T, class Cell> T fixed_tile_sum_of(uint32_t nx, uint32_t ny, uint32_t nz, Cell cell)` -- `cell(i, j, k, idx)` returns a `T` (`float`, `glm::vec3`); the order is `fixed_tile_sum()`'s, each tile from `T{}`. `fixed_tile_sum()` and `fixed_tile_dot()` call it, so the order lives once (TD-9).
- Tests: `fixed_tile_sum_of` equals `fixed_tile_sum` bit for bit on a random field over ragged tiles (5 x 7 x 9), and a `glm::vec3` sum equals three float sums. Existing goldens and the CG tests unchanged (bits unchanged by construction).

### Task 2: `wall_load()`

- `physics/airflow/wall_load.hpp`:
  - `struct WallLoad { glm::vec3 force{}; glm::vec3 moment{}; };` -- N and N m, in the grid's frame (the world's axes).
  - `[[nodiscard]] WallLoad wall_load(const MacGrid& g, const SolidField& f, const FluidRegions& r, glm::vec3 about) noexcept;` -- the rule above over the cells `r.solid` marks fluid, summed with `fixed_tile_sum_of`; `about` in the grid's frame (the world shifted by the grid's origin). All the solids together; per-part loads wait for a second body (D4).
  - With no solids (`!f.any`): zero.
- Tests (`tests/test_airflow_wall_load.cpp`), each against a field written into `g.p` after `set_solids()`:
  1. d'Alembert: a uniform `p` on a sphere gives `|F| ≤ 1e-6 p A_sphere` (the area vectors close; rounding only).
  2. Archimedes: `p = −ρ g·x` (linear) on a sphere `R = 6 dx` gives `F = ρ g V` within 3 dx/(2R) relative (the cut cells read `p` up to half a cell off the wall). Moment about the centre `|M| ≤ 1e-3 |F| R`; about a point offset by `d`, `M = (c − o) × F` within the same band. Mutations: the sign flipped (buoyancy down), the moment arm from the wrong point.
  3. A sheet across a channel: `p` = `p_a` on one side and `p_b` on the other gives `F = (p_b − p_a) A e` exactly to rounding.
  4. No solids: zero.

### Task 3: S1's P3 checks (design test 1)

- In `test_airflow_solids_step.cpp`, beside the P2 checks, S1's configuration at `R/dx` 6 (and 12 under `SPADE_AIRFLOW_VALIDATION`, with the order check):
  - step 1: `C_M = F_x h / (ρ (4/3)πR³ U)`, band 0.5 ± 0.24 (6), ± 0.12 (12);
  - step 3: `max |C_p − C_p,pot|` over samples every 15° on four meridians, one cell out along the normal, trilinear in the cell-centre pressure (a test helper), ≤ 0.32 (6), ≤ 0.20 (12); the drag `|F_x| / (½ρU²πR²)` ≤ 0.06 (6), ≤ 0.04 (12); side forces ≤ 0.01; order check `e(12) ≤ 0.7 e(6)` at `r = 7R/6`.
  - Mutations: `wall_load` returning 0 (C_M = 0) and sign-flipped (C_M = −0.5) both fail.
- A new record block "S1 ..., P3 checks" in `tests/data/airflow/records/s1-sphere.md` (I2 grammar).

### Task 4: S3's sheet force

- In S3 (both parities): `F h / (ρ U L A)`, band 1 ± 0.033. The S3 record blocks gain the measure (regenerated, cause named: P3's force joins).

### Task 5: Publish beside a sheet (S5, design test 8)

- `PublishInput` gains `std::array<std::span<const float>, 3> theta{}` (the face apertures; empty: every face open). Refused (invalid_argument) when a non-empty one's size is not its component's face count.
- `gradient()`: along each axis, a neighbour across a closed face (θ = 0) is not used; the difference goes one-sided from the cell, as at a non-periodic domain edge; with both closed, 0.
- Test (`test_airflow_publish.cpp`): S5's field (a sheet in `y = y_s`, x and z periodic, jumps `U` in u and w, shear `S dx = 0.01 U`): `|ω|` next to the sheet within 1e-3 of `S` relative, `|q̂| ≤ 1e-3 S`. Mutation: the closed flag ignored (a central difference across), `|ω| ≈ 70 S`.
- A record `tests/data/airflow/records/s5-publish-sheet.md`.
- Callers: LiveRun (Interface) and `flow_movie` keep compiling (the field defaults empty); Interface passes `s.solids.theta` when solids are set (told at READY).

### Task 6: records, docs, gates

- Records repasted at the gated head; Test/Docs gets the numbers for its 04 rows.
- Design §6 status line, `07-status` and PHY rows; consumers line for the lead's merge.
- Gates as above, then READY naming P3.

## Review focus

- A cell next to two sheets (both faces closed along an axis): its gradient along that axis is 0, not a difference across either.
- A cut cell at the domain's edge: its domain face's aperture enters `A_c` like any other.
- `about` far from the body: the moment's rounding grows with the arm; the test's offset point checks it.
- A sheet splitting two regions with different gauges: the jump `p_b − p_a` mixes the gauges; one-way display accepts it, and the record names it if a case meets it.
- An all-solid grid or no fluid cell: zero, no division.
