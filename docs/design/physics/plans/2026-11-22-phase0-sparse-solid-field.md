# Phase 0 of the resolved-blades plan: the sparse solid field, and the proof that no number moves (O190)

Status: DESIGN, not code. Physics, 2026-11-22 (box clock, stamped at commit). Parent: `2026-11-22-resolved-blades.md` §4 (Phase 0, the memory diet). Cameron's term (O190): Phase 0 must PROVE "no change to the numbers" as a RED-first test: the digests and the full `-L gpu` run equal before and after. This document fixes the first and largest piece, the sparse solid field (about 70 of the 136 bytes a cell Phase 0 removes), and writes the proof test before the code. The other two pieces (recomputed limiter bounds, aliased scratch) are listed with their own proof obligations, not designed here.

## 1. What the device holds today for solids (read from the code, 2026-11-22)

With solids present every face of the grid carries these, in dense flat arrays, u then v then w concatenated (`pack_solid_field`, `spike/airflow/solids_chain.cpp:28-55`; sizes `step_chain.cpp:360-405`, resident, `solids_binding_bytes()` `solids_chain.cpp:168-181`):

| binding | content | bytes | per cell (3 faces a cell) |
|---|---|---|---|
| 13 `airflow_apertures` | theta, the open fraction of the face (1.0 default) | 4 a face | 12 |
| 14 `airflow_wall_velocity` | the wall's velocity at the face (0 default) | 4 a face | 12 |
| 15 `airflow_phi` | the signed distance at the face (float max default) | 4 a face | 12 |
| 16 `airflow_face_tags` | the wall-link index (u32, `kNoWallLinks` default) and the free-slip layer (1 byte a face, 0 default) | 5 a face | 15 |
| 17 `airflow_wall_links` | 48 B a link (inv_f[6], wall[6]) | not per cell | none |
| 18 `airflow_region_labels` | a u32 a CELL (the closed region's label) | 4 a cell | 4 |
| 12 `airflow_mg_weights` | the multigrid's face weights, per weighted level | 4 a face | ~12 at level 0, plus the coarse levels (about 14 in all) |
| 20 `airflow_pcg` | the CG's seven vectors (X, B, R, Z, D, Q, Best), present only with solids | 28 a cell | 28 (NOT solid-sparse: a global solve; see §6) |

So the solid-field arrays plus the labels and weights are **about 12 + 12 + 12 + 15 + 4 + 14 = 69 B a cell**, and in the film's box **more than 98% of every one of those arrays holds the default**: the solids are four hubs and arms in 30 M cells. (The host's `SolidField` also keeps theta, wall, phi, link, layer per face, 17 B a face, `solids.hpp:167-184`: the CPU stays dense and stays the authority.)

**Readers** (found by reading, so the list is the work list):
- through the accessors in `engine/shaders/airflow_pcg.slang` (`aperture_base` :157, `face_theta` :164, `face_wall` :166, `face_closed` :169, `face_layer` :247, `solid_cell` :151): `airflow_solid_divergence`, `airflow_solid_gradient`, `airflow_solid_velocities`, `airflow_wall_load`, and through `solid_cell` the pcg_measure, pcg_update, solid_divergence, wall_load kernels;
- DIRECTLY, bypassing the accessors: `airflow_publish_fields` (its own `aperture_base`, `:86-98`), `airflow_sgs` (`:118-123`: apertures, phi, face_tags, wall_links), `airflow_viscous` (`:104-116`: apertures, phi, wall_velocity, face_tags, wall_links); region_labels directly in dye_publish, publish_fields, sgs, region_mean;
- the multigrid weights through `face_weight()` (`mg_levels.slang:84`) in the mg kernels and pcg_apply/pcg_update.

## 2. The design: a tile directory in front of the same arrays

**One rule: a face in a tile with no solid reads the DEFAULT the dense array would hold there, exactly.** The defaults are the dense field's (`make_solid_field`, `solids.cpp:609-617`): theta 1.0f, wall 0.0f, phi `FLT_MAX`, link `kNoWallLinks`, layer 0; region label 0; weight 1.0f.

- **Tiles.** Each component's face grid (sizes `(nx+1)·ny·nz`, `nx·(ny+1)·nz`, `nx·ny·(nz+1)`) is cut into tiles of 8 x 8 x 8 faces in the component's own face index (the last tile partial). 8 is chosen because a tile is then 512 faces = one 2 KB float run per array, a power of two (shift and mask, no division), small enough that the four rotor hubs and arms touch under 2% of the film box's tiles.
- **Directory.** One `uint` per tile per component (a new binding after the current last, appended): `0xFFFFFFFF` = a default tile, else the tile's slot in the pools. Cost: 3 components x cells/512 x 4 B = **0.024 B a cell**.
- **Pools.** The existing bindings 13 to 16 keep their element types and become pools of slots (slot s holds faces `s*512 ..`), so the host packer (`pack_solid_field`) fills only the allocated tiles; binding 17 is unchanged. A tile is ALLOCATED when any of its faces differs from the default in ANY of theta, wall, phi, link, layer (decided on the host by comparing to the defaults, so `build_solids`' result is the one source of truth).
- **One accessor.** `face_theta(c, idx)`, `face_wall`, `face_closed`, `face_layer`, and a new `face_phi`, `face_link` become: decompose idx to the face coordinates (the kernels already do, e.g. `airflow_solid_gradient` computes `f`), form the tile and the offset inside it, read the directory, return the default or the pool entry. The direct readers (publish_fields, sgs, viscous) are changed to call the accessors, so the dense indexing exists in ONE place.
- **Region labels and MG weights.** Same tiles, per CELL for the labels (a cell directory, 1 u32 a 512 cells) and per face for the weights of each level (tile the level's face grid; coarse levels are small, kept dense in the first cut and measured). A weight is the default 1.0f wherever the three apertures around it are 1.
- **Moving solids.** Static parts (the frame, the hubs) pack once; a part that moves every step (the rotor discs' solid hubs) re-packs its tiles each step: the directory is rewritten and only the allocated pool slots are uploaded. The film has no moving solid; this is the plan's Phase 2 concern and is designed there.
- **Result in the film's box (30.45 M cells, derived):** the dense 69 B a cell becomes about 1.5 B (the allocated tiles, at under 2%, plus directories): **about 67 B a cell saved, 1.9 GiB at 30.45 M cells** (67 B x 30.45 M = 2.04 GB). (Measured: the dense arrays are 51 B a cell without labels and weights; the "about 70 B" of the parent plan is this table's 69.)

## 3. What "no change to the numbers" means, and how it is proved

The GPU solver's authority is the CPU `FluidStep`, held to it BIT FOR BIT by the existing GPU tests (the survey of `tests/`, 2026-11-22). The sparse field changes WHERE a float is read from and never WHICH float, so the proof is that every existing bit-for-bit comparison passes UNEDITED, and that a new test compares sparse against dense directly:

**RED first, in this order (each is written and seen to FAIL before the code that makes it pass):**
1. **`SolidTileMap.EveryFaceReadsWhatTheDenseFieldHolds` (CPU, new).** For each scene of `tests/test_airflow_solids.cpp` (sphere, box, hull, a sheet, a two-part mix, a periodic axis), `build_solids` into a dense `SolidField`; pack it sparse; read EVERY face of every component through the host mirror of the accessor (the same index arithmetic, a pure function the shader's is checked against); require the 5 values equal **as bits** (`std::bit_cast`) to the dense field's. Also: a fully open grid allocates no tile; a tile allocated for one non-default face holds the rest as defaults. RED against a stub packer.
2. **`SolidTileMap.TheBytesFallToUnderTwoPercent` (CPU, new).** The film's box (268 x 424 x 268) with the OpenFrame-5F's four motor cylinders and an arm box each: the packed bytes a cell must be at most 2.5 (the dense: 51). RED against the dense packer.
3. **`GpuAirflowSolidsSparse.SameBitsAsDense` (GPU, new).** The same device, the same case, dense then sparse: every published field, the CG iteration count and the residual history equal as bits, at workgroup sizes 32, 64 and 128, for the solids, viscous (Cs on) and wall-load paths. RED until the shaders read through the directory.
4. **The pinned digests stay as they are, and a change to one is a FAILURE of Phase 0, not a re-record:** `AirflowStep.TheSolverCoresDigestIsPinned` (`0x7ea42593859fbceb`), `AirflowConfigs.EachBoundaryConfigurationsDigestIsPinned`, `AirflowSolidsStep.NoPartsIsPlanABitForBit`, the GPU parity and invariance digests (`testing::state_digest`; `kTwoWorldIsolationExpectedDigest`), the standard-set device digest. None is edited in the Phase 0 branches; the branch's gate quotes their values before and after.
5. **The GPU-vs-CPU bit tests stay as they are** (`GpuAirflowStep.*MatchesFluidStepBitForBit`, `GpuAirflowSolids.*BitForBit`, `GpuAirflowProjection.*BitForBit`, `SameBitsAtEveryWorkgroupSize`): they compare against the DENSE CPU field, so they are the independent proof.
6. **The SPIR-V pins:** the rows that change must be EXACTLY the kernels this piece touches (the five accessor kernels, publish_fields, sgs, viscous, the mg kernels that read weights, the pcg kernels): the list is written into the commit message BEFORE re-recording, and `SpirvPin` must still show every other row byte-identical. A kernel outside the list that changes is a defect, not a re-record. The rows re-record in the same commit as their shader (the pin file's own rule).
7. **The whole `-L gpu`** (460/460 on 18f19e1e) equal before and after, with the count quoted; the Docker leg (consumer on and off) as at every code gate.
8. **The VRAM read:** the allocation probe (approved, gpu-alone, no steps) before and after at the film's box: the dense 30.45 M cells with solids (221 B a cell, 6.4 GiB) are REFUSED by `check_vram` today (the 4 GiB policy cap, and the ~6.0 GiB the 1 GiB reserve leaves), CORRECTED 2026-11-22T17:56Z (the earlier "fit at 6.3 to 6.6 GiB" was wrong); the probe reads what a box that DOES fit holds, before and after; the figure drops by the table's 1.9 GiB (an equality with the arithmetic is not claimed; the number is read, not derived).

## 4. The other two pieces of Phase 0 (not designed here; their proof is the same bit tests)

- **Recomputed limiter bounds** (`lo`/`hi`, 9 floats a cell, `fluid_step.cpp`): the advection recomputes the min and max of the stencil instead of storing them. A min or max is exact, so the result is bit identical if the operands and the order of ties are the same; proof: `GpuAirflowStep.*MatchesFluidStepBitForBit` unedited, plus a CPU test that the recomputed bounds equal the stored ones as bits on the existing scenes (RED first).
- **Aliased scratch** (the multigrid's p/b/r, the divergence, the viscous and the du/dv/dw right-hand sides): buffers whose lifetimes do not overlap share memory. Proof: a lifetime table in the design (which kernel writes, which reads, which frees) and the same bit tests; no arithmetic changes.
Both are written up in their own design notes before their code; each saves about 25 to 35 B and about 15 to 20 B a cell (the parent plan's figures, "derived", to be measured by the allocation probe).

## 5. Order and size inside Phase 0 (CORRECTED 2026-11-22T17:56Z: Phase 0 is about 11 to 12 days, not the parent plan's 7 to 9; this is the sparse piece's share)

1. Tile map, host packer and the host accessor mirror, tests 1 and 2 (RED, then green): 2 days.
2. The directory binding, the shader accessors, the direct readers moved onto them, test 3: 2.5 days.
3. Labels and the first-level weights sparse: 1.5 days.
4. Pin re-record (the listed rows), digests quoted, the full `-L gpu`, the probe before and after: 1 day.
About 6 days for this piece plus its share of one gate; the limiter bounds add about 1.5 days, the aliased scratch about 2.5 and the shared gate (pins, full -L gpu, probe) 1.5: **Phase 0 is about 11.5 days** (CORRECTED 2026-11-22T17:56Z; the parent's 7 to 9 priced the whole of Phase 0 at about this piece). The first visible payoff, the deeper O158 wake, needs the sparse field plus the limiter bounds under today's 4 GiB cap (about day 8.5), the sparse field alone if the policy cap is lifted (about day 7); the full diet is about day 11.5.
Core's share (the data layout, pinned structs, the Slang twin) runs alongside and owns the shared layout files; this piece adds ONE binding and new accessors, and touches no existing struct, so only the rows listed above re-record.

## 6. What this does not claim, and the open points

- The CG's seven vectors (28 B a cell, solids only) stay: the projection solve is global and every cell is in it. They go to the aliased-scratch piece (their lifetime is the pressure solve only), not here.
- The saving is **measured on the film's box only**: with many solids (a dense frame, a ducted cinewhoop) more tiles allocate; the test 2 bound is for the film's four motors and arms. A scene with solids filling more than about a fifth of its tiles would pay the directory and slot overhead for nothing; the packer then falls back to dense (`kParamsFlag` bit, so the dense path stays and is tested).
- **Decisions needed from Core:** the new binding's number and the directory's descriptor placement (set 1, `set_one_bindings.hpp`); whether the pool bindings may keep their element types (this design assumes yes). **Decision for Cameron:** none; the piece changes no number and no picture.
- The reading above is of the code as of master `019a4463`; line references are to it.
