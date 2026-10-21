# The Default Drone Library — Plan (DRAFT for the lead)

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking. **Reviewed and approved by the lead, 2026-10-21; every open question is answered (Rulings).**

**Goal:** The library gets five drones and the parts they are made of.
- Four drones are real drone classes, each with a body, rotors, motors and a battery: a Phantom 3-class quad, an Iris+-class quad, a 5-inch FPV quad and a Crazyflie-class micro.
- The fifth is a basic parametric quad (frame, arms, rotors) that fits the competition drone easily.
- Motors, propellers, ESCs and batteries become library entries of their own.
- Every number in every entry names its source and its grade, and every source has a license check.
- The reference quad's two-point prop table is labelled plainly as a test fixture (open question 2).

**Architecture:**
- **Parts are what the airframe builder already holds.** `vehicles/airframe_compile.hpp` has `MotorBlock`, `PropBlock`, `EscBlock` and `BatteryBlock`. A part entry is one of those blocks as a file (`*.motor.yaml`, `*.prop.yaml`, `*.esc.yaml`, `*.battery.yaml`) plus a `sources` block. `scene/part_file.hpp` holds one emitter and one parser per kind (TD-9).
- **A drone is an `AirframeSpec` compiled.** It is built in code from its body parts (published dimensions) and the part entries, loaded from their files. `compile_airframe()` turns it into the model file, which is a generated golden, the same way the reference quad is generated today (`tests/test_library_quad.cpp`). A generated sidecar beside it, `<name>.sources.yaml`, cites every input number and names the part entries it was built from, by file and content hash (open question 1).
- **The basic quad is a function.** `vehicles/basic_quad.hpp`: `basic_quad_airframe(BasicQuadParams) -> Result<AirframeSpec>`. It takes the layout, prop, motor, ESC and battery, and an optional fitted mass, inertia and kappa. A fitted mass and inertia are met exactly by a computed remainder part, so `compile_airframe()` does not change. A remainder that is not a physical body is refused by name.
- **The library index goes to version 2**, with the four part kinds. A version 1 index still reads.

**Tech Stack:** C++23, GoogleTest through ctest, MSVC + Ninja presets, gcc-13 through the leg. No new dependency. No kernel, binding or Slang change.

**Spec:** the backlog row (Cameron, AskUserQuestion 2026-10-21, master `6657069`), the lead's brief of 2026-10-21, and the Kat lead's competition-drone parameters (relayed 2026-10-21). Kat's documents, read in place and read-only: `KAT/design-specs/integration/drone-builder-datapack.md` §1 and §7, `drone-builder-catalog.md` (Kat's generic part catalog, 2026-10-03), and `content/airframes/competition-700.airframe.json`. Research notes behind the source tables: kept in the Core session's scratchpad, not the repo. Base: master `0076fd1`.

## Global Constraints

- **Published numbers only; no copied CAD, mesh or data file** (the repo is public). A drone's geometry is boxes, cylinders and tubes sized from published dimensions. A data table in the repo is our own resampling of a cited source, never a copy of a source's file.
- **Every number cites its source and its grade, in the entry's own file** (the part files) or in its generated sidecar (the model files). The grades are:
  - `measured`: a test stand or tunnel, in a primary source;
  - `published`: the manufacturer's own specification;
  - `retailer`: a seller's or a third party's listing; weaker, and named so;
  - `derived`: computed from cited numbers by code in this repo, which the entry names;
  - `fitted`: fitted against data (for example, Kat's flight logs of the VQ1 simulator);
  - `assumed`: a plausible default with no source for this airframe. It is named so in the UI too (Interface).
- **Every source has a license line**: public domain, facts-only-cite, or not usable. A source that forbids redistribution feeds no number in the repo. That rules out APC's PE0 and PER files (open question 4).
- **No golden moves, and every identity holds.** The reference quad stays byte for byte. Its pinned `model_identity` and every test built on `reference_quad()` stay put. Its library index entry is relabelled only (Task L6). New drones add new generated goldens. Each is final only once a fresh (`-NoSeed`) Docker leg reproduces it (TD-12).
- **Determinism:** all derivations run in double, in a fixed order, with `+ − × /` and `sqrt` only (TD-3: no libm under `engine/`). They round to float once, at the model's own fields.
- **Refuse, never skip (L6).** A part file with an unknown key, a field without a source, a source without a license line, or a grade that is not one of the six is `invalid_argument`, naming the file, the field and the source id.
- **Layering:** `vehicles/basic_quad.*` includes `vehicles/` and `core/` only. `scene/part_file.*` includes `vehicles/airframe_compile.hpp` for the block types and nothing from `sim/`.
- **The tiered gate per task**: release build, release CPU suite and `gcc-check.sh` on every changed file. A `-NoSeed` leg runs for each task that adds or changes a golden or a format (L2, L3, L4, L5). All heavy runs go through `jobrun.py` with tickets `spade-core-<what>-<sha>`.
- **Where the code lives:** one branch per task, cut from master: `core/library-parts` (L1-L2), `core/basic-quad` (L3), `core/library-drones` (L4-L5), `core/library-fixture` (L6). Never push. Never stash.

## What the sources give, and what they don't (the license check)

The detailed notes, with URLs, pages and quotes, are in the research notes named under **Spec**. Task L1 turns this table into `assets/library/SOURCES.md`, the library's bibliography.

| Source | What it gives us | License | Grade |
|---|---|---|---|
| NASA Ames, Russell, Jung, Willink, Glasner, "Wind Tunnel and Hover Performance Test Results for Multicopter UAS Vehicles", AHS 72nd Forum, 2016 (NTRS 20160007399; a conference paper, not a TM) | Phantom 3 Adv. and Iris+: rotor diameter (9.4 and 9.6 in), rotor-to-rotor spacing (9.8 × 9.9 and 10.4 × 16.1 in), nominal weight 2.8 lb, baseline hover 5,300 and 5,400 rpm at 2.8 lb, hover rpm ranges, figure of merit (0.58-0.67 Phantom, 0.72 Iris+ peak), Phantom motor-ESC efficiency (50-77 %). Hover and tunnel results are **figures only**; no tables | Public domain: "This is a work of the U.S. Government and is not subject to copyright protection" (p.1); NTRS "public use permitted" | measured |
| NASA/TM-2018-219758 (the follow-on data report) | the tabulated data. **Not found** on NTRS and not read | public domain if found | — |
| DJI Phantom 3 spec pages | mass 1216 g (Standard) / 1280 g (Adv., Pro), diagonal 350 mm, 4S 4480 mAh 15.2 V 68 Wh 365 g pack | facts only, cite (DJI's site terms not read) | published |
| Retailers (Phantom 3 spares) | motor 2312, 960 KV; prop 9450, pitch 4.5 or 5.0 in (they conflict) | facts only, cite | retailer |
| Adafruit / iFixit (Iris+) | 1282 g with battery, 550 mm motor to motor, 920 KV, 9.5 × 4.5 props, 3S 5.1 Ah 8C | facts only, cite (no 3DR primary read) | retailer |
| UIUC Propeller Database, Vols 1-4 (Brandt, Deters, Ananda, Dantsker, Selig) | measured static C_T, C_P against rpm; C_T, C_P against J at set rpm; blade geometry r/R, c/R, β. Has: APC Thin Electric 10x5 (NASA calls it close to the Phantom's rotor), APC TE 9x4.5 and SF 10x4.7 (near the Iris+'s), GWS 5x4.3 and Micro Invent 5x4 tri-blade (5-inch), Crazyflie 1.85-in prop (static only). Has **no** DJI 9450, **no** 6-inch prop and **no** commercial FPV prop | **Unknown**: no license or terms on the site, only a citation request (quoted in SOURCES.md) and "© 1994-2026 UIUC Applied Aerodynamics Group". We store our own resampled tables, cited, never the files (open question 3) | measured |
| APC PE0 (geometry) and PER (performance) files | computed (vortex theory) performance, and blade geometry | **Not usable**: APC's terms prohibit "reproduction, distribution, display or transmission" of site content | — |
| Bitcraze Crazyflie 2.1 datasheet, Rev 3 | 29 g takeoff, 92 × 92 × 29 mm motor to motor incl. feet, 1S 250 mAh, 7 × 16 mm coreless motors | facts only, cite | published |
| Bitcraze "PWM to thrust" page (2015) | one motor's current, thrust, voltage and rpm at 16 PWM steps (to 57.9 g at 23,882 rpm) | facts only, cite (Bitcraze's license page 404s) | measured |
| Luis & Le Ny, arXiv:1608.05786, Table 2.3.1 | Crazyflie 2.0 arm 39.73 mm, rotor radius 23.13 mm, inertia diag(1.395, 1.436, 2.173)e-5 kg·m² | facts only, cite (arXiv, author copyright) | measured, second-hand |
| Retailers (5- and 6-inch FPV parts) | e.g. a 2207 1855 KV motor: 50 mΩ, 35.08 A peak, 31.6 g; 6S 1300 mAh packs ~240 g; HQ 6x3.5x3 props ~6.4 g | facts only, cite | retailer |
| Kat: `competition-700.airframe.json` and the datapack | the competition drone, **fitted to the AI Grand Prix VQ1 simulator, not hardware**: mass 0.700 kg, X layout ±0.075 × ±0.10 m, the measured thrust curve (0 to 52.5 m/s², 9.188 N per motor), airspeed loss (k 0.00028, exp 2, floor 0.35), drag 0.028 kg/m, tau 0.033 s, kappa 0.022 m (flagged), inertia (assumed; it equals NeuroBEM's published tensor) | **Owner's call** (open question 5) | fitted / assumed |
| Kat's generic catalog (`drone-builder-catalog.md`) | generic parts P5T, P6T, REF5, M2207-1750/2450, packs, ESCs. Its J-tables take their **shape** from APC's PER3 files | the APC-derived shapes fall under open question 4 | retailer / derived |
| LiPo OCV against state of charge | **no usable table found** under an open license. Zhao & Howey's printed coefficients are too rounded to use | — | — |

**What this means, plainly.** The two NASA-tested drones get primary-source size, mass, rotor diameter and a hover check point, but their prop curves are a near-match UIUC prop, not their own. The FPV quad and the 6-inch defaults stand mostly on retailer numbers. The Crazyflie is the best covered: a datasheet, a measured prop and a measured thrust table. Each entry says which of these it is.

## Per drone: the inputs and their grades

**Phantom 3-class** (`drones/phantom3_class.model.yaml`):
- Mass 1.280 kg (DJI, published).
- Rotors at the corners of a 249 × 251 mm rectangle (NASA's 9.8 × 9.9 in; measured), which gives the 350 mm diagonal DJI states.
- Rotor diameter 0.2388 m (NASA 9.4 in; measured).
- Motor 960 KV (retailer); its resistance and no-load current are assumed.
- 4S 4480 mAh pack, 365 g (DJI, published). Cell resistance is assumed; the OCV table is open question 8.
- Prop curve: the APC Thin Electric 10x5 coefficients from UIUC, applied at 9.4 in (derived, citing NASA p.15's agreement).
- Body: a central box and four arm tubes sized to the published diagonal and the mass that remains (derived).
- **Check:** hover at 2.8 lb (12.46 N) needs 5,300 rpm ± 5 % (NASA Table 2). With the UIUC coefficients that is C_T ≈ 0.100 at J = 0, which is plausible for that prop. Physics owns the tolerance.

**Iris+-class** (`drones/iris_plus_class.model.yaml`):
- Mass 1.282 kg (retailer, with battery).
- Rotors at 264 × 409 mm (NASA's 10.4 × 16.1 in; measured). NASA's spacing gives a 488 mm diagonal, not the 550 mm the retailers quote; the file says so.
- Rotor diameter 0.2438 m (NASA 9.6 in; measured).
- Motor 920 KV and 3S 5.1 Ah 8C pack (both retailer).
- Prop curve: the APC TE 9x4.5 coefficients from UIUC, applied at 9.6 in (derived).
- **Check:** hover at 12.46 N at 5,400 rpm ± 5 % (NASA Table 2).

**5-inch FPV** (`drones/fpv_5inch.model.yaml`):
- Frame 220 mm motor to motor (assumed until a cited frame is chosen).
- Micro Invent 5x4 tri-blade prop (UIUC, measured).
- 2207 1855 KV motor (retailer) on 6S 1300 mAh (retailer).
- Mass is the sum of the parts (derived).
- **Check:** the thrust-to-weight ratio lands in 8-12, a band typical of the class. The band itself is `assumed` (no source yet), and Physics may set it.

**Crazyflie-class** (`drones/crazyflie_class.model.yaml`):
- Mass 0.029 kg and a 92 mm motor-to-motor span (Bitcraze, published).
- Arm 39.73 mm and inertia diag(1.395, 1.436, 2.173)e-5 (Luis & Le Ny; measured, second-hand). The inertia is met by the remainder part.
- Prop: UIUC Crazyflie 1.85 in, specimen t1 (measured, static only; Physics supplies the J dependence, P-a).
- Coreless DC motor, KV 14,000 rpm/V (retailer). The chain's equations hold for a brushed DC motor; Physics confirms (P-c).
- 1S 250 mAh pack (Bitcraze, published); its mass is assumed.
- **Check:** at the Bitcraze table's 9,374 rpm one motor gives 7.9 g ± 10 % (measured). Hover at 29 g needs about 7.3 g per motor.

**Basic quad, 6-inch** (`drones/basic_quad_6inch.model.yaml`). This is `basic_quad_airframe()` at its defaults, every one `assumed` unless noted:
- X layout ±0.075 × ±0.10 m (the competition drone's, per Kat).
- A 6x3.5x3 prop. Its curve is our own scaling from the nearest UIUC 5-inch tri-blade, so **derived and assumed**, because UIUC has no 6-inch prop.
- 2306 1700 KV motor, 6S 1300 mAh pack (retailer).
- Mass is the sum of the parts.
- Kat's best candidate set (datapack §7.2: a 6-inch tri-blade, 2207-2450 on 4S, R_eff about 3× the datasheet, a cap of about 18,300 rpm) informs the competition variant's assumed parts, `basic_quad_competition` (Rulings, answer 5).

## Inputs from Physics, Interface, Kat and Test/Docs

- **Physics (P-a): the propeller tables.** For each prop above: C_T(J) and C_Q(J) on the chain's uniform J grid (UIUC convention, C_Q = C_P / 2π). They are resampled from the UIUC static and wind-tunnel runs, and Physics chooses which rpm run (Reynolds number). The tables carry `ct_static`/`cq_static` and their grades. Physics names its own resampling rule, and Core carries the numbers in the prop files. For a static-only prop (the Crazyflie's), Physics says how C_T falls with J, or the table stays static and the entry says so.
- **Physics (P-b): the blade definitions for the actuator line.** For each prop: r/R, c/R and β(r/R) from the UIUC geometry files (measured), the blade count, the hub cut-out, and the airfoil polars the actuator line will use. The airfoils are unknown for every one of these props, so their polars are `assumed`. Core adds an optional `blade` block to the prop file (Task L2); Physics fills it.
- **Physics (P-c): the chain's fit to each drone.** Confirm the coreless DC motor fits the chain's equations. Set the effective resistance where the datasheet value cannot reproduce a measured point (Kat measured R_eff ≈ 3× on its rebuild). Set the tolerance of each drone's check (above).
- **Physics (P-d): a measured thrust curve, if open question 6 says yes.** The competition drone's curve (throttle to thrust, with the airspeed loss) as a propulsion mode. That is a model-format change (model_version 2, scene version 6), so it gets its own plan.
- **Physics: kappa.** The competition drone's 0.022 m against the catalog props' 9-12 mm. `basic_quad_airframe()` takes a fitted kappa as `cq_static = kappa · ct_static / D` and labels it `fitted`, but Physics says whether 0.022 m is believable.
- **Interface:** the Library panel's categories for the part kinds, and the "assumed" marking in the UI. Swapping a part into a drone is the builder's (Interface and Kat), later.
- **Kat:** aggregates from its flight logs if a check needs them (through the lead). Kat's part IDs (P5T, P6T, M2207-…) are kept as the part files' names where a part is the same.
- **Test/Docs:** a validation row per drone check (above), naming the source and the tolerance.

## Review Focus

1. **A number with no source, or a source with no license line.** The library test walks every numeric leaf of every entry, refuses a missing source or grade by the field's path, and refuses a source id that `SOURCES.md` does not list.
2. **A part file read as a block and written back changes a bit.** Every float and double survives text and back, bit for bit, as the model file's do (`%.9g` floats, shortest round-trip doubles).
3. **A fitted inertia smaller than the parts' own.** The remainder part would have negative principal moments. It is refused, naming the axis.
4. **An old build meets a version 2 index.** It refuses it as `schema_mismatch`, naming both versions. A version 1 index is still read by the new build.
5. **The UI shows an `assumed` number as if it were measured.** The grade reaches the library API (`LibraryEntry`), so Interface can show it.

---

## Task L1: The sources registry and the grades

**Files:** create `assets/library/SOURCES.md` (the bibliography, human-readable) and `assets/library/sources.yaml` (the machine list: `id`, `citation`, `url`, `retrieved`, `license`, `license_evidence`). Create `engine/scene/library_sources.{hpp,cpp}`: `load_sources()`, `enum class Grade { measured, published, retailer, derived, fitted, assumed }` and `struct SourceRef { std::string source; Grade grade; std::string note; }`. Test: `tests/test_library_sources.cpp`.

- [ ] RED: `SourcesListEveryLicense`: every source has a non-empty `license`, one of `public_domain | facts_cite | not_usable`. `UnusableSourcesFeedNothing`: an entry citing a `not_usable` source is refused by name. `UnknownGradeIsRefused`.
- [ ] GREEN: the reader, one parser (TD-9), and the canonical text.
- [ ] Gate; commit.

## Task L2: Part files and library version 2

**Files:** create `engine/scene/part_file.{hpp,cpp}`. It has `motor_to_yaml` / `motor_from_yaml` and the same for prop, ESC and battery: `vehicles::MotorBlock` and the others, key for key, plus `sources:`, a map from each field's name to a `SourceRef`. The prop file has an optional `blade: {r_over_r, c_over_r, beta_deg, blades, hub_cutout, airfoil}` block for P-b. Modify `engine/scene/library.{hpp,cpp}`: `kLibraryVersion = 2`, `LibraryKind` gains `motor`, `prop`, `esc` and `battery` by suffix, and a version 1 index reads. Test: `tests/test_part_file.cpp`, plus additions to `tests/test_library.cpp`.

- [ ] RED:
  - `PartFilesRoundTripBitForBit`;
  - `AFieldWithoutASourceIsRefusedByPath`;
  - `UnknownKeysAreRefusedBlamedOnTheVersion`;
  - `LibraryVersion1StillReads`;
  - `LibraryVersion2NamesThePartKinds`;
  - `AnOlderBuildRefusesVersion2` (via the version gate's message).
- [ ] GREEN.
- [ ] Gate; leg (format).

## Task L3: `basic_quad_airframe()`

**Files:** create `engine/vehicles/basic_quad.{hpp,cpp}`. `BasicQuadParams` holds: arm x and y half-spans; a centre-box size and mass; arm tube radius and mass per metre; the motor, prop, ESC and battery blocks; and `std::optional<double>` for each of mass, inertia diagonal and kappa, each a fit target. The function returns `Result<AirframeSpec>` with a remainder part when a fit target is set. Test: `tests/test_basic_quad.cpp`.

- [ ] RED:
  - `TheCompiledBodyMeetsAFittedMassAndInertiaExactly`: the competition drone's 0.700 kg and [0.0025, 0.0021, 0.0043] at its layout;
  - `AFittedInertiaBelowThePartsIsRefusedNamingTheAxis`;
  - `AFittedKappaSetsCqStatic`;
  - `TheDefaultsCompile`, with every assumed field reported by `basic_quad_assumptions()`, so the UI can list them.
- [ ] GREEN.
- [ ] Gate; leg (new engine .cpp).

## Task L4: The part entries (needs P-a for the props)

**Files:** under `assets/library/parts/`, each file with its `sources`:
- props: `uiuc_apce_10x5`, `uiuc_apce_9x4.5`, `uiuc_mit_5x4`, `uiuc_cfnq_45t1`, `p6t_6inch_assumed`;
- motors: `phantom3_2312_960kv`, `iris_920kv`, `fpv_2207_1855kv`, `cf_7x16_coreless`, `fpv_2306_1700kv`;
- batteries: `phantom3_4s_4480`, `iris_3s_5100`, `fpv_6s_1300`, `cf_1s_250`;
- ESCs, one per drone.

The generator `tests/test_library_parts.cpp` writes the files with `SPADE_LIBRARY_WRITE`, as the objects' generator does.

- [ ] RED: each file must exist and load; each prop's tables must match Physics' P-a numbers bit for bit.
- [ ] GREEN: generated.
- [ ] Gate; leg (goldens).

## Task L5: The four drones and the basic quad

**Files:** create `tests/library_drones.hpp` (the `AirframeSpec` of each drone, built from the part files, with every body dimension's source) and `tests/test_library_drones.cpp` (the generator and the checks). The output is five model files plus their `.sources.yaml` sidecars under `assets/library/drones/`, and five new index entries.

- [ ] RED:
  - per drone: `<Drone>FileIsItsCompiledSpec` (the golden);
  - `<Drone>EveryInputCitesASource`;
  - the checks: `Phantom3HoversAtNasasBaselineRpm`, `IrisPlusHoversAtNasasBaselineRpm`, `CrazyflieMatchesBitcrazesThrustTable`, `FpvThrustToWeightIsInItsClass`, `BasicQuadDefaultsHover`.
- [ ] GREEN: generated.
- [ ] Gate; leg (goldens).

## Task L6: The reference quad is labelled a test fixture

**Files:** `assets/library/library.yaml`: rename the entry "Reference quad" to "Reference quad (test fixture)" and move it to the category Interface names. Add `assets/library/drones/reference_quad.sources.yaml`, which says its prop table is a synthetic linear ramp (C_T 0.12 to 0.04, C_Q 0.008 to 0.004 over J 0 to 0.8, `tests/reference_airframe.hpp:60-61`), graded `assumed`, and that the drone is a fixture. The model file does not change by a byte.

- [ ] RED: `TheReferenceQuadIsLabelledAFixture` (its index name and its sidecar).
- [ ] GREEN.
- [ ] Gate.

## Task L7: Realistic-looking drones (added by the lead, 2026-10-21)

Cameron asked for a realistic-looking drone, and boxes and tubes alone don't meet that. Each drone gets meshes generated in our own code from published dimensions, never from downloaded CAD:
- **props**, lofted from P-b's blade definitions: r/R, c/R and β, with an assumed airfoil section;
- **motor bells**;
- **a body shell** with rounded edges;
- **arms**, as tubes with end caps.

Two rules on solids:
- **Props are drawn only.** Rotor parts stay out of the solids; the rotor model carries the props in the air.
- **The body shell is a solid** when it is closed and every feature is at least 2 dx. Otherwise the SDF parts stay the solid, and the shell is visual only.

Rendering already draws mesh parts (`render_parts`). The tasks are drafted after L1-L3.

## Rulings (the lead, 2026-10-21)

The plan is approved in shape: the grades, the license lines, refuse-never-skip and no golden moves.

| Question | Ruling |
|---|---|
| 1 | (a) A `sources.yaml` sidecar |
| 2 | (a) The reference quad is labelled a fixture |
| 6 | (a) The measured curve gets its own later plan |
| 7 | (a) NASA's stated numbers as check points. TM-2018-219758 was not found by the lead either |
| 8 | (a) An assumed OCV table until Physics finds an open one |
| 3 | (Cameron, 2026-10-21) Our own resampled UIUC tables, cited as UIUC asks; never their files |
| 4 | (Cameron) **No APC data at all.** That includes the shapes Kat's catalog derived from APC's PER3, so Kat's parts that took their shape from APC feed nothing. Blades come from UIUC's measured geometry |
| 5 | (Cameron) The competition fit **goes in the library**: "The Grand Prix published nothing, these were derived sim values and require no concern. Have them in the library." |

**The competition variant (from answer 5).** `drones/basic_quad_competition.model.yaml` is `basic_quad_airframe()` with Kat's competition-700 values:
- Every value is graded `fitted` or `assumed`, and carries Kat's own provenance: source, confidence and date, from `competition-700.airframe.json`.
- Its sidecar states the caveat: fitted to the AI Grand Prix VQ1 simulator, not hardware.
- It joins Task L5, with a check that the compiled hover thrust matches the curve's hover point (norm 0.264 at 9.712 m/s²) within Physics' P-c tolerance.

All eight questions are answered. The build order is B7b, B0, then L1 onward.

**P-a and P-b move to Core.** Physics now carries the GPU solver, so Core drafts the prop tables and the blade definitions from the UIUC files. Physics reviews the resampling rule and the result. Physics keeps P-c (the chain fits and tolerances) and the kappa view.

**Order:** B7b's GREEN, then B0 from Physics' GPU plan (`c7a1be2`), then this library.

## Open questions for the lead (each one Cameron's, by AskUserQuestion)

1. **Where a model's citations live.** (a, recommended) A generated sidecar `<name>.sources.yaml` beside each model file, so the model format does not change. (b) A `sources` key inside the model block: a scene version 6 and model_version 2. (c) Only in the generator's source code.
2. **The reference quad.** (a, recommended) Label it a test fixture: no golden or identity moves. (b) Replace its prop table with real data. That moves its golden, its pinned `model_identity` and the tests built on `reference_quad()` (about 12 files).
3. **UIUC's data.** UIUC states no license. (a, recommended) Store only our own resampled tables, cited as UIUC asks; measured numbers are facts. (b) And ask UIUC for written permission first. That is an outward-facing email, so Cameron's to send. (c) Use no UIUC numbers.
4. **APC.** APC's terms forbid redistribution. (a, recommended) Use no APC file or number, including the shapes Kat's catalog derived from APC's PER3. The actuator line's blades come from UIUC's measured geometry. (b) Ask APC for permission (an outward-facing request).
5. **The competition drone in a public repo.** Its values are fitted to the AI Grand Prix VQ1 simulator, and the competition's rules may restrict publishing them. (a, recommended) The library's basic quad uses generic 6-inch defaults. The competition fit stays in Kat as a `BasicQuadParams` set it applies at run time. (b) Publish the fitted set in the library after Cameron checks the rules.
6. **The measured thrust curve.** (a, recommended) Not in this plan. A later plan carries it (Physics' mode, model_version 2 and scene version 6), and here the competition quad is fitted through parts. (b) In this plan, before L5.
7. **NASA's tabulated data.** (a, recommended) Use the 2016 paper's stated numbers only, as check points. (b) Digitise its hover figures as well (derived, with the method named). (c) Ask NASA for TM-2018-219758 (outward-facing).
8. **The LiPo OCV table.** No open, usable table was found. (a, recommended) Keep the chain's current OCV shape, graded `assumed`, until Physics finds one. (b) Physics derives one from a CC BY source (Pakjoo & Piegari 2026, not yet checked).

## Notes for later

- A drone's mesh for the editor: each drone is drawn from its SDF parts until a mesh is chosen. Any mesh must be made here from published dimensions, never downloaded CAD.
- Kat's catalog has more parts (3-inch, 7-inch, cameras, VTX, GNSS). They join as part files once the four kinds land, each with its sources.
