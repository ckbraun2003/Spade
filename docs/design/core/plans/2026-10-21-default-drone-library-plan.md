# The Default Drone Library — Plan (approved, 2026-10-21)

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking. **Reviewed and approved by the lead, 2026-10-21; every open question is answered (Rulings).**

**Goal:** The library gets six drones and the parts they are made of.
- Four drones are real drone classes, each with a body, rotors, motors and a battery: a Phantom 3-class quad, an Iris+-class quad, a 5-inch FPV quad and a Crazyflie-class micro.
- The other two are one basic parametric quad (frame, arms, rotors) in two forms: generic 6-inch defaults (`basic_quad_6inch`) and the competition fit (`basic_quad_competition`, ruling 5).
- Motors, propellers, ESCs and batteries become library entries of their own.
- Every number in every entry names its source and its grade, and every source has a license check.
- The reference quad's two-point prop table is labelled plainly as a test fixture (ruling 2).
- Each drone looks like one: lofted props, motor bells, a body shell and capped arms, as meshes generated in our own code (Task L7).

**Architecture:**
- **Parts are what the airframe builder already holds.** `vehicles/airframe_compile.hpp` has `MotorBlock`, `PropBlock`, `EscBlock` and `BatteryBlock`. A part entry is one of those blocks as a file (`*.motor.yaml`, `*.prop.yaml`, `*.esc.yaml`, `*.battery.yaml`) plus a `sources` block. `scene/part_file.hpp` holds one emitter and one parser per kind (TD-9).
- **A drone is an `AirframeSpec` compiled.** It is built in code from its body parts (published dimensions) and the part entries, loaded from their files. `compile_airframe()` turns it into the model file, which is a generated golden, the same way the reference quad is generated today (`tests/test_library_quad.cpp`). A generated sidecar beside it, `<name>.sources.yaml`, cites every input number and names the part entries it was built from, by file and content hash (ruling 1).
- **The basic quad is a function.** `vehicles/basic_quad.hpp`: `basic_quad_airframe(BasicQuadParams) -> Result<AirframeSpec>`. It takes the layout, prop, motor, ESC and battery, and an optional fitted mass, inertia and kappa. A fitted mass and inertia are met exactly by a computed remainder part, so `compile_airframe()` does not change. A remainder that is not a physical body is refused by name.
- **The library index goes to version 2**, with the four part kinds. A version 1 index still reads.

**Tech Stack:** C++23, GoogleTest through ctest, MSVC + Ninja presets, gcc-13 through the leg. No new dependency. No kernel, binding or Slang change.

**Spec:** the backlog row (Cameron, AskUserQuestion 2026-10-21, master `6657069`), the lead's brief of 2026-10-21, the Kat lead's competition-drone parameters (relayed 2026-10-21), and Cameron's answers to questions 3-5 (relayed by the lead, 2026-10-21; the backlog on master). Kat's documents, read in place and read-only: `KAT/design-specs/integration/drone-builder-datapack.md` §1 and §7, `drone-builder-catalog.md` (Kat's generic part catalog, 2026-10-03), and `content/airframes/competition-700.airframe.json`. Research notes behind the source tables: kept in the Core session's scratchpad, not the repo. Drafted at master `0076fd1`; each task branches from master when it starts.

## Global Constraints

- **Published numbers only; no copied CAD, mesh or data file** (the repo is public). A drone's geometry is boxes, cylinders and tubes sized from published dimensions. A data table in the repo is our own resampling of a cited source, never a copy of a source's file.
- **Every number cites its source and its grade, in the entry's own file** (the part files) or in its generated sidecar (the model files). The grades are:
  - `measured`: a test stand or tunnel, in a primary source;
  - `published`: the manufacturer's own specification;
  - `retailer`: a seller's or a third party's listing; weaker, and named so;
  - `derived`: computed from cited numbers by code in this repo, which the entry names;
  - `fitted`: fitted against data (for example, Kat's flight logs of the VQ1 simulator);
  - `assumed`: a plausible default with no source for this airframe. It is named so in the UI too (Interface).
- **Every source has a license line**: public domain, facts-only-cite, or not usable. A source that forbids redistribution feeds no number in the repo. **No APC data at all** (ruling 4): not APC's PE0 and PER files, and not the shapes Kat's catalog derived from APC's PER3. A Kat part that took its shape from APC feeds nothing.
- **No golden moves, and every identity holds.** The reference quad stays byte for byte. Its pinned `model_identity` and every test built on `reference_quad()` stay put. Its library index entry is relabelled only (Task L6). New drones add new generated goldens. Each is final only once a fresh (`-NoSeed`) Docker leg reproduces it (TD-12).
- **Determinism:** all derivations run in double, in a fixed order, with `+ − × /` and `sqrt` only (TD-3: no libm under `engine/`). They round to float once, at the model's own fields.
- **Refuse, never skip (L6).** A part file with an unknown key, a field without a source, a source without a license line, or a grade that is not one of the six is `invalid_argument`, naming the file, the field and the source id.
- **Layering:** `vehicles/basic_quad.*` includes `vehicles/` and `core/` only. `scene/part_file.*` includes `vehicles/airframe_compile.hpp` for the block types and nothing from `sim/`.
- **The tiered gate per task**: release build, release CPU suite and `gcc-check.sh` on every changed file. A `-NoSeed` leg runs for each task that adds or changes a golden or a format (L2, L3, L4, L5, L7). All heavy runs go through `jobrun.py` with tickets `spade-core-<what>-<sha>`.
- **Where the code lives:** one branch per task, cut from master: `core/library-parts` (L1-L2), `core/basic-quad` (L3), `core/library-props` (L4a), `core/library-drones` (L4-L5), `core/library-fixture` (L6), `core/library-meshes` (L7). Never push. Never stash.

## What the sources give, and what they don't (the license check)

The detailed notes, with URLs, pages and quotes, are in the research notes named under **Spec**. Task L1 turns this table into `assets/library/SOURCES.md`, the library's bibliography.

| Source | What it gives us | License | Grade |
|---|---|---|---|
| NASA Ames, Russell, Jung, Willink, Glasner, "Wind Tunnel and Hover Performance Test Results for Multicopter UAS Vehicles", AHS 72nd Forum, 2016 (NTRS 20160007399; a conference paper, not a TM) | Phantom 3 Adv. and Iris+: rotor diameter (9.4 and 9.6 in), rotor-to-rotor spacing (9.8 × 9.9 and 10.4 × 16.1 in), nominal weight 2.8 lb, baseline hover 5,300 and 5,400 rpm at 2.8 lb, hover rpm ranges, figure of merit (0.58-0.67 Phantom, 0.72 Iris+ peak), Phantom motor-ESC efficiency (50-77 %). Hover and tunnel results are **figures only**; no tables | Public domain: "This is a work of the U.S. Government and is not subject to copyright protection" (p.1); NTRS "public use permitted" | measured |
| NASA/TM-2018-219758 (the follow-on data report) | the tabulated data. **Not found** on NTRS and not read | public domain if found | — (ruling 7: the 2016 paper's stated numbers only, as check points) |
| DJI Phantom 3 spec pages | mass 1216 g (Standard) / 1280 g (Adv., Pro), diagonal 350 mm, 4S 4480 mAh 15.2 V 68 Wh 365 g pack | facts only, cite (DJI's site terms not read) | published |
| Retailers (Phantom 3 spares) | motor 2312, 960 KV; prop 9450, pitch 4.5 or 5.0 in (they conflict) | facts only, cite | retailer |
| Adafruit / iFixit (Iris+) | 1282 g with battery, 550 mm motor to motor, 920 KV, 9.5 × 4.5 props, 3S 5.1 Ah 8C | facts only, cite (no 3DR primary read) | retailer |
| UIUC Propeller Database, Vols 1-4 (Brandt, Deters, Ananda, Dantsker, Selig) | measured static C_T, C_P against rpm; C_T, C_P against J at set rpm; blade geometry r/R, c/R, β. Has: APC Thin Electric 10x5 (NASA calls it close to the Phantom's rotor), APC TE 9x4.5 and SF 10x4.7 (near the Iris+'s), GWS 5x4.3 and Micro Invent 5x4 tri-blade (5-inch), Crazyflie 1.85-in prop (static only). Has **no** DJI 9450, **no** 6-inch prop and **no** commercial FPV prop | **Unknown**: no license or terms on the site, only a citation request (quoted in SOURCES.md) and "© 1994-2026 UIUC Applied Aerodynamics Group". We store our own resampled tables, cited as UIUC asks, never the files (ruling 3) | measured |
| APC PE0 (geometry) and PER (performance) files | computed (vortex theory) performance, and blade geometry | **Not usable**: APC's terms prohibit "reproduction, distribution, display or transmission" of site content | — |
| Bitcraze Crazyflie 2.1 datasheet, Rev 3 | 29 g takeoff, 92 × 92 × 29 mm motor to motor incl. feet, 1S 250 mAh, 7 × 16 mm coreless motors | facts only, cite | published |
| Bitcraze "PWM to thrust" page (2015) | one motor's current, thrust, voltage and rpm at 16 PWM steps (to 57.9 g at 23,882 rpm) | facts only, cite (Bitcraze's license page 404s) | measured |
| Luis & Le Ny, arXiv:1608.05786, Table 2.3.1 | Crazyflie 2.0 arm 39.73 mm, rotor radius 23.13 mm, inertia diag(1.395, 1.436, 2.173)e-5 kg·m² | facts only, cite (arXiv, author copyright) | measured, second-hand |
| Retailers (5- and 6-inch FPV parts) | e.g. a 2207 1855 KV motor: 50 mΩ, 35.08 A peak, 31.6 g; 6S 1300 mAh packs ~240 g; HQ 6x3.5x3 props ~6.4 g | facts only, cite | retailer |
| Kat: `competition-700.airframe.json` and the datapack | the competition drone, **fitted to the AI Grand Prix VQ1 simulator, not hardware**: mass 0.700 kg, X layout ±0.075 × ±0.10 m, the measured thrust curve (0 to 52.5 m/s², 9.188 N per motor), airspeed loss (k 0.00028, exp 2, floor 0.35), drag 0.028 kg/m, tau 0.033 s, kappa 0.022 m (flagged), inertia (assumed; it equals NeuroBEM's published tensor) | **In the library** (ruling 5; Cameron: "these were derived sim values and require no concern"). Each value keeps Kat's provenance | fitted / assumed |
| Kat's generic catalog (`drone-builder-catalog.md`) | generic parts P5T, P6T, REF5, M2207-1750/2450, packs, ESCs. Its J-tables take their **shape** from APC's PER3 files | **Not usable** wherever APC gave the shape (ruling 4); a part ID may still name a part that took nothing from APC | — |
| LiPo OCV against state of charge | **no usable table found** under an open license. Zhao & Howey's printed coefficients are too rounded to use | — | — (ruling 8: the chain's current OCV shape, `assumed`) |

**What this means, plainly.** The two NASA-tested drones get primary-source size, mass, rotor diameter and a hover check point, but their prop curves are a near-match UIUC prop, not their own. The FPV quad and the 6-inch defaults stand mostly on retailer numbers. The Crazyflie is the best covered: a datasheet, a measured prop and a measured thrust table. Each entry says which of these it is.

## Per drone: the inputs and their grades

**Phantom 3-class** (`drones/phantom3_class.model.yaml`):
- Mass 1.280 kg (DJI, published).
- Rotors at the corners of a 249 × 251 mm rectangle (NASA's 9.8 × 9.9 in; measured), which gives the 350 mm diagonal DJI states.
- Rotor diameter 0.2388 m (NASA 9.4 in; measured).
- Motor 960 KV (retailer); its resistance and no-load current are assumed.
- 4S 4480 mAh pack, 365 g (DJI, published). Cell resistance is assumed; the OCV is the chain's current shape, `assumed` (ruling 8).
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
- X layout ±0.080 × ±0.10 m, so same-side hubs clear the 6-inch discs by 7.6 mm (the lead's ruling (b), 2026-10-21). Kat's ±0.075 × ±0.10 m is the competition drone's.
- A 6x3.5x3 prop. Its curve is our own scaling from the nearest UIUC 5-inch tri-blade, so **derived and assumed**, because UIUC has no 6-inch prop.
- 2306 1700 KV motor, 6S 1300 mAh pack (retailer).
- Mass is the sum of the parts.
- It uses no Kat catalog prop: those took their shape from APC (ruling 4).

**Basic quad, competition** (`drones/basic_quad_competition.model.yaml`; ruling 5). This is `basic_quad_airframe()` with Kat's competition-700 values (`competition-700.airframe.json`, read 2026-10-21).
- **Grades.** Kat's labels `measured`, `tuned` and `derived` are all against the AI Grand Prix VQ1 simulator, so they grade `fitted` here. Kat's `assumed` stays `assumed`. The sidecar carries Kat's own source, confidence and date for each value, and states the caveat: fitted to the AI Grand Prix VQ1 simulator, not hardware.
- Mass 0.700 kg (Kat: tuned, medium, 2026-07-27), `fitted`. It is met by the remainder part.
- Inertia diag(0.0025, 0.0021, 0.0043) kg·m² (Kat: assumed, low, undated; it equals NeuroBEM's published tensor), `assumed`. It is met by the remainder part.
- X layout ±0.075 × ±0.10 m (Kat: derived, medium, 2026-07-29), `fitted`.
- Kappa 0.022 m (Kat: derived, flagged), `fitted`, set as `cq_static = kappa · ct_static / D`. Physics' kappa view applies.
- Drag 0.028 kg/m, componentwise (Kat: tuned, high, 2026-07-29), `fitted`. It becomes a `DragSpec` if Physics confirms Kat's form is the engine's componentwise mode (P-c).
- Maximum thrust 9.188 N per motor (Kat: derived from its thrust curve). The parts are chosen so the compiled maximum meets it within P-c's tolerance. They follow Kat's best candidate set (datapack §7.2): the 6-inch tri-blade below, a 2207 2450 KV motor on 4S, and R_eff about 3× the datasheet value (`fitted`). The motor and pack values are `retailer` where cited, `assumed` otherwise.
- **Not carried in this plan:**
  - the thrust curve and the airspeed loss (ruling 6: a later plan);
  - tau 0.033 s, the envelope and the rate loop (controller values; the airframe has no field for them);
  - the collision radius 0.3 m (it becomes `proxy_radius` only if Physics agrees).
- **Check:** the compiled hover thrust matches the curve's hover point (norm 0.264 at 9.712 m/s²) within P-c's tolerance.

## Inputs from Physics, Interface, Kat and Test/Docs

- **P-a: the propeller tables. Core drafts them (Task L4a); Physics reviews the rule and the result.** For each prop above: C_T(J) and C_Q(J) points (UIUC convention, C_Q = C_P / 2π), our own resampling of the UIUC static and wind-tunnel runs, never their files (ruling 3). `compile_airframe()` puts them on the chain's uniform 33-point grid itself (`propeller_resample_table()`). The tables carry `ct_static`/`cq_static` and their grades. A static-only prop (the Crazyflie's) keeps a static table and says so, unless Physics' review gives a J dependence.
- **P-b: the blade definitions for the actuator line. Core drafts them (Task L4a); Physics reviews.** For each prop: r/R, c/R and β(r/R) from the UIUC geometry files (measured), the blade count and the hub cut-out, in the prop file's optional `blade` block (Task L2). The airfoils are unknown for every one of these props. The polar the actuator line uses is Physics' choice, graded `assumed`.
- **Physics (P-c): the chain's fit to each drone.** Confirm the coreless DC motor fits the chain's equations. Set the effective resistance where the datasheet value cannot reproduce a measured point (Kat measured R_eff ≈ 3× on its rebuild). Set the tolerance of each drone's check (above).
- **Physics (P-d): a measured thrust curve. Not in this plan (ruling 6).** The competition drone's curve (throttle to thrust, with the airspeed loss) as a propulsion mode is a model-format change (model_version 2, scene version 6), so it gets its own later plan.
- **Physics: kappa.** The competition drone's 0.022 m against the catalog props' 9-12 mm. `basic_quad_airframe()` takes a fitted kappa as `cq_static = kappa · ct_static / D` and labels it `fitted`, but Physics says whether 0.022 m is believable.
- **Interface:** the Library panel's categories for the part kinds, and the "assumed" marking in the UI. Swapping a part into a drone is the builder's (Interface and Kat), later.
- **Kat:** aggregates from its flight logs if a check needs them (through the lead). A Kat part ID (M2207-…) names a part file only where the part is the same and took nothing from APC. Kat's props (P5T, P6T, REF5) took their J-table shapes from APC and feed nothing (ruling 4).
- **Test/Docs:** a validation row per drone check (above), naming the source and the tolerance.

## Review Focus

1. **A number with no source, or a source with no license line.** The library test walks every numeric leaf of every entry, refuses a missing source or grade by the field's path, and refuses a source id that `SOURCES.md` does not list.
2. **A part file read as a block and written back changes a bit.** Every float and double survives text and back, bit for bit, as the model file's do (`%.9g` floats, shortest round-trip doubles).
3. **A fitted inertia smaller than the parts' own.** The remainder part would have negative principal moments. It is refused, naming the axis.
4. **An old build meets a version 2 index.** It refuses it as `schema_mismatch`, naming both versions. A version 1 index is still read by the new build.
5. **The UI shows an `assumed` number as if it were measured.** The grade reaches the library API (`LibraryEntry`), so Interface can show it.
6. **A competition value loses Kat's provenance.** The sidecar test walks `basic_quad_competition`'s inputs and refuses one without Kat's source, confidence and date, or without the simulator caveat (Task L5).

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

## Task L4a: The prop tables and blade definitions (P-a, P-b; Core drafts, Physics reviews)

**Files:** create `tools/library_props/resample_uiuc.py` (standard-library Python). It reads the UIUC files from a local folder named on its command line. The files are downloaded by hand and never committed (ruling 3). It writes `tests/library_props.gen.hpp`: our own numbers as C++ constants, and comments naming each prop's UIUC volume, file, run rpm and retrieval date. Test: `tests/test_library_props.cpp`.

**The resampling rule (Physics reviews it before the numbers are generated):**
- **Run:** for each prop, the UIUC wind-tunnel run whose rpm is nearest the drone's hover rpm, so the Reynolds number is the drone's.
- **J = 0:** C_T and C_P from the static run, linearly interpolated in rpm to the chosen run's rpm.
- **The table:** C_T and C_P linearly interpolated in J, in double, at J = 0.05 k, for k = 1 up to the run's last measured J (no extrapolation). Then C_Q = C_P / 2π.
- **Blade (P-b):** r/R, c/R and β linearly interpolated from the geometry file onto r/R = 0.15, 0.20, …, 1.00. The hub cut-out is the file's first station. The blade count comes from the prop's own name.
- **The 6-inch tri-blade** for the basic quads: `uiuc_mit_5x4`'s coefficients, applied at 6 in. Graded `derived` and `assumed`, since UIUC has no 6-inch prop.
- **The output:** shortest round-trip doubles, so the C++ side never recomputes a number.

- [ ] RED: `TheGeneratedTablesAreWhatTheRuleGives`: the tool re-run on a small synthetic input committed beside it (our own numbers, not UIUC's) reproduces its expected header byte for byte. `EveryPropTableStartsAtItsStaticPoint`. `NoTableExtendsPastItsRun`.
- [ ] GREEN: the tool. The header is generated from the UIUC files and reviewed by Physics before it is committed.
- [ ] Gate (no leg: a test header and a tool; gcc on the test).

## Task L4: The part entries (needs L4a for the props)

**Files:** under `assets/library/parts/`, each file with its `sources`:
- props: `uiuc_apce_10x5`, `uiuc_apce_9x4.5`, `uiuc_mit_5x4`, `uiuc_cfnq_45t1`, `triblade_6inch_scaled`;
- motors: `phantom3_2312_960kv`, `iris_920kv`, `fpv_2207_1855kv`, `cf_7x16_coreless`, `fpv_2306_1700kv`, `comp_2207_2450kv`;
- batteries: `phantom3_4s_4480`, `iris_3s_5100`, `fpv_6s_1300`, `cf_1s_250`, `comp_4s` (its capacity `assumed`);
- ESCs, one per drone.

The generator `tests/test_library_parts.cpp` writes the files with `SPADE_LIBRARY_WRITE`, as the objects' generator does.

- [ ] RED: each file must exist and load; each prop's tables must match `tests/library_props.gen.hpp` bit for bit.
- [ ] GREEN: generated.
- [ ] Gate; leg (goldens).

## Task L5: The four drones and the two basic quads

**Files:** create `tests/library_drones.hpp` (the `AirframeSpec` of each drone, built from the part files, with every body dimension's source) and `tests/test_library_drones.cpp` (the generator and the checks). The output is six model files plus their `.sources.yaml` sidecars under `assets/library/drones/`, and six new index entries.

- [ ] RED:
  - per drone: `<Drone>FileIsItsCompiledSpec` (the golden);
  - `<Drone>EveryInputCitesASource`;
  - the checks: `Phantom3HoversAtNasasBaselineRpm`, `IrisPlusHoversAtNasasBaselineRpm`, `CrazyflieMatchesBitcrazesThrustTable`, `FpvThrustToWeightIsInItsClass`, `BasicQuadDefaultsHover`, `BasicQuadCompetitionHoversAtTheCurvesHoverPoint`;
  - `BasicQuadCompetitionCarriesKatsProvenance`: every input has Kat's source, confidence and date, and the sidecar states the simulator caveat.
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

**The lead's ruling (2026-10-21): no format change.**
- **Props are mesh parts on their rotor's hub** (`attach: rotor`, mounted in the hub frame; `vehicles/model_part.hpp`). They are drawn and posed by the blade azimuth, and are never solids: the solids take only `PartFilter::rigid_only`. The rotor model carries the props in the air.
- **The body, arms and motor bells are closed solid meshes**, and they are the parts themselves: drawn, and the airflow solids. Every feature is at least 2 dx at D1's 4 mm, that is 8 mm. A rounded body volume and 12 mm arm tubes meet that. A detail below 8 mm (an antenna, a GPS mast) stays out of L7.
- **Authoring a prop mesh on a rotor is a compile addition.** The model file already carries rotor parts. `AirframeSpec` cannot author one yet: its records (`PartInertia` + `PartGeometrySpec`) have no attach or rotor, and the compile gathers no part from a propeller. L7 adds that to `vehicles/`, with the prop's mass staying in `PropBlock.mass`, so no mass is counted twice.
**The checks, answered (2026-10-21).**
- **Rendering:** `render_parts` draws posed rotor parts, mesh parts included, on GL and on the CPU.
  - Spinning needs Physics' blade azimuth ψ (P11), and two callers that pose with ψ = 0 today: `sandbox/vehicle_draw.hpp:110-112` and `tools/flow_movie/main.cpp:880`.
  - The opposite-hand prop needs its own mirrored mesh, re-wound so it faces outward. A flipped mount can't make one.
- **Physics** (`docs/design/physics/plans/2026-10-13-airflow-solids-design.md` §3-4): one wall per rigid part, with no sheet. Body, 4 arms and 4 bells give "9 parts, 9 walls, 0 sheets". The reference quad's 12 mm arms are already clean walls at 4 mm. L7's meshes keep these rules:
  - **The classifier.** A part is thin when min(declared `min_feature`, its smallest part-frame bounds extent) < 2 dx. Declare `min_feature` ≥ 8 mm, and keep every part's bounds ≥ 8 mm on all three axes.
  - **The measured safeguard.** Aim for ≥ 12 mm (3 dx) through every section. The 12 mm arms are clean at 4 mm, leak at 6 mm and are refused at 7 mm. No thin lips, flanges or shells on the body.
  - **Closed and outward-wound.** The sign of φ comes from the winding.
  - **Overlap is fine.** The solid is the parts' union, so sink arm-end caps and bell bases into the body or arm rather than placing them flush. Leave no gap between parts narrower than 8 mm; an air slot the grid can't resolve is the failure to avoid.
  - Physics checks each L7 model at 4 mm with `build_quad()`'s pattern (`tests/test_vehicles_airflow_solids.cpp`) once a generator emits it.

**Its outline.** The full tasks are drafted after L3 and go to the lead.
- **L7a, the mesh generators.** These are functions that build a `TriangleMesh` through `make_mesh()`:
  - a prop lofted from L4a's blade block, with an assumed section named in the sidecar;
  - a motor bell;
  - a rounded-box body shell;
  - an arm tube with end caps.

  Circles need sin and cos. TD-3 allows no libm under `engine/`, so they use `core/fp32_math`'s portable functions, or the meshes are generated outside `engine/` and committed as `.spmesh` assets. The draft picks one.
- **L7b, the drones wear them.** Each L5 model file gains its mesh parts, with `.spmesh` sidecars beside it (B5's format). Those goldens are new in this plan, so regenerating them moves no existing golden. Each one still needs its `-NoSeed` leg.
- **L7c, a prop mesh on each hub**: the compile addition above, and, from Rendering's answer (the lead, 2026-10-21), the two things a spinning prop needs:
  - **Posed by the blade azimuth.** Both callers that pose rotor parts with ψ = 0 today, `sandbox/vehicle_draw.hpp:110-112` and `tools/flow_movie/main.cpp:880`, take Physics' ψ (P11) instead. Those files are other realms' and go through the lead; L7c lands after P11.
  - **A mirrored mesh for the opposite hand.** L7a's prop generator emits both hands: the mirror image, re-wound so it faces outward. Each hub carries its own hand's mesh. A flipped mount can't make it.

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

**The competition variant (answer 5)** is under **Per drone: Basic quad, competition**, and its checks are in Task L5.

**P-a and P-b move to Core** (Task L4a). Physics now carries the GPU solver, so Core drafts the prop tables and the blade definitions from the UIUC files. Physics reviews the resampling rule and the result. Physics keeps P-c (the chain fits and tolerances) and the kappa view.

**Order:** B7b and B0 are merged. Next is L1, then L2 and L3, which don't depend on P-a. L4a goes to Physics for review before L4.

**The eight questions** and their options are in this file's history (`0a16708`). The answers are in the table above.

## Notes for later

- Until L7 lands, each drone is drawn from its SDF parts.
- **A visual-only carrier** (the lead, 2026-10-21) would be a render mesh beside the physics shape, the way game engines split render and collision. It is a later CORE-7 design, made only if a real detail needs it. L7 needs none.
- **L2's branch:** L1 merged first (`61724be`), so L2 is cut fresh from master as `core/part-files`.
- **L2's review minors go in L3's first commits, red first** (the lead, 2026-10-21, at L2's merge `1c65501`):
  1. A shape's unused member (`size` on a tensor shape, `tensor` on any other), when set, is refused by name rather than dropped on write (refuse, never skip), so every struct the writer accepts round-trips.
  2. Citing `stator` or `protocol` says "the motor's field 'stator' takes no citation", and the header says exactly which keys the "part schema v1" blame covers.
  3. A test for version 2's message on an unknown file suffix.
- **L3's review, the lead's rulings (2026-10-21; L3 approved at `db765ff`).** Core's rulings 1-4 are accepted: `half_span_x`/`half_span_z`; Kat's FLU (Ixx, Iyy, Izz) maps to design (x, y, z) = (0.0025, 0.0043, 0.0021); Kat's rotor order is not carried; kappa beside tables was refused in L3. Then, in an L3 follow-up after L3 merges:
  - **(a) Kappa scales the tables, it does not drop them.** L4a's thrust tables stay. `basic_quad_airframe()` derives C_Q(J) = κ·C_T(J)/D at the ct_table's J points, and a C_P table the same way if a prop ever carries one (C_P = 2π·C_Q). The derived table is graded `fitted`, naming κ's source. Physics confirms the formula before L5 builds on it.
  - **(b) The defaults are a valid generic 6-inch quad.** Same-side hubs clear D by at least 5 mm (half_span_x 0.080). The competition fit keeps Kat's derived ±0.075 layout, with its 2.4 mm disc overlap named in its assumptions. `basic_quad` gains an advisory, not a refusal: it names any pair of rotor discs closer than D, with the overlap in mm. The lead has asked Physics whether overlapping actuator discs need anything.
  - **Physics' answers (relayed by the lead, 2026-10-21).**
    - (a) is exact: T = C_T·ρn²D⁴ and Q = C_Q·ρn²D⁵ (n in rev/s), so C_Q(J) = κ·C_T(J)/D. A derived C_P = 2π·C_Q is metadata only, since the rotor model reads no power table.
    - (b) Overlapping discs are sound: each disc's increments add independently, and Peskin's kernel is a partition of unity, so each disc keeps exactly its own thrust and torque. The advisory also names what isn't modelled for any close layout, overlapping or not: a rotor's thrust does not respond to its neighbour's induced inflow (the shared wake).
  - **(c) The competition test prints its triangle margin** (I1 + I2 − I3 of the remainder, 1.74e-5 kg m² today), so a default change shows its cost before it becomes a refusal. L5's 2207/4S competition parts get their own margin line.
- **Thumbnails, one owner per entry** (the lead's condition on Interface's N14b, 2026-10-21; agreed with Interface the same day). Interface owns the generator (`tests/test_library_thumbnails.cpp`, 96 × 96 PNGs under `assets/library/thumbnails/`) and the 9 entries' thumbnails N14b adds. Each entry Core adds in L4 or L5 is Core's. The same commit adds its `thumbnail:` key and runs the generator with `SPADE_LIBRARY_WRITE=1`, so the PNG lands with the entry. N14b changes `tests/test_library.cpp`'s committed-index check from "no thumbnail" to "every entry has one".
- Kat's catalog has more parts (3-inch, 7-inch, cameras, VTX, GNSS). They join as part files once the four kinds land, each with its sources.
