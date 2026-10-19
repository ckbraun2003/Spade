# Kat's builder, slice 1: the contract with Spade

**Owner:** Interface (Spade's side) and Kat's Editor (Kat's side). Spade Host owns and reviews the facade's contract.
**Status:** agreed between Interface and Kat's Editor on 2026-10-19, as a read-only design exchange; for the lead. Building starts when Editor is free.
**Asked by:** the lead (2026-10-19). Kat's lead adopted the first builder slice: the builder writes a case, shows `case_issues()` and `resolve()`'s grid and box, and queues a DRY run through `launch_run()` and the queue.
**Read against:** Spade 3302e8d, and Kat's prefix bump to Spade bf93683 (staged; it has `include/analysis/`, `lib/spade_analysis.lib` and `bin/spade_analysis_runner.exe`). Kat's side is Editor's note, `editor-c/.superpowers/sdd/builder/slice1-host-design.md`.

## 1. What Kat has, and what slice 1 builds

- Kat writes no model, part or case file today. Its editor writes `project.kproj`, `scenes/*.kscene` (v7, `instances[].airframe_ref`), and `packages/<name>/{manifest,graph,airframe}.json`.
- Slice 1 builds the drone's `ModelType` in memory from its `airframe.json`, through kat_host_spade's existing converter (`quadrotor_model_from_airframe()`), and passes it to `resolve()`. Kat writes no model file in slice 1.
- Kat's law holds: only `dronesim/spade/` includes Spade headers. The facade is a C-ABI DLL there, in kat_host_spade's pattern. The case crosses it as JSON with Spade's YAML key names, so a `CaseIssue` path names its JSON node and its form widget directly. u64 values travel as 16-hex strings. The facade maps; it never validates. Spade's checks are the only checks.

## 2. Files and folders

| Path, under the project | What | Written by |
|---|---|---|
| `analysis/<case>.case.yaml` | the case, `model.file = "<case>.model.yaml"` | `save_case()` |
| `analysis/<case>.model.yaml` | Core's model file | not in slice 1 |
| `analysis/runs/` | `runs_dir`: `queue.txt` and the run folders | the queue, `create_run_folder()` |

The runs stay apart from Kat's trainer runs. A case pins a model, not a scene or package.

## 3. The calls

Headers install at `include/analysis/` (`#include "analysis/case.hpp"`), target `spade::analysis`.

| Step | Call | Thread |
|---|---|---|
| The form | `case_issues(case)`: every problem, in field order, `{field, message}`, shown verbatim at its field. Issues with no widget (`model`, `model.identity`, `rotors`) go in a list above the form. | GUI |
| The resolved case | `resolve(case, model)`: the grid `nx×ny×nz` and `dx`, the box (`n·dx`, drawn as a wireframe cube on the drone), nests and the finest spacing, `base_bytes`, each rotor's R and R/dx, estimated polars, the six faces, the hash. A pin mismatch is "model.identity" or "model.parts"; Kat offers "Re-pin case". | GUI, 150 ms after the last edit |
| Save, load | `save_case`, `load_case` | GUI |
| Queue a dry run | `create_run_folder(runs_dir, case_name, resolved, date, commit)`, then `enqueue_run(QueueSetup{runs_dir, runner, LaunchOptions{RunKind::dry_run, pace_ms}}, folder)` | the run worker |
| Run the queue | `step_queue(setup)`, on a 1 Hz timer while `queue.txt` has entries | the run worker |
| The run list | `read_progress(folder)`: kind, state, simulated vs target, steps, elapsed, ms per step, memory, the message verbatim | the run worker |
| Cancel | `cancel_run(folder)` | the run worker |

- **One run worker** owns every call that touches `runs_dir`, in order: a Windows read or replace retries for up to about 1 s.
- **`runner`** is `spade_analysis_runner`'s absolute path. The DLL's CMake copies `$<TARGET_FILE:spade::analysis_runner>` beside the editor. It is never searched for.
- **A started run outlives the editor.** On open, a non-empty `queue.txt` restarts the timer.
- **Honesty (INT-8, and run.hpp's L6 for the dry-run label):** a default or derived value has a visible chip, in Spade's words ("default: 10 × reach", "derived: model rotor"). A dry run carries a DRY RUN badge everywhere it is shown. "Queue CFD run" stays disabled with "no airflow solver yet (W1)".
- **The date** is the host's UTC day, `YYYY-MM-DD`.
- **Memory:** `progress.memory_bytes` is what the run holds; a dry run's is about 0. The CFD need Kat shows is `ResolvedCase.base_bytes`, from `resolve()`.

## 4. What Spade adds for slice 1

| # | Change | Owner | Status |
|---|---|---|---|
| S1 | A dry run is EXEMPT from the memory gate: its need is recorded as 0, it starts at once, and its progress says "dry run: no memory gate". Real runs keep need plus margin. Kat measured 5.6 to 8.1 GiB free commit on the box while sessions work, so with the gate a dry run (0.2 GiB + 6 GiB) sometimes waited. | Interface | the lead's ruling, 2026-10-19; to build, with a test |
| S2 | `create_run_folder()` takes the kind, and a dry run's folder ends `_dry`. An existing folder is not refused: the lowest free `_2`, `_3`… is taken, so a cancelled or faulted run's folder is kept beside its re-run. It returns the path it made; a host never builds the name. | Interface | agreed; to build, with a test |
| S3 | `save_case()` is atomic: a temporary file, then a replace, as `progress.yaml` is written. | Interface | agreed; to build, with a test |
| S4 | `spade::build_identity()`, installed: the commit plus a dirty flag, stamped at build time through one shared `stamp_identity.cmake`, plus a `spadeConfig` variable. Until it exists, Kat's host refuses to make a run folder rather than invent a commit. | Core | assigned by the lead |

S1 to S3 land after bf93683, so Kat needs a further prefix bump for them. Kat's lead schedules it.

## 5. Before a real run (after slice 1)

- **Core's model file** (`model_version 1`), with a round trip that keeps `model_identity()`. Kat then writes `<case>.model.yaml`.
- **The run folder holds its own copy of the model**, with the run's `case.yaml` re-pointed to it, so a run never depends on the project's copy. Imported meshes travel as mesh parts with their `.spmesh` sidecars, copied the same way.
- **The chain pin:** the case gains idle and the state of charge, and pins `propulsion_identity()`. The chain lives in the model file's `propulsion:` block.
- **`queue.lock`** before a second host (a headless runner, a CLI) can step the same `runs_dir`. Today `queue.txt` has no lock, and one editor per project gives one host per `runs_dir`.
