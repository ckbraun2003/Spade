# The v1 baselines (`INT-4`, `SL14b`): spec and plan

**Owner:** Interface. **Status:** approved by the lead 2026-10-02, with all three decisions taken (see "Decisions"); storage amended the same day by Test/Docs's governance ruling. Tasks 1–4 done: captured 2026-10-03, merged `0935c6f`. Task 5, the guard, is in progress. `INT-4` (signed 2026-10-02): capture `SL14b`'s baselines now, while v1 still builds and runs. **Done when** (`../../backlog.md`): the baselines are committed, with how they were captured. Comparing successors against them is later work (`SL14b`'s P6), not this plan.

**v1 is not touched.** Nothing in `src/ include/ examples/ assets/` changes, and no capture shim goes into v1. The perishable baselines are taken from outside the running tools. The one code change is in `engine/tools/viewer/` (Interface's, v2 tool code), and only for the durable half.

## What the tools can do today

Read from source on 2026-10-02 (file:line references are in the review thread). Nothing was built or run.

| | `spade_viewer` (8 scenes, v2 physics, v1 rendering) | v1 `Sandbox` (3 scenes, v1 engine) |
|---|---|---|
| Invocation | `spade_viewer <scene> [cpu\|vulkan]`. Scenes: drop, bounce, shower, gate, hover, wind, flight, swarm | `Sandbox [fluid\|spheres\|cubes]`, run from `bin/`, because shaders load relative to the working directory |
| Stepping | Fixed 4 ms dt, 4 substeps, `step(1)` per tick, with the command hook before each tick. **The number of ticks per frame comes from the wall clock** (v1's `GetDeltaTime`, clamped to 0.05 s) | dt is **wall-clock** `GetDeltaTime()/10`, 10 substeps. **Starts paused**; `M` toggles play once per frame the key is held |
| Determinism | Per tick, yes, on the CPU backend (seeded streams; no clock or unseeded RNG in `scenes.cpp`). Vulkan is banded, not bit-identical | **None.** Velocities and colours are seeded from `std::random_device` |
| Window | Always, 1280x720, GL 4.3. No headless mode | Always, 1920x1080, GL 4.3 with compute shaders |
| Frames, run length | No capture; runs until Esc or close | No capture; runs until Esc or close |
| Reports | `fps \| bodies \| backend` about once a second; never the tick | `FPS \| Mem` (process PrivateUsage) every frame |

- **`spade_bench`** has no benchmark built from the viewer scenes. `bench_sim.cpp:55-63` says it can't include `scenes.cpp` because of the `SPADE_BUILD_V1` gate. It also **measures no memory**.
- **`engine/testing/replay.hpp`** already has what a trajectory needs: `state_digest` and `world_digest`.

## What dies with v1, and what doesn't

| Baseline | The 8 viewer scenes | The 3 Sandbox scenes |
|---|---|---|
| Reference frames | **Perishable**: v1 renders them | **Perishable** |
| Performance | **Perishable:** the viewer process's memory with v1 rendering. **Durable:** physics step time and memory, which are v2 | **Perishable**, "fresh baselines only" |
| Functional | **Durable:** the trajectory is v2 physics from v1-free `scenes.cpp` | Not applicable: a capability description only, since nothing is deterministic |

The perishable column needs **no code at all**. It's a harness that runs the tools as they are and records what they show. The durable column needs the viewer's v1-free setup code to be runnable without a window. It has to be done before quarantine, not necessarily first. `INT-4` says "now", so it's in this plan, after the perishable capture.

## Design

### A. The perishable capture: a harness, no code changes

`engine/tools/viewer/capture-v1-baselines.ps1` (PowerShell, ASCII). For each of the 11 scenes it:
1. **Launches** the tool as `scripts/demo.ps1` does: from `bin/`, with the same arguments and the CPU backend, with stdout captured to a file.
2. **Waits** for the tool's window.
3. **Captures frames** of that window with `PrintWindow(PW_RENDERFULLCONTENT)`, which works when the window is covered. Each capture is checked not to be blank. If it is, the harness falls back to bringing the window to the top and copying it from the screen.
   - **Viewer:** the first frame, then +1, +3 and +8 s of wall time.
   - **Sandbox:** the paused first frame, then +2, +5 and +10 s of play.
4. **Starts play (Sandbox only).** It posts `M` key-down and key-up messages to the window. That needs no focus, because GLFW reads the scancode from the message. It holds the key for about 1.5 frame times, where the frame time comes from the `FPS` lines.
5. **Checks motion (Sandbox only).** Play toggles once per frame the key is seen down, so one hold toggles it once or twice. The harness differences two captures to confirm the scene is moving. It retries up to five times, then asks the user (see below). A frame is never recorded as "playing" unless motion was seen.
6. **Samples memory:** process private bytes and working set (`Get-Process`), once a second. The Sandbox's own `FPS`/`Mem` lines are kept as the tool printed them.
7. **Closes** the window (`WM_CLOSE`), so the tool exits cleanly and stdout is flushed.

**What these frames are, stated plainly:** wall-clock moments, not ticks. The viewer can't say which tick it drew, and the Sandbox is unseeded. They show what each scene **looks like**: geometry, motion between moments, materials. That is exactly the visual axis `SL14b` defines ("never a pixel diff"). Render time is recorded but, per `SL14b`, is not comparable.

### B. The durable capture: trajectories and physics performance (8 scenes)

**1. Extract one setup path.**
- `bridge.cpp:358-424` creates the `Simulation` (4 ms, 4 substeps), spawns bodies in order, registers models, spawns vehicles and flushes once. That is v1-free code inside a v1-linked file.
- It moves verbatim into `engine/tools/viewer/setup.{hpp,cpp}` as `make_simulation(const Scene&, BackendDesc) -> Result<SceneRun>`, where `SceneRun` holds the `Simulation` and the vehicle refs. `bridge.cpp` calls it; the window path is otherwise unchanged.
- This is the first step of the harvest `03-v1-retirement.md` already requires, since the successors need exactly this function.

**2. Add a headless mode:** `spade_viewer <scene> cpu --trajectory <file> [--ticks N]`, default N = 3000 (12 s; flight crosses its gate at about 7.9 s). No window opens and v1 is not initialised. Per tick it calls the hook and then `step(1)`, as the window loop does. It writes:
- **a header:** scene, backend, dt, substeps, N and seeds. The harness adds the provenance lines: capture time, commit and reason (`TD-1`);
- **checkpoints:** every 25 ticks, the tick, the running chain digest (every tick's `state_digest` folded up to that tick), `state_digest`, and each world's `world_digest`. The running chain catches a divergence at any tick and makes every prefix checkable;
- **observables:** a few per scene, so a reader can check the trajectory looks right. For example, each world's body-centroid height every 250 ticks, the vehicle positions, and the gate-crossing tick for flight. The characterisations recorded in `scenes.cpp` (wind peaks near 3.4 s; flight's gate at about 7.86 s) are checked against these;
- **performance, on stdout, not in the file:** per-`step(1)` time (median and p90 over ticks 100..N), plus private bytes and working set after setup and at the end. The harness collects them into the archive's `performance.json`, so the trajectory file stays byte-stable. The clock is read around `step()` in the tool, never inside it (`L1`). Machine, GPU, driver and compiler are recorded as `tests/bench/baselines.json`'s `_meta` does, and the numbers are reference only, like that file.

*Deviation from `SL14b`'s text ("via `spade_bench`"):* `spade_bench` can't build `scenes.cpp` without reaching into a v1-gated tool, and it measures no memory. The viewer's headless mode uses the same constructors, the same setup and the same dt.

### C. Storage (Test/Docs's ruling, 2026-10-02)

The two kinds of data go to two homes, because in this repo "golden" means CPU-sourced and asserted (`TD-1`).

**`tests/golden/viewer/`: the trajectory goldens.**
- `<scene>.trajectory.txt`, one per viewer scene. Each file's header is its provenance record.
- `README.md`: what they are and how `TD-1` governs a regeneration. Understand why a file moved first; never shorten, skip or drop checkpoints; no per-platform copies; final once the Docker gcc leg reproduces it (`TD-12`).
- `scenes.cpp` and `setup.cpp` build as the `spade_viewer_scenes` library in every configuration, with `spade_fp_strict` and `spade_warnings`. The viewer links it, and the guard will. On MSVC `spade_fp_strict` adds nothing, so this box's trajectories cannot move.

**`tests/v1-baselines/`: the archive, recorded once, never asserted, never regenerated.** If something in it is wrong after quarantine, annotate its README instead of re-capturing.
- `README.md`: characterisation of all eleven scenes, provenance (commit, machine, toolchain, GPU and driver, date), how each item was captured, and the asymmetries `SL14b` names.
- `viewer/<scene>/t<seconds>.png` and `sandbox/<scene>/<moment>.png`, each with `capture.json` and `stdout.txt`.
- `performance.json`: the trajectory runs' step time and memory.

PNGs are written by the harness through .NET's `System.Drawing`, so there's no new dependency. That's 44 frames, an estimated 3–10 MB at native resolution (decision 2).

## Decisions

**Taken by the lead on 2026-10-02: yes to all three.**
- Decision 1 has two conditions: the extraction is its own commit, shown to be a pure move with `git diff --color-moved=zebra`; and `03-v1-retirement.md` says `setup.cpp` is harvested, not quarantined.
- Decision 3 is capped at 5 s or less per scene case on debug, and 40 s or less across the 8 cases.
  - Each case checks the committed checkpoints up to its own N, chosen from a debug measurement.
  - With `SPADE_FULL_VIEWER_TRAJECTORIES=1`, the same cases run all 3000 ticks. The Docker leg sets it.
  - No new label; one ctest case per scene.

1. **The viewer edit for the durable half** (part B).
   - Recommended: yes. `engine/tools/viewer/` is Interface's v2 tool code, the extraction is a pure move, and the new mode is opt-in.
   - `SL14a`'s "zero edits" binds the move commit itself.
   - If the answer is no, part A still stands alone. The trajectories would then be captured in the successor work, before quarantine.
2. **Frames in git at native resolution (about 3–10 MB once) or half resolution (about 1–3 MB).** Recommended: native. This is the last time they can be taken.
3. **A guard test.**
   - A `spade_tests` case would rebuild the eight trajectories from v1-free `scenes.cpp` and `setup.cpp` and assert the committed digests. An engine change that moves a viewer scene then fails loudly, the way a moved golden does.
   - Recommended: yes, under Test/Docs's governance, with `TD-12`'s gcc leg as cross-check. The cost (about 10–30 s at 1000 ticks) needs measuring.
   - It can follow this plan; it isn't needed for "done".

## Where the user is needed

**Probably only to leave the desktop unlocked for about 10 minutes during Task 3, at a time the user picks.** Eleven windows open and close by themselves. The harness posts its own keys, and `PrintWindow` captures them even if covered.

The user is needed at the screen **only as a fallback**, if Task 1's dry run shows the automated `M` or the capture doesn't work on this machine:
- if the harness prints `PRESS M ONCE NOW`, click the Sandbox window and tap `M` once;
- if `PrintWindow` returns blank frames, leave the windows uncovered while they run.

Task 1's dry run settles which case applies, and I'll tell the lead which before the sitting is booked. Judging whether each scene "looks right" is done afterwards from the committed frames, by the lead and the user. It isn't done live.

## Review focus

1. **Part A changes nothing in the tree except new files:** no edit to v1, the viewer, `demo.ps1` or any build file.
2. **A missed or doubled `M`** must be caught by the motion check, never recorded as a "playing" frame. A blank `PrintWindow` frame must be caught by the blank check.
3. **The extraction must be a pure move.** The diff of `bridge.cpp` and `setup.cpp` should show the same statements in the same order; a reordered spawn or flush changes every trajectory.
4. **The trajectory file must change when the physics does.** SL18's mutation control: a changed seed or restitution in a scratch build must change the digests from that checkpoint on, and the chain digest.
5. **Stale output:** each run writes into a fresh directory, and the harness refuses to overwrite a committed baseline without `-Force`.

## Tasks

Branch `interface/v1-baselines` in `../spade-wt/interface`. The baselines and their README are committed on that branch, since they come from its tools.

- [x] **1. The harness** (part A). It needs no build of its own. The dry run uses any built `bin/`. The real capture (Task 3) runs the worktree's `build-ninja/release` binaries, rebuilt at the branch commit in Task 2's slot, so the baselines name the commit that produced them. Proof: a dry run of `drop` and `Sandbox spheres`, including the motion check, the blank check and the `-Force` refusal. *Short slot: one viewer window and one Sandbox window, about 1 min each, no input.*
- [x] **2. Extract setup, and the headless mode** (part B; only if decision 1 is yes). Proof:
  - it builds under /W4 /WX;
  - the extraction diff is a pure move;
  - two `--trajectory` runs are byte-identical apart from the timing and memory lines;
  - the mutation control goes red;
  - the observables match the recorded characterisations.
  
  *Short slot: an incremental `spade_viewer` build of about 5 min, then about 2 min of headless runs.*
- [x] **3. The capture run:** all 11 scenes with part A, and the 8 headless runs with part B. Then commit `tests/golden/viewer/` and `tests/v1-baselines/`. *One slot, about 15 min, with the desktop unlocked. It starts with the incremental build of the storage changes; no build runs during the capture itself.*
- [x] **4. Docs.**
  - `03-v1-retirement.md`: the order step "capture baselines" done, with a link, and the deviation from "via `spade_bench`".
  - `07-status.md`: the `SL14b` row.
  - `00-decisions.md`: `INT-4` "done".
- [ ] **5. The guard test,** if decision 3 is yes. It can follow "done".
