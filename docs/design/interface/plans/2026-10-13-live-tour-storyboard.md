# The live smoke tour, restructured -- storyboard

**Owner:** Interface. **Status:** **approved by the lead, 2026-10-13**, pending Test/Docs' sign-off on the coverage table (§4) before any step is cut. Rendering consulted on the airflow chapter. The lead's rulings: the analytic heatmap is RETIRED, feature and toggle (it draws a synthetic field that could be read as flow data; the real speed slice replaces it). The builder leaves the tour now, and its MODE leaves the sandbox once the editor covers each builder action (§2, chapter 3: it does).
**Asked by** Cameron (relayed 2026-10-13 19:59 UTC): "clean up, restructure, and consolidate ensuring you account for the new visualization when available. A lot of the features demoed are hard to actually visualize, and are a bit dated in comparison to where we're at with the system."
**Base:** master; the inventory is of `sandbox/live_tour.cpp` as last run, `Desktop\spade-smoke\2026-10-13_1e3cc3f107a9` (PASS, 38 steps, 128 check ids, 154 s of video, 12.1 MB).

## 1. Today's tour, in brief

| Main | Steps | Video | What a viewer can see | Verdict |
|---|---|---|---|---|
| drone_box | 10 | 62 s | The drone on its stand: orbit, attitude, level, throttle, HUD; also the analytic air-speed heatmap, wind and turbulence (little visible change: the drone is pinned), and a Vulkan refusal (a status line) | Keep the flying, the camera, thrust and the HUD; cut heatmap, wind, turbulence, vulkan_refused |
| builder | 7 | 22 s | Primitive boxes placed, dragged, recoloured, duplicated, deleted | Cut: the editor does the same on real scenes |
| shower | 2 | 13 s | 1000 bodies pour into a bowl | Cut: its physics checks move headless |
| bounce | 2 | 8 s | Four drops at different restitution | Cut: headless |
| gate | 2 | 4 s | Five lobs through a ring | Cut: already headless (`test_viewer_gate.cpp:51`) |
| hover | 2 | 7 s | An open-loop quad hovering | Cut: headless |
| wind | 2 | 7 s | The same quad in turbulence | Cut: headless |
| editor | 11 | 25 s | Open, pick, select, place, move (gizmo), inspect, undo, run, world, conflict, save | Keep, all 11 (EDT-011) |

The scene mains were the strangler viewer's successors: worth keeping as physics regressions, but dated as a demonstration, and each "open" repeats the same five checks. The heatmap is an analytic stand-in for the air, which the real airflow chapter replaces. Wind and turbulence change almost nothing on screen, because the drone is pinned. Every one of their checks is accounted for in §4.

## 2. The new tour

**Three chapters, about 85 s of video** (today 154 s), each opened by a title card with its caption.

**Opening card** (3 s): "Spade: the drone box, its air, and the editor", plus the build and date.

### Chapter 1: The drone box (about 35 s)

*Caption: "The drone box: the drone held at the centre, flown by hand. Attitude, thrust, the camera."*

| Step | On screen | Checks |
|---|---|---|
| drone_box.open | The drone on its stand; the Drone panel docked | gpu-path, stepping, view-beside-panel |
| drone_box.camera | A shortened orbit: around, over, in to the near clamp, out, drag | around, over, nearer-clamps, further, drag |
| drone_box.attitude | Pitch, roll, yaw on the keys | pitch-, roll-, yaw-follows |
| drone_box.level | R: back to level | target-levels, drone-levels |
| drone_box.throttle | Throttle up and back: the rotors spin faster | rotors-speed-up |
| drone_box.hud | F1 hides, then shows the HUD | hidden, shown |

Arm/disarm and the floor are Cameron's premise. The sandbox's DroneSim has neither: they come with LiveRun's held drone (Core's throttle and arm command, the case's floor). When chapter 2 is live, the drone box moves onto LiveRun's drone, and these steps gain arm and disarm.

### Chapter 2: Airflow (about 30 s when available, a 4 s card until then)

*Caption: "The air around the drone: vortex cores, streamlines, a pressure slice, C_p on the drone. 32³ live on the CPU, in slow motion."*

**The capability check, as an EXPIRING known-open entry, not a check that passes on any refusal** (Test/Docs' condition, the lead agreeing). The chapter opens a LiveRun of the drone-box case and steps it once:
- **When LiveRun refuses:** today it refuses because the rotors aren't coupled, before Core's command lands. The chapter shows a card, "Airflow not yet available: <LiveRun's reason, verbatim>", and raises the anomaly `airflow/not-available`. `kKnownOpen` tolerates exactly that anomaly with the entry "airflow-not-available: LiveRun refuses until the rotors are coupled". When LiveRun steps rotors, the entry stops firing and the run FAILS as expired, so someone must remove the entry and make the chapter live. A later LiveRun regression that refused for another reason would show a card and fail, since only this refusal is known-open.
- **When LiveRun steps:** the chapter runs, as follows.

| Step | On screen | Checks |
|---|---|---|
| airflow.open | The held drone, a 32³ box, the run line in the bottom band | starts, publishes (`publish()` succeeds), run-line-shown |
| airflow.arm | Arm, throttle up: the rotors spin up and shed the start-up ring | armed, rotors-spin-up |
| airflow.vortices | λ2 iso, coloured by speed, with streamlines from a ring at each hub | flow-ready (`flow_ready()`, with "compiling flow shaders…" while false), drawn (`draw()` succeeded, nothing `unhonoured`) |
| airflow.slice | A pressure slice through the hub plane | drawn |
| airflow.surface | C_p on the solid drone (P2, R2.5) | drawn, cp-finite |
| airflow.turn | Pitch and roll: the wake follows the attitude | drawn |

The live policy is Rendering's advice. 64³ was measured at about 261 ms per step plus 54 ms per publish. Scaling by cell count, not measuring, gives 32³ at about 33 + 7 ms per frame, roughly 25 fps, and 48³ at about 7 fps. So the chapter runs 32³ live: one `LiveRun::step()` and one `publish()` per displayed frame, with the slow-motion factor (h over the wall time per frame) in the caption. Flow quality is Auto live and Full for stills.

GL draws ONE flow layer per frame, so the volume and iso views take turns. Streamlines, slices, tracers and one surface colouring can share a frame. 128³ live, with λ2 and C_p together, waits for the GPU path (D2).

**One look.** Rendering's `render/flow_look.hpp` (in review, `rendering/flow-look`) gives the tour flow_movie's palette and helpers, so the tour and the films are one system. The steps are gated on what exists: `airflow.surface` needs P2's solids. Until then the step shows "C_p needs the solid drone (P2)" and raises `airflow/cp-needs-p2`, tolerated by the expiring known-open entry "cp-needs-p2", which fails the run as expired once P2 lands.

### Chapter 3: The editor (about 25 s)

*Caption: "The editor: open a scene, place, select, move, edit, undo, run, save."*

All 11 of today's editor steps stay, each a declared step (EDT-011), with its 8 stills (editor_picker … editor_conflict, the editor plan's look-and-feel acceptance).

**The builder leaves the tour now, and its mode leaves the sandbox (the lead's ruling).** Every builder action already has an editor edit that the editor's panels issue:

| Builder action | The editor's own | Reached from |
|---|---|---|
| place | the library's place (`AddAsset` / `AddVehicle`) | the Library panel; the tour's `editor.place` |
| select and drag | pick and the move gizmo | the view; `editor.select`, `editor.move` |
| recolour | `SetAssetMaterial` (and `AddMaterial` for a new colour) | the inspector (`editor_panels_ui.cpp`) |
| duplicate | `DuplicateObject` | the inspector; `editor.inspect` |
| delete | `RemoveObject` | the inspector |
| camera | the editor's orbit, pan and dolly | the view |

So no action needs porting, and the mode is removed with its code. The builder's checks that stay meaningful move to the editor's tests (§4). Where a §4 row names `test_sandbox_builder.cpp`, the implementation re-homes it to the editor's own test for the same operation (`test_editor_*`), since that file goes with the mode. Each such move shows the editor test's assertion, as Test/Docs asked.

**`editor.fly`** (N17, merged `4f25016`, 2026-10-22; after `editor.add_drone`; the end-goal design's §6). The library's reference quad, placed where `editor.add_drone` places it and selected, is run, and the Pilot section names it (`piloted`). The step then flies it through the session's own frames with the pilot's keys held as `FrameInput`:
1. 1.5 s disarmed: Run's pop (`editor.add_drone/placed-at`, known-open), then rest.
2. Space, then 1.5 s at idle.
3. PageUp held until the throttle holds 1.3 m g, flown to 3 s.
4. Up held for 2.5 s.

Each frame records the piloted drone's height, velocity, design-frame attitude (+X the nose), rotor speeds and vertical thrust. Test/Docs' `armed`, `climbs` and `follows-attitude` judge those frames, measured from the run's own frames, so none depends on the spawn height (the lead's note). The headless twin is `FlightController.TheToursStagingFromThePilotsKeysPassesArmedClimbsAndFollows` (`tests/test_flight_controller.cpp`). Don't "fix" the first two back; the third is why their margins hold (the lead's confirmation, 2026-10-22):
- **The throttle step is 1.3 m g, not full throttle.** At full throttle the quad climbs at 29 m/s. At that speed, drag and the rotors' falling C_T put it outside `climbs`' drag-free premise: 0.498 against the 0.5 floor. That's physical (Physics' review), not a bug. 1.3 m g is inside the 1.1 to 1.4 m g range `climbs` was simulated for. First run: 0.696 (band [0.5, 1.1]).
- **The pitch command is -30 degrees held, judged from the press.** The keys can't hold -15 degrees. A held arrow ramps the setpoint at 60 degrees a second to the keys' 30-degree limit, and a released one levels it (`sandbox/pilot_keys.hpp`). Setting the panel to -15 would be levelled by the next frame's keys. `follows-attitude` counts from the press, the 0.5 s ramp included: within 3 degrees by 0.92 s (1.0 allowed), 4.9 m/s gained along the tilt. Don't move the judgement to the setpoint's arrival (0.41 s) to buy margin.
- **The 0.08 s margin is deterministic.** The tour steps a fixed 1/60 s per frame, never the wall clock. `run_live_smoke()` (`sandbox/live_tour.cpp`) calls `GlTargetSink::set_fixed_delta(kDt)`, and `poll()` replaces the measured frame time with it, so `delta_seconds()` is always 1/60. Run's steps take that dt (whole 4 ms steps, the remainder carried), and so do the pilot's keys. The wall clock feeds only the overlay's ms readouts. A loaded box draws slower but steps the same. The twin, sampled every step, reaches at 0.91 s; the tour, sampled every 60 Hz frame, at 0.92 s.

**Closing card** (4 s): the verdict and the run's numbers.

## 3. What stays the same

- **Kat's journal mechanisms:** the declared-step ledger, anomalies, known-open entries (two, expiring, in chapter 2) and the END line.
- **Every harness and recording anomaly stays** (they fire only on failure, so they aren't in §4): recording/{ffmpeg-missing, ffmpeg-start, poster, size-changed, ffmpeg-write, still, ffmpeg-exit}, harness/{window-closed, frame-failed, poster-undeclared, snapshot-undeclared, drone-session, no-window}, and inj.target. Only harness/builder-session goes, with the builder mode.
- **One added check:** `editor.open/gpu-path` (Test/Docs' suggestion), so the editor's own session checks its renderer, now that the builder's and the scenes' gpu-path checks drop as duplicates.
- **The run-level checks:** frames-recorded, ui-font, mp4-written, mp4-sendable (under 30 MB; about 85 s at today's CRF should come to about 7 MB) -- the working copy's size check; since 2026-10-21 a video is posted only under `Desktop/Projects/Videos` by `Videos/CATEGORIES.md`, through the lead (`interface/07-status.md`), and the posters, now `01_drone_box`, `02_airflow` and `03_editor`.
- **The editor's 8 stills.** Snapshots of removed mains don't exist today.
- **Stale descriptions,** fixed in the same branch: `07-status.md:25`, `live_tour.hpp:9`, `main.cpp:86`, and `live-smoke.ps1`'s synopsis.

## 4. Coverage: every check id, under the new tour

One row per check id from the last journal: kept, moved to an existing headless test, a new headless test (shown red, then green), or dropped with its reason. Test/Docs' rules apply:
- a moved check names its test and assertion, on the same object, with the same or a stronger predicate;
- a new test is red, then green;
- "hard to see" alone is never a reason to drop.

The new headless tests (24 check ids) go in three places:
- `test_sandbox_drone_view.cpp`, `test_sandbox_builder.cpp` and `test_sandbox_camera_and_channels.cpp`: small additions;
- `test_sandbox_scene_sessions.cpp` (new): each scene file composes as a session without a window, with its vehicle count;
- `test_viewer_scene_properties.cpp` (new): shower, bounce, hover and wind run headless, asserting the same predicates the tour asserted.

The lead decided both open points: the heatmap's three checks are dropped because the feature is retired, and the builder rows re-home to the editor's tests when the mode goes.

| Check id | Under the new tour | Where, or why |
|---|---|---|
| `bounce.drops/all-landed` | new test | test_viewer_scene_properties.cpp: bounce, every lane landed |
| `bounce.drops/rebound-follows-restitution` | new test | same test: rebound heights strictly increasing with e (the helpers are already tested on synthetic data, test_sandbox_scene_checks.cpp:24-46) |
| `bounce.drops/session` | dropped | a precondition of the scene's second step, which leaves the tour |
| `bounce.open/assets` | dropped | a precondition (--assets given); the editor opens the same assets |
| `bounce.open/bodies` | new test | the same new test: vehicle_count per lane == N; test_viewer_gate.cpp:54 already asserts 5 for gate |
| `bounce.open/composed` | new test | test_sandbox_scene_sessions.cpp (new): SceneFileSession::create succeeds for each scene file without a window (red: a missing record) |
| `bounce.open/gpu-path` | dropped | duplicates drone_box.open/gpu-path, kept (the same GL renderer) |
| `bounce.open/records` | moved | test_sandbox_builtin_records.cpp:78, :44 |
| `builder.camera/dollies` | new test | test_sandbox_camera_and_channels.cpp: a scroll dolly brings the camera nearer (red: dolly away); :227 tests only the bounds |
| `builder.camera/orbits` | moved | test_sandbox_camera_and_channels.cpp:263 (OrbitCamera stays) |
| `builder.camera/pans` | moved | test_sandbox_camera_and_channels.cpp:284, the Q/E target-moves assertion (the tour's move_up) |
| `builder.delete/removes-one` | new test | the builder mode leaves the sandbox; counterpart: test_editor_scene_edits.cpp:106 RemovingAVehicleRemovesItsOwnModelButNotASharedOne (the removal at :109), plus a new assertion there that the scene holds one vehicle fewer (red: a removal that removes nothing) |
| `builder.duplicate/adds-one` | dropped | the builder mode leaves the sandbox; counterpart: editor.inspect/duplicated (kept) |
| `builder.open/empty` | dropped | a precondition of the builder's own steps, which leave the tour |
| `builder.open/gpu-path` | dropped | duplicates drone_box.open/gpu-path, kept (the same GL renderer) |
| `builder.open/view-between-panels` | dropped | duplicates editor.open/view-between-panels, kept (the editor replaces the builder in the tour) |
| `builder.place/on-ground` | dropped | the builder mode leaves the sandbox; counterpart: none found: the editor's library place has no on-the-ground assertion (Test/Docs to judge) |
| `builder.place/rendered` | dropped | the builder mode leaves the sandbox; counterpart: none found: no editor test asserts one drawable per object (Test/Docs to judge) |
| `builder.place/three-placed` | dropped | the builder mode leaves the sandbox; counterpart: editor.place/unique-names (kept): two library boxes placed, with distinct names |
| `builder.recolour/has-object` | dropped | a precondition of reaches-material; the builder mode leaves the sandbox |
| `builder.recolour/reaches-material` | new test | the builder mode leaves the sandbox; counterpart: a new test in test_editor_scene_edits.cpp: SetAssetMaterial puts the named material on the asset (red: it sets another asset's); today only the refusal (:163) is tested |
| `builder.select_drag/height-kept` | dropped | the builder mode leaves the sandbox; counterpart: editor.move/drags-along-x (kept): it also asserts dy = dz = 0 |
| `builder.select_drag/moves-in-x` | dropped | the builder mode leaves the sandbox; counterpart: editor.move/drags-along-x (kept): dx = 1 +/- 0.05 |
| `builder.select_drag/selects` | dropped | the builder mode leaves the sandbox; counterpart: editor.select/selects-vehicle (kept) |
| `drone_box.attitude/pitch-follows` | kept | chapter 1, orientations |
| `drone_box.attitude/roll-follows` | kept | chapter 1, orientations |
| `drone_box.attitude/yaw-follows` | kept | chapter 1, orientations |
| `drone_box.heatmap/downwash-shows` | dropped | as view-on: the feature is retired; its field tests (test_sandbox_drone_view.cpp:207, :327) go with it |
| `drone_box.heatmap/view-off` | dropped | as view-on: the feature is retired |
| `drone_box.heatmap/view-on` | dropped | the feature is retired (the lead, 2026-10-13): the analytic heatmap draws a synthetic field that could be read as flow data; the real speed slice (chapter 2) replaces it |
| `drone_box.hud/hidden` | kept | chapter 1 (GL sink; only the tour can see it) |
| `drone_box.hud/shown` | kept | chapter 1 (GL sink; only the tour can see it) |
| `drone_box.level/drone-levels` | kept | kept; plus a headless test (test_sandbox_drone.cpp: levelling back from a tilt) so it no longer depends on the attitude step |
| `drone_box.level/target-levels` | kept | chapter 1, orientations |
| `drone_box.open/gpu-path` | kept | chapter 1, opening |
| `drone_box.open/stepping` | kept | chapter 1, opening |
| `drone_box.open/view-beside-panel` | kept | chapter 1, opening |
| `drone_box.orbit/around` | kept | chapter 1, the camera (shortened) |
| `drone_box.orbit/back-in` | dropped | staging for the next step, not a property; the chapter's camera step keeps around/over/clamps/drag |
| `drone_box.orbit/drag` | kept | chapter 1, the camera (shortened) |
| `drone_box.orbit/further` | kept | chapter 1, the camera (shortened) |
| `drone_box.orbit/nearer-clamps` | kept | chapter 1, the camera (shortened) |
| `drone_box.orbit/over` | kept | chapter 1, the camera (shortened) |
| `drone_box.throttle/rotors-speed-up` | kept | chapter 1, thrust states (tour-only today) |
| `drone_box.turbulence/applied` | new test | in the new calm-again test: set moderate, apply, options().turbulence == moderate (red: an apply that drops the turbulence); neither :351 nor drone:141 asserts it |
| `drone_box.turbulence/calm-again` | new test | test_sandbox_drone_view.cpp (new): set moderate and check it applied (the row above), then reset every option: DronePhysicsOptions{} (red: a reset that keeps the turbulence) |
| `drone_box.vulkan_refused/control-put-back` | moved | test_sandbox_drone_view.cpp:372; test_sandbox_drone.cpp:180, :191 |
| `drone_box.vulkan_refused/kept-cpu` | moved | test_sandbox_drone_view.cpp:372 SelectingVulkanIsRefusedVisiblyAndTheStandStaysOnCpu |
| `drone_box.vulkan_refused/reason-shown` | moved | test_sandbox_drone_view.cpp:372 |
| `drone_box.wind/applied` | moved | test_sandbox_drone_view.cpp:351 |
| `drone_box.wind/attitude-carried` | moved | test_sandbox_drone.cpp:209 ARebuildCarriesTheAttitudeAcross (1e-6 rad, stronger) |
| `drone_box.wind/rebuilds` | moved | test_sandbox_drone_view.cpp:351 ADraggedSliderDoesNotRebuildUntilReleased |
| `editor.conflict/open` | kept | chapter 3, the editor (EDT-011: each operation a declared step) |
| `editor.conflict/shown` | kept | chapter 3, the editor (EDT-011: each operation a declared step) |
| `editor.conflict/undo-resolves` | kept | chapter 3, the editor (EDT-011: each operation a declared step) |
| `editor.inspect/duplicated` | kept | chapter 3, the editor (EDT-011: each operation a declared step) |
| `editor.inspect/edited` | kept | chapter 3, the editor (EDT-011: each operation a declared step) |
| `editor.inspect/open` | kept | chapter 3, the editor (EDT-011: each operation a declared step) |
| `editor.inspect/other-unchanged` | kept | chapter 3, the editor (EDT-011: each operation a declared step) |
| `editor.move/drags-along-x` | kept | chapter 3, the editor (EDT-011: each operation a declared step) |
| `editor.move/handle-in-view` | kept | chapter 3, the editor (EDT-011: each operation a declared step) |
| `editor.move/one-undo` | kept | chapter 3, the editor (EDT-011: each operation a declared step) |
| `editor.move/open` | kept | chapter 3, the editor (EDT-011: each operation a declared step) |
| `editor.open/composes` | kept | chapter 3, the editor (EDT-011: each operation a declared step) |
| `editor.open/opens` | kept | chapter 3, the editor (EDT-011: each operation a declared step) |
| `editor.open/view-between-panels` | kept | chapter 3, the editor (EDT-011: each operation a declared step) |
| `editor.open/work-copy` | kept | chapter 3, the editor (EDT-011: each operation a declared step) |
| `editor.picker/lists-scenes` | kept | chapter 3, the editor (EDT-011: each operation a declared step) |
| `editor.picker/open` | kept | chapter 3, the editor (EDT-011: each operation a declared step) |
| `editor.place/open` | kept | chapter 3, the editor (EDT-011: each operation a declared step) |
| `editor.place/unique-names` | kept | chapter 3, the editor (EDT-011: each operation a declared step) |
| `editor.run/carries-pose` | kept | chapter 3, the editor (EDT-011: each operation a declared step) |
| `editor.run/open` | kept | chapter 3, the editor (EDT-011: each operation a declared step) |
| `editor.run/runs` | kept | chapter 3, the editor (EDT-011: each operation a declared step) |
| `editor.save/byte-equal` | kept | chapter 3, the editor (EDT-011: each operation a declared step) |
| `editor.save/open` | kept | chapter 3, the editor (EDT-011: each operation a declared step) |
| `editor.select/frames-selection` | kept | chapter 3, the editor (EDT-011: each operation a declared step) |
| `editor.select/open` | kept | chapter 3, the editor (EDT-011: each operation a declared step) |
| `editor.select/selects-vehicle` | kept | chapter 3, the editor (EDT-011: each operation a declared step) |
| `editor.undo/bytes-equal` | kept | chapter 3, the editor (EDT-011: each operation a declared step) |
| `editor.undo/open` | kept | chapter 3, the editor (EDT-011: each operation a declared step) |
| `editor.world/open` | kept | chapter 3, the editor (EDT-011: each operation a declared step) |
| `editor.world/re-pinned` | kept | chapter 3, the editor (EDT-011: each operation a declared step) |
| `gate.lobs/reach-the-gate` | moved | test_viewer_gate.cpp:51 EveryLobPassesThroughTheRing |
| `gate.lobs/session` | dropped | a precondition of the scene's second step, which leaves the tour |
| `gate.lobs/through-the-ring` | moved | test_viewer_gate.cpp:51 |
| `gate.open/assets` | dropped | a precondition (--assets given); the editor opens the same assets |
| `gate.open/bodies` | new test | the same new test: vehicle_count per lane == N; test_viewer_gate.cpp:54 already asserts 5 for gate |
| `gate.open/composed` | new test | test_sandbox_scene_sessions.cpp (new): SceneFileSession::create succeeds for each scene file without a window (red: a missing record) |
| `gate.open/gpu-path` | dropped | duplicates drone_box.open/gpu-path, kept (the same GL renderer) |
| `gate.open/records` | moved | test_sandbox_builtin_records.cpp:78, :44 |
| `harness/frames-recorded` | kept | run level |
| `harness/mp4-sendable` | kept | run level |
| `harness/mp4-written` | kept | run level |
| `harness/poster-bounce` | dropped | the bounce main leaves the tour, so it has no poster; its properties move headless (rows above) |
| `harness/poster-builder` | dropped | the builder main leaves the tour; its operations live in the editor chapter |
| `harness/poster-drone_box` | kept | kept with its main |
| `harness/poster-editor` | kept | kept with its main |
| `harness/poster-gate` | dropped | the gate main leaves the tour, so it has no poster; its properties move headless (rows above) |
| `harness/poster-hover` | dropped | the hover main leaves the tour, so it has no poster; its properties move headless (rows above) |
| `harness/poster-shower` | dropped | the shower main leaves the tour, so it has no poster; its properties move headless (rows above) |
| `harness/poster-wind` | dropped | the wind main leaves the tour, so it has no poster; its properties move headless (rows above) |
| `harness/ui-font` | kept | run level |
| `hover.holds/holds-height` | new test | test_viewer_scene_properties.cpp: hover, max |dy| < 0.25 m |
| `hover.holds/level` | new test | same test: tilt < 0.05 rad |
| `hover.holds/session` | dropped | a precondition of the scene's second step, which leaves the tour |
| `hover.open/assets` | dropped | a precondition (--assets given); the editor opens the same assets |
| `hover.open/bodies` | new test | the same new test: vehicle_count per lane == N; test_viewer_gate.cpp:54 already asserts 5 for gate |
| `hover.open/composed` | new test | test_sandbox_scene_sessions.cpp (new): SceneFileSession::create succeeds for each scene file without a window (red: a missing record) |
| `hover.open/gpu-path` | dropped | duplicates drone_box.open/gpu-path, kept (the same GL renderer) |
| `hover.open/records` | moved | test_sandbox_builtin_records.cpp:78, :44 |
| `shower.open/assets` | dropped | a precondition (--assets given); the editor opens the same assets |
| `shower.open/bodies` | new test | the same new test: vehicle_count per lane == N; test_viewer_gate.cpp:54 already asserts 5 for gate |
| `shower.open/composed` | new test | test_sandbox_scene_sessions.cpp (new): SceneFileSession::create succeeds for each scene file without a window (red: a missing record) |
| `shower.open/gpu-path` | dropped | duplicates drone_box.open/gpu-path, kept (the same GL renderer) |
| `shower.open/records` | moved | test_sandbox_builtin_records.cpp:78, :44 |
| `shower.pour/finite` | new test | test_viewer_scene_properties.cpp (new): shower run headless N ticks; every body finite |
| `shower.pour/held-by-the-bowl` | new test | same test: none with |x| or |z| > 4 or y < 0 |
| `shower.pour/poured` | new test | same test: mean height falls > 0.5 m |
| `shower.pour/session` | dropped | a precondition of the scene's second step, which leaves the tour |
| `wind.gusts/carried-by-the-air` | new test | same test: max |dy| > 0.5 m |
| `wind.gusts/finite` | new test | test_viewer_scene_properties.cpp: wind, every body finite |
| `wind.gusts/session` | dropped | a precondition of the scene's second step, which leaves the tour |
| `wind.open/assets` | dropped | a precondition (--assets given); the editor opens the same assets |
| `wind.open/bodies` | new test | the same new test: vehicle_count per lane == N; test_viewer_gate.cpp:54 already asserts 5 for gate |
| `wind.open/composed` | new test | test_sandbox_scene_sessions.cpp (new): SceneFileSession::create succeeds for each scene file without a window (red: a missing record) |
| `wind.open/gpu-path` | dropped | duplicates drone_box.open/gpu-path, kept (the same GL renderer) |
| `wind.open/records` | moved | test_sandbox_builtin_records.cpp:78, :44 |

Totals: dropped 36, kept 53, moved 15, new test 24; 128 check ids.

## 5. Order, after the lead's OK and Test/Docs' sign-off

1. The new headless tests, red then green, before any step is cut, so coverage never dips.
2. The tour: chapter 1 trimmed; the scene and builder mains removed (posters and kMains with them); chapter 2's capability card; the stale descriptions fixed. One live smoke run, its journal attached.
3. Chapter 2's live views as LiveRun steps rotors (Core's command) and flow_look lands; C_p after P2.
