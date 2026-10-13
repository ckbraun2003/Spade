# The live smoke tour, restructured -- storyboard

**Owner:** Interface. **Status:** draft for the lead's review, 2026-10-13. Rendering consulted (the airflow chapter); Test/Docs to sign off the coverage table (§4) before any step is cut.
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

**The capability check, not a silent drop.** The chapter opens a LiveRun of the drone-box case and steps it once:
- **When LiveRun refuses:** today it refuses because the rotors aren't coupled, before Core's command lands. The chapter shows a card, "Airflow not yet available: <LiveRun's reason, verbatim>", and its one step checks that the reason was shown (`airflow.capability/announced`). It's a real check, so a refusal can't pass silently.
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

**One look.** Rendering's `render/flow_look.hpp` (in review, `rendering/flow-look`) gives the tour flow_movie's palette and helpers, so the tour and the films are one system. The steps are gated on what exists: `airflow.surface` needs P2's solids, and until then the step announces "C_p needs the solid drone (P2)" and checks that announcement.

### Chapter 3: The editor (about 25 s)

*Caption: "The editor: open a scene, place, select, move, edit, undo, run, save."*

All 11 of today's editor steps stay, each a declared step (EDT-011), with its 8 stills (editor_picker … editor_conflict, the editor plan's look-and-feel acceptance). The builder's operations (place, select and drag, duplicate, delete) are the editor's own on real scenes, so the builder chapter goes. Its checks move to `test_sandbox_builder.cpp` (§4). Whether the builder MODE itself is retired from the sandbox is the lead's call; this storyboard only takes it out of the tour.

**Closing card** (4 s): the verdict and the run's numbers.

## 3. What stays the same

- **Kat's journal mechanisms:** the declared-step ledger, anomalies, known-open entries (none today) and the END line.
- **The run-level checks:** frames-recorded, ui-font, mp4-written, mp4-sendable (under 30 MB; about 85 s at today's CRF should come to about 7 MB), and the posters, now `01_drone_box`, `02_airflow` and `03_editor`.
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

Two decisions belong to the lead:
- the analytic heatmap's toggle check, retired with the heatmap if it goes;
- the builder mode.

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
| `builder.camera/orbits` | moved | test_sandbox_camera_and_channels.cpp:263 |
| `builder.camera/pans` | moved | test_sandbox_camera_and_channels.cpp:284 |
| `builder.delete/removes-one` | moved | test_sandbox_builder.cpp:324, :339 |
| `builder.duplicate/adds-one` | moved | test_sandbox_builder.cpp:351 |
| `builder.open/empty` | dropped | a precondition of the builder's own steps, which leave the tour |
| `builder.open/gpu-path` | dropped | duplicates drone_box.open/gpu-path, kept (the same GL renderer) |
| `builder.open/view-between-panels` | dropped | duplicates editor.open/view-between-panels, kept (the editor replaces the builder in the tour) |
| `builder.place/on-ground` | moved | test_sandbox_builder.cpp:222 |
| `builder.place/rendered` | moved | test_sandbox_builder.cpp:489 |
| `builder.place/three-placed` | moved | test_sandbox_builder.cpp:229 |
| `builder.recolour/has-object` | dropped | a precondition of reaches-material |
| `builder.recolour/reaches-material` | new test | test_sandbox_builder.cpp: recolouring object 0 sets materials[material_base].base_color to the chosen colour (red: recolour writes the wrong slot) |
| `builder.select_drag/height-kept` | moved | test_sandbox_builder.cpp:278 |
| `builder.select_drag/moves-in-x` | moved | test_sandbox_builder.cpp:278 |
| `builder.select_drag/selects` | moved | test_sandbox_builder.cpp:247 |
| `drone_box.attitude/pitch-follows` | kept | chapter 1, orientations |
| `drone_box.attitude/roll-follows` | kept | chapter 1, orientations |
| `drone_box.attitude/yaw-follows` | kept | chapter 1, orientations |
| `drone_box.heatmap/downwash-shows` | moved | test_sandbox_drone_view.cpp:207 (observed > 1 m/s) and :327 (plume > 5 m/s from every azimuth) on the same field; the session-to-panel wiring goes with view-on |
| `drone_box.heatmap/view-off` | new test | as view-on |
| `drone_box.heatmap/view-on` | new test | test_sandbox_drone_view.cpp: the session's V toggle sets and clears panel.view_heatmap, if reachable without a frame; else retired with the analytic heatmap (feature gone, replaced by the airflow chapter): lead's call |
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
| `drone_box.turbulence/applied` | moved | test_sandbox_drone_view.cpp:351 (the rebuild path) and test_sandbox_drone.cpp:141 (severe turbulence) |
| `drone_box.turbulence/calm-again` | new test | test_sandbox_drone_view.cpp: resetting every option returns DronePhysicsOptions{} (red: a reset that keeps the turbulence) |
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

Totals: dropped 26, kept 53, moved 25, new test 24; 128 check ids.

## 5. Order, after the lead's OK and Test/Docs' sign-off

1. The new headless tests, red then green, before any step is cut, so coverage never dips.
2. The tour: chapter 1 trimmed; the scene and builder mains removed (posters and kMains with them); chapter 2's capability card; the stale descriptions fixed. One live smoke run, its journal attached.
3. Chapter 2's live views as LiveRun steps rotors (Core's command) and flow_look lands; C_p after P2.
