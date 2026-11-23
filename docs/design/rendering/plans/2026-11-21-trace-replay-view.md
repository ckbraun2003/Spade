# Trace-replay view (O150, the PX4 flight Demo) -- design note, for Core's and Interface's agreement

Rendering, 2026-11-21. A note before code: nothing is built. Formats agreed with Core (trace.bin v1) and Interface (estimate.csv), 2026-11-21.

## What it is

`tools/trace_replay/` (new, outside `engine/`, L7; no engine call is added). One tool: a recorded Spade run in, a film out. It draws the X500 from the run's truth at real speed. Beside the 3D view is a panel of Spade's true altitude against PX4's estimate over time. The arm state and the flight phase show as labels. It runs the engine's GL or CPU renderer already used by `tools/flow_movie` and reads only files. It never steps the simulation, so it cannot change a trace.

## What it reads

1. **The run folder** that `LockstepHost::close()` writes (fc-core section 4): `run.yaml` (dt_ns, substeps, seeds, hashes) and `trace.bin`. From `trace.bin` per tick it needs only the tick, the truth position, the orientation and the body rate. It also needs the applied rotor command for the rotor spin, but the first cut draws no blur. The sensor samples are skipped. The panel needs the true altitude only, and that is the truth z.
2. **PX4's side**, as one small file: `estimate.csv` (`t_s, z_est_m, armed, nav_state`). **Agreed with Interface (2026-11-21):** `z_est_m` = -`vehicle_local_position.z`, up-positive, relative to PX4's EKF origin (where it started), not sea level; `armed` = `vehicle_status.arming_state == 2` (1/0); `nav_state` is PX4's integer; one row per `vehicle_local_position` sample (about 50 Hz); `t_s` = (ulog timestamp - first `time_usec`)/1e6 on the lockstep clock (a link step = 4000 us = 8 ticks), never wall time; no reset inside a run (a Spade reset ends the PX4 run and a new run has its own file from t_s = 0). Written post-run by a pyulog step in the image, network off. It is written by Interface's side from the `.ulg`. The tool does NOT parse ULog; a parser belongs with the owner of the link (Interface), and the CSV keeps this tool's input one flat, testable format. If Interface would rather give the link's own telemetry (`HIL_STATE_QUATERNION` is truth, not an estimate), that is a second source of the same CSV.
3. **The drone**: the existing X500 build-parts table (`tools/flow_movie/data/x500_build`), through `add_build_parts`, as the flow films draw it. Kept untouched.

## Time and the trace's header (agreed with Core)

- Real speed: the film's frame k (30 fps) is the trace tick nearest `k / 30 s` of sim time, and no tick is interpolated. At 250 Hz that is 8 or 9 ticks a frame. This holds the pose from the nearest tick rather than a mix of two ticks. A 5 m climb and landing is slow enough for it to look smooth.
- **`trace.bin` v1, as Core proposed (2026-11-21; final when Core's `tools/fc_host` RED tests pin it; Core tells me of any change before it lands).** Little-endian, f32, no padding. A 192-byte header names every offset (magic "SPTR", version 1, header_size, dt_ns, substeps, rotor_count, tick_count, first_tick, tick_record_size, tick_section_offset, sample_section_offset, sample_count, the trace_hash text), so I read by the header, not by a fixed layout. A tick record (76 + 4 x rotor_count bytes): u64 tick, u64 time_ns (the state at the END of the tick), f32[3] pos, f32[3] vel, f32[4] orientation (w, x, y, z; body to world), f32[3] omega_body, f32[rotor_count] rotor_cmd, then the sample indices. Record i is at tick_section_offset + i x tick_record_size, so frame selection is O(1). The sample section is skipped. The trailer is the last 64 bytes: the lower-case SHA-256 of every earlier byte; the reader recomputes it (`spade::sha256_hex`) and refuses a mismatch, or a file whose size is not sample_section_offset + the sample bytes + 64. The run folder is written once at `close()`, so the tool reads finished runs only (no live view; if the Demo wants one, Core adds a streaming writer).
- **World frame: Spade's own, +y up, right-handed** (not z up). The truth height is `pos.y` (plus `world_origin_altitude_m` from `run.yaml` where a barometer is bound, which the panel does not need: its datum is the take-off point). The drawing converts nothing: the scene is Spade's own frame, as the engine's renderer already draws it.
- The run's tick zero and PX4's time base agree by the lockstep clock above: Spade's tick k is at k x dt, PX4's rows at t_s on the same clock. Closed.

## What the film shows

- 3D view: the X500, ground plane, a slow orbit camera that follows the drone's position (fixed height 8 degrees above its centre, no motion of the world), 1920x1080 for the site, 30 fps. The cap on a posted video is 1920x1080, so no 4K here.
- Panel (right third): "height above take-off point (m)" against time, two lines. PX4's estimate is relative to its own EKF origin, so Spade's truth is plotted on the same datum, relative to the X500's resting height at t = 0 (a constant offset would otherwise read as an estimation error); the film's caption says so. Spade's truth is solid and PX4's estimate is dashed, each with a small legend. A moving marker sits at the current time. The axes are metres and seconds, with fixed ticks (0 to a little over the 5 m climb); there is no auto-scaling that moves between runs. Drawn as the engine's sprite shapes (lines, polygons) in an overlay, since sprites already exist.
- Labels: "ARMED" / "DISARMED" from `armed`, and the phase from the Demo's scripted flight (the lead, 2026-11-21): arm, take off, climb to 5 m, hold 60 s, land, disarm, labelled "arm", "take-off", "climb to 5 m", "hold", "land", "landed". The phase comes from `nav_state` plus the truth altitude (climb ends at 5 m, hold is 60 s there, landed at rest on the ground); the tool says what it reads and does not infer more. Title: "PX4 flies Spade's X500 (simulation)". Length under 180 s (posted as Demo/Spade/YYYY-MM-DD_HHMM_px4-flies-spade-x500.mp4, by Oversight). Text is burned in by the recorder's ffmpeg step, as the flow films' captions are (`recorder.hpp`); the time-varying labels use `drawtext` with time windows from the CSV. No font is added to the engine.
- A short caption card and the record, as every film: "Simulation"; what is real (Spade's truth from the run, PX4's own estimate from its log) and what is stand-in (the X500's measured parameters; the sensors and the mapping are models).

## Test plan, RED first

0. (done in the note) Datum: the truth's height is taken relative to its first tick's.
1. Reader: the header's refusals (bad magic, version, a short file, a file that fails its hash trailer), each named.
2. Frame selection: tick for frame k at 250 Hz, 1000 Hz and a run shorter than the film.
3. `estimate.csv` reader: an unknown column, a non-monotonic time, an NED-signed file are all refused by name.
4. Panel geometry: a known two-line series maps to known pixel endpoints (pure function, no GL).
5. Phase labels from a scripted CSV with arm, climb, hold and land.
6. A CPU frame of a one-tick trace: the drone's bounding box lands where the camera maps it (same check as the camera tests, no GL needed).
7. Replay-equals-live (not mine): a film from a replay run and from the live run of the same trace file is byte-identical, since the tool reads files only.

## Out of scope

No rotor blur in the first cut (a blade angle needs the rotor speed and the film would be at 1/286 time scale to see blades; at real speed they would alias). No wind, no flow, no sensor traces. No new engine API. The gap between the PX4 estimate's reporting rate and the trace's rate is not smoothed: the dashed line is drawn from PX4's own samples as they come.

## What I need before code

- **Core**: the path of a generated sample run folder for the reader's RED test, once `fc_host`'s test builds one.
- **Interface**: the first 5 rows of a real `estimate.csv` (promised).
- Effort: about 2 days after the formats are agreed. The camera tool (video 4) is the next item; I start it while the note waits.
