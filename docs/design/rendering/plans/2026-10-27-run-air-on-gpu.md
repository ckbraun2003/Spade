# Run air on the GPU: LiveRun's fluid step on GpuFluid

**Owner:** Rendering (GpuFluid), with Physics (LiveRun) and Interface (the editor's AirflowRun). **Status:** design note for the lead, 2026-10-27; no code. **Base:** master `0bfc6d6`.

The editor's Run air flies its drone's air on the CPU at about 1/43 of real time on its 32³ grid. B7's GpuFluid (`spike/airflow/step_chain.hpp`, merged at `e34d2be`) runs the same step on the device, bit for bit on the device of record. This note says how the editor could use it, what it would gain, what blocks it, and the smallest first slice.

## 1. The seam

**Who owns what.** LiveRun (`engine/analysis/live_run.*`) is Physics'. The editor's `AirflowRun` (`sandbox/editor_airflow.hpp`) is Interface's. GpuFluid and its kernels are Rendering's. Today `AirflowRun` owns one LiveRun on a worker thread and calls `step()` one step at a time, and `advance_frame()` at each 1/30 s of simulated time. It then copies the published `FlowVolume` into an `AirFrame` for the window.

**Where the step splits.** LiveRun's step is: the drone's step (CPU), its rotor rows, `rotor_discs()`, `spread_actuator()` into `du`, then `fluid_step()`. `advance_frame()` ends with `publish_render_volume()`. Only the last two touch the grid. The seam is a fluid backend inside LiveRun with three calls:

- `step(discs) -> FluidHealth`: the CPU backend spreads the discs and calls `fluid_step()`, as today; the GPU backend is `GpuFluid::step(discs)`, which gathers the same discs on the device.
- `publish(origin, FlowVolume&)`: `publish_render_volume()` or `GpuFluid::publish()`.
- `read(MacGrid&)`: what needs the fields on the host (LiveRun's save, a test).

LiveRun keeps everything else: the drone, the controls, the discs, the clock, the run label. `AirflowRun` chooses the backend at `start()` and its worker loop doesn't change. GpuFluid stays in `spade_airflow_runner`, which the sandbox may already link (its CMake note names "the sandbox's demonstration harness").

**What crosses each step.**

| Direction | What | When | Size at 32³ |
|---|---|---|---|
| host to device | the discs' records (`disc_records()`) | each step they change; under the Pilot section, every step | 4 x 304 floats, 4.9 KB |
| device to host | nothing | each step | |
| device to host | the published halves (`publish()`) | each frame, 30 a second | 32,768 cells x 16 B, 512 KB |
| device to host | u, v, w and p (`read()`) | only for LiveRun's save, or a test | 0.53 MB |

The rotor probes and the downwash readout come from LiveRun's rows and body and from the published volume (`sandbox/airflow_live.hpp`), so the editor needs no other readback.

## 2. Expected speed

Measured once, through jobrun's gpu class, on the RTX 3060 Ti (07:46Z): GpuFluid on the editor's grid (32³ at 25 mm, h 1 ms, the air's nu, four discs of the reference quad at hover, open faces, no parts), from rest, beside the CPU's `spread_actuator()` and `fluid_step()` on the same box. The test was a scratch opt-in test, not committed (Slice 2 makes it a real one).

| | GPU | CPU |
|---|---|---|
| one step, the discs unchanged (mean of 300) | 1.94 ms | 35.6 ms |
| one step, the discs changed every step (the Pilot's case) | 2.01 ms | (the same) |
| one publish | 1.31 ms | 17.8 ms |
| one read of u, v, w and p | 0.54 ms | |

A frame is 1/30 s of simulated time: 33.3 steps at h = 1 ms, then a publish. The fluid and the publish cost the CPU 33.3 x 35.6 + 17.8 = 1204 ms of wall time a frame, 1/36 of real time, and the GPU 33.3 x 2.01 + 1.3 = 68 ms, 1/2 of real time: 18x. The editor's measured 1/43 is 1433 ms a frame, so about 230 ms a frame (7 ms a step) goes outside the fluid: the drone's step, the discs, the hand-over, and whatever the editor's other threads take from the worker. That share is not measured here, and it stays on the CPU. If it is real, Run air on the GPU lands between 1/2 and 1/9 of real time. Slice 1 should time it.

At 32³ the GPU's step is bound by its launches, not its cells: the D1 box has 220 times the cells and takes 33 ms, only 17 times longer. About 190 dispatches, each with its barrier, and one submit and fence wait a step. Real time (1 ms a step) would need fewer of them. Two levers, both later and both measured first: a frame's steps in one submission when its discs hold for the frame (`ChainRunner::submit(chain, n)` already takes a count), and fused kernels in the step's chain.

## 3. What blocks it

1. **Physics' plan-A fix.** Solid-free D1 diverges under plan A (`n_v` 2) on the CPU, and GpuFluid follows it bit for bit: on 112x81x112 both go non-finite at step 68. Every odd-ny grid probed went; no even one did within 176 steps. The editor's 32³ is even, and its R/dx is 3, not 19, and it runs on the CPU today. Before Run air moves, three things need confirming: (a) the fix's cause covers the editor's case, at any cube size a region may have (an odd n included); (b) the fix lands in the GPU's chain too, so GpuFluid stays bit-equal (if it adds V-cycles, that's a push word; if it brings the CG to solid-free runs, that's B5's chain); (c) the editor's own runs keep their bits, or the change is announced.
2. **Solids.** Run air flies its drone with the parts out of the air (`air_model()` clears them), so the first slice needs none. Physics' P-1 (thick parts solid) would need G3: GpuFluid takes solids once Physics' B5.6 hooks land.
3. **Health.** LiveRun refuses a step whose `FluidHealth` reports a fault; GpuFluid's health today carries only `pressure_iterations`, with no NaN scan and no residual. Until a device scan exists, the GPU backend checks each published frame on the host (flow_movie's `frame_check.hpp`: a non-finite half, or a cell past ten times the case's expected top speed), and a bad frame ends the run with its error, one frame (at most 34 steps) later than the CPU's per-step check. A device scan (a NaN count, step_chain's, deferred in B7) closes that gap.

## 4. Refusals and the CPU fallback

`GpuFluid::create()` refuses what `make_fluid_scratch()` refuses, a SolidField with parts, more than 16 discs, a workgroup size with no variant, and what the device can't hold (its limits, the VRAM budget); no Vulkan device refuses at the context. Each refusal falls back to the CPU backend before the run starts, and the run label says so ("air on the CPU: <why the GPU refused>"). A failure mid-run (a device lost, a refused submit) ends the run with its error in the last frame, as a CPU fault does: the state lives on the device, and moving it to the CPU mid-run is not in the first slice. On the device of record the two backends give the same bits, so the fallback is the same run, only slower; on another device the GPU's numbers keep their band (TD-14).

## 5. The smallest first slice

**Slice 1 (Physics' LiveRun, with Rendering): the seam, CPU only.** LiveRun gains the backend; the CPU backend is today's code moved behind it. Tests: LiveRun's tests unchanged; a new one runs the reference quad's live case for 60 steps under a changing program and compares u, v, w, p and every frame's halves with and without the seam, bit for bit.

**Slice 2 (Rendering): the GPU backend.** A `GpuLiveFluid` over GpuFluid, chosen by a LiveRun start option, nothing in the editor yet. Tests: the same 60 steps on 32³ with the GPU backend against the CPU's, bit for bit on the device of record, with the discs changing every step (the Pilot's case); the frame check stopping a run, by a test hook that poisons one step; each refusal falling back, by name; the step time at 32³, opt-in.

**Slice 3 (Interface): Run air uses it.** `AirflowRun::start()` asks for the GPU backend and falls back; the inspector shows which. Tests: a forced refusal runs on the CPU and says so; the tour's editor step still passes.

Slices 1 and 2 can be reviewed together. Neither needs the plan-A fix to land; Slice 3 waits on it (section 3).
