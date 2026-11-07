# L1, the LBM lattice: the GPU twin -- plan

**Owner:** Physics. **Status:** DRAFT 2026-11-06, the outline approved by the lead; this file is the plan only, nothing built. **Spec:** `2026-11-04-l1-lbm-design.md` (sections 1, 2 and 4; section 8 for the rung) and the CPU reference it twins (`engine/physics/lbm/`). **Order:** after L1's READY (the CPU reference); the E/F inputs for Core's stage 7 follow.

**Goal:** the CPU reference's two passes (`lbm_macro`, `lbm_collide_stream`) on the device of record (the RTX 3060 Ti, driver 572.83), bit-equal to it by `TD-14`: this is the high-fidelity tier's engine (D9, D10).

**Out of scope, named so it is not lost:** publishing an LBM flow field (it comes with the LBM module plan, through stage 7 batch F's publish path, so Interface has one `FlowVolume` reader); the module API wiring; solids (L2: `lbm_flags` is reserved below and not declared); the fused one-kernel step (T7, optional).

## Global constraints

- **`TD-14`, bit-equal where a kernel is exact.** Every comparison is a `memcmp` of the whole array. An op that cannot be made bit-equal gets a measured band and its own record (`TD-14`, `TD-2`), never a quietly loosened `memcmp`.
- **Division and sqrt (CORE-3, SPIR-V rules E1/E2).** Vulkan does not round `/` or `sqrt` correctly. Every `/` and every `sqrt` in `lbm_*.slang` goes through `engine/shaders/fp32_math.slang`'s `div32_ftz` and `sqrt32_ftz` (integer-corrected, bit-equal to the host's `divss`/`sqrtss` under FTZ|DAZ, NaN bits included, proven by `tests/test_fp32_exact.cpp` and `tests/test_gpu_fp32_math.cpp`), with `NoContraction` on every other float op (`SPIR-V rule P1`). No bare `OpFDiv`, `Sqrt` or `InverseSqrt` appears (E1, E2). Cameron's 2026-10-22 lift of R3 for "the airflow solver's sqrt" is read by the lead to cover the LBM as the airflow solver's high-fidelity tier; that reading is the lead's to report.
- **What the step needs, nothing more.** The whole step uses `+ - *`, comparisons and these: `/` in `macro_of` (`ux = jx / rho`, three of them), `omega_of` (`1 / (3 (nu + nu_t) + 1/2)`), WALE (`num / den`) and the wall mean (`1 / crossed`, crossed in {1, 2, 3}); `sqrt` in WALE (`sqrt(sd2)`, `sqrt(s2)`, `sqrt(sqrt(sd2))`). No `pow`, no libm, no other intrinsic (`TD-3`, `SPIR-V rule P5`). **If a later change to the collision needs anything else, stop and tell the lead.**
- **Subnormals: the mechanism, named (CORE-5 admits any Vulkan 1.1 device, `SPIR-V rule P3` forbids requesting a denormal mode).**
  - *CPU:* the step runs under `spade::math::ScopedFlushSubnormals` (MXCSR 0x9FC0: FTZ and DAZ, round to nearest, all masked), through the wrapper rule (a thin wrapper, then one call to a `SPADE_NOINLINE` body). The WorkerPool runs each item under the caller's word, so the threaded step is under it too. **Today `step_flow` is NOT under the scope** (the CPU reference runs with gradual underflow); T0 puts it there and measures whether any case ever makes a subnormal.
  - *Device, `/` and `sqrt`:* explicit and device-independent. `div32_ftz` and `sqrt32_ftz` flush in integer bit operations, on any device.
  - *Device, `+ - *`:* the 3060 Ti's own default, which flushes fp32 arithmetic and keeps loads, stores and copies (measured, `GpuDenormProbe`, `tests/test_gpu_denorm_probe.cpp`, passing at `39d9a0a`). That is evidence for the device of record only. It is not enforced in the kernel (a flush after every op would be thousands of integer ops a node), so **the bit-equal claim is the device of record's, and another device is banded by `TD-14` and measured when it is added**. T1 adds an LBM probe: a field seeded with subnormal populations stepped on both, `memcmp` equal, so the claim is tested on the LBM's own op mix and not only on the NS tier's.
  - The CPU's MXCSR alone is not evidence for the device; the rows above are.
- **Float controls (`SPIR-V rule P6`):** every airflow kernel declares `SignedZeroInfNanPreserve` for fp32. I take the LBM kernels to be airflow kernels for P6 (the health line's finite check and the stability map's blow-up detection depend on NaN-correct compares); Core confirms when it reviews the scan rows.
- **Line-cited port (fp32_math.slang's rule 1).** Each Slang function is the same statements in the same order as the C++ it twins, and carries the C++ line range it was ported from. A reviewer diffs the two side by side.
- **One queue, one ticket at a time (L28);** GPU tests run through jobrun's `gpu` class; a `Gpu*` test starts with the usual skip when Vulkan is absent.
- **Rule 3:** the files outside `engine/physics/lbm`, `spike/lbm` and `tests/test_*lbm*.cpp` are listed below and were granted: LBM rows in `kSpirvModules` and `tests/test_slang_layouts.cpp` (Core reviews them), the `engine/CMakeLists.txt` and `tests/CMakeLists.txt` entries, and the `bindings.slang` lines (Core reviews them).

## What is on the device

- `f`: 27 x N fp32, direction-major, shifted (f' = f - w), one copy (the esoteric rule needs no second); `u`: 3 x N fp32. 120 B a node (124 B with L2's flags). A single storage buffer is limited by `maxStorageBufferRange` (4 GiB on NVIDIA devices; read it from the device report at T1, it is not measured here), so one `f` buffer holds N up to about 39.7M nodes (342^3). Larger needs `f` split by direction groups and is a later task; `create()`-time refusal by name (Core's `device_limits`, L6) before any allocation.
- **Bindings (the lead's answer 1):** set 1 bindings 40 to 47, named `lbm_*`, in `bindings.slang` section F, append-only and never renumbered; Core keeps 28 to 39 for stage 7. Declared now: `lbm_f` (40, `RWStructuredBuffer<float>`), `lbm_u` (41). Reserved, not declared: `lbm_flags` (42, L2), 43 to 47. The chain runner binds `buffers[i]` at `binding_base + i`; the LBM chain uses `binding_base` 40 and two buffers (check the runner's per-kernel subset layout at T1).
- **Push constants (one range, at most 128 B):** nx, ny, nz, parity (the step counter's low bit), nu, cw, the periodic mask, and the 18 floats of `wall_u[axis][side][3]`: 100 B.

## Files

- Create: `engine/shaders/kernels/lbm_macro.slang`, `lbm_collide_stream.slang`, `engine/shaders/lbm_lattice.slang` (kC, kW, opposite, the RR3 tables, written as the C++ `lattice.hpp` has them); `spike/lbm/lbm_chain.hpp/.cpp`, `spike/lbm/CMakeLists.txt`; `tests/test_gpu_lbm.cpp`.
- Modify: `engine/shaders/shared/bindings.slang` (lines 40, 41), `engine/CMakeLists.txt` (the two kernels, the spike target), `tests/CMakeLists.txt` (`test_gpu_lbm.cpp`), `tests/test_slang_layouts.cpp` (`kSpirvModules` rows: `lbm_macro`, `lbm_collide_stream`, profile `parity`, with the exact-op rules E1/E2 and P6 as the scan lists them), `engine/physics/lbm/lbm_flow.hpp` (T0: the scoped step).

## Tasks (each RED/GREEN, with its named mutation; one ticket at a time)

### T0: the CPU step under the flush scope (no GPU)

- Wrapper `step_flow_flushed(LbmFlow&, WorkerPool*)` and `flow_sums_flushed`: the scope, then one call to a `SPADE_NOINLINE` body. The drivers and tests that stand for "the reference" call these.
- Measure first (scratch, not committed): read MXCSR's DE and UE flags around every step of C0a, C0b (64^3), C1 Re 100 and the stability map's points. If none raises a flag, no record moves and T0 says so. If one does, the cases that moved are re-run and the old records kept (`TD-1`).
- RED: `LbmFlush.AStepOnASubnormalFieldFlushesLikeTheScope` (a field of subnormal populations, expected bits from the host under the scope, outside it differ). Mutation: the scope removed (the step leaves subnormals); the OR-word scope (`saved | 0x8040`) with an RC-down caller.

### T1: the lattice tables, the params and the probe

- `lbm_lattice.slang` against `lattice.hpp`: a dump kernel writes kC, kW, opposite and the RR3 constants to a buffer; the test compares them to the CPU's, bit for bit. The `kSpirvModules` rows and the `bindings.slang` lines land here, with Core's review.
- The denormal probe on the LBM's op mix: one step of `lbm_macro` over a field seeded with subnormal populations against the scoped CPU (`GpuLbm.SubnormalsFlushAsTheScopeDoes`).
- Mutation: one table entry wrong (the dump test); a flush removed from `sqrt32_ftz`'s path (the probe).

### T2: `lbm_macro`

- Bit-equal `u` after one step, on both parities, over the 12x10x7 periodic Taylor-Green of `LbmThreads` (odd sizes, every wrap) and a quiet field.
- Mutation: the parity word swapped; `/` replaced by the bare operator (the scan rejects it; the test names the rule).

### T3: `lbm_collide_stream`, periodic

- `memcmp` of the whole `f` and `u` after n steps for n in {1, 2, 7, 64} (both parities, many times over) against the serial CPU step, WALE on and off, nu in {0, 1e-3}.
- Mutation: contraction left on one op (the scan and the test); the third-order recursion dropped (the unit test and this one); a slot swapped on the odd write.

### T4: walls and the lid

- The lid box of `LbmThreads` (the moving-wall term, the corner links, the periodic z) bit-equal at the same n.
- Mutation: the wall term dropped; the wall mean `1 / crossed` replaced by a multiply by a rounded reciprocal.

### T5: the case digests and the records

- C0b at 64^3 and C1 Re 100 to their full length on the device, a digest of `f` compared to the CPU's at each sample step; the records (`tests/data/airflow/records/`) name the `describe()` device line as the path. The diagnostics (K, mass, momentum) stay summed in double on the host from downloaded populations on both paths (design section 4); a sample every 20 steps or more at large N.
- If any of this is not bit-equal: stop, find the op, and only then propose a band (`TD-14`).

### T6: the performance record

- MLUPS per rung on the device at 64^3, 128^3 and the largest N the buffer allows, solo in the queue, with the holders recorded; this is the Vulkan column of `lbm_seconds`. My estimate to be replaced by this measurement: about 360 B moved a node-step (macro 108 R + 12 W; collide 108 R + about 24 effective for the neighbours' `u` + 108 W) at 448 GB/s peak, of the order of 0.5 to 1 GLUPS, roughly 600 to 1200 times one CPU thread's 0.79 MLUPS. At that rate C0b at 128^3 (4074 steps) takes seconds to tens of seconds.

### T7 (optional): the fused step

- One kernel reading the previous step's `u` from a ping-pong pair (D2's W1: 136 B a node and a one-step lag in nu_t), a different kernel with its own bit test against the CPU twin of that scheme, never against W2's bits. Only if T6 shows the second pass costs more than a third of the step.

## Review focus

- A reduction or atomic slipped into either kernel (the esoteric rule is why there is none).
- A `/` or `sqrt` that reaches the device without `div32_ftz` / `sqrt32_ftz`, or a contractable op without `NoContraction` (the scan list must name both kernels).
- The CPU and the shader disagreeing on the order of the 27-term sums, or on the nodes the wall mean (`crossed`) is taken over.
- The subnormal claim stated for a device other than the device of record.
- Binding numbers outside 40 to 47, or a renumbering.
- A single buffer past `maxStorageBufferRange` accepted instead of refused by name.
