# Measure fp32 denormal behaviour on NVIDIA (the RTX 3060 Ti) — plan

**Owner:** Core, with Physics (probe kernels) and Test/Docs (the build flag and the scanner). **Ruling:** the user's, 2026-10-04: "Measure NVIDIA first" (`../../backlog.md`, `7909561`). **Status:** approved by the lead (2026-10-04), with the three additions marked "lead" below.

## Why

`VulkanContext` refuses a device whose `shaderDenormPreserveFloat32` is false, because every kernel declares `DenormPreserve 32` and requesting that mode on such a device is undefined behaviour (`cmake/SpadeSlang.cmake`, `compute/vulkan/context.cpp`).

This machine's RTX 3060 Ti reports:
- `shaderDenormPreserveFloat32 = false`;
- `shaderDenormFlushToZeroFloat32 = false`.

So it can request neither mode. 72 of the 83 gpu tests skip here. With the mode not requested, its fp32 denormal behaviour is implementation-defined, and nobody has measured it.

## What the ruling allows, and what it does not

- **Allowed:** measure. Run the gpu suite and denormal probe kernels on the 3060 Ti with the denormal mode NOT requested, under a test-only flag.
- **Not allowed:** any rule change.
  - Default builds keep the pin and the refusal, byte for byte.
  - An admission rule, if the results support one, comes back for the user's signature.
- **The flag never reaches an installed build.**

## The flag: `SPADE_MEASURE_UNPINNED_DENORMS` (CMake option, OFF)

It is configured only in a dedicated build tree, for example `build-ninja/release-unpinned` in a worktree. It is never a preset, and never `build-host` or `install-host`. When ON:

1. **Kernels (Test/Docs, `cmake/SpadeSlang.cmake`).** Every kernel and workgroup variant compiles without `-denorm-mode-fp32 preserve`. `-fp-mode precise` and everything else stays.
2. **The SPIR-V gate (Test/Docs, `engine/testing/spirv_scan.hpp`).** Rule P3 inverts for this build. A module must carry NO `DenormPreserve 32`, so the build measures what it says it measures. `SlangSpirv.FloatControlsPinned` reads the option through a compile definition on `spade_tests` only.
3. **Admission (Core, `compute/vulkan/context.cpp`).** Under a PRIVATE compile definition on `spade_compute`, `vulkan_available()` and `create()` skip ONLY the denormal-preservation clause. The loader, device and `SPADE_FORCE_NO_VULKAN` checks are unchanged. The context records that it runs unpinned, so a test can name it.
4. **The install guard (Test/Docs).**
   - With the option ON, the first install rule is `install(CODE "message(FATAL_ERROR ...)")`, so an install from this tree fails before copying anything.
   - The scanner (`consumer-smoke.sh`, or a leg step) asserts that the installed `spade_compute` archive and headers carry no trace of the option. One option is a marker string compiled only under it.
5. **The configure guard (Test/Docs; lead).** With the option ON, configure refuses unless Spade is the top-level project (`PROJECT_IS_TOP_LEVEL`). A consumer that pulls Spade in through `add_subdirectory` or FetchContent never installs, so the install guard alone would not stop it.

## The measurement

- **M1. Probe kernels (Physics; class list from Physics' grep of every kernel's code lines, 2026-10-04).**
  - **Already covered; run unchanged in the measurement build:** `fp32_math_probe.slang` and the 13 `GpuFp32Math` tests already diff `exp32`, `log32`, `sin32`, `cos32`, `log32_div` and correctly rounded division bit for bit against the host, at both subnormal boundaries.
  - **New probe modes**, on the same `probe_runner`, for the primitive classes the kernels actually emit. Each is compared bit for bit with the CPU twin (x86 SSE, FTZ and DAZ off):
    - load/store pass-through; add and subtract; multiply; abs and negate (PTX has `.ftz` forms of each); min and max;
    - **mul then add**, both `NoContraction`, with a subnormal product. fma is never emitted: `-fp-mode precise` marks every contractable op, and the SPIR-V scan forbids sum-of-products opcodes. This also shows whether the driver honours `NoContraction`, since a fused and an unfused result differ here;
    - **floor of a ±subnormal, then the int conversion.** `grid_build.slang` takes the cell index as `floor(pos / cell_size)`: preserved, floor(−denorm) is −1; flushed, it is −0 and the cell is 0. A flipped broadphase cell is the largest state consequence of any class;
    - **compares against zero** (`<`, `>`, `==`) with ±subnormal operands, which feed contact and penetration sign tests and clamps. A flushed −denorm compares equal to 0;
    - **max/min/clamp(x, 0)** with a subnormal x: 32 sites, in the rotor and command clamps;
    - **sqrt of a subnormal, and division by a subnormal** (1/length: inf or finite). These are the glm mirrors: `length()` in drag and collision, and the quaternion normalize in integrate.
  - **Cut:** int→float, which cannot produce a subnormal. float→int truncation of a subnormal is 0 either way; `floor` above is the case that matters.
  - **Harness rules.**
    - Buffers are float-typed, as in `fp32_math_probe` and the physics kernels, so pass-through exercises their real load and store path (a uint buffer's `asfloat`/`asuint` pair folds to nothing). Every operand comes from a buffer the compiler cannot see into, so nothing is constant-folded; the host writes and reads exact bits through memcpy. Integer results (floor then int, packed compare bits) come back as small exact floats, out of the subnormal range, so a store-side flush cannot corrupt them. (Physics, while building M1, 2026-10-05.)
    - Every class gets a control row on normal operands. A difference that also shows on normal operands is accuracy (div or sqrt ULP), not denormals.
  - For each class, report one of: identical; inputs flushed (DAZ-like); outputs flushed (FTZ-like); accuracy (the control row differs too); or other (show the bits).
  - **Out of physics scope:** `round()` and the 16 divisions in `raster_background.slang` are Rendering's; M2 covers them through the render gpu tests. No kernel does half-precision arithmetic.
- **M2. The gpu suite (Core).** All 83 `Gpu*` tests in the unpinned build on the 3060 Ti. For each: pass or fail, and on failure, the quantity, element and bits that differ. The parity framework already reports these.
- **M3. Cross-check (Core).** Map each M2 failure to an M1 class. A failure that maps to no class is a finding of its own.

## The report, and what follows it

`docs/design/core/2026-10-04-nvidia-denorm-report.md` holds:
- the driver and device, with the driver version pinned in every table;
- the M1 table;
- the M2 table;
- the M3 map;
- **its evidence limits (lead).** A probe kernel and the real kernels may compile differently, because NVIDIA's compiler may pick different instructions in context. So M2, on the real kernels, is the stronger evidence, and M1 explains it. Neither speaks for another driver version.

Then exactly one of:
- **Every class identical and M2 green:** propose a measured admission rule for the user's signature. For example, "admit a device without `shaderDenormPreserveFloat32` when a probe kernel run at context creation shows the classes the engine uses preserve denormals with the mode not requested." The proposal names its cost: an undefined-by-spec behaviour, held by measurement. It must also say what happens on a driver change (lead): either re-probe at every context creation (and state the cost), or admit by driver version.
- **Otherwise:** the list of operations that differ, and what each would take (kernel changes, a narrower rule, or none).

## Order

1. Test/Docs: the option, the Slang flag, inverted P3, the install guard and the scanner. Gate: the default build is unchanged, i.e. a full suite, the gcc leg, and `FloatControlsPinned` green as today.
2. Core: the admission bypass under the definition, with two new tests on a device that reports no fp32 denormal preservation (this box):
   - in the default build, `create()` still refuses with `Code::unavailable`, naming `shaderDenormPreserveFloat32`. No test pins that on real hardware today, because the Iris preserves.
     - It runs only on a device that reports no fp32 denormal preservation, and elsewhere skips with a reason naming the capability.
     - So that the skip cannot hide a regression on this box (lead), it first asserts that the capability bit is false, read directly from the device, and only then expects the refusal.
   - in the unpinned build, `create()` admits the device and reports that it runs unpinned.
3. Physics: the probe kernel and its test.
4. Core: run M1 and M2 on the 3060 Ti, then write the report.

Steps 1 to 3 are code on branches, with red/green and the gcc check. Step 4 is docs.
