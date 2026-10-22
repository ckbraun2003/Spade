# Exact fp32 division and sqrt in fp32_math.slang: Core's plan (2026-10-22)

The lead gave the go on 2026-10-22, relaying Cameron's choice of the estimate forms and his restated E1/E2 (backlog 81dedfb). The lead APPROVED it on 2026-10-22 and ruled:

- **Grants.** spirv_scan.hpp, the test_slang_layouts.cpp comments and 00-decisions' "in effect" are Test/Docs' edits, with Core reviewing the scan. Core edits Physics' three gnss comments, and Physics reads the diff. The device gate is Rendering's to run, with Core's ops on its probe.
- **(a)** core/exact-div-sqrt is cut on top of test-docs/e1e2-taint (c49c0b4), so the GREEN gates with the scan in place. The lead merges the scan first, then this branch, in the same batch.
- **(b)** The branch gets its own seeded leg: the proof's MXCSR intrinsics on gcc-13 need their own run.
- **(c)** Changed by the lead the same day, from Physics' approved FTZ plan (physics/airflow-ftz cfae09f): C0 creates the scope directly at engine/core/detail/flush_subnormals.hpp, so it has one home from the start, and the tests include it from there. Physics' branch then adds the fixed 0x9FC0 word (FTZ, DAZ, round-to-nearest, all masked; the battery's divss/sqrtss reference is unchanged by it), the off-x86-64 #error, SPADE_NOINLINE and a dated CORE-3 note in core/00-decisions.md, with Core's agreement. The engine scans apply: no "chrono" substring, even in a comment.
- **(d)** The GPU label goes through jobrun's gpu class: nothing queued after 23:45Z and none while Cameron is gaming. Correctness suites may run beside other GPU work; any timing test runs alone.

Built from Rendering's `exact_fp32.slang` @ f71252b, its device battery and its numpy model. Rules: E1/E2 as restated (81dedfb, c79b1b4). Branch: `core/exact-div-sqrt` off master, in `spade-wt/core-2` (basic-quad-b is merged and the tree is clean).

## 1. Design

### 1.1 Division: `div32_ftz(a, b)`

This is Rendering's `exact_div` with x86 NaN bits, and with the estimate as an argument. The `_with` helper is the one correction (TD-9): the op and the probe's injected-estimate mode both call it.

```slang
static const uint kBitsDefaultNaN = 0xFFC00000u;  // x86 QNaN indefinite
[ForceInline]  // the estimate never crosses an OpFunctionCall (the taint scan)
float div32_ftz_with(float a, float b, float y) {      // y ~ 1/bm, bm in [1,2)
    // screen (1.3), then exact_div's body unchanged:
    // Q = uint(am * y * 2^s);  R = asint((A << s) - B * Q);
    // k = int(floor(float(R) * y * 2^-23 + 0.5));  Q += k;  R -= k * B;
    // if (2R > B) Q++; else if (2R < -B) Q--;       // strict: no ties exist
    // e >= 255 -> sign|inf;  e <= 0 -> sign (FTZ)
}
float div32_ftz(float a, float b) {
    return div32_ftz_with(a, b, 1.0f / asfloat((asuint(b) & 0x7FFFFFu) | 0x3F800000u)); // E1 estimate
}
```

**Proof.** The quotient significand is q\* = A·2^s/B, in [2^23, 2^24). The estimate is y = (1/bm)(1+ε).

- **Q0:** |d0| = |q\* − Q0| < 2^24|ε| + 2.
- **Remainder:** R0 = B·d0, with B < 2^24. The 32-bit wrap is exact iff |R0| < 2^31, which holds when |d0| < 128, i.e. |ε| < ≈2^-17.
- **Rounded step:** t = R0·y·2^-23 = d0(1+ε)(1+3δ), so |t − d0| < 2^-9. Then k = round(t) leaves |q\* − Q1| < ½ + 2^-9.
- **Final:** 2R1 against ±B decides |q\* − Q| < ½ exactly. One ±1 suffices because |d1| < 3/2. Ties are impossible (Rendering's 2-adic proof).
- **Margin:** at Vulkan's 2.5 ulp, |d0| ≤ 7. The proof covers ≥ ±63 ulp of y. The numpy model (2×10^5 pairs) had 0 misses through ±62; the first misses came at ±64.
- **No fma:** only the slack in those bounds is used, so it holds whether the driver fuses or contracts the estimate's FMul/FAdd or not.

### 1.2 Sqrt: `sqrt32_ftz(x)`

This is `exact_sqrt` with the same seam: `sqrt32_ftz(x) = sqrt32_ftz_with(x, rsqrt(xm))`, where xm = `asfloat(fx | ((127+odd) << 23))` and odd = (ex+1) & 1. The rsqrt (InverseSqrt) is the E2 estimate.

**Proof.** S\* = √X, where X = M·2^(23+odd) is in [2^46, 2^48).

- **S0:** |e0| = |S\* − S0| < 2^24|ε| + 2.
- **Remainder:** R0 = e0(2S\* − e0) is exact in the wrap when |e0| < 63, i.e. |ε| < ≈2^-18.
- **Rounded step:** it lands within ½ + 2^-10.
- **Final:** in integers, R1 > S1 ⟺ √X > S1 + ½, and R1 ≤ −S1 ⟺ √X < S1 − ½. There are no ties.
- **Margin:** at 2 ulp, |e0| ≤ 6. The proof covers ≥ ±31 ulp. The model had 0 misses through ±30 and its first at ±32. Rendering's "~15" is conservative.

### 1.3 Special cases (divss/sqrtss under MXCSR FTZ|DAZ, bit for bit)

| Input | div32_ftz | sqrt32_ftz |
|---|---|---|
| NaN | a's quieted (`a \| 0x00400000`), else b's (divss returns its first source) | `x \| 0x00400000` |
| Invalid | 0/0 and inf/inf → `0xFFC00000` | negative normal, −inf → `0xFFC00000` |
| inf | inf/finite → ±inf; finite/inf → ±0 | +inf |
| 0 or subnormal (DAZ) | x/0 → ±inf; 0/x → ±0 (sign sa^sb) | ±0, sign kept |
| Overflow | biased e ≥ 255 → ±inf | none |
| Underflow (FTZ) | e ≤ 0 → ±0. Nothing rounds across a binade, so both tininess rules agree | none |

- **Canary:** `0x00FFFFFF / 2` is FLT_MIN·(1 − 2^-24). divss under FTZ gives +0, where IEEE gives FLT_MIN.
- **Rendering's model:** `ref_div` (IEEE, then a flush in integers) would get the canary wrong. Its samples never hit it. The C++ test uses the hardware's divss.

### 1.4 TD-9: log32_div is deleted

- log32 calls `div32_ftz(f, 2.0f + f)`, and probe mode 4 calls `div32_ftz`.
- **log32's bits don't change on any device.** Its domain is all normal: f is +0 or normal, 2 + f is in [1.71, 2.42], and the quotient is in [2.4e-8, 0.243]. So the flush never applies, and the result is IEEE's, the host's `/`.
- **Side effects:** the gaussian's four kernels (medium_update, rotors, sensor_imu, sensor_gnss) gain the estimate's OpFDiv. They are parity-profile, where E1 doesn't apply. log32 also gets faster.
- **The header's rule 3 and audits** are rewritten to the restated E1/E2. Kat has no log32_div (grep).

### 1.5 Rendering's three calls: recommendations

- **(a) NaN: exact x86 bits** (table 1.3).
  - The module's standard is bit-identity, and the special branch costs the normal path nothing.
  - The device already keeps NaNs through asfloat and the store (log32's edge battery).
  - If the 3060 Ti changes a payload, payload NaNs fall back to comparing as a class, and that is recorded.
- **(b) Names: `div32_ftz` and `sqrt32_ftz`, over the whole domain,** documented as "divss/sqrtss under MXCSR FTZ|DAZ, NaN bits included". The `_ftz` suffix warns that, unlike log32/exp32, they flush.
- **(c) Keep the truncation.** It is the form the 39.8e9-result sweep measured. Rounding adds an FAdd and a Floor on the estimate to win 1 unit of 128.

## 2. Red/green tasks

| Commit | Files (all Core's) | Expected |
|---|---|---|
| **C0** | new `tests/flush_subnormals.hpp` (the `ScopedFlushSubnormals` now at test_fp32_math.cpp:633-645), which test_fp32_math.cpp includes | green; a refactor |
| **C1 RED** | **Tests:** new `tests/fp32_exact_battery.hpp` (the line-cited port, `nudge`, the reference, the specials, `near_ties`, splitmix) and new `tests/test_fp32_exact.cpp`; one line in `tests/CMakeLists.txt` (standing grant); 3 suites in `tests/test_gpu_fp32_math.cpp`. **Slang stubs:** `div32_ftz` → `log32_div`, `sqrt32_ftz` → `x`; the `_with` forms ignore their estimate. **Probe:** modes 7 (sqrt), 8 (div with an injected y), 9 (sqrt with an injected r). The port stops at Q0/S0 | **Fail:** Fp32Exact's Division, Sqrt and FarPast tests (misses even at k = 0); the 3 new Gpu suites. **Green:** the rest, including FloatControlsPinned (no OpFDiv yet) |
| **C2 GREEN** | **Slang:** the real ops; log32 calls div32_ftz; log32_div deleted; header rewritten. **Also:** the probe's header, `rng.slang:77`'s comment, the port's correction, Physics' comments (if granted) | all green except `SlangSpirv.FloatControlsPinned` (E1 and E2 findings), until Test/Docs' scan merges |

**The list (the lead, via Test/Docs, 2026-10-22).** fp32_math.slang gains a section headed exactly "Functions allowed to hold an estimate (E1/E2)". The taint closure that Test/Docs and Core agreed (engine/testing/spirv_scan.hpp, pass 3) proves only that no float estimate reaches anything but an integer conversion. It also passes an uncorrected integer use such as `uint(a / b)`. So the guarantee covers exactly the functions the injected-error battery covers, and an estimate outside this list is a review finding. Each entry names its battery tests:

| Function | Estimate | Battery |
|---|---|---|
| `div32_ftz_with` (and `div32_ftz`, which supplies y) | OpFDiv `1/bm` | `Fp32Exact.DivisionRoundsExactlyWithEstimatesThreeUlpsOff`, `Fp32Exact.TheCorrectionHoldsFarPastTheVulkanBound`, `GpuFp32Math.ExactCorrectionRoundsInjectedEstimatesOnTheDevice` |
| `sqrt32_ftz_with` (and `sqrt32_ftz`, which supplies r) | InverseSqrt `xm` | `Fp32Exact.SqrtRoundsExactlyWithEstimatesThreeUlpsOffOverItsWholeInputSpace`, `Fp32Exact.TheCorrectionHoldsFarPastTheVulkanBound`, `GpuFp32Math.ExactCorrectionRoundsInjectedEstimatesOnTheDevice` |

C1 adds the heading with the two stubs listed. C2 keeps it exact. No FAbs or other op outside the closure is applied to an estimate; one would need a dated amendment.

**Owed to Test/Docs with C1:** the SPIR-V use chains of each estimate, the op names and the hash, from slangc on the GREEN draft in the scratchpad. Check that no estimate reaches an OpFunctionCall, and use no FAbs.

### CPU proof battery (`Fp32Exact.*`; runs on the leg)

The estimates are the correctly rounded 1/bm (host divss) or 1/√xm (via double), pushed by every k ∈ [−3, 3] bits. Where the exact estimate is a power of two (bm = 1, xm = 1), Vulkan's ulp is the gap above, so those rows use k ∈ [−6, 3]. That set holds every float within the bound, so the sqrt row is a proof by exhaustion.

| Test | Inputs | Default | `SPADE_EXACT_SWEEP=1` |
|---|---|---|---|
| `TheHostReferenceFlushes` | `0x00FFFFFF/2` → +0; subnormal/subnormal → `0xFFC00000`; sqrt(−subnormal) → −0; IEEE values without the scope | 6 | — |
| `DivisionRoundsExactlyWithEstimatesThreeUlpsOff` | every fa for b = 1 and b = 2 − 2^-23; 2^22 random significand pairs; 2^20 near-ties | ≈7e7 | every fa × 16 b, every fb × 8 a, 2^28 random, 2^25 near-ties |
| `SqrtRoundsExactlyWithEstimatesThreeUlpsOffOverItsWholeInputSpace` | all 2^24 floats in [1, 4) (every (fx, odd)) | 1.2e8 | k ∈ [−8, 8] |
| `TheCorrectionHoldsFarPastTheVulkanBound` | 2^20 random. Asserted: div k = ±32, sqrt k = ±16. Printed: the first failing k | ≈5e6 | — |
| `SpecialsAndExponentEdgesMatchTheHostBitForBit` | every pair of ~48 specials (including NaN payloads); 2^16 pairs per result exponent −1..2 and 253..256; 2^22 random bit patterns, k ∈ {−3, 0, 3} | ≈1.4e7 | 2^28 random |
| `InjectedErrorIsVisibleWithoutTheCorrection` | k = ±3 with the port stopped at Q0, and stopped after the k-step: EXPECT misses > 0 | small | — |
| `ThePortIsIndependentOfTheFlushMode` | the port gives the same bits under the default MXCSR and under FTZ\|DAZ | small | — |

**Budget:** ≤ 5 s release and ≤ 40 s debug, measured under ctest. Rows past that move behind the env var.

**MXCSR and determinism.**

- **The flush:** `_mm_getcsr`/`_mm_setcsr` (`<xmmintrin.h>`) are the same on MSVC x64 and gcc x86-64. The existing helper is the one C0 moves; Rendering's `Flushing` copy dies with its branch. Non-x86 skips with a reason.
- **The reference:** `_mm_div_ss`/`_mm_sqrt_ss` (and the `_ps` forms) run inside the scope, in a `noinline` function, over data in memory, so nothing folds to IEEE at compile time. The canary fails if the scope is not in effect.
- **The port:** uint32_t for every Slang int op and `std::bit_cast` for asint/asfloat, so there is no signed-overflow UB. It compiles under `spade_fp_strict` (-ffp-contract=off) and /fp:precise.
- **Inputs:** fixed-seed splitmix64, single-threaded. The `[Fp32Exact] name: n=… misses=…` lines must match between MSVC and the leg's gcc-13.

### Device tests (`tests/test_gpu_fp32_math.cpp`, Gpu-prefixed so labelled `gpu`)

| Test | Inputs |
|---|---|
| `GpuFp32Math.ExactDivisionMatchesTheHostUnderFtzDaz` | specials², 1/b over [1, 2), a/3 over [0.5, 2), 2^20 near-ties, 2^22 random, the exponent edges; compared as bits, NaN included |
| `GpuFp32Math.ExactSqrtMatchesTheHostUnderFtzDaz` | biased exponents 0-1 (both signs), 126-127 (the whole correction space) and 254-255 |
| `GpuFp32Math.ExactCorrectionRoundsInjectedEstimatesOnTheDevice` | modes 8/9, 2^20 each, with the host's k ∈ [−3, 3], ±16 and ±32 estimates. Checked against divss/sqrtss and against the C++ port, which proves the port is a faithful port |
| existing `CorrectlyRoundedDivisionMatchesTheHostDivide` | now through div32_ftz; unchanged |

**What moves: nothing.**

- No CPU code changes, so no golden or digest moves.
- `Log32*`, `EdgeCases*`, the constants roster (no new constants) and the gaussian parity tests stay bit-identical.
- A changed GPU bit is a bug, not a band move.

## 3. Cross-realm files (grants needed before the build)

| File | Realm | Change and why |
|---|---|---|
| `engine/testing/spirv_scan.hpp` | Test/Docs | E1/E2 become the agreed taint closure: the EXACT probe now holds the estimates. Test/Docs writes it red-first on its branch |
| `tests/test_slang_layouts.cpp` (comments ~440-446, ~527-533) | Test/Docs | fp32_math_probe's "no OpFDiv … integer long division" and sensor_gnss's "no OpFDiv at all" go stale |
| `docs/design/test-docs/00-decisions.md` | Test/Docs | E1/E2 become "in effect" at Core's merge |
| `engine/sensors/gnss.cpp:78-81`, `gnss.hpp:211-214`, `engine/shaders/kernels/sensor_gnss.slang:27-31` | Physics | Comment only: "log32_div() is a restoring integer long division, domain-restricted" becomes "fp32_math.slang's div32_ftz is the exact division". The recursion argument stays, and the stale ":83-88" cite is fixed |
| `rendering/exact-div-sqrt` scratch merge (`exact_fp32_probe.slang`, `test_gpu_exact_fp32.cpp`) | Rendering | The probe imports fp32_math and calls div32_ftz/sqrt32_ftz; `same()` compares bits. This is the device gate; never merged |

Core's own files, no grant: `fp32_math.slang`, `fp32_math_probe.slang`, `rng.slang` (a comment), `test_gpu_fp32_math.cpp`, `test_fp32_math.cpp`, the three new test files, and `tests/CMakeLists.txt` (standing grant, named in READY).

## 4. Gates

| Gate | What, where |
|---|---|
| gcc check | `--compile` on the three changed and new .cpp files; plain on the two headers |
| Release CPU | `-LE gpu`, in jobrun `test` chunks |
| Debug timing | `ctest -R Fp32Exact` once in debug: < 60 s (the batch gate runs debug) |
| GPU label | all of `-L gpu` on the box's RTX 3060 Ti (jobrun `gpu`, one slot). fp32_math reaches rng, which reaches 4 parity kernels. Skips are named (TD-13). At plan time `gpu-0` was free with no game; Rendering's D1 film is the other GPU job, so check `--queue-status` |
| SPIR-V | `SlangSpirv.*`, after Test/Docs' scan merges |
| Leg | a seeded `docker-leg.ps1`, in the background (`test --floor-gb 8`): the proof's only gcc run, with counts equal to MSVC's |
| Full CPU proof | `SPADE_EXACT_SWEEP=1 … --gtest_filter=Fp32Exact.*`, recorded in READY |
| **Device gate** | Rendering's full sweep (`SPADE_EXACT_SWEEP=1 GpuExactFp32.*`, a few minutes) on Core's tip, ticket `spade-rendering-exact-sweep-<sha>`: 0 misses, NaN as bits |

Not here: Rendering's kernel adoption; rng_next_gauss's sqrt (R3 was lifted only for the solver, so that is Cameron's call).

## 5. Open questions

1. **Lead:** grants. Recommended: Core edits Physics' three comments and Physics reads them; Test/Docs takes the test_slang_layouts.cpp comments into its scan branch.
2. **Lead / Test/Docs:** recommended: the scan merges first and Core's GREEN gates on top. The alternative is one combined merge.
3. **Lead:** this branch's own seeded leg (recommended: it is the proof's only gcc run), or only the batch gate's leg (the tier rule)?
4. **Lead:** the MXCSR scope's home. Recommended: `tests/flush_subnormals.hpp` now, and Physics promotes it to engine/core for the CPU flush (spec §4.5). The alternative is engine/core now.
5. **Rendering:** do you agree with 1.5 (a) x86 NaN bits, (b) the names and the `_with` seam, (c) truncation, and will your sweep compare NaN as bits?
6. **Rendering:** when can the device gate run beside the D1 film? And do you want Core's injected-estimate device test kept or dropped?
