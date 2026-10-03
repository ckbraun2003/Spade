# Physics — verification and grades

**Owner:** Physics for what is checked and the bands' content. Test/Docs owns the harness, golden-corpus governance, the SPIR-V scanner and the gates (`../test-docs/`). **Normative.**

## How a physics change is checked

1. **Against closed forms in double.** Each model's reference values are recomputed in the test from the header's equations, never read back from a run:
   - momentum-theory roots for the rotor;
   - Dryden coefficients at the branch thresholds;
   - ballistic and energy behaviour for the integrator;
   - SDF distances.
   `test_rotor.cpp` is the model: it checks the defining equations, then independent closed forms, then the whole pass.
2. **Physics validation:**
   - a ballistic drop;
   - symplectic energy behaviour;
   - hover trim (thrust = weight gives station-keeping);
   - climb and descent against momentum theory;
   - Dryden spectral sanity.
3. **The golden corpus** (`tests/golden/scenarios/`): CPU-golden, bit-identical per platform. A moved digest is a deliberate physics change or a bug, and the two look identical in a diff. **Understand it before regenerating**, and record why in that scenario's provenance block (the governance rules are Test/Docs's).
4. **CPU↔GPU parity** (`engine/testing/parity.hpp`): per-scenario bands on positions, velocities, orientations, angular rates and specific force, compared abs-or-rel. Bands are **measured, then pinned with margin and device provenance**, and **never widened** to admit a new pass. Several bands are pinned at zero because those quantities are bit-identical across backends: every `bounce` quantity, and the contact pair's positions, velocities and rates.
5. **Invariance:** solo vs in-set batching, workgroup sizes `{32, 64, 128}`, and CPU↔Vulkan snapshot restore are bit-identical (`test_gpu_invariance.cpp`).

**GPU parity runs only where the gate has a device** (`TD-13`). On the development box all `gpu` tests run inside `scripts\test.ps1`, and a skipped one is reported by name and does not count toward a green gate. The Docker leg excludes them (`-LE gpu`), so it says nothing about agreement. Anyone adding or touching a GPU physics path runs `ctest -L gpu` on real hardware and says so in the commit.

## Grades (`PHY-2`)

`L3`/`L4`: reference needs a CPU implementation and a golden; banded needs a measured band against the reference, with its record. The GPU is never a golden source.

| Module | CPU | Vulkan | Evidence |
|---|---|---|---|
| Integrator | reference | banded | `ballistic` golden; `parity::ballistic` |
| Static contact | reference | banded (bit-identical rows) | `bounce` golden; `parity::bounce` |
| Dynamic contact, Gauss-Seidel | reference | banded (bit-identical contact pairs) | `shower` golden; `parity::shower`, `contact_pair` |
| Drag (T0) | reference | banded | `quad_hover` golden; `parity::drag_componentwise` |
| Rotor element | reference | banded | `quad_hover` golden; `parity::quad_hover` |
| Dryden gust | reference | banded via `CORE-3` | `quad_hover` (moderate); `parity::medium` |
| IMU | reference | banded via `CORE-3` | `quad_hover`'s IMU; unit tests |
| GNSS | **not yet reference**: no golden scenario carries a receiver (one is ruled, `PHY-6`) | banded via `CORE-3` | unit tests; `parity::gnss_receiver`, `gnss_receiver_body` |
| Dynamic contact, Jacobi | not placed | not placed | built and unit-tested, not in the schedule |
| Rotor wake | best-effort, CPU only, not read by stepping (`PHY-3`) | — | `test_rotor_wake.cpp` |

A world's grade is its weakest module's (`L3`). Until the grade check exists (Core), this table is the declaration.

## Writing a physics test

- Derive the expected value from the model's equations, in double. A transcription of the implementation into the test agrees with the implementation's bugs.
- Pin the degenerate inputs (zero, negative, NaN, overflow) to the documented answer. Totality is a tested claim, not a comment.
- A GPU case is `Gpu*`-named, starts with `if (!compute::vulkan_available()) GTEST_SKIP()`, and is run on hardware before it is committed.
