# set_body_state: re-pose a live body

**Owner:** Core. **Status:** APPROVED by the lead 2026-10-13 (Q3 and Q4 settled below); Kat's lead has it, and Q1-Q2 are for Kat's Runtime. Implementation follows mass-from-mesh, unless Kat asks sooner.
**Base:** master `c3c1537`.
**Why:** Kat's training host re-poses bodies after a world reset without despawning them, so sensor bindings and refs survive (relayed by the lead, 2026-10-13). Runtime's contract is C5 1.2, `kathost_body_set_state`. **Decided** (the lead, 2026-10-13): the call takes the body row's native state, stores the quaternion as given, keeps every attached row and leaves rotors alone.

## 1. The call

```cpp
// Sets a live body's kinematic state at a step boundary. THE BODY ROW'S OWN FRAME:
// pos is the centre of mass (world, m), orient is body -> world (principal axes),
// vel is world (m/s), omega_body is BODY frame (rad/s) -- what body() returns and
// BodySpawn takes. NOT VehicleSpawn's design frame. Read the state with body().
[[nodiscard]] Result<void> set_body_state(BodyRef ref, glm::vec3 pos, glm::quat orient,
                                          glm::vec3 vel, glm::vec3 omega_body);
```

- **Frame.** BodySpawn's convention (`simulation.hpp:359-371`), and the layout's (`layout.hpp:112-134`, `layouts.slang:145-161`). The setter matches the getter. For a vehicle, the caller passes `ref.body` and reads `body()`, not `vehicle_state()`. Spawn converts VehicleSpawn's design origin through `design_to_principal` and `com_offset` (`simulation.hpp:335-347`, `simulation.cpp:2248-2249`). The setter does not.
- **`set_vehicle_state` (the design frame) waits until someone needs it.** It would convert through spawn's `to_body_state` (`design_frame.cpp:13-26`), so the two cannot disagree (TD-9).
- **The quaternion is stored as given.** Spawn normalizes (`simulation.cpp:1432`). The setter must not reuse that step, or `body()` would not return what was set. The tolerance is Kat's: refused unless `1 - 1e-6 <= |q| <= 1 + 1e-6`, with |q| = `std::sqrt(dot(q, q))` in fp32. `sqrt` is IEEE-exact and allowed (TD-3; `gnss.hpp:3-10`). Spelled so a NaN refuses. One substep runs on a q up to 2e-6 off unit length. Integrate renormalizes on the way out (`integrator.cpp:161-166`).

## 2. Contract

**When.** At any step boundary. `Simulation` is used from one thread (`simulation.hpp:105-107`). No code inside a step can reach it: a behavior gets only a `SubstepContext` (`behavior.hpp:64`). So every call between two `step()`s is at a boundary (`simulation.hpp:819-823`). `set_rotor_commands()` and `apply_wrench()` rely on the same fact and carry no flag (`simulation.cpp:2328-2440`). Kat's time-zero rule is Kat's. A mid-run set clears the body's contact history (§3), so it is not promised to be a no-op.

**Writes:** `pos`, `orient` (as given), `vel` and `omega_body`. Sets `specific_force` to 0, so it reads 0 until the next step, as a spawn's does (`simulation.cpp:1439`). Integrate rewrites it at the start of each substep, before the IMU pass reads it (`integrator.cpp:104`; `imu.hpp:44-53`).
**Keeps:** `mass`, `inv_inertia_diag`, `proxy_radius` and `flags`. Also `force_acc` and `torque_acc`. Those two hold Spade's applied wrench: `apply_wrench()` adds to them, and the next substep consumes them (`simulation.hpp:1155-1176`, `integrator.cpp:168-173`). Spade has no held wrench. A wrench applied before the set still acts. This differs from spawn, which zeroes both (`simulation.cpp:1437-1438`). At a boundary with no wrench pending, both are 0 either way.
**Keeps every attached row, untouched.** IMU and GNSS rows: stream, bias, phase and `last_index`. Their rings. Drag rows, rotor rows and any module's rows. Also the generation, the tick and the queue. It draws nothing, advances no stream, writes no sample and queues no op.
**Recomputes:** nothing. No derived per-body value exists (§3).
**No stored spawn state exists to protect.** The spawn record lives only in the queued op until the flush (`simulation.hpp:1659-1679`, `simulation.cpp:1397`). No registered array keeps a spawn pose.
**All or nothing.** Every check runs before the first write.

**Refusals, in this order:**

| Case | Code | Message |
|---|---|---|
| Poisoned | `invalid_argument` | `poisoned_` verbatim, "step: state invalid since tick N: ...", as `snapshot()` returns it (`simulation.cpp:2449-2451`) |
| Null, stale, foreign or despawn-queued ref | `not_found` | `validate_ref()`'s own messages (`simulation.cpp:1634-1669`) |
| A non-finite `pos`, `vel` or `omega_body` | `invalid_argument` | "set_body_state: pos/vel/omega_body must all be finite" |
| Orientation non-finite, or \|q\| outside 1 ± 1e-6 | `invalid_argument` | "set_body_state: orientation must be finite with \|q\| within 1 ± 1e-6 (got \|q\| = X)" |
| Body spawned but not yet flushed | `not_found` | "set_body_state: body is spawned but not yet flushed (call step() or flush_structural() first)". This is `apply_wrench()`'s rule: the queued init would overwrite the set (`simulation.cpp:2406-2412`) |
| Buried: `eval(sdf, pos)` < -r, r the body's effective proxy radius | `invalid_argument` | Spawn's message, prefixed "set_body_state: " (`simulation.cpp:1696-1705, 2076-2077`). Open question 1 |

**The poison check is new to mutators.** Today only `step()` and `snapshot()` refuse while poisoned (`simulation.cpp:1165-1167, 2449-2451`). `spawn`, `despawn`, `apply_wrench`, `set_rotor_commands` and `reseed` do not. This call is the first mutator to refuse, as the lead asked.

## 3. Per-body history, and contacts

The Markov property ("a body set to S evolves like a body spawned at S") needs every piece of per-body history to be cleared. The inventory:

| History | Where | Ruling |
|---|---|---|
| Last substep's specific force (the IMU's input) | `BodyState::specific_force` | Zeroed |
| Contact warm-start or manifold | None: "There is no manifold and no warm starting" (`contacts.hpp:67-70`). Both contact passes are pure (`contacts.hpp:32-36`, `grid.hpp:134-143`) | Nothing to clear today |
| Contact velocity change | `integrate.contact_dv` scratch. Integrate zeroes it every substep, for every slot (`integrator.cpp:20-29`; `module.hpp:378-386`) | Already 0 at every boundary |
| Broadphase | `GridScratch`, rebuilt every call (`grid.hpp:379-380`) | Nothing kept |
| Sleep counters, CCD state | None: no flag bits are assigned (`layout.hpp:93-95`), and detection is discrete (`contacts.hpp:82-93`) | Nothing |
| World-frame inertia, angular acceleration | Not stored: inertia is a body-frame diagonal, rebuilt per substep (`integrator.cpp:130-138`), and α is not state (`imu.hpp:87-98`) | Nothing |
| Rotor ω and command | `RotorRow::omega` and `omega_cmd` (`builtin_state.cpp:256-257`) | Kept. Rotors are not body state, and Kat flies bare bodies. ψ is not in the engine yet |

So the doc comment says: "clears the body's contact and solver history". Today that is `specific_force` alone. **A later per-body cache owes a clear here** (a warm-start, a manifold, a module's per-body history). That goes in a census beside `mark_vulkan_dirty()`'s (`simulation.hpp:1726-1779`). Test 2a catches a cache that is missed.

## 4. GPU mirror

The call ends with `mark_vulkan_dirty()`, like `apply_wrench()` (`simulation.cpp:2438`). The next `step()` uploads the whole walk (`simulation.cpp:1203-1213`; `compute/vulkan/backend.hpp:107-115`). It joins the census list (`simulation.hpp:1740-1773`), with a `VulkanDirtyTracking` case. No device state is kept per body across steps: the contact scratch is zero at boundaries.

## 5. Determinism, snapshot and replay

- **An input, like `set_rotor_commands()`.** A run is a function of (desc, dt, substeps, the API calls) (`simulation.hpp:73-74`). The engine keeps no journal. A replay applies the same calls from tick k (`simulation.hpp:89-94`), and the caller's input script carries them (`replay.hpp:213-228`). The call draws nothing, so it needs no seed or tick rule.
- **Snapshot.** The set lands in the registered body row, so the next `snapshot()` carries it. A `restore()` after a set returns the blob's state, as for any other write.
- **The scenario file** has two input kinds, `wrench` and `rotor_commands` (`scenario_file.hpp:76, 195`). A `body_state` kind is Test/Docs' to add when a corpus scenario needs one.

## 6. Tests (RED first; Runtime's conformance cases, Spade's half)

1. **Round trip (Kat 1).** Set S, then `body()` returns `pos`, `orient`, `vel` and `omega_body` bit for bit, including a q that is in tolerance but not normalized. `specific_force` is 0. `force_acc` and `torque_acc` hold a wrench applied before the set. Mass, inertia, proxy radius, flags, tick and the queue are unchanged.
2. **Markov replay, exact: the key test (Kat 2a), on CPU and Vulkan.** Original run: a bare body under a wrench applied every step, ω = 0. Record S = `body()` at tick k, and the body row at ticks k+1..k+N. Replay: the same desc, with the body spawned elsewhere, resting on the floor (it has contact history), stepped to tick k. Then set S, apply the same wrench each step, and `memcmp` the row against the original at every tick. On Vulkan the replay is compared with a Vulkan original (same-backend determinism, `simulation.hpp:1240-1242`), and `vulkan_upload_count()` rises by one.
3. **Replay with rotation (Kat 2b).** Test 2, with ω ≠ 0. Exact on our side, because `omega_body` is native. **Kat's bands are Kat-side only:** they come from Kat's own world-to-body conversion.
4. **Twin.** A body set to S and a `BodySpawn` at S evolve bit-identically over N steps, with no wrench pending. Precondition, asserted: `normalize(S.orient) == S.orient` bit for bit, because spawn normalizes and the setter does not.
5. **A no-op set at tick 0 (Kat 3).** A body with an IMU and a GNSS, flushed. Set it to its own `body()` state. `state_digest` equals a no-call twin's after the call and after N steps, sensors included.
6. **Bindings survive (Kat 4).** The IMU and GNSS rows, the rings, and the rotor and drag rows are byte-equal before and after the call, and `last_index` does not move. After N steps, the IMU's `noise`, `bias_a` and `bias_g`, and the GNSS `noise` and `bias`, equal an un-set twin's. Their draws do not depend on kinematics (12 normals per IMU sample, `imu.hpp:197-203`). The samples' values differ.
7. **Time (Kat 5: the rule is Kat-side only).** Our half: a set is legal at a mid-run boundary, and an unflushed body is refused with `not_found`.
8. **Bad arguments (Kat 6).** Every refusal in §2: NaN and Inf in each field; \|q\| just outside each bound, and 0; a null, stale, foreign or despawn-queued ref; a buried pose; poisoned (with `step()`'s text). After each, `state_digest` is unchanged.
9. **Restore after a set (Kat 7: the return to spawn is Kat-side only).** Our half: snapshot, set, step, restore, and the state equals the snapshot's. Also: a snapshot taken after a set restores the set state.
10. **CPU and GPU agree after a set**, within the parity bands (`engine/testing/parity.hpp`).
12. **The |q| edge (the lead, once Runtime confirms 1 ± 2e-6).** A quaternion Kat accepts, built in double at |q| = 1 + 0.99e-6 and converted to fp32 as kat_host_spade does, is accepted here; one at 1 ± 2.5e-6 is refused.
11. **Census (the lead's Q4 guard).** Every per-body array of the standard set is listed and classified "kept" or "cleared". An unlisted per-body array fails the test by name.

## 7. Open questions

| # | For | Question | Recommendation |
|---|---|---|---|
| 1 | Kat's Runtime | Refuse a pose that is buried in the world SDF, as spawn does? | Yes: the uncapped Baumgarte correction would eject it (`simulation.cpp:1696-1705`). **Kat's host agrees (Runtime to confirm):** kat_host_spade maps it to E_INVALID with nothing changed, as its spawn already does. |
| 2 | Kat's Runtime | The \|q\| bound is computed in fp32 here. Does Runtime's check use the same expression, so the two never disagree at the edge? | **Kat's host proposes (2026-10-13; Runtime to confirm):** Kat's double check at 1 ± 1e-6 is the binding one, and Spade's fp32 bound sits OUTSIDE it with a margin, 1 ± 2e-6, so no quaternion Kat accepts is refused here. Core agrees: the double-to-fp32 conversion moves \|q\| by far less than 1e-6. Adopt it when Runtime confirms. |
| 3 | Lead | Sensor reset (fresh bias walks), dropped for now (YAGNI)? | **Settled (the lead): dropped.** If ever needed, it zeroes bias and phase and keeps `last_index`, because TA5 forbids rewinding an index (`rings.hpp:15-17`) |
| 4 | Lead | A module that keeps per-body history: a per-array clear hook now, or a review line until one exists? | **Settled (the lead): a review line, plus a CENSUS TEST.** It lists every per-body array of the standard set, each classified "kept by set_body_state" or "cleared", so a new per-body array fails it until someone classifies it, like the dirty-tracking census. A developer module's per-body history is documented as the developer's responsibility. |
