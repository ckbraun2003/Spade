# The end-goal smoke's checks and bands (N20)

**Owner:** Test/Docs. **Status:** approved and merged by the lead, 2026-10-19 (fc1fbac). The lead decided the downwash band (§5): ±25%, with C8q approved. Amended the same day (the lead, with Interface): the band runs headless at 64³ through the editor's own run path, and the live 32³ chapter carries `downwash-present`.
**For:** the editor end-goal design's §6 steps (`docs/design/interface/plans/2026-10-19-editor-end-goal-design.md`, INT-10).
**Fixed a priori, 2026-10-19,** before any run of these steps. A band changes only by a recorded lead decision (`01-verification.md`).

## 0. Rules for every check

- **A predicate on state the step produced,** never "the step ran". Each check below says which failure it catches.
- **One implementation.** Each predicate is a display-free function. The tour step and a headless test both call it (TD-9). Proposed home: `sandbox/end_goal_checks.{hpp,cpp}`, written by Test/Docs on its branch and reviewed by Interface.
- **Red first.** Each predicate's test fails on a stub of the fault it exists to catch, then passes on a stub of the right state (§8). When the feature lands, the same predicate judges the real state.
- **Units:** SI. "Pick point" is where the click's ray met the surface. Times are simulated time.

## 1. `editor.library`

- **lists-index:** the panel's entries are the index's entries, in the index's order. For every entry, the name, category and file are equal. The counts are equal, and at least 9 (§3's first entries). *Catches:* a folder walk, a dropped entry, a reorder.
- **thumbnails:** every entry shows the image decoded from its own thumbnail file. The step decodes each file; the shown image must be the same size and differ from it nowhere ("differs" as in §6). The file's image must not be one colour: at least 1% of it differs from its dominant colour. *Catches:* missing or unread thumbnails shown as a placeholder, a framed placeholder, and another entry's image.

## 2. `editor.add_drone`

- **placed-at:**
  - The drone's x and z equal the pick point's, to 1 mm.
  - It **rests**: measured with the scene's SDF without the drone, the gap from its lowest posed-part point to the surface beneath lies within [−2, +5] mm. The pick march stops within 1e-3·t of the surface, and the smoke clicks from 5 m or closer.
  - After 1.0 s of Run, disarmed, it has moved no more than 10 mm and tilted no more than 2°.
  - *Catches:* a buried or floating drone, and a drone launched by interpenetration.
  - *Expected red:* `rest_height()` returns the built-in drone's 25 mm (`kDroneBodyHalf.y`) for every model. The reference quad's battery hangs lower, so this stays red until placing uses the model's own lowest point. The lead gave the fix to Interface (2026-10-19).
- **drawn:** one drawable per part of the model (`render_parts`), each at its posed part's transform (`posed_parts()`) to 1e-5 m and 1e-5 rad. The hull, arms and discs from `append_vehicle_items` don't count. *Catches:* the placeholder drawn instead of the parts; parts drawn at the design frame rather than the placed pose.
- **propulsion-kept:** the placed model's model, parts and propulsion identities equal the library file's. *Catches:* a copy that drops the chain (§2a).

## 3. `editor.add_objects`

- **object-added:**
  - Placing a library object adds exactly one asset, under a name no other asset has.
  - Its x and z equal the pick point's, to 1 mm.
  - It **rests on the picked surface,** whether that is the ground or another object. Measured with the scene's SDF without the object, the gap from its lowest collider point to the surface beneath lies within [−2, +5] mm.
  - It is **drawn**: the render scene gains one drawable per union primitive of the object's collider, plus one per CSG root (as `scene_from_world()` splits a collider), or one per mesh for an import. A gate (a torus and two posts) is three. The expected count is taken from the object's collider, not from `scene_from_world()`'s output (TD-4). Each drawable sits at the asset's pose to 1e-5 m, with the asset's material.
  - After 1.0 s of Run it has moved no more than 10 mm and tilted no more than 2°.
  - *Catches:* the tour's dropped builder checks `on-ground` and `rendered`; a buried, floating or doubled object; an object placed but not drawn.
- **import-named:** the imported object's name is the glTF mesh's name, or the file's stem when the mesh has none, and the source shown is the imported path. The `.spmesh` sidecar it wrote loads back with the vertex and triangle counts `read_gltf` read. *Catches:* an unnamed import; a sidecar not written or not readable.
- Known-open entry: `mesh-contact-open` (§6 of the design).

## 4. `editor.fly`

**What each frame records:** t; the height y and vertical velocity v_y of the centre of mass; the attitude; each rotor's ω and its static thrust k_T ω² along its axis n_i, as `drone_sim` reports `rotor_thrust`. m is the model's mass, g the scene's gravity.

- **armed:**
  - Before arming, every rotor's ω is 0.
  - Within 0.5 s of arming, every rotor turns at no less than 90% of its idle command, and above 0. It's judged at the last frame at or before 0.5 s after arming.
  - At idle the drone stays down: its height changes by no more than 5 mm over 1.0 s.
  - *Catches:* an arm that's ignored; idle above hover.
- **climbs:**
  - **Window:** W = 1.0 s, starting at t0, the first frame after throttle-up where the rotors' vertical thrust F_y = Σ k_T ω_i² (n_i·ŷ) reaches 1.10 m g or more AND the centre of mass has risen 2R above its height at the throttle-up.
    - The rotor model does apply ground effect (Cheeseman–Bennett, `rotor_ground_factor`, up to 1.25 and outside k_T ω²). After rising 2R, the reference quad's rotors are about 2.9R above the ground, where the factor is 1.008.
  - **Premise:** F_y ≥ 1.10 m g at every frame in W. Otherwise the step is mis-staged, and it fails with that reason, never passes.
  - **Prediction:** the drag-free double integral of the reported thrust, Δh_pred = v_y(t0)·W + Σ_k (F_y,k/m − g)(t0 + W − t_k) Δt_k. Under the premise, Δh_pred ≥ 0.49 m, so noise can't pass the check.
  - **Check:** 0.5 Δh_pred ≤ Δh ≤ 1.10 Δh_pred, and v_y > 0 at the window's end.
  - *Lower bound 0.5:* the prediction uses the static thrust, but the rotor model lowers thrust in climb. Its inflow closure λ(V_c / v_h), with λ(x + λ) = 1, gives λ ≈ 0.84 at 2 m/s.
    - Simulated for the reference quad (point mass, λ, ground effect and C_D A = 0.01 m²), the ratio Δh / Δh_pred from 2R up is 0.70 to 0.71 at F_y = 1.1, 1.2 and 1.4 m g. Drag at these speeds changes it by under 0.01.
    - So 0.5 leaves 0.2 for frame-rate integration and anything the point mass leaves out.
    - This holds for moderate climb speeds only, as simulated above. At full throttle the reference quad climbs at about 29 m/s, where drag is no longer small: N17's `editor.fly` scored 0.498 there, just under 0.5. Physics agrees that is physical, not a defect (the lead, 2026-10-22). The step is therefore staged as a 1.3 m g throttle step (Interface, N17, merged `4f25016`), which scores 0.696. A full-throttle climb below 0.5 is staged outside the premise and isn't a failure. Any later use at higher thrust needs the bound derived again first, with drag in it.
    - A pinned, paused or stuck drone, or thrust that never reaches the body, gives Δh ≈ 0.
  - *Upper bound 1.10:* out of ground effect, nothing in the simulation lifts beyond the rotors' static thrust, and inflow only lowers it. Missing gravity gives 11 × Δh_pred at the premise's minimum thrust, and thrust applied twice gives over 2×.
- **follows-attitude:**
  - After a pitch command θ_cmd (the step uses 15°, nose down), within 1.0 s the body's pitch is within 3° of θ_cmd, and it stays there for at least 0.5 s and through 1.5 s after the command (Interface's review: a drone that reaches the pitch and levels off fails).
  - Roll stays within 3° of 0.
  - By the end of those 1.5 s, the horizontal velocity along the nose's heading has gained more than 0.5 m/s since the command. It's a gain, not a speed, so a drone already moving can't pass on its old speed. At a 15° tilt, F_y ≈ m g gives g·tan 15° ≈ 2.6 m/s² forward, so 0.5 m/s is under a quarter of the drag-free value.
  - *Catches:* an attitude that's drawn but doesn't tilt the thrust; a sign error (nose down, flying backward); cross-coupling.

## 5. `editor.airflow`

LiveRun, drone held (design decision 1), until P2b.

**Two regions, one path** (the lead, 2026-10-19, with Interface). A developed wake at 64³ costs 4 to 5 minutes (Interface measured 261 ms per CPU step and 54 ms per publish), too long inside a tour step. So:
- **The chapter shows 32³ live** and checks `downwash-present` on screen.
- **`downwash`, the band, runs headless at 64³ through the editor's own run path:** the same EditorRun and region wiring the window uses, without the window, not bare LiveRun. That way a wiring bug in the editor can't hide behind a headless run of other code. The step names the headless check.
- **The headless check's tier:** release only, opt-in under `SPADE_AIRFLOW_VALIDATION=1` like C8. It runs per batch, and on any branch that touches the editor's airflow path.

- **region-resolved:** the grid (nx, ny, nz), dx and R/dx shown equal LiveRun's resolve for the same case, compared as numbers, not text.
  - Each check states the R/dx it needs: `downwash` needs **R/dx ≥ 4**, the readable-flow rule; `downwash-present` needs **R/dx ≥ 2**. Below that, the check fails as `airflow/under-resolved`.
  - The regions, both a 0.8 m box around the reference quad (R 76.2 mm): headless at 64³ (dx 12.5 mm, R/dx 6.1); live at 32³ (dx 25 mm, R/dx 3.05).
- **publishes:** every frame of the step publishes a new generation, `validate_flow_volume()` is Ok, and `no_data` is 0.
- **downwash** (headless, 64³) is the physical check. For each rotor i:
  - **One frame:** the rotors, the body and the published volume share the volume's world frame. LiveRun publishes centred on its box, with the held body at the origin. A probe point outside the volume's box is `absent:`, never read: the sampler holds the edge values past the box.
  - **Bad input:** a time that isn't finite and strictly increasing, or a state that isn't finite, is `absent:`. A case whose density, mass or gravity isn't positive is `mis-staged:`. A rotor making no thrust at the last frame is a judged fail.
  - **Where:** p_i = hub_i − 2R_i n_i. The hub and thrust axis come from the model's rotor rows posed by the held body (the rule `rotor_discs()` uses), taken from the model, not from LiveRun's disc rows. So a disc the coupling misplaces is sampled where the rotor really is.
  - **What:** u_i is the area-averaged downward axial velocity over a disc of radius 0.5 R_i, centred on p_i, normal to n_i. It's the mean of −n_i · v over every point of a square lattice of pitch 0.05 R_i inside that disc, each point sampled by render's `flow_sample` rule (cell-centred trilinear). A point with no data makes the check `absent:`, which never passes.
    - **Why an area, not the axis point** (Physics, 2026-10-19, before any run): on the exact axis the probe sits inside each rotor's hub vortex, where the constant-circulation swirl concentrates and the wake is least resolved and least like momentum theory. Rendering measured ω_y ≈ 230 to 330 1/s 0.5R below. The 0.5R disc averages that core out. It's still inside the contracted slipstream (radius about 0.75R at 2R), where the vortex-cylinder model's axial velocity is close to its axis value. It also tolerates the jets' inward lean as they merge (perhaps 0.2 to 0.3R at 2R). Physics checked the reference by a Biot–Savart sum over the semi-infinite uniform vortex cylinder: at 2R the axial velocity rises from the axis value to 1.0034 times it at 0.5R, so the 0.5R average is 1.0017 times the axis value. The reference stays v_h(1 + 2/√5). That's about 3 cells across at R/dx 6 and 5 at R/dx 10.
  - **Reference:** C8's momentum theory (`tests/data/airflow/README.md`).
    - v_ref,i = v_h,i (1 + 2/√5) = 1.894 v_h,i, where v_h,i = √(T_i / (2ρπR_i²)).
    - T_i = k_T ω_i², from LiveState's `rotor_omega`. That is the rigid-body path's thrust, independent of the disc conversion. ρ is the case's.
    - For the reference quad at hover thrust (0.526 kg, so T = 1.29 N per rotor), v_ref = 10.2 m/s.
  - **Premise:**
    - The controls are constant from arming through the window, and the attitude is level: tilted no more than 1°.
    - Σ T_i ≥ 0.5 m g. The rotors are loaded, so a near-zero reference can't let still air pass.
  - **Developed:** from t_dev = max(0.30 s, 4L / min_i v_ref,i) after arming, where L is the height of the hub plane above the box's bottom face. The check takes L from the frames: the highest hub above the published volume's floor, not a declared number.
    - In the 0.8 m box, L = 0.38 m, so t_dev = 0.30 s.
    - Reason: C8 (disc 0.96 m above its bottom face, v_ref 7.58 m/s) averaged from 0.32 s, which is 2.5 wake transits. Four transits, plus the 0.30 s floor, cover the rotors' spin-up and the starting vortex leaving the box.
  - **Window:** [t_dev, t_dev + 0.20 s], the mean over every published frame in it: 6 or more at 30 fps. Fewer fails, as an absence (TD-5).
  - **Band:** each rotor's mean within ±25% of its own v_ref: 0.75 ≤ ū_i / v̄_ref,i ≤ 1.25 (the lead's decision, 2026-10-19).
  - **This band is a coupling check, not a validation claim.** A pass shows the airflow is coupled to the rotors: air moves the right way, from the right places, at roughly the right speed. It does not show the solver's accuracy at this resolution. Only C8 and C8q make accuracy claims.
  - **Why not C8's 10% here:** C8's 10% was set for one isolated disc at R/dx 10, where it measured −2.2%. The chapter's region is at R/dx 4 to 6, with four rotors whose centres are 2.04R apart. Neither effect is measured yet, and a band set without evidence, then widened after a run, is what the a-priori rule forbids. ±25% still fails:
    - still air;
    - reversed flow;
    - a stopped or missing rotor;
    - a disc misplaced by more than its slipstream's radius (about 0.73R at 2R below);
    - a thrust off by ×2 or more, where the true bias is 10% or less.
  - **C8's band where it holds, as C8q (approved by the lead, 2026-10-19: an opt-in validation record, run in a quiet window):**
    - The setup: the reference quad, held at C8's R/dx 10 (128³ in a 0.975 m box), with h = dx/8 as in C8, still air, no floor. The measure, development rule and window are the same as above. It runs on `LiveRun`, the stepping core behind the editor's airflow chapter.
      - **Air's viscosity, not inviscid** (Physics, 2026-10-22, before any run). ν is the case's 1.46e-5 m²/s: the case refuses ν ≤ 0, and this is the editor's real value. The cell Reynolds number is about 5300, effectively inviscid, and C8's 10% is unchanged. The record states ν as a difference from C8, ruled immaterial before the run: viscous spreading reaches about 0.05R by 0.5 s, and the probe disc's edge sits about 0.23R inside the slipstream.
      - **The model's parts cleared: no solids** (Physics, 2026-10-22, before any run). This is what the editor's airflow chapter flies (`sandbox/airflow_live.hpp`), and it keeps C8's bare-disc premise. With the body in the grid, the 10% wouldn't hold as a validation claim:
        - the motor and arm shadow about 20% of each probe disc;
        - the body sits 0.66 dx under the inboard 11% of each disc;
        - with solids the pressure solve is MGPCG, not plan A's;
        - C8 at +6.0% leaves only 4% of headroom.

        The body-in case stays with `downwash` (±25%). One body-in run, report only and with no band, may run in a quiet window. A band ever set from it is a regression band (it catches change), never a validation band (01).
    - The band: each rotor within C8's 10%, and the four within 10% of their mean. The layout is symmetric under quarter turns, and the spin direction is second order on the axial velocity. The four are equal only statistically, though: each pair is a mirror image with its spin flipped, and an inviscid wake averaged over 0.20 s can scatter more than 5%. So the mutual band is 10%, not the 5% first proposed (Physics, 2026-10-19, before any run).
    - It appends a validation record to `tests/data/airflow/records/c8q-reference-quad.md`. The record also reports the observed mutual spread, (max − min) / mean of the four rotors' means, as a number. Any later tightening is then set in advance from evidence: for example, a run at a 0.5 s window with its band fixed before it runs (the lead, 2026-10-19).
    - **Its verdict is an ensemble's** (the lead, 2026-10-22, before its first run; 01-verification, "A case judged past its horizon"; `2026-10-22-c8-spread.md`). The window lies past the wake's horizon `T_p`, so one run is one trajectory, and on C8 a 1-ulp nudge moved a one-run verdict across the band.
      - Four runs that differ only by a nudge in the test, as C8's: run 0 as it is, and three with 1 ulp up on 110 of the discs' increments, each from its own seed, all fixed before the first run. The nudge goes in through `LiveRun::set_test_increment_nudge()`, a test-only hook that is never saved and leaves an unset run the same to the bit (the lead's grant, on Interface's review, 2026-10-22).
      - Four, not C8's eight, because each run is about 35 minutes. On C8 the 0.5R disc's spread across 16 runs was an sd of 0.38% (`2026-10-22-c8-spread.md`), against a 10% band.
      - Both bands must hold on the ensemble's mean (each rotor's mean over the four runs) and on the worst run. The record reports each rotor's spread across the runs.
    - It's heavy: 4 times C8's cells and 1.4 times its steps, so about 5.5 times C8's 6-minute run, roughly 35 minutes in release, per run. That is an estimate: two steps are timed in one short job before the ensemble's job is sized (the lead, 2026-10-22). Timed: 1.77 s a step, so about 16 minutes for a run's 526 steps. A stopped job is re-queued whole. A run resumes from its save only where the job itself resumes from one, as D1's film does; C8q's job doesn't (the lead, 2026-10-22).
    - Falling outside 10% is a finding for Physics, not a change to the band.
- **downwash-present** (on screen, 32³), the cheap a-priori check on the live view. For each rotor i:
  - The point, measure, reference, premise, development and window are `downwash`'s, at the live region's R/dx, which must be 2 or more.
  - **Check:** each rotor's window mean ū_i ≥ 0.25 v̄_ref,i, from 6 or more published frames. It's a floor, not a band: it proves direction and presence. The magnitude is the headless `downwash`'s to judge.
  - **Fails:**
    - still air: about 0, where the floor is about 2.5 m/s at hover thrust;
    - a reversed rotor, whose air goes up (ū_i < 0);
    - a stopped rotor, which has only its neighbours' entrainment on its axis.
  - **Why 0.25 holds at R/dx 2 to 3:**
    - The slipstream's momentum flux is fixed by the thrust, so its speed falls as one over the square root of its area.
    - At R/dx 2, the disc's 2dx smearing widens the slipstream at 2R from 0.73R to about 1.7R. That's 5.4 times the area, so the speed falls to about 0.43 v_ref, still above 0.25.
    - Reaching 0.25 would take 16 times the area: a radius of 2.9R, wider than the 2.04R between rotor centres.
- **run-line-shown:** the line shown equals `LiveRun::run_line()` for the frame shown, exactly.
- Known-open entries: `follow-needs-p2b` and `cp-needs-p2`.

## 6. `editor.views`

"Differs" means at least 1% of the view's pixels differ by more than 8/255 in some channel.

- **each-mode-drawn:**
  - **A kind `supports()` accepts:** `apply_view` applied this kind and `draw()` is Ok, and `unhonoured()` is empty. The view differs from its own background, the same view and mode with the scene's items hidden, so it isn't blank. One colour wouldn't do: the editor's background is a sky gradient and a grid. It also differs from the previous mode's frame, so the switch took effect.
  - **The premise, a still view:** the mode grabbed a second time, with no request between, changes by at most 0.25% of the view's pixels, a quarter of "differs". Past it, a switch that changed nothing could pass on the view's own motion, so the check is `mis-staged:`. On the first wiring, a dropped request still differed by 0.3126%: the HUD's stats box was inside the view.
  - **A kind `supports()` refuses:** the text shown is its `why_not`, verbatim.
  - *Catches:* a blank view; a switch that changed nothing; a refusal drawn as an empty view.
- **legend-shown:** `apply_view` returns one legend per layer of the view's spec, and fewer is `absent:`. Each legend shown equals the one returned: quantity, unit, colour map, range and whether the range came from the frame. While the region runs, every flow layer's range must come from the frame, since a fixed range reads the same over still air; a fixed one is `mis-staged:`. Its min must also be below its max, and a speed legend's max at least 1 m/s, so still air fails here as well.
- **two-views:** after Split, there are two views with different `ViewSpec`s. Each half differs from its own background (that view with the items hidden), and the two halves differ from each other.

## 7. `editor.save_reopen`

- **round-trip:**
  - The reopened scene, emitted again, is byte-equal to the saved file.
  - The scene identity, the region component and the drone's model, parts and propulsion identities are all equal.
  - Behaviourally: 60 steps of Run from the reopened scene give bit-identical body states to 60 steps from the scene before saving. "Body state" means every named field of each body, only the padding left out, so a reader that drops a mass or an inertia fails even at rest. A different body count is a judged fail.
  - The identities are required by name: the scene, the region, and each drone's model, parts and propulsion. One left out is `absent:`.
  - *Catches:* a field written but not read back, and one read back but not applied.

## 7a. Measuring a placed object

Test/Docs owns these, at Interface's request, so a step never measures what its own check judges (TD-4). The step hands over raw inputs: the collider, the world's SDF without the object, and the posed drawables.
- **The drawables declared** (`declared_drawables`): counted from the collider's postfix program itself, not through the renderer's `split_program()`. A primitive under nothing but union is one drawable. A subtree rooted at subtract, intersect or smooth_union is one, whatever it holds. A malformed program declares 0, which fails.
- **The rest gap** (`rest_gap`): the least distance, by the world's SDF without the object, over every vertex of the object's posed drawables. For the drone, those are its parts. Negative means buried that deep. It's NaN, which fails, with no points, with a non-finite point or distance, or when a draw item names a mesh the scene lacks. This is how §2's and §3's "lowest point to the surface beneath" is measured.
  - **Its bias, and so its preconditions** (Interface's review). Vertices lie on the surface, so the gap reads high, never low, by however far the true lowest point lies between vertices.
    - It's exact for the shipped primitives posed upright (yawed only): a sphere's pole, a box's or cylinder's bottom face and rim, and a torus's underside are all vertices.
    - So the end-goal step places objects upright, with nothing else within 5 mm sideways. A least distance is not "beneath": a wall that close would read as the rest.
    - It doesn't measure a tilted primitive: a 0.5 m sphere tilted 15° reads up to about 8 mm high, past the +5 mm band.
    - It doesn't measure a CSG object either: its surface-nets mesh sits up to half a mesh cell inside the surface.

## 8. Headless tests, red first

These go on a Test/Docs branch, against stubs, before each feature lands. Batch 1 (`test-docs/end-goal-checks`) holds downwash, climbs, follows-attitude, armed, placed-at, drawn and object-added. Batch 2 holds the views, library, import, region bookkeeping and round-trip; round-trip is judged on real state only at M2. Each later runs on the real feature's state, by milestone:
- **M1:** library, add_drone, fly, raster views.
- **M2:** airflow, save_reopen.
- **M3:** add_objects.

| Predicate | Red stubs | Green stub |
|---|---|---|
| downwash | still air; reversed flow; the field shifted by R; one rotor's jet missing; Σ T below 0.5 m g (named); R/dx below 4 (named); fewer than 6 frames in the window (named) | a top-hat jet of radius 0.73R at v_ref on each axis, from 2R above to the box's floor |
| downwash-present | still air; one rotor reversed; one rotor stopped; R/dx below 2 (named) | the same jets at 0.43 v_ref and radius 1.7R, the R/dx 2 estimate |
| climbs | a pinned history (Δh = 0); gravity missing; thrust applied twice; the premise unmet (named); the frames ending inside the window (named) | drag-free kinematics of the reported thrust; the same at 0.7 of it, the inflow closure's loss |
| follows-attitude | a tilt with no acceleration; a reversed acceleration; 5° of roll cross-coupled | a tilt reached in 0.4 s, with the forward speed rising |
| object-added | buried 1 cm; floating 1 cm; two drawables for one SDF object; a drawable off the pose | resting at 0 gap, one drawable at the pose |
| round-trip | an emitter that drops the region; a reader that ignores the propulsion block | an exact copy |

## 9. Asked of others

- **Interface:** agree on the predicates' home. Each end-goal step exposes the per-frame observables above (body state, rotor ω and thrust, the pick point and the posed parts) to the predicate.
- **Physics:** confirm that k_T ω² is the right T for momentum theory here (P7a's static value, exact in hover), and review the downwash reasoning in §5.
- **The lead:** decided, 2026-10-19: ±25% on screen, as a coupling check; C8q at C8's 10% for validation.
