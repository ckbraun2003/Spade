# Spade — pivot audit

> Against `design-specs/pivot-audit-brief.md` (`d56da212`, amended `70e5a4dd`, `248a9f63`).
> **My side of every seam only.** Where a joint is named below it is dated and quoted from
> Spade's side and handed to the Overseer unassembled.
>
> **Every count in this file names its predicate and its population.** Every command is
> re-runnable from the repo root at `79e0f73e`.

---

## The one-line finding

**Spade's *engine* is the least racing-shaped thing in the estate, and Spade's *boundary* is the
most.** The physics, the SDF world format and the render vocabulary are genuinely
domain-agnostic — a gate is `torus + 2 boxes`, an instance of the vocabulary and not a primitive
in it. What encodes racing is one layer up: the **vehicle model**, which names its actuators
`rotors` and its sensors `imu`, and has no other word for either. And what encodes a *settled*
answer is the boundary: a repo topology that was false three days after it was written and is
still LOCKED in the ruler.

---

## Exhibit 1 — "Spade stays in its own repo, consumed as a pinned, versioned dependency"

| | |
|---|---|
| **EXHIBIT** | `design-specs/archive/spec-corpus-v1/kat-platform-foundational-design.html`, the Spade boundary |
| **BUILT WHEN** | ~2026-08-05. **LOCKED** row, quoted: *"Spade — **Stays a separate general-purpose engine**, consumed behind a sim-host interface. Drone-specific sim (6DOF, sensors) lives in a Kat-side module built on Spade."* (`:323`) and *"One Kat monorepo — … **Spade stays in its own repo**, consumed as a pinned, versioned dependency."* (`:376`) |
| **RACING TEST** | Not racing-shaped. This one is entirely the second defect. |
| **SETTLED TEST** | **FAILS.** It is not merely a stale fact — *pinned, versioned dependency* is a claim that Spade's interface is **stable enough to version against**, which is the assumption the monorepo exists to avoid while the platform is young. The ruler says so in the same sentence about every other artifact and then exempts Spade from its own reasoning. |
| **TRUE NOW** | `spade/` entered the monorepo at **`47844424`, 2026-08-08** — *three days* after the ruler was written. There is no `spade/.git`. 257 tracked files. |
| **VERDICT** | **DELETE** — it served a design we no longer hold. |
| **AMENDMENT** | Strike the external-repo claim from all **six** sites in the ruler and rewrite the `LOCKED` Spade row to the rule that actually governs: *in-monorepo, hardened in place, with a provable standalone identity and a stated extraction trigger.* This **overrides a `LOCKED` row**, which is why it goes to the user rather than being taken here. |

**Requirement 5 — the count, with predicate and population.** Population: every line of
`design-specs/archive/spec-corpus-v1/kat-platform-foundational-design.html`. Predicate: asserts Spade is external to the
Kat repository. **Count: 6** — `:318` (*"plus Spade as external engine"*), `:323` (the LOCKED
row), `:350` (the architecture diagram node, literally `SPADE["Spade (external repo)…"]`), `:376`
(*"its own repo, consumed as a pinned, versioned dependency"*), `:502` (*"Spade maturation roadmap
(in Spade's repo, driven by Kat's needs)"*), `:583` (*"sequencing … happens in Spade's own
planning"*).

```
grep -n "Spade\|spade" design-specs/archive/spec-corpus-v1/kat-platform-foundational-design.html
```

**Requirement 3 — two independent derivations, and what the second could have contradicted.**
First: the ruler's own text. Second: **git**, `git log --diff-filter=A -- spade/ | tail -3`.
The second source *could* have said `spade/` was added recently as a vendored copy of a genuinely
external repository, or that a submodule pointer had been flattened — either would have made the
ruler's sentence a description of a topology that was once true. It says neither: the first commit
is a from-scratch scaffold, `47844424 2026-08-08 build(spade): v2 scaffold — presets, engine tree,
core ids/error/time, gtest wiring`. **The sentence was never true of this repository.**

**Requirement 4 — consequence chain. Four dependents, all still standing:**

1. **`spade/tests/` is unreachable from every root build** — Exhibit 2, correct under this rule.
2. **The `spade` ctest label and D11's budget carve-out** — `design-specs/archive/phases/kat-editor-workspace-phase.md:2049`: *"D11 gives spade its own workflow outside the T0–T3 vocabulary"*. Exhibit 2.
3. **"Never edit or police it" propagated as "a separate repository"** into at least two registers — `docs/superpowers/plans/2026-09-07-editor-design-corpus-consolidation.md:32` (*"**`spade/` is a separate repository.** Never edit or police it."*) and `design-specs/kat-active-work.md:320`. The narrowing is legitimate; **its stated reason is false**, and a rule whose reason is false is one someone will correctly discard.
4. **The sim-host interface exists to hold a version boundary** that has no versions on the other side of it.

**Requirement 6 — counterfactual.** With a second vertical at design time the boundary would have
been drawn around *capability* rather than *repository* — "Spade supplies rigid-body dynamics,
collision and render; the drone-specific module supplies airframes, actuators and sensors" — and
that division survives the topology change unharmed. It is in fact the division the code already
has. **The ruler drew the right line and then justified it with the wrong fact.**

**Requirement 9 — disconfirmed expectation.** I expected the claim to appear **once or twice**, as
a sentence in a repo-layout paragraph. Measured **6**, including a node in the architecture
*diagram*. Prediction recorded before running the grep above. A claim that appears six times in
one artifact is not a stale sentence, it is a **design premise**, and that changes the verdict from
"fix a line" to "override a LOCKED row".

**Requirement 7 — unknown, specific to this exhibit.** I cannot determine **whether the external-
repo premise was ever acted on** — whether a `spade` repository was created and abandoned, or
whether the sentence was aspirational from the first day. `git log` covers this repository only.
It would take the user's own recollection or a GitHub account listing. This matters because it
separates *"a plan we reversed"* from *"a plan we never started"*, and only the first deserves an
extraction trigger in the amended row.

---

## Exhibit 2 — 872 tests labelled `T0` that no gate can reach

| | |
|---|---|
| **EXHIBIT** | `spade/tests/CMakeLists.txt` (the severability header and the `LABELS "spade;T0"` block, `:479`–`:560`) |
| **BUILT WHEN** | Under Exhibit 1's premise. The header still states the mechanism correctly and says *"DO NOT 'FIX' THIS BY ADDING add_subdirectory(spade) TO THE ROOT."* |
| **RACING TEST** | Passes — nothing here is racing-shaped. |
| **SETTLED TEST** | **FAILS**, in the sharpest available form: the exemption's *remedy* was deleted and the *exemption* stayed. |
| **TRUE NOW** | The 872 tests carry **both** labels. My run at `79e0f73e`: `T0 = 311.08 sec*proc (872 tests)` and `spade = 311.08 sec*proc (872 tests)` — the same population. No root build reaches them (`grep -n "add_subdirectory(spade" CMakeLists.txt` → 0 hits; the only mention is a comment that `dronesim/CMakeLists.txt` adds it under an option). `.github/workflows/` is **absent**. The only gate is `docker/run-ci-linux.sh:171`–`182`, and **nothing triggers it.** |
| **VERDICT** | **OPEN** — the separation is correct and must stay; what is sealed is that nothing can observe its absence. |
| **AMENDMENT** | Two named changes, both mine. **(a)** `spade/tests/CMakeLists.txt` stops emitting `T0` on tests no `T0` sweep can reach — either drop the tier label or register `spade` as a declared tier population, and the choice is Test/Docs' because `tests/tiers.json` is theirs. **(b)** `docker/run-ci-linux.sh`'s spade-linux section gets `CMAKE_BUILD_PARALLEL_LEVEL` pinned; it currently runs a bare `cmake --build spade/build-ci`, which is ninja's all-cores default — **the exact value `spade/scripts/build.ps1` refuses at its parameter binder.** |

**Requirement 5 — the count, with predicate and population.** Population: tests registered by
`spade/tests/CMakeLists.txt` and discovered into `spade/build-gui`. Predicate: carries ctest label
`T0`. **Count: 872, equal to the `spade` population exactly.**

```
ctest --test-dir spade/build-gui -L spade      # Label Time Summary prints both
```

**Requirement 3 — two derivations, and the contradiction available.** First: the CMake source,
which sets `LABELS "spade;T0"`. That source alone is **not sufficient** — the same file documents,
at length, that `PROPERTIES LABELS "spade;T0"` is *silently flattened* by
`GoogleTestAddTests.cmake` into `LABELS spade T0`, where `T0` dangles unpaired and is dropped. So
the second source is the **running ctest's own Label Time Summary**, which could have shown
`spade = 872` and **no `T0` row at all** — the outcome the source file predicts for the un-worked-
around case. It shows both at 872: the workaround holds. *This is the one exhibit where reading
only the code would have produced the opposite answer.*

**⚠ Ordering, stated because requirement 9 turns on it.** I ran that ctest at 02:10, before I had
read D11 or this brief. So my "expectation" that spade sat outside the tier vocabulary was formed
**after** the measurement that refutes it. **It is therefore a contradiction between two sources
(requirement 3), not a pre-registered disconfirmation, and I am not counting it as one.**

**Requirement 4 — consequence chain.** D11 exempted spade's 342.3 s leg from `budget_test_s` on
the grounds that it *"gives spade its own workflow"*. That workflow was `.github/workflows/
spade.yml`, deleted at `9b5fa453`. So: the budget is unmechanised (`tiers.json` declares
`budget_test_s`; no code reads it), the exemption from it is mechanised only by unreachability,
and `tiers.json:71`'s own `overrun_action` names a policy that also has no mechanism. **The budget
is a wish, the policy written to stop it being one is a wish, and the carve-out from both now
rests on a file that does not exist.**

**Requirement 6 — counterfactual.** With a second vertical there would be *more* engine surface
and the same gate, so this gets worse with scale rather than better: the unreachable suite is the
one that grows.

**Requirement 7 — unknown, specific to this exhibit.** I cannot determine **what a root
`ctest -L T0` actually reports today** — whether it silently excludes these 872 or fails to
enumerate. That needs a configured root tree and a run, which is a box, and the box is Runtime's.
Until then I state only what `-L spade` measured in my own tree.

---

## Exhibit 3 — the vehicle model names its actuators `rotors`

| | |
|---|---|
| **EXHIBIT** | `spade/engine/vehicles/model_type.hpp:103`–`107`; `spade/engine/sim/simulation.hpp:246`–`258` (`VehicleRef`) |
| **BUILT WHEN** | Quoted from the source: *"covers quad/hexa/octo; four IMUs covers a redundant triad plus a spare."* |
| **RACING TEST** | **DESCRIPTION.** `VehicleRef` has `rotor_count` + `rotor_slots[]` and `imu_count` + `imu_sensors[]`. There is no `actuators` and no `sensors` — the *kind* is welded into the member name. A fixed-wing (control surfaces), a VTOL (both), a tethered inspection platform or a ground rover cannot be expressed. |
| **SETTLED TEST** | **FAILS.** Fixed inline arrays, and the source says the cost is deliberate: *"THE SIZE THAT COSTS: kMaxModelRotors + kMaxModelImuMounts slots, inline, so a VehicleRef is around eighty bytes."* Adding a third actuator kind is not a new value, it is a new member and a new size. |
| **TRUE NOW** | `kMaxModelRotors = 8`, `kMaxModelDragBodies = 4`, `kMaxModelImuMounts = 4`. |
| **VERDICT** | **GENERALISE.** |
| **AMENDMENT** | `VehicleRef`'s two typed arrays become one actuator table and one sensor table, each row carrying a kind tag; `rotor` becomes the first kind and the rotor path keeps its O(1) array write by construction (the slots are already carried for exactly that reason). This overrides nothing signed — **the bound's rationale is a comment, not a ruling**, which is itself the finding: *an 80-byte budget is doing the work of a design decision nobody made.* |

**Requirement 9 — disconfirmed expectation, pre-registered.** Before running anything I predicted
**`kMaxModelRotors = 4`**, a quadrotor, and said so. Measured **8**, with the rationale *"covers
quad/hexa/octo"*.

```
grep -n "kMaxModelRotors\|kMaxModelImuMounts" spade/engine/vehicles/model_type.hpp
```

**This weakens my own exhibit and I am keeping it in.** Whoever wrote that bound *did* generalise
— deliberately, one axis, with a stated reason. The finding is not "they assumed a quad". It is
that **the generalisation ran to the edge of the vertical and stopped there**: `4 → 8` crosses
quad to octo and does not cross multirotor to anything else. A bound that anticipates hexa and
octo is evidence the author was thinking about variation, which makes the *absence* of a non-rotor
actuator a stronger signal, not a weaker one.

**Requirement 3 — two derivations, and the contradiction available.** First: `model_type.hpp`'s
constants. Second: `VehicleRef` in `simulation.hpp`, a different file with a different job.
It could have contradicted the first by carrying a generic `elements[]` with the rotor bound used
only as a capacity hint — in which case the naming would be local to the model type and the
runtime structure would already be general. It does not: `rotor_slots` and `imu_sensors` are
separate, differently-typed members. **Two files, one vocabulary, no escape hatch.**

**Requirement 6 — counterfactual.** With a delivery or inspection vertical in view, the third
thing anyone would have added is a **payload** — mass that changes mid-flight. Spade has
`kMaxModelDragBodies` and a mass on the body template, and nothing that says mass is time-varying.
A second vertical would have forced an actuator table on day one, because a winch is an actuator
that is not a rotor.

**Requirement 7 — unknown, specific to this exhibit.** I cannot determine **what raising a bound
actually costs at run time**. The comment points at *"vehicles/model_type.hpp for why the bounds
exist and what raising one costs"* and that is the file the comment is in — the pointer is
circular. Measuring it needs a benchmark leg against a modified bound, which is a box.

---

## Exhibit 4 — one sensor kind, against three the ruler promises

| | |
|---|---|
| **EXHIBIT** | `spade/engine/sensors/` — the whole directory |
| **BUILT WHEN** | The ruler, `:497`, quoted: *"Sim-host interface (editor-side): how Kat drives a simulation — load world, step(dt, seed), body state I/O, **sensor synthesis (IMU samples with noise models, camera frames, GPS fixes)**, spawn/despawn, headless mode."* |
| **RACING TEST** | **DESCRIPTION, by omission.** IMU-only is precisely the sensor suite a racing quadrotor needs, because pose comes from outside the vehicle. Every non-racing vertical is defined by the sensor it navigates with — GNSS for delivery, camera/lidar for inspection, barometer + magnetometer for anything outdoors and slow. |
| **SETTLED TEST** | **FAILS.** `capacities.sensors` in the world file allocates a count of a thing that has exactly one type. |
| **TRUE NOW** | **4 files, 1 kind.** `imu.cpp`, `imu.hpp`, `rings.hpp` (a ring buffer, not a sensor), `../shaders/kernels/sensor_imu.slang`. |
| **VERDICT** | **GENERALISE.** |
| **AMENDMENT** | The sensor seam grows a kind tag and a second implementation before it grows a third — and the second should be **GNSS, not camera**, because a GNSS fix is a pose sample with noise and dropout and needs no render path, so it tests the *polymorphism* without dragging in the renderer. Named, so it can be argued with: `sensors/gnss.{hpp,cpp}` + `PollResult<GnssFix>`, mirroring `PollResult<ImuSample>`. |

**Requirement 5 — the count, with predicate and population.** Population: tracked files under
`spade/engine/` whose path contains `sensor`. Predicate: implements a sensor model (excludes
`rings.hpp`, a transport). **Count: 1 kind across 3 implementing files.**

```
find spade/engine -path "*sensor*" -type f | sort
```

**Requirement 3 — two derivations, and the contradiction available.** First: the directory
listing. Second: the ruler's `:497`, an artifact written by a different author at a different
time for a different purpose. It could have contradicted the listing by promising IMU only — in
which case one sensor kind would be *completion*, not a gap. It promises three. **The gap is
against the estate's own stated intent, not against my opinion of what a simulator should have.**

**Requirement 4 — consequence chain.** `VehicleRef::imu_sensors` (Exhibit 3) is typed on it;
`capacities.sensors` in every world file allocates against it; `sensor_imu.slang` is a GPU kernel
named for the kind; and the ruler's own §8 sentence has stood unamended since ~2026-08-05 while
promising two things that do not exist.

**Requirement 9 — disconfirmed expectation, pre-registered.** I predicted **1** sensor kind (PR2)
and measured 1 — *confirmed, not disconfirmed*, and I am saying so rather than dressing it up. The
disconfirmation on this exhibit is the **second** source: I expected the ruler to promise IMU
alone, and it promises camera frames and GPS fixes as well. That turned this from an
incompleteness I would have shrugged at into an exhibit with a citable origin.

**Requirement 7 — unknown, specific to this exhibit.** I cannot determine **whether camera-frame
synthesis was ever attempted and removed.** The render path exists and could serve one; whether
anyone wired it is not visible in `spade/`'s history without a full-tree search I have not run.
It would take `git log -S "camera" -- spade/`, which is a cheap next step and belongs in the
amendment set rather than here as a guess.

---

## Exhibit 5 — a world file fixes its own allocation forever, and rejects every unknown key

| | |
|---|---|
| **EXHIBIT** | `spade/engine/world/world_file.{hpp,cpp}`; the canonical header in every `*.world.yaml` |
| **BUILT WHEN** | Quoted from the generated header: *"Per-world allocation. Decided once, here, and never grown at run time, so every one of these must be > 0."* and *"EVERY KEY BELOW IS REQUIRED, and an UNKNOWN key is an ERROR at every level rather than a warning."* |
| **RACING TEST** | **INSTANCE.** Nothing in the schema knows what a gate is. Geometry is SDF primitives + CSG; a gate is `torus + 2 boxes` (`gate.world.yaml`). This is the format working as intended. |
| **SETTLED TEST** | **FAILS, twice.** (a) `capacities: {bodies, force_elements, sensors, contacts}` is authored once and never grown — a delivery drone that spawns a payload, or a swarm that grows, cannot be expressed without editing the world. (b) An unknown key is a hard error at **every** level, and `world_version` is *"the one upgrade path"* — so every vertical-specific field is a global schema version bump affecting every file. |
| **TRUE NOW** | Both quotes are live in the current generated output; `world_version: 2` is what this build writes. |
| **VERDICT** | **OPEN** — the shape is right and it is sealed. |
| **AMENDMENT** | A namespaced extension area, `x-`-prefixed, admitted at every level and **round-tripped unchanged** by `world_to_yaml()`. The estate already has this convention — Components ships `x-kat-airframe` — so this is adopting a house rule, not inventing one. `capacities` separately gains a declared `growable: false` today, so that the decision is *visible* rather than implied by a comment, and a later `true` is a schema value rather than a redesign. |

**Requirement 3 — two derivations, and the contradiction available.** First: the schema header
text. Second: `maximal.world.yaml`, the fixture that exists to exercise every field. It could have
contradicted the header by carrying a vendor block, a passthrough map or an unrecognised key that
survives a round trip — which would have shown the strictness was documentation rather than
enforcement. It carries none. **Every key in the maximal fixture is a schema key.**

**Requirement 6 — counterfactual.** With a second vertical the strictness would likely still have
been chosen — it is a good rule, and its stated reason (*"a reader of this version must never
silently drop a field a later version added"*) is about evolution, not against it. What would have
been added alongside it is the escape hatch, because two verticals cannot both own the global
version number.

**Requirement 7 — unknown, specific to this exhibit.** I cannot determine **whether `capacities`
is enforced as a hard arena bound at run time or merely as a reservation** — i.e. whether a spawn
past `bodies` fails or grows. `world_set.hpp` mentions `kMaxSdfDepth` as a build-time bound, and
the failure mode for exceeding `capacities.bodies` is not visible in the files I read. It decides
whether the amendment is a schema change alone or a schema change plus an allocator change.

---

## Exhibit 6 — the canonical "everything the format can express" fixture is a race track

| | |
|---|---|
| **EXHIBIT** | `spade/tests/golden/worlds/maximal.world.yaml` |
| **BUILT WHEN** | Its own header: *"the complete, static description of one world"*, generated by `spade::world_to_yaml()`. |
| **RACING TEST** | **DESCRIPTION — and this is Exhibit 0's disease, reproduced one layer down.** The schema is general; the worked example under it is entirely racing: `visuals: ["mesh:track/gate_ring", "material:track/asphalt", "mesh:props/banner"]`, `props: [mesh:props/tower, mesh:props/flag]`, materials `asphalt` / `beacon` / `decal`. The brief says of the ruler: *"the thesis is written general and every illustration under it is racing."* **The same sentence is true of the world format, and the world format is mine.** |
| **SETTLED TEST** | Passes. Nothing here assumes the answer stops changing. |
| **TRUE NOW** | 7 world fixtures: `ballistic`, `bounce`, `gate`, `maximal`, `quad_hover`, `shower`, `two_world_isolation`. Racing-sense geometry: **1** (`gate`). Racing-sense *dressing* on the format-coverage fixture: all of it. |
| **VERDICT** | **GENERALISE.** |
| **AMENDMENT** | Re-dress `maximal.world.yaml`'s render-only refs to a vertical-neutral set, and add **one** second fixture from a different vertical — an inspection world: a heightfield terrain, a building-facade box stack, a landing pad, a non-zero `wind` and a non-standard `air_density`. It costs one fixture and it makes the format's generality *demonstrated* rather than asserted. The fixture is generated, so the change is to the generator's inputs, not a hand edit. |

**Requirement 9 — disconfirmed expectation, pre-registered.** Before measuring I predicted **under
15** tracked `spade/` files would match `racing|race|lap|track|gate`. **Disconfirmed: 59 for
`gate` alone.** But the disconfirmation was **my predicate's fault, not the code's**, and the
correction is the more useful finding:

```
POP=$(git ls-files spade/ | grep -v '^spade/legacy/' | grep -v '^spade/build' | grep -v '^spade/install')   # 257 files
echo "$POP" | tr '\n' '\0' | xargs -0 grep -ril  gate | wc -l    # 104  substring: propagate, aggregate, mitigate
echo "$POP" | tr '\n' '\0' | xargs -0 grep -riwl gate | wc -l    #  59  whole word
#   sense-split of those 59: 13 racing-sense, 46 gating-sense (build gate / test gate / gated)
```

**One word, three populations — 104, 59, 13 — and only the third answers the question I was
asking.** `race` matched *trace*; `lap` matched *overlap* and *elapsed* (whole-word `lap`: **0**,
so Spade has no lap counting at all). ⭐ **A substring count and a homograph count are two
different errors and the second survives fixing the first.** Whole-word matching promoted my
measurement from wrong to ambiguous; only naming the *sense* made it a finding.

**Requirement 3 — two derivations, and the contradiction available.** First: the vocabulary count
above. Second: the fixture corpus itself, read as geometry rather than as text. It could have
contradicted the count by showing racing structure in fixtures whose *names* are neutral —
`maximal` could have contained a lap gate sequence. It contains a format-coverage salad
(plane, sphere, box, cylinder, capsule, torus, heightfield, all four CSG ops) dressed in racing
*materials*. **The dressing is the whole finding: the geometry generalises and the labels do not.**

**Requirement 7 — unknown, specific to this exhibit.** I cannot determine **whether any consumer
resolves those `mesh:track/*` refs to real assets**, or whether they are strings nothing loads —
the header says *"the only rule is that each one names something"*, which is a syntactic rule.
If they resolve, re-dressing touches an asset pipeline; if not, it is a one-line fixture change.
That difference is the whole cost of the amendment.

---

## Exhibit 7 — SELF-AUDIT: the guard I wrote that enforces more than it says

| | |
|---|---|
| **EXHIBIT** | `SlangSpirv.EveryCompiledVariantIsScanned`, `spade/tests/test_slang_layouts.cpp` — and the note I added to it at `79e0f73e`, today |
| **BUILT WHEN** | 2026-09-20, during the Jacobi gather. The guard asserts two compiled variants of a module differ in **exactly one** SPIR-V word, the LocalSize x-operand. |
| **RACING TEST** | Passes — it is a codegen invariant, with no domain content at all. |
| **SETTLED TEST** | **Passes, and it is the interesting kind of pass.** It has no exception list, deliberately. It met the gather at 3 entry points and again at 2, and **the files were split rather than the number moved.** A guard that cannot be relaxed is the opposite of settled-shaped: it forces the design to move instead of the threshold. |
| **TRUE NOW** | Green at 872/872. The note landed today because the one-entry-point-per-module rule it enforces was nowhere written down. |
| **VERDICT** | **KEEP.** |
| **AMENDMENT** | *(required — no verdict without one, and this one is real.)* The guard asserts the **count** is 1; it should assert the **reason** — that every differing word is a LocalSize operand. Today, a Slang codegen change that made two variants differ in one *unrelated* word would pass silently, and a change that made a single entry point emit two LocalSize words would fail with a message blaming entry-point count. **The assertion and its explanation are currently held together by nothing but my comment.** Named change: compare the differing word's offset against the LocalSize instruction's operand offset, and keep `ASSERT_EQ(differing.size(), 1)` as the second line rather than the first. |

**Requirement 9 — disconfirmed expectation.** I expected this guard to be the *only* place the
one-entry-point rule was enforced. That is why I wrote the note. It is also **unverified**: I did
not grep for a second enforcer before asserting sole enforcement in a comment that now ships at
`79e0f73e`. Pre-registered before measuring: **no second enforcing site.**

```
git ls-files spade/ | grep -v '^spade/legacy/' | tr '
' ' '   | xargs -0 grep -rnil "entry point"                    # 33 files MENTION it
grep -n -i "entry.point" spade/cmake/SpadeSlang.cmake    # 1 hit, and it is prose
grep -rn -i "entry.point" spade/tests/*.cpp              # 5 hits outside my file
```

**Confirmed - and it took two corrections to get there.** 33 files was a *citation* count, not an
enforcement count: *a grep for a name finds the citations, not the thing named.* And of the 5 hits
in sibling test files, **every one uses "entry point" in the C++ sense - a public function you
call** (`test_determinism.cpp:2759`, *"every entry point taking a VehicleRef validates `body`"*) -
not the Slang `[shader("compute")]` sense my guard means. That is the **second homograph in this
audit**, after `gate`, and both were caught only by looking at what the matches said rather than
how many there were. My committed comment stands - but it stands on a check I ran *after* shipping
it, which is the wrong order and is the reason requirement 9 exists.

---

## What §10's row named that I did not find, and what it missed

**Named and confirmed:** the repo-topology boundary (Exhibit 1), `spade/tests/` unreachable from
every root build (Exhibit 2).

**Named, and NOT confirmed on my side of the seam: "worlds that are tracks."** The two files the
Overseer cited — `gate-corridor.world.yaml`, `circuit-track.world.yaml` — **are not in `spade/`.**
Spade's own world corpus is 7 fixtures, 1 of which is course-shaped, and the format underneath
them is general. Those two files live on the Kat side of the sim-host seam and are somebody else's
exhibit; I have not reached across to audit them. **What is true on my side is narrower and
different: the format generalises and its canonical illustration does not** (Exhibit 6).

**Missed by the row, found here:** Exhibits 3, 4, 5 and 7 — the vehicle model's actuator
vocabulary, the single sensor kind against three promised, the world schema's two seals, and the
guard. **Exhibit 3 is the one I would defend hardest**, because it is the only place in Spade
where a racing assumption is welded into a *type* rather than into a label, and it is the one the
starting scope did not name.

---

## Estate note, handed over rather than claimed

`.github/` survives with exactly one tracked file, `PULL_REQUEST_TEMPLATE.md`, in a repo whose
hosted CI was deleted at `9b5fa453` and which is under `PUSH-GATE`. That is not Spade's and I have
not touched it; it is recorded here because I measured it while confirming that
`.github/workflows/` is genuinely absent (it is — Exhibit 2).

---

## Addendum, 2026-09-21 — closing Exhibit 4's unknown, and correcting Exhibits 3 and 4

> **This section corrects the committed text of Exhibits 3 and 4 above, at `82ddfeff`, and it is
> written here rather than edited into them because `design-specs/pivot-seams.md` (`a042ef82`)
> already cites Exhibit 3 by that SHA.** *A later section that does not name what it supersedes
> joins it.* What follows names it.

**Exhibit 4 recorded an unknown:** *"whether camera-frame synthesis was ever attempted and
removed… it would take `git log -S "camera" -- spade/`."* Ran it. The answer is not the one the
unknown anticipated, and it makes Exhibits 3 and 4 **narrower and more serious at the same time.**

### What was measured

```
for s in CameraSample camera_sensor CameraSensor poll_camera; do
    git log --oneline -S "$s" -- spade/ | wc -l ;  done      # 0, 0, 0, 0
grep -c "^SPADE_COMPONENT_TRAIT" spade/engine/objects/component.hpp  # 10 -- SEE ERRATUM
grep -rn '#include.*objects/' spade/engine/sim/                      # 0 hits
grep -rn '#include.*sim/'     spade/engine/objects/                  # spawn_helpers.hpp:29
```

`git log -S "camera" -- spade/` returns **57** commits, which is the citation trap again — almost
all of them are the render/fly-through camera. The four *sensor-shaped* symbols return **0, 0, 0,
0: camera-frame synthesis was never attempted.**

### But a generic sensor vocabulary DOES exist, one layer up

`spade/engine/objects/component.hpp` declares **ten** component kinds — `ComponentTypeId` runs
`transform = 0` through `fluid = 9` — and three of them are exactly the abstraction Exhibits 3 and
4 asked for:

> ⛔ **ERRATUM, 2026-09-21, SAME DAY: THIS SAID *NINE* AND THE COUNT IS *TEN*.**
> The traits occupy lines **113–122**, not `114`–`122`, and `grep -c '^SPADE_COMPONENT_TRAIT'`
> returns **10**. The omitted kind is `TransformComponent`, **line 113** — the FIRST one, not a
> trailing one.
>
> ⭐⭐⭐ **THE RANGE AND THE COUNT AGREED WITH EACH OTHER, WHICH IS EXACTLY WHY NEITHER WAS
> CHECKED.** `114`–`122` really does contain precisely nine trait lines. The citation was
> INTERNALLY COHERENT and externally short by one, because the range began one line below the
> first trait. **A cross-check between two numbers taken from the same look can only confirm the
> look.** Worse, the code block above presents `# 9 component kinds` as if it were the adjacent
> command's output; that unanchored `grep -n` actually prints **12** lines (the `#define` at 106
> and the `#undef` at 124 on top of the ten), so the figure shown was never any run's answer.
>
> ⚠ **AND THE FIRST DRAFT OF THIS ERRATUM WAS ITSELF WRONG, IN TWO NUMBERS OF THREE** — it named
> `FluidComponent` at line 123 as the missing kind and put the unanchored count at 11, because I
> assumed "off by one" meant "stopped reading early" when it meant "started late." It never
> shipped: the numbers were re-run before the commit. **A CORRECTION IS A CLAIM AND EARNS NO
> DISCOUNT ON THE VERIFICATION IT DEMANDS OF WHAT IT CORRECTS.**
>
> ⚠ **The exhibit is unaffected in direction and slightly larger in size.** `TransformComponent`
> is not itself an unbacked kind, so the count changes and the argument does not.

```
SensorComponent        sensor
ForceElementComponent  force_element
CameraComponent        camera            <- component id 7
```

**So Exhibit 4's framing — "one sensor kind" — was measured in the wrong layer.** Spade has a
kind-tagged sensor component *and* a camera component already. What it does not have is a
`CameraSample`, and `PollResult<ImuSample>` remains the only poll type in `sim/`.

### And the dependency runs one way, which is the whole finding

```
sim/      includes nothing from objects/
objects/  includes sim/simulation.hpp  (spawn_helpers.hpp:29)
```

`objects/README.md` states the rule: *"Components hold handles into the SoA arenas `spade_state`
already owns; they never own simulation data themselves."*

⭐⭐⭐ **THE GENERAL VOCABULARY IS BUILT ON TOP OF THE WELDED ONE AND HOLDS ONLY HANDLES INTO IT, SO
IT CANNOT GENERALISE WHAT IT SITS ON.** A `CameraComponent` can name a camera that **no simulation
arena can ever produce a sample for** — component id 7 exists and `CameraSample` has never been
written. The kind tag is real, it is one layer too high, and it points downward at
`VehicleRef::imu_sensors[]`.

### What this changes

| | |
|---|---|
| **Exhibit 3** | **Verdict unchanged — `GENERALISE` — and the reason is stronger.** I wrote *"the only place in Spade where a vertical assumption sits in a TYPE rather than a label."* That still holds, and now it also explains why the layer above did not save it: `ForceElementComponent` is generic and `rotor_slots[]` is not, and the generic one holds handles into the typed arenas. **Generalising `objects/` would change nothing.** The arenas are the subject. |
| **Exhibit 4** | **Verdict unchanged — `GENERALISE` — but my AMENDMENT was wrong and is replaced.** I proposed adding `sensors/gnss.{hpp,cpp}` + `PollResult<GnssFix>` *"mirroring `PollResult<ImuSample>`"*. **Mirroring is the defect, not the fix:** a second hand-typed sensor path reproduces exactly the welding Exhibit 3 objects to, one file later. The replacement: give the sim layer's sensor arena the kind tag that `objects::SensorComponent` already has, so `poll()` dispatches on a tag rather than a type name — *then* GNSS is the first row rather than the second special case. |

### Requirement 9 — disconfirmed expectation, pre-registered

I predicted this probe would show camera synthesis **attempted and removed**, on the reasoning that
the render path exists and the ruler promised camera frames. **Measured: never attempted (0, 0, 0,
0) — and a camera COMPONENT present the whole time.** I had looked for a deleted implementation and
found a live declaration with nothing behind it, which is the more awkward of the two findings:
*nothing was lost; something was declared and never connected.*

### Requirement 7 — the unknown that replaces the one now closed

I cannot determine **whether `objects::SensorComponent` is reachable from any shipping path**, or
whether the whole `objects/` layer is the sandbox's authoring model with no runtime consumer. The
24th spec's own wording — *"the object model is designed in full and built minimally"* — allows
either. It decides whether the addendum's amendment is one change or two, and it needs a caller
trace I have not run. ⛔ **Recorded as unknown rather than assumed: this realm has three instances
of asserting a caller it had not traced, and this is exactly where the fourth would go.**

---

## Addendum 2, 2026-09-21 — Exhibit 5 answered: the fixtures are SURVIVORS, and the floor is a missing-field check wearing an allocation check's clothes

> **Corrects and sharpens Exhibit 5 above (`82ddfeff`).** Prompted by a question from the
> coordinator that Runtime's `E8` made askable: *does the total-constraint-invisible-because-total
> shape exist on Spade's side of `C5` — are the 7 world fixtures the POPULATION or the SURVIVORS?*
> **They are the survivors, and the tax they paid is measurable.**

### The constraint is real, total, and enforced on both paths

```
spade/engine/world/builder.cpp:281-290   "world capacity 'bodies' must be > 0"  (x4 fields)
spade/engine/world/world_file.cpp:1168   validate_world_desc(world)  -- the loader calls the
                                          SAME function, not a copy
spade/engine/sim/world_set.cpp:242       "every world capacity must be > 0"  -- a SECOND guard
                                          at instantiation
```

**Requirement 9 — disconfirmed expectation, pre-registered.** `parse_capacities`
(`world_file.cpp:717`) reads four `uint32_t` fields and checks **none** of them against zero, while
the generated header asserts *"every one of these must be > 0."* I predicted that made the header's
other claim — *"loading it runs exactly the validation build() runs"* — **false**, with the error
surfacing later and elsewhere. **Measured false.** `world_file.cpp:1166` says *"THE SAME VALIDATION
WorldBuilder::build() RUNS. Not a copy of it, not a subset chosen for a file — the same function"*
and it does call `validate_world_desc`. ⭐ *A parser that omits a check is not evidence of a missing
check; I read one function and inferred the absence of a caller I had not traced — which is the
fourth instance of the defect I named as a risk two commits ago, caught this time before it shipped.*

### What the floor costs, measured

Population: all 7 `spade/tests/golden/worlds/*.world.yaml`. Predicate: the declared value of each
`capacities` field.

| fixture | bodies | force_elements | sensors | contacts |
|---|---|---|---|---|
| ballistic | 4 | 4 | **1** | **1** |
| bounce | 4 | 2 | **1** | **1** |
| gate | 5 | **1** | **1** | **1** |
| maximal | 8 | 4 | 2 | 16 |
| quad_hover | 1 | 5 | **1** | **1** |
| shower | 128 | 4 | **1** | **1** |
| two_world_isolation | 16 | 4 | **1** | **1** |

⭐⭐⭐ **`1` IS THE FLOOR'S FINGERPRINT. Six of seven declare `sensors: 1`; six of seven declare
`contacts: 1`.** Where a world genuinely uses a resource the number varies and is interesting —
`shower` allocates 128 bodies, `maximal` 16 contacts, `quad_hover` 5 force elements. Where it does
not, the number is **exactly the smallest legal value**. `gate.world.yaml` declares `sensors: 1`
and contains **no sensor**: `spawns: []` and a static SDF. `ballistic.world.yaml` — a world whose
name is its physics — declares `contacts: 1`.

### The same species as Runtime's `E8`, and the same sentence fits

Runtime's `E8`: *"THE WORDS SAY 'EXACTLY ONE'. THE REASON JUSTIFIES ONLY 'NOT TWO'. THE CODE
IMPLEMENTS THE WORDS."*

**Mine: the words say `> 0`; the reason justifies only "NOT DEFAULTED."** `uint32_t` is 0 when a
field is unset, so `> 0` is a very good check for *the author forgot this field* — and it is
indistinguishable, at the call site, from a check for *this world legitimately needs none of these*.
The code implements the words. **And the cost is exactly Runtime's: it produces no broken artifact
to trip over, it produces an ABSENCE.** A world with no contacts (free-flight navigation, the
canonical non-racing sim), no force elements (vacuum ballistics, a pure integrator check) or no
sensors cannot be authored by any route. **None of them are in my 7 because none of them can exist,
and I originally read 7 fixtures as a corpus rather than as a filtered result.**

| | |
|---|---|
| **VERDICT** | **`OPEN` — unchanged, and now with a measured cost rather than an argued one.** |
| **AMENDMENT (replaces Exhibit 5's, which named only the `x-` extension area)** | Separate *unset* from *zero*. `capacities` fields become explicitly-defaulted and the floor becomes `>= 0` with a **presence** check instead of a **magnitude** check — the four fields are already `check_map`-required at `world_file.cpp:719`, so presence is enforced one line above where magnitude is enforced downstream, and the magnitude check is buying nothing that the required-key check does not already buy. Exhibit 5's `x-` extension area and `growable: false` stand alongside it. |

### Requirement 7 — the unknown this one leaves

I cannot determine **whether any arena allocates eagerly on capacity**, i.e. whether `bodies: 0`
would merely reserve nothing or would break an invariant downstream that assumes at least one slot.
`world_set.cpp:242`'s second guard suggests somebody believed it mattered there specifically. That
decides whether the amendment is a two-line predicate change or an allocator audit, and it needs a
read of the arena construction path I have not done.

---

## Ruling — `objects/` is the sandbox's authoring model, and that makes the Exhibit 4 amendment ONE change

Runtime traced reachability from their side of `C5` and found **zero** `objects::` or `objects/`
references in `dronesim/spade/`, with `SensorComponent` appearing outside `spade/` only in
`design-specs/*.md` and `AGENTS.md` — no code. **The negative is load-bearing because the host
includes engine headers by bare subtree path, so `<objects/component.hpp>` would have resolved.**
It is a choice, not a missing include path.

**Reachability was theirs to measure; INTENT is mine to rule, and their trace says nothing about
it.** The intent record is in my own tree, `spade/engine/objects/README.md`: *"The engine design
spec's §4 ratified this model and never built it"*, and the 24th spec's ④ — *"the object model is
designed in full and built minimally."*

**RULING: `objects/` is ratified-but-unconsumed — the sandbox's composition and identity layer,
not a runtime model in waiting.** It was never planned to replace `sim/`'s arenas; it was planned
to *name* them, which is what "components hold handles into the SoA arenas `spade_state` already
owns" says. So:

- **The Exhibit 4 replacement amendment is ONE change, not two:** kind-tag the `sim/` sensor arena.
  Nothing in `objects/` needs to move, and nothing downstream of it breaks, because there is
  nothing downstream of it.
- ⛔ **And the inverse, stated so nobody reads the ruling as a demotion:** this does **not** make
  `objects/` dead code to harvest. It is the ratified model for the layer it serves. What it is not
  is a reason to believe the welding in Exhibit 3 has a fix waiting upstairs.

---

## ADDENDUM 3 -- EX-4 WAS MEASURED IN `spade/engine/` AND STATED ABOUT THE PLATFORM (2026-09-21)

**Occasioned by `design-specs/capability-model.md` §7 (`395b718c`), which rules that ALL
capabilities are drone capabilities and the host is the SIMULATOR of them. That makes
*"can this host simulate this drone?"* a question with a measurable answer on my side.**

### What EX-4 said

> *"One sensor kind against three promised."* Measured with `-S CameraSample|CameraSensor|
> camera_sensor|poll_camera` over `spade/` -> **0, 0, 0, 0**, and that measurement is still correct.

### What is true at the seam

```
                  engine arena            host binding                     advertised by
                  (spade/engine/)         (dronesim/spade/host.cpp)        kathost_caps_query
cap.sensor.imu    ImuSensorRow            add_imu_sensor                   YES
cap.sensor.camera NONE                    CameraDesc, entry.camera         YES
cap.sensor.gnss   row as of 5daee4f5,     "gps" -> KATHOST_E_INVALID       NO, and honestly
                  arena is leg 2
```

⛔ **SO THE PLATFORM SIMULATES TWO OF THREE, NOT ONE OF THREE. `cap.sensor.camera` IS ADVERTISED AND
IT IS HONEST** -- `kathost_sensor_bind` accepts `"camera"` and constructs a host-layer camera over
the renderer (`dronesim/spade/sensors.{h,cpp}`). **A CAMERA SENSOR EXISTS. IT IS NOT IN THE ENGINE.**

⭐⭐⭐ **I MEASURED THE ENGINE AND STATED A CONCLUSION ABOUT THE SYSTEM.** The grep was right, the
scope was never written down, and the sentence it produced was about a layer the grep did not cover.
**This is the same defect as three others found the same day** -- a "14 files" figure that was 14
only for source files and 17 in fact, an `objects/` linkage answer that answered the linkage question
instead of the reachability one, and a `snapshot.cpp` grep whose zero hits meant nothing because the
serializer is registry-driven. ***FOUR INSTANCES, ONE DEFECT: A MEASUREMENT TAKEN IN ONE SCOPE AND
REPORTED IN ANOTHER.***

### ✅ AND IT CHANGES A LEG-2 PREMISE FROM A NECESSITY INTO A CHOICE

The camera proves host-layer sensing is a COMPLETE pattern, persistence included: `entry.camera->
restoreState(...)` and `SensorRosterSnapshot` (`host.cpp:937-947`, `host_internal.h:293`) carry its
state across a restore through the HOST's own snapshot, never Spade's registry. **There are two
persistence mechanisms, and only one of them is the registry walk.**

**So GNSS did not HAVE to be an engine arena.** It still should be, and the reason is specific rather
than architectural taste:

* its bias is **correlated simulation state on the CPU<->GPU parity band**, which compares BITS; the
  camera is not in that band at all
* an engine array **rides the registry walk**, so it is covered by the determinism digest corpus --
  the host roster is not
* the camera is derived from **render state the host already owns**; a receiver's bias is not derived
  from anything the host holds

⚠ **That is a design argument, not a measurement, and it is labelled as one.** What is measured is
that the alternative exists and works.

### 📌 WHAT THIS DOES NOT CHANGE

EX-4's DIRECTION stands: the ENGINE modelled one sensor kind, the kind tag was welded inside that
kind's header, and `5daee4f5`/`8f9eb2aa` are the amendment. **The exhibit was right about the engine
and wrong about the platform, and only the second sentence was ever written down.**

⚠ **ROUTED, NOT ACTED ON:** `dronesim/spade/host.cpp:2150-2157` states the gnss exclusion's reason --
*"no native Spade primitive; `kathost_sensor_bind` returns `KATHOST_E_INVALID` for it"*. **Leg 2
removes exactly that condition**, so that comment and the advertised list become stale the moment the
arena lands. `dronesim/` is not this realm's interior; the edit goes through the coordinator.
