# Spade -- The Object Graph, Components, and Behaviors

**What this is.** The consolidated specification of Spade's object model: object identity and
composition, the component type registry, the storage shape, serialization, and declared
behaviors. It is the document a reader consults before adding a component type, a field on
`Object`, or a behavior.

**What it harvests.** Sections 3 and 4 of the 24th spec (`superseded/kat-spade-library-sandbox.html`) --
rulings `SL3`, `SL4`, `SL5`, `SL6` -- and the as-built implementation that landed on master:
`spade/engine/objects/` (`README.md`, `object.hpp`, `component.hpp`, `graph.hpp`,
`serialize.hpp`, `behavior.hpp`, `behaviors/kinematic_mover.hpp`) plus the "Unreleased -- the
object model" section of `spade/CHANGELOG.md`. Unusually for this corpus the subject is both
specified and built, so the built code is a source with standing of its own.

**Series owned.** `SL3`, `SL4`, `SL5`, and the **registry half of `SL6`** (registration order, GPU
eligibility, determinism obligations). `SL6`'s *schedule* half -- which slots exist and why their
positions are a parity contract -- is owned by `02-engine.md`. `SL7`-`SL9` (the v1 transfer audit), `SL8` (SPH
fluid), `SL10`-`SL13` (the sandbox) and `SL14`-`SL18` (quarantine, build, vocabulary, phases,
verification) belong to sibling documents in this series; they are referenced here, not
restated.

**Status.** The 24th spec is **DRAFT** -- drafted 2026-08-30, sign-off still owed. Its **Plan A
is COMPLETE and MERGED to master.** Both statements are true simultaneously and neither softens
the other: the implementation shipped ahead of the signature.

---

## 0. Status of record

| Fact | State |
|---|---|
| 24th spec (`superseded/kat-spade-library-sandbox.html`) | The **document** remains a 2026-08-30 DRAFT and always will -- it is superseded and kept as a source. ✅ **The `SL1`-`SL18` RULING SERIES it carried is SIGNED 2026-09-17**, in its live homes. *A document and the rules it carried expire on different events.* |
| Plan A -- the object model (`SL3`-`SL6`) | **COMPLETE and MERGED to master.** `9ed536f6` is an ancestor of `master`. |
| Plan B -- SPH fluid (`SL8`) | **UNWRITTEN.** No plan document; no SPH pass exists in `spade/engine/physics/`. |
| Plan C -- the sandbox (`SL10`-`SL13`) | **WRITTEN 2026-09-17** (`plan-c-sandbox.md`), **not executed.** No `spade_sandbox` target exists. |
| `ComponentTypeId::fluid` | **DECLARED, NOT IMPLEMENTED.** Reserved so adding SPH later cannot force a renumber -- section 3. ⚠⚠ **A KEEP TODAY THAT PLAN C RECLASSIFIES.** It is declared-and-unread, which is correctly a keep -- but `kNames` (`graph.cpp`) carries all ten names and `component_type_id_from_name("fluid")` **resolves today**, so the moment an enumerator puts that table in front of a person it becomes a roster entry advertising a type the engine cannot construct. *Not "is it implemented" but "does anything assert it to someone who will act on it".* Plan C task C5 adds availability as a declared property beside `kNames` rather than filtering in the UI, because a UI filter leaves the next enumerator to repeat it. |

The determinism headline, stated first because it is the fact most worth confirming up front:
**`kSnapshotVersion` did not move and no golden file changed.** Every scenario digest, every
render golden and every CPU-GPU parity band is byte-identical to what it was before the object
model existed. Section 1 is why.

---

## 1. `SL3` -- the object graph is composition and identity, never state

**`SL3`**: the object graph never influences buffer layout, pass order, or any value the physics
computes. Objects and components may be created, reordered, renamed and destroyed freely,
because **slot assignment** -- which already existed and was already deterministic -- is what
the GPU sees. Pool iteration order feeds nothing.

It follows, and this is the operative half of the ruling, that **the object graph is not
registered state.** It is composition and identity, reconstructible from a description.
Components hold *handles into the SoA arenas `spade_state` already owns*; they never own
simulation data themselves.

### What it is NOT -- read this before adding a field

The test a reader applies before putting a field on `Object` or inside a component:

> Could this be rebuilt from a saved description, with no reference to how the simulation
> happens to be running right now?

Position and orientation qualify -- they are authored placement. A velocity, a contact set, or
an accumulated force does not: that is state, it lives in a `spade_state` arena, and a component
references the slot holding it.

### The consequences, all of which are the point

- `kSnapshotVersion` does not change, and the registry walk does not change -- it does not know
  `ObjectGraph` exists.
- Every existing CPU-GPU parity band and determinism test remains valid, unmodified.
- A snapshot taken before this model existed still restores after it.
- `Simulation` holds no `ObjectGraph` member at all. Its only object-model coupling is a
  borrowed `const objects::BehaviorRegistry*` (section 6).

That is why a structural addition this large cost the determinism estate nothing. It is not an
optimization; it is the architecture.

**Cost if wrong.** If a future component genuinely needs to own persistent state, it must be
registered and the snapshot version bumped -- a known, bounded, deliberate change rather than a
silent one.

### The claim is tested, not argued

`spade/tests/test_objects_determinism.cpp` is the proof, and two choices in it exist to
make it falsifiable rather than merely green. **The graph is churned *between* steps, not built
once before the run** -- a graph constructed beforehand and left alone proves almost nothing,
since `ObjectGraph` and `Simulation` share no member, so of course a `std::vector` filled in
advance does not change a float computed later. The failure worth excluding is a **coupling
through something global** (an allocator sequence, a shared RNG, a static cache), and that only
surfaces if graph mutation is interleaved with the ticks being compared. And **it runs the
committed scenario corpus**, through the same `scenario_from_yaml()` / `start_scenario()` /
`advance_scenario()` entry points the golden digests are produced through, so it cannot drift
from the corpus the way a transcribed scenario would. Both the 64-bit whole-set digest and the
full snapshot blob are compared.

The test header also records what it structurally **cannot** see: both runs happen in one
process, so a byte that is *indeterminate* rather than wrong -- padding inside a snapshotted
struct, say -- holds the same garbage in both and compares equal. A single-process comparison
can prove that adding the object graph changed nothing; it can never prove the bytes were
defined to begin with.

---

## 2. `SL4` -- the component model

**`SL4`** specifies the model in full:

- **Object** -- a generational handle plus a stable name, owned by an `ObjectGraph`. It has a
  transform and zero or more attached components. Objects may nest; nesting affects transform
  composition only and never pass order.
- **Component** -- a typed part attached to exactly one object. It holds configuration plus
  *references* to engine state slots; it does not hold simulation state. Type ids are
  **compile-time registered and monotonic**, never `typeid().hash_code()`, because the id is
  what a saved scene stores and a hash is neither stable across compilers nor meaningful across
  versions.
- **Lifecycle** -- attach and detach are structural changes.
- **Serialization** -- a graph serializes by component type plus configuration; slot references
  resolve at load, so a saved scene survives a change in slot assignment.

**The container is named `ObjectGraph`, not `Scene`, deliberately.** `spade::render::RenderScene`
already exists and means something different (the renderer's per-frame view of a world). Two
things called a scene, one namespace apart, is a prose trap even where C++ would disambiguate
them.

> **SPEC vs BUILT:** `SL4`'s lifecycle clause reads "attach and detach are *structural changes*
> and obey the engine's existing rule: queued, applied only at step boundaries, never mid-step."
> The built `ObjectGraph::attach` / `detach` mutate immediately; there is no queue. The **built
> code wins on what exists**; the **spec wins on what was intended**, and the intent is not yet
> contradicted, because no queue is reachable from a step: `Simulation` holds no `ObjectGraph`,
> so nothing inside a substep can observe a mid-step attach. The obligation becomes live the
> moment `SubstepContext` gains an `ObjectGraph` (see `kinematic_mover`, section 6), and must be
> discharged then.

> **SPEC vs BUILT:** `SL4` says a graph "serializes by component type **id**". The built
> serializer writes the registered **name** (section 5). The built form is the stronger one --
> readable, diffable, and the name-to-id mapping stays single-sourced -- and it preserves `SL4`'s
> actual requirement, which is that the key be registered rather than positional.

The content layer reached this vocabulary first: `content/recipes/*.json` already describes
scenes as instances of prefabs with poses, which is why the recipe format and the object model
map onto each other cleanly.

---

## 3. `SL5` -- the component types, and why the ids are frozen

**`SL5`**: the model above is specified in full, but **implementation is limited to the component
types the sandbox exercises.** `ComponentTypeId` is hand-assigned, monotonic from zero, and
dense. As built in `spade/engine/objects/component.hpp`:

| id | enumerator | struct | holds |
|---|---|---|---|
| 0 | `transform` | `TransformComponent` | nothing -- an empty marker; placement lives on `Object` |
| 1 | `body` | `BodyComponent` | `world_index`, `body_slot` into the bodies arena |
| 2 | `mesh` | `MeshComponent` | `draw_item` into the `RenderScene`'s draw items |
| 3 | `material` | `MaterialComponent` | `material_index` |
| 4 | `collider` | `ColliderComponent` | `sphere_radius` (0 == the world's default sphere proxy) |
| 5 | `sensor` | `SensorComponent` | `sensor_slot` |
| 6 | `force_element` | `ForceElementComponent` | `element_slot` |
| 7 | `camera` | `CameraComponent` | `fov_degrees`, `near_plane`, `far_plane`, `active` |
| 8 | `behavior` | `BehaviorComponent` | `behavior_index` into the `BehaviorRegistry` |
| 9 | `fluid` | `FluidComponent` | `rest_density`, `stiffness`, `viscosity` -- **reserved for Plan B** |

`kComponentTypeCount = 10u`. Every one of the ten has a `SPADE_COMPONENT_TRAIT` specialization
giving it an id and a registered name.

> **RECONCILED:** `SL5` says implementation is limited to "ten" component types; `SL17`'s Phase
> P1 row says "the nine component types". These are not in conflict. **Ten are DECLARED** -- and
> `component.hpp` as built is the tiebreaker: it contains exactly ten enumerators, ten structs,
> ten traits, and `kComponentTypeCount = 10u`. **Nine are functional at P1.** `Fluid` is the
> tenth: it has an id, a struct, storage in `ObjectGraph`, and a full JSON round trip, but
> nothing in the engine consumes it -- there is no SPH pass anywhere in `spade/engine/physics/`.
> `Fluid` becomes live with Plan B (`SL8`). A reader checking one number against the other has
> not found a bug.

### Why the id is frozen

**The id IS the serialization key**, which is the entire reason it is assigned by hand rather
than derived. `typeid().hash_code()` is not stable across compilers, is not stable across builds
under some ABIs, and carries no ordering -- all three break a saved graph. A saved scene stores
these numbers, so all three properties are load-bearing.

**To add a component type:** append an enumerator at the end, append a name row at the end of
`kNames` in `graph.cpp`, bump `kComponentTypeCount`, and add a `SPADE_COMPONENT_TRAIT` line. A
type with no trait fails to **compile** rather than silently taking a default id.

**Never renumber an existing one.** It silently reinterprets every scene ever saved. This is the
one edit the design forbids outright. `FluidComponent` is declared now, ahead of Plan B
implementing SPH, **specifically so that discovering it later would not force a renumber.** The
reservation is the whole reason it is there.

A `static_assert` caps the count at 32: the per-object mask is a `uint32`, and a 33rd type needs
a wider mask, not a bit that shifts off the end and is silently dropped.

Queries, systems iterating by component signature, and archetype storage are **designed against,
not built.** Inventing the model twice is the expensive failure; building it incrementally
against a settled design is not.

---

## 4. Storage shape and generational handles

One parallel `std::vector<T>` per component type, indexed by object slot, plus a per-object
`uint32` bitmask where **bit N is type id N**.

**The mask is the source of truth** for "does this object have a `T`". A store holds a value at
every live slot, so only the bit distinguishes *attached* from *never attached*, and every
accessor consults it before it reads. Stores grow lazily: a graph that never attaches a
`FluidComponent` never allocates the fluid store at all.

`ObjectGraph` is a recycling slot pool. **Generation parity matches `BodyRef`'s deliberately --
odd is live, even is dead, 0 means never issued** -- so there is one rule to learn across the
engine rather than two that look alike and differ in a corner. `create()` takes a slot's counter
even to odd; `destroy()` takes it odd to even, and returns `false` rather than doing nothing
quietly for a null, out-of-range or already-destroyed handle, so a double-destroy is reportable
at the call site.

**Where it diverges from `BodyRef` is the part a reader who knows `BodyRef` will otherwise get
backwards.** `BodyRef`'s generations live in a *registered* array (`body_generation`) so
lifecycle survives snapshot/restore like every other piece of engine state. These generations
live in a plain member and are **not** registered -- not an oversight: registering them would
put the object graph inside the snapshot, which is the one thing `SL3` forbids. The counters are
rebuilt with the graph from its saved description, so nothing is lost, but an `ObjectId` is
meaningful only against the graph instance that issued it and does **not** survive a snapshot
restore the way a `BodyRef` does.

**Pointer lifetime.** Slots live in a `std::vector`, so any `create()` that grows the pool
invalidates every pointer `get()` has handed out; a later `attach<T>()` can reallocate that
type's store. The `ObjectId` is the stable reference. Store ids, resolve late.

**Iteration order is slot order**, a property of the graph alone and not of the order objects
happened to be created in -- which is what makes a serialized graph byte-stable for a given
graph.

**Hierarchy is unowned.** `Object::parent` is a plain field callers write through `get()`.
`ObjectGraph` neither validates nor maintains it: it does not reject a cycle, and `destroy()`
does not null the parent of the destroyed object's children. A child therefore keeps naming a
dead handle -- which `alive()` correctly reports as dead, so the dangling state is
**detectable**, never silently reattached to whatever recycles the slot. That is the property
worth having at this layer. The bill lands on the serializer.

---

## 5. Serialization

`to_json` / `from_json` in `serialize.hpp`. Objects are written in slot order, so the output is
deterministic for a given graph.

**Components travel by registered name.** A saved graph is readable and diffable, and the
name-to-id mapping stays single-sourced in `component.hpp` (`component_type_name` /
`component_type_id_from_name`), so the two cannot drift.

**Parent links travel by array index, and the asymmetry is deliberate.** Component types are a
fixed vocabulary and every name in it is unique by construction. Object names are neither --
they are user data, and nothing forbids two objects called `hub`. Resolving a parent by name
would silently attach the child to whichever one the loader reached first: a scene that reloads
with a quietly different hierarchy and no error anywhere. An index cannot be ambiguous. `name`
stays on every object as an attribute, so a human reading the file still sees what each entry is.

Loading is two passes -- every object without its parent link, then the parent links by array
position -- because a parent may sit at a higher slot than its child and the file must not
require ordering.

**A dangling parent is an error, not a silent re-root.** A dead parent is in no objects array, so
there is no index to write. `to_json()` returns a `Result` and **refuses**, naming the child.
Writing it as a root would save a hierarchy the author did not build, which is precisely the
silent change the in-memory design took care to avoid. A caller that wants re-rooting does it
explicitly, where it is visible -- the sandbox (Plan C) at delete time is the natural place.

`find_by_name` returns the first match in slot order. Names are not unique -- that is exactly why
parent links are indices -- so it is a convenience for tests and tools, never the resolution
mechanism for anything the format stores.

Note for archaeologists: `graph.hpp`'s prose still says the serializer "writes `parent` as the
parent's NAME". That is a forward-looking note from the plan; `serialize.hpp` records the
departure from it explicitly, and the built serializer writes indices. The index form is correct
and current.

---

## 6. `SL6` -- behaviors

"Behavior" is the one word in the framing with no precedent in the corpus, and the naive form of
it is incompatible with guarantees the engine is built on. The pass schedule is *data*; passes
cannot reorder themselves; there is no wall clock below `Simulation`; every pass is one interface
with two implementations, `execute_cpu` and `record_gpu`, same math, same operation order, fp32
both. A Unity-style `MonoBehaviour` -- arbitrary user code, arbitrary order, per-frame --
violates every one of those, and would not fail loudly. It would quietly destroy CPU-GPU parity,
the property the entire GPU program was built to establish.

**`SL6`: a behavior is a declared, ordered, fixed-step pass.** It is **data** -- a stable name,
one of two fixed slots, declared read/write masks over `ComponentTypeId` bits, a required
`execute_cpu` and an optional `record_gpu`. It does not choose where in the substep it runs.

### The two fixed slots

The schedule stays fixed and grows from eight passes to ten (user ruling, 2026-08-30). Behaviors
do not insert themselves anywhere; two slots are added to the fixed array and a behavior chooses
which of the two it belongs to. There is no third slot.

- **`BehaviorsKinematic`** -- after `MediumUpdate`, before `ForceElements`.
- **`BehaviorsForce`** -- after `ForceElements`.

**Why each position is a parity requirement, and why both slots are inert without a registry, is
in `02-engine.md` -- that document owns the schedule half of `SL6`.** It is not restated here.
What matters on this side of the split is that the position is *not* a per-behavior choice:
`physics/schedule.hpp`'s "fixed at compile time; there is no API to add, remove or reorder one"
stays literally true, and a behavior selects a slot rather than an index.

### Registration order is execution order

Within a slot, and it is stated rather than incidental: two behaviors writing the same
accumulator must compose in a defined sequence or the result is not reproducible.
`register_behavior` refuses a null `execute_cpu`, and refuses a duplicate name -- names are
`BehaviorComponent`'s serialization key, so a duplicate would make a saved graph ambiguous.

### GPU eligibility is a refusal, never a silent fallback

A behavior with no `record_gpu` latches the **whole registry** ineligible for the
GPU-authoritative path, and it does not recover. Degrading such a world to the CPU quietly would
put a world into the parity corpus **whose behavior did not run identically on both backends** --
the corpus would then be comparing two different experiments and passing. A CPU-only behavior
must declare itself so, and the world containing it is refused.

**Cost if wrong.** If the declared read/write sets turn out too coarse, behaviors serialize more
than necessary -- a performance cost, not a correctness one. The reverse error, permitting
undeclared access, would be silent divergence, which is why the declaration is mandatory rather
than advisory.

### Determinism obligations, inherited not renegotiated

No wall clock, and randomness only through the existing per-world, per-system domain-separated
`splitmix64` streams. **Nothing in the behavior API offers either, and that absence is the
mechanism rather than a rule to remember.** No allocation inside a step; structural effects
queued to step boundaries; the schedule stays data.

`set_behaviors()` is its own call rather than a `create()` parameter: a registry is authored
alongside a scene, which is built after the `Simulation`, and threading it through `create()`
would put it inside `WorldSetDesc` -- the thing snapshots and the replay config hash are computed
from. Behaviors are composition (`SL3`), not registered state, and keeping the attachment out of
the descriptor is what keeps that true. The registry is read once per step, where the
`SubstepContext` is built, so swapping registries between steps is well defined and swapping one
mid-step is impossible.

### `kinematic_mover`, the first behavior

It moves one body around a circle, and **its pose is a closed form of `(params, tick)`, never an
incremental integration.** That distinction is the whole design: a restored snapshot lands
exactly where the original run was, because the pose is recomputed from the tick rather than
carried in accumulated state the snapshot would have to know about; stepping 400 times and
stepping 4 x 100 give bit-identical results; and it is idempotent within a step, so every substep
of step k writes the same pose and collision sees a stable one. It uses `sin32`/`cos32` from
`core/fp32_math.hpp`, not `std::sin`/`std::cos`, whose results are not portable across platforms
or libm versions -- a behavior using them would move parity from "measured" to "hoped for".

Configuration travels beside the function as `BehaviorDesc::user_data`, an opaque pointer passed
back verbatim -- the minimum shape a parameterised behavior can have while staying data. The
function cannot be a capturing lambda (it is a plain pointer), cannot read a global (two
`Simulation`s in one process would then share one behavior's configuration), and cannot find its
own registry entry. **The pointee must outlive the registry and must not change during a step**:
it is read on the determinism-critical path, and mutating it mid-step would make the result
depend on when the mutation landed.

The mover binds its body through `params.body_slot` -- an index into the world's body slice,
matching `DragBodyRow::body_slot` -- rather than through `BehaviorComponent` plus
`BodyComponent`. The component path is the *designed* one; it needs the `ObjectGraph` in
`SubstepContext`, which nothing yet requires, and the user ruled the object model "designed fully,
built minimally". This is the capability the demo-mission program and the
`tracking-moving-target` training structure were both blocked on; it supplies the engine
mechanism only, and does not adopt DM3's mover/Target schema, which stays that addendum's to
define.

---

## 7. One invariant, one site

**A recycled slot is reset in `create()` and nowhere else.** `destroy()` bumps the generation and
frees the slot; it scrubs nothing. `detach<T>()` clears the type's bit and deliberately does
**not** overwrite the stored value -- the bit already makes it unreachable through
`component<T>()`.

The reasoning is a testability argument, not a style preference: **two sites maintaining one
invariant make each other untestable.** Neither could be removed without the other silently
covering for it, so no mutation could reach either one, and a guard over the invariant would
print green against a codebase where both sites had rotted. One invariant, one site, so the reset
can actually be shown to fail.

This is the local form of the estate-wide rule that a constant, test or fixture validated only
against something that cannot vary is unvalidated.

---

## 8. Files

| File | What it holds |
|---|---|
| `object.hpp` | `ObjectId` (= `Handle<ObjectTag>`) and `Object` -- name, parent, placement. |
| `component.hpp` | The ten types, `ComponentTypeId`, `ComponentTraits<T>`. |
| `graph.hpp` / `.cpp` | `ObjectGraph`: lifecycle, component attachment, masks, `for_each`. |
| `serialize.hpp` / `.cpp` | JSON round trip, `find_by_name`. |
| `behavior.hpp` / `.cpp` | `BehaviorSlot`, `BehaviorDesc`, `BehaviorRegistry`. |
| `behaviors/kinematic_mover.*` | The first declared behavior. |
| `spawn_helpers.hpp` / `.cpp` | Bulk seeded placement. **Compiles into `spade_sim`**, not `spade_objects`: it calls `Simulation`, and `spade_sim` already links `spade::objects`, so the other direction would be a target cycle. |

`behavior.hpp` forward-declares `physics::SubstepContext` and includes nothing from `physics/`:
`schedule.hpp` holds that struct and gives it a `const BehaviorRegistry*`, so mutual includes
would be a cycle -- and neither direction needs one, since a function pointer taking `const
SubstepContext&` is declarable against an incomplete type.

✅ **NO DEFECT IS BEING DESCRIBED IN THIS PARAGRAPH.** Every sentence in it is about a design that
was **rejected**; the built code is the repair (`behavior.hpp:118` — `struct Entry { std::string
name; ... }`). ⚠ **Stated first and separately, because a grep that surfaces any single line of what
follows reads as a live bug report** — measured 2026-09-18, when this realm quoted *"is a
use-after-free that usually still reads the right bytes"* out of it and escalated a defect that does
not exist. ⭐ **A warning about a rejected design and a report of a live one are the same words in a
different relation, and only the relation distinguishes them** — so the relation has to survive being
read one line at a time.

`BehaviorRegistry` **owns** its entry names, a correction to the plan rather than a style choice.
The plan stored `BehaviorDesc` values with the strings in a parallel `std::vector<std::string>`,
repointing each stored `name` view. That dangles: growing the vector moves its elements, and a
short `std::string` keeps its characters inside the object (SSO), so every view into one points
at freed memory the moment a reallocation happens -- and every name in the tests and in
`kinematic_mover` is short. Worse, it is a use-after-free that usually still reads the right
bytes, so a functional test cannot be relied on to catch it. Owning the string removes the
aliasing rather than managing it.

---

## 9. Verification obligations

From `SL18`, the ones this document's subject owns. **This is a SUBSET and always was** -- `SL18`
has **nine** obligations; the complete list, with an owner named against each, is
`06-sandbox-and-v1.md` s.7. ⚠ **Read that one before concluding `SL18` is satisfied**: until
2026-09-17 the five below were the only part of `SL18` present anywhere in the live spec set, so the
four belonging to no single document belonged to no document at all, and this list could be
satisfied in full while `SL18` was not.

⚠ **And one of the five below is only partly observable.** *Behavior determinism* asserts identical
trajectories **across backends** -- and the cross-backend half is measured by a GPU suite that **no
CI job has ever run** (`07-status.md` rule 14). The refusal half is proven by construction; the
agreement half is developer-machine-only.

- **The object graph does not perturb physics** -- an existing golden scenario produces
  bit-identical results with and without an object graph built over it. The `SL3` claim, tested
  rather than argued. **Discharged** by `test_objects_determinism.cpp`.
- **Snapshot compatibility** -- a snapshot taken before P1 restores after it, unmodified.
- **Behavior determinism** -- a world with a behavior produces identical trajectories across
  runs, processes and backends; a CPU-only behavior is *refused* from the GPU path rather than
  silently degraded.
- **Frozen ids** -- `test_objects_component.cpp` pins the ordering of `ComponentTypeId`.
- **Guard discipline** -- every guard must be shown to **fail under the mutation it exists to
  catch**. A constant, test or fixture validated only against something that cannot vary is
  unvalidated.

Open, and belonging to sibling documents: `SL8` SPH parity (Plan B, unwritten), the sandbox
obligations of `SL10`-`SL13` (Plan C written 2026-09-17, unexecuted), and the `SL7` transfer register, which still
carries one open row -- SPH fluid -- so v1 must not be quarantined until Plan B closes it.
