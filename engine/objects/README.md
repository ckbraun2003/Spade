# `objects/` — the composition half of the ECS

`spade::objects` is object identity, composition and declared behaviors. The
engine design spec's §4 ratified this model and never built it; its *identity*
half already existed (`core/ids.hpp`'s generational `Handle`, `BodyRef` and the
registered `body_generation` array), and this module supplies the objects those
handles name.

## What it is NOT — read this before adding a field

**The object graph is not registered state.** It is composition and identity,
reconstructible from a description. Components hold *handles into the SoA arenas
`spade_state` already owns*; they never own simulation data themselves.

That is the load-bearing claim of the whole module (24th spec SL3), and it is
what lets a structural addition this large cost the determinism estate nothing:
GPU buffer order is driven by slot assignment, never by pool iteration order, so
`kSnapshotVersion`, the registry walk, and every CPU↔GPU parity band are
byte-unchanged. **`kSnapshotVersion` did not move for any of this.**

A field belongs on `Object` only if it survives: *could this be rebuilt from a
saved description, with no reference to how the simulation happens to be running
right now?* Position and orientation qualify — they are authored placement. A
velocity, a contact set or an accumulated force does not: that is state, it lives
in a `spade_state` arena, and a component references the slot holding it.

`test_objects_determinism.cpp` is the proof, and its header records what a
single-process comparison structurally cannot see.

## The ten component types, and why the ids are frozen

`ComponentTypeId` is hand-assigned, monotonic from zero, and dense:

| id | type | id | type |
|---|---|---|---|
| 0 | `transform` | 5 | `sensor` |
| 1 | `body` | 6 | `force_element` |
| 2 | `mesh` | 7 | `camera` |
| 3 | `material` | 8 | `behavior` |
| 4 | `collider` | 9 | `fluid` |

**The id IS the serialization key**, which is the entire reason it is assigned by
hand rather than derived. `typeid().hash_code()` is not stable across compilers,
is not stable across builds under some ABIs, and carries no ordering — all three
break a saved graph.

**To add a component type:** append an enumerator at the end, append a name row
at the end of `kNames` in `graph.cpp`, bump `kComponentTypeCount`, and add a
`SPADE_COMPONENT_TRAIT` line. A type with no trait fails to *compile* rather
than silently taking a default id.

**Never renumber an existing one.** It silently reinterprets every scene ever
saved. `FluidComponent` is declared now, ahead of Plan B implementing SPH,
specifically so that discovering it later would not force a renumber.

A `static_assert` caps the count at 32: the per-object mask is a `uint32`, and a
33rd type needs a wider mask, not a bit that shifts off the end.

## Storage shape

One parallel `std::vector<T>` per component type, indexed by object slot, plus a
per-object `uint32` bitmask where **bit N is type id N**. The mask is the source
of truth for "does this object have a `T`": a store holds a value at every live
slot, so only the bit distinguishes *attached* from *never attached*, and every
accessor consults it before it reads.

`ObjectGraph` is a recycling slot pool. Generation parity matches `BodyRef`'s
deliberately — **odd is live, even is dead, 0 means never issued** — so there is
one rule to learn across the engine. Where it diverges is the part a reader who
knows `BodyRef` will otherwise get backwards: `BodyRef`'s generations live in a
*registered* array so they survive snapshot/restore; these do not, because
registering them would put the object graph inside the snapshot, which is the one
thing SL3 forbids.

**One invariant, one site.** A recycled slot is reset in `create()` and nowhere
else. `destroy()` bumps the generation and frees the slot; it scrubs nothing.
Two sites maintaining one invariant would make each other untestable — neither
could be removed without the other silently covering, so no mutation could reach
either one.

## Serialization

`to_json`/`from_json` (`serialize.hpp`). Components travel **by registered
name**, so a saved graph is readable and the name↔id mapping stays
single-sourced. Parent links travel **by array index**, and the asymmetry is
deliberate: component types are a fixed vocabulary, unique by construction;
object names are user data, and nothing forbids two objects called `hub`.
Resolving a parent by name would attach a child to whichever one the loader
reached first — a scene reloading with a quietly different hierarchy and no error
anywhere.

Nothing owns hierarchy: `destroy()` does not null the parent of the destroyed
object's children, so a child can name a dead handle. That state is *detectable*
rather than silent — `alive()` reports the parent dead instead of reattaching the
child to whatever recycles the slot — and `to_json()` **refuses** to save it,
naming the child. A caller that wants re-rooting does it explicitly.

## Behaviors

A behavior is data: a name, one of **two fixed slots**, declared read/write
masks, a required `execute_cpu` and an optional `record_gpu`.

The two slots and their positions are the ruling, not a convenience:

- **`BehaviorsKinematic`** runs after `MediumUpdate` and **before**
  `ForceElements`, so a pose written by a kinematic behavior is set before
  anything reads it — both collision passes do.
- **`BehaviorsForce`** runs **after** `ForceElements`, so behavior wrenches
  accumulate after the rotors-then-drag order the golden corpus pins. Float
  addition is not associative; running before would change `force_acc`'s last
  bits.

The substep schedule is therefore ten passes, not `physics/schedule.hpp`'s
original eight. Both slots are inert when no registry is attached, which is every
golden-corpus scenario — so the corpus keeps re-proving that inertness on every
run rather than it having been a one-time observation.

**Registration order is execution order** within a slot: two behaviors touching
the same accumulator must compose in a defined sequence or the result is not
reproducible.

**GPU eligibility is a refusal, never a fallback.** A behavior with no
`record_gpu` latches the whole registry ineligible for the GPU-authoritative
path, and it does not recover. Degrading such a world to the CPU quietly would
put a world into the parity corpus whose behavior did not run identically on both
backends — the corpus would be comparing two different experiments and passing.

No wall clock, and RNG only through the domain-separated `splitmix64` streams.
Nothing here offers either, and that absence is the mechanism rather than a rule
to remember. `kinematic_mover` is the first behavior: its pose is a closed form
of `(params, tick)`, never an integration, which is what lets a restored
snapshot resume exactly where the original run was.

## Files

| File | What it holds |
|---|---|
| `object.hpp` | `ObjectId` (= `Handle<ObjectTag>`) and `Object` — name, parent, placement. |
| `component.hpp` | The ten types, `ComponentTypeId`, `ComponentTraits<T>`. |
| `graph.hpp` / `.cpp` | `ObjectGraph`: lifecycle, component attachment, masks, `for_each`. |
| `serialize.hpp` / `.cpp` | JSON round trip, `find_by_name`. |
| `behavior.hpp` / `.cpp` | `BehaviorSlot`, `BehaviorDesc`, `BehaviorRegistry`. |
| `behaviors/kinematic_mover.*` | The first declared behavior. |
| `spawn_helpers.hpp` / `.cpp` | Bulk seeded placement. **Compiles into `spade_sim`**, not `spade_objects`: it calls `Simulation`, and `spade_sim` already links `spade::objects`, so the other direction would be a target cycle. |
