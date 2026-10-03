# Core — state, many worlds, snapshot and replay

**Owner:** Core. **Normative.** Where the code is today is in `07-status.md`.

## Registered state

- **Every authoritative quantity lives in a registered array.** An array has a name, an element size and a per-world capacity. Anything not registered is configuration or derived storage, and is not in a snapshot.
- **Arrays are world-partitioned SoA.** World `w` owns slots `[w*cap, (w+1)*cap)` of each array, and each array has a `slot_to_world` sibling. Capacities are fixed per world at `create()`; nothing reallocates mid-run.
- **Slots are assigned deterministically.** The lowest free slot is taken, in call order, and a freed slot is zeroed. Buffer order follows slot assignment, never object-graph order (`SL3`).
- **Registration order is the schema.** The registry walk defines snapshot section order and digest order. New arrays are appended. Doing so changes the schema hash, not the blob format version.
- **Modules register their own state.** This replaces the KAT-era rule that the state arrays were frozen. Derived GPU storage (mirrors, grid keys, scratch) is never registered.
- **Field sample rows are scratch** (module-API stage 3). Each world's row of field samples, such as gravity, density and wind, is rewritten by its providers in the Fields phase of every substep, before any reader runs. So it is not registered, not in any snapshot, and not in any digest. A restored run recomputes it in its first substep.
- **Element layouts are std430 PODs.** Field offsets are pinned with `static_assert`s, and Slang mirrors them through generated checks (`engine D9`). A POD's layout is a wire contract, so it is not reordered.

## Many worlds

- **World index is world identity.** One dispatch steps N worlds; N = 1 is the degenerate case (`engine D8`, `L8`).
- **Cross-world interaction is structurally impossible.** Every per-world binding is subscripted by the same world index in one place. The broad-phase key includes the world.
- **Batching is invariant.** A world stepped alone and the same world stepped inside a set produce identical bytes. This is tested, not assumed.
- **The CPU steps worlds serially** with identical per-world math, so a parity divergence names its world.

## Snapshot and restore (`L2`)

- **Format.** The blob starts with a header carrying:
  - magic `SPSN`;
  - format version (2);
  - schema hash;
  - tick;
  - world count;
  - array count;
  - configuration identity: an FNV-1a fold over the module set, taking each module's name and version in set order, then each compiled pass's name and phase in compiled order (`sim/module.hpp`, `compile_schedule`).

  One section per array follows, keyed by name. Blobs are untrusted input and are validated before anything is written. A version-1 blob is refused with the version message.
- **Snapshot refuses a non-empty structural queue** rather than dropping it.
- **Restore refuses a different configuration.** The configuration hash is stored in a registered row, and restore compares it byte for byte. Before that check, `Simulation::restore` refuses a blob whose configuration identity differs from its own module set's. Restore discards the structural queue. `dt`, substeps and the module set are configuration. The blob carries only their fingerprints, and they must match:
  - `dt` and substeps through the configuration hash;
  - the module set through the header's identity.
- **Reset is restore plus reseed** (`engine A3`). `reseed(seed)` re-derives every stream from a new world seed after a restore, so a reset does not replay the same noise.
- **Cursor state is in the blob; poll cursors are not** (`engine A4`). Ring write cursors, biases and streams are registered state. A caller's `since_index` is the caller's to keep.

## The configuration hash

- **What it is.** An FNV-1a fold over everything that affects stepping (`WorldSetDesc`, `dt`, substeps), in a pinned fold order.
- **What it excludes.** Spawn lists and render-only data: visual references, materials, lighting, props and per-node materials.
- **New fields must declare themselves.** A size guard on `WorldDesc` stops compilation until a new field is either hashed or explicitly excluded.
- **Target.** The module set and the in-phase pass order join the hash (`01-modules-and-scheduler.md`).
- **Today** they are checked through the snapshot header's configuration identity instead. `replay_config` is digested, so changing what it holds would move every golden. The identity moves into the hash with the next deliberate golden regeneration (module-API plan, Ruling 1).

## Randomness

- **Streams.** Every stochastic system draws from a `splitmix64` stream derived as `seed ^ fnv1a64(tag) ^ index`. The tag names the system (`"dryden"`, `"sensor.imu"`, `"sensor.gnss"`, …), and the index separates instances.
- **Ownership.** Each module owns its tags. A stream lives inside the module's registered row, so it is snapshotted with the state it drives.
- **No other source.** No `random_device`, no unseeded default, no clock. The stream's constants, derivation and Gaussian draw order are a wire contract.

## Replay and the digest

The determinism digest is an FNV-1a fold over the registry walk, in registration order. Golden scenarios carry their expected digest. Changing one is a Test/Docs-governed act with written provenance. The CPU is the golden source, and the GPU never is (`L4`).
