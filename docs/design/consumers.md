# Spade — consumers

Spade is a library. A consumer builds against the installed `spade::` package (`find_package(spade CONFIG)`) and the file formats Spade defines. **This is the only document in the library where a consumer's own rule series may appear.**

## KAT

KAT is a drone SDK and editor, and the consumer Spade was first built for. Spade was extracted from the KAT monorepo with its full history on 2026-09-28.

| What KAT uses | From Spade |
|---|---|
| Simulation, worlds, vehicles, sensors, rendering | the installed `spade::` targets |
| World description | the world-file format (`world_version`, owned by Core) |
| Session ring and replay | the snapshot format (`kSnapshotVersion`, owned by Core) |

**Format changes a consumer sees:**
- **Snapshot format v2** (module API stage 1, merged `062e1fa`). The header grows to 40 bytes and carries the configuration identity: the module set, versions and compiled order. A v1 blob is refused with the version message. A blob restores only into a simulation with the same identity (`L2`). KAT passes blobs through without parsing them, so a KAT session ring that holds v1 blobs cannot restore them after the upgrade.
- **Module API stage 3** (merged `42b352a`). The standard module set gains an `environment` module and a `dryden.sample` pass, so its configuration identity changes. A snapshot taken before this merge is refused on restore. The snapshot format itself is unchanged.

**What lives in KAT, not here:**
- The sim-host contract **C5** and its adapter (`dronesim/spade/`): the mapping from a drone description to Spade constructs, plus KAT-side timing and stamping.
- KAT's content: worlds, scenes, meshes.
- KAT's training system and editor.

**KAT's rule series** that older Spade text cites. KAT owns them; Spade does not treat them as authority:

| Series | What it is |
|---|---|
| `C1`–`C5` | KAT's platform contracts (C5 = sim host) |
| `HS1`–`HS12`, `F-1`–`F-6` | KAT's sim-host spec |
| `TS*`, `TR*` | KAT's training structures and training system |
| `B*` | KAT's amendment batches |
| `G*` | KAT's configuration architecture |
| `CP*`, `DM*` | KAT's component catalog and demo missions |
| `R1`–`R15` | KAT's errata |
| `CN-*`, `CG*`, `CS*`, `SC*`, `SD*`, `TA*`, `MV*`, `T0`–`T3` | other KAT series |

**Legacy Spade rulings that are about KAT.** Each keeps its ID and is homed here. None binds Spade's design.

| ID | Ruling | Status |
|---|---|---|
| charter `SA1` | Spade is the backend of KAT's M1b milestone | record |
| charter `P5` | A thin adapter above Spade (dronesim implements C5); no KAT types below the seam | superseded by this page and `L7`. The boundary holds from Spade's side as "no domain in the core" |
| `SL16` (KAT half) | KAT-side vocabulary of the 24th spec | record; the Spade half is in `test-docs/00-decisions.md` |
| `interleaving-shuffle.md` (§13.3, open) | A KAT runtime proposal; Spade only supplies `state_digest` | KAT's to decide (`superseded/2026-09-consolidation/interleaving-shuffle.md`) |

**Moved to KAT** (Kat `2a7b63cc`, 2026-10-03): the 30 render-agreement cases and the bookmark check, now against Spade's installed public render API only, with their data owned by Kat. Kat's sim-host tests no longer read Spade's `tests/golden`; nothing in Kat reads a Spade checkout. Spade rebuilds its agreement bands on its own content (`backlog.md`).

**The drone builder** is a joint spec with Kat (`plans/2026-10-03-drone-builder-engine-design.md`). Spade pushes and Kat pulls, by the user's ruling of 2026-10-03.

## Other consumers

None yet. A vehicle-dynamics or F1-style consumer would assemble its domain the same way: templates and modules on the public API, with no engine changes for its domain.
