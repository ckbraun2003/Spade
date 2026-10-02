# Spade — consumers

Spade is a library. A consumer builds against the installed `spade::` package (`find_package(spade CONFIG)`) and the file formats Spade defines. **This is the only document in the library where a consumer's own rule series may appear.**

## KAT

KAT is a drone SDK and editor, and the consumer Spade was first built for. Spade was extracted from the KAT monorepo with its full history on 2026-09-28.

| What KAT uses | From Spade |
|---|---|
| Simulation, worlds, vehicles, sensors, rendering | the installed `spade::` targets |
| World description | the world-file format (`world_version`, owned by Core) |
| Session ring and replay | the snapshot format (`kSnapshotVersion`, owned by Core) |

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

**Moving to KAT:** the 30 render-agreement cases and the scene-drift guard that read KAT's worlds from `../KAT` (restructure plan R4). Spade rebuilds its agreement bands on its own content (`backlog.md`).

## Other consumers

None yet. A vehicle-dynamics or F1-style consumer would assemble its domain the same way: templates and modules on the public API, with no engine changes for its domain.
