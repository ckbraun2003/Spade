# Spade design library

The design record for Spade, organised by realm. Start with `INDEX.md` if you're here to do one thing.

## Top level (lead)

| File | What it is |
|---|---|
| `00-charter.md` | Identity and the laws `L1`–`L8` |
| `01-engine-model.md` | Modules, fields, regions, objects, responders, scheduler, cameras, grades |
| `02-realms.md` | Who owns what, and how realms work together |
| `consumers.md` | KAT and any other consumer; the only place their rule series appear |
| `backlog.md` | Cross-realm work that isn't built yet |
| `plans/` | The lead's specs and plans |
| `superseded/` | Earlier documents, kept unedited |

## Realm libraries

| Realm | Directory | Prefix for new rulings |
|---|---|---|
| Core | `core/` | `CORE-n` |
| Physics | `physics/` | `PHY-n` |
| Rendering | `rendering/` | `RND-n` |
| Interface | `interface/` | `INT-n` |
| Test/Docs | `test-docs/` | `TD-n` |

Each library has a `README.md`; `00-decisions.md` (the only place a decision's status lives); numbered specs; `07-status.md` (the only place that describes what exists today); and `plans/`.

## Rule series

| Series | What | Home |
|---|---|---|
| `L1`–`L8` | Charter laws | `00-charter.md` |
| `D1`–`D12` | Engine decisions (2026-08-08) | realm `00-decisions.md` rows, see R3 |
| `A1`–`A11` | Engine addendum (2026-08-10) | realm `00-decisions.md` rows, see R3 |
| `P1`–`P8`, `SA1`–`SA3` | KAT-era pillars and amendments | realm `00-decisions.md` rows, see R3 |
| `RS1`–`RS15`, `SR-*` | Rendering and scene | `rendering/00-decisions.md` |
| `SL1`–`SL18` | Library, objects, sandbox | split across Core, Interface and Test/Docs, see R3 |
| `CORE-`, `PHY-`, `RND-`, `INT-`, `TD-` | New rulings, from 2026-10-01 | each realm's `00-decisions.md` |

The "see R3" homes are filled in by the restructure plan's cross-check.

## Citing and signing

- **Citing.** Qualify an ambiguous ID with its series (`engine D7`, `SPIR-V rule P1`). Cite a document by section, never by line number.
- **Signing.** The user signs a ruling by marking its `00-decisions.md` row `signed <date>`. A law changes only with the user's signature.
