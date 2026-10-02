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
| `D1`–`D12` | Engine decisions (2026-08-08) | Core `D2 D6* D7 D8 D9* D12*`; Physics `D1 D3 D4* D5 D6*`; Interface `D4* D10`; Test/Docs `D9* D11 D12*` |
| `A1`–`A11` | Engine addendum (2026-08-10) | Core `A1* A3 A4 A7 A9`; Rendering `A1* A2 A5 A6 A8`; Test/Docs `A10 A11` (records) |
| `P1`–`P8` | KAT-era pillars | Core `P1–P4 P8`; Rendering `P6`; Physics `P7`; `consumers.md` `P5` |
| `SA1`–`SA3` | KAT-era errata amendments | `consumers.md` `SA1`; Test/Docs `SA2`; Core `SA3` |
| `RS1`–`RS15`, `SR-*`, `PA-*` | Rendering and scene | Rendering (`RS5` shared with Core) |
| `SL1`–`SL18` | Library, objects, sandbox | `00-charter.md` `SL1 SL17`; Core `SL3–SL6`; Physics `SL8`; Interface `SL2 SL2a SL2b SL7 SL9a–f* SL10–SL14c SL15a`; Test/Docs `SL7* SL15b* SL16* SL18`; `consumers.md` `SL16*` |
| SPIR-V rules `P1`–`P5`, `E1`, `E2` | Shader scan rules | Test/Docs |
| `D-S5-1`, `D-S6-2` | In-program execution decisions | Core, Physics |
| `CORE-`, `PHY-`, `RND-`, `INT-`, `TD-` | New rulings, from 2026-10-01 | each realm's `00-decisions.md` |

`*` marks a split ruling: each listed realm homes its own half. Assigned 2026-10-01 (restructure plan R3) from a full inventory of the 94 ID tokens in `superseded/2026-09-consolidation/`; every row above is present in its realm's `00-decisions.md` as of 2026-10-02.

## Citing and signing

- **Citing.** Qualify an ambiguous ID with its series (`engine D7`, `SPIR-V rule P1`). Cite a document by section, never by line number.
- **Signing.** The user signs a ruling by marking its `00-decisions.md` row `signed <date>`. A law changes only with the user's signature.
