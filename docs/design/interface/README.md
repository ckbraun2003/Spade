# Interface library

The editor (the sandbox grows into it), the viewer, templates and examples, packaging, install and export, the out-of-tree consumer, the CLI and demos, and user guides. Code:
- `sandbox/` (`spade_sandbox`, the reference application and the editor-to-be);
- `engine/tools/viewer/` (`spade_viewer`, which renders v2 physics through the v1 bridge until v1 is quarantined);
- `examples/` (v2), `tests/consumer/`, `cmake/spadeConfig.cmake.in` and the install rules;
- `scripts/demo.ps1`;
- the matching `tests/test_sandbox_*.cpp` suites.

## Read in this order

1. `01-editor.md`: what the sandbox is for, the rules every editor feature keeps, and the drone sim box.
2. `02-library-surface.md`: what Spade installs, how a consumer links it, and the sandbox's public-surface rule.
3. `03-v1-retirement.md`: the transfer register, the per-system dispositions, and the quarantine that is still blocked.
4. `07-status.md`: what exists today and what is open.
5. `00-decisions.md`: the ruling behind an ID, and whether it is in force.

## If you're here to…

| …do this | read |
|---|---|
| add an editor panel or control | `01-editor.md` (headless first; the window is a dumb shell) |
| change what the sandbox renders, or how | `01-editor.md` (`SL10` same path, `SL11` the sink seam), then Rendering's library |
| add a scene, preset or template | `01-editor.md` (scene sources), `02-library-surface.md` (templates use the public API only) |
| change what Spade installs, or the consumer project | `02-library-surface.md` |
| use an engine header from the sandbox | `02-library-surface.md` (`SL2b`): if it is not installed, that is a finding for Core, not a wider include path |
| retire or quarantine v1, or touch `spade_viewer` | `03-v1-retirement.md`. **Blocked while the SPH row is open** |
| find out whether `SL10`, `SL14a`, `engine D10` or `INT-n` is in force | `00-decisions.md` |

## Series

- **Owned here:** `SL2`, `SL2a`, `SL2b`, `SL7` (the rule), the Interface halves of `SL9a`–`SL9f`, `SL10`–`SL14c`, `SL15a`, `engine D10`, the quadrotor-template half of `engine D4`, and new rulings `INT-n`.
- **Shared:** `SL7` with Test/Docs (the rule here, the guard there); `SL9a`–`SL9f` with Core and Rendering, each homing its own half; `engine D4` with Physics (the layer there, the template here).
- **Quoted, not owned:** the laws `L1`–`L8` (`../00-charter.md`); `SL3`–`SL6` (Core, the object model and behaviors); `SL8` (Physics, SPH); `SL15b`, `SL16`, `SL18` (Test/Docs); `RS*` (Rendering).

Cite as `SL10` or `INT-2`, by document and section, never by line number. Status lives in `00-decisions.md` and `07-status.md` only.

## History

The source text is `../superseded/2026-09-consolidation/06-sandbox-and-v1.md` (`SL7`–`SL15`), `01-charter.md` §6 (`SL2`, `SL2a`, `SL2b`) and `02-engine.md` (`D4`, `D10`). The old sandbox plan is in `plans/` (superseded by the editor direction). The drone sim box's design and plan are the lead's, in `../plans/`. Active Interface plans go in `plans/`, dated.
