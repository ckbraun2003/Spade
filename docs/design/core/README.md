# Core library

Core owns the engine's foundation: the module API and the scheduler; state, snapshot, replay and the configuration hash; worlds, regions, fields and objects; grades; the backend seam; the `Simulation` API and the world-file schema; math, RNG and `Result`. The boundaries with the other realms are drawn in `../02-realms.md`.

## Read in this order

1. `../01-engine-model.md`, the target structure Core builds.
2. `01-modules-and-scheduler.md`: modules, the step, phases, behaviors.
3. `02-state-and-snapshot.md`: registered state, many worlds, snapshot, configuration hash, RNG.
4. `03-world-objects-regions.md`: the world description and file, regions and fields, objects and components.
5. `04-api-and-backend.md`: API conventions, numerics, the backend seam, grades.
6. `07-status.md`: what exists today and what is owed. **Read this before trusting any "is built" assumption.**

`00-decisions.md` holds every ruling Core is the home of, with its status. `plans/` holds Core's active plans when there are any.

## If you're here to…

| …do this | read |
|---|---|
| add state to a module | `02-state-and-snapshot.md` "Registered state" |
| add a pass or change pass order | `01-modules-and-scheduler.md`. Order is a numeric contract |
| touch snapshot, restore, reseed or the config hash | `02-state-and-snapshot.md` |
| change the world file | `03-world-objects-regions.md` "The world description" (version bump plus an upgrade path) |
| add a component type | `03-world-objects-regions.md` "Objects and components" (append; never renumber) |
| add a field or a region | `03-world-objects-regions.md` "Regions and fields" (target design; not built yet) |
| add a backend knob or touch the Vulkan recorder | `04-api-and-backend.md` "The backend seam" |
| add a public API call | `04-api-and-backend.md` "The public API" |

## What's next

1. **The module-API spec** (`../backlog.md`, first row): modules, scheduler phases, regions, the field registry and grades, building `../01-engine-model.md`. It is done when today's ten passes run as built-in modules through the new scheduler with every golden unchanged. It also settles the open questions in `07-status.md`.
2. **Translation lock on both backends** (`../backlog.md`, with Physics). The drone stand pins its vehicle with a CPU behavior, so Vulkan refuses it (`CORE-1`). A lock that runs on the GPU lets the stand use Vulkan.

## Series

- **Owned:** `CORE-n`, plus the legacy rows listed in `00-decisions.md`: `engine D2`, `D6` (interface half), `D7`, `D8`, `D9`, `D12` (error half), `A1` (schedule half), `A3`, `A4`, `A7`, `A9`; `charter P1`–`P4`, `P8`, `SA3`; `SL3`–`SL6`; `RS5` (format half); `D-S5-1`.
- **Quoted, owned elsewhere:** the laws `L1`–`L8` (`../00-charter.md`); `engine D1`, `D3`–`D5`, `D6` (provider half) (Physics); `RS*` and `SR-*`, including `RS5`'s meaning half (Rendering); `SL1`, `SL2*`, `SL7`–`SL18` (Interface, Test/Docs, lead); KAT series (`../consumers.md`).

## Citing

Qualify legacy IDs by series: `engine A3`, `charter P4`. Cite documents by name and section, never by line number.
