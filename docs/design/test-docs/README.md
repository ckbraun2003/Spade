# Test/Docs library

Test/Docs owns how Spade is verified and documented: the test harness, golden-corpus governance, the parity harness, the SPIR-V scanner, the bench, the build and gate scripts, and the structure of `docs/`. Each realm writes its own tests and specs; this library sets the rules they follow. Boundaries: `../02-realms.md`.

## Read in this order

1. `../00-charter.md`, especially `L3` and `L4` (grades, and the GPU is never a golden source).
2. `01-verification.md`: the suite, grades, goldens, parity, the SPIR-V scanner, the bench, guards.
3. `02-build-and-gate.md`: scripts, presets, the gate, this machine, pushing and committing.
4. `03-documentation.md`: where docs live, and the rules for writing them.
5. `07-status.md`: the measured baseline and what is owed. **Read it before quoting a test count.**

`00-decisions.md` holds every ruling Test/Docs is the home of. `08-lessons.md` is the short version of what verification here has taught. `plans/` holds active plans when there are any.

## If you're here to…

| …do this | read |
|---|---|
| add a test | `01-verification.md` "The suite" |
| add a GPU test | `TD-6`, then "The suite" |
| change a golden or a digest | `TD-1`, then `01-verification.md` "Goldens" |
| add or change a parity band | `TD-2`, then your realm's verification spec |
| add a kernel | `01-verification.md` "SPIR-V scanner" |
| report a test count or a timing | `TD-8` and `07-status.md` |
| build, test, or change a script | `02-build-and-gate.md` |
| write or move a doc | `03-documentation.md` |

## Series

- **Owned:** `TD-n`; `SPIR-V rule P1`–`P5`, `E1`, `E2`; and the legacy rows in `00-decisions.md`: `engine D11`, `D12` (toolchain half), `A10`, `A11`; `charter SA2`; `SL7` (guard half), `SL15b` (test half), `SL16`, `SL18`.
- **Quoted, owned elsewhere:** the laws (`../00-charter.md`); `engine D9`, `A7`, `CORE-n` (Core); `SL8`, `PHY-n` (Physics); `SR-*`, `RND-n` (Rendering); `SL7`'s rule, `SL2*`, `SL15a` (Interface); KAT series (`../consumers.md`).

## Citing

Qualify by series: `engine D11`, `SPIR-V rule P3`, `TD-1`. Cite documents by name and section, never by line number.
