# Spade — agent guide

Spade is a general-purpose, deterministic simulation and rendering engine (C++23). Projects such as KAT build on it; nothing in Spade should be specific to any one of them.

- **Layout:** `engine/` is the engine, `sandbox/` the editor app, `tests/` the test suite, `docs/design/` the design record. `src/ include/ examples/ assets/` are the frozen v1 engine — leave them alone.
- **Build and test:** `scripts\build.ps1` and `scripts\test.ps1`, run in the foreground.
- **Determinism matters:** the same seed and inputs should give the same results, so keep wall-clock time and unseeded randomness out of stepping.
- **Make changes easy to review:** small, clear diffs, with a short note on what changed and why.
- **Keep docs honest:** if you change behaviour, update the doc that describes it.
- **Ask first** before pushing or doing anything hard to undo.
