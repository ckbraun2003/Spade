# Spade — agent guide

Spade is a general-purpose, deterministic simulation and rendering engine (C++23). Projects such as KAT build on it; nothing in Spade should be specific to any one of them.

- **Layout:** `engine/` is the engine, `sandbox/` the editor app, `tests/` the test suite, `docs/design/` the design record. `src/ include/ examples/ assets/` are the frozen v1 engine — leave them alone.
- **Build and test:** `scripts\build.ps1` and `scripts\test.ps1`, run in the foreground.
- **Determinism matters:** the same seed and inputs should give the same results, so keep wall-clock time and unseeded randomness out of stepping.
- **Make changes easy to review:** small, clear diffs, with a short note on what changed and why.
- **Keep docs honest:** if you change behaviour, update the doc that describes it.
- **Ask first** before pushing or doing anything hard to undo.
- **Jobs:** every heavy build or test goes through jobrun with `--ticket <tree>-<realm>-<what>-<sha>` (e.g. `spade-physics-b5-1130608`), `--cwd` your realm's worktree, and a subagent using its realm's name. Check `--queue-status` first, and never queue a job that duplicates one already queued or running (same command, same folder). End every subagent with a handback when its task is done; stop one Vantage flags as stuck and restart it from its resume note.
- **Night queue:** queue heavy work for the downtime with `python C:\Users\Cameron\Desktop\Vantage\vantage.py queue --downtime --ticket <tree>-<realm>-<what>-<sha> --cwd <dir> -- <command>`. It runs through jobrun as usual and its results land in the brief.
- **Streams:** show Cameron work live with `python C:\Users\Cameron\Desktop\Vantage\vantage.py stream start --title "<what>" --window "<window title>" --ticket <...>` (one window only, never the desktop or a game; at most two at once, two hours max), and end it with `stream stop <id>` when done.
