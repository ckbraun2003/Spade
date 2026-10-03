# Viewer trajectory goldens (`INT-4`, `SL14b`)

One file per `spade_viewer` scene: the CPU trajectory that its successor scene must reproduce bit for bit (`SL14b`). These are goldens under `TD-1`, and the trajectory guard in `spade_tests` asserts them.

- **Format:** `engine/tools/viewer/trajectory.cpp`'s header comment. Each checkpoint line carries the running chain digest, so any prefix of the file can be checked.
- **Provenance:** each file's first lines say when it was captured, from which commit, and why.
- **Regenerate** only as `TD-1` allows. Understand why it moved first. Then run `engine\tools\viewer\capture-v1-baselines.ps1 -Part trajectories -Force -Reason "<what changed and why>"`. Never shorten, skip or drop checkpoints to get past a mismatch. A regeneration is final once the Docker gcc leg reproduces it (`TD-12`).
- The frames and performance numbers from the same capture are an archive in `tests/v1-baselines/`.
