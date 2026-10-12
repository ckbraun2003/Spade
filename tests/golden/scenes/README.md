# Scene-file goldens (`SCN-002`)

Committed scene files that `spade::scene::scene_to_yaml()` must reproduce byte for byte, and that load back to the same bytes. `SceneFile.AScenesTextRoundTripsByteForByte` asserts them. They are goldens under `TD-1`.

- **`gate_run.scene.yaml`** exercises every section of the schema (`docs/design/interface/plans/2026-10-03-scene-file-draft.md`):
  - both optional asset halves;
  - a real design frame;
  - an unused model;
  - a non-zero spare.
  Its world is `../worlds/gate.world.yaml`, pinned by that world's real hash, so it moves when the gate world does.
- **`parts.scene.yaml`** is the first scene-version-2 text: a model with parts (`CORE-7`; the parts and meshes plan, Task A3). It has rigid and rotor parts, wall and porous surfaces, and a CSG shape, and a second model with no parts writes none. It is **hand-written** from the plan's schema, not generated (`TD-4`), so `SceneFile.PartsTextMatchesTheGolden` never writes it: a missing file fails. It names the gate world as `gate_run` does.
- **Provenance:** each file's first lines are `# provenance: ...` comments, saying when it was generated, from which commit, and why. The writer never produces them; the test requires at least one and sets them aside before comparing.
- **Regenerate** only as `TD-1` allows. Understand why it moved first, then delete the file and re-run the test. It writes the canonical text under a provenance stub and fails; replace the stub with the provenance, then commit. A regeneration is final once the Docker gcc leg reproduces it (`TD-12`).
- Line endings are LF everywhere (`../.gitattributes`).
