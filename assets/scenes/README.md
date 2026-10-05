# Scenes (and their worlds in `../worlds/`)

Engine content: scene files (`*.scene.yaml`, `spade::scene`'s format, `SCN-001`–`SCN-009`) and the world files they name (`../worlds/*.world.yaml`). `spade::scene::compose_file()` composes a scene with its world, and `instantiate()` makes the run. They are the editor's built-in scene sources (`SL13`) and the live smoke's main functions to come. They are not goldens.

## The viewer scenes

Each `spade_viewer` scene is here as a world and a scene. `bounce` and `swarm` are four-world scenes in the viewer, so each lane is its own pair: `<scene>_lane_<i>`.

`tests/test_viewer_successors.cpp` ties each one to the viewer. It runs the viewer's own setup (which the trajectory goldens in `tests/golden/viewer/` pin) beside the scene, and before every step asserts that every row the viewer holds is the scene run's row, byte for byte. A change to one of these files that changes the run fails there.

What the files do not hold, and the test supplies from the viewer:
- the per-world physics records (turbulence, contact parameters, the broad-phase grid) and the run's seed, which world file v2 does not carry (`WorldInstanceDesc`);
- flight's command hook, which is code.

Where a run differs by construction, the test says why:
- `compose()` pads every capacity by at least one slot (`SCN-007`);
- the viewer's bare bodies are vehicles of a body-only model here (`drop_body` and the like);
- a lane's world is world 0 of its own run.

## Regenerating

Delete a scene's files and run the test. It drafts them from the viewer scene into the build tree (`test-output/viewer-successors/`) and fails, naming them. Review the drafts, then copy them here. A file the writer did not produce is still valid content: loading validates every field.
