# Part-built drones in the editor (L88l)

Interface, 2026-11-19. Design only, no code, for the lead before any code. Generic: nothing here is specific to one drone; the X500 build supplies the figures and the tests' shape only.

## 0. What I read

- **Core's entries** (`core/library-x500-entries2`): 13 `part_version: 2` parts under `assets/library/parts/x500/` (kinds `frame`, `motor`, `prop`, `battery`, `payload`) and `drone.x500_build`, an index entry with a `parts_map` and no `file`. A part is blocks only: `mass_properties`, `install_points`, `meshes` (store refs, each `solid` or `visual`) or `shapes` (boxes and cylinders), `source`, and a `sources` citation with a grade per cited field. Of the 13, ten carry meshes only, three (battery, radio, electronics) shapes only. Meshes: 97 objects, 6.4 MB in the store (the frame 61, the companion computer 18).
- **Core's API.** On master: `open_libraries`, `list_entries` (`EntryPeek`: kind, mass, install kinds), `find_entry`, the typed `SetEntry::parts_map` (`PartsMapSlot`: slot, part, parent, install_point, placement, adjust, count), `compose_placement` (`library_place.hpp`: `pose = parent . install_point . adjust . part_to_mount`, double, per item, a bad item refused by name without hiding the others), `read_part_blocks`. On `core/library-part-meshes` (not yet on master): `part_meshes(set, id)`, a part's objects read from `<store>/<sha256>/<file>`, `{sha256, file, visual, mesh}` each; it refuses `store_file_missing` for the whole part when no store was given or one file is absent.
- **Why the editor shows none today.** `scene::load_library()` on a version-3 root returns the entries of the six kinds it knows and skips `part_version: 2` files; a drone with a parts_map and no file is not a `LibraryEntry` at all.
- **The editor today.** `load_library_items()` builds `LibraryItem`s (model, asset, part); a v1 part is inspect-only with graded rows and an "assumed" mark on its tile; a library drone with a sidecar shows "Built from" rows; a placed model is a copy (EDT-012) whose `parts` may be mesh parts (`imported_part()`, sidecars beside the scene by `placed_mesh_file()`), drawn by `append_vehicle_parts()`.

## 1. The library panel

A second reader beside the first, `load_built_items(root_dir, store)`, in a new display-free header `sandbox/editor_built_library.hpp`. It opens the same root the panel already reads (`open_libraries`, the host's store), lists the entries of kind `part` and `drone`, and keeps those `load_library_items()` did not list (same file path): the v2 parts and the drones with a parts_map. A root with none of them adds no row, so every existing row and its position stays as it is.

- **Where a row sits.** After every existing row of its own category, in the root's index order; a category nothing else uses is appended at the end. So the `drones` group lists the model drones first, then `X500 build`.
- **A part's row** (`LibraryItem` gains `built_part`: its `PartBlocks`, kind, entry hash): name, kind (`frame`, `motor`, `prop`, `battery`, `payload`), mass (`EntryPeek::mass_kg`). Its tile says "assumed" by the rule v1 parts use today: any cited field graded `assumed`. Inspect-only, like a v1 part. The inspector shows graded rows as v1 parts do (`graded_rows`): `mass_properties.mass` and each `install_points.<name>`, each with its grade, source id and note from the part's `sources`; then a Geometry row (`61 meshes, store found`, `3 shapes`, `none`) and the `source` record (id, licence, retrieved).
- **A drone's row** (`Kind::built_drone`): name, "drone, built from N parts"; the inspector's "Built from" block as `inspect_drone` does it, one row per slot (slot, part name, kind, count) in slot order; the summed mass and the weakest grade among the parts' cited fields (`assumed` weakest, the enum's own order); the part store's state (`found at <path>`, or `not found: set SPADE_PART_STORE`). Its tile arms it as a drone's does (`Command::arm`, the placement gesture unchanged).
- Entries the set refuses to list, or a part whose blocks do not read, are named and the rest still list (as an entry that will not load is today).

## 2. Placing a part-built drone

`AddBuiltDrone{id, vehicle, start}`, a new scene edit modelled on `AddMeshParts`: the workspace resolves the entry before the edit applies (`resolve_built_drone(set, id)`), the edit holds the result, and the text form (`scene add-built-drone <id> <name> pos ... [hash sha256:...]`) re-resolves it, so `--apply` works; a given hash that differs from the entry's refuses (`hash_mismatch`), none given places the current entry. One edit, one undo.

**Resolving** is pure and display-free:
1. Slots in dependency order (parent before child; a cycle, an unknown parent or an install point the parent lacks is refused naming the slot).
2. One `compose_placement` call per level: the frame slot by its `placement`; each slotted part by its parent's already-composed pose, the install point's pose from the parent's blocks, the slot's `adjust` and the install point's `adjust_range`. An `adjust_out_of_range` or `invalid_request` for one slot refuses the placement naming the slot and the axis; no half drone is placed.
3. The pose conversion from the design frame of the contract (+X forward, +Y left, +Z up) to Spade's body frame (+X forward, +Y up, +Z right): one fixed rotation applied to every pose, never per part (open question 2).
4. One `PartDesc` per mesh object, its `mesh` from `part_meshes` (an object a part places several times, such as four identical motors, shares one `TriangleMesh`), `mount_pos` and `mount_orient` the converted pose; a `visual` object is drawn only. A part with shapes only gets one `PartDesc` per shape (an SDF box or cylinder at its `pos`/`orient`).
5. The model is built around them. Mass is the sum of the parts' masses and the centre of mass their mass-weighted mean (each part's own `com` when it declares one); inertia is the point masses plus a part's own tensor when it declares one. It has no rotors and no propulsion chain, so the pilot panel says "no propulsion chain" as it does for any such vehicle (open question 1).

**When the part store is absent or lacks a file** the part falls back as a whole (`part_meshes` refuses the whole part), never a half-drawn part:

| the part has | store found | store absent or a file missing |
|---|---|---|
| meshes | drawn from the store | its `shapes` as stand-ins, if it has any; else not drawn |
| shapes only | its shapes | its shapes (the store is not needed) |
| neither | not drawn | not drawn |

Every fallback is named in three places: the inspector's Geometry row ("drawn as stand-in shapes: `<file>` not in the store"), the status line once on placement ("part store not found at `<path>`; 10 of 13 parts have no stand-in"), and the placement's notes in the test. A part with meshes and no shapes (ten of the 13 today) is invisible without the store; open question 5.

The store's location comes from `SPADE_PART_STORE` (the variable `spade_part_import` already reads), else none: no setting is invented.

Saving the scene writes the parts as sidecars beside it by the existing rule (`placed_mesh_file()`, named by content hash, so four identical motors write one); the X500 build is 6.4 MB (its 97 distinct objects; open question 4).

## 3. What the user can and cannot edit

Can, as for any vehicle: place, move, rotate (start state, `SetVehicleStart`), rename, duplicate, remove, reorder, select; undo and redo; run it as a rigid body; save and reopen. A part row in the inspector shows its own fields read-only.

Cannot, and the inspector says why: a part's geometry or its place on the drone (library facts, hash-pinned); a part's mass, centre of mass, install points or grades (editing would make a grade a claim the library did not make); swapping a part or editing the parts_map (that is the airframe builder's, KAT's). The slot `adjust` is carried (it moves the part where the map says) but has no editing UI: YAGNI until a user needs it; the `adjust_range` that would bound it is already read.

The placed drone is a copy (EDT-012). Nothing is ever written to the library, and the scene does not refer to it.

## 4. What it reuses and what is new

Reused unchanged: `LibraryItem`, the panel's tiles, arming and placement (`Command::arm`, `click`), `graded_rows`, `any_assumed`, the `inspect_drone` "Built from" layout, the scene edit machinery (`apply_panels`, undo, edit text, `--edit --apply`, `AddModelFile`'s sidecar save), `PartDesc` with a mesh and `imported_part()`, `append_vehicle_parts()`, `render_parts`, airflow solids from part poses, `SetVehicleStart` and the object edits, `EditorSession` itself (it draws `PartDesc`s already; the CSG upload gate is not touched).

New: `sandbox/editor_built_library.hpp` (`load_built_items`, `resolve_built_drone`, the body-only model, the fallback notes); the `AddBuiltDrone` edit and its text verb; `LibraryItem::built_part` and `Kind::built_drone`; the panel's extra rows and Geometry row in `editor_panels_ui.cpp`; the workspace opening the set and the store once with the library. Core's `part_meshes` and the L88l entries are needed on master first.

## 5. Tests

Display-free, in a new `tests/test_editor_built_library.cpp` (a small synthetic root and a tiny synthetic store made in a unique temp folder, so no test needs the box's real store):
- `BuiltLibrary`: the v2 parts and the drone list beside the old rows, which keep their positions and content; a root with no v2 parts lists exactly what it did; a part's kind, mass, assumed mark and per-field grades; a refused entry is named and the rest list.
- `BuiltPlacement`: poses equal hand-computed values for frame, motor and prop (two levels), with an `adjust` in range; out of range refuses naming slot and axis, placing nothing; a cycle and a missing install point refuse; the design-to-body rotation (a part ahead and above lands on +X and +Y); four identical motors share one mesh.
- `BuiltFallback`: store present gives meshes (a `visual` object drawn only); absent gives stand-in shapes for a shapes-only part and for a mesh part with shapes, nothing for a mesh part without, each named; one missing file falls the whole part back.
- Edit: apply, undo and redo; the text form round-trips and a wrong `hash` refuses; save and reopen draws the same parts (the pattern of `test_editor_place_meshes.cpp`); mass and centre of mass of the body-only model against hand sums.
- A second synthetic drone with other part kinds and no X500 names passes the same cases, and a source scan that `editor_built_*.hpp` names no drone (the repo's scan-test pattern).
- Panel rows: the inspector's row builder is display-free and tested; `UiConventions` is in the `-R` for any `sandbox/*.cpp` change.

Gates as usual: gcc-check on every changed `.cpp`, the Linux leg, `-L gpu` (the editor draws), and after the merge one recorded live smoke placing the real build with the store set and with it unset, posted per the video rules.

## 6. Open questions

1. **What does the placed drone fly as?** `compile` (the contract's route to a model from a parts_map) is a shape, not built. Recommended: the body-only stand-in above now, labelled as such, and `compile`'s model when Core builds it (the parts stay as they are). Physics or Core to say if they want the stand-in's inertia otherwise.
2. **The design-to-body rotation.** I read the contract's frame (+X forward, +Y left, +Z up) and the frame part's own placement quaternion carrying the file's Y-up axes into it. Core to confirm one fixed rotation to Spade's body (+X forward, +Y up, +Z right) is the whole conversion and that the origin is the parts_map's own.
3. **The store's location.** `SPADE_PART_STORE`, else none, as recommended. A setting in the editor would be a new surface; say if wanted.
4. **Sidecars or store refs on save.** Existing rule (copies, 6.4 MB for the X500 build) recommended: no schema change. A scene that refers to the store by `{sha256, file}` would be Core's.
5. **Stand-ins for the mesh-only parts.** Ten of the 13 draw nothing without the store. Recommended: Core adds a `shapes` stand-in to them (the part blocks already allow both). Otherwise the editor shows the named gap.
