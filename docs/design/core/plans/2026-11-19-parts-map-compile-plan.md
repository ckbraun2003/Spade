# compile: a drone's parts map to a flying model (plan)

Core, 2026-11-19. DRAFT for the lead; docs only. Spade's L103: `compile` is one slice after the stand-ins, with Physics' `compile_airframe` doing the model and the x500 as the acceptance. This plan is the seam: what Core builds in front of `compile_airframe`, and what Core and Physics agree before code.

## 1. What it is

`compile(drone, options)` takes a library drone entry with a `parts_map` and gives the flying model a host would otherwise hand-author: a `ModelFile` the engine runs. It is NOT a second compiler. It reads the drone's parts, builds the `AirframeSpec` (`engine/vehicles/airframe_compile.hpp`, design frame) and calls `compile_airframe()`; the principal-axes conversion (`design_to_principal`, `com_offset`), the fitted rotor, the model's rules and the issues list are Physics'.

The input needs the part STORE (the meshes give the mass distribution). Without it `compile` refuses `store_file_missing` (THE RULE, library contract section 3: physics uses meshes only, never the stand-in shapes).

## 2. The steps

1. **Poses.** `compose_placement` over every slot, in the library's design frame (+X forward, +Y left, +Z up), as `tests/test_library_x500_entries.cpp` already does.
2. **One frame change.** Every pose and install point goes through the one fixed rotation Rx(-90 deg), `[sqrt(1/2), -sqrt(1/2), 0, 0]`, (x, y, z) -> (x, z, -y), into the `AirframeSpec`'s design frame (+Y up, +X forward, +Z right). For the X500 this is exactly the build table's frame, so the table is the cross-check. The origin is the parts map's own (the frame's origin); `compile_airframe` moves it to the centre of mass.
3. **Mass records (Physics' answer: tensor records, geometry `none`).** `compile_airframe` gives a record a mesh only when its geometry source is `mesh`, and that makes it a wall part and a "mesh record" for the drag rule, so the compile resolves each mesh object to a TENSOR record itself, `source none` (mass only): `TriangleMesh::mass_properties()` (volume, centroid, inertia per unit density) for each `role: solid` object of `part_meshes` (a visual object never enters), uniform density `rho = part mass / sum of the part's solid objects' volumes`, mass `rho V` at the object's centroid, the tensor `rho I` turned by the part's pose and moved by the parallel-axis rule: the kit constants' own method (`tests/x500_model.hpp` `kit_tensor()` and `parallel_axis()`). A part with `shapes` and no meshes is its uniform box; a part that declares `mass_properties.com`/`inertia` uses them. One method for every part, mesh or box.
4. **Blocks.** Motor, prop, ESC and battery blocks come from the parts' v1 spec blocks (L103, section 4); the rotors from the motor install points (`pos` = the motor point composed with the frame, `spin`) and the prop part's `disc` point: the hub is the disc, thrust along the rotor frame's +Y. The prop's mass is the block's (on the hub, a thin disc), never a mesh record: a prop part's mesh is drawn, not summed.
5. **Compile.** `compile_airframe()`; its issues come back as the compile's, each tagged with the slot it came from.
6. **Result.** The model, its composite and fit as Physics returns them, plus what the library adds: the drone's hash and each part's hash and grade (the model's provenance).

## 3. Acceptance

The X500 drone (`drone.x500_build`), with the store: mass 1.6326 kg; the centre of mass and the full tensor equal `kKitCom` and `kKitXX..kKitYZ` in `tests/x500_model.hpp` (Physics' constants, from the same meshes and masses); `design_to_principal` a 5.4 degree rotation about z. Same method in double arithmetic, so the only difference is the constants' printing: Physics prints them to 9 to 10 digits (`kKitXX 0.0237928957`), so a 1e-9 relative pin can fail on that rounding. Physics regenerates them to 17 significant digits on a branch after batch 40 (the lead's ruling, L107), so the pin is 1e-9 relative once that lands. The test prints and asserts the frame's object count (61 solids). The object count must match: the frame's 61 store objects, all `role: solid` in the entries (checked: none is visual), the motor 1, the 6C 4, the GPS 2, the Pi 18, the camera 7. The test runs only with `SPADE_PART_STORE`; a second test with no store asserts the refusal.

## 4. Open: the seam, for Physics through the lead

1. **The spec blocks (L103).** The kit's motor and prop get a v1 spec block on the part itself, beside its blocks. Today a `part_version: 2` file is blocks-only and a `part_version: 1` file is a spec; a file with both needs one rule. Proposal: `part_version: 2` accepts the v1 block (`motor:`, `prop:`, `esc:`, `battery:`) as another top-level block, read by the existing `part_file` reader, with the blocks reader ignoring it as it ignores citation paths it does not own. Physics sends the numbers and grades (done: the iris-class stand-in values, kv 880 marked "stand-in: the winding is unpublished"; the 1045 prop as the library's UIUC APC 10x5 at 0.254 m).
2. **Wall parts (settled with Physics).** The flow parts stay a separate input (`options.flow_parts`: the X500's proxy solids for the stack, arm tubes, pack and motors), and the mass records are tensor records (step 3), so the compile makes no mesh walls. A "mesh record with source none" does not exist in `compile_airframe`; the tensor record is what it does with a mesh record before `composite_inertia()`.
3. **Drag (settled).** The parts map has no drag, and a tensor record gives the estimate no area (a silent zero), so `options.drag` (a `DragSpec`) is required and the compile refuses without it. No guess. The acceptance test passes a zero `DragSpec`.
4. **The motor (settled).** A tensor record at its pose, block mass 0, NOT `motor_mesh` (that works in the hub frame, and the hub is the disc, 0.0607 m above the table's origin; the kit constants have the four motors from their meshes at the table poses, 0.055 kg each). The v1 motor block's `kv` is rpm/V (920, or the graded 880), not the model file's rad/(s V).

## 5. Size and order

One slice after the stand-ins: roughly four to five days. In order: the v1 block beside the blocks (1 day, a RED test per rule), `AirframeSpec` assembly (2 days), the acceptance test and the C ABI op `compile` (1-2 days; `evaluate` stays `not_implemented`). Physics reads this plan before any code; the lead carries the answers to the four points.
