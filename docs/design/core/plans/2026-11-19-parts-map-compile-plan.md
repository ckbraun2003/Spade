# compile: a drone's parts map to a flying model (plan)

Core, 2026-11-19. DRAFT for the lead; docs only. Spade's L103: `compile` is one slice after the stand-ins, with Physics' `compile_airframe` doing the model and the x500 as the acceptance. This plan is the seam: what Core builds in front of `compile_airframe`, and what Core and Physics agree before code.

## 1. What it is

`compile(drone, options)` takes a library drone entry with a `parts_map` and gives the flying model a host would otherwise hand-author: a `ModelFile` the engine runs. It is NOT a second compiler. It reads the drone's parts, builds the `AirframeSpec` (`engine/vehicles/airframe_compile.hpp`, design frame) and calls `compile_airframe()`; the principal-axes conversion (`design_to_principal`, `com_offset`), the fitted rotor, the model's rules and the issues list are Physics'.

The input needs the part STORE (the meshes give the mass distribution). Without it `compile` refuses `store_file_missing` (THE RULE, library contract section 3: physics uses meshes only, never the stand-in shapes).

## 2. The steps

1. **Poses.** `compose_placement` over every slot, in the library's design frame (+X forward, +Y left, +Z up), as `tests/test_library_x500_entries.cpp` already does.
2. **One frame change.** Every pose and install point goes through the one fixed rotation Rx(-90 deg), `[sqrt(1/2), -sqrt(1/2), 0, 0]`, (x, y, z) -> (x, z, -y), into the `AirframeSpec`'s design frame (+Y up, +X forward, +Z right). For the X500 this is exactly the build table's frame, so the table is the cross-check. The origin is the parts map's own (the frame's origin); `compile_airframe` moves it to the centre of mass.
3. **Mass records.** Each part is a `PartInertia` record; a part with solid mesh objects is one mesh record per object (`part_meshes`: `role: solid` objects; a visual object never enters), all at the part's density `rho = part mass / sum of the objects' volumes` (uniform density, the same split Physics' kit tensor uses for the frame's 61 objects), at the object's placement (part pose x the object's own frame, which is the part's). A part with `shapes` and no meshes is its box (uniform). A part with a `mass_properties.com`/`inertia` declared uses them as a tensor record. One method for every part, mesh or box.
4. **Blocks.** Motor, prop, ESC and battery blocks come from the parts' v1 spec blocks (L103, section 4); the rotors from the motor install points (`pos` = the motor point composed with the frame, `spin`) and the prop part's `disc` point: the hub is the disc, thrust along the rotor frame's +Y. The prop's mass is the block's (on the hub, a thin disc), never a mesh record: a prop part's mesh is drawn, not summed.
5. **Compile.** `compile_airframe()`; its issues come back as the compile's, each tagged with the slot it came from.
6. **Result.** The model, its composite and fit as Physics returns them, plus what the library adds: the drone's hash and each part's hash and grade (the model's provenance).

## 3. Acceptance

The X500 drone (`drone.x500_build`), with the store: mass 1.6326 kg; the centre of mass and the full tensor equal `kKitCom` and `kKitXX..kKitYZ` in `tests/x500_model.hpp` (Physics' constants, from the same meshes and masses); `design_to_principal` a 5.4 degree rotation about z. A test pins them to 1e-9 relative (same method, double arithmetic), runs only with `SPADE_PART_STORE`, and a second test with no store asserts the refusal.

## 4. Open: the seam, for Physics through the lead

1. **The spec blocks (L103).** The kit's motor and prop get a v1 spec block on the part itself, beside its blocks. Today a `part_version: 2` file is blocks-only and a `part_version: 1` file is a spec; a file with both needs one rule. Proposal: `part_version: 2` accepts the v1 block (`motor:`, `prop:`, `esc:`, `battery:`) as another top-level block, read by the existing `part_file` reader, with the blocks reader ignoring it as it ignores citation paths it does not own. Physics sends the numbers and grades (done: the iris-class stand-in values, kv 880 marked "stand-in: the winding is unpublished"; the 1045 prop as the library's UIUC APC 10x5 at 0.254 m).
2. **Wall parts.** Each mesh record becomes a body-frame wall part for the flow unless its `geometry` is `none`. The X500 flow model uses Physics' proxy solids (stack, arm tubes, pack, motors), not 80 mesh walls. Proposal: the compile marks every record `source none` (mass only) and the flow parts stay a separate input (`options.flow_parts`), so the compile gives the model Physics already grades and nothing the flow solver was not tuned for.
3. **Drag.** A mesh record beside no `DragSpec` is refused (DRAG AND MESHES). The parts map has no drag; `options.drag` (a `DragSpec`, none by default) must be given, or the compile refuses with that issue. No guess.
4. **The motor.** Mass 0 in the motor block ("counted elsewhere") when the motor part's mesh is a record; or `motor_mesh` (a mesh in the hub frame at the motor's mass). Proposal: mesh record at the motor's pose, block mass 0, the same as every other part.

## 5. Size and order

One slice after the stand-ins: roughly four to five days. In order: the v1 block beside the blocks (1 day, a RED test per rule), `AirframeSpec` assembly (2 days), the acceptance test and the C ABI op `compile` (1-2 days; `evaluate` stays `not_implemented`). Physics reads this plan before any code; the lead carries the answers to the four points.
