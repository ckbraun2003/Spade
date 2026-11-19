#pragma once

// ---------------------------------------------------------------------------
// scene/library_compile.hpp -- compile_drone: a drone's parts map to a flying model (L88l, L103, L107; docs/design/core/plans/2026-11-19-parts-map-compile-plan.md).
//
// NOT a second compiler: it assembles the airframe builder's AirframeSpec from the drone's parts and calls vehicles::compile_airframe(), whose model, fit and
// issues are Physics'. What it does in front of it:
//
//   1. poses      compose_placement over every slot (the library's design frame: +X forward, +Y left, +Z up), a part with a `seat` install point standing
//                 on its parent's point by that seat (its own mount offset is the seat's inverse: a rotor's hub bottom on the motor's boss);
//   2. one frame  every pose goes through the one fixed rotation Rx(-90 deg), (x, y, z) -> (x, z, -y), into the AirframeSpec's design frame (+X forward,
//                 +Y up, +Z right). The origin is the parts map's own; compile_airframe moves it to the centre of mass;
//   3. masses     each part is mass records in `AirframeSpec::parts`, mass only (no wall parts: walls come from flow parts alone). A part with solid mesh
//                 objects (part_meshes: `role: solid`) is one TENSOR record per object, by the kit constants' own method: TriangleMesh::mass_properties()
//                 at a uniform density `rho = part mass / the sum of its solid objects' volumes`, mass rho V at the centroid, the tensor rho I turned by
//                 the object's pose. A part with `shapes` and no meshes is its uniform boxes (the shapes are the stand-ins of a part with meshes, and
//                 meshes win: THE RULE of the library contract); a part with neither is a point mass. The motor, ESC and pack blocks carry mass 0 (their
//                 parts' records are the mass); the propeller's mass is its block's, a thin disc on every hub;
//   4. blocks     motor, prop, ESC and battery from the parts' `spec:` blocks (part_spec.hpp), `kv` in rpm/V; the rotors from the frame's motor install
//                 points (spin) and each prop's `disc` point (the hub: the actuator disc), thrust along the rotor frame's +Y.
//                 A part is authored in the axes of its own file (+Y up, as the kit's CAD and glTF), so a prop's thrust is its own +Y; the frame is placed
//                 into the design frame by a placement that turns those axes to +Z up (the X500: Rx(+90 deg) then Ry(180)), and step 2 turns them back.
//
// THE STORE. Without it a mesh part is `store_file_missing` (part_meshes' refusal): physics never uses the stand-in shapes, or the same inputs would give
// another inertia on a machine without the store.
//
// DRAG IS REQUIRED. A tensor record gives the compile's drag estimate no area (a silent zero), so `options.drag` is given or the compile is refused
// (`invalid_request`, field `drag`); a zero DragSpec says "none". No guess.
// ---------------------------------------------------------------------------

#include <optional>
#include <string_view>

#include "scene/library_set.hpp"
#include "vehicles/airframe_compile.hpp"

namespace spade::scene {

struct CompileOptions {
    std::optional<vehicles::DragSpec> drag{};  // required
};

struct CompiledDrone {
    vehicles::AirframeSpec spec{};              // as assembled, in the airframe's design frame
    vehicles::CompiledAirframe compiled{};
};

// Refused: bare_id, root_not_loaded, entry_not_found (as find_entry); invalid_entry (not a drone with a parts map, a part that cannot be read, no motor
// point, no spec for a block the airframe needs); invalid_request (field `drag`: none given); store_file_missing (a mesh part, no store or no file);
// invalid_argument from compile_airframe, its issues joined in the message.
[[nodiscard]] LibraryResult<CompiledDrone> compile_drone(const LibrarySet& set, std::string_view id, const CompileOptions& options);

}  // namespace spade::scene
