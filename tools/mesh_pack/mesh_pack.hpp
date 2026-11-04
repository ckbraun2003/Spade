#pragma once

// ---------------------------------------------------------------------------
// tools/mesh_pack/mesh_pack.hpp -- spade_mesh_pack: any mesh file
// spade_mesh_import reads, to one .spmesh sidecar per object (CORE-6 D; the
// STEP converter plan, Task S1). Build-tree only (the lead's L23 5): it is not
// installed, and Kat reads meshes through spade::mesh_import itself. The STEP
// converter (tools/step_convert/) hands it the 3MF it writes.
//
//   spade_mesh_pack IN --out DIR [--unit U] [--tolerance LEN] [--source TEXT]
//
// IN is .gltf, .glb, .stl, .obj or .3mf (the extension, in any case, picks the
// reader). Each object the reader returns, in its order, is written as
// DIR/<name>.spmesh through save_mesh_file() (DIR is created if missing; a
// file there of the same name is replaced whole), and the mapping printed,
// one "<object> -> <file>" line each. Nothing is written unless every object
// was read and checked: the readers return all or nothing.
//
// THE PROVENANCE (MeshProvenance, outside the content hash):
//   unit       the unit the source declared (MeshObject::unit): metre for
//              glTF, --unit for STL and OBJ (which need it: they carry none),
//              the model's for 3MF. --unit beside a glTF or a 3MF is refused:
//              those files declare their own.
//   tolerance  --tolerance, in metres; 0 (not recorded) when absent.
//   source     "<S>: <object>", where S is --source, or IN's file name.
//
// LENGTHS AND UNITS. A unit is m, mm, cm, um, in or ft. --tolerance is a
// number and a unit together ("0.1mm", "1e-4m"), finite and >= 0; a bare
// number is refused, so a unit is never guessed.
//
// FILE NAMES. An object's name, as UTF-8, with every character a file name
// may not hold on Windows or Linux (control characters, < > : " / \ | ? *)
// replaced by '_' and trailing dots and spaces dropped; empty becomes
// "object", and a Windows device name (CON, PRN, AUX, NUL, COM1-9, LPT1-9,
// with or without an extension) gets a leading '_'. Two objects whose names
// match without regard to case get "-2", "-3", ... in order, so a
// case-insensitive file system never folds two into one.
//
// EXIT CODES: 0 written; 2 a usage error (nothing read); 3 the reader's
// unavailable (a feature this build does not read, named); 4 any other
// refusal of the file or a mesh (invalid_argument, capacity_exceeded, ...);
// 1 an input or output that cannot be read or written, or an internal
// failure. Every refusal is one line on the error stream, the reader's own
// message verbatim.
// ---------------------------------------------------------------------------

#include <filesystem>
#include <span>
#include <string>

namespace spade::mesh_pack {

// What one run printed and returned.
struct Outcome {
    int exit_code = 0;
    std::string out;  // the standard output's text
    std::string err;  // the error stream's text
};

// One run on `args` (the command line after the program's name), as main()
// gives them: paths stay paths, so a Windows wide argument survives.
[[nodiscard]] Outcome run(std::span<const std::filesystem::path> args);

}  // namespace spade::mesh_pack
