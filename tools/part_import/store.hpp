#pragma once

// ---------------------------------------------------------------------------
// part_import/store.hpp -- the local part store's names (design note, section 3). <store>/<sha256>/<original file name> is the source;
// the converted sidecars are under <store>/<sha256>/<key>/, where <key> names everything that changes a sidecar.
// ---------------------------------------------------------------------------

#include <string>

#include "transform.hpp"

namespace spade::partimport {

// Everything that changes a sidecar: the converter's version (a number for the mesh readers, the OpenCASCADE and wheel versions for
// STEP), the tessellation tolerance (m) and angle (rad) (STEP; 0 for the others), the unit (empty when the file declares its own),
// the axes, the scale and the object selection (empty: every object).
struct KeyInputs {
    std::string converter;
    double tolerance_m = 0.0;
    double angle_rad = 0.0;
    std::string unit;
    Axes axes;
    double scale = 1.0;
    std::string object;
};

// The canonical text of the inputs: "spade-part-import-key-1;converter=...;tolerance=...;angle=...;unit=...;axes=...;scale=...;
// object=..." (one line), numbers as json_number writes them.
[[nodiscard]] std::string cache_key_text(const KeyInputs& in);
// The first 16 hex digits of that text's sha256.
[[nodiscard]] std::string cache_key(const KeyInputs& in);

}  // namespace spade::partimport
