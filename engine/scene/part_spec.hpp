#pragma once

// ---------------------------------------------------------------------------
// scene/part_spec.hpp -- a part's flight spec beside its blocks (L103, L107; the parts-map compile plan,
// docs/design/core/plans/2026-11-19-parts-map-compile-plan.md section 4).
//
// A blocks part (part_version 2: install points, meshes, shapes) may carry the BODY of a v1 part file (part_file.hpp) under a `spec:` key: one block of
// the kinds motor, prop, esc or battery, with its `sources`. It sits under `spec:` and not at the top level because the two readers each own `sources`:
// the blocks reader's cites mass_properties and install points, the part-file reader's cites the block's fields, and both are exact. The blocks reader
// ignores `spec`; this reads it as the v1 file it is, so the one set of rules (part_file.hpp) checks it and nothing is written twice. `kv` is the
// part file's: rpm/V.
// ---------------------------------------------------------------------------

#include <optional>
#include <string_view>

#include "core/error.hpp"
#include "scene/part_file.hpp"

namespace spade::scene {

// The part's `spec:`, or none when the text has no such key. invalid_argument, its context naming `spec`: `spec` is not a mapping, it holds no motor,
// prop, esc or battery block or more than one, or the v1 reader refuses it (its own message, its line and column those of the spec read as a v1 text).
[[nodiscard]] Result<std::optional<PartFile>> part_spec_from_text(std::string_view part_text);

}  // namespace spade::scene
