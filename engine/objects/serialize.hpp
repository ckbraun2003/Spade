// serialize.hpp -- object graph <-> JSON (24th spec SL4, Plan A Task 6).
//
// COMPONENTS SERIALIZE BY REGISTERED NAME, not by numeric id, so a saved graph
// is readable and diffable. The name<->id mapping stays single-sourced in
// component.hpp (component_type_name / component_type_id_from_name), so the
// two cannot drift.
//
// PARENT LINKS DO NOT. They are written as the parent's INDEX IN THE OBJECTS
// ARRAY -- and that is a deliberate departure from the plan, which said to
// write the parent's name. The plan's reasoning for names was readability, and
// it is correct for component TYPES: those are a fixed vocabulary, and every
// name in it is unique by construction. Object names are neither. They are
// user data, nothing forbids two objects called "arm", and resolving a parent
// by name would then silently attach the child to whichever one the loader
// reached first -- a scene that reloads with a quietly different hierarchy and
// no error anywhere. An index cannot be ambiguous. `name` stays on every
// object as an attribute, so a human reading the file still sees what each
// entry is.
//
// A DANGLING PARENT IS AN ERROR, NOT A SILENT RE-ROOT. Nothing owns hierarchy
// (see graph.hpp): destroy() does not null the parent of the destroyed
// object's children, so a child can name a dead handle. That state is
// detectable rather than silent in memory, and this is where the bill comes
// due -- a dead parent is in no objects array, so there is no index to write.
// to_json() therefore returns a Result and REFUSES, naming the child. The
// alternative -- writing it as a root -- would save a hierarchy the author did
// not build, which is precisely the silent change the in-memory design took
// care to avoid. A caller that wants re-rooting must do it explicitly, where
// it is visible; the sandbox (Plan C) is the natural place, at delete time.

#pragma once

#include <string>
#include <string_view>

#include "core/error.hpp"
#include "objects/graph.hpp"

namespace spade::objects {

// Objects are written in slot order (ObjectGraph::for_each), so the output is
// deterministic for a given graph. Fails only on a dangling parent link.
[[nodiscard]] Result<std::string> to_json(const ObjectGraph& graph);

// Errors: invalid_argument naming the offending key for an unknown component
// name, a vector of the wrong length, a parent index out of range, or a
// malformed document.
[[nodiscard]] Result<ObjectGraph> from_json(std::string_view text);

// Handles are an in-memory fact and are NOT preserved across a round trip;
// name is an object's stable identity for a reader. Returns the FIRST match in
// slot order, or kNull. Names are not unique -- that is exactly why parent
// links above are indices and not names -- so this is a convenience for tests
// and tools, never the resolution mechanism for anything the format stores.
[[nodiscard]] ObjectId find_by_name(const ObjectGraph& graph, std::string_view name) noexcept;

}  // namespace spade::objects
