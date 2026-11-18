#pragma once

// world/mesh_import/outcome.hpp -- what a reader in its collecting mode does at the one place it builds an object's mesh (L88): it keeps the
// object's outcome, the mesh or make_mesh()'s refusal of it, and goes on to the next object instead of stopping. Internal to
// spade::mesh_import (it lives with the readers: no file outside world/mesh_import/ may include the reader headers).

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "world/mesh_import.hpp"

namespace spade::detail {

inline void record_outcome(std::vector<MeshObjectOutcome>& into, std::string name, MeshUnit unit,
                           const Result<std::shared_ptr<const TriangleMesh>>& built) {
    into.push_back(MeshObjectOutcome{std::move(name), unit, built});
}

}  // namespace spade::detail
