#pragma once

// world/mesh_import/outcome.hpp -- what a reader in its collecting mode does at the one place it builds an object's mesh (L88): it keeps the
// object's outcome, the mesh or make_mesh()'s refusal of it (with the kind make_mesh_diagnosed names), and goes on to the next object instead of stopping. Internal to
// spade::mesh_import (it lives with the readers: no file outside world/mesh_import/ may include the reader headers).

#include <expected>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "world/mesh_import.hpp"

namespace spade::detail {

using DiagnosedMesh = std::expected<std::shared_ptr<const TriangleMesh>, MeshDiagnosis>;

inline void record_outcome(std::vector<MeshObjectOutcome>& into, std::string name, MeshUnit unit, const DiagnosedMesh& built) {
    MeshObjectOutcome o{std::move(name), unit, {}, std::nullopt};
    if (built) {
        o.mesh = *built;
    } else {
        o.mesh = std::unexpected(built.error().error);
        o.kind = built.error().kind;
    }
    into.push_back(std::move(o));
}

}  // namespace spade::detail
