#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <string_view>
#include <vector>

#include <glm/glm.hpp>

#include "physics/contacts.hpp"
#include "state/layout.hpp"
#include "vehicles/model_type.hpp"

// ---------------------------------------------------------------------------
// physics/contact_sets.hpp -- the contact sets' configuration table: its name,
// its row layout and the function that builds it. Physics' design is L9 (the
// part-contact note); Core's storage for it is L10, with L11's rulings.
//
// THE TABLE. static_contact declares it (sim/standard_modules.cpp, a
// ConfigTableDecl). register_model() rebuilds it from every registered model,
// in registration order, outside the walk, the digest and the snapshot. The
// registry identity folds it while it is non-empty, so a blob restores only
// beside the same sets (Simulation::restore()). The GPU binds it at set 0,
// binding 28 (bindings.slang's contact_sets), one buffer of the capacity
// below, uploaded at create() and after every register_model().
//
// THE ROW LAYOUT (agreed by Physics and Core, 2026-11-03). The table is float4
// rows (kContactSetRowFloats floats each):
//
//   rows 0 .. model_count - 1: model m's range {first, count, 0, 0}, in
//     registration order (row m - 1 for ModelTypeId m). first and count are
//     exact-integer floats (below 2^24, so exact), and first is an ABSOLUTE
//     row index into this table. A model without a set has the row
//     {0, 0, 0, 0}.
//   then the point rows {x, y, z, 0}: body frame (the centre of mass at the
//     origin, the principal axes), so a point's world position is
//     pos + orient * (x, y, z), as BodyState's pos and orient give it.
//
// When no registered model has a set, the build appends NOTHING: the table is
// empty, so a part-less registry's identity is master's.
//
// THE POINT ORDER (Physics' derivation). Per rigid part, in part order
// (posed_parts() at the identity pose, PartFilter::rigid_only): its
// part_bounds() box's 8 corners, corner c taking max.x if c & 1, max.y if
// c & 2 and max.z if c & 4 (else the min), through that part's posed
// transform. Then per rotor, in rotor order: 8 rim points at k * 45 degrees,
// k = 0..7, hub + R (cos X_hub + sin Z_hub), the hub the rotor's local_pos,
// X_hub and Z_hub the hub frame's axes (+Y is the thrust); cos and sin are the
// constants 1, 0.70710677f, 0, -0.70710677f and -1, with no libm.
//
// BodyState::contact_set (state/layout.hpp) names a model's range row:
// the body's ModelTypeId when its model's count is non-zero, 0 (the sphere)
// otherwise.
// ---------------------------------------------------------------------------

namespace spade::physics {

// The table's name in the module set: "static_contact.contact_sets".
inline constexpr std::string_view kContactSetsTable = "contact_sets";

// Floats per row, kContactSetRowFloats (physics/contacts.hpp, which the pass
// reads it from): a float4, for the range rows and the point rows alike.

// The table's capacity, in rows (L11). Fixed, because the device buffer is
// sized once at create(): the step's command buffer is recorded once and its
// descriptor set is written once. register_model() refuses a model whose
// registration would take the table past it, on both backends, and leaves the
// registry as it was. 16384 rows is 256 KiB: about 135 models the size of the
// Phantom (11 parts and 4 rotors, 121 rows each).
inline constexpr uint32_t kContactSetsCapacityRows = 16384;

// static_contact.resolve's index of the table in its access list
// (sim/standard_modules.cpp's kStaticContactAccess, which pins it).
inline constexpr std::size_t kStaticContactSetsAccess = 2;

// The table from every registered model, in registration order (a
// ConfigBuildFn: `out` arrives empty and the build appends the table's
// floats): contact_points() of each, laid out as above.
void build_contact_sets(std::span<const vehicles::ModelType> models, std::vector<float>& out);

// One model's contact points (L9), body frame, in the table's point order;
// empty for a model without a body-attached part, which keeps the sphere. A
// part whose part_bounds() is refused gives no corners.
[[nodiscard]] std::vector<glm::vec3> contact_points(const vehicles::ModelType& model);

// The lowest world y over the model's contact points at `body`, each placed by
// contact_point_world() (physics/contacts.hpp), the pass's own expression, so
// the editor's rest_height() and the contact agree to the bit (L9's one source
// for rest); nullopt for a model without a set.
[[nodiscard]] std::optional<float> lowest_contact_point(const vehicles::ModelType& model, const BodyState& body);

}  // namespace spade::physics
