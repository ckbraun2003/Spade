#include "physics/contact_sets.hpp"

namespace spade::physics {

// Physics' derivation lands here, on top of L10 (the lead's grant): the row
// layout and the point order are physics/contact_sets.hpp's. Until then no
// model has a set, so the table is empty and every body keeps the sphere.
void build_contact_sets(std::span<const vehicles::ModelType> /*models*/, std::vector<float>& /*out*/) {}

}  // namespace spade::physics
