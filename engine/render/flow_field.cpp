#include "render/flow_field.hpp"

// RED: stubs.

namespace spade::render {

FieldPalette field_palette_of(ColourMapId) { return FieldPalette{}; }

Result<FieldLayer> slice_flow_volume(const FlowVolume&, FlowQuantity, const FieldLayer& geometry) { return geometry; }

Result<void> validate_flow_surface_colouring(const FlowSurfaceColouring&) { return {}; }

}  // namespace spade::render
