#pragma once

// ---------------------------------------------------------------------------
// flow_field -- the field channel for flow volumes (airflow visualization
// plan section 5).
//
// SLICES. A FieldLayer filled by sampling a volume quantity at each field
// cell's centre, by render/flow_sample.hpp's one rule. The existing CPU
// and GL slice view then draws airflow with no new drawing code. A no-data
// sample is NaN in the layer, which the layer draws as no-data (L6).
// field_palette_of() gives a layer the flow views' published colour maps,
// so slices and volumes share one mapping.
//
// SURFACE COLOURING. Each opaque pixel inside the volume's box takes a
// quantity's value at its world position, reconstructed from depth,
// through a transfer function's colours. Gauge pressure on the vehicle is
// the surface C_p view an aerodynamicist reads first. Unlit, and it
// changes no geometry. The raster applies it after the meshes and CSG and
// before the field layers, so slices, volumes and lines draw over it. A
// pixel whose sample is no-data keeps its shaded colour and is counted.
// GL refuses it until its pass lands (plan step 7).
// ---------------------------------------------------------------------------

#include "core/error.hpp"
#include "core/flow_volume.hpp"
#include "render/field_layer.hpp"
#include "render/transfer_function.hpp"

namespace spade::render {

// The map's 256 entries as palette stops (0-255 per channel).
[[nodiscard]] FieldPalette field_palette_of(ColourMapId id);

// `geometry`'s rectangle, cells and colour map, with its values sampled
// from `volume`. invalid_argument, by name: a volume that fails
// validate_flow_volume(), or a result that fails validate_field_layer().
[[nodiscard]] Result<FieldLayer> slice_flow_volume(const FlowVolume& volume, FlowQuantity quantity,
                                                   const FieldLayer& geometry);

struct FlowSurfaceColouring {
    const FlowVolume* volume = nullptr;  // the caller's, outliving every render() that reads it
    TransferFunction colours;            // its quantity, colours and range; extinction unused
};

// invalid_argument, by name: no volume, a volume that fails
// validate_flow_volume(), or colours that fail validate_transfer_function().
[[nodiscard]] Result<void> validate_flow_surface_colouring(const FlowSurfaceColouring& colouring);

}  // namespace spade::render
