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
// GL draws it in its mesh and CSG passes, banded against the CPU.
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
    // RND-8, the C_p view. walls_only colours only draw items flagged
    // flow_wall; otherwise every surface in the box takes the colour. The
    // analytic ground takes it on neither path.
    bool walls_only = false;
    // Where the value is read: offset_cells volume cells off the surface,
    // along its normal turned toward the eye, so a wall's pressure comes from
    // the fluid on the seen side, not from cells across it (a thin part, or
    // a sheet, keeps each side's own). 0 reads at the surface. A face normal
    // on meshes, the distance gradient on CSG.
    float offset_cells = 0.0f;
    // With an offset, read at delta and 2 delta and take 2 v(delta) - v(2
    // delta): the value extrapolated linearly to the wall. On a curved wall a
    // single read is biased by dp/dn = rho u_t^2 / R; this removes the
    // first-order part (R2.4: at 8 cells per radius, 0.43 in C_p becomes
    // about 0.2) for up to three times a read's noise. Meant for pressure.
    bool extrapolate = false;
};
inline constexpr float kMaxSurfaceOffsetCells = 4.0f;

// invalid_argument, by name: no volume, a volume that fails
// validate_flow_volume(), colours that fail validate_transfer_function(), an
// offset that is not finite in [0, kMaxSurfaceOffsetCells], or extrapolate
// with no offset to extrapolate from.
[[nodiscard]] Result<void> validate_flow_surface_colouring(const FlowSurfaceColouring& colouring);

// The C_p view's colours (RND-8): a fixed-range pressure transfer function
// over [p_ref + cp_min q_ref, p_ref + cp_max q_ref], so the map reads
// C_p = (p - p_ref) / q_ref. The host passes q_ref, the reference dynamic
// pressure in Pa, and p_ref in Pa (0 is the outlets' ambient). spade::
// analysis' reference_speed() gives the speed: the tip for rotorcraft, the
// freestream for a propeller. Render chooses neither. invalid_argument, by
// name: q_ref not finite and > 0, p_ref not finite, a C_p range not finite
// with min < max, or a pressure range that collapses in float.
[[nodiscard]] Result<TransferFunction> cp_colours(float q_ref, float p_ref, float cp_min, float cp_max,
                                                  ColourMapId map = ColourMapId::cool_warm);

}  // namespace spade::render
