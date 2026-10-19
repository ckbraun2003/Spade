// ---------------------------------------------------------------------------
// render/flow_look.hpp -- one look for the airflow views (spade::render).
//
// The helpers that give the recorded films (tools/flow_movie) and the
// sandbox's live tour one look by construction rather than by copy (the tour
// storyboard, Interface, 2026-10-13): a transfer function over a fixed range,
// with or without extinction; a coloured slice; streamlines coloured by
// speed; a rotor's rim. Each puts the engine's own views into a scene or
// returns a value. None draws, and none makes a picture the views would not
// draw.
// ---------------------------------------------------------------------------
#pragma once

#include <cstdint>
#include <span>

#include <glm/vec3.hpp>

#include "core/error.hpp"
#include "core/flow_volume.hpp"
#include "render/flow_field.hpp"
#include "render/flow_lines.hpp"
#include "render/flow_render.hpp"
#include "render/scene.hpp"
#include "render/transfer_function.hpp"

namespace spade::render {

// A transfer function over [0, range_max] in `map`, absorbing from `onset` of
// the range (0..1) up to `sigma` per metre at its top: a volume view.
[[nodiscard]] TransferFunction volume_transfer(FlowQuantity quantity, ColourMapId map, float range_max, float onset,
                                               float sigma);

// A colour-only transfer function over [range_min, range_max]: colouring an
// iso-surface or a streamline.
[[nodiscard]] TransferFunction colours_over(FlowQuantity quantity, ColourMapId map, float range_min, float range_max);

// Streamlines traced from `seeds` through `volume` (trace_streamlines()),
// each coloured by its speed through `by_speed`, or in `constant` when given,
// appended to scene.flow_polylines. A line of fewer than two points is
// skipped. Whatever trace_streamlines() refuses, it refuses.
[[nodiscard]] Result<void> add_streamlines(RenderScene& scene, const FlowVolume& volume,
                                           std::span<const glm::vec3> seeds, const StreamlineOptions& options,
                                           const TransferFunction& by_speed, const glm::vec3* constant = nullptr);

// The same, from lines already traced in a volume of `header` (a ViewCache's,
// render/view_mode.hpp): add_streamlines() is trace_streamlines() then this.
void add_traced_streamlines(RenderScene& scene, const FlowVolumeHeader& header, std::span<const Streamline> lines,
                            const TransferFunction& by_speed, const glm::vec3* constant = nullptr);

// A slice of `quantity` over `geometry` (slice_flow_volume()), coloured in
// `map` over [range_min, range_max] in 128 bins, appended to
// scene.field_layers. Whatever slice_flow_volume() refuses, it refuses.
[[nodiscard]] Result<void> add_slice(RenderScene& scene, const FlowVolume& volume, FlowQuantity quantity,
                                     FieldLayer geometry, ColourMapId map, float range_min, float range_max);

// A circle of `radius` about `axis` through `centre`, closed (its last point
// is its first), in `colour`: a rotor's rim. Built with math::cos32 and
// math::sin32, so every platform draws the same points (TD-3).
[[nodiscard]] FlowPolyline circle(glm::vec3 centre, glm::vec3 axis, float radius, glm::vec3 colour,
                                  uint32_t segments = 96);

}  // namespace spade::render
