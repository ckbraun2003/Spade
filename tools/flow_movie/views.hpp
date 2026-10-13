#pragma once

// The recorder's shots: helpers that put a published volume's views into a
// scene (iso-surfaces, a volume, streamlines, a slice, rotor rings) and aim
// an orbiting camera. Each helper draws through the engine's own views; none
// invents a picture the views would not draw.

#include <cstdint>
#include <span>
#include <string>
#include <vector>

#include <glm/vec3.hpp>

#include "core/error.hpp"
#include "core/flow_volume.hpp"
#include "render/flow_field.hpp"
#include "render/flow_lines.hpp"
#include "render/flow_render.hpp"
#include "render/scene.hpp"
#include "render/target.hpp"
#include "render/transfer_function.hpp"

namespace flow_movie {

// An empty scene for flow views: a dark sky, the volume's box as its bounds.
[[nodiscard]] spade::render::RenderScene dark_scene(const spade::FlowVolume& volume);

// The box's centre and its far corner, in the world.
[[nodiscard]] glm::vec3 volume_centre(const spade::FlowVolume& volume);
[[nodiscard]] glm::vec3 volume_size(const spade::FlowVolume& volume);

// A camera `distance` from `target`, at an azimuth about +y (0: on +z) and an
// elevation above the horizontal, both in degrees, looking at the target.
[[nodiscard]] spade::render::Camera orbit(glm::vec3 target, float distance, float azimuth_deg, float elevation_deg);

// A transfer function over a fixed range, absorbing from `onset` of the range
// up to `sigma` per metre at its top.
[[nodiscard]] spade::render::TransferFunction volume_transfer(spade::render::FlowQuantity quantity,
                                                              spade::render::ColourMapId map, float range_max,
                                                              float onset, float sigma);

// A colour-only transfer function over [range_min, range_max].
[[nodiscard]] spade::render::TransferFunction colours_over(spade::render::FlowQuantity quantity,
                                                           spade::render::ColourMapId map, float range_min,
                                                           float range_max);

// Streamlines from `seeds`, each coloured by its speed through `by_speed`,
// or in `constant` when given.
[[nodiscard]] spade::Result<void> add_streamlines(spade::render::RenderScene& scene, const spade::FlowVolume& volume,
                                                  std::span<const glm::vec3> seeds,
                                                  const spade::render::StreamlineOptions& options,
                                                  const spade::render::TransferFunction& by_speed,
                                                  const glm::vec3* constant = nullptr);

// A slice of one quantity, `cells_u` by `cells_v`, coloured over a fixed range.
[[nodiscard]] spade::Result<void> add_slice(spade::render::RenderScene& scene, const spade::FlowVolume& volume,
                                            spade::render::FlowQuantity quantity, spade::render::FieldLayer geometry,
                                            spade::render::ColourMapId map, float range_min, float range_max);

// A circle of `radius` about `axis` through `centre`: a rotor's rim.
[[nodiscard]] spade::render::FlowPolyline circle(glm::vec3 centre, glm::vec3 axis, float radius, glm::vec3 colour,
                                                 uint32_t segments = 96);

}  // namespace flow_movie
