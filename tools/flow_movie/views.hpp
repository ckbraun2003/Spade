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
#include "render/flow_look.hpp"
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

// The look the films share with the sandbox's tour (render/flow_look.hpp).
using spade::render::add_slice;
using spade::render::add_streamlines;
using spade::render::circle;
using spade::render::colours_over;
using spade::render::volume_transfer;

}  // namespace flow_movie
