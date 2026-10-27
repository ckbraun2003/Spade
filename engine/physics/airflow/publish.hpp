#pragma once

#include <array>
#include <cstdint>
#include <span>

#include <glm/glm.hpp>

#include "core/error.hpp"
#include "core/flow_volume.hpp"
#include "physics/airflow/mac_grid.hpp"

// ---------------------------------------------------------------------------
// PUBLISH: the render volume (spec §5; Task 8b, the lead 2026-10-12). Fills
// Core's spade::FlowVolume from the MAC grid at cell centres, one frame at a
// time, for Rendering's views (and Interface's harness, which calls it per
// frame).
//   V = (u, v, w, dye): the face velocities averaged to the centre; dye is 0
//       until the solver carries the dye scalar (the demonstration plan, P10).
//   S = (p gauge, |omega|, q-hat, lambda2-hat). The velocity gradient G at the
//       centre takes its diagonal from the cell's own faces and its off-
//       diagonal from central differences of the centred velocities (wrapped
//       on a periodic axis, one-sided at any other axis' edges); omega is its curl, Q = |omega|^2 / 4 - |S|^2 / 2
//       with S its symmetric part, and q-hat = sign(Q) sqrt|Q|. sqrt only
//       (TD-3).
// Every value goes through spade::fp16::from_f32: a NaN, an infinity or
// |x| >= 65520 is written as fp16::kNoDataBits, counted in header.no_data and
// left out of the channel's range (L6).
//
// SOLID CELLS (the demonstration plan, P6). With a solid mask, the first layer
// of solid cells -- any solid cell with a fluid face-neighbour -- takes the
// mean of its fluid face-neighbours' channels (faces -x, +x, -y, +y, -z, +z;
// across the wrap on a periodic axis),
// so a surface sampled for C_p reads the wall's pressure; deeper solid cells
// are no-data in every channel.
//
// CLOSED FACES (the solids design, section 6; P3, S5). With the apertures
// given, a velocity gradient along an axis never differences across a closed
// face, a wall's or a sheet's: it goes one-sided from the cell, as at a
// non-periodic domain edge, and is 0 with both faces closed. A central
// difference across a sheet would mix its two sides' velocities and draw
// vorticity, Q and lambda2 along a sheet that has none.
// ---------------------------------------------------------------------------
namespace spade::physics::airflow {

struct PublishInput {
    glm::vec3 origin{0.0f};            // the grid's minimum corner in the world, metres
    std::span<const uint8_t> solid{};  // one per cell, nonzero = solid; empty = all fluid
    std::array<bool, 3> periodic{};    // per axis: the grid wraps there, so its edge differences wrap too
    // The face apertures (SolidField::theta), per component; empty: every face
    // open. A difference never crosses a closed face (theta = 0).
    std::array<std::span<const float>, 3> theta{};
};

struct PublishCounts {
    uint32_t extrapolated = 0;  // solid cells written from their fluid side (also header.extrapolated_cells)
};

// Refused (invalid_argument): a solid mask, a grid array or a non-empty
// aperture array whose size does not match the grid's shape. `out`'s storage is reused when its size fits.
// With subnormals flushed as fluid_step() flushes them (fluid_step.hpp).
[[nodiscard]] Result<PublishCounts> publish_render_volume(const MacGrid& g, const PublishInput& in, FlowVolume& out);

// THE HEADER FROM THE HALVES (plan B, B6; the lead, 2026-10-26): what
// publish_render_volume() puts in the header, from the halves alone, by the
// same rule in the same place (TD-9), for halves packed elsewhere (the GPU's
// pack_f16, spike/airflow/publish_chain.hpp). Per channel, the minimum and
// maximum of the stored halves (fp16::to_f32) that are not no-data, in cell
// order with `<` and `>`, so a tie of -0 and +0 keeps the first; 0 and 0 for
// a channel with none. no_data counts the kNoDataBits halves: exactly the
// halves from_f32 calls no data, since its every other result is finite
// (fp16.hpp). origin, cell, the dims, extrapolated_cells and the generation
// are left as they are. Refused (invalid_argument): v or s not nx ny nz x
// kFlowHalvesPerCell halves.
[[nodiscard]] Result<void> flow_header_from_halves(FlowVolume& volume);

// The solid cells publish_render_volume() writes from their fluid side, the
// first solid layer (PublishCounts::extrapolated), by its own neighbour walk:
// a solid cell with a fluid face-neighbour, across the wrap on a periodic
// axis. It depends on the mask and the periodic axes only, never on the flow,
// so a caller counts it once per mask. Refused (invalid_argument): a solid
// mask whose size does not match the shape's.
[[nodiscard]] Result<uint32_t> extrapolated_cells(const GridShape& shape, const PublishInput& in);

}  // namespace spade::physics::airflow
