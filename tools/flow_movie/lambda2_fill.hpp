#pragma once

// A stand-in until Publish fills the lambda2 channel (it writes 0 there
// today): lambda2 from the published velocity itself, by core's
// lambda2_hat(), so the quad video can draw vortex surfaces now. Captions
// call it "lambda2 from the published velocity". When Publish fills the
// channel, the recorder uses Publish's and this goes.

#include "core/flow_volume.hpp"

namespace flow_movie {

// Writes kFlowLambda2Hat for every cell from central differences of the
// cell-centred velocity, one-sided at the box's faces, and updates the
// channel's header range. A cell whose stencil holds a no-data velocity is
// written no-data and counted.
void fill_lambda2(spade::FlowVolume& volume);

}  // namespace flow_movie
