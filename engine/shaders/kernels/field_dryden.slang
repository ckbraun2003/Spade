// ---------------------------------------------------------------------------
// field_dryden.slang (module-API stage 3) -- the dryden.sample pass: each
// world's wind, the mean wind plus this substep's gust, written into its field
// sample row. physics/schedule.cpp's pass_dryden_sample(), one thread per
// WORLD, after medium_update advanced the filter (a barrier between).
//
// The expression is dryden_medium_sample()'s (dryden.slang), so a reader of
// the stored wind sees the bits that reader used to compute inline.
// ---------------------------------------------------------------------------

import layouts;
import bindings;
import dryden;

[shader("compute")]
[numthreads(SPADE_WG, 1, 1)]
void field_dryden(uint3 tid : SV_DispatchThreadID) {
    const uint world = tid.x;
    if (world >= pass_params.world_count) {
        return;
    }
    // Only this pass's field: gravity and density belong to environment.sample.
    field_samples[world].wind = world_params[world].wind + dryden_turbulence(dryden[world], dryden_params[world]);
}
