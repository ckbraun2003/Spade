// ---------------------------------------------------------------------------
// field_environment.slang (module-API stage 3) -- the environment.sample pass:
// gravity and density, copied from each world's WorldParams row into its
// field sample row. physics/schedule.cpp's pass_environment_sample(), one
// thread per WORLD. Copies, not arithmetic, so the row matches the CPU's bit
// for bit.
// ---------------------------------------------------------------------------

import layouts;
import bindings;

[shader("compute")]
[numthreads(SPADE_WG, 1, 1)]
void field_environment(uint3 tid : SV_DispatchThreadID) {
    const uint world = tid.x;
    if (world >= pass_params.world_count) {
        return;
    }
    FieldSampleRow row = field_samples[world];
    row.gravity = world_params[world].gravity;
    row.density = world_params[world].air_density;
    field_samples[world] = row;
}
