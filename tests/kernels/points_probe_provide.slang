// points_probe_provide.slang -- the TEST provider of the point-sample tests (module API stage 7, Task D1): the device twin of
// tests/point_samples_gpu_support.hpp's point_provide_cpu(). One thread per point row: the module places its own element rows at
// (i, i, i) for row i (Core's positions pass writes body rows only, L43), then every row's wind.x = 2 * pos.y and density =
// pos.x. All exact in fp32, so the device rows equal the CPU's bit for bit. Rows per world ride in `sweep`, k in `level`.
import layouts;
import bindings;

[shader("compute")]
[numthreads(SPADE_WG, 1, 1)]
void points_probe_provide(uint3 tid : SV_DispatchThreadID) {
    const uint rows = pass_params.sweep;
    if (rows == 0u || tid.x >= pass_params.world_count * rows) {
        return;
    }
    const uint local = tid.x % rows;
    const uint base = tid.x * 8u;
    if (local >= pass_params.body_capacity && local < pass_params.body_capacity + pass_params.element_capacity * pass_params.level) {
        const float f = float(local);
        airflow_points[base + 0u] = f;
        airflow_points[base + 1u] = f;
        airflow_points[base + 2u] = f;
    }
    airflow_points[base + 4u] = 2.0f * airflow_points[base + 1u];
    airflow_points[base + 7u] = airflow_points[base + 0u];
}
