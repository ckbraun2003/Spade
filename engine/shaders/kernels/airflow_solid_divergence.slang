// ---------------------------------------------------------------------------
// airflow_solid_divergence.slang (plan B, task B5.2) -- the solids
// projection's right side: projection.cpp's solid_divergence(), its CPU twin
// and the authority for every expression. For every cell in a region,
//   b = b_coef * (((du + dv) + dw) * inv_dx),
// du = flux(u_{i+1}) - flux(u_i) and so on, of the aperture flux
//   flux = theta vel + (1 - theta) wall
// (theta and the wall velocity per face from the solid field, airflow_pcg.slang's
// packing), and b = 0 in a cell of no region (a solid cell). b_coef and inv_dx
// are the host's floats (airflow_params_layout). Written to airflow_pcg's b,
// before the region means come off (B5.3).
//
// One invocation per cell; the push's count (word 2) is the grid's cells.
// Every operation is a correctly rounded add, subtract or multiply,
// uncontracted (-fp-mode precise), in the CPU's order, so b is the CPU's bit
// for bit. Beside B2's airflow_divergence, which stays plan A's.
//
// SET 1, bindings.slang section F: reads airflow_params, airflow_u, _v, _w,
// airflow_apertures (13), airflow_wall_velocity (14) and
// airflow_region_labels (18); writes airflow_pcg (20).
// ---------------------------------------------------------------------------

import bindings;
import airflow_params_layout;
import airflow_pcg;

struct SolidDivergencePush {
    uint unused0;
    uint unused1;
    uint count;  // the grid's cells
    uint unused2;
};

[[vk::push_constant]] ConstantBuffer<SolidDivergencePush> solid_divergence_push;

float velocity(uint c, uint idx) {
    if (c == 0u) return airflow_u[idx];
    if (c == 1u) return airflow_v[idx];
    return airflow_w[idx];
}

// projection.cpp's flux(): theta vel + (1 - theta) wall, at face idx of
// component c.
float flux(uint c, uint idx) {
    const float t = face_theta(c, idx);
    return t * velocity(c, idx) + (1.0f - t) * face_wall(c, idx);
}

[shader("compute")]
[numthreads(SPADE_WG, 1, 1)]
void airflow_solid_divergence(uint3 tid : SV_DispatchThreadID) {
    const uint c = tid.x;
    if (c >= solid_divergence_push.count) { return; }
    if (solid_cell(c)) {
        pcg_set(kPcgB, c, 0.0f);
        return;
    }
    const uint nx = grid_dims().x;
    const uint ny = grid_dims().y;
    const uint i = c % nx;
    const uint j = (c / nx) % ny;
    const uint k = c / (nx * ny);
    // mac_grid.hpp's u_index, v_index and w_index.
    const uint u0 = i + (nx + 1u) * (j + ny * k);
    const uint v0 = i + nx * (j + (ny + 1u) * k);
    const uint w0 = c;
    const float du = flux(0u, u0 + 1u) - flux(0u, u0);
    const float dv = flux(1u, v0 + nx) - flux(1u, v0);
    const float dw = flux(2u, w0 + nx * ny) - flux(2u, w0);
    pcg_set(kPcgB, c, b_coef() * (((du + dv) + dw) * grid_inv_dx()));
}
