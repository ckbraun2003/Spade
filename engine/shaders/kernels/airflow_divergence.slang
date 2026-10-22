// ---------------------------------------------------------------------------
// airflow_divergence.slang (plan B, task B2) -- the projection's right side:
// pressure.cpp's divergence() and project()'s b = b_coef * div, its CPU twin
// and the authority for every expression. For every cell,
//   airflow_rhs = b_coef * (((u_{i+1} - u_i) + (v_{j+1} - v_j) + (w_{k+1} - w_k)) * inv_dx)
// with b_coef = rho dx^2 / h and inv_dx = 1 / dx (airflow_params_layout), each
// the host's float from the CPU's own expression (projection_floats()).
//
// Plan A's projection, without solids (the solids path's aperture divergence
// is B5's). One invocation per cell; the push's count (word 2) is the grid's
// cells. Every operation is a correctly rounded add or multiply, so b is the
// CPU's bit for bit.
// ---------------------------------------------------------------------------

import bindings;
import airflow_params_layout;

struct DivergencePush {
    uint unused0;
    uint unused1;
    uint count;  // the grid's cells
    uint unused2;
};

[[vk::push_constant]] ConstantBuffer<DivergencePush> divergence_push;

[shader("compute")]
[numthreads(SPADE_WG, 1, 1)]
void airflow_divergence(uint3 tid : SV_DispatchThreadID) {
    const uint c = tid.x;
    if (c >= divergence_push.count) { return; }
    const uint nx = grid_dims().x;
    const uint ny = grid_dims().y;
    const float inv_dx = grid_inv_dx();
    const float scale = b_coef();
    const uint i = c % nx;
    const uint j = (c / nx) % ny;
    const uint k = c / (nx * ny);
    // mac_grid.hpp's u_index, v_index and w_index.
    const uint u0 = i + (nx + 1u) * (j + ny * k);
    const uint v0 = i + nx * (j + (ny + 1u) * k);
    const uint w0 = c;
    const float du = airflow_u[u0 + 1u] - airflow_u[u0];
    const float dv = airflow_v[v0 + nx] - airflow_v[v0];
    const float dw = airflow_w[w0 + nx * ny] - airflow_w[w0];
    airflow_rhs[c] = scale * (((du + dv) + dw) * inv_dx);
}
