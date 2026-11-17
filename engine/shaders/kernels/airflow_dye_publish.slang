// ---------------------------------------------------------------------------
// airflow_dye_publish.slang (P10) -- the dye into publish's V channel 3, on the GPU: engine/physics/airflow/publish.cpp's
// fluid_channels() with PublishInput::dye, and its solid cells' mean of the fluid neighbours' channels, term for term, for
// the one channel. Runs after airflow_publish_fields, which has written the eight fp32 channels a cell into airflow_advect
// (channel c of cell n at 8 n + c) with the dye channel 0; this kernel overwrites channel 3 only:
//   a fluid cell:                 its dye (the cur half of airflow_dye);
//   a solid cell with fluid face-neighbours: the mean of theirs, summed in the order -x, +x, -y, +y, -z, +z (across the
//                                 wrap on a periodic axis of more than one cell), times kInverse[count];
//   a solid cell with none:       left as airflow_publish_fields wrote it (no data).
// So a dye-less runner never runs it and a dye-less publish is airflow_publish_fields', bit for bit.
//
// Reads airflow_dye, airflow_region_labels (the solid mask) and airflow_params; writes airflow_advect. Publish's packing
// to halves (airflow_pack_f16) follows, unchanged.
// ---------------------------------------------------------------------------

import layouts;
import bindings;
import airflow_params_layout;

struct DyePublishPush {
    uint unused0;
    uint unused1;
    uint cell_count;
    uint unused2;
};

[[vk::push_constant]] ConstantBuffer<DyePublishPush> dye_publish_push;

static const uint kDyeHeader = 4u;
static const uint kChannels = 8u;
static const uint kNoRegion = 0xFFFFFFFFu;
bool cell_solid(uint cell) { return has_solids() && airflow_region_labels[cell] == kNoRegion; }

uint axis_cells(uint a) {
    const uint3 n = grid_dims();
    return a == 0u ? n.x : (a == 1u ? n.y : n.z);
}

uint cell_index(uint i, uint j, uint k) {
    const uint3 n = grid_dims();
    return i + n.x * (j + n.y * k);
}

static const float kInverse[7] = { 0.0, 1.0, 0.5, 0.3333333432674407958984375, 0.25,
                                   0.20000000298023223876953125, 0.16666667163372039794921875 };

[shader("compute")]
[numthreads(SPADE_WG, 1, 1)]
void airflow_dye_publish(uint3 tid : SV_DispatchThreadID) {
    const uint cell = tid.x;
    if (cell >= dye_publish_push.cell_count) { return; }
    if (!cell_solid(cell)) {
        airflow_advect[kChannels * cell + 3u] = airflow_dye[kDyeHeader + cell];
        return;
    }
    const uint3 n = grid_dims();
    const uint i = cell % n.x;
    const uint j = (cell / n.x) % n.y;
    const uint k = cell / (n.x * n.y);
    const uint at[3] = { i, j, k };
    float sum = 0.0;
    uint fluid = 0u;
    for (uint a = 0u; a < 3u; ++a) {
        const uint na = axis_cells(a);
        for (uint dir = 0u; dir < 2u; ++dir) {
            const bool edge = dir == 0u ? at[a] == 0u : at[a] + 1u == na;
            if (edge && !(axis_periodic(a) && na > 1u)) continue;
            uint nb[3] = { i, j, k };
            if (edge) {
                nb[a] = dir == 0u ? na - 1u : 0u;
            } else {
                nb[a] = dir == 0u ? at[a] - 1u : at[a] + 1u;
            }
            const uint idx = cell_index(nb[0], nb[1], nb[2]);
            if (cell_solid(idx)) continue;
            sum += airflow_dye[kDyeHeader + idx];
            ++fluid;
        }
    }
    if (fluid == 0u) return;  // no data: airflow_publish_fields' NaN stays
    airflow_advect[kChannels * cell + 3u] = sum * kInverse[fluid];
}
