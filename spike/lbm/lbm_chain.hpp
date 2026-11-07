#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <string_view>

#include "bindings.gen.hpp"
#include "spike/airflow/chain_runner.hpp"

// ---------------------------------------------------------------------------
// spike/lbm/lbm_chain.hpp -- the LBM tier's device buffers (L1, the GPU twin,
// docs/design/physics/plans/2026-11-06-l1-gpu-twin-plan.md, T1).
//
// The LBM kernels bind bindings.slang section F's bindings 40 and 41 (set 1;
// 28 to 33 are module API stage 7's appendices, 34 to 39 spare). The airflow chain runner binds
// buffers[i] at binding_base + i, so an LBM runner is built with
// binding_base = kLbmBindingBase and its buffers in the order below: its
// descriptor set layout holds only those bindings (stage 7's per-kernel subset
// layout, decision 1), and set 0 is empty. T2 onward add the chain itself.
// ---------------------------------------------------------------------------

namespace spade::lbm_gpu {

using airflow_gpu::BufferKind;

struct LbmSetBuffer {
    std::string_view name;
    uint32_t binding = 0;
    BufferKind kind = BufferKind::device_only;
};

inline constexpr uint32_t kLbmBindingBase = compute::gen::kBinding_lbm_f;

// The populations (resident, read back for the digest) and the velocity.
inline constexpr std::array<LbmSetBuffer, 2> kLbmSetBuffers{{
    {"lbm_f", compute::gen::kBinding_lbm_f, BufferKind::staged},
    {"lbm_u", compute::gen::kBinding_lbm_u, BufferKind::staged},
}};

static_assert(
    [] {
        for (uint32_t i = 0; i < kLbmSetBuffers.size(); ++i) {
            if (kLbmSetBuffers[i].binding != kLbmBindingBase + i) return false;
        }
        return true;
    }(),
    "kLbmSetBuffers is in binding order from lbm_f: entry i is binding kLbmBindingBase + i");

// The LBM kernels' push block: lbm_params.slang's LbmPush, field for field (the
// dump kernel echoes it and GpuLbmLattice.ThePushBlockReadsBackWordForWord
// compares). 25 words, 100 bytes, under kMaxChainPushBytes.
struct LbmPush {
    uint32_t nx = 0;
    uint32_t ny = 0;
    uint32_t nz = 0;
    uint32_t parity = 0;  // the step counter's low bit
    float nu = 0.0f;
    float cw = 0.5f;
    uint32_t periodic_mask = 7;                 // bit a set: axis a is periodic
    std::array<float, 18> wall_u{};             // [(axis * 2 + side) * 3 + component]
};

inline constexpr uint32_t kLbmPushWords = 25;
static_assert(sizeof(LbmPush) == 4 * kLbmPushWords && sizeof(LbmPush) <= compute::kMaxChainPushBytes,
              "LbmPush is 25 words and fits a chain command's push block");
static_assert(offsetof(LbmPush, nu) == 16 && offsetof(LbmPush, periodic_mask) == 24 && offsetof(LbmPush, wall_u) == 28,
              "LbmPush's offsets are lbm_params.slang's");

}  // namespace spade::lbm_gpu
