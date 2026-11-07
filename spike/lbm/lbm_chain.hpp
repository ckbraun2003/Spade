#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <string_view>
#include <vector>

#include "bindings.gen.hpp"
#include "physics/lbm/lbm_flow.hpp"
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

// The kernels of an LBM step runner, in the order RunnerDesc::kernels lists them: pipeline 0 is lbm_macro (the
// velocity pass), pipeline 1 lbm_collide_stream. lbm_lattice_dump (T1) is a runner of its own.
inline constexpr uint32_t kLbmMacroPipeline = 0;
inline constexpr uint32_t kLbmCollidePipeline = 1;

// The push block of a CPU flow: its grid, parity (the step counter's low bit), nu and cw, which axes are periodic and
// the walls' velocities, flattened as lbm_params.slang says: wall_u[(axis * 2 + side) * 3 + component].
[[nodiscard]] inline LbmPush lbm_push_of(const physics::lbm::LbmFlow& fl) {
    LbmPush p;
    p.nx = fl.grid.nx;
    p.ny = fl.grid.ny;
    p.nz = fl.grid.nz;
    p.parity = static_cast<uint32_t>(fl.grid.step & 1u);
    p.nu = fl.nu;
    p.cw = fl.cw;
    p.periodic_mask = 0;
    for (uint32_t a = 0; a < 3; ++a) p.periodic_mask |= fl.walls.periodic[a] ? (1u << a) : 0u;
    for (uint32_t a = 0; a < 3; ++a) {
        for (uint32_t side = 0; side < 2; ++side) {
            for (uint32_t k = 0; k < 3; ++k) p.wall_u[(a * 2 + side) * 3 + k] = fl.walls.wall_u[a][side][k];
        }
    }
    return p;
}

// `steps` LBM steps as dispatches, starting at step counter `first_step`: for each step the velocity pass and then
// the collide-and-stream pass (the chain puts a barrier between the two, which is the CPU's pool return), the push
// block's parity alternating. Groups are (ceil(nx / workgroup), ny, nz), a thread an x. With `macro_only`, just the
// velocity pass of each step (the T2 test).
[[nodiscard]] inline std::vector<compute::ChainCommand> lbm_step_chain(LbmPush push, uint64_t first_step,
                                                                       uint32_t steps, uint32_t workgroup,
                                                                       bool macro_only = false) {
    std::vector<compute::ChainCommand> chain;
    const std::array<uint32_t, 3> groups{(push.nx + workgroup - 1) / workgroup, push.ny, push.nz};
    for (uint32_t s = 0; s < steps; ++s) {
        push.parity = static_cast<uint32_t>((first_step + s) & 1u);
        for (const uint32_t pipeline : {kLbmMacroPipeline, kLbmCollidePipeline}) {
            if (macro_only && pipeline == kLbmCollidePipeline) continue;
            compute::ChainCommand c{.is_dispatch = true, .pipeline = pipeline, .groups = groups};
            std::memcpy(c.push.data(), &push, sizeof(push));
            chain.push_back(c);
        }
    }
    return chain;
}

}  // namespace spade::lbm_gpu
