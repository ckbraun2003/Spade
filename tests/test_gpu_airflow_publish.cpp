// ---------------------------------------------------------------------------
// test_gpu_airflow_publish.cpp -- plan B, task B6: publish on the GPU
// (engine/shaders/kernels/airflow_publish_fields.slang and
// airflow_pack_f16.slang) against its CPU twin, publish_render_volume()
// (engine/physics/airflow/publish.cpp), on the chain runner
// (spike/airflow/chain_runner.hpp).
//
//   GpuAirflowPublish   pack_f16 alone, every rounding case against
//                       fp16::from_f32, to the bit; the two kernels against
//                       publish_render_volume's halves on an open box, a
//                       periodic one and one with solids and closed faces:
//                       every channel to the bit on the device of record
//                       (|omega|, q-hat and lambda2-hat take fp32_math's
//                       exact sqrt and division; another device keeps their
//                       band); the same bits at workgroup sizes 32, 64 and
//                       128. B6's seam (spike/airflow/publish_chain.hpp):
//                       the whole FlowVolume read back from the device,
//                       header and all, publish_render_volume's on the
//                       device of record, on the three boxes and a fourth
//                       with a solid slab across a periodic edge and a NaN
//                       beside the block.
//   AirflowPublishHeader  (no device) flow_header_from_halves() and
//                       extrapolated_cells(), publish's header rule and
//                       neighbour walk: the header they make is publish's,
//                       field for field, on the four boxes; a range is the
//                       stored halves' in cell order (the tie of -0 and +0
//                       kept first), no-data counted per half; and from_f32
//                       calls a value no data exactly when its half is
//                       kNoDataBits.
//
// The box is 16 x 12 x 20 at 1/16 m, no two sizes alike. The velocity is a
// swirl about y with a shear and fixed-seed noise, so every channel varies;
// the solids are a 4^3 block (56 first-layer cells, 8 deeper) with its faces
// closed, and a sheet of closed x-faces through the fluid beside it.
//
// Every Gpu* test starts with the vulkan_skip_reason() skip (gpu_skip.hpp).
// ---------------------------------------------------------------------------

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <bit>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <vector>

#include "airflow_pack_f16.spv.gen.hpp"
#include "airflow_publish_fields.spv.gen.hpp"
#include "compute/backend.hpp"
#include "compute/spirv_variants.hpp"
#include "compute/vulkan/context.hpp"
#include "airflow_exact_device.hpp"
#include "compute/vulkan/dispatch_chain.hpp"
#include "core/flow_volume.hpp"
#include "core/fp16.hpp"
#include "gpu_skip.hpp"
#include "physics/airflow/mac_grid.hpp"
#include "physics/airflow/publish.hpp"
#include "physics/airflow/regions.hpp"
#include "spike/airflow/chain_runner.hpp"
#include "spike/airflow/publish_chain.hpp"

namespace {

namespace af = spade::physics::airflow;
namespace gen = spade::compute::gen;
using spade::airflow_gpu::BufferKind;
using spade::airflow_gpu::ChainRunner;
using spade::airflow_gpu::RunnerBuffer;
using spade::airflow_gpu::RunnerDesc;
using spade::compute::ChainCommand;
using spade::compute::SpirvVariantSet;
using spade::compute::VulkanContext;

constexpr af::GridShape kShape{16, 12, 20, 1.0f / 16.0f};
constexpr uint32_t kChannels = spade::kFlowChannelCount;

// Set 1's bindings the two kernels use (bindings.slang section F); the runner
// binds buffers[i] at binding i, so every binding up to the last is given.
constexpr uint32_t kParams = 0, kU = 1, kV = 2, kW = 3, kPressure = 4, kScratch = 6, kApertures = 13,
                   kSolid = 18, kPublishV = 25, kPublishS = 26, kBufferCount = 27;

// Two kernels, one push block, by the airflow push convention
// (airflow_params_layout.slang): {0, 0, cell_count, 0}.
struct CellPush {
    uint32_t unused0 = 0;
    uint32_t unused1 = 0;
    uint32_t cell_count = 0;
    uint32_t unused2 = 0;
};

constexpr uint32_t kFieldsPipeline = 0, kPackPipeline = 1;

// read_flow_volume() reads section F's bindings with the runner's
// binding_base 0, as this test binds them.
static_assert(kPublishV == gen::kBinding_airflow_publish_v && kPublishS == gen::kBinding_airflow_publish_s);

// The grid's minimum corner in the world: not the origin, so the header's
// copy of it is seen.
constexpr glm::vec3 kOrigin{0.5f, -1.25f, 2.0f};

// |omega|, q-hat and lambda2-hat take fp32_math's sqrt32_ftz and div32_ftz,
// and publish_render_volume() flushes as the device does, so on the device of
// record every half is the CPU's (TD-14 as amended 2026-10-22;
// airflow_exact_device.hpp). Another device or driver keeps their band, in
// binary16 steps: a last-bit change in fp32 can carry a half across its
// rounding point to its neighbour, one step; two would take an fp32 error of a
// whole half step (2^-11 of the value), a real error, not rounding. Measured
// 2026-10-21 on an NVIDIA GeForce RTX 3060 Ti with the device's own sqrt and
// division: |omega| and lambda2-hat one step in at most 1 of 3840 halves per
// box, q-hat none; u, v, w, dye and p to the bit on all three boxes
// (the solid layer's mean and the no-data cells included); the pack to the
// bit on 30720 values. Red check: the pack's ties rounded up, and the centred
// u's 1/2 as 1/4, fail the pack (the two ties) and the halves (every u).
constexpr uint32_t kBandHalfSteps = 1;

[[nodiscard]] std::unique_ptr<VulkanContext> make_context() {
    auto result = VulkanContext::create(spade::compute::BackendDesc{.kind = spade::compute::BackendKind::vulkan});
    if (!result.has_value()) return nullptr;
    return std::move(*result);
}

// splitmix64: a fixed-seed stream, the same on every compiler.
struct SplitMix {
    uint64_t state = 0;
    uint64_t next() noexcept {
        uint64_t z = (state += 0x9E3779B97F4A7C15ull);
        z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ull;
        z = (z ^ (z >> 27)) * 0x94D049BB133111EBull;
        return z ^ (z >> 31);
    }
    // Uniform in [-1, 1), from 24 bits.
    float symmetric() noexcept { return static_cast<float>(next() >> 40) * 0x1p-23f - 1.0f; }
};

[[nodiscard]] uint32_t bits(float f) noexcept { return std::bit_cast<uint32_t>(f); }

[[nodiscard]] std::size_t cell_count(const af::GridShape& s) { return std::size_t{s.nx} * s.ny * s.nz; }

[[nodiscard]] std::array<std::size_t, 3> face_counts(const af::GridShape& s) {
    return {std::size_t{s.nx + 1u} * s.ny * s.nz, std::size_t{s.nx} * (s.ny + 1u) * s.nz,
            std::size_t{s.nx} * s.ny * (s.nz + 1u)};
}

template <class T>
[[nodiscard]] std::span<const std::byte> bytes_of(const std::vector<T>& v) noexcept {
    return std::as_bytes(std::span<const T>(v));
}

template <class T>
[[nodiscard]] std::span<std::byte> writable_bytes_of(std::vector<T>& v) noexcept {
    return std::as_writable_bytes(std::span<T>(v));
}

// The swirl of the advection test, a pressure that varies on every axis, and
// noise on both.
[[nodiscard]] af::MacGrid swirl(const af::GridShape& s) {
    af::MacGrid g;
    g.shape = s;
    const auto n = face_counts(s);
    g.u.assign(n[0], 0.0f);
    g.v.assign(n[1], 0.0f);
    g.w.assign(n[2], 0.0f);
    g.p.assign(cell_count(s), 0.0f);
    const glm::vec3 centre = 0.5f * s.dx * glm::vec3(static_cast<float>(s.nx), static_cast<float>(s.ny),
                                                     static_cast<float>(s.nz));
    constexpr float kOmega = 25.0f;
    SplitMix rng{0x9B11511ull};
    af::for_each_sample(s, 0, [&](std::size_t i, glm::vec3 x) {
        g.u[i] = -kOmega * (x.z - centre.z) + rng.symmetric();
    });
    af::for_each_sample(s, 1, [&](std::size_t i, glm::vec3 x) {
        g.v[i] = 10.0f * (x.x - centre.x) + rng.symmetric();
    });
    af::for_each_sample(s, 2, [&](std::size_t i, glm::vec3 x) {
        g.w[i] = kOmega * (x.x - centre.x) + rng.symmetric();
    });
    for (uint32_t k = 0; k < s.nz; ++k) {
        for (uint32_t j = 0; j < s.ny; ++j) {
            for (uint32_t i = 0; i < s.nx; ++i) {
                g.p[af::cell_index(s, i, j, k)] = 40.0f * static_cast<float>(i) - 25.0f * static_cast<float>(j) +
                                                  10.0f * static_cast<float>(k) + 30.0f * rng.symmetric();
            }
        }
    }
    return g;
}

// A box to publish: its periodic axes, and optionally solids and apertures.
struct Box {
    const char* name = "";
    std::array<bool, 3> periodic{};
    std::vector<uint8_t> solid{};               // per cell; empty: all fluid
    std::array<std::vector<float>, 3> theta{};  // per component; empty: every face open
};

[[nodiscard]] bool in_block(uint32_t i, uint32_t j, uint32_t k) {
    return i >= 6u && i < 10u && j >= 4u && j < 8u && k >= 8u && k < 12u;
}

// A 4^3 solid block, every face touching it closed; a sheet of closed
// x-faces at i = 3 over j 2..9, k 2..17; and some faces half open, which
// close nothing (only theta = 0 does).
[[nodiscard]] Box solids_box(const af::GridShape& s) {
    Box b;
    b.name = "solids";
    b.solid.assign(cell_count(s), 0u);
    for (uint32_t k = 0; k < s.nz; ++k) {
        for (uint32_t j = 0; j < s.ny; ++j) {
            for (uint32_t i = 0; i < s.nx; ++i) b.solid[af::cell_index(s, i, j, k)] = in_block(i, j, k) ? 1u : 0u;
        }
    }
    const auto n = face_counts(s);
    for (uint32_t c = 0; c < 3; ++c) b.theta[c].assign(n[c], 1.0f);
    const auto solid_at = [&](int64_t i, int64_t j, int64_t k) {
        return i >= 0 && j >= 0 && k >= 0 && i < s.nx && j < s.ny && k < s.nz &&
               in_block(static_cast<uint32_t>(i), static_cast<uint32_t>(j), static_cast<uint32_t>(k));
    };
    for (uint32_t k = 0; k < s.nz; ++k) {
        for (uint32_t j = 0; j < s.ny; ++j) {
            for (uint32_t i = 0; i <= s.nx; ++i) {
                const bool closed = solid_at(i, j, k) || solid_at(int64_t{i} - 1, j, k) ||
                                    (i == 3u && j >= 2u && j < 10u && k >= 2u && k < 18u);
                b.theta[0][af::u_index(s, i, j, k)] = closed ? 0.0f : (i == 12u ? 0.5f : 1.0f);
            }
        }
    }
    for (uint32_t k = 0; k < s.nz; ++k) {
        for (uint32_t j = 0; j <= s.ny; ++j) {
            for (uint32_t i = 0; i < s.nx; ++i) {
                const bool closed = solid_at(i, j, k) || solid_at(i, int64_t{j} - 1, k);
                b.theta[1][af::v_index(s, i, j, k)] = closed ? 0.0f : 1.0f;
            }
        }
    }
    for (uint32_t k = 0; k <= s.nz; ++k) {
        for (uint32_t j = 0; j < s.ny; ++j) {
            for (uint32_t i = 0; i < s.nx; ++i) {
                const bool closed = solid_at(i, j, k) || solid_at(i, j, int64_t{k} - 1);
                b.theta[2][af::w_index(s, i, j, k)] = closed ? 0.0f : 1.0f;
            }
        }
    }
    return b;
}

[[nodiscard]] std::vector<Box> boxes(const af::GridShape& s) {
    std::vector<Box> out;
    out.push_back(Box{.name = "open"});
    out.push_back(Box{.name = "periodic", .periodic = {true, false, true}});
    out.push_back(solids_box(s));
    return out;
}

// The solids box on a periodic x and z, with a solid slab across the x edge
// (i 0..1, j 0..2, k 0..3): its six cells (0, 0..1, 0..2) reach fluid only
// across a wrap (x, and z at k = 0), so the first layer is counted only by a
// walk that wraps. The slab's faces stay open (theta only bends gradients).
[[nodiscard]] Box edge_box(const af::GridShape& s) {
    Box b = solids_box(s);
    b.name = "edge-nan";
    b.periodic = {true, false, true};
    for (uint32_t k = 0; k < 4u; ++k) {
        for (uint32_t j = 0; j < 3u; ++j) {
            for (uint32_t i = 0; i < 2u; ++i) b.solid[af::cell_index(s, i, j, k)] = 1u;
        }
    }
    return b;
}

// The swirl with a NaN on the u face between cells (4, 5, 9) and (5, 5, 9):
// cell (5, 5, 9) and its neighbours' gradients are NaN, and so is the mean
// its solid neighbour (6, 5, 9) takes, which is no-data in every channel and
// still counted as extrapolated.
[[nodiscard]] af::MacGrid nan_swirl(const af::GridShape& s) {
    af::MacGrid g = swirl(s);
    g.u[af::u_index(s, 5, 5, 9)] = std::bit_cast<float>(0x7FC00000u);
    return g;
}

// The four cases the seam's tests run: the three boxes on the swirl, and the
// edge box on the NaN swirl.
struct Case {
    af::MacGrid g;
    Box box;
};

[[nodiscard]] std::vector<Case> seam_cases(const af::GridShape& s) {
    std::vector<Case> out;
    for (Box& b : boxes(s)) out.push_back(Case{swirl(s), std::move(b)});
    out.push_back(Case{nan_swirl(s), edge_box(s)});
    return out;
}

// The box as publish's input; its spans point into `box`.
[[nodiscard]] af::PublishInput input_of(const Box& box) {
    af::PublishInput in;
    in.origin = kOrigin;
    in.solid = box.solid;
    in.periodic = box.periodic;
    for (uint32_t c = 0; c < 3; ++c) in.theta[c] = box.theta[c];
    return in;
}

[[nodiscard]] spade::FlowVolume publish_on_cpu(const af::MacGrid& g, const Box& box) {
    const af::PublishInput in = input_of(box);
    spade::FlowVolume out;
    const auto done = af::publish_render_volume(g, in, out);
    EXPECT_TRUE(done.has_value()) << box.name << ": " << done.error().context;
    return out;
}

// What the device holds after the chain: the fp32 channels (8 per cell) and
// the halves, as FlowVolume::v and ::s hold them; after the full chain, the
// volume read_flow_volume() makes of them.
struct Published {
    std::vector<float> channels;
    std::vector<uint16_t> v, s;
    spade::FlowVolume volume;
};

[[nodiscard]] ChainCommand dispatch(uint32_t pipeline, uint32_t cells, uint32_t workgroup_size) {
    ChainCommand c{.is_dispatch = true, .pipeline = pipeline};
    const CellPush push{.cell_count = cells};
    std::memcpy(c.push.data(), &push, sizeof push);
    c.groups = {(cells + workgroup_size - 1u) / workgroup_size, 1u, 1u};
    return c;
}

// The two kernels on the device; with `channels`, the scratch starts as them
// and only the pack runs.
[[nodiscard]] std::optional<Published> publish_on_gpu(VulkanContext& ctx, const af::MacGrid& g, const Box& box,
                                                      uint32_t workgroup_size,
                                                      const std::vector<float>* channels = nullptr) {
    const af::GridShape& s = g.shape;
    const std::size_t cells = cell_count(s);
    const auto n = face_counts(s);
    const bool has_theta = !box.theta[0].empty();
    // airflow_params: Physics' 16 words (the advection offsets unused here),
    // then publish's flags (bit 0 solids, bit 1 apertures).
    std::vector<uint32_t> params(17u, 0u);
    params[0] = s.nx;
    params[1] = s.ny;
    params[2] = s.nz;
    params[4] = bits(s.dx);
    params[5] = bits(1.0f / s.dx);
    for (uint32_t a = 0; a < 3; ++a) params[8 + a] = box.periodic[a] ? 1u : 0u;
    params[16] = (box.solid.empty() ? 0u : 1u) | (has_theta ? 2u : 0u);
    std::vector<float> theta;
    for (const auto& t : box.theta) theta.insert(theta.end(), t.begin(), t.end());
    // The solid mask as region labels: kNoRegion where solid, a region elsewhere.
    std::vector<uint32_t> solid(box.solid.size(), 0u);
    for (std::size_t c = 0; c < solid.size(); ++c) solid[c] = box.solid[c] != 0u ? af::kNoRegion : 0u;

    std::vector<RunnerBuffer> buffers(kBufferCount);
    for (uint32_t b = 0; b < kBufferCount; ++b) {
        buffers[b] = {"airflow_unused_" + std::to_string(b), 4u, BufferKind::device_only};
    }
    buffers[kParams] = {"airflow_params", params.size() * 4u, BufferKind::staged};
    buffers[kU] = {"airflow_u", n[0] * 4u, BufferKind::resident};
    buffers[kV] = {"airflow_v", n[1] * 4u, BufferKind::resident};
    buffers[kW] = {"airflow_w", n[2] * 4u, BufferKind::resident};
    buffers[kPressure] = {"airflow_pressure", cells * 4u, BufferKind::resident};
    buffers[kScratch] = {"airflow_advect", kChannels * cells * 4u, BufferKind::staged};
    if (has_theta) buffers[kApertures] = {"airflow_apertures", theta.size() * 4u, BufferKind::resident};
    if (!solid.empty()) buffers[kSolid] = {"airflow_region_labels", solid.size() * 4u, BufferKind::resident};
    buffers[kPublishV] = {"airflow_publish_v", 2u * cells * 4u, BufferKind::staged};
    buffers[kPublishS] = {"airflow_publish_s", 2u * cells * 4u, BufferKind::staged};

    const SpirvVariantSet* const kernels[] = {&gen::kSpvVariants_airflow_publish_fields,
                                              &gen::kSpvVariants_airflow_pack_f16};
    auto runner = ChainRunner::create(ctx, RunnerDesc{.buffers = buffers,
                                                      .kernels = kernels,
                                                      .push_bytes = sizeof(CellPush),
                                                      .workgroup_size = workgroup_size});
    if (!runner) {
        ADD_FAILURE() << runner.error().context;
        return std::nullopt;
    }
    ChainRunner& r = **runner;
    const auto check = [](const spade::Result<void>& done) {
        if (!done) ADD_FAILURE() << done.error().context;
        return done.has_value();
    };
    if (!check(r.upload(kParams, bytes_of(params))) || !check(r.upload(kU, bytes_of(g.u))) ||
        !check(r.upload(kV, bytes_of(g.v))) || !check(r.upload(kW, bytes_of(g.w))) ||
        !check(r.upload(kPressure, bytes_of(g.p)))) {
        return std::nullopt;
    }
    if (has_theta && !check(r.upload(kApertures, bytes_of(theta)))) return std::nullopt;
    if (!solid.empty() && !check(r.upload(kSolid, bytes_of(solid)))) return std::nullopt;
    if (channels != nullptr && !check(r.upload(kScratch, bytes_of(*channels)))) return std::nullopt;

    const auto count = static_cast<uint32_t>(cells);
    std::vector<ChainCommand> chain;
    if (channels == nullptr) {
        spade::airflow_gpu::append_publish(chain, {.fields = kFieldsPipeline, .pack = kPackPipeline}, count,
                                           workgroup_size);
    } else {
        chain.push_back(dispatch(kPackPipeline, count, workgroup_size));
    }
    const auto id = r.record(chain);
    if (!id) {
        ADD_FAILURE() << id.error().context;
        return std::nullopt;
    }
    if (!check(r.submit(*id, 1))) return std::nullopt;
    Published out;
    out.channels.assign(kChannels * cells, 0.0f);
    out.v.assign(spade::kFlowHalvesPerCell * cells, 0u);
    out.s.assign(spade::kFlowHalvesPerCell * cells, 0u);
    if (!check(r.read(kScratch, writable_bytes_of(out.channels))) ||
        !check(r.read(kPublishV, writable_bytes_of(out.v))) || !check(r.read(kPublishS, writable_bytes_of(out.s)))) {
        return std::nullopt;
    }
    if (channels == nullptr) {
        const auto extrapolated = af::extrapolated_cells(s, input_of(box));
        if (!extrapolated) {
            ADD_FAILURE() << extrapolated.error().context;
            return std::nullopt;
        }
        if (!check(spade::airflow_gpu::read_flow_volume(r, s, kOrigin, *extrapolated, out.volume))) {
            return std::nullopt;
        }
    }
    return out;
}


// A half's place on the number line, so adjacent halves are one apart.
[[nodiscard]] int32_t ordered(uint16_t h) noexcept {
    const int32_t magnitude = h & 0x7FFF;
    return (h & 0x8000) != 0 ? -magnitude : magnitude;
}

// Per channel, how many halves differ from the CPU's, and by how many steps
// at most (no-data on one side only counts as a step past any band).
struct ChannelDiff {
    std::size_t differing = 0;
    uint32_t worst_steps = 0;
};

[[nodiscard]] std::array<ChannelDiff, kChannels> channel_diffs(const Published& gpu, const spade::FlowVolume& cpu) {
    std::array<ChannelDiff, kChannels> d{};
    const std::size_t cells = cpu.v.size() / spade::kFlowHalvesPerCell;
    for (std::size_t cell = 0; cell < cells; ++cell) {
        for (uint32_t c = 0; c < kChannels; ++c) {
            const std::size_t at = cell * spade::kFlowHalvesPerCell + c % spade::kFlowHalvesPerCell;
            const uint16_t a = c < spade::kFlowHalvesPerCell ? gpu.v[at] : gpu.s[at];
            const uint16_t b = c < spade::kFlowHalvesPerCell ? cpu.v[at] : cpu.s[at];
            if (a == b) continue;
            ++d[c].differing;
            const bool no_data = a == spade::fp16::kNoDataBits || b == spade::fp16::kNoDataBits;
            const auto steps = no_data ? std::numeric_limits<uint32_t>::max()
                                       : static_cast<uint32_t>(std::abs(ordered(a) - ordered(b)));
            d[c].worst_steps = std::max(d[c].worst_steps, steps);
        }
    }
    return d;
}

constexpr const char* kChannelNames[kChannels] = {"u", "v", "w", "dye", "p", "|omega|", "q-hat", "lambda2-hat"};

// The header fields that differ in any bit, named; empty when none do.
[[nodiscard]] std::string header_differences(const spade::FlowVolumeHeader& a, const spade::FlowVolumeHeader& b) {
    std::string out;
    const auto field = [&](const std::string& name, uint32_t x, uint32_t y) {
        if (x == y) return;
        char line[96];
        std::snprintf(line, sizeof line, " %s 0x%08x != 0x%08x;", name.c_str(), x, y);
        out += line;
    };
    for (int axis = 0; axis < 3; ++axis) field("origin", bits(a.origin[axis]), bits(b.origin[axis]));
    field("cell", bits(a.cell), bits(b.cell));
    field("nx", a.nx, b.nx);
    field("ny", a.ny, b.ny);
    field("nz", a.nz, b.nz);
    for (uint32_t c = 0; c < kChannels; ++c) {
        field(std::string(kChannelNames[c]) + " min", bits(a.channel_min[c]), bits(b.channel_min[c]));
        field(std::string(kChannelNames[c]) + " max", bits(a.channel_max[c]), bits(b.channel_max[c]));
    }
    field("no_data", a.no_data, b.no_data);
    field("extrapolated_cells", a.extrapolated_cells, b.extrapolated_cells);
    return out;
}

// FNV-1a over a volume's halves and header bits: publish's output named in
// one word, printed so a refactor of publish shows it moved nothing.
[[nodiscard]] uint64_t digest(const spade::FlowVolume& volume) {
    uint64_t h = 0xCBF29CE484222325ull;
    const auto word = [&](uint32_t w) {
        for (int b = 0; b < 4; ++b) {
            h ^= (w >> (8 * b)) & 0xFFu;
            h *= 0x100000001B3ull;
        }
    };
    for (const uint16_t x : volume.v) word(x);
    for (const uint16_t x : volume.s) word(x);
    const spade::FlowVolumeHeader& hd = volume.header;
    for (int axis = 0; axis < 3; ++axis) word(bits(hd.origin[axis]));
    word(bits(hd.cell));
    word(hd.nx);
    word(hd.ny);
    word(hd.nz);
    for (uint32_t c = 0; c < kChannels; ++c) {
        word(bits(hd.channel_min[c]));
        word(bits(hd.channel_max[c]));
    }
    word(hd.no_data);
    word(hd.extrapolated_cells);
    return h;
}

constexpr uint32_t kWorkgroupSize = 64;

// ===========================================================================
// GpuAirflowPublish
// ===========================================================================

// pack_f16 alone: every case of fp16::from_f32 (ties both ways, the
// subnormal and overflow edges, the signs of zero, NaN and infinity) and a
// fixed-seed sweep of every exponent, uploaded as the scratch and packed;
// each half must be from_f32's to the bit.
TEST(GpuAirflowPublish, PackMatchesFromF32BitForBit) {
    if (const auto why = spade::testing::vulkan_skip_reason()) GTEST_SKIP() << *why;
    auto ctx = make_context();
    ASSERT_NE(ctx, nullptr);
    const af::MacGrid g = swirl(kShape);
    const std::size_t cells = cell_count(kShape);
    std::vector<float> channels(kChannels * cells, 0.0f);
    const float edges[] = {0.0f,
                           -0.0f,
                           1.0f,
                           -1.0f,
                           65504.0f,
                           65519.99f,
                           65520.0f,
                           -65520.0f,
                           0x1p-24f,
                           0x1p-25f,
                           0x1.000002p-25f,
                           0x1p-26f,
                           0x1.ffcp-15f,
                           0x1p-14f,
                           1.0f + 0x1p-11f,              // a tie, rounds to even (down)
                           1.0f + 0x1.8p-10f,            // a tie, rounds to even (up)
                           1.0f + 0x1p-11f + 0x1p-23f,  // just past a tie
                           std::numeric_limits<float>::infinity(),
                           -std::numeric_limits<float>::infinity(),
                           std::numeric_limits<float>::quiet_NaN(),
                           std::bit_cast<float>(0xFFC00001u),
                           std::numeric_limits<float>::denorm_min(),
                           std::numeric_limits<float>::max()};
    std::size_t at = 0;
    for (const float e : edges) channels[at++] = e;
    SplitMix rng{0xF16ull};
    while (at < channels.size()) channels[at++] = std::bit_cast<float>(static_cast<uint32_t>(rng.next()));
    const auto gpu = publish_on_gpu(*ctx, g, Box{.name = "pack"}, kWorkgroupSize, &channels);
    ASSERT_TRUE(gpu.has_value());
    std::size_t mismatches = 0;
    for (std::size_t cell = 0; cell < cells; ++cell) {
        for (uint32_t c = 0; c < kChannels; ++c) {
            const std::size_t h = cell * spade::kFlowHalvesPerCell + c % spade::kFlowHalvesPerCell;
            const uint16_t got = c < spade::kFlowHalvesPerCell ? gpu->v[h] : gpu->s[h];
            const uint16_t want = spade::fp16::from_f32(channels[kChannels * cell + c]).bits;
            if (got != want && mismatches++ < 8u) {
                ADD_FAILURE() << "f32 0x" << std::hex << bits(channels[kChannels * cell + c]) << ": half 0x" << got
                              << ", from_f32 0x" << want;
            }
        }
    }
    std::printf("[ publish ] pack: %zu values, %zu differ from from_f32\n", channels.size(), mismatches);
    EXPECT_EQ(mismatches, 0u);
}

// The two kernels against publish_render_volume's halves: u, v, w, dye and p
// to the bit (the solid layer's mean included, and no-data where the CPU
// writes it); |omega|, q-hat and lambda2-hat to the bit on the device of
// record, elsewhere within kBandHalfSteps.
TEST(GpuAirflowPublish, HalvesMatchPublishRenderVolume) {
    if (const auto why = spade::testing::vulkan_skip_reason()) GTEST_SKIP() << *why;
    auto ctx = make_context();
    ASSERT_NE(ctx, nullptr);
    const bool bit_equal = spade::testing::airflow_bit_equal_device(*ctx);
    const af::MacGrid g = swirl(kShape);
    for (const Box& box : boxes(kShape)) {
        const spade::FlowVolume cpu = publish_on_cpu(g, box);
        const auto gpu = publish_on_gpu(*ctx, g, box, kWorkgroupSize);
        ASSERT_TRUE(gpu.has_value()) << box.name;
        const auto d = channel_diffs(*gpu, cpu);
        std::size_t no_data = 0;
        for (const uint16_t h : gpu->v) no_data += h == spade::fp16::kNoDataBits ? 1u : 0u;
        for (const uint16_t h : gpu->s) no_data += h == spade::fp16::kNoDataBits ? 1u : 0u;
        std::printf("[ publish ] %-8s no-data halves %zu (CPU header %u), extrapolated cells %u\n", box.name, no_data,
                    cpu.header.no_data, cpu.header.extrapolated_cells);
        EXPECT_EQ(no_data, cpu.header.no_data) << box.name;
        for (uint32_t c = 0; c < kChannels; ++c) {
            std::printf("[ publish ] %-8s %-12s %5zu of %zu halves differ, by %u steps at most\n", box.name,
                        kChannelNames[c], d[c].differing, cell_count(kShape), d[c].worst_steps);
            if (bit_equal || c <= spade::kFlowPressure) {
                EXPECT_EQ(d[c].differing, 0u) << box.name << ", " << kChannelNames[c] << ": not to the bit";
            } else {
                EXPECT_LE(d[c].worst_steps, kBandHalfSteps) << box.name << ", " << kChannelNames[c];
            }
        }
    }
}

// No reduction, nothing shared between invocations: the workgroup size
// cannot change a bit.
TEST(GpuAirflowPublish, SameBitsAtEveryWorkgroupSize) {
    if (const auto why = spade::testing::vulkan_skip_reason()) GTEST_SKIP() << *why;
    auto ctx = make_context();
    ASSERT_NE(ctx, nullptr);
    const af::MacGrid g = swirl(kShape);
    const Box box = solids_box(kShape);
    std::optional<Published> first;
    for (const uint32_t wg : spade::compute::kSupportedWorkgroupSizes) {
        auto gpu = publish_on_gpu(*ctx, g, box, wg);
        ASSERT_TRUE(gpu.has_value()) << wg;
        if (!first) {
            first = std::move(gpu);
            continue;
        }
        EXPECT_TRUE(gpu->v == first->v && gpu->s == first->s) << "halves differ at " << wg;
        EXPECT_EQ(std::memcmp(gpu->channels.data(), first->channels.data(), gpu->channels.size() * 4u), 0)
            << "channels differ at " << wg;
    }
}


// Whether an opt-in gate is set, as test_core_fp16.cpp asks it.
[[nodiscard]] bool env_gate_is_set(const char* name) {
#if defined(_MSC_VER)
#pragma warning(push)
#pragma warning(disable : 4996)
#endif
    return std::getenv(name) != nullptr;
#if defined(_MSC_VER)
#pragma warning(pop)
#endif
}

[[nodiscard]] double ms_since(std::chrono::steady_clock::time_point t0) {
    return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
}

[[nodiscard]] double median(std::vector<double> v) {
    std::sort(v.begin(), v.end());
    return v[v.size() / 2u];
}

// B6's seam: append_publish() and read_flow_volume() give the whole volume
// publish_render_volume() gives, header and all, on the device of record:
// every half, every header field to the bit (generation aside), on the three
// boxes and the edge box on the NaN swirl. Another device keeps the halves'
// band, and its header must still be publish's rule on its own halves with
// the CPU's dims, origin and extrapolated count.
TEST(GpuAirflowPublish, TheVolumeFromTheDeviceIsPublishRenderVolumes) {
    if (const auto why = spade::testing::vulkan_skip_reason()) GTEST_SKIP() << *why;
    auto ctx = make_context();
    ASSERT_NE(ctx, nullptr);
    const bool bit_equal = spade::testing::airflow_bit_equal_device(*ctx);
    for (const Case& c : seam_cases(kShape)) {
        const spade::FlowVolume cpu = publish_on_cpu(c.g, c.box);
        const auto gpu = publish_on_gpu(*ctx, c.g, c.box, kWorkgroupSize);
        ASSERT_TRUE(gpu.has_value()) << c.box.name;
        const spade::FlowVolume& got = gpu->volume;
        EXPECT_NE(got.generation, 0u) << c.box.name;
        EXPECT_NE(got.generation, cpu.generation) << c.box.name;
        std::printf("[ publish ] seam %-8s no-data %u (CPU %u), extrapolated %u (CPU %u)\n", c.box.name,
                    got.header.no_data, cpu.header.no_data, got.header.extrapolated_cells,
                    cpu.header.extrapolated_cells);
        EXPECT_EQ(got.header.extrapolated_cells, cpu.header.extrapolated_cells) << c.box.name;
        if (bit_equal) {
            EXPECT_EQ(header_differences(got.header, cpu.header), "") << c.box.name;
            EXPECT_TRUE(got.v == cpu.v && got.s == cpu.s) << c.box.name << ": the halves are not the CPU's";
            continue;
        }
        spade::FlowVolume own = got;
        own.header.channel_min.fill(7.0f);
        ASSERT_TRUE(af::flow_header_from_halves(own).has_value());
        EXPECT_EQ(header_differences(got.header, own.header), "") << c.box.name;
        EXPECT_TRUE(got.header.nx == cpu.header.nx && got.header.ny == cpu.header.ny &&
                    got.header.nz == cpu.header.nz && bits(got.header.cell) == bits(cpu.header.cell))
            << c.box.name;
    }
}

// P8's rule (plan B, B6): read the fp32 channels back and pack them on the
// CPU, or pack on the device and read the halves back; keep the cheaper. On
// plan B's D1 box (B7, as amended 2026-10-21), 240 x 177 x 168 at 4 mm, 7.14 M
// cells (B6 re-measured there, 2026-10-26; the numbers below were at the
// first box, 224 x 168 x 160, 6.0 M cells), with one runner and two
// chains: the fields alone, and the fields then the pack. Each path is timed
// on the host from its submit to its last byte packed, the median of five,
// with the kernels' own times from the chains' marks. Opt-in
// (SPADE_AIRFLOW_P8=1): about 1 GiB of device memory and a minute on the box.
// Measured 2026-10-21, NVIDIA GeForce RTX 3060 Ti: publish_fields 1.25 ms,
// pack_f16 0.75 ms. B1's staging as drafted (HOST_COHERENT, not cached): fp32
// back + CPU pack 1176 ms (read 808 ms of 184 MiB, pack 347 ms), device pack
// + halves back 404 ms. With HOST_CACHED staging: 348 ms (read 19 ms, pack
// 327 ms) against 13 ms. Kept: the device pack, by P8's rule, either way.
TEST(GpuAirflowPublish, P8ReadbackCostAtTheD1Box) {
    if (const auto why = spade::testing::vulkan_skip_reason()) GTEST_SKIP() << *why;
    if (!env_gate_is_set("SPADE_AIRFLOW_P8")) GTEST_SKIP() << "P8's measurement: set SPADE_AIRFLOW_P8=1 to run it";
    auto ctx = make_context();
    ASSERT_NE(ctx, nullptr);
    const af::GridShape shape{240, 177, 168, 0.004f};
    const af::MacGrid g = swirl(shape);
    const std::size_t cells = cell_count(shape);
    const auto n = face_counts(shape);
    std::vector<uint32_t> params(17u, 0u);
    params[0] = shape.nx;
    params[1] = shape.ny;
    params[2] = shape.nz;
    params[4] = bits(shape.dx);
    params[5] = bits(1.0f / shape.dx);
    std::vector<RunnerBuffer> buffers(kBufferCount);
    for (uint32_t b = 0; b < kBufferCount; ++b) {
        buffers[b] = {"airflow_unused_" + std::to_string(b), 4u, BufferKind::device_only};
    }
    buffers[kParams] = {"airflow_params", params.size() * 4u, BufferKind::staged};
    buffers[kU] = {"airflow_u", n[0] * 4u, BufferKind::resident};
    buffers[kV] = {"airflow_v", n[1] * 4u, BufferKind::resident};
    buffers[kW] = {"airflow_w", n[2] * 4u, BufferKind::resident};
    buffers[kPressure] = {"airflow_pressure", cells * 4u, BufferKind::resident};
    buffers[kScratch] = {"airflow_advect", kChannels * cells * 4u, BufferKind::staged};
    buffers[kPublishV] = {"airflow_publish_v", 2u * cells * 4u, BufferKind::staged};
    buffers[kPublishS] = {"airflow_publish_s", 2u * cells * 4u, BufferKind::staged};
    const SpirvVariantSet* const kernels[] = {&gen::kSpvVariants_airflow_publish_fields,
                                              &gen::kSpvVariants_airflow_pack_f16};
    auto runner = ChainRunner::create(*ctx, RunnerDesc{.buffers = buffers,
                                                       .kernels = kernels,
                                                       .push_bytes = sizeof(CellPush),
                                                       .workgroup_size = kWorkgroupSize});
    ASSERT_TRUE(runner.has_value()) << runner.error().context;
    ChainRunner& r = **runner;
    ASSERT_TRUE(r.upload(kParams, bytes_of(params)).has_value());
    ASSERT_TRUE(r.upload(kU, bytes_of(g.u)).has_value());
    ASSERT_TRUE(r.upload(kV, bytes_of(g.v)).has_value());
    ASSERT_TRUE(r.upload(kW, bytes_of(g.w)).has_value());
    ASSERT_TRUE(r.upload(kPressure, bytes_of(g.p)).has_value());
    const auto count = static_cast<uint32_t>(cells);
    const ChainCommand fields_only[] = {ChainCommand{}, dispatch(kFieldsPipeline, count, kWorkgroupSize),
                                        ChainCommand{}};
    const ChainCommand fields_and_pack[] = {ChainCommand{}, dispatch(kFieldsPipeline, count, kWorkgroupSize),
                                            ChainCommand{}, dispatch(kPackPipeline, count, kWorkgroupSize),
                                            ChainCommand{}};
    const auto a = r.record(fields_only);
    const auto b = r.record(fields_and_pack);
    ASSERT_TRUE(a.has_value() && b.has_value());

    std::vector<float> channels(kChannels * cells);
    std::vector<uint16_t> v(spade::kFlowHalvesPerCell * cells), s(spade::kFlowHalvesPerCell * cells);
    std::vector<uint16_t> v_cpu(v.size()), s_cpu(s.size());
    std::vector<double> fp32_total, fp32_read, fp32_pack, half_total, half_read, fields_ms, pack_ms;
    for (int rep = 0; rep < 5; ++rep) {
        // fp32 back, packed on the CPU.
        auto t0 = std::chrono::steady_clock::now();
        ASSERT_TRUE(r.submit(*a, 1).has_value());
        const auto t_read = std::chrono::steady_clock::now();
        ASSERT_TRUE(r.read(kScratch, writable_bytes_of(channels)).has_value());
        fp32_read.push_back(ms_since(t_read));
        const auto t_pack = std::chrono::steady_clock::now();
        for (std::size_t cell = 0; cell < cells; ++cell) {
            for (uint32_t c = 0; c < kChannels; ++c) {
                std::vector<uint16_t>& texture = c < spade::kFlowHalvesPerCell ? v_cpu : s_cpu;
                texture[cell * spade::kFlowHalvesPerCell + c % spade::kFlowHalvesPerCell] =
                    spade::fp16::from_f32(channels[kChannels * cell + c]).bits;
            }
        }
        fp32_pack.push_back(ms_since(t_pack));
        fp32_total.push_back(ms_since(t0));
        // Packed on the device, the halves back.
        t0 = std::chrono::steady_clock::now();
        ASSERT_TRUE(r.submit(*b, 1).has_value());
        const auto t_halves = std::chrono::steady_clock::now();
        ASSERT_TRUE(r.read(kPublishV, writable_bytes_of(v)).has_value());
        ASSERT_TRUE(r.read(kPublishS, writable_bytes_of(s)).has_value());
        half_read.push_back(ms_since(t_halves));
        half_total.push_back(ms_since(t0));
        if (const auto d = r.durations_ns(*b); d.has_value() && d->size() == 3u) {
            fields_ms.push_back(static_cast<double>((*d)[1]) * 1e-6);
            pack_ms.push_back(static_cast<double>((*d)[2]) * 1e-6);
        }
    }
    // The two paths publish the same halves.
    EXPECT_TRUE(v == v_cpu && s == s_cpu) << "the device's pack and the CPU's differ";
    std::printf("[ P8 ] %u x %u x %u (%zu cells), median of 5\n", shape.nx, shape.ny, shape.nz, cells);
    if (!fields_ms.empty()) {
        std::printf("[ P8 ] kernels: publish_fields %.2f ms, pack_f16 %.2f ms\n", median(fields_ms), median(pack_ms));
    }
    std::printf("[ P8 ] fp32 back + CPU pack: %.1f ms (read %.1f ms of %.0f MiB, pack %.1f ms)\n", median(fp32_total),
                median(fp32_read), static_cast<double>(channels.size() * 4u) / 1048576.0, median(fp32_pack));
    std::printf("[ P8 ] device pack + halves back: %.1f ms (read %.1f ms of %.0f MiB)\n", median(half_total),
                median(half_read), static_cast<double>((v.size() + s.size()) * 2u) / 1048576.0);
    std::printf("[ P8 ] keep: %s\n", median(half_total) <= median(fp32_total) ? "the device pack" : "the CPU pack");
}

// ===========================================================================
// AirflowPublishHeader (no device)
// ===========================================================================

// flow_header_from_halves() counts a half as no data when it is kNoDataBits,
// where publish counted from_f32's flag: the two agree for every input
// because from_f32 returns kNoDataBits for every no-data case and otherwise
// a zero or a magnitude below 0x7C00, never an exponent field of 31
// (fp16.hpp). Swept here: NaNs of both signs and several payloads, the
// infinities, the edge of the range (65504, 65519.99, 65520 and past),
// FLT_MAX, the subnormal edges, and a strided sweep of every bit pattern.
TEST(AirflowPublishHeader, NoDataIsExactlyTheNoDataBits) {
    std::vector<uint32_t> inputs = {0x7FC00000u, 0x7F800001u, 0x7FC00001u, 0x7FFFFFFFu, 0xFFC00000u, 0xFF800001u,
                                    0x7F800000u, 0xFF800000u, 0x7F7FFFFFu, 0xFF7FFFFFu, 0x00000000u, 0x80000000u,
                                    0x00000001u, 0x33800000u, 0x33000000u, 0x387FC000u};
    for (const float x : {65504.0f, 65519.99f, 65520.0f, 65536.0f, 1.0e6f}) {
        inputs.push_back(bits(x));
        inputs.push_back(bits(-x));
    }
    for (uint64_t i = 0; i <= 0xFFFFFFFFull; i += 4093u) inputs.push_back(static_cast<uint32_t>(i));
    std::size_t disagreeing = 0;
    std::size_t no_data = 0;
    for (const uint32_t f : inputs) {
        const spade::fp16::Half h = spade::fp16::from_f32(std::bit_cast<float>(f));
        no_data += h.no_data ? 1u : 0u;
        const bool by_bits = h.bits == spade::fp16::kNoDataBits;
        const bool exponent_31 = (h.bits & 0x7C00u) == 0x7C00u;
        if ((h.no_data != by_bits || (!h.no_data && exponent_31)) && disagreeing++ < 8u) {
            ADD_FAILURE() << "f32 0x" << std::hex << f << ": half 0x" << h.bits << ", no_data " << h.no_data;
        }
    }
    std::printf("[ header ] %zu inputs, %zu no data, %zu disagreeing\n", inputs.size(), no_data, disagreeing);
    EXPECT_EQ(disagreeing, 0u);
    EXPECT_GT(no_data, 1000u);  // the sweep crossed the NaNs, the infinities and the range's edge
}

// A range is the stored halves' in cell order: -0 seen before +0 stays the
// minimum, +0 seen before -0 stays both; a channel with no valid half keeps
// 0 and 0; no-data is counted per half; the header's other fields are left
// as they were. A size that does not match the dims is refused.
TEST(AirflowPublishHeader, TheRangeIsTheStoredHalvesInCellOrder) {
    constexpr uint16_t kNo = spade::fp16::kNoDataBits;
    spade::FlowVolume vol;
    vol.header.origin = kOrigin;
    vol.header.cell = 0.25f;
    vol.header.nx = 3;
    vol.header.ny = 1;
    vol.header.nz = 1;
    vol.header.channel_min.fill(7.0f);
    vol.header.channel_max.fill(7.0f);
    vol.header.no_data = 99;
    vol.header.extrapolated_cells = 2;
    vol.generation = 5;
    // Per cell: u, v, w, dye | p, |omega|, q-hat, lambda2-hat.
    vol.v = {0x8000, 0x0000, kNo, kNo,  //
             0x0000, 0x8000, kNo, 0x3C00,  //
             0x3C00, kNo, kNo, kNo};
    vol.s = {0xC000, 0x3800, 0x3800, 0x3800,  //
             0x0001, 0x3800, 0x3800, 0x3800,  //
             0x7BFF, 0x3800, 0x3800, 0x3800};
    ASSERT_TRUE(af::flow_header_from_halves(vol).has_value());
    const spade::FlowVolumeHeader& h = vol.header;
    EXPECT_EQ(bits(h.channel_min[0]), 0x80000000u) << "u: -0 came first and +0 is no less";
    EXPECT_EQ(bits(h.channel_max[0]), bits(1.0f));
    EXPECT_EQ(bits(h.channel_min[1]), 0u) << "v: +0 came first and -0 is no less";
    EXPECT_EQ(bits(h.channel_max[1]), 0u) << "v: +0 came first and -0 is no more";
    EXPECT_EQ(bits(h.channel_min[2]), 0u) << "w: no valid half keeps 0";
    EXPECT_EQ(bits(h.channel_max[2]), 0u) << "w: no valid half keeps 0";
    EXPECT_EQ(h.channel_min[3], 1.0f);
    EXPECT_EQ(h.channel_max[3], 1.0f);
    EXPECT_EQ(h.channel_min[4], -2.0f);
    EXPECT_EQ(h.channel_max[4], 65504.0f);
    EXPECT_EQ(h.channel_min[5], 0.5f);
    EXPECT_EQ(h.no_data, 6u) << "per half: v 1, w 3, dye 2";
    EXPECT_EQ(header_differences(h, spade::FlowVolumeHeader{.origin = kOrigin, .cell = 0.25f, .nx = 3, .ny = 1,
                                                            .nz = 1, .channel_min = h.channel_min,
                                                            .channel_max = h.channel_max, .no_data = 6,
                                                            .extrapolated_cells = 2}),
              "");
    EXPECT_EQ(vol.generation, 5u) << "the generation is the caller's";
    vol.s.pop_back();
    const auto refused = af::flow_header_from_halves(vol);
    ASSERT_FALSE(refused.has_value());
    EXPECT_EQ(refused.error().code, spade::Code::invalid_argument);
}

// The header flow_header_from_halves() and extrapolated_cells() make from
// publish's own halves is publish's, field for field, on the three boxes and
// the edge box on the NaN swirl (deep cells, a wrapped first layer, NaN
// channels). Each case's digest is printed: a refactor of publish that moves
// no bit prints the same four.
TEST(AirflowPublishHeader, TheHeaderFromTheHalvesIsPublishs) {
    for (const Case& c : seam_cases(kShape)) {
        const spade::FlowVolume cpu = publish_on_cpu(c.g, c.box);
        spade::FlowVolume rebuilt;
        rebuilt.v = cpu.v;
        rebuilt.s = cpu.s;
        rebuilt.header.origin = kOrigin;
        rebuilt.header.cell = kShape.dx;
        rebuilt.header.nx = kShape.nx;
        rebuilt.header.ny = kShape.ny;
        rebuilt.header.nz = kShape.nz;
        rebuilt.header.channel_min.fill(7.0f);
        rebuilt.header.channel_max.fill(-7.0f);
        rebuilt.header.no_data = 99;
        ASSERT_TRUE(af::flow_header_from_halves(rebuilt).has_value()) << c.box.name;
        const auto extrapolated = af::extrapolated_cells(kShape, input_of(c.box));
        ASSERT_TRUE(extrapolated.has_value()) << c.box.name;
        rebuilt.header.extrapolated_cells = *extrapolated;
        std::printf("[ header ] %-8s digest %016llx, no-data %u, extrapolated %u\n", c.box.name,
                    static_cast<unsigned long long>(digest(cpu)), cpu.header.no_data,
                    cpu.header.extrapolated_cells);
        EXPECT_EQ(header_differences(rebuilt.header, cpu.header), "") << c.box.name;
    }
    // The edge box's own counts: its first layer wraps, and its NaN mean is
    // no data yet extrapolated.
    const Case edge{nan_swirl(kShape), edge_box(kShape)};
    const spade::FlowVolume cpu = publish_on_cpu(edge.g, edge.box);
    af::PublishInput no_wrap = input_of(edge.box);
    no_wrap.periodic = {false, false, false};
    const auto unwrapped = af::extrapolated_cells(kShape, no_wrap);
    ASSERT_TRUE(unwrapped.has_value());
    EXPECT_EQ(*unwrapped + 6u, cpu.header.extrapolated_cells) << "(0, 0..1, 0..2) reach fluid only by a wrap";
    const std::size_t at = af::cell_index(kShape, 6, 5, 9) * spade::kFlowHalvesPerCell;
    EXPECT_EQ(cpu.v[at], spade::fp16::kNoDataBits) << "the solid beside the NaN is no data";
    const std::vector<uint8_t> wrong(cell_count(kShape) - 1u, 0u);
    af::PublishInput refused_in;
    refused_in.solid = wrong;
    const auto refused = af::extrapolated_cells(kShape, refused_in);
    ASSERT_FALSE(refused.has_value());
    EXPECT_EQ(refused.error().code, spade::Code::invalid_argument);
}

}  // namespace
