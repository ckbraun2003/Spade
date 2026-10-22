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
//                       u, v, w, dye and p to the bit, |omega|, q-hat and
//                       lambda2-hat within their band; the same bits at
//                       workgroup sizes 32, 64 and 128.
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
#include "compute/vulkan/dispatch_chain.hpp"
#include "core/flow_volume.hpp"
#include "core/fp16.hpp"
#include "gpu_skip.hpp"
#include "physics/airflow/mac_grid.hpp"
#include "physics/airflow/publish.hpp"
#include "physics/airflow/regions.hpp"
#include "spike/airflow/chain_runner.hpp"

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

// The band for |omega|, q-hat and lambda2-hat, in binary16 steps: their sqrt
// and division do not round correctly on the device, and a last-bit change in
// fp32 can carry a half across its rounding point to its neighbour, one step;
// two would take an fp32 error of a whole half step (2^-11 of the value), a
// real error, not rounding. Measured 2026-10-21 on an NVIDIA GeForce RTX 3060
// Ti, on B1's runner: |omega| and lambda2-hat one step in at most 1 of 3840
// halves per box, q-hat none; u, v, w, dye and p to the bit on all three boxes
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

[[nodiscard]] spade::FlowVolume publish_on_cpu(const af::MacGrid& g, const Box& box) {
    af::PublishInput in;
    in.solid = box.solid;
    in.periodic = box.periodic;
    for (uint32_t c = 0; c < 3; ++c) in.theta[c] = box.theta[c];
    spade::FlowVolume out;
    const auto done = af::publish_render_volume(g, in, out);
    EXPECT_TRUE(done.has_value()) << box.name << ": " << done.error().context;
    return out;
}

// What the device holds after the chain: the fp32 channels (8 per cell) and
// the halves, as FlowVolume::v and ::s hold them.
struct Published {
    std::vector<float> channels;
    std::vector<uint16_t> v, s;
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
    if (channels == nullptr) chain.push_back(dispatch(kFieldsPipeline, count, workgroup_size));
    chain.push_back(dispatch(kPackPipeline, count, workgroup_size));
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
// writes it); |omega|, q-hat and lambda2-hat within kBandHalfSteps.
TEST(GpuAirflowPublish, HalvesMatchPublishRenderVolume) {
    if (const auto why = spade::testing::vulkan_skip_reason()) GTEST_SKIP() << *why;
    auto ctx = make_context();
    ASSERT_NE(ctx, nullptr);
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
            if (c <= spade::kFlowPressure) {
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

// P8's rule (plan B, B6): read the fp32 channels back and pack them on the
// CPU, or pack on the device and read the halves back; keep the cheaper. On
// plan B's D1 box (B7), 224 x 168 x 160, 6.0 M cells, with one runner and two
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
    const af::GridShape shape{224, 168, 160, 0.004f};
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

}  // namespace
