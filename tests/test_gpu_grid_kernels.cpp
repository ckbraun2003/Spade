// test_gpu_grid_kernels.cpp -- the domain-neutral grid kernels on a device against their CPU twins (module API stage 7,
// Task B3; engine/world/grid_kernels.hpp, engine/shaders/kernels/grid_shift.slang and grid_points.slang), through the
// ChainRunner. The twins are pinned on their own in test_grid_kernels.cpp; here the device must match them: the shift bit
// for bit, the point interpolation within the band (TD-14; a band wider than 1e-4 relative would be a grade change,
// TD-2, never a widening), and each at all three workgroup sizes bit-identical to the others (A7).
//
// grid_raster is NOT here: it moved to batch E, where the part table it reads exists (the lead, 2026-11-07).
// Every test skips with its reason when no device is admitted (TD-6).
#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <memory>
#include <span>
#include <string>
#include <vector>

#include "bindings.gen.hpp"
#include "compute/backend.hpp"
#include "compute/dispatch_shape.hpp"
#include "compute/spirv_variants.hpp"
#include "compute/vulkan/context.hpp"
#include "gpu_skip.hpp"
#include "grid_points.spv.gen.hpp"
#include "grid_shift.spv.gen.hpp"
#include "spike/airflow/chain_runner.hpp"
#include "world/grid_kernels.hpp"

namespace {

using spade::airflow_gpu::BufferKind;
using spade::airflow_gpu::ChainRunner;
using spade::airflow_gpu::kAirflowSetBuffers;
using spade::airflow_gpu::RunnerBuffer;
using spade::airflow_gpu::RunnerDesc;
using spade::compute::ChainCommand;
using spade::compute::SpirvVariantSet;
using spade::compute::VulkanContext;
using spade::world::GridDims;
using spade::world::GridFrame;
namespace gen = spade::compute::gen;

constexpr uint32_t kU = gen::kBinding_airflow_u;
constexpr uint32_t kV = gen::kBinding_airflow_v;
constexpr uint32_t kW = gen::kBinding_airflow_w;

struct ShiftPush {
    uint32_t nx, ny, nz, fill;
    int32_t sx, sy, sz;
    uint32_t unused;
};
struct PointsPush {
    uint32_t nx, ny, nz, comps;
    float ox, oy, oz, inv_cell;
    uint32_t point_count, unused0, unused1, unused2;
};
static_assert(sizeof(ShiftPush) == 32 && sizeof(PointsPush) == 48);

[[nodiscard]] std::vector<RunnerBuffer> buffers_for(uint64_t u_bytes, uint64_t v_bytes, uint64_t w_bytes) {
    std::vector<RunnerBuffer> out;
    for (uint32_t b = 0; b <= kW; ++b) {
        out.push_back(RunnerBuffer{std::string(kAirflowSetBuffers[b].name), 4, kAirflowSetBuffers[b].kind});
    }
    out[kU] = {"airflow_u", u_bytes, BufferKind::staged};
    out[kV] = {"airflow_v", v_bytes, BufferKind::staged};
    out[kW] = {"airflow_w", w_bytes, BufferKind::staged};
    return out;
}

template <typename T>
[[nodiscard]] std::span<const std::byte> bytes_of(const std::vector<T>& v) {
    return std::as_bytes(std::span(v));
}
template <typename T>
[[nodiscard]] std::span<std::byte> writable_bytes_of(std::vector<T>& v) {
    return std::as_writable_bytes(std::span(v));
}

// Runs grid_shift over `src` on the device at `workgroup_size`; returns the destination words.
[[nodiscard]] bool device_shift(VulkanContext& ctx, const std::vector<uint32_t>& src, GridDims d, std::array<int32_t, 3> s,
                                uint32_t fill, uint32_t workgroup_size, std::vector<uint32_t>& out, std::string& why) {
    const auto groups = spade::compute::grid_groups({d.nx, d.ny, d.nz}, workgroup_size, 1);
    const std::vector<RunnerBuffer> buffers = buffers_for(src.size() * 4, src.size() * 4, 4);
    const SpirvVariantSet* const kernels[] = {&gen::kSpvVariants_grid_shift};
    auto runner = ChainRunner::create(ctx, RunnerDesc{.buffers = buffers, .kernels = kernels, .push_bytes = sizeof(ShiftPush),
                                                      .workgroup_size = workgroup_size});
    if (!runner) {
        why = runner.error().context;
        return false;
    }
    const std::vector<uint32_t> zeros(src.size(), 0u);
    if (!(*runner)->upload(kU, bytes_of(src)) || !(*runner)->upload(kV, bytes_of(zeros))) {
        why = "upload";
        return false;
    }
    ChainCommand c{.is_dispatch = true, .pipeline = 0, .groups = groups};
    const ShiftPush push{d.nx, d.ny, d.nz, fill, s[0], s[1], s[2], 0};
    std::memcpy(c.push.data(), &push, sizeof(push));
    const ChainCommand chain[] = {c};
    auto id = (*runner)->record(chain);
    if (!id || !(*runner)->submit(*id, 1)) {
        why = "record/submit";
        return false;
    }
    out.assign(src.size(), 0u);
    if (!(*runner)->read(kV, writable_bytes_of(out))) {
        why = "read";
        return false;
    }
    return true;
}

[[nodiscard]] bool device_points(VulkanContext& ctx, const std::vector<float>& field, GridDims d, uint32_t comps,
                                 const GridFrame& f, const std::vector<float>& points, uint32_t workgroup_size,
                                 std::vector<float>& out, std::string& why) {
    const uint32_t count = static_cast<uint32_t>(points.size() / 3);
    const std::vector<RunnerBuffer> buffers = buffers_for(field.size() * 4, points.size() * 4, uint64_t{count} * comps * 4);
    const SpirvVariantSet* const kernels[] = {&gen::kSpvVariants_grid_points};
    auto runner = ChainRunner::create(ctx, RunnerDesc{.buffers = buffers, .kernels = kernels, .push_bytes = sizeof(PointsPush),
                                                      .workgroup_size = workgroup_size});
    if (!runner) {
        why = runner.error().context;
        return false;
    }
    if (!(*runner)->upload(kU, bytes_of(field)) || !(*runner)->upload(kV, bytes_of(points))) {
        why = "upload";
        return false;
    }
    ChainCommand c{.is_dispatch = true, .pipeline = 0, .groups = {(count + workgroup_size - 1) / workgroup_size, 1, 1}};
    const PointsPush push{d.nx, d.ny, d.nz, comps, f.origin[0], f.origin[1], f.origin[2], f.inv_cell, count, 0, 0, 0};
    std::memcpy(c.push.data(), &push, sizeof(push));
    const ChainCommand chain[] = {c};
    auto id = (*runner)->record(chain);
    if (!id || !(*runner)->submit(*id, 1)) {
        why = "record/submit";
        return false;
    }
    out.assign(uint64_t{count} * comps, -1.0f);
    if (!(*runner)->read(kW, writable_bytes_of(out))) {
        why = "read";
        return false;
    }
    return true;
}

constexpr uint32_t kFill = 0xDEADBEEFu;

// splitmix64: a fixed-seed stream, the same on every compiler.
struct SplitMix64 {
    uint64_t state = 0;
    uint64_t next() noexcept {
        uint64_t z = (state += 0x9E3779B97F4A7C15ull);
        z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ull;
        z = (z ^ (z >> 27)) * 0x94D049BB133111EBull;
        return z ^ (z >> 31);
    }
};

}  // namespace

TEST(GpuGridShift, TheDeviceMatchesTheCpuTwinBitForBitAtEverySize) {
    if (const auto why = spade::testing::vulkan_skip_reason()) GTEST_SKIP() << *why;
    auto ctx = VulkanContext::create(spade::compute::BackendDesc{.kind = spade::compute::BackendKind::vulkan});
    ASSERT_TRUE(ctx.has_value()) << ctx.error().context;

    const GridDims d{33, 17, 9};
    std::vector<uint32_t> src(d.cells());
    SplitMix64 rng{0x51F7ull};
    for (uint32_t& w : src) w = static_cast<uint32_t>(rng.next());
    src[0] = 0x7FC00001u;  // a NaN payload, negative zero and a denormal ride along
    src[1] = 0x80000000u;
    src[2] = 0x00000001u;

    const std::array<std::array<int32_t, 3>, 4> shifts = {{{0, 0, 0}, {1, 0, 0}, {-3, 2, 1}, {40, 0, 0}}};
    for (const auto& s : shifts) {
        std::vector<uint32_t> twin(src.size(), 0u);
        ASSERT_TRUE(spade::world::grid_shift(src, twin, d, s, kFill).has_value());
        for (const uint32_t size : {32u, 64u, 128u}) {
            std::vector<uint32_t> device;
            std::string why;
            ASSERT_TRUE(device_shift(**ctx, src, d, s, kFill, size, device, why)) << why;
            EXPECT_EQ(device, twin) << "shift " << s[0] << "," << s[1] << "," << s[2] << " size " << size;
        }
    }
}

TEST(GpuGridPoints, TheDeviceMatchesTheCpuTwinWithinTheBandAndTheSizesAreBitIdentical) {
    if (const auto why = spade::testing::vulkan_skip_reason()) GTEST_SKIP() << *why;
    auto ctx = VulkanContext::create(spade::compute::BackendDesc{.kind = spade::compute::BackendKind::vulkan});
    ASSERT_TRUE(ctx.has_value()) << ctx.error().context;

    const GridDims d{9, 7, 5};
    const uint32_t comps = 3;
    SplitMix64 rng{0xA11ull};
    const auto uniform = [&rng](float lo, float hi) {
        return lo + (hi - lo) * (static_cast<float>(rng.next() >> 40) / 16777216.0f);
    };
    std::vector<float> field(d.cells() * comps);
    for (float& v : field) v = uniform(-5.0f, 5.0f);
    const GridFrame frame{.origin = {-1.0f, 0.5f, 2.0f}, .inv_cell = 2.0f};  // cell 0.5: the grid spans 4.5 x 3.5 x 2.5
    std::vector<float> points;
    for (int i = 0; i < 300; ++i) {  // inside, and well outside on every side so the border clamp is exercised
        points.push_back(uniform(-2.0f, 4.0f));
        points.push_back(uniform(-1.0f, 5.0f));
        points.push_back(uniform(1.0f, 6.0f));
    }

    std::vector<float> twin(points.size() / 3 * comps, 0.0f);
    ASSERT_TRUE(spade::world::grid_points(field, d, comps, frame, points, twin).has_value());
    std::vector<float> reference;
    for (const uint32_t size : {32u, 64u, 128u}) {
        std::vector<float> device;
        std::string why;
        ASSERT_TRUE(device_points(**ctx, field, d, comps, frame, points, size, device, why)) << why;
        ASSERT_EQ(device.size(), twin.size());
        for (std::size_t i = 0; i < device.size(); ++i) {
            EXPECT_NEAR(device[i], twin[i], 2e-6f * std::max(1.0f, std::fabs(twin[i]))) << "value " << i << " size " << size;
        }
        if (reference.empty()) {
            reference = device;
        } else {
            EXPECT_EQ(std::memcmp(device.data(), reference.data(), device.size() * 4), 0)
                << "size " << size << " is not bit-identical to size 32";
        }
    }
}
