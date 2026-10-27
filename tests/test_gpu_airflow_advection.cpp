// ---------------------------------------------------------------------------
// test_gpu_airflow_advection.cpp -- plan B, task B3: the advection kernel
// (engine/shaders/kernels/airflow_advection.slang) against its CPU twin,
// advect_velocity() (engine/physics/airflow/advect.cpp), on the chain runner
// (spike/airflow/chain_runner.hpp).
//
//   GpuAirflowAdvection   each pass banded against advect.cpp, on clamped and
//                         on periodic axes: the forward pass (trace_rk2,
//                         advect) against the CPU scratch's hat, lo and hi;
//                         the backward pass and the clamp (advect_back,
//                         maccormack_clamp) from the CPU's hat, against the
//                         CPU's result; the two passes together; the same
//                         bits at workgroup sizes 32, 64 and 128.
//
// The box is 16 x 12 x 20 at 1/16 m, no two sizes alike, so a swapped axis
// or stride shows. The velocity is a swirl about y with a shear and fixed-
// seed noise, fast enough that the traces cross up to two cells and leave the
// box at its corners (the clamp's and the wrap's paths), and rough enough
// that the MacCormack clamp acts.
//
// Every Gpu* test starts with the vulkan_skip_reason() skip (gpu_skip.hpp).
// ---------------------------------------------------------------------------

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <limits>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <vector>

#include "airflow_advection.spv.gen.hpp"
#include "compute/backend.hpp"
#include "compute/spirv_variants.hpp"
#include "compute/vulkan/context.hpp"
#include "airflow_exact_device.hpp"
#include "compute/vulkan/dispatch_chain.hpp"
#include "core/detail/flush_subnormals.hpp"
#include "gpu_skip.hpp"
#include "physics/airflow/advect.hpp"
#include "physics/airflow/mac_grid.hpp"
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
constexpr float kStepS = 0.01f;

// Every pass is advect.cpp's bit for bit on the device of record: the
// periodic wrap's division is fp32_math's div32_ftz, and the CPU reference
// runs under the step's subnormal flush (TD-14 as amended 2026-10-22;
// airflow_exact_device.hpp). Another device or driver keeps the floor band,
// relative to the field's largest speed (plan B: 4x the largest measurement,
// at least 4 ulps; measured bit-equal on the RTX 3060 Ti, 2026-10-21, so the
// floor). Red check: the correction's 1/2 as
// 1/4 fails the backward and both-pass tests by 0.055 (clamped) and 0.30
// (periodic) of the top speed.
constexpr float kBandRelative = 4.0f * 0x1p-23f;

// Set 1's first seven bindings: the runner binds buffers[i] at binding i.
enum Buffer : uint32_t { kParams = 0, kU, kV, kW, kPressure, kNext, kAdvect, kBufferCount };

// The kernel's push block (airflow_advection.slang's AdvectionPush).
struct AdvectionPush {
    uint32_t component = 0;
    uint32_t pass = 0;
    uint32_t face_count = 0;
    uint32_t unused = 0;
};

enum class Passes { forward, backward, both };

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

[[nodiscard]] std::array<std::size_t, 3> face_counts(const af::GridShape& s) {
    return {std::size_t{s.nx + 1u} * s.ny * s.nz, std::size_t{s.nx} * (s.ny + 1u) * s.nz,
            std::size_t{s.nx} * s.ny * (s.nz + 1u)};
}

[[nodiscard]] af::DomainBc clamped_box() { return af::DomainBc{}; }  // every face open: no axis wraps

[[nodiscard]] af::DomainBc periodic_x_and_z() {
    af::DomainBc bc;
    bc.face[af::kXMinus] = bc.face[af::kXPlus] = af::FaceBc::periodic;
    bc.face[af::kZMinus] = bc.face[af::kZPlus] = af::FaceBc::periodic;
    return bc;
}

// A swirl about y through the box's centre at 25 rad/s (12.5 m/s at the x
// faces: two cells in a step), v sheared across x at up to 5 m/s, and noise of
// up to 1 m/s on every face.
[[nodiscard]] af::MacGrid swirl(const af::GridShape& s) {
    af::MacGrid g;
    g.shape = s;
    const auto n = face_counts(s);
    g.u.assign(n[0], 0.0f);
    g.v.assign(n[1], 0.0f);
    g.w.assign(n[2], 0.0f);
    g.p.assign(std::size_t{s.nx} * s.ny * s.nz, 0.0f);
    const glm::vec3 centre = 0.5f * s.dx * glm::vec3(static_cast<float>(s.nx), static_cast<float>(s.ny),
                                                     static_cast<float>(s.nz));
    constexpr float kOmega = 25.0f;
    SplitMix rng{0xAD7EC7ull};
    af::for_each_sample(s, 0, [&](std::size_t i, glm::vec3 x) {
        g.u[i] = -kOmega * (x.z - centre.z) + rng.symmetric();
    });
    af::for_each_sample(s, 1, [&](std::size_t i, glm::vec3 x) {
        g.v[i] = 10.0f * (x.x - centre.x) + rng.symmetric();
    });
    af::for_each_sample(s, 2, [&](std::size_t i, glm::vec3 x) {
        g.w[i] = kOmega * (x.x - centre.x) + rng.symmetric();
    });
    return g;
}

[[nodiscard]] float largest_speed(const af::MacGrid& g) {
    float top = 0.0f;
    for (const auto* field : {&g.u, &g.v, &g.w}) {
        for (const float v : *field) top = std::max(top, std::abs(v));
    }
    return top;
}

// The CPU twin's result: the scratch's hat, lo and hi and the advected faces,
// each the three components packed u, v, w (the kernel's layout).
struct Advected {
    std::vector<float> hat, lo, hi, next;
};

[[nodiscard]] std::vector<float> packed(const std::array<std::vector<float>, 3>& c) {
    std::vector<float> out;
    for (const auto& v : c) out.insert(out.end(), v.begin(), v.end());
    return out;
}

[[nodiscard]] Advected advect_on_cpu(const af::MacGrid& in, const af::DomainBc& bc) {
    af::MacGrid out = in;
    af::AdvectScratch s = af::make_advect_scratch(in.shape);
    const spade::math::ScopedFlushSubnormals flush;  // the step's mode
    af::advect_velocity(in, out, bc, kStepS, s);
    return {packed(s.hat), packed(s.lo), packed(s.hi), packed({out.u, out.v, out.w})};
}

template <class T>
[[nodiscard]] std::span<const std::byte> bytes_of(const std::vector<T>& v) noexcept {
    return std::as_bytes(std::span<const T>(v));
}

template <class T>
[[nodiscard]] std::span<std::byte> writable_bytes_of(std::vector<T>& v) noexcept {
    return std::as_writable_bytes(std::span<T>(v));
}

[[nodiscard]] ChainCommand dispatch(const AdvectionPush& push, uint32_t workgroup_size) {
    ChainCommand c{.is_dispatch = true, .pipeline = 0};
    std::memcpy(c.push.data(), &push, sizeof push);
    c.groups = {(push.face_count + workgroup_size - 1u) / workgroup_size, 1u, 1u};
    return c;
}

// The kernel's passes on the device. With `cpu_forward`, airflow_advect
// starts as the CPU's hat, lo and hi (the backward pass on its own); the
// result is what the device holds after the chain.
[[nodiscard]] std::optional<Advected> advect_on_gpu(VulkanContext& ctx, const af::MacGrid& in, const af::DomainBc& bc,
                                                    Passes passes, uint32_t workgroup_size,
                                                    const Advected* cpu_forward = nullptr) {
    const auto n = face_counts(in.shape);
    const std::size_t total = n[0] + n[1] + n[2];
    const std::array<bool, 3> periodic = af::periodic_axes(bc);
    // airflow_params, Physics' draft: dims, grid {dx, 1/dx, h}, periodic,
    // offsets_advect {hat, lo, hi, faces_total}.
    const std::vector<uint32_t> params = {in.shape.nx,
                                          in.shape.ny,
                                          in.shape.nz,
                                          0u,
                                          bits(in.shape.dx),
                                          bits(1.0f / in.shape.dx),
                                          bits(kStepS),
                                          0u,
                                          periodic[0] ? 1u : 0u,
                                          periodic[1] ? 1u : 0u,
                                          periodic[2] ? 1u : 0u,
                                          0u,
                                          0u,
                                          static_cast<uint32_t>(total),
                                          static_cast<uint32_t>(2u * total),
                                          static_cast<uint32_t>(total)};
    const RunnerBuffer buffers[kBufferCount] = {
        {"airflow_params", params.size() * 4u, BufferKind::staged},
        {"airflow_u", n[0] * 4u, BufferKind::resident},
        {"airflow_v", n[1] * 4u, BufferKind::resident},
        {"airflow_w", n[2] * 4u, BufferKind::resident},
        {"airflow_pressure", 4u, BufferKind::device_only},  // bound, not read: set 1 is contiguous
        {"airflow_next", total * 4u, BufferKind::staged},
        {"airflow_advect", 3u * total * 4u, BufferKind::staged},
    };
    const SpirvVariantSet* const kernels[] = {&gen::kSpvVariants_airflow_advection};
    auto runner = ChainRunner::create(ctx, RunnerDesc{.buffers = buffers,
                                                      .kernels = kernels,
                                                      .push_bytes = sizeof(AdvectionPush),
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
    if (!check(r.upload(kParams, bytes_of(params))) || !check(r.upload(kU, bytes_of(in.u))) ||
        !check(r.upload(kV, bytes_of(in.v))) || !check(r.upload(kW, bytes_of(in.w)))) {
        return std::nullopt;
    }
    if (cpu_forward != nullptr) {
        std::vector<float> forward = cpu_forward->hat;
        forward.insert(forward.end(), cpu_forward->lo.begin(), cpu_forward->lo.end());
        forward.insert(forward.end(), cpu_forward->hi.begin(), cpu_forward->hi.end());
        if (!check(r.upload(kAdvect, bytes_of(forward)))) return std::nullopt;
    }
    std::vector<ChainCommand> chain;
    for (const uint32_t pass : {0u, 1u}) {
        if ((pass == 0u && passes == Passes::backward) || (pass == 1u && passes == Passes::forward)) continue;
        for (uint32_t c = 0; c < 3; ++c) {
            chain.push_back(dispatch({c, pass, static_cast<uint32_t>(n[c]), 0u}, workgroup_size));
        }
    }
    const auto id = r.record(chain);
    if (!id) {
        ADD_FAILURE() << id.error().context;
        return std::nullopt;
    }
    if (!check(r.submit(*id, 1))) return std::nullopt;
    Advected out;
    std::vector<float> forward(3u * total, 0.0f);
    out.next.assign(total, 0.0f);
    if (!check(r.read(kAdvect, writable_bytes_of(forward))) || !check(r.read(kNext, writable_bytes_of(out.next)))) {
        return std::nullopt;
    }
    out.hat.assign(forward.begin(), forward.begin() + static_cast<std::ptrdiff_t>(total));
    out.lo.assign(forward.begin() + static_cast<std::ptrdiff_t>(total),
                  forward.begin() + static_cast<std::ptrdiff_t>(2u * total));
    out.hi.assign(forward.begin() + static_cast<std::ptrdiff_t>(2u * total), forward.end());
    return out;
}

// How far the device's values are from the CPU's: the largest difference,
// and how many differ in any bit.
struct Diff {
    float worst = 0.0f;
    std::size_t differing = 0;
    std::size_t count = 0;
};

[[nodiscard]] Diff diff(const std::vector<float>& gpu, const std::vector<float>& cpu) {
    Diff d;
    d.count = cpu.size();
    if (gpu.size() != cpu.size()) {
        ADD_FAILURE() << "sizes differ: " << gpu.size() << " vs " << cpu.size();
        return d;
    }
    for (std::size_t i = 0; i < cpu.size(); ++i) {
        if (bits(gpu[i]) == bits(cpu[i])) continue;
        ++d.differing;
        const float delta = std::abs(gpu[i] - cpu[i]);
        d.worst = std::isfinite(delta) ? std::max(d.worst, delta) : std::numeric_limits<float>::infinity();
    }
    return d;
}

void expect_in_band(const VulkanContext& ctx, const char* what, const char* box, const Diff& d, float speed) {
    std::printf("[ advection ] %-8s %-24s worst %.3g m/s (%.3g of the top speed), %zu of %zu differ\n", box, what,
                static_cast<double>(d.worst), static_cast<double>(d.worst / speed), d.differing, d.count);
    if (spade::testing::airflow_bit_equal_device(ctx)) {
        EXPECT_EQ(d.differing, 0u) << box << ", " << what;
        return;
    }
    EXPECT_LE(d.worst, kBandRelative * speed) << box << ", " << what;
}

struct Box {
    const char* name;
    af::DomainBc bc;
};

[[nodiscard]] std::array<Box, 2> boxes() {
    return {Box{"clamped", clamped_box()}, Box{"periodic", periodic_x_and_z()}};
}

constexpr uint32_t kWorkgroupSize = 64;

// ===========================================================================
// GpuAirflowAdvection
// ===========================================================================

// The forward pass: trace_rk2 back from every face and advect()'s sample
// there, against advect.cpp's scratch (hat, and the stencil's lo and hi).
TEST(GpuAirflowAdvection, ForwardPassMatchesAdvectCpp) {
    if (const auto why = spade::testing::vulkan_skip_reason()) GTEST_SKIP() << *why;
    auto ctx = make_context();
    ASSERT_NE(ctx, nullptr);
    const af::MacGrid in = swirl(kShape);
    const float speed = largest_speed(in);
    for (const Box& box : boxes()) {
        const Advected cpu = advect_on_cpu(in, box.bc);
        const auto gpu = advect_on_gpu(*ctx, in, box.bc, Passes::forward, kWorkgroupSize);
        ASSERT_TRUE(gpu.has_value()) << box.name;
        expect_in_band(*ctx, "forward: hat", box.name, diff(gpu->hat, cpu.hat), speed);
        expect_in_band(*ctx, "forward: lo", box.name, diff(gpu->lo, cpu.lo), speed);
        expect_in_band(*ctx, "forward: hi", box.name, diff(gpu->hi, cpu.hi), speed);
    }
}

// The backward pass on its own: airflow_advect starts as the CPU's hat, lo
// and hi, so advect_back() and maccormack_clamp() alone stand between the
// device's result and advect.cpp's.
TEST(GpuAirflowAdvection, BackwardPassAndClampMatchAdvectCpp) {
    if (const auto why = spade::testing::vulkan_skip_reason()) GTEST_SKIP() << *why;
    auto ctx = make_context();
    ASSERT_NE(ctx, nullptr);
    const af::MacGrid in = swirl(kShape);
    const float speed = largest_speed(in);
    for (const Box& box : boxes()) {
        const Advected cpu = advect_on_cpu(in, box.bc);
        // The clamp must act somewhere, or this would not test it.
        std::size_t clamped = 0;
        for (std::size_t i = 0; i < cpu.next.size(); ++i) {
            clamped += (bits(cpu.next[i]) == bits(cpu.lo[i]) || bits(cpu.next[i]) == bits(cpu.hi[i])) ? 1u : 0u;
        }
        std::printf("[ advection ] %-8s %zu of %zu faces at the clamp's bounds\n", box.name, clamped, cpu.next.size());
        EXPECT_GT(clamped, cpu.next.size() / 50u) << box.name << ": the clamp hardly acts on this field";
        const auto gpu = advect_on_gpu(*ctx, in, box.bc, Passes::backward, kWorkgroupSize, &cpu);
        ASSERT_TRUE(gpu.has_value()) << box.name;
        expect_in_band(*ctx, "backward + clamp", box.name, diff(gpu->next, cpu.next), speed);
    }
}

// The two passes in one chain, as the fluid step records them.
TEST(GpuAirflowAdvection, BothPassesMatchAdvectCpp) {
    if (const auto why = spade::testing::vulkan_skip_reason()) GTEST_SKIP() << *why;
    auto ctx = make_context();
    ASSERT_NE(ctx, nullptr);
    const af::MacGrid in = swirl(kShape);
    const float speed = largest_speed(in);
    for (const Box& box : boxes()) {
        const Advected cpu = advect_on_cpu(in, box.bc);
        const auto gpu = advect_on_gpu(*ctx, in, box.bc, Passes::both, kWorkgroupSize);
        ASSERT_TRUE(gpu.has_value()) << box.name;
        expect_in_band(*ctx, "both passes", box.name, diff(gpu->next, cpu.next), speed);
    }
}

// No reduction, nothing shared between invocations: the workgroup size
// cannot change a bit.
TEST(GpuAirflowAdvection, SameBitsAtEveryWorkgroupSize) {
    if (const auto why = spade::testing::vulkan_skip_reason()) GTEST_SKIP() << *why;
    auto ctx = make_context();
    ASSERT_NE(ctx, nullptr);
    const af::MacGrid in = swirl(kShape);
    for (const Box& box : boxes()) {
        std::optional<Advected> first;
        for (const uint32_t wg : spade::compute::kSupportedWorkgroupSizes) {
            auto gpu = advect_on_gpu(*ctx, in, box.bc, Passes::both, wg);
            ASSERT_TRUE(gpu.has_value()) << box.name << " at " << wg;
            if (!first) {
                first = std::move(gpu);
                continue;
            }
            EXPECT_EQ(diff(gpu->next, first->next).differing, 0u) << box.name << " at " << wg;
            EXPECT_EQ(diff(gpu->hat, first->hat).differing, 0u) << box.name << " at " << wg;
        }
    }
}

}  // namespace
