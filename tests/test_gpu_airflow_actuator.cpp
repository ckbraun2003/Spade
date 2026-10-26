// ---------------------------------------------------------------------------
// test_gpu_airflow_actuator.cpp -- plan B, task B4: the rotor discs on the
// GPU (engine/shaders/kernels/airflow_actuator_gather.slang) against their
// CPU twin, spread_actuator() (engine/physics/airflow/actuator.cpp), on the
// chain runner (spike/airflow/chain_runner.hpp).
//
//   GpuAirflowActuator   the gather against spread_actuator() on the
//                        reference quad's four discs: level and tilted, with
//                        the rotors' swirl and thrust-only; the same bits at
//                        workgroup sizes 32, 64 and 128; and, opt-in, C8's
//                        far wake with the gather's increments in the CPU
//                        step, judged past its horizon against C8's one-ulp
//                        ensemble ("GPU gather in the CPU step": not the
//                        full-GPU C8, which follows when the step's chain
//                        exists).
//
// The quad's discs are assets/library/drones/reference_quad.model.yaml's
// four rotors (hubs 0.11 m out on x and z, 0.0264 m up; radius 0.0762 m;
// spins +1, -1, +1, -1) at hover thrust, m g / 4, and its torque ratio, in a
// 44 x 28 x 46 box at 1 cm (no two sizes alike). The increments start as
// fixed-seed noise, so the gather's read-add-write order shows.
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
#include <cstdlib>
#include <cstring>
#include <limits>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <thread>
#include <vector>

#include <glm/gtc/quaternion.hpp>

#include "airflow_actuator_gather.spv.gen.hpp"
#include "compute/backend.hpp"
#include "compute/spirv_variants.hpp"
#include "compute/vulkan/context.hpp"
#include "airflow_exact_device.hpp"
#include "compute/vulkan/dispatch_chain.hpp"
#include "gpu_skip.hpp"
#include "physics/airflow/actuator.hpp"
#include "physics/airflow/fluid_step.hpp"
#include "physics/airflow/mac_grid.hpp"
#include "physics/airflow/rotor_discs.hpp"
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

constexpr af::GridShape kShape{44, 28, 46, 0.01f};
constexpr float kRho = 1.225f;
constexpr float kStepS = 0.000125f;

// The gather is spread_actuator()'s bit for bit on the device of record:
// peskin4's square roots are fp32_math's sqrt32_ftz, and spread_actuator()
// flushes as the device does (TD-14 as amended 2026-10-22;
// airflow_exact_device.hpp). Another device or driver keeps the band the
// device's own sqrt needed, relative to the largest increment the discs add
// (plan B: 4x the largest measurement, at least 4 ulps): 2.08e-7 at worst on
// the RTX 3060 Ti, 2026-10-21 (thrust-only; level 1.01e-7, tilted 1.90e-7).
// Red check: peskin4's 4 a^2 as 4.01 a^2 fails every case by about 1e-3.
constexpr float kBandRelative = 4.0f * 2.08e-7f;

// Set 1's bindings the gather uses (bindings.slang section F); the runner
// binds buffers[i] at binding i, so every binding up to the last is given.
constexpr uint32_t kParams = 0, kIncrements = 7, kDiscs = 22, kBufferCount = 23;
constexpr uint32_t kDiscStride = 304;  // floats per disc record (airflow_actuator_gather.slang)

// The kernel's push block: face_count in word 2, as every airflow kernel's.
struct GatherPush {
    uint32_t component = 0;
    uint32_t unused = 0;
    uint32_t face_count = 0;
    uint32_t disc_count = 0;
};

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

template <class T>
[[nodiscard]] std::span<const std::byte> bytes_of(const std::vector<T>& v) noexcept {
    return std::as_bytes(std::span<const T>(v));
}

template <class T>
[[nodiscard]] std::span<std::byte> writable_bytes_of(std::vector<T>& v) noexcept {
    return std::as_writable_bytes(std::span<T>(v));
}

// The reference quad's four discs at hover, its body at `orient`, its rotors'
// hubs about the box's middle; `swirl` false gives them no torque.
[[nodiscard]] std::vector<af::ActuatorDisc> quad_discs(const af::GridShape& s, glm::quat orient, bool swirl) {
    constexpr float kHubOut = 0.109999999f;
    constexpr float kHubUp = 0.0264030416f;
    constexpr float kRadius = 0.0762000009f;
    constexpr float kMass = 0.526000023f;
    constexpr float kTorquePerThrust = 2.04075654e-08f / 2.00861859e-06f;  // torque_coeff / thrust_coeff
    const float thrust = kMass * 9.80665f / 4.0f;
    const glm::vec3 hubs[4] = {{kHubOut, kHubUp, 0.0f}, {0.0f, kHubUp, kHubOut}, {-kHubOut, kHubUp, 0.0f},
                               {0.0f, kHubUp, -kHubOut}};
    const float spins[4] = {1.0f, -1.0f, 1.0f, -1.0f};
    const glm::vec3 middle = 0.5f * s.dx * glm::vec3(static_cast<float>(s.nx), static_cast<float>(s.ny),
                                                     static_cast<float>(s.nz));
    const af::DiscBody body{.com_world = middle - glm::vec3(0.0f, kHubUp, 0.0f), .orient_world = orient};
    std::vector<af::ActuatorDisc> out;
    for (uint32_t r = 0; r < 4; ++r) {
        const af::DiscRotor rotor{.hub_body = hubs[r],
                                  .radius = kRadius,
                                  .spin_dir = spins[r],
                                  .thrust = thrust,
                                  .torque = swirl ? thrust * kTorquePerThrust : 0.0f};
        const auto disc = af::disc_from_rotor(rotor, body, glm::vec3(0.0f), s);
        EXPECT_TRUE(disc.has_value()) << "rotor " << r << ": " << disc.error().context;
        if (disc) out.push_back(*disc);
    }
    return out;
}

// The discs' records, as the kernel reads them, from disc_spread().
[[nodiscard]] std::vector<float> disc_records(std::span<const af::ActuatorDisc> discs, const af::GridShape& s,
                                              float rho, float h) {
    std::vector<float> out(std::max<std::size_t>(discs.size(), 1u) * kDiscStride, 0.0f);
    for (std::size_t d = 0; d < discs.size(); ++d) {
        const af::DiscSpread sp = af::disc_spread(discs[d], s, rho, h);
        float* r = out.data() + d * kDiscStride;
        for (uint32_t a = 0; a < 3; ++a) {
            r[a] = discs[d].centre[static_cast<glm::length_t>(a)];
            r[4 + a] = discs[d].axis[static_cast<glm::length_t>(a)];
        }
        r[3] = sp.reach;
        r[7] = sp.scale;
        r[8] = sp.swirl_scale;
        r[9] = sp.swirl ? 1.0f : 0.0f;
        for (uint32_t i = 0; i < af::kDiscPoints; ++i) {
            for (uint32_t a = 0; a < 3; ++a) {
                r[16 + 3 * i + a] = sp.points[i][static_cast<glm::length_t>(a)];
                r[160 + 3 * i + a] = sp.tangent[i][static_cast<glm::length_t>(a)];
            }
        }
    }
    return out;
}

// du, dv and dw packed u, v, w, as airflow_increments holds them.
[[nodiscard]] std::vector<float> packed(const std::vector<float>& du, const std::vector<float>& dv,
                                        const std::vector<float>& dw) {
    std::vector<float> out(du);
    out.insert(out.end(), dv.begin(), dv.end());
    out.insert(out.end(), dw.begin(), dw.end());
    return out;
}

// spread_actuator() for every disc, in order, onto `start`'s copy.
[[nodiscard]] std::vector<float> spread_on_cpu(const af::GridShape& s, std::span<const af::ActuatorDisc> discs,
                                               const std::vector<float>& start, float h) {
    const auto n = face_counts(s);
    std::vector<float> du(start.begin(), start.begin() + static_cast<std::ptrdiff_t>(n[0]));
    std::vector<float> dv(start.begin() + static_cast<std::ptrdiff_t>(n[0]),
                          start.begin() + static_cast<std::ptrdiff_t>(n[0] + n[1]));
    std::vector<float> dw(start.begin() + static_cast<std::ptrdiff_t>(n[0] + n[1]), start.end());
    for (const af::ActuatorDisc& d : discs) af::spread_actuator(d, s, kRho, h, du, dv, dw);
    return packed(du, dv, dw);
}

// The gather on the device, from `start`; the increments it leaves. With
// `best_ms`, also the three components' time, the fastest of five submits.
[[nodiscard]] std::optional<std::vector<float>> gather_on_gpu(VulkanContext& ctx, const af::GridShape& s,
                                                              std::span<const af::ActuatorDisc> discs,
                                                              const std::vector<float>& start, float h,
                                                              uint32_t workgroup_size, double* best_ms = nullptr) {
    const auto n = face_counts(s);
    std::vector<uint32_t> params(16u, 0u);
    params[0] = s.nx;
    params[1] = s.ny;
    params[2] = s.nz;
    params[4] = bits(s.dx);
    params[5] = bits(1.0f / s.dx);
    const std::vector<float> records = disc_records(discs, s, kRho, h);
    std::vector<RunnerBuffer> buffers(kBufferCount);
    for (uint32_t b = 0; b < kBufferCount; ++b) {
        buffers[b] = {"airflow_unused_" + std::to_string(b), 4u, BufferKind::device_only};
    }
    buffers[kParams] = {"airflow_params", params.size() * 4u, BufferKind::staged};
    buffers[kIncrements] = {"airflow_increments", start.size() * 4u, BufferKind::staged};
    buffers[kDiscs] = {"airflow_discs", records.size() * 4u, BufferKind::staged};
    const SpirvVariantSet* const kernels[] = {&gen::kSpvVariants_airflow_actuator_gather};
    auto runner = ChainRunner::create(ctx, RunnerDesc{.buffers = buffers,
                                                      .kernels = kernels,
                                                      .push_bytes = sizeof(GatherPush),
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
    if (!check(r.upload(kParams, bytes_of(params))) || !check(r.upload(kIncrements, bytes_of(start))) ||
        !check(r.upload(kDiscs, bytes_of(records)))) {
        return std::nullopt;
    }
    std::vector<ChainCommand> chain;
    chain.push_back(ChainCommand{});  // marks either side, for the time
    for (uint32_t c = 0; c < 3; ++c) {
        const GatherPush push{c, 0u, static_cast<uint32_t>(n[c]), static_cast<uint32_t>(discs.size())};
        ChainCommand cmd{.is_dispatch = true, .pipeline = 0};
        std::memcpy(cmd.push.data(), &push, sizeof push);
        cmd.groups = {(push.face_count + workgroup_size - 1u) / workgroup_size, 1u, 1u};
        chain.push_back(cmd);
    }
    chain.push_back(ChainCommand{});
    const auto id = r.record(chain);
    if (!id) {
        ADD_FAILURE() << id.error().context;
        return std::nullopt;
    }
    if (best_ms != nullptr) {
        *best_ms = 0.0;
        for (int run = 0; run < 5; ++run) {
            if (!check(r.submit(*id, 1))) return std::nullopt;
            const auto d = r.durations_ns(*id);
            if (!d) {
                ADD_FAILURE() << d.error().context;
                return std::nullopt;
            }
            if (d->empty()) break;  // the device cannot time compute work
            uint64_t sum = 0;
            for (const uint64_t x : *d) sum += x;
            const double ms = static_cast<double>(sum) * 1e-6;
            if (run == 0 || ms < *best_ms) *best_ms = ms;
        }
        if (!check(r.upload(kIncrements, bytes_of(start)))) return std::nullopt;
    }
    if (!check(r.submit(*id, 1))) return std::nullopt;
    std::vector<float> out(start.size(), 0.0f);
    if (!check(r.read(kIncrements, writable_bytes_of(out)))) return std::nullopt;
    return out;
}

// Fixed-seed noise of up to 1e-3 m/s on every face.
[[nodiscard]] std::vector<float> noisy_start(const af::GridShape& s) {
    const auto n = face_counts(s);
    std::vector<float> out(n[0] + n[1] + n[2]);
    SplitMix rng{0xD15Cull};
    for (float& v : out) v = 1.0e-3f * rng.symmetric();
    return out;
}

struct Diff {
    float worst = 0.0f;
    std::size_t differing = 0;
};

[[nodiscard]] Diff diff(const std::vector<float>& gpu, const std::vector<float>& cpu) {
    Diff d;
    for (std::size_t i = 0; i < cpu.size(); ++i) {
        if (bits(gpu[i]) == bits(cpu[i])) continue;
        ++d.differing;
        const float delta = std::abs(gpu[i] - cpu[i]);
        d.worst = std::isfinite(delta) ? std::max(d.worst, delta) : std::numeric_limits<float>::infinity();
    }
    return d;
}

// The largest increment the discs add: the scale the band is relative to.
[[nodiscard]] float largest_added(const std::vector<float>& after, const std::vector<float>& start) {
    float top = 0.0f;
    for (std::size_t i = 0; i < start.size(); ++i) top = std::max(top, std::abs(after[i] - start[i]));
    return top;
}

struct Case {
    const char* name;
    glm::quat orient;
    bool swirl;
};

[[nodiscard]] std::array<Case, 3> cases() {
    const glm::quat tilted = glm::angleAxis(0.3f, glm::normalize(glm::vec3(1.0f, 0.0f, 0.4f)));
    return {Case{"level", glm::quat(1.0f, 0.0f, 0.0f, 0.0f), true}, Case{"tilted", tilted, true},
            Case{"thrust-only", tilted, false}};
}

constexpr uint32_t kWorkgroupSize = 64;

// ===========================================================================
// GpuAirflowActuator
// ===========================================================================

// The gather against spread_actuator(), disc after disc, from the same
// increments: the reference quad level and tilted with its rotors' swirl, and
// tilted thrust-only (no swirl: a component the axes do not reach is skipped).
TEST(GpuAirflowActuator, GatherMatchesSpreadActuatorOnTheQuadsDiscs) {
    if (const auto why = spade::testing::vulkan_skip_reason()) GTEST_SKIP() << *why;
    auto ctx = make_context();
    ASSERT_NE(ctx, nullptr);
    const bool bit_equal = spade::testing::airflow_bit_equal_device(*ctx);
    const std::vector<float> start = noisy_start(kShape);
    for (const Case& c : cases()) {
        const std::vector<af::ActuatorDisc> discs = quad_discs(kShape, c.orient, c.swirl);
        ASSERT_EQ(discs.size(), 4u) << c.name;
        const std::vector<float> cpu = spread_on_cpu(kShape, discs, start, kStepS);
        const auto gpu = gather_on_gpu(*ctx, kShape, discs, start, kStepS, kWorkgroupSize);
        ASSERT_TRUE(gpu.has_value()) << c.name;
        const float added = largest_added(cpu, start);
        std::size_t touched = 0;
        for (std::size_t i = 0; i < start.size(); ++i) touched += bits(cpu[i]) != bits(start[i]) ? 1u : 0u;
        ASSERT_GT(added, 0.0f) << c.name << ": the discs added nothing";
        const Diff d = diff(*gpu, cpu);
        std::printf("[ actuator ] %-11s %zu of %zu faces touched (largest increment %.3g m/s); %zu differ, worst "
                    "%.3g m/s (%.3g of it)\n",
                    c.name, touched, start.size(), static_cast<double>(added), d.differing,
                    static_cast<double>(d.worst), static_cast<double>(d.worst / added));
        if (bit_equal) {
            EXPECT_EQ(d.differing, 0u) << c.name;
        } else {
            EXPECT_LE(d.worst, kBandRelative * added) << c.name;
        }
    }
}

// The gather's time in the D1 box (240 x 177 x 168 at 4 mm, R/dx 19), the
// quad's discs tilted with swirl at the box's middle: u, v and w, the fastest
// of five. Opt-in with SPADE_AIRFLOW_TIMINGS, as the projection's D1 time is;
// it prices the exact square root against the device's own (the D1-box speed
// check, the lead, 2026-10-22).
TEST(GpuAirflowActuator, GatherTimeInTheD1Box) {
    if (const auto why = spade::testing::vulkan_skip_reason()) GTEST_SKIP() << *why;
#if defined(_MSC_VER)
#pragma warning(push)
#pragma warning(disable : 4996)
#endif
    const bool timings = std::getenv("SPADE_AIRFLOW_TIMINGS") != nullptr;
#if defined(_MSC_VER)
#pragma warning(pop)
#endif
    if (!timings) GTEST_SKIP() << "opt-in: SPADE_AIRFLOW_TIMINGS=1";
    auto ctx = make_context();
    ASSERT_NE(ctx, nullptr);
    constexpr af::GridShape kD1{240, 177, 168, 0.004f};
    const std::vector<float> start = noisy_start(kD1);
    const std::vector<af::ActuatorDisc> discs = quad_discs(kD1, cases()[1].orient, true);
    double ms = 0.0;
    ASSERT_TRUE(gather_on_gpu(*ctx, kD1, discs, start, kStepS, kWorkgroupSize, &ms).has_value());
    std::printf("measure: the gather in the D1 box, 240x177x168 at 4 mm: %.3f ms for u, v and w\n", ms);
}

// No reduction, nothing shared between invocations: the workgroup size
// cannot change a bit.
TEST(GpuAirflowActuator, SameBitsAtEveryWorkgroupSize) {
    if (const auto why = spade::testing::vulkan_skip_reason()) GTEST_SKIP() << *why;
    auto ctx = make_context();
    ASSERT_NE(ctx, nullptr);
    const std::vector<float> start = noisy_start(kShape);
    const std::vector<af::ActuatorDisc> discs = quad_discs(kShape, cases()[1].orient, true);
    std::optional<std::vector<float>> first;
    for (const uint32_t wg : spade::compute::kSupportedWorkgroupSizes) {
        auto gpu = gather_on_gpu(*ctx, kShape, discs, start, kStepS, wg);
        ASSERT_TRUE(gpu.has_value()) << wg;
        if (!first) {
            first = std::move(gpu);
            continue;
        }
        EXPECT_EQ(diff(*gpu, *first).differing, 0u) << "at " << wg;
    }
}

[[nodiscard]] bool validation_enabled() {
#if defined(_MSC_VER)
#pragma warning(push)
#pragma warning(disable : 4996)
#endif
    return std::getenv("SPADE_AIRFLOW_VALIDATION") != nullptr;
#if defined(_MSC_VER)
#pragma warning(pop)
#endif
}

// C8's far wake (test_airflow_validation.cpp's C8ActuatorDiscFarWake...,
// its record tests/data/airflow/records/c8-actuator-disc.md) on the CPU step,
// with the GPU gather's increments in place of spread_actuator()'s ("GPU
// gather in the CPU step": not the full-GPU C8). The disc and the step are
// fixed, so the gather's increments are the same every step: gathered once,
// from zero, as the step's cleared increments are. The control: the CPU's own
// increments copied in the same way are the CPU's run to the bit.
//
// ON THE DEVICE OF RECORD (airflow_exact_device.hpp) the gather is
// spread_actuator()'s bit for bit (peskin4's roots are sqrt32_ftz; TD-14 as
// amended 2026-10-22), so the run IS the CPU's run: every gathered increment
// and both probes' series over all 768 steps equal the CPU's, and the record
// block says so, with C8's verdict the CPU run's (K2b; the lead, 2026-10-26,
// on Test/Docs' plan). Its red is B4's banded block, b8ec4c9, on record.
//
// ELSEWHERE the gather may differ from spread_actuator() in last bits, and
// the far wake is chaotic past its horizon T_p, so the run is judged as a
// path that is not bit-identical to the CPU's (test-docs/01-verification.md,
// "A case judged past its horizon"; C8's numbers from its re-cut record,
// abaafcc, and docs/design/test-docs/2026-10-22-c8-spread.md):
//   * up to T_p (step 104), both probes' series against the CPU's run, banded
//     per TD-14: 4x the measured gap, never under 4 ulps of the scene's scale;
//   * past T_p, C8's measure, the mean downwash over the 80 v faces within
//     0.5R of the axis 2R below over steps 385 to 768: within C8's 10% of
//     momentum theory, and within 4x the ensemble's largest deviation
//     (0.0437 m/s) of the ensemble's mean (8.0333 m/s);
//   * the run leaves the CPU's by 1e-3 on the axis probe no sooner than the
//     one-ulp ensemble's earliest member does (step 141 of 16).
// Opt-in with C8 (SPADE_AIRFLOW_VALIDATION=1): three 768-step runs side by
// side, about 6 minutes, so, as C8 is, run from the test binary with
// --gtest_filter (ctest gives a Gpu test 180 s).
TEST(GpuAirflowActuator, C8GpuGatherInTheCpuStep) {
    if (const auto why = spade::testing::vulkan_skip_reason()) GTEST_SKIP() << *why;
    if (!validation_enabled()) GTEST_SKIP() << "set SPADE_AIRFLOW_VALIDATION=1 to run C8 three times (~6 min)";
    auto ctx = make_context();
    ASSERT_NE(ctx, nullptr);
    const af::GridShape shape{64, 128, 64, 0.01f};
    const af::DomainBc bc;  // open faces, still air: every face an outlet
    const float vh = 4.0f;
    const float radius = 0.1f;
    const float area = static_cast<float>(3.14159265358979323846) * radius * radius;
    const af::ActuatorDisc disc{.centre = {0.32f, 0.96f, 0.32f}, .axis = {0.0f, -1.0f, 0.0f}, .radius = radius,
                                .thrust = 2.0f * kRho * area * vh * vh};
    ASSERT_TRUE(af::check_disc(disc, shape).has_value());
    const af::FluidConfig cfg{.rho = kRho, .nu = 0.0f, .h = shape.dx / 8.0f, .n_v = 2};
    const auto n = face_counts(shape);
    const std::vector<float> zero(n[0] + n[1] + n[2], 0.0f);
    const auto gathered = gather_on_gpu(*ctx, shape, std::span<const af::ActuatorDisc>(&disc, 1), zero, cfg.h,
                                        kWorkgroupSize);
    ASSERT_TRUE(gathered.has_value());
    const std::vector<float> spread = spread_on_cpu(shape, std::span<const af::ActuatorDisc>(&disc, 1), zero, cfg.h);
    const Diff once = diff(*gathered, spread);

    constexpr uint32_t kSteps = 768;
    constexpr uint32_t kTp = 104;                  // C8's horizon (the spread note, section 4)
    constexpr uint32_t kFirst = 385;               // C8's window: steps 385 to 768
    constexpr double kEnsembleMean = 8.0333;       // the eight runs' mean of the measure (abaafcc)
    constexpr double kEnsembleDeviation = 0.0437;  // their largest deviation from it
    constexpr uint32_t kEarliestParting = 141;     // the 16 members' earliest 1e-3 parting
    // Up to T_p: 4x the measured gap, never under 4 ulps of momentum theory's
    // 7.5777 m/s as a float (2^-21 m/s each). Measured 2026-10-22 at 3eacd6a
    // (RTX 3060 Ti): the axis 3.45e-5, the 0.5R mean 7.21e-6. The axis is a
    // mean of 4 faces, so it carries the gather's 110 differing increments
    // (2.38e-7 at most) less averaged than the 80 faces of the 0.5R mean.
    constexpr double kUlpFloor = 4.0 * 0x1p-21;
    constexpr double kAxisGapMeasured = 3.45e-5;
    constexpr double kDiscGapMeasured = 7.21e-6;
    const double axis_band = std::max(4.0 * kAxisGapMeasured, kUlpFloor);
    const double disc_band = std::max(4.0 * kDiscGapMeasured, kUlpFloor);

    // The v faces 2R below (j = 76) within 0.5R (5 cells) of the axis (x = z =
    // 0.32 m, the corner of cells 31 and 32), by their centres, as C8 picks them.
    constexpr uint32_t kProbeJ = 76;
    std::vector<std::size_t> disc_half_r;
    for (uint32_t k = 0; k < shape.nz; ++k) {
        for (uint32_t i = 0; i < shape.nx; ++i) {
            const int64_t a = 2 * int64_t{i} + 1 - 64;
            const int64_t b = 2 * int64_t{k} + 1 - 64;
            if (a * a + b * b <= 4 * 5 * 5) disc_half_r.push_back(af::v_index(shape, i, kProbeJ, k));
        }
    }
    ASSERT_EQ(disc_half_r.size(), 80u);

    // One run from still air, both probes (downwash, -v) after every step:
    // spread_actuator() each step (null), or `fixed` copied in as the step's
    // increments.
    struct Probes {
        std::vector<double> axis;
        std::vector<double> disc;
        std::optional<uint32_t> fault_at;
    };
    const auto run = [&](const std::vector<float>* fixed) {
        Probes out;
        af::MacGrid g = *af::make_grid(shape, bc);
        af::FluidScratch s = *af::make_fluid_scratch(g, bc, cfg);
        const auto nu = static_cast<std::ptrdiff_t>(n[0]);
        const auto nv = static_cast<std::ptrdiff_t>(n[1]);
        for (uint32_t k = 0; k < kSteps; ++k) {
            if (fixed != nullptr) {
                std::copy(fixed->begin(), fixed->begin() + nu, s.du.begin());
                std::copy(fixed->begin() + nu, fixed->begin() + nu + nv, s.dv.begin());
                std::copy(fixed->begin() + nu + nv, fixed->end(), s.dw.begin());
            } else {
                af::spread_actuator(disc, shape, kRho, cfg.h, s.du, s.dv, s.dw);
            }
            if (af::fluid_step(g, bc, cfg, s).fault != af::FluidFault::none) {
                out.fault_at = k + 1;
                break;
            }
            const auto v = [&](uint32_t i, uint32_t kk) {
                return static_cast<double>(g.v[af::v_index(shape, i, kProbeJ, kk)]);
            };
            out.axis.push_back(-0.25 * (v(31, 31) + v(32, 31) + v(31, 32) + v(32, 32)));
            double sum = 0.0;
            for (const std::size_t f : disc_half_r) sum += static_cast<double>(g.v[f]);
            out.disc.push_back(-sum / static_cast<double>(disc_half_r.size()));
        }
        return out;
    };
    // Side by side; each run is single-threaded, so its bits are the same
    // whatever runs beside it.
    Probes cpu;
    Probes copied;
    Probes gpu;
    {
        std::thread a([&] { cpu = run(nullptr); });
        std::thread b([&] { copied = run(&spread); });
        std::thread c([&] { gpu = run(&*gathered); });
        a.join();
        b.join();
        c.join();
    }
    for (const Probes* p : {&cpu, &copied, &gpu}) {
        ASSERT_FALSE(p->fault_at.has_value()) << "a run faulted at step " << *p->fault_at;
    }

    double axis_gap = 0.0;
    double disc_gap = 0.0;
    for (uint32_t k = 0; k < kTp; ++k) {
        axis_gap = std::max(axis_gap, std::abs(gpu.axis[k] - cpu.axis[k]));
        disc_gap = std::max(disc_gap, std::abs(gpu.disc[k] - cpu.disc[k]));
    }
    const auto window = [](const std::vector<double>& probe, uint32_t first, uint32_t last) {
        double sum = 0.0;
        for (uint32_t k = first; k <= last; ++k) sum += probe[k - 1];
        return sum / static_cast<double>(last - first + 1);
    };
    // The first step at which `run` leaves the CPU's by more than 1e-3 of it
    // (kSteps: never).
    const auto parts_at = [](const std::vector<double>& run, const std::vector<double>& base) {
        for (std::size_t k = 0; k < base.size(); ++k) {
            if (std::abs(run[k] - base[k]) > 1.0e-3 * std::abs(base[k])) return static_cast<uint32_t>(k + 1);
        }
        return kSteps;
    };
    const double expected = vh * (1.0 + 2.0 / std::sqrt(5.0));
    const double mean = window(gpu.disc, kFirst, kSteps);
    const uint32_t parting = parts_at(gpu.axis, cpu.axis);
    const bool theory_within = std::abs(mean - expected) <= 0.1 * expected;
    const bool spread_within = std::abs(mean - kEnsembleMean) <= 4.0 * kEnsembleDeviation;
    const auto verdict = [](bool within) { return within ? "within" : "MISS"; };
    // The copy is the step's own increments: the CPU's through it are the CPU.
    EXPECT_EQ(copied.axis, cpu.axis) << "copying the increments in is not the step's spread";
    EXPECT_EQ(copied.disc, cpu.disc) << "copying the increments in is not the step's spread";

    if (spade::testing::airflow_bit_equal_device(*ctx)) {
        std::size_t series_differing = 0;
        for (uint32_t k = 0; k < kSteps; ++k) {
            series_differing += (gpu.axis[k] != cpu.axis[k] ? 1u : 0u) + (gpu.disc[k] != cpu.disc[k] ? 1u : 0u);
        }
        const bool increments_equal = once.differing == 0u;
        const bool series_equal = series_differing == 0u;
        std::printf("=== validation record: C8 actuator-disc far wake, GPU gather in the CPU step ===\n"
                    "path: the CPU step (release) with airflow_actuator_gather's increments (plan B, B4; its square "
                    "roots fp32_math's sqrt32_ftz), on %s\n"
                    "config: as C8: 64x128x64 at dx 1 cm, R 0.1 m (10 cells), T %.4f N, v_h 4 m/s, still air, inviscid, "
                    "h = %.6f, n_v 2, %u steps; the gather's increments copied in each step\n"
                    "measure: the gathered increments differing from spread_actuator()'s in any bit = %zu of %zu\n"
                    "reference: spread_actuator(), the CPU step's own increments\n"
                    "tolerance: 0, bit-equal (TD-14 as amended 2026-10-22: an exact kernel on the device of record)\n"
                    "verdict: %s\n"
                    "measure: both probes' values (the axis and the 0.5R mean, 2R below) differing from the CPU's run "
                    "over steps 1 to %u = %zu of %u\n"
                    "reference: the CPU's run, spread_actuator() each step\n"
                    "tolerance: 0, bit-equal: the run is the CPU's run\n"
                    "verdict: %s\n"
                    "measure: the mean downwash over the 80 v faces within 0.5R of the axis, 2R below the disc, steps "
                    "%u to %u = %.4f m/s\n"
                    "reference: momentum theory, v_h (1 + 2/sqrt 5) = %.4f m/s (tests/data/airflow/README.md, C8)\n"
                    "tolerance: within 10%% (C8's: plan A Task 11, spec \xc2\xa7" "11.2); the CPU run's verdict, the "
                    "run being the CPU's\n"
                    "verdict: %s\n",
                    spade::compute::describe(ctx->device_report()).c_str(), static_cast<double>(disc.thrust),
                    static_cast<double>(cfg.h), kSteps, once.differing, spread.size(),
                    verdict(increments_equal), kSteps, series_differing, 2u * kSteps, verdict(series_equal), kFirst,
                    kSteps, mean,
                    expected, verdict(theory_within));
        EXPECT_EQ(once.differing, 0u) << "the gather is not spread_actuator()'s on the device of record";
        EXPECT_EQ(gpu.axis, cpu.axis) << "the axis probe's series is not the CPU's";
        EXPECT_EQ(gpu.disc, cpu.disc) << "the 0.5R mean's series is not the CPU's";
        EXPECT_TRUE(theory_within) << mean << " against momentum theory's " << expected;
        return;
    }
    std::printf("=== validation record: C8 actuator-disc far wake, GPU gather in the CPU step ===\n"
                "path: the CPU step (release) with airflow_actuator_gather's increments (plan B, B4), on %s\n"
                "config: as C8: 64x128x64 at dx 1 cm, R 0.1 m (10 cells), T %.4f N, v_h 4 m/s, still air, inviscid, "
                "h = %.6f, n_v 2, %u steps; the gather's increments, %zu of %zu differing from spread_actuator's by "
                "%.3g m/s at most, copied in each step\n"
                "measure: the mean downwash over the 80 v faces within 0.5R of the axis, 2R below the disc, steps %u "
                "to %u = %.4f m/s\n"
                "reference: momentum theory, v_h (1 + 2/sqrt 5) = %.4f m/s (tests/data/airflow/README.md, C8)\n"
                "tolerance: within 10%% (C8's: plan A Task 11, spec \xc2\xa7" "11.2)\n"
                "verdict: %s\n"
                "measure: the same mean = %.4f m/s\n"
                "reference: C8's one-ulp ensemble, the mean of its eight runs = %.4f m/s (C8's record, abaafcc)\n"
                "tolerance: within 4x the ensemble's largest deviation, 4 x %.4f = %.4f m/s (test-docs/"
                "01-verification.md, a case judged past its horizon)\n"
                "verdict: %s\n"
                "measure: the 0.5R mean's largest |gather - CPU| through step %u (T_p) = %.2e m/s\n"
                "reference: the CPU's run, spread_actuator() each step\n"
                "tolerance: within 4x the measured gap, never under 4 ulps, %.2e m/s (TD-14; the gap measured "
                "2026-10-22 at 3eacd6a)\n"
                "verdict: %s\n"
                "measure: the axis probe's largest |gather - CPU| through step %u (T_p) = %.2e m/s\n"
                "reference: the CPU's run, spread_actuator() each step\n"
                "tolerance: within 4x the measured gap, never under 4 ulps, %.2e m/s (TD-14; the gap measured "
                "2026-10-22 at 3eacd6a)\n"
                "verdict: %s\n"
                "measure: the step at which the run leaves the CPU's by 1e-3 on the axis 2R below = %u\n"
                "reference: the one-ulp ensemble's earliest member, step %u of 16 (2026-10-22-c8-spread.md)\n"
                "tolerance: no sooner than step %u (2026-10-22-c8-spread.md, fixed before the run)\n"
                "verdict: %s\n"
                "measure: the CPU's run, the same 0.5R mean = %.4f m/s\n"
                "reference: none\n"
                "tolerance: none (report only)\n"
                "verdict: report only\n",
                spade::compute::describe(ctx->device_report()).c_str(), static_cast<double>(disc.thrust),
                static_cast<double>(cfg.h), kSteps, once.differing, spread.size(), static_cast<double>(once.worst),
                kFirst, kSteps, mean, expected, verdict(theory_within), mean, kEnsembleMean, kEnsembleDeviation,
                4.0 * kEnsembleDeviation, verdict(spread_within), kTp, disc_gap, disc_band,
                verdict(disc_gap <= disc_band), kTp, axis_gap, axis_band, verdict(axis_gap <= axis_band), parting,
                kEarliestParting, kEarliestParting, verdict(parting >= kEarliestParting),
                window(cpu.disc, kFirst, kSteps));
    EXPECT_LE(disc_gap, disc_band) << "the 0.5R mean up to T_p";
    EXPECT_LE(axis_gap, axis_band) << "the axis probe up to T_p";
    EXPECT_TRUE(theory_within) << mean << " against momentum theory's " << expected;
    EXPECT_TRUE(spread_within) << mean << " against the ensemble's " << kEnsembleMean;
    EXPECT_GE(parting, kEarliestParting) << "the gather's run parts sooner than any one-ulp member's";
}

}  // namespace
