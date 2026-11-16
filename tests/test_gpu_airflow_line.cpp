// ---------------------------------------------------------------------------
// test_gpu_airflow_line.cpp -- the actuator line's device twin (P4c, L61; spike/airflow/line_chain.hpp, the kernels
// engine/shaders/kernels/airflow_line_{sample,load,spread}.slang, the plan
// .superpowers/sdd/2026-11-10-wake/p4c-plan.md).
//
//   GpuAirflowLine.PackedRecordsRunThroughTheKernelsAsSpreadLineBitForBit   no device: the three kernels written out
//       in plain C++ over the PACKED records (the kernels' own expressions and loop order), against spread_line() on
//       the same grid, bit for bit: du, dv, dw and the reports, two rotors (a left hand and a tilted axis), three
//       kernel widths, four sub-azimuths. It pins the records' layout and the gather order before a shader is run.
//   GpuAirflowLine.TheFluidStepsALineAndMatchesTheCpu   GpuFluid::step(discs, lines) against the CPU's spread_line() +
//       spread_actuator() + fluid_step() for three steps, a rotor and a disc: u, v, w, p and the report equal on the
//       device of record.
//   GpuAirflowLine.TheKernelsOnTheDeviceMatchSpreadLine   the same case on the device (skipped without one): the sample, load and spread
//       kernels over the packed records, du, dv, dw and the reports against spread_line(): zero faces differ on the device of record
//       (airflow_bit_equal_device), the largest relative difference reported elsewhere.
// ---------------------------------------------------------------------------

#include <gtest/gtest.h>

#include <algorithm>
#include <bit>
#include <cmath>
#include <cstdint>
#include <array>
#include <cstdio>
#include <cstring>
#include <memory>
#include <span>
#include <string>
#include <vector>

#include <glm/glm.hpp>

#include "airflow_exact_device.hpp"
#include "airflow_line_load.spv.gen.hpp"
#include "airflow_line_sample.spv.gen.hpp"
#include "airflow_line_spread.spv.gen.hpp"
#include "compute/backend.hpp"
#include "compute/spirv_variants.hpp"
#include "compute/vulkan/context.hpp"
#include "compute/vulkan/dispatch_chain.hpp"
#include "core/detail/flush_subnormals.hpp"
#include "gpu_skip.hpp"
#include "physics/airflow/actuator.hpp"
#include "physics/airflow/blade.hpp"
#include "physics/airflow/fluid_step.hpp"
#include "physics/airflow/line.hpp"
#include "physics/airflow/mac_grid.hpp"
#include "spike/airflow/chain_runner.hpp"
#include "spike/airflow/line_chain.hpp"
#include "spike/airflow/step_chain.hpp"

namespace {

namespace af = spade::physics::airflow;
namespace ag = spade::airflow_gpu;

constexpr float kDx = 0.004f;
constexpr float kRho = 1.225f;
constexpr float kH = 0.0005f;
constexpr uint32_t kN = 52;  // 0.208 m: reach R + 6 dx = 0.1002 m about a rotor near the middle

af::BladeShape blade_of(uint32_t blades) {
    af::BladeShape b;
    b.blades = blades;
    b.radius = 0.0762f;
    b.stations = {{0.0114f, 0.0085f, 30.0f}, {0.0457f, 0.0148f, 22.0f}, {0.0762f, 0.0010f, 14.0f}};
    return b;
}

// A smooth-ish field with a different value at every face (an LCG's, in [-3, 3] m/s).
af::MacGrid noisy_grid() {
    af::MacGrid g;
    g.shape = {kN, kN, kN, kDx};
    uint32_t state = 0x1234ABCDu;
    const auto next = [&state] {
        state = state * 1664525u + 1013904223u;
        return (static_cast<float>(state >> 8) / 16777216.0f - 0.5f) * 6.0f;
    };
    g.u.resize(std::size_t{kN + 1u} * kN * kN);
    g.v.resize(std::size_t{kN} * (kN + 1u) * kN);
    g.w.resize(std::size_t{kN} * kN * (kN + 1u));
    g.p.assign(std::size_t{kN} * kN * kN, 0.0f);
    for (float& x : g.u) x = next();
    for (float& x : g.v) x = next();
    for (float& x : g.w) x = next();
    return g;
}

// --- the kernels, in plain C++ over the packed floats ---------------------------------------
struct Taps {
    int lo[3];
    int n[3];
    float s;
    const float* w[3];
};

Taps taps_of(const float* rec, uint32_t c) {
    const float* comp = rec + ag::kLineHeaderWords + c * ag::kLineComponentWords;
    Taps t{};
    for (uint32_t a = 0; a < 3; ++a) {
        t.lo[a] = static_cast<int>(comp[a]);
        t.n[a] = static_cast<int>(comp[3 + a]);
        t.w[a] = comp + 7 + ag::kLineTaps * a;
    }
    t.s = comp[6];
    return t;
}

std::size_t face_of(const af::GridShape& s, uint32_t c, int i, int j, int k) {
    const auto ui = static_cast<uint32_t>(i);
    const auto uj = static_cast<uint32_t>(j);
    const auto uk = static_cast<uint32_t>(k);
    return c == 0 ? af::u_index(s, ui, uj, uk) : c == 1 ? af::v_index(s, ui, uj, uk) : af::w_index(s, ui, uj, uk);
}

// airflow_line_sample: one (element, component) a thread.
float kernel_sample(const af::MacGrid& g, const float* rec, uint32_t c) {
    const Taps t = taps_of(rec, c);
    if (!(t.s > 0.0f)) return 0.0f;
    const std::vector<float>& data = c == 0 ? g.u : c == 1 ? g.v : g.w;
    float acc = 0.0f;
    for (int k = 0; k < t.n[2]; ++k) {
        for (int j = 0; j < t.n[1]; ++j) {
            const float wjk = t.w[1][j] * t.w[2][k];
            for (int i = 0; i < t.n[0]; ++i) {
                acc += t.w[0][i] * wjk * data[face_of(g.shape, c, t.lo[0] + i, t.lo[1] + j, t.lo[2] + k)];
            }
        }
    }
    return acc / t.s;
}

// The polar's lookup from airflow_line_polar's floats (blade.cpp's polar_at()).
void kernel_polar(const std::vector<float>& polar, float cos_alpha, float sin_alpha, float& cl, float& cd) {
    const uint32_t n = static_cast<uint32_t>(polar[0]);
    const float* key = polar.data() + 1;
    const float* cls = key + n;
    const float* cds = cls + n;
    const float k = af::diamond_angle(cos_alpha, sin_alpha);
    if (!std::isfinite(k)) {
        cl = k;
        cd = k;
        return;
    }
    if (k <= key[0]) {
        cl = cls[0];
        cd = cds[0];
        return;
    }
    if (k >= key[n - 1]) {
        cl = cls[n - 1];
        cd = cds[n - 1];
        return;
    }
    uint32_t hi = 1;
    while (key[hi] <= k) ++hi;  // the first key above k
    const uint32_t lo = hi - 1;
    const float t = (k - key[lo]) / (key[hi] - key[lo]);
    cl = cls[lo] + (cls[hi] - cls[lo]) * t;
    cd = cds[lo] + (cds[hi] - cds[lo]) * t;
}

// airflow_line_load: one element a thread; writes kLineForceStride floats.
void kernel_load(const float* rec, const float* vel, const std::vector<float>& polar, float* force) {
    const glm::vec3 axis(rec[0], rec[1], rec[2]);
    const glm::vec3 motion(rec[3], rec[4], rec[5]);
    const glm::vec3 air(vel[0], vel[1], vel[2]);
    const float chord = rec[6];
    const float sin_beta = rec[7];
    const float cos_beta = rec[8];
    const float r = rec[9];
    const float drs = rec[10];
    const float omega = rec[11];
    const float rho = rec[13];
    const float ua = -glm::dot(air, axis);
    const float wt = omega * r - glm::dot(air, motion);
    float axial = 0.0f;
    float tangent = 0.0f;
    const float w2 = ua * ua + wt * wt;
    if (w2 != 0.0f) {
        const float w = std::sqrt(w2);
        const float cphi = wt / w;
        const float sphi = ua / w;
        const float sin_alpha = sin_beta * cphi - cos_beta * sphi;
        const float cos_alpha = cos_beta * cphi + sin_beta * sphi;
        float cl = 0.0f;
        float cd = 0.0f;
        kernel_polar(polar, cos_alpha, sin_alpha, cl, cd);
        const float q = 0.5f * rho * w2 * chord;
        const float lift = q * cl;
        const float drag = q * cd;
        axial = lift * cphi - drag * sphi;
        tangent = lift * sphi + drag * cphi;
    }
    const glm::vec3 force_v = (-axial * axis + tangent * motion) * drs;
    force[0] = force_v.x;
    force[1] = force_v.y;
    force[2] = force_v.z;
    force[3] = axial;
    force[4] = tangent;
    force[5] = force[6] = force[7] = 0.0f;
}

// airflow_line_spread: one face of component c a thread, a gather over the elements in order.
void kernel_spread(const af::GridShape& s, const std::vector<float>& elems, uint32_t count,
                   const std::vector<float>& force, uint32_t c, std::vector<float>& out) {
    const uint32_t cx = s.nx + (c == 0 ? 1u : 0u);
    const uint32_t cy = s.ny + (c == 1 ? 1u : 0u);
    for (std::size_t f = 0; f < out.size(); ++f) {
        const int i = static_cast<int>(f % cx);
        const int j = static_cast<int>((f / cx) % cy);
        const int k = static_cast<int>(f / (std::size_t{cx} * cy));
        float acc = out[f];
        for (uint32_t e = 0; e < count; ++e) {
            const float* rec = elems.data() + std::size_t{e} * ag::kLineStride;
            const Taps t = taps_of(rec, c);
            const int di = i - t.lo[0];
            const int dj = j - t.lo[1];
            const int dk = k - t.lo[2];
            if (di < 0 || di >= t.n[0] || dj < 0 || dj >= t.n[1] || dk < 0 || dk >= t.n[2]) continue;
            const float amount = force[std::size_t{e} * ag::kLineForceStride + c] * rec[12];
            if (!(t.s > 0.0f) || amount == 0.0f) continue;
            const float coef = amount / t.s;
            const float wjk = t.w[1][dj] * t.w[2][dk];
            acc += coef * (t.w[0][di] * wjk);
        }
        out[f] = acc;
    }
}

bool same_bits(const std::vector<float>& a, const std::vector<float>& b, std::size_t* first = nullptr) {
    if (a.size() != b.size()) return false;
    for (std::size_t i = 0; i < a.size(); ++i) {
        if (std::bit_cast<uint32_t>(a[i]) != std::bit_cast<uint32_t>(b[i])) {
            if (first != nullptr) *first = i;
            return false;
        }
    }
    return true;
}

}  // namespace

TEST(GpuAirflowLine, PackedRecordsRunThroughTheKernelsAsSpreadLineBitForBit) {
    const af::BladeShape blade_a = blade_of(2);
    const af::BladeShape blade_b = blade_of(3);
    const af::PolarTable polar = af::estimated_polar();
    const af::MacGrid grid = noisy_grid();
    const uint32_t n_sub = 4;
    for (const float eps : {2.0f, 1.5f, 3.0f}) {
        af::LineRotor a;
        a.centre = glm::vec3(0.104f, 0.104f, 0.104f);
        a.blade = &blade_a;
        a.polar = &polar;
        a.omega = 400.0f;
        a.psi = 0.3f;
        a.eps_cells = eps;
        af::LineRotor b;  // left hand, the axis tilted in the x-y plane
        b.centre = glm::vec3(0.105f, 0.103f, 0.106f);
        b.axis = glm::vec3(0.6f, 0.8f, 0.0f);
        b.ref = glm::vec3(0.8f, -0.6f, 0.0f);
        b.spin = -1.0f;
        b.blade = &blade_b;
        b.polar = &polar;
        b.omega = 250.0f;
        b.psi = 1.1f;
        b.eps_cells = eps;
        const std::vector<af::LineRotor> rotors{a, b};

        // The CPU: the rotors' own calls, one after the other onto the same increments.
        std::vector<float> du(grid.u.size(), 0.0f);
        std::vector<float> dv(grid.v.size(), 0.0f);
        std::vector<float> dw(grid.w.size(), 0.0f);
        std::vector<af::LineReport> want;
        for (const af::LineRotor& r : rotors) {
            const auto rep = af::spread_line(r, grid, kRho, kH, n_sub, du, dv, dw);
            ASSERT_TRUE(rep.has_value()) << rep.error().context;
            want.push_back(*rep);
        }

        // The device twin's host half and the kernels on the packed floats.
        const auto records = ag::line_records(rotors, grid.shape, kRho, kH, n_sub);
        ASSERT_TRUE(records.has_value()) << records.error().context;
        ASSERT_EQ(records->per_rotor.size(), 2u);
        const uint32_t count = records->per_rotor[0] + records->per_rotor[1];
        ASSERT_GT(count, 0u);
        ASSERT_EQ(records->elems.size(), std::size_t{count} * ag::kLineStride) << "eps " << eps;
        const std::vector<float> polar_floats = ag::polar_records(polar);
        ASSERT_EQ(polar_floats.size(), 1u + 3u * polar.key.size());

        const spade::math::ScopedFlushSubnormals flush;  // the CPU's own scope, as the device runs under it
        std::vector<float> vel(std::size_t{count} * 3u);
        for (uint32_t e = 0; e < count; ++e) {
            const float* rec = records->elems.data() + std::size_t{e} * ag::kLineStride;
            for (uint32_t c = 0; c < 3; ++c) vel[std::size_t{e} * 3u + c] = kernel_sample(grid, rec, c);
        }
        std::vector<float> force(std::size_t{count} * ag::kLineForceStride);
        for (uint32_t e = 0; e < count; ++e) {
            kernel_load(records->elems.data() + std::size_t{e} * ag::kLineStride, vel.data() + std::size_t{e} * 3u,
                        polar_floats, force.data() + std::size_t{e} * ag::kLineForceStride);
        }
        std::vector<float> gu(grid.u.size(), 0.0f);
        std::vector<float> gv(grid.v.size(), 0.0f);
        std::vector<float> gw(grid.w.size(), 0.0f);
        kernel_spread(grid.shape, records->elems, count, force, 0, gu);
        kernel_spread(grid.shape, records->elems, count, force, 1, gv);
        kernel_spread(grid.shape, records->elems, count, force, 2, gw);
        std::size_t at = 0;
        EXPECT_TRUE(same_bits(gu, du, &at)) << "du, eps " << eps << ", first difference at " << at;
        EXPECT_TRUE(same_bits(gv, dv, &at)) << "dv, eps " << eps << ", first difference at " << at;
        EXPECT_TRUE(same_bits(gw, dw, &at)) << "dw, eps " << eps << ", first difference at " << at;

        const std::vector<af::LineReport> got = ag::line_reports(*records, rotors, force);
        ASSERT_EQ(got.size(), want.size());
        for (std::size_t r = 0; r < want.size(); ++r) {
            EXPECT_EQ(std::bit_cast<uint32_t>(got[r].thrust), std::bit_cast<uint32_t>(want[r].thrust)) << "rotor " << r;
            EXPECT_EQ(std::bit_cast<uint32_t>(got[r].torque), std::bit_cast<uint32_t>(want[r].torque)) << "rotor " << r;
            EXPECT_EQ(std::bit_cast<uint32_t>(got[r].power), std::bit_cast<uint32_t>(want[r].power)) << "rotor " << r;
        }
        // Not vacuous: the load moved the air.
        float peak = 0.0f;
        for (const float x : du) peak = std::max(peak, std::fabs(x));
        EXPECT_GT(peak, 0.0f);
    }
}

namespace {

namespace gen = spade::compute::gen;
using spade::airflow_gpu::BufferKind;
using spade::airflow_gpu::ChainRunner;
using spade::airflow_gpu::RunnerBuffer;
using spade::compute::ChainCommand;
using spade::compute::SpirvVariantSet;
using spade::compute::VulkanContext;

struct LinePush {
    uint32_t a = 0;      // the component (spread); the element count (sample, load)
    uint32_t b = 0;      // the element count (spread)
    uint32_t count = 0;  // the invocations: word 2 as in every airflow kernel
    uint32_t d = 0;
};

[[nodiscard]] std::unique_ptr<VulkanContext> make_context() {
    auto result = VulkanContext::create(spade::compute::BackendDesc{.kind = spade::compute::BackendKind::vulkan});
    if (!result.has_value()) return nullptr;
    return std::move(*result);
}

template <typename T>
[[nodiscard]] std::span<const std::byte> bytes_of(const std::vector<T>& v) {
    return std::as_bytes(std::span(v));
}

[[nodiscard]] ChainCommand dispatch_of(uint32_t pipeline, const LinePush& push, uint32_t workgroup_size) {
    ChainCommand cmd{.is_dispatch = true, .pipeline = pipeline};
    std::memcpy(cmd.push.data(), &push, sizeof push);
    cmd.groups = {(push.count + workgroup_size - 1u) / workgroup_size, 1u, 1u};
    return cmd;
}

}  // namespace

TEST(GpuAirflowLine, TheKernelsOnTheDeviceMatchSpreadLine) {
    if (const auto why = spade::testing::vulkan_skip_reason()) GTEST_SKIP() << *why;
    auto ctx = make_context();
    ASSERT_NE(ctx, nullptr);
    const bool bit_equal = spade::testing::airflow_bit_equal_device(*ctx);
    const af::BladeShape blade_a = blade_of(2);
    const af::BladeShape blade_b = blade_of(3);
    const af::PolarTable polar = af::estimated_polar();
    const af::MacGrid grid = noisy_grid();
    const af::GridShape& s = grid.shape;
    const std::array<uint32_t, 3> faces{static_cast<uint32_t>(grid.u.size()), static_cast<uint32_t>(grid.v.size()),
                                        static_cast<uint32_t>(grid.w.size())};
    const uint32_t total = faces[0] + faces[1] + faces[2];
    constexpr uint32_t kWorkgroup = 64;
    for (const uint32_t n_sub : {1u, 4u}) {
        for (const float eps : {2.0f, 1.5f}) {
            af::LineRotor a;
            a.centre = glm::vec3(0.104f, 0.104f, 0.104f);
            a.blade = &blade_a;
            a.polar = &polar;
            a.omega = 400.0f;
            a.psi = 0.3f;
            a.eps_cells = eps;
            af::LineRotor b;
            b.centre = glm::vec3(0.105f, 0.103f, 0.106f);
            b.axis = glm::vec3(0.6f, 0.8f, 0.0f);
            b.ref = glm::vec3(0.8f, -0.6f, 0.0f);
            b.spin = -1.0f;
            b.blade = &blade_b;
            b.polar = &polar;
            b.omega = 250.0f;
            b.psi = 1.1f;
            b.eps_cells = eps;
            const std::vector<af::LineRotor> rotors{a, b};

            // The increments start as something (the discs', say): the line adds to it.
            std::vector<float> start(total);
            uint32_t state = 0x5EEDu + n_sub;
            for (float& x : start) {
                state = state * 1664525u + 1013904223u;
                x = (static_cast<float>(state >> 8) / 16777216.0f - 0.5f) * 2.0e-3f;
            }
            std::vector<float> du(start.begin(), start.begin() + faces[0]);
            std::vector<float> dv(start.begin() + faces[0], start.begin() + faces[0] + faces[1]);
            std::vector<float> dw(start.begin() + faces[0] + faces[1], start.end());
            std::vector<af::LineReport> want;
            for (const af::LineRotor& r : rotors) {
                const auto rep = af::spread_line(r, grid, kRho, kH, n_sub, du, dv, dw);
                ASSERT_TRUE(rep.has_value()) << rep.error().context;
                want.push_back(*rep);
            }

            const auto records = ag::line_records(rotors, s, kRho, kH, n_sub);
            ASSERT_TRUE(records.has_value()) << records.error().context;
            const uint32_t count = records->per_rotor[0] + records->per_rotor[1];
            const std::vector<float> polar_floats = ag::polar_records(polar);
            std::vector<uint32_t> params(16u, 0u);
            params[0] = s.nx;
            params[1] = s.ny;
            params[2] = s.nz;
            params[4] = std::bit_cast<uint32_t>(s.dx);
            params[5] = std::bit_cast<uint32_t>(1.0f / s.dx);

            std::vector<RunnerBuffer> buffers(gen::kBinding_airflow_line_force + 1u);
            for (uint32_t i = 0; i < buffers.size(); ++i) {
                buffers[i] = {"airflow_unused_" + std::to_string(i), 4u, BufferKind::device_only};
            }
            buffers[gen::kBinding_airflow_params] = {"airflow_params", params.size() * 4u, BufferKind::staged};
            buffers[gen::kBinding_airflow_u] = {"airflow_u", grid.u.size() * 4u, BufferKind::staged};
            buffers[gen::kBinding_airflow_v] = {"airflow_v", grid.v.size() * 4u, BufferKind::staged};
            buffers[gen::kBinding_airflow_w] = {"airflow_w", grid.w.size() * 4u, BufferKind::staged};
            buffers[gen::kBinding_airflow_increments] = {"airflow_increments", start.size() * 4u, BufferKind::staged};
            buffers[gen::kBinding_airflow_line_elems] = {"airflow_line_elems", records->elems.size() * 4u,
                                                         BufferKind::staged};
            buffers[gen::kBinding_airflow_line_polar] = {"airflow_line_polar", polar_floats.size() * 4u,
                                                         BufferKind::staged};
            buffers[gen::kBinding_airflow_line_vel] = {"airflow_line_vel", std::size_t{count} * 4u * 4u,
                                                       BufferKind::device_only};
            const uint64_t force_bytes = std::size_t{count} * ag::kLineForceStride * 4u;
            buffers[gen::kBinding_airflow_line_force] = {"airflow_line_force", force_bytes, BufferKind::staged};
            const SpirvVariantSet* const kernels[] = {&gen::kSpvVariants_airflow_line_sample,
                                                      &gen::kSpvVariants_airflow_line_load,
                                                      &gen::kSpvVariants_airflow_line_spread};
            auto runner = ChainRunner::create(*ctx, spade::airflow_gpu::RunnerDesc{.buffers = buffers,
                                                                                   .kernels = kernels,
                                                                                   .push_bytes = sizeof(LinePush),
                                                                                   .workgroup_size = kWorkgroup});
            ASSERT_TRUE(runner.has_value()) << runner.error().context;
            ChainRunner& r = **runner;
            ASSERT_TRUE(r.upload(gen::kBinding_airflow_params, bytes_of(params)).has_value());
            ASSERT_TRUE(r.upload(gen::kBinding_airflow_u, bytes_of(grid.u)).has_value());
            ASSERT_TRUE(r.upload(gen::kBinding_airflow_v, bytes_of(grid.v)).has_value());
            ASSERT_TRUE(r.upload(gen::kBinding_airflow_w, bytes_of(grid.w)).has_value());
            ASSERT_TRUE(r.upload(gen::kBinding_airflow_increments, bytes_of(start)).has_value());
            ASSERT_TRUE(r.upload(gen::kBinding_airflow_line_elems, bytes_of(records->elems)).has_value());
            ASSERT_TRUE(r.upload(gen::kBinding_airflow_line_polar, bytes_of(polar_floats)).has_value());
            std::vector<ChainCommand> chain;
            chain.push_back(dispatch_of(0, LinePush{count, 0u, 3u * count, 0u}, kWorkgroup));
            chain.push_back(dispatch_of(1, LinePush{count, 0u, count, 0u}, kWorkgroup));
            for (uint32_t c = 0; c < 3; ++c) {
                chain.push_back(dispatch_of(2, LinePush{c, count, faces[c], 0u}, kWorkgroup));
            }
            const auto id = r.record(chain);
            ASSERT_TRUE(id.has_value()) << id.error().context;
            ASSERT_TRUE(r.submit(*id, 1).has_value());
            std::vector<float> out(total, 0.0f);
            ASSERT_TRUE(r.read(gen::kBinding_airflow_increments, std::as_writable_bytes(std::span(out))).has_value());
            std::vector<float> force(std::size_t{count} * ag::kLineForceStride, 0.0f);
            ASSERT_TRUE(r.read(gen::kBinding_airflow_line_force, std::as_writable_bytes(std::span(force))).has_value());

            std::vector<float> cpu(total);
            std::copy(du.begin(), du.end(), cpu.begin());
            std::copy(dv.begin(), dv.end(), cpu.begin() + faces[0]);
            std::copy(dw.begin(), dw.end(), cpu.begin() + faces[0] + faces[1]);
            std::size_t differing = 0;
            float worst = 0.0f;
            float scale = 0.0f;
            for (std::size_t i = 0; i < total; ++i) {
                scale = std::max(scale, std::fabs(cpu[i] - start[i]));
                if (std::bit_cast<uint32_t>(out[i]) != std::bit_cast<uint32_t>(cpu[i])) {
                    ++differing;
                    worst = std::max(worst, std::fabs(out[i] - cpu[i]));
                }
            }
            std::printf("measure: line on the device, n_sub %u eps %.1f cells, %u elements: %zu of %u faces differ, "
                        "worst %.3g (relative to the largest increment %.3g: %.3g)\n",
                        n_sub, static_cast<double>(eps), count, differing, total, static_cast<double>(worst),
                        static_cast<double>(scale), static_cast<double>(scale > 0.0f ? worst / scale : 0.0f));
            EXPECT_GT(scale, 0.0f) << "not vacuous";
            if (bit_equal) {
                EXPECT_EQ(differing, 0u) << "n_sub " << n_sub << " eps " << eps;
            } else {
                EXPECT_LE(worst, 4.0f * 2.08e-7f * scale) << "n_sub " << n_sub << " eps " << eps;
            }
            const std::vector<af::LineReport> got = ag::line_reports(*records, rotors, force);
            ASSERT_EQ(got.size(), want.size());
            for (std::size_t k = 0; k < want.size(); ++k) {
                if (bit_equal) {
                    EXPECT_EQ(std::bit_cast<uint32_t>(got[k].thrust), std::bit_cast<uint32_t>(want[k].thrust))
                        << "rotor " << k;
                    EXPECT_EQ(std::bit_cast<uint32_t>(got[k].torque), std::bit_cast<uint32_t>(want[k].torque))
                        << "rotor " << k;
                } else {
                    EXPECT_NEAR(got[k].thrust, want[k].thrust, 1.0e-4f * std::fabs(want[k].thrust) + 1e-9f)
                        << "rotor " << k;
                }
            }
        }
    }
}

// The whole path: GpuFluid::step(discs, lines) against the CPU's spread_line(), spread_actuator() and fluid_step(),
// three steps with the rotor turning (psi advances by omega h a step) and a disc beside it. The line is added to the
// increments first, the disc after it, on both sides.
TEST(GpuAirflowLine, TheFluidStepsALineAndMatchesTheCpu) {
    if (const auto why = spade::testing::vulkan_skip_reason()) GTEST_SKIP() << *why;
    auto ctx = make_context();
    ASSERT_NE(ctx, nullptr);
    const bool bit_equal = spade::testing::airflow_bit_equal_device(*ctx);
    const af::BladeShape blade = blade_of(2);
    const af::PolarTable polar = af::estimated_polar();
    const af::GridShape shape{kN, kN, kN, kDx};
    const af::DomainBc bc{};
    const af::FluidConfig cfg{.rho = kRho, .nu = 1.5e-5f, .h = kH, .n_v = 2, .visc_sweeps = 8};
    const uint32_t n_sub = 2;
    af::LineRotor rotor;
    rotor.centre = glm::vec3(0.104f, 0.104f, 0.104f);
    rotor.blade = &blade;
    rotor.polar = &polar;
    rotor.omega = 400.0f;
    rotor.psi = 0.3f;
    const std::vector<af::ActuatorDisc> discs{af::ActuatorDisc{
        .centre = {0.07f, 0.104f, 0.104f},
        .axis = {0.0f, -1.0f, 0.0f},
        .radius = 0.03f,
        .thrust = 0.2f,
        .torque = 0.001f}};

    af::MacGrid start = *af::make_grid(shape, bc);
    uint32_t state = 0xC0FFEEu;
    for (std::vector<float>* comp : {&start.u, &start.v, &start.w}) {
        for (float& x : *comp) {
            state = state * 1664525u + 1013904223u;
            x = (static_cast<float>(state >> 8) / 16777216.0f - 0.5f) * 1.0f;
        }
    }
    auto scratch = af::make_fluid_scratch(start, bc, cfg);
    ASSERT_TRUE(scratch.has_value()) << scratch.error().context;
    af::MacGrid cpu = start;

    const std::vector<af::LineRotor> first{rotor};
    const auto probe = ag::line_records(first, shape, kRho, kH, n_sub);
    ASSERT_TRUE(probe.has_value()) << probe.error().context;
    const ag::LineSetup setup{.polar = &polar, .elements = probe->per_rotor[0], .n_sub = n_sub};
    auto gpu = ag::GpuFluid::create(*ctx, start, bc, cfg, nullptr, 64u, &setup);
    ASSERT_TRUE(gpu.has_value()) << gpu.error().context;
    EXPECT_TRUE((*gpu)->line_reports().empty());

    for (int step = 0; step < 3; ++step) {
        const std::vector<af::LineRotor> rotors{rotor};
        const auto rep = af::spread_line(rotor, cpu, kRho, kH, n_sub, scratch->du, scratch->dv, scratch->dw);
        ASSERT_TRUE(rep.has_value()) << rep.error().context;
        for (const af::ActuatorDisc& d : discs) {
            af::spread_actuator(d, shape, kRho, kH, scratch->du, scratch->dv, scratch->dw);
        }
        const af::FluidHealth want_health = af::fluid_step(cpu, bc, cfg, *scratch);
        ASSERT_EQ(want_health.fault, af::FluidFault::none);
        const auto got_health = (*gpu)->step(discs, rotors);
        ASSERT_TRUE(got_health.has_value()) << got_health.error().context;
        af::MacGrid read = start;
        ASSERT_TRUE((*gpu)->read(read).has_value());
        std::size_t differing = 0;
        float scale = 0.0f;
        for (const auto& [a, b] : {std::pair{&read.u, &cpu.u}, std::pair{&read.v, &cpu.v}, std::pair{&read.w, &cpu.w},
                                   std::pair{&read.p, &cpu.p}}) {
            for (std::size_t i = 0; i < a->size(); ++i) {
                scale = std::max(scale, std::fabs((*b)[i]));
                if (std::bit_cast<uint32_t>((*a)[i]) != std::bit_cast<uint32_t>((*b)[i])) ++differing;
            }
        }
        std::printf("measure: GpuFluid with a line, step %d: %zu faces and cells differ from the CPU\n",
                    step + 1, differing);
        if (bit_equal) {
            EXPECT_EQ(differing, 0u) << "step " << step + 1;
        }
        ASSERT_EQ((*gpu)->line_reports().size(), 1u);
        if (bit_equal) {
            EXPECT_EQ(std::bit_cast<uint32_t>((*gpu)->line_reports()[0].thrust), std::bit_cast<uint32_t>(rep->thrust))
                << "step " << step + 1;
        }
        EXPECT_NE(rep->thrust, 0.0f) << "not vacuous";
        rotor.psi += rotor.omega * kH;
        rotor.psi = std::fmod(rotor.psi, 6.2831853f);
    }

    // Refusals: no setup, the wrong element count, a polar that is not the setup's.
    auto plain = ag::GpuFluid::create(*ctx, start, bc, cfg);
    ASSERT_TRUE(plain.has_value()) << plain.error().context;
    const std::vector<af::LineRotor> one{rotor};
    EXPECT_FALSE((*plain)->step(discs, one).has_value());
    const af::BladeShape blade3 = blade_of(3);
    af::LineRotor three = rotor;
    three.blade = &blade3;
    const std::vector<af::LineRotor> wrong{three};
    EXPECT_FALSE((*gpu)->step(discs, wrong).has_value()) << "3 blades is not the setup's element count";
    af::PolarTable other = polar;
    other.cl[0] += 0.5f;
    af::LineRotor changed = rotor;
    changed.polar = &other;
    const std::vector<af::LineRotor> changed_polar{changed};
    EXPECT_FALSE((*gpu)->step(discs, changed_polar).has_value()) << "a polar that is not the setup's";
}
