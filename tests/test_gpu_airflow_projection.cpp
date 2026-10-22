// ---------------------------------------------------------------------------
// test_gpu_airflow_projection.cpp -- plan B, task B2: the projection without
// solids on the GPU (engine/shaders/kernels/airflow_divergence,
// airflow_mg_smooth, airflow_mg_residual, airflow_mg_restrict,
// airflow_mg_prolong and airflow_gradient_subtract, over
// engine/shaders/mg_levels.slang) against its CPU twins (pressure.cpp,
// multigrid.cpp and projection.cpp in engine/physics/airflow), on the chain
// runner (spike/airflow/chain_runner.hpp), with the chain and the level table
// spike/airflow/projection_chain.hpp builds.
//
//   GpuAirflowProjection  the divergence, the residual, the restriction, the
//                         prolongation and the gradient subtracted, bit for
//                         bit; one red-black sweep on every level, banded (its
//                         division); one V(2,2) projection at 32^3 against
//                         project(), banded, and on a closed domain, whose
//                         mean airflow_tile_sum and airflow_remove_mean take
//                         out (remove_mean()); the same bits at workgroup sizes
//                         32, 64 and 128; the projection's time at 64^3 and
//                         128^3 (M1).
//
// THE HIERARCHIES (multigrid.hpp, Physics' mg-odd-axes rule):
//   * 16 x 16 x 16, every face open: halved throughout, no weights;
//   * 17 x 16 x 15, walls on x: odd axes, so its coarse level is past the
//     halving rule and carries plan A's weights (a closed partner);
//   * 30 x 12 x 16, periodic in x and z: x is an even ring that is not a
//     multiple of 4, so it is held, and its coarse levels carry the held
//     axis's 4x weights.
// Every face kind meets every kernel: an outlet's ghost, a wall's nothing and
// a periodic wrap.
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
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <vector>

#include "airflow_divergence.spv.gen.hpp"
#include "airflow_gradient_subtract.spv.gen.hpp"
#include "airflow_mg_prolong.spv.gen.hpp"
#include "airflow_mg_residual.spv.gen.hpp"
#include "airflow_mg_restrict.spv.gen.hpp"
#include "airflow_mg_smooth.spv.gen.hpp"
#include "airflow_remove_mean.spv.gen.hpp"
#include "airflow_tile_sum.spv.gen.hpp"
#include "compute/backend.hpp"
#include "compute/spirv_variants.hpp"
#include "compute/vulkan/context.hpp"
#include "compute/vulkan/dispatch_chain.hpp"
#include "gpu_skip.hpp"
#include "physics/airflow/boundaries.hpp"
#include "physics/airflow/mac_grid.hpp"
#include "physics/airflow/multigrid.hpp"
#include "physics/airflow/pressure.hpp"
#include "physics/airflow/projection.hpp"
#include "spike/airflow/chain_runner.hpp"
#include "spike/airflow/projection_chain.hpp"

namespace {

namespace af = spade::physics::airflow;
namespace ag = spade::airflow_gpu;
namespace gen = spade::compute::gen;
using spade::airflow_gpu::BufferKind;
using spade::airflow_gpu::ChainRunner;
using spade::airflow_gpu::RunnerBuffer;
using spade::airflow_gpu::RunnerDesc;
using spade::compute::ChainCommand;
using spade::compute::SpirvVariantSet;
using spade::compute::VulkanContext;

constexpr uint32_t kWorkgroupSize = 64;
constexpr float kRho = 1.225f;

// Set 1, bindings 0..23 (bindings.slang section F); 5 to 8, 13 to 20 and 22 are
// bound, not read.
enum Buffer : uint32_t {
    kParams = 0,
    kU = 1,
    kV = 2,
    kW = 3,
    kPressure = 4,
    kRhs = 9,
    kResidual = 10,
    kMultigrid = 11,
    kMgWeights = 12,
    kScalars = 21,
    kPartials = 23,
    kBufferCount = 24,
};

// airflow_scalars' slots: remove_mean()'s sum and B5's.
constexpr uint32_t kScalarSlots = 8;

// RunnerDesc::kernels' order.
constexpr ag::ProjectionPipelines kPipelines{
    .divergence = 0, .smooth = 1, .residual = 2, .restrict_residual = 3, .prolong = 4, .gradient = 5,
    .tile_sum = 6, .remove_mean = 7};

[[nodiscard]] std::unique_ptr<VulkanContext> make_context() {
    auto result = VulkanContext::create(spade::compute::BackendDesc{.kind = spade::compute::BackendKind::vulkan});
    if (!result.has_value()) return nullptr;
    return std::move(*result);
}

// splitmix64: a fixed-seed stream, the same on every compiler.
struct SplitMix {
    uint64_t state;
    uint64_t next() {
        uint64_t z = (state += 0x9E3779B97F4A7C15ull);
        z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ull;
        z = (z ^ (z >> 27)) * 0x94D049BB133111EBull;
        return z ^ (z >> 31);
    }
    // In [-1, 1), from the top 24 bits: exact in a float.
    float unit() { return static_cast<float>(static_cast<int64_t>(next() >> 40) - (int64_t{1} << 23)) * 0x1p-23f; }
};

template <class T>
[[nodiscard]] std::span<const std::byte> bytes_of(const std::vector<T>& v) noexcept {
    return std::as_bytes(std::span<const T>(v));
}

template <class T>
[[nodiscard]] std::span<std::byte> writable_bytes_of(std::vector<T>& v) noexcept {
    return std::as_writable_bytes(std::span<T>(v));
}

// One test domain: a shape and its faces.
struct Domain {
    const char* name;
    af::GridShape shape;
    af::DomainBc bc;
};

[[nodiscard]] std::vector<Domain> domains() {
    Domain open{"16x16x16 open", {16, 16, 16, 1.0f / 16.0f}, {}};
    Domain walls{"17x16x15 walls on x", {17, 16, 15, 1.0f / 16.0f}, {}};
    walls.bc.face[af::kXMinus] = walls.bc.face[af::kXPlus] = af::FaceBc::wall;
    Domain ring{"30x12x16 periodic x, z", {30, 12, 16, 1.0f / 16.0f}, {}};
    ring.bc.face[af::kXMinus] = ring.bc.face[af::kXPlus] = af::FaceBc::periodic;
    ring.bc.face[af::kZMinus] = ring.bc.face[af::kZPlus] = af::FaceBc::periodic;
    return {open, walls, ring};
}

[[nodiscard]] af::Multigrid hierarchy(const Domain& d) {
    af::Multigrid mg = af::make_multigrid(d.shape.nx, d.shape.ny, d.shape.nz, af::periodic_axes(d.bc));
    af::set_kinds(mg, af::face_kinds(d.bc));
    return mg;
}

// Fixed-seed noise in every level's p, b and r.
void fill_levels(af::Multigrid& mg, uint64_t seed) {
    SplitMix rng{seed};
    for (af::PoissonLevel& L : mg.levels) {
        for (float& x : L.p) x = rng.unit();
        for (float& x : L.b) x = rng.unit();
        for (float& x : L.r) x = rng.unit();
    }
}

// A velocity field with every face set: a swirl plus fixed-seed noise, so the
// divergence is nowhere zero.
void fill_velocity(af::MacGrid& g, uint64_t seed) {
    SplitMix rng{seed};
    for (float& x : g.u) x = rng.unit();
    for (float& x : g.v) x = 0.5f * rng.unit();
    for (float& x : g.w) x = rng.unit();
    for (float& x : g.p) x = 0.25f * rng.unit();
}

// The levels as the device holds them: level 0's p, b and r in their own
// buffers, levels 1..L packed in airflow_multigrid.
struct LevelBuffers {
    std::vector<float> p0, b0, r0, mg;
};

[[nodiscard]] LevelBuffers pack(const af::Multigrid& mg, const ag::MgLayout& layout) {
    LevelBuffers out{mg.levels[0].p, mg.levels[0].b, mg.levels[0].r, {}};
    out.mg.assign(static_cast<std::size_t>(std::max<uint64_t>(layout.multigrid_floats, 1u)), 0.0f);
    for (std::size_t l = 1; l < mg.levels.size(); ++l) {
        const af::PoissonLevel& L = mg.levels[l];
        std::copy(L.p.begin(), L.p.end(), out.mg.begin() + layout.levels[l][3]);
        std::copy(L.b.begin(), L.b.end(), out.mg.begin() + layout.levels[l][4]);
        std::copy(L.r.begin(), L.r.end(), out.mg.begin() + layout.levels[l][5]);
    }
    return out;
}

void unpack(const LevelBuffers& in, const ag::MgLayout& layout, af::Multigrid& mg) {
    mg.levels[0].p = in.p0;
    mg.levels[0].b = in.b0;
    mg.levels[0].r = in.r0;
    for (std::size_t l = 1; l < mg.levels.size(); ++l) {
        af::PoissonLevel& L = mg.levels[l];
        const auto take = [&](std::vector<float>& x, uint32_t at) {
            std::copy(in.mg.begin() + at, in.mg.begin() + at + static_cast<std::ptrdiff_t>(x.size()), x.begin());
        };
        take(L.p, layout.levels[l][3]);
        take(L.b, layout.levels[l][4]);
        take(L.r, layout.levels[l][5]);
    }
}

[[nodiscard]] bool check(const spade::Result<void>& r) {
    if (r.has_value()) return true;
    ADD_FAILURE() << r.error().context;
    return false;
}

// The projection's kernels on one domain, with its grid and its levels.
class Device {
public:
    [[nodiscard]] static std::optional<Device> create(VulkanContext& ctx, const Domain& d,
                                                      uint32_t workgroup_size = kWorkgroupSize) {
        Device dev;
        dev.domain_ = d;
        dev.workgroup_size_ = workgroup_size;
        const af::Multigrid mg = hierarchy(d);
        auto layout = ag::mg_layout(mg);
        if (!layout) {
            ADD_FAILURE() << layout.error().context;
            return std::nullopt;
        }
        dev.layout_ = std::move(*layout);
        const af::GridShape& s = d.shape;
        const uint64_t cells = uint64_t{s.nx} * s.ny * s.nz;
        dev.params_.assign(ag::projection_params_words(dev.layout_), 0u);
        dev.params_[0] = s.nx;
        dev.params_[1] = s.ny;
        dev.params_[2] = s.nz;
        dev.params_[4] = std::bit_cast<uint32_t>(s.dx);
        ag::write_projection_params(dev.params_, dev.layout_, af::face_kinds(d.bc),
                                    af::projection_floats(s, kRho, kStepS));
        std::vector<RunnerBuffer> buffers(kBufferCount);
        for (uint32_t b = 0; b < kBufferCount; ++b) {
            buffers[b] = {"airflow_unused_" + std::to_string(b), 4u, BufferKind::device_only};
        }
        buffers[kParams] = {"airflow_params", dev.params_.size() * 4u, BufferKind::staged};
        buffers[kU] = {"airflow_u", uint64_t{s.nx + 1u} * s.ny * s.nz * 4u, BufferKind::staged};
        buffers[kV] = {"airflow_v", uint64_t{s.nx} * (s.ny + 1u) * s.nz * 4u, BufferKind::staged};
        buffers[kW] = {"airflow_w", uint64_t{s.nx} * s.ny * (s.nz + 1u) * 4u, BufferKind::staged};
        buffers[kPressure] = {"airflow_pressure", cells * 4u, BufferKind::staged};
        buffers[kRhs] = {"airflow_rhs", cells * 4u, BufferKind::staged};
        buffers[kResidual] = {"airflow_residual", cells * 4u, BufferKind::staged};
        buffers[kMultigrid] = {"airflow_multigrid", std::max<uint64_t>(dev.layout_.multigrid_floats, 1u) * 4u,
                               BufferKind::staged};
        buffers[kMgWeights] = {"airflow_mg_weights", std::max<std::size_t>(dev.layout_.weights.size(), 1u) * 4u,
                               BufferKind::staged};
        buffers[kScalars] = {"airflow_scalars", kScalarSlots * 4u, BufferKind::device_only};
        buffers[kPartials] = {"airflow_partials", uint64_t{ag::tile_sum_tiles(s)} * 4u, BufferKind::device_only};
        const SpirvVariantSet* const kernels[] = {
            &gen::kSpvVariants_airflow_divergence, &gen::kSpvVariants_airflow_mg_smooth,
            &gen::kSpvVariants_airflow_mg_residual, &gen::kSpvVariants_airflow_mg_restrict,
            &gen::kSpvVariants_airflow_mg_prolong, &gen::kSpvVariants_airflow_gradient_subtract,
            &gen::kSpvVariants_airflow_tile_sum,   &gen::kSpvVariants_airflow_remove_mean};
        auto runner = ChainRunner::create(ctx, RunnerDesc{.buffers = buffers,
                                                          .kernels = kernels,
                                                          .push_bytes = sizeof(ag::ProjectionPush),
                                                          .workgroup_size = workgroup_size});
        if (!runner) {
            ADD_FAILURE() << runner.error().context;
            return std::nullopt;
        }
        dev.runner_ = std::move(*runner);
        if (!check(dev.runner_->upload(kParams, bytes_of(dev.params_)))) return std::nullopt;
        if (!dev.layout_.weights.empty() && !check(dev.runner_->upload(kMgWeights, bytes_of(dev.layout_.weights)))) {
            return std::nullopt;
        }
        return dev;
    }

    static constexpr float kStepS = 1.0f / 512.0f;

    [[nodiscard]] const ag::MgLayout& layout() const noexcept { return layout_; }
    [[nodiscard]] uint32_t workgroup_size() const noexcept { return workgroup_size_; }

    [[nodiscard]] bool upload_levels(const af::Multigrid& mg) {
        const LevelBuffers b = pack(mg, layout_);
        return check(runner_->upload(kPressure, bytes_of(b.p0))) && check(runner_->upload(kRhs, bytes_of(b.b0))) &&
               check(runner_->upload(kResidual, bytes_of(b.r0))) && check(runner_->upload(kMultigrid, bytes_of(b.mg)));
    }

    [[nodiscard]] bool read_levels(af::Multigrid& mg) const {
        LevelBuffers b = pack(mg, layout_);
        if (!check(runner_->read(kPressure, writable_bytes_of(b.p0))) ||
            !check(runner_->read(kRhs, writable_bytes_of(b.b0))) ||
            !check(runner_->read(kResidual, writable_bytes_of(b.r0))) ||
            !check(runner_->read(kMultigrid, writable_bytes_of(b.mg)))) {
            return false;
        }
        unpack(b, layout_, mg);
        return true;
    }

    [[nodiscard]] bool upload_grid(const af::MacGrid& g) {
        return check(runner_->upload(kU, bytes_of(g.u))) && check(runner_->upload(kV, bytes_of(g.v))) &&
               check(runner_->upload(kW, bytes_of(g.w))) && check(runner_->upload(kPressure, bytes_of(g.p)));
    }

    [[nodiscard]] bool read_grid(af::MacGrid& g) const {
        return check(runner_->read(kU, writable_bytes_of(g.u))) && check(runner_->read(kV, writable_bytes_of(g.v))) &&
               check(runner_->read(kW, writable_bytes_of(g.w))) &&
               check(runner_->read(kPressure, writable_bytes_of(g.p)));
    }

    [[nodiscard]] bool read_rhs(std::vector<float>& out) const { return check(runner_->read(kRhs, writable_bytes_of(out))); }

    // Records and submits `chain` `n` times; its marks' durations after.
    [[nodiscard]] std::optional<std::vector<uint64_t>> run(const std::vector<ChainCommand>& chain, uint64_t n = 1) {
        const auto id = runner_->record(chain);
        if (!id) {
            ADD_FAILURE() << id.error().context;
            return std::nullopt;
        }
        if (!check(runner_->submit(*id, n))) return std::nullopt;
        auto d = runner_->durations_ns(*id);
        if (!d) {
            ADD_FAILURE() << d.error().context;
            return std::nullopt;
        }
        return *d;
    }

    [[nodiscard]] ChainCommand dispatch(uint32_t pipeline, const ag::ProjectionPush& push) const {
        return ag::projection_dispatch(pipeline, push, workgroup_size_);
    }

private:
    Domain domain_{};
    uint32_t workgroup_size_ = kWorkgroupSize;
    ag::MgLayout layout_;
    std::vector<uint32_t> params_;
    std::unique_ptr<ChainRunner> runner_;
};

[[nodiscard]] uint32_t cells_of(const af::PoissonLevel& L) { return L.nx * L.ny * L.nz; }

// The values of `a` and `b` whose bits differ.
[[nodiscard]] std::size_t differing(const std::vector<float>& a, const std::vector<float>& b) {
    std::size_t n = 0;
    for (std::size_t i = 0; i < a.size(); ++i) {
        if (std::bit_cast<uint32_t>(a[i]) != std::bit_cast<uint32_t>(b[i])) ++n;
    }
    return n;
}

// max |a - b| / max |b|.
[[nodiscard]] double relative(const std::vector<float>& a, const std::vector<float>& b) {
    double worst = 0.0;
    double top = 0.0;
    for (std::size_t i = 0; i < a.size(); ++i) {
        worst = std::max(worst, std::abs(static_cast<double>(a[i]) - static_cast<double>(b[i])));
        top = std::max(top, std::abs(static_cast<double>(b[i])));
    }
    return top > 0.0 ? worst / top : worst;
}

}  // namespace

// The residual, the restriction and the prolongation on every level of every
// hierarchy, from fixed-seed noise: adds and multiplies only, so bit for bit.
TEST(GpuAirflowProjection, ResidualRestrictionAndProlongationMatchTheCpuBitForBit) {
    if (const auto why = spade::testing::vulkan_skip_reason()) GTEST_SKIP() << *why;
    auto ctx = make_context();
    ASSERT_NE(ctx, nullptr);
    for (const Domain& d : domains()) {
        SCOPED_TRACE(d.name);
        auto dev = Device::create(*ctx, d);
        ASSERT_TRUE(dev.has_value());
        const af::Multigrid base = [&] {
            af::Multigrid mg = hierarchy(d);
            fill_levels(mg, 0xB2ull);
            return mg;
        }();
        const std::size_t levels = base.levels.size();
        std::printf("measure: %s: %zu levels, coarsest %ux%ux%u, %zu weights\n", d.name, levels,
                    base.levels.back().nx, base.levels.back().ny, base.levels.back().nz,
                    dev->layout().weights.size());
        for (uint32_t l = 0; l < levels; ++l) {
            SCOPED_TRACE("level " + std::to_string(l));
            // residual()
            af::Multigrid cpu = base;
            af::residual(cpu.levels[l]);
            af::Multigrid gpu = base;
            ASSERT_TRUE(dev->upload_levels(base));
            ASSERT_TRUE(dev->run({dev->dispatch(kPipelines.residual, {.a = l, .count = cells_of(base.levels[l])})}));
            ASSERT_TRUE(dev->read_levels(gpu));
            EXPECT_EQ(differing(gpu.levels[l].r, cpu.levels[l].r), 0u) << "residual";
            if (l + 1u == levels) continue;
            // restrict_residual() and cycle()'s zeroed coarse p
            cpu = base;
            af::restrict_residual(cpu.levels[l], cpu.levels[l + 1u]);
            std::fill(cpu.levels[l + 1u].p.begin(), cpu.levels[l + 1u].p.end(), 0.0f);
            ASSERT_TRUE(dev->upload_levels(base));
            ASSERT_TRUE(dev->run(
                {dev->dispatch(kPipelines.restrict_residual, {.a = l, .count = cells_of(base.levels[l + 1u])})}));
            ASSERT_TRUE(dev->read_levels(gpu));
            EXPECT_EQ(differing(gpu.levels[l + 1u].b, cpu.levels[l + 1u].b), 0u) << "restriction";
            EXPECT_EQ(differing(gpu.levels[l + 1u].p, cpu.levels[l + 1u].p), 0u) << "the coarse p zeroed";
            // prolong_add()
            cpu = base;
            af::prolong_add(cpu.levels[l + 1u], cpu.levels[l]);
            ASSERT_TRUE(dev->upload_levels(base));
            ASSERT_TRUE(dev->run({dev->dispatch(kPipelines.prolong, {.a = l, .count = cells_of(base.levels[l])})}));
            ASSERT_TRUE(dev->read_levels(gpu));
            EXPECT_EQ(differing(gpu.levels[l].p, cpu.levels[l].p), 0u) << "prolongation";
        }
    }
}

// The divergence and the gradient subtracted: the projection's first and last
// stages, adds and multiplies only, so bit for bit. The gradient's CPU twin is
// inside project(), so the GPU subtracts project()'s own final p from the same
// u*, and must leave project()'s velocity.
TEST(GpuAirflowProjection, DivergenceAndGradientMatchTheCpuBitForBit) {
    if (const auto why = spade::testing::vulkan_skip_reason()) GTEST_SKIP() << *why;
    auto ctx = make_context();
    ASSERT_NE(ctx, nullptr);
    for (const Domain& d : domains()) {
        SCOPED_TRACE(d.name);
        auto dev = Device::create(*ctx, d);
        ASSERT_TRUE(dev.has_value());
        const af::GridShape& s = d.shape;
        af::MacGrid g = *af::make_grid(s, d.bc);
        fill_velocity(g, 0xD1ull);
        // divergence() * b_coef, project()'s right side
        const af::ProjectionFloats floats = af::projection_floats(s, kRho, Device::kStepS);
        std::vector<float> rhs(g.p.size());
        af::divergence(g, rhs);
        for (float& x : rhs) x = floats.b_coef * x;
        const uint32_t cells = s.nx * s.ny * s.nz;
        ASSERT_TRUE(dev->upload_grid(g));
        ASSERT_TRUE(dev->run({dev->dispatch(kPipelines.divergence, {.count = cells})}));
        std::vector<float> gpu_rhs(rhs.size());
        ASSERT_TRUE(dev->read_rhs(gpu_rhs));
        EXPECT_EQ(differing(gpu_rhs, rhs), 0u) << "divergence";
        // project()'s velocity from its own final p
        if (std::none_of(af::face_kinds(d.bc).begin(), af::face_kinds(d.bc).end(),
                         [](af::FaceKind k) { return k == af::FaceKind::pressure_outlet; })) {
            continue;  // a closed domain's project() removes means first; its gradient is the same code
        }
        af::MacGrid cpu = g;
        af::ProjectionScratch scratch = af::make_projection_scratch(s, d.bc);
        (void)af::project(cpu, d.bc, kRho, Device::kStepS, 2, scratch);
        af::MacGrid start = g;
        start.p = cpu.p;
        ASSERT_TRUE(dev->upload_grid(start));
        const uint32_t faces[3] = {static_cast<uint32_t>(g.u.size()), static_cast<uint32_t>(g.v.size()),
                                   static_cast<uint32_t>(g.w.size())};
        std::vector<ChainCommand> chain;
        for (uint32_t c = 0; c < 3; ++c) chain.push_back(dev->dispatch(kPipelines.gradient, {.a = c, .count = faces[c]}));
        ASSERT_TRUE(dev->run(chain));
        af::MacGrid gpu = g;
        ASSERT_TRUE(dev->read_grid(gpu));
        EXPECT_EQ(differing(gpu.u, cpu.u), 0u) << "u";
        EXPECT_EQ(differing(gpu.v, cpu.v), 0u) << "v";
        EXPECT_EQ(differing(gpu.w, cpu.w), 0u) << "w";
    }
}

// One red-black sweep on every level of every hierarchy, from fixed-seed
// noise, against smooth_red_black(level, 1). The division (sum - b) / diag is
// SPIR-V's OpFDiv, not correctly rounded, so the sweep is banded: 4x the
// largest measurement, max |gpu - cpu| / max |cpu| over the level (TD-14).
// Measured 2026-10-22 (RTX 3060 Ti): see kSweepBand.
TEST(GpuAirflowProjection, OneSweepOnEveryLevelWithinItsBand) {
    if (const auto why = spade::testing::vulkan_skip_reason()) GTEST_SKIP() << *why;
    // Measured 1.71e-7, the worst over the three hierarchies; 4x is 6.84e-7.
    constexpr double kSweepBand = 7.0e-7;
    auto ctx = make_context();
    ASSERT_NE(ctx, nullptr);
    double worst = 0.0;
    std::size_t differ = 0;
    std::size_t total = 0;
    for (const Domain& d : domains()) {
        SCOPED_TRACE(d.name);
        auto dev = Device::create(*ctx, d);
        ASSERT_TRUE(dev.has_value());
        af::Multigrid base = hierarchy(d);
        fill_levels(base, 0x5Eull);
        for (uint32_t l = 0; l < base.levels.size(); ++l) {
            SCOPED_TRACE("level " + std::to_string(l));
            af::Multigrid cpu = base;
            af::smooth_red_black(cpu.levels[l], 1);
            std::vector<ChainCommand> chain;
            ag::append_sweeps(chain, dev->layout(), kPipelines, l, 1, dev->workgroup_size());
            ASSERT_TRUE(dev->upload_levels(base));
            ASSERT_TRUE(dev->run(chain));
            af::Multigrid gpu = base;
            ASSERT_TRUE(dev->read_levels(gpu));
            const double rel = relative(gpu.levels[l].p, cpu.levels[l].p);
            worst = std::max(worst, rel);
            differ += differing(gpu.levels[l].p, cpu.levels[l].p);
            total += gpu.levels[l].p.size();
            EXPECT_LE(rel, kSweepBand);
        }
    }
    std::printf("measure: one sweep: %zu of %zu values differ, worst %.3g of the level's largest\n", differ, total,
                worst);
}

namespace {

// A 32^3-class run of project() against the GPU chain, from the same u* and
// warm start; max |gpu - cpu| / max |cpu| for p and each velocity component.
struct ProjectionDiff {
    double p = 0.0;
    double velocity = 0.0;
    std::size_t differing = 0;
};

[[nodiscard]] std::optional<ProjectionDiff> project_both(VulkanContext& ctx, const Domain& d, uint32_t n_v,
                                                         uint32_t workgroup_size, af::MacGrid* gpu_out = nullptr) {
    auto dev = Device::create(ctx, d, workgroup_size);
    if (!dev) return std::nullopt;
    af::MacGrid g = *af::make_grid(d.shape, d.bc);
    fill_velocity(g, 0x32ull);
    af::MacGrid cpu = g;
    af::ProjectionScratch scratch = af::make_projection_scratch(d.shape, d.bc);
    (void)af::project(cpu, d.bc, kRho, Device::kStepS, n_v, scratch);
    const auto chain =
        ag::projection_chain(dev->layout(), d.shape, af::face_kinds(d.bc), n_v, kPipelines, workgroup_size);
    if (!dev->upload_grid(g) || !dev->run(chain)) return std::nullopt;
    af::MacGrid gpu = g;
    if (!dev->read_grid(gpu)) return std::nullopt;
    ProjectionDiff out;
    out.p = relative(gpu.p, cpu.p);
    out.velocity = std::max({relative(gpu.u, cpu.u), relative(gpu.v, cpu.v), relative(gpu.w, cpu.w)});
    out.differing = differing(gpu.p, cpu.p) + differing(gpu.u, cpu.u) + differing(gpu.v, cpu.v) +
                    differing(gpu.w, cpu.w);
    if (gpu_out != nullptr) *gpu_out = gpu;
    return out;
}

[[nodiscard]] std::vector<Domain> projection_domains() {
    Domain open{"32^3 open", {32, 32, 32, 1.0f / 32.0f}, {}};
    Domain mixed{"33x32x31 walls on x", {33, 32, 31, 1.0f / 32.0f}, {}};
    mixed.bc.face[af::kXMinus] = mixed.bc.face[af::kXPlus] = af::FaceBc::wall;
    Domain ring{"30x32x32 periodic x, z", {30, 32, 32, 1.0f / 32.0f}, {}};
    ring.bc.face[af::kXMinus] = ring.bc.face[af::kXPlus] = af::FaceBc::periodic;
    ring.bc.face[af::kZMinus] = ring.bc.face[af::kZPlus] = af::FaceBc::periodic;
    // Closed: no outlet, so project() removes the means. 18 and 14 leave part
    // tiles at the far edges of tile_sum's 4^3 tiles.
    Domain closed{"18x16x14 closed", {18, 16, 14, 1.0f / 16.0f}, {}};
    for (af::FaceBc& f : closed.bc.face) f = af::FaceBc::wall;
    return {open, mixed, ring, closed};
}

}  // namespace

// One V(2,2) projection (n_v 1) at 32^3 against project(): the divergence,
// the cycle and the gradient, from the same noisy u* and warm start. Banded
// by the sweeps' division: 4x the largest measurement (TD-14).
// Measured 2026-10-22 (RTX 3060 Ti): see kProjectionBand.
TEST(GpuAirflowProjection, OneVCycleProjectionAt32CubedWithinItsBand) {
    if (const auto why = spade::testing::vulkan_skip_reason()) GTEST_SKIP() << *why;
    // Measured velocity 3.33e-7 (18 x 16 x 14 closed), p 2.7e-7 there; 4x the
    // velocity's is 1.33e-6.
    constexpr double kProjectionBand = 1.4e-6;
    auto ctx = make_context();
    ASSERT_NE(ctx, nullptr);
    for (const Domain& d : projection_domains()) {
        SCOPED_TRACE(d.name);
        const auto diff = project_both(*ctx, d, 1, kWorkgroupSize);
        ASSERT_TRUE(diff.has_value());
        std::printf("measure: %s, one V(2,2): p %.3g, velocity %.3g of the largest; %zu values differ\n", d.name,
                    diff->p, diff->velocity, diff->differing);
        EXPECT_LE(diff->p, kProjectionBand);
        EXPECT_LE(diff->velocity, kProjectionBand);
    }
}

// The same chain at workgroup sizes 32, 64 and 128 gives the same bits: no
// reduction, no atomic, and every invocation's work fixed by its index.
TEST(GpuAirflowProjection, SameBitsAtEveryWorkgroupSize) {
    if (const auto why = spade::testing::vulkan_skip_reason()) GTEST_SKIP() << *why;
    auto ctx = make_context();
    ASSERT_NE(ctx, nullptr);
    for (const Domain& d : projection_domains()) {
        SCOPED_TRACE(d.name);
        af::MacGrid first;
        ASSERT_TRUE(project_both(*ctx, d, 2, 64, &first).has_value());
        for (const uint32_t wg : {32u, 128u}) {
            af::MacGrid other;
            ASSERT_TRUE(project_both(*ctx, d, 2, wg, &other).has_value());
            EXPECT_EQ(differing(other.p, first.p) + differing(other.u, first.u) + differing(other.v, first.v) +
                          differing(other.w, first.w),
                      0u)
                << "at " << wg;
        }
    }
}

// The projection's time on the device at 64^3 and 128^3 (M1), open faces,
// n_v 2 as C8 and the films run: the divergence, each V-cycle and the
// gradient, from the runner's marks, the fastest of five submits. With
// SPADE_AIRFLOW_TIMINGS=1 also the D1 box (240 x 177 x 168) and the CPU's
// project() on each, for the ratio.
TEST(GpuAirflowProjection, TimingsAt64And128Cubed) {
    if (const auto why = spade::testing::vulkan_skip_reason()) GTEST_SKIP() << *why;
#if defined(_MSC_VER)
#pragma warning(push)
#pragma warning(disable : 4996)
#endif
    const bool extended = std::getenv("SPADE_AIRFLOW_TIMINGS") != nullptr;
#if defined(_MSC_VER)
#pragma warning(pop)
#endif
    auto ctx = make_context();
    ASSERT_NE(ctx, nullptr);
    std::vector<Domain> boxes = {{"64^3 open", {64, 64, 64, 1.0f / 64.0f}, {}},
                                 {"128^3 open", {128, 128, 128, 1.0f / 128.0f}, {}}};
    if (extended) boxes.push_back({"D1 box 240x177x168 open", {240, 177, 168, 0.01f}, {}});
    constexpr uint32_t kNv = 2;
    for (const Domain& d : boxes) {
        SCOPED_TRACE(d.name);
        auto dev = Device::create(*ctx, d);
        ASSERT_TRUE(dev.has_value());
        af::MacGrid g = *af::make_grid(d.shape, d.bc);
        fill_velocity(g, 0x64ull);
        ASSERT_TRUE(dev->upload_grid(g));
        // Marks: after the divergence, after each cycle, after the gradient.
        std::vector<ChainCommand> chain;
        chain.push_back(ChainCommand{});
        const uint32_t cells = d.shape.nx * d.shape.ny * d.shape.nz;
        chain.push_back(dev->dispatch(kPipelines.divergence, {.count = cells}));
        chain.push_back(ChainCommand{});
        for (uint32_t n = 0; n < kNv; ++n) {
            ag::append_v_cycle(chain, dev->layout(), kPipelines, 0, dev->workgroup_size());
            chain.push_back(ChainCommand{});
        }
        const uint32_t faces[3] = {static_cast<uint32_t>(g.u.size()), static_cast<uint32_t>(g.v.size()),
                                   static_cast<uint32_t>(g.w.size())};
        for (uint32_t c = 0; c < 3; ++c) chain.push_back(dev->dispatch(kPipelines.gradient, {.a = c, .count = faces[c]}));
        chain.push_back(ChainCommand{});
        std::vector<uint64_t> best;
        for (int run = 0; run < 5; ++run) {
            ASSERT_TRUE(dev->upload_grid(g));
            const auto d_ns = dev->run(chain);
            ASSERT_TRUE(d_ns.has_value());
            if (d_ns->empty()) GTEST_SKIP() << "the device cannot time compute work";
            uint64_t sum = 0;
            for (const uint64_t x : *d_ns) sum += x;
            uint64_t best_sum = 0;
            for (const uint64_t x : best) best_sum += x;
            if (best.empty() || sum < best_sum) best = *d_ns;
        }
        uint64_t total = 0;
        for (const uint64_t x : best) total += x;
        // best: [start, divergence, cycle 1, cycle 2, gradient]
        std::printf("measure: %s, n_v %u, %zu levels, %zu dispatches: %.3f ms (divergence %.3f, V-cycles %.3f and "
                    "%.3f, gradient %.3f)\n",
                    d.name, kNv, dev->layout().levels.size(), chain.size() - (kNv + 3u),
                    static_cast<double>(total) * 1e-6, static_cast<double>(best[1]) * 1e-6,
                    static_cast<double>(best[2]) * 1e-6, static_cast<double>(best[3]) * 1e-6,
                    static_cast<double>(best[4]) * 1e-6);
        if (extended) {
            af::MacGrid cpu = g;
            af::ProjectionScratch scratch = af::make_projection_scratch(d.shape, d.bc);
            const auto t0 = std::chrono::steady_clock::now();
            (void)af::project(cpu, d.bc, kRho, Device::kStepS, kNv, scratch);
            const auto t1 = std::chrono::steady_clock::now();
            const double cpu_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
            std::printf("measure: %s, the CPU's project(): %.1f ms, %.1fx the device's\n", d.name, cpu_ms,
                        cpu_ms / (static_cast<double>(total) * 1e-6));
        }
    }
}
