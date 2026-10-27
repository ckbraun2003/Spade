// ---------------------------------------------------------------------------
// test_gpu_airflow_solids.cpp -- plan B, task B5 (B5.1b, and B5.2's operator:
// "B5.2a"): the solids projection's first kernels (engine/shaders/kernels/
// airflow_tile_dot, airflow_pcg_measure, airflow_pcg_scalars,
// airflow_solid_divergence, airflow_solid_gradient and airflow_pcg_apply,
// over engine/shaders/airflow_pcg.slang) against their CPU twins
// (reduce.cpp's fixed_tile_dot(); pcg.cpp's measure and quotients;
// projection.cpp's solid_divergence() and subtract_gradient_with_apertures();
// pressure.cpp's apply_operator()), on the chain runner
// (spike/airflow/chain_runner.hpp), with the solid field packed by
// spike/airflow/solids_chain.hpp's pack_solid_field().
//
//   AirflowSolidsChain  (host, no device; not gpu-labelled) the packer: every
//                       binding against the SolidField it packs, on
//                       build_solids()' sphere and sheet; the pcg block from
//                       pcg.hpp; the host's tables against airflow_pcg.slang's
//   GpuAirflowSolids    bit for bit on the device of record
//                       (airflow_exact_device.hpp; born exact, so no band was
//                       ever measured: another device or driver only
//                       reports): the tile dot in reduce.hpp's two-level
//                       order; the quotients by div32_ftz; the max-norm,
//                       order-free with a NaN planted; the aperture divergence
//                       and gradient, and the operator, on three solids
//                       cases; the same bits at workgroup sizes 32, 64 and 128
//
// THE SOLIDS CASES (the plan's B5.2): S1's configuration at 16^3 (a sphere
// R = 4 dx centred on a cell corner, inflow on -x, the other faces open);
// S3's channel (32 x 8 x 8, every axis periodic) with its sheet on the odd
// plane 17; and an odd grid with a held periodic ring (17 x 14 x 15: x open,
// y periodic, where 14 cannot halve to an even ring, z walls), a sphere at its
// centre. The parts move (a rigid velocity, and the spheres spin), so the wall
// velocity is not 0 near them, and the fields are fixed-seed noise with a
// subnormal in every 61 values, so the flush is exercised too.
//
// Every CPU twin runs under the step's subnormal flush (ScopedFlushSubnormals),
// as fluid_step() runs it. Every Gpu* test starts with the vulkan_skip_reason()
// skip (gpu_skip.hpp).
// ---------------------------------------------------------------------------

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <fstream>
#include <limits>
#include <map>
#include <memory>
#include <optional>
#include <regex>
#include <span>
#include <sstream>
#include <string>
#include <vector>

#include "airflow_exact_device.hpp"
#include "airflow_pcg_apply.spv.gen.hpp"
#include "airflow_pcg_measure.spv.gen.hpp"
#include "airflow_pcg_scalars.spv.gen.hpp"
#include "airflow_solid_divergence.spv.gen.hpp"
#include "airflow_solid_gradient.spv.gen.hpp"
#include "airflow_solid_shapes.hpp"
#include "airflow_tile_dot.spv.gen.hpp"
#include "airflow_tile_sum.spv.gen.hpp"
#include "compute/backend.hpp"
#include "compute/spirv_variants.hpp"
#include "compute/vulkan/context.hpp"
#include "compute/vulkan/dispatch_chain.hpp"
#include "core/detail/flush_subnormals.hpp"
#include "gpu_skip.hpp"
#include "physics/airflow/boundaries.hpp"
#include "physics/airflow/fluid_step.hpp"
#include "physics/airflow/mac_grid.hpp"
#include "physics/airflow/multigrid.hpp"
#include "physics/airflow/pcg.hpp"
#include "physics/airflow/pressure.hpp"
#include "physics/airflow/projection.hpp"
#include "physics/airflow/reduce.hpp"
#include "physics/airflow/regions.hpp"
#include "physics/airflow/solids.hpp"
#include "spike/airflow/chain_runner.hpp"
#include "spike/airflow/projection_chain.hpp"
#include "spike/airflow/solids_chain.hpp"

namespace {

namespace af = spade::physics::airflow;
namespace ag = spade::airflow_gpu;
namespace gen = spade::compute::gen;
using airflow_test::Kind;
using airflow_test::Scene;
using airflow_test::Shape;
using spade::airflow_gpu::BufferKind;
using spade::airflow_gpu::ChainRunner;
using spade::airflow_gpu::RunnerBuffer;
using spade::airflow_gpu::RunnerDesc;
using spade::compute::ChainCommand;
using spade::compute::SpirvVariantSet;
using spade::compute::VulkanContext;

constexpr float kRho = 1.225f;
constexpr float kStepS = 1.0f / 512.0f;
constexpr uint32_t kWorkgroupSize = 64;

[[nodiscard]] uint32_t bits(float f) noexcept { return std::bit_cast<uint32_t>(f); }

[[nodiscard]] std::string hex(float f) {
    char text[16];
    std::snprintf(text, sizeof(text), "0x%08x", bits(f));
    return text;
}

template <class T>
[[nodiscard]] std::span<const std::byte> bytes_of(const std::vector<T>& v) noexcept {
    return std::as_bytes(std::span<const T>(v));
}

template <class T>
[[nodiscard]] std::span<std::byte> writable_bytes_of(std::vector<T>& v) noexcept {
    return std::as_writable_bytes(std::span<T>(v));
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
    // In [-1, 1), from the top 24 bits: exact in a float.
    float unit() noexcept {
        return static_cast<float>(static_cast<int64_t>(next() >> 40) - (int64_t{1} << 23)) * 0x1p-23f;
    }
};

// A float of either sign between 2^-12 and 2^13, built from bits (no libm):
// sums of these and of their products depend on their order in fp32.
[[nodiscard]] float mixed_magnitude(uint64_t r) noexcept {
    const uint32_t sign = static_cast<uint32_t>(r >> 63) << 31;
    const uint32_t exponent = 127u - 12u + static_cast<uint32_t>((r >> 32) % 25u);
    const uint32_t mantissa = static_cast<uint32_t>(r) & 0x7FFFFFu;
    return std::bit_cast<float>(sign | (exponent << 23) | mantissa);
}

// Noise of the given scale, with one value in 61 a subnormal of either sign,
// which the flush must treat alike on both twins.
void fill_noise(std::vector<float>& v, uint64_t seed, float scale) {
    SplitMix rng{seed};
    std::size_t n = 0;
    for (float& x : v) x = (++n % 61u == 0u) ? (rng.unit() < 0.0f ? -1.0e-40f : 3.0e-41f) : scale * rng.unit();
}

// The values of `a` and `b` whose bits differ.
[[nodiscard]] std::size_t differing(std::span<const float> a, std::span<const float> b) {
    std::size_t n = 0;
    for (std::size_t i = 0; i < a.size(); ++i) n += bits(a[i]) != bits(b[i]) ? 1u : 0u;
    return n;
}

[[nodiscard]] uint32_t cells_of(const af::GridShape& s) noexcept { return s.nx * s.ny * s.nz; }

[[nodiscard]] std::array<uint32_t, 3> faces_of(const af::GridShape& s) noexcept {
    return {(s.nx + 1u) * s.ny * s.nz, s.nx * (s.ny + 1u) * s.nz, s.nx * s.ny * (s.nz + 1u)};
}

[[nodiscard]] std::unique_ptr<VulkanContext> make_context() {
    auto result = VulkanContext::create(spade::compute::BackendDesc{.kind = spade::compute::BackendKind::vulkan});
    if (!result.has_value()) return nullptr;
    return std::move(*result);
}

[[nodiscard]] bool check(const spade::Result<void>& r) {
    if (r.has_value()) return true;
    ADD_FAILURE() << r.error().context;
    return false;
}

// ===========================================================================
// The solids cases
// ===========================================================================

struct SolidsCase {
    std::string name;
    af::GridShape shape;
    af::DomainBc bc;
    Scene scene;
};

// S1's configuration at 16^3: a sphere R = 4 dx on the box's middle cell
// corner, inflow U = 1 m/s on -x, the other faces open; it translates and
// spins.
[[nodiscard]] SolidsCase s1_sphere() {
    const float dx = 0.01f;
    SolidsCase c{"S1's sphere at 16^3", {16, 16, 16, dx}, {}, {}};
    c.bc.ambient = glm::vec3(1.0f, 0.0f, 0.0f);
    c.scene.shapes.push_back(Shape{.name = "sphere", .kind = Kind::sphere, .centre = glm::vec3(8.0f * dx),
                                   .radius = 4.0f * dx, .omega = {1.5f, -2.0f, 3.0f},
                                   .velocity = {0.25f, -0.125f, 0.5f}, .min_feature = 8.0f * dx});
    return c;
}

// S3's channel: 32 x 8 x 8, dx 1/32 m, every axis periodic, its sheet 0.25 dx
// thick across the section on face plane 17 (test_airflow_solids_step.cpp's
// s3_sheet()); the sheet moves.
[[nodiscard]] SolidsCase s3_sheet() {
    const float dx = 1.0f / 32.0f;
    SolidsCase c{"S3's sheet on plane 17", {32, 8, 8, dx}, {}, {}};
    c.bc.face.fill(af::FaceBc::periodic);
    c.scene.shapes.push_back(Shape{.name = "sheet", .kind = Kind::box, .centre = {17.0f * dx, 4.0f * dx, 4.0f * dx},
                                   .half = {0.125f * dx, 20.0f * dx, 20.0f * dx},
                                   .velocity = {0.5f, 0.25f, -0.25f}, .min_feature = 0.25f * dx});
    return c;
}

// An odd grid with a held periodic ring: 17 x 14 x 15, x open (inflow on -x),
// y periodic (14 = 2 mod 4: held), z walls; a sphere R = 4 dx at its centre.
[[nodiscard]] SolidsCase held_ring() {
    const float dx = 0.01f;
    SolidsCase c{"17x14x15, y a held ring", {17, 14, 15, dx}, {}, {}};
    c.bc.ambient = glm::vec3(1.0f, 0.0f, 0.0f);
    c.bc.face[af::kYMinus] = c.bc.face[af::kYPlus] = af::FaceBc::periodic;
    c.bc.face[af::kZMinus] = c.bc.face[af::kZPlus] = af::FaceBc::wall;
    c.scene.shapes.push_back(Shape{.name = "sphere", .kind = Kind::sphere,
                                   .centre = 0.5f * dx * glm::vec3(17.0f, 14.0f, 15.0f), .radius = 4.0f * dx,
                                   .omega = {-1.0f, 2.5f, 0.5f}, .velocity = {-0.375f, 0.25f, 0.125f},
                                   .min_feature = 8.0f * dx});
    return c;
}

[[nodiscard]] std::vector<SolidsCase> solids_cases() { return {s1_sphere(), s3_sheet(), held_ring()}; }

// A case built as the CPU step builds it: set_solids() (the solid field, the
// regions, level 0's weights from theta and the coarse levels' from them),
// and the face kinds set on the multigrid as project() sets them.
struct Built {
    std::string name;
    af::GridShape shape;
    af::DomainBc bc;
    af::FluidScratch s;
    ag::MgLayout layout;
    uint32_t closed_regions = 0;
};

[[nodiscard]] std::optional<Built> build(const SolidsCase& c) {
    const af::FluidConfig cfg{.rho = kRho, .nu = 0.0f, .h = kStepS, .n_v = af::kSolidsCgIterations,
                              .interior = af::SolidInterior::extrapolate};
    const af::MacGrid g = *af::make_grid(c.shape, c.bc);
    auto s = af::make_fluid_scratch(g, c.bc, cfg);
    if (!s) {
        ADD_FAILURE() << c.name << ": " << s.error().context;
        return std::nullopt;
    }
    Built b{c.name, c.shape, c.bc, std::move(*s), {}, 0};
    const auto parts = c.scene.parts();
    if (const auto set = af::set_solids(b.s, g, c.bc, parts, c.scene.sampler()); !set) {
        ADD_FAILURE() << c.name << ": " << set.error().context;
        return std::nullopt;
    }
    if (!b.s.solids.any) {
        ADD_FAILURE() << c.name << ": no solid";
        return std::nullopt;
    }
    af::set_kinds(b.s.projection.mg, af::face_kinds(c.bc));
    (void)af::check_regions(b.s.regions, c.shape, c.bc, b.s.solids);
    for (const uint8_t closed : b.s.regions.closed) b.closed_regions += closed != 0u ? 1u : 0u;
    auto layout = ag::mg_layout(b.s.projection.mg);
    if (!layout) {
        ADD_FAILURE() << layout.error().context;
        return std::nullopt;
    }
    b.layout = std::move(*layout);
    return b;
}

// A grid of fixed-seed noise on every face and cell.
[[nodiscard]] af::MacGrid noisy_grid(const af::GridShape& s, const af::DomainBc& bc, uint64_t seed) {
    af::MacGrid g = *af::make_grid(s, bc);
    fill_noise(g.u, seed, 1.0f);
    fill_noise(g.v, seed + 1u, 0.5f);
    fill_noise(g.w, seed + 2u, 1.0f);
    fill_noise(g.p, seed + 3u, 4.0f);
    return g;
}

// ===========================================================================
// The device
// ===========================================================================

// Set 1, bindings 0..23 (bindings.slang section F); the runner binds
// buffers[i] at binding i, the unread ones as 4-byte placeholders.
enum Buffer : uint32_t {
    kParams = 0,
    kU = 1,
    kV = 2,
    kW = 3,
    kPressure = 4,
    kMgWeights = 12,
    kApertures = 13,
    kWallVelocity = 14,
    kPhi = 15,
    kFaceTags = 16,
    kWallLinks = 17,
    kRegionLabels = 18,
    kPcg = 20,
    kScalars = 21,
    kPartials = 23,
    kBufferCount = 24,
};

// RunnerDesc::kernels' order.
enum Pipeline : uint32_t {
    kTileSumPipeline = 0,
    kTileDotPipeline,
    kMeasurePipeline,
    kScalarsPipeline,
    kSolidDivergencePipeline,
    kSolidGradientPipeline,
    kPcgApplyPipeline,
};

constexpr ag::SolidsPipelines kPipelines{.tile_sum = kTileSumPipeline,
                                         .tile_dot = kTileDotPipeline,
                                         .measure = kMeasurePipeline,
                                         .scalars = kScalarsPipeline};

// The kernels' push (each the airflow convention: 16 bytes, the count in
// word 2) is projection_chain.hpp's ProjectionPush.
static_assert(sizeof(ag::ProjectionPush) == 16);

// airflow_params for `shape`: the dims, dx, 1 / dx, h and the periodic axes,
// and, with a built case, the projection's words (face kinds, coefficients,
// level table), word 16's solid bits and the pcg block after the level table.
[[nodiscard]] std::vector<uint32_t> params_for(const af::GridShape& s, const Built* b) {
    std::vector<uint32_t> p(32, 0u);
    if (b != nullptr) p.assign(ag::projection_params_words(b->layout), 0u);
    p[0] = s.nx;
    p[1] = s.ny;
    p[2] = s.nz;
    p[4] = bits(s.dx);
    p[5] = bits(1.0f / s.dx);
    p[6] = bits(kStepS);
    if (b == nullptr) return p;
    const std::array<bool, 3> periodic = af::periodic_axes(b->bc);
    for (uint32_t a = 0; a < 3; ++a) p[8 + a] = periodic[a] ? 1u : 0u;
    ag::write_projection_params(p, b->layout, af::face_kinds(b->bc), af::projection_floats(s, kRho, kStepS));
    p[ag::kParamsFlags] = ag::solids_flags(ag::pack_solid_field(b->s.solids));
    const auto block = static_cast<uint32_t>(p.size());
    p.resize(block + ag::kPcgBlockWords, 0u);
    ag::write_pcg_block(p, block, b->closed_regions, af::kSolidsCgIterations, af::kSolidsCgCap);
    return p;
}

// B5's kernels on one grid (and, with a built case, its solids).
class Device {
public:
    [[nodiscard]] static std::optional<Device> create(VulkanContext& ctx, const af::GridShape& s, const Built* b,
                                                      uint32_t workgroup_size = kWorkgroupSize) {
        Device dev;
        dev.shape_ = s;
        dev.workgroup_size_ = workgroup_size;
        const std::vector<uint32_t> params = params_for(s, b);
        const uint64_t cells = cells_of(s);
        const auto faces = faces_of(s);
        const auto size_of = [](const auto& v) {
            return std::max<uint64_t>(4u, uint64_t{v.size()} * sizeof(v[0]));
        };
        std::vector<RunnerBuffer> buffers(kBufferCount);
        for (uint32_t i = 0; i < kBufferCount; ++i) {
            buffers[i] = {std::string(ag::kAirflowSetBuffers[i].name), 4u, BufferKind::device_only};
        }
        buffers[kParams] = {"airflow_params", params.size() * 4u, BufferKind::staged};
        buffers[kU] = {"airflow_u", faces[0] * 4ull, BufferKind::staged};
        buffers[kV] = {"airflow_v", faces[1] * 4ull, BufferKind::staged};
        buffers[kW] = {"airflow_w", faces[2] * 4ull, BufferKind::staged};
        buffers[kPressure] = {"airflow_pressure", cells * 4u, BufferKind::staged};
        buffers[kPcg] = {"airflow_pcg", ag::kPcgVectors * cells * 4u, BufferKind::staged};
        buffers[kScalars] = {"airflow_scalars", ag::kScalarSlots * 4u, BufferKind::staged};
        buffers[kPartials] = {"airflow_partials", 2ull * ag::tile_sum_tiles(s) * 4u, BufferKind::staged};
        ag::PackedSolidField packed;
        if (b != nullptr) {
            packed = ag::pack_solid_field(b->s.solids);
            buffers[kMgWeights] = {"airflow_mg_weights", size_of(b->layout.weights), BufferKind::resident};
            buffers[kApertures] = {"airflow_apertures", size_of(packed.apertures), BufferKind::resident};
            buffers[kWallVelocity] = {"airflow_wall_velocity", size_of(packed.wall_velocity), BufferKind::resident};
            buffers[kPhi] = {"airflow_phi", size_of(packed.phi), BufferKind::resident};
            buffers[kFaceTags] = {"airflow_face_tags", size_of(packed.face_tags), BufferKind::resident};
            buffers[kWallLinks] = {"airflow_wall_links", size_of(packed.wall_links), BufferKind::resident};
            buffers[kRegionLabels] = {"airflow_region_labels", size_of(b->s.regions.label), BufferKind::resident};
        }
        const SpirvVariantSet* const kernels[] = {
            &gen::kSpvVariants_airflow_tile_sum,         &gen::kSpvVariants_airflow_tile_dot,
            &gen::kSpvVariants_airflow_pcg_measure,      &gen::kSpvVariants_airflow_pcg_scalars,
            &gen::kSpvVariants_airflow_solid_divergence, &gen::kSpvVariants_airflow_solid_gradient,
            &gen::kSpvVariants_airflow_pcg_apply};
        auto runner = ChainRunner::create(ctx, RunnerDesc{.buffers = buffers,
                                                          .kernels = kernels,
                                                          .push_bytes = sizeof(ag::ProjectionPush),
                                                          .workgroup_size = workgroup_size});
        if (!runner) {
            ADD_FAILURE() << runner.error().context;
            return std::nullopt;
        }
        dev.runner_ = std::move(*runner);
        ChainRunner& r = *dev.runner_;
        if (!check(r.upload(kParams, bytes_of(params)))) return std::nullopt;
        if (b != nullptr) {
            const auto upload = [&](uint32_t at, const auto& v) {
                return v.empty() || check(r.upload(at, bytes_of(v)));
            };
            if (!upload(kMgWeights, b->layout.weights) || !upload(kApertures, packed.apertures) ||
                !upload(kWallVelocity, packed.wall_velocity) || !upload(kPhi, packed.phi) ||
                !upload(kFaceTags, packed.face_tags) || !upload(kWallLinks, packed.wall_links) ||
                !upload(kRegionLabels, b->s.regions.label)) {
                return std::nullopt;
            }
        }
        return dev;
    }

    [[nodiscard]] uint32_t workgroup_size() const noexcept { return workgroup_size_; }

    [[nodiscard]] bool upload(Buffer at, const std::vector<float>& v) {
        return check(runner_->upload(at, bytes_of(v)));
    }
    [[nodiscard]] bool read(Buffer at, std::vector<float>& v) const {
        return check(runner_->read(at, writable_bytes_of(v)));
    }

    [[nodiscard]] bool upload_grid(const af::MacGrid& g) {
        return upload(kU, g.u) && upload(kV, g.v) && upload(kW, g.w) && upload(kPressure, g.p);
    }
    [[nodiscard]] bool read_grid(af::MacGrid& g) const {
        return read(kU, g.u) && read(kV, g.v) && read(kW, g.w) && read(kPressure, g.p);
    }

    // Records and submits `chain` once.
    [[nodiscard]] bool run(const std::vector<ChainCommand>& chain) {
        const auto id = runner_->record(chain);
        if (!id) {
            ADD_FAILURE() << id.error().context;
            return false;
        }
        return check(runner_->submit(*id, 1));
    }

    [[nodiscard]] ChainCommand dispatch(Pipeline pipeline, const ag::ProjectionPush& push) const {
        return ag::projection_dispatch(pipeline, push, workgroup_size_);
    }

private:
    af::GridShape shape_{};
    uint32_t workgroup_size_ = kWorkgroupSize;
    std::unique_ptr<ChainRunner> runner_;
};

// airflow_pcg's contents: the seven vectors, each `cells` floats.
struct PcgVectors {
    uint32_t cells = 0;
    std::vector<float> all;

    explicit PcgVectors(uint32_t n, float fill = std::numeric_limits<float>::quiet_NaN())
        : cells(n), all(std::size_t{ag::kPcgVectors} * n, fill) {}

    [[nodiscard]] std::span<float> at(ag::PcgVector v) {
        return std::span<float>(all).subspan(static_cast<std::size_t>(v) * cells, cells);
    }
    [[nodiscard]] std::vector<float> copy(ag::PcgVector v) const {
        const auto first = all.begin() + static_cast<std::ptrdiff_t>(static_cast<std::size_t>(v) * cells);
        return {first, first + cells};
    }
    void set(ag::PcgVector v, const std::vector<float>& x) { std::copy(x.begin(), x.end(), at(v).begin()); }
};

// The device of record asserts; another device or driver reports (TD-14 as
// amended 2026-10-22; these kernels were born exact, so no band exists).
void expect_none_differ(bool bit_equal, std::size_t n, std::size_t of, const std::string& what) {
    std::printf("measure: %s: %zu of %zu values differ\n", what.c_str(), n, of);
    if (bit_equal) {
        EXPECT_EQ(n, 0u) << what;
    }
}

}  // namespace

// ===========================================================================
// AirflowSolidsChain -- the host side, no device
// ===========================================================================

// pack_solid_field() against the field it packs, every binding and every
// face, on build_solids()' sphere and a sheet in one grid (both parts move, so
// the wall velocity is set; the sphere leaves wall links and the free-slip
// layers): theta, the wall velocity and phi as floats from aperture_base(c);
// the link per face, then the layer bytes from word F, little-endian; a link
// in 12 words; the solid bits iff SolidField::any. And a field with no part.
TEST(AirflowSolidsChain, ThePackerWritesBindings13To17ByTheConfirmedRule) {
    const af::GridShape shape{20, 18, 17, 0.01f};
    Scene scene;
    scene.shapes.push_back(Shape{.name = "sphere", .kind = Kind::sphere, .centre = {0.07f, 0.09f, 0.085f},
                                 .radius = 0.04f, .omega = {0.0f, 1.0f, 2.0f}, .velocity = {0.5f, 0.0f, -0.25f},
                                 .min_feature = 0.08f});
    scene.shapes.push_back(Shape{.name = "sheet", .kind = Kind::box, .centre = {0.155f, 0.09f, 0.085f},
                                 .half = {0.001f, 0.05f, 0.05f}, .velocity = {0.0f, 0.75f, 0.0f},
                                 .min_feature = 0.002f});
    af::SolidField f = *af::make_solid_field(shape);
    ASSERT_TRUE(af::build_solids(f, shape, {false, false, false}, scene.parts(), scene.sampler()).has_value());
    ASSERT_TRUE(f.any) << f.run_line;
    ASSERT_NE(f.run_line.find("1 sheets"), std::string::npos) << f.run_line;

    const ag::PackedSolidField p = ag::pack_solid_field(f);
    const std::size_t faces = f.theta[0].size() + f.theta[1].size() + f.theta[2].size();
    ASSERT_EQ(p.apertures.size(), faces);
    ASSERT_EQ(p.wall_velocity.size(), faces);
    ASSERT_EQ(p.phi.size(), faces);
    ASSERT_EQ(p.face_tags.size(), faces + (faces + 3u) / 4u);
    ASSERT_EQ(p.wall_links.size(), f.links.size() * ag::kWallLinkWords);
    EXPECT_TRUE(p.has_solids);
    EXPECT_EQ(ag::solids_flags(p), ag::kParamsFlagSolids | ag::kParamsFlagTheta);

    // Every face: what the field holds, where the rule puts it.
    std::size_t mismatched = 0;
    std::size_t closed = 0;
    std::size_t linked = 0;
    std::size_t layered = 0;
    std::size_t moving = 0;
    for (uint32_t c = 0; c < 3; ++c) {
        const uint64_t base = ag::aperture_base(shape, c);
        ASSERT_EQ(base, c == 0 ? 0u : (c == 1 ? f.theta[0].size() : f.theta[0].size() + f.theta[1].size()));
        for (std::size_t idx = 0; idx < f.theta[c].size(); ++idx) {
            const std::size_t at = base + idx;
            const uint32_t layer = (p.face_tags[faces + at / 4u] >> (8u * (at % 4u))) & 0xFFu;
            const bool same = bits(p.apertures[at]) == bits(f.theta[c][idx]) &&
                              bits(p.wall_velocity[at]) == bits(f.wall[c][idx]) &&
                              bits(p.phi[at]) == bits(f.phi[c][idx]) && p.face_tags[at] == f.link[c][idx] &&
                              layer == f.layer[c][idx];
            mismatched += same ? 0u : 1u;
            closed += af::closed(f.theta[c][idx]) ? 1u : 0u;
            linked += f.link[c][idx] != af::kNoWallLinks ? 1u : 0u;
            layered += f.layer[c][idx] != 0u ? 1u : 0u;
            moving += f.wall[c][idx] != 0.0f ? 1u : 0u;
        }
    }
    EXPECT_EQ(mismatched, 0u);
    // The case exercises each part of the rule.
    EXPECT_GT(closed, 0u);
    EXPECT_GT(linked, 0u);
    EXPECT_GT(layered, 0u);
    EXPECT_GT(moving, 0u);
    // The layer words' unused bytes past the last face are 0.
    for (std::size_t at = faces; at < (faces + 3u) / 4u * 4u; ++at) {
        EXPECT_EQ((p.face_tags[faces + at / 4u] >> (8u * (at % 4u))) & 0xFFu, 0u) << "byte " << at;
    }
    // Every wall link: inv_f[0..5], then wall[0..5].
    std::size_t link_mismatched = 0;
    for (std::size_t l = 0; l < f.links.size(); ++l) {
        for (uint32_t s = 0; s < 6; ++s) {
            link_mismatched += p.wall_links[ag::kWallLinkWords * l + s] == bits(f.links[l].inv_f[s]) ? 0u : 1u;
            link_mismatched += p.wall_links[ag::kWallLinkWords * l + 6u + s] == bits(f.links[l].wall[s]) ? 0u : 1u;
        }
    }
    EXPECT_EQ(link_mismatched, 0u);
    std::printf("measure: packed %zu faces (%zu closed, %zu linked, %zu in a free-slip layer), %zu wall links\n",
                faces, closed, linked, layered, f.links.size());

    // No part: every face open, no solid bit, no wall link.
    const af::SolidField open = *af::make_solid_field(shape);
    const ag::PackedSolidField q = ag::pack_solid_field(open);
    EXPECT_FALSE(q.has_solids);
    EXPECT_EQ(ag::solids_flags(q), 0u);
    EXPECT_EQ(q.apertures.size(), faces);
    EXPECT_TRUE(q.wall_links.empty());
}

// The pcg block: word 28 its offset, then the closed-region count, pcg.hpp's
// kResidualAnnounce, kStallGain and kStallIterations, and the floor and cap,
// nothing else written.
TEST(AirflowSolidsChain, ThePcgBlockHoldsPcgHppsConstants) {
    std::vector<uint32_t> params(64, 0xA5A5A5A5u);
    ag::write_pcg_block(params, 50, 3, af::kSolidsCgIterations, af::kSolidsCgCap);
    EXPECT_EQ(params[ag::kParamsPcgBlock], 50u);
    EXPECT_EQ(params[50], 3u);
    EXPECT_EQ(params[51], bits(af::kResidualAnnounce));
    EXPECT_EQ(params[52], bits(af::kStallGain));
    EXPECT_EQ(params[53], af::kStallIterations);
    EXPECT_EQ(params[54], af::kSolidsCgIterations);
    EXPECT_EQ(params[55], af::kSolidsCgCap);
    std::size_t touched = 0;
    for (std::size_t w = 0; w < params.size(); ++w) {
        if (w == ag::kParamsPcgBlock || (w >= 50 && w < 50 + ag::kPcgBlockWords)) continue;
        touched += params[w] != 0xA5A5A5A5u ? 1u : 0u;
    }
    EXPECT_EQ(touched, 0u);
}

// The host's tables are airflow_pcg.slang's: every `static const uint kPcg*`
// and `kSlot*` in the module has the value of solids_chain.hpp's constant of
// the same name, and the vectors are pcg_bytes()' seven floats a cell.
TEST(AirflowSolidsChain, TheHostsTablesAreAirflowPcgSlangs) {
    const std::map<std::string, uint32_t> host{
        {"kPcgX", ag::kPcgX},       {"kPcgB", ag::kPcgB},         {"kPcgR", ag::kPcgR},
        {"kPcgZ", ag::kPcgZ},       {"kPcgD", ag::kPcgD},         {"kPcgQ", ag::kPcgQ},
        {"kPcgBest", ag::kPcgBest}, {"kPcgVectors", ag::kPcgVectors}, {"kSlotRz", ag::kSlotRz},
        {"kSlotDq", ag::kSlotDq},   {"kSlotRzNext", ag::kSlotRzNext}, {"kSlotAlpha", ag::kSlotAlpha},
        {"kSlotBeta", ag::kSlotBeta}, {"kSlotRMax", ag::kSlotRMax},   {"kSlotBMax", ag::kSlotBMax},
        {"kSlotResidual", ag::kSlotResidual}};
    std::ifstream in(std::string(SPADE_ENGINE_DIR) + "/shaders/airflow_pcg.slang");
    ASSERT_TRUE(in.good()) << SPADE_ENGINE_DIR << "/shaders/airflow_pcg.slang";
    std::stringstream text;
    text << in.rdbuf();
    const std::string source = text.str();
    const std::regex constant(R"(static const uint (k(?:Pcg|Slot)\w+) = (\d+)u;)");
    std::map<std::string, uint32_t> device;
    for (auto it = std::sregex_iterator(source.begin(), source.end(), constant); it != std::sregex_iterator(); ++it) {
        device[(*it)[1].str()] = static_cast<uint32_t>(std::stoul((*it)[2].str()));
    }
    EXPECT_EQ(device, host);
    EXPECT_EQ(uint64_t{ag::kPcgVectors} * 1000u * sizeof(float), af::pcg_bytes(1000u));
    EXPECT_EQ(ag::pcg_offset(ag::kPcgBest, af::GridShape{5, 7, 9, 1.0f}), 6u * 5u * 7u * 9u);
    for (const auto& [name, slot] : host) {
        if (name.rfind("kSlot", 0) != 0) continue;
        EXPECT_GT(slot, ag::kMeanSlot) << name << ": slot 0 is remove_mean()'s";
        EXPECT_LT(slot, ag::kScalarSlots) << name;
    }
}

// ===========================================================================
// GpuAirflowSolids -- B5.1b: the dot, the quotients, the max
// ===========================================================================

namespace {

// airflow_scalars before a run: every slot a sentinel, so a write to the
// wrong slot shows.
constexpr float kScalarSentinel = -12345.0f;

// Each tile's dot from 0 in (k, j, i) order, the tiles in (tk, tj, ti) order:
// reduce.hpp's tiles, spelled out, each product rounded first. (i, j, k) order
// inside a tile with `ijk`.
[[nodiscard]] std::vector<float> tile_dots(const std::vector<float>& a, const std::vector<float>& b, uint32_t nx,
                                           uint32_t ny, uint32_t nz, bool ijk = false) {
    std::vector<float> out;
    for (uint32_t tk = 0; tk < nz; tk += 4) {
        for (uint32_t tj = 0; tj < ny; tj += 4) {
            for (uint32_t ti = 0; ti < nx; ti += 4) {
                float s = 0.0f;
                const uint32_t ie = std::min(ti + 4u, nx);
                const uint32_t je = std::min(tj + 4u, ny);
                const uint32_t ke = std::min(tk + 4u, nz);
                const auto add = [&](uint32_t i, uint32_t j, uint32_t k) {
                    const std::size_t c = i + std::size_t{nx} * (j + std::size_t{ny} * k);
                    const float product = a[c] * b[c];
                    s += product;
                };
                if (ijk) {
                    for (uint32_t i = ti; i < ie; ++i)
                        for (uint32_t j = tj; j < je; ++j)
                            for (uint32_t k = tk; k < ke; ++k) add(i, j, k);
                } else {
                    for (uint32_t k = tk; k < ke; ++k)
                        for (uint32_t j = tj; j < je; ++j)
                            for (uint32_t i = ti; i < ie; ++i) add(i, j, k);
                }
                out.push_back(s);
            }
        }
    }
    return out;
}

[[nodiscard]] float block_sum(const std::vector<float>& tiles, std::size_t b, uint32_t block) {
    float s = 0.0f;
    for (std::size_t t = b * block; t < std::min((b + 1) * block, tiles.size()); ++t) s += tiles[t];
    return s;
}

[[nodiscard]] float two_level_total(const std::vector<float>& tiles, uint32_t block) {
    float total = 0.0f;
    for (std::size_t b = 0; b * block < tiles.size(); ++b) total += block_sum(tiles, b, block);
    return total;
}

// airflow_partials after a dot: each tile's own dot, but block b's first
// tile, which holds the block's sum (tile_sum's pass 1 writes it in place).
[[nodiscard]] std::vector<float> partials_after(const std::vector<float>& tiles, uint32_t block) {
    std::vector<float> out = tiles;
    for (std::size_t b = 0; b * block < tiles.size(); ++b) out[b * block] = block_sum(tiles, b, block);
    return out;
}

// Two vectors of mixed magnitudes, one value in 37 near 1e-20 so its product
// is subnormal and flushed on both twins.
[[nodiscard]] std::array<std::vector<float>, 2> dot_vectors(uint32_t cells, uint64_t seed) {
    std::array<std::vector<float>, 2> v;
    for (uint32_t f = 0; f < 2; ++f) {
        SplitMix rng{seed + f};
        v[f].resize(cells);
        std::size_t n = 0;
        for (float& x : v[f]) {
            x = mixed_magnitude(rng.next());
            if (++n % 37u == 0u) x *= 0x1p-60f;
        }
    }
    return v;
}

// fixed_tile_dot(r, z) on the device at every workgroup size: its total in
// kSlotRz, every other slot untouched, and its partials the CPU's tiles with
// each block's sum on its first tile; all the CPU's bits.
void check_tile_dot(uint32_t nx, uint32_t ny, uint32_t nz, uint64_t seed) {
    auto ctx = make_context();
    ASSERT_NE(ctx, nullptr);
    const bool bit_equal = spade::testing::airflow_bit_equal_device(*ctx);
    constexpr uint32_t kBlock = af::kReduceBlockTiles;
    const af::GridShape shape{nx, ny, nz, 1.0f};
    const uint32_t cells = cells_of(shape);
    const auto [r, z] = dot_vectors(cells, seed);

    float cpu = 0.0f;
    std::vector<float> tiles;
    float array_order = 0.0f;
    float ijk_order = 0.0f;
    {
        const spade::math::ScopedFlushSubnormals flush;  // the step's mode
        cpu = af::fixed_tile_dot(r, z, nx, ny, nz);
        tiles = tile_dots(r, z, nx, ny, nz);
        for (uint32_t c = 0; c < cells; ++c) {
            const float product = r[c] * z[c];
            array_order += product;
        }
        ijk_order = two_level_total(tile_dots(r, z, nx, ny, nz, true), kBlock);
        EXPECT_EQ(bits(two_level_total(tiles, kBlock)), bits(cpu)) << "the spelled-out order is not reduce.hpp's";
    }
    // The data tells the mutations' orders apart.
    EXPECT_NE(bits(array_order), bits(cpu)) << "the data does not tell array order apart; change the seed";
    EXPECT_NE(bits(ijk_order), bits(cpu)) << "the data does not tell (i, j, k) order apart; change the seed";
    std::printf("[GpuAirflowSolids] dot %ux%ux%u: cpu %s, array order %s, (i, j, k) tiles %s, %zu tiles\n", nx, ny,
                nz, hex(cpu).c_str(), hex(array_order).c_str(), hex(ijk_order).c_str(), tiles.size());

    for (const uint32_t wg : spade::compute::kSupportedWorkgroupSizes) {
        SCOPED_TRACE("workgroup size " + std::to_string(wg));
        auto dev = Device::create(*ctx, shape, nullptr, wg);
        ASSERT_TRUE(dev.has_value());
        PcgVectors pcg(cells);
        pcg.set(ag::kPcgR, r);
        pcg.set(ag::kPcgZ, z);
        ASSERT_TRUE(dev->upload(kPcg, pcg.all));
        ASSERT_TRUE(dev->upload(kScalars, std::vector<float>(ag::kScalarSlots, kScalarSentinel)));
        std::vector<ChainCommand> chain;
        ag::append_tile_dot(chain, shape, kPipelines, ag::kPcgR, ag::kPcgZ, ag::kSlotRz, wg);
        ASSERT_TRUE(dev->run(chain));
        std::vector<float> scalars(ag::kScalarSlots);
        std::vector<float> partials(2u * ag::tile_sum_tiles(shape));
        ASSERT_TRUE(dev->read(kScalars, scalars));
        ASSERT_TRUE(dev->read(kPartials, partials));
        partials.resize(tiles.size());
        const std::vector<float> want = partials_after(tiles, kBlock);
        const std::size_t partial_mismatches = differing(partials, want);
        std::printf("[GpuAirflowSolids] dot %ux%ux%u wg %u: gpu %s, partial mismatches %zu\n", nx, ny, nz, wg,
                    hex(scalars[ag::kSlotRz]).c_str(), partial_mismatches);
        expect_none_differ(bit_equal, bits(scalars[ag::kSlotRz]) != bits(cpu) ? 1u : 0u, 1u, "the dot");
        expect_none_differ(bit_equal, partial_mismatches, want.size(), "the dot's partials");
        std::size_t disturbed = 0;
        for (uint32_t s = 0; s < ag::kScalarSlots; ++s) {
            if (s != ag::kSlotRz) disturbed += bits(scalars[s]) != bits(kScalarSentinel) ? 1u : 0u;
        }
        EXPECT_EQ(disturbed, 0u) << "a slot other than r . z's was written";
    }
}

}  // namespace

// fixed_tile_dot() on the device, the CG's r . z: ragged tiles (one block of
// 12); two whole blocks (32^3, 512 tiles); and a ragged last block of ragged
// tiles (61 x 45 x 43, 2112 tiles: 8 blocks and 64). Mutations: the products
// summed in array order; a tile's products in (i, j, k) order.
TEST(GpuAirflowSolids, TileDotSameBitsAtEveryWorkgroupSizeAndAsTheCpu) {
    if (const auto why = spade::testing::vulkan_skip_reason()) GTEST_SKIP() << *why;
    {
        SCOPED_TRACE("5x7x9");
        check_tile_dot(5, 7, 9, 0xD07A5ull);
    }
    {
        SCOPED_TRACE("32^3");
        check_tile_dot(32, 32, 32, 0xD0732ull);
    }
    {
        SCOPED_TRACE("61x45x43");
        check_tile_dot(61, 45, 43, 0xD0761ull);
    }
}

namespace {

// airflow_pcg_scalars' three quotients from one set of scalars, against
// pcg.cpp's expressions under the step's flush.
struct Quotients {
    float alpha, beta, residual;
};

[[nodiscard]] Quotients cpu_quotients(float rz, float dq, float rz_next, float r_max, float b_max) {
    const spade::math::ScopedFlushSubnormals flush;  // the step's mode
    return {rz / dq, rz_next / rz, b_max > 0.0f ? r_max / b_max : 0.0f};
}

[[nodiscard]] float f32(uint32_t b) noexcept { return std::bit_cast<float>(b); }

}  // namespace

// rz / dq, rz_next / rz and r_max / b_max on the device are divss's bits under
// FTZ|DAZ: three pairs the 3060 Ti's own OpFDiv misses (0x3F800800 / 3.0: the
// host 0x3EAAB555, the device's division 0x3EAAB556), one in each quotient;
// b_max = 0 (the residual 0); a subnormal quotient and a subnormal operand
// (flushed); a zero denominator and 0 / 0; and 400 fixed-seed pairs.
// Mutation: a bare `/`, red here and an E1 finding in AirflowRunnerSpirv.
TEST(GpuAirflowSolids, TheResidualsQuotientIsExact) {
    if (const auto why = spade::testing::vulkan_skip_reason()) GTEST_SKIP() << *why;
    auto ctx = make_context();
    ASSERT_NE(ctx, nullptr);
    const bool bit_equal = spade::testing::airflow_bit_equal_device(*ctx);
    const af::GridShape shape{4, 4, 4, 1.0f};
    auto dev = Device::create(*ctx, shape, nullptr);
    ASSERT_TRUE(dev.has_value());
    const std::vector<ChainCommand> chain{ag::scalars_dispatch(kPipelines, ag::ScalarsOp::alpha),
                                          ag::scalars_dispatch(kPipelines, ag::ScalarsOp::beta),
                                          ag::scalars_dispatch(kPipelines, ag::ScalarsOp::residual)};
    struct Round {
        float rz, dq, rz_next, r_max, b_max;
    };
    const float miss = f32(0x3F800800u);
    std::vector<Round> rounds{
        {miss, 3.0f, 1.0f, 1.0f, 2.0f},            // alpha: the named pair
        {3.0f, 7.0f, miss, 0.5f, 3.0f},            // beta
        {1.0f, 5.0f, 2.0f, miss, 3.0f},            // the residual
        {-miss, 3.0f, -miss, miss, 3.0f},          // signs
        {2.0f, 3.0f, 1.0f, 0.25f, 0.0f},           // b_max = 0: the residual is 0
        {1.0e-30f, 1.0e10f, 1.0e-40f, 1.0e-30f, 1.0e9f},  // a subnormal quotient; a subnormal operand
        {1.0f, 0.0f, 0.0f, 1.0f, 1.0e-40f},        // x / 0; 0 / 1; a subnormal b_max reads as 0
        {0.0f, 0.0f, 1.0f, 0.0f, 1.0f},            // 0 / 0
    };
    SplitMix rng{0x0D1Full};
    for (int n = 0; n < 400; ++n) {
        const auto any = [&] { return mixed_magnitude(rng.next()); };
        const float rz = any();
        const float dq = any();
        const float rz_next = any();
        rounds.push_back({rz, dq, rz_next, std::fabs(any()), std::fabs(any())});
    }
    std::size_t differ = 0;
    std::size_t named = 0;
    for (std::size_t i = 0; i < rounds.size(); ++i) {
        const Round& q = rounds[i];
        std::vector<float> scalars(ag::kScalarSlots, kScalarSentinel);
        scalars[ag::kSlotRz] = q.rz;
        scalars[ag::kSlotDq] = q.dq;
        scalars[ag::kSlotRzNext] = q.rz_next;
        scalars[ag::kSlotRMax] = q.r_max;
        scalars[ag::kSlotBMax] = q.b_max;
        ASSERT_TRUE(dev->upload(kScalars, scalars));
        ASSERT_TRUE(dev->run(chain));
        std::vector<float> gpu(ag::kScalarSlots);
        ASSERT_TRUE(dev->read(kScalars, gpu));
        const Quotients cpu = cpu_quotients(q.rz, q.dq, q.rz_next, q.r_max, q.b_max);
        const std::size_t here = (bits(gpu[ag::kSlotAlpha]) != bits(cpu.alpha) ? 1u : 0u) +
                                 (bits(gpu[ag::kSlotBeta]) != bits(cpu.beta) ? 1u : 0u) +
                                 (bits(gpu[ag::kSlotResidual]) != bits(cpu.residual) ? 1u : 0u);
        differ += here;
        if (i < 3) named += here;
        if (i < 8 || here != 0u) {
            std::printf("[GpuAirflowSolids] round %zu: alpha %s (cpu %s), beta %s (cpu %s), residual %s (cpu %s)\n", i,
                        hex(gpu[ag::kSlotAlpha]).c_str(), hex(cpu.alpha).c_str(), hex(gpu[ag::kSlotBeta]).c_str(),
                        hex(cpu.beta).c_str(), hex(gpu[ag::kSlotResidual]).c_str(), hex(cpu.residual).c_str());
        }
        // The inputs are left as they were.
        for (const uint32_t s : {ag::kSlotRz, ag::kSlotDq, ag::kSlotRzNext, ag::kSlotRMax, ag::kSlotBMax}) {
            EXPECT_EQ(bits(gpu[s]), bits(scalars[s])) << "round " << i << ", slot " << s;
        }
    }
    EXPECT_EQ(bits(cpu_quotients(miss, 3.0f, 0.0f, 0.0f, 0.0f).alpha), 0x3EAAB555u) << "the named pair's host bits";
    expect_none_differ(bit_equal, named, 9u, "the three named pairs' quotients");
    expect_none_differ(bit_equal, differ, 3u * rounds.size(), "the quotients");
}

namespace {

// pcg.cpp's measure() loop (:50-56), spelled out: over the cells not marked
// solid, std::max of |b - q| and of |b| from 0.
struct MaxNorm {
    float r_max = 0.0f;
    float b_max = 0.0f;
};

[[nodiscard]] MaxNorm cpu_max_norm(const std::vector<float>& b, const std::vector<float>& q,
                                   const std::vector<uint8_t>& solid) {
    const spade::math::ScopedFlushSubnormals flush;  // the step's mode
    MaxNorm m;
    for (std::size_t i = 0; i < b.size(); ++i) {
        if (solid[i] != 0u) continue;
        m.r_max = std::max(m.r_max, std::fabs(b[i] - q[i]));
        m.b_max = std::max(m.b_max, std::fabs(b[i]));
    }
    return m;
}

}  // namespace

// The max-norm residual from b and A x: the CPU's own measure (solve_pcg_to()
// at a floor and cap of 0 measures its x and stops) against
// airflow_pcg_measure and the residual quotient, with a NaN planted in x at a
// fluid cell (so A x, and r = b - A x, are NaN around it), and the largest
// |b| of all planted at a solid cell (masked: it must not win). The same bits
// at workgroup sizes 32, 64 and 128 and with blocks of 256 and 64 tiles: the
// max is order-free. Mutation: the device's max() in place of the select.
TEST(GpuAirflowSolids, TheMaxNormIsOrderFree) {
    if (const auto why = spade::testing::vulkan_skip_reason()) GTEST_SKIP() << *why;
    auto ctx = make_context();
    ASSERT_NE(ctx, nullptr);
    const bool bit_equal = spade::testing::airflow_bit_equal_device(*ctx);
    for (const SolidsCase& c : {s1_sphere(), held_ring()}) {
        SCOPED_TRACE(c.name);
        auto built = build(c);
        ASSERT_TRUE(built.has_value());
        Built& b = *built;
        const af::GridShape& s = b.shape;
        const uint32_t cells = cells_of(s);
        const std::vector<uint8_t>& solid = b.s.regions.solid;
        af::PcgScratch pcg = af::make_pcg_scratch(cells);
        fill_noise(pcg.b, 0xB0ull, 2.0f);
        fill_noise(pcg.x, 0xB1ull, 1.0f);
        // A NaN in x at a fluid cell, and the largest |b| at a solid one.
        uint32_t fluid = cells / 2u + 3u;
        while (fluid < cells && solid[fluid] != 0u) ++fluid;
        uint32_t inside = 0;
        while (inside < cells && solid[inside] == 0u) ++inside;
        ASSERT_LT(fluid, cells) << "no fluid cell past the middle";
        ASSERT_LT(inside, cells) << "the case has no solid cell";
        pcg.x[fluid] = std::numeric_limits<float>::quiet_NaN();
        pcg.b[inside] = 1.0e6f;
        af::PcgResult measured{};
        {
            const spade::math::ScopedFlushSubnormals flush;  // the step's mode
            measured =
                af::solve_pcg_to(b.s.projection.mg, pcg, 0, 0, 0.0f, af::PcgGauge{nullptr, nullptr, solid.data()});
        }
        ASSERT_EQ(measured.iterations, 0u);
        const MaxNorm cpu = cpu_max_norm(pcg.b, pcg.q, solid);
        ASSERT_TRUE(std::isnan(pcg.q[fluid])) << "the planted NaN did not reach r";
        ASSERT_LT(cpu.b_max, 1.0e6f) << "the solid cell was not masked";
        {
            const spade::math::ScopedFlushSubnormals flush;
            EXPECT_EQ(bits(cpu.b_max > 0.0f ? cpu.r_max / cpu.b_max : 0.0f), bits(measured.residual))
                << "the spelled-out max is not pcg.cpp's";
        }
        std::printf("[GpuAirflowSolids] %s: cpu r_max %s, b_max %s, residual %s\n", c.name.c_str(),
                    hex(cpu.r_max).c_str(), hex(cpu.b_max).c_str(), hex(measured.residual).c_str());

        std::optional<std::array<uint32_t, 3>> first;
        for (const uint32_t wg : spade::compute::kSupportedWorkgroupSizes) {
            for (const uint32_t block : {af::kReduceBlockTiles, 64u}) {
                SCOPED_TRACE("workgroup size " + std::to_string(wg) + ", block " + std::to_string(block));
                auto dev = Device::create(*ctx, s, &b, wg);
                ASSERT_TRUE(dev.has_value());
                PcgVectors v(cells);
                v.set(ag::kPcgB, pcg.b);
                v.set(ag::kPcgQ, pcg.q);
                ASSERT_TRUE(dev->upload(kPcg, v.all));
                ASSERT_TRUE(dev->upload(kScalars, std::vector<float>(ag::kScalarSlots, kScalarSentinel)));
                std::vector<ChainCommand> chain;
                ag::append_measure(chain, s, kPipelines, wg, block);
                chain.push_back(ag::scalars_dispatch(kPipelines, ag::ScalarsOp::residual));
                ASSERT_TRUE(dev->run(chain));
                std::vector<float> scalars(ag::kScalarSlots);
                ASSERT_TRUE(dev->read(kScalars, scalars));
                const std::array<uint32_t, 3> got{bits(scalars[ag::kSlotRMax]), bits(scalars[ag::kSlotBMax]),
                                                  bits(scalars[ag::kSlotResidual])};
                const std::size_t n = (got[0] != bits(cpu.r_max) ? 1u : 0u) + (got[1] != bits(cpu.b_max) ? 1u : 0u) +
                                      (got[2] != bits(measured.residual) ? 1u : 0u);
                std::printf("[GpuAirflowSolids] %s wg %u block %u: gpu r_max %s, b_max %s, residual %s\n",
                            c.name.c_str(), wg, block, hex(scalars[ag::kSlotRMax]).c_str(),
                            hex(scalars[ag::kSlotBMax]).c_str(), hex(scalars[ag::kSlotResidual]).c_str());
                expect_none_differ(bit_equal, n, 3u, "r_max, b_max and the residual");
                if (!first) first = got;
                EXPECT_EQ(got, *first) << "the max is not order-free";
            }
        }
    }
}

// ===========================================================================
// GpuAirflowSolids -- B5.2: the divergence, the gradient, the operator
// ===========================================================================

// The aperture divergence and the gradient subtracted with apertures, from
// the same noisy u* and p, against projection.cpp's solid_divergence() and
// subtract_gradient_with_apertures(), on the three solids cases: adds,
// subtracts and multiplies only, so bit for bit. Mutations: (1 - t) dropped
// from the flux; a closed face subtracting the gradient.
TEST(GpuAirflowSolids, DivergenceAndGradientWithAperturesMatchTheCpuBitForBit) {
    if (const auto why = spade::testing::vulkan_skip_reason()) GTEST_SKIP() << *why;
    auto ctx = make_context();
    ASSERT_NE(ctx, nullptr);
    const bool bit_equal = spade::testing::airflow_bit_equal_device(*ctx);
    for (const SolidsCase& c : solids_cases()) {
        SCOPED_TRACE(c.name);
        auto built = build(c);
        ASSERT_TRUE(built.has_value());
        const Built& b = *built;
        const af::GridShape& s = b.shape;
        const uint32_t cells = cells_of(s);
        const af::SolidField& f = b.s.solids;
        // The case exercises each branch: closed faces with a moving wall,
        // partial apertures, and (but on the sheet) solid cells.
        std::size_t closed = 0;
        std::size_t partial = 0;
        std::size_t moving = 0;
        for (uint32_t k = 0; k < 3; ++k) {
            for (std::size_t i = 0; i < f.theta[k].size(); ++i) {
                closed += af::closed(f.theta[k][i]) ? 1u : 0u;
                partial += f.theta[k][i] > 0.0f && f.theta[k][i] < 1.0f ? 1u : 0u;
                moving += f.theta[k][i] > 0.0f && f.wall[k][i] != 0.0f ? 1u : 0u;
            }
        }
        const auto solid_cells = static_cast<std::size_t>(
            std::count(b.s.regions.label.begin(), b.s.regions.label.end(), af::kNoRegion));
        std::printf("[GpuAirflowSolids] %s: %zu closed faces, %zu partial, %zu open with a wall velocity, %zu solid "
                    "cells, %u closed regions\n",
                    c.name.c_str(), closed, partial, moving, solid_cells, b.closed_regions);
        EXPECT_GT(closed, 0u);
        EXPECT_GT(moving, 0u);

        auto dev = Device::create(*ctx, s, &b);
        ASSERT_TRUE(dev.has_value());
        const af::MacGrid g = noisy_grid(s, b.bc, 0xD1Full);
        const af::ProjectionFloats floats = af::projection_floats(s, kRho, kStepS);

        // The divergence, into airflow_pcg's b (every other vector untouched).
        std::vector<float> cpu_b(cells);
        {
            const spade::math::ScopedFlushSubnormals flush;  // the step's mode
            af::solid_divergence(g, f, b.s.regions, floats, cpu_b);
        }
        ASSERT_TRUE(dev->upload_grid(g));
        PcgVectors pcg(cells);
        ASSERT_TRUE(dev->upload(kPcg, pcg.all));
        ASSERT_TRUE(dev->run({dev->dispatch(kSolidDivergencePipeline, {.count = cells})}));
        PcgVectors gpu(cells);
        ASSERT_TRUE(dev->read(kPcg, gpu.all));
        expect_none_differ(bit_equal, differing(gpu.copy(ag::kPcgB), cpu_b), cells, c.name + ": the divergence");
        std::size_t others = 0;
        for (std::size_t i = 0; i < gpu.all.size(); ++i) {
            if (i / cells != ag::kPcgB) others += std::isnan(gpu.all[i]) ? 0u : 1u;
        }
        EXPECT_EQ(others, 0u) << "a vector other than b was written";

        // The gradient, from the same u* and the noisy p.
        af::MacGrid cpu = g;
        {
            const spade::math::ScopedFlushSubnormals flush;
            af::subtract_gradient_with_apertures(cpu, af::face_kinds(b.bc), floats.g_coef, f);
        }
        ASSERT_TRUE(dev->upload_grid(g));
        const auto faces = faces_of(s);
        std::vector<ChainCommand> chain;
        for (uint32_t k = 0; k < 3; ++k) {
            chain.push_back(dev->dispatch(kSolidGradientPipeline, {.a = k, .count = faces[k]}));
        }
        ASSERT_TRUE(dev->run(chain));
        af::MacGrid out = g;
        ASSERT_TRUE(dev->read_grid(out));
        expect_none_differ(bit_equal, differing(out.u, cpu.u), out.u.size(), c.name + ": the gradient, u");
        expect_none_differ(bit_equal, differing(out.v, cpu.v), out.v.size(), c.name + ": the gradient, v");
        expect_none_differ(bit_equal, differing(out.w, cpu.w), out.w.size(), c.name + ": the gradient, w");
        EXPECT_EQ(differing(out.p, g.p), 0u) << "p was written";
    }
}

// The CG's operator, q = A v over level 0's weighted stencil, for v = x and v
// = d (fixed-seed noise), against apply_operator() on the three solids cases.
// Mutation: level 0 unweighted (plan A's stencil).
TEST(GpuAirflowSolids, TheOperatorMatchesApplyOperatorBitForBit) {
    if (const auto why = spade::testing::vulkan_skip_reason()) GTEST_SKIP() << *why;
    auto ctx = make_context();
    ASSERT_NE(ctx, nullptr);
    const bool bit_equal = spade::testing::airflow_bit_equal_device(*ctx);
    for (const SolidsCase& c : solids_cases()) {
        SCOPED_TRACE(c.name);
        auto built = build(c);
        ASSERT_TRUE(built.has_value());
        const Built& b = *built;
        const uint32_t cells = cells_of(b.shape);
        const af::PoissonLevel& L0 = b.s.projection.mg.levels[0];
        ASSERT_FALSE(L0.w[0].empty()) << "level 0 is not weighted";
        ASSERT_EQ(b.layout.levels[0][9], 1u);
        auto dev = Device::create(*ctx, b.shape, &b);
        ASSERT_TRUE(dev.has_value());
        PcgVectors pcg(cells);
        std::vector<float> x(cells);
        std::vector<float> d(cells);
        fill_noise(x, 0xA0ull, 1.0f);
        fill_noise(d, 0xA1ull, 3.0f);
        pcg.set(ag::kPcgX, x);
        pcg.set(ag::kPcgD, d);
        for (const ag::PcgVector v : {ag::kPcgX, ag::kPcgD}) {
            const std::vector<float>& in = v == ag::kPcgX ? x : d;
            std::vector<float> cpu(cells);
            {
                const spade::math::ScopedFlushSubnormals flush;  // the step's mode
                af::apply_operator(L0, in, cpu);
            }
            ASSERT_TRUE(dev->upload(kPcg, pcg.all));
            ASSERT_TRUE(dev->run({dev->dispatch(kPcgApplyPipeline, {.a = v, .count = cells})}));
            PcgVectors gpu(cells);
            ASSERT_TRUE(dev->read(kPcg, gpu.all));
            expect_none_differ(bit_equal, differing(gpu.copy(ag::kPcgQ), cpu), cells,
                               c.name + (v == ag::kPcgX ? ": A x" : ": A d"));
            gpu.set(ag::kPcgQ, pcg.copy(ag::kPcgQ));
            EXPECT_EQ(differing(gpu.all, pcg.all), 0u) << "a vector other than q was written";
        }
    }
}

// Every B5 kernel gives the same bits at workgroup sizes 32, 64 and 128, on
// S1's sphere: each invocation's work is fixed by its index, and every
// reduction's order by its tiles and the push's block.
TEST(GpuAirflowSolids, SameBitsAtEveryWorkgroupSize) {
    if (const auto why = spade::testing::vulkan_skip_reason()) GTEST_SKIP() << *why;
    auto ctx = make_context();
    ASSERT_NE(ctx, nullptr);
    auto built = build(s1_sphere());
    ASSERT_TRUE(built.has_value());
    const Built& b = *built;
    const af::GridShape& s = b.shape;
    const uint32_t cells = cells_of(s);
    const af::MacGrid g = noisy_grid(s, b.bc, 0x5A3Eull);
    PcgVectors start(cells);
    for (uint32_t v = 0; v < ag::kPcgVectors; ++v) {
        std::vector<float> x(cells);
        fill_noise(x, 0x5A30ull + v, 2.0f);
        start.set(static_cast<ag::PcgVector>(v), x);
    }
    std::vector<float> scalars(ag::kScalarSlots, kScalarSentinel);
    scalars[ag::kSlotRz] = f32(0x3F800800u);
    scalars[ag::kSlotDq] = 3.0f;
    scalars[ag::kSlotRzNext] = 7.0f;
    const auto faces = faces_of(s);

    // Each kernel's chain at one workgroup size, and every word it leaves in
    // the grid, airflow_pcg and airflow_scalars.
    const auto run = [&](uint32_t wg, const std::vector<ChainCommand>& chain) -> std::optional<std::vector<float>> {
        auto dev = Device::create(*ctx, s, &b, wg);
        if (!dev || !dev->upload_grid(g) || !dev->upload(kPcg, start.all) || !dev->upload(kScalars, scalars) ||
            !dev->run(chain)) {
            return std::nullopt;
        }
        af::MacGrid out = g;
        PcgVectors pcg(cells);
        std::vector<float> sc(ag::kScalarSlots);
        if (!dev->read_grid(out) || !dev->read(kPcg, pcg.all) || !dev->read(kScalars, sc)) return std::nullopt;
        std::vector<float> all = pcg.all;
        for (const std::vector<float>* x : {&out.u, &out.v, &out.w, &out.p, &sc}) {
            all.insert(all.end(), x->begin(), x->end());
        }
        return all;
    };
    const auto chain_for = [&](const std::string& kernel, uint32_t wg) {
        std::vector<ChainCommand> chain;
        const auto d = [&](Pipeline p, const ag::ProjectionPush& push) {
            chain.push_back(ag::projection_dispatch(p, push, wg));
        };
        if (kernel == "airflow_tile_dot") {
            ag::append_tile_dot(chain, s, kPipelines, ag::kPcgD, ag::kPcgQ, ag::kSlotDq, wg);
        }
        if (kernel == "airflow_pcg_measure") ag::append_measure(chain, s, kPipelines, wg, af::kReduceBlockTiles);
        if (kernel == "airflow_pcg_scalars") {
            for (const auto op : {ag::ScalarsOp::alpha, ag::ScalarsOp::beta, ag::ScalarsOp::residual}) {
                chain.push_back(ag::scalars_dispatch(kPipelines, op));
            }
        }
        if (kernel == "airflow_solid_divergence") d(kSolidDivergencePipeline, {.count = cells});
        if (kernel == "airflow_solid_gradient") {
            for (uint32_t k = 0; k < 3; ++k) d(kSolidGradientPipeline, {.a = k, .count = faces[k]});
        }
        if (kernel == "airflow_pcg_apply") d(kPcgApplyPipeline, {.a = ag::kPcgD, .count = cells});
        return chain;
    };
    for (const char* kernel : {"airflow_tile_dot", "airflow_pcg_measure", "airflow_pcg_scalars",
                               "airflow_solid_divergence", "airflow_solid_gradient", "airflow_pcg_apply"}) {
        SCOPED_TRACE(kernel);
        const auto first = run(64, chain_for(kernel, 64));
        ASSERT_TRUE(first.has_value());
        for (const uint32_t wg : {32u, 128u}) {
            const auto other = run(wg, chain_for(kernel, wg));
            ASSERT_TRUE(other.has_value());
            const std::size_t n = differing(*other, *first);
            std::printf("[GpuAirflowSolids] %s at %u: %zu of %zu words differ from 64's\n", kernel, wg, n,
                        first->size());
            EXPECT_EQ(n, 0u) << "at " << wg;
        }
    }
}
