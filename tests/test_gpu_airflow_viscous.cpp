// ---------------------------------------------------------------------------
// test_gpu_airflow_viscous.cpp -- plan B, task B3: the velocity conditions
// (engine/shaders/kernels/airflow_velocity_bc.slang) and the implicit
// viscosity (airflow_viscous.slang) against their CPU twins,
// apply_velocity_boundaries() (engine/physics/airflow/boundaries.cpp) and
// diffuse() (viscous.cpp), on the chain runner (spike/airflow/chain_runner.hpp).
//
//   GpuAirflowViscous   the conditions and the viscous solve bit for bit on
//                       the device of record (airflow_exact_device.hpp; born
//                       exact, so no band was ever measured: another device
//                       or driver only reports), on the walled cavity and the
//                       nine boundary configurations (test_airflow_configs.cpp's
//                       shapes and faces); the viscous solve with solids --
//                       fixed samples and wall links from build_solids() --
//                       on an open and a periodic box; the same bits at
//                       workgroup sizes 32, 64 and 128; with
//                       SPADE_AIRFLOW_TIMINGS=1, the solve's time in the D1 box.
//
// The CPU references run under the step's subnormal flush
// (ScopedFlushSubnormals), as fluid_step() runs them. The fields are fixed-seed
// noise with a few subnormals mixed in, so the flush is exercised too.
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
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <vector>

#include "airflow_exact_device.hpp"
#include "airflow_solid_shapes.hpp"
#include "airflow_velocity_bc.spv.gen.hpp"
#include "airflow_viscous.spv.gen.hpp"
#include "compute/backend.hpp"
#include "compute/spirv_variants.hpp"
#include "compute/vulkan/context.hpp"
#include "compute/vulkan/dispatch_chain.hpp"
#include "core/detail/flush_subnormals.hpp"
#include "gpu_skip.hpp"
#include "physics/airflow/boundaries.hpp"
#include "physics/airflow/mac_grid.hpp"
#include "physics/airflow/solids.hpp"
#include "physics/airflow/viscous.hpp"
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

// Set 1's bindings 0-17: the runner binds buffers[i] at binding i.
enum Buffer : uint32_t {
    kParams = 0,
    kU,
    kV,
    kW,
    kPressure,
    kNext,
    kAdvect,
    kIncrements,
    kViscousRhs,
    kRhs,
    kResidual,
    kMultigrid,
    kMgWeights,
    kApertures,
    kWallVelocity,
    kPhi,
    kFaceTags,
    kWallLinks,
    kBufferCount
};

enum Pipeline : uint32_t { kBcPipeline = 0, kViscousPipeline = 1 };

// The kernels' push blocks (airflow_velocity_bc.slang's VelocityBcPush,
// airflow_viscous.slang's ViscousPush).
struct VelocityBcPush {
    uint32_t axis = 0;
    uint32_t count = 0;
    uint32_t fixed_lo = 0;
    uint32_t fixed_hi = 0;
    uint32_t ambient = 0;
    uint32_t unused[3] = {};
};

struct ViscousPush {
    uint32_t component = 0;
    uint32_t mode = 0;  // 0 prepare, 1 sweep, 2 alias
    uint32_t colour = 0;
    uint32_t count = 0;
    uint32_t alpha = 0;
    uint32_t unused[3] = {};
    uint32_t ghost[6] = {};
};

constexpr uint32_t kPushBytes = sizeof(ViscousPush);
static_assert(sizeof(VelocityBcPush) <= kPushBytes);

constexpr uint32_t kWorkgroupSize = 64;
constexpr uint32_t kSweeps = 8;  // FluidConfig's visc_sweeps
constexpr float kStepS = 0.05f;
constexpr float kNu = 0.02f;

[[nodiscard]] std::unique_ptr<VulkanContext> make_context() {
    auto result = VulkanContext::create(spade::compute::BackendDesc{.kind = spade::compute::BackendKind::vulkan});
    if (!result.has_value()) return nullptr;
    return std::move(*result);
}

[[nodiscard]] uint32_t bits(float f) noexcept { return std::bit_cast<uint32_t>(f); }

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
    uint64_t state;
    uint64_t next() noexcept {
        uint64_t z = (state += 0x9E3779B97F4A7C15ull);
        z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ull;
        z = (z ^ (z >> 27)) * 0x94D049BB133111EBull;
        return z ^ (z >> 31);
    }
    // In [-1, 1), from the top 24 bits: exact in a float.
    float unit() noexcept { return static_cast<float>(static_cast<int32_t>(next() >> 40) - (1 << 23)) * 0x1p-23f; }
};

// Noise of the given scale on every face, with one face in 61 a subnormal of
// either sign, which the flush must treat alike on both sides.
[[nodiscard]] af::MacGrid noisy(const af::GridShape& s, uint64_t seed, float scale) {
    af::MacGrid g = *af::make_grid(s, af::DomainBc{});
    SplitMix rng{seed};
    std::size_t n = 0;
    for (std::vector<float>* c : {&g.u, &g.v, &g.w}) {
        for (float& x : *c) {
            x = (++n % 61u == 0u) ? (rng.unit() < 0.0f ? -1.0e-40f : 3.0e-41f) : scale * rng.unit();
        }
    }
    return g;
}

[[nodiscard]] std::array<std::size_t, 3> face_counts(const af::GridShape& s) {
    return {std::size_t{s.nx + 1u} * s.ny * s.nz, std::size_t{s.nx} * (s.ny + 1u) * s.nz,
            std::size_t{s.nx} * s.ny * (s.nz + 1u)};
}

// airflow_params: dims, grid {dx, 1/dx, h}, periodic, the flags and the face
// kinds (airflow_params_layout.slang); the rest unread here.
[[nodiscard]] std::vector<uint32_t> params_for(const af::GridShape& s, const af::DomainBc& bc, bool solids) {
    std::vector<uint32_t> p(32, 0u);
    p[0] = s.nx;
    p[1] = s.ny;
    p[2] = s.nz;
    p[4] = bits(s.dx);
    p[5] = bits(1.0f / s.dx);
    p[6] = bits(kStepS);
    const std::array<bool, 3> periodic = af::periodic_axes(bc);
    for (uint32_t a = 0; a < 3; ++a) p[8 + a] = periodic[a] ? 1u : 0u;
    p[16] = solids ? 3u : 0u;  // kParamsFlagSolids | kParamsFlagTheta
    const std::array<af::FaceKind, 6> kinds = af::face_kinds(bc);
    for (uint32_t side = 0; side < 6; ++side) p[18 + side] = static_cast<uint32_t>(kinds[side]);
    return p;
}

// TEST-LOCAL PACKER, deleted when B5's lands. B5 (Physics) owns the host
// packing of a SolidField into bindings 13-17, one home under TD-9 (the
// lead's ruling, 2026-10-22); this copy of its rule, confirmed by Physics the
// same day, lives in this file only until then. Per face, u then v then w:
// theta, the wall velocity and phi as floats; face_tags the wall-link index
// (or kNoWallLinks), then the free-slip layer as little-endian bytes from
// word F (face f at word F + f / 4, byte f % 4); a wall link 12 words,
// inv_f[0..5] then wall[0..5].
struct PackedSolids {
    std::vector<float> theta;
    std::vector<float> wall;
    std::vector<float> phi;
    std::vector<uint32_t> tags;
    std::vector<uint32_t> links;
};

[[nodiscard]] PackedSolids pack(const af::SolidField& f) {
    PackedSolids out;
    for (uint32_t c = 0; c < 3; ++c) {
        out.theta.insert(out.theta.end(), f.theta[c].begin(), f.theta[c].end());
        out.wall.insert(out.wall.end(), f.wall[c].begin(), f.wall[c].end());
        out.phi.insert(out.phi.end(), f.phi[c].begin(), f.phi[c].end());
        out.tags.insert(out.tags.end(), f.link[c].begin(), f.link[c].end());
    }
    const std::size_t total = out.tags.size();
    out.tags.resize(total + (total + 3u) / 4u, 0u);
    std::size_t face = 0;
    for (uint32_t c = 0; c < 3; ++c) {
        for (const uint8_t layer : f.layer[c]) {
            out.tags[total + face / 4u] |= uint32_t{layer} << (8u * (face % 4u));
            ++face;
        }
    }
    for (const af::WallLinks& l : f.links) {
        for (const float x : l.inv_f) out.links.push_back(bits(x));
        for (const float x : l.wall) out.links.push_back(bits(x));
    }
    if (out.links.empty()) out.links.push_back(0u);  // bound, never read
    return out;
}

// The velocity conditions: apply_axis() per axis, one invocation per line.
[[nodiscard]] std::vector<ChainCommand> bc_chain(const af::GridShape& s, const af::DomainBc& bc, uint32_t wg) {
    std::vector<ChainCommand> chain;
    const uint32_t lines[3] = {s.ny * s.nz, s.nx * s.nz, s.nx * s.ny};
    for (uint32_t axis = 0; axis < 3; ++axis) {
        const auto lo = static_cast<af::Side>(2u * axis);
        const auto hi = static_cast<af::Side>(2u * axis + 1u);
        VelocityBcPush push{.axis = axis,
                            .count = lines[axis],
                            .fixed_lo = bits(af::fixed_normal_velocity(bc, lo)),
                            .fixed_hi = bits(af::fixed_normal_velocity(bc, hi)),
                            .ambient = bits(bc.ambient[static_cast<glm::length_t>(axis)])};
        ChainCommand cmd{.is_dispatch = true, .pipeline = kBcPipeline};
        std::memcpy(cmd.push.data(), &push, sizeof push);
        cmd.groups = {(push.count + wg - 1u) / wg, 1u, 1u};
        chain.push_back(cmd);
    }
    return chain;
}

// diffuse(): per component, prepare, then colour 0 and 1 `sweeps` times, then
// the periodic alias face; with `marks`, a timestamp either side.
[[nodiscard]] std::vector<ChainCommand> viscous_chain(const af::GridShape& s, const af::DomainBc& bc, float alpha,
                                                      uint32_t sweeps, uint32_t wg, bool marks = false) {
    std::vector<ChainCommand> chain;
    if (marks) chain.push_back(ChainCommand{});
    const std::array<bool, 3> periodic = af::periodic_axes(bc);
    const auto dispatch = [&](const ViscousPush& push) {
        ChainCommand cmd{.is_dispatch = true, .pipeline = kViscousPipeline};
        std::memcpy(cmd.push.data(), &push, sizeof push);
        cmd.groups = {(push.count + wg - 1u) / wg, 1u, 1u};
        chain.push_back(cmd);
    };
    for (uint32_t c = 0; c < 3; ++c) {
        const uint32_t cnt[3] = {s.nx + (c == 0u ? 1u : 0u), s.ny + (c == 1u ? 1u : 0u), s.nz + (c == 2u ? 1u : 0u)};
        ViscousPush base{.component = c, .alpha = bits(alpha)};
        for (uint32_t side = 0; side < 6; ++side) {
            // A wall's own velocity; an inflow's ambient (read only where the
            // side's kind is fixed_velocity).
            base.ghost[side] = bits(bc.face[side] == af::FaceBc::wall
                                        ? bc.wall_velocity[side][static_cast<glm::length_t>(c)]
                                        : bc.ambient[static_cast<glm::length_t>(c)]);
        }
        ViscousPush prepare = base;
        prepare.mode = 0u;
        prepare.count = cnt[0] * cnt[1] * cnt[2];
        dispatch(prepare);
        for (uint32_t sweep = 0; sweep < sweeps; ++sweep) {
            for (const uint32_t colour : {0u, 1u}) {
                ViscousPush relax = base;
                relax.mode = 1u;
                relax.colour = colour;
                relax.count = ((cnt[0] + 1u) / 2u) * cnt[1] * cnt[2];
                dispatch(relax);
            }
        }
        if (periodic[c]) {
            ViscousPush alias = base;
            alias.mode = 2u;
            alias.count = cnt[0] * cnt[1] * cnt[2] / cnt[c];
            dispatch(alias);
        }
    }
    if (marks) chain.push_back(ChainCommand{});
    return chain;
}

// Runs `chain` on the device from `in` (and `solids`, packed), and returns the
// velocity it leaves. With `best_ms`, the chain's time, the fastest of five.
[[nodiscard]] std::optional<af::MacGrid> run_on_gpu(VulkanContext& ctx, const af::MacGrid& in,
                                                    const af::DomainBc& bc, const af::SolidField* solids,
                                                    const std::vector<ChainCommand>& chain, uint32_t wg,
                                                    double* best_ms = nullptr) {
    const auto n = face_counts(in.shape);
    const std::size_t total = n[0] + n[1] + n[2];
    const bool walls = solids != nullptr && solids->any;
    const std::vector<uint32_t> params = params_for(in.shape, bc, walls);
    const PackedSolids packed = walls ? pack(*solids) : PackedSolids{};
    const auto size_of = [](const auto& v) { return std::max<std::size_t>(4u, v.size() * 4u); };
    const RunnerBuffer buffers[kBufferCount] = {
        {"airflow_params", params.size() * 4u, BufferKind::staged},
        {"airflow_u", n[0] * 4u, BufferKind::resident},
        {"airflow_v", n[1] * 4u, BufferKind::resident},
        {"airflow_w", n[2] * 4u, BufferKind::resident},
        // Bound, not read: set 1 is contiguous.
        {"airflow_pressure", 4u, BufferKind::device_only},
        {"airflow_next", 4u, BufferKind::device_only},
        {"airflow_advect", 4u, BufferKind::device_only},
        {"airflow_increments", 4u, BufferKind::device_only},
        {"airflow_viscous_rhs", total * 4u, BufferKind::device_only},
        {"airflow_rhs", 4u, BufferKind::device_only},
        {"airflow_residual", 4u, BufferKind::device_only},
        {"airflow_multigrid", 4u, BufferKind::device_only},
        {"airflow_mg_weights", 4u, BufferKind::device_only},
        {"airflow_apertures", size_of(packed.theta), BufferKind::resident},
        {"airflow_wall_velocity", size_of(packed.wall), BufferKind::resident},
        {"airflow_phi", size_of(packed.phi), BufferKind::resident},
        {"airflow_face_tags", size_of(packed.tags), BufferKind::resident},
        {"airflow_wall_links", size_of(packed.links), BufferKind::resident},
    };
    const SpirvVariantSet* const kernels[] = {&gen::kSpvVariants_airflow_velocity_bc,
                                              &gen::kSpvVariants_airflow_viscous};
    auto runner = ChainRunner::create(
        ctx, RunnerDesc{.buffers = buffers, .kernels = kernels, .push_bytes = kPushBytes, .workgroup_size = wg});
    if (!runner) {
        ADD_FAILURE() << runner.error().context;
        return std::nullopt;
    }
    ChainRunner& r = **runner;
    const auto check = [](const spade::Result<void>& done) {
        if (!done) ADD_FAILURE() << done.error().context;
        return done.has_value();
    };
    const auto upload_state = [&] {
        return check(r.upload(kU, bytes_of(in.u))) && check(r.upload(kV, bytes_of(in.v))) &&
               check(r.upload(kW, bytes_of(in.w)));
    };
    if (!check(r.upload(kParams, bytes_of(params))) || !upload_state()) return std::nullopt;
    if (walls && (!check(r.upload(kApertures, bytes_of(packed.theta))) ||
                  !check(r.upload(kWallVelocity, bytes_of(packed.wall))) ||
                  !check(r.upload(kPhi, bytes_of(packed.phi))) || !check(r.upload(kFaceTags, bytes_of(packed.tags))) ||
                  !check(r.upload(kWallLinks, bytes_of(packed.links))))) {
        return std::nullopt;
    }
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
        if (!upload_state()) return std::nullopt;
    }
    if (!check(r.submit(*id, 1))) return std::nullopt;
    af::MacGrid out = in;
    if (!check(r.read(kU, writable_bytes_of(out.u))) || !check(r.read(kV, writable_bytes_of(out.v))) ||
        !check(r.read(kW, writable_bytes_of(out.w)))) {
        return std::nullopt;
    }
    return out;
}

[[nodiscard]] float alpha_of(const af::GridShape& s) { return kNu * kStepS / (s.dx * s.dx); }  // diffuse()'s

[[nodiscard]] af::MacGrid diffuse_on_cpu(const af::MacGrid& in, const af::DomainBc& bc,
                                         const af::SolidField* solids) {
    af::MacGrid g = in;
    std::vector<float> rhs(std::max({g.u.size(), g.v.size(), g.w.size()}), 0.0f);
    const spade::math::ScopedFlushSubnormals flush;  // the step's mode
    af::diffuse(g, bc, kNu, kStepS, kSweeps, rhs, solids);
    return g;
}

// How many faces differ in any bit, over the three components.
[[nodiscard]] std::size_t differing(const af::MacGrid& gpu, const af::MacGrid& cpu) {
    std::size_t out = 0;
    const std::vector<float>* const g[3] = {&gpu.u, &gpu.v, &gpu.w};
    const std::vector<float>* const c[3] = {&cpu.u, &cpu.v, &cpu.w};
    for (uint32_t k = 0; k < 3; ++k) {
        for (std::size_t i = 0; i < c[k]->size(); ++i) out += bits((*g[k])[i]) != bits((*c[k])[i]) ? 1u : 0u;
    }
    return out;
}

[[nodiscard]] std::size_t faces_of(const af::MacGrid& g) { return g.u.size() + g.v.size() + g.w.size(); }

struct Config {
    const char* name;
    af::GridShape shape;
    af::DomainBc bc;
};

[[nodiscard]] af::DomainBc all(af::FaceBc kind) {
    af::DomainBc bc;
    bc.face.fill(kind);
    return bc;
}

// The walled cavity and test_airflow_configs.cpp's nine configurations (its
// shapes and faces; the viscosity and step are this file's).
[[nodiscard]] std::vector<Config> configs() {
    std::vector<Config> out;
    {
        af::DomainBc bc = all(af::FaceBc::wall);
        bc.wall_velocity[af::kYPlus] = glm::vec3(1.0f, 0.0f, 0.0f);
        out.push_back({"walled cavity 16^3, lid", {16, 16, 16, 1.0f / 16.0f}, bc});
    }
    out.push_back({"periodic 16^3", {16, 16, 16, 0.1f}, all(af::FaceBc::periodic)});
    out.push_back({"periodic 12x8x8", {12, 8, 8, 0.1f}, all(af::FaceBc::periodic)});
    {
        af::DomainBc bc = all(af::FaceBc::wall);
        bc.wall_velocity[af::kYPlus] = glm::vec3(1.0f, 0.0f, 0.2f);
        bc.wall_velocity[af::kXMinus] = glm::vec3(0.0f, -0.3f, 0.1f);
        out.push_back({"walls, two moving, 10x6x8", {10, 6, 8, 0.1f}, bc});
    }
    {
        af::DomainBc bc;
        bc.face = {af::FaceBc::wall, af::FaceBc::wall,     af::FaceBc::wall,
                   af::FaceBc::wall, af::FaceBc::periodic, af::FaceBc::periodic};
        bc.wall_velocity[af::kYPlus] = glm::vec3(1.0f, 0.0f, 0.0f);
        out.push_back({"cavity slab 16x16x1", {16, 16, 1, 1.0f / 16.0f}, bc});
    }
    out.push_back({"open 9x7x5 (odd sizes)", {9, 7, 5, 0.1f}, af::DomainBc{}});
    out.push_back({"open 16x12x16", {16, 12, 16, 0.05f}, af::DomainBc{}});
    {
        af::DomainBc bc;
        bc.ambient = glm::vec3(0.8f, -0.2f, 0.3f);
        out.push_back({"wind 12x10x8", {12, 10, 8, 0.1f}, bc});
    }
    {
        af::DomainBc bc;
        bc.ambient = glm::vec3(1.0f, 0.0f, 0.0f);
        bc.face[af::kYMinus] = af::FaceBc::wall;
        bc.wall_velocity[af::kYMinus] = glm::vec3(1.0f, 0.0f, 0.0f);
        bc.face[af::kZMinus] = af::FaceBc::periodic;
        bc.face[af::kZPlus] = af::FaceBc::periodic;
        out.push_back({"floor under wind 12x8x8", {12, 8, 8, 0.1f}, bc});
    }
    {
        af::DomainBc bc;
        bc.face = {af::FaceBc::periodic, af::FaceBc::periodic, af::FaceBc::wall,
                   af::FaceBc::open,     af::FaceBc::wall,     af::FaceBc::wall};
        out.push_back({"one-cell x 1x8x8", {1, 8, 8, 0.1f}, bc});
    }
    return out;
}

// Solids: a sphere spinning about z (a moving wall) and a box turned about z,
// in a wind, from build_solids() (fixed samples and wall links).
[[nodiscard]] af::SolidField solids_for(const af::GridShape& s, const af::DomainBc& bc) {
    airflow_test::Shape sphere;
    sphere.name = "sphere";
    sphere.kind = airflow_test::Kind::sphere;
    sphere.centre = glm::vec3(0.55f, 0.62f, 0.5f);
    sphere.radius = 0.23f;
    sphere.omega = glm::vec3(0.0f, 0.0f, 3.0f);
    airflow_test::Shape box;
    box.name = "box";
    box.kind = airflow_test::Kind::box;
    box.centre = glm::vec3(1.12f, 0.55f, 0.48f);
    box.half = glm::vec3(0.16f, 0.21f, 0.13f);
    const float ca = std::cos(0.4f);
    const float sa = std::sin(0.4f);
    box.to_local = glm::mat3(ca, -sa, 0.0f, sa, ca, 0.0f, 0.0f, 0.0f, 1.0f);
    const airflow_test::Scene scene{{sphere, box}};
    af::SolidField f = *af::make_solid_field(s);
    const std::vector<af::SolidPart> parts = scene.parts();
    const spade::Result<void> ok = af::build_solids(f, s, af::periodic_axes(bc), parts, scene.sampler());
    EXPECT_TRUE(ok.has_value()) << (ok ? "" : ok.error().context);
    return f;
}

[[nodiscard]] std::vector<Config> solid_boxes() {
    af::DomainBc wind;
    wind.ambient = glm::vec3(0.6f, 0.0f, 0.1f);
    af::DomainBc ring = wind;
    ring.face[af::kZMinus] = ring.face[af::kZPlus] = af::FaceBc::periodic;
    return {{"solids, wind 24x18x16", {24, 18, 16, 1.0f / 16.0f}, wind},
            {"solids, periodic z 24x18x16", {24, 18, 16, 1.0f / 16.0f}, ring}};
}

}  // namespace

// The velocity conditions on every configuration: apply_velocity_boundaries()
// to the bit, under the step's flush (DAZ: the outlets' sign tests see a
// subnormal as zero, as the device's do). Copies and selects only.
TEST(GpuAirflowViscous, VelocityBoundariesMatchTheCpuBitForBit) {
    if (const auto why = spade::testing::vulkan_skip_reason()) GTEST_SKIP() << *why;
    auto ctx = make_context();
    ASSERT_NE(ctx, nullptr);
    for (const Config& c : configs()) {
        SCOPED_TRACE(c.name);
        const af::MacGrid start = noisy(c.shape, 0xBCull, 1.0f);
        af::MacGrid cpu = start;
        {
            const spade::math::ScopedFlushSubnormals flush;  // the step's mode
            af::apply_velocity_boundaries(cpu, c.bc);
        }
        const auto gpu = run_on_gpu(*ctx, start, c.bc, nullptr, bc_chain(c.shape, c.bc, kWorkgroupSize),
                                    kWorkgroupSize);
        ASSERT_TRUE(gpu.has_value());
        const std::size_t d = differing(*gpu, cpu);
        std::printf("[ viscous ] %-28s conditions: %zu of %zu faces differ\n", c.name, d, faces_of(cpu));
        EXPECT_EQ(d, 0u);
    }
}

// diffuse() on every configuration, eight sweeps per component: to the bit on
// the device of record (the division is div32_ftz); elsewhere reported only.
TEST(GpuAirflowViscous, DiffuseMatchesTheCpuBitForBit) {
    if (const auto why = spade::testing::vulkan_skip_reason()) GTEST_SKIP() << *why;
    auto ctx = make_context();
    ASSERT_NE(ctx, nullptr);
    const bool bit_equal = spade::testing::airflow_bit_equal_device(*ctx);
    for (const Config& c : configs()) {
        SCOPED_TRACE(c.name);
        const af::MacGrid start = noisy(c.shape, 0xD1FFull, 1.0f);
        const af::MacGrid cpu = diffuse_on_cpu(start, c.bc, nullptr);
        ASSERT_GT(differing(cpu, start), 0u) << "diffuse() changed nothing";
        const auto gpu = run_on_gpu(*ctx, start, c.bc, nullptr,
                                    viscous_chain(c.shape, c.bc, alpha_of(c.shape), kSweeps, kWorkgroupSize),
                                    kWorkgroupSize);
        ASSERT_TRUE(gpu.has_value());
        const std::size_t d = differing(*gpu, cpu);
        std::printf("[ viscous ] %-28s diffuse: %zu of %zu faces differ\n", c.name, d, faces_of(cpu));
        if (bit_equal) {
            EXPECT_EQ(d, 0u);
        }
    }
}

// diffuse() with solids: the fixed samples take their wall velocity and the
// wall links replace their neighbours, on an open box and a periodic one.
TEST(GpuAirflowViscous, DiffuseWithSolidsMatchesTheCpuBitForBit) {
    if (const auto why = spade::testing::vulkan_skip_reason()) GTEST_SKIP() << *why;
    auto ctx = make_context();
    ASSERT_NE(ctx, nullptr);
    const bool bit_equal = spade::testing::airflow_bit_equal_device(*ctx);
    for (const Config& c : solid_boxes()) {
        SCOPED_TRACE(c.name);
        const af::SolidField f = solids_for(c.shape, c.bc);
        ASSERT_TRUE(f.any);
        std::size_t fixed = 0;
        std::size_t linked = 0;
        for (uint32_t k = 0; k < 3; ++k) {
            for (std::size_t i = 0; i < f.theta[k].size(); ++i) {
                fixed += (af::closed(f.theta[k][i]) || f.phi[k][i] < 0.0f) ? 1u : 0u;
                linked += f.link[k][i] != af::kNoWallLinks ? 1u : 0u;
            }
        }
        ASSERT_GT(fixed, 0u);
        ASSERT_GT(linked, 0u);
        const af::MacGrid start = noisy(c.shape, 0x501Dull, 1.0f);
        const af::MacGrid cpu = diffuse_on_cpu(start, c.bc, &f);
        const auto gpu = run_on_gpu(*ctx, start, c.bc, &f,
                                    viscous_chain(c.shape, c.bc, alpha_of(c.shape), kSweeps, kWorkgroupSize),
                                    kWorkgroupSize);
        ASSERT_TRUE(gpu.has_value());
        const std::size_t d = differing(*gpu, cpu);
        std::printf("[ viscous ] %-28s %zu fixed, %zu linked of %zu faces (%zu links); %zu differ\n", c.name, fixed,
                    linked, faces_of(cpu), f.links.size(), d);
        if (bit_equal) {
            EXPECT_EQ(d, 0u);
        }
    }
}

// No reduction and nothing shared between invocations: the solve with solids
// gives the same bits at every supported workgroup size.
TEST(GpuAirflowViscous, SameBitsAtEveryWorkgroupSize) {
    if (const auto why = spade::testing::vulkan_skip_reason()) GTEST_SKIP() << *why;
    auto ctx = make_context();
    ASSERT_NE(ctx, nullptr);
    const Config c = solid_boxes()[1];
    const af::SolidField f = solids_for(c.shape, c.bc);
    const af::MacGrid start = noisy(c.shape, 0x3264128ull, 1.0f);
    std::optional<af::MacGrid> first;
    for (const uint32_t wg : spade::compute::kSupportedWorkgroupSizes) {
        SCOPED_TRACE("workgroup size " + std::to_string(wg));
        auto gpu = run_on_gpu(*ctx, start, c.bc, &f, viscous_chain(c.shape, c.bc, alpha_of(c.shape), kSweeps, wg),
                              wg);
        ASSERT_TRUE(gpu.has_value());
        if (!first) {
            first = std::move(gpu);
            continue;
        }
        EXPECT_EQ(differing(*gpu, *first), 0u);
    }
}

// The viscous solve's time in the D1 box (240 x 177 x 168 at 4 mm, the air's
// nu, h 0.38 ms), eight sweeps per component, the fastest of five. Opt-in with
// SPADE_AIRFLOW_TIMINGS, as the projection's and the gather's D1 times are.
TEST(GpuAirflowViscous, DiffuseTimeInTheD1Box) {
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
    af::DomainBc bc;
    bc.ambient = glm::vec3(-3.0f, 0.0f, 0.0f);
    const float alpha = 1.4607e-5f * 3.8e-4f / (kD1.dx * kD1.dx);
    const af::MacGrid start = noisy(kD1, 0xD1ull, 3.0f);
    double ms = 0.0;
    ASSERT_TRUE(run_on_gpu(*ctx, start, bc, nullptr, viscous_chain(kD1, bc, alpha, kSweeps, kWorkgroupSize, true),
                           kWorkgroupSize, &ms)
                    .has_value());
    std::printf("measure: diffuse in the D1 box, 240x177x168 at 4 mm, %u sweeps per component: %.3f ms\n", kSweeps,
                ms);
}
