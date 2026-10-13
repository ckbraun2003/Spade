// The airflow solver core's second golden: one digest per boundary
// configuration (the lead, 2026-10-12, after the CPU-performance task). The
// first golden (test_airflow_step.cpp) is one walled cavity; these nine pin
// every face kind and the paths the cavity never takes -- periodic wraps,
// moving walls on two faces, a quasi-2D slab, open faces in still air and
// under a wind (inflow), a floor under a wind, odd sizes the multigrid cannot
// halve, a one-cell axis, and the actuator source (only discs check_disc
// accepts) -- so any later change to the CPU reference (performance work,
// plan B's GPU twin banding against the same configurations) shows exactly
// which configuration moved.
//
// Each run: random velocities from a fixed LCG, six fluid steps, then the
// render-volume publish with a solid mask (every 7th cell) and the grid's
// periodic flags. The digest folds u, v, w and p, then the published V and S
// textures, by FNV-1a.
//
// THE GOLDENS (TD-1, TD-12).
//   generated  2026-10-12, MSVC release (VS 2022), physics/airflow-core: the
//              same nine values before the CPU-performance task (a20a847) and
//              after it (05099fa), which is how that task proved itself
//              bit-identical
//   cross-check MSVC debug agrees
//   gcc        the gcc-13 Docker leg (TD-12) reproduces all nine on a fresh -NoSeed
//              volume at 4622f84 (build-docker/4622f84a242b, 2026-10-12): FINAL
//   moved      before the gcc leg, by the final review's fixes (2026-10-12):
//              I2, periodic axes never coarsened to an odd size ("periodic 12x8x6");
//              M3, the solid layer's fluid neighbour across a periodic wrap
//              ("periodic 16^3" and "periodic 12x8x6"; the floor case's z-wrap
//              neighbours are all solid under the every-7th mask);
//              I3, only discs check_disc accepts: "periodic 12x8x6, disc" became
//              "periodic 12x8x8, disc" with R = 2 dx, and the floor case's disc
//              R = 2 dx (R = 3 dx left the y axis, silently dropping momentum)
// A change to any value here is a golden regeneration under TD-1, with its
// reason in the commit.
#include <gtest/gtest.h>

#include <cstdint>
#include <cstdio>
#include <string>
#include <vector>

#include "core/flow_volume.hpp"
#include "physics/airflow/actuator.hpp"
#include "physics/airflow/fluid_step.hpp"
#include "physics/airflow/publish.hpp"

namespace {

using namespace spade::physics::airflow;

struct Lcg {
    uint32_t state = 12345u;
    float next() noexcept {
        state = state * 1664525u + 1013904223u;
        return static_cast<float>(state >> 8) * (2.0f / 16777216.0f) - 1.0f;
    }
};

[[nodiscard]] uint64_t fold(uint64_t h, const void* data, std::size_t bytes) noexcept {
    const auto* b = static_cast<const unsigned char*>(data);
    for (std::size_t i = 0; i < bytes; ++i) {
        h ^= b[i];
        h *= 1099511628211ull;
    }
    return h;
}

struct Config {
    const char* name;
    GridShape shape;
    DomainBc bc;
    float nu;
    float disc_radius_cells;  // 0: no actuator disc; else its radius in cells, checked by check_disc
    uint64_t digest;
};

[[nodiscard]] DomainBc all(FaceBc kind) {
    DomainBc bc;
    bc.face.fill(kind);
    return bc;
}

[[nodiscard]] std::vector<Config> configs() {
    std::vector<Config> out;
    out.push_back({"periodic 16^3", {16, 16, 16, 0.1f}, all(FaceBc::periodic), 0.01f, 0.0f, 0x28b57f5d7ad1d200ull});
    out.push_back(
        {"periodic 12x8x8, disc", {12, 8, 8, 0.1f}, all(FaceBc::periodic), 0.0f, 2.0f, 0xde265578accd7f72ull});
    {
        DomainBc bc = all(FaceBc::wall);
        bc.wall_velocity[kYPlus] = glm::vec3(1.0f, 0.0f, 0.2f);
        bc.wall_velocity[kXMinus] = glm::vec3(0.0f, -0.3f, 0.1f);
        out.push_back({"walls, two moving, 10x6x8", {10, 6, 8, 0.1f}, bc, 0.02f, 0.0f, 0xdff746e12694c171ull});
    }
    {
        DomainBc bc;
        bc.face = {FaceBc::wall, FaceBc::wall, FaceBc::wall, FaceBc::wall, FaceBc::periodic, FaceBc::periodic};
        bc.wall_velocity[kYPlus] = glm::vec3(1.0f, 0.0f, 0.0f);
        out.push_back({"cavity slab 16x16x1", {16, 16, 1, 1.0f / 16.0f}, bc, 0.01f, 0.0f, 0xb5eb25514662a3bdull});
    }
    out.push_back({"open 9x7x5 (odd sizes)", {9, 7, 5, 0.1f}, DomainBc{}, 0.01f, 0.0f, 0xc668e2d327d8af84ull});
    out.push_back({"open 16x12x16, disc", {16, 12, 16, 0.05f}, DomainBc{}, 0.0f, 3.0f, 0xecc7a987b2fc3d2aull});
    {
        DomainBc bc;  // a wind: inflow where it enters, outlets elsewhere
        bc.ambient = glm::vec3(0.8f, -0.2f, 0.3f);
        out.push_back({"wind 12x10x8", {12, 10, 8, 0.1f}, bc, 0.01f, 0.0f, 0xc44b75cabaec9cdeull});
    }
    {
        DomainBc bc;  // the drone box: a rolling-road floor under a wind, periodic in z
        bc.ambient = glm::vec3(1.0f, 0.0f, 0.0f);
        bc.face[kYMinus] = FaceBc::wall;
        bc.wall_velocity[kYMinus] = glm::vec3(1.0f, 0.0f, 0.0f);
        bc.face[kZMinus] = FaceBc::periodic;
        bc.face[kZPlus] = FaceBc::periodic;
        out.push_back({"floor under wind 12x8x8, disc", {12, 8, 8, 0.1f}, bc, 0.01f, 2.0f, 0x45d8ea22d9759deaull});
    }
    {
        DomainBc bc;
        bc.face = {FaceBc::periodic, FaceBc::periodic, FaceBc::wall, FaceBc::open, FaceBc::wall, FaceBc::wall};
        out.push_back({"one-cell x 1x8x8", {1, 8, 8, 0.1f}, bc, 0.01f, 0.0f, 0x247bfda845f95023ull});
    }
    return out;
}

[[nodiscard]] uint64_t run(const Config& c) {
    const GridShape& shape = c.shape;
    MacGrid g = *make_grid(shape, c.bc);
    Lcg rng;
    for (float& x : g.u) x = 0.3f * rng.next();
    for (float& x : g.v) x = 0.3f * rng.next();
    for (float& x : g.w) x = 0.3f * rng.next();
    const FluidConfig cfg{.rho = 1.0f, .nu = c.nu, .h = 0.4f * shape.dx, .n_v = 2, .visc_sweeps = 4};
    FluidScratch s = *make_fluid_scratch(g, c.bc, cfg);
    const float L[3] = {shape.nx * shape.dx, shape.ny * shape.dx, shape.nz * shape.dx};
    const ActuatorDisc disc{.centre = {0.5f * L[0], 0.5f * L[1], 0.5f * L[2]}, .axis = {0.0f, -1.0f, 0.0f},
                            .radius = c.disc_radius_cells * shape.dx, .thrust = 0.2f};
    const bool has_disc = c.disc_radius_cells > 0.0f;
    if (has_disc) {
        const auto ok = check_disc(disc, shape);  // the final review's I3: only discs the solver accepts
        EXPECT_TRUE(ok.has_value()) << c.name << ": " << (ok ? std::string() : ok.error().context);
    }
    for (int n = 0; n < 6; ++n) {
        if (has_disc) spread_actuator(disc, shape, cfg.rho, cfg.h, s.du, s.dv, s.dw);
        const FluidHealth h = fluid_step(g, c.bc, cfg, s);
        EXPECT_EQ(h.fault, FluidFault::none) << describe(h, g.shape);
    }
    uint64_t h = 1469598103934665603ull;
    h = fold(h, g.u.data(), g.u.size() * sizeof(float));
    h = fold(h, g.v.data(), g.v.size() * sizeof(float));
    h = fold(h, g.w.data(), g.w.size() * sizeof(float));
    h = fold(h, g.p.data(), g.p.size() * sizeof(float));
    spade::FlowVolume vol;
    std::vector<uint8_t> solid(std::size_t{shape.nx} * shape.ny * shape.nz, 0);
    for (std::size_t i = 0; i < solid.size(); i += 7) solid[i] = 1;
    const PublishInput in{.solid = solid,
                          .periodic = {c.bc.face[kXMinus] == FaceBc::periodic, c.bc.face[kYMinus] == FaceBc::periodic,
                                       c.bc.face[kZMinus] == FaceBc::periodic}};
    EXPECT_TRUE(publish_render_volume(g, in, vol).has_value());
    h = fold(h, vol.v.data(), vol.v.size() * sizeof(uint16_t));
    h = fold(h, vol.s.data(), vol.s.size() * sizeof(uint16_t));
    return h;
}

TEST(AirflowConfigs, EachBoundaryConfigurationsDigestIsPinned) {
    for (const Config& c : configs()) {
        const uint64_t d = run(c);
        std::printf("airflow config digest %-32s 0x%016llx\n", c.name, static_cast<unsigned long long>(d));
        EXPECT_EQ(d, c.digest) << c.name;
    }
}

}  // namespace
