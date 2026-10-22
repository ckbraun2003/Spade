#include "physics/airflow/fluid_step.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <string>

#include "core/detail/flush_subnormals.hpp"
#include "physics/airflow/multigrid.hpp"
#include "physics/airflow/pcg.hpp"

namespace spade::physics::airflow {

namespace {

constexpr const char* kSideNames[6] = {"x-", "x+", "y-", "y+", "z-", "z+"};
constexpr const char* kQuantityNames[4] = {"u", "v", "w", "p"};

void add_and_clear(std::vector<float>& vel, std::vector<float>& d) noexcept {
    for (std::size_t i = 0; i < vel.size(); ++i) {
        vel[i] += d[i];
        d[i] = 0.0f;
    }
}

[[nodiscard]] bool positive_finite(float x) noexcept { return x > 0.0f && std::isfinite(x); }

// The faces whose prescribed normal fluxes do not balance in a closed domain,
// as a bit per face; 0 when the domain has an outlet or balances. A wall
// prescribes its normal wall velocity; an open face the ambient enters (an
// inflow) prescribes the ambient's normal component (the final review's I1).
[[nodiscard]] uint8_t unbalanced_faces(const DomainBc& bc, const GridShape& s) noexcept {
    const std::array<FaceKind, 6> kinds = face_kinds(bc);
    if (std::any_of(kinds.begin(), kinds.end(), [](FaceKind k) { return k == FaceKind::pressure_outlet; })) {
        return 0;
    }
    const double n[3] = {static_cast<double>(s.nx), static_cast<double>(s.ny), static_cast<double>(s.nz)};
    const double dx2 = static_cast<double>(s.dx) * static_cast<double>(s.dx);
    double net = 0.0;
    double magnitude = 0.0;
    uint8_t faces = 0;
    for (uint32_t side = 0; side < 6; ++side) {
        if (kinds[side] != FaceKind::fixed_velocity) continue;  // walls and inflows; periodic faces carry no net flux
        const uint32_t axis = side / 2u;
        const double normal = fixed_normal_velocity(bc, static_cast<Side>(side));
        if (normal == 0.0) continue;
        const double area = n[(axis + 1u) % 3u] * n[(axis + 2u) % 3u] * dx2;
        const double outward = (side % 2u == 0u ? -normal : normal) * area;
        net += outward;
        magnitude += std::fabs(outward);
        faces = static_cast<uint8_t>(faces | (1u << side));
    }
    return std::fabs(net) > 1.0e-6 * magnitude ? faces : uint8_t{0};
}

// The first non-finite sample in u, v, w, then p, each in array order.
[[nodiscard]] FluidHealth scan(const MacGrid& g) noexcept {
    const std::vector<float>* arrays[4] = {&g.u, &g.v, &g.w, &g.p};
    for (uint8_t q = 0; q < 4; ++q) {
        const std::vector<float>& a = *arrays[q];
        for (std::size_t i = 0; i < a.size(); ++i) {
            if (!std::isfinite(a[i])) return FluidHealth{.fault = FluidFault::non_finite, .quantity = q, .index = i};
        }
    }
    return {};
}

[[nodiscard]] std::string face_list(uint8_t faces) {
    std::string out;
    for (uint32_t side = 0; side < 6; ++side) {
        if ((faces & (1u << side)) == 0u) continue;
        if (!out.empty()) out += ", ";
        out += kSideNames[side];
    }
    return out;
}

}  // namespace

std::string describe(const FluidHealth& health, const GridShape& s) {
    switch (health.fault) {
        case FluidFault::none:
            if (health.unconverged) {
                return "airflow fluid step: healthy, but the pressure solve left a relative residual of " +
                       std::to_string(health.pressure_residual) + ", above " +
                       std::to_string(health.capped ? kResidualAnnounce : kPlanAResidualAnnounce) +
                       (health.capped ? " at its cap of " + std::to_string(health.pressure_iterations) + " iterations"
                                      : std::string()) +
                       " (announced, L6)";
            }
            return "airflow fluid step: healthy";
        case FluidFault::ill_posed_domain:
            return "airflow fluid step: a closed domain's prescribed face fluxes do not balance (faces " +
                   face_list(health.faces) + ")";
        case FluidFault::ill_posed_region: {
            const uint64_t c = health.index;
            return "airflow fluid step: a closed fluid region (its first cell (" + std::to_string(c % s.nx) + ", " +
                   std::to_string((c / s.nx) % s.ny) + ", " + std::to_string(c / (uint64_t{s.nx} * s.ny)) +
                   ")) has no outlet and takes a net prescribed flux through faces " + face_list(health.faces);
        }
        case FluidFault::non_finite: {
            const uint32_t q = health.quantity;
            const uint64_t cx = q == 0u ? s.nx + 1u : s.nx;
            const uint64_t cy = q == 1u ? s.ny + 1u : s.ny;
            const uint64_t i = health.index % cx;
            const uint64_t j = (health.index / cx) % cy;
            const uint64_t k = health.index / (cx * cy);
            return std::string("airflow fluid step: a non-finite ") + kQuantityNames[q] + " at sample (" +
                   std::to_string(i) + ", " + std::to_string(j) + ", " + std::to_string(k) +
                   "); the step is refused until a new scratch is made";
        }
    }
    return "airflow fluid step: unknown fault";
}

uint64_t fluid_bytes(const GridShape& s, const DomainBc& bc) noexcept {
    const uint64_t nx = s.nx;
    const uint64_t ny = s.ny;
    const uint64_t nz = s.nz;
    const uint64_t uf = (nx + 1u) * ny * nz;
    const uint64_t vf = nx * (ny + 1u) * nz;
    const uint64_t wf = nx * ny * (nz + 1u);
    const uint64_t cells = nx * ny * nz;
    const uint64_t faces = uf + vf + wf;
    const uint64_t floats = (faces + cells)                       // the grid
                            + (faces + cells)                     // next, advection's output
                            + 3u * faces                          // advection's hat, lo and hi
                            + 3u * multigrid_cells(s.nx, s.ny, s.nz, periodic_axes(bc))  // every level's p, b, r
                            + cells                               // the divergence
                            + std::max({uf, vf, wf})              // the viscous right side
                            + faces                               // du, dv and dw
                            // plan A's weights on the levels past the halving rule
                            + multigrid_plan_a_weight_faces(s.nx, s.ny, s.nz, periodic_axes(bc));
    return floats * sizeof(float);
}

Result<void> check_domain(const DomainBc& bc, const GridShape& shape) {
    if (const uint8_t faces = unbalanced_faces(bc, shape); faces != 0u) {
        return std::unexpected(Error{Code::invalid_argument,
                                     "airflow fluid step: a closed domain (no outlet) cannot take a net prescribed "
                                     "flux through its walls and inflows; it does through faces " +
                                         face_list(faces)});
    }
    return {};
}

uint64_t solid_bytes(const GridShape& s, const DomainBc& bc) noexcept {
    const std::array<bool, 3> periodic = periodic_axes(bc);
    const uint64_t cells = uint64_t{s.nx} * s.ny * s.nz;
    // Every level's face weights, less those plan A's levels already hold.
    return solid_field_bytes(s) + regions_bytes(s) + pcg_bytes(cells) +
           (multigrid_faces(s.nx, s.ny, s.nz, periodic) - multigrid_plan_a_weight_faces(s.nx, s.ny, s.nz, periodic)) *
               sizeof(float);
}

std::string check_grid_note(const GridShape& shape, const DomainBc& bc) {
    const std::array<uint32_t, 3> c = multigrid_coarsest(shape.nx, shape.ny, shape.nz, periodic_axes(bc));
    const uint64_t cells = uint64_t{c[0]} * c[1] * c[2];
    if (cells <= kCoarsestCells) return {};
    return "airflow grid: the pressure multigrid stops at " + std::to_string(c[0]) + " x " + std::to_string(c[1]) +
           " x " + std::to_string(c[2]) + " (" + std::to_string(cells) + " cells, above " +
           std::to_string(kCoarsestCells) +
           "): every axis there is under 4 cells or a periodic ring that cannot halve to an even one, so the solve "
           "may miss its mark (each step's residual is announced)";
}

Result<void> check_grid(const GridShape& shape, const DomainBc& bc, bool with_solids) {
    if (Result<void> ok = check_shape(shape, bc); !ok) return ok;
    if (Result<void> ok = check_domain(bc, shape); !ok) return ok;
    const uint64_t bytes = fluid_bytes(shape, bc) + (with_solids ? solid_bytes(shape, bc) : 0u);
    if (bytes > kGridByteCap) {
        const char* what = with_solids ? " bytes of grid, scratch and solids" : " bytes of grid and scratch";
        return std::unexpected(Error{Code::capacity_exceeded, "airflow fluid step: " + std::to_string(bytes) + what +
                                                                  ", past the cap of " + std::to_string(kGridByteCap)});
    }
    return {};
}

Result<FluidScratch> make_fluid_scratch(const MacGrid& g, const DomainBc& bc, const FluidConfig& cfg) {
    const auto refuse = [](const char* what) {
        return std::unexpected(Error{Code::invalid_argument, std::string("airflow fluid step: ") + what});
    };
    if (!positive_finite(cfg.rho)) return refuse("rho must be positive and finite");
    if (!positive_finite(cfg.h)) return refuse("h must be positive and finite");
    if (!(cfg.nu >= 0.0f) || !std::isfinite(cfg.nu)) return refuse("nu must be non-negative and finite");
    if (cfg.n_v == 0u) return refuse("n_v must be at least 1");
    if (cfg.nu > 0.0f && cfg.visc_sweeps == 0u) return refuse("visc_sweeps must be at least 1 when nu > 0");
    if (cfg.interior == SolidInterior::extrapolate && cfg.nu > 0.0f) {
        return refuse("the free-slip interior (SolidInterior::extrapolate) is the inviscid lean mode: nu must be 0");
    }
    if (Result<void> ok = check_grid(g.shape, bc, false); !ok) return std::unexpected(ok.error());
    FluidScratch s;
    s.next = g;
    s.advect = make_advect_scratch(g.shape);
    s.projection = make_projection_scratch(g.shape, bc);
    s.rhs.assign(std::max({g.u.size(), g.v.size(), g.w.size()}), 0.0f);
    s.du.assign(g.u.size(), 0.0f);
    s.dv.assign(g.v.size(), 0.0f);
    s.dw.assign(g.w.size(), 0.0f);
    return s;
}

Result<void> set_solids(FluidScratch& s, const MacGrid& g, const DomainBc& bc, std::span<const SolidPart> parts,
                        const SolidSampler& sample) {
    if (s.solids.theta[0].empty()) {
        // The grid's first solids: the cap with them, before any is allocated.
        if (Result<void> ok = check_grid(g.shape, bc, true); !ok) return ok;
        Result<SolidField> made = make_solid_field(g.shape);
        if (!made) return std::unexpected(made.error());
        s.solids = std::move(*made);
    }
    const std::array<bool, 3> periodic = periodic_axes(bc);
    if (Result<void> ok = build_solids(s.solids, g.shape, periodic, parts, sample); !ok) {
        clear_weights(s.projection.mg);
        return ok;
    }
    if (!s.solids.any) {
        clear_weights(s.projection.mg);
        return {};
    }
    label_regions(s.regions, g.shape, periodic, s.solids);
    set_weights(s.projection.mg, {s.solids.theta[0], s.solids.theta[1], s.solids.theta[2]});
    if (s.projection.pcg.x.size() != g.p.size()) s.projection.pcg = make_pcg_scratch(g.p.size());
    return {};
}

namespace {

// fluid_step()'s body, run under its wrapper's scope: every float op of the
// step is in here, none in the wrapper (the wrapper rule,
// core/detail/flush_subnormals.hpp).
SPADE_NOINLINE FluidHealth step_flushed(MacGrid& g, const DomainBc& bc, const FluidConfig& cfg,
                                        FluidScratch& s) noexcept {
    if (s.health.fault != FluidFault::none) return s.health;
    const bool solid = s.solids.any;
    if (solid) {
        if (const RegionFault rf = check_regions(s.regions, g.shape, bc, s.solids); rf.region != kNoRegion) {
            s.health = FluidHealth{.fault = FluidFault::ill_posed_region, .faces = rf.faces, .index = rf.cell};
            return s.health;
        }
    } else if (const uint8_t faces = unbalanced_faces(bc, g.shape); faces != 0u) {
        s.health = FluidHealth{.fault = FluidFault::ill_posed_domain, .faces = faces};
        return s.health;
    }
    apply_velocity_boundaries(g, bc);
    if (solid) apply_solid_velocities(g, s.solids, cfg.interior);
    advect_velocity(g, s.next, bc, cfg.h, s.advect);
    g.u.swap(s.next.u);
    g.v.swap(s.next.v);
    g.w.swap(s.next.w);
    add_and_clear(g.u, s.du);
    add_and_clear(g.v, s.dv);
    add_and_clear(g.w, s.dw);
    apply_velocity_boundaries(g, bc);
    if (solid) apply_solid_velocities(g, s.solids, cfg.interior);
    if (cfg.nu > 0.0f) {
        diffuse(g, bc, cfg.nu, cfg.h, cfg.visc_sweeps, s.rhs, solid ? &s.solids : nullptr);
        apply_velocity_boundaries(g, bc);
        // A domain face a solid covers keeps the solid's velocity, not the
        // boundary's (P2 Task 11, the final review's finding 3).
        if (solid) apply_solid_velocities(g, s.solids, cfg.interior);
    }
    const ProjectionReport rep =
        project(g, bc, cfg.rho, cfg.h, cfg.n_v, s.projection, solid ? &s.solids : nullptr, solid ? &s.regions : nullptr,
                cfg.cg_cap);
    s.health = scan(g);
    if (s.health.fault == FluidFault::none) {
        s.health.pressure_residual = rep.residual;
        s.health.pressure_iterations = rep.iterations;
        // With solids the CG ran on to the mark, so a miss is its cap reached.
        // Plan A's fixed cycles are announced above their own mark (2a).
        s.health.unconverged = rep.residual > (solid ? kResidualAnnounce : kPlanAResidualAnnounce);
        s.health.capped = solid && s.health.unconverged;
    }
    return s.health;
}

}  // namespace

// Subnormals flushed as the RTX 3060 Ti flushes them (spec section 4.5 as
// amended 2026-10-22): the scope covers every return of the body.
FluidHealth fluid_step(MacGrid& g, const DomainBc& bc, const FluidConfig& cfg, FluidScratch& s) noexcept {
    const spade::math::ScopedFlushSubnormals flush;
    return step_flushed(g, bc, cfg, s);
}

}  // namespace spade::physics::airflow
