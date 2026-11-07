#pragma once

#include <cstdint>
#include <optional>
#include <string>

// ---------------------------------------------------------------------------
// physics/lbm/lbm_cost.hpp -- what an LBM run costs, and what the lattice is
// stated to hold (L1; the design: docs/design/physics/plans/2026-11-04-l1-lbm-
// design.md, section 8). Header-only, plain numbers: it includes nothing of
// analysis/, so the analysis layer asks it with the numbers of its ResolvedCase
// (base.nx, ny, nz, dx; spec.time; spec.fluid) and not the other way round.
//
//   lbm_cost(nx, ny, nz, steps)         node updates, steps, resident bytes
//   lbm_seconds(cost, rung, device)     the cost over a recorded rate, or nothing
//                                       where no row is recorded (never a guess)
//   lbm_covers(rung, u_lat, nu_lat)     whether the point is inside the rung's
//                                       coverage table, with the refusal's text
//   lbm_u_lat / lbm_nu_lat / lbm_steps  the lattice units of a physical case
//
// THE COST TABLE (kLbmCostRows). Each row is a measured rate with where, when
// and under what load it was measured, and names the record it is copied from
// (tests/data/airflow/records/): LbmCost.TheRowsAreTheRecordsNumbers reads those
// records and compares. A quiet re-probe replaces a row only if it is higher; a
// row is never edited, a better one is added beside it with its own date.
// ---------------------------------------------------------------------------

namespace spade::physics::lbm {

// The rungs of the stability map (design section 8). R0 is RR3 + WALE (C_w 0.5), no fallback.
enum class LbmRung : uint8_t { r0 };

// 27 populations and 3 velocity components a node, fp32: the CPU reference's and the device's resident bytes.
// (124 with L2's flags, which are not declared yet.)
inline constexpr uint32_t kLbmBytesPerNode = (27 + 3) * 4;

struct LbmCost {
    uint64_t nodes = 0;
    uint64_t steps = 0;
    uint64_t node_updates = 0;  // nodes x steps
    uint64_t bytes = 0;         // resident: nodes x kLbmBytesPerNode
    friend constexpr bool operator==(const LbmCost&, const LbmCost&) = default;
};

[[nodiscard]] constexpr LbmCost lbm_cost(uint64_t nx, uint64_t ny, uint64_t nz, uint64_t steps) noexcept {
    const uint64_t nodes = nx * ny * nz;
    return LbmCost{.nodes = nodes, .steps = steps, .node_updates = nodes * steps, .bytes = nodes * kLbmBytesPerNode};
}

// Where a rate was measured. The Vulkan row waits for the GPU twin's T6.
struct LbmDevice {
    enum class Kind : uint8_t { cpu, vulkan };
    Kind kind = Kind::cpu;
    uint32_t threads = 1;  // the CPU reference's WorkerPool size; 1 for the device
    friend constexpr bool operator==(const LbmDevice&, const LbmDevice&) = default;
};

inline constexpr LbmDevice kCpuOneThread{LbmDevice::Kind::cpu, 1};
inline constexpr LbmDevice kCpuSixThreads{LbmDevice::Kind::cpu, 6};

struct LbmCostRow {
    LbmRung rung;
    LbmDevice device;
    double mlups;                 // million node updates a second
    const char* machine;          // what ran it
    const char* date;             // when
    const char* box_state;        // what else was on the box, and how the rate was taken
    const char* record_file;      // the record it is copied from: the file in tests/data/airflow/records/,
    const char* record_identity;  // the record's opening line,
    const char* record_quantity;  // and the measure in it
};

inline constexpr LbmCostRow kLbmCostRows[] = {
    {LbmRung::r0, kCpuOneThread, 0.792, "CPU reference, MSVC 1943, release, scalar (the dev box)",
     "2026-11-05", "solo in Spade's queue, Kat's gate jobs on the box; the periodic probe, gradual underflow",
     "l1-stability-map.md", "L1 cost record, R0, 64^3 Taylor-Green, u_lat 0.100 nu_lat 1e-5, solo probe, LBM",
     "single-thread rate, solo in Spade's queue, Kat's gate jobs on the box"},
    {LbmRung::r0, kCpuSixThreads, 3.652, "CPU reference, MSVC 1943, release, 6 threads (WorkerPool; the dev box)",
     "2026-11-06",
     "shared box, other jobs running; C0b 128^3 (the walled C1 slab runs 3.3 to 3.9, 64^3 periodic 3.3 to 4.6)",
     "c0-taylor-green.md", "C0 Taylor-Green vortex, Re 1600, 128^3, LBM", "rate, 6 threads, shared box"},
};

[[nodiscard]] constexpr const LbmCostRow* lbm_cost_row(LbmRung rung, LbmDevice device) noexcept {
    for (const LbmCostRow& row : kLbmCostRows) {
        if (row.rung == rung && row.device == device) return &row;
    }
    return nullptr;
}

// Seconds for the cost at the recorded rate, or nothing where no row is recorded for the device. Never an
// extrapolation to another thread count or to a device nobody has measured.
[[nodiscard]] constexpr std::optional<double> lbm_seconds(const LbmCost& cost, LbmRung rung,
                                                          LbmDevice device) noexcept {
    const LbmCostRow* row = lbm_cost_row(rung, device);
    if (row == nullptr) return std::nullopt;
    return static_cast<double>(cost.node_updates) / (row->mlups * 1.0e6);
}

// ---- the coverage table (design section 8) ---------------------------------

struct LbmCoverage {
    LbmRung rung;
    double max_u_lat;      // stated: any nu_lat down to 0
    double edge_row_u_lat; // the edge row recorded (L5's nominal)
    double first_break;    // the lowest u_lat tried that broke at some nu_lat
};

// R0: 0.275 held at all five nu_lat (0 to 1e-3); 0.254 is L5's nominal row; the first break is at 0.30. Tested on the
// 64^3 periodic Taylor-Green field to t* 20: no walls, nests or sustained flow (L2, L4 and L5 carry their own).
inline constexpr LbmCoverage kLbmCoverage[] = {{LbmRung::r0, 0.275, 0.254, 0.30}};

[[nodiscard]] constexpr const LbmCoverage* lbm_coverage(LbmRung rung) noexcept {
    for (const LbmCoverage& c : kLbmCoverage) {
        if (c.rung == rung) return &c;
    }
    return nullptr;
}

struct LbmCovered {
    bool covered = false;
    std::string why;  // empty when covered; else the point and the rung, for resolve() to name
};

[[nodiscard]] inline LbmCovered lbm_covers(LbmRung rung, double u_lat, double nu_lat) {
    const LbmCoverage* c = lbm_coverage(rung);
    const std::string point = "u_lat " + std::to_string(u_lat) + ", nu_lat " + std::to_string(nu_lat);
    if (c == nullptr) return {false, point + ": no coverage table for this rung"};
    // NaN fails every comparison, so it is refused by the negated tests.
    if (!(u_lat > 0.0) || !(nu_lat >= 0.0)) {
        return {false, point + ": a lattice velocity above 0 and a viscosity of 0 or more are needed"};
    }
    if (u_lat > c->max_u_lat) {
        return {false, point + " is outside rung R0's coverage: the lattice is stated to hold u_lat up to " +
                           std::to_string(c->max_u_lat) + " (any nu_lat down to 0); the map's first break is at " +
                           std::to_string(c->first_break)};
    }
    return {true, {}};
}

// ---- lattice units of a physical case --------------------------------------

// dt = u_lat dx / u_ref: the physical step that makes u_ref the lattice velocity u_lat.
[[nodiscard]] constexpr double lbm_dt(double u_ref, double dx, double u_lat) noexcept { return u_lat * dx / u_ref; }
// The lattice velocity a physical step dt gives the speed u_ref on cells of dx.
[[nodiscard]] constexpr double lbm_u_lat(double u_ref, double dt, double dx) noexcept { return u_ref * dt / dx; }
// nu_lat = nu dt / dx^2.
[[nodiscard]] constexpr double lbm_nu_lat(double nu, double dt, double dx) noexcept { return nu * dt / (dx * dx); }
// Steps to cover a duration: the whole steps that reach it.
[[nodiscard]] inline uint64_t lbm_steps(double duration, double dt) noexcept {
    const double exact = duration / dt;
    const uint64_t whole = static_cast<uint64_t>(exact);
    return static_cast<double>(whole) < exact ? whole + 1 : whole;  // no libm under engine/
}

}  // namespace spade::physics::lbm
