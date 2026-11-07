// lbm_support.hpp -- what the LBM tier's opt-in runs (the cases and the stability map's points) share: the
// environment switches and the step they choose. Tests only; nothing under engine/ reads an environment variable.
//
//   SPADE_LBM_THREADS=<n>   splits the step over n threads (the WorkerPool, fixed row chunks: the same bits as one
//                           thread, LbmThreads); unset or 1 is the serial step.
//   SPADE_LBM_GRADUAL=1     runs the unscoped step (gradual underflow) instead of the flush scope (FTZ|DAZ, as the
//                           device of record flushes; lbm_flow.hpp), for the comparison of the two. A record made
//                           with it says so.
//
// Every printed run line names its thread count, and every run ends with a digest of its final populations and the
// switch state, so two runs compare in one word.
#pragma once

#include <bit>
#include <cmath>
#include <numbers>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>

#include "physics/lbm/lbm_flow.hpp"

namespace spade::testing::lbm_support {

using spade::physics::lbm::FlowSums;
using spade::physics::lbm::LbmFlow;
using spade::physics::lbm::WorkerPool;

inline const char* env_text(const char* name) {
#if defined(_MSC_VER)
#pragma warning(push)
#pragma warning(disable : 4996)
#endif
    return std::getenv(name);
#if defined(_MSC_VER)
#pragma warning(pop)
#endif
}

inline uint32_t case_threads() {
    static const uint32_t n = [] {
        const char* v = env_text("SPADE_LBM_THREADS");
        const long k = v != nullptr ? std::strtol(v, nullptr, 10) : 1;
        return k > 1 ? static_cast<uint32_t>(k) : 1u;
    }();
    return n;
}

// The step runs under the flush scope unless SPADE_LBM_GRADUAL is set.
inline bool case_gradual() {
    static const bool g = env_text("SPADE_LBM_GRADUAL") != nullptr;
    return g;
}

inline WorkerPool* case_pool();

inline void case_step(LbmFlow& fl) {
    if (case_gradual()) {
        spade::physics::lbm::step_flow(fl, case_pool());
    } else {
        spade::physics::lbm::step_flow_flushed(fl, case_pool());
    }
}

inline FlowSums case_sums(LbmFlow& fl) {
    return case_gradual() ? spade::physics::lbm::flow_sums(fl, case_pool())
                          : spade::physics::lbm::flow_sums_flushed(fl, case_pool());
}

inline constexpr double kBoxTwoPi = 2.0 * std::numbers::pi;

// A field with structure for the split to break: 12 x 10 x 7 (7 z-planes of 10 rows is 70 rows, 18 chunks with a
// short last one of 2 rows), Taylor-Green velocity with a pressure, WALE on, optionally a box whose y faces are
// walls with the high one a moving lid.
inline LbmFlow box_field(bool lid) {
    constexpr uint32_t nx = 12;
    constexpr uint32_t ny = 10;
    constexpr uint32_t nz = 7;
    LbmFlow fl = spade::physics::lbm::make_flow(nx, ny, nz, 0.004f, 0.5f);
    if (lid) {
        fl.walls.periodic[1] = false;
        fl.walls.wall_u[1][1] = {0.08f, 0.0f, 0.0f};
    }
    const double u0 = 0.1;
    for (uint32_t z = 0; z < nz; ++z) {
        for (uint32_t y = 0; y < ny; ++y) {
            for (uint32_t x = 0; x < nx; ++x) {
                const double X = kBoxTwoPi * x / nx;
                const double Y = kBoxTwoPi * y / ny;
                const double Z = kBoxTwoPi * z / nz;
                const double p = u0 * u0 / 16.0 * (std::cos(2 * X) + std::cos(2 * Y)) * (std::cos(2 * Z) + 2.0);
                spade::physics::lbm::set_node_equilibrium(fl, x, y, z, static_cast<float>(1.0 + 3.0 * p),
                                     static_cast<float>(u0 * std::sin(X) * std::cos(Y) * std::cos(Z)),
                                     static_cast<float>(-u0 * std::cos(X) * std::sin(Y) * std::cos(Z)),
                                     static_cast<float>(0.3 * u0 * std::sin(Z)));
            }
        }
    }
    return fl;
}

// Two diagnostics are the same bits: each double by its bit pattern (so -0 differs from 0, and a NaN equals its own
// bits), `finite` by value. Never memcmp the struct: FlowSums ends in a bool after five doubles, so its padding
// bytes are never written and a byte compare reads whatever the stack held (gcc-13 -NoSeed failed on it, MSVC
// happened to match).
inline bool same_sums(const FlowSums& a, const FlowSums& b) {
    const auto same = [](double x, double y) { return std::bit_cast<uint64_t>(x) == std::bit_cast<uint64_t>(y); };
    return same(a.kinetic, b.kinetic) && same(a.mass_m1, b.mass_m1) && same(a.momentum[0], b.momentum[0]) &&
           same(a.momentum[1], b.momentum[1]) && same(a.momentum[2], b.momentum[2]) && a.finite == b.finite;
}

// FNV-1a over the populations' bits and the step count: the run's final state in one word, to compare two runs.
inline uint64_t state_digest(const LbmFlow& fl) {
    uint64_t h = 1469598103934665603ull;
    const auto mix = [&h](uint64_t byte) { h = (h ^ byte) * 1099511628211ull; };
    for (const float v : fl.grid.f) {
        uint32_t bits = 0;
        std::memcpy(&bits, &v, sizeof bits);
        for (int b = 0; b < 4; ++b) mix((bits >> (8 * b)) & 0xFFu);
    }
    for (int b = 0; b < 8; ++b) mix((fl.grid.step >> (8 * b)) & 0xFFu);
    return h;
}

inline void print_digest(const char* what, const LbmFlow& fl) {
    std::printf("DIGEST %s: f 0x%016llx after %llu steps, flush scope %s, %u thread(s)\n", what,
                static_cast<unsigned long long>(state_digest(fl)), static_cast<unsigned long long>(fl.grid.step),
                case_gradual() ? "off (gradual underflow)" : "on", case_threads());
}

inline WorkerPool* case_pool() {
    static const std::unique_ptr<WorkerPool> pool =
        case_threads() > 1 ? std::make_unique<WorkerPool>(case_threads()) : nullptr;
    return pool.get();
}
}  // namespace spade::testing::lbm_support
