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
