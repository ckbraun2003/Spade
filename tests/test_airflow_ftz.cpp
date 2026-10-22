// The CPU airflow step flushes subnormals as the RTX 3060 Ti does (the FTZ
// plan, docs/design/physics/plans/2026-10-22-airflow-ftz-plan.md, Tasks 1-3;
// spec section 4.5 as amended 2026-10-22). Each guarded entry point --
// fluid_step, spread_actuator, wall_load and publish_render_volume (the
// lead's D1) -- runs under the one MXCSR scope: FTZ and DAZ set, round to
// nearest, every exception masked (the word 0x9FC0, D3), and the caller's
// word written back whole on every exit (D4), on whichever thread calls it.
//
// These tests write the word a caller might hold with _mm_setcsr and read it
// back after the call; they do no float work between the two. A flush test
// reads its operands from memory or through volatile, so nothing folds at
// compile time, where compilers use IEEE and ignore MXCSR (the plan's F3).
// Every case's numbers come from the plan's Task 0 measurement (MSVC
// release, 2026-10-22), each restated where it is used.
#include <gtest/gtest.h>

#include <xmmintrin.h>

#include <array>
#include <bit>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <limits>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <glm/glm.hpp>

#include "airflow_solid_shapes.hpp"
#include "core/detail/flush_subnormals.hpp"
#include "core/flow_volume.hpp"
#include "physics/airflow/actuator.hpp"
#include "physics/airflow/advect.hpp"
#include "physics/airflow/boundaries.hpp"
#include "physics/airflow/fluid_step.hpp"
#include "physics/airflow/publish.hpp"
#include "physics/airflow/viscous.hpp"
#include "physics/airflow/wall_load.hpp"

namespace {

using namespace spade::physics::airflow;
using spade::FlowVolume;
using spade::math::kFlushSubnormalsWord;
using spade::math::ScopedFlushSubnormals;

constexpr unsigned kDefaultWord = 0x1F80u;  // every exception masked, round to nearest, no flag
constexpr unsigned kRoundDown = 0x2000u;    // RC = 01
constexpr unsigned kCallerWord = kDefaultWord | kRoundDown;  // 0x3F80: a caller rounding down

[[nodiscard]] uint32_t bits_of(float x) {
    uint32_t u = 0;
    std::memcpy(&u, &x, sizeof(u));
    return u;
}

[[nodiscard]] float float_of(uint32_t u) {
    float x = 0.0f;
    std::memcpy(&x, &u, sizeof(x));
    return x;
}

// By bits: a zero exponent and a non-zero fraction.
[[nodiscard]] bool subnormal(float x) {
    const uint32_t b = bits_of(x);
    return (b & 0x7F800000u) == 0u && (b & 0x007FFFFFu) != 0u;
}

[[nodiscard]] uint64_t count(std::span<const float> v) {
    uint64_t n = 0;
    for (const float x : v) n += subnormal(x) ? 1u : 0u;
    return n;
}

// Every array a fluid step writes: the grid, the increments, the viscous
// right side, advection's output and scratch, the divergence, and every
// multigrid level's p, b and r.
[[nodiscard]] std::vector<std::pair<std::string, uint64_t>> census(const MacGrid& g, const FluidScratch& s) {
    std::vector<std::pair<std::string, uint64_t>> out;
    out.emplace_back("u", count(g.u));
    out.emplace_back("v", count(g.v));
    out.emplace_back("w", count(g.w));
    out.emplace_back("p", count(g.p));
    out.emplace_back("du", count(s.du));
    out.emplace_back("dv", count(s.dv));
    out.emplace_back("dw", count(s.dw));
    out.emplace_back("rhs", count(s.rhs));
    out.emplace_back("next.u", count(s.next.u));
    out.emplace_back("next.v", count(s.next.v));
    out.emplace_back("next.w", count(s.next.w));
    out.emplace_back("next.p", count(s.next.p));
    for (uint32_t c = 0; c < 3; ++c) {
        out.emplace_back("hat[" + std::to_string(c) + "]", count(s.advect.hat[c]));
        out.emplace_back("lo[" + std::to_string(c) + "]", count(s.advect.lo[c]));
        out.emplace_back("hi[" + std::to_string(c) + "]", count(s.advect.hi[c]));
    }
    out.emplace_back("div", count(s.projection.div));
    for (std::size_t l = 0; l < s.projection.mg.levels.size(); ++l) {
        const PoissonLevel& lv = s.projection.mg.levels[l];
        out.emplace_back("level " + std::to_string(l) + " p", count(lv.p));
        out.emplace_back("level " + std::to_string(l) + " b", count(lv.b));
        out.emplace_back("level " + std::to_string(l) + " r", count(lv.r));
    }
    return out;
}

[[nodiscard]] uint64_t total(const std::vector<std::pair<std::string, uint64_t>>& c) {
    uint64_t t = 0;
    for (const auto& entry : c) t += entry.second;
    return t;
}

[[nodiscard]] uint64_t digest(const MacGrid& g) {
    uint64_t h = 1469598103934665603ull;
    const auto fold = [&](const std::vector<float>& v) {
        const auto* bytes = reinterpret_cast<const unsigned char*>(v.data());
        for (std::size_t i = 0; i < v.size() * sizeof(float); ++i) {
            h ^= bytes[i];
            h *= 1099511628211ull;
        }
    };
    fold(g.u);
    fold(g.v);
    fold(g.w);
    fold(g.p);
    return h;
}

// THE TINY LID (Task 0). AirflowStep's 16^3 cavity (test_airflow_step.cpp,
// cavity_bc()) from rest, its lid slowed to 1e-30 m/s, the first of the
// plan's 2^-8 ladder to leave a subnormal. Measured 2026-10-22 (MSVC release,
// master's step, nothing flushed): one step raises DE and UE and leaves 6361
// subnormals -- w 6, the divergence 1962, level 0's b 3104 and r 1105, level
// 1's r 165, level 2's r 19; diffuse() alone, from the step's own
// pre-diffusion state, makes 1349 in u. Under FTZ+DAZ: none.
constexpr float kTinyLid = 1.0e-30f;
const GridShape kLidShape{16, 16, 16, 1.0f / 16.0f};
const FluidConfig kLidCfg{.rho = 1.0f, .nu = 0.01f, .h = 0.5f / 16.0f, .n_v = 2, .visc_sweeps = 8};

[[nodiscard]] DomainBc lid_bc(float lid) {
    DomainBc bc;
    bc.face.fill(FaceBc::wall);
    bc.wall_velocity[kYPlus] = glm::vec3(lid, 0.0f, 0.0f);
    return bc;
}

struct Lcg {
    uint32_t state = 12345u;
    float next() noexcept {
        state = state * 1664525u + 1013904223u;
        return static_cast<float>(state >> 8) * (2.0f / 16777216.0f) - 1.0f;
    }
};

// ---------------------------------------------------------------------------
// Task 1: the scope in its one home (core/detail/flush_subnormals.hpp).
// ---------------------------------------------------------------------------

// A caller rounding down and holding a sticky flag (IE): inside, the word is
// exactly 0x9FC0 whatever the caller's (D3); after, the caller's word is back
// exactly, its flag included (D4). Mutations: no restore (the after row goes
// red); Core's old word, saved | 0x8040 (the inside row goes red).
TEST(AirflowFtz, TheScopeWritesOneWordAndRestoresTheCallers) {
    constexpr unsigned caller = kCallerWord | 0x0001u;  // 0x3F81
    EXPECT_EQ(kFlushSubnormalsWord, 0x9FC0u);
    const unsigned saved = _mm_getcsr();
    _mm_setcsr(caller);
    unsigned inside = 0;
    {
        const ScopedFlushSubnormals flush;
        inside = _mm_getcsr();
    }
    const unsigned after = _mm_getcsr();
    _mm_setcsr(saved);
    EXPECT_EQ(inside, 0x9FC0u) << "inside: 0x" << std::hex << inside;
    EXPECT_EQ(after, caller) << "after: 0x" << std::hex << after;
}

// One multiply, in its own call, of operands read through volatile: the word
// in effect at the call is the one it runs under, and nothing folds at
// compile time.
SPADE_NOINLINE uint32_t product_bits(const volatile float* a, const volatile float* b) {
    const float x = *a;
    const float y = *b;
    return bits_of(x * y);
}

// The plan's F4 rows, as this CPU gives them under the scope (tininess after
// rounding) and without it (IEEE gradual underflow). Outside, upper and lower
// both give 2^-126, so the rows discriminate. "daz" is a subnormal operand
// whose product is subnormal too, so FTZ alone also zeroes it; "daz to
// normal" is a subnormal operand whose product is normal (2^-127 x 4 =
// 2^-125), which only DAZ zeroes (the plan's correction, 2026-10-22).
// Mutations: FTZ only, 0x9F80 ("daz to normal" goes red); DAZ only, 0x1FC0
// ("lower" goes red).
TEST(AirflowFtz, ProductsAtTheSmallestNormalFlushAsThisCpuDoes) {
    struct Row {
        const char* name;
        uint32_t a;
        uint32_t b;
        uint32_t inside;
        uint32_t outside;
    };
    const Row rows[] = {
        {"upper", 0x1FFFFFFEu, 0x20000001u, 0x00800000u, 0x00800000u},
        {"lower", 0x1FFFF448u, 0x200005DCu, 0x00000000u, 0x00800000u},
        {"exact", 0x20000000u, 0x20000000u, 0x00800000u, 0x00800000u},
        {"neg-upper", 0x9FFFFFFEu, 0x20000001u, 0x80800000u, 0x80800000u},
        {"daz", 0x00400000u, 0x3F800000u, 0x00000000u, 0x00400000u},
        {"daz to normal", 0x00400000u, 0x40800000u, 0x00000000u, 0x01000000u},
        {"2^-70 squared", 0x1C800000u, 0x1C800000u, 0x00000000u, 0x00000200u},
    };
    for (const Row& r : rows) {
        volatile float a = float_of(r.a);
        volatile float b = float_of(r.b);
        uint32_t inside = 0;
        {
            const ScopedFlushSubnormals flush;
            inside = product_bits(&a, &b);
        }
        const uint32_t outside = product_bits(&a, &b);
        EXPECT_EQ(inside, r.inside) << r.name << " inside the scope: 0x" << std::hex << inside;
        EXPECT_EQ(outside, r.outside) << r.name << " outside the scope: 0x" << std::hex << outside;
    }
}

// The scope on the main thread, then on a std::thread: each reads 0x9FC0
// inside, the worker its own prior word after, and the main thread's word is
// unchanged after the join (MXCSR is per thread; the scope has no static
// state). Mutation: a process-wide "set once" flag in the constructor (the
// worker then reads its own default word inside).
TEST(AirflowFtz, EachThreadKeepsItsOwnWord) {
    const unsigned main_before = _mm_getcsr();
    unsigned main_inside = 0;
    {
        const ScopedFlushSubnormals flush;
        main_inside = _mm_getcsr();
    }
    unsigned before = 0;
    unsigned inside = 0;
    unsigned after = 0;
    std::thread t([&] {
        before = _mm_getcsr();
        {
            const ScopedFlushSubnormals flush;
            inside = _mm_getcsr();
        }
        after = _mm_getcsr();
    });
    t.join();
    std::printf("main 0x%04X, inside 0x%04X; worker 0x%04X -> inside 0x%04X -> 0x%04X\n", main_before, main_inside,
                before, inside, after);
    EXPECT_EQ(main_inside, 0x9FC0u);
    EXPECT_EQ(inside, 0x9FC0u);
    EXPECT_EQ(after, before);
    EXPECT_EQ(_mm_getcsr(), main_before);
}

// ---------------------------------------------------------------------------
// Task 2: fluid_step under the scope.
// ---------------------------------------------------------------------------

// The step's own stages before its viscous solve (fluid_step.cpp: the
// boundaries, advection, the swap, the increments, the boundaries), with no
// solids. Unguarded: these are stage functions, not entry points.
void pre_diffusion(MacGrid& g, const DomainBc& bc, FluidScratch& s) {
    apply_velocity_boundaries(g, bc);
    advect_velocity(g, s.next, bc, kLidCfg.h, s.advect);
    g.u.swap(s.next.u);
    g.v.swap(s.next.v);
    g.w.swap(s.next.w);
    apply_velocity_boundaries(g, bc);
}

// From rest, the tiny lid's viscous solve makes subnormals when nothing
// flushes (the precondition, printed); one guarded step leaves none, in any
// array it writes. Red on master. Mutation: the scope removed from the
// fluid_step wrapper.
TEST(AirflowFtz, AStepFromRestLeavesNoSubnormal) {
    const DomainBc bc = lid_bc(kTinyLid);
    {
        MacGrid g = *make_grid(kLidShape, bc);
        FluidScratch s = *make_fluid_scratch(g, bc, kLidCfg);
        pre_diffusion(g, bc, s);
        std::vector<float> rhs(s.rhs.size(), 0.0f);
        diffuse(g, bc, kLidCfg.nu, kLidCfg.h, kLidCfg.visc_sweeps, rhs, nullptr);
        const uint64_t made = count(g.u) + count(g.v) + count(g.w) + count(rhs);
        std::printf("tiny lid %.3e: diffuse() unguarded makes %llu subnormals (u %llu, v %llu, w %llu, rhs %llu)\n",
                    static_cast<double>(kTinyLid), static_cast<unsigned long long>(made),
                    static_cast<unsigned long long>(count(g.u)), static_cast<unsigned long long>(count(g.v)),
                    static_cast<unsigned long long>(count(g.w)), static_cast<unsigned long long>(count(rhs)));
        ASSERT_GT(made, 0u) << "the case no longer makes a subnormal when nothing flushes";
    }
    MacGrid g = *make_grid(kLidShape, bc);
    FluidScratch s = *make_fluid_scratch(g, bc, kLidCfg);
    ASSERT_EQ(fluid_step(g, bc, kLidCfg, s).fault, FluidFault::none);
    const auto c = census(g, s);
    for (const auto& [name, n] : c) EXPECT_EQ(n, 0u) << name << " holds subnormals after a guarded step";
    std::printf("after one step: %llu subnormals\n", static_cast<unsigned long long>(total(c)));
}

// Three returns -- healthy, a poisoned scratch (fluid_step.cpp's first early
// return) and a closed box taking a net inflow (the domain check's) -- each
// from a caller rounding down: its word comes back exactly, sticky flags
// included (the step's own are discarded, D4). And the caller's rounding mode
// does not reach the step: a healthy step's digest from that caller equals
// the default caller's (D3). Mutations: a hand-written restore before the
// body's last return only (the early rows go red); Core's old word,
// saved | 0x8040 (the digest row goes red).
TEST(AirflowFtz, TheStepRestoresTheCallersWordOnEveryReturn) {
    const DomainBc bc = lid_bc(1.0f);
    const auto stepped = [&](unsigned word, uint64_t* d) {
        MacGrid g = *make_grid(kLidShape, bc);
        FluidScratch s = *make_fluid_scratch(g, bc, kLidCfg);
        const unsigned saved = _mm_getcsr();
        _mm_setcsr(word);
        const FluidHealth h = fluid_step(g, bc, kLidCfg, s);
        const unsigned after = _mm_getcsr();
        _mm_setcsr(saved);
        EXPECT_EQ(h.fault, FluidFault::none);
        if (d != nullptr) *d = digest(g);
        return after;
    };
    uint64_t from_caller = 0;
    uint64_t from_default = 0;
    EXPECT_EQ(stepped(kCallerWord, &from_caller), kCallerWord) << "healthy";
    EXPECT_EQ(stepped(kDefaultWord, &from_default), kDefaultWord) << "healthy, the default word";
    EXPECT_EQ(from_caller, from_default) << "the caller's rounding mode reached the step";

    {  // A poisoned scratch: the first early return.
        const DomainBc open;
        MacGrid g = *make_grid({8, 8, 8, 0.125f}, open);
        g.u[u_index(g.shape, 3, 4, 5)] = std::numeric_limits<float>::quiet_NaN();
        const FluidConfig cfg{.h = 0.01f};
        FluidScratch s = *make_fluid_scratch(g, open, cfg);
        ASSERT_EQ(fluid_step(g, open, cfg, s).fault, FluidFault::non_finite);
        const unsigned saved = _mm_getcsr();
        _mm_setcsr(kCallerWord);
        const FluidHealth h = fluid_step(g, open, cfg, s);
        const unsigned after = _mm_getcsr();
        _mm_setcsr(saved);
        EXPECT_EQ(h.fault, FluidFault::non_finite);
        EXPECT_EQ(after, kCallerWord) << "a poisoned scratch's return";
    }
    {  // A closed box pushed in at x- and nowhere out: the domain check's return.
        DomainBc closed;
        closed.face.fill(FaceBc::wall);
        closed.wall_velocity[kXMinus] = glm::vec3(0.5f, 0.0f, 0.0f);
        DomainBc piston = closed;
        piston.wall_velocity[kXPlus] = glm::vec3(0.5f, 0.0f, 0.0f);
        MacGrid g = *make_grid({8, 8, 8, 0.125f}, piston);
        const FluidConfig cfg{.h = 0.01f};
        FluidScratch s = *make_fluid_scratch(g, piston, cfg);
        const unsigned saved = _mm_getcsr();
        _mm_setcsr(kCallerWord);
        const FluidHealth h = fluid_step(g, closed, cfg, s);
        const unsigned after = _mm_getcsr();
        _mm_setcsr(saved);
        EXPECT_EQ(h.fault, FluidFault::ill_posed_domain);
        EXPECT_EQ(after, kCallerWord) << "an ill-posed domain's return";
    }
}

// MXCSR is per thread, and nothing in the engine starts one (the plan's F1):
// a step on a new thread flushes there, by the scope inside the entry point.
// The worker's tiny-lid step equals the main thread's, leaves no subnormal,
// and leaves the worker's word as it found it; the main thread's word is
// untouched. Mutation: a process-wide "set once" flag in the scope's
// constructor (the worker then steps with its own default word).
TEST(AirflowFtz, AWorkerThreadStepsFlushedAndIsRestored) {
    const DomainBc bc = lid_bc(kTinyLid);
    struct Run {
        uint64_t digest = 0;
        uint64_t subnormals = 0;
        unsigned before = 0;
        unsigned after = 0;
    };
    const auto run = [&bc] {
        Run r;
        MacGrid g = *make_grid(kLidShape, bc);
        FluidScratch s = *make_fluid_scratch(g, bc, kLidCfg);
        r.before = _mm_getcsr();
        (void)fluid_step(g, bc, kLidCfg, s);
        r.after = _mm_getcsr();
        r.digest = digest(g);
        r.subnormals = total(census(g, s));
        return r;
    };
    const unsigned main_word = _mm_getcsr();
    const Run main_run = run();
    Run worker;
    std::thread t([&] { worker = run(); });
    t.join();
    std::printf("main: digest 0x%016llx, word 0x%04X -> 0x%04X; worker: digest 0x%016llx, word 0x%04X -> 0x%04X, "
                "%llu subnormals\n",
                static_cast<unsigned long long>(main_run.digest), main_run.before, main_run.after,
                static_cast<unsigned long long>(worker.digest), worker.before, worker.after,
                static_cast<unsigned long long>(worker.subnormals));
    EXPECT_EQ(main_run.subnormals, 0u);
    EXPECT_EQ(main_run.after, main_run.before);
    EXPECT_EQ(worker.digest, main_run.digest);
    EXPECT_EQ(worker.subnormals, 0u);
    EXPECT_EQ(worker.after, worker.before);
    EXPECT_EQ(_mm_getcsr(), main_word);
}

// THE TINY LID'S GOLDEN (TD-1, TD-12): 16 steps from rest, FNV-1a over u, v,
// w and p. MSVC and gcc-13 agree only if both flush inside the step: unflushed,
// the 16 steps leave 1997 subnormals and the digest 0x010b593c983c26ef on MSVC,
// and a caller's rounding mode inside (RC down) gives 0xfb4bd05a784d81f8
// (Task 0, 2026-10-22).
//   generated  2026-10-22, MSVC release: Task 0's scratch run of master's
//              step under the word 0x9FC0 gave it, and this branch's guarded
//              fluid_step gives it at GREEN; the -NoSeed gcc-13 leg is to
//              reproduce it (TD-12).
// Mutation: the scope compiled under _MSC_VER only (the gcc leg goes red).
constexpr uint64_t kTinyLidDigest = 0xe2b5d178b90aba65ull;

TEST(AirflowFtz, TheTinyLidDigestIsPinned) {
    const DomainBc bc = lid_bc(kTinyLid);
    MacGrid g = *make_grid(kLidShape, bc);
    FluidScratch s = *make_fluid_scratch(g, bc, kLidCfg);
    for (int n = 0; n < 16; ++n) ASSERT_EQ(fluid_step(g, bc, kLidCfg, s).fault, FluidFault::none);
    const uint64_t d = digest(g);
    std::printf("tiny lid digest, 16 steps: 0x%016llx\n", static_cast<unsigned long long>(d));
    EXPECT_EQ(d, kTinyLidDigest);
}

// ---------------------------------------------------------------------------
// Task 3: the sources, the wall load and publish (D1).
// ---------------------------------------------------------------------------

// THE TINY DISC (Task 0): AirflowActuator's disc on a 32^3 grid, its thrust
// and its torque 1e-30 x 2^-24 (5.96e-38, normal), so the swirl reaches du and
// dw too. The spread's own products, formed here unguarded as spread_actuator
// forms them, give subnormal increments (measured 2026-10-22: du 340, dv 336,
// dw 340); the guarded spread writes none, and the caller's word comes back.
// Mutation: the scope removed from the spread_actuator wrapper.
constexpr float kTinyThrust = std::bit_cast<float>(0x01A24260u);

TEST(AirflowFtz, ATinyDiscsSpreadLeavesNoSubnormal) {
    const GridShape shape{32, 32, 32, 1.0f / 32.0f};
    const MacGrid g = *make_grid(shape, DomainBc{});
    const float rho = 1.225f;
    const float h = 0.004f;
    const ActuatorDisc disc{.centre = {0.5f, 0.5f, 0.5f}, .axis = {0.0f, -1.0f, 0.0f}, .radius = 0.2f,
                            .thrust = kTinyThrust, .torque = kTinyThrust};
    ASSERT_TRUE(check_disc(disc, shape).has_value());
    {  // The precondition: spread_actuator's arithmetic, unguarded, here.
        const DiscSpread sp = disc_spread(disc, shape, rho, h);
        const float inv_dx = 1.0f / shape.dx;
        uint64_t made = 0;
        for (uint32_t c = 0; c < 3; ++c) {
            const float axis_c = disc.axis[static_cast<glm::length_t>(c)];
            for_each_sample(shape, c, [&](std::size_t, glm::vec3 x) {
                float sum = 0.0f;
                float swirl_sum = 0.0f;
                for (uint32_t i = 0; i < kDiscPoints; ++i) {
                    const glm::vec3 f = (x - sp.points[i]) * inv_dx;
                    const float w = peskin4(f.x) * peskin4(f.y) * peskin4(f.z);
                    sum += w;
                    swirl_sum += sp.tangent[i][static_cast<glm::length_t>(c)] * w;
                }
                made += subnormal(sp.scale * axis_c * sum + sp.swirl_scale * swirl_sum) ? 1u : 0u;
            });
        }
        std::printf("tiny disc %.3e N: %llu subnormal increments unguarded\n", static_cast<double>(kTinyThrust),
                    static_cast<unsigned long long>(made));
        ASSERT_GT(made, 0u) << "the disc no longer makes a subnormal increment when nothing flushes";
    }
    std::vector<float> du(g.u.size(), 0.0f), dv(g.v.size(), 0.0f), dw(g.w.size(), 0.0f);
    const unsigned saved = _mm_getcsr();
    _mm_setcsr(kCallerWord);
    spread_actuator(disc, shape, rho, h, du, dv, dw);
    const unsigned after = _mm_getcsr();
    _mm_setcsr(saved);
    EXPECT_EQ(after, kCallerWord);
    EXPECT_EQ(count(du), 0u);
    EXPECT_EQ(count(dv), 0u);
    EXPECT_EQ(count(dw), 0u);
}

// A subnormal pressure, p = 2^-130 (0x00080000), in the cut cell beside
// AirflowWallLoad's sphere (here at dx 0.1 m) with the largest area vector,
// the moment taken about a box corner so the arm does not lie along it:
// -p A and the arm x (-p A) are non-zero subnormals unguarded (the
// precondition, printed; measured 2026-10-22: -p A = 0x00000BCC in each
// component), and the guarded wall load reads p as zero (DAZ), so its force
// and moment are exactly 0. Mutation: the scope removed from the wall_load
// wrapper.
constexpr float kWallDx = 0.1f;

TEST(AirflowFtz, AWallLoadReadsASubnormalPressureAsZero) {
    using airflow_test::Kind;
    using airflow_test::Scene;
    using airflow_test::Shape;
    const float dx = kWallDx;
    const GridShape shape{32, 32, 32, dx};
    const DomainBc bc;
    MacGrid g = *make_grid(shape, bc);
    FluidScratch s = *make_fluid_scratch(g, bc, FluidConfig{.h = 0.001f});
    Scene scene;
    scene.shapes.push_back(Shape{.name = "sphere", .kind = Kind::sphere, .centre = glm::vec3(16.0f * dx),
                                 .radius = 6.0f * dx, .min_feature = 12.0f * dx});
    ASSERT_TRUE(set_solids(s, g, bc, scene.parts(), scene.sampler()).has_value()) << s.solids.run_line;
    // The fluid cell with the largest area vector (wall_load.cpp's a).
    const float area = dx * dx;
    const SolidField& f = s.solids;
    std::size_t cut = g.p.size();
    glm::vec3 a_cut(0.0f);
    float largest = 0.0f;
    for (uint32_t k = 0; k < shape.nz; ++k) {
        for (uint32_t j = 0; j < shape.ny; ++j) {
            for (uint32_t i = 0; i < shape.nx; ++i) {
                const std::size_t c = cell_index(shape, i, j, k);
                g.p[c] = s.regions.solid[c] != 0u ? std::numeric_limits<float>::quiet_NaN() : 0.0f;
                if (s.regions.solid[c] != 0u) continue;
                const glm::vec3 a(area * (f.theta[0][u_index(shape, i + 1u, j, k)] - f.theta[0][u_index(shape, i, j, k)]),
                                  area * (f.theta[1][v_index(shape, i, j + 1u, k)] - f.theta[1][v_index(shape, i, j, k)]),
                                  area * (f.theta[2][w_index(shape, i, j, k + 1u)] - f.theta[2][w_index(shape, i, j, k)]));
                const float size = std::fabs(a.x) + std::fabs(a.y) + std::fabs(a.z);
                if (size > largest) {
                    largest = size;
                    cut = c;
                    a_cut = a;
                }
            }
        }
    }
    ASSERT_LT(cut, g.p.size()) << "no cut cell";
    volatile uint32_t p_bits = 0x00080000u;  // 2^-130
    g.p[cut] = float_of(p_bits);
    volatile float p = g.p[cut];
    const glm::vec3 push = -p * a_cut;  // unguarded, as wall_load forms it
    std::printf("cut cell %zu: a = (%.4e, %.4e, %.4e) m^2, -p a = (0x%08X, 0x%08X, 0x%08X) unguarded\n", cut,
                static_cast<double>(a_cut.x), static_cast<double>(a_cut.y), static_cast<double>(a_cut.z),
                bits_of(push.x), bits_of(push.y), bits_of(push.z));
    ASSERT_TRUE(subnormal(push.x) || subnormal(push.y) || subnormal(push.z))
        << "-p a is no longer a non-zero subnormal when nothing flushes";
    const glm::vec3 about(32.0f * dx, 0.0f, 0.0f);
    const std::size_t ci = cut % shape.nx;
    const std::size_t cj = (cut / shape.nx) % shape.ny;
    const std::size_t ck = cut / (std::size_t{shape.nx} * shape.ny);
    const glm::vec3 arm = glm::vec3((static_cast<float>(ci) + 0.5f) * dx, (static_cast<float>(cj) + 0.5f) * dx,
                                    (static_cast<float>(ck) + 0.5f) * dx) -
                          about;
    const glm::vec3 turn = glm::cross(arm, push);  // unguarded
    std::printf("the moment's arm x (-p a) = (0x%08X, 0x%08X, 0x%08X) unguarded\n", bits_of(turn.x),
                bits_of(turn.y), bits_of(turn.z));
    ASSERT_TRUE(turn.x != 0.0f || turn.y != 0.0f || turn.z != 0.0f) << "the moment is zero even when nothing flushes";
    const unsigned saved = _mm_getcsr();
    _mm_setcsr(kCallerWord);
    const WallLoad load = wall_load(g, s.solids, s.regions, about);
    const unsigned after = _mm_getcsr();
    _mm_setcsr(saved);
    EXPECT_EQ(after, kCallerWord);
    for (int q = 0; q < 3; ++q) {
        EXPECT_EQ(load.force[q], 0.0f) << "force " << q << ": 0x" << std::hex << bits_of(load.force[q]);
        EXPECT_EQ(load.moment[q], 0.0f) << "moment " << q << ": 0x" << std::hex << bits_of(load.moment[q]);
    }
}

// publish_render_volume's two returns, an ok publish and a refused one (a
// solid mask of the wrong size), each from a caller rounding down: its word
// comes back exactly. No flush test: an f16 texel cannot show one (the plan's
// F2). Mutation: a hand-written restore on the ok path only.
TEST(AirflowFtz, PublishRestoresTheCallersWordOnEveryReturn) {
    MacGrid g = *make_grid({8, 8, 8, 0.125f}, DomainBc{});
    Lcg rng;
    for (float& x : g.u) x = 0.3f * rng.next();
    for (float& x : g.v) x = 0.3f * rng.next();
    for (float& x : g.w) x = 0.3f * rng.next();
    for (float& x : g.p) x = rng.next();
    const std::vector<uint8_t> wrong(7, 0);
    for (const bool refused : {false, true}) {
        FlowVolume vol;
        PublishInput in{};
        if (refused) in.solid = wrong;
        const unsigned saved = _mm_getcsr();
        _mm_setcsr(kCallerWord);
        const auto counts = publish_render_volume(g, in, vol);
        const unsigned after = _mm_getcsr();
        _mm_setcsr(saved);
        EXPECT_EQ(counts.has_value(), !refused);
        EXPECT_EQ(after, kCallerWord) << (refused ? "the refused return" : "the ok return");
    }
}

}  // namespace
