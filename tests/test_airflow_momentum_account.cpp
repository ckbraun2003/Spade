// L86p: the CPU step's momentum account against the time step. A fixed downward force on a disc of faces in still air in an open box: the
// impulse put in is the sum over steps of h times the force, and the box's momentum is rho times the sum of the cells' velocity. The GPU step
// on the X500 loses about 7e-4 of the momentum per step at every h, so what the box holds of the impulse falls as h shrinks
// (physics/x500-diag, ecf56f8a); this test runs the reference step's stages by hand, each one switchable, to find the stage.
// Found: apply_velocity_boundaries() on the pressure-outlet faces (boundaries.cpp), applied once a step. The split below tells which of its two
// behaviours: the copy of the neighbouring face, or the refusal of inflow.
// Opt-in: SPADE_MOMENTUM_ACCOUNT=1 prints the table (a few CPU minutes).
#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

#include "physics/airflow/advect.hpp"
#include "physics/airflow/boundaries.hpp"
#include "physics/airflow/fluid_step.hpp"
#include "physics/airflow/projection.hpp"
#include "physics/airflow/sgs.hpp"
#include "physics/airflow/viscous.hpp"

namespace {

using namespace spade::physics::airflow;

// What the outlet faces get before and after the advection. `engine` is apply_velocity_boundaries()'s own rule for a still-air box (every face
// an outlet, ambient 0): the face takes a copy of the interior face next to it, and a copy pointing into the box is refused (set to 0).
// `ghost` is the fix candidate: the zero-gradient copy (inflow allowed) is made only before the advection, which reads the box's faces as its ghost
// values, and no stage overwrites an outlet face after: the projection's p = 0 sets it.
enum class Outlets { engine, copy_only, refusal_only, untouched, ghost };

template <class At>
void outlet_axis(Outlets m, uint32_t n, uint32_t m1, uint32_t m2, At at) noexcept {
    for (uint32_t b = 0; b < m2; ++b) {
        for (uint32_t a = 0; a < m1; ++a) {
            const float next_lo = at(1, a, b);
            const float next_hi = at(n - 1u, a, b);
            switch (m) {
                case Outlets::engine:
                    at(0, a, b) = next_lo > 0.0f ? 0.0f : next_lo;
                    at(n, a, b) = next_hi < 0.0f ? 0.0f : next_hi;
                    break;
                case Outlets::copy_only:
                    at(0, a, b) = next_lo;
                    at(n, a, b) = next_hi;
                    break;
                case Outlets::refusal_only:
                    if (at(0, a, b) > 0.0f) at(0, a, b) = 0.0f;
                    if (at(n, a, b) < 0.0f) at(n, a, b) = 0.0f;
                    break;
                case Outlets::untouched:
                case Outlets::ghost:
                    break;
            }
        }
    }
}

void outlet_faces(MacGrid& g, Outlets m) noexcept {
    const GridShape& s = g.shape;
    outlet_axis(m, s.nx, s.ny, s.nz, [&](uint32_t q, uint32_t a, uint32_t b) -> float& { return g.u[u_index(s, q, a, b)]; });
    outlet_axis(m, s.ny, s.nx, s.nz, [&](uint32_t q, uint32_t a, uint32_t b) -> float& { return g.v[v_index(s, a, q, b)]; });
    outlet_axis(m, s.nz, s.nx, s.ny, [&](uint32_t q, uint32_t a, uint32_t b) -> float& { return g.w[w_index(s, a, b, q)]; });
}

struct Stages {
    Outlets outlets = Outlets::engine;
    bool advect = true;
    float sgs_cs = 0.17f;
    float nu = 1.5e-5f;
};

struct Account {
    double impulse = 0.0;   // N s
    double momentum = 0.0;  // kg m/s, downward
    double ratio = 0.0;
};

// The reference step with each stage switchable (fluid_step.cpp's step_flushed() without solids), the force added where the engine adds du/dv/dw.
Account run(const Stages& st, float h, double seconds, uint32_t n, float dx, float thrust, float disc_radius) {
    const DomainBc bc;  // all six faces open: in still air, pressure outlets
    MacGrid g = *make_grid({n, n, n, dx}, bc);
    FluidConfig cfg{.rho = 1.225f, .nu = st.nu, .h = h};
    cfg.sgs_cs = st.sgs_cs;
    cfg.n_v = 8;
    FluidScratch s = *make_fluid_scratch(g, bc, cfg);
    const GridShape& sh = g.shape;
    // The disc: v faces of one layer inside the radius, at 0.7 of the box's height, centred in x and z.
    const uint32_t layer = static_cast<uint32_t>(0.7f * static_cast<float>(n));
    const float centre = 0.5f * dx * static_cast<float>(n);
    std::vector<std::size_t> faces;
    for (uint32_t k = 0; k < n; ++k) {
        for (uint32_t i = 0; i < n; ++i) {
            const float x = (static_cast<float>(i) + 0.5f) * dx - centre;
            const float z = (static_cast<float>(k) + 0.5f) * dx - centre;
            if (x * x + z * z <= disc_radius * disc_radius) faces.push_back(v_index(sh, i, layer, k));
        }
    }
    const double volume = static_cast<double>(dx) * dx * dx;
    // Every face takes dv so that rho dx^3 sum(dv) = h F.
    const float dv = -static_cast<float>(static_cast<double>(h) * thrust / (static_cast<double>(cfg.rho) * volume * static_cast<double>(faces.size())));
    const uint32_t steps = static_cast<uint32_t>(std::lround(seconds / static_cast<double>(h)));
    Account a;
    for (uint32_t t = 0; t < steps; ++t) {
        outlet_faces(g, st.outlets == Outlets::ghost ? Outlets::copy_only : st.outlets);
        if (st.advect) {
            advect_velocity(g, s.next, bc, h, s.advect, s.pool.get());
            g.u.swap(s.next.u);
            g.v.swap(s.next.v);
            g.w.swap(s.next.w);
        }
        for (const std::size_t f : faces) g.v[f] += dv;
        a.impulse += static_cast<double>(h) * thrust;
        outlet_faces(g, st.outlets);
        if (st.sgs_cs > 0.0f) {
            if (s.sgs.nut.size() != std::size_t{n} * n * n) sgs_size(s.sgs, sh);
            sgs_diffuse(g, bc, nullptr, nullptr, st.sgs_cs, h, s.sgs, s.pool.get());
            outlet_faces(g, st.outlets);
        }
        if (st.nu > 0.0f) {
            diffuse(g, bc, st.nu, h, cfg.visc_sweeps, s.rhs, s.rhs_uv, nullptr, s.pool.get());
            outlet_faces(g, st.outlets);
        }
        project(g, bc, cfg.rho, h, cfg.n_v, s.projection, nullptr, nullptr, cfg.cg_cap, s.pool.get());
    }
    double sum = 0.0;
    for (uint32_t k = 0; k < n; ++k) {
        for (uint32_t j = 0; j < n; ++j) {
            for (uint32_t i = 0; i < n; ++i) sum += 0.5 * (static_cast<double>(g.v[v_index(sh, i, j, k)]) + g.v[v_index(sh, i, j + 1u, k)]);
        }
    }
    a.momentum = -sum * static_cast<double>(cfg.rho) * volume;
    a.ratio = a.impulse > 0.0 ? a.momentum / a.impulse : 0.0;
    return a;
}

// The engine's own step (fluid_step) with fixed forces on discs of faces: the pins below. A Jet is a disc of one component's faces, the
// disc's plane normal to that component's axis at `layer`, centred across it at (ca, cb) in cells, radius in metres; `thrust` is its whole
// force in newtons along +axis, put in where the engine puts a source (s.du, s.dv, s.dw), so the step adds rho dx^3 dv = h F after its advection.
struct Jet {
    uint32_t axis = 1;
    uint32_t layer = 0;
    float ca = 0.0f;
    float cb = 0.0f;
    float radius = 0.127f;
    float thrust = 0.0f;
};

struct Engine {
    double impulse = 0.0;  // N s along y
    double momentum = 0.0;  // kg m/s along y
    float peak = 0.0f;     // the largest face speed at the end, m/s
    bool healthy = true;   // no non-finite value and no fault in any step
};

Engine run_engine(const std::vector<Jet>& jets, float h, double seconds, uint32_t n, float dx) {
    const DomainBc bc;  // all six faces open: in still air, pressure outlets
    MacGrid g = *make_grid({n, n, n, dx}, bc);
    FluidConfig cfg{.rho = 1.225f, .nu = 1.5e-5f, .h = h};
    cfg.sgs_cs = 0.17f;
    cfg.n_v = 8;
    FluidScratch s = *make_fluid_scratch(g, bc, cfg);
    const GridShape& sh = g.shape;
    const double volume = static_cast<double>(dx) * dx * dx;
    struct Source {
        uint32_t axis;
        std::vector<std::size_t> faces;
        float dv;
        float thrust;
    };
    std::vector<Source> sources;
    for (const Jet& j : jets) {
        Source src{j.axis, {}, 0.0f, j.thrust};
        for (uint32_t b2 = 0; b2 < n; ++b2) {
            for (uint32_t a2 = 0; a2 < n; ++a2) {
                const float da = (static_cast<float>(a2) + 0.5f - j.ca) * dx;
                const float db = (static_cast<float>(b2) + 0.5f - j.cb) * dx;
                if (da * da + db * db > j.radius * j.radius) continue;
                // The two axes across: x, z for a y jet; y, z for an x jet; x, y for a z jet.
                if (j.axis == 1u) src.faces.push_back(v_index(sh, a2, j.layer, b2));
                else if (j.axis == 0u) src.faces.push_back(u_index(sh, j.layer, a2, b2));
                else src.faces.push_back(w_index(sh, a2, b2, j.layer));
            }
        }
        src.dv = static_cast<float>(static_cast<double>(h) * j.thrust / (static_cast<double>(cfg.rho) * volume * static_cast<double>(src.faces.size())));
        sources.push_back(std::move(src));
    }
    const uint32_t steps = static_cast<uint32_t>(std::lround(seconds / static_cast<double>(h)));
    Engine e;
    for (uint32_t t = 0; t < steps; ++t) {
        for (const Source& src : sources) {
            std::vector<float>& d = src.axis == 0u ? s.du : (src.axis == 1u ? s.dv : s.dw);
            for (const std::size_t f : src.faces) d[f] += src.dv;
            if (src.axis == 1u) e.impulse += static_cast<double>(h) * src.thrust;
        }
        const FluidHealth health = fluid_step(g, bc, cfg, s);
        if (health.fault != FluidFault::none) {
            e.healthy = false;
            return e;
        }
    }
    double sum = 0.0;
    for (uint32_t k = 0; k < n; ++k) {
        for (uint32_t j = 0; j < n; ++j) {
            for (uint32_t i = 0; i < n; ++i) sum += 0.5 * (static_cast<double>(g.v[v_index(sh, i, j, k)]) + g.v[v_index(sh, i, j + 1u, k)]);
        }
    }
    e.momentum = sum * static_cast<double>(cfg.rho) * volume;
    for (const std::vector<float>* c : {&g.u, &g.v, &g.w}) {
        for (const float x : *c) e.peak = std::max(e.peak, std::fabs(x));
    }
    return e;
}

}  // namespace

#if defined(_MSC_VER)
#pragma warning(push)
#pragma warning(disable : 4996)  // getenv
#endif
TEST(AirflowMomentumAccount, TheCpuStepsStagesAgainstTheTimeStep) {
    if (std::getenv("SPADE_MOMENTUM_ACCOUNT") == nullptr) GTEST_SKIP() << "opt-in: SPADE_MOMENTUM_ACCOUNT=1 (CPU, minutes)";
    struct Variant {
        const char* name;
        Stages st;
    };
    const std::array<Variant, 5> variants = {{
        {"the engine's rule (copy, refuse inflow)", {Outlets::engine, true, 0.17f, 1.5e-5f}},
        {"copy only", {Outlets::copy_only, true, 0.17f, 1.5e-5f}},
        {"refusal only", {Outlets::refusal_only, true, 0.17f, 1.5e-5f}},
        {"outlet faces untouched", {Outlets::untouched, true, 0.17f, 1.5e-5f}},
        {"the fix candidate: the copy only as the advection's ghost", {Outlets::ghost, true, 0.17f, 1.5e-5f}},
    }};
    const std::array<float, 3> steps = {0.0002f, 0.0001f, 0.00005f};
    const uint32_t n = 64;
    const float dx = 0.01f;
    for (const Variant& v : variants) {
        std::printf("[ momentum ] %s\n", v.name);
        for (const float h : steps) {
            const Account a = run(v.st, h, 0.03, n, dx, 16.073f, 0.127f);
            std::printf("[ momentum ]   h %.3f ms: box momentum %.5f kg m/s of the impulse %.5f N s: ratio %.4f\n", static_cast<double>(h) * 1000.0,
                        a.momentum, a.impulse, a.ratio);
        }
    }
    SUCCEED();
}

// A fixed downward force in still air: the box holds the impulse put in, whatever the step. At this commit it does not: the open faces refuse
// inflow once a step (boundaries.cpp), the entrainment the disc draws is wiped each step and the box keeps a smaller share the smaller h is
// (0.70, 0.56, 0.40 at h 0.2, 0.1, 0.05 ms on the 64^3 box of the diagnostic above). The ratio at h and at h/2 must agree within 1.5%.
TEST(AirflowMomentumAccount, TheBoxKeepsTheSameShareOfItsImpulseAtHAndAtHalfH) {
    const uint32_t n = 32;
    const float dx = 0.02f;
    const std::array<float, 3> steps = {0.0004f, 0.0002f, 0.0001f};
    std::array<double, 3> ratio{};
    for (std::size_t i = 0; i < steps.size(); ++i) {
        // Downward: thrust along -y; the disc centred in x and z at 0.7 of the height.
        const Engine e = run_engine({Jet{.axis = 1, .layer = 22, .ca = 16.0f, .cb = 16.0f, .radius = 0.127f, .thrust = -16.073f}}, steps[i], 0.02, n, dx);
        ASSERT_TRUE(e.healthy);
        ratio[i] = e.momentum / e.impulse;  // both negative: the force and the air go down
        std::printf("[ momentum ] h %.2f ms: the box holds %.4f of the impulse\n", static_cast<double>(steps[i]) * 1000.0, ratio[i]);
    }
    EXPECT_NEAR(ratio[0], ratio[1], 0.015 * ratio[1]) << "h 0.4 ms against 0.2 ms";
    EXPECT_NEAR(ratio[1], ratio[2], 0.015 * ratio[2]) << "h 0.2 ms against 0.1 ms";
}

// The refusal of backflow is gone, so the open faces take inflow: two cases that could blow up an open boundary stay bounded at h 0.2 ms, the
// largest step the solver is run at: a jet that leaves through a face at an angle, and a disc close to a side face that draws its air in
// through it. Run to 0.1 s (500 steps); the largest face speed is under 30 m/s (a 16 N disc's wake is about 10 m/s) and no step faults.
TEST(AirflowMomentumAccount, AJetLeavingThroughAFaceAtAnAngleStaysBounded) {
    // A disc 4 cells from the x+ face pushing +x with a downward share: leaves through x+ at about 30 degrees below the axis.
    const std::vector<Jet> jets = {Jet{.axis = 0, .layer = 28, .ca = 16.0f, .cb = 16.0f, .radius = 0.127f, .thrust = 14.0f},
                                   Jet{.axis = 1, .layer = 16, .ca = 28.0f, .cb = 16.0f, .radius = 0.127f, .thrust = -8.0f}};
    const Engine e = run_engine(jets, 0.0002f, 0.1, 32, 0.02f);
    EXPECT_TRUE(e.healthy);
    EXPECT_LT(e.peak, 30.0f);
}

TEST(AirflowMomentumAccount, ADiscDrawingItsAirInThroughASideFaceStaysBounded) {
    // A downward disc whose edge is 2 cells from the x- face: the side face is the nearest source of the air it entrains.
    const Engine e = run_engine({Jet{.axis = 1, .layer = 22, .ca = 9.0f, .cb = 16.0f, .radius = 0.127f, .thrust = -16.073f}}, 0.0002f, 0.1, 32, 0.02f);
    EXPECT_TRUE(e.healthy);
    EXPECT_LT(e.peak, 30.0f);
}
#if defined(_MSC_VER)
#pragma warning(pop)
#endif
