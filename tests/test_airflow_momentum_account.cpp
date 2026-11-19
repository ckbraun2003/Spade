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
    float swirl = 0.0f;  // N m about the jet's axis (axis 1 only): tangential forces on the u and w faces of the disc's layer
};

struct Engine {
    double impulse = 0.0;  // N s along y
    double momentum = 0.0;  // kg m/s along y
    float peak = 0.0f;     // the largest face speed at the end, m/s
    bool healthy = true;   // no non-finite value and no fault in any step
    double angular = 0.0;          // kg m^2/s about the first jet's axis (y), rho dV sum (dz u - dx w)
    double angular_impulse = 0.0;  // N m s: the sum over steps of h times the first jet's swirl
    double column_near = 0.0;      // m/s, the downward air in the first jet's disc column (r < 0.9 R) one cell under its layer
    double column_far = 0.0;       // and three cells under
    std::vector<float> peaks;      // the largest face speed every trace_every steps
};

// fluid_step()'s stages by hand (no solids) with the outlet rule of `st`: the sources the caller put in s.du, s.dv, s.dw are added after the advection.
FluidHealth step_by_stages(MacGrid& g, const DomainBc& bc, const FluidConfig& cfg, FluidScratch& s, const Stages& st) {
    const float h = cfg.h;
    outlet_faces(g, st.outlets == Outlets::ghost ? Outlets::copy_only : st.outlets);
    advect_velocity(g, s.next, bc, h, s.advect, s.pool.get());
    g.u.swap(s.next.u);
    g.v.swap(s.next.v);
    g.w.swap(s.next.w);
    std::vector<float>* vel[3] = {&g.u, &g.v, &g.w};
    std::vector<float>* inc[3] = {&s.du, &s.dv, &s.dw};
    for (int c = 0; c < 3; ++c) {
        for (std::size_t i = 0; i < vel[c]->size(); ++i) {
            (*vel[c])[i] += (*inc[c])[i];
            (*inc[c])[i] = 0.0f;
        }
    }
    outlet_faces(g, st.outlets);
    if (st.sgs_cs > 0.0f) {
        if (s.sgs.nut.size() != std::size_t{g.shape.nx} * g.shape.ny * g.shape.nz) sgs_size(s.sgs, g.shape);
        sgs_diffuse(g, bc, nullptr, nullptr, st.sgs_cs, h, s.sgs, s.pool.get());
        outlet_faces(g, st.outlets);
    }
    if (st.nu > 0.0f) {
        diffuse(g, bc, st.nu, h, cfg.visc_sweeps, s.rhs, s.rhs_uv, nullptr, s.pool.get());
        outlet_faces(g, st.outlets);
    }
    const ProjectionReport rep = project(g, bc, cfg.rho, h, cfg.n_v, s.projection, nullptr, nullptr, cfg.cg_cap, s.pool.get());
    FluidHealth health;
    health.pressure_residual = rep.residual;
    for (const std::vector<float>* c : {&g.u, &g.v, &g.w}) {
        for (const float x : *c) {
            if (!std::isfinite(x)) health.fault = FluidFault::non_finite;
        }
    }
    return health;
}

Engine run_engine(const std::vector<Jet>& jets, float h, double seconds, uint32_t n, float dx, uint32_t trace_every = 0, const Stages* stages = nullptr) {
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
    struct Swirl {  // the tangential forces of an axis-1 jet: u faces and w faces, each its own increment
        std::vector<std::size_t> u_faces, w_faces;
        std::vector<float> du, dw;
    };
    std::vector<Swirl> swirls(jets.size());
    std::vector<Source> sources;
    for (std::size_t ji = 0; ji < jets.size(); ++ji) {
        const Jet& j = jets[ji];
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
        if (j.axis == 1u && j.swirl != 0.0f) {
            // Force on the air (Fx, Fz) = q (dz, -dx) / r at each u and w face of the layer inside the radius: a torque q (dz^2 + dx^2) / r about y.
            Swirl& sw = swirls[ji];
            std::vector<float> fu, fw;
            double torque_per_q = 0.0;
            for (uint32_t k = 0; k < n; ++k) {
                for (uint32_t i = 0; i <= n; ++i) {  // u faces: x = i dx, z = (k + 0.5) dx
                    const float dxm = (static_cast<float>(i) - j.ca) * dx;
                    const float dzm = (static_cast<float>(k) + 0.5f - j.cb) * dx;
                    const float r = std::sqrt(dxm * dxm + dzm * dzm);
                    if (r > j.radius || r < 1.0e-6f) continue;
                    sw.u_faces.push_back(u_index(sh, i, j.layer, k));
                    fu.push_back(dzm / r);
                    torque_per_q += static_cast<double>(dzm) * dzm / r;
                }
            }
            for (uint32_t k = 0; k <= n; ++k) {
                for (uint32_t i = 0; i < n; ++i) {  // w faces: x = (i + 0.5) dx, z = k dx
                    const float dxm = (static_cast<float>(i) + 0.5f - j.ca) * dx;
                    const float dzm = (static_cast<float>(k) - j.cb) * dx;
                    const float r = std::sqrt(dxm * dxm + dzm * dzm);
                    if (r > j.radius || r < 1.0e-6f) continue;
                    sw.w_faces.push_back(w_index(sh, i, j.layer, k));
                    fw.push_back(-dxm / r);
                    torque_per_q += static_cast<double>(dxm) * dxm / r;
                }
            }
            const double q = static_cast<double>(j.swirl) / torque_per_q;
            const double to_dv = static_cast<double>(h) * q / (static_cast<double>(cfg.rho) * volume);
            for (const float f : fu) sw.du.push_back(static_cast<float>(to_dv * f));
            for (const float f : fw) sw.dw.push_back(static_cast<float>(to_dv * f));
        }
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
        for (const Swirl& sw : swirls) {
            for (std::size_t i = 0; i < sw.u_faces.size(); ++i) s.du[sw.u_faces[i]] += sw.du[i];
            for (std::size_t i = 0; i < sw.w_faces.size(); ++i) s.dw[sw.w_faces[i]] += sw.dw[i];
        }
        if (!jets.empty()) e.angular_impulse += static_cast<double>(h) * jets[0].swirl;
        const FluidHealth health = stages != nullptr ? step_by_stages(g, bc, cfg, s, *stages) : fluid_step(g, bc, cfg, s);
        if (health.fault != FluidFault::none) {
            e.healthy = false;
            return e;
        }
        if (trace_every != 0u && (t + 1u) % trace_every == 0u) {
            float pk = 0.0f;
            for (const std::vector<float>* c : {&g.u, &g.v, &g.w}) {
                for (const float x : *c) pk = std::max(pk, std::fabs(x));
            }
            e.peaks.push_back(pk);  // SPADE_MOMENTUM_TRACE=1: the largest face speed and where
            float peak = 0.0f;
            const char* where = "u";
            std::size_t at = 0;
            const std::vector<float>* comps[3] = {&g.u, &g.v, &g.w};
            const char* names[3] = {"u", "v", "w"};
            for (int c = 0; c < 3; ++c) {
                for (std::size_t i = 0; i < comps[c]->size(); ++i) {
                    if (std::fabs((*comps[c])[i]) > peak) {
                        peak = std::fabs((*comps[c])[i]);
                        where = names[c];
                        at = i;
                    }
                }
            }
            const uint32_t cx = std::string(where) == "u" ? n + 1u : n;
            const uint32_t cy = std::string(where) == "v" ? n + 1u : n;
            std::printf("[ momentum ] step %u: peak %.2f m/s in %s at (%zu, %zu, %zu), pressure residual %.2e\n", t + 1u, static_cast<double>(peak), where,
                        at % cx, (at / cx) % cy, at / (std::size_t{cx} * cy), static_cast<double>(health.pressure_residual));
        }
    }
    double sum = 0.0;
    for (uint32_t k = 0; k < n; ++k) {
        for (uint32_t j = 0; j < n; ++j) {
            for (uint32_t i = 0; i < n; ++i) sum += 0.5 * (static_cast<double>(g.v[v_index(sh, i, j, k)]) + g.v[v_index(sh, i, j + 1u, k)]);
        }
    }
    e.momentum = sum * static_cast<double>(cfg.rho) * volume;
    if (!jets.empty()) {
        const Jet& j0 = jets[0];
        double ang = 0.0;
        for (uint32_t k = 0; k < n; ++k) {
            for (uint32_t j = 0; j < n; ++j) {
                for (uint32_t i = 0; i < n; ++i) {
                    const double uc = 0.5 * (static_cast<double>(g.u[u_index(sh, i, j, k)]) + g.u[u_index(sh, i + 1u, j, k)]);
                    const double wc = 0.5 * (static_cast<double>(g.w[w_index(sh, i, j, k)]) + g.w[w_index(sh, i, j, k + 1u)]);
                    const double dxm = (static_cast<double>(i) + 0.5 - j0.ca) * dx;
                    const double dzm = (static_cast<double>(k) + 0.5 - j0.cb) * dx;
                    ang += dzm * uc - dxm * wc;
                }
            }
        }
        e.angular = ang * static_cast<double>(cfg.rho) * volume;
        for (int depth : {1, 3}) {
            const int jj = static_cast<int>(j0.layer) - depth;
            double acc = 0.0;
            uint32_t cnt = 0;
            for (uint32_t k = 0; k < n; ++k) {
                for (uint32_t i = 0; i < n; ++i) {
                    const double dxm = (static_cast<double>(i) + 0.5 - j0.ca) * dx;
                    const double dzm = (static_cast<double>(k) + 0.5 - j0.cb) * dx;
                    if (dxm * dxm + dzm * dzm > 0.81 * j0.radius * j0.radius) continue;
                    acc += -0.5 * (static_cast<double>(g.v[v_index(sh, i, static_cast<uint32_t>(jj), k)]) + g.v[v_index(sh, i, static_cast<uint32_t>(jj) + 1u, k)]);
                    ++cnt;
                }
            }
            (depth == 1 ? e.column_near : e.column_far) = cnt != 0u ? acc / cnt : 0.0;
        }
    }
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

// A fixed downward force in still air: the box holds the impulse put in, whatever the step. Until L90 it did not: the open faces copied their
// neighbour and refused inflow once a step (boundaries.cpp), the entrainment the disc draws was wiped each step and the box kept a smaller share the
// smaller h was (0.75, 0.62, 0.45 at h 0.4, 0.2, 0.1 ms here; 0.70, 0.56, 0.40 at h 0.2, 0.1, 0.05 ms on the 64^3 box of the diagnostic above). The
// share at h and at h/2 must agree within 1.5%.
TEST(AirflowMomentumAccount, TheBoxKeepsTheSameShareOfItsImpulseAtHAndAtHalfH) {
    const uint32_t n = 24;
    const float dx = 0.02f;
    const std::array<float, 3> steps = {0.0004f, 0.0002f, 0.0001f};
    std::array<double, 3> ratio{};
    for (std::size_t i = 0; i < steps.size(); ++i) {
        // Downward: thrust along -y; the disc centred in x and z at 0.7 of the height.
        const Engine e = run_engine({Jet{.axis = 1, .layer = 17, .ca = 12.0f, .cb = 12.0f, .radius = 0.127f, .thrust = -16.073f}}, steps[i], 0.02, n, dx);
        ASSERT_TRUE(e.healthy);
        ratio[i] = e.momentum / e.impulse;  // both negative: the force and the air go down
        std::printf("[ momentum ] h %.2f ms: the box holds %.4f of the impulse\n", static_cast<double>(steps[i]) * 1000.0, ratio[i]);
    }
    EXPECT_NEAR(ratio[0], ratio[1], 0.015 * ratio[1]) << "h 0.4 ms against 0.2 ms";
    EXPECT_NEAR(ratio[1], ratio[2], 0.015 * ratio[2]) << "h 0.2 ms against 0.1 ms";
}

// The open faces take inflow (L90), so a backflow instability is the risk, and two cases that could grow one stay bounded at h 0.2 ms, the largest
// step the solver is run at: a jet that leaves through a face at an angle, and a disc close to a side face that draws its air in through it. 450
// steps (0.09 s; 600 took 36 s in debug under ctest, over the 30 s kept), the largest face speed sampled every 75: under 40 m/s (the forcing disc's own faces carry about 34, a 14 N disc on a coarse grid),
// and the last half no higher than 1.1 times the first. The same jet run to 3000 steps under each outlet rule is the opt-in test below; copying the
// neighbour's face instead of leaving it, or no ambient ghost in the advection, gives 42 to 59 m/s there, noisy and not bounded.
void expect_bounded(const Engine& e) {
    EXPECT_TRUE(e.healthy);
    ASSERT_EQ(e.peaks.size(), 6u);
    float first = 0.0f;
    float last = 0.0f;
    for (std::size_t i = 0; i < e.peaks.size(); ++i) {
        EXPECT_LT(e.peaks[i], 40.0f) << "sample " << i;
        (i < 3 ? first : last) = std::max(i < 3 ? first : last, e.peaks[i]);
    }
    EXPECT_LE(last, 1.1f * first) << "the face speed grows";
}

TEST(AirflowMomentumAccount, AJetLeavingThroughAFaceAtAnAngleStaysBounded) {
    // A disc 4 cells from the x+ face pushing +x, and a downward disc at the face: the jet leaves through x+ at an angle.
    const std::vector<Jet> jets = {Jet{.axis = 0, .layer = 20, .ca = 12.0f, .cb = 12.0f, .radius = 0.127f, .thrust = 14.0f},
                                   Jet{.axis = 1, .layer = 12, .ca = 20.0f, .cb = 12.0f, .radius = 0.127f, .thrust = -8.0f}};
    expect_bounded(run_engine(jets, 0.0002f, 0.09, 24, 0.02f, 75u));
}

TEST(AirflowMomentumAccount, ADiscDrawingItsAirInThroughASideFaceStaysBounded) {
    // A downward disc whose edge is under a cell from the x- face: the side face is the nearest source of the air it entrains.
    expect_bounded(run_engine({Jet{.axis = 1, .layer = 17, .ca = 7.0f, .cb = 12.0f, .radius = 0.127f, .thrust = -16.073f}}, 0.0002f, 0.09, 24, 0.02f, 75u));
}

// Opt-in (SPADE_MOMENTUM_ACCOUNT=1): the two backflow cases and the 32^3 momentum account under each outlet rule, to choose the fix's rule.
TEST(AirflowMomentumAccount, TheOutletRulesOnTheBackflowCases) {
    if (std::getenv("SPADE_MOMENTUM_ACCOUNT") == nullptr) GTEST_SKIP() << "opt-in: SPADE_MOMENTUM_ACCOUNT=1 (CPU, minutes)";
    struct Rule {
        const char* name;
        Outlets outlets;
    };
    const std::array<Rule, 4> rules = {{{"the old rule (copy, refuse inflow)", Outlets::engine},
                                        {"copy, inflow taken", Outlets::copy_only},
                                        {"outlet faces untouched", Outlets::untouched},
                                        {"copy before the advection only", Outlets::ghost}}};
    const std::vector<Jet> angled = {Jet{.axis = 0, .layer = 28, .ca = 16.0f, .cb = 16.0f, .radius = 0.127f, .thrust = 14.0f},
                                     Jet{.axis = 1, .layer = 16, .ca = 28.0f, .cb = 16.0f, .radius = 0.127f, .thrust = -8.0f}};
    const std::vector<Jet> side = {Jet{.axis = 1, .layer = 22, .ca = 9.0f, .cb = 16.0f, .radius = 0.127f, .thrust = -16.073f}};
    const std::vector<Jet> down = {Jet{.axis = 1, .layer = 22, .ca = 16.0f, .cb = 16.0f, .radius = 0.127f, .thrust = -16.073f}};
    for (const Rule& r : rules) {
        const Stages st{r.outlets, true, 0.17f, 1.5e-5f};
        const Engine a = run_engine(angled, 0.0002f, 0.1, 32, 0.02f, 0u, &st);
        const Engine b = run_engine(side, 0.0002f, 0.1, 32, 0.02f, 0u, &st);
        const Engine c1 = run_engine(down, 0.0004f, 0.02, 32, 0.02f, 0u, &st);
        const Engine c2 = run_engine(down, 0.0002f, 0.02, 32, 0.02f, 0u, &st);
        const Engine c3 = run_engine(down, 0.0001f, 0.02, 32, 0.02f, 0u, &st);
        std::printf("[ momentum ] %s: angled jet peak %.1f m/s (healthy %d), side entrainment peak %.1f m/s (healthy %d), share at h 0.4/0.2/0.1 ms %.3f %.3f %.3f\n",
                    r.name, static_cast<double>(a.peak), a.healthy ? 1 : 0, static_cast<double>(b.peak), b.healthy ? 1 : 0, c1.momentum / c1.impulse,
                    c2.momentum / c2.impulse, c3.momentum / c3.impulse);
    }
    SUCCEED();
}
// Opt-in: the angled jet for 3000 steps at h 0.2 ms under each outlet rule, the largest face speed every 250 steps: bounded means no growth over the
// last half of the run, not one sample under a limit.
TEST(AirflowMomentumAccount, TheAngledJetOverThreeThousandSteps) {
    if (std::getenv("SPADE_MOMENTUM_ACCOUNT") == nullptr) GTEST_SKIP() << "opt-in: SPADE_MOMENTUM_ACCOUNT=1 (CPU, minutes)";
    struct Rule {
        const char* name;
        Outlets outlets;
    };
    const std::array<Rule, 3> rules = {{{"the old rule (copy, refuse inflow)", Outlets::engine},
                                        {"copy, inflow taken", Outlets::copy_only},
                                        {"outlet faces untouched", Outlets::untouched}}};
    const std::vector<Jet> angled = {Jet{.axis = 0, .layer = 28, .ca = 16.0f, .cb = 16.0f, .radius = 0.127f, .thrust = 14.0f},
                                     Jet{.axis = 1, .layer = 16, .ca = 28.0f, .cb = 16.0f, .radius = 0.127f, .thrust = -8.0f}};
    for (const Rule& r : rules) {
        std::printf("[ momentum ] %s\n", r.name);
        const Stages st{r.outlets, true, 0.17f, 1.5e-5f};
        run_engine(angled, 0.0002f, 0.6, 32, 0.02f, 250u, &st);
    }
    SUCCEED();
}
// Opt-in: the X500-like case (a swirling downward disc, in the middle and with its edge two cells from a side face) under each outlet rule: the air
// through its disc column, and the angular momentum in the box against the angular impulse put in (what leaves through the side faces, or comes in
// through them with the wake's swirl). h 0.2 and 0.1 ms to 0.03 s.
TEST(AirflowMomentumAccount, TheSwirlingDiscUnderEachOutletRule) {
    if (std::getenv("SPADE_MOMENTUM_ACCOUNT") == nullptr) GTEST_SKIP() << "opt-in: SPADE_MOMENTUM_ACCOUNT=1 (CPU, minutes)";
    struct Rule {
        const char* name;
        Outlets outlets;
    };
    const std::array<Rule, 3> rules = {{{"old (copy, refuse inflow)", Outlets::engine}, {"copy, inflow taken", Outlets::copy_only}, {"outlet faces untouched", Outlets::untouched}}};
    for (const float ca : {16.0f, 9.0f}) {
        for (const Rule& r : rules) {
            const Stages st{r.outlets, true, 0.17f, 1.5e-5f};
            for (const float h : {0.0002f, 0.0001f}) {
                const Engine e = run_engine({Jet{.axis = 1, .layer = 22, .ca = ca, .cb = 16.0f, .radius = 0.127f, .thrust = -16.073f, .swirl = 0.06f}}, h, 0.03, 32,
                                            0.02f, 0u, &st);
                std::printf("[ momentum ] disc at %s, %s, h %.1f ms: share of the impulse %.3f, angular share %.3f, column 1 cell under %.3f m/s, 3 cells under %.3f m/s, peak %.1f\n",
                            ca == 16.0f ? "the middle" : "a side face", r.name, static_cast<double>(h) * 1000.0, e.momentum / e.impulse,
                            e.angular / e.angular_impulse, e.column_near, e.column_far, static_cast<double>(e.peak));
            }
        }
    }
    SUCCEED();
}
#if defined(_MSC_VER)
#pragma warning(pop)
#endif
