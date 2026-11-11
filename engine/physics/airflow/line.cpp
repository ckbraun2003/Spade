#include "physics/airflow/line.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <format>
#include <numbers>
#include <string>

#include "core/detail/flush_subnormals.hpp"
#include "core/fp32_math.hpp"

namespace spade::physics::airflow {

namespace {

constexpr float kPi = std::numbers::pi_v<float>;
constexpr int kReach = 6;  // the kernel's half-width in cells: 3 eps with eps 2 dx
constexpr int kMaxTaps = 2 * kReach + 1;

Error refuse(std::string why) { return Error{Code::invalid_argument, std::move(why)}; }

bool finite3(glm::vec3 v) noexcept { return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z); }

// One axis of one component's stencil: the faces lo .. lo + n - 1 and their weights exp(-((x_i - pos) / eps)^2).
struct AxisTaps {
    int lo = 0;
    int n = 0;
    float sum = 0.0f;
    std::array<float, kMaxTaps> w{};
};

AxisTaps axis_taps(float pos, float dx, float offset, uint32_t count) noexcept {
    AxisTaps t;
    const float f = pos / dx - offset;  // the point in this component's face index
    const int lo = std::max(0, static_cast<int>(-std::floor(-(f - static_cast<float>(kReach)))));
    const int hi = std::min(static_cast<int>(count) - 1, static_cast<int>(std::floor(f + static_cast<float>(kReach))));
    if (hi < lo) return t;
    t.lo = lo;
    t.n = hi - lo + 1;
    for (int i = 0; i < t.n; ++i) {
        const float q = (static_cast<float>(lo + i) - f) * 0.5f;  // (x_i - pos) / eps, eps = 2 dx
        t.w[static_cast<std::size_t>(i)] = math::exp32(-(q * q));
        t.sum += t.w[static_cast<std::size_t>(i)];
    }
    return t;
}

// The stencil of component c (0 u, 1 v, 2 w) at a point; sum is the weights' sum, the product of the axes' sums.
struct Stencil {
    std::array<AxisTaps, 3> ax;
    float sum = 0.0f;
};

Stencil stencil_at(const GridShape& s, uint32_t c, glm::vec3 pos) noexcept {
    const std::array<uint32_t, 3> n{s.nx + (c == 0 ? 1u : 0u), s.ny + (c == 1 ? 1u : 0u), s.nz + (c == 2 ? 1u : 0u)};
    Stencil st;
    for (uint32_t a = 0; a < 3; ++a) {
        const float offset = a == c ? 0.0f : 0.5f;
        st.ax[a] = axis_taps(pos[static_cast<glm::length_t>(a)], s.dx, offset, n[a]);
    }
    st.sum = st.ax[0].sum * st.ax[1].sum * st.ax[2].sum;
    return st;
}

std::size_t face_index(const GridShape& s, uint32_t c, int i, int j, int k) noexcept {
    const auto ui = static_cast<uint32_t>(i);
    const auto uj = static_cast<uint32_t>(j);
    const auto uk = static_cast<uint32_t>(k);
    return c == 0 ? u_index(s, ui, uj, uk) : c == 1 ? v_index(s, ui, uj, uk) : w_index(s, ui, uj, uk);
}

// The kernel-weighted mean of a component's faces; fixed order k, j, i.
float stencil_mean(const GridShape& s, uint32_t c, const Stencil& st, const std::vector<float>& data) noexcept {
    if (!(st.sum > 0.0f)) return 0.0f;
    float acc = 0.0f;
    for (int k = 0; k < st.ax[2].n; ++k) {
        for (int j = 0; j < st.ax[1].n; ++j) {
            const float wjk = st.ax[1].w[static_cast<std::size_t>(j)] * st.ax[2].w[static_cast<std::size_t>(k)];
            for (int i = 0; i < st.ax[0].n; ++i) {
                const std::size_t f = face_index(s, c, st.ax[0].lo + i, st.ax[1].lo + j, st.ax[2].lo + k);
                acc += st.ax[0].w[static_cast<std::size_t>(i)] * wjk * data[f];
            }
        }
    }
    return acc / st.sum;
}

// Adds amount * w_i / sum to every face of the stencil (amount: the component's whole increment: impulse over rho dx^3).
void stencil_add(const GridShape& s, uint32_t c, const Stencil& st, float amount, std::span<float> out) noexcept {
    if (!(st.sum > 0.0f) || amount == 0.0f) return;
    const float coef = amount / st.sum;
    for (int k = 0; k < st.ax[2].n; ++k) {
        for (int j = 0; j < st.ax[1].n; ++j) {
            const float wjk = st.ax[1].w[static_cast<std::size_t>(j)] * st.ax[2].w[static_cast<std::size_t>(k)];
            for (int i = 0; i < st.ax[0].n; ++i) {
                const std::size_t f = face_index(s, c, st.ax[0].lo + i, st.ax[1].lo + j, st.ax[2].lo + k);
                out[f] += coef * (st.ax[0].w[static_cast<std::size_t>(i)] * wjk);
            }
        }
    }
}

uint32_t elements_per_blade(const BladeShape& b, float dx) noexcept {
    const float span = b.radius - b.stations.front().r;
    return std::max(1u, static_cast<uint32_t>(-std::floor(-(span / dx))));
}

// The chord and pitch (degrees) at r, linear between stations, held outside them.
void station_at(const BladeShape& b, float r, float& chord, float& beta_deg) noexcept {
    const auto& st = b.stations;
    if (r <= st.front().r) {
        chord = st.front().chord;
        beta_deg = st.front().beta_deg;
        return;
    }
    for (std::size_t i = 1; i < st.size(); ++i) {
        if (r <= st[i].r) {
            const float t = (r - st[i - 1].r) / (st[i].r - st[i - 1].r);
            chord = st[i - 1].chord + t * (st[i].chord - st[i - 1].chord);
            beta_deg = st[i - 1].beta_deg + t * (st[i].beta_deg - st[i - 1].beta_deg);
            return;
        }
    }
    chord = st.back().chord;
    beta_deg = st.back().beta_deg;
}

}  // namespace

Result<void> check_line(const LineRotor& rotor, const GridShape& shape, float rho, float h, uint32_t n_sub) {
    if (rotor.blade == nullptr) return std::unexpected(refuse("line: no blade"));
    if (rotor.polar == nullptr) return std::unexpected(refuse("line: no polar"));
    if (auto b = check_blade(*rotor.blade); !b) return std::unexpected(b.error());
    if (!std::isfinite(rotor.omega) || rotor.omega < 0.0f) {
        return std::unexpected(refuse("line: omega must be finite and >= 0"));
    }
    if (rotor.spin != 1.0f && rotor.spin != -1.0f) return std::unexpected(refuse("line: spin must be +1 or -1"));
    if (!finite3(rotor.axis) || std::fabs(glm::length(rotor.axis) - 1.0f) > 1e-3f) {
        return std::unexpected(refuse("line: axis must be a unit vector"));
    }
    if (!finite3(rotor.ref) || std::fabs(glm::length(rotor.ref) - 1.0f) > 1e-3f ||
        std::fabs(glm::dot(rotor.ref, rotor.axis)) > 1e-3f) {
        return std::unexpected(refuse("line: ref must be a unit vector perpendicular to the axis"));
    }
    if (!std::isfinite(rotor.psi) || std::fabs(rotor.psi) > 50.0f) {
        return std::unexpected(refuse("line: psi must be finite and wrapped (|psi| <= 50 rad, sin32's domain)"));
    }
    if (!finite3(rotor.centre)) return std::unexpected(refuse("line: centre must be finite"));
    if (n_sub == 0) return std::unexpected(refuse("line: n_sub must be at least 1"));
    if (!(rho > 0.0f) || !std::isfinite(rho)) return std::unexpected(refuse("line: rho must be positive and finite"));
    if (!(h > 0.0f) || !std::isfinite(h)) return std::unexpected(refuse("line: h must be positive and finite"));
    if (!(shape.dx > 0.0f) || shape.nx == 0 || shape.ny == 0 || shape.nz == 0) {
        return std::unexpected(refuse("line: grid must have cells and a positive dx"));
    }
    const float reach = rotor.blade->radius + line_margin(shape.dx);
    const std::array<float, 3> extent{static_cast<float>(shape.nx) * shape.dx, static_cast<float>(shape.ny) * shape.dx,
                                      static_cast<float>(shape.nz) * shape.dx};
    for (glm::length_t a = 0; a < 3; ++a) {
        const float c = rotor.centre[a];
        if (c - reach < 0.0f || c + reach > extent[static_cast<std::size_t>(a)]) {
            return std::unexpected(refuse(std::format("line: the kernel's reach (R + 6 dx = {:.4g} m) leaves the grid on "
                                                      "axis {}: centre {:.4g} m in [0, {:.4g}]",
                                                      reach, static_cast<int>(a), c, extent[static_cast<std::size_t>(a)])));
        }
    }
    return {};
}

std::vector<LineElement> line_elements(const LineRotor& rotor, float dx, float h, uint32_t n_sub) {
    const BladeShape& blade = *rotor.blade;
    const uint32_t per = elements_per_blade(blade, dx);
    const float r0 = blade.stations.front().r;
    const float dr = (blade.radius - r0) / static_cast<float>(per);
    const glm::vec3 side = glm::cross(rotor.axis, rotor.ref);  // azimuth 90 degrees
    std::vector<LineElement> out;
    out.reserve(std::size_t{n_sub} * blade.blades * per);
    for (uint32_t k = 0; k < n_sub; ++k) {
        const float psi_k = rotor.psi + rotor.omega * h * (static_cast<float>(k) + 0.5f) / static_cast<float>(n_sub);
        for (uint32_t b = 0; b < blade.blades; ++b) {
            const float phi = rotor.spin * psi_k + 2.0f * kPi * static_cast<float>(b) / static_cast<float>(blade.blades);
            const float c = math::cos32(phi);
            const float s = math::sin32(phi);
            const glm::vec3 d = rotor.ref * c + side * s;
            const glm::vec3 m = rotor.spin * glm::cross(rotor.axis, d);
            for (uint32_t e = 0; e < per; ++e) {
                LineElement el;
                el.r = r0 + (static_cast<float>(e) + 0.5f) * dr;
                el.dr = dr;
                el.pos = rotor.centre + d * el.r;
                el.motion = m;
                float beta_deg = 0.0f;
                station_at(blade, el.r, el.chord, beta_deg);
                const float beta = beta_deg * kPi / 180.0f;
                el.sin_beta = math::sin32(beta);
                el.cos_beta = math::cos32(beta);
                out.push_back(el);
            }
        }
    }
    return out;
}

glm::vec3 line_sample(const MacGrid& grid, glm::vec3 pos) noexcept {
    const Stencil su = stencil_at(grid.shape, 0, pos);
    const Stencil sv = stencil_at(grid.shape, 1, pos);
    const Stencil sw = stencil_at(grid.shape, 2, pos);
    return {stencil_mean(grid.shape, 0, su, grid.u), stencil_mean(grid.shape, 1, sv, grid.v),
            stencil_mean(grid.shape, 2, sw, grid.w)};
}

namespace {

// spread_line()'s body, under its wrapper's scope (core/detail/flush_subnormals.hpp's wrapper rule). A face's sum takes its elements
// in element order, the order a per-face gather over them gives, so the device twin reproduces this scatter.
SPADE_NOINLINE LineReport spread_flushed(const LineRotor& rotor, const MacGrid& grid, float rho, float h, uint32_t n_sub,
                                         std::span<float> du, std::span<float> dv, std::span<float> dw) {
    const GridShape& s = grid.shape;
    const float dx3 = s.dx * s.dx * s.dx;
    const float share = 1.0f / static_cast<float>(n_sub);
    double thrust = 0.0;
    double torque = 0.0;
    for (const LineElement& e : line_elements(rotor, s.dx, h, n_sub)) {
        const Stencil st[3] = {stencil_at(s, 0, e.pos), stencil_at(s, 1, e.pos), stencil_at(s, 2, e.pos)};
        const glm::vec3 air(stencil_mean(s, 0, st[0], grid.u), stencil_mean(s, 1, st[1], grid.v),
                            stencil_mean(s, 2, st[2], grid.w));
        const float ua = -glm::dot(air, rotor.axis);
        const float wt = rotor.omega * e.r - glm::dot(air, e.motion);
        const SectionLoad load = section_load(*rotor.polar, e.chord, e.sin_beta, e.cos_beta, ua, wt, rho);
        // The air takes the opposite of the blade's load: -axial along the axis, +tangent along the blade's motion.
        const glm::vec3 force = (-load.axial * rotor.axis + load.tangent * e.motion) * (e.dr * share);
        const float to_dv = h / (rho * dx3);
        stencil_add(s, 0, st[0], force.x * to_dv, du);
        stencil_add(s, 1, st[1], force.y * to_dv, dv);
        stencil_add(s, 2, st[2], force.z * to_dv, dw);
        thrust += static_cast<double>(load.axial) * static_cast<double>(e.dr * share);
        torque += static_cast<double>(load.tangent) * static_cast<double>(e.r) * static_cast<double>(e.dr * share);
    }
    LineReport rep;
    rep.thrust = static_cast<float>(thrust);
    rep.torque = static_cast<float>(torque);
    rep.power = static_cast<float>(torque * static_cast<double>(rotor.omega));
    return rep;
}

}  // namespace

Result<LineReport> spread_line(const LineRotor& rotor, const MacGrid& grid, float rho, float h, uint32_t n_sub,
                               std::span<float> du, std::span<float> dv, std::span<float> dw) {
    if (auto c = check_line(rotor, grid.shape, rho, h, n_sub); !c) return std::unexpected(c.error());
    const GridShape& g = grid.shape;
    if (grid.u.size() != du.size() || grid.v.size() != dv.size() || grid.w.size() != dw.size() ||
        grid.u.size() != std::size_t{g.nx + 1u} * g.ny * g.nz || grid.v.size() != std::size_t{g.nx} * (g.ny + 1u) * g.nz ||
        grid.w.size() != std::size_t{g.nx} * g.ny * (g.nz + 1u)) {
        return std::unexpected(refuse("line: the velocity arrays and du, dv, dw must be the grid's face counts"));
    }
    const spade::math::ScopedFlushSubnormals flush;
    return spread_flushed(rotor, grid, rho, h, n_sub, du, dv, dw);
}

}  // namespace spade::physics::airflow
