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
static_assert(kMaxTaps == kLineTaps, "line.hpp's kLineTaps is the 6-cell support's 13 taps");

Error refuse(std::string why) { return Error{Code::invalid_argument, std::move(why)}; }

bool finite3(glm::vec3 v) noexcept { return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z); }

using AxisTaps = LineAxisTaps;
using Stencil = LineStencil;

AxisTaps axis_taps(float pos, float dx, float offset, uint32_t count, float eps_cells) noexcept {
    AxisTaps t;
    const float f = pos / dx - offset;  // the point in this component's face index
    const int lo = std::max(0, static_cast<int>(-std::floor(-(f - static_cast<float>(kReach)))));
    const int hi = std::min(static_cast<int>(count) - 1, static_cast<int>(std::floor(f + static_cast<float>(kReach))));
    if (hi < lo) return t;
    t.lo = lo;
    t.n = hi - lo + 1;
    for (int i = 0; i < t.n; ++i) {
        const float q = (static_cast<float>(lo + i) - f) / eps_cells;  // (x_i - pos) / eps (eps_cells 2: the same bits as the old * 0.5f)
        t.w[static_cast<std::size_t>(i)] = math::exp32(-(q * q));
        t.sum += t.w[static_cast<std::size_t>(i)];
    }
    return t;
}

Stencil stencil_at(const GridShape& s, uint32_t c, glm::vec3 pos, float eps_cells) noexcept {
    const std::array<uint32_t, 3> n{s.nx + (c == 0 ? 1u : 0u), s.ny + (c == 1 ? 1u : 0u), s.nz + (c == 2 ? 1u : 0u)};
    Stencil st;
    for (uint32_t a = 0; a < 3; ++a) {
        const float offset = a == c ? 0.0f : 0.5f;
        st.ax[a] = axis_taps(pos[static_cast<glm::length_t>(a)], s.dx, offset, n[a], eps_cells);
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

Result<void> check_line_damping(float sgs_cs) {
    if (!(sgs_cs > 0.0f)) {  // 0, negative and NaN alike
        return std::unexpected(refuse("actuator lines need a subgrid model (sgs_cs above 0): without one a line run breaks down within a few thousand "
                                      "steps, whether nu is 0 or the air's own (L105a, docs/design/physics/plans/2026-11-19-x500-kit-remeasure.md)"));
    }
    return {};
}

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
    if (!(rotor.eps_cells >= 1.0f && rotor.eps_cells <= 3.0f)) {
        return std::unexpected(refuse("line: eps_cells must be in [1, 3] (the kernel's support is 6 cells)"));
    }
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

LineStencil line_stencil(const GridShape& shape, uint32_t component, glm::vec3 pos, float eps_cells) noexcept {
    return stencil_at(shape, component, pos, eps_cells);
}

glm::vec3 line_sample(const MacGrid& grid, glm::vec3 pos, float eps_cells) noexcept {
    const Stencil su = stencil_at(grid.shape, 0, pos, eps_cells);
    const Stencil sv = stencil_at(grid.shape, 1, pos, eps_cells);
    const Stencil sw = stencil_at(grid.shape, 2, pos, eps_cells);
    return {stencil_mean(grid.shape, 0, su, grid.u), stencil_mean(grid.shape, 1, sv, grid.v),
            stencil_mean(grid.shape, 2, sw, grid.w)};
}

glm::vec3 line_segment_induction(glm::vec3 p, glm::vec3 a, glm::vec3 b, float eps_m) noexcept {
    const glm::vec3 r1 = p - a;
    const glm::vec3 r2 = p - b;
    const glm::vec3 l = b - a;
    const float cx = r1.y * r2.z - r1.z * r2.y;
    const float cy = r1.z * r2.x - r1.x * r2.z;
    const float cz = r1.x * r2.y - r1.y * r2.x;
    const float c2 = cx * cx + cy * cy + cz * cz;
    const float l2 = l.x * l.x + l.y * l.y + l.z * l.z;
    const float n1 = std::sqrt(r1.x * r1.x + r1.y * r1.y + r1.z * r1.z);
    const float n2 = std::sqrt(r2.x * r2.x + r2.y * r2.y + r2.z * r2.z);
    if (!(l2 > 0.0f) || !(n1 > 0.0f) || !(n2 > 0.0f) || !(c2 > 1.0e-12f * l2)) return glm::vec3(0.0f);
    float f = 1.0f;
    if (eps_m > 0.0f) {
        const float q = (c2 / l2) / (eps_m * eps_m);  // (x / eps)^2, x the distance to the filament's line
        if (q > 20.0f) return glm::vec3(0.0f);
        f = math::exp32(-q);
    }
    const float k = (l.x * r1.x + l.y * r1.y + l.z * r1.z) / n1 - (l.x * r2.x + l.y * r2.y + l.z * r2.z) / n2;
    const float scale = f * k / (4.0f * kPi * c2);
    return glm::vec3(cx * scale, cy * scale, cz * scale);
}

LineTipTables line_tip_tables(const LineRotor& rotor, std::span<const LineElement> els, float dx,
                              const LineTipParams& pr) {
    LineTipTables tb;
    if (els.empty()) return tb;
    tb.per = elements_per_blade(*rotor.blade, dx);
    tb.eps_m = rotor.eps_cells * dx;
    const std::size_t segs = static_cast<std::size_t>(std::max(1, pr.segments));
    tb.th.resize(segs + 1);
    tb.ct.resize(segs + 1);
    tb.st.resize(segs + 1);
    for (std::size_t j = 0; j <= segs; ++j) {
        // Graded: the chords are short where the filament starts next to the element (a chord must be shorter than the element's
        // distance to it, dr / 2) and long far away: theta_j = wake (j / n)^2.
        const float x = static_cast<float>(j) / static_cast<float>(segs);
        tb.th[j] = pr.wake_angle * x * x;
        tb.ct[j] = math::cos32(tb.th[j]);
        tb.st[j] = math::sin32(tb.th[j]);
    }
    // The circulation is filtered along the span by this kernel's own Gaussian (zero beyond the ends): the circulation the smeared
    // field itself carries. An unfiltered Gamma has element-to-element modes the iteration amplifies (a mode of wavelength 2 dr
    // is fed back with gain ~ pi c / (4 dr) per unit relaxation), and the unsmeared induction of such modes is not physical.
    const float dr0 = els[0].dr;
    tb.reach = std::max(0, static_cast<int>(std::floor(3.0f * tb.eps_m / dr0)));
    tb.kern.resize(static_cast<std::size_t>(tb.reach) + 1);
    for (int m = 0; m <= tb.reach; ++m) {
        const float q = static_cast<float>(m) * dr0 / tb.eps_m;
        tb.kern[static_cast<std::size_t>(m)] = math::exp32(-(q * q));
        tb.kern_sum += (m == 0 ? 1.0f : 2.0f) * tb.kern[static_cast<std::size_t>(m)];
    }
    float chord_max = 0.0f;
    for (std::size_t k = 0; k < tb.per && k < els.size(); ++k) chord_max = std::max(chord_max, els[k].chord);
    // The damping: a smooth-mode feedback gain g ~ (pi c / 4 eps) bounds the stable step, 1 / (1 + g); params.relax > 0 overrides.
    tb.relax = pr.relax > 0.0f ? pr.relax : std::min(0.5f, 1.0f / (1.0f + 0.7853982f * chord_max / tb.eps_m));
    return tb;
}

float line_tip_correct(const LineRotor& rotor, std::span<const LineElement> els, float dx, float rho,
                       std::span<glm::vec3> air, const LineTipParams& pr) {
    if (!(rotor.omega > 0.0f) || els.empty() || els.size() != air.size()) return 0.0f;
    const LineTipTables tb = line_tip_tables(rotor, els, dx, pr);
    const std::size_t per = tb.per;
    const float eps_m = tb.eps_m;
    const std::size_t segs = tb.th.size() - 1;
    const int reach = tb.reach;
    std::vector<float> gamma(per);
    std::vector<float> gfilt(per);
    std::vector<float> ua(per);
    std::vector<float> strength(per + 1);
    std::vector<glm::vec3> verts((per + 1) * (segs + 1));
    std::vector<glm::vec3> sampled(per);
    float last = 0.0f;
    for (std::size_t g0 = 0; g0 + per <= els.size(); g0 += per) {
        const LineElement* e = &els[g0];
        glm::vec3* u = &air[g0];
        for (std::size_t k = 0; k < per; ++k) sampled[k] = u[k];
        const glm::vec3 d = (e[0].pos - rotor.centre) / e[0].r;  // the blade's direction, outward
        const glm::vec3 m = e[0].motion;
        const float r_in = e[0].r - 0.5f * e[0].dr;
        for (int it = 0; it < pr.iterations; ++it) {
            for (std::size_t k = 0; k < per; ++k) {  // each element's circulation from its corrected air
                ua[k] = -glm::dot(u[k], rotor.axis);
                const float wt = rotor.omega * e[k].r - glm::dot(u[k], e[k].motion);
                const float w2 = ua[k] * ua[k] + wt * wt;
                gamma[k] = 0.0f;
                if (w2 == 0.0f) continue;
                const SectionLoad load =
                    section_load(*rotor.polar, e[k].chord, e[k].sin_beta, e[k].cos_beta, ua[k], wt, rho);
                const float w = std::sqrt(w2);
                gamma[k] = (load.axial * (wt / w) + load.tangent * (ua[k] / w)) / (rho * w);  // L' / (rho W)
            }
            for (std::size_t k = 0; k < per; ++k) {
                float acc = 0.0f;
                for (int off = -reach; off <= reach; ++off) {
                    const long j = static_cast<long>(k) + off;
                    if (j < 0 || j >= static_cast<long>(per)) continue;
                    acc += tb.kern[static_cast<std::size_t>(off < 0 ? -off : off)] * gamma[static_cast<std::size_t>(j)];
                }
                gfilt[k] = acc / tb.kern_sum;
            }
            for (std::size_t v = 0; v <= per; ++v) {  // the filament shed at boundary v (root, between elements, tip)
                const float g_in = v == 0 ? 0.0f : gfilt[v - 1];
                const float g_out = v == per ? 0.0f : gfilt[v];
                strength[v] = rotor.spin * (g_in - g_out);
                const float ua_v = v == 0 ? ua[0] : v == per ? ua[per - 1] : 0.5f * (ua[v - 1] + ua[v]);
                const float sink = std::max(ua_v, 0.0f) / rotor.omega;  // m of descent per radian of blade rotation
                const float rv = r_in + static_cast<float>(v) * e[0].dr;
                for (std::size_t j = 0; j <= segs; ++j) {
                    verts[v * (segs + 1) + j] =
                        rotor.centre + (d * tb.ct[j] - m * tb.st[j]) * rv - rotor.axis * (sink * tb.th[j]);
                }
            }
            float change = 0.0f;
            for (std::size_t k = 0; k < per; ++k) {
                // The element's place from the blade's direction (the device twin has no positions: centre + d r).
                const glm::vec3 p = rotor.centre + d * e[k].r;
                glm::vec3 du(0.0f);
                for (std::size_t v = 0; v <= per; ++v) {
                    const float rv = r_in + static_cast<float>(v) * e[0].dr;
                    if (strength[v] == 0.0f || std::fabs(rv - e[k].r) > 3.0f * eps_m) continue;
                    for (std::size_t j = 0; j < segs; ++j) {
                        du += strength[v] *
                              line_segment_induction(p, verts[v * (segs + 1) + j], verts[v * (segs + 1) + j + 1], eps_m);
                    }
                }
                const glm::vec3 next = u[k] + tb.relax * ((sampled[k] + du) - u[k]);
                change = std::max(change, glm::length(next - u[k]));
                u[k] = next;
            }
            last = change;
        }
    }
    return last;
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
    const std::vector<LineElement> elements = line_elements(rotor, s.dx, h, n_sub);
    // One element's load, its force on the air and its share of the report, from the air it felt and its stencils.
    const auto apply = [&](const LineElement& e, const Stencil (&st)[3], glm::vec3 air) {
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
    };
    if (!rotor.tip_correction) {
        for (const LineElement& e : elements) {
            const Stencil st[3] = {stencil_at(s, 0, e.pos, rotor.eps_cells), stencil_at(s, 1, e.pos, rotor.eps_cells),
                                   stencil_at(s, 2, e.pos, rotor.eps_cells)};
            apply(e, st, glm::vec3(stencil_mean(s, 0, st[0], grid.u), stencil_mean(s, 1, st[1], grid.v),
                                   stencil_mean(s, 2, st[2], grid.w)));
        }
    } else {
        // The correction needs every element's sampled air of a blade first: sample all, correct, then load and spread.
        std::vector<std::array<Stencil, 3>> stencils(elements.size());
        std::vector<glm::vec3> air(elements.size());
        for (std::size_t i = 0; i < elements.size(); ++i) {
            const LineElement& e = elements[i];
            for (uint32_t c = 0; c < 3; ++c) stencils[i][c] = stencil_at(s, c, e.pos, rotor.eps_cells);
            air[i] = glm::vec3(stencil_mean(s, 0, stencils[i][0], grid.u), stencil_mean(s, 1, stencils[i][1], grid.v),
                               stencil_mean(s, 2, stencils[i][2], grid.w));
        }
        (void)line_tip_correct(rotor, elements, s.dx, rho, air);
        for (std::size_t i = 0; i < elements.size(); ++i) {
            const Stencil st[3] = {stencils[i][0], stencils[i][1], stencils[i][2]};
            apply(elements[i], st, air[i]);
        }
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
