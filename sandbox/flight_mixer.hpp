// N17's mixer (the end-goal design's ruling 2): from any model's rotor layout
// to each rotor's thrust. A pure function that Physics reviews; it moves to
// vehicles/ when it leaves the sandbox.
//
// THE MATRIX. In the model's principal body frame, about its centre of mass,
// rotor i pushes T_i along its axis n_i = local_orient * +Y at its arm
// r_i = local_pos, and its reaction torque is -spin_i k_Q w^2 n_i, with
// k_Q w^2 = c_i T_i, c_i = k_Q / k_T (vehicles/rotor.cpp's law). So the
// effectiveness matrix B maps T to the wrench [collective, moment]:
//     collective_i = n_i . +Y
//     moment_i     = r_i x n_i - spin_i c_i n_i
// the moment about body x (roll, about the nose), y (yaw, about up) and z
// (pitch, about the right).
//
// ALLOCATION. The minimum-norm thrusts T = B^T (B B^T)^-1 w, then, when some
// rotor would leave [t_min, t_max]: yaw is given up first (scaled by the
// largest share that fits, with the collective, roll and pitch kept); then
// roll and pitch scale together (yaw at zero), so the tilt keeps its
// direction; then the collective itself is clamped. The solution is linear
// in w, so each share is exact. drone_sim.hpp's mix_thrusts_yaw_last,
// generalised from the plus quad.
// The choice, named (Physics' review): the collective is kept and the tilt
// gives way, as drone_sim does, not "airmode" (PX4's order, which shifts the
// collective to keep roll and pitch near idle or full throttle).
//
// REFUSED, naming the model: no rotors; a rotor whose k_T isn't > 0; and a
// layout B B^T can't invert. That is found by factoring it in the order
// collective, roll, pitch, yaw: the first axis whose pivot falls below 1e-6
// of its diagonal depends on the ones before it, and is named ("no yaw
// authority": every rotor spinning one way).
#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <span>
#include <string>
#include <vector>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include "core/error.hpp"
#include "vehicles/model_type.hpp"

namespace spade::sandbox::flight {

// What the rotors are asked for, in the principal body frame about the
// centre of mass: the collective along body +Y, N, and the moment about body
// x (roll), y (yaw) and z (pitch), N m.
struct Wrench {
    float collective = 0.0f;
    glm::vec3 moment{0.0f};
};

// A layout's effectiveness matrix and its minimum-norm inverse, one entry per
// rotor: its column of B and its row of P = B^T (B B^T)^-1, each in the order
// collective, moment x, y, z.
struct Mixer {
    std::vector<std::array<float, 4>> b;
    std::vector<std::array<float, 4>> p;
};

struct Allocation {
    std::vector<float> thrust;        // per rotor, N, within [t_min, t_max]
    float yaw_kept = 1.0f;            // the share of the asked yaw given
    float attitude_kept = 1.0f;       // the share of the asked roll and pitch given
    bool collective_clamped = false;  // the collective alone left the rotors' range
};

namespace detail {

[[nodiscard]] inline Error mixer_refusal(const vehicles::ModelType& m, const std::string& why) {
    return Error{Code::invalid_argument, "flight mixer: model '" + m.name + "': " + why};
}

[[nodiscard]] inline std::array<float, 4> as_row(const Wrench& w) {
    return {w.collective, w.moment.x, w.moment.y, w.moment.z};
}

// T = P w.
[[nodiscard]] inline std::vector<float> solve(const Mixer& m, const Wrench& w) {
    const std::array<float, 4> v = as_row(w);
    std::vector<float> t(m.p.size(), 0.0f);
    for (std::size_t i = 0; i < m.p.size(); ++i) {
        for (std::size_t k = 0; k < 4; ++k) t[i] += m.p[i][k] * v[k];
    }
    return t;
}

// The largest share s in [0, 1] with base + s * add inside [lo, hi] for every
// rotor; base itself is inside.
[[nodiscard]] inline float largest_share(const std::vector<float>& base, const std::vector<float>& add, float lo,
                                         float hi) {
    float s = 1.0f;
    for (std::size_t i = 0; i < base.size(); ++i) {
        if (add[i] > 0.0f) s = std::min(s, (hi - base[i]) / add[i]);
        if (add[i] < 0.0f) s = std::min(s, (lo - base[i]) / add[i]);
    }
    return std::clamp(s, 0.0f, 1.0f);
}

[[nodiscard]] inline bool inside(const std::vector<float>& t, float lo, float hi) {
    const float slack = 1e-6f * std::max(1.0f, std::abs(hi));
    return std::all_of(t.begin(), t.end(), [&](float x) { return x >= lo - slack && x <= hi + slack; });
}

}  // namespace detail

[[nodiscard]] inline Result<Mixer> make_mixer(const vehicles::ModelType& model) {
    if (model.rotors.empty()) return std::unexpected(detail::mixer_refusal(model, "no rotors to fly with"));
    Mixer m;
    for (std::size_t i = 0; i < model.rotors.size(); ++i) {
        const vehicles::RotorDesc& r = model.rotors[i];
        if (!(r.thrust_coeff > 0.0f) || !std::isfinite(r.thrust_coeff)) {
            return std::unexpected(detail::mixer_refusal(
                model, "rotor " + std::to_string(i) + "'s k_T is " + std::to_string(r.thrust_coeff) +
                           ", not > 0: it gives no thrust to allocate"));
        }
        const glm::vec3 n = r.local_orient * glm::vec3(0.0f, 1.0f, 0.0f);
        const float c = r.torque_coeff / r.thrust_coeff;
        const glm::vec3 moment = glm::cross(r.local_pos, n) - (r.spin_dir * c) * n;
        m.b.push_back({n.y, moment.x, moment.y, moment.z});
    }

    // G = B B^T, factored (Cholesky, in double) in the order collective,
    // roll, pitch, yaw, so a dependent axis is named.
    double g[4][4] = {};
    for (const std::array<float, 4>& col : m.b) {
        for (std::size_t r = 0; r < 4; ++r) {
            for (std::size_t c = 0; c < 4; ++c) g[r][c] += static_cast<double>(col[r]) * col[c];
        }
    }
    constexpr std::array<std::size_t, 4> kOrder{0, 1, 3, 2};
    constexpr std::array<const char*, 4> kAxis{
        "no collective authority (thrust along body +Y)", "no roll authority (moment about body +X)",
        "no pitch authority (moment about body +Z)", "no yaw authority (moment about body +Y)"};
    double l[4][4] = {};
    for (std::size_t a = 0; a < 4; ++a) {
        const std::size_t ra = kOrder[a];
        double d = g[ra][ra];
        for (std::size_t k = 0; k < a; ++k) d -= l[a][k] * l[a][k];
        if (!(d > 1e-6 * g[ra][ra]) || !(g[ra][ra] > 0.0)) {
            return std::unexpected(detail::mixer_refusal(
                model, std::string(kAxis[a]) + ": the rotors' layout can't produce it independently of the "
                                               "axes before it, so no allocation exists"));
        }
        l[a][a] = std::sqrt(d);
        for (std::size_t b2 = a + 1; b2 < 4; ++b2) {
            double s = g[kOrder[b2]][ra];
            for (std::size_t k = 0; k < a; ++k) s -= l[b2][k] * l[a][k];
            l[b2][a] = s / l[a][a];
        }
    }
    // G^-1, column by column through the factor (permuted), then P = B^T G^-1.
    double inv[4][4] = {};
    for (std::size_t col = 0; col < 4; ++col) {
        double y[4] = {};
        for (std::size_t a = 0; a < 4; ++a) {
            double s = kOrder[a] == col ? 1.0 : 0.0;
            for (std::size_t k = 0; k < a; ++k) s -= l[a][k] * y[k];
            y[a] = s / l[a][a];
        }
        double x[4] = {};
        for (std::size_t a = 4; a-- > 0;) {
            double s = y[a];
            for (std::size_t k = a + 1; k < 4; ++k) s -= l[k][a] * x[k];
            x[a] = s / l[a][a];
        }
        for (std::size_t a = 0; a < 4; ++a) inv[kOrder[a]][col] = x[a];
    }
    for (const std::array<float, 4>& col : m.b) {
        std::array<float, 4> row{};
        for (std::size_t c = 0; c < 4; ++c) {
            double s = 0.0;
            for (std::size_t k = 0; k < 4; ++k) s += static_cast<double>(col[k]) * inv[k][c];
            row[c] = static_cast<float>(s);
        }
        m.p.push_back(row);
    }
    return m;
}

// B T: the wrench these thrusts give.
[[nodiscard]] inline Wrench wrench_of(const Mixer& m, std::span<const float> thrust) {
    std::array<float, 4> w{};
    for (std::size_t i = 0; i < m.b.size() && i < thrust.size(); ++i) {
        for (std::size_t k = 0; k < 4; ++k) w[k] += m.b[i][k] * thrust[i];
    }
    return Wrench{w[0], glm::vec3(w[1], w[2], w[3])};
}

// Each rotor's thrust for `want`, within [t_min, t_max] (see ALLOCATION).
[[nodiscard]] inline Allocation allocate(const Mixer& m, const Wrench& want, float t_min, float t_max) {
    t_max = std::max(t_min, t_max);
    Allocation a;
    a.thrust = detail::solve(m, want);
    if (detail::inside(a.thrust, t_min, t_max)) return a;

    // Yaw first: the collective, roll and pitch, plus the largest share of yaw.
    const std::vector<float> tilt = detail::solve(m, Wrench{want.collective, {want.moment.x, 0.0f, want.moment.z}});
    if (detail::inside(tilt, t_min, t_max)) {
        const std::vector<float> yaw = detail::solve(m, Wrench{0.0f, {0.0f, want.moment.y, 0.0f}});
        a.yaw_kept = detail::largest_share(tilt, yaw, t_min, t_max);
        for (std::size_t i = 0; i < a.thrust.size(); ++i) a.thrust[i] = tilt[i] + a.yaw_kept * yaw[i];
    } else {
        // Then roll and pitch together, yaw at zero.
        a.yaw_kept = 0.0f;
        const std::vector<float> lift = detail::solve(m, Wrench{want.collective, glm::vec3(0.0f)});
        if (detail::inside(lift, t_min, t_max)) {
            const std::vector<float> turn = detail::solve(m, Wrench{0.0f, {want.moment.x, 0.0f, want.moment.z}});
            a.attitude_kept = detail::largest_share(lift, turn, t_min, t_max);
            for (std::size_t i = 0; i < a.thrust.size(); ++i) a.thrust[i] = lift[i] + a.attitude_kept * turn[i];
        } else {
            // Then the collective itself.
            a.attitude_kept = 0.0f;
            a.collective_clamped = true;
            a.thrust = lift;
        }
    }
    for (float& t : a.thrust) t = std::clamp(t, t_min, t_max);
    return a;
}

}  // namespace spade::sandbox::flight
