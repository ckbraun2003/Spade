#pragma once

// ---------------------------------------------------------------------------
// lambda2, the vortex criterion of Jeong and Hussain (1995): the middle
// eigenvalue of S^2 + Omega^2, where S and Omega are the velocity gradient's
// symmetric and antisymmetric parts. A vortex core is where lambda2 < 0.
// Publish (physics/airflow) writes lambda2_hat() into a flow volume's
// kFlowLambda2Hat channel, from the gradient it already forms; the views draw it.
//
// The eigenvalues come from a cyclic Jacobi solve on the symmetric 3x3, with
// a fixed pivot order and a fixed number of sweeps, no early exit, and only
// sqrt and division. So it is deterministic across compilers (TD-3), and a
// golden that folds it survives gcc. The closed form would need acos and
// cube roots. Compiled once under spade_fp_strict, as fp32_math.cpp is.
// ---------------------------------------------------------------------------

#include <array>

namespace spade {

// G[c][a] = d u_c / d x_a, as publish.cpp builds it. The transpose gives the
// same lambda2, so the other convention is harmless.
using VelocityGradient = std::array<std::array<float, 3>, 3>;

// A symmetric 3x3 by its upper triangle.
struct Symmetric3 {
    float xx = 0.0f, yy = 0.0f, zz = 0.0f;
    float xy = 0.0f, xz = 0.0f, yz = 0.0f;
};

// Jacobi sweeps, each rotating the pivots (0,1), (0,2), (1,2) in turn. Six
// is twice what the worst case tests/test_core_lambda2.cpp can construct
// needs to reach float precision; that file records the measurement.
inline constexpr int kLambda2Sweeps = 6;

// S^2 + Omega^2 of `g`, computed as (G^2 + (G^2)^T) / 2, which is the same.
[[nodiscard]] Symmetric3 lambda2_tensor(const VelocityGradient& g) noexcept;

// The eigenvalues of `m`, ascending, after `sweeps` Jacobi sweeps.
[[nodiscard]] std::array<float, 3> symmetric_eigenvalues(const Symmetric3& m,
                                                         int sweeps = kLambda2Sweeps) noexcept;

// lambda2 of `g`: below zero in a vortex core. Its units are 1/s^2.
[[nodiscard]] float lambda2(const VelocityGradient& g) noexcept;

// What a flow volume stores: sign(lambda2) sqrt(|lambda2|), in 1/s, as
// kFlowQHat stores Q. A drone wash's gradients of about 1e4 1/s give a
// lambda2 near 1e8 1/s^2, far past binary16's 65504; the root keeps vortex
// cores in range, and the sign (the vortex test) exact.
[[nodiscard]] float lambda2_hat(const VelocityGradient& g) noexcept;

}  // namespace spade
