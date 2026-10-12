// lambda2 (core/lambda2.hpp). RED: a stub.
#include "core/lambda2.hpp"

namespace spade {

Symmetric3 lambda2_tensor(const VelocityGradient&) noexcept { return {}; }

std::array<float, 3> symmetric_eigenvalues(const Symmetric3&, int) noexcept { return {}; }

float lambda2(const VelocityGradient&) noexcept { return 0.0f; }

float lambda2_hat(const VelocityGradient&) noexcept { return 0.0f; }

}  // namespace spade
