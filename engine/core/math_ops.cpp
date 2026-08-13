#include "core/math_ops.hpp"

#include "core/fp32_math.hpp"

namespace spade::math {

glm::quat integrate_orientation(glm::quat q, glm::vec3 omega_body, float dt) noexcept {
    // half_dt_omega IS half_angle * axis whenever omega_body != 0 (its own
    // length is |omega_body| * dt / 2 = half_angle), so multiplying it by
    // sinc(half_angle) below yields sin(half_angle) * axis directly, with no
    // separate axis normalization needed and no special case for
    // omega_body == 0 (half_dt_omega is then the zero vector, which is the
    // correct vector part regardless of sinc_half's value).
    const glm::vec3 half_dt_omega = 0.5f * dt * omega_body;
    const float theta = glm::length(omega_body) * dt;  // total angle swept this substep
    const float half_angle = 0.5f * theta;

    float cos_half;
    float sinc_half;  // sin(half_angle) / half_angle
    if (theta < 1e-6f) {
        // Small-angle Taylor series -- avoids the sin(x)/x division as
        // half_angle -> 0 (see header comment for the parity rationale).
        const float half_angle_sq = half_angle * half_angle;
        cos_half = 1.0f - 0.5f * half_angle_sq;
        sinc_half = 1.0f - half_angle_sq / 6.0f;
    } else {
        // cos32/sin32, NOT std::cos/std::sin: IEEE mandates correct rounding
        // for neither, so a libm here would make every orientation in the
        // engine a function of which C runtime built it (fp32_math.hpp's
        // opening note is the whole argument, and the golden determinism
        // corpus is where it was caught the first time).
        //
        // THE ACCURACY DOMAIN IS SATISFIED BY CONSTRUCTION HERE, which is the
        // one thing a reader should check rather than assume. sin32/cos32 are
        // <= 1 ulp for |x| <= kMaxAccurateAngle == 100 rad; this call site
        // passes HALF the angle swept in ONE SUBSTEP. 100 rad of half-angle is
        // 200 rad in a substep -- about 32 revolutions of the airframe in a
        // single dt/substeps, four orders of magnitude past anything a
        // physical vehicle or a stable integrator produces. Outside the domain
        // the pair stays total and stays bit-portable (fp32_math.hpp), so even
        // that absurd case degrades in accuracy only, never in determinism.
        cos_half = cos32(half_angle);
        sinc_half = sin32(half_angle) / half_angle;
    }

    const glm::quat delta{cos_half, sinc_half * half_dt_omega};
    return glm::normalize(q * delta);
}

glm::mat3 inertia_world(glm::mat3 I_body, glm::quat orientation) noexcept {
    const glm::mat3 R = glm::mat3_cast(orientation);
    return R * I_body * glm::transpose(R);
}

glm::mat3 inertia_world(glm::vec3 principal_moments_body, glm::quat orientation) noexcept {
    const glm::mat3 I_body{
        principal_moments_body.x, 0.0f, 0.0f,
        0.0f, principal_moments_body.y, 0.0f,
        0.0f, 0.0f, principal_moments_body.z,
    };
    return inertia_world(I_body, orientation);
}

glm::vec3 gyroscopic_torque(glm::mat3 I_body, glm::vec3 omega_body) noexcept {
    return -glm::cross(omega_body, I_body * omega_body);
}

}  // namespace spade::math
