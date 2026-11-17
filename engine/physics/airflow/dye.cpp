#include "physics/airflow/dye.hpp"

#include <algorithm>
#include <cmath>
#include <string>

#include "core/detail/flush_subnormals.hpp"

namespace spade::physics::airflow {

namespace {

[[nodiscard]] float ramp(float v, float inv_dx) noexcept {
    const float x = v * inv_dx + 0.5f;
    return x < 0.0f ? 0.0f : (x > 1.0f ? 1.0f : x);
}

void emit_flushed(std::span<float> dye, const GridShape& s, std::span<const ActuatorDisc> discs, float h,
                  const DyeRelease& e) noexcept {
    const float dx = s.dx;
    const float inv_dx = 1.0f / dx;
    const float k = e.rate * h;
    const uint32_t n[3] = {s.nx, s.ny, s.nz};
    for (const ActuatorDisc& d : discs) {
        // A cube about the disc that holds every cell with a non-zero increment: 2 dx along the axis, r_out R + dx / 2 across (and
        // the GPU kernel takes the same cube: airflow_dye_source.slang's in_cube()).
        const float reach = e.r_out * d.radius + 2.0f * dx;
        uint32_t lo[3];
        uint32_t hi[3];
        for (int a = 0; a < 3; ++a) {
            const float c = d.centre[a];
            lo[a] = static_cast<uint32_t>(std::max(0.0f, std::floor((c - reach) * inv_dx)));
            hi[a] = std::min(n[a], static_cast<uint32_t>(std::max(0.0f, std::ceil((c + reach) * inv_dx))) + 1u);
        }
        for (uint32_t kz = lo[2]; kz < hi[2]; ++kz) {
            for (uint32_t j = lo[1]; j < hi[1]; ++j) {
                for (uint32_t i = lo[0]; i < hi[0]; ++i) {
                    const glm::vec3 p((static_cast<float>(i) + 0.5f) * dx, (static_cast<float>(j) + 0.5f) * dx,
                                      (static_cast<float>(kz) + 0.5f) * dx);
                    const float inc = dye_increment(d, p, inv_dx, k, e.r_in, e.r_out);
                    if (inc > 0.0f) {
                        float& cell = dye[i + std::size_t{s.nx} * (j + std::size_t{s.ny} * kz)];
                        cell = cell + (1.0f - cell) * inc;
                    }
                }
            }
        }
    }
}

}  // namespace

Result<void> check_dye_release(const DyeRelease& r) {
    const auto refuse = [](const std::string& what) {
        return std::unexpected(Error{Code::invalid_argument, "airflow dye release: " + what});
    };
    if (!std::isfinite(r.rate) || !(r.rate >= 0.0f)) return refuse("rate must be finite and at least 0");
    if (!std::isfinite(r.r_in) || !std::isfinite(r.r_out) || !(r.r_in >= 0.0f) || !(r.r_out > r.r_in) || !(r.r_out <= 2.0f)) {
        return refuse("r_in must be at least 0 and below r_out, which is at most 2 (r / R)");
    }
    return {};
}

float dye_increment(const ActuatorDisc& disc, glm::vec3 p, float inv_dx, float k, float r_in, float r_out) noexcept {
    const float rx = p.x - disc.centre.x;
    const float ry = p.y - disc.centre.y;
    const float rz = p.z - disc.centre.z;
    const float along = (rx * disc.axis.x + ry * disc.axis.y) + rz * disc.axis.z;
    if (!(along >= 0.0f)) return 0.0f;  // upstream of the plane: nothing
    const float a = along * inv_dx;
    if (!(a < 2.0f)) return 0.0f;
    const float phi = a < 1.0f ? (3.0f - 2.0f * a + std::sqrt(1.0f + 4.0f * a - 4.0f * a * a)) * 0.125f
                               : (5.0f - 2.0f * a - std::sqrt(-7.0f + 12.0f * a - 4.0f * a * a)) * 0.125f;
    const float qx = rx - along * disc.axis.x;
    const float qy = ry - along * disc.axis.y;
    const float qz = rz - along * disc.axis.z;
    const float r = std::sqrt((qx * qx + qy * qy) + qz * qz);
    float w = ramp(r_out * disc.radius - r, inv_dx);
    if (r_in > 0.0f) w = w * ramp(r - r_in * disc.radius, inv_dx);
    const float inc = (k * (2.0f * phi)) * w;
    return inc > 1.0f ? 1.0f : inc;
}

void emit_dye(std::span<float> dye, const GridShape& shape, std::span<const ActuatorDisc> discs, float h,
              const DyeRelease& release) noexcept {
    const spade::math::ScopedFlushSubnormals flush;
    emit_flushed(dye, shape, discs, h, release);
}

}  // namespace spade::physics::airflow
