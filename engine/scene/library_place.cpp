#include "scene/library_place.hpp"

#include <array>
#include <cmath>  // std::isfinite, std::fabs and std::sqrt only (TD-3: sqrt is correctly rounded by IEEE 754)
#include <string>
#include <utility>

#include "core/detail/math_ops_f64.hpp"

// ---------------------------------------------------------------------------
// scene/library_place.cpp (L88l). A pose is a position and a quaternion [w, x, y, z]. Every expression is spelt left to right with
// + - * / and sqrt, and the file is built under spade_fp_strict (no contraction), so MSVC and gcc give the same bits.
// ---------------------------------------------------------------------------

namespace spade::scene {

namespace {

using Vec3 = std::array<double, 3>;
using Quat = std::array<double, 4>;  // w, x, y, z

struct Pose {
    Vec3 p{0.0, 0.0, 0.0};
    Quat q{1.0, 0.0, 0.0, 0.0};
};

[[nodiscard]] Quat normalised(const Quat& q) {
    const double n = std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    return Quat{q[0] / n, q[1] / n, q[2] / n, q[3] / n};
}

// a (x) b, the Hamilton product: b's rotation first, then a's.
[[nodiscard]] Quat qmul(const Quat& a, const Quat& b) {
    return Quat{a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3],
                a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2],
                a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1],
                a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0]};
}

// v turned by the unit quaternion q: v + 2 w (u x v) + 2 u x (u x v), u = (x, y, z).
[[nodiscard]] Vec3 rotate(const Quat& q, const Vec3& v) {
    const double w = q[0], ux = q[1], uy = q[2], uz = q[3];
    const double cx = uy * v[2] - uz * v[1];
    const double cy = uz * v[0] - ux * v[2];
    const double cz = ux * v[1] - uy * v[0];
    const double dx = uy * cz - uz * cy;
    const double dy = uz * cx - ux * cz;
    const double dz = ux * cy - uy * cx;
    return Vec3{v[0] + 2.0 * w * cx + 2.0 * dx, v[1] + 2.0 * w * cy + 2.0 * dy, v[2] + 2.0 * w * cz + 2.0 * dz};
}

// a . b: the position of b carried into a's frame, the rotations multiplied.
[[nodiscard]] Pose compose(const Pose& a, const Pose& b) {
    const Vec3 r = rotate(a.q, b.p);
    Pose out;
    out.p = Vec3{a.p[0] + r[0], a.p[1] + r[1], a.p[2] + r[2]};
    out.q = normalised(qmul(a.q, b.q));
    return out;
}

// One rotation of `deg` degrees about a unit axis: [cos(deg/2), sin(deg/2) axis].
[[nodiscard]] Quat about(int axis, double deg) {
    double s = 0.0, c = 1.0;
    math::sincos_deg_arith(deg / 2.0, s, c);
    Quat q{c, 0.0, 0.0, 0.0};
    q[static_cast<std::size_t>(axis)] = s;
    return q;
}

// Rz(yaw) . Ry(pitch) . Rx(roll), intrinsic.
[[nodiscard]] Quat euler_deg(double yaw, double pitch, double roll) {
    return qmul(qmul(about(3, yaw), about(2, pitch)), about(1, roll));
}

[[nodiscard]] bool finite3(const std::array<double, 3>& v) {
    return std::isfinite(v[0]) && std::isfinite(v[1]) && std::isfinite(v[2]);
}

LibraryRefusal refusal(std::string code, std::string field, std::string message) {
    LibraryRefusal r;
    r.code = std::move(code);
    r.field = std::move(field);
    r.message = std::move(message);
    return r;
}

// Reads a pose that is given: finite numbers, a quaternion of unit length to 2e-6 in its square. Returns the refusal field, or empty.
[[nodiscard]] std::string read_pose(const SetPose& in, const std::string& name, Pose& out, std::string& why) {
    if (!finite3(in.pos)) {
        why = "the position must be three finite numbers";
        return name + ".pos";
    }
    const Quat& q = in.quat;
    if (!std::isfinite(q[0]) || !std::isfinite(q[1]) || !std::isfinite(q[2]) || !std::isfinite(q[3])) {
        why = "the quaternion must be four finite numbers";
        return name + ".quat";
    }
    const double n2 = q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3];
    if (std::fabs(n2 - 1.0) > 2e-6) {
        why = "the quaternion [w, x, y, z] must be a unit quaternion (length 1 to 1e-6)";
        return name + ".quat";
    }
    out.p = in.pos;
    out.q = normalised(q);
    return {};
}

constexpr double kZero = 1e-15;

// The result's spelling: w >= 0; a w within 1e-15 of 0 is 0 and the first axis beyond 1e-15 is positive.
[[nodiscard]] Quat spelt(Quat q) {
    if (std::fabs(q[0]) <= kZero) {
        q[0] = 0.0;
        for (std::size_t i = 1; i < 4; ++i) {
            if (std::fabs(q[i]) > kZero) {
                if (q[i] < 0.0) {
                    for (double& c : q) c = -c;
                }
                break;
            }
        }
    } else if (q[0] < 0.0) {
        for (double& c : q) c = -c;
    }
    return q;
}

// Returns the first refusal of `item`, or none; on none, `out` is the pose.
[[nodiscard]] std::optional<LibraryRefusal> place(const PlacementItem& item, SetPose& out) {
    Pose parent, ip, mount;
    std::string why;
    if (item.parent) {
        if (const std::string f = read_pose(*item.parent, "parent", parent, why); !f.empty()) return refusal("invalid_request", f, why);
    }
    if (const std::string f = read_pose(item.install_point, "install_point", ip, why); !f.empty()) return refusal("invalid_request", f, why);
    if (item.part_to_mount) {
        if (const std::string f = read_pose(*item.part_to_mount, "part_to_mount", mount, why); !f.empty()) {
            return refusal("invalid_request", f, why);
        }
    }
    SetAdjust adj;
    if (item.adjust) adj = *item.adjust;
    if (!finite3(adj.pos)) return refusal("invalid_request", "adjust.pos", "the position must be three finite numbers");
    const std::array<std::pair<const char*, double>, 3> angles = {{{"adjust.yaw_deg", adj.yaw_deg}, {"adjust.pitch_deg", adj.pitch_deg}, {"adjust.roll_deg", adj.roll_deg}}};
    for (const auto& [field, deg] : angles) {
        if (!std::isfinite(deg) || std::fabs(deg) > 1e6) return refusal("invalid_request", field, "the angle must be finite, within 1e6 degrees");
    }
    if (item.adjust_range) {
        const AdjustRange& r = *item.adjust_range;
        const std::array<double, 3> lo{r.pos_min.x, r.pos_min.y, r.pos_min.z}, hi{r.pos_max.x, r.pos_max.y, r.pos_max.z};
        if (!finite3(lo) || !finite3(hi) || lo[0] > hi[0] || lo[1] > hi[1] || lo[2] > hi[2]) {
            return refusal("invalid_request", "adjust_range.pos_min", "pos_min and pos_max must be finite, with pos_min at most pos_max on every axis");
        }
        const std::array<std::pair<const char*, const std::array<double, 2>*>, 3> pairs = {
            {{"adjust_range.yaw_deg", &r.yaw_deg}, {"adjust_range.pitch_deg", &r.pitch_deg}, {"adjust_range.roll_deg", &r.roll_deg}}};
        for (const auto& [field, range] : pairs) {
            if (!std::isfinite((*range)[0]) || !std::isfinite((*range)[1]) || (*range)[0] > (*range)[1]) {
                return refusal("invalid_request", field, "the range must be two finite numbers, min at most max");
            }
        }
        for (std::size_t i = 0; i < 3; ++i) {
            if (adj.pos[i] < lo[i] || adj.pos[i] > hi[i]) {
                return refusal("adjust_out_of_range", "adjust.pos[" + std::to_string(i) + "]",
                               "the adjustment " + std::to_string(adj.pos[i]) + " m is outside [" + std::to_string(lo[i]) + ", " + std::to_string(hi[i]) + "] m");
            }
        }
        for (std::size_t i = 0; i < 3; ++i) {
            const double v = angles[i].second;
            const std::array<double, 2>& range = *pairs[i].second;
            if (v < range[0] || v > range[1]) {
                return refusal("adjust_out_of_range", angles[i].first,
                               "the adjustment " + std::to_string(v) + " degrees is outside [" + std::to_string(range[0]) + ", " + std::to_string(range[1]) + "]");
            }
        }
    }
    Pose adjust;
    adjust.p = adj.pos;
    adjust.q = euler_deg(adj.yaw_deg, adj.pitch_deg, adj.roll_deg);
    const Pose world = compose(compose(compose(parent, ip), adjust), mount);
    out.pos = world.p;
    out.quat = spelt(normalised(world.q));
    return std::nullopt;
}

}  // namespace

std::vector<PlacedItem> compose_placement(const std::vector<PlacementItem>& items) {
    std::vector<PlacedItem> out;
    out.reserve(items.size());
    for (const PlacementItem& item : items) {
        PlacedItem placed;
        placed.key = item.key;
        SetPose pose;
        if (std::optional<LibraryRefusal> r = place(item, pose)) {
            placed.refusal = std::move(*r);
        } else {
            placed.pose = pose;
        }
        out.push_back(std::move(placed));
    }
    return out;
}

}  // namespace spade::scene
