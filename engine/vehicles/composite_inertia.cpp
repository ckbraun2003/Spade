#include "vehicles/composite_inertia.hpp"

#include <cmath>
#include <cstddef>
#include <string>

namespace spade::vehicles {
namespace {

using Mat = double[3][3];  // row-major: m[row][col]

[[nodiscard]] bool is_finite(double x) noexcept { return std::isfinite(x); }

[[nodiscard]] Error part_error(std::size_t k, const std::string& what) {
    return Error{Code::invalid_argument, "composite_inertia: part " + std::to_string(k) + " " + what};
}

void to_rows(const glm::dmat3& g, Mat& m) noexcept {
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c) m[r][c] = g[c][r];  // glm is column-major
}

// Cyclic Jacobi for a symmetric 3x3 matrix, in double. On return `a` is
// diagonal (its diagonal holds the eigenvalues) and the columns of `v` are the
// eigenvectors: A v = v D. A matrix that is already diagonal takes no rotation,
// so `v` stays the identity bit for bit. Only + - * / and sqrt are used, all
// correctly rounded by IEEE 754, and the pair order and stopping rule are
// fixed, so the result is the same on every conforming platform.
void jacobi(Mat& a, Mat& v) noexcept {
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c) v[r][c] = (r == c) ? 1.0 : 0.0;
    constexpr int kPairs[3][2] = {{0, 1}, {0, 2}, {1, 2}};
    for (int sweep = 0; sweep < 50; ++sweep) {
        const double off = std::fabs(a[0][1]) + std::fabs(a[0][2]) + std::fabs(a[1][2]);
        if (off == 0.0) return;
        for (const auto& pq : kPairs) {
            const int p = pq[0];
            const int q = pq[1];
            const double apq = a[p][q];
            if (apq == 0.0) continue;
            // Rounding noise is zeroed, not rotated. A part authored with a 90
            // degree turn leaves off-diagonals near 1e-16 of the diagonal (cos
            // pi/2 is not 0 in double). Rotating them would give a symmetric
            // airframe a rotation of about 1e-18 instead of the identity, and
            // where two moments are equal (Ixx = Izz on a plus quad) even a 45
            // degree one. Below 1e-12 of the diagonal, an off-diagonal moves the
            // eigenvalues by about 1e-24 of themselves, far under float output.
            if (std::fabs(apq) <= 1e-12 * (std::fabs(a[p][p]) + std::fabs(a[q][q]))) {
                a[p][q] = 0.0;
                a[q][p] = 0.0;
                continue;
            }
            const double theta = (a[q][q] - a[p][p]) / (2.0 * apq);
            const double t2 = theta * theta;
            double t = 0.0;
            if (is_finite(t2)) {
                t = 1.0 / (std::fabs(theta) + std::sqrt(t2 + 1.0));
                if (theta < 0.0) t = -t;
            } else {
                t = 1.0 / (2.0 * theta);
            }
            const double c = 1.0 / std::sqrt(t * t + 1.0);
            const double s = t * c;
            a[p][p] -= t * apq;
            a[q][q] += t * apq;
            a[p][q] = 0.0;
            a[q][p] = 0.0;
            for (int r = 0; r < 3; ++r) {
                if (r == p || r == q) continue;
                const double arp = a[r][p];
                const double arq = a[r][q];
                a[r][p] = c * arp - s * arq;
                a[p][r] = a[r][p];
                a[r][q] = s * arp + c * arq;
                a[q][r] = a[r][q];
            }
            for (int r = 0; r < 3; ++r) {
                const double vrp = v[r][p];
                const double vrq = v[r][q];
                v[r][p] = c * vrp - s * vrq;
                v[r][q] = s * vrp + c * vrq;
            }
        }
    }
}

// Eigenvalues of a symmetric tensor, through the same Jacobi method.
void eigenvalues(const glm::dmat3& t, double out[3]) noexcept {
    Mat a;
    Mat v;
    to_rows(t, a);
    jacobi(a, v);
    for (int k = 0; k < 3; ++k) out[k] = a[k][k];
}

// Positive semidefinite and the triangle inequality, with a tolerance of
// 1e-9 of the trace for rounding. `strict` also requires every moment > 0.
[[nodiscard]] bool physical(const double e[3], bool strict) noexcept {
    const double trace = std::fabs(e[0]) + std::fabs(e[1]) + std::fabs(e[2]);
    const double tol = 1e-9 * trace;
    for (int k = 0; k < 3; ++k) {
        if (!is_finite(e[k])) return false;
        if (strict ? !(e[k] > 0.0) : e[k] < -tol) return false;
    }
    return e[0] + e[1] >= e[2] - tol && e[0] + e[2] >= e[1] - tol && e[1] + e[2] >= e[0] - tol;
}

[[nodiscard]] Result<void> check_part(const PartInertia& p, std::size_t k) {
    if (!is_finite(p.mass) || !(p.mass > 0.0)) return std::unexpected(part_error(k, "has a mass that is not positive"));
    for (int i = 0; i < 3; ++i) {
        if (!is_finite(p.position[i])) return std::unexpected(part_error(k, "has a non-finite position"));
    }
    const double qn = p.orientation.w * p.orientation.w + p.orientation.x * p.orientation.x +
                      p.orientation.y * p.orientation.y + p.orientation.z * p.orientation.z;
    if (!is_finite(qn) || !(qn > 0.0)) {
        return std::unexpected(part_error(k, "has a zero or non-finite orientation"));
    }
    const auto positive = [](double x) { return std::isfinite(x) && x > 0.0; };
    const auto non_negative = [](double x) { return std::isfinite(x) && x >= 0.0; };
    switch (p.shape) {
        case PartShape::point:
            break;
        case PartShape::sphere:
            if (!positive(p.size.x)) return std::unexpected(part_error(k, "is a sphere without a positive radius"));
            break;
        case PartShape::box:
            if (!positive(p.size.x) || !positive(p.size.y) || !positive(p.size.z)) {
                return std::unexpected(part_error(k, "is a box without three positive extents"));
            }
            break;
        case PartShape::cylinder:
        case PartShape::tube:
            if (!positive(p.size.x) || !non_negative(p.size.y)) {
                return std::unexpected(part_error(k, "needs a positive radius and a non-negative length"));
            }
            break;
        case PartShape::tensor: {
            double scale = 0.0;
            for (int c = 0; c < 3; ++c)
                for (int r = 0; r < 3; ++r) {
                    if (!is_finite(p.tensor[c][r])) {
                        return std::unexpected(part_error(k, "has a non-finite tensor"));
                    }
                    scale = std::fmax(scale, std::fabs(p.tensor[c][r]));
                }
            for (int c = 0; c < 3; ++c)
                for (int r = c + 1; r < 3; ++r) {
                    if (std::fabs(p.tensor[c][r] - p.tensor[r][c]) > 1e-12 * scale) {
                        return std::unexpected(part_error(k, "has a tensor that is not symmetric"));
                    }
                }
            double e[3];
            eigenvalues(p.tensor, e);
            if (!physical(e, false)) {
                return std::unexpected(part_error(
                    k, "has a tensor that is not positive semidefinite or breaks the triangle inequality"));
            }
            break;
        }
        default:
            return std::unexpected(part_error(k, "has an unknown shape"));
    }
    return {};
}

}  // namespace

glm::dmat3 part_inertia_local(const PartInertia& p) noexcept {
    const double m = p.mass;
    glm::dmat3 i(0.0);
    switch (p.shape) {
        case PartShape::point:
            break;
        case PartShape::sphere: {
            const double r = p.size.x;
            const double v = 0.4 * m * r * r;
            i[0][0] = v;
            i[1][1] = v;
            i[2][2] = v;
            break;
        }
        case PartShape::box: {
            const double a2 = p.size.x * p.size.x;
            const double b2 = p.size.y * p.size.y;
            const double c2 = p.size.z * p.size.z;
            i[0][0] = m * (b2 + c2) / 12.0;
            i[1][1] = m * (a2 + c2) / 12.0;
            i[2][2] = m * (a2 + b2) / 12.0;
            break;
        }
        case PartShape::cylinder: {
            const double r2 = p.size.x * p.size.x;
            const double l2 = p.size.y * p.size.y;
            i[0][0] = m * (3.0 * r2 + l2) / 12.0;
            i[1][1] = 0.5 * m * r2;
            i[2][2] = i[0][0];
            break;
        }
        case PartShape::tube: {
            const double r2 = p.size.x * p.size.x;
            const double l2 = p.size.y * p.size.y;
            i[0][0] = m * (6.0 * r2 + l2) / 12.0;
            i[1][1] = m * r2;
            i[2][2] = i[0][0];
            break;
        }
        case PartShape::tensor:
            i = p.tensor;
            break;
    }
    return i;
}

Result<CompositeInertia> composite_inertia(std::span<const PartInertia> parts) {
    if (parts.empty()) return std::unexpected(Error{Code::invalid_argument, "composite_inertia: no parts"});
    for (std::size_t k = 0; k < parts.size(); ++k) {
        if (Result<void> r = check_part(parts[k], k); !r) return std::unexpected(r.error());
    }

    // Mass and centre of mass, in declaration order.
    double mass = 0.0;
    glm::dvec3 moment(0.0);
    for (const PartInertia& p : parts) {
        mass += p.mass;
        moment += p.mass * p.position;
    }
    const glm::dvec3 com = moment / mass;

    // Inertia about the centre of mass, design frame.
    glm::dmat3 inertia(0.0);
    for (const PartInertia& p : parts) {
        const glm::dmat3 rot = glm::mat3_cast(glm::normalize(p.orientation));  // local -> design
        const glm::dmat3 local = part_inertia_local(p);
        const glm::dvec3 d = p.position - com;
        const double d2 = glm::dot(d, d);
        glm::dmat3 shift(0.0);
        for (int c = 0; c < 3; ++c)
            for (int r = 0; r < 3; ++r) shift[c][r] = p.mass * ((r == c ? d2 : 0.0) - d[r] * d[c]);
        inertia += rot * local * glm::transpose(rot) + shift;
    }
    // Symmetrize away rounding before diagonalizing.
    for (int c = 0; c < 3; ++c)
        for (int r = c + 1; r < 3; ++r) {
            const double s = 0.5 * (inertia[c][r] + inertia[r][c]);
            inertia[c][r] = s;
            inertia[r][c] = s;
        }

    Mat a;
    Mat v;
    to_rows(inertia, a);
    jacobi(a, v);
    const double eig[3] = {a[0][0], a[1][1], a[2][2]};
    if (!physical(eig, true)) {
        return std::unexpected(Error{Code::invalid_argument,
                                     "composite_inertia: the composite has a principal moment that is not "
                                     "positive, or breaks the triangle inequality"});
    }

    // Label each eigenvector with the design axis it lies nearest: repeatedly
    // take the largest remaining |v[axis][col]|, first in row-then-column order
    // on a tie.
    int col_of_axis[3] = {-1, -1, -1};
    bool col_used[3] = {false, false, false};
    for (int round = 0; round < 3; ++round) {
        int best_axis = -1;
        int best_col = -1;
        double best = -1.0;
        for (int r = 0; r < 3; ++r) {
            if (col_of_axis[r] >= 0) continue;
            for (int c = 0; c < 3; ++c) {
                if (col_used[c]) continue;
                if (std::fabs(v[r][c]) > best) {
                    best = std::fabs(v[r][c]);
                    best_axis = r;
                    best_col = c;
                }
            }
        }
        col_of_axis[best_axis] = best_col;
        col_used[best_col] = true;
    }

    // Body axis k, in design coordinates: column col_of_axis[k], signed to point
    // along +design axis k.
    Mat b;  // b[r][k] = component r of body axis k
    double moments[3];
    for (int k = 0; k < 3; ++k) {
        const int c = col_of_axis[k];
        const double sign = v[k][c] < 0.0 ? -1.0 : 1.0;
        for (int r = 0; r < 3; ++r) b[r][k] = sign * v[r][c];
        moments[k] = eig[c];
    }
    // Right-handed: if det < 0, flip the axis that is least aligned with its
    // design axis (lowest index on a tie).
    const double det = b[0][0] * (b[1][1] * b[2][2] - b[1][2] * b[2][1]) -
                       b[0][1] * (b[1][0] * b[2][2] - b[1][2] * b[2][0]) +
                       b[0][2] * (b[1][0] * b[2][1] - b[1][1] * b[2][0]);
    if (det < 0.0) {
        int flip = 0;
        for (int k = 1; k < 3; ++k) {
            if (std::fabs(b[k][k]) < std::fabs(b[flip][flip])) flip = k;
        }
        for (int r = 0; r < 3; ++r) b[r][flip] = -b[r][flip];
    }

    // design -> body is B^T: a design vector's body coordinates are its
    // projections onto the body axes.
    glm::dmat3 r_bd(0.0);
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c) r_bd[c][r] = b[c][r];  // (B^T)[r][c] = B[c][r]

    CompositeInertia out;
    out.mass = static_cast<float>(mass);
    out.center_of_mass = glm::vec3(static_cast<float>(com.x), static_cast<float>(com.y), static_cast<float>(com.z));
    out.principal_moments = glm::vec3(static_cast<float>(moments[0]), static_cast<float>(moments[1]),
                                      static_cast<float>(moments[2]));
    const glm::dquat q = glm::quat_cast(r_bd);
    out.design_to_body = glm::quat(static_cast<float>(q.w), static_cast<float>(q.x), static_cast<float>(q.y),
                                   static_cast<float>(q.z));
    out.inertia_design = inertia;
    return out;
}

}  // namespace spade::vehicles
