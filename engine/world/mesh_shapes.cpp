#include "world/mesh_shapes.hpp"

#include <algorithm>
#include <cmath>  // std::sqrt and std::isfinite only (TD-3)
#include <cstddef>
#include <cstdio>
#include <string>
#include <utility>
#include <vector>

#include "core/fp32_math.hpp"

namespace spade::shapes {

namespace {

constexpr double kPi = 3.141592653589793;

[[nodiscard]] Error invalid(std::string what) { return Error{Code::invalid_argument, std::move(what)}; }

[[nodiscard]] std::string num(double v) {
    char buf[32];
    std::snprintf(buf, sizeof buf, "%.9g", v);
    return buf;
}

[[nodiscard]] bool positive(double v) { return std::isfinite(v) && v > 0.0; }

// cos and sin of 2 pi k / n, k = 0 .. n - 1, through cos32 / sin32 inside
// [0, 2 pi).
struct Circle {
    std::vector<double> c;
    std::vector<double> s;
};

[[nodiscard]] Circle circle(uint32_t n) {
    Circle out;
    out.c.resize(n);
    out.s.resize(n);
    for (uint32_t k = 0; k < n; ++k) {
        const float a = static_cast<float>(2.0 * kPi * static_cast<double>(k) / static_cast<double>(n));
        out.c[k] = math::cos32(a);
        out.s[k] = math::sin32(a);
    }
    return out;
}

void add_triangle(MeshInput& in, uint32_t a, uint32_t b, uint32_t c) {
    in.indices.push_back(a);
    in.indices.push_back(b);
    in.indices.push_back(c);
}

// One point of a surface of revolution's profile: a pole (radius 0) or a ring.
struct ProfilePoint {
    double radius = 0.0;
    double y = 0.0;
};

// Appends a surface of revolution about +Y. `profile` runs from the top
// pole, outward and down the outside, to the bottom pole; each point between
// is a ring of n vertices (radius cos, y, radius sin). Each band's triangles
// are (upper k, upper k+1, lower k) and (upper k+1, lower k+1, lower k):
// with the ring turning from +X toward +Z, that faces outward.
void append_lathe(MeshInput& in, const std::vector<ProfilePoint>& profile, uint32_t n) {
    const Circle ring = circle(n);
    std::vector<uint32_t> first(profile.size());
    for (std::size_t i = 0; i < profile.size(); ++i) {
        first[i] = static_cast<uint32_t>(in.positions.size());
        const ProfilePoint& p = profile[i];
        if (p.radius == 0.0) {
            in.positions.emplace_back(0.0, p.y, 0.0);
            continue;
        }
        for (uint32_t k = 0; k < n; ++k) in.positions.emplace_back(p.radius * ring.c[k], p.y, p.radius * ring.s[k]);
    }
    for (std::size_t i = 0; i + 1 < profile.size(); ++i) {
        const bool upper_pole = profile[i].radius == 0.0;
        const bool lower_pole = profile[i + 1].radius == 0.0;
        for (uint32_t k = 0; k < n; ++k) {
            const uint32_t k1 = (k + 1) % n;
            const uint32_t u0 = first[i] + (upper_pole ? 0 : k);
            const uint32_t u1 = first[i] + (upper_pole ? 0 : k1);
            const uint32_t l0 = first[i + 1] + (lower_pole ? 0 : k);
            const uint32_t l1 = first[i + 1] + (lower_pole ? 0 : k1);
            if (!upper_pole) add_triangle(in, u0, u1, l0);
            if (!lower_pole) add_triangle(in, u1, l1, l0);
        }
    }
}

[[nodiscard]] Result<GeneratedMesh> finish(const MeshInput& in, const char* name, double min_feature) {
    Result<std::shared_ptr<const TriangleMesh>> mesh = make_mesh(in, name);
    if (!mesh) return std::unexpected(mesh.error());
    return GeneratedMesh{std::move(*mesh), static_cast<float>(min_feature)};
}

// The rounded box's grid lines on one axis of half-extent h: -h to the inner
// box's -(h - rho) in `steps`, then their mirror, so the grid is symmetric bit
// for bit.
[[nodiscard]] std::vector<double> box_lines(double h, double rho, uint32_t steps) {
    std::vector<double> lo(steps + 1);
    for (uint32_t i = 0; i < steps; ++i) lo[i] = -h + rho * static_cast<double>(i) / static_cast<double>(steps);
    lo[steps] = -(h - rho);
    std::vector<double> out = lo;
    for (uint32_t i = steps + 1; i-- > 0;) out.push_back(-lo[i]);
    return out;
}

// The NACA 4-digit camber line and half-thickness, as fractions of the chord.
[[nodiscard]] double camber_line(double x, double m, double p) {
    if (m == 0.0) return 0.0;
    if (x < p) return m / (p * p) * (2.0 * p * x - x * x);
    return m / ((1.0 - p) * (1.0 - p)) * ((1.0 - 2.0 * p) + 2.0 * p * x - x * x);
}

[[nodiscard]] double half_thickness(double x, double t) {
    const double poly = x * (-0.1260 + x * (-0.3516 + x * (0.2843 + x * -0.1015)));
    return 5.0 * t * (0.2969 * std::sqrt(x) + poly);
}

[[nodiscard]] Result<void> ring_segments(const char* fn, uint32_t segments) {
    if (segments >= 3 && segments <= kMaxRingSegments) return {};
    return std::unexpected(invalid(std::string(fn) + ": segments " + std::to_string(segments) + " must be in [3, " +
                                   std::to_string(kMaxRingSegments) + "]"));
}

[[nodiscard]] Result<void> refuse(std::string what) { return std::unexpected(invalid("prop: " + std::move(what))); }

[[nodiscard]] Result<void> check_prop(const PropMeshSpec& s) {
    const std::size_t n = s.r_over_r.size();
    if (!positive(s.diameter)) return refuse("diameter " + num(s.diameter) + " must be finite and above 0");
    if (s.blades == 0 || s.blades > kMaxPropBlades) {
        return refuse("blades " + std::to_string(s.blades) + " must be in [1, " + std::to_string(kMaxPropBlades) + "]");
    }
    if (n < 2 || n > kMaxPropStations) {
        return refuse("r_over_r has " + std::to_string(n) + " stations; give 2 to " + std::to_string(kMaxPropStations));
    }
    if (s.c_over_r.size() != n) {
        return refuse("c_over_r has " + std::to_string(s.c_over_r.size()) + " values for " + std::to_string(n) +
                      " stations");
    }
    if (s.beta_deg.size() != n) {
        return refuse("beta_deg has " + std::to_string(s.beta_deg.size()) + " values for " + std::to_string(n) +
                      " stations");
    }
    if (!(std::isfinite(s.hub_cutout) && s.hub_cutout >= 0.0 && s.hub_cutout < 1.0)) {
        return refuse("hub_cutout " + num(s.hub_cutout) + " must be in [0, 1)");
    }
    for (std::size_t j = 0; j < n; ++j) {
        const std::string at = "[" + std::to_string(j) + "] = ";
        const double r = s.r_over_r[j];
        if (!(std::isfinite(r) && r > 0.0 && r <= 1.0)) return refuse("r_over_r" + at + num(r) + " must be in (0, 1]");
        if (j == 0 && r < s.hub_cutout) {
            return refuse("r_over_r" + at + num(r) + " starts below hub_cutout " + num(s.hub_cutout));
        }
        if (j > 0 && !(r > s.r_over_r[j - 1])) {
            return refuse("r_over_r" + at + num(r) + " is not above r_over_r[" + std::to_string(j - 1) +
                          "] = " + num(s.r_over_r[j - 1]));
        }
        if (!positive(s.c_over_r[j])) return refuse("c_over_r" + at + num(s.c_over_r[j]) + " must be above 0");
        const double b = s.beta_deg[j];
        if (!(std::isfinite(b) && b > -90.0 && b < 90.0)) {
            return refuse("beta_deg" + at + num(b) + " must be in (-90, 90)");
        }
    }
    if (!positive(s.hub_gap)) return refuse("hub_gap " + num(s.hub_gap) + " must be above 0");
    const double hub = s.hub_cutout * (s.diameter * 0.5) - s.hub_gap;
    if (!(hub > 0.0)) {
        return refuse("hub_gap " + num(s.hub_gap) + " leaves no hub: hub_cutout R - hub_gap = " + num(hub));
    }
    if (!positive(s.hub_height)) return refuse("hub_height " + num(s.hub_height) + " must be above 0");
    if (s.hub_segments < 3 || s.hub_segments > kMaxRingSegments) {
        return refuse("hub_segments " + std::to_string(s.hub_segments) + " must be in [3, " +
                      std::to_string(kMaxRingSegments) + "]");
    }
    const PropSection& c = s.section;
    if (!(std::isfinite(c.camber) && c.camber >= 0.0 && c.camber < 0.2)) {
        return refuse("section.camber " + num(c.camber) + " must be in [0, 0.2)");
    }
    if (!(std::isfinite(c.camber_at) && c.camber_at > 0.0 && c.camber_at < 1.0)) {
        return refuse("section.camber_at " + num(c.camber_at) + " must be in (0, 1)");
    }
    if (!(std::isfinite(c.thickness_root) && c.thickness_root > 0.0 && c.thickness_root < 0.5)) {
        return refuse("section.thickness_root " + num(c.thickness_root) + " must be in (0, 0.5)");
    }
    if (!(std::isfinite(c.thickness_tip) && c.thickness_tip > 0.0 && c.thickness_tip < 0.5)) {
        return refuse("section.thickness_tip " + num(c.thickness_tip) + " must be in (0, 0.5)");
    }
    if (c.chord_points < 2 || c.chord_points > kMaxChordPoints) {
        return refuse("section.chord_points " + std::to_string(c.chord_points) + " must be in [2, " +
                      std::to_string(kMaxChordPoints) + "]");
    }
    return {};
}

}  // namespace

Result<GeneratedMesh> rounded_box(glm::dvec3 size, double edge_radius, uint32_t segments) {
    if (!(positive(size.x) && positive(size.y) && positive(size.z))) {
        return std::unexpected(invalid("rounded_box: size (" + num(size.x) + ", " + num(size.y) + ", " + num(size.z) +
                                       ") must be finite and above 0"));
    }
    const double smallest = std::min({size.x, size.y, size.z});
    if (!(std::isfinite(edge_radius) && edge_radius > 0.0 && edge_radius < 0.5 * smallest)) {
        return std::unexpected(invalid("rounded_box: edge_radius " + num(edge_radius) +
                                       " must be above 0 and below half the smallest extent, " + num(0.5 * smallest)));
    }
    if (segments == 0 || segments > kMaxBoxSegments) {
        return std::unexpected(invalid("rounded_box: segments " + std::to_string(segments) + " must be in [1, " +
                                       std::to_string(kMaxBoxSegments) + "]"));
    }

    const glm::dvec3 h = size * 0.5;
    const glm::dvec3 inner = h - glm::dvec3(edge_radius);
    const std::vector<double> lines[3] = {box_lines(h.x, edge_radius, segments),
                                          box_lines(h.y, edge_radius, segments),
                                          box_lines(h.z, edge_radius, segments)};
    const uint32_t g = 2 * (segments + 1);
    const auto at = [g](uint32_t i, uint32_t j, uint32_t k) {
        return (static_cast<std::size_t>(i) * g + j) * g + k;
    };

    // The cube's surface grid points, in (i, j, k) order, each moved onto the
    // rounded box. A point past the inner box on one axis only is on a flat
    // face already and keeps its bits; one past it on two or three axes goes
    // onto the inner box's edge cylinder or corner sphere,
    // inner + rho (p - inner) / |p - inner|, inner the nearest inner point.
    MeshInput in;
    in.unit = MeshUnit::metre;
    std::vector<uint32_t> index(static_cast<std::size_t>(g) * g * g, 0xFFFFFFFFu);
    for (uint32_t i = 0; i < g; ++i) {
        for (uint32_t j = 0; j < g; ++j) {
            for (uint32_t k = 0; k < g; ++k) {
                const bool surface = i == 0 || i == g - 1 || j == 0 || j == g - 1 || k == 0 || k == g - 1;
                if (!surface) continue;
                const glm::dvec3 p(lines[0][i], lines[1][j], lines[2][k]);
                glm::dvec3 near(0.0);
                glm::dvec3 d(0.0);
                int past = 0;
                for (int a = 0; a < 3; ++a) {
                    near[a] = std::clamp(p[a], -inner[a], inner[a]);
                    d[a] = p[a] - near[a];
                    past += d[a] != 0.0 ? 1 : 0;
                }
                glm::dvec3 q = p;
                if (past > 1) {
                    const double len = std::sqrt(d.x * d.x + d.y * d.y + d.z * d.z);
                    for (int a = 0; a < 3; ++a) {
                        if (d[a] != 0.0) q[a] = near[a] + edge_radius * (d[a] / len);
                    }
                }
                index[at(i, j, k)] = static_cast<uint32_t>(in.positions.size());
                in.positions.push_back(q);
            }
        }
    }
    // Each face's grid, two triangles a cell, counter-clockwise from outside:
    // on the face normal to axis a, the other two axes (b, c) turn as
    // b x c = a.
    for (int a = 0; a < 3; ++a) {
        const int b = (a + 1) % 3;
        const int c = (a + 2) % 3;
        for (const uint32_t side : {0u, g - 1}) {
            const auto v = [&](uint32_t u, uint32_t w) {
                uint32_t ijk[3] = {0, 0, 0};
                ijk[a] = side;
                ijk[b] = u;
                ijk[c] = w;
                return index[at(ijk[0], ijk[1], ijk[2])];
            };
            for (uint32_t u = 0; u + 1 < g; ++u) {
                for (uint32_t w = 0; w + 1 < g; ++w) {
                    const uint32_t q00 = v(u, w);
                    const uint32_t q10 = v(u + 1, w);
                    const uint32_t q11 = v(u + 1, w + 1);
                    const uint32_t q01 = v(u, w + 1);
                    if (side != 0) {
                        add_triangle(in, q00, q10, q11);
                        add_triangle(in, q00, q11, q01);
                    } else {
                        add_triangle(in, q00, q11, q10);
                        add_triangle(in, q00, q01, q11);
                    }
                }
            }
        }
    }
    return finish(in, "rounded_box", smallest);
}

Result<GeneratedMesh> capped_tube(double radius, double length, uint32_t segments) {
    if (!positive(radius)) return std::unexpected(invalid("capped_tube: radius " + num(radius) + " must be above 0"));
    if (!positive(length)) return std::unexpected(invalid("capped_tube: length " + num(length) + " must be above 0"));
    if (Result<void> ok = ring_segments("capped_tube", segments); !ok) return std::unexpected(ok.error());

    // From the top pole down its cap to the equator, then the bottom cap, each
    // y the top's negated bit for bit.
    const uint32_t rings = std::max(2u, segments / 4);
    const double half = length * 0.5;
    std::vector<ProfilePoint> cap;  // the top cap's rings below its pole, ending at the equator
    for (uint32_t j = 1; j < rings; ++j) {
        const float phi = static_cast<float>(0.5 * kPi * static_cast<double>(j) / static_cast<double>(rings));
        const double sp = math::sin32(phi);
        const double cp = math::cos32(phi);
        cap.push_back({radius * sp, half + radius * cp});
    }
    cap.push_back({radius, half});
    std::vector<ProfilePoint> profile{{0.0, half + radius}};
    profile.insert(profile.end(), cap.begin(), cap.end());
    for (std::size_t j = cap.size(); j-- > 0;) profile.push_back({cap[j].radius, -cap[j].y});
    profile.push_back({0.0, -(half + radius)});

    MeshInput in;
    in.unit = MeshUnit::metre;
    append_lathe(in, profile, segments);
    return finish(in, "capped_tube", 2.0 * radius);
}

Result<GeneratedMesh> motor_bell(double radius, double height, double chamfer, uint32_t segments) {
    if (!positive(radius)) return std::unexpected(invalid("motor_bell: radius " + num(radius) + " must be above 0"));
    if (!positive(height)) return std::unexpected(invalid("motor_bell: height " + num(height) + " must be above 0"));
    if (!(std::isfinite(chamfer) && chamfer > 0.0 && chamfer < std::min(radius, height))) {
        return std::unexpected(invalid("motor_bell: chamfer " + num(chamfer) +
                                       " must be above 0 and below the lesser of radius and height, " +
                                       num(std::min(radius, height))));
    }
    if (Result<void> ok = ring_segments("motor_bell", segments); !ok) return std::unexpected(ok.error());

    MeshInput in;
    in.unit = MeshUnit::metre;
    append_lathe(in, {{0.0, 0.0}, {radius - chamfer, 0.0}, {radius, -chamfer}, {radius, -height}, {0.0, -height}},
                 segments);
    return finish(in, "motor_bell", std::min(2.0 * radius, height));
}

Result<GeneratedMesh> prop(const PropMeshSpec& s, PropHand hand) {
    if (Result<void> ok = check_prop(s); !ok) return std::unexpected(ok.error());
    const double big_r = s.diameter * 0.5;
    const double hub = s.hub_cutout * big_r - s.hub_gap;
    const std::size_t stations = s.r_over_r.size();
    const uint32_t n = s.section.chord_points;
    const uint32_t ring = 2 * n + 1;

    MeshInput in;
    in.unit = MeshUnit::metre;
    // The hub, its own shell.
    const double hh = 0.5 * s.hub_height;
    append_lathe(in, {{0.0, hh}, {hub, hh}, {hub, -hh}, {0.0, -hh}}, s.hub_segments);

    // Blade 0's sections, each in its plane x = r: the leading edge, the
    // upper surface to the trailing edge, and the lower surface back. A
    // chord point (x, y), fractions of the chord from the leading edge, lies
    // along = (x - 1/4) c down the pitched chord, (z cos beta, y -sin beta),
    // and off = y c along its normal, (z sin beta, y cos beta).
    std::vector<glm::dvec3> blade;
    blade.reserve(stations * ring);
    double thinnest = 0.0;
    const double r0 = s.r_over_r.front();
    const double span = s.r_over_r.back() - r0;
    for (std::size_t j = 0; j < stations; ++j) {
        const double r = s.r_over_r[j] * big_r;
        const double c = s.c_over_r[j] * big_r;
        const double u = (s.r_over_r[j] - r0) / span;
        const double t = s.section.thickness_root * (1.0 - u) + s.section.thickness_tip * u;
        thinnest = j == 0 ? t * c : std::min(thinnest, t * c);
        const float beta = static_cast<float>(s.beta_deg[j] * kPi / 180.0);
        const double cb = math::cos32(beta);
        const double sb = math::sin32(beta);
        const auto point = [&](double x, double y) {
            const double along = (x - 0.25) * c;
            const double off = y * c;
            return glm::dvec3(r, -along * sb + off * cb, along * cb + off * sb);
        };
        std::vector<glm::dvec3> upper;
        std::vector<glm::dvec3> lower;
        for (uint32_t i = 1; i <= n; ++i) {
            const double f = static_cast<double>(i) / static_cast<double>(n);
            const double x = f * f;
            const double yc = camber_line(x, s.section.camber, s.section.camber_at);
            const double yt = half_thickness(x, t);
            upper.push_back(point(x, yc + yt));
            lower.push_back(point(x, yc - yt));
        }
        blade.push_back(point(0.0, 0.0));
        blade.insert(blade.end(), upper.begin(), upper.end());
        blade.insert(blade.end(), lower.rbegin(), lower.rend());
    }

    // Blade b at 2 pi b / blades about +Y, R_y; blade 0 unturned.
    const Circle turn = circle(s.blades);
    for (uint32_t b = 0; b < s.blades; ++b) {
        const uint32_t first = static_cast<uint32_t>(in.positions.size());
        for (const glm::dvec3& p : blade) {
            if (b == 0) {
                in.positions.push_back(p);
            } else {
                in.positions.emplace_back(p.x * turn.c[b] + p.z * turn.s[b], p.y, -p.x * turn.s[b] + p.z * turn.c[b]);
            }
        }
        const auto v = [&](std::size_t j, uint32_t k) { return first + static_cast<uint32_t>(j) * ring + k; };
        // The loft: each ring turns counter-clockwise seen from the tip (+X),
        // and the stations step out toward it, so (j k, j k+1, j+1 k) faces out.
        for (std::size_t j = 0; j + 1 < stations; ++j) {
            for (uint32_t k = 0; k < ring; ++k) {
                const uint32_t k1 = (k + 1) % ring;
                add_triangle(in, v(j, k), v(j, k1), v(j + 1, k));
                add_triangle(in, v(j, k1), v(j + 1, k1), v(j + 1, k));
            }
        }
        // The caps, in strips pairing upper i (ring index i) with lower i
        // (2n + 1 - i). In ring order a cap faces +X: right for the tip,
        // reversed for the root.
        for (const std::size_t j : {std::size_t{0}, stations - 1}) {
            const auto cap = [&](uint32_t p0, uint32_t p1, uint32_t p2) {
                if (j == 0) {
                    add_triangle(in, v(j, p0), v(j, p2), v(j, p1));
                } else {
                    add_triangle(in, v(j, p0), v(j, p1), v(j, p2));
                }
            };
            cap(0, 1, 2 * n);
            for (uint32_t i = 1; i < n; ++i) {
                cap(i, i + 1, 2 * n - i);
                cap(i, 2 * n - i, 2 * n + 1 - i);
            }
        }
    }

    // The cw hand: the mirror z -> -z, each triangle re-wound to face out.
    if (hand == PropHand::cw) {
        for (glm::dvec3& p : in.positions) p.z = -p.z;
        for (std::size_t i = 0; i < in.indices.size(); i += 3) std::swap(in.indices[i + 1], in.indices[i + 2]);
    }
    return finish(in, hand == PropHand::ccw ? "prop_ccw" : "prop_cw", thinnest);
}

}  // namespace spade::shapes
