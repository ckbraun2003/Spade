#pragma once
// scene.hpp -- the camera spike's scene, camera path, lens tables and the fp64 CPU oracle. One definition shared by the GPU upload and the
// oracle, so a disagreement is the traversal and the arithmetic, not the scene. Bench code: it uses libm freely (nothing here feeds a
// pinned number).

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <vector>

namespace cs {

struct V3 {
    double x = 0, y = 0, z = 0;
};
inline V3 operator+(V3 a, V3 b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
inline V3 operator-(V3 a, V3 b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
inline V3 operator*(V3 a, double s) { return {a.x * s, a.y * s, a.z * s}; }
inline double dot(V3 a, V3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
inline V3 cross(V3 a, V3 b) { return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x}; }
inline double length(V3 a) { return std::sqrt(dot(a, a)); }
inline V3 normalize(V3 a) { return a * (1.0 / length(a)); }

struct Mat3 {
    double m[9]{1, 0, 0, 0, 1, 0, 0, 0, 1};  // row-major
};
inline V3 mul(const Mat3& r, V3 v) {
    return {r.m[0] * v.x + r.m[1] * v.y + r.m[2] * v.z, r.m[3] * v.x + r.m[4] * v.y + r.m[5] * v.z, r.m[6] * v.x + r.m[7] * v.y + r.m[8] * v.z};
}
inline V3 mul_t(const Mat3& r, V3 v) {  // the transpose (the inverse of a rotation)
    return {r.m[0] * v.x + r.m[3] * v.y + r.m[6] * v.z, r.m[1] * v.x + r.m[4] * v.y + r.m[7] * v.z, r.m[2] * v.x + r.m[5] * v.y + r.m[8] * v.z};
}
inline Mat3 rot_y(double a) {
    Mat3 r;
    const double c = std::cos(a), s = std::sin(a);
    r.m[0] = c; r.m[2] = s; r.m[6] = -s; r.m[8] = c;
    return r;
}

inline uint64_t splitmix(uint64_t& s) {
    uint64_t z = (s += 0x9E3779B97F4A7C15ull);
    z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ull;
    z = (z ^ (z >> 27)) * 0x94D049BB133111EBull;
    return z ^ (z >> 31);
}
// A deterministic number in [0, 1) from (env, slot).
inline double hash01(uint32_t env, uint32_t slot) {
    uint64_t s = (static_cast<uint64_t>(env) << 32) ^ (0xABCD0000ull + slot);
    return static_cast<double>(splitmix(s) >> 11) * (1.0 / 9007199254740992.0);
}

// ---- the lens configs (KAT's two P1 operating points; the pinhole is at AIGP's half size, the KB4 numbers are stated, T265-like) ----
struct Config {
    const char* name;
    uint32_t W, H;
    bool grey;
    bool kb4;
    double fx, fy, cx, cy;
    double k[4];
};
inline Config config_by_name(const std::string& n) {
    if (n == "fisheye") return {"fisheye", 384, 384, true, true, 193.0, 193.0, 192.0, 192.0, {-0.008, 0.002, -0.0005, 0.0001}};
    return {"colour", 320, 180, false, false, 160.0, 160.0, 160.0, 90.0, {0, 0, 0, 0}};
}

// Unit ray directions in the camera frame (x right, y down, z forward) on a (W*aa) x (H*aa) grid, float4 each.
inline std::vector<float> make_table(const Config& c, uint32_t aa) {
    const uint32_t tw = c.W * aa, th = c.H * aa;
    std::vector<float> t(static_cast<size_t>(tw) * th * 4);
    for (uint32_t j = 0; j < th; ++j)
        for (uint32_t i = 0; i < tw; ++i) {
            const double px = (i + 0.5) / aa, py = (j + 0.5) / aa;
            const double a = (px - c.cx) / c.fx, b = (py - c.cy) / c.fy;
            V3 d;
            if (!c.kb4) {
                d = normalize({a, b, 1.0});
            } else {
                const double rd = std::sqrt(a * a + b * b);
                if (rd < 1e-12) {
                    d = {0, 0, 1};
                } else {
                    double th_ = rd;  // theta_d = r_d is the image radius over f; solve theta_d = theta (1 + k1 th^2 + ... + k4 th^8) by Newton
                    for (int it = 0; it < 12; ++it) {
                        const double t2 = th_ * th_;
                        const double f = th_ * (1 + t2 * (c.k[0] + t2 * (c.k[1] + t2 * (c.k[2] + t2 * c.k[3])))) - rd;
                        const double fp = 1 + t2 * (3 * c.k[0] + t2 * (5 * c.k[1] + t2 * (7 * c.k[2] + t2 * 9 * c.k[3])));
                        th_ -= f / fp;
                    }
                    d = {std::sin(th_) * a / rd, std::sin(th_) * b / rd, std::cos(th_)};
                }
            }
            const size_t o = (static_cast<size_t>(j) * tw + i) * 4;
            t[o] = static_cast<float>(d.x);
            t[o + 1] = static_cast<float>(d.y);
            t[o + 2] = static_cast<float>(d.z);
            t[o + 3] = 0.0f;
        }
    return t;
}

inline std::vector<float> make_srgb_table() {
    std::vector<float> t(1024);
    for (int i = 0; i < 1024; ++i) {
        const double x = i / 1023.0;
        t[i] = static_cast<float>(x <= 0.0031308 ? 12.92 * x : 1.055 * std::pow(x, 1.0 / 2.4) - 0.055);
    }
    return t;
}

// ---- the scene: per env, four gates, one torus mesh, the ground y = 0 ----
inline constexpr V3 kOuter{1.0, 1.0, 0.15};
inline constexpr V3 kInner{0.65, 0.65, 0.25};
inline constexpr double kEps = 1e-4;
inline constexpr double kTmax = 50.0;
inline constexpr double kLatticeCell = 100.0;
inline constexpr uint32_t kInstancesPerEnv = 5;  // gates 0..3, then the mesh

struct Placed {
    Mat3 R;
    V3 t;
    uint32_t label;
};
struct EnvScene {
    std::array<Placed, 5> inst;  // [0..3] gates, [4] mesh (its base pose; a dynamic frame turns it)
};
inline EnvScene make_env(uint32_t env, uint32_t frame, bool dynamic) {
    EnvScene e;
    for (uint32_t g = 0; g < 4; ++g) {
        e.inst[g].R = rot_y((hash01(env, 10 + g) - 0.5) * 0.6);
        e.inst[g].t = {(hash01(env, 20 + g) - 0.5) * 1.6, 1.4 + (hash01(env, 30 + g) - 0.5) * 0.4, 2.0 + 3.0 * g};
        e.inst[g].label = g + 1;
    }
    const double yaw = 0.3 + (dynamic ? 0.05 * frame : 0.0);
    e.inst[4].R = rot_y(yaw);
    e.inst[4].t = {-1.5 + (hash01(env, 40) - 0.5) * 0.5, 1.0, 6.0};
    e.inst[4].label = 5;
    return e;
}

struct Mesh {
    std::vector<float> verts;  // float4 each
    std::vector<uint32_t> idx;
};
inline Mesh make_torus(uint32_t U = 64, uint32_t Vn = 32, double major = 0.5, double minor = 0.2) {
    Mesh m;
    const double pi2 = 6.283185307179586;
    for (uint32_t u = 0; u < U; ++u)
        for (uint32_t v = 0; v < Vn; ++v) {
            const double a = pi2 * u / U, b = pi2 * v / Vn;
            const double r = major + minor * std::cos(b);
            m.verts.push_back(static_cast<float>(r * std::cos(a)));
            m.verts.push_back(static_cast<float>(minor * std::sin(b)));
            m.verts.push_back(static_cast<float>(r * std::sin(a)));
            m.verts.push_back(0.0f);
        }
    for (uint32_t u = 0; u < U; ++u)
        for (uint32_t v = 0; v < Vn; ++v) {
            const uint32_t u1 = (u + 1) % U, v1 = (v + 1) % Vn;
            const uint32_t a = u * Vn + v, b = u1 * Vn + v, c = u1 * Vn + v1, d = u * Vn + v1;
            m.idx.insert(m.idx.end(), {a, b, c, a, c, d});
        }
    return m;
}

// ---- the camera path ----
struct Pose {
    float pos[3];
    float quat[4];  // x y z w
};
inline Pose camera_pose(uint32_t env, double t) {
    const double ta = std::fmod(t, 2.5);
    const double ha = hash01(env, 1) * 6.28, hb = hash01(env, 2) * 6.28, hc = hash01(env, 3) * 6.28, hd = hash01(env, 4) * 6.28;
    const V3 p{0.5 * std::sin(0.7 * ta + ha), 1.4 + 0.2 * std::sin(0.5 * ta + hb), -6.0 + 4.0 * ta};
    const double yaw = 0.2 * std::sin(0.9 * ta + hc), pitch = -0.05 + 0.05 * std::sin(1.3 * ta + hd);
    const V3 f{std::cos(pitch) * std::sin(yaw), std::sin(pitch), std::cos(pitch) * std::cos(yaw)};
    const V3 right = normalize(V3{-f.z, 0.0, f.x});
    const V3 up = cross(right, f);
    const V3 down = up * -1.0;
    // R = [right | down | f] as columns (camera to world)
    const double m00 = right.x, m01 = down.x, m02 = f.x, m10 = right.y, m11 = down.y, m12 = f.y, m20 = right.z, m21 = down.z, m22 = f.z;
    double qx, qy, qz, qw;
    const double tr = m00 + m11 + m22;
    if (tr > 0) {
        const double s = std::sqrt(tr + 1.0) * 2;
        qw = 0.25 * s; qx = (m21 - m12) / s; qy = (m02 - m20) / s; qz = (m10 - m01) / s;
    } else if (m00 > m11 && m00 > m22) {
        const double s = std::sqrt(1.0 + m00 - m11 - m22) * 2;
        qw = (m21 - m12) / s; qx = 0.25 * s; qy = (m01 + m10) / s; qz = (m02 + m20) / s;
    } else if (m11 > m22) {
        const double s = std::sqrt(1.0 + m11 - m00 - m22) * 2;
        qw = (m02 - m20) / s; qx = (m01 + m10) / s; qy = 0.25 * s; qz = (m12 + m21) / s;
    } else {
        const double s = std::sqrt(1.0 + m22 - m00 - m11) * 2;
        qw = (m10 - m01) / s; qx = (m02 + m20) / s; qy = (m12 + m21) / s; qz = 0.25 * s;
    }
    return {{static_cast<float>(p.x), static_cast<float>(p.y), static_cast<float>(p.z)},
            {static_cast<float>(qx), static_cast<float>(qy), static_cast<float>(qz), static_cast<float>(qw)}};
}
// The shutter samples of frame f at 30 Hz: sample k of N at t_cap - shutter x period x (1 - k/(N-1)); shutter fraction 0.5.
inline double shutter_time(uint32_t frame, uint32_t k, uint32_t n) {
    const double period = 1.0 / 30.0, shutter = 0.5, tcap = frame * period;
    if (n <= 1) return tcap;
    return tcap - shutter * period * (1.0 - static_cast<double>(k) / (n - 1));
}

// ---- the fp64 oracle ----
inline Mat3 quat_to_mat(const float q[4]) {
    const double x = q[0], y = q[1], z = q[2], w = q[3];
    Mat3 r;
    r.m[0] = 1 - 2 * (y * y + z * z); r.m[1] = 2 * (x * y - z * w);     r.m[2] = 2 * (x * z + y * w);
    r.m[3] = 2 * (x * y + z * w);     r.m[4] = 1 - 2 * (x * x + z * z); r.m[5] = 2 * (y * z - x * w);
    r.m[6] = 2 * (x * z - y * w);     r.m[7] = 2 * (y * z + x * w);     r.m[8] = 1 - 2 * (x * x + y * y);
    return r;
}

inline double sd_box(V3 p, V3 h) {
    const V3 q{std::fabs(p.x) - h.x, std::fabs(p.y) - h.y, std::fabs(p.z) - h.z};
    const V3 qm{std::max(q.x, 0.0), std::max(q.y, 0.0), std::max(q.z, 0.0)};
    return length(qm) + std::min(std::max(q.x, std::max(q.y, q.z)), 0.0);
}
inline double gate_sdf(V3 p) { return std::max(sd_box(p, kOuter), -sd_box(p, kInner)); }
inline V3 gate_normal(V3 p) {
    const double e = 1e-3;
    return normalize({gate_sdf({p.x + e, p.y, p.z}) - gate_sdf({p.x - e, p.y, p.z}), gate_sdf({p.x, p.y + e, p.z}) - gate_sdf({p.x, p.y - e, p.z}),
                      gate_sdf({p.x, p.y, p.z + e}) - gate_sdf({p.x, p.y, p.z - e})});
}

struct OHit {
    double t = kTmax;
    uint32_t label = 0;
    V3 colour{};
};

inline V3 sky_colour(V3 d) {
    const double h = std::min(std::max(d.y * 0.5 + 0.5, 0.0), 1.0);
    const V3 a{0.75, 0.85, 0.95}, b{0.30, 0.50, 0.90};
    return a + (b - a) * h;
}

// The scene traced in double: gates by an accurate sphere trace (eps 1e-7), the mesh by brute force, the ground analytically.
inline OHit oracle_trace(const EnvScene& e, const Mesh& mesh, V3 o, V3 d) {
    OHit best;
    best.colour = sky_colour(d);
    const V3 sun = normalize({0.4, 0.8, 0.6});
    const double kAmb = 0.25, kSunI = 0.9;
    double tHit = kTmax;
    V3 nHit{};
    V3 albedo{};
    uint32_t label = 0;
    bool hit = false;
    for (uint32_t g = 0; g < 4; ++g) {
        const Placed& p = e.inst[g];
        const V3 oo = mul_t(p.R, o - p.t), od = mul_t(p.R, d);
        const V3 h{kOuter.x + 0.002, kOuter.y + 0.002, kOuter.z + 0.002};
        double t0 = 1e-4, t1 = std::min(tHit, kTmax);
        bool ok = true;
        const double oc[3] = {oo.x, oo.y, oo.z}, dc[3] = {od.x, od.y, od.z}, hc[3] = {h.x, h.y, h.z};
        for (int a = 0; a < 3; ++a) {
            if (std::fabs(dc[a]) < 1e-14) {
                if (std::fabs(oc[a]) > hc[a]) ok = false;
                continue;
            }
            double ta = (-hc[a] - oc[a]) / dc[a], tb = (hc[a] - oc[a]) / dc[a];
            if (ta > tb) std::swap(ta, tb);
            t0 = std::max(t0, ta);
            t1 = std::min(t1, tb);
        }
        if (!ok || t0 >= t1) continue;
        double t = t0;
        for (int i = 0; i < 600; ++i) {
            const double dist = gate_sdf(oo + od * t);
            if (dist < 1e-7) {
                if (t < tHit) {
                    tHit = t;
                    nHit = mul(p.R, gate_normal(oo + od * t));
                    albedo = {0.90, 0.40, 0.10};
                    label = p.label;
                    hit = true;
                }
                break;
            }
            t += dist;
            if (t > t1) break;
        }
    }
    {
        const Placed& p = e.inst[4];
        const V3 oo = mul_t(p.R, o - p.t), od = mul_t(p.R, d);
        // bounding sphere radius 0.71 about the object origin
        const double bq = dot(oo, od), cq = dot(oo, oo) - 0.71 * 0.71;
        if (bq * bq - cq >= 0) {
            for (size_t i = 0; i + 2 < mesh.idx.size(); i += 3) {
                auto vtx = [&](uint32_t k) { return V3{mesh.verts[k * 4], mesh.verts[k * 4 + 1], mesh.verts[k * 4 + 2]}; };
                const V3 v0 = vtx(mesh.idx[i]), v1 = vtx(mesh.idx[i + 1]), v2 = vtx(mesh.idx[i + 2]);
                const V3 e1 = v1 - v0, e2 = v2 - v0, pv = cross(od, e2);
                const double det = dot(e1, pv);
                if (std::fabs(det) < 1e-14) continue;
                const double inv = 1.0 / det;
                const V3 tv = oo - v0;
                const double u = dot(tv, pv) * inv;
                if (u < 0 || u > 1) continue;
                const V3 qv = cross(tv, e1);
                const double v = dot(od, qv) * inv;
                if (v < 0 || u + v > 1) continue;
                const double t = dot(e2, qv) * inv;
                if (t > 1e-4 && t < tHit) {
                    tHit = t;
                    nHit = mul(p.R, normalize(cross(e1, e2)));
                    albedo = {0.30, 0.60, 0.90};
                    label = p.label;
                    hit = true;
                }
            }
        }
    }
    if (hit) {
        V3 n = nHit;
        if (dot(n, d) > 0) n = n * -1.0;
        const double lam = std::max(dot(n, sun), 0.0);
        best.colour = albedo * (kAmb + kSunI * lam);
        best.t = tHit;
        best.label = label;
    }
    if (d.y < -1e-6) {
        const double tg = -o.y / d.y;
        if (tg > 1e-4 && tg < tHit) {
            const V3 pl = o + d * tg;
            const int cx = static_cast<int>(std::floor(pl.x * 0.5)), cz = static_cast<int>(std::floor(pl.z * 0.5));
            const double chk = (((cx + cz) & 1) == 0) ? 0.45 : 0.35;
            best.colour = V3{chk, chk, chk * 0.9} * (kAmb + kSunI * sun.y);
            best.label = 0;
            best.t = tg;
        }
    }
    return best;
}

inline uint8_t encode_srgb(const std::vector<float>& table, double lin) {
    const double v = std::min(std::max(lin, 0.0), 1.0) * 1023.0;
    const uint32_t i = std::min(static_cast<uint32_t>(v), 1022u);
    const double f = v - i;
    const double e = table[i] + (table[i + 1] - table[i]) * f;
    return static_cast<uint8_t>(static_cast<uint32_t>(e * 255.0 + 0.5));
}

}  // namespace cs
