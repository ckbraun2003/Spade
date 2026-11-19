#include "scene/part_blocks.hpp"

#include <algorithm>
#include <array>
#include <charconv>
#include <cmath>  // std::isfinite and std::fabs only (TD-3)
#include <cstddef>
#include <cstdint>
#include <exception>
#include <set>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include <yaml-cpp/yaml.h>

// ---------------------------------------------------------------------------
// scene/part_blocks.cpp (L88). One pass reads the document, fills the blocks and collects every issue; validate_part_blocks() returns the
// issues, read_part_blocks() the blocks when there is none. The blocks and their rules are in part_blocks.hpp.
// ---------------------------------------------------------------------------

namespace spade::scene {

namespace {

constexpr std::array<std::string_view, 6> kFormatNames = {"step", "3mf", "gltf", "stl", "obj", "spmesh"};

struct UnitName {
    std::string_view name;
    MeshUnit unit;
};
constexpr std::array<UnitName, 6> kUnits = {{{"metre", MeshUnit::metre},
                                             {"millimetre", MeshUnit::millimetre},
                                             {"centimetre", MeshUnit::centimetre},
                                             {"micrometre", MeshUnit::micrometre},
                                             {"inch", MeshUnit::inch},
                                             {"foot", MeshUnit::foot}}};

struct Reader {
    PartBlocks blocks{};
    std::vector<PartIssue> issues{};

    void add(const std::string& field, const std::string& message) {
        issues.push_back(PartIssue{Code::invalid_argument, blocks.id, field, message});
    }

    // Only the listed keys: any other is refused by its path.
    void only_keys(const YAML::Node& map, const std::string& path, std::initializer_list<std::string_view> allowed) {
        for (const auto& kv : map) {
            const std::string key = kv.first.IsScalar() ? kv.first.Scalar() : std::string();
            if (std::find(allowed.begin(), allowed.end(), std::string_view(key)) == allowed.end()) {
                add(path + "." + key, "unknown key '" + key + "'");
            }
        }
    }

    [[nodiscard]] std::optional<double> number(const YAML::Node& node, const std::string& field) {
        if (!node.IsScalar()) {
            add(field, "must be a number");
            return std::nullopt;
        }
        const std::string& text = node.Scalar();
        double value = 0.0;
        const char* end = text.data() + text.size();
        const auto [ptr, ec] = std::from_chars(text.data(), end, value);
        if (text.empty() || ec != std::errc{} || ptr != end || !std::isfinite(value)) {
            add(field, "must be a finite number, found '" + text + "'");
            return std::nullopt;
        }
        return value;
    }

    // `count` finite numbers.
    [[nodiscard]] std::optional<std::vector<double>> numbers(const YAML::Node& node, const std::string& field, std::size_t count) {
        if (!node.IsSequence() || node.size() != count) {
            add(field, "must be a list of " + std::to_string(count) + " numbers");
            return std::nullopt;
        }
        std::vector<double> out;
        out.reserve(count);
        for (std::size_t i = 0; i < count; ++i) {
            const std::optional<double> v = number(node[i], field);
            if (!v) return std::nullopt;
            out.push_back(*v);
        }
        return out;
    }

    [[nodiscard]] std::optional<std::string> text(const YAML::Node& node, const std::string& field) {
        if (!node.IsScalar() || node.Scalar().empty()) {
            add(field, "must be a non-empty string");
            return std::nullopt;
        }
        return node.Scalar();
    }

    void read_geometry(const YAML::Node& node);
    void read_mass(const YAML::Node& node);
    void read_install_points(const YAML::Node& node);
    void read_adjust_range(const YAML::Node& node, const std::string& at, PartInstallPoint& ip);
    void read_meshes(const YAML::Node& node);
    void read_shapes(const YAML::Node& node);
    [[nodiscard]] std::optional<glm::dquat> unit_quaternion(const YAML::Node& node, const std::string& field);
    void read_source(const YAML::Node& node);
    void read_material(const YAML::Node& node);
    void read_citations(const YAML::Node& node);
};

[[nodiscard]] bool is_hex64(const std::string& s) {
    return s.size() == 64 && std::all_of(s.begin(), s.end(), [](char c) { return (c >= '0' && c <= '9') || (c >= 'a' && c <= 'f'); });
}

// Source axis names: x, -x, y, -y, z, -z -> +-1, +-2, +-3.
[[nodiscard]] int axis_of(const std::string& s) {
    if (s == "x") return 1;
    if (s == "-x") return -1;
    if (s == "y") return 2;
    if (s == "-y") return -2;
    if (s == "z") return 3;
    if (s == "-z") return -3;
    return 0;
}

// The determinant of the matrix whose row i is the signed unit vector of axes[i].
[[nodiscard]] int axes_determinant(const std::array<int, 3>& axes) {
    int m[3][3] = {};
    for (int i = 0; i < 3; ++i) m[i][std::abs(axes[static_cast<std::size_t>(i)]) - 1] = axes[static_cast<std::size_t>(i)] > 0 ? 1 : -1;
    return m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1]) - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0]) +
           m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
}

void Reader::read_geometry(const YAML::Node& node) {
    if (!node.IsMap()) {
        add("geometry", "must be a mapping");
        return;
    }
    only_keys(node, "geometry", {"source", "to_part", "fallback"});
    PartGeometry g;
    bool have_format = false;

    const YAML::Node src = node["source"];
    if (!src.IsDefined() || !src.IsMap()) {
        add("geometry.source", "is required and must be a mapping");
    } else {
        only_keys(src, "geometry.source", {"format", "sha256", "object", "unit"});
        if (const YAML::Node f = src["format"]; !f.IsDefined()) {
            add("geometry.source.format", "is required");
        } else if (const std::optional<std::string> s = text(f, "geometry.source.format")) {
            const auto it = std::find(kFormatNames.begin(), kFormatNames.end(), std::string_view(*s));
            if (it == kFormatNames.end()) {
                add("geometry.source.format", "'" + *s + "' is not one of step, 3mf, gltf, stl, obj, spmesh");
            } else {
                g.format = static_cast<GeometryFormat>(it - kFormatNames.begin());
                have_format = true;
            }
        }
        if (const YAML::Node h = src["sha256"]; !h.IsDefined()) {
            add("geometry.source.sha256", "is required");
        } else if (const std::optional<std::string> s = text(h, "geometry.source.sha256")) {
            if (!is_hex64(*s)) {
                add("geometry.source.sha256", "must be 64 lower-case hex characters, the source file's SHA-256");
            } else {
                g.sha256 = *s;
            }
        }
        if (const YAML::Node o = src["object"]; o.IsDefined()) {
            if (const std::optional<std::string> s = text(o, "geometry.source.object")) g.object = *s;
        }
        const YAML::Node u = src["unit"];
        const bool needs_unit = have_format && (g.format == GeometryFormat::stl || g.format == GeometryFormat::obj);
        if (u.IsDefined()) {
            if (have_format && !needs_unit) {
                add("geometry.source.unit", "is only for stl and obj; this format declares its own unit");
            } else if (const std::optional<std::string> s = text(u, "geometry.source.unit")) {
                const auto it = std::find_if(kUnits.begin(), kUnits.end(), [&](const UnitName& n) { return n.name == *s; });
                if (it == kUnits.end()) {
                    add("geometry.source.unit", "'" + *s + "' is not one of metre, millimetre, centimetre, micrometre, inch, foot");
                } else {
                    g.unit = it->unit;
                }
            }
        } else if (needs_unit) {
            add("geometry.source.unit", "is required for stl and obj, which carry no unit");
        }
    }

    if (const YAML::Node t = node["to_part"]; t.IsDefined()) {
        if (!t.IsMap()) {
            add("geometry.to_part", "must be a mapping");
        } else {
            only_keys(t, "geometry.to_part", {"axes", "scale"});
            if (const YAML::Node a = t["axes"]; a.IsDefined()) {
                if (!a.IsSequence() || a.size() != 3) {
                    add("geometry.to_part.axes", "must be three of x, -x, y, -y, z, -z");
                } else {
                    std::array<int, 3> axes{};
                    bool ok = true;
                    for (std::size_t i = 0; i < 3; ++i) {
                        axes[i] = a[i].IsScalar() ? axis_of(a[i].Scalar()) : 0;
                        ok = ok && axes[i] != 0;
                    }
                    if (!ok) {
                        add("geometry.to_part.axes", "must be three of x, -x, y, -y, z, -z");
                    } else if (std::abs(axes[0]) == std::abs(axes[1]) || std::abs(axes[0]) == std::abs(axes[2]) ||
                               std::abs(axes[1]) == std::abs(axes[2])) {
                        add("geometry.to_part.axes", "must use each source axis once (a permutation with signs)");
                    } else if (axes_determinant(axes) != 1) {
                        add("geometry.to_part.axes", "is a mirror (determinant -1), which reverses winding; flip one more axis to make it a rotation");
                    } else {
                        g.axes = axes;
                    }
                }
            }
            if (const YAML::Node s = t["scale"]; s.IsDefined()) {
                if (const std::optional<double> v = number(s, "geometry.to_part.scale")) {
                    if (!(*v > 0.0)) {
                        add("geometry.to_part.scale", "must be above 0");
                    } else {
                        g.scale = *v;
                    }
                }
            }
        }
    }

    if (const YAML::Node f = node["fallback"]; f.IsDefined()) {
        if (!f.IsMap()) {
            add("geometry.fallback", "must be a mapping");
        } else {
            only_keys(f, "geometry.fallback", {"kind", "size"});
            GeometryFallback fb;
            std::size_t want = 0;
            bool kind_ok = false;
            if (const std::optional<std::string> k = text(f["kind"], "geometry.fallback.kind")) {
                if (*k == "box") {
                    fb.kind = GeometryFallback::Kind::box;
                    want = 3;
                    kind_ok = true;
                } else if (*k == "tube") {
                    fb.kind = GeometryFallback::Kind::tube;
                    want = 2;
                    kind_ok = true;
                } else if (*k == "cylinder") {
                    fb.kind = GeometryFallback::Kind::cylinder;
                    want = 2;
                    kind_ok = true;
                } else {
                    add("geometry.fallback.kind", "'" + *k + "' is not one of box, tube, cylinder");
                }
            }
            if (kind_ok) {
                if (const std::optional<std::vector<double>> v = numbers(f["size"], "geometry.fallback.size", want)) {
                    if (std::any_of(v->begin(), v->end(), [](double d) { return !(d > 0.0); })) {
                        add("geometry.fallback.size", "every size must be above 0");
                    } else {
                        fb.size = *v;
                        g.fallback = fb;
                    }
                }
            }
        }
    }
    blocks.geometry = g;
}

void Reader::read_mass(const YAML::Node& node) {
    if (!node.IsMap()) {
        add("mass_properties", "must be a mapping");
        return;
    }
    only_keys(node, "mass_properties", {"mass", "com", "inertia"});
    PartMassProperties m;
    if (!node["mass"].IsDefined()) {
        add("mass_properties.mass", "is required");
    } else if (const std::optional<double> v = number(node["mass"], "mass_properties.mass")) {
        if (!(*v > 0.0)) {
            add("mass_properties.mass", "must be above 0 kg");
        } else {
            m.mass = *v;
        }
    }
    if (const YAML::Node c = node["com"]; c.IsDefined()) {
        if (const std::optional<std::vector<double>> v = numbers(c, "mass_properties.com", 3)) m.com = glm::dvec3((*v)[0], (*v)[1], (*v)[2]);
    }
    if (const YAML::Node t = node["inertia"]; t.IsDefined()) {
        if (!node["com"].IsDefined()) {
            add("mass_properties.inertia", "needs `com`: the tensor is about the centre of mass");
        } else if (const std::optional<std::vector<double>> v = numbers(t, "mass_properties.inertia", 9)) {
            const std::vector<double>& a = *v;  // row-major
            double big = 0.0;
            for (double d : a) big = std::max(big, std::fabs(d));
            const double tol = 1e-9 * big;
            const bool symmetric = std::fabs(a[1] - a[3]) <= tol && std::fabs(a[2] - a[6]) <= tol && std::fabs(a[5] - a[7]) <= tol;
            // A symmetric 3x3 [m00 m01 m02; . m11 m12; . . m22]: positive definite by its leading minors; positive semi-definite (to a
            // relative 1e-12) by its diagonal, its three principal 2x2 minors and its determinant. J = tr/2 - I is PSD exactly when the
            // principal moments obey the triangle inequality.
            const auto det3 = [](double m00, double m01, double m02, double m11, double m12, double m22) {
                return m00 * (m11 * m22 - m12 * m12) - m01 * (m01 * m22 - m12 * m02) + m02 * (m01 * m12 - m11 * m02);
            };
            const auto pd = [&](double m00, double m01, double m02, double m11, double m12, double m22) {
                return m00 > 0.0 && m00 * m11 - m01 * m01 > 0.0 && det3(m00, m01, m02, m11, m12, m22) > 0.0;
            };
            const auto psd = [&](double m00, double m01, double m02, double m11, double m12, double m22) {
                const double t1 = 1e-12 * big, t2 = t1 * big, t3 = t2 * big;
                return m00 >= -t1 && m11 >= -t1 && m22 >= -t1 && m00 * m11 - m01 * m01 >= -t2 && m00 * m22 - m02 * m02 >= -t2 &&
                       m11 * m22 - m12 * m12 >= -t2 && det3(m00, m01, m02, m11, m12, m22) >= -t3;
            };
            if (!symmetric) {
                add("mass_properties.inertia", "must be symmetric (the off-diagonal pairs differ)");
            } else if (!pd(a[0], a[1], a[2], a[4], a[5], a[8])) {
                add("mass_properties.inertia", "must be positive definite (a body's tensor)");
            } else {
                const double half = 0.5 * (a[0] + a[4] + a[8]);
                if (!psd(half - a[0], -a[1], -a[2], half - a[4], -a[5], half - a[8])) {
                    add("mass_properties.inertia", "breaks the triangle inequality of its principal moments (no body has it)");
                } else {
                    glm::dmat3 I(0.0);
                    for (int r = 0; r < 3; ++r) {
                        for (int c = 0; c < 3; ++c) I[c][r] = a[static_cast<std::size_t>(r * 3 + c)];
                    }
                    m.inertia = I;
                }
            }
        }
    }
    blocks.mass_properties = m;
}

void Reader::read_install_points(const YAML::Node& node) {
    if (!node.IsSequence()) {
        add("install_points", "must be a list");
        return;
    }
    std::set<std::string> names;
    for (std::size_t i = 0; i < node.size(); ++i) {
        const std::string at = "install_points[" + std::to_string(i) + "]";
        const YAML::Node p = node[i];
        if (!p.IsMap()) {
            add(at, "must be a mapping");
            continue;
        }
        only_keys(p, at, {"name", "kind", "pos", "orient", "spin", "adjust_range"});
        PartInstallPoint ip;
        if (const std::optional<std::string> s = text(p["name"], at + ".name")) {
            if (!names.insert(*s).second) {
                add(at + ".name", "'" + *s + "' is used twice in this part");
            } else {
                ip.name = *s;
            }
        }
        std::string kind;
        if (const std::optional<std::string> s = text(p["kind"], at + ".kind")) {
            if (std::find(kInstallPointKinds.begin(), kInstallPointKinds.end(), std::string_view(*s)) == kInstallPointKinds.end()) {
                add(at + ".kind", "'" + *s + "' is not one of motor, prop_hub, battery, esc, fc, companion, camera, gnss, radio, imu, vtx, antenna");
            } else {
                ip.kind = kind = *s;
            }
        }
        if (const std::optional<std::vector<double>> v = numbers(p["pos"], at + ".pos", 3)) ip.pos = glm::dvec3((*v)[0], (*v)[1], (*v)[2]);
        if (const std::optional<glm::dquat> q = unit_quaternion(p["orient"], at + ".orient")) ip.orient = *q;
        if (const YAML::Node a = p["adjust_range"]; a.IsDefined()) read_adjust_range(a, at, ip);
        if (const YAML::Node s = p["spin"]; s.IsDefined()) {
            if (!kind.empty() && kind != "motor") {
                add(at + ".spin", "is only for a motor point");
            } else if (const std::optional<double> v = number(s, at + ".spin")) {
                if (*v != 1.0 && *v != -1.0) {
                    add(at + ".spin", "must be +1 or -1");
                } else {
                    ip.spin = static_cast<int>(*v);
                }
            }
        }
        blocks.install_points.push_back(ip);
    }
}

std::optional<glm::dquat> Reader::unit_quaternion(const YAML::Node& node, const std::string& field) {
    const std::optional<std::vector<double>> v = numbers(node, field, 4);
    if (!v) return std::nullopt;
    const double n2 = (*v)[0] * (*v)[0] + (*v)[1] * (*v)[1] + (*v)[2] * (*v)[2] + (*v)[3] * (*v)[3];
    if (std::fabs(n2 - 1.0) > 2e-6) {
        add(field, "must be a unit quaternion [w, x, y, z] (length 1 to 1e-6)");
        return std::nullopt;
    }
    return glm::dquat((*v)[0], (*v)[1], (*v)[2], (*v)[3]);
}

// {pos_min, pos_max, yaw_deg, pitch_deg, roll_deg}: every key optional (a missing one is no range), every min at most its max, an angle within 360.
void Reader::read_adjust_range(const YAML::Node& node, const std::string& at, PartInstallPoint& ip) {
    const std::string field = at + ".adjust_range";
    if (!node.IsMap()) {
        add(field, "must be a mapping");
        return;
    }
    only_keys(node, field, {"pos_min", "pos_max", "yaw_deg", "pitch_deg", "roll_deg"});
    AdjustRange r;
    bool vectors_ok = true;
    const auto vec = [&](const char* key, glm::dvec3& out) {
        const YAML::Node n = node[key];
        if (!n.IsDefined()) return;
        if (const std::optional<std::vector<double>> v = numbers(n, field + "." + key, 3)) {
            out = glm::dvec3((*v)[0], (*v)[1], (*v)[2]);
        } else {
            vectors_ok = false;
        }
    };
    vec("pos_min", r.pos_min);
    vec("pos_max", r.pos_max);
    if (vectors_ok && (r.pos_min.x > r.pos_max.x || r.pos_min.y > r.pos_max.y || r.pos_min.z > r.pos_max.z)) {
        add(field + ".pos_min", "must not exceed pos_max on any axis");
    }
    const auto angles = [&](const char* key, std::array<double, 2>& out) {
        const YAML::Node n = node[key];
        if (!n.IsDefined()) return;
        const std::optional<std::vector<double>> v = numbers(n, field + "." + key, 2);
        if (!v) return;
        if ((*v)[0] > (*v)[1]) {
            add(field + "." + key, "min must not exceed max");
        } else if (std::fabs((*v)[0]) > 360.0 || std::fabs((*v)[1]) > 360.0) {
            add(field + "." + key, "an angle is within 360 degrees");
        } else {
            out = {(*v)[0], (*v)[1]};
        }
    };
    angles("yaw_deg", r.yaw_deg);
    angles("pitch_deg", r.pitch_deg);
    angles("roll_deg", r.roll_deg);
    ip.adjust_range = r;
}

// A bare file name: no directory part, not . or .., no control character.
[[nodiscard]] bool bare_file_name(const std::string& s) {
    if (s == "." || s == "..") return false;
    return std::none_of(s.begin(), s.end(), [](char c) { return c == '/' || c == '\\' || static_cast<unsigned char>(c) < 0x20 || c == 0x7f; });
}

[[nodiscard]] bool is_hex16(const std::string& s) {
    return s.size() == 16 && std::all_of(s.begin(), s.end(), [](char c) { return (c >= '0' && c <= '9') || (c >= 'a' && c <= 'f'); });
}

void Reader::read_meshes(const YAML::Node& node) {
    if (!node.IsSequence() || node.size() == 0) {
        add("meshes", "must be a non-empty list of {store: {sha256, file}, hash, role}");
        return;
    }
    std::set<std::pair<std::string, std::string>> seen;
    for (std::size_t i = 0; i < node.size(); ++i) {
        const std::string at = "meshes[" + std::to_string(i) + "]";
        const YAML::Node m = node[i];
        if (!m.IsMap()) {
            add(at, "must be a mapping");
            continue;
        }
        only_keys(m, at, {"store", "hash", "role"});
        PartMesh pm;
        bool store_ok = false;
        const YAML::Node store = m["store"];
        if (!store.IsMap()) {
            add(at + ".store", "must be {sha256, file}: the part store's reference");
        } else {
            only_keys(store, at + ".store", {"sha256", "file"});
            bool sha_ok = false, file_ok = false;
            if (const std::optional<std::string> s = text(store["sha256"], at + ".store.sha256")) {
                if (!is_hex64(*s)) {
                    add(at + ".store.sha256", "must be 64 lower-case hex characters");
                } else {
                    pm.sha256 = *s;
                    sha_ok = true;
                }
            }
            if (const std::optional<std::string> s = text(store["file"], at + ".store.file")) {
                if (!bare_file_name(*s)) {
                    add(at + ".store.file", "must be a bare file name, with no directory part");
                } else {
                    pm.file = *s;
                    file_ok = true;
                }
            }
            store_ok = sha_ok && file_ok;
        }
        if (store_ok && !seen.insert({pm.sha256, pm.file}).second) {
            add(at + ".store", "this {sha256, file} is already listed in meshes");
        }
        if (const YAML::Node h = m["hash"]; h.IsDefined()) {
            if (const std::optional<std::string> s = text(h, at + ".hash")) {
                if (!is_hex16(*s)) {
                    add(at + ".hash", "must be the mesh's content hash: 16 lower-case hex characters");
                } else {
                    pm.hash = *s;
                }
            }
        }
        if (const YAML::Node role = m["role"]; role.IsDefined()) {
            if (const std::optional<std::string> s = text(role, at + ".role")) {
                if (*s == "visual") {
                    pm.visual = true;
                } else if (*s != "solid") {
                    add(at + ".role", "is solid (the default) or visual");
                }
            }
        }
        blocks.meshes.push_back(pm);
    }
}

void Reader::read_shapes(const YAML::Node& node) {
    if (!node.IsSequence() || node.size() == 0) {
        add("shapes", "must be a non-empty list of {kind: box | cylinder, size}");
        return;
    }
    for (std::size_t i = 0; i < node.size(); ++i) {
        const std::string at = "shapes[" + std::to_string(i) + "]";
        const YAML::Node s = node[i];
        if (!s.IsMap()) {
            add(at, "must be a mapping");
            continue;
        }
        only_keys(s, at, {"kind", "size", "pos", "orient"});
        PartShape shape;
        std::size_t count = 0;
        if (const std::optional<std::string> k = text(s["kind"], at + ".kind")) {
            if (*k == "box") {
                shape.kind = PartShape::Kind::box;
                count = 3;
            } else if (*k == "cylinder") {
                shape.kind = PartShape::Kind::cylinder;
                count = 2;
            } else {
                add(at + ".kind", "'" + *k + "' is not box or cylinder");
            }
        }
        if (count != 0) {
            if (const std::optional<std::vector<double>> v = numbers(s["size"], at + ".size", count)) {
                if (std::any_of(v->begin(), v->end(), [](double x) { return x <= 0.0; })) {
                    add(at + ".size", "every size must be above 0");
                } else {
                    shape.size = *v;
                }
            }
        }
        if (const YAML::Node p = s["pos"]; p.IsDefined()) {
            if (const std::optional<std::vector<double>> v = numbers(p, at + ".pos", 3)) shape.pos = glm::dvec3((*v)[0], (*v)[1], (*v)[2]);
        }
        if (const YAML::Node o = s["orient"]; o.IsDefined()) {
            if (const std::optional<glm::dquat> q = unit_quaternion(o, at + ".orient")) shape.orient = *q;
        }
        blocks.shapes.push_back(shape);
    }
}

void Reader::read_source(const YAML::Node& node) {
    if (!node.IsMap()) {
        add("source", "must be a mapping");
        return;
    }
    only_keys(node, "source", {"id", "url", "license", "retrieved"});
    PartSourceRecord r;
    if (const std::optional<std::string> s = text(node["id"], "source.id")) r.id = *s;
    if (const std::optional<std::string> s = text(node["url"], "source.url")) r.url = *s;
    if (const std::optional<std::string> s = text(node["license"], "source.license")) r.license = *s;
    if (const std::optional<std::string> s = text(node["retrieved"], "source.retrieved")) {
        const std::string& d = *s;
        const bool shape = d.size() == 10 && d[4] == '-' && d[7] == '-' &&
                           std::all_of(d.begin(), d.end(), [](char c) { return c == '-' || (c >= '0' && c <= '9'); });
        const int month = shape ? std::stoi(d.substr(5, 2)) : 0;
        const int day = shape ? std::stoi(d.substr(8, 2)) : 0;
        if (!shape || month < 1 || month > 12 || day < 1 || day > 31) {
            add("source.retrieved", "must be a date, YYYY-MM-DD");
        } else {
            r.retrieved = d;
        }
    }
    blocks.source = r;
}

void Reader::read_material(const YAML::Node& node) {
    if (const std::optional<std::string> s = text(node, "material")) {
        const bool control = std::any_of(s->begin(), s->end(), [](char c) { return static_cast<unsigned char>(c) < 0x20 || c == 0x7f; });
        if (s->size() > 64 || control) {
            add("material", "must be at most 64 characters with no control character");
        } else {
            blocks.material = *s;
        }
    }
}

// The citations under `sources`: the block paths only; every other entry is KAT's.
void Reader::read_citations(const YAML::Node& node) {
    std::set<std::string> cited;
    if (node.IsDefined() && !node.IsMap()) {
        add("sources", "must be a mapping from a field's path to its citation");
    } else if (node.IsDefined()) {
        for (const auto& kv : node) {
            const std::string path = kv.first.IsScalar() ? kv.first.Scalar() : std::string();
            const bool ours = path == "mass_properties.mass" || path == "mass_properties.com" || path == "mass_properties.inertia" ||
                              path.rfind("install_points.", 0) == 0;
            if (!ours) continue;
            const YAML::Node c = kv.second;
            if (!c.IsMap()) {
                add(path, "its citation must be {source, grade, note}");
                continue;
            }
            only_keys(c, path, {"source", "grade", "note"});
            SourceRef ref;
            bool ok = true;
            // A missing key is an undefined node, and asking one its type throws: test IsDefined() first (an `assumed` citation has a note and no `source`).
            const auto text = [&](const char* key) { const YAML::Node v = c[key]; return v.IsDefined() && v.IsScalar() ? v.Scalar() : std::string(); };
            const auto has_text = [&](const char* key) { const YAML::Node v = c[key]; return v.IsDefined() && v.IsScalar(); };
            ref.source = text("source");
            ref.note = text("note");
            if (!has_text("grade")) {
                add(path, "its citation needs a grade");
                ok = false;
            } else if (const Result<Grade> g = parse_grade(text("grade"), path); !g) {
                add(path, "its grade '" + text("grade") + "' is not one of measured, published, retailer, derived, fitted, assumed");
                ok = false;
            } else {
                ref.grade = *g;
            }
            if (ok && ref.grade != Grade::assumed && ref.source.empty()) {
                add(path, "a grade other than assumed cites a source");
                ok = false;
            }
            if (ok && (ref.grade == Grade::derived || ref.grade == Grade::assumed) && ref.note.empty()) {
                add(path, "a " + std::string(grade_name(ref.grade)) + " value carries a note");
                ok = false;
            }
            if (ok) {
                cited.insert(path);
                blocks.sources[path] = ref;
            }
        }
    }
    const auto need = [&](const std::string& path) {
        if (cited.count(path) == 0 && std::none_of(issues.begin(), issues.end(), [&](const PartIssue& i) { return i.field == path; })) {
            add(path, "no citation: cite it under `sources` as {source, grade, note}");
        }
    };
    if (blocks.mass_properties) {
        need("mass_properties.mass");
        if (blocks.mass_properties->com) need("mass_properties.com");
        if (blocks.mass_properties->inertia) need("mass_properties.inertia");
    }
    for (const PartInstallPoint& p : blocks.install_points) {
        if (!p.name.empty()) need("install_points." + p.name);
    }
}

Reader read(std::string_view text) {
    Reader r;
    YAML::Node root;
    try {
        root = YAML::Load(std::string(text));
    } catch (const std::exception& e) {
        r.add("", std::string("the text is not valid YAML or JSON: ") + e.what());
        return r;
    }
    if (!root.IsMap()) {
        r.add("", "the document must be a mapping");
        return r;
    }
    if (const YAML::Node id = root["id"]; id.IsScalar() && !id.Scalar().empty()) {
        r.blocks.id = id.Scalar();
    } else {
        r.add("id", "is required: a non-empty string");
    }
    const YAML::Node geometry = root["geometry"], mass = root["mass_properties"], points = root["install_points"],
                     source = root["source"], material = root["material"], meshes = root["meshes"], shapes = root["shapes"];
    if (!geometry.IsDefined() && !mass.IsDefined() && !points.IsDefined() && !source.IsDefined() && !material.IsDefined() &&
        !meshes.IsDefined() && !shapes.IsDefined()) {
        r.add("", "the document has no part block: give at least one of geometry, mass_properties, install_points, source, material, meshes, shapes");
        return r;
    }
    if (geometry.IsDefined()) r.read_geometry(geometry);
    if (mass.IsDefined()) r.read_mass(mass);
    if (points.IsDefined()) r.read_install_points(points);
    if (source.IsDefined()) r.read_source(source);
    if (geometry.IsDefined() && !source.IsDefined()) r.add("source", "is required with `geometry`: where the file came from");
    if (material.IsDefined()) r.read_material(material);
    if (meshes.IsDefined()) r.read_meshes(meshes);
    if (shapes.IsDefined()) r.read_shapes(shapes);
    r.read_citations(root["sources"]);
    return r;
}

}  // namespace

std::string_view geometry_format_name(GeometryFormat format) noexcept {
    const auto i = static_cast<std::size_t>(format);
    return i < kFormatNames.size() ? kFormatNames[i] : std::string_view{};
}

std::vector<PartIssue> validate_part_blocks(std::string_view text) { return read(text).issues; }

Result<PartBlocks> read_part_blocks(std::string_view text) {
    Reader r = read(text);
    if (!r.issues.empty()) {
        const PartIssue& i = r.issues.front();
        return std::unexpected(Error{i.code, "part '" + i.part_id + "': " + (i.field.empty() ? std::string() : i.field + ": ") + i.message});
    }
    return std::move(r.blocks);
}

}  // namespace spade::scene
