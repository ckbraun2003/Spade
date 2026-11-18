// build_parts.hpp -- the real-parts drone of the web clips (the lead, 2026-11-18, L88l): the X500 frame plus a flight controller, GPS mast,
// battery, companion computer, camera and radio, drawn PRESENTATION ONLY over the held body, from the plain table Core writes
// (tools/flow_movie/data/x500_build/parts.yaml). The flow's solids are the model's and are untouched: nothing here reaches the solve.
//
// The table is one list `parts:`; each entry has `name` (unique), `kind` (frame controller gps battery computer camera radio mast),
// `mass_kg` (Physics' business: read, kept, not drawn), `pose: {position: [x, y, z] m, orientation: [w, x, y, z]}` in the X500 body frame
// (y up, +X forward, the origin the model's centre of mass, the frame of KAT's glTF), and ONE shape: `objects` (a list of `store_ref:
// {sha256, file}` with an optional `offset: {position, orientation}`; the file is <store>/<sha256>/<file>, a .spmesh or a .gltf/.glb),
// `box: [sx, sy, sz]` m, or `cylinder: {radius, length}` m along the part's local y, centred. Unknown kinds and keys are refused.
//
// Colours: a part's .spmesh, box and cylinder take its KIND's colour; a glTF keeps its own palette (the X500 frame's materials).
#pragma once

#include <array>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <numbers>
#include <optional>
#include <set>
#include <string>
#include <string_view>
#include <vector>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <yaml-cpp/yaml.h>

#include "core/error.hpp"
#include "render/gltf.hpp"
#include "render/mesh_resolver.hpp"
#include "render/scene.hpp"
#include "visual_mesh.hpp"
#include "world/mesh_file.hpp"

namespace flow_movie {

enum class BuildShape { objects, box, cylinder };

struct BuildObject {
    std::string sha256;
    std::string file;
    glm::vec3 offset_position{0.0f};
    glm::quat offset_orient{1.0f, 0.0f, 0.0f, 0.0f};
};

struct BuildPart {
    std::string name;
    std::string kind;
    double mass_kg = 0.0;
    glm::vec3 position{0.0f};
    glm::quat orientation{1.0f, 0.0f, 0.0f, 0.0f};
    BuildShape shape = BuildShape::box;
    std::vector<BuildObject> objects;
    glm::vec3 box{0.0f};
    float radius = 0.0f;
    float length = 0.0f;
};

namespace build_detail {

inline constexpr std::array<std::string_view, 8> kKinds{"frame", "controller", "gps", "battery", "computer", "camera", "radio", "mast"};

[[nodiscard]] inline std::unexpected<spade::Error> bad(const std::string& why) {
    return std::unexpected(spade::Error{spade::Code::invalid_argument, "build parts: " + why});
}

// Every key of `node` must be one of `allowed`.
[[nodiscard]] inline std::optional<std::string> unknown_key(const YAML::Node& node, std::initializer_list<std::string_view> allowed) {
    for (const auto& kv : node) {
        const std::string key = kv.first.as<std::string>();
        bool known = false;
        for (std::string_view a : allowed) known = known || a == key;
        if (!known) return key;
    }
    return std::nullopt;
}

[[nodiscard]] inline spade::Result<std::vector<double>> numbers(const YAML::Node& node, std::size_t count, const std::string& what) {
    if (!node || !node.IsSequence() || node.size() != count) return bad(what + " is not a list of " + std::to_string(count) + " numbers");
    std::vector<double> out;
    for (const auto& item : node) {
        double v = 0.0;
        try {
            v = item.as<double>();
        } catch (const YAML::Exception&) {
            return bad(what + " has an item that is not a number");
        }
        if (!std::isfinite(v)) return bad(what + " is not finite");
        out.push_back(v);
    }
    return out;
}

struct PoseRead {
    glm::vec3 position{0.0f};
    glm::quat orientation{1.0f, 0.0f, 0.0f, 0.0f};
};

[[nodiscard]] inline spade::Result<PoseRead> pose_of(const YAML::Node& node, const std::string& what) {
    if (!node || !node.IsMap()) return bad(what + " has no pose {position, orientation}");
    if (const auto key = unknown_key(node, {"position", "orientation"})) return bad(what + ": unknown key '" + *key + "' in a pose");
    const auto p = numbers(node["position"], 3, what + " position");
    if (!p) return std::unexpected(p.error());
    const auto q = numbers(node["orientation"], 4, what + " orientation");
    if (!q) return std::unexpected(q.error());
    const double norm = std::sqrt((*q)[0] * (*q)[0] + (*q)[1] * (*q)[1] + (*q)[2] * (*q)[2] + (*q)[3] * (*q)[3]);
    if (std::abs(norm - 1.0) > 1e-3) return bad(what + " orientation is not a unit quaternion (norm " + std::to_string(norm) + ")");
    PoseRead out;
    out.position = glm::vec3(static_cast<float>((*p)[0]), static_cast<float>((*p)[1]), static_cast<float>((*p)[2]));
    out.orientation = glm::normalize(glm::quat(static_cast<float>((*q)[0]), static_cast<float>((*q)[1]), static_cast<float>((*q)[2]),
                                               static_cast<float>((*q)[3])));
    return out;
}

[[nodiscard]] inline bool is_hex64(const std::string& s) {
    if (s.size() != 64u) return false;
    for (char c : s) {
        if (!((c >= '0' && c <= '9') || (c >= 'a' && c <= 'f'))) return false;
    }
    return true;
}

[[nodiscard]] inline std::string extension_of(const std::string& file) {
    const std::size_t dot = file.rfind('.');
    return dot == std::string::npos ? std::string() : file.substr(dot);
}

}  // namespace build_detail

[[nodiscard]] inline bool kind_known(std::string_view kind) {
    for (std::string_view k : build_detail::kKinds) {
        if (k == kind) return true;
    }
    return false;
}

// A part kind's colour (lit, matte): nothing for a kind the table does not have. Distinct per kind, the frame a neutral grey.
[[nodiscard]] inline std::optional<spade::render::Material> kind_material(std::string_view kind) {
    const auto make = [](float r, float g, float b) {
        spade::render::Material m;
        m.base_color = glm::vec4(r, g, b, 1.0f);
        return m;
    };
    if (kind == "frame") return make(0.30f, 0.31f, 0.33f);
    if (kind == "controller") return make(0.10f, 0.45f, 0.30f);
    if (kind == "gps") return make(0.85f, 0.85f, 0.88f);
    if (kind == "battery") return make(0.85f, 0.45f, 0.10f);
    if (kind == "computer") return make(0.15f, 0.55f, 0.20f);
    if (kind == "camera") return make(0.12f, 0.12f, 0.14f);
    if (kind == "radio") return make(0.20f, 0.30f, 0.75f);
    if (kind == "mast") return make(0.60f, 0.60f, 0.62f);
    return std::nullopt;
}

// The table's parts, in file order. Refused, every message prefixed "build parts:" and naming the part and the piece.
[[nodiscard]] inline spade::Result<std::vector<BuildPart>> parse_build_parts(std::string_view text) {
    using build_detail::bad;
    YAML::Node root;
    try {
        root = YAML::Load(std::string(text));
    } catch (const YAML::Exception& e) {
        return bad(std::string("not YAML: ") + e.what());
    }
    try {
        if (!root || !root.IsMap() || !root["parts"]) return bad("the table has no `parts` list");
        if (const auto key = build_detail::unknown_key(root, {"parts"})) return bad("unknown key '" + *key + "' at the top (only `parts`)");
        const YAML::Node list = root["parts"];
        if (!list.IsSequence() || list.size() == 0u) return bad("the table has no parts");
        std::vector<BuildPart> parts;
        std::set<std::string> names;
        for (const auto& entry : list) {
            if (!entry.IsMap()) return bad("a part is not a map");
            BuildPart part;
            part.name = entry["name"] ? entry["name"].as<std::string>() : std::string();
            if (part.name.empty()) return bad("a part has no name");
            const std::string who = "part '" + part.name + "'";
            if (!names.insert(part.name).second) return bad(who + " is listed twice");
            if (const auto key = build_detail::unknown_key(entry, {"name", "kind", "mass_kg", "pose", "objects", "box", "cylinder"})) {
                return bad(who + ": unknown key '" + *key + "'");
            }
            part.kind = entry["kind"] ? entry["kind"].as<std::string>() : std::string();
            if (!kind_known(part.kind)) return bad(who + ": unknown kind '" + part.kind + "'");
            if (entry["mass_kg"]) {
                part.mass_kg = entry["mass_kg"].as<double>();
                if (!(std::isfinite(part.mass_kg) && part.mass_kg >= 0.0)) return bad(who + ": mass_kg is not a finite number from 0 up");
            }
            const auto pose = build_detail::pose_of(entry["pose"], who);
            if (!pose) return std::unexpected(pose.error());
            part.position = pose->position;
            part.orientation = pose->orientation;
            const int shapes = (entry["objects"] ? 1 : 0) + (entry["box"] ? 1 : 0) + (entry["cylinder"] ? 1 : 0);
            if (shapes != 1) return bad(who + " needs exactly one shape (objects, box or cylinder), has " + std::to_string(shapes));
            if (entry["box"]) {
                const auto b = build_detail::numbers(entry["box"], 3, who + " box");
                if (!b) return std::unexpected(b.error());
                if (!((*b)[0] > 0.0 && (*b)[1] > 0.0 && (*b)[2] > 0.0)) return bad(who + ": every side of the box must be above 0");
                part.shape = BuildShape::box;
                part.box = glm::vec3(static_cast<float>((*b)[0]), static_cast<float>((*b)[1]), static_cast<float>((*b)[2]));
            } else if (entry["cylinder"]) {
                const YAML::Node c = entry["cylinder"];
                if (!c.IsMap() || !c["radius"] || !c["length"]) return bad(who + ": a cylinder is {radius, length}");
                if (const auto key = build_detail::unknown_key(c, {"radius", "length"})) {
                    return bad(who + ": unknown key '" + *key + "' in a cylinder");
                }
                const double radius = c["radius"].as<double>();
                const double length = c["length"].as<double>();
                if (!(std::isfinite(radius) && std::isfinite(length) && radius > 0.0 && length > 0.0)) {
                    return bad(who + ": the cylinder's radius and length must be finite and above 0");
                }
                part.shape = BuildShape::cylinder;
                part.radius = static_cast<float>(radius);
                part.length = static_cast<float>(length);
            } else {
                const YAML::Node objs = entry["objects"];
                if (!objs.IsSequence() || objs.size() == 0u) return bad(who + ": objects is an empty list");
                part.shape = BuildShape::objects;
                for (const auto& o : objs) {
                    if (!o.IsMap() || !o["store_ref"]) return bad(who + ": an object has no store_ref");
                    if (const auto key = build_detail::unknown_key(o, {"store_ref", "offset"})) {
                        return bad(who + ": unknown key '" + *key + "' in an object");
                    }
                    const YAML::Node ref = o["store_ref"];
                    if (!ref.IsMap() || !ref["sha256"] || !ref["file"]) return bad(who + ": a store_ref is {sha256, file}");
                    if (const auto key = build_detail::unknown_key(ref, {"sha256", "file"})) {
                        return bad(who + ": unknown key '" + *key + "' in a store_ref");
                    }
                    BuildObject object;
                    object.sha256 = ref["sha256"].as<std::string>();
                    object.file = ref["file"].as<std::string>();
                    if (!build_detail::is_hex64(object.sha256)) return bad(who + ": sha256 '" + object.sha256 + "' is not 64 lower-case hex digits");
                    if (object.file.empty() || object.file.find('/') != std::string::npos || object.file.find('\\') != std::string::npos ||
                        object.file.find("..") != std::string::npos) {
                        return bad(who + ": file '" + object.file + "' must be a plain file name in the hash folder");
                    }
                    const std::string ext = build_detail::extension_of(object.file);
                    if (ext != ".spmesh" && ext != ".gltf" && ext != ".glb") {
                        return bad(who + ": file '" + object.file + "' has an extension that is not .spmesh, .gltf or .glb");
                    }
                    if (o["offset"]) {
                        const auto off = build_detail::pose_of(o["offset"], who + " object " + object.file + " offset");
                        if (!off) return std::unexpected(off.error());
                        object.offset_position = off->position;
                        object.offset_orient = off->orientation;
                    }
                    part.objects.push_back(std::move(object));
                }
            }
            parts.push_back(std::move(part));
        }
        return parts;
    } catch (const YAML::Exception& e) {
        return bad(std::string("a field has the wrong type: ") + e.what());
    }
}

namespace build_detail {
inline void one_submesh(spade::render::MeshData& m) {
    m.submesh_first_index = {0u};
    m.submesh_index_count = {static_cast<uint32_t>(m.indices.size())};
    m.submesh_material = {0u};
}
inline void quad(spade::render::MeshData& m, glm::vec3 a, glm::vec3 b, glm::vec3 c, glm::vec3 d, glm::vec3 n) {
    const auto base = static_cast<uint32_t>(m.positions.size());
    for (glm::vec3 v : {a, b, c, d}) {
        m.positions.push_back(v);
        m.normals.push_back(n);
    }
    for (uint32_t i : {0u, 1u, 2u, 0u, 2u, 3u}) m.indices.push_back(base + i);
}
}  // namespace build_detail

// A box of `size` metres centred on the origin: 12 triangles, flat normals, one submesh on material 0.
[[nodiscard]] inline spade::render::MeshData box_mesh(glm::vec3 size) {
    spade::render::MeshData m;
    const glm::vec3 h = size * 0.5f;
    using build_detail::quad;
    quad(m, {h.x, -h.y, -h.z}, {h.x, h.y, -h.z}, {h.x, h.y, h.z}, {h.x, -h.y, h.z}, {1, 0, 0});
    quad(m, {-h.x, -h.y, h.z}, {-h.x, h.y, h.z}, {-h.x, h.y, -h.z}, {-h.x, -h.y, -h.z}, {-1, 0, 0});
    quad(m, {-h.x, h.y, -h.z}, {-h.x, h.y, h.z}, {h.x, h.y, h.z}, {h.x, h.y, -h.z}, {0, 1, 0});
    quad(m, {-h.x, -h.y, h.z}, {-h.x, -h.y, -h.z}, {h.x, -h.y, -h.z}, {h.x, -h.y, h.z}, {0, -1, 0});
    quad(m, {-h.x, -h.y, h.z}, {h.x, -h.y, h.z}, {h.x, h.y, h.z}, {-h.x, h.y, h.z}, {0, 0, 1});
    quad(m, {h.x, -h.y, -h.z}, {-h.x, -h.y, -h.z}, {-h.x, h.y, -h.z}, {h.x, h.y, -h.z}, {0, 0, -1});
    build_detail::one_submesh(m);
    return m;
}

// A cylinder along local y, centred, 24 sides (smooth side normals, flat caps), one submesh on material 0.
[[nodiscard]] inline spade::render::MeshData cylinder_mesh(float radius, float length) {
    spade::render::MeshData m;
    constexpr uint32_t kSides = 24u;
    const float half = length * 0.5f;
    for (uint32_t i = 0; i < kSides; ++i) {
        const float a0 = 2.0f * std::numbers::pi_v<float> * static_cast<float>(i) / kSides;
        const float a1 = 2.0f * std::numbers::pi_v<float> * static_cast<float>(i + 1u) / kSides;
        const glm::vec3 d0(std::cos(a0), 0.0f, std::sin(a0));
        const glm::vec3 d1(std::cos(a1), 0.0f, std::sin(a1));
        const auto base = static_cast<uint32_t>(m.positions.size());
        m.positions.insert(m.positions.end(), {d0 * radius + glm::vec3(0, -half, 0), d1 * radius + glm::vec3(0, -half, 0),
                                               d1 * radius + glm::vec3(0, half, 0), d0 * radius + glm::vec3(0, half, 0)});
        m.normals.insert(m.normals.end(), {d0, d1, d1, d0});
        for (uint32_t k : {0u, 2u, 1u, 0u, 3u, 2u}) m.indices.push_back(base + k);
        for (const float side : {1.0f, -1.0f}) {  // the caps, a fan triangle each
            const auto cap = static_cast<uint32_t>(m.positions.size());
            const glm::vec3 up(0.0f, side, 0.0f);
            m.positions.insert(m.positions.end(), {glm::vec3(0, side * half, 0), d0 * radius + up * half, d1 * radius + up * half});
            m.normals.insert(m.normals.end(), {up, up, up});
            const std::array<uint32_t, 3> order = side > 0.0f ? std::array<uint32_t, 3>{0u, 2u, 1u} : std::array<uint32_t, 3>{0u, 1u, 2u};
            for (uint32_t k : order) m.indices.push_back(cap + k);
        }
    }
    build_detail::one_submesh(m);
    return m;
}

// Every part drawn at the body's pose x its own (x an object's offset), presentation only. A store object is <store>/<sha256>/<file>:
// a .spmesh drawn flat in its kind's colour, a glTF with its own palette. All or nothing: a part that cannot be drawn is refused naming
// the part and the file, and the scene is left exactly as it was (meshes, items and palette).
[[nodiscard]] inline spade::Result<void> add_build_parts(spade::render::RenderScene& scene, const std::vector<BuildPart>& parts,
                                                        const std::filesystem::path& store, glm::vec3 body_position, glm::quat body_orient) {
    const std::size_t meshes = scene.meshes.size();
    const std::size_t statics = scene.statics.size();
    const std::size_t materials = scene.materials.size();
    const auto fail = [&](const BuildPart& part, const std::string& what, const spade::Error& e) -> spade::Result<void> {
        scene.meshes.resize(meshes);
        scene.statics.resize(statics);
        scene.materials.resize(materials);
        return std::unexpected(spade::Error{e.code, "build parts: part '" + part.name + "' " + what + ": " + e.context});
    };
    const auto kind_coloured = [](spade::render::MeshData m, const std::string& kind) {
        const auto colour = kind_material(kind);
        m.source_materials = {colour ? *colour : spade::render::Material{}};
        m.submesh_material.assign(std::max<std::size_t>(m.submesh_index_count.size(), 1u), 0u);
        return m;
    };
    for (const BuildPart& part : parts) {
        const glm::vec3 base_pos = body_position + body_orient * part.position;
        const glm::quat base_orient = body_orient * part.orientation;
        const auto draw = [&](spade::render::MeshData m, glm::vec3 pos, glm::quat orient, const std::string& what) -> spade::Result<void> {
            if (auto r = flow_movie::add_visual_mesh(scene, m, pos, orient); !r) return fail(part, what, r.error());
            return {};
        };
        if (part.shape == BuildShape::box) {
            if (auto r = draw(kind_coloured(box_mesh(part.box), part.kind), base_pos, base_orient, "box"); !r) return r;
        } else if (part.shape == BuildShape::cylinder) {
            if (auto r = draw(kind_coloured(cylinder_mesh(part.radius, part.length), part.kind), base_pos, base_orient, "cylinder"); !r) return r;
        } else {
            for (const BuildObject& object : part.objects) {
                const std::filesystem::path file = store / object.sha256 / object.file;
                spade::render::MeshData mesh;
                if (build_detail::extension_of(object.file) == ".spmesh") {
                    auto loaded = spade::load_mesh_file(file);
                    if (!loaded) return fail(part, "object " + file.string(), loaded.error());
                    mesh = spade::render::flat_mesh(*loaded->mesh);
                    build_detail::one_submesh(mesh);
                    mesh = kind_coloured(std::move(mesh), part.kind);
                } else {
                    auto loaded = spade::render::load_gltf(file);
                    if (!loaded) return fail(part, "object " + file.string(), loaded.error());
                    mesh = std::move(*loaded);
                }
                if (auto r = draw(std::move(mesh), base_pos + base_orient * object.offset_position, base_orient * object.offset_orient,
                                  "object " + file.string());
                    !r) {
                    return r;
                }
            }
        }
    }
    return {};
}

}  // namespace flow_movie
