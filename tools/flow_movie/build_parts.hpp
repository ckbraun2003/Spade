// build_parts.hpp -- the real-parts drone of the web clips (the lead, 2026-11-18, L88l): the X500 frame plus a flight controller, GPS mast,
// battery, companion computer, camera and radio, drawn PRESENTATION ONLY over the held body, from the plain table Core writes
// (tools/flow_movie/data/x500_build/parts.yaml). The flow's solids are the model's and are untouched: nothing here reaches the solve.
//
// The table is one list `parts:`; each entry has `name` (unique), `kind` (frame controller gps battery computer camera radio mast rotor),
// `mass_kg` (Physics' business: read, kept, not drawn), `pose: {position: [x, y, z] m, orientation: [w, x, y, z]}` in the X500 body frame
// (y up, +X forward, the origin the model's centre of mass, the frame of KAT's glTF), and ONE shape: `objects` (a list of `store_ref:
// {sha256, file}` with an optional `offset: {position, orientation}`; the file is <store>/<sha256>/<file>, a .spmesh or a .gltf/.glb),
// `box: [sx, sy, sz]` m, or `cylinder: {radius, length}` m along the part's local y, centred. Unknown kinds and keys are refused.
//
// Colours: a part's .spmesh, box and cylinder take its `material:` (a name of real_material(): carbon_fibre, aluminium_bare, ...), else
// its KIND's colour; an object's own `material:` wins; a glTF's materials are replaced by real ones keyed on their NAMES
// (real_material_for_gltf), a name not mapped keeping the file's colour. Lambert only: the renderer has no specular term.
//
// A ROTOR (`kind: rotor`, `rotor: i` the index of its omega in the logged body's rotor speeds, `spin: 1 or -1`, optional `radius: m`)
// is drawn as its blade SPREAD over the frame's shutter (prop_blur_angles: many see-through copies of one mesh at a low alpha), a
// stopped or speedless one as one solid blade; `radius` scales the glTF's `prop` primitive radially about the hub to the SIMULATED
// rotor's radius (the drawn blade's tips are where the cloud's rings are), the motor bell as drawn.
#pragma once

#include <algorithm>
#include <array>
#include <cctype>
#include <charconv>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <numbers>
#include <optional>
#include <set>
#include <span>
#include <string>
#include <string_view>
#include <system_error>
#include <vector>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
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
    std::string material;  // a real_material() name; "": the part's
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
    std::string material;  // a real_material() name; "": the kind's colour
    int rotor = -1;        // kind rotor: the index of its omega in the logged rotor speeds
    int spin = 0;          // kind rotor: +1 or -1, the sense the blade turns
    std::vector<std::string> hide_materials;  // glTF materials (last word) whose primitives are not drawn
    float radius_m = 0.0f; // kind rotor: the simulated rotor's radius the drawn prop is scaled to (0: the file's own)
};

namespace build_detail {

// "mass_only": a mass carrier for the physics, not a drawing of anything (accepted, nothing drawn)
inline constexpr std::array<std::string_view, 10> kKinds{"frame", "controller", "gps", "battery", "computer", "camera", "radio", "mast", "rotor", "mass_only"};

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
    if (kind == "rotor") return make(0.04f, 0.04f, 0.045f);
    return std::nullopt;
}

// THE REAL MATERIALS (the lead, 2026-11-18: the role colours "read as a toy"): colours, lambert, for the things the drone is made of.
inline constexpr std::array<std::string_view, 8> kRealMaterials{"carbon_fibre", "aluminium_bare", "aluminium_anodised", "pcb_green",
                                                                "plastic_black", "plastic_white", "rubber_black", "gold"};

[[nodiscard]] inline std::optional<spade::render::Material> real_material(std::string_view name) {
    const auto make = [](float r, float g, float b) {
        spade::render::Material m;
        m.base_color = glm::vec4(r, g, b, 1.0f);
        return m;
    };
    // Core's table writes the shorter names: pcb, aluminium, carbon (the presets below) and lipo_wrap (a LiPo pack's blue shrink wrap)
    if (name == "pcb") name = "pcb_green";
    if (name == "aluminium") name = "aluminium_bare";
    if (name == "carbon") name = "carbon_fibre";
    if (name == "lipo_wrap") return make(0.04f, 0.08f, 0.26f);
    if (name == "carbon_fibre") return make(0.025f, 0.027f, 0.032f);
    if (name == "aluminium_bare") return make(0.70f, 0.71f, 0.73f);
    if (name == "aluminium_anodised") return make(0.11f, 0.12f, 0.15f);
    if (name == "pcb_green") return make(0.03f, 0.21f, 0.11f);
    if (name == "plastic_black") return make(0.045f, 0.045f, 0.05f);
    if (name == "plastic_white") return make(0.86f, 0.86f, 0.84f);
    if (name == "rubber_black") return make(0.02f, 0.02f, 0.022f);
    if (name == "gold") return make(0.83f, 0.62f, 0.23f);
    return std::nullopt;
}

[[nodiscard]] inline bool real_material_known(std::string_view name) { return real_material(name).has_value(); }

// The last word of a glTF material name ("drone/x500-body/carbon_plates" -> "carbon_plates"; no folders: the name).
[[nodiscard]] inline std::string_view last_word(std::string_view name) {
    const std::size_t slash = name.rfind('/');
    return slash == std::string_view::npos ? name : name.substr(slash + 1u);
}

// The real material a glTF's own material is replaced by, from its name's last word; nothing for a name not mapped (its file colour stays).
[[nodiscard]] inline std::optional<std::string> real_material_for_gltf(std::string_view name) {
    const std::string_view word = last_word(name);
    if (word == "carbon_plates" || word == "arms") return "carbon_fibre";
    if (word == "metal") return "aluminium_bare";
    if (word == "stator" || word == "bell") return "aluminium_anodised";
    if (word == "prop" || word == "autopilot" || word == "antenna_holder") return "plastic_black";
    if (word == "landing_rubber" || word == "rails_rubber") return "rubber_black";
    if (word == "autopilot_rubber") return "plastic_white";
    return std::nullopt;
}

// A glTF's material names from its JSON, in order ("" for an unnamed one; none when it has no materials). A glTF is JSON, which YAML reads.
[[nodiscard]] inline spade::Result<std::vector<std::string>> gltf_material_names(const std::filesystem::path& file) {
    std::ifstream in(file, std::ios::binary);
    if (!in) return std::unexpected(spade::Error{spade::Code::io_error, "gltf " + file.string() + ": cannot be read"});
    const std::string text((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
    std::vector<std::string> names;
    try {
        const YAML::Node root = YAML::Load(text);
        if (root && root.IsMap() && root["materials"] && root["materials"].IsSequence()) {
            for (const auto& m : root["materials"]) names.push_back(m.IsMap() && m["name"] ? m["name"].as<std::string>() : std::string());
        }
    } catch (const YAML::Exception& e) {
        return std::unexpected(spade::Error{spade::Code::invalid_argument, "gltf " + file.string() + ": its JSON is not readable: " + e.what()});
    }
    return names;
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
        if (const auto key = build_detail::unknown_key(root, {"parts", "hide_materials"})) {
            return bad("unknown key '" + *key + "' at the top (only `parts` and `hide_materials`)");
        }
        // one top-level hide_materials: glTF materials (whole path or last word) left out of the frame's drawing (its mesh carries the block)
        std::vector<std::string> frame_hides;
        if (root["hide_materials"]) {
            if (!root["hide_materials"].IsSequence()) return bad("the top-level hide_materials is a list of material names");
            for (const auto& h : root["hide_materials"]) {
                if (!h.IsScalar() || h.as<std::string>().empty()) return bad("the top-level hide_materials has an item that is not a name");
                frame_hides.push_back(h.as<std::string>());
            }
        }
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
            if (const auto key = build_detail::unknown_key(entry, {"name", "kind", "mass_kg", "mass_grade", "mass_source", "pose_grade", "hide_materials", "material", "rotor", "spin", "radius", "pose", "objects", "box", "cylinder"})) {
                return bad(who + ": unknown key '" + *key + "'");
            }
            part.kind = entry["kind"] ? entry["kind"].as<std::string>() : std::string();
            if (!kind_known(part.kind)) return bad(who + ": unknown kind '" + part.kind + "'");
            if (entry["mass_kg"]) {
                part.mass_kg = entry["mass_kg"].as<double>();
                if (!(std::isfinite(part.mass_kg) && part.mass_kg >= 0.0)) return bad(who + ": mass_kg is not a finite number from 0 up");
            }
            if (entry["hide_materials"]) {
                if (!entry["hide_materials"].IsSequence()) return bad(who + ": hide_materials is a list of material names");
                for (const auto& h : entry["hide_materials"]) {
                    if (!h.IsScalar()) return bad(who + ": hide_materials has an item that is not a name");
                    const std::string word = h.as<std::string>();
                    if (word.empty() || (std::isdigit(static_cast<unsigned char>(word.front())) != 0 && word.find_first_not_of("0123456789") == std::string::npos)) {
                        return bad(who + ": hide_materials item '" + word + "' is not a material name");
                    }
                    part.hide_materials.push_back(word);
                }
            }
            if (entry["material"]) {
                part.material = entry["material"].as<std::string>();
                if (!real_material_known(part.material)) return bad(who + ": unknown material '" + part.material + "'");
            }
            if (part.kind == "rotor") {
                if (!entry["rotor"]) return bad(who + " is a rotor and needs `rotor: i`, the index of its speed");
                part.rotor = entry["rotor"].as<int>();
                if (part.rotor < 0 || part.rotor > 15) return bad(who + ": rotor " + std::to_string(part.rotor) + " is not an index from 0 to 15");
                if (!entry["objects"]) return bad(who + ": a rotor is drawn from `objects` (its blade's file)");
                if (!entry["spin"]) return bad(who + " is a rotor and needs `spin: 1` or `-1`");
                part.spin = entry["spin"].as<int>();
                if (part.spin != 1 && part.spin != -1) return bad(who + ": spin " + std::to_string(part.spin) + " is not 1 or -1");
                if (entry["radius"]) {
                    const double r = entry["radius"].as<double>();
                    if (!(std::isfinite(r) && r > 0.0 && r <= 1.0)) return bad(who + ": the rotor's radius must be above 0 and at most 1 m");
                    part.radius_m = static_cast<float>(r);
                }
            } else if (entry["rotor"] || entry["spin"] || entry["radius"]) {
                return bad(who + ": rotor, spin and radius are for a rotor (kind: rotor)");
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
                    if (const auto key = build_detail::unknown_key(o, {"store_ref", "offset", "material"})) {
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
                    if (o["material"]) {
                        object.material = o["material"].as<std::string>();
                        if (!real_material_known(object.material)) return bad(who + ": unknown material '" + object.material + "' on an object");
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
        for (BuildPart& part : parts) {
            if (part.kind == "frame") part.hide_materials.insert(part.hide_materials.end(), frame_hides.begin(), frame_hides.end());
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

// THE BLADE BLUR: the azimuths, about the hub's y, of `samples` copies of a blade spread over omega x shutter radians, centred on the
// blade's own pose, advancing in the spin's direction (+1 or -1); all 0 when it is not turning, none for no samples.
[[nodiscard]] inline std::vector<double> prop_blur_angles(double omega, int spin, double shutter_s, uint32_t samples) {
    std::vector<double> angles(samples, 0.0);
    if (!(omega > 0.0) || samples == 0u) return angles;
    const double sweep = omega * shutter_s * (spin < 0 ? -1.0 : 1.0);
    for (uint32_t i = 0; i < samples; ++i) angles[i] = sweep * ((static_cast<double>(i) + 0.5) / static_cast<double>(samples) - 0.5);
    return angles;
}

struct PropBlur {
    uint32_t samples = 24u;    // copies of the blade over the shutter
    double shutter_s = 1.0 / 60.0;  // a 180 degree shutter at 30 fps
    float alpha = 0.18f;       // each copy's opacity
};

// "alpha=0.4,samples=32,shutter=0.02" over PropBlur's defaults (--prop-blur): alpha in (0, 1], samples a whole number from 1 to 128, shutter
// in seconds in (0, 0.1). Empty: the defaults. Refused, prefixed "prop blur:": an unknown key, no '=', a value that is not a number, one out of range.
[[nodiscard]] inline spade::Result<PropBlur> parse_prop_blur(std::string_view text) {
    const auto bad_blur = [](const std::string& why) {
        return std::unexpected(spade::Error{spade::Code::invalid_argument, "prop blur: " + why});
    };
    PropBlur blur;
    if (text.empty()) return blur;
    std::size_t at = 0;
    while (at <= text.size()) {
        const std::size_t comma = text.find(',', at);
        const std::string item(text.substr(at, comma == std::string_view::npos ? std::string_view::npos : comma - at));
        at = comma == std::string_view::npos ? text.size() + 1u : comma + 1u;
        if (item.empty()) return bad_blur("an empty item (a stray comma)");
        const std::size_t eq = item.find('=');
        if (eq == std::string::npos) return bad_blur("'" + item + "' has no '=': write key=value");
        const std::string key = item.substr(0, eq);
        const std::string value = item.substr(eq + 1u);
        double number = 0.0;
        const auto [end, ec] = std::from_chars(value.data(), value.data() + value.size(), number);
        if (value.empty() || ec != std::errc{} || end != value.data() + value.size() || !std::isfinite(number)) {
            return bad_blur("'" + key + "' is not a number: '" + value + "'");
        }
        if (key == "alpha") {
            if (!(number > 0.0 && number <= 1.0)) return bad_blur("alpha must be above 0 and at most 1");
            blur.alpha = static_cast<float>(number);
        } else if (key == "samples") {
            if (!(number >= 1.0 && number <= 128.0 && number == std::floor(number))) return bad_blur("samples must be a whole number from 1 to 128");
            blur.samples = static_cast<uint32_t>(number);
        } else if (key == "shutter") {
            if (!(number > 0.0 && number < 0.1)) return bad_blur("shutter is in seconds, above 0 and under 0.1");
            blur.shutter_s = number;
        } else {
            return bad_blur("unknown key '" + key + "' (alpha samples shutter)");
        }
    }
    return blur;
}

struct BuildDraw {
    std::span<const double> omega{};  // the logged rotor speeds (rad/s), by a rotor part's `rotor` index; none: every blade frozen
    PropBlur blur{};
};

// The primitives of `mesh` whose glTF material (names[the submesh's material]) ends in one of `hide` are left out of the drawing: their indices
// and submesh entries go, the palette and the vertices stay. Each name must match a primitive: one that matches nothing is refused naming
// it (a typo must not leave the block drawn), the mesh left as it was. Nothing to hide: nothing done.
[[nodiscard]] inline spade::Result<void> hide_materials(spade::render::MeshData& mesh, const std::vector<std::string>& names,
                                                        const std::vector<std::string>& hide) {
    if (hide.empty()) return {};
    for (const std::string& word : hide) {
        bool found = false;
        for (std::size_t s = 0; s < mesh.submesh_material.size(); ++s) {
            const uint32_t m = mesh.submesh_material[s];
            found = found || (m < names.size() && (names[m] == word || last_word(names[m]) == word));
        }
        if (!found) {
            return std::unexpected(spade::Error{spade::Code::invalid_argument, "build parts: hide_materials '" + word + "' matches no primitive of the mesh"});
        }
    }
    std::vector<uint32_t> indices, first, count, material;
    for (std::size_t s = 0; s < mesh.submesh_first_index.size() && s < mesh.submesh_index_count.size() && s < mesh.submesh_material.size(); ++s) {
        const uint32_t m = mesh.submesh_material[s];
        const bool hidden = m < names.size() && (std::find(hide.begin(), hide.end(), names[m]) != hide.end() ||
                                                 std::find(hide.begin(), hide.end(), std::string(last_word(names[m]))) != hide.end());
        if (hidden) continue;
        first.push_back(static_cast<uint32_t>(indices.size()));
        count.push_back(mesh.submesh_index_count[s]);
        material.push_back(m);
        const auto begin = mesh.indices.begin() + mesh.submesh_first_index[s];
        indices.insert(indices.end(), begin, begin + mesh.submesh_index_count[s]);
    }
    mesh.indices = std::move(indices);
    mesh.submesh_first_index = std::move(first);
    mesh.submesh_index_count = std::move(count);
    mesh.submesh_material = std::move(material);
    return {};
}

// The `prop` primitive of `mesh` scaled radially (x and z, about the y axis through the hub) so its blade tip is at `radius`: the
// submeshes whose glTF material (names[the submesh's material]) ends in `prop`; everything else, and the blade's thickness, as drawn.
// Refused, the mesh left as it was: a radius not above 0, no prop primitive, a prop with no extent.
[[nodiscard]] inline spade::Result<void> scale_prop_radially(spade::render::MeshData& mesh, const std::vector<std::string>& names, float radius) {
    const auto bad_scale = [](const std::string& why) {
        return std::unexpected(spade::Error{spade::Code::invalid_argument, "build parts: scaling the prop: " + why});
    };
    if (!(radius > 0.0f)) return bad_scale("the radius must be above 0");
    std::vector<uint8_t> marked(mesh.positions.size(), 0u);
    bool any = false;
    for (std::size_t s = 0; s < mesh.submesh_first_index.size() && s < mesh.submesh_index_count.size() && s < mesh.submesh_material.size(); ++s) {
        const uint32_t m = mesh.submesh_material[s];
        if (m >= names.size() || last_word(names[m]) != "prop") continue;
        for (uint32_t i = mesh.submesh_first_index[s]; i < mesh.submesh_first_index[s] + mesh.submesh_index_count[s] && i < mesh.indices.size(); ++i) {
            const uint32_t v = mesh.indices[i];
            if (v < marked.size()) {
                marked[v] = 1u;
                any = true;
            }
        }
    }
    if (!any) return bad_scale("the mesh has no `prop` primitive (by its material names)");
    float reach = 0.0f;
    for (std::size_t v = 0; v < marked.size(); ++v) {
        if (marked[v] != 0u) reach = std::max(reach, std::hypot(mesh.positions[v].x, mesh.positions[v].z));
    }
    if (!(reach > 0.0f)) return bad_scale("the prop has no radial extent");
    const float factor = radius / reach;
    for (std::size_t v = 0; v < marked.size(); ++v) {
        if (marked[v] == 0u) continue;
        mesh.positions[v].x *= factor;
        mesh.positions[v].z *= factor;
    }
    return {};
}

// Every part drawn at the body's pose x its own (x an object's offset), presentation only. A store object is <store>/<sha256>/<file>:
// a .spmesh drawn flat in its material (the object's, else the part's, else its kind's colour), a glTF with its materials replaced by
// real ones keyed on their names (a name not mapped keeps the file's colour). A rotor is its blade spread over the shutter (BuildDraw).
// All or nothing: a part that cannot be drawn is refused naming the part and the file, and the scene is left exactly as it was.
[[nodiscard]] inline spade::Result<void> add_build_parts(spade::render::RenderScene& scene, const std::vector<BuildPart>& parts,
                                                        const std::filesystem::path& store, glm::vec3 body_position, glm::quat body_orient,
                                                        const BuildDraw& draw_options = {}) {
    const std::size_t meshes = scene.meshes.size();
    const std::size_t statics = scene.statics.size();
    const std::size_t materials = scene.materials.size();
    const auto fail = [&](const BuildPart& part, const std::string& what, const spade::Error& e) -> spade::Result<void> {
        scene.meshes.resize(meshes);
        scene.statics.resize(statics);
        scene.materials.resize(materials);
        return std::unexpected(spade::Error{e.code, "build parts: part '" + part.name + "' " + what + ": " + e.context});
    };
    // The one-colour palette of a mesh without its own: the object's material, else the part's, else the kind's colour.
    const auto coloured = [](spade::render::MeshData m, const BuildPart& part, const std::string& object_material) {
        const std::string& name = !object_material.empty() ? object_material : part.material;
        std::optional<spade::render::Material> colour = name.empty() ? kind_material(part.kind) : real_material(name);
        m.source_materials = {colour ? *colour : spade::render::Material{}};
        m.submesh_material.assign(std::max<std::size_t>(m.submesh_index_count.size(), 1u), 0u);
        return m;
    };
    for (const BuildPart& part : parts) {
        if (part.kind == "mass_only") continue;  // a mass carrier, not a drawing
        const glm::vec3 base_pos = body_position + body_orient * part.position;
        const glm::quat base_orient = body_orient * part.orientation;
        const auto draw = [&](spade::render::MeshData m, glm::vec3 pos, glm::quat orient, const std::string& what) -> spade::Result<void> {
            if (auto r = flow_movie::add_visual_mesh(scene, m, pos, orient); !r) return fail(part, what, r.error());
            return {};
        };
        if (part.shape == BuildShape::box) {
            if (auto r = draw(coloured(box_mesh(part.box), part, ""), base_pos, base_orient, "box"); !r) return r;
        } else if (part.shape == BuildShape::cylinder) {
            if (auto r = draw(coloured(cylinder_mesh(part.radius, part.length), part, ""), base_pos, base_orient, "cylinder"); !r) return r;
        } else {
            for (const BuildObject& object : part.objects) {
                const std::filesystem::path file = store / object.sha256 / object.file;
                spade::render::MeshData mesh;
                std::vector<std::string> names;  // a glTF's material names, in its palette's order
                if (build_detail::extension_of(object.file) == ".spmesh") {
                    auto loaded = spade::load_mesh_file(file);
                    if (!loaded) return fail(part, "object " + file.string(), loaded.error());
                    mesh = spade::render::flat_mesh(*loaded->mesh);
                    build_detail::one_submesh(mesh);
                    mesh = coloured(std::move(mesh), part, object.material);
                } else {
                    auto loaded = spade::render::load_gltf(file);
                    if (!loaded) return fail(part, "object " + file.string(), loaded.error());
                    mesh = std::move(*loaded);
                    if (auto read = gltf_material_names(file); read && read->size() == mesh.source_materials.size()) names = std::move(*read);
                    if (!part.hide_materials.empty()) {
                        if (names.empty()) return fail(part, "object " + file.string(), spade::Error{spade::Code::invalid_argument, "its material names are needed to hide primitives"});
                        if (auto hidden = hide_materials(mesh, names, part.hide_materials); !hidden) return fail(part, "object " + file.string(), hidden.error());
                    }
                    for (std::size_t i = 0; i < names.size(); ++i) {  // real materials, keyed on the names
                        const std::string chosen = !object.material.empty() ? object.material : !part.material.empty() ? part.material : std::string();
                        const auto mapped = !chosen.empty() ? std::optional<std::string>(chosen) : real_material_for_gltf(names[i]);
                        if (mapped) mesh.source_materials[i] = *real_material(*mapped);
                    }
                    if (names.empty() && (!object.material.empty() || !part.material.empty())) {
                        const auto colour = real_material(!object.material.empty() ? object.material : part.material);
                        for (auto& m : mesh.source_materials) m = *colour;
                    }
                    if (part.kind == "rotor" && part.radius_m > 0.0f) {
                        if (names.empty()) return fail(part, "object " + file.string(), spade::Error{spade::Code::invalid_argument, "its material names are needed to find the prop to scale"});
                        if (auto scaled = scale_prop_radially(mesh, names, part.radius_m); !scaled) return fail(part, "object " + file.string(), scaled.error());
                    }
                }
                glm::vec3 pos = base_pos + base_orient * object.offset_position;
                const glm::quat orient = base_orient * object.offset_orient;
                const double omega = part.kind == "rotor" && part.rotor >= 0 && static_cast<std::size_t>(part.rotor) < draw_options.omega.size()
                                         ? draw_options.omega[static_cast<std::size_t>(part.rotor)]
                                         : 0.0;
                if (part.kind == "rotor" && omega > 0.0 && draw_options.blur.samples > 0u) {
                    // the blade spread over the shutter: the prop's materials at the blur's alpha (every one for a mesh without names)
                    for (std::size_t i = 0; i < mesh.source_materials.size(); ++i) {
                        if (names.empty() || last_word(names[i]) == "prop") mesh.source_materials[i].base_color.a = draw_options.blur.alpha;
                    }
                    const std::vector<double> angles = prop_blur_angles(omega, part.spin, draw_options.blur.shutter_s, draw_options.blur.samples);
                    const std::size_t first_item = scene.statics.size();
                    const glm::quat turn0 = glm::angleAxis(static_cast<float>(angles.front()), glm::vec3(0.0f, 1.0f, 0.0f));
                    if (auto r = draw(std::move(mesh), pos, orient * turn0, "object " + file.string()); !r) return r;
                    for (std::size_t i = 1; i < angles.size(); ++i) {
                        spade::render::DrawItem copy = scene.statics[first_item];
                        const glm::quat turn = glm::angleAxis(static_cast<float>(angles[i]), glm::vec3(0.0f, 1.0f, 0.0f));
                        copy.local_to_world = glm::translate(glm::mat4(1.0f), pos) * glm::mat4_cast(orient * turn);
                        scene.statics.push_back(copy);
                    }
                    for (std::size_t i = first_item; i < scene.statics.size(); ++i) scene.statics[i].see_through = true;
                } else if (auto r = draw(std::move(mesh), pos, orient, "object " + file.string()); !r) {
                    return r;
                }
            }
        }
    }
    return {};
}

}  // namespace flow_movie
