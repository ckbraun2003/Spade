#include "scene/scene_file.hpp"

#include <algorithm>
#include <charconv>
#include <cstddef>
#include <exception>
#include <fstream>
#include <initializer_list>
#include <iterator>
#include <memory>
#include <string>
#include <string_view>
#include <system_error>
#include <utility>
#include <vector>

#include <glm/common.hpp>
#include <yaml-cpp/yaml.h>

#include "core/rng.hpp"
#include "core/validate.hpp"
#include "physics/forces.hpp"  // drag_mode
#include "scene/detail/model_block.hpp"
#include "world/detail/yaml_text.hpp"
#include "world/mesh.hpp"
#include "world/mesh_file.hpp"
#include "world/world_file.hpp"

namespace spade::scene {
namespace {

// The text primitives and the world file's SDF and material encodings are
// world/detail/yaml_text.hpp's, shared with the world file: one encoding of
// an SDF program or a material, wherever it appears.
using yaml_text::append_sdf_nodes;
using yaml_text::append_sdf_transforms;
using yaml_text::at;
using yaml_text::dec;
using yaml_text::field;
using yaml_text::float_field;
using yaml_text::float_list;
using yaml_text::format_float;
using yaml_text::has;
using yaml_text::mark_of;
using yaml_text::material_flow;
using yaml_text::parse_uint;
using yaml_text::path_text;
using yaml_text::quat_field;
using yaml_text::quote_yaml;
using yaml_text::scalar;
using yaml_text::string_field;
using yaml_text::uint_field;
using yaml_text::vec3_field;

[[nodiscard]] Error invalid(std::string context) {
    return Error{Code::invalid_argument, std::move(context)};
}

// The first scene version that carries a model's parts (CORE-7), and the
// first that carries a mesh part's `shape: {mesh: {file, hash}}` (CORE-6).
constexpr uint32_t kPartsVersion = 2;
constexpr uint32_t kMeshPartsVersion = 3;
static_assert(kSceneFileOldestVersion < kPartsVersion && kPartsVersion < kMeshPartsVersion &&
                  kMeshPartsVersion <= kSceneFileVersion,
              "this build reads the versions that carry parts and mesh parts");

// The schema a diagnostic blames: the file's OWN declared version wherever
// the reader has one, because a key set is version-gated (a model's `parts`),
// so "unknown in the current version" would blame the wrong one (the world
// file's check_map note has the case that taught it).
[[nodiscard]] std::string schema(uint32_t version) {
    return "scene schema v" + dec(version);
}

[[nodiscard]] Result<void> check_map(const YAML::Node& node, std::string_view what,
                                     std::initializer_list<std::string_view> allowed, std::string_view schema) {
    return yaml_text::check_map(node, what, allowed, schema);
}

// The world file's unit-quaternion test (validate_world_desc): 1e-3 on the
// squared length, so a value that went through text at 9 digits still passes,
// and nothing is re-normalized (that would move a loaded value's last bit).
[[nodiscard]] bool unit(const glm::quat& q) {
    const float q2 = q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z;
    return glm::abs(q2 - 1.0f) <= 1e-3f;
}

// Drag modes by NAME, the scenario reader's spelling; index into this table IS
// the physics::drag_mode value.
constexpr std::string_view kDragModeNames[] = {"quadratic", "componentwise"};
static_assert(physics::drag_mode::quadratic == 0 && physics::drag_mode::componentwise == 1,
              "kDragModeNames is indexed by physics::drag_mode");

// A part's attachment and surface kind by NAME; index into each table IS the
// enumerator's value.
constexpr std::string_view kPartAttachNames[] = {"body", "rotor"};
static_assert(static_cast<uint32_t>(vehicles::PartAttach::body) == 0 &&
                  static_cast<uint32_t>(vehicles::PartAttach::rotor) == 1,
              "kPartAttachNames is indexed by PartAttach");
constexpr std::string_view kPartSurfaceNames[] = {"wall", "porous"};
static_assert(static_cast<uint32_t>(vehicles::PartSurface::Kind::wall) == 0 &&
                  static_cast<uint32_t>(vehicles::PartSurface::Kind::porous) == 1,
              "kPartSurfaceNames is indexed by PartSurface::Kind");

// The version a scene is written as: the oldest that carries everything in it
// (scene_file.hpp, property 3).
[[nodiscard]] uint32_t written_version(const SceneDesc& scene) {
    uint32_t version = kSceneFileOldestVersion;
    for (const vehicles::ModelType& m : scene.models) {
        for (const vehicles::PartDesc& p : m.parts) {
            if (p.mesh) return kMeshPartsVersion;
            version = kPartsVersion;
        }
    }
    return version;
}

}  // namespace

// The model block's helpers and the block itself (detail/model_block.hpp) are
// defined in this file, outside the anonymous namespace, where they sit:
// shared with the model file (scene/model_file.cpp).
std::string detail::relative_path_problem(std::string_view file, std::string_view kind) {
    if (file.find('\\') != std::string_view::npos) return "must use forward slashes";
    if (file.front() == '/' || file.find(':') != std::string_view::npos) {
        return "must be relative to the " + std::string(kind) + " file";
    }
    return {};
}

namespace {

// ===========================================================================
// Emission
// ===========================================================================

// The header's first two lines are the rule and "-- schema version N.", the
// version written; the rest follows.
constexpr std::string_view kHeaderRule =
    "# =============================================================================\n";
constexpr std::string_view kFileHeader = R"YAML(#
# WHAT THIS IS: a world reference plus the objects placed in it -- materials,
# vehicle models, static assets, and vehicles with their start states.
# spade::scene::compose() turns it into one runnable world.
#
# GENERATED by spade::scene::scene_to_yaml(). Hand editing is supported: every
# field is validated on load, and re-saving normalizes the file back to this
# canonical form (fixed key order, fixed float spelling, these comments).
#
# UNITS are SI -- metres, kilograms, seconds, radians -- in a right-handed,
# Y-up frame. FLOATS carry 9 significant digits, so every value survives a
# load and a save bit for bit. Quaternions are [w, x, y, z].
#
# NAMES, NEVER INDICES: a visual and a collider's nodes name their materials.
# Names are unique within materials, within models, and across assets and
# vehicles together.
# =============================================================================
)YAML";

[[nodiscard]] std::string vec3_list(const glm::vec3& v) {
    return float_list({v.x, v.y, v.z});
}

[[nodiscard]] std::string quat_list(const glm::quat& q) {
    return float_list({q.w, q.x, q.y, q.z});
}

// "0x" and 16 lowercase hex digits, as the scenario files write digests.
[[nodiscard]] std::string hex64(uint64_t value) {
    static constexpr char kHex[] = "0123456789abcdef";
    std::string out = "0x";
    for (int shift = 60; shift >= 0; shift -= 4) {
        out += kHex[(value >> shift) & 0xfu];
    }
    return out;
}

// An SDF program's `transforms` and `nodes` keys at `indent`, each list's
// items two spaces deeper: the world file's encoding, shared by an asset's
// collider and a part's shape.
void emit_sdf_lists(std::string& out, const SdfProgram& program, const std::string& indent) {
    const std::string items = indent + "  ";
    if (program.transforms.empty()) {
        out += indent + "transforms: []\n";
    } else {
        out += indent + "transforms:\n";
        append_sdf_transforms(out, program.transforms, items);
    }
    if (program.nodes.empty()) {
        out += indent + "nodes: []\n";
    } else {
        out += indent + "nodes:\n";
        append_sdf_nodes(out, program.nodes, items);
    }
}

// One part, in PartDesc's field order. The shape is `{sdf: {transforms,
// nodes}}`, the world file's encoding with no node_materials (render data,
// which validate_scene() has already refused on a part), or, for a mesh part
// (version 3), `{mesh: {file, hash}}`: the sidecar's path and the mesh's
// content hash, never the mesh.
// `indent` is the model's (emit_model), so the part's list item sits two
// spaces deeper and its keys four.
void emit_part(std::string& out, const vehicles::PartDesc& p, const std::string& indent) {
    out += indent + "  - name: " + quote_yaml(p.name) + "\n";
    out += indent + "    role: " + quote_yaml(p.role) + "\n";
    out += indent + "    attach: " + quote_yaml(kPartAttachNames[static_cast<uint32_t>(p.attach)]) + "\n";
    out += indent + "    rotor: " + dec(p.rotor) + "\n";
    out += indent + "    mount: {position: " + vec3_list(p.mount_pos) + ", orientation: " +
           quat_list(p.mount_orient) + "}\n";
    out += indent + "    surface: {kind: " +
           quote_yaml(kPartSurfaceNames[static_cast<uint32_t>(p.surface.kind)]) +
           ", inv_permeability: " + format_float(p.surface.inv_permeability) +
           ", inertial_coeff: " + format_float(p.surface.inertial_coeff) + "}\n";
    out += indent + "    min_feature: " + format_float(p.min_feature) + "\n";
    out += indent + "    shape:\n";
    if (p.mesh) {
        out += indent + "      mesh: {file: " + quote_yaml(p.mesh_file) +
               ", hash: " + quote_yaml(hex64(p.mesh->content_hash())) + "}\n";
        return;
    }
    out += indent + "      sdf:\n";
    emit_sdf_lists(out, p.shape, indent + "        ");
}

}  // namespace

void detail::emit_model(std::string& out, const vehicles::ModelType& m, std::string_view lead,
                        std::string_view indent) {
    const std::string in(indent);
    out += std::string(lead) + "name: " + quote_yaml(m.name) + "\n";
    out += in + "version: " + dec(m.version) + "\n";
    out += in + "param_schema_id: " + dec(m.param_schema_id) + "\n";
    out += in + "visual_ref: " + quote_yaml(m.visual_ref) + "\n";
    out += in + "design_to_principal: " + quat_list(m.design_to_principal) + "\n";
    out += in + "com_offset: " + vec3_list(m.com_offset) + "\n";
    out += in + "body: {mass: " + format_float(m.body.mass) +
           ", inertia_diag: " + vec3_list(m.body.inertia_diag) + "}\n";
    out += in + "proxy_radius: " + format_float(m.proxy_radius) + "\n";

    if (m.rotors.empty()) {
        out += in + "rotors: []\n";
    } else {
        out += in + "rotors:\n";
        for (const vehicles::RotorDesc& r : m.rotors) {
            out += in + "  - {local_pos: " + vec3_list(r.local_pos) +
                   ", local_orient: " + quat_list(r.local_orient) +
                   ", spin_dir: " + format_float(r.spin_dir) + ", tau: " + format_float(r.tau) +
                   ", radius: " + format_float(r.radius) +
                   ", thrust_coeff: " + format_float(r.thrust_coeff) +
                   ", torque_coeff: " + format_float(r.torque_coeff) + "}\n";
        }
    }

    if (m.drag_bodies.empty()) {
        out += in + "drag_bodies: []\n";
    } else {
        out += in + "drag_bodies:\n";
        for (const vehicles::DragBodyDesc& d : m.drag_bodies) {
            out += in + "  - {mode: " + quote_yaml(kDragModeNames[d.mode]) +
                   ", area: " + format_float(d.area) + ", coeffs: " + vec3_list(d.coeffs) +
                   ", local_pos: " + vec3_list(d.local_pos) +
                   ", local_orient: " + quat_list(d.local_orient) + "}\n";
        }
    }

    if (m.imu_mounts.empty()) {
        out += in + "imu_mounts: []\n";
    } else {
        out += in + "imu_mounts:\n";
        for (const vehicles::ImuMountDesc& i : m.imu_mounts) {
            out += in + "  - {mount_pos: " + vec3_list(i.mount_pos) +
                   ", mount_orient: " + quat_list(i.mount_orient) +
                   ", rate_divider: " + dec(i.rate_divider) + ", sigma_a: " + format_float(i.sigma_a) +
                   ", sigma_g: " + format_float(i.sigma_g) + ", sigma_ba: " + format_float(i.sigma_ba) +
                   ", sigma_bg: " + format_float(i.sigma_bg) + "}\n";
        }
    }

    // Parts (version 2, CORE-7): written only when there are any, so a model
    // without them is the text it always was. No `parts: []`.
    if (!m.parts.empty()) {
        out += in + "parts:\n";
        for (const vehicles::PartDesc& p : m.parts) {
            emit_part(out, p, in);
        }
    }
}

namespace {

void emit_asset(std::string& out, const SceneAsset& a) {
    out += "  - name: " + quote_yaml(a.name) + "\n";
    out += "    pose: {position: " + vec3_list(a.pose.position) +
           ", orientation: " + quat_list(a.pose.rotation) +
           ", scale: " + format_float(a.pose.scale) + "}\n";

    // The world file's SDF encoding, nested; its node_materials by name.
    out += "    collider:\n";
    out += "      sdf:\n";
    emit_sdf_lists(out, a.collider, "        ");
    out += "        node_materials: [";
    for (std::size_t i = 0; i < a.collider_materials.size(); ++i) {
        if (i != 0) out += ", ";
        out += quote_yaml(a.collider_materials[i]);
    }
    out += "]\n";

    out += "    visual: {mesh_ref: " + quote_yaml(a.visual.mesh_ref) +
           ", material: " + quote_yaml(a.visual.material) + "}\n";
}

void emit_vehicle(std::string& out, const SceneVehicle& v) {
    out += "  - name: " + quote_yaml(v.name) + "\n";
    out += "    model: " + quote_yaml(v.model) + "\n";
    out += "    start: {position: " + vec3_list(v.start.pos) +
           ", orientation: " + quat_list(v.start.orient) + ", velocity: " + vec3_list(v.start.vel) +
           ", omega_body: " + vec3_list(v.start.omega_body) +
           ", rotor_omega: " + format_float(v.start.rotor_omega) + "}\n";
}

[[nodiscard]] std::string emit(const SceneDesc& scene) {
    const uint32_t version = written_version(scene);
    std::string out;
    out.reserve(4096);
    out += kHeaderRule;
    out += "# Spade scene file -- schema version " + dec(version) + ".\n";
    out += kFileHeader;

    out += "\nscene_version: " + dec(version);
    out += "\nname: " + quote_yaml(scene.name) + "\n";

    out +=
        "\n"
        "# The world this scene is placed in: a path relative to this file, and\n"
        "# FNV-1a 64 of the world file's canonical text of that world. Another\n"
        "# world is refused.\n"
        "world:\n";
    out += "  file: " + quote_yaml(scene.world.file) + "\n";
    out += "  hash: " + quote_yaml(hex64(scene.world.hash)) + "\n";

    out +=
        "\n"
        "# Appended after the world's material palette. A name the world already\n"
        "# uses is refused.\n";
    if (scene.materials.empty()) {
        out += "materials: []\n";
    } else {
        out += "materials:\n";
        for (const MaterialDesc& m : scene.materials) {
            out += "  - " + material_flow(m) + "\n";
        }
    }

    out +=
        "\n"
        "# Vehicle models, registered in this order. `design_to_principal` turns the\n"
        "# design frame into the principal-axis body frame; `com_offset` is the\n"
        "# centre of mass from the design origin, in the design frame. Rotor, drag\n"
        "# and IMU mounts are in the body frame.\n";
    if (version >= kPartsVersion) {
        out +=
            "# Parts (scene version 2): a part's mount is in the body frame, or, on a\n"
            "# rotor, in that rotor's hub frame at azimuth 0. Its shape is the world\n"
            "# file's SDF encoding, in the part's own frame.\n";
    }
    if (version >= kMeshPartsVersion) {
        out +=
            "# A mesh part (scene version 3) names its .spmesh sidecar: a path\n"
            "# relative to this file, and FNV-1a 64 of the mesh's content. Another\n"
            "# mesh is refused.\n";
    }
    if (scene.models.empty()) {
        out += "models: []\n";
    } else {
        out += "models:\n";
        for (const vehicles::ModelType& m : scene.models) {
            detail::emit_model(out, m, "  - ", "    ");
        }
    }

    out +=
        "\n"
        "# Static scenery, in scene order. `collider.sdf` is the world file's SDF\n"
        "# encoding in the asset's frame, with node_materials by name; no nodes means\n"
        "# no collision. An empty visual means none.\n";
    if (scene.assets.empty()) {
        out += "assets: []\n";
    } else {
        out += "assets:\n";
        for (const SceneAsset& a : scene.assets) {
            emit_asset(out, a);
        }
    }

    out +=
        "\n"
        "# Vehicles, in scene order. A start is in its model's design frame.\n";
    if (scene.vehicles.empty()) {
        out += "vehicles: []\n";
    } else {
        out += "vehicles:\n";
        for (const SceneVehicle& v : scene.vehicles) {
            emit_vehicle(out, v);
        }
    }

    out +=
        "\n"
        "# Slots reserved for objects that appear at runtime, added to the world's\n"
        "# capacities when the scene is composed.\n";
    out += "spare: {bodies: " + dec(scene.spare.bodies) +
           ", force_elements: " + dec(scene.spare.force_elements) +
           ", sensors: " + dec(scene.spare.sensors) + ", contacts: " + dec(scene.spare.contacts) +
           "}\n";
    return out;
}

// ===========================================================================
// Parsing
// ===========================================================================

[[nodiscard]] Result<YAML::Node> sequence_field(const YAML::Node& map, const char* key,
                                                std::string_view what) {
    Result<YAML::Node> node = field(map, key, what);
    if (!node) return std::unexpected(node.error());
    if (!node->IsSequence()) {
        return std::unexpected(at(*node, std::string(what) + "." + key + " must be a sequence"));
    }
    return node;
}

[[nodiscard]] Result<YAML::Node> map_field(const YAML::Node& map, const char* key, std::string_view what,
                                           std::initializer_list<std::string_view> allowed, std::string_view schema) {
    Result<YAML::Node> node = field(map, key, what);
    if (!node) return std::unexpected(node.error());
    if (Result<void> r = check_map(*node, std::string(what) + "." + key, allowed, schema); !r) {
        return std::unexpected(r.error());
    }
    return node;
}

[[nodiscard]] Result<uint64_t> parse_hash(const YAML::Node& node, std::string_view what) {
    const Result<std::string> text = scalar(node, what);
    if (!text) return std::unexpected(text.error());
    const std::string_view body(*text);
    const auto refuse = [&] {
        return std::unexpected(
            at(node, std::string(what) + ": '" + *text + "' must be \"0x\" followed by 16 hex digits"));
    };
    if (body.size() != 18 || body[0] != '0' || body[1] != 'x') return refuse();
    uint64_t value = 0;
    const std::from_chars_result r = std::from_chars(body.data() + 2, body.data() + body.size(), value, 16);
    if (r.ec != std::errc{} || r.ptr != body.data() + body.size()) return refuse();
    return value;
}

// One of `names` by name, at `map[key]`: the index is the enumerator's value.
template <std::size_t N>
[[nodiscard]] Result<uint32_t> named_field(const YAML::Node& map, const char* key, const std::string& what,
                                           const std::string_view (&names)[N], std::string_view kind) {
    const Result<YAML::Node> node = field(map, key, what);
    if (!node) return std::unexpected(node.error());
    const std::string where = what + "." + key;
    const Result<std::string> name = scalar(*node, where);
    if (!name) return std::unexpected(name.error());
    const std::string_view* found = std::find(std::begin(names), std::end(names), std::string_view(*name));
    if (found != std::end(names)) return static_cast<uint32_t>(found - std::begin(names));
    std::string expected;
    for (std::size_t i = 0; i < N; ++i) {
        expected += (i == 0) ? "" : (i + 1 == N) ? " or " : ", ";
        expected += names[i];
    }
    return std::unexpected(
        at(*node, where + ": unknown " + std::string(kind) + " '" + *name + "' (expected " + expected + ")"));
}

[[nodiscard]] Result<vehicles::RotorDesc> parse_rotor(const YAML::Node& node, const std::string& what,
                                                      std::string_view schema) {
    if (Result<void> r = check_map(node, what,
                                   {"local_pos", "local_orient", "spin_dir", "tau", "radius",
                                    "thrust_coeff", "torque_coeff"},
                                   schema);
        !r) {
        return std::unexpected(r.error());
    }
    vehicles::RotorDesc out{};
    const Result<glm::vec3> local_pos = vec3_field(node, "local_pos", what);
    if (!local_pos) return std::unexpected(local_pos.error());
    const Result<glm::quat> local_orient = quat_field(node, "local_orient", what);
    if (!local_orient) return std::unexpected(local_orient.error());
    const Result<float> spin_dir = float_field(node, "spin_dir", what);
    if (!spin_dir) return std::unexpected(spin_dir.error());
    const Result<float> tau = float_field(node, "tau", what);
    if (!tau) return std::unexpected(tau.error());
    const Result<float> radius = float_field(node, "radius", what);
    if (!radius) return std::unexpected(radius.error());
    const Result<float> thrust_coeff = float_field(node, "thrust_coeff", what);
    if (!thrust_coeff) return std::unexpected(thrust_coeff.error());
    const Result<float> torque_coeff = float_field(node, "torque_coeff", what);
    if (!torque_coeff) return std::unexpected(torque_coeff.error());
    out.local_pos = *local_pos;
    out.local_orient = *local_orient;
    out.spin_dir = *spin_dir;
    out.tau = *tau;
    out.radius = *radius;
    out.thrust_coeff = *thrust_coeff;
    out.torque_coeff = *torque_coeff;
    return out;
}

[[nodiscard]] Result<vehicles::DragBodyDesc> parse_drag_body(const YAML::Node& node, const std::string& what,
                                                             std::string_view schema) {
    if (Result<void> r = check_map(node, what, {"mode", "area", "coeffs", "local_pos", "local_orient"}, schema); !r) {
        return std::unexpected(r.error());
    }
    vehicles::DragBodyDesc out{};
    const Result<YAML::Node> mode_node = field(node, "mode", what);
    if (!mode_node) return std::unexpected(mode_node.error());
    const Result<std::string> mode = scalar(*mode_node, what + ".mode");
    if (!mode) return std::unexpected(mode.error());
    const std::string_view* found =
        std::find(std::begin(kDragModeNames), std::end(kDragModeNames), std::string_view(*mode));
    if (found == std::end(kDragModeNames)) {
        return std::unexpected(at(*mode_node, what + ".mode: unknown drag mode '" + *mode +
                                                  "' (expected quadratic or componentwise)"));
    }
    out.mode = static_cast<uint32_t>(found - std::begin(kDragModeNames));
    const Result<float> area = float_field(node, "area", what);
    if (!area) return std::unexpected(area.error());
    const Result<glm::vec3> coeffs = vec3_field(node, "coeffs", what);
    if (!coeffs) return std::unexpected(coeffs.error());
    const Result<glm::vec3> local_pos = vec3_field(node, "local_pos", what);
    if (!local_pos) return std::unexpected(local_pos.error());
    const Result<glm::quat> local_orient = quat_field(node, "local_orient", what);
    if (!local_orient) return std::unexpected(local_orient.error());
    out.area = *area;
    out.coeffs = *coeffs;
    out.local_pos = *local_pos;
    out.local_orient = *local_orient;
    return out;
}

[[nodiscard]] Result<vehicles::ImuMountDesc> parse_imu_mount(const YAML::Node& node, const std::string& what,
                                                             std::string_view schema) {
    if (Result<void> r = check_map(node, what,
                                   {"mount_pos", "mount_orient", "rate_divider", "sigma_a", "sigma_g",
                                    "sigma_ba", "sigma_bg"},
                                   schema);
        !r) {
        return std::unexpected(r.error());
    }
    vehicles::ImuMountDesc out{};
    const Result<glm::vec3> mount_pos = vec3_field(node, "mount_pos", what);
    if (!mount_pos) return std::unexpected(mount_pos.error());
    const Result<glm::quat> mount_orient = quat_field(node, "mount_orient", what);
    if (!mount_orient) return std::unexpected(mount_orient.error());
    const Result<uint32_t> rate_divider = uint_field<uint32_t>(node, "rate_divider", what);
    if (!rate_divider) return std::unexpected(rate_divider.error());
    const Result<float> sigma_a = float_field(node, "sigma_a", what);
    if (!sigma_a) return std::unexpected(sigma_a.error());
    const Result<float> sigma_g = float_field(node, "sigma_g", what);
    if (!sigma_g) return std::unexpected(sigma_g.error());
    const Result<float> sigma_ba = float_field(node, "sigma_ba", what);
    if (!sigma_ba) return std::unexpected(sigma_ba.error());
    const Result<float> sigma_bg = float_field(node, "sigma_bg", what);
    if (!sigma_bg) return std::unexpected(sigma_bg.error());
    out.mount_pos = *mount_pos;
    out.mount_orient = *mount_orient;
    out.rate_divider = *rate_divider;
    out.sigma_a = *sigma_a;
    out.sigma_g = *sigma_g;
    out.sigma_ba = *sigma_ba;
    out.sigma_bg = *sigma_bg;
    return out;
}

// An SDF program's `transforms` and `nodes` lists, in the mapping `sdf` (whose
// keys the caller has checked): the world file's encoding, shared by an
// asset's collider and a part's shape.
[[nodiscard]] Result<SdfProgram> parse_sdf_lists(const YAML::Node& sdf, const std::string& sdf_what,
                                                 std::string_view schema) {
    SdfProgram out{};
    const Result<YAML::Node> transforms = sequence_field(sdf, "transforms", sdf_what);
    if (!transforms) return std::unexpected(transforms.error());
    for (std::size_t i = 0; i < transforms->size(); ++i) {
        const Result<SdfTransform> t =
            yaml_text::parse_sdf_transform((*transforms)[i], sdf_what + ".transforms[" + dec(i) + "]", schema);
        if (!t) return std::unexpected(t.error());
        out.transforms.push_back(*t);
    }
    const Result<YAML::Node> nodes = sequence_field(sdf, "nodes", sdf_what);
    if (!nodes) return std::unexpected(nodes.error());
    for (std::size_t i = 0; i < nodes->size(); ++i) {
        const Result<SdfNode> n = yaml_text::parse_sdf_node((*nodes)[i], sdf_what + ".nodes[" + dec(i) + "]", schema);
        if (!n) return std::unexpected(n.error());
        out.nodes.push_back(*n);
    }
    return out;
}

// A mesh part's geometry, as its shape names it.
struct PartMesh {
    std::shared_ptr<const TriangleMesh> mesh{};
    std::string file{};
};

// A mesh part's shape (version 3, CORE-6): `shape.mesh` is {file, hash}, the
// sidecar's path relative to the <kind> file and the mesh's content hash
// (SCN-001's form). The sidecar is read from base_dir / file, against the
// hash, by load_mesh_file(), whose refusals keep their own code and message,
// after the dotted path and with the line. Nothing here repairs a mesh.
[[nodiscard]] Result<PartMesh> parse_part_mesh(const YAML::Node& shape, const std::string& shape_what,
                                               std::string_view schema, std::string_view kind,
                                               const std::filesystem::path& base_dir) {
    const std::string what = shape_what + ".mesh";
    const Result<YAML::Node> node = map_field(shape, "mesh", shape_what, {"file", "hash"}, schema);
    if (!node) return std::unexpected(node.error());
    const Result<YAML::Node> file_node = field(*node, "file", what);
    if (!file_node) return std::unexpected(file_node.error());
    const Result<std::string> file = scalar(*file_node, what + ".file");
    if (!file) return std::unexpected(file.error());
    if (file->empty()) {
        return std::unexpected(at(*file_node, what + ".file must name the mesh's sidecar"));
    }
    if (const std::string problem = detail::relative_path_problem(*file, kind); !problem.empty()) {
        return std::unexpected(at(*file_node, what + ".file '" + *file + "' " + problem));
    }
    const Result<YAML::Node> hash_node = field(*node, "hash", what);
    if (!hash_node) return std::unexpected(hash_node.error());
    const Result<uint64_t> hash = parse_hash(*hash_node, what + ".hash");
    if (!hash) return std::unexpected(hash.error());

    if (base_dir.empty()) {
        const std::string k(kind);
        return std::unexpected(at(*node, what + ": a sidecar's path is relative to the " + k +
                                             " file, and this text came with no directory: read a " + k +
                                             " with mesh parts with load_" + k +
                                             "_file, or give a base directory"));
    }
    std::filesystem::path sidecar;
    try {
        // The file is UTF-8 text; path(std::string) would read it in the
        // ANSI code page on Windows.
        std::u8string utf8;
        utf8.reserve(file->size());
        for (const char c : *file) utf8.push_back(static_cast<char8_t>(c));
        sidecar = base_dir / std::filesystem::path(utf8);
    } catch (const std::exception& e) {
        return std::unexpected(
            at(*file_node, what + ".file '" + *file + "' is not a path this platform can open: " + e.what()));
    }
    Result<LoadedMesh> loaded = load_mesh_file(sidecar, *hash);
    if (!loaded) {
        return std::unexpected(Error{loaded.error().code, what + ": " + loaded.error().context + mark_of(*node)});
    }
    return PartMesh{std::move(loaded->mesh), *file};
}

// One part (version 2, CORE-7). Only the text's shape is checked here; the
// part's rules are ModelType::issues()'s, which validate_scene() runs (TD-9).
// `reads_mesh`: the key set is version 3 or later, so a shape may be a mesh.
[[nodiscard]] Result<vehicles::PartDesc> parse_part(const YAML::Node& node, const std::string& what,
                                                    std::string_view schema, bool reads_mesh,
                                                    std::string_view file_kind,
                                                    const std::filesystem::path& base_dir) {
    if (Result<void> r = check_map(node, what,
                                   {"name", "role", "attach", "rotor", "mount", "surface", "min_feature", "shape"},
                                   schema);
        !r) {
        return std::unexpected(r.error());
    }
    vehicles::PartDesc out{};
    const Result<std::string> name = string_field(node, "name", what);
    if (!name) return std::unexpected(name.error());
    const Result<std::string> role = string_field(node, "role", what);
    if (!role) return std::unexpected(role.error());
    const Result<uint32_t> attach = named_field(node, "attach", what, kPartAttachNames, "part attachment");
    if (!attach) return std::unexpected(attach.error());
    const Result<uint32_t> rotor = uint_field<uint32_t>(node, "rotor", what);
    if (!rotor) return std::unexpected(rotor.error());

    const std::string mount_what = what + ".mount";
    const Result<YAML::Node> mount = map_field(node, "mount", what, {"position", "orientation"}, schema);
    if (!mount) return std::unexpected(mount.error());
    const Result<glm::vec3> position = vec3_field(*mount, "position", mount_what);
    if (!position) return std::unexpected(position.error());
    const Result<glm::quat> orientation = quat_field(*mount, "orientation", mount_what);
    if (!orientation) return std::unexpected(orientation.error());

    const std::string surface_what = what + ".surface";
    const Result<YAML::Node> surface =
        map_field(node, "surface", what, {"kind", "inv_permeability", "inertial_coeff"}, schema);
    if (!surface) return std::unexpected(surface.error());
    const Result<uint32_t> kind = named_field(*surface, "kind", surface_what, kPartSurfaceNames, "part surface");
    if (!kind) return std::unexpected(kind.error());
    const Result<float> inv_permeability = float_field(*surface, "inv_permeability", surface_what);
    if (!inv_permeability) return std::unexpected(inv_permeability.error());
    const Result<float> inertial_coeff = float_field(*surface, "inertial_coeff", surface_what);
    if (!inertial_coeff) return std::unexpected(inertial_coeff.error());

    const Result<float> min_feature = float_field(node, "min_feature", what);
    if (!min_feature) return std::unexpected(min_feature.error());

    // shape: {sdf: {transforms, nodes}} -- no node_materials, which is render
    // data a part never carries, so it is an unknown key here -- or, from
    // version 3, {mesh: {file, hash}}: exactly one of the two. A version-2
    // file's key set stays as it was, so `mesh` there is an unknown key.
    const std::string shape_what = what + ".shape";
    const Result<YAML::Node> shape = reads_mesh ? map_field(node, "shape", what, {"sdf", "mesh"}, schema)
                                                : map_field(node, "shape", what, {"sdf"}, schema);
    if (!shape) return std::unexpected(shape.error());
    SdfProgram program{};
    PartMesh mesh{};
    if (reads_mesh && has(*shape, "sdf") == has(*shape, "mesh")) {
        return std::unexpected(at(*shape, shape_what + ": exactly one of sdf and mesh, and " +
                                              (has(*shape, "sdf") ? "both are given" : "neither is given")));
    }
    if (reads_mesh && has(*shape, "mesh")) {
        Result<PartMesh> m = parse_part_mesh(*shape, shape_what, schema, file_kind, base_dir);
        if (!m) return std::unexpected(m.error());
        mesh = std::move(*m);
    } else {
        const Result<YAML::Node> sdf = map_field(*shape, "sdf", shape_what, {"transforms", "nodes"}, schema);
        if (!sdf) return std::unexpected(sdf.error());
        Result<SdfProgram> parsed = parse_sdf_lists(*sdf, shape_what + ".sdf", schema);
        if (!parsed) return std::unexpected(parsed.error());
        program = std::move(*parsed);
    }

    out.name = *name;
    out.role = *role;
    out.attach = static_cast<vehicles::PartAttach>(*attach);
    out.rotor = *rotor;
    out.mount_pos = *position;
    out.mount_orient = *orientation;
    out.shape = std::move(program);
    out.mesh = std::move(mesh.mesh);
    out.mesh_file = std::move(mesh.file);
    out.surface.kind = static_cast<vehicles::PartSurface::Kind>(*kind);
    out.surface.inv_permeability = *inv_permeability;
    out.surface.inertial_coeff = *inertial_coeff;
    out.min_feature = *min_feature;
    return out;
}

}  // namespace

// `format.key_version` gates the key set (`parts` from scene version 2, a
// part's mesh from version 3), and every diagnostic blames `format.schema`,
// the file's own. `base_dir` resolves a mesh part's sidecar.
Result<vehicles::ModelType> detail::parse_model(const YAML::Node& node, const std::string& what,
                                                const ModelBlockFormat& format,
                                                const std::filesystem::path& base_dir) {
    const std::string file_schema(format.schema);
    const bool reads_parts = format.key_version >= kPartsVersion;
    if (reads_parts) {
        if (Result<void> r = check_map(node, what,
                                       {"name", "version", "param_schema_id", "visual_ref", "design_to_principal",
                                        "com_offset", "body", "proxy_radius", "rotors", "drag_bodies",
                                        "imu_mounts", "parts"},
                                       file_schema);
            !r) {
            return std::unexpected(r.error());
        }
    } else if (Result<void> r = check_map(node, what,
                                          {"name", "version", "param_schema_id", "visual_ref",
                                           "design_to_principal", "com_offset", "body", "proxy_radius",
                                           "rotors", "drag_bodies", "imu_mounts"},
                                          file_schema);
               !r) {
        return std::unexpected(r.error());
    }
    vehicles::ModelType out{};
    const Result<std::string> name = string_field(node, "name", what);
    if (!name) return std::unexpected(name.error());
    const Result<uint32_t> version = uint_field<uint32_t>(node, "version", what);
    if (!version) return std::unexpected(version.error());
    const Result<uint32_t> schema_id = uint_field<uint32_t>(node, "param_schema_id", what);
    if (!schema_id) return std::unexpected(schema_id.error());
    const Result<std::string> visual_ref = string_field(node, "visual_ref", what);
    if (!visual_ref) return std::unexpected(visual_ref.error());
    const Result<glm::quat> design_to_principal = quat_field(node, "design_to_principal", what);
    if (!design_to_principal) return std::unexpected(design_to_principal.error());
    const Result<glm::vec3> com_offset = vec3_field(node, "com_offset", what);
    if (!com_offset) return std::unexpected(com_offset.error());

    const Result<YAML::Node> body = map_field(node, "body", what, {"mass", "inertia_diag"}, file_schema);
    if (!body) return std::unexpected(body.error());
    const Result<float> mass = float_field(*body, "mass", what + ".body");
    if (!mass) return std::unexpected(mass.error());
    const Result<glm::vec3> inertia = vec3_field(*body, "inertia_diag", what + ".body");
    if (!inertia) return std::unexpected(inertia.error());
    const Result<float> proxy_radius = float_field(node, "proxy_radius", what);
    if (!proxy_radius) return std::unexpected(proxy_radius.error());

    out.name = *name;
    out.version = *version;
    out.param_schema_id = *schema_id;
    out.visual_ref = *visual_ref;
    out.design_to_principal = *design_to_principal;
    out.com_offset = *com_offset;
    out.body.mass = *mass;
    out.body.inertia_diag = *inertia;
    out.proxy_radius = *proxy_radius;

    const Result<YAML::Node> rotors = sequence_field(node, "rotors", what);
    if (!rotors) return std::unexpected(rotors.error());
    for (std::size_t i = 0; i < rotors->size(); ++i) {
        const Result<vehicles::RotorDesc> r =
            parse_rotor((*rotors)[i], what + ".rotors[" + dec(i) + "]", file_schema);
        if (!r) return std::unexpected(r.error());
        out.rotors.push_back(*r);
    }
    const Result<YAML::Node> drag = sequence_field(node, "drag_bodies", what);
    if (!drag) return std::unexpected(drag.error());
    for (std::size_t i = 0; i < drag->size(); ++i) {
        const Result<vehicles::DragBodyDesc> d =
            parse_drag_body((*drag)[i], what + ".drag_bodies[" + dec(i) + "]", file_schema);
        if (!d) return std::unexpected(d.error());
        out.drag_bodies.push_back(*d);
    }
    const Result<YAML::Node> imus = sequence_field(node, "imu_mounts", what);
    if (!imus) return std::unexpected(imus.error());
    for (std::size_t i = 0; i < imus->size(); ++i) {
        const Result<vehicles::ImuMountDesc> m =
            parse_imu_mount((*imus)[i], what + ".imu_mounts[" + dec(i) + "]", file_schema);
        if (!m) return std::unexpected(m.error());
        out.imu_mounts.push_back(*m);
    }

    // parts: optional, as the writer omits it for a model without them; an
    // empty list is no parts.
    if (reads_parts && has(node, "parts")) {
        const Result<YAML::Node> parts = sequence_field(node, "parts", what);
        if (!parts) return std::unexpected(parts.error());
        for (std::size_t i = 0; i < parts->size(); ++i) {
            Result<vehicles::PartDesc> p = parse_part((*parts)[i], what + ".parts[" + dec(i) + "]", file_schema,
                                                      format.key_version >= kMeshPartsVersion, format.kind, base_dir);
            if (!p) return std::unexpected(p.error());
            out.parts.push_back(std::move(*p));
        }
    }
    return out;
}

namespace {

[[nodiscard]] Result<SceneAsset> parse_asset(const YAML::Node& node, const std::string& what,
                                             std::string_view schema) {
    if (Result<void> r = check_map(node, what, {"name", "pose", "collider", "visual"}, schema); !r) {
        return std::unexpected(r.error());
    }
    SceneAsset out{};
    const Result<std::string> name = string_field(node, "name", what);
    if (!name) return std::unexpected(name.error());
    out.name = *name;

    const std::string pose_what = what + ".pose";
    const Result<YAML::Node> pose = map_field(node, "pose", what, {"position", "orientation", "scale"}, schema);
    if (!pose) return std::unexpected(pose.error());
    const Result<glm::vec3> position = vec3_field(*pose, "position", pose_what);
    if (!position) return std::unexpected(position.error());
    const Result<glm::quat> orientation = quat_field(*pose, "orientation", pose_what);
    if (!orientation) return std::unexpected(orientation.error());
    const Result<float> scale = float_field(*pose, "scale", pose_what);
    if (!scale) return std::unexpected(scale.error());
    out.pose.position = *position;
    out.pose.rotation = *orientation;
    out.pose.scale = *scale;

    // collider: {sdf: {transforms, nodes, node_materials}} -- the world file's
    // transform and node encodings; node_materials holds NAMES here.
    const std::string collider_what = what + ".collider";
    const Result<YAML::Node> collider = map_field(node, "collider", what, {"sdf"}, schema);
    if (!collider) return std::unexpected(collider.error());
    const std::string sdf_what = collider_what + ".sdf";
    const Result<YAML::Node> sdf =
        map_field(*collider, "sdf", collider_what, {"transforms", "nodes", "node_materials"}, schema);
    if (!sdf) return std::unexpected(sdf.error());

    Result<SdfProgram> program = parse_sdf_lists(*sdf, sdf_what, schema);
    if (!program) return std::unexpected(program.error());
    out.collider = std::move(*program);
    const Result<YAML::Node> names = sequence_field(*sdf, "node_materials", sdf_what);
    if (!names) return std::unexpected(names.error());
    for (std::size_t i = 0; i < names->size(); ++i) {
        const Result<std::string> n = scalar((*names)[i], sdf_what + ".node_materials[" + dec(i) + "]");
        if (!n) return std::unexpected(n.error());
        out.collider_materials.push_back(*n);
    }

    const std::string visual_what = what + ".visual";
    const Result<YAML::Node> visual = map_field(node, "visual", what, {"mesh_ref", "material"}, schema);
    if (!visual) return std::unexpected(visual.error());
    const Result<std::string> mesh_ref = string_field(*visual, "mesh_ref", visual_what);
    if (!mesh_ref) return std::unexpected(mesh_ref.error());
    const Result<std::string> material = string_field(*visual, "material", visual_what);
    if (!material) return std::unexpected(material.error());
    out.visual.mesh_ref = *mesh_ref;
    out.visual.material = *material;
    return out;
}

[[nodiscard]] Result<SceneVehicle> parse_vehicle(const YAML::Node& node, const std::string& what,
                                                 std::string_view schema) {
    if (Result<void> r = check_map(node, what, {"name", "model", "start"}, schema); !r) {
        return std::unexpected(r.error());
    }
    SceneVehicle out{};
    const Result<std::string> name = string_field(node, "name", what);
    if (!name) return std::unexpected(name.error());
    const Result<std::string> model = string_field(node, "model", what);
    if (!model) return std::unexpected(model.error());
    out.name = *name;
    out.model = *model;

    const std::string start_what = what + ".start";
    const Result<YAML::Node> start = map_field(
        node, "start", what, {"position", "orientation", "velocity", "omega_body", "rotor_omega"}, schema);
    if (!start) return std::unexpected(start.error());
    const Result<glm::vec3> position = vec3_field(*start, "position", start_what);
    if (!position) return std::unexpected(position.error());
    const Result<glm::quat> orientation = quat_field(*start, "orientation", start_what);
    if (!orientation) return std::unexpected(orientation.error());
    const Result<glm::vec3> velocity = vec3_field(*start, "velocity", start_what);
    if (!velocity) return std::unexpected(velocity.error());
    const Result<glm::vec3> omega_body = vec3_field(*start, "omega_body", start_what);
    if (!omega_body) return std::unexpected(omega_body.error());
    const Result<float> rotor_omega = float_field(*start, "rotor_omega", start_what);
    if (!rotor_omega) return std::unexpected(rotor_omega.error());
    out.start.pos = *position;
    out.start.orient = *orientation;
    out.start.vel = *velocity;
    out.start.omega_body = *omega_body;
    out.start.rotor_omega = *rotor_omega;
    return out;
}

// spare is optional, and so is each of its counts (SCN-007): absent means 0.
// Its keys are still accounted for, unknown and duplicate alike.
[[nodiscard]] Result<SceneSpare> parse_spare(const YAML::Node& node, std::string_view schema) {
    constexpr std::string_view kWhat = "scene.spare";
    if (Result<void> r = check_map(node, kWhat, {"bodies", "force_elements", "sensors", "contacts"}, schema); !r) {
        return std::unexpected(r.error());
    }
    SceneSpare out{};
    const std::pair<const char*, uint32_t*> counts[] = {{"bodies", &out.bodies},
                                                        {"force_elements", &out.force_elements},
                                                        {"sensors", &out.sensors},
                                                        {"contacts", &out.contacts}};
    for (const auto& [key, slot] : counts) {
        if (!has(node, key)) continue;
        const Result<uint32_t> n = uint_field<uint32_t>(node, key, kWhat);
        if (!n) return std::unexpected(n.error());
        *slot = *n;
    }
    return out;
}

[[nodiscard]] Result<SceneDesc> parse_scene(const YAML::Node& root, const std::filesystem::path& base_dir) {
    if (!root.IsMap()) {
        return std::unexpected(at(root, "a scene file's top level must be a mapping with a 'scene_version' key"));
    }

    // VERSION FIRST, before the unknown-key sweep -- the world file's rule: a
    // later version's keys would otherwise be reported as unknown, a true but
    // useless answer to "this build does not read that version".
    const Result<YAML::Node> version_node = field(root, "scene_version", "scene");
    if (!version_node) return std::unexpected(version_node.error());
    const Result<uint32_t> version = parse_uint<uint32_t>(*version_node, "scene.scene_version");
    if (!version) return std::unexpected(version.error());
    if (*version < kSceneFileOldestVersion || *version > kSceneFileVersion) {
        return std::unexpected(Error{Code::schema_mismatch,
                                     "scene file is schema version " + dec(*version) + ", this build reads versions " +
                                         dec(kSceneFileOldestVersion) + " to " + dec(kSceneFileVersion) +
                                         " only" + mark_of(*version_node)});
    }
    // Every diagnostic below blames the file's own version.
    const std::string file_schema = schema(*version);

    // The top level is the same in every version this build reads; a model's
    // key set is not (parse_model).
    if (Result<void> r = check_map(root, "scene",
                                   {"scene_version", "name", "world", "materials", "models", "assets",
                                    "vehicles", "spare"},
                                   file_schema);
        !r) {
        return std::unexpected(r.error());
    }

    SceneDesc out{};
    const Result<std::string> name = string_field(root, "name", "scene");
    if (!name) return std::unexpected(name.error());
    out.name = *name;

    const Result<YAML::Node> world = map_field(root, "world", "scene", {"file", "hash"}, file_schema);
    if (!world) return std::unexpected(world.error());
    const Result<std::string> file = string_field(*world, "file", "scene.world");
    if (!file) return std::unexpected(file.error());
    const Result<YAML::Node> hash_node = field(*world, "hash", "scene.world");
    if (!hash_node) return std::unexpected(hash_node.error());
    const Result<uint64_t> hash = parse_hash(*hash_node, "scene.world.hash");
    if (!hash) return std::unexpected(hash.error());
    out.world.file = *file;
    out.world.hash = *hash;

    const Result<YAML::Node> materials = sequence_field(root, "materials", "scene");
    if (!materials) return std::unexpected(materials.error());
    for (std::size_t i = 0; i < materials->size(); ++i) {
        const Result<MaterialDesc> m =
            yaml_text::parse_material((*materials)[i], "scene.materials[" + dec(i) + "]", file_schema);
        if (!m) return std::unexpected(m.error());
        out.materials.push_back(*m);
    }

    const Result<YAML::Node> models = sequence_field(root, "models", "scene");
    if (!models) return std::unexpected(models.error());
    for (std::size_t i = 0; i < models->size(); ++i) {
        Result<vehicles::ModelType> m = detail::parse_model(
            (*models)[i], "scene.models[" + dec(i) + "]", detail::ModelBlockFormat{*version, file_schema, "scene"},
            base_dir);
        if (!m) return std::unexpected(m.error());
        out.models.push_back(std::move(*m));
    }

    const Result<YAML::Node> assets = sequence_field(root, "assets", "scene");
    if (!assets) return std::unexpected(assets.error());
    for (std::size_t i = 0; i < assets->size(); ++i) {
        const Result<SceneAsset> a = parse_asset((*assets)[i], "scene.assets[" + dec(i) + "]", file_schema);
        if (!a) return std::unexpected(a.error());
        out.assets.push_back(*a);
    }

    const Result<YAML::Node> vehicle_list = sequence_field(root, "vehicles", "scene");
    if (!vehicle_list) return std::unexpected(vehicle_list.error());
    for (std::size_t i = 0; i < vehicle_list->size(); ++i) {
        const Result<SceneVehicle> v =
            parse_vehicle((*vehicle_list)[i], "scene.vehicles[" + dec(i) + "]", file_schema);
        if (!v) return std::unexpected(v.error());
        out.vehicles.push_back(*v);
    }

    if (has(root, "spare")) {
        const Result<SceneSpare> spare = parse_spare(root["spare"], file_schema);
        if (!spare) return std::unexpected(spare.error());
        out.spare = *spare;
    }
    return out;
}

// yaml-cpp reports every syntax failure by throwing; nothing may leave a Spade
// API that way (world_from_yaml's rule). YAML::Exception::what() carries the
// mark.
[[nodiscard]] Result<SceneDesc> parse_text(std::string_view yaml_text, const std::filesystem::path& base_dir) {
    try {
        const YAML::Node root = YAML::Load(std::string(yaml_text));
        return parse_scene(root, base_dir);
    } catch (const YAML::Exception& e) {
        return std::unexpected(Error{Code::invalid_argument, std::string(e.what())});
    } catch (const std::exception& e) {
        return std::unexpected(Error{Code::internal, std::string("scene file parse failed: ") + e.what()});
    }
}

}  // namespace

// ===========================================================================
// The model block's rules (detail/model_block.hpp)
// ===========================================================================

Result<void> detail::validate_model(const vehicles::ModelType& m, std::string_view schema, std::string_view kind) {
    if (Result<void> r = m.validate(); !r) {
        return r;
    }
    // ModelType accepts any orientable rotation, because registration
    // normalizes it. A FILE carries a unit quaternion (SCN-009), as it does
    // every other orientation.
    if (!unit(m.design_to_principal)) {
        return std::unexpected(invalid("model '" + m.name + "': design_to_principal must be a unit quaternion"));
    }
    // A part's shape must be expressible as its file (SCN-002), as an
    // asset's collider must. Every other part rule is the model's, above.
    for (const vehicles::PartDesc& p : m.parts) {
        if (Result<void> r = yaml_text::check_sdf_representable(
                p.shape, "model '" + m.name + "' part '" + p.name + "' shape", schema);
            !r) {
            return r;
        }
        // A mesh part's sidecar name (CORE-6): relative, forward slashes,
        // and only beside a mesh. Empty is fine in memory; the writer
        // needs it (check_model_writable).
        if (!p.mesh_file.empty()) {
            const std::string who = "model '" + m.name + "' part '" + p.name + "': mesh_file '" + p.mesh_file + "' ";
            if (const std::string problem = relative_path_problem(p.mesh_file, kind); !problem.empty()) {
                return std::unexpected(invalid(who + problem));
            }
            if (!p.mesh) {
                return std::unexpected(invalid(who + "names a sidecar, but the part holds no mesh"));
            }
        }
    }
    return {};
}

Result<void> detail::check_model_writable(const vehicles::ModelType& m, std::string_view kind) {
    for (const vehicles::PartDesc& p : m.parts) {
        if (p.mesh && p.mesh_file.empty()) {
            return std::unexpected(invalid("model '" + m.name + "' part '" + p.name +
                                           "': a mesh part needs mesh_file, its sidecar's path relative to the " +
                                           std::string(kind) +
                                           " file; the writer names the sidecar and never writes it (save the "
                                           "mesh with save_mesh_file, then name it)"));
        }
    }
    return {};
}

// ===========================================================================
// Public API
// ===========================================================================

Result<void> validate_scene(const SceneDesc& scene) {
    if (scene.name.empty()) {
        return std::unexpected(invalid("scene name must not be empty"));
    }

    // --- the world reference (SCN-001) -------------------------------------
    const std::string& file = scene.world.file;
    if (file.empty()) {
        return std::unexpected(invalid("scene world.file must name the world file"));
    }
    if (const std::string problem = detail::relative_path_problem(file, "scene"); !problem.empty()) {
        return std::unexpected(invalid("scene world.file '" + file + "' " + problem));
    }

    // --- materials: the world palette's rules (validate_material, the one
    // definition), plus unique names (SCN-004) -------------------------------
    for (std::size_t i = 0; i < scene.materials.size(); ++i) {
        const MaterialDesc& m = scene.materials[i];
        if (Result<void> r = validate_material(m, "scene material " + dec(i)); !r) {
            return r;
        }
        for (std::size_t j = 0; j < i; ++j) {
            if (scene.materials[j].name == m.name) {
                return std::unexpected(invalid("duplicate material name '" + m.name + "'"));
            }
        }
    }

    // --- models (SCN-009): the model block's rules, shared with the model
    // file, then unique names -------------------------------------------------
    for (std::size_t i = 0; i < scene.models.size(); ++i) {
        const vehicles::ModelType& m = scene.models[i];
        if (Result<void> r = detail::validate_model(m, schema(kPartsVersion), "scene"); !r) {
            return r;
        }
        for (std::size_t j = 0; j < i; ++j) {
            if (scene.models[j].name == m.name) {
                return std::unexpected(invalid("duplicate model name '" + m.name + "'"));
            }
        }
    }

    // Asset and vehicle names share one namespace (SCN-004): both are objects
    // in the editor's hierarchy.
    std::vector<std::string_view> objects;
    objects.reserve(scene.assets.size() + scene.vehicles.size());
    const auto claim = [&](const std::string& name) -> Result<void> {
        for (const std::string_view seen : objects) {
            if (seen == name) {
                return std::unexpected(invalid("name '" + name +
                                               "' is used twice; names are unique across assets and "
                                               "vehicles together"));
            }
        }
        objects.push_back(name);
        return {};
    };

    // --- assets ------------------------------------------------------------
    for (std::size_t i = 0; i < scene.assets.size(); ++i) {
        const SceneAsset& a = scene.assets[i];
        if (a.name.empty()) {
            return std::unexpected(invalid("scene asset " + dec(i) + " has no name"));
        }
        if (Result<void> r = claim(a.name); !r) {
            return r;
        }
        const std::string where = "asset '" + a.name + "': ";

        if (!finite(a.pose.position) || !finite(a.pose.rotation) || !finite(a.pose.scale)) {
            return std::unexpected(invalid(where + "pose must be finite"));
        }
        if (!(a.pose.scale > 0.0f)) {
            return std::unexpected(invalid(where + "pose scale must be > 0 (uniform scale only)"));
        }
        if (!unit(a.pose.rotation)) {
            return std::unexpected(invalid(where + "pose orientation must be a unit quaternion"));
        }

        const SdfProgram& collider = a.collider;
        if (!collider.node_materials.empty()) {
            return std::unexpected(invalid(where +
                                           "collider node materials are given by name, in "
                                           "collider_materials, never by index"));
        }
        if (collider.nodes.empty() && !collider.transforms.empty()) {
            return std::unexpected(invalid(where + "a collider with no nodes must have no transforms"));
        }
        if (const Result<uint32_t> depth = collider.validate(); !depth) {
            return std::unexpected(Error{depth.error().code, where + "collider: " + depth.error().context});
        }
        // A scene must be expressible as its own file (SCN-002), or a save and
        // reload composes to another config_hash.
        if (Result<void> r = yaml_text::check_sdf_representable(collider, where + "collider SDF",
                                                                schema(kSceneFileOldestVersion));
            !r) {
            return r;
        }
        if (!a.collider_materials.empty() && a.collider_materials.size() != collider.nodes.size()) {
            return std::unexpected(invalid(where + "collider_materials must be empty or name one material per "
                                                   "collider node (" +
                                           dec(collider.nodes.size()) + " nodes, " +
                                           dec(a.collider_materials.size()) + " names)"));
        }
        for (std::size_t k = 0; k < a.collider_materials.size(); ++k) {
            if (a.collider_materials[k].empty()) {
                return std::unexpected(invalid(where + "node material " + dec(k) + " is an empty name"));
            }
        }

        const bool has_mesh = !a.visual.mesh_ref.empty();
        if (has_mesh != !a.visual.material.empty()) {
            return std::unexpected(
                invalid(where + "a visual needs both a mesh_ref and a material name, or neither"));
        }
        if (collider.nodes.empty() && !has_mesh) {
            return std::unexpected(invalid(where + "has neither a collider nor a visual"));
        }
    }

    // --- vehicles ----------------------------------------------------------
    for (std::size_t i = 0; i < scene.vehicles.size(); ++i) {
        const SceneVehicle& v = scene.vehicles[i];
        if (v.name.empty()) {
            return std::unexpected(invalid("scene vehicle " + dec(i) + " has no name"));
        }
        if (Result<void> r = claim(v.name); !r) {
            return r;
        }
        const std::string where = "vehicle '" + v.name + "': ";

        bool declared = false;
        for (const vehicles::ModelType& m : scene.models) {
            declared = declared || m.name == v.model;
        }
        if (!declared) {
            return std::unexpected(
                invalid(where + "unknown model '" + v.model + "'; the scene declares no model of that name"));
        }

        const VehicleSpawn& s = v.start;
        if (!finite(s.pos) || !finite(s.orient) || !finite(s.vel) || !finite(s.omega_body)) {
            return std::unexpected(invalid(where + "start must be finite"));
        }
        if (!unit(s.orient)) {
            return std::unexpected(invalid(where + "start orientation must be a unit quaternion"));
        }
        if (!finite(s.rotor_omega) || !(s.rotor_omega >= 0.0f)) {
            return std::unexpected(invalid(where + "start rotor_omega must be finite and >= 0"));
        }
    }
    return {};
}

Result<SceneDesc> scene_from_yaml(std::string_view yaml_text, const std::filesystem::path& base_dir) {
    Result<SceneDesc> scene = parse_text(yaml_text, base_dir);
    if (!scene) return scene;
    if (Result<void> r = validate_scene(*scene); !r) {
        return std::unexpected(r.error());
    }
    return scene;
}

Result<std::string> scene_to_yaml(const SceneDesc& scene) {
    if (Result<void> r = validate_scene(scene); !r) {
        return std::unexpected(r.error());
    }
    // The text names a mesh part's sidecar and never holds the mesh, so a
    // mesh part needs its name here (CORE-6). save_mesh_file() writes it.
    for (const vehicles::ModelType& m : scene.models) {
        if (Result<void> r = detail::check_model_writable(m, "scene"); !r) {
            return std::unexpected(r.error());
        }
    }
    try {
        return emit(scene);
    } catch (const std::exception& e) {
        return std::unexpected(Error{Code::internal, std::string("scene file emission failed: ") + e.what()});
    }
}

Result<SceneDesc> load_scene_file(const std::filesystem::path& path) {
    std::string text;
    try {
        std::ifstream in(path, std::ios::binary);
        if (!in) {
            return std::unexpected(Error{Code::io_error, "cannot open scene file '" + path_text(path) + "'"});
        }
        text.assign(std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>());
        if (in.bad()) {
            return std::unexpected(Error{Code::io_error, "failed reading scene file '" + path_text(path) + "'"});
        }
    } catch (const std::exception& e) {
        return std::unexpected(
            Error{Code::io_error, "failed reading scene file '" + path_text(path) + "': " + e.what()});
    }

    // A mesh part's sidecar is relative to the scene file's directory, "."
    // for a bare file name.
    const std::filesystem::path base_dir = path.has_parent_path() ? path.parent_path() : std::filesystem::path(".");
    Result<SceneDesc> scene = scene_from_yaml(text, base_dir);
    if (!scene) {
        return std::unexpected(Error{scene.error().code, path_text(path) + ": " + scene.error().context});
    }
    return scene;
}

Result<uint64_t> world_hash(const WorldDesc& world) {
    const Result<std::string> text = world_to_yaml(world);
    if (!text) return std::unexpected(text.error());
    return rng::fnv1a64(*text);
}

}  // namespace spade::scene
