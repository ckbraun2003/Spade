// The editor's edit text: one line per edit. Display-free (SL15b).
//
// Every scene and world edit has a text form, so the command line and a script
// can make any edit the window can (EDT-002):
//
//   scene set-asset-pose box_1 pos 0 1.79999995 0 rot 1 0 0 0 scale 1
//   world set-capacities bodies 4 force-elements 8 sensors 2 contacts 4
//
// - Tokens are separated by spaces. A name that is empty, starts with '#', or
//   holds a space, a tab, a quote or a backslash is written in double quotes,
//   with \" \\ \n \r \t escapes.
// - Floats are written with 9 significant digits (the scene file's spelling),
//   so they parse back bit for bit. Hashes are written as 0x and 16 hex digits.
// - Fields are positional, behind keywords, in a fixed order. The parser
//   accepts exactly what the writer writes, and an error names the first bad
//   token: "token 7 ('x'): expected a position".
//
// add-asset has a short form for a single primitive ("box 0.5 0.5 0.5") and a
// full form that writes the whole collider. add-vehicle writes the whole model
// through the inspector's table (model_params()). For these two,
// format_edit() parses its own line back and compares the scene file's
// canonical text of the result, so a field the text form does not carry
// is refused rather than silently dropped.
//
// docs/design/interface/plans/2026-10-05-editor-plan.md Task 8.

#pragma once

#include <algorithm>
#include <array>
#include <charconv>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
#include <string>
#include <string_view>
#include <system_error>
#include <type_traits>
#include <utility>
#include <variant>
#include <vector>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include "editor_model_params.hpp"
#include "editor_scene_edits.hpp"
#include "editor_world_edits.hpp"

namespace spade::sandbox::editor {

using AnyEdit = std::variant<SceneEdit, WorldEdit>;

namespace text {

// A list count the text form accepts: a guard against a typo that would
// allocate millions of entries, far above anything a validator accepts.
inline constexpr std::size_t kMaxCount = 65536;

class Writer {
  public:
    Writer& kw(std::string_view keyword) {
        sep();
        out_ += keyword;
        return *this;
    }

    Writer& word(std::string_view w) {
        sep();
        if (!needs_quotes(w)) {
            out_ += w;
            return *this;
        }
        out_ += '"';
        for (const char c : w) {
            switch (c) {
                case '"': out_ += "\\\""; break;
                case '\\': out_ += "\\\\"; break;
                case '\n': out_ += "\\n"; break;
                case '\r': out_ += "\\r"; break;
                case '\t': out_ += "\\t"; break;
                default: out_ += c;
            }
        }
        out_ += '"';
        return *this;
    }

    Writer& f(float v) {
        char buf[32];
        const std::to_chars_result r = std::to_chars(buf, buf + sizeof(buf), v, std::chars_format::general, 9);
        sep();
        out_.append(buf, r.ptr);
        return *this;
    }

    Writer& u(uint64_t v) {
        sep();
        out_ += std::to_string(v);
        return *this;
    }

    Writer& hex(uint64_t v) {
        char buf[16];
        const std::to_chars_result r = std::to_chars(buf, buf + sizeof(buf), v, 16);
        const auto n = static_cast<std::size_t>(r.ptr - buf);
        sep();
        out_ += "0x";
        out_.append(16 - n, '0');
        out_.append(buf, n);
        return *this;
    }

    Writer& v3(const glm::vec3& v) { return f(v.x).f(v.y).f(v.z); }
    Writer& q(const glm::quat& v) { return f(v.w).f(v.x).f(v.y).f(v.z); }
    Writer& v4(const glm::vec4& v) { return f(v.x).f(v.y).f(v.z).f(v.w); }

    [[nodiscard]] std::string take() { return std::move(out_); }

  private:
    [[nodiscard]] static bool needs_quotes(std::string_view w) {
        if (w.empty() || w.front() == '#') return true;
        return std::any_of(w.begin(), w.end(), [](char c) {
            return c == ' ' || c == '\t' || c == '"' || c == '\\' || c == '\n' || c == '\r';
        });
    }
    void sep() {
        if (!out_.empty()) out_ += ' ';
    }

    std::string out_;
};

// Reads a line token by token. The first failure sticks: later reads return
// defaults and change nothing, so a parser reads straight through and checks
// ok() once at the end.
class Reader {
  public:
    explicit Reader(std::string_view line) { tokenize(line); }

    [[nodiscard]] bool ok() const noexcept { return !failed_; }
    [[nodiscard]] const Error& error() const noexcept { return error_; }

    [[nodiscard]] bool next_is(std::string_view keyword) const noexcept {
        return !failed_ && i_ < toks_.size() && !toks_[i_].quoted && toks_[i_].text == keyword;
    }

    void expect(std::string_view keyword) {
        const std::string what = "'" + std::string(keyword) + "'";
        const Token* t = take(what);
        if (t != nullptr && (t->quoted || t->text != keyword)) fail_at(i_ - 1, what);
    }

    [[nodiscard]] std::string word(std::string_view what) {
        const Token* t = take(what);
        return t == nullptr ? std::string() : t->text;
    }

    [[nodiscard]] float f(std::string_view what) {
        const Token* t = take(what);
        float v = 0.0f;
        if (t == nullptr) return v;
        const char* b = t->text.data();
        const char* e = b + t->text.size();
        const std::from_chars_result r = std::from_chars(b, e, v);
        if (t->quoted || r.ec != std::errc() || r.ptr != e) fail_at(i_ - 1, what);
        return v;
    }

    [[nodiscard]] uint64_t u(std::string_view what) {
        const Token* t = take(what);
        uint64_t v = 0;
        if (t == nullptr) return v;
        const char* b = t->text.data();
        const char* e = b + t->text.size();
        const std::from_chars_result r = std::from_chars(b, e, v);
        if (t->quoted || r.ec != std::errc() || r.ptr != e) fail_at(i_ - 1, what);
        return v;
    }

    [[nodiscard]] uint32_t u32(std::string_view what) {
        const uint64_t v = u(what);
        if (ok() && v > std::numeric_limits<uint32_t>::max()) fail_at(i_ - 1, std::string(what) + " under 2^32");
        return static_cast<uint32_t>(v);
    }

    [[nodiscard]] std::size_t count(std::string_view what) {
        const uint64_t v = u(what);
        if (ok() && v > kMaxCount) {
            fail_at(i_ - 1, std::string(what) + " of at most " + std::to_string(kMaxCount));
        }
        return static_cast<std::size_t>(v);
    }

    [[nodiscard]] uint64_t hex(std::string_view what) {
        const Token* t = take(what);
        uint64_t v = 0;
        if (t == nullptr) return v;
        const std::string& s = t->text;
        const bool prefixed = s.size() > 2 && s[0] == '0' && (s[1] == 'x' || s[1] == 'X');
        bool parsed = false;
        if (prefixed) {
            const char* e = s.data() + s.size();
            const std::from_chars_result r = std::from_chars(s.data() + 2, e, v, 16);
            parsed = r.ec == std::errc() && r.ptr == e;
        }
        if (t->quoted || !parsed) fail_at(i_ - 1, what);
        return v;
    }

    [[nodiscard]] glm::vec3 v3(std::string_view what) {
        const float x = f(what);
        const float y = f(what);
        const float z = f(what);
        return glm::vec3(x, y, z);
    }

    [[nodiscard]] glm::quat q(std::string_view what) {
        const float w = f(what);
        const float x = f(what);
        const float y = f(what);
        const float z = f(what);
        return glm::quat(w, x, y, z);
    }

    [[nodiscard]] glm::vec4 v4(std::string_view what) {
        const float x = f(what);
        const float y = f(what);
        const float z = f(what);
        const float w = f(what);
        return glm::vec4(x, y, z, w);
    }

    // No tokens may remain.
    void end() {
        if (!failed_ && i_ < toks_.size()) fail_at(i_, "the end of the line");
    }

    // The token just read was well formed but is not one of `expected`.
    void reject(std::string_view expected) {
        if (!failed_ && i_ > 0) fail_at(i_ - 1, expected);
    }

  private:
    struct Token {
        std::string text;
        bool quoted = false;
    };

    void fail_with(std::string message) {
        if (failed_) return;
        failed_ = true;
        error_ = Error{Code::invalid_argument, std::move(message)};
    }

    void fail_at(std::size_t index, std::string_view expected) {
        fail_with("token " + std::to_string(index + 1) + " ('" + toks_[index].text + "'): expected " +
                  std::string(expected));
    }

    const Token* take(std::string_view what) {
        if (failed_) return nullptr;
        if (i_ >= toks_.size()) {
            fail_with("the end of the line after token " + std::to_string(toks_.size()) + ": expected " +
                      std::string(what));
            return nullptr;
        }
        return &toks_[i_++];
    }

    void tokenize(std::string_view s) {
        const auto space = [](char c) { return c == ' ' || c == '\t' || c == '\r' || c == '\n'; };
        std::size_t i = 0;
        while (i < s.size()) {
            if (space(s[i])) {
                ++i;
                continue;
            }
            Token t;
            if (s[i] != '"') {
                while (i < s.size() && !space(s[i])) t.text += s[i++];
                toks_.push_back(std::move(t));
                continue;
            }
            t.quoted = true;
            const std::size_t start = i++;
            bool closed = false;
            while (i < s.size()) {
                const char c = s[i++];
                if (c == '"') {
                    closed = true;
                    break;
                }
                if (c != '\\') {
                    t.text += c;
                    continue;
                }
                const char e = i < s.size() ? s[i++] : '\0';
                switch (e) {
                    case '"': t.text += '"'; break;
                    case '\\': t.text += '\\'; break;
                    case 'n': t.text += '\n'; break;
                    case 'r': t.text += '\r'; break;
                    case 't': t.text += '\t'; break;
                    default:
                        fail_with("column " + std::to_string(i) + ": unknown escape in a quoted name; the escapes "
                                  "are \\\" \\\\ \\n \\r and \\t");
                        return;
                }
            }
            if (!closed) {
                fail_with("column " + std::to_string(start + 1) + ": unterminated quote; close the name with \"");
                return;
            }
            toks_.push_back(std::move(t));
        }
    }

    std::vector<Token> toks_;
    std::size_t i_ = 0;
    bool failed_ = false;
    Error error_{Code::invalid_argument, std::string()};
};

using Element = vehicles::ModelIssue::Element;

// ---- the pieces several edits share ----

inline void put(Writer& w, const SdfPose& p) {
    w.kw("pos").v3(p.position).kw("rot").q(p.rotation).kw("scale").f(p.scale);
}

[[nodiscard]] inline SdfPose get_pose(Reader& r) {
    SdfPose p;
    r.expect("pos");
    p.position = r.v3("a position (x y z)");
    r.expect("rot");
    p.rotation = r.q("a rotation (w x y z)");
    r.expect("scale");
    p.scale = r.f("a scale");
    return p;
}

inline void put(Writer& w, const VehicleSpawn& s) {
    w.kw("pos").v3(s.pos).kw("rot").q(s.orient).kw("vel").v3(s.vel).kw("omega").v3(s.omega_body);
    w.kw("rotor-omega").f(s.rotor_omega);
}

[[nodiscard]] inline VehicleSpawn get_start(Reader& r) {
    VehicleSpawn s;
    r.expect("pos");
    s.pos = r.v3("a position (x y z)");
    r.expect("rot");
    s.orient = r.q("an orientation (w x y z)");
    r.expect("vel");
    s.vel = r.v3("a velocity (x y z)");
    r.expect("omega");
    s.omega_body = r.v3("a body rate (x y z)");
    r.expect("rotor-omega");
    s.rotor_omega = r.f("a rotor speed");
    return s;
}

inline constexpr std::array<std::pair<std::string_view, MaterialShading>, 3> kShadings = {{
    {"lambert", MaterialShading::lambert},
    {"unlit", MaterialShading::unlit},
    {"emissive", MaterialShading::emissive},
}};

inline void put(Writer& w, const MaterialDesc& m) {
    w.word(m.name).kw("color").v4(m.base_color).kw("shading");
    for (const auto& [name, shading] : kShadings) {
        if (shading == m.shading) {
            w.kw(name);
            return;
        }
    }
    w.u(static_cast<uint32_t>(m.shading));  // not a shading the file knows; the validator will say so
}

[[nodiscard]] inline MaterialDesc get_material(Reader& r) {
    MaterialDesc m;
    m.name = r.word("a material name");
    r.expect("color");
    m.base_color = r.v4("a colour (r g b a)");
    r.expect("shading");
    for (const auto& [name, shading] : kShadings) {
        if (r.next_is(name)) {
            (void)r.word("a shading");
            m.shading = shading;
            return m;
        }
    }
    m.shading = static_cast<MaterialShading>(r.u32("a shading (lambert, unlit or emissive)"));
    return m;
}

inline void put(Writer& w, const SpawnPoint& s) {
    w.word(s.name).kw("pos").v3(s.position).kw("rot").q(s.orientation);
}

[[nodiscard]] inline SpawnPoint get_spawn(Reader& r) {
    SpawnPoint s;
    s.name = r.word("a spawn point name");
    r.expect("pos");
    s.position = r.v3("a position (x y z)");
    r.expect("rot");
    s.orientation = r.q("an orientation (w x y z)");
    return s;
}

inline void put(Writer& w, const PropDesc& p) {
    w.word(p.mesh_ref);
    put(w, p.pose);
    w.kw("material").u(p.material);
}

[[nodiscard]] inline PropDesc get_prop(Reader& r) {
    PropDesc p;
    p.mesh_ref = r.word("a mesh reference");
    p.pose = get_pose(r);
    r.expect("material");
    p.material = r.u32("a material index");
    return p;
}

inline constexpr std::array<std::pair<std::string_view, Element>, 4> kElements = {{
    {"model", Element::model},
    {"rotor", Element::rotor},
    {"drag-body", Element::drag_body},
    {"imu-mount", Element::imu_mount},
}};

[[nodiscard]] inline std::string_view element_word(Element e) {
    for (const auto& [word, element] : kElements) {
        if (element == e) return word;
    }
    return "model";
}

[[nodiscard]] inline Element get_element(Reader& r) {
    const std::string w = r.word("a part");
    for (const auto& [word, element] : kElements) {
        if (w == word) return element;
    }
    r.reject("a part (model, rotor, drag-body or imu-mount)");
    return Element::model;
}

[[nodiscard]] inline const ModelParam* find_param(Element e, std::string_view field) {
    for (const ModelParam& p : model_params()) {
        if (p.element == e && p.field == field) return &p;
    }
    return nullptr;
}

// A ParamValue's alternatives are in ParamKind's order, so a value's index is its kind.
inline void put_value(Writer& w, const ParamValue& v) {
    if (const float* x = std::get_if<float>(&v)) w.f(*x);
    if (const uint32_t* x = std::get_if<uint32_t>(&v)) w.u(*x);
    if (const glm::vec3* x = std::get_if<glm::vec3>(&v)) w.v3(*x);
    if (const glm::quat* x = std::get_if<glm::quat>(&v)) w.q(*x);
    if (const std::string* x = std::get_if<std::string>(&v)) w.word(*x);
}

[[nodiscard]] inline ParamValue get_value(Reader& r, ParamKind kind, std::string_view field) {
    const std::string what = "a value for " + std::string(field);
    switch (kind) {
        case ParamKind::scalar: return r.f(what);
        case ParamKind::count: return r.u32(what);
        case ParamKind::vec3: return r.v3(what);
        case ParamKind::quat: return r.q(what);
        case ParamKind::text: return r.word(what);
    }
    return 0.0f;
}

// ---- add-asset ----

struct Primitive {
    std::string_view word;
    SdfPrim kind;
    int params;
};

inline constexpr std::array<Primitive, 5> kPrimitives = {{
    {"sphere", SdfPrim::sphere, 1},
    {"box", SdfPrim::box, 3},
    {"cylinder", SdfPrim::cylinder, 2},
    {"capsule", SdfPrim::capsule, 2},
    {"torus", SdfPrim::torus, 2},
}};

// One primitive at its collider's origin, as WorldBuilder lays out its parameters.
[[nodiscard]] inline scene::SceneAsset primitive_asset(std::string name, SdfPrim kind, glm::vec4 params,
                                                       const SdfPose& pose) {
    scene::SceneAsset a;
    a.name = std::move(name);
    a.pose = pose;
    a.collider.transforms.push_back(SdfTransform{});
    SdfNode n{};
    n.kind = static_cast<uint32_t>(kind);
    n.op = static_cast<uint32_t>(SdfOp::none);
    n.params = params;
    a.collider.nodes.push_back(n);
    return a;
}

// The scene file's canonical text of an asset or a vehicle, alone in a scene;
// none when the scene file would refuse it.
[[nodiscard]] inline std::optional<std::string> canonical_text(const scene::SceneDesc& s) {
    const Result<std::string> t = scene::scene_to_yaml(s);
    if (!t) return std::nullopt;
    return *t;
}

[[nodiscard]] inline std::optional<std::string> asset_text(const scene::SceneAsset& a) {
    scene::SceneDesc s;
    s.name = "text";
    s.world.file = "w.world.yaml";
    s.assets = {a};
    return canonical_text(s);
}

[[nodiscard]] inline std::optional<std::string> vehicle_text(const scene::SceneVehicle& v,
                                                             const vehicles::ModelType& m) {
    scene::SceneDesc s;
    s.name = "text";
    s.world.file = "w.world.yaml";
    s.models = {m};
    s.vehicles = {v};
    s.vehicles[0].model = m.name;
    return canonical_text(s);
}

[[nodiscard]] inline const Primitive* short_form_of(const scene::SceneAsset& a) {
    if (a.collider.nodes.size() != 1) return nullptr;
    const SdfNode& n = a.collider.nodes[0];
    for (const Primitive& p : kPrimitives) {
        if (n.op != static_cast<uint32_t>(SdfOp::none) || n.kind != static_cast<uint32_t>(p.kind)) continue;
        glm::vec4 params(0.0f);
        for (int i = 0; i < p.params; ++i) params[i] = n.params[i];
        const std::optional<std::string> mine = asset_text(a);
        const std::optional<std::string> twin = asset_text(primitive_asset(a.name, p.kind, params, a.pose));
        return mine && twin && *mine == *twin ? &p : nullptr;
    }
    return nullptr;
}

inline Result<void> put_edit(Writer& w, const AddAsset& e) {
    const scene::SceneAsset& a = e.asset;
    w.kw("add-asset").word(a.name);
    if (const Primitive* p = short_form_of(a)) {
        w.kw(p->word);
        for (int i = 0; i < p->params; ++i) w.f(a.collider.nodes[0].params[i]);
        put(w, a.pose);
        return {};
    }
    put(w, a.pose);
    w.kw("transforms").u(a.collider.transforms.size());
    for (const SdfTransform& t : a.collider.transforms) {
        for (int c = 0; c < 4; ++c) {
            for (int row = 0; row < 4; ++row) w.f(t.world_to_local[c][row]);
        }
        w.f(t.scale);
    }
    w.kw("nodes").u(a.collider.nodes.size());
    for (const SdfNode& n : a.collider.nodes) w.u(n.kind).u(n.op).u(n.transform).v4(n.params);
    w.kw("node-materials").u(a.collider.node_materials.size());
    for (const uint32_t m : a.collider.node_materials) w.u(m);
    w.kw("collider-materials").u(a.collider_materials.size());
    for (const std::string& m : a.collider_materials) w.word(m);
    w.kw("visual").word(a.visual.mesh_ref).word(a.visual.material);
    return {};
}

[[nodiscard]] inline AddAsset get_add_asset(Reader& r) {
    std::string name = r.word("an asset name");
    for (const Primitive& p : kPrimitives) {
        if (!r.next_is(p.word)) continue;
        (void)r.word("a primitive");
        glm::vec4 params(0.0f);
        for (int i = 0; i < p.params; ++i) params[i] = r.f("a size");
        const SdfPose pose = get_pose(r);
        return AddAsset{primitive_asset(std::move(name), p.kind, params, pose)};
    }
    AddAsset e;
    e.asset.name = std::move(name);
    e.asset.pose = get_pose(r);
    r.expect("transforms");
    const std::size_t transforms = r.count("a transform count");
    for (std::size_t i = 0; r.ok() && i < transforms; ++i) {
        SdfTransform t;
        for (int c = 0; c < 4; ++c) {
            for (int row = 0; row < 4; ++row) t.world_to_local[c][row] = r.f("a matrix entry");
        }
        t.scale = r.f("a transform scale");
        e.asset.collider.transforms.push_back(t);
    }
    r.expect("nodes");
    const std::size_t nodes = r.count("a node count");
    for (std::size_t i = 0; r.ok() && i < nodes; ++i) {
        SdfNode n{};
        n.kind = r.u32("a node kind");
        n.op = r.u32("a node operator");
        n.transform = r.u32("a node transform index");
        n.params = r.v4("node parameters");
        e.asset.collider.nodes.push_back(n);
    }
    r.expect("node-materials");
    const std::size_t indices = r.count("a node-material count");
    for (std::size_t i = 0; r.ok() && i < indices; ++i) e.asset.collider.node_materials.push_back(r.u32("an index"));
    r.expect("collider-materials");
    const std::size_t names = r.count("a collider-material count");
    for (std::size_t i = 0; r.ok() && i < names; ++i) e.asset.collider_materials.push_back(r.word("a material name"));
    r.expect("visual");
    e.asset.visual.mesh_ref = r.word("a mesh reference");
    e.asset.visual.material = r.word("a material name");
    return e;
}

// ---- add-vehicle ----

inline Result<void> put_edit(Writer& w, const AddVehicle& e) {
    const vehicles::ModelType& m = e.model;
    w.kw("add-vehicle").word(e.vehicle.name);
    put(w, e.vehicle.start);
    w.kw("model").word(m.name).kw("parts").u(m.rotors.size()).u(m.drag_bodies.size()).u(m.imu_mounts.size());
    const auto rows = [&](Element element, std::size_t index) -> Result<void> {
        for (const ModelParam& p : model_params()) {
            if (p.element != element) continue;
            const Result<ParamValue> v = get_model_param(m, element, index, p.field);
            if (!v) return std::unexpected(v.error());
            w.kw(p.field);
            put_value(w, *v);
        }
        return {};
    };
    if (Result<void> r = rows(Element::model, 0); !r) return r;
    for (std::size_t i = 0; i < m.rotors.size(); ++i) {
        w.kw("rotor");
        if (Result<void> r = rows(Element::rotor, i); !r) return r;
    }
    for (std::size_t i = 0; i < m.drag_bodies.size(); ++i) {
        w.kw("drag-body");
        if (Result<void> r = rows(Element::drag_body, i); !r) return r;
    }
    for (std::size_t i = 0; i < m.imu_mounts.size(); ++i) {
        w.kw("imu-mount");
        if (Result<void> r = rows(Element::imu_mount, i); !r) return r;
    }
    return {};
}

[[nodiscard]] inline AddVehicle get_add_vehicle(Reader& r) {
    AddVehicle e;
    e.vehicle.name = r.word("a vehicle name");
    e.vehicle.start = get_start(r);
    r.expect("model");
    e.model.name = r.word("a model name");
    e.vehicle.model = e.model.name;
    r.expect("parts");
    const std::size_t rotors = r.count("a rotor count");
    const std::size_t drag = r.count("a drag body count");
    const std::size_t imu = r.count("an IMU mount count");
    if (!r.ok()) return e;
    e.model.rotors.resize(rotors);
    e.model.drag_bodies.resize(drag);
    e.model.imu_mounts.resize(imu);
    const auto rows = [&](Element element, std::size_t index) {
        for (const ModelParam& p : model_params()) {
            if (p.element != element) continue;
            r.expect(p.field);
            const ParamValue v = get_value(r, p.kind, p.field);
            if (r.ok()) (void)set_model_param(e.model, element, index, p.field, v);
        }
    };
    rows(Element::model, 0);
    for (std::size_t i = 0; r.ok() && i < rotors; ++i) {
        r.expect("rotor");
        rows(Element::rotor, i);
    }
    for (std::size_t i = 0; r.ok() && i < drag; ++i) {
        r.expect("drag-body");
        rows(Element::drag_body, i);
    }
    for (std::size_t i = 0; r.ok() && i < imu; ++i) {
        r.expect("imu-mount");
        rows(Element::imu_mount, i);
    }
    return e;
}

// ---- the other scene edits ----

inline Result<void> put_edit(Writer& w, const RemoveObject& e) {
    w.kw("remove").word(e.name);
    return {};
}
inline Result<void> put_edit(Writer& w, const RenameObject& e) {
    w.kw("rename").word(e.from).word(e.to);
    return {};
}
inline Result<void> put_edit(Writer& w, const DuplicateObject& e) {
    w.kw("duplicate").word(e.name);
    return {};
}
inline Result<void> put_edit(Writer& w, const MoveObject& e) {
    w.kw("move").word(e.name).u(e.to);
    return {};
}
inline Result<void> put_edit(Writer& w, const SetAssetPose& e) {
    w.kw("set-asset-pose").word(e.asset);
    put(w, e.pose);
    return {};
}
inline Result<void> put_edit(Writer& w, const SetAssetMaterial& e) {
    w.kw("set-asset-material").word(e.asset).word(e.material);
    return {};
}
inline Result<void> put_edit(Writer& w, const SetVehicleStart& e) {
    w.kw("set-vehicle-start").word(e.vehicle);
    put(w, e.start);
    return {};
}
inline Result<void> put_edit(Writer& w, const AddMaterial& e) {
    w.kw("add-material");
    put(w, e.material);
    return {};
}
inline Result<void> put_edit(Writer& w, const RepointWorld& e) {
    w.kw("repoint-world").word(e.file).hex(e.hash);
    return {};
}
inline Result<void> put_edit(Writer& w, const SetModelParam& e) {
    const ModelParam* p = find_param(e.element, e.field);
    if (p == nullptr) {
        return std::unexpected(Error{Code::invalid_argument, "set-model-param: " + part_name(e.element, e.index) +
                                                                 " has no parameter '" + e.field +
                                                                 "', so the edit has no text form"});
    }
    if (e.value.index() != static_cast<std::size_t>(p->kind)) {
        return std::unexpected(Error{Code::invalid_argument, "set-model-param: the value for " + e.field +
                                                                 " is of another kind than the parameter, so the "
                                                                 "edit has no text form"});
    }
    w.kw("set-model-param").word(e.vehicle).kw(element_word(e.element)).u(e.index).kw(e.field);
    put_value(w, e.value);
    return {};
}

// ---- the world edits ----

inline Result<void> put_edit(Writer& w, const RenameWorld& e) {
    w.kw("rename").word(e.name);
    return {};
}
inline Result<void> put_edit(Writer& w, const SetEnvironment& e) {
    const Environment& v = e.environment;
    w.kw("set-environment").kw("gravity").v3(v.gravity).kw("wind").v3(v.wind);
    w.kw("air-density").f(v.air_density).kw("temperature").f(v.temperature_k).kw("seed").u(v.seed);
    return {};
}
inline Result<void> put_edit(Writer& w, const SetCapacities& e) {
    const Capacities& c = e.capacities;
    w.kw("set-capacities").kw("bodies").u(c.bodies).kw("force-elements").u(c.force_elements);
    w.kw("sensors").u(c.sensors).kw("contacts").u(c.contacts);
    return {};
}
inline Result<void> put_edit(Writer& w, const AddWorldMaterial& e) {
    w.kw("add-material");
    put(w, e.material);
    return {};
}
inline Result<void> put_edit(Writer& w, const SetWorldMaterial& e) {
    w.kw("set-material").u(e.index);
    put(w, e.material);
    return {};
}
inline Result<void> put_edit(Writer& w, const SetLighting& e) {
    const LightingDesc& l = e.lighting;
    w.kw("set-lighting").kw("sun-direction").v3(l.sun_direction).kw("sun-color").v3(l.sun_color);
    w.kw("sun-intensity").f(l.sun_intensity).kw("ambient").v3(l.ambient_color);
    w.kw("sky-zenith").v3(l.sky_zenith).kw("sky-horizon").v3(l.sky_horizon);
    return {};
}
inline Result<void> put_edit(Writer& w, const AddSpawn& e) {
    w.kw("add-spawn");
    put(w, e.spawn);
    return {};
}
inline Result<void> put_edit(Writer& w, const SetSpawn& e) {
    w.kw("set-spawn").word(e.name);
    put(w, e.spawn);
    return {};
}
inline Result<void> put_edit(Writer& w, const RemoveSpawn& e) {
    w.kw("remove-spawn").word(e.name);
    return {};
}
inline Result<void> put_edit(Writer& w, const AddProp& e) {
    w.kw("add-prop");
    put(w, e.prop);
    return {};
}
inline Result<void> put_edit(Writer& w, const SetProp& e) {
    w.kw("set-prop").u(e.index);
    put(w, e.prop);
    return {};
}
inline Result<void> put_edit(Writer& w, const RemoveProp& e) {
    w.kw("remove-prop").u(e.index);
    return {};
}

inline constexpr std::string_view kSceneVerbs =
    "a scene edit (add-asset, add-vehicle, remove, rename, duplicate, move, set-asset-pose, set-asset-material, "
    "set-vehicle-start, add-material, repoint-world or set-model-param)";
inline constexpr std::string_view kWorldVerbs =
    "a world edit (rename, set-environment, set-capacities, add-material, set-material, set-lighting, add-spawn, "
    "set-spawn, remove-spawn, add-prop, set-prop or remove-prop)";

}  // namespace text

[[nodiscard]] inline Result<SceneEdit> parse_edit(std::string_view line) {
    text::Reader r(line);
    r.expect("scene");
    const std::string verb = r.word(text::kSceneVerbs);
    SceneEdit out = RemoveObject{};
    if (verb == "add-asset") {
        out = text::get_add_asset(r);
    } else if (verb == "add-vehicle") {
        out = text::get_add_vehicle(r);
    } else if (verb == "remove") {
        out = RemoveObject{r.word("an asset or vehicle name")};
    } else if (verb == "rename") {
        RenameObject e;
        e.from = r.word("an asset or vehicle name");
        e.to = r.word("its new name");
        out = e;
    } else if (verb == "duplicate") {
        out = DuplicateObject{r.word("an asset or vehicle name")};
    } else if (verb == "move") {
        MoveObject e{};
        e.name = r.word("an asset or vehicle name");
        e.to = static_cast<std::size_t>(r.u("a place in its list, from 0"));
        out = e;
    } else if (verb == "set-asset-pose") {
        SetAssetPose e;
        e.asset = r.word("an asset name");
        e.pose = text::get_pose(r);
        out = e;
    } else if (verb == "set-asset-material") {
        SetAssetMaterial e;
        e.asset = r.word("an asset name");
        e.material = r.word("a material name");
        out = e;
    } else if (verb == "set-vehicle-start") {
        SetVehicleStart e;
        e.vehicle = r.word("a vehicle name");
        e.start = text::get_start(r);
        out = e;
    } else if (verb == "add-material") {
        out = AddMaterial{text::get_material(r)};
    } else if (verb == "repoint-world") {
        RepointWorld e;
        e.file = r.word("a world file path");
        e.hash = r.hex("a world hash (0x and 16 hex digits)");
        out = e;
    } else if (verb == "set-model-param") {
        SetModelParam e;
        e.vehicle = r.word("a vehicle name");
        e.element = text::get_element(r);
        e.index = static_cast<std::size_t>(r.u("a part index, from 0"));
        e.field = r.word("a parameter name");
        if (const ModelParam* p = text::find_param(e.element, e.field)) {
            e.value = text::get_value(r, p->kind, e.field);
        } else {
            r.reject("a parameter of " + std::string(text::element_word(e.element)) +
                     " in the inspector's table (model_params())");
        }
        out = e;
    } else {
        r.reject(text::kSceneVerbs);
    }
    r.end();
    if (!r.ok()) return std::unexpected(r.error());
    return out;
}

[[nodiscard]] inline Result<WorldEdit> parse_world_edit(std::string_view line) {
    text::Reader r(line);
    r.expect("world");
    const std::string verb = r.word(text::kWorldVerbs);
    WorldEdit out = RemoveProp{};
    if (verb == "rename") {
        out = RenameWorld{r.word("a world name")};
    } else if (verb == "set-environment") {
        Environment v;
        r.expect("gravity");
        v.gravity = r.v3("a gravity vector (x y z)");
        r.expect("wind");
        v.wind = r.v3("a wind vector (x y z)");
        r.expect("air-density");
        v.air_density = r.f("an air density");
        r.expect("temperature");
        v.temperature_k = r.f("a temperature in kelvin");
        r.expect("seed");
        v.seed = r.u("a seed");
        out = SetEnvironment{v};
    } else if (verb == "set-capacities") {
        Capacities c;
        r.expect("bodies");
        c.bodies = r.u32("a body capacity");
        r.expect("force-elements");
        c.force_elements = r.u32("a force-element capacity");
        r.expect("sensors");
        c.sensors = r.u32("a sensor capacity");
        r.expect("contacts");
        c.contacts = r.u32("a contact capacity");
        out = SetCapacities{c};
    } else if (verb == "add-material") {
        out = AddWorldMaterial{text::get_material(r)};
    } else if (verb == "set-material") {
        SetWorldMaterial e;
        e.index = r.u32("a material index");
        e.material = text::get_material(r);
        out = e;
    } else if (verb == "set-lighting") {
        LightingDesc l;
        r.expect("sun-direction");
        l.sun_direction = r.v3("a sun direction (x y z)");
        r.expect("sun-color");
        l.sun_color = r.v3("a sun colour (r g b)");
        r.expect("sun-intensity");
        l.sun_intensity = r.f("a sun intensity");
        r.expect("ambient");
        l.ambient_color = r.v3("an ambient colour (r g b)");
        r.expect("sky-zenith");
        l.sky_zenith = r.v3("a zenith colour (r g b)");
        r.expect("sky-horizon");
        l.sky_horizon = r.v3("a horizon colour (r g b)");
        out = SetLighting{l};
    } else if (verb == "add-spawn") {
        out = AddSpawn{text::get_spawn(r)};
    } else if (verb == "set-spawn") {
        SetSpawn e;
        e.name = r.word("a spawn point name");
        e.spawn = text::get_spawn(r);
        out = e;
    } else if (verb == "remove-spawn") {
        out = RemoveSpawn{r.word("a spawn point name")};
    } else if (verb == "add-prop") {
        out = AddProp{text::get_prop(r)};
    } else if (verb == "set-prop") {
        SetProp e;
        e.index = static_cast<std::size_t>(r.u("a prop index, from 0"));
        e.prop = text::get_prop(r);
        out = e;
    } else if (verb == "remove-prop") {
        out = RemoveProp{static_cast<std::size_t>(r.u("a prop index, from 0"))};
    } else {
        r.reject(text::kWorldVerbs);
    }
    r.end();
    if (!r.ok()) return std::unexpected(r.error());
    return out;
}

// A line of an edits file: "scene ..." or "world ...".
[[nodiscard]] inline Result<AnyEdit> parse_any_edit(std::string_view line) {
    text::Reader r(line);
    if (r.next_is("scene")) {
        Result<SceneEdit> e = parse_edit(line);
        if (!e) return std::unexpected(e.error());
        return AnyEdit{std::move(*e)};
    }
    if (r.next_is("world")) {
        Result<WorldEdit> e = parse_world_edit(line);
        if (!e) return std::unexpected(e.error());
        return AnyEdit{std::move(*e)};
    }
    (void)r.word("'scene' or 'world'");
    r.reject("'scene' or 'world'");
    return std::unexpected(r.error());
}

// One line, or why the edit has none. For add-asset and add-vehicle the line
// is parsed back and the scene file's canonical text of the result compared,
// so a field the text form does not carry is refused, never dropped.
[[nodiscard]] inline Result<std::string> format_edit(const SceneEdit& edit) {
    text::Writer w;
    w.kw("scene");
    const Result<void> written = std::visit([&](const auto& e) { return text::put_edit(w, e); }, edit);
    if (!written) return std::unexpected(written.error());
    std::string line = w.take();

    const auto lost = [&](std::string_view what) {
        return Error{Code::invalid_argument, std::string(what) +
                                                 " has a field its text form does not carry, so the edit has no "
                                                 "text form; the text form needs extending"};
    };
    if (const AddAsset* a = std::get_if<AddAsset>(&edit)) {
        const Result<SceneEdit> back = parse_edit(line);
        if (!back) return std::unexpected(back.error());
        if (text::asset_text(a->asset) != text::asset_text(std::get<AddAsset>(*back).asset)) {
            return std::unexpected(lost("asset '" + a->asset.name + "'"));
        }
    } else if (const AddVehicle* v = std::get_if<AddVehicle>(&edit)) {
        const Result<SceneEdit> back = parse_edit(line);
        if (!back) return std::unexpected(back.error());
        const AddVehicle& twin = std::get<AddVehicle>(*back);
        if (text::vehicle_text(v->vehicle, v->model) != text::vehicle_text(twin.vehicle, twin.model)) {
            return std::unexpected(lost("vehicle '" + v->vehicle.name + "'"));
        }
    }
    return line;
}

[[nodiscard]] inline Result<std::string> format_world_edit(const WorldEdit& edit) {
    text::Writer w;
    w.kw("world");
    const Result<void> written = std::visit([&](const auto& e) { return text::put_edit(w, e); }, edit);
    if (!written) return std::unexpected(written.error());
    return w.take();
}

}  // namespace spade::sandbox::editor
