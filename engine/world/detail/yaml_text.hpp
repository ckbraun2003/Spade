#pragma once

// ---------------------------------------------------------------------------
// world/detail/yaml_text.hpp -- the YAML text every Spade text format shares:
// the world file (world/world_file.cpp), the scene file (scene/scene_file.cpp)
// and the test-support scenario reader (testing/scenario_file.hpp).
//
// Two layers:
//   - the primitives: locale-free numbers, quoted strings, mappings with every
//     key accounted for, and the dotted-path diagnostics;
//   - the world file's encodings of SDF transforms, SDF nodes and materials,
//     which the scene file's asset colliders and materials reuse. One wire
//     encoding, written in one place.
//
// Promoted out of world_file.cpp's anonymous namespace when the scene file
// became the third parser: the moment scenario_file.hpp's own note named for
// it (Core's drone-builder plan, Task D).
//
// INTERNAL, NOT INSTALLED. It names yaml-cpp types, and yaml-cpp is a private
// dependency that no installed header may expose (engine/CMakeLists.txt's
// yaml-cpp note); the install rule for world/ excludes detail/. Include it
// only from a .cpp, or from test support that is never installed.
//
// ===========================================================================
// TEXT <-> NUMBER, AND WHY NEITHER DIRECTION TOUCHES A LOCALE
//
// The world file's whole value as an engine artifact rests on a float
// surviving the trip to text and back UNCHANGED (world_file.hpp, property 1).
// Two things can break that, and both are avoided here rather than hoped
// against:
//
//   PRECISION. 9 significant digits is exactly FLT_DECIMAL_DIG: the smallest
//   number of decimal digits that distinguishes every binary32 from every
//   other. 8 digits silently merges neighbours; more digits is noise.
//
//   LOCALE. This is the subtle one. yaml-cpp's own as<float>() runs the text
//   through an iostream, which is imbued with the global locale, so a host
//   process that has ever called std::locale::global(std::locale("de_DE")) --
//   an editor, a Python binding, a Qt application -- makes "0.1" parse as 0.
//   The C library's strtof/snprintf have the same defect through LC_NUMERIC.
//   std::to_chars and std::from_chars are the two conversions the standard
//   defines with NO locale dependence at all: [charconv] specifies from_chars'
//   grammar as strtod's "in the "C" locale", which is precisely the global
//   constraint's "parsed with strtof under the classic locale" -- spelled in
//   the one form no other code in the process can defeat. tests/
//   test_world_file.cpp pins this with a round trip taken under a
//   comma-decimal global locale.
//
// SPELLING, AND THE HONEST STATUS OF THE NORMALIZATION BELOW. The whole
// emitted form is in fact standard-pinned: [charconv] defines to_chars'
// `general` format as printf's %g, and C's %g inherits %e's mandate that the
// exponent carry a sign and at least two digits. So two conforming
// implementations already owe each other the same bytes, digits and
// punctuation alike, and format_float()'s exponent normalization is NOT
// load-bearing for portability.
//
// It is kept anyway, as insurance with a cost of nothing: this file's output
// is committed to tests/golden/worlds/*.world.yaml and compared BYTE FOR BYTE
// by CI on a different compiler and libc than the one that wrote it, so the
// failure mode of a standard-library bug here is a red build on an unrelated
// change rather than anything a reader could diagnose. Normalizing costs one
// pass over a 15-byte string and removes the whole class.
// ===========================================================================

#include <algorithm>
#include <array>
#include <cassert>
#include <charconv>
#include <concepts>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <initializer_list>
#include <iterator>
#include <span>
#include <string>
#include <string_view>
#include <system_error>
#include <utility>
#include <vector>

#include <glm/gtc/quaternion.hpp>
#include <glm/vec3.hpp>
#include <glm/vec4.hpp>
#include <yaml-cpp/yaml.h>

#include "core/error.hpp"
#include "core/validate.hpp"
#include "world/builder.hpp"  // MaterialDesc, MaterialShading
#include "world/sdf.hpp"      // SdfTransform, SdfNode, SdfPrim, SdfOp

namespace spade::yaml_text {

// ===========================================================================
// Emission
// ===========================================================================

template <std::integral T>
[[nodiscard]] inline std::string dec(T value) {
    char buf[24];
    const std::to_chars_result r = std::to_chars(buf, buf + sizeof(buf), value, 10);
    assert(r.ec == std::errc{} && "24 bytes holds any 64-bit integer in base 10");
    return std::string(buf, r.ptr);
}

[[nodiscard]] inline std::string format_float(float value) {
    char buf[40];
    const std::to_chars_result r =
        std::to_chars(buf, buf + sizeof(buf), value, std::chars_format::general, 9);
    // The longest general/9 form of a binary32 is "-1.23456789e-45" (15
    // bytes), so overflowing 40 is impossible; this is an invariant, not a
    // case to handle.
    assert(r.ec == std::errc{} && "40 bytes holds any %.9g of a binary32");

    const std::string_view text(buf, static_cast<std::size_t>(r.ptr - buf));
    const std::size_t e = text.find('e');
    if (e == std::string_view::npos) {
        return std::string(text);
    }

    std::string out(text.substr(0, e));
    out += 'e';
    std::string_view exponent = text.substr(e + 1);
    char sign = '+';
    if (!exponent.empty() && (exponent.front() == '+' || exponent.front() == '-')) {
        sign = exponent.front();
        exponent.remove_prefix(1);
    }
    while (exponent.size() > 1 && exponent.front() == '0') {
        exponent.remove_prefix(1);
    }
    out += sign;
    if (exponent.size() < 2) {
        out += '0';
    }
    out.append(exponent);
    return out;
}

// A YAML double-quoted scalar. Double-quoted (rather than plain or single-
// quoted) unconditionally, because it is the only style that can carry every
// string a name might hold -- including one that starts with '*', looks like a
// number, or contains a control byte -- under one rule instead of a table of
// exceptions.
[[nodiscard]] inline std::string quote_yaml(std::string_view text) {
    static constexpr char kHex[] = "0123456789abcdef";
    std::string out;
    out.reserve(text.size() + 2);
    out += '"';
    for (const char c : text) {
        const unsigned char u = static_cast<unsigned char>(c);
        switch (c) {
            case '"': out += "\\\""; break;
            case '\\': out += "\\\\"; break;
            case '\n': out += "\\n"; break;
            case '\r': out += "\\r"; break;
            case '\t': out += "\\t"; break;
            default:
                if (u < 0x20 || u == 0x7f) {
                    out += "\\x";
                    out += kHex[u >> 4];
                    out += kHex[u & 0x0f];
                } else {
                    // Printable ASCII, and UTF-8 continuation bytes verbatim.
                    out += c;
                }
                break;
        }
    }
    out += '"';
    return out;
}

[[nodiscard]] inline std::string float_list(std::span<const float> values) {
    std::string out = "[";
    for (std::size_t i = 0; i < values.size(); ++i) {
        if (i != 0) {
            out += ", ";
        }
        out += format_float(values[i]);
    }
    out += ']';
    return out;
}

[[nodiscard]] inline std::string float_list(std::initializer_list<float> values) {
    return float_list(std::span<const float>(values.begin(), values.size()));
}

// std::filesystem::path::string() can throw on a path the native narrow
// encoding cannot represent, and nothing may be thrown out of a Spade API.
[[nodiscard]] inline std::string path_text(const std::filesystem::path& path) noexcept {
    try {
        return path.string();
    } catch (...) {
        return "<unprintable path>";
    }
}

// ===========================================================================
// Parsing
// ===========================================================================

[[nodiscard]] inline std::string mark_of(const YAML::Node& node) {
    const YAML::Mark mark = node.Mark();
    if (mark.is_null()) {
        return {};
    }
    return " (line " + dec(mark.line + 1) + ", column " + dec(mark.column + 1) + ")";
}

[[nodiscard]] inline Error at(const YAML::Node& node, std::string message) {
    return Error{Code::invalid_argument, std::move(message) + mark_of(node)};
}

// A mapping, with EVERY key accounted for. Unknown keys and duplicate keys are
// both rejected here: the first would let a later version's field pass
// unnoticed through an older reader, and the second is a silent last-one-wins
// in every YAML library there is.
//
// `schema` names the schema the unknown-key diagnostic blames ("schema v2",
// "scenario schema v1", "scene schema v1"). Each format passes its own, and
// passes the file's OWN declared version wherever a shape's key set is
// version-gated (world_file.cpp's check_map note has the case that taught it).
[[nodiscard]] inline Result<void> check_map(const YAML::Node& node, std::string_view what,
                                            std::initializer_list<std::string_view> allowed,
                                            std::string_view schema) {
    if (!node.IsMap()) {
        return std::unexpected(at(node, std::string(what) + " must be a mapping"));
    }
    std::vector<std::string> seen;
    seen.reserve(allowed.size());
    for (YAML::const_iterator it = node.begin(); it != node.end(); ++it) {
        if (!it->first.IsScalar()) {
            return std::unexpected(at(it->first, "non-scalar key in " + std::string(what)));
        }
        const std::string key = it->first.Scalar();
        if (std::find(allowed.begin(), allowed.end(), std::string_view(key)) == allowed.end()) {
            return std::unexpected(at(it->first, "unknown key '" + key + "' in " + std::string(what) +
                                                     " -- " + std::string(schema) +
                                                     " does not define it"));
        }
        if (std::find(seen.begin(), seen.end(), key) != seen.end()) {
            return std::unexpected(at(it->first, "duplicate key '" + key + "' in " +
                                                     std::string(what)));
        }
        seen.push_back(key);
    }
    return {};
}

[[nodiscard]] inline Result<YAML::Node> field(const YAML::Node& map, const char* key,
                                              std::string_view what) {
    const YAML::Node child = map[key];
    if (!child.IsDefined() || child.IsNull()) {
        return std::unexpected(
            at(map, "missing required key '" + std::string(key) + "' in " + std::string(what)));
    }
    return child;
}

// For the few keys whose absence means "the default", each named in its
// format's schema note where it is read.
[[nodiscard]] inline bool has(const YAML::Node& map, const char* key) {
    const YAML::Node child = map[key];
    return child.IsDefined() && !child.IsNull();
}

[[nodiscard]] inline Result<std::string> scalar(const YAML::Node& node, std::string_view what) {
    if (!node.IsScalar()) {
        return std::unexpected(at(node, std::string(what) + " must be a scalar"));
    }
    return node.Scalar();
}

// See the locale note at the top of this file: node.Scalar() then from_chars,
// never node.as<float>().
[[nodiscard]] inline Result<float> parse_float(const YAML::Node& node, std::string_view what) {
    const Result<std::string> text = scalar(node, what);
    if (!text) {
        return std::unexpected(text.error());
    }
    std::string_view body(*text);
    // from_chars rejects a leading '+'; format_float never writes one, but a
    // hand editor may, and refusing "+1.5" would be pedantry with no payoff.
    if (!body.empty() && body.front() == '+') {
        body.remove_prefix(1);
    }

    float value = 0.0f;
    const std::from_chars_result r =
        std::from_chars(body.data(), body.data() + body.size(), value);
    if (r.ec == std::errc::result_out_of_range) {
        return std::unexpected(at(node, std::string(what) + ": '" + *text +
                                            "' is outside the range of a 32-bit float"));
    }
    if (r.ec != std::errc{} || r.ptr != body.data() + body.size()) {
        return std::unexpected(
            at(node, std::string(what) + ": '" + *text + "' is not a decimal number"));
    }
    // from_chars' general format accepts "inf"/"nan" -- the engine does not.
    if (!finite(value)) {
        return std::unexpected(at(node, std::string(what) + ": '" + *text +
                                            "' is not finite; an infinity or a NaN in a file is "
                                            "always an authoring error"));
    }
    return value;
}

// Decimal always; also `0x`/`0X` hex when `allow_hex`. The prefix decides the
// base, so nothing is ambiguous.
//
// Hex is the scenario file's accommodation, for its `seed` and
// `expected_digest`: a hand-authored file a human reads as bit patterns (a
// digest, a seed transcribed from a debug print), which this engine never
// writes back. The world and scene files are real round-trip formats whose
// writers always spell integers with dec(), so their readers never meet a hex
// literal and keep allow_hex off.
template <std::unsigned_integral T>
[[nodiscard]] inline Result<T> parse_uint(const YAML::Node& node, std::string_view what,
                                          bool allow_hex = false) {
    const Result<std::string> text = scalar(node, what);
    if (!text) {
        return std::unexpected(text.error());
    }
    std::string_view body(*text);
    if (!body.empty() && body.front() == '+') {
        body.remove_prefix(1);
    }
    int base = 10;
    if (allow_hex && body.size() > 2 && body[0] == '0' && (body[1] == 'x' || body[1] == 'X')) {
        base = 16;
        body.remove_prefix(2);
    }

    T value = 0;
    const std::from_chars_result r =
        std::from_chars(body.data(), body.data() + body.size(), value, base);
    if (r.ec == std::errc::result_out_of_range) {
        return std::unexpected(at(node, std::string(what) + ": '" + *text + "' does not fit in " +
                                            dec(sizeof(T) * 8) + " unsigned bits"));
    }
    if (r.ec != std::errc{} || r.ptr != body.data() + body.size()) {
        return std::unexpected(at(node, std::string(what) + ": '" + *text +
                                            (allow_hex
                                                 ? "' is not a non-negative decimal or 0x-hex integer"
                                                 : "' is not a non-negative decimal integer")));
    }
    return value;
}

template <std::size_t N>
[[nodiscard]] inline Result<std::array<float, N>> parse_floats(const YAML::Node& node,
                                                               std::string_view what) {
    if (!node.IsSequence()) {
        return std::unexpected(at(node, std::string(what) + " must be a sequence of " + dec(N) +
                                            " numbers"));
    }
    if (node.size() != N) {
        return std::unexpected(at(node, std::string(what) + " must hold exactly " + dec(N) +
                                            " numbers, found " + dec(node.size())));
    }
    std::array<float, N> out{};
    for (std::size_t i = 0; i < N; ++i) {
        const Result<float> value = parse_float(node[i], std::string(what) + "[" + dec(i) + "]");
        if (!value) {
            return std::unexpected(value.error());
        }
        out[i] = *value;
    }
    return out;
}

// Composed field readers -- `what` is the containing mapping's dotted path, so
// every diagnostic reads like "world.sdf.nodes[3].params[2]: ...".
[[nodiscard]] inline Result<float> float_field(const YAML::Node& map, const char* key,
                                               std::string_view what) {
    const Result<YAML::Node> node = field(map, key, what);
    if (!node) {
        return std::unexpected(node.error());
    }
    return parse_float(*node, std::string(what) + "." + key);
}

template <std::unsigned_integral T>
[[nodiscard]] inline Result<T> uint_field(const YAML::Node& map, const char* key,
                                          std::string_view what, bool allow_hex = false) {
    const Result<YAML::Node> node = field(map, key, what);
    if (!node) {
        return std::unexpected(node.error());
    }
    return parse_uint<T>(*node, std::string(what) + "." + key, allow_hex);
}

template <std::size_t N>
[[nodiscard]] inline Result<std::array<float, N>> floats_field(const YAML::Node& map,
                                                               const char* key,
                                                               std::string_view what) {
    const Result<YAML::Node> node = field(map, key, what);
    if (!node) {
        return std::unexpected(node.error());
    }
    return parse_floats<N>(*node, std::string(what) + "." + key);
}

[[nodiscard]] inline Result<std::string> string_field(const YAML::Node& map, const char* key,
                                                      std::string_view what) {
    const Result<YAML::Node> node = field(map, key, what);
    if (!node) {
        return std::unexpected(node.error());
    }
    return scalar(*node, std::string(what) + "." + key);
}

[[nodiscard]] inline Result<glm::vec3> vec3_field(const YAML::Node& map, const char* key,
                                                  std::string_view what) {
    const Result<std::array<float, 3>> v = floats_field<3>(map, key, what);
    if (!v) {
        return std::unexpected(v.error());
    }
    return glm::vec3((*v)[0], (*v)[1], (*v)[2]);
}

// [w, x, y, z] in the file, which is glm::quat's constructor order. NOT
// re-normalized here: whoever consumes the rotation normalizes it, and
// silently fixing a file would hide an authoring error the engine is willing
// to report -- and would perturb a loaded value's last bit, breaking its
// round trip.
[[nodiscard]] inline Result<glm::quat> quat_field(const YAML::Node& map, const char* key,
                                                  std::string_view what) {
    const Result<std::array<float, 4>> q = floats_field<4>(map, key, what);
    if (!q) {
        return std::unexpected(q.error());
    }
    return glm::quat((*q)[0], (*q)[1], (*q)[2], (*q)[3]);
}

// ===========================================================================
// The world file's encodings, shared with the scene file
//
// Enum names. The FILE CARRIES NAMES, never the underlying integers: an
// integer in a text format is a promise that the enumeration never gets
// reordered, and no enumeration keeps that promise forever. Index into these
// tables IS the enumerator value, and the static_asserts below are what keeps
// the two in step.
// ===========================================================================

inline constexpr std::string_view kPrimNames[] = {"plane",   "sphere", "box",        "cylinder",
                                                  "capsule", "torus",  "heightfield"};
static_assert(std::size(kPrimNames) == kSdfPrimCount, "one name per SdfPrim");

// `none` is present so the table is index-aligned with SdfOp, but it is NOT a
// legal value in a file: a node is a primitive because it carries `prim`, not
// because it carries `op: none`.
inline constexpr std::string_view kOpNames[] = {"none", "union", "intersect", "subtract",
                                                "smooth_union"};
static_assert(std::size(kOpNames) == kSdfOpCount, "one name per SdfOp");

// World schema v2. Index into this table IS the MaterialShading enumerator value.
inline constexpr std::string_view kShadingNames[] = {"lambert", "unlit", "emissive"};
static_assert(std::size(kShadingNames) == kMaterialShadingCount, "one name per MaterialShading");

// One `transforms:` list's entries, each item's dash at `indent`: an index
// comment, then the 16 floats of world_to_local in COLUMN-MAJOR order, four
// per line (one line per basis column, translation last), then the scale.
// The caller writes the `transforms:` key and the empty form.
inline void append_sdf_transforms(std::string& out, std::span<const SdfTransform> transforms,
                                  std::string_view indent) {
    const std::string open = std::string(indent) + "- world_to_local: [";
    const std::string cont(open.size(), ' ');
    for (std::size_t i = 0; i < transforms.size(); ++i) {
        const SdfTransform& t = transforms[i];
        out += std::string(indent) + "# [" + dec(i) + "]\n";
        for (int column = 0; column < 4; ++column) {
            out += (column == 0) ? open : cont;
            for (int row = 0; row < 4; ++row) {
                out += format_float(t.world_to_local[column][row]);
                if (column != 3 || row != 3) {
                    out += ',';
                }
                if (row != 3) {
                    out += ' ';
                }
            }
            out += (column == 3) ? "]\n" : "\n";
        }
        out += std::string(indent) + "  scale: " + format_float(t.scale) + "\n";
    }
}

// One `nodes:` list's entries, one flow mapping per line at `indent`.
inline void append_sdf_nodes(std::string& out, std::span<const SdfNode> nodes,
                             std::string_view indent) {
    for (const SdfNode& n : nodes) {
        const bool is_prim = n.op == static_cast<uint32_t>(SdfOp::none);
        out += std::string(indent) + "- {";
        out += is_prim ? "prim: " : "op: ";
        out += is_prim ? kPrimNames[n.kind] : kOpNames[n.op];
        out += ", transform: " + dec(n.transform);
        out += ", params: " + float_list({n.params.x, n.params.y, n.params.z, n.params.w}) + "}\n";
    }
}

// A material as one flow mapping, without the list dash.
[[nodiscard]] inline std::string material_flow(const MaterialDesc& m) {
    return "{name: " + quote_yaml(m.name) + ", base_color: " +
           float_list({m.base_color.x, m.base_color.y, m.base_color.z, m.base_color.w}) +
           ", shading: " + quote_yaml(kShadingNames[static_cast<uint32_t>(m.shading)]) + "}";
}

[[nodiscard]] inline Result<SdfTransform> parse_sdf_transform(const YAML::Node& node,
                                                              std::string_view what,
                                                              std::string_view schema) {
    if (Result<void> r = check_map(node, what, {"world_to_local", "scale"}, schema); !r) {
        return std::unexpected(r.error());
    }
    const Result<std::array<float, 16>> matrix = floats_field<16>(node, "world_to_local", what);
    if (!matrix) return std::unexpected(matrix.error());
    const Result<float> scale = float_field(node, "scale", what);
    if (!scale) return std::unexpected(scale.error());

    SdfTransform out{};  // zeroes _pad, which the file never carries
    for (int column = 0; column < 4; ++column) {
        for (int row = 0; row < 4; ++row) {
            out.world_to_local[column][row] =
                (*matrix)[static_cast<std::size_t>(column) * 4 + static_cast<std::size_t>(row)];
        }
    }
    out.scale = *scale;
    return out;
}

[[nodiscard]] inline Result<SdfNode> parse_sdf_node(const YAML::Node& node, std::string_view what,
                                                    std::string_view schema) {
    if (Result<void> r = check_map(node, what, {"prim", "op", "transform", "params"}, schema); !r) {
        return std::unexpected(r.error());
    }

    const YAML::Node prim_key = node["prim"];
    const YAML::Node op_key = node["op"];
    const bool has_prim = prim_key.IsDefined() && !prim_key.IsNull();
    const bool has_op = op_key.IsDefined() && !op_key.IsNull();
    if (has_prim == has_op) {
        return std::unexpected(
            at(node, std::string(what) +
                         " must carry exactly one of 'prim' (a primitive leaf) or 'op' (a CSG "
                         "operator)"));
    }

    SdfNode out{};  // zeroes _pad, which the file never carries
    if (has_prim) {
        const Result<std::string> name = scalar(prim_key, std::string(what) + ".prim");
        if (!name) return std::unexpected(name.error());
        const std::string_view* found =
            std::find(std::begin(kPrimNames), std::end(kPrimNames), std::string_view(*name));
        if (found == std::end(kPrimNames)) {
            return std::unexpected(at(prim_key, "unknown SDF primitive '" + *name + "' in " +
                                                    std::string(what)));
        }
        out.kind = static_cast<uint32_t>(found - std::begin(kPrimNames));
        out.op = static_cast<uint32_t>(SdfOp::none);
    } else {
        const Result<std::string> name = scalar(op_key, std::string(what) + ".op");
        if (!name) return std::unexpected(name.error());
        const std::string_view* found =
            std::find(std::begin(kOpNames), std::end(kOpNames), std::string_view(*name));
        if (found == std::end(kOpNames)) {
            return std::unexpected(
                at(op_key, "unknown SDF operator '" + *name + "' in " + std::string(what)));
        }
        const uint32_t op = static_cast<uint32_t>(found - std::begin(kOpNames));
        if (op == static_cast<uint32_t>(SdfOp::none)) {
            return std::unexpected(at(op_key, "'none' is not an operator; a primitive leaf is "
                                              "written with a 'prim' key (" +
                                                  std::string(what) + ")"));
        }
        out.op = op;
        out.kind = 0;
    }

    const Result<uint32_t> transform = uint_field<uint32_t>(node, "transform", what);
    if (!transform) return std::unexpected(transform.error());
    const Result<std::array<float, 4>> params = floats_field<4>(node, "params", what);
    if (!params) return std::unexpected(params.error());

    out.transform = *transform;
    out.params = glm::vec4((*params)[0], (*params)[1], (*params)[2], (*params)[3]);
    return out;
}

[[nodiscard]] inline Result<MaterialDesc> parse_material(const YAML::Node& node,
                                                         std::string_view what,
                                                         std::string_view schema) {
    if (Result<void> r = check_map(node, what, {"name", "base_color", "shading"}, schema); !r) {
        return std::unexpected(r.error());
    }
    const Result<std::string> name = string_field(node, "name", what);
    if (!name) return std::unexpected(name.error());
    const Result<std::array<float, 4>> base_color = floats_field<4>(node, "base_color", what);
    if (!base_color) return std::unexpected(base_color.error());

    const Result<YAML::Node> shading_node = field(node, "shading", what);
    if (!shading_node) return std::unexpected(shading_node.error());
    const Result<std::string> shading_name = scalar(*shading_node, std::string(what) + ".shading");
    if (!shading_name) return std::unexpected(shading_name.error());
    const std::string_view* found = std::find(std::begin(kShadingNames), std::end(kShadingNames),
                                              std::string_view(*shading_name));
    if (found == std::end(kShadingNames)) {
        return std::unexpected(at(*shading_node, "unknown material shading '" + *shading_name +
                                                       "' in " + std::string(what)));
    }

    MaterialDesc out;
    out.name = *name;
    out.base_color =
        glm::vec4((*base_color)[0], (*base_color)[1], (*base_color)[2], (*base_color)[3]);
    out.shading = static_cast<MaterialShading>(found - std::begin(kShadingNames));
    return out;
}

}  // namespace spade::yaml_text
