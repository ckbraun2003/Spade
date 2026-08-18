#include "world/world_file.hpp"

#include <algorithm>
#include <array>
#include <cassert>
#include <charconv>
#include <concepts>
#include <cstddef>
#include <exception>
#include <fstream>
#include <initializer_list>
#include <iterator>
#include <span>
#include <string>
#include <string_view>
#include <system_error>
#include <utility>
#include <vector>

#include <yaml-cpp/yaml.h>

#include "core/validate.hpp"

namespace spade {
namespace {

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

template <std::integral T>
[[nodiscard]] std::string dec(T value) {
    char buf[24];
    const std::to_chars_result r = std::to_chars(buf, buf + sizeof(buf), value, 10);
    assert(r.ec == std::errc{} && "24 bytes holds any 64-bit integer in base 10");
    return std::string(buf, r.ptr);
}

[[nodiscard]] std::string format_float(float value) {
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

// ===========================================================================
// Enum names. The FILE CARRIES NAMES, never the underlying integers: an
// integer in a text format is a promise that the enumeration never gets
// reordered, and no enumeration keeps that promise forever. Index into these
// tables IS the enumerator value, and the static_asserts below are what keeps
// the two in step.
// ===========================================================================

constexpr std::string_view kPrimNames[] = {"plane", "sphere",    "box",        "cylinder",
                                           "capsule", "torus",   "heightfield"};
static_assert(std::size(kPrimNames) == kSdfPrimCount, "one name per SdfPrim");

// `none` is present so the table is index-aligned with SdfOp, but it is NOT a
// legal value in a file: a node is a primitive because it carries `prim`, not
// because it carries `op: none`.
constexpr std::string_view kOpNames[] = {"none", "union", "intersect", "subtract", "smooth_union"};
static_assert(std::size(kOpNames) == kSdfOpCount, "one name per SdfOp");

// ===========================================================================
// Emission
// ===========================================================================

// A YAML double-quoted scalar. Double-quoted (rather than plain or single-
// quoted) unconditionally, because it is the only style that can carry every
// string a name might hold -- including one that starts with '*', looks like a
// number, or contains a control byte -- under one rule instead of a table of
// exceptions.
[[nodiscard]] std::string quote_yaml(std::string_view text) {
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

[[nodiscard]] std::string float_list(std::span<const float> values) {
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

[[nodiscard]] std::string float_list(std::initializer_list<float> values) {
    return float_list(std::span<const float>(values.begin(), values.size()));
}

// The document preamble. Constant text: canonical emission means the file is a
// pure function of the WorldDesc, and that includes every comment in it.
constexpr std::string_view kFileHeader = R"YAML(# =============================================================================
# Spade world file -- schema version 1.
#
# WHAT THIS IS: the complete, static description of one world -- its analytic
# SDF geometry, its named spawn points, its environment, and its fixed
# capacities. It is exactly what spade::WorldBuilder::build() produces, and
# loading it runs exactly the validation build() runs.
#
# GENERATED by spade::world_to_yaml(). Hand editing is supported: every field
# is validated on load, and re-saving normalizes the file back to this
# canonical form (fixed key order, fixed float spelling, these comments).
#
# UNITS are SI throughout -- metres, kilograms, seconds, kelvin -- in a
# right-handed, Y-up frame.
#
# FLOATS carry 9 significant digits. That is exactly what an IEEE-754 binary32
# needs to survive a decimal round trip without losing a bit, and this engine
# is fp32 end to end: fewer digits would silently change the world.
#
# EVERY KEY BELOW IS REQUIRED, and an UNKNOWN key is an ERROR at every level
# rather than a warning -- a reader of this version must never silently drop a
# field a later version added. `world_version` is the one upgrade path.
# =============================================================================
)YAML";

[[nodiscard]] std::string emit(const WorldDesc& world) {
    std::string out;
    out.reserve(4096);
    out += kFileHeader;

    out += "\nworld_version: ";
    out += dec(kWorldFileVersion);
    out += "\nname: ";
    out += quote_yaml(world.name);
    out += '\n';

    // --- environment -------------------------------------------------------
    const Environment& env = world.environment;
    out +=
        "\n"
        "# The static environment every body in this world sees.\n"
        "#\n"
        "# `seed` is the AUTHORING seed and is carried for the fidelity of this file\n"
        "# alone. Simulation IGNORES it: each world instance takes its rng root from\n"
        "# WorldInstanceDesc::seed at create() time, which is what lets one world file\n"
        "# back a whole fleet of independently-seeded instances.\n"
        "environment:\n";
    {
        // Laid out as a small table so the unit comments line up. The column
        // widths come from the values themselves, so the result is still a
        // pure function of the WorldDesc -- alignment is not a hidden input.
        struct Row {
            std::string key;
            std::string value;
            std::string comment;
        };
        const std::vector<Row> rows = {
            {"gravity", float_list({env.gravity.x, env.gravity.y, env.gravity.z}), "m/s^2"},
            {"wind", float_list({env.wind.x, env.wind.y, env.wind.z}),
             "m/s, the steady component (turbulence is per-instance)"},
            {"air_density", format_float(env.air_density), "kg/m^3"},
            {"temperature_k", format_float(env.temperature_k), "K"},
            {"seed", dec(env.seed), "authoring only -- see above"},
        };
        std::size_t key_width = 0;
        std::size_t value_width = 0;
        for (const Row& row : rows) {
            key_width = std::max(key_width, row.key.size());
            value_width = std::max(value_width, row.value.size());
        }
        for (const Row& row : rows) {
            out += "  " + row.key + ":" + std::string(key_width - row.key.size() + 1, ' ') +
                   row.value + std::string(value_width - row.value.size() + 2, ' ') + "# " +
                   row.comment + "\n";
        }
    }

    // --- capacities --------------------------------------------------------
    const Capacities& caps = world.capacities;
    out +=
        "\n"
        "# Per-world allocation. Decided once, here, and never grown at run time, so\n"
        "# every one of these must be > 0.\n"
        "capacities:\n";
    out += "  bodies: " + dec(caps.bodies) + "\n";
    out += "  force_elements: " + dec(caps.force_elements) + "\n";
    out += "  sensors: " + dec(caps.sensors) + "\n";
    out += "  contacts: " + dec(caps.contacts) + "\n";

    // --- spawns ------------------------------------------------------------
    out +=
        "\n"
        "# Named poses -- how a scenario says \"put the vehicle here\" without\n"
        "# hard-coding coordinates. Names are unique within a world.\n"
        "# `orientation` is a unit quaternion, [w, x, y, z].\n";
    if (world.spawns.empty()) {
        out += "spawns: []\n";
    } else {
        out += "spawns:\n";
        for (const SpawnPoint& s : world.spawns) {
            out += "  - {name: " + quote_yaml(s.name) +
                   ", position: " + float_list({s.position.x, s.position.y, s.position.z}) +
                   ", orientation: " +
                   float_list({s.orientation.w, s.orientation.x, s.orientation.y,
                               s.orientation.z}) +
                   "}\n";
        }
    }

    // --- sdf ---------------------------------------------------------------
    out +=
        "\n"
        "# The analytic signed-distance program, exactly as the builder compiled it:\n"
        "# a FLAT POSTFIX list. A `prim` node pushes one distance; an `op` node pops\n"
        "# two (a = deeper, b = shallower, i.e. \"a b op\") and pushes one. A\n"
        "# well-formed program leaves exactly one value on the stack.\n"
        "sdf:\n";

    out +=
        "  # Node poses, stored PRE-INVERTED (world -> local), because that is the\n"
        "  # direction evaluation needs. 16 floats in COLUMN-MAJOR order, written four\n"
        "  # per line -- one line per basis column, translation last -- followed by the\n"
        "  # uniform scale the local distance is multiplied by. Rigid + uniform scale\n"
        "  # only: a non-uniform scale would destroy the field's metric.\n"
        "  # transforms[0] is the identity, by convention.\n";
    if (world.sdf.transforms.empty()) {
        out += "  transforms: []\n";
    } else {
        out += "  transforms:\n";
        const std::string open = "    - world_to_local: [";
        const std::string cont(open.size(), ' ');
        for (std::size_t i = 0; i < world.sdf.transforms.size(); ++i) {
            const SdfTransform& t = world.sdf.transforms[i];
            out += "    # [" + dec(i) + "]\n";
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
            out += "      scale: " + format_float(t.scale) + "\n";
        }
    }

    out +=
        "  # `params`, by prim (unused lanes are zero):\n"
        "  #   plane        [n.x, n.y, n.z, offset]       unit normal; solid is dot(p,n) <= offset\n"
        "  #   sphere       [radius, 0, 0, 0]\n"
        "  #   box          [hx, hy, hz, 0]               half extents about the local origin\n"
        "  #   cylinder     [radius, half_height, 0, 0]   capped, local +Y axis\n"
        "  #   capsule      [radius, half_height, 0, 0]   segment (0,+-h,0), local +Y\n"
        "  #   torus        [major_r, minor_r, 0, 0]      ring in local XZ, hole axis +Y\n"
        "  #   heightfield  [amplitude, freq_x, freq_z, base_y]\n"
        "  # and by op:\n"
        "  #   smooth_union [k, 0, 0, 0]                  blend radius; k <= 0 degenerates to min\n"
        "  #   union / intersect / subtract  [0, 0, 0, 0]\n"
        "  # `transform` indexes `transforms` above; on an `op` node it is unused.\n";
    if (world.sdf.nodes.empty()) {
        out += "  nodes: []\n";
    } else {
        out += "  nodes:\n";
        for (const SdfNode& n : world.sdf.nodes) {
            const bool is_prim = n.op == static_cast<uint32_t>(SdfOp::none);
            out += "    - {";
            out += is_prim ? "prim: " : "op: ";
            out += is_prim ? kPrimNames[n.kind] : kOpNames[n.op];
            out += ", transform: " + dec(n.transform);
            out += ", params: " +
                   float_list({n.params.x, n.params.y, n.params.z, n.params.w}) + "}\n";
        }
    }

    // --- visuals -----------------------------------------------------------
    out +=
        "\n"
        "# Render-only references (mesh/material ids the presentation layer resolves).\n"
        "# Physics never reads these; the only rule is that each one names something.\n";
    if (world.visual_refs.empty()) {
        out += "visuals: []\n";
    } else {
        out += "visuals:\n";
        for (const std::string& ref : world.visual_refs) {
            out += "  - " + quote_yaml(ref) + "\n";
        }
    }

    return out;
}

// ===========================================================================
// Parsing
// ===========================================================================

[[nodiscard]] std::string mark_of(const YAML::Node& node) {
    const YAML::Mark mark = node.Mark();
    if (mark.is_null()) {
        return {};
    }
    return " (line " + dec(mark.line + 1) + ", column " + dec(mark.column + 1) + ")";
}

[[nodiscard]] Error at(const YAML::Node& node, std::string message) {
    return Error{Code::invalid_argument, std::move(message) + mark_of(node)};
}

// A mapping, with EVERY key accounted for. Unknown keys and duplicate keys are
// both rejected here: the first would let a v2 field pass unnoticed through a
// v1 reader, and the second is a silent last-one-wins in every YAML library
// there is.
[[nodiscard]] Result<void> check_map(const YAML::Node& node, std::string_view what,
                                     std::initializer_list<std::string_view> allowed) {
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
            return std::unexpected(
                at(it->first, "unknown key '" + key + "' in " + std::string(what) +
                                  " -- schema v" + dec(kWorldFileVersion) + " does not define it"));
        }
        if (std::find(seen.begin(), seen.end(), key) != seen.end()) {
            return std::unexpected(at(it->first, "duplicate key '" + key + "' in " +
                                                     std::string(what)));
        }
        seen.push_back(key);
    }
    return {};
}

[[nodiscard]] Result<YAML::Node> field(const YAML::Node& map, const char* key,
                                       std::string_view what) {
    const YAML::Node child = map[key];
    if (!child.IsDefined() || child.IsNull()) {
        return std::unexpected(
            at(map, "missing required key '" + std::string(key) + "' in " + std::string(what)));
    }
    return child;
}

[[nodiscard]] Result<std::string> scalar(const YAML::Node& node, std::string_view what) {
    if (!node.IsScalar()) {
        return std::unexpected(at(node, std::string(what) + " must be a scalar"));
    }
    return node.Scalar();
}

// See the locale note at the top of this file: node.Scalar() then from_chars,
// never node.as<float>().
[[nodiscard]] Result<float> parse_float(const YAML::Node& node, std::string_view what) {
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
                                            "' is not finite; an infinity or a NaN in a world is "
                                            "always an authoring error"));
    }
    return value;
}

// DECIMAL ONLY, DELIBERATELY -- the one place this parser diverges from its
// copy in engine/testing/scenario_file.hpp, which additionally accepts a
// `0x`/`0X` prefix for its `seed` and `expected_digest` fields. Not a gap:
// every uint this file reads (Environment::seed included) is always WRITTEN
// by emit()'s dec() (see "seed" below), never as a hex literal, because
// world_to_yaml() is a real round-trip writer and a scenario file is not (it
// is hand-authored test support that never gets written back out) -- so
// there is no field here a human would ever spell in hex for this reader to
// meet. See scenario_file.hpp's parse_uint for the full account of why the
// two copies differ here and nowhere else.
template <std::unsigned_integral T>
[[nodiscard]] Result<T> parse_uint(const YAML::Node& node, std::string_view what) {
    const Result<std::string> text = scalar(node, what);
    if (!text) {
        return std::unexpected(text.error());
    }
    std::string_view body(*text);
    if (!body.empty() && body.front() == '+') {
        body.remove_prefix(1);
    }

    T value = 0;
    const std::from_chars_result r =
        std::from_chars(body.data(), body.data() + body.size(), value, 10);
    if (r.ec == std::errc::result_out_of_range) {
        return std::unexpected(at(node, std::string(what) + ": '" + *text + "' does not fit in " +
                                            dec(sizeof(T) * 8) + " unsigned bits"));
    }
    if (r.ec != std::errc{} || r.ptr != body.data() + body.size()) {
        return std::unexpected(at(node, std::string(what) + ": '" + *text +
                                            "' is not a non-negative decimal integer"));
    }
    return value;
}

template <std::size_t N>
[[nodiscard]] Result<std::array<float, N>> parse_floats(const YAML::Node& node,
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
        const Result<float> value =
            parse_float(node[i], std::string(what) + "[" + dec(i) + "]");
        if (!value) {
            return std::unexpected(value.error());
        }
        out[i] = *value;
    }
    return out;
}

// Composed field readers -- `what` is the containing mapping's dotted path, so
// every diagnostic reads like "world.sdf.nodes[3].params[2]: ...".
[[nodiscard]] Result<float> float_field(const YAML::Node& map, const char* key,
                                        std::string_view what) {
    const Result<YAML::Node> node = field(map, key, what);
    if (!node) {
        return std::unexpected(node.error());
    }
    return parse_float(*node, std::string(what) + "." + key);
}

template <std::unsigned_integral T>
[[nodiscard]] Result<T> uint_field(const YAML::Node& map, const char* key, std::string_view what) {
    const Result<YAML::Node> node = field(map, key, what);
    if (!node) {
        return std::unexpected(node.error());
    }
    return parse_uint<T>(*node, std::string(what) + "." + key);
}

template <std::size_t N>
[[nodiscard]] Result<std::array<float, N>> floats_field(const YAML::Node& map, const char* key,
                                                        std::string_view what) {
    const Result<YAML::Node> node = field(map, key, what);
    if (!node) {
        return std::unexpected(node.error());
    }
    return parse_floats<N>(*node, std::string(what) + "." + key);
}

[[nodiscard]] Result<std::string> string_field(const YAML::Node& map, const char* key,
                                               std::string_view what) {
    const Result<YAML::Node> node = field(map, key, what);
    if (!node) {
        return std::unexpected(node.error());
    }
    return scalar(*node, std::string(what) + "." + key);
}

[[nodiscard]] Result<Environment> parse_environment(const YAML::Node& node) {
    constexpr std::string_view kWhat = "world.environment";
    if (Result<void> r = check_map(node, kWhat,
                                   {"gravity", "wind", "air_density", "temperature_k", "seed"});
        !r) {
        return std::unexpected(r.error());
    }

    const Result<std::array<float, 3>> gravity = floats_field<3>(node, "gravity", kWhat);
    if (!gravity) return std::unexpected(gravity.error());
    const Result<std::array<float, 3>> wind = floats_field<3>(node, "wind", kWhat);
    if (!wind) return std::unexpected(wind.error());
    const Result<float> air_density = float_field(node, "air_density", kWhat);
    if (!air_density) return std::unexpected(air_density.error());
    const Result<float> temperature_k = float_field(node, "temperature_k", kWhat);
    if (!temperature_k) return std::unexpected(temperature_k.error());
    const Result<uint64_t> seed = uint_field<uint64_t>(node, "seed", kWhat);
    if (!seed) return std::unexpected(seed.error());

    Environment env;
    env.gravity = glm::vec3((*gravity)[0], (*gravity)[1], (*gravity)[2]);
    env.wind = glm::vec3((*wind)[0], (*wind)[1], (*wind)[2]);
    env.air_density = *air_density;
    env.temperature_k = *temperature_k;
    env.seed = *seed;
    return env;
}

[[nodiscard]] Result<Capacities> parse_capacities(const YAML::Node& node) {
    constexpr std::string_view kWhat = "world.capacities";
    if (Result<void> r =
            check_map(node, kWhat, {"bodies", "force_elements", "sensors", "contacts"});
        !r) {
        return std::unexpected(r.error());
    }

    const Result<uint32_t> bodies = uint_field<uint32_t>(node, "bodies", kWhat);
    if (!bodies) return std::unexpected(bodies.error());
    const Result<uint32_t> force_elements = uint_field<uint32_t>(node, "force_elements", kWhat);
    if (!force_elements) return std::unexpected(force_elements.error());
    const Result<uint32_t> sensors = uint_field<uint32_t>(node, "sensors", kWhat);
    if (!sensors) return std::unexpected(sensors.error());
    const Result<uint32_t> contacts = uint_field<uint32_t>(node, "contacts", kWhat);
    if (!contacts) return std::unexpected(contacts.error());

    Capacities caps;
    caps.bodies = *bodies;
    caps.force_elements = *force_elements;
    caps.sensors = *sensors;
    caps.contacts = *contacts;
    return caps;
}

[[nodiscard]] Result<std::vector<SpawnPoint>> parse_spawns(const YAML::Node& node) {
    if (!node.IsSequence()) {
        return std::unexpected(at(node, "world.spawns must be a sequence"));
    }
    std::vector<SpawnPoint> spawns;
    spawns.reserve(node.size());
    for (std::size_t i = 0; i < node.size(); ++i) {
        const std::string what = "world.spawns[" + dec(i) + "]";
        const YAML::Node entry = node[i];
        if (Result<void> r = check_map(entry, what, {"name", "position", "orientation"}); !r) {
            return std::unexpected(r.error());
        }
        const Result<std::string> name = string_field(entry, "name", what);
        if (!name) return std::unexpected(name.error());
        const Result<std::array<float, 3>> position = floats_field<3>(entry, "position", what);
        if (!position) return std::unexpected(position.error());
        const Result<std::array<float, 4>> orientation =
            floats_field<4>(entry, "orientation", what);
        if (!orientation) return std::unexpected(orientation.error());

        SpawnPoint spawn;
        spawn.name = *name;
        spawn.position = glm::vec3((*position)[0], (*position)[1], (*position)[2]);
        // [w, x, y, z] in the file; glm::quat's constructor takes (w, x, y, z).
        // NOT re-normalized -- see validate_world_desc()'s contract.
        spawn.orientation = glm::quat((*orientation)[0], (*orientation)[1], (*orientation)[2],
                                      (*orientation)[3]);
        spawns.push_back(std::move(spawn));
    }
    return spawns;
}

[[nodiscard]] Result<SdfTransform> parse_transform(const YAML::Node& node, std::string_view what) {
    if (Result<void> r = check_map(node, what, {"world_to_local", "scale"}); !r) {
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

[[nodiscard]] Result<SdfNode> parse_node(const YAML::Node& node, std::string_view what) {
    if (Result<void> r = check_map(node, what, {"prim", "op", "transform", "params"}); !r) {
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

[[nodiscard]] Result<SdfProgram> parse_sdf(const YAML::Node& node) {
    constexpr std::string_view kWhat = "world.sdf";
    if (Result<void> r = check_map(node, kWhat, {"transforms", "nodes"}); !r) {
        return std::unexpected(r.error());
    }

    const Result<YAML::Node> transforms = field(node, "transforms", kWhat);
    if (!transforms) return std::unexpected(transforms.error());
    if (!transforms->IsSequence()) {
        return std::unexpected(at(*transforms, "world.sdf.transforms must be a sequence"));
    }
    const Result<YAML::Node> nodes = field(node, "nodes", kWhat);
    if (!nodes) return std::unexpected(nodes.error());
    if (!nodes->IsSequence()) {
        return std::unexpected(at(*nodes, "world.sdf.nodes must be a sequence"));
    }

    SdfProgram program;
    program.transforms.reserve(transforms->size());
    for (std::size_t i = 0; i < transforms->size(); ++i) {
        const Result<SdfTransform> t =
            parse_transform((*transforms)[i], "world.sdf.transforms[" + dec(i) + "]");
        if (!t) return std::unexpected(t.error());
        program.transforms.push_back(*t);
    }
    program.nodes.reserve(nodes->size());
    for (std::size_t i = 0; i < nodes->size(); ++i) {
        const Result<SdfNode> n = parse_node((*nodes)[i], "world.sdf.nodes[" + dec(i) + "]");
        if (!n) return std::unexpected(n.error());
        program.nodes.push_back(*n);
    }
    return program;
}

[[nodiscard]] Result<std::vector<std::string>> parse_visuals(const YAML::Node& node) {
    if (!node.IsSequence()) {
        return std::unexpected(at(node, "world.visuals must be a sequence"));
    }
    std::vector<std::string> refs;
    refs.reserve(node.size());
    for (std::size_t i = 0; i < node.size(); ++i) {
        const Result<std::string> ref =
            scalar(node[i], "world.visuals[" + dec(i) + "]");
        if (!ref) return std::unexpected(ref.error());
        refs.push_back(*ref);
    }
    return refs;
}

[[nodiscard]] Result<WorldDesc> parse_world(const YAML::Node& root) {
    if (!root.IsMap()) {
        return std::unexpected(
            at(root, "a world file's top level must be a mapping with a 'world_version' key"));
    }

    // VERSION FIRST, before the unknown-key sweep. A v2 file is full of keys a
    // v1 reader does not know, and "unknown key 'foo'" would be a true but
    // useless answer to a question whose real answer is "this build does not
    // read version 2".
    const Result<YAML::Node> version_node = field(root, "world_version", "world");
    if (!version_node) return std::unexpected(version_node.error());
    const Result<uint32_t> version = parse_uint<uint32_t>(*version_node, "world.world_version");
    if (!version) return std::unexpected(version.error());
    if (*version != kWorldFileVersion) {
        // schema_mismatch, NOT invalid_argument, and the distinction is
        // user-visible rather than taxonomic: every other rejection in this
        // loader means "fix your file", while this one means "this build
        // cannot read this artifact" -- a caller may reasonably respond by
        // upgrading, or by migrating the file, and it needs to be able to tell
        // the two apart without matching on a string. state/snapshot.cpp:250
        // says the identical sentence about the identical situation (a
        // versioned artifact the running build does not read) with this exact
        // code; one engine, one answer.
        return std::unexpected(Error{Code::schema_mismatch,
                                     "world file is schema version " + dec(*version) +
                                         ", this build reads version " + dec(kWorldFileVersion) +
                                         " only" + mark_of(*version_node)});
    }

    if (Result<void> r = check_map(root, "world",
                                   {"world_version", "name", "environment", "capacities", "spawns",
                                    "sdf", "visuals"});
        !r) {
        return std::unexpected(r.error());
    }

    WorldDesc world;

    const Result<std::string> name = string_field(root, "name", "world");
    if (!name) return std::unexpected(name.error());
    world.name = *name;

    const Result<YAML::Node> env_node = field(root, "environment", "world");
    if (!env_node) return std::unexpected(env_node.error());
    const Result<Environment> env = parse_environment(*env_node);
    if (!env) return std::unexpected(env.error());
    world.environment = *env;

    const Result<YAML::Node> caps_node = field(root, "capacities", "world");
    if (!caps_node) return std::unexpected(caps_node.error());
    const Result<Capacities> caps = parse_capacities(*caps_node);
    if (!caps) return std::unexpected(caps.error());
    world.capacities = *caps;

    const Result<YAML::Node> spawns_node = field(root, "spawns", "world");
    if (!spawns_node) return std::unexpected(spawns_node.error());
    const Result<std::vector<SpawnPoint>> spawns = parse_spawns(*spawns_node);
    if (!spawns) return std::unexpected(spawns.error());
    world.spawns = *spawns;

    const Result<YAML::Node> sdf_node = field(root, "sdf", "world");
    if (!sdf_node) return std::unexpected(sdf_node.error());
    const Result<SdfProgram> sdf = parse_sdf(*sdf_node);
    if (!sdf) return std::unexpected(sdf.error());
    world.sdf = *sdf;

    const Result<YAML::Node> visuals_node = field(root, "visuals", "world");
    if (!visuals_node) return std::unexpected(visuals_node.error());
    const Result<std::vector<std::string>> visuals = parse_visuals(*visuals_node);
    if (!visuals) return std::unexpected(visuals.error());
    world.visual_refs = *visuals;

    // THE SAME VALIDATION WorldBuilder::build() RUNS. Not a copy of it, not a
    // subset chosen for a file -- the same function (world/builder.hpp).
    const Result<uint32_t> depth = validate_world_desc(world);
    if (!depth) {
        return std::unexpected(depth.error());
    }
    return world;
}

// std::filesystem::path::string() can throw on a path the native narrow
// encoding cannot represent, and nothing may be thrown out of a Spade API.
[[nodiscard]] std::string path_text(const std::filesystem::path& path) noexcept {
    try {
        return path.string();
    } catch (...) {
        return "<unprintable path>";
    }
}

}  // namespace

// ===========================================================================
// Public API
// ===========================================================================

Result<std::string> world_to_yaml(const WorldDesc& world) {
    const Result<uint32_t> valid = validate_world_desc(world);
    if (!valid) {
        return std::unexpected(
            Error{valid.error().code, "world_to_yaml: " + valid.error().context});
    }

    // REPRESENTABILITY. Schema v1 carries a node's discriminated identity
    // (`prim` XOR `op`), its transform index and its four params -- and
    // nothing else. The two fields it does not carry are an operator node's
    // `kind` (sdf.hpp: unused on an operator) and the explicit padding both
    // PODs hold for std430. Both are zero in everything WorldBuilder produces
    // and in everything this loader returns, so refusing a non-zero one costs
    // nothing and buys the round trip its BIT-EXACTNESS: what cannot be
    // written cannot be silently dropped.
    for (std::size_t i = 0; i < world.sdf.nodes.size(); ++i) {
        const SdfNode& node = world.sdf.nodes[i];
        if (node._pad != 0) {
            return std::unexpected(Error{Code::invalid_argument,
                                         "world_to_yaml: SDF node " + dec(i) +
                                             " has non-zero padding, which schema v" +
                                             dec(kWorldFileVersion) + " does not carry"});
        }
        if (node.op != static_cast<uint32_t>(SdfOp::none) && node.kind != 0) {
            return std::unexpected(
                Error{Code::invalid_argument,
                      "world_to_yaml: SDF operator node " + dec(i) +
                          " carries a non-zero primitive kind, which schema v" +
                          dec(kWorldFileVersion) +
                          " does not carry (an operator's `kind` is unused; the builder always "
                          "leaves it 0)"});
        }
    }
    for (std::size_t i = 0; i < world.sdf.transforms.size(); ++i) {
        const SdfTransform& t = world.sdf.transforms[i];
        if (t._pad[0] != 0.0f || t._pad[1] != 0.0f || t._pad[2] != 0.0f) {
            return std::unexpected(Error{Code::invalid_argument,
                                         "world_to_yaml: SDF transform " + dec(i) +
                                             " has non-zero padding, which schema v" +
                                             dec(kWorldFileVersion) + " does not carry"});
        }
    }

    return emit(world);
}

Result<WorldDesc> world_from_yaml(std::string_view yaml_text) {
    // yaml-cpp reports every syntax failure by throwing; nothing may leave a
    // Spade API that way. YAML::Exception::what() already carries the mark
    // ("yaml-cpp: error at line L, column C: ..."), which is the context the
    // loader policy asks for.
    try {
        const YAML::Node root = YAML::Load(std::string(yaml_text));
        return parse_world(root);
    } catch (const YAML::Exception& e) {
        return std::unexpected(Error{Code::invalid_argument, std::string(e.what())});
    } catch (const std::exception& e) {
        return std::unexpected(
            Error{Code::internal, std::string("world file parse failed: ") + e.what()});
    }
}

Result<void> save_world_file(const WorldDesc& world, const std::filesystem::path& path) {
    const Result<std::string> text = world_to_yaml(world);
    if (!text) {
        return std::unexpected(text.error());
    }
    try {
        std::ofstream out(path, std::ios::binary | std::ios::trunc);
        if (!out) {
            return std::unexpected(Error{Code::io_error, "cannot open world file '" +
                                                             path_text(path) + "' for writing"});
        }
        out.write(text->data(), static_cast<std::streamsize>(text->size()));
        out.close();
        if (!out) {
            return std::unexpected(
                Error{Code::io_error, "failed writing world file '" + path_text(path) + "'"});
        }
    } catch (const std::exception& e) {
        return std::unexpected(Error{Code::io_error, "failed writing world file '" +
                                                         path_text(path) + "': " + e.what()});
    }
    return {};
}

Result<WorldDesc> load_world_file(const std::filesystem::path& path) {
    std::string text;
    try {
        std::ifstream in(path, std::ios::binary);
        if (!in) {
            return std::unexpected(
                Error{Code::io_error, "cannot open world file '" + path_text(path) + "'"});
        }
        text.assign(std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>());
        if (in.bad()) {
            return std::unexpected(
                Error{Code::io_error, "failed reading world file '" + path_text(path) + "'"});
        }
    } catch (const std::exception& e) {
        return std::unexpected(Error{Code::io_error, "failed reading world file '" +
                                                         path_text(path) + "': " + e.what()});
    }

    Result<WorldDesc> world = world_from_yaml(text);
    if (!world) {
        // The parse diagnostics carry a YAML mark; only this layer knows which
        // file they came from.
        return std::unexpected(
            Error{world.error().code, path_text(path) + ": " + world.error().context});
    }
    return world;
}

}  // namespace spade
