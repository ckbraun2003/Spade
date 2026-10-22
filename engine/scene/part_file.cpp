#include "scene/part_file.hpp"

#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <filesystem>
#include <fstream>
#include <initializer_list>
#include <iterator>
#include <optional>
#include <string>
#include <string_view>
#include <type_traits>
#include <utility>
#include <vector>

#include <yaml-cpp/yaml.h>

#include "world/detail/yaml_text.hpp"

namespace spade::scene {
namespace {

using yaml_text::at;
using yaml_text::dec;
using yaml_text::double_field;
using yaml_text::field;
using yaml_text::format_double;
using yaml_text::mark_of;
using yaml_text::parse_double;
using yaml_text::parse_uint;
using yaml_text::path_text;
using yaml_text::quote_yaml;
using yaml_text::scalar;
using yaml_text::string_field;
using yaml_text::uint_field;

// Every diagnostic blames the part file's own schema.
constexpr std::string_view kSchema = "part schema v1";
constexpr std::string_view kTop = "part file";

// The spellings, in enum order (TD-9: the only place each is written).
constexpr std::array<std::string_view, 4> kKindNames = {"motor", "prop", "esc", "battery"};
constexpr std::array<std::string_view, 4> kSuffixes = {".motor.yaml", ".prop.yaml", ".esc.yaml", ".battery.yaml"};
// Indexed by vehicles::PartShape: point, sphere, box, cylinder, tube, tensor.
constexpr std::array<std::string_view, 6> kShapeNames = {"point", "sphere", "box", "cylinder", "tube", "tensor"};
static_assert(static_cast<std::size_t>(vehicles::PartShape::tensor) == 5, "kShapeNames is indexed by PartShape");

using Table = std::vector<std::array<double, 2>>;

[[nodiscard]] Error invalid(std::string context) { return Error{Code::invalid_argument, std::move(context)}; }

[[nodiscard]] std::string kind_text(PartKind kind) { return std::string(kKindNames[static_cast<std::size_t>(kind)]); }

// ---------------------------------------------------------------------------
// THE KEYS. One walk per block, in declaration order, which the writer, the
// reader, the finite check and the citation list all take (TD-9: each key
// list is written once). The identity strings are `text`; everything else
// cites.
// ---------------------------------------------------------------------------

template <class V, class M>
void walk_motor(V& v, M& b) {
    v.number("kv", b.kv);
    v.number("resistance", b.resistance);
    v.number("no_load_current", b.no_load_current);
    v.number("no_load_voltage", b.no_load_voltage);
    v.number("current_max", b.current_max);
    v.number("pole_pairs", b.pole_pairs);
    v.number("inductance", b.inductance);
    v.number("rotor_inertia", b.rotor_inertia);
    v.text("stator", b.stator);
    v.number("mass", b.mass);
}

template <class V, class P>
void walk_prop(V& v, P& b) {
    v.number("diameter", b.diameter);
    v.number("pitch", b.pitch);
    v.count("blades", b.blades);
    v.number("inertia", b.inertia);
    v.number("ct_static", b.ct_static);
    v.number("cq_static", b.cq_static);
    v.table("ct_table", b.ct_table);
    v.table("cq_table", b.cq_table);
    v.number("mass", b.mass);
}

template <class V, class E>
void walk_esc(V& v, E& b) {
    v.number("current_continuous", b.current_continuous);
    v.number("current_burst", b.current_burst);
    v.number("current_total", b.current_total);
    v.count("channels", b.channels);
    v.number("on_resistance", b.on_resistance);
    v.flag("braking", b.braking);
    v.text("protocol", b.protocol);
    v.number("mass", b.mass);
}

template <class V, class B>
void walk_battery(V& v, B& b) {
    v.count("cells_series", b.cells_series);
    v.count("cells_parallel", b.cells_parallel);
    v.number("cell_capacity", b.cell_capacity);
    v.number("cell_resistance", b.cell_resistance);
    v.number("cell_voltage_nominal", b.cell_voltage_nominal);
    v.number("cell_voltage_full", b.cell_voltage_full);
    v.number("cell_voltage_cutoff", b.cell_voltage_cutoff);
    v.table("ocv_table", b.ocv_table);
    v.number("c_rating", b.c_rating);
    v.number("polarization_resistance", b.polarization_resistance);
    v.number("polarization_capacitance", b.polarization_capacitance);
    v.number("mass", b.mass);
}

// The block's walk, for a part of either constness.
template <class V, class Part>
void walk(V& v, Part& part) {
    using P = std::remove_const_t<Part>;
    if constexpr (std::is_same_v<P, MotorPart>) {
        walk_motor(v, part.block);
    } else if constexpr (std::is_same_v<P, PropPart>) {
        walk_prop(v, part.block);
    } else if constexpr (std::is_same_v<P, EscPart>) {
        walk_esc(v, part.block);
    } else {
        static_assert(std::is_same_v<P, BatteryPart>);
        walk_battery(v, part.block);
    }
}

template <class Part>
constexpr PartKind kind_of() {
    if constexpr (std::is_same_v<Part, MotorPart>) {
        return PartKind::motor;
    } else if constexpr (std::is_same_v<Part, PropPart>) {
        return PartKind::prop;
    } else if constexpr (std::is_same_v<Part, EscPart>) {
        return PartKind::esc;
    } else {
        return PartKind::battery;
    }
}

constexpr std::array<std::string_view, 5> kBladeKeys = {"r_over_r", "c_over_r", "beta_deg", "hub_cutout", "airfoil"};

// Every key of the block, cited or not.
struct KeyList {
    std::vector<std::string> keys;
    void number(const char* k, const double&) { keys.emplace_back(k); }
    void count(const char* k, const uint32_t&) { keys.emplace_back(k); }
    void flag(const char* k, const bool&) { keys.emplace_back(k); }
    void text(const char* k, const std::string&) { keys.emplace_back(k); }
    void table(const char* k, const Table&) { keys.emplace_back(k); }
};

// The keys that cite: all but the identity strings.
struct CitedList {
    std::vector<std::string> keys;
    void number(const char* k, const double&) { keys.emplace_back(k); }
    void count(const char* k, const uint32_t&) { keys.emplace_back(k); }
    void flag(const char* k, const bool&) { keys.emplace_back(k); }
    void text(const char*, const std::string&) {}
    void table(const char* k, const Table&) { keys.emplace_back(k); }
};

// The identity strings: fields that take no citation.
struct IdentityList {
    std::vector<std::string> keys;
    void number(const char*, const double&) {}
    void count(const char*, const uint32_t&) {}
    void flag(const char*, const bool&) {}
    void text(const char* k, const std::string&) { keys.emplace_back(k); }
    void table(const char*, const Table&) {}
};

template <class Part>
[[nodiscard]] std::vector<std::string> identity_of(const Part& part) {
    IdentityList list;
    walk(list, part);
    return list.keys;
}

[[nodiscard]] const std::optional<vehicles::BlockShape>& shape_of(const MotorPart& p) { return p.block.shape; }
[[nodiscard]] const std::optional<vehicles::BlockShape>& shape_of(const PropPart& p) { return p.block.shape; }
[[nodiscard]] const std::optional<vehicles::BlockShape>& shape_of(const EscPart& p) { return p.block.shape; }
[[nodiscard]] const std::optional<vehicles::BlockShape>& shape_of(const BatteryPart& p) { return p.block.shape; }

template <class Part>
[[nodiscard]] std::vector<std::string> cited_of(const Part& part) {
    CitedList list;
    walk(list, part);
    if (shape_of(part)) list.keys.emplace_back("shape");
    if constexpr (std::is_same_v<Part, PropPart>) {
        if (part.blade) {
            for (const std::string_view k : kBladeKeys) list.keys.push_back("blade." + std::string(k));
        }
    }
    return list.keys;
}

// ---------------------------------------------------------------------------
// THE RULES both directions apply, so what is written reads back.
// ---------------------------------------------------------------------------

// Every number finite, named by its path.
struct FiniteCheck {
    std::string what;
    std::optional<Error> error;
    void fail(const std::string& path, double v) {
        if (!error) {
            error = invalid(path + " is not finite (" + format_double(v) + "); a part file carries finite numbers");
        }
    }
    void number(const char* k, const double& v) {
        if (!std::isfinite(v)) fail(what + "." + k, v);
    }
    void count(const char*, const uint32_t&) {}
    void flag(const char*, const bool&) {}
    void text(const char*, const std::string&) {}
    void table(const char* k, const Table& t) {
        for (std::size_t i = 0; i < t.size(); ++i) {
            for (std::size_t j = 0; j < 2; ++j) {
                if (!std::isfinite(t[i][j])) fail(what + "." + k + "[" + dec(i) + "][" + dec(j) + "]", t[i][j]);
            }
        }
    }
};

[[nodiscard]] Result<void> check_finite(std::string_view path, std::initializer_list<double> values) {
    for (const double v : values) {
        if (!std::isfinite(v)) {
            return std::unexpected(invalid(std::string(path) + " is not finite (" + format_double(v) +
                                           "); a part file carries finite numbers"));
        }
    }
    return {};
}

[[nodiscard]] Result<void> check_shape(const vehicles::BlockShape& s, const std::string& what) {
    const auto k = static_cast<std::size_t>(s.shape);
    if (k >= kShapeNames.size()) return std::unexpected(invalid(what + ".kind is not a shape"));
    if (s.shape == vehicles::PartShape::point) {
        return std::unexpected(invalid(what + " is a point: leave the key out for a point mass"));
    }
    // The member the kind does not use has no key in the text, so a set one
    // (any bit, -0 too) would be dropped: refused instead (refuse, never skip).
    const auto any_bit = [](std::initializer_list<double> values) {
        return std::any_of(values.begin(), values.end(), [](double v) { return std::bit_cast<uint64_t>(v) != 0; });
    };
    const std::string name(kShapeNames[k]);
    if (s.shape == vehicles::PartShape::tensor) {
        if (any_bit({s.size.x, s.size.y, s.size.z})) {
            return std::unexpected(
                invalid(what + ".size is set on a tensor shape, which takes only its tensor: leave it zero"));
        }
        for (int i = 0; i < 3; ++i) {
            if (Result<void> r = check_finite(what + ".tensor", {s.tensor[i][0], s.tensor[i][1], s.tensor[i][2]}); !r) {
                return r;
            }
        }
        return {};
    }
    for (int i = 0; i < 3; ++i) {
        if (any_bit({s.tensor[i][0], s.tensor[i][1], s.tensor[i][2]})) {
            return std::unexpected(invalid(what + ".tensor is set on a " + name +
                                           " shape, which takes only its size: leave it zero"));
        }
    }
    return check_finite(what + ".size", {s.size.x, s.size.y, s.size.z});
}

[[nodiscard]] Result<void> check_blade(const BladeGeometry& b) {
    const std::string what = "prop.blade";
    const std::size_t n = b.r_over_r.size();
    if (b.c_over_r.size() != n || b.beta_deg.size() != n) {
        const bool chord = b.c_over_r.size() != n;
        return std::unexpected(invalid(what + " has " + dec(n) + " stations in r_over_r but " +
                                       dec(chord ? b.c_over_r.size() : b.beta_deg.size()) + " in " +
                                       (chord ? "c_over_r" : "beta_deg") + "; every list has one per station"));
    }
    if (n < 2) return std::unexpected(invalid(what + " needs at least 2 stations, found " + dec(n)));
    for (std::size_t i = 0; i < n; ++i) {
        const std::string at_i = "[" + dec(i) + "]";
        if (Result<void> r = check_finite(what + ".r_over_r" + at_i, {b.r_over_r[i]}); !r) return r;
        if (Result<void> r = check_finite(what + ".c_over_r" + at_i, {b.c_over_r[i]}); !r) return r;
        if (Result<void> r = check_finite(what + ".beta_deg" + at_i, {b.beta_deg[i]}); !r) return r;
        if (!(b.r_over_r[i] > 0.0 && b.r_over_r[i] <= 1.0)) {
            return std::unexpected(invalid(what + ".r_over_r" + at_i + " " + format_double(b.r_over_r[i]) +
                                           " is outside (0, 1]"));
        }
        if (i > 0 && !(b.r_over_r[i] > b.r_over_r[i - 1])) {
            return std::unexpected(invalid(what + ".r_over_r must be strictly increasing: " + at_i + " " +
                                           format_double(b.r_over_r[i]) + " is not above [" + dec(i - 1) + "] " +
                                           format_double(b.r_over_r[i - 1])));
        }
        if (!(b.c_over_r[i] > 0.0)) {
            return std::unexpected(invalid(what + ".c_over_r" + at_i + " must be above 0, found " +
                                           format_double(b.c_over_r[i])));
        }
    }
    if (Result<void> r = check_finite(what + ".hub_cutout", {b.hub_cutout}); !r) return r;
    if (!(b.hub_cutout >= 0.0)) {
        return std::unexpected(invalid(what + ".hub_cutout must be at least 0, found " + format_double(b.hub_cutout)));
    }
    if (b.hub_cutout > b.r_over_r[0]) {
        return std::unexpected(invalid(what + ".hub_cutout " + format_double(b.hub_cutout) +
                                       " is above the first station's r/R " + format_double(b.r_over_r[0])));
    }
    if (b.airfoil.empty()) return std::unexpected(invalid(what + ".airfoil must not be empty"));
    return {};
}

[[nodiscard]] bool is_identity(const vehicles::MountPose& m) {
    return m.position.x == 0.0 && m.position.y == 0.0 && m.position.z == 0.0 && m.orientation.w == 1.0 &&
           m.orientation.x == 0.0 && m.orientation.y == 0.0 && m.orientation.z == 0.0;
}

// The citations against the part's cited fields: none extra, none missing,
// each grade one of the six. `identity` names the fields that take none.
[[nodiscard]] Result<void> check_citations(const std::string& kind, const std::vector<std::string>& cited,
                                           const std::vector<std::string>& identity, const PartSources& sources) {
    for (std::size_t i = 0; i < sources.size(); ++i) {
        const std::string& f = sources[i].field;
        if (std::find(cited.begin(), cited.end(), f) == cited.end()) {
            if (std::find(identity.begin(), identity.end(), f) != identity.end()) {
                return std::unexpected(
                    invalid("sources." + f + ": the " + kind + "'s field '" + f + "' takes no citation"));
            }
            if (f == "shape") return std::unexpected(invalid("sources.shape: the " + kind + " has no shape"));
            if (f.starts_with("blade.")) {
                return std::unexpected(invalid("sources." + f + ": the " + kind + " has no blade"));
            }
            return std::unexpected(invalid("sources." + f + ": the " + kind + " has no field '" + f + "'"));
        }
        for (std::size_t j = 0; j < i; ++j) {
            if (sources[j].field == f) return std::unexpected(invalid("sources has two citations of '" + f + "'"));
        }
        if (grade_name(sources[i].ref.grade).empty()) {
            return std::unexpected(invalid("sources." + f + " has grade " +
                                           dec(static_cast<uint32_t>(sources[i].ref.grade)) +
                                           ", which is not one of the six"));
        }
    }
    for (const std::string& f : cited) {
        const bool found =
            std::any_of(sources.begin(), sources.end(), [&](const FieldSource& s) { return s.field == f; });
        if (!found) return std::unexpected(invalid(kind + "." + f + " has no citation in sources"));
    }
    return {};
}

template <class Part>
[[nodiscard]] Result<void> validate(const Part& part) {
    const std::string kind = kind_text(kind_of<Part>());
    FiniteCheck finite{kind, std::nullopt};
    walk(finite, part);
    if (finite.error) return std::unexpected(*finite.error);
    if (const auto& s = shape_of(part)) {
        if (Result<void> r = check_shape(*s, kind + ".shape"); !r) return r;
    }
    if constexpr (std::is_same_v<Part, PropPart>) {
        if (part.blade) {
            if (Result<void> r = check_blade(*part.blade); !r) return r;
        }
    }
    if constexpr (std::is_same_v<Part, EscPart> || std::is_same_v<Part, BatteryPart>) {
        if (!is_identity(part.block.mount)) {
            return std::unexpected(invalid(kind + ".mount is the drone's, not the part's: a part file carries none "
                                                  "(leave it the identity)"));
        }
    }
    return check_citations(kind, cited_of(part), identity_of(part), part.sources);
}

// ---------------------------------------------------------------------------
// THE WRITER
// ---------------------------------------------------------------------------

[[nodiscard]] std::string double_list(const double* values, std::size_t n) {
    std::string out = "[";
    for (std::size_t i = 0; i < n; ++i) {
        if (i != 0) out += ", ";
        out += format_double(values[i]);
    }
    return out + "]";
}

[[nodiscard]] std::string double_list(const std::vector<double>& values) {
    return double_list(values.data(), values.size());
}

struct Writer {
    std::string& out;
    void line(const char* k, const std::string& value) {
        out += "  ";
        out += k;
        out += ": " + value + "\n";
    }
    void number(const char* k, const double& v) { line(k, format_double(v)); }
    void count(const char* k, const uint32_t& v) { line(k, dec(v)); }
    void flag(const char* k, const bool& v) { line(k, v ? "true" : "false"); }
    void text(const char* k, const std::string& v) { line(k, quote_yaml(v)); }
    void table(const char* k, const Table& t) {
        if (t.empty()) {
            line(k, "[]");
            return;
        }
        out += "  ";
        out += k;
        out += ":\n";
        for (const std::array<double, 2>& row : t) out += "    - " + double_list(row.data(), 2) + "\n";
    }
};

template <class Part>
[[nodiscard]] Result<std::string> write(const Part& part) {
    if (Result<void> r = validate(part); !r) return std::unexpected(r.error());
    const std::string kind = kind_text(kind_of<Part>());
    std::string out = "part_version: " + dec(kPartVersion) + "\n" + kind + ":\n";
    Writer w{out};
    walk(w, part);
    if (const auto& s = shape_of(part)) {
        const std::string name(kShapeNames[static_cast<std::size_t>(s->shape)]);
        if (s->shape == vehicles::PartShape::tensor) {
            // Row-major: glm indexes [column][row].
            const std::array<double, 9> t = {s->tensor[0][0], s->tensor[1][0], s->tensor[2][0],
                                             s->tensor[0][1], s->tensor[1][1], s->tensor[2][1],
                                             s->tensor[0][2], s->tensor[1][2], s->tensor[2][2]};
            out += "  shape: {kind: " + name + ", tensor: " + double_list(t.data(), t.size()) + "}\n";
        } else {
            const std::array<double, 3> size = {s->size.x, s->size.y, s->size.z};
            out += "  shape: {kind: " + name + ", size: " + double_list(size.data(), size.size()) + "}\n";
        }
    }
    if constexpr (std::is_same_v<Part, PropPart>) {
        if (part.blade) {
            const BladeGeometry& b = *part.blade;
            out += "  blade:\n";
            out += "    r_over_r: " + double_list(b.r_over_r) + "\n";
            out += "    c_over_r: " + double_list(b.c_over_r) + "\n";
            out += "    beta_deg: " + double_list(b.beta_deg) + "\n";
            out += "    hub_cutout: " + format_double(b.hub_cutout) + "\n";
            out += "    airfoil: " + quote_yaml(b.airfoil) + "\n";
        }
    }
    out += "sources:\n";
    for (const std::string& f : cited_of(part)) {
        const auto it = std::find_if(part.sources.begin(), part.sources.end(),
                                     [&](const FieldSource& s) { return s.field == f; });
        const SourceRef& ref = it->ref;  // validate() found every one
        out += "  " + f + ": {";
        if (!ref.source.empty()) out += "source: " + quote_yaml(ref.source) + ", ";
        out += "grade: " + std::string(grade_name(ref.grade));
        if (!ref.note.empty()) out += ", note: " + quote_yaml(ref.note);
        out += "}\n";
    }
    return out;
}

// ---------------------------------------------------------------------------
// THE READER
// ---------------------------------------------------------------------------

// A mapping whose every key is in `allowed`, none twice (yaml_text::
// check_map's rule and words, for a key list built at run time).
[[nodiscard]] Result<void> check_keys(const YAML::Node& node, const std::string& what,
                                      const std::vector<std::string>& allowed) {
    if (!node.IsMap()) return std::unexpected(at(node, what + " must be a mapping"));
    std::vector<std::string> seen;
    for (YAML::const_iterator it = node.begin(); it != node.end(); ++it) {
        if (!it->first.IsScalar()) return std::unexpected(at(it->first, "non-scalar key in " + what));
        const std::string key = it->first.Scalar();
        if (std::find(allowed.begin(), allowed.end(), key) == allowed.end()) {
            return std::unexpected(at(it->first, "unknown key '" + key + "' in " + what + " -- " +
                                                     std::string(kSchema) + " does not define it"));
        }
        if (std::find(seen.begin(), seen.end(), key) != seen.end()) {
            return std::unexpected(at(it->first, "duplicate key '" + key + "' in " + what));
        }
        seen.push_back(key);
    }
    return {};
}

// `n` numbers (any count when `n` is 0) at `node`.
[[nodiscard]] Result<std::vector<double>> number_list(const YAML::Node& node, const std::string& what, std::size_t n) {
    if (!node.IsSequence() || (n != 0 && node.size() != n)) {
        return std::unexpected(at(node, what + " must be a sequence of " + (n != 0 ? dec(n) + " " : "") + "numbers"));
    }
    std::vector<double> out;
    out.reserve(node.size());
    for (std::size_t i = 0; i < node.size(); ++i) {
        const Result<double> v = parse_double(node[i], what + "[" + dec(i) + "]");
        if (!v) return std::unexpected(v.error());
        out.push_back(*v);
    }
    return out;
}

struct Reader {
    const YAML::Node& node;
    std::string what;
    std::optional<Error> error;
    void number(const char* k, double& slot) {
        if (error) return;
        const Result<double> v = double_field(node, k, what);
        if (!v) error = v.error();
        else slot = *v;
    }
    void count(const char* k, uint32_t& slot) {
        if (error) return;
        const Result<uint32_t> v = uint_field<uint32_t>(node, k, what);
        if (!v) error = v.error();
        else slot = *v;
    }
    void flag(const char* k, bool& slot) {
        if (error) return;
        const Result<std::string> v = string_field(node, k, what);
        if (!v) {
            error = v.error();
        } else if (*v == "true" || *v == "false") {
            slot = *v == "true";
        } else {
            error = at(node[k], what + "." + k + ": '" + *v + "' must be true or false");
        }
    }
    void text(const char* k, std::string& slot) {
        if (error) return;
        Result<std::string> v = string_field(node, k, what);
        if (!v) error = v.error();
        else slot = std::move(*v);
    }
    void table(const char* k, Table& slot) {
        if (error) return;
        const Result<YAML::Node> t = field(node, k, what);
        if (!t) {
            error = t.error();
            return;
        }
        const std::string path = what + "." + k;
        if (!t->IsSequence()) {
            error = at(*t, path + " must be a sequence of [x, value] pairs ([] for none)");
            return;
        }
        Table out;
        for (std::size_t i = 0; i < t->size(); ++i) {
            const Result<std::vector<double>> row = number_list((*t)[i], path + "[" + dec(i) + "]", 2);
            if (!row) {
                error = row.error();
                return;
            }
            out.push_back({(*row)[0], (*row)[1]});
        }
        slot = std::move(out);
    }
};

[[nodiscard]] Result<vehicles::BlockShape> read_shape(const YAML::Node& node, const std::string& what) {
    if (node.IsNull()) return std::unexpected(at(node, what + " is null: leave the key out for none"));
    if (Result<void> r = check_keys(node, what, {"kind", "size", "tensor"}); !r) return std::unexpected(r.error());
    const Result<std::string> kind = string_field(node, "kind", what);
    if (!kind) return std::unexpected(kind.error());
    if (*kind == "point") {
        return std::unexpected(at(node, what + " is a point: leave the key out for a point mass"));
    }
    std::size_t k = 1;
    while (k < kShapeNames.size() && kShapeNames[k] != *kind) ++k;
    if (k == kShapeNames.size()) {
        return std::unexpected(at(node["kind"], what + ".kind '" + *kind +
                                                    "' must be sphere, box, cylinder, tube or tensor"));
    }
    vehicles::BlockShape out{};
    out.shape = static_cast<vehicles::PartShape>(k);
    const bool tensor = out.shape == vehicles::PartShape::tensor;
    const char* const needed = tensor ? "tensor" : "size";
    const char* const other = tensor ? "size" : "tensor";
    if (node[other].IsDefined()) {
        return std::unexpected(at(node[other], what + "." + other + " is not a " + *kind + "'s: a " + *kind +
                                                   " has " + needed));
    }
    const Result<YAML::Node> values = field(node, needed, what);
    if (!values) return std::unexpected(values.error());
    const Result<std::vector<double>> v = number_list(*values, what + "." + needed, tensor ? 9 : 3);
    if (!v) return std::unexpected(v.error());
    if (tensor) {
        // Row-major text; glm indexes [column][row].
        for (int row = 0; row < 3; ++row) {
            for (int col = 0; col < 3; ++col) out.tensor[col][row] = (*v)[static_cast<std::size_t>(3 * row + col)];
        }
    } else {
        out.size = {(*v)[0], (*v)[1], (*v)[2]};
    }
    return out;
}

[[nodiscard]] Result<BladeGeometry> read_blade(const YAML::Node& node) {
    const std::string what = "prop.blade";
    if (node.IsNull()) return std::unexpected(at(node, what + " is null: leave the key out for none"));
    if (Result<void> r = check_keys(node, what, std::vector<std::string>(kBladeKeys.begin(), kBladeKeys.end())); !r) {
        return std::unexpected(r.error());
    }
    BladeGeometry out{};
    for (const auto& [key, slot] : {std::pair{"r_over_r", &out.r_over_r}, std::pair{"c_over_r", &out.c_over_r},
                                    std::pair{"beta_deg", &out.beta_deg}}) {
        const Result<YAML::Node> list = field(node, key, what);
        if (!list) return std::unexpected(list.error());
        Result<std::vector<double>> values = number_list(*list, what + "." + key, 0);
        if (!values) return std::unexpected(values.error());
        *slot = std::move(*values);
    }
    const Result<double> hub = double_field(node, "hub_cutout", what);
    if (!hub) return std::unexpected(hub.error());
    out.hub_cutout = *hub;
    Result<std::string> airfoil = string_field(node, "airfoil", what);
    if (!airfoil) return std::unexpected(airfoil.error());
    out.airfoil = std::move(*airfoil);
    if (Result<void> r = check_blade(out); !r) {
        return std::unexpected(Error{r.error().code, r.error().context + mark_of(node)});
    }
    return out;
}

// One optional, non-empty string of a citation.
[[nodiscard]] Result<std::string> citation_text(const YAML::Node& node, const char* key, const std::string& what,
                                                std::string_view none) {
    const YAML::Node value = node[key];
    if (!value.IsDefined()) return std::string{};
    const std::string path = what + "." + key;
    if (value.IsNull()) return std::unexpected(at(value, path + " is null: leave the key out " + std::string(none)));
    Result<std::string> text = scalar(value, path);
    if (!text) return text;
    if (text->empty()) {
        return std::unexpected(at(value, path + " must not be empty: leave the key out " + std::string(none)));
    }
    return text;
}

[[nodiscard]] Result<PartSources> read_sources(const YAML::Node& root, const std::string& kind,
                                               const std::vector<std::string>& cited,
                                               const std::vector<std::string>& identity) {
    const Result<YAML::Node> sources = field(root, "sources", kTop);
    if (!sources) return std::unexpected(sources.error());
    if (!sources->IsMap()) return std::unexpected(at(*sources, "sources must be a mapping, one citation per field"));
    std::vector<std::string> seen;
    for (YAML::const_iterator it = sources->begin(); it != sources->end(); ++it) {
        if (!it->first.IsScalar()) return std::unexpected(at(it->first, "non-scalar key in sources"));
        const std::string f = it->first.Scalar();
        if (std::find(seen.begin(), seen.end(), f) != seen.end()) {
            return std::unexpected(at(it->first, "duplicate key '" + f + "' in sources"));
        }
        seen.push_back(f);
        if (std::find(cited.begin(), cited.end(), f) == cited.end()) {
            const Result<void> r = check_citations(kind, cited, identity, PartSources{FieldSource{f, SourceRef{}}});
            return std::unexpected(at(it->first, r ? "sources." + f + ": not a field of the " + kind
                                                   : r.error().context));
        }
    }
    PartSources out;
    out.reserve(cited.size());
    for (const std::string& f : cited) {
        const YAML::Node node = (*sources)[f];
        if (!node.IsDefined()) return std::unexpected(at(*sources, kind + "." + f + " has no citation in sources"));
        const std::string what = "sources." + f;
        if (Result<void> r = yaml_text::check_map(node, what, {"source", "grade", "note"}, kSchema); !r) {
            return std::unexpected(r.error());
        }
        SourceRef ref{};
        Result<std::string> source = citation_text(node, "source", what, "for an assumed value that cites none");
        if (!source) return std::unexpected(source.error());
        ref.source = std::move(*source);
        const Result<std::string> grade_text = string_field(node, "grade", what);
        if (!grade_text) return std::unexpected(grade_text.error());
        const Result<Grade> grade = parse_grade(*grade_text, what + ".grade");
        if (!grade) return std::unexpected(Error{grade.error().code, grade.error().context + mark_of(node["grade"])});
        ref.grade = *grade;
        Result<std::string> note = citation_text(node, "note", what, "for none");
        if (!note) return std::unexpected(note.error());
        ref.note = std::move(*note);
        out.push_back(FieldSource{f, std::move(ref)});
    }
    return out;
}

template <class Part>
[[nodiscard]] Result<Part> read_root(const YAML::Node& root) {
    const PartKind kind = kind_of<Part>();
    const std::string name = kind_text(kind);
    if (!root.IsMap()) {
        return std::unexpected(at(root, "a part file's top level must be a mapping with a 'part_version' key"));
    }
    // VERSION FIRST, before the unknown-key sweep (the scene's rule).
    const Result<YAML::Node> version_node = field(root, "part_version", kTop);
    if (!version_node) return std::unexpected(version_node.error());
    const Result<uint32_t> version = parse_uint<uint32_t>(*version_node, "part_version");
    if (!version) return std::unexpected(version.error());
    if (*version != kPartVersion) {
        return std::unexpected(Error{Code::schema_mismatch, "part file is schema version " + dec(*version) +
                                                                ", this build reads version " + dec(kPartVersion) +
                                                                " only" + mark_of(*version_node)});
    }
    // The kind's key next, so a file of another kind says so before its key
    // is called unknown.
    const Result<YAML::Node> block = field(root, name.c_str(), kTop);
    if (!block) return std::unexpected(block.error());
    if (Result<void> r = yaml_text::check_map(root, kTop, {"part_version", name, "sources"}, kSchema); !r) {
        return std::unexpected(r.error());
    }

    Part out{};
    KeyList keys;
    walk(keys, out);
    keys.keys.emplace_back("shape");
    if constexpr (std::is_same_v<Part, PropPart>) keys.keys.emplace_back("blade");
    if (Result<void> r = check_keys(*block, name, keys.keys); !r) return std::unexpected(r.error());

    Reader reader{*block, name, std::nullopt};
    walk(reader, out);
    if (reader.error) return std::unexpected(*reader.error);

    std::optional<vehicles::BlockShape> shape;
    if ((*block)["shape"].IsDefined()) {
        Result<vehicles::BlockShape> s = read_shape((*block)["shape"], name + ".shape");
        if (!s) return std::unexpected(s.error());
        shape = *s;
    }
    out.block.shape = shape;
    if constexpr (std::is_same_v<Part, PropPart>) {
        if ((*block)["blade"].IsDefined()) {
            Result<BladeGeometry> b = read_blade((*block)["blade"]);
            if (!b) return std::unexpected(b.error());
            out.blade = std::move(*b);
        }
    }

    Result<PartSources> sources = read_sources(root, name, cited_of(out), identity_of(out));
    if (!sources) return std::unexpected(sources.error());
    out.sources = std::move(*sources);
    return out;
}

// yaml-cpp reports every syntax failure by throwing; nothing may leave a Spade
// API that way (the scene file's rule).
template <class Part>
[[nodiscard]] Result<Part> read(std::string_view text) {
    try {
        const YAML::Node root = YAML::Load(std::string(text));
        return read_root<Part>(root);
    } catch (const YAML::Exception& e) {
        return std::unexpected(Error{Code::invalid_argument, std::string(e.what())});
    } catch (const std::exception& e) {
        return std::unexpected(Error{Code::internal, std::string("part file parse failed: ") + e.what()});
    }
}

}  // namespace

std::string_view part_kind_name(PartKind kind) noexcept {
    const auto i = static_cast<std::size_t>(kind);
    return i < kKindNames.size() ? kKindNames[i] : std::string_view{};
}

std::string_view part_suffix(PartKind kind) noexcept {
    const auto i = static_cast<std::size_t>(kind);
    return i < kSuffixes.size() ? kSuffixes[i] : std::string_view{};
}

PartKind part_kind(const PartFile& part) noexcept { return static_cast<PartKind>(part.index()); }

std::vector<std::string> cited_fields(const PartFile& part) {
    return std::visit([](const auto& p) { return cited_of(p); }, part);
}

Result<std::string> motor_to_yaml(const MotorPart& part) { return write(part); }
Result<std::string> prop_to_yaml(const PropPart& part) { return write(part); }
Result<std::string> esc_to_yaml(const EscPart& part) { return write(part); }
Result<std::string> battery_to_yaml(const BatteryPart& part) { return write(part); }

Result<std::string> part_to_yaml(const PartFile& part) {
    return std::visit([](const auto& p) { return write(p); }, part);
}

Result<MotorPart> motor_from_yaml(std::string_view text) { return read<MotorPart>(text); }
Result<PropPart> prop_from_yaml(std::string_view text) { return read<PropPart>(text); }
Result<EscPart> esc_from_yaml(std::string_view text) { return read<EscPart>(text); }
Result<BatteryPart> battery_from_yaml(std::string_view text) { return read<BatteryPart>(text); }

Result<PartFile> part_from_yaml(std::string_view text, PartKind kind) {
    const auto as_file = [](auto r) -> Result<PartFile> {
        if (!r) return std::unexpected(r.error());
        return PartFile{std::move(*r)};
    };
    switch (kind) {
        case PartKind::motor: return as_file(motor_from_yaml(text));
        case PartKind::prop: return as_file(prop_from_yaml(text));
        case PartKind::esc: return as_file(esc_from_yaml(text));
        case PartKind::battery: return as_file(battery_from_yaml(text));
    }
    return std::unexpected(invalid("part kind " + dec(static_cast<uint32_t>(kind)) + " is not one of the four"));
}

Result<void> check_part_sources(const PartFile& part, const SourceRegistry& registry) {
    const std::string kind(part_kind_name(part_kind(part)));
    return std::visit(
        [&](const auto& p) -> Result<void> {
            for (const FieldSource& s : p.sources) {
                if (Result<void> r = check_source_ref(registry, s.ref, kind + "." + s.field); !r) return r;
            }
            return {};
        },
        part);
}

Result<PartFile> load_part_file(const std::filesystem::path& path, const SourceRegistry& registry) {
    const std::string name = path_text(path.filename());
    std::size_t k = 0;
    while (k < kSuffixes.size() && !name.ends_with(kSuffixes[k])) ++k;
    if (k == kSuffixes.size()) {
        return std::unexpected(invalid(path_text(path) + ": a part file's name must end in .motor.yaml, .prop.yaml, "
                                                         ".esc.yaml or .battery.yaml"));
    }
    std::string text;
    try {
        std::ifstream in(path, std::ios::binary);
        if (!in) return std::unexpected(Error{Code::io_error, "cannot open part file '" + path_text(path) + "'"});
        text.assign(std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>());
        if (in.bad()) {
            return std::unexpected(Error{Code::io_error, "failed reading part file '" + path_text(path) + "'"});
        }
    } catch (const std::exception& e) {
        return std::unexpected(
            Error{Code::io_error, "failed reading part file '" + path_text(path) + "': " + e.what()});
    }
    Result<PartFile> part = part_from_yaml(text, static_cast<PartKind>(k));
    if (!part) return std::unexpected(Error{part.error().code, path_text(path) + ": " + part.error().context});
    if (Result<void> r = check_part_sources(*part, registry); !r) {
        return std::unexpected(Error{r.error().code, path_text(path) + ": " + r.error().context});
    }
    return part;
}

}  // namespace spade::scene
