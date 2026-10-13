// The analysis case's file, `.case.yaml` (see the header). Written by hand in
// one fixed order through world/detail/yaml_text.hpp, as scene_file.cpp is,
// so the same case is the same bytes; read with yaml-cpp, every key checked.

#include <array>
#include <charconv>
#include <initializer_list>
#include <fstream>
#include <iterator>
#include <optional>
#include <string>
#include <utility>

#include <yaml-cpp/yaml.h>

#include "analysis/case.hpp"
#include "world/detail/yaml_text.hpp"

namespace spade::analysis {

namespace {

using yaml_text::at;
using yaml_text::dec;
using yaml_text::field;
using yaml_text::float_field;
using yaml_text::float_list;
using yaml_text::format_float;
using yaml_text::has;
using yaml_text::path_text;
using yaml_text::quote_yaml;
using yaml_text::scalar;
using yaml_text::string_field;
using yaml_text::uint_field;

constexpr std::string_view kSchema = "case schema v1";

constexpr std::string_view kFileHeader =
    "# A Spade analysis case (.case.yaml): one drone held at the centre of a box\n"
    "# of air. Written by spade::analysis (analysis/case.hpp); read and write it\n"
    "# through that API. Lengths in metres, times in seconds, angles in degrees.\n";

constexpr std::array<std::string_view, 3> kFlightNames = {"hover", "forward", "hover_in_wind"};
constexpr std::array<std::string_view, 3> kRotorModelNames = {"disc", "line", "blade_resolved"};

[[nodiscard]] std::string hex64(uint64_t value) {
    static constexpr char kHex[] = "0123456789abcdef";
    std::string out = "0x";
    for (int shift = 60; shift >= 0; shift -= 4) out += kHex[(value >> shift) & 0xfu];
    return out;
}

[[nodiscard]] std::string flag(bool b) { return b ? "true" : "false"; }

// ---------------------------------------------------------------------------
// Writing
// ---------------------------------------------------------------------------

[[nodiscard]] std::string emit(const AnalysisCase& c) {
    std::string out(kFileHeader);
    out += "version: " + dec(kCaseFileVersion) + "\n";
    out += "model:\n";
    out += "  file: " + quote_yaml(c.model.file) + "\n";
    out += "  identity: " + quote_yaml(hex64(c.model.identity)) + "\n";
    out += "  parts: " + quote_yaml(hex64(c.model.parts)) + "\n";
    out += "controls:\n";
    out += "  armed: " + flag(c.controls.armed) + "\n";
    out += "  throttle: " + format_float(c.controls.throttle) + "\n";
    out += "  pitch_deg: " + format_float(c.controls.pitch_deg) + "\n";
    out += "  roll_deg: " + format_float(c.controls.roll_deg) + "\n";
    out += "  yaw_deg: " + format_float(c.controls.yaw_deg) + "\n";
    out += "flight:\n";
    out += "  kind: " + quote_yaml(kFlightNames[static_cast<std::size_t>(c.flight.kind)]) + "\n";
    out += "  speed: " + format_float(c.flight.speed) + "\n";
    out += "  heading_deg: " + format_float(c.flight.heading_deg) + "\n";
    if (c.flight.floor_height) out += "  floor_height: " + format_float(*c.flight.floor_height) + "\n";
    out += "fluid:\n";
    out += "  density: " + format_float(c.fluid.density) + "\n";
    out += "  kinematic_viscosity: " + format_float(c.fluid.kinematic_viscosity) + "\n";
    out += "grid:\n";
    if (c.grid.box) out += "  box: " + format_float(*c.grid.box) + "\n";
    if (c.grid.spacing) out += "  spacing: " + format_float(*c.grid.spacing) + "\n";
    out += "  nests: " + dec(c.grid.nests) + "\n";
    out += c.rotors.empty() ? "rotors: []\n" : "rotors:\n";
    for (const RotorSetup& r : c.rotors) {
        out += "  - model: " + quote_yaml(kRotorModelNames[static_cast<std::size_t>(r.model)]) + "\n";
        out += "    blade: " + quote_yaml(r.blade) + "\n";
    }
    out += c.blades.empty() ? "blades: []\n" : "blades:\n";
    for (const BladeDefinition& b : c.blades) {
        out += "  - name: " + quote_yaml(b.name) + "\n";
        out += "    blade_count: " + dec(b.blade_count) + "\n";
        out += "    hub_radius: " + format_float(b.hub_radius) + "\n";
        out += "    collective_deg: " + format_float(b.collective_deg) + "\n";
        out += b.stations.empty() ? "    stations: []\n" : "    stations:\n";
        for (const BladeStation& s : b.stations) {
            out += "      - r_over_radius: " + format_float(s.r_over_radius) + "\n";
            out += "        chord: " + format_float(s.chord) + "\n";
            out += "        twist_deg: " + format_float(s.twist_deg) + "\n";
            out += "        airfoil: " + quote_yaml(s.airfoil) + "\n";
        }
    }
    out += c.airfoils.empty() ? "airfoils: []\n" : "airfoils:\n";
    for (const Airfoil& a : c.airfoils) {
        out += "  - id: " + quote_yaml(a.id) + "\n";
        out += "    source: " + quote_yaml(a.source) + "\n";
        out += a.polars.empty() ? "    polars: []\n" : "    polars:\n";
        for (const Polar& p : a.polars) {
            out += "      - reynolds: " + format_float(p.reynolds) + "\n";
            out += "        alpha_deg: " + float_list(p.alpha_deg) + "\n";
            out += "        cl: " + float_list(p.cl) + "\n";
            out += "        cd: " + float_list(p.cd) + "\n";
        }
    }
    out += "time:\n";
    out += "  duration: " + format_float(c.time.duration) + "\n";
    out += "  startup: " + format_float(c.time.startup) + "\n";
    if (c.time.step) out += "  step: " + format_float(*c.time.step) + "\n";
    const Outputs& o = c.outputs;
    out += o.history_every || o.snapshot_every ? "outputs:\n" : "outputs: {}\n";
    if (o.history_every) out += "  history_every: " + format_float(*o.history_every) + "\n";
    if (o.snapshot_every) out += "  snapshot_every: " + format_float(*o.snapshot_every) + "\n";
    return out;
}

// ---------------------------------------------------------------------------
// Reading
// ---------------------------------------------------------------------------

[[nodiscard]] Result<void> check_map(const YAML::Node& node, std::string_view what,
                                     std::initializer_list<std::string_view> allowed) {
    return yaml_text::check_map(node, what, allowed, kSchema);
}

[[nodiscard]] Result<YAML::Node> map_field(const YAML::Node& map, const char* key, std::string_view what,
                                           std::initializer_list<std::string_view> allowed) {
    Result<YAML::Node> node = field(map, key, what);
    if (!node) return std::unexpected(node.error());
    if (Result<void> r = check_map(*node, std::string(what) + "." + key, allowed); !r) return std::unexpected(r.error());
    return node;
}

[[nodiscard]] Result<YAML::Node> sequence_field(const YAML::Node& map, const char* key, std::string_view what) {
    Result<YAML::Node> node = field(map, key, what);
    if (!node) return std::unexpected(node.error());
    if (!node->IsSequence()) {
        return std::unexpected(at(*node, std::string(what) + "." + key + " must be a sequence"));
    }
    return node;
}

[[nodiscard]] Result<bool> bool_field(const YAML::Node& map, const char* key, std::string_view what) {
    const Result<std::string> text = string_field(map, key, what);
    if (!text) return std::unexpected(text.error());
    if (*text == "true") return true;
    if (*text == "false") return false;
    return std::unexpected(at(map[key], std::string(what) + "." + key + ": '" + *text + "' must be true or false"));
}

[[nodiscard]] Result<std::vector<float>> floats_seq(const YAML::Node& map, const char* key, std::string_view what) {
    const Result<YAML::Node> node = sequence_field(map, key, what);
    if (!node) return std::unexpected(node.error());
    std::vector<float> out;
    out.reserve(node->size());
    for (std::size_t i = 0; i < node->size(); ++i) {
        const Result<float> v = yaml_text::parse_float((*node)[i], std::string(what) + "." + key + "[" + dec(i) + "]");
        if (!v) return std::unexpected(v.error());
        out.push_back(*v);
    }
    return out;
}

template <std::size_t N>
[[nodiscard]] Result<uint32_t> name_field(const YAML::Node& map, const char* key, std::string_view what,
                                          const std::array<std::string_view, N>& names) {
    const Result<std::string> text = string_field(map, key, what);
    if (!text) return std::unexpected(text.error());
    for (std::size_t i = 0; i < N; ++i) {
        if (names[i] == *text) return static_cast<uint32_t>(i);
    }
    std::string known;
    for (const std::string_view n : names) known += (known.empty() ? "" : ", ") + std::string(n);
    return std::unexpected(at(map[key], std::string(what) + "." + key + ": '" + *text + "' is not one of " + known));
}

[[nodiscard]] Result<uint64_t> parse_hash(const YAML::Node& node, std::string_view what) {
    const Result<std::string> text = scalar(node, what);
    if (!text) return std::unexpected(text.error());
    const std::string_view body(*text);
    const auto refuse = [&] {
        return std::unexpected(at(node, std::string(what) + ": '" + *text + "' must be \"0x\" followed by 16 hex digits"));
    };
    if (body.size() != 18 || body[0] != '0' || body[1] != 'x') return refuse();
    uint64_t value = 0;
    const std::from_chars_result r = std::from_chars(body.data() + 2, body.data() + body.size(), value, 16);
    if (r.ec != std::errc{} || r.ptr != body.data() + body.size()) return refuse();
    return value;
}

// An optional float: absent from the file, absent here.
[[nodiscard]] Result<std::optional<float>> optional_float(const YAML::Node& map, const char* key, std::string_view what) {
    if (!has(map, key)) return std::optional<float>{};
    const Result<float> v = float_field(map, key, what);
    if (!v) return std::unexpected(v.error());
    return std::optional<float>(*v);
}

// Each `TRY(x, expr)` binds the value of a Result or returns its error.
#define SPADE_CASE_TRY(name, expr)                                  \
    auto name##_r = (expr);                                         \
    if (!name##_r) return std::unexpected(name##_r.error());        \
    auto name = std::move(*name##_r)

[[nodiscard]] Result<AnalysisCase> parse_case(const YAML::Node& root) {
    if (!root.IsMap()) return std::unexpected(at(root, "a case file must be a mapping"));
    // The version first: a newer file's keys are a schema mismatch, not unknown keys.
    SPADE_CASE_TRY(version, uint_field<uint32_t>(root, "version", "case"));
    if (version != kCaseFileVersion) {
        return std::unexpected(Error{Code::schema_mismatch, "case: version " + dec(version) + "; this reader knows " +
                                                                std::string(kSchema) + " only" + yaml_text::mark_of(root)});
    }
    if (Result<void> r = check_map(root, "case", {"version", "model", "controls", "flight", "fluid", "grid", "rotors",
                                                  "blades", "airfoils", "time", "outputs"});
        !r) {
        return std::unexpected(r.error());
    }
    AnalysisCase c;

    SPADE_CASE_TRY(model, map_field(root, "model", "case", {"file", "identity", "parts"}));
    SPADE_CASE_TRY(model_file, string_field(model, "file", "case.model"));
    SPADE_CASE_TRY(identity_node, field(model, "identity", "case.model"));
    SPADE_CASE_TRY(identity, parse_hash(identity_node, "case.model.identity"));
    SPADE_CASE_TRY(parts_node, field(model, "parts", "case.model"));
    SPADE_CASE_TRY(parts, parse_hash(parts_node, "case.model.parts"));
    c.model = ModelRef{model_file, identity, parts};

    SPADE_CASE_TRY(controls, map_field(root, "controls", "case", {"armed", "throttle", "pitch_deg", "roll_deg", "yaw_deg"}));
    SPADE_CASE_TRY(armed, bool_field(controls, "armed", "case.controls"));
    SPADE_CASE_TRY(throttle, float_field(controls, "throttle", "case.controls"));
    SPADE_CASE_TRY(pitch, float_field(controls, "pitch_deg", "case.controls"));
    SPADE_CASE_TRY(roll, float_field(controls, "roll_deg", "case.controls"));
    SPADE_CASE_TRY(yaw, float_field(controls, "yaw_deg", "case.controls"));
    c.controls = Controls{armed, throttle, pitch, roll, yaw};

    SPADE_CASE_TRY(flight, map_field(root, "flight", "case", {"kind", "speed", "heading_deg", "floor_height"}));
    SPADE_CASE_TRY(kind, name_field(flight, "kind", "case.flight", kFlightNames));
    SPADE_CASE_TRY(speed, float_field(flight, "speed", "case.flight"));
    SPADE_CASE_TRY(heading, float_field(flight, "heading_deg", "case.flight"));
    SPADE_CASE_TRY(floor_height, optional_float(flight, "floor_height", "case.flight"));
    c.flight = Flight{static_cast<FlightKind>(kind), speed, heading, floor_height};

    SPADE_CASE_TRY(fluid, map_field(root, "fluid", "case", {"density", "kinematic_viscosity"}));
    SPADE_CASE_TRY(density, float_field(fluid, "density", "case.fluid"));
    SPADE_CASE_TRY(nu, float_field(fluid, "kinematic_viscosity", "case.fluid"));
    c.fluid = Fluid{density, nu};

    SPADE_CASE_TRY(grid, map_field(root, "grid", "case", {"box", "spacing", "nests"}));
    SPADE_CASE_TRY(box, optional_float(grid, "box", "case.grid"));
    SPADE_CASE_TRY(spacing, optional_float(grid, "spacing", "case.grid"));
    SPADE_CASE_TRY(nests, uint_field<uint32_t>(grid, "nests", "case.grid"));
    c.grid = GridSetup{box, spacing, nests};

    SPADE_CASE_TRY(rotors, sequence_field(root, "rotors", "case"));
    for (std::size_t i = 0; i < rotors.size(); ++i) {
        const std::string what = "case.rotors[" + dec(i) + "]";
        const YAML::Node r = rotors[i];
        if (Result<void> ok = check_map(r, what, {"model", "blade"}); !ok) return std::unexpected(ok.error());
        SPADE_CASE_TRY(rmodel, name_field(r, "model", what, kRotorModelNames));
        SPADE_CASE_TRY(blade, string_field(r, "blade", what));
        c.rotors.push_back(RotorSetup{static_cast<RotorModel>(rmodel), blade});
    }

    SPADE_CASE_TRY(blades, sequence_field(root, "blades", "case"));
    for (std::size_t i = 0; i < blades.size(); ++i) {
        const std::string what = "case.blades[" + dec(i) + "]";
        const YAML::Node b = blades[i];
        if (Result<void> ok =
                check_map(b, what, {"name", "blade_count", "hub_radius", "collective_deg", "stations"});
            !ok) {
            return std::unexpected(ok.error());
        }
        BladeDefinition def;
        SPADE_CASE_TRY(name, string_field(b, "name", what));
        SPADE_CASE_TRY(count, uint_field<uint32_t>(b, "blade_count", what));
        SPADE_CASE_TRY(hub, float_field(b, "hub_radius", what));
        SPADE_CASE_TRY(collective, float_field(b, "collective_deg", what));
        def.name = name;
        def.blade_count = count;
        def.hub_radius = hub;
        def.collective_deg = collective;
        SPADE_CASE_TRY(stations, sequence_field(b, "stations", what));
        for (std::size_t k = 0; k < stations.size(); ++k) {
            const std::string at_k = what + ".stations[" + dec(k) + "]";
            const YAML::Node s = stations[k];
            if (Result<void> ok = check_map(s, at_k, {"r_over_radius", "chord", "twist_deg", "airfoil"}); !ok) {
                return std::unexpected(ok.error());
            }
            SPADE_CASE_TRY(r_over, float_field(s, "r_over_radius", at_k));
            SPADE_CASE_TRY(chord, float_field(s, "chord", at_k));
            SPADE_CASE_TRY(twist, float_field(s, "twist_deg", at_k));
            SPADE_CASE_TRY(airfoil, string_field(s, "airfoil", at_k));
            def.stations.push_back(BladeStation{r_over, chord, twist, airfoil});
        }
        c.blades.push_back(std::move(def));
    }

    SPADE_CASE_TRY(airfoils, sequence_field(root, "airfoils", "case"));
    for (std::size_t i = 0; i < airfoils.size(); ++i) {
        const std::string what = "case.airfoils[" + dec(i) + "]";
        const YAML::Node a = airfoils[i];
        if (Result<void> ok = check_map(a, what, {"id", "source", "polars"}); !ok) return std::unexpected(ok.error());
        Airfoil foil;
        SPADE_CASE_TRY(id, string_field(a, "id", what));
        SPADE_CASE_TRY(source, string_field(a, "source", what));
        foil.id = id;
        foil.source = source;
        SPADE_CASE_TRY(polars, sequence_field(a, "polars", what));
        for (std::size_t k = 0; k < polars.size(); ++k) {
            const std::string at_k = what + ".polars[" + dec(k) + "]";
            const YAML::Node p = polars[k];
            if (Result<void> ok = check_map(p, at_k, {"reynolds", "alpha_deg", "cl", "cd"}); !ok) {
                return std::unexpected(ok.error());
            }
            SPADE_CASE_TRY(re, float_field(p, "reynolds", at_k));
            SPADE_CASE_TRY(alpha, floats_seq(p, "alpha_deg", at_k));
            SPADE_CASE_TRY(cl, floats_seq(p, "cl", at_k));
            SPADE_CASE_TRY(cd, floats_seq(p, "cd", at_k));
            foil.polars.push_back(Polar{re, std::move(alpha), std::move(cl), std::move(cd)});
        }
        c.airfoils.push_back(std::move(foil));
    }

    SPADE_CASE_TRY(timing, map_field(root, "time", "case", {"duration", "startup", "step"}));
    SPADE_CASE_TRY(duration, float_field(timing, "duration", "case.time"));
    SPADE_CASE_TRY(startup, float_field(timing, "startup", "case.time"));
    SPADE_CASE_TRY(step, optional_float(timing, "step", "case.time"));
    c.time = TimeSetup{duration, startup, step};

    SPADE_CASE_TRY(outputs, map_field(root, "outputs", "case", {"history_every", "snapshot_every"}));
    SPADE_CASE_TRY(history, optional_float(outputs, "history_every", "case.outputs"));
    SPADE_CASE_TRY(snapshot, optional_float(outputs, "snapshot_every", "case.outputs"));
    c.outputs = Outputs{history, snapshot};
    return c;
}

#undef SPADE_CASE_TRY

}  // namespace

Result<std::string> case_to_yaml(const AnalysisCase& c) {
    if (Result<void> ok = validate_case(c); !ok) return std::unexpected(ok.error());
    return emit(c);
}

// yaml-cpp reports a syntax failure by throwing; nothing leaves a Spade API
// that way.
Result<AnalysisCase> case_from_yaml(std::string_view text) {
    Result<AnalysisCase> c = [&]() -> Result<AnalysisCase> {
        try {
            return parse_case(YAML::Load(std::string(text)));
        } catch (const YAML::Exception& e) {
            return std::unexpected(Error{Code::invalid_argument, std::string(e.what())});
        } catch (const std::exception& e) {
            return std::unexpected(Error{Code::internal, std::string("case file parse failed: ") + e.what()});
        }
    }();
    if (!c) return c;
    if (Result<void> ok = validate_case(*c); !ok) return std::unexpected(ok.error());
    return c;
}

Result<AnalysisCase> load_case(const std::filesystem::path& file) {
    std::string text;
    try {
        std::ifstream in(file, std::ios::binary);
        if (!in) return std::unexpected(Error{Code::io_error, "cannot open case file '" + path_text(file) + "'"});
        text.assign(std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>());
        if (in.bad()) return std::unexpected(Error{Code::io_error, "failed reading case file '" + path_text(file) + "'"});
    } catch (const std::exception& e) {
        return std::unexpected(Error{Code::io_error, "failed reading case file '" + path_text(file) + "': " + e.what()});
    }
    Result<AnalysisCase> c = case_from_yaml(text);
    if (!c) return std::unexpected(Error{c.error().code, path_text(file) + ": " + c.error().context});
    return c;
}

Result<void> save_case(const AnalysisCase& c, const std::filesystem::path& file) {
    const Result<std::string> text = case_to_yaml(c);
    if (!text) return std::unexpected(text.error());
    try {
        std::ofstream out(file, std::ios::binary | std::ios::trunc);
        if (!out) return std::unexpected(Error{Code::io_error, "cannot write case file '" + path_text(file) + "'"});
        out << *text;
        out.flush();
        if (!out) return std::unexpected(Error{Code::io_error, "failed writing case file '" + path_text(file) + "'"});
    } catch (const std::exception& e) {
        return std::unexpected(Error{Code::io_error, "failed writing case file '" + path_text(file) + "': " + e.what()});
    }
    return {};
}

}  // namespace spade::analysis
