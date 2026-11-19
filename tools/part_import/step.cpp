#include "step.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <cstring>

#include <nlohmann/json.hpp>

namespace spade::partimport {

namespace fs = std::filesystem;
using nlohmann::json;

namespace {

[[nodiscard]] std::string trim(std::string s) {
    while (!s.empty() && std::isspace(static_cast<unsigned char>(s.back()))) s.pop_back();
    std::size_t i = 0;
    while (i < s.size() && std::isspace(static_cast<unsigned char>(s[i]))) ++i;
    return s.substr(i);
}

[[nodiscard]] Problem converter_failed(const std::string& why) { return Problem{"converter_failed", why}; }

[[nodiscard]] Expected<json> parse_object(const std::string& text) {
    json j = json::parse(text, nullptr, false);
    if (j.is_discarded() || !j.is_object()) {
        const std::string head = trim(text).substr(0, 80);
        return std::unexpected(converter_failed("the converter's answer is not JSON: '" + head + "'"));
    }
    return j;
}

[[nodiscard]] std::string text_of(const json& j, const char* key) {
    const auto it = j.find(key);
    return (it != j.end() && it->is_string()) ? it->get<std::string>() : std::string();
}

// The converter's `colour` ([r, g, b] 0..255, or null) and `colours` (how many distinct) of one object; none when the colour is absent or null.
[[nodiscard]] std::optional<SurfaceColour> read_colour(const json& j) {
    const auto c = j.find("colour");
    if (c == j.end() || !c->is_array() || c->size() != 3) return std::nullopt;
    SurfaceColour out;
    for (std::size_t k = 0; k < 3; ++k) {
        if (!(*c)[k].is_number_integer()) return std::nullopt;
        const auto v = (*c)[k].get<long long>();
        if (v < 0 || v > 255) return std::nullopt;
        out.rgb[k] = static_cast<std::uint8_t>(v);
    }
    out.colours = j.value("colours", 1u);
    return out;
}

[[nodiscard]] bool read_vec3(const json& j, const char* key, glm::dvec3& out) {
    const auto it = j.find(key);
    if (it == j.end() || !it->is_array() || it->size() != 3) return false;
    for (int i = 0; i < 3; ++i) {
        if (!(*it)[i].is_number()) return false;
        out[i] = (*it)[i].get<double>();
    }
    return true;
}

}  // namespace

double unit_metres(const std::string& unit) {
    if (unit == "m") return 1.0;
    if (unit == "mm") return 1e-3;
    if (unit == "cm") return 1e-2;
    if (unit == "um") return 1e-6;
    if (unit == "in") return 0.0254;
    if (unit == "ft") return 0.3048;
    return 0.0;
}

Expected<double> parse_length_m(const std::string& text, const std::string& flag) {
    std::size_t i = 0;
    while (i < text.size() && (std::isdigit(static_cast<unsigned char>(text[i])) || std::strchr(".eE+-", text[i]) != nullptr)) ++i;
    const std::string number = text.substr(0, i), unit = text.substr(i);
    char* end = nullptr;
    const double v = number.empty() ? 0.0 : std::strtod(number.c_str(), &end);
    const double scale = unit_metres(unit);
    if (number.empty() || end == nullptr || *end != '\0' || scale == 0.0 || !std::isfinite(v) || v <= 0.0) {
        return std::unexpected(Problem{"usage", flag + " '" + text + "' is not a positive length with a unit (m, mm, cm, um, in, ft), for example 0.1mm"});
    }
    return v * scale;
}

Expected<double> parse_angle(const std::optional<std::string>& text) {
    if (!text) return 0.5;
    char* end = nullptr;
    const double v = std::strtod(text->c_str(), &end);
    if (text->empty() || end == nullptr || *end != '\0' || !std::isfinite(v) || v <= 0.0) {
        return std::unexpected(Problem{"usage", "--angle '" + *text + "' is not a positive number of radians"});
    }
    return v;
}

const char* step_install_text() {
    return "python -m venv .venv-step && .venv-step/Scripts/python -m pip install --require-hashes -r tools/step_convert/requirements.txt "
           "(bin/python on Linux or macOS); or point SPADE_STEP_PYTHON at a python that has it, and SPADE_STEP_CONVERT at step_convert.py";
}

ToolSearch find_step_tools(const Options& options, const RunEnv& env) {
    ToolSearch s;
    const auto env_value = [&](const char* name) -> std::string {
        const char* v = env.getenv ? env.getenv(name) : nullptr;
        return v != nullptr ? std::string(v) : std::string();
    };
    std::string python;
    if (options.step_python) {
        python = *options.step_python;
        s.searched.push_back("--step-python " + python);
    } else if (const std::string e = env_value("SPADE_STEP_PYTHON"); !e.empty()) {
        python = e;
        s.searched.push_back("SPADE_STEP_PYTHON=" + python);
    } else {
        for (const char* rel : {".venv-step/Scripts/python.exe", ".venv-step/bin/python"}) {
            const fs::path p = env.exe_dir / rel;
            s.searched.push_back(p.string());
            std::error_code ec;
            if (fs::is_regular_file(p, ec)) {
                python = p.string();
                break;
            }
        }
    }
    fs::path converter;
    if (options.step_convert) {
        converter = *options.step_convert;
        s.searched.push_back("--step-convert " + converter.string());
    } else if (const std::string e = env_value("SPADE_STEP_CONVERT"); !e.empty()) {
        converter = e;
        s.searched.push_back("SPADE_STEP_CONVERT=" + e);
    } else {
        converter = env.exe_dir / "step_convert.py";
        s.searched.push_back(converter.string());
    }
    std::error_code ec;
    if (!python.empty() && fs::is_regular_file(converter, ec)) s.tools = StepTools{python, converter};
    return s;
}

ConverterOutcome run_converter(const StepTools& tools, const std::vector<std::string>& args, const RunEnv& env) {
    ProcessOptions o;
    o.argv = {tools.python, tools.converter.string()};
    o.argv.insert(o.argv.end(), args.begin(), args.end());
    o.cancelled = env.cancelled;
    const ProcessResult r = run_process(o);
    ConverterOutcome c;
    c.started = r.started;
    c.killed = r.killed;
    c.exit = r.exit;
    c.out = r.out;
    c.err = r.err;
    c.start_error = r.start_error;
    return c;
}

Problem converter_problem(const ConverterOutcome& c) {
    if (!c.started) return {"step_unavailable", "the STEP converter could not be started: " + c.start_error};
    const std::string said = trim(c.err).empty() ? "the converter exited with code " + std::to_string(c.exit) : trim(c.err);
    if (c.exit == 3 && said.find("not installed") != std::string::npos) return {"step_unavailable", said};
    if (c.exit == 3) return {"file_invalid", said};
    if (c.exit == 4) return {"mesh_refused", said};
    return converter_failed(said);
}

Expected<StepProbe> parse_probe(const std::string& json_text) {
    const Expected<json> j = parse_object(json_text);
    if (!j) return std::unexpected(j.error());
    StepProbe p;
    p.ocp = j->value("ocp", false);
    p.python = text_of(*j, "python");
    p.occt = text_of(*j, "occt");
    p.wheel = text_of(*j, "wheel");
    p.reason = text_of(*j, "reason");
    return p;
}

// The converter's optional "skipped" list: the names of the parts it left out because they hold no solid.
static std::vector<std::string> skipped_names(const json& answer) {
    std::vector<std::string> names;
    if (const auto s = answer.find("skipped"); s != answer.end() && s->is_array()) {
        for (const json& e : *s) {
            const std::string name = e.is_object() ? text_of(e, "name") : std::string();
            if (!name.empty()) names.push_back(name);
        }
    }
    return names;
}

Expected<StepInspect> parse_inspect(const std::string& json_text) {
    const Expected<json> j = parse_object(json_text);
    if (!j) return std::unexpected(j.error());
    StepInspect r;
    r.unit = text_of(*j, "unit");
    r.occt = text_of(*j, "occt");
    r.wheel = text_of(*j, "wheel");
    const double u = unit_metres(r.unit);
    const auto solids = j->find("solids");
    if (u == 0.0 || solids == j->end() || !solids->is_array()) {
        return std::unexpected(converter_failed("the converter's inspect answer has no unit or no solids"));
    }
    for (const json& s : *solids) {
        StepSolid o;
        o.name = text_of(s, "name");
        if (!s.is_object() || o.name.empty() || !s.contains("volume") || !s["volume"].is_number() || !read_vec3(s, "bbox_min", o.lo) ||
            !read_vec3(s, "bbox_max", o.hi)) {
            return std::unexpected(converter_failed("the converter's inspect answer has a solid without a name, volume or box"));
        }
        o.colour = read_colour(s);
        o.volume_m3 = s["volume"].get<double>() * u * u * u;
        o.lo *= u;
        o.hi *= u;
        r.solids.push_back(std::move(o));
    }
    r.skipped = skipped_names(*j);
    return r;
}

Expected<EmitReport> parse_emit(const std::string& json_text) {
    const Expected<json> j = parse_object(json_text);
    if (!j) return std::unexpected(j.error());
    EmitReport r;
    r.skipped = skipped_names(*j);
    r.unit = text_of(*j, "unit");
    r.occt = text_of(*j, "occt");
    r.wheel = text_of(*j, "wheel");
    if (const auto c = j->find("colours"); c != j->end() && c->is_array()) {
        for (const json& e : *c) r.colours.push_back({text_of(e, "name"), read_colour(e)});
    }
    std::vector<std::string> flagged;
    if (const auto f = j->find("flagged"); f != j->end() && f->is_array()) {
        for (const json& n : *f) {
            if (n.is_string()) flagged.push_back(n.get<std::string>());
        }
    }
    if (const auto c = j->find("cross_check"); c != j->end() && c->is_array()) {
        for (const json& e : *c) {
            CrossCheck x;
            x.name = text_of(e, "name");
            x.brep_volume = e.value("brep_volume", 0.0);
            x.mesh_volume = e.value("mesh_volume", 0.0);
            x.difference = e.value("difference", 0.0);
            x.flagged = std::find(flagged.begin(), flagged.end(), x.name) != flagged.end();
            r.checks.push_back(std::move(x));
        }
    }
    return r;
}

}  // namespace spade::partimport
