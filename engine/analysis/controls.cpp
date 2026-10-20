// A control program (see the header).

#include "analysis/controls.hpp"

#include <cmath>
#include <fstream>
#include <iterator>
#include <string>
#include <utility>

#include <yaml-cpp/yaml.h>

#include "core/path_text.hpp"
#include "world/detail/yaml_text.hpp"

namespace spade::analysis {

namespace {

namespace fs = std::filesystem;
using yaml_text::at;
using yaml_text::format_float;

constexpr std::string_view kSchema = "controls schema v1";

[[nodiscard]] std::string flag(bool b) { return b ? "true" : "false"; }

[[nodiscard]] Result<bool> bool_field(const YAML::Node& map, const char* key, std::string_view what) {
    const Result<std::string> text = yaml_text::string_field(map, key, what);
    if (!text) return std::unexpected(text.error());
    if (*text == "true") return true;
    if (*text == "false") return false;
    return std::unexpected(at(map[key], std::string(what) + "." + key + ": '" + *text + "' must be true or false"));
}

// The first issue as an error, naming its field.
[[nodiscard]] Error first_issue(const std::vector<CaseIssue>& issues) {
    return Error{Code::invalid_argument, "control program: " + issues.front().field + ": " + issues.front().message};
}

[[nodiscard]] Result<ControlProgram> parse_program(const YAML::Node& root) {
    if (!root.IsMap()) return std::unexpected(Error{Code::invalid_argument, "a control program is a map"});
    const Result<uint32_t> version = yaml_text::uint_field<uint32_t>(root, "version", "controls");
    if (!version) return std::unexpected(version.error());
    if (*version != kControlsFileVersion) {
        return std::unexpected(Error{Code::schema_mismatch, "controls version " + std::to_string(*version) +
                                                                " is not " + std::to_string(kControlsFileVersion)});
    }
    if (Result<void> r = yaml_text::check_map(root, "controls", {"version", "points"}, kSchema); !r) {
        return std::unexpected(r.error());
    }
    const Result<YAML::Node> points = yaml_text::field(root, "points", "controls");
    if (!points) return std::unexpected(points.error());
    if (!points->IsSequence()) return std::unexpected(at(*points, "controls.points must be a sequence"));
    ControlProgram p;
    for (std::size_t i = 0; i < points->size(); ++i) {
        const YAML::Node n = (*points)[i];
        const std::string what = "controls.points[" + std::to_string(i) + "]";
        if (Result<void> r = yaml_text::check_map(n, what, {"t", "armed", "throttle", "pitch_deg", "roll_deg", "yaw_deg"},
                                                  kSchema);
            !r) {
            return std::unexpected(r.error());
        }
        ControlPoint c;
        const Result<float> t = yaml_text::float_field(n, "t", what);
        const Result<bool> armed = bool_field(n, "armed", what);
        const Result<float> throttle = yaml_text::float_field(n, "throttle", what);
        const Result<float> pitch = yaml_text::float_field(n, "pitch_deg", what);
        const Result<float> roll = yaml_text::float_field(n, "roll_deg", what);
        const Result<float> yaw = yaml_text::float_field(n, "yaw_deg", what);
        for (const Error* e : {t ? nullptr : &t.error(), armed ? nullptr : &armed.error(),
                               throttle ? nullptr : &throttle.error(), pitch ? nullptr : &pitch.error(),
                               roll ? nullptr : &roll.error(), yaw ? nullptr : &yaw.error()}) {
            if (e != nullptr) return std::unexpected(*e);
        }
        c.t_s = *t;
        c.controls = Controls{*armed, *throttle, *pitch, *roll, *yaw};
        p.points.push_back(c);
    }
    if (const std::vector<CaseIssue> issues = program_issues(p); !issues.empty()) {
        return std::unexpected(first_issue(issues));
    }
    return p;
}

}  // namespace

Controls controls_at(const ControlProgram& program, float t_s) noexcept {
    Controls now{};
    for (const ControlPoint& c : program.points) {
        if (c.t_s > t_s) break;
        now = c.controls;
    }
    return now;
}

std::vector<CaseIssue> program_issues(const ControlProgram& program) {
    std::vector<CaseIssue> out;
    if (program.points.empty()) {
        out.push_back({"points", "a control program needs at least one point"});
        return out;
    }
    for (std::size_t i = 0; i < program.points.size(); ++i) {
        const ControlPoint& c = program.points[i];
        const std::string at_i = "points[" + std::to_string(i) + "].";
        if (!std::isfinite(c.t_s)) {
            out.push_back({at_i + "t", "must be finite"});
        } else if (i == 0 && c.t_s != 0.0f) {
            out.push_back({at_i + "t", "the first point is at 0 s, not " + format_float(c.t_s)});
        } else if (i > 0 && !(c.t_s > program.points[i - 1].t_s)) {
            out.push_back({at_i + "t", format_float(c.t_s) + " s must be after the point before it (" +
                                           format_float(program.points[i - 1].t_s) + " s)"});
        }
        const Controls& k = c.controls;
        if (!(k.throttle >= 0.0f && k.throttle <= 1.0f)) {
            out.push_back({at_i + "throttle", format_float(k.throttle) + " must be in [0, 1]"});
        }
        if (!std::isfinite(k.pitch_deg)) out.push_back({at_i + "pitch_deg", "must be finite"});
        if (!std::isfinite(k.roll_deg)) out.push_back({at_i + "roll_deg", "must be finite"});
        if (!std::isfinite(k.yaw_deg)) out.push_back({at_i + "yaw_deg", "must be finite"});
    }
    return out;
}

Result<std::string> program_to_yaml(const ControlProgram& program) {
    if (const std::vector<CaseIssue> issues = program_issues(program); !issues.empty()) {
        return std::unexpected(first_issue(issues));
    }
    std::string out = "version: " + std::to_string(kControlsFileVersion) + "\npoints:\n";
    for (const ControlPoint& c : program.points) {
        const Controls& k = c.controls;
        out += "  - {t: " + format_float(c.t_s) + ", armed: " + flag(k.armed) + ", throttle: " + format_float(k.throttle) +
               ", pitch_deg: " + format_float(k.pitch_deg) + ", roll_deg: " + format_float(k.roll_deg) +
               ", yaw_deg: " + format_float(k.yaw_deg) + "}\n";
    }
    return out;
}

Result<ControlProgram> program_from_yaml(std::string_view text) {
    try {
        return parse_program(YAML::Load(std::string(text)));
    } catch (const YAML::Exception& e) {
        return std::unexpected(Error{Code::invalid_argument, std::string("control program: ") + e.what()});
    } catch (const std::exception& e) {
        return std::unexpected(Error{Code::internal, std::string("control program parse failed: ") + e.what()});
    }
}

Result<void> save_program(const ControlProgram& program, const fs::path& file) {
    const Result<std::string> text = program_to_yaml(program);
    if (!text) return std::unexpected(text.error());
    std::ofstream out(file, std::ios::binary | std::ios::trunc);
    if (!out) return std::unexpected(Error{Code::io_error, "cannot write control program '" + path_text(file) + "'"});
    out << *text;
    out.close();
    if (!out) return std::unexpected(Error{Code::io_error, "failed writing control program '" + path_text(file) + "'"});
    return {};
}

Result<ControlProgram> load_program(const fs::path& file) {
    std::ifstream in(file, std::ios::binary);
    if (!in) return std::unexpected(Error{Code::io_error, "cannot open control program '" + path_text(file) + "'"});
    const std::string text((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
    if (in.bad()) return std::unexpected(Error{Code::io_error, "failed reading control program '" + path_text(file) + "'"});
    return program_from_yaml(text);
}

ControlProgram showcase_program(const ShowcaseTrim& trim) {
    const float h = trim.hover;
    return ControlProgram{{
        {0.0f, Controls{false, 0.0f, 0.0f, 0.0f, 0.0f}},        // at rest, disarmed
        {0.2f, Controls{true, h, 0.0f, 0.0f, 0.0f}},            // arm at hover: the spin-up sheds the start-up ring
        {1.2f, Controls{true, h, 15.0f, 0.0f, 0.0f}},           // pitch
        {1.8f, Controls{true, h, 0.0f, 15.0f, 0.0f}},           // roll
        {2.4f, Controls{true, trim.climb, 0.0f, 0.0f, 0.0f}},   // climb
        {3.0f, Controls{true, h, 0.0f, 0.0f, 45.0f}},           // yaw
    }};
}

float showcase_seconds() noexcept { return 3.6f; }

}  // namespace spade::analysis
