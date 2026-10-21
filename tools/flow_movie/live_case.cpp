#include "live_case.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <numbers>
#include <string>
#include <utility>

#include <glm/glm.hpp>

#include "analysis/live_run.hpp"  // showcase_trim()
#include "core/path_text.hpp"
#include "scene/model_file.hpp"
#include "scene/scene_file.hpp"
#include "vehicles/model_part.hpp"
#include "vehicles/propulsion_steady.hpp"
#include "vehicles/model_identity.hpp"

namespace flow_movie {
namespace an = spade::analysis;
namespace fs = std::filesystem;
using spade::vehicles::ModelType;

namespace {

[[nodiscard]] spade::Error invalid(std::string what) { return spade::Error{spade::Code::invalid_argument, std::move(what)}; }

[[nodiscard]] std::string hex(uint64_t v) {
    char buffer[24];
    std::snprintf(buffer, sizeof buffer, "%016llx", static_cast<unsigned long long>(v));
    return buffer;
}

// FNV-1a 64: a mark that reads the same in every build.
[[nodiscard]] uint64_t fnv(const std::string& text) {
    uint64_t h = 1469598103934665603ull;
    for (const char ch : text) {
        h ^= static_cast<unsigned char>(ch);
        h *= 1099511628211ull;
    }
    return h;
}

// The case, the program and both models, as one line the cache's mark holds.
[[nodiscard]] spade::Result<std::string> identity_of(const LiveCase& lc) {
    const auto case_text = an::case_to_yaml(lc.c);
    if (!case_text) return std::unexpected(case_text.error());
    const auto program_text = an::program_to_yaml(lc.program);
    if (!program_text) return std::unexpected(program_text.error());
    // The chain is the model's, the idle and charge the case's (INT-11).
    const std::string propulsion =
        lc.run_model.propulsion
            ? "propulsion " + hex(spade::vehicles::propulsion_identity(*lc.run_model.propulsion)) + " idle " +
                  std::to_string(lc.c.power.idle.value_or(an::kDefaultIdleDuty)) + " soc " +
                  std::to_string(lc.c.power.state_of_charge.value_or(an::kDefaultStateOfCharge))
            : std::string("unpowered");
    return "case " + hex(fnv(*case_text)) + ", controls " + hex(fnv(*program_text)) + ", model " +
           hex(spade::vehicles::model_identity(lc.run_model)) + ", parts " +
           hex(spade::vehicles::parts_identity(lc.draw_model)) + ", " + propulsion + ", " +
           std::to_string(lc.seconds) + " s";
}

}  // namespace

spade::Result<ModelType> case_model(const an::AnalysisCase& c, const fs::path& case_file) {
    const fs::path scene = case_file.parent_path() / c.model.file;
    auto loaded = spade::scene::load_scene_file(scene);
    if (!loaded) {
        return std::unexpected(spade::Error{loaded.error().code, "case_model: the case's model file " + scene.string() +
                                                                     ": " + loaded.error().context});
    }
    for (const ModelType& m : loaded->models) {
        const an::ModelRef pins = an::pin_model(c.model.file, m);
        if (pins.identity == c.model.identity && pins.parts == c.model.parts && pins.propulsion == c.model.propulsion) {
            return m;
        }
    }
    return std::unexpected(invalid("case_model: no model in " + scene.string() + " has the case's pinned identity " +
                                   hex(c.model.identity) + ", parts " + hex(c.model.parts) + " and propulsion " +
                                   hex(c.model.propulsion) +
                                   "; re-pin the case to the model it is for"));
}

spade::Result<LiveCase> live_case_from_files(const fs::path& case_file, const fs::path& controls_file, double seconds) {
    LiveCase lc;
    auto loaded = an::load_case(case_file);
    if (!loaded) return std::unexpected(loaded.error());
    lc.c = std::move(*loaded);
    auto model = case_model(lc.c, case_file);
    if (!model) return std::unexpected(model.error());
    lc.run_model = *model;
    lc.draw_model = std::move(*model);
    if (controls_file.empty()) {
        // Flown around the drone's own hover: its chain and the case's power
        // (INT-11).
        auto trim = an::showcase_trim(lc.c, lc.run_model);
        if (!trim) return std::unexpected(trim.error());
        lc.program = an::showcase_program(*trim);
        lc.program_name = "the showcase program";
        lc.seconds = seconds > 0.0 ? seconds : an::showcase_seconds();
    } else {
        auto program = an::load_program(controls_file);
        if (!program) return std::unexpected(program.error());
        lc.program = std::move(*program);
        lc.program_name = controls_file.filename().string();
        lc.seconds = seconds > 0.0 ? seconds : lc.c.time.duration;
    }
    lc.name = case_file.stem().string();
    auto id = identity_of(lc);
    if (!id) return std::unexpected(id.error());
    lc.identity = std::move(*id);
    return lc;
}

namespace {

// The reference quad as the editor places it: the library's model file
// (N4), its propulsion chain included (N5a).
constexpr const char* kLibraryQuad = "library/drones/reference_quad.model.yaml";

[[nodiscard]] spade::Result<LiveCase> powered_quad(double seconds, float box, float spacing, float step,
                                                   std::string name, std::string notes) {
    const fs::path file = fs::path(SPADE_ASSETS_DIR) / kLibraryQuad;
    auto loaded = spade::scene::load_model_file(file);
    if (!loaded) {
        return std::unexpected(spade::Error{loaded.error().code, "the reference quad: " + loaded.error().context});
    }
    if (!loaded->model.propulsion) {
        return std::unexpected(invalid("the reference quad: " + spade::path_text(file) +
                                       " has no propulsion chain, which a powered case flies on"));
    }
    LiveCase lc;
    lc.draw_model = std::move(loaded->model);
    lc.run_model = lc.draw_model;
    lc.run_model.parts.clear();
    // Idle 5 % and a full charge: the film's case carries them (INT-11).
    lc.c.power = an::Power{.idle = 0.05f, .state_of_charge = 1.0f};
    lc.c.model = an::pin_model(std::string("assets/") + kLibraryQuad, lc.run_model);
    lc.c.rotors.assign(lc.run_model.rotors.size(), an::RotorSetup{});
    lc.c.grid.box = box;
    lc.c.grid.spacing = spacing;
    lc.c.time.duration = static_cast<float>(seconds);
    lc.c.time.step = step;
    lc.c.flight.kind = an::FlightKind::hover_in_wind;
    lc.c.flight.speed = 3.0f;
    // The showcase's moves around this quad's HOVER: Interface's trim, by
    // Physics' rule (the steady state for m g / n per rotor, and the climb at
    // 1.3 times the weight), flown on its own chain (TD-9: one hover rule).
    auto trim = an::showcase_trim(lc.c, lc.run_model);
    if (!trim) return std::unexpected(trim.error());
    lc.program = an::showcase_program(*trim);
    lc.program_name = "the showcase around hover";
    lc.seconds = seconds;
    lc.name = std::move(name);
    lc.notes = std::move(notes);

    // The rotors' speed at an armed throttle, as LiveRun commands it.
    const spade::vehicles::RotorDesc& rotor = lc.run_model.rotors.front();
    const auto omega_at = [&lc](double throttle) -> spade::Result<float> {
        auto duty = spade::vehicles::armed_duty(true, throttle, static_cast<double>(*lc.c.power.idle));
        if (!duty) return std::unexpected(duty.error());
        auto cmd = spade::vehicles::rotor_omega_for_duty(*lc.run_model.propulsion, *duty, lc.c.fluid.density,
                                                         static_cast<double>(*lc.c.power.state_of_charge));
        if (!cmd) return std::unexpected(cmd.error());
        return cmd->omega_cmd;
    };
    auto at_hover = omega_at(trim->hover);
    if (!at_hover) return std::unexpected(at_hover.error());
    auto at_top = omega_at(trim->climb);
    if (!at_top) return std::unexpected(at_top.error());
    // The speed colours reach the far wake at the program's hardest thrust,
    // its climb: momentum theory's 2 v_h, v_h = sqrt(T / (2 rho pi R^2)),
    // T = k_T w^2, plus the wind.
    const float thrust = rotor.thrust_coeff * *at_top * *at_top;
    const float v_h = std::sqrt(thrust / (2.0f * lc.c.fluid.density * std::numbers::pi_v<float> * rotor.radius *
                                          rotor.radius));
    lc.speed_max = 2.0f * v_h + lc.c.flight.speed;
    lc.vortex_fraction = 1.0f / 6.0f;
    char line[220];
    std::snprintf(line, sizeof line,
                  "; hover: throttle %.3f, %.0f rad/s; thrust step: %.3f, %.0f rad/s",
                  static_cast<double>(trim->hover), *at_hover, static_cast<double>(trim->climb), *at_top);
    lc.notes += line;
    auto id = identity_of(lc);
    if (!id) return std::unexpected(id.error());
    lc.identity = std::move(*id);
    return lc;
}

}  // namespace

spade::Result<LiveCase> quad_live_case(double seconds) {
    return powered_quad(seconds, 0.8f, 0.0125f, 0.0005f, "the powered quad in a wind",
                        "actuator discs, not blades; the airframe is drawn but not yet in the flow (P2); idle 5 % "
                        "and the charge are the film's own (chain pin)");
}

spade::Result<LiveCase> smoke_live_case(double seconds) {
    return powered_quad(seconds, 0.8f, 0.1f, 0.004f, "case smoke",
                        "the powered reference quad on an 8^3 grid: the --case path end to end, not a flow to read");
}

}  // namespace flow_movie
