#include "live_case.hpp"

#include <cstdint>
#include <cstdio>
#include <string>
#include <utility>

#include <glm/glm.hpp>

#include "analysis/live_run.hpp"  // showcase_trim()
#include "scene/scene_file.hpp"
#include "vehicles/model_part.hpp"
#include "vehicles/model_identity.hpp"
#include "world/sdf.hpp"

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
    return "case " + hex(fnv(*case_text)) + ", controls " + hex(fnv(*program_text)) + ", model " +
           hex(spade::vehicles::model_identity(lc.run_model)) + ", parts " +
           hex(spade::vehicles::parts_identity(lc.draw_model)) + ", " + std::to_string(lc.seconds) + " s";
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
        if (spade::vehicles::model_identity(m) == c.model.identity && spade::vehicles::parts_identity(m) == c.model.parts) {
            return m;
        }
    }
    return std::unexpected(invalid("case_model: no model in " + scene.string() + " has the case's pinned identity " +
                                   hex(c.model.identity) + " and parts " + hex(c.model.parts) +
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
        // Flown around the drone's own hover. Until the case pins its chain
        // (INT-11) there is no propulsion here, so a model with rotors is
        // refused as LiveRun::start() would refuse it.
        auto trim = an::showcase_trim(lc.c, lc.run_model, std::nullopt);
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

spade::Result<LiveCase> smoke_live_case(double seconds) {
    const fs::path scene = fs::path(SPADE_ASSETS_DIR) / "scenes" / "hover.scene.yaml";
    auto loaded = spade::scene::load_scene_file(scene);
    if (!loaded) return std::unexpected(loaded.error());
    if (loaded->models.empty()) return std::unexpected(invalid("smoke case: " + scene.string() + " has no model"));
    LiveCase lc;
    lc.draw_model = loaded->models.front();
    lc.run_model = lc.draw_model;
    lc.run_model.parts.clear();
    lc.run_model.rotors.clear();
    // The hover quad has no parts; the smoke draws a plain hull box on it, so
    // the per-frame drone (render_parts) runs end to end. The fluid never
    // sees it (the run model has no parts).
    if (lc.draw_model.parts.empty()) {
        spade::vehicles::PartDesc hull;
        hull.name = "smoke hull";
        hull.role = "body";
        hull.shape.transforms.push_back(spade::SdfTransform{});
        spade::SdfNode box{};
        box.kind = static_cast<uint32_t>(spade::SdfPrim::box);
        box.params = glm::vec4(0.12f, 0.03f, 0.12f, 0.0f);
        hull.shape.nodes.push_back(box);
        hull.min_feature = 0.06f;
        lc.draw_model.parts.push_back(std::move(hull));
    }
    lc.c.model = an::ModelRef{"hover.scene.yaml", spade::vehicles::model_identity(lc.run_model),
                              spade::vehicles::parts_identity(lc.run_model)};
    lc.c.rotors.assign(lc.run_model.rotors.size(), an::RotorSetup{});
    lc.c.grid.box = 0.8f;
    lc.c.grid.spacing = 0.1f;
    lc.c.time.duration = static_cast<float>(seconds);
    lc.c.time.step = 0.004f;
    lc.c.flight.kind = an::FlightKind::forward;
    lc.c.flight.speed = 3.0f;
    auto trim = an::showcase_trim(lc.c, lc.run_model, std::nullopt);  // no rotors: a zero trim
    if (!trim) return std::unexpected(trim.error());
    lc.program = an::showcase_program(*trim);
    lc.program_name = "the showcase program";
    lc.seconds = seconds;
    lc.name = "case smoke";
    lc.notes = "the hover quad without its rotors (until they couple), drawn as a plain hull the fluid does not see "
               "(until P2): wind past a held body";
    auto id = identity_of(lc);
    if (!id) return std::unexpected(id.error());
    lc.identity = std::move(*id);
    return lc;
}

}  // namespace flow_movie
