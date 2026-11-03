#pragma once

// ---------------------------------------------------------------------------
// c8q_case.hpp -- C8q's case (docs/design/test-docs/2026-10-19-end-goal-checks.md
// §5): the ONE builder (TD-9) that the CPU ensembles (test_end_goal_c8q.cpp)
// and B8's GPU record (test_gpu_airflow_records.cpp) include. Header-only;
// it reads the library's model file under SPADE_ASSETS_DIR and the case's
// power from the sandbox (airflow_live.hpp), so its tests build with the
// sandbox.
//
// The library's reference quad, its parts cleared as the editor's airflow
// chapter flies it, held level in still air: a 0.975 m box at 128^3 (R/dx
// 10.0), h = dx/8 as C8's, air's viscosity (Physics, 2026-10-22), no floor,
// armed at the hover throttle from t = 0.
//
// Its variants:
// - rotor_0_alone, C8q's banded measure (§5, 2026-10-22): rotors 1 to 3 push
//   nothing and spread no swirl. Their thrust coefficient is the least normal
//   float, about 1e-32 N at hover, since a model refuses 0, and their torque
//   coefficient is 0. Rotor 0 keeps its own torque, as it flies. The hover
//   throttle is trimmed on the four-rotor model first, so rotor 0's speed, and
//   with it its reference, are the quad's. Only rotor 0 is judged, against a
//   quarter of the weight, so the loaded premise reads its share as it reads
//   the quad's.
// Physics' discriminating runs for the first ensemble's miss, one run each:
// - no_torque: every rotor's torque coefficient is 0, so the discs spread no
//   swirl;
// - rotor_0_alone_no_torque: rotor 0 alone, as above, with its torque at 0 too.
// ---------------------------------------------------------------------------

#include <cstdint>
#include <filesystem>
#include <limits>
#include <string>
#include <utility>

#include "analysis/case.hpp"
#include "analysis/controls.hpp"
#include "analysis/live_run.hpp"
#include "core/error.hpp"
#include "scene/model_file.hpp"
#include "vehicles/model_type.hpp"
#include "../sandbox/airflow_live.hpp"

namespace spade::testing {

inline constexpr float kC8qBoxM = 0.975f;
inline constexpr uint32_t kC8qCells = 128;
inline constexpr uint32_t kC8qFrames = 15;  // to 0.50 s: t_dev 0.30 s plus the 0.20 s window

enum class C8qVariant { as_flown, rotor_0_alone, no_torque, rotor_0_alone_no_torque };

struct C8qAir {
    vehicles::ModelType model;  // what LiveRun steps: the library quad, its parts cleared
    analysis::AnalysisCase c;
    analysis::ResolvedCase resolved;
    analysis::Controls hover;
    float judged_mass = 0.0f;   // the mass whose weight the judged rotors carry
    bool rotor_0_only = false;  // judge rotor 0's probe alone
};

// C8q's case for `variant`. Refused as the model file, the trim or resolve()
// refuses, with resolve()'s first issue.
[[nodiscard]] inline Result<C8qAir> c8q_air(C8qVariant variant = C8qVariant::as_flown) {
    namespace an = analysis;
    namespace af = sandbox::airflow;
    const std::filesystem::path file =
        std::filesystem::path(SPADE_ASSETS_DIR) / "library" / "drones" / "reference_quad.model.yaml";
    Result<scene::ModelFile> loaded = scene::load_model_file(file);
    if (!loaded) return std::unexpected(loaded.error());
    C8qAir q;
    q.model = loaded->model;
    q.model.parts.clear();
    if (variant == C8qVariant::no_torque || variant == C8qVariant::rotor_0_alone_no_torque) {
        for (auto& rotor : q.model.rotors) rotor.torque_coeff = 0.0f;
    }
    an::AnalysisCase& c = q.c;
    c.model = an::pin_model(file.generic_string(), q.model);
    c.power = an::Power{.idle = static_cast<float>(af::kIdleDuty),
                        .state_of_charge = static_cast<float>(af::kStateOfCharge)};
    c.rotors.assign(q.model.rotors.size(), an::RotorSetup{});
    c.grid.box = kC8qBoxM;
    c.grid.spacing = kC8qBoxM / static_cast<float>(kC8qCells);
    c.time.step = *c.grid.spacing / 8.0f;
    c.time.duration = static_cast<float>(kC8qFrames) / static_cast<float>(an::kFramesPerSecond);
    const Result<an::ShowcaseTrim> trim = an::showcase_trim(c, q.model);
    if (!trim) return std::unexpected(trim.error());
    q.hover = an::Controls{true, trim->hover, 0.0f, 0.0f, 0.0f};
    q.judged_mass = q.model.body.mass;
    if (variant == C8qVariant::rotor_0_alone || variant == C8qVariant::rotor_0_alone_no_torque) {
        for (std::size_t i = 1; i < q.model.rotors.size(); ++i) {
            q.model.rotors[i].thrust_coeff = std::numeric_limits<float>::min();
            q.model.rotors[i].torque_coeff = 0.0f;
        }
        c.model = an::pin_model(file.generic_string(), q.model);
        q.judged_mass /= static_cast<float>(q.model.rotors.size());
        q.rotor_0_only = true;
    }
    auto resolved = an::resolve(c, q.model);
    if (!resolved) {
        const an::CaseIssue& first = resolved.error().front();
        return std::unexpected(Error{Code::invalid_argument, "C8q's case does not resolve: " + first.field + ": " +
                                                                 first.message});
    }
    q.resolved = std::move(*resolved);
    return q;
}

}  // namespace spade::testing
