#pragma once

// ---------------------------------------------------------------------------
// solid_drone_rig -- the reference quad's airframe as solids in the airflow,
// for the flow views (D1's drawing side; the lead, 2026-10-21). The library's
// model file, held at hover at the box centre in a 3 m/s wind along +x, its
// parts made solids by P2's one call (set_solids) and stepped a few times, as
// LiveRun will do once Interface wires it; the volume published with the
// solid mask (P6), so the first solid layer holds its fluid side's values and
// deeper solid cells are no-data. Physics' numbers: dx 4 mm classifies all
// ten parts as walls with no leak (5 mm announces two, 6 mm is marginal).
// The scene draws the same pose through render_parts, walls flagged.
// ---------------------------------------------------------------------------

#include <algorithm>
#include <array>
#include <cstdint>
#include <filesystem>
#include <string>
#include <vector>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include "airflow_vehicle_sampler.hpp"  // Physics' adapter; the engine's once it moves (TD-9)
#include "core/flow_volume.hpp"
#include "physics/airflow/fluid_step.hpp"
#include "physics/airflow/mac_grid.hpp"
#include "physics/airflow/publish.hpp"
#include "render/scene.hpp"
#include "render_parts/render_parts.hpp"
#include "scene/model_file.hpp"
#include "state/layout.hpp"
#include "vehicles/part_pose.hpp"

namespace spade::test {

inline constexpr float kSolidDroneDx = 0.004f;
inline constexpr float kSolidDroneWind = 3.0f;  // m/s, along +x
inline constexpr uint32_t kSolidDroneSteps = 3;

struct SolidDrone {
    vehicles::ModelType model;
    FlowVolume volume;
    render::RenderScene scene;  // the posed parts, walls flagged, no ground
    uint32_t extrapolated = 0;  // publish's count of first-layer solid cells
    float q_ref = 0.0f;         // 1/2 rho U^2
    std::string run_line;       // set_solids()'s announcement (L6)
    std::string error;          // empty on success: what failed, by name
};

// The quad's COM sits at the world origin; the grid is 96 x 32 x 96 cells
// (0.384 x 0.128 x 0.384 m) around it, every face open at the wind.
[[nodiscard]] inline SolidDrone solid_drone() {
    namespace af = physics::airflow;
    SolidDrone out;
    const std::filesystem::path file =
        std::filesystem::path(SPADE_ASSETS_DIR) / "library" / "drones" / "reference_quad.model.yaml";
    auto loaded = scene::load_model_file(file);
    if (!loaded) {
        out.error = "the library quad: " + loaded.error().context;
        return out;
    }
    out.model = std::move(loaded->model);

    const af::GridShape shape{96, 32, 96, kSolidDroneDx};
    const glm::vec3 origin = -0.5f * kSolidDroneDx * glm::vec3(96.0f, 32.0f, 96.0f);
    af::DomainBc bc;
    bc.ambient = glm::vec3(kSolidDroneWind, 0.0f, 0.0f);
    const af::FluidConfig cfg{.rho = 1.225f, .nu = 0.0f, .h = 0.5f * kSolidDroneDx / kSolidDroneWind,
                              .n_v = af::kSolidsCgIterations, .interior = af::SolidInterior::extrapolate};
    out.q_ref = 0.5f * cfg.rho * kSolidDroneWind * kSolidDroneWind;

    BodyState body{};
    body.pos = glm::vec3(0.0f);
    body.orient = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);  // level, at hover
    const std::array<float, 4> still{};
    auto posed = vehicles::posed_parts(out.model, body, still, still, vehicles::PartFilter::rigid_only);
    if (!posed) {
        out.error = "posed_parts: " + posed.error().context;
        return out;
    }
    auto grid = af::make_grid(shape, bc);
    if (!grid) {
        out.error = "make_grid: " + grid.error().context;
        return out;
    }
    af::MacGrid& g = *grid;
    std::fill(g.u.begin(), g.u.end(), kSolidDroneWind);
    auto scratch = af::make_fluid_scratch(g, bc, cfg);
    if (!scratch) {
        out.error = "make_fluid_scratch: " + scratch.error().context;
        return out;
    }
    af::FluidScratch& s = *scratch;
    const airflow_test::VehicleSampler sampler(out.model, *posed, origin);
    auto parts = sampler.parts();
    if (!parts) {
        out.error = "the sampler's parts: " + parts.error().context;
        return out;
    }
    if (auto set = af::set_solids(s, g, bc, *parts, sampler.sampler()); !set) {
        out.error = "set_solids: " + set.error().context;
        return out;
    }
    out.run_line = s.solids.run_line;
    for (uint32_t i = 0; i < kSolidDroneSteps; ++i) {
        const af::FluidHealth health = af::fluid_step(g, bc, cfg, s);
        if (health.fault != af::FluidFault::none) {
            out.error = "fluid_step " + std::to_string(i) + " faulted";
            return out;
        }
    }
    auto published = af::publish_render_volume(g, {.origin = origin, .solid = s.regions.solid}, out.volume);
    if (!published) {
        out.error = "publish_render_volume: " + published.error().context;
        return out;
    }
    out.extrapolated = published->extrapolated;

    out.scene.materials.assign(1, render::Material{});
    out.scene.bounds = render::Aabb{.min = glm::vec3(-1.0f), .max = glm::vec3(1.0f)};
    auto prepared = render_parts::prepare_parts(out.model);
    if (!prepared) {
        out.error = "prepare_parts: " + prepared.error().context;
        return out;
    }
    if (auto bound = render_parts::bind_parts(*prepared, *posed, render_parts::PartsLook{}, out.scene); !bound) {
        out.error = "bind_parts: " + bound.error().context;
        return out;
    }
    return out;
}

}  // namespace spade::test
