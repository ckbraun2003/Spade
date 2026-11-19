// ---------------------------------------------------------------------------
// test_gpu_airflow_line_stills.cpp -- L86i: the camera quadcopter (phantom3_class, its fitted UIUC APC 10x5) on the deep D1 box with
// its rotors as ACTUATOR LINES (P4c) and the tip-ring dye (P10), on the device: the stills that show the blades' helices.
//
//   GpuAirflowLineStills.ThePhantomsLinesOnTheDeepBox   opt-in: SPADE_LINE_STILLS_OUT=<folder> (GPU, minutes)
//
// What it builds is what the web solve will be: the case as tools/flow_movie's still-air clip resolves it (hover in no wind, 0.2 ms
// step, Cs 0.17, the quad's rigid parts as solids, the deep grid), the rows LiveRun holds at hover, one line a rotor from
// vehicles::rotor_lines() with the part's blade and polar (scene::line_propeller()), the dye released at the rotors' discs (zero load:
// the lines carry the force). Knobs: SPADE_LINE_STILLS_STEPS (default "300,700,1200,1800"), SPADE_LINE_STILLS_NSUB (8),
// SPADE_LS_DIAG=1 (L86p, the step-size diagnostic): per sample step, the pressure solve's health since the last sample (largest residual, mean
// iterations, steps capped / unconverged / stalled), the vertical momentum flux through four planes under the hubs (rho sum w^2 dA, the mass flux
// and the pressure's sum, all in newtons or kg/s), and the tip's travel a step; SPADE_LS_STEP (the time step, s; 0.0002), SPADE_LINE_STILLS_FILE (the model, library/drones/phantom3_class.model.yaml), SPADE_LS_VIEWS (a, b, c; default a), SPADE_LS_DIST, SPADE_LS_AIM, SPADE_DYE_SIGMA,
// SPADE_DYE_ONSET (flow_still.hpp), SPADE_LINE_STILLS_DISCS=1 (the same run on discs, the baseline),
// SPADE_LS_RING=1 (the disc tip ring too), SPADE_TIP_GAIN (4), SPADE_TIP_FRAC (1). L106's frame cache: SPADE_LINE_STILLS_CACHE=<dir> with
// SPADE_LINE_STILLS_KEEP="49,99" (the frames; frame f is t = (f + 1) x 1 ms) and SPADE_LINE_STILLS_MEAN=N (the N frames ending at each) writes just those
// frames, in flow_movie's cache format, and nothing else; SPADE_LS_STEP is then a whole number of ns whose steps in 1 ms are whole.
// ---------------------------------------------------------------------------

#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include "core/fp16.hpp"
#include <filesystem>
#include <fstream>
#include <iterator>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <glm/glm.hpp>

#include "b7_harness.hpp"
#include "compute/backend.hpp"
#include "compute/vulkan/context.hpp"
#include "d1_case.hpp"
#include "flow_still.hpp"
#include "../tools/flow_movie/cache.hpp"
#include "x500_model.hpp"
#include "gpu_skip.hpp"
#include "physics/airflow/actuator.hpp"
#include "physics/airflow/dye.hpp"
#include "physics/airflow/line.hpp"
#include "physics/airflow/rotor_discs.hpp"
#include "scene/model_file.hpp"
#include "scene/part_file.hpp"
#include "scene/prop_line.hpp"
#include "spike/airflow/line_chain.hpp"
#include "spike/airflow/step_chain.hpp"
#include "vehicles/rotor_discs.hpp"
#include "vehicles/rotor_lines.hpp"

namespace {

namespace af = spade::physics::airflow;
namespace ag = spade::airflow_gpu;
namespace an = spade::analysis;
namespace vh = spade::vehicles;
using flow_still::env_cstr;

constexpr float kTwoPi = 6.28318530718f;

[[nodiscard]] std::string read_text(const std::filesystem::path& p) {
    std::ifstream in(p, std::ios::binary);
    return std::string((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
}

std::vector<uint32_t> steps_from_env() {
    const char* v = env_cstr("SPADE_LINE_STILLS_STEPS");
    std::stringstream in(v != nullptr ? v : "300,700,1200,1800");
    std::vector<uint32_t> out;
    std::string item;
    while (std::getline(in, item, ',')) out.push_back(static_cast<uint32_t>(std::strtoul(item.c_str(), nullptr, 10)));
    return out;
}

// The phantom as a still-air D1 case on the deep grid (tests/d1_case.hpp's builder is the reference quad's), its box built as
// b7::d1_box() builds the reference quad's. `box.discs` are the rows' discs at the commanded hover speed.
struct Phantom {
    b7::B7Box box;
    std::vector<vh::RotorRow> rows;
    spade::BodyState body{};
    float weight_n = 0.0f;
};

[[nodiscard]] spade::Result<Phantom> phantom_box(const std::filesystem::path& file, const std::string& pin,
                                                 const af::GridShape& grid, const glm::vec3& hold) {
    auto loaded = spade::scene::load_model_file(file);
    if (!loaded) return std::unexpected(loaded.error());
    if (!loaded->model.propulsion) {
        return std::unexpected(spade::Error{spade::Code::invalid_argument, "the model has no propulsion chain"});
    }
    spade::vehicles::ModelType model = std::move(loaded->model);
    an::AnalysisCase c;
    c.power = an::Power{.idle = 0.05f, .state_of_charge = 1.0f};
    c.model = an::pin_model(pin, model);
    c.rotors.assign(model.rotors.size(), an::RotorSetup{});
    c.grid.box = grid.dx * static_cast<float>(std::max(grid.nx, grid.nz));
    c.grid.spacing = grid.dx;
    c.time.duration = 1.0f;
    c.time.step = flow_still::env_float("SPADE_LS_STEP", 0.0002f);  // L86p: 0.0001 for the step-size probe
    c.flight.kind = an::FlightKind::hover;  // still air
    c.flight.speed = 0.0f;
    auto trim = an::showcase_trim(c, model);
    if (!trim) return std::unexpected(trim.error());

    auto resolved = an::resolve(c, model, an::ThinWalls::drawn_by_the_live_run);
    if (!resolved) {
        std::string issues;
        for (const an::CaseIssue& i : resolved.error()) issues += (issues.empty() ? "" : "; ") + i.field + ": " + i.message;
        return std::unexpected(spade::Error{spade::Code::invalid_argument, "the case does not resolve: " + issues});
    }
    resolved->hold_from_centre = hold;
    resolved->base = grid;
    resolved->base_bytes = af::fluid_bytes(resolved->base, resolved->domain);
    const af::GridShape& shape = resolved->base;
    const glm::vec3 origin = -0.5f * shape.dx *
                                 glm::vec3(static_cast<float>(shape.nx), static_cast<float>(shape.ny),
                                           static_cast<float>(shape.nz)) -
                             resolved->hold_from_centre;
    auto live = b7::live_hover(*resolved, model, trim->hover);
    if (!live) return std::unexpected(live.error());

    Phantom p;
    p.rows = live->rows;
    p.body = live->body;
    b7::B7Box& box = p.box;
    box.name = "phantom on the deep box";
    box.origin = origin;
    box.bc = resolved->domain;
    box.h_ns = static_cast<uint64_t>(static_cast<double>(*c.time.step) * 1.0e9 + 0.5);
    box.cfg = af::FluidConfig{.rho = resolved->spec.fluid.density, .nu = resolved->spec.fluid.kinematic_viscosity,
                              .h = static_cast<float>(static_cast<double>(box.h_ns) / 1.0e9)};
    box.cfg.n_v = b7::kSolidsCgIterations;
    box.cfg.sgs_cs = 0.17f;
    auto g = af::make_grid(shape, box.bc);
    if (!g) return std::unexpected(g.error());
    box.grid = std::move(*g);

    auto d1 = std::make_shared<b7::D1Solids>();
    d1->model = model;
    spade::BodyState hover{};
    hover.pos = p.body.pos;
    hover.orient = p.body.orient;
    const std::vector<float> still(d1->model.rotors.size(), 0.0f);
    auto posed = vh::posed_parts(d1->model, hover, still, still, vh::PartFilter::rigid_only);
    if (!posed) return std::unexpected(posed.error());
    d1->solids.emplace(d1->model, std::move(*posed), origin);
    auto parts = d1->solids->parts();
    if (!parts) return std::unexpected(parts.error());
    d1->parts = std::move(*parts);
    box.set_box_solids = [d1](af::FluidScratch& s, const af::MacGrid& gr, const af::DomainBc& bc) {
        return af::set_solids(s, gr, bc, d1->parts, d1->solids->sampler());
    };
    auto s = b7::box_scratch(box);
    if (!s) return std::unexpected(s.error());
    box.solids = s->solids;
    box.solid_cells = s->regions.solid;

    for (vh::RotorRow& r : p.rows) r.omega = r.omega_cmd;
    box.discs.resize(p.rows.size());
    if (auto d = vh::rotor_discs(p.rows, p.body, origin, shape, box.discs); !d) return std::unexpected(d.error());
    box.about = p.body.pos - origin;
    return p;
}

TEST(GpuAirflowLineStills, ThePhantomsLinesOnTheDeepBox) {
    const char* out_env = env_cstr("SPADE_LINE_STILLS_OUT");
    if (out_env == nullptr) GTEST_SKIP() << "opt-in: SPADE_LINE_STILLS_OUT=<folder> (GPU, minutes)";
    if (const auto why = spade::testing::vulkan_skip_reason()) GTEST_SKIP() << *why;
    const std::filesystem::path out(out_env);
    std::filesystem::create_directories(out);
    const char* model_env = env_cstr("SPADE_LINE_STILLS_FILE");
    const bool discs_only = env_cstr("SPADE_LINE_STILLS_DISCS") != nullptr;
    const uint32_t n_sub = env_cstr("SPADE_LINE_STILLS_NSUB") != nullptr
                               ? static_cast<uint32_t>(std::strtoul(env_cstr("SPADE_LINE_STILLS_NSUB"), nullptr, 10))
                               : 8u;

    // SPADE_LS_GRID=near: the 2 mm cropped near field (tests/d1_case.hpp's kD1NearGrid); else the 4 mm deep box.
    // SPADE_LS_DRONE=x500 (L86k): the X500 (tools/flow_movie/data/x500) with the kit's 10-inch prop on kD1X500Grid, 2.5 mm.
    const bool x500 = env_cstr("SPADE_LS_DRONE") != nullptr && std::string(env_cstr("SPADE_LS_DRONE")) == "x500";
    af::GridShape grid = env_cstr("SPADE_LS_GRID") != nullptr ? spade::testing::kD1NearGrid : spade::testing::kD1DeepGrid;
    if (x500) grid = spade::testing::kD1X500Grid;
    const std::string model_rel = model_env != nullptr ? model_env : "library/drones/phantom3_class.model.yaml";
    const std::filesystem::path model_path =
        x500 ? spade::testing::x500::data_dir() / spade::testing::x500::kModelFile : std::filesystem::path(SPADE_ASSETS_DIR) / model_rel;
    auto made = phantom_box(model_path, x500 ? std::string("x500") : "assets/" + model_rel, grid,
                            x500 ? spade::testing::d1_x500_hold() : spade::testing::d1_hold(grid));
    ASSERT_TRUE(made.has_value()) << made.error().context;
    Phantom& ph = *made;
    b7::B7Box& box = ph.box;
    const af::GridShape shape = box.grid.shape;

    // The prop: the library's APC 10x5 (phantom3_class's), its blade and B1's fitted polar; the X500's is that scaled, graded estimated.
    const std::filesystem::path prop_file = x500 ? spade::testing::x500::data_dir() / spade::testing::x500::kPropFile
                                                 : std::filesystem::path(SPADE_ASSETS_DIR) / "library/parts/uiuc_apce_10x5.prop.yaml";
    auto part = spade::scene::prop_from_yaml(read_text(prop_file));
    ASSERT_TRUE(part.has_value()) << part.error().context;
    auto prop = spade::scene::line_propeller(*part);
    ASSERT_TRUE(prop.has_value()) << prop.error().context;

    // The lines at the held hover, psi from 0; the element count is fixed for the run.
    std::vector<float> psi(ph.rows.size(), 0.0f);
    std::vector<af::LineRotor> lines(ph.rows.size());
    ASSERT_TRUE(vh::rotor_lines(ph.rows, ph.body, psi, box.origin, shape, prop->blade, prop->polar, lines).has_value());
    uint32_t elements = 0;
    for (const af::LineRotor& l : lines) {
        elements += static_cast<uint32_t>(af::line_elements(l, shape.dx, box.cfg.h, n_sub).size());
    }
    // The dye is released at the discs, with no load: the lines carry the force.
    std::vector<af::ActuatorDisc> dye_discs = box.discs;
    if (!discs_only) {
        for (af::ActuatorDisc& d : dye_discs) {
            d.thrust = 0.0f;
            d.torque = 0.0f;
        }
    }

    auto ctx = spade::compute::VulkanContext::create(
        spade::compute::BackendDesc{.kind = spade::compute::BackendKind::vulkan});
    ASSERT_TRUE(ctx.has_value()) << ctx.error().context;
    const ag::LineSetup line_setup{.polar = &prop->polar, .elements = elements, .n_sub = n_sub, .tip_correction = true,
                                 .binned = env_cstr("SPADE_LINE_STILLS_UNBINNED") == nullptr,
                                 .rotors = static_cast<uint32_t>(ph.rows.size())};
    // The tips' release (the helices) and, with SPADE_LS_RING=1, the disc tip ring's as before; SPADE_TIP_GAIN is the tip release's gain.
    af::DyeRelease ring{};
    if (flow_still::env_cstr("SPADE_LS_RING") == nullptr) ring.rate = 0.0f;
    const af::TipRelease tip{.gain = flow_still::env_float("SPADE_TIP_GAIN", 4.0f),
                             .tip_frac = flow_still::env_float("SPADE_TIP_FRAC", 1.0f)};
    const ag::DyeSetup dye_setup{ring, discs_only ? std::nullopt : std::optional<af::TipRelease>(tip)};
    auto gpu = ag::GpuFluid::create(**ctx, box.grid, box.bc, box.cfg, &box.solids, 64u,
                                    discs_only ? nullptr : &line_setup, &dye_setup);
    ASSERT_TRUE(gpu.has_value()) << gpu.error().context;

    const float omega = ph.rows.empty() ? 0.0f : ph.rows.front().omega;
    std::printf("[ lines   ] %zu rotors, %u elements a step (n_sub %u), omega %.1f rad/s (%.0f rpm), R %.4f m = %.1f cells, "
                "fitted polar %s; box %ux%ux%u at %.0f mm, h %.2f ms\n",
                ph.rows.size(), elements, n_sub, static_cast<double>(omega), static_cast<double>(omega) * 9.5492965855,
                static_cast<double>(prop->blade.radius), static_cast<double>(prop->blade.radius / shape.dx),
                prop->fitted ? "yes" : "no", shape.nx, shape.ny, shape.nz, static_cast<double>(shape.dx) * 1000.0,
                static_cast<double>(box.cfg.h) * 1000.0);
    const bool diag = env_cstr("SPADE_LS_DIAG") != nullptr;
    if (diag && !ph.rows.empty()) {
        const double tip_speed = static_cast<double>(omega) * static_cast<double>(prop->blade.radius);
        const double cells = tip_speed * static_cast<double>(box.cfg.h) / static_cast<double>(shape.dx);
        std::printf("[ diag    ] tip speed %.1f m/s: %.2f cells a step, %.2f cells between the %u sub-azimuths of a step; the sub-azimuths all read the "
                    "grid as the step begins\n",
                    tip_speed, cells, cells / static_cast<double>(n_sub), n_sub);
    }
    std::fflush(stdout);
    struct PressureStats {
        double max_residual = 0.0;
        uint64_t iterations = 0;
        uint32_t steps = 0, capped = 0, unconverged = 0, stalled = 0;
    } pstats;
    double impulse = 0.0;  // N s: the sum over steps of h times the thrust the rotors put in (the lines' report, or the discs' fixed thrust)

    std::vector<uint32_t> at = steps_from_env();
    // The frames the cache keeps (SPADE_LINE_STILLS_CACHE): frame f ends at step (f + 1) x spf, spf the steps in 1 ms (the d1-still group's frame: 5 at
    // 0.2 ms). L106: SPADE_LINE_STILLS_KEEP="49,99" lists the frames and SPADE_LINE_STILLS_MEAN=N keeps the N frames ending at each (the look's mean
    // window); a listed frame's step is run to and published whatever SPADE_LINE_STILLS_STEPS says. Without KEEP, the one frame SPADE_LINE_STILLS_CACHE_FRAME (200).
    const uint32_t spf = static_cast<uint32_t>(std::lround(0.001 / static_cast<double>(box.cfg.h)));
    std::vector<std::pair<uint32_t, uint32_t>> kept;  // (step, frame)
    if (const char* keep = env_cstr("SPADE_LINE_STILLS_KEEP"); keep != nullptr) {
        const uint32_t window = env_cstr("SPADE_LINE_STILLS_MEAN") != nullptr
                                    ? std::max<uint32_t>(1u, static_cast<uint32_t>(std::strtoul(env_cstr("SPADE_LINE_STILLS_MEAN"), nullptr, 10)))
                                    : 1u;
        std::stringstream in(keep);
        std::string item;
        while (std::getline(in, item, ',')) {
            const uint32_t end_frame = static_cast<uint32_t>(std::strtoul(item.c_str(), nullptr, 10));
            for (uint32_t k = 0; k < window && k <= end_frame; ++k) {
                const uint32_t frame = end_frame - k;
                kept.emplace_back((frame + 1u) * spf, frame);
                at.push_back((frame + 1u) * spf);
            }
        }
        std::printf("[ lines   ] the cache keeps %zu frames (%u steps a frame, %.4f ms a frame)\n", kept.size(), spf,
                    static_cast<double>(spf) * static_cast<double>(box.cfg.h) * 1000.0);
    } else {
        const uint32_t frame = env_cstr("SPADE_LINE_STILLS_CACHE_FRAME") != nullptr
                                   ? static_cast<uint32_t>(std::strtoul(env_cstr("SPADE_LINE_STILLS_CACHE_FRAME"), nullptr, 10))
                                   : 200u;
        kept.emplace_back((frame + 1u) * spf, frame);
    }
    const uint32_t last = at.empty() ? 0u : *std::max_element(at.begin(), at.end());
    const std::vector<flow_still::Look> looks = flow_still::dye_looks();
    const float dist = flow_still::env_float("SPADE_LS_DIST", 0.9f);
    const float aim = flow_still::env_float("SPADE_LS_AIM", -0.08f);
    const char* views_wanted = env_cstr("SPADE_LS_VIEWS") != nullptr ? env_cstr("SPADE_LS_VIEWS") : "a";  // of a, b, c
    const auto t_start = std::chrono::steady_clock::now();
    spade::FlowVolume volume;
    double step_s = 0.0;
    uint32_t steps_timed = 0;
    for (uint32_t step = 1; step <= last; ++step) {
        for (std::size_t i = 0; i < lines.size(); ++i) lines[i].psi = psi[i];
        const auto t_step = std::chrono::steady_clock::now();
        const auto health = discs_only ? (*gpu)->step(box.discs) : (*gpu)->step(dye_discs, lines);
        step_s += std::chrono::duration<double>(std::chrono::steady_clock::now() - t_step).count();
        ++steps_timed;
        if (step == 20 && !discs_only) {  // where a line step's time goes: the host's packing alone, 5 times
            const auto t_pack = std::chrono::steady_clock::now();
            for (int rep = 0; rep < 5; ++rep) {
                const auto records = ag::line_records(lines, shape, box.cfg.rho, box.cfg.h, n_sub);
                ASSERT_TRUE(records.has_value());
            }
            std::printf("[ lines   ] line_records() alone: %.1f ms a step; the whole step %.1f ms so far\n",
                        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t_pack).count() / 5.0,
                        step_s / static_cast<double>(steps_timed) * 1000.0);
            std::fflush(stdout);
        }
        ASSERT_TRUE(health.has_value()) << "step " << step << ": " << health.error().context;
        ASSERT_EQ(health->fault, af::FluidFault::none) << "step " << step;
        pstats.max_residual = std::max(pstats.max_residual, static_cast<double>(health->pressure_residual));
        pstats.iterations += health->pressure_iterations;
        ++pstats.steps;
        pstats.capped += health->capped ? 1u : 0u;
        pstats.unconverged += health->unconverged ? 1u : 0u;
        pstats.stalled += health->stalled ? 1u : 0u;
        if (diag) {
            double step_thrust = 0.0;
            if (discs_only) {
                for (const af::ActuatorDisc& d : box.discs) step_thrust += static_cast<double>(d.thrust);
            } else {
                for (const af::LineReport& r : (*gpu)->line_reports()) step_thrust += static_cast<double>(r.thrust);
            }
            impulse += step_thrust * static_cast<double>(box.cfg.h);
        }
        for (float& a : psi) a = std::fmod(a + omega * box.cfg.h, kTwoPi);
        if (std::find(at.begin(), at.end(), step) == at.end()) continue;

        double thrust = 0.0;
        for (const af::LineReport& r : (*gpu)->line_reports()) thrust += static_cast<double>(r.thrust);
        ASSERT_TRUE((*gpu)->publish(box.origin, volume).has_value());
        // SPADE_LINE_STILLS_CACHE=<dir>: the published frame in flow_movie's cache format as frame SPADE_LINE_STILLS_CACHE_FRAME (200, the
        // d1-still group's S8 f200: step (200 + 1) * 5 = 1005, so list that step in SPADE_LINE_STILLS_STEPS), for Rendering's look.
        if (const char* cache_dir = env_cstr("SPADE_LINE_STILLS_CACHE"); cache_dir != nullptr) {
            const auto kept_here = std::find_if(kept.begin(), kept.end(), [&](const std::pair<uint32_t, uint32_t>& k) { return k.first == step; });
            if (kept_here != kept.end()) {
                const uint32_t frame = kept_here->second;
                std::filesystem::create_directories(cache_dir);
                double lines_thrust = 0.0;
                for (const af::LineReport& r : (*gpu)->line_reports()) lines_thrust += static_cast<double>(r.thrust);
                // Rendering's layout (flow_movie's reader): the held body's position, its quaternion w x y z, each rotor's omega, then the lines' thrust.
                std::vector<double> live{static_cast<double>(ph.body.pos.x), static_cast<double>(ph.body.pos.y), static_cast<double>(ph.body.pos.z),
                                         static_cast<double>(ph.body.orient.w), static_cast<double>(ph.body.orient.x),
                                         static_cast<double>(ph.body.orient.y), static_cast<double>(ph.body.orient.z)};
                for (const vh::RotorRow& r : ph.rows) live.push_back(static_cast<double>(r.omega));
                live.resize(11, 0.0);
                live.push_back(lines_thrust);
                const flow_movie::FrameLog log{step, static_cast<double>(step) * static_cast<double>(box.cfg.h), live};
                const uint32_t solid = static_cast<uint32_t>(std::count_if(box.solid_cells.begin(), box.solid_cells.end(), [](uint8_t m) { return m != 0u; }));
                const flow_movie::FrameBound bound{12.0f, solid};
                const auto written = flow_movie::write_frame(cache_dir, frame, volume, log, bound);
                ASSERT_TRUE(written.has_value()) << written.error().context;
                std::printf("[ lines   ] frame %u written to %s (step %u)\n", frame, cache_dir, step);
                std::fflush(stdout);
            }
        }
        std::printf("[ lines   ] solve %.0f ms a step; step %u (%.3f s), wall %.0f s: lines' thrust %.3f N in all (%zu reports)\n",
                    step_s / static_cast<double>(steps_timed) * 1000.0, step,
                    static_cast<double>(step) * static_cast<double>(box.cfg.h),
                    std::chrono::duration<double>(std::chrono::steady_clock::now() - t_start).count(), thrust,
                    (*gpu)->line_reports().size());
        std::fflush(stdout);
        if (diag && !lines.empty()) {
            std::printf("[ diag    ] step %u: pressure solve over the last %u steps: largest residual %.3e, %.1f iterations a step, capped %u, "
                        "unconverged %u, stalled %u\n",
                        step, pstats.steps, pstats.max_residual,
                        pstats.steps != 0u ? static_cast<double>(pstats.iterations) / static_cast<double>(pstats.steps) : 0.0, pstats.capped,
                        pstats.unconverged, pstats.stalled);
            pstats = {};
            const spade::FlowVolumeHeader& hd = volume.header;
            {  // THE CLOSED ACCOUNT: the whole box's vertical momentum against the impulse put in so far, and the air through rotor 0's disc.
                double down = 0.0, lateral_x = 0.0, lateral_z = 0.0;
                for (uint32_t k = 0; k < hd.nz; ++k) {
                    for (uint32_t j = 0; j < hd.ny; ++j) {
                        for (uint32_t i = 0; i < hd.nx; ++i) {
                            const std::size_t cell = spade::flow_cell_index(hd, i, j, k);
                            const float u = spade::fp16::to_f32(spade::flow_half(volume, cell, spade::kFlowU));
                            const float w = spade::fp16::to_f32(spade::flow_half(volume, cell, spade::kFlowV));
                            const float z = spade::fp16::to_f32(spade::flow_half(volume, cell, spade::kFlowW));
                            if (std::isnan(w)) continue;
                            down += -static_cast<double>(w);
                            lateral_x += static_cast<double>(u);
                            lateral_z += static_cast<double>(z);
                        }
                    }
                }
                const double dv = static_cast<double>(hd.cell) * hd.cell * hd.cell * static_cast<double>(box.cfg.rho);
                std::printf("[ diag    ] step %u (%.3f s): box momentum down %.5f kg m/s against the impulse put in %.5f N s: ratio %.4f (sideways x %.5f, z %.5f)\n",
                            step, static_cast<double>(step) * static_cast<double>(box.cfg.h), down * dv, impulse, impulse > 0.0 ? down * dv / impulse : 0.0,
                            lateral_x * dv, lateral_z * dv);
                for (const float drop : {0.0125f, 0.05f}) {
                    const int j = static_cast<int>(std::floor((lines[0].centre.y - drop) / hd.cell));
                    if (j < 0 || j >= static_cast<int>(hd.ny)) continue;
                    double sum = 0.0;
                    uint32_t n = 0;
                    const float r_max = 0.9f * static_cast<float>(prop->blade.radius);
                    for (uint32_t k = 0; k < hd.nz; ++k) {
                        for (uint32_t i = 0; i < hd.nx; ++i) {
                            const float dx = (static_cast<float>(i) + 0.5f) * hd.cell - lines[0].centre.x;
                            const float dz = (static_cast<float>(k) + 0.5f) * hd.cell - lines[0].centre.z;
                            if (dx * dx + dz * dz > r_max * r_max) continue;
                            const float w = spade::fp16::to_f32(
                                spade::flow_half(volume, spade::flow_cell_index(hd, i, static_cast<uint32_t>(j), k), spade::kFlowV));
                            if (std::isnan(w)) continue;
                            sum += -static_cast<double>(w);
                            ++n;
                        }
                    }
                    std::printf("[ diag    ] step %u: mean downward air in rotor 0's disc column %.4f m under the hub: %.3f m/s (%u cells)\n", step,
                                static_cast<double>(drop), n != 0u ? sum / n : 0.0, n);
                }
            }
            for (const float drop : {0.05f, 0.10f, 0.20f, 0.30f}) {
                const int j = static_cast<int>(std::floor((lines[0].centre.y - drop) / hd.cell));
                if (j < 0 || j >= static_cast<int>(hd.ny)) continue;
                double w2 = 0.0, mass = 0.0, press = 0.0;
                for (uint32_t k = 0; k < hd.nz; ++k) {
                    for (uint32_t i = 0; i < hd.nx; ++i) {
                        const std::size_t cell = spade::flow_cell_index(hd, i, static_cast<uint32_t>(j), k);
                        const float w = spade::fp16::to_f32(spade::flow_half(volume, cell, spade::kFlowV));
                        const float p = spade::fp16::to_f32(spade::flow_half(volume, cell, spade::kFlowPressure));
                        if (std::isnan(w) || std::isnan(p)) continue;
                        w2 += static_cast<double>(w) * w;
                        mass += -static_cast<double>(w);
                        press += p;
                    }
                }
                const double area = static_cast<double>(hd.cell) * hd.cell;
                std::printf("[ diag    ] step %u: %.2f m under the hubs: momentum flux rho sum w^2 dA %.3f N, mass flux %.4f kg/s, pressure sum "
                            "p dA %.3f N, together %.3f N (the lines' thrust %.3f N)\n",
                            step, static_cast<double>(drop), static_cast<double>(box.cfg.rho) * w2 * area,
                            static_cast<double>(box.cfg.rho) * mass * area, press * area, static_cast<double>(box.cfg.rho) * w2 * area + press * area,
                            thrust);
            }
            std::fflush(stdout);
        }
        const struct View {
            const char* name;
            float az, el, target_y;
        } views[] = {{"a", 25.0f, 8.0f, aim}, {"b", 25.0f, 8.0f, aim - 0.25f}, {"c", 90.0f, 2.0f, aim - 0.10f}};
        for (const View& v : views) {
            if (std::strchr(views_wanted, v.name[0]) == nullptr) continue;
            const spade::render::Camera cam = flow_still::orbit(glm::vec3(0.0f, v.target_y, 0.0f), dist, v.az, v.el);
            for (const flow_still::Look& look : looks) {
                const std::string path = (out / (std::string(discs_only ? "discs" : "lines") + "-s" + std::to_string(step) +
                                                 "-" + look.name + "-" + v.name + ".ppm")).string();
                const auto drawn = flow_still::draw_volume(volume, cam, look, path);
                ASSERT_TRUE(drawn.has_value()) << drawn.error().context;
            }
        }
    }
}

}  // namespace
