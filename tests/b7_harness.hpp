#pragma once

// B7's acceptance harness (the GPU solver plan, B7): the GPU fluid step's
// first end-to-end run. A candidate runs beside the CPU from the same state,
// and u, v, w and p, each step's health and the wall load must match it bit
// for bit (TD-14 as amended 2026-10-22: every kernel exact and both twins
// flush, so no band). The step and the film's 0.50 s are timed, and the
// record block is read back as pasted. A candidate names what it can't yet
// report, and the block names those "not compared", never a match (the lead,
// 2026-10-27). The CPU stand-ins are in test_gpu_airflow_d1_acceptance.cpp,
// Rendering's GpuFluid in test_gpu_airflow_d1_device.cpp.

#include <algorithm>
#include <array>
#include <bit>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <functional>
#include <limits>
#include <memory>
#include <numbers>
#include <optional>
#include <span>
#include <string>
#include <utility>
#include <vector>

#include <glm/glm.hpp>

#include "airflow_record.hpp"
#include "airflow_solid_shapes.hpp"
#include "d1_case.hpp"
#include "analysis/live_run.hpp"
#include "core/flow_volume.hpp"
#include "physics/airflow/actuator.hpp"
#include "physics/airflow/fluid_step.hpp"
#include "physics/airflow/multigrid.hpp"
#include "physics/airflow/pcg.hpp"
#include "physics/airflow/publish.hpp"
#include "physics/airflow/regions.hpp"
#include "physics/airflow/wall_load.hpp"
#include "vehicles/airflow_solids.hpp"
#include "vehicles/part_pose.hpp"
#include "vehicles/rotor_discs.hpp"

namespace b7 {

using namespace spade::physics::airflow;
using airflow_test::Kind;
using airflow_test::Scene;
using airflow_test::Shape;
namespace an = spade::analysis;

inline constexpr uint32_t kComparedSteps = 32;  // N: the steps compared with the CPU's
inline constexpr uint32_t kWarmSteps = 3;       // W: the steps before the timed ones
inline constexpr uint32_t kTimedSteps = 9;      // M: the step time is their median
inline constexpr uint32_t kFilmFrames = 15;     // the film's 0.50 s at LiveRun's frame rate
inline constexpr double kStepBoundMs = 190.0;   // the plan's 2x profiling trigger (B7)
inline constexpr uint32_t kNudgeStep = 5;       // s0: the nudged stand-in's first wrong step
inline constexpr uint64_t kNsPerSecond = 1'000'000'000;
static_assert(2 * kFilmFrames == an::kFramesPerSecond, "the film's 0.50 s");

// What a run compares and times. The CPU stand-in can't afford the D1 box's
// film (1316 steps at ~20 s), so there its plan compares 2 steps, times one,
// and leaves the film out of the block; the GPU's gate run compares 2 and
// times none (timed 0: no step-time measure).
struct RunPlan {
    uint32_t compared = kComparedSteps;
    uint32_t warm = kWarmSteps;
    uint32_t timed = kTimedSteps;
    bool film = true;  // the film's 0.50 s, kFilmFrames frames
};
inline constexpr RunPlan kFullPlan{};
inline constexpr RunPlan kD1StandInPlan{.compared = 2, .warm = 0, .timed = 1, .film = false};
inline constexpr RunPlan kShortPlan{.compared = 2, .warm = 0, .timed = 0, .film = false};

// What a candidate reports beside u, v, w and p. What it can't yet report is
// left out of the comparison and named "not compared" in the block.
struct Reports {
    bool fault = true;  // with, where either side faulted, its quantity, faces and index
    bool unconverged = true;
    bool pressure_iterations = true;
    bool pressure_residual = true;
    bool stalled = true;
    bool capped = true;
    bool wall_load = true;  // the force and the moment
    bool solids = true;     // the solid field at init
};

// The names, in the comparison's order, of what `r` reports (`reported`
// true) or doesn't.
[[nodiscard]] inline std::vector<std::string> report_names(const Reports& r, bool reported) {
    const std::array<std::pair<bool, const char*>, 8> all = {{{r.fault, "fault"},
                                                              {r.unconverged, "unconverged"},
                                                              {r.pressure_iterations, "pressure_iterations"},
                                                              {r.pressure_residual, "pressure_residual"},
                                                              {r.stalled, "stalled"},
                                                              {r.capped, "capped"},
                                                              {r.wall_load, "the wall load"},
                                                              {r.solids, "the solids"}}};
    std::vector<std::string> out;
    for (const auto& [on, name] : all) {
        if (on == reported) out.emplace_back(name);
    }
    return out;
}

[[nodiscard]] inline std::string joined(const std::vector<std::string>& names) {
    std::string out;
    for (const std::string& n : names) out += (out.empty() ? "" : ", ") + n;
    return out;
}

// The GPU fluid step as B7 drives it: the seam Physics agreed with Rendering
// (2026-10-27), which Rendering's GpuFluid (spike/airflow/step_chain) takes.
class FluidStepper {
  public:
    virtual ~FluidStepper() = default;
    // The record's name for it: the path line's, and the opening line's.
    [[nodiscard]] virtual std::string name() const = 0;
    // What it reports beside u, v, w and p; the rest is not compared.
    [[nodiscard]] virtual Reports reports() const { return {}; }
    // One step in LiveRun's order: each disc spread into the increments,
    // then the fluid step.
    [[nodiscard]] virtual spade::Result<FluidHealth> step(std::span<const ActuatorDisc> discs) = 0;
    // u, v, w and p as the last step left them.
    [[nodiscard]] virtual spade::Result<void> read(MacGrid& out) = 0;
    // The pressure's load on the solids about `about`, as wall_load() gives it.
    [[nodiscard]] virtual spade::Result<WallLoad> wall_load(glm::vec3 about) = 0;
    // The solids as the step reads them. The CPU has no packed form, so this
    // is the solid field's own arrays.
    [[nodiscard]] virtual spade::Result<void> read_solids(SolidField& out) = 0;
    // The current flow published as the film publishes it (LiveRun's
    // advance_frame()).
    [[nodiscard]] virtual spade::Result<void> publish(const PublishInput& in, spade::FlowVolume& out) = 0;
};

// The seam's creation: from the box's initial state and its solids, built
// once on the host (nullptr: none).
using MakeStepper = std::function<spade::Result<std::unique_ptr<FluidStepper>>(
    const MacGrid& g, const DomainBc& bc, const FluidConfig& cfg, const SolidField* solids)>;

// A scratch for the seam's solids: a field already built, set as set_solids()
// sets its own after build_solids() (fluid_step.cpp): the regions labelled,
// the multigrid's face weights and the CG's vectors.
[[nodiscard]] inline spade::Result<FluidScratch> scratch_with(const MacGrid& g, const DomainBc& bc,
                                                              const FluidConfig& cfg, const SolidField* solids) {
    spade::Result<FluidScratch> s = make_fluid_scratch(g, bc, cfg);
    if (!s || solids == nullptr || !solids->any) return s;
    if (spade::Result<void> ok = check_grid(g.shape, bc, true); !ok) return std::unexpected(ok.error());
    s->solids = *solids;
    label_regions(s->regions, g.shape, periodic_axes(bc), s->solids);
    set_weights(s->projection.mg, {s->solids.theta[0], s->solids.theta[1], s->solids.theta[2]});
    if (s->projection.pcg.x.size() != g.p.size()) s->projection.pcg = make_pcg_scratch(g.p.size());
    return s;
}

// LiveRun's step on the CPU (engine/analysis/live_run.cpp): each disc spread
// into the scratch's increments, then fluid_step(); the wall load on request.
class CpuStandIn : public FluidStepper {
  public:
    CpuStandIn(const MacGrid& g, const DomainBc& bc, const FluidConfig& cfg, FluidScratch s)
        : grid_(g), bc_(bc), cfg_(cfg), scratch_(std::move(s)) {}

    [[nodiscard]] static spade::Result<std::unique_ptr<FluidStepper>> create(const MacGrid& g, const DomainBc& bc,
                                                                             const FluidConfig& cfg,
                                                                             const SolidField* solids) {
        spade::Result<FluidScratch> s = scratch_with(g, bc, cfg, solids);
        if (!s) return std::unexpected(s.error());
        return std::make_unique<CpuStandIn>(g, bc, cfg, std::move(*s));
    }

    [[nodiscard]] std::string name() const override { return "CPU stand-in"; }

    [[nodiscard]] spade::Result<FluidHealth> step(std::span<const ActuatorDisc> discs) override {
        for (const ActuatorDisc& d : discs) {
            spread_actuator(d, grid_.shape, cfg_.rho, cfg_.h, scratch_.du, scratch_.dv, scratch_.dw);
        }
        ++steps_;
        return fluid_step(grid_, bc_, cfg_, scratch_);
    }

    [[nodiscard]] spade::Result<void> read(MacGrid& out) override {
        out = grid_;
        return {};
    }

    [[nodiscard]] spade::Result<WallLoad> wall_load(glm::vec3 about) override {
        return spade::physics::airflow::wall_load(grid_, scratch_.solids, scratch_.regions, about);
    }

    [[nodiscard]] spade::Result<void> read_solids(SolidField& out) override {
        out = scratch_.solids;
        return {};
    }

    [[nodiscard]] spade::Result<void> publish(const PublishInput& in, spade::FlowVolume& out) override {
        if (const auto published = publish_render_volume(grid_, in, out); !published) {
            return std::unexpected(published.error());
        }
        return {};
    }

  protected:
    MacGrid grid_;
    DomainBc bc_;
    FluidConfig cfg_;
    FluidScratch scratch_;
    uint32_t steps_ = 0;  // the steps taken
};

// A box the harness runs: its initial state, its solid body and the field
// LiveRun's set_solids() builds for it, its discs, and its step in whole ns
// for the film's frame times.
struct B7Box {
    std::string name;    // the record's opening line
    std::string config;  // its config line, before the harness's counts
    MacGrid grid;        // the initial state
    DomainBc bc;
    FluidConfig cfg;
    uint64_t h_ns = 0;
    // set_solids() on a fresh scratch, as LiveRun calls it: the CPU's own run
    // builds its solids through this, the candidate gets `solids`. Empty for
    // a box without solids.
    std::function<spade::Result<void>(FluidScratch&, const MacGrid&, const DomainBc&)> set_box_solids;
    SolidField solids;
    std::vector<uint8_t> solid_cells;  // the publish's mask (LiveRun's publish.solid)
    std::vector<ActuatorDisc> discs;
    glm::vec3 about{0.0f};  // the wall load's moment point: the body's centre
};

// A box with its solids, or without them for a candidate that can't take
// them yet (GpuFluid until G3).
enum class BoxSolids { with, none };

// A fresh scratch's solids for `box`, set as LiveRun sets them.
[[nodiscard]] inline spade::Result<FluidScratch> box_scratch(const B7Box& box) {
    spade::Result<FluidScratch> s = make_fluid_scratch(box.grid, box.bc, box.cfg);
    if (!s || !box.set_box_solids) return s;
    if (spade::Result<void> ok = box.set_box_solids(*s, box.grid, box.bc); !ok) return std::unexpected(ok.error());
    return s;
}

// The stand-in box: 32 x 40 x 24 at 1 cm, every face a wall, still air, a
// sphere below one disc that pushes -y with a little swirl. Nothing in it is
// symmetric, so a kernel that swaps two axes shows. Without solids it has no
// sphere and plan A's V-cycles, as LiveRun runs a case without solids.
[[nodiscard]] inline spade::Result<B7Box> stand_in_box(BoxSolids solids = BoxSolids::with) {
    const bool with = solids == BoxSolids::with;
    B7Box box;
    const float dx = 0.01f;
    const GridShape shape{32, 40, 24, dx};
    box.name = with ? "stand-in box 32x40x24" : "stand-in box 32x40x24 without its sphere";
    box.bc.face.fill(FaceBc::wall);
    box.h_ns = 5'000'000;
    box.cfg = FluidConfig{.rho = 1.225f, .nu = 1.4607e-5f, .h = 0.005f};
    if (with) box.cfg.n_v = kSolidsCgIterations;
    spade::Result<MacGrid> g = make_grid(shape, box.bc);
    if (!g) return std::unexpected(g.error());
    box.grid = std::move(*g);
    const float radius = 6.0f * dx;
    const float vh = 1.0f;  // m/s, the disc's induced velocity
    const float area = std::numbers::pi_v<float> * radius * radius;
    box.discs.push_back(ActuatorDisc{.centre = {0.17f, 0.30f, 0.12f}, .axis = {0.0f, -1.0f, 0.0f}, .radius = radius,
                                     .thrust = 2.0f * box.cfg.rho * area * vh * vh, .torque = 1.0e-4f});
    if (spade::Result<void> ok = check_disc(box.discs[0], shape); !ok) return std::unexpected(ok.error());
    box.about = {0.15f, 0.14f, 0.13f};
    if (with) {
        auto scene = std::make_shared<Scene>();
        scene->shapes.push_back(Shape{.name = "sphere", .kind = Kind::sphere, .centre = box.about,
                                      .radius = 4.0f * dx, .min_feature = 8.0f * dx});
        box.set_box_solids = [scene](FluidScratch& s, const MacGrid& g, const DomainBc& bc) {
            return set_solids(s, g, bc, scene->parts(), scene->sampler());
        };
    }
    spade::Result<FluidScratch> s = box_scratch(box);
    if (!s) return std::unexpected(s.error());
    box.solids = s->solids;
    box.solid_cells = s->regions.solid;
    const std::string body =
        with ? std::string("a sphere R 4 dx at (15, 14, 13) dx")
             : std::string("no solids until G3 (the sphere left out; GpuFluid takes solids with Physics' hooks)");
    const std::string solve = with ? "CG floor " + std::to_string(kSolidsCgIterations)
                                   : "n_v " + std::to_string(box.cfg.n_v) + " (plan A's V-cycles)";
    box.config = airflow_test::formatted([&](char* b, std::size_t n) {
        return std::snprintf(b, n,
                             "32x40x24 at dx 1 cm, every face a wall, still air at the start; %s; one disc R 6 dx at "
                             "(17, 30, 12) dx pushing -y, thrust %.4f N (v_h 1 m/s), torque 1e-4 N m; rho 1.225, nu "
                             "1.4607e-5, h 5 ms, %s",
                             body.c_str(), static_cast<double>(box.discs[0].thrust), solve.c_str());
    });
    return box;
}

// The D1 box's solids, kept for the set_solids() calls: the parts borrow the
// model.
struct D1Solids {
    spade::vehicles::ModelType model;
    std::optional<spade::vehicles::AirflowSolids> solids;
    std::vector<SolidPart> parts;
};

// The D1 box from the one builder (tests/d1_case.hpp): kD1Grid at 4 mm, the
// reference quad's rigid parts as solids, held at hover in the case's wind.
// LiveRun takes one step for its rotors' rows and held body; the discs are
// those rows at their commanded hover speed, held constant. That step's
// pressure is capped at one CG iteration: the rows, the held body and the
// solids don't depend on it. The box's solids must be LiveRun's (its
// set_solids() line and solid cells), or it refuses. Without solids it is
// the same air and discs, at plan A's V-cycles.
[[nodiscard]] inline spade::Result<B7Box> d1_box(BoxSolids solids = BoxSolids::with) {
    namespace vh = spade::vehicles;
    const bool with = solids == BoxSolids::with;
    spade::Result<spade::testing::PoweredQuad> q = spade::testing::d1_case(0.5);
    if (!q) return std::unexpected(q.error());
    spade::Result<an::ResolvedCase> resolved = spade::testing::resolve_with_grid(q->c, q->run_model, q->grid);
    if (!resolved) return std::unexpected(resolved.error());
    const GridShape& shape = resolved->base;
    const glm::vec3 origin = -0.5f * shape.dx *
                             glm::vec3(static_cast<float>(shape.nx), static_cast<float>(shape.ny),
                                       static_cast<float>(shape.nz));
    std::vector<vh::RotorRow> rows;
    spade::BodyState body{};
    std::string live_line;
    uint32_t live_cells = 0;
    {
        spade::Result<an::LiveRun> run = an::LiveRun::start(*resolved, q->run_model);
        if (!run) return std::unexpected(run.error());
        run->set_test_solids_max_iterations(1);
        if (spade::Result<void> s = run->step(an::Controls{.armed = true, .throttle = q->trim.hover}); !s) {
            return std::unexpected(s.error());
        }
        rows.assign(run->rotor_rows().begin(), run->rotor_rows().end());
        body = run->body();
        live_line = run->solids().run_line;
        live_cells = run->solids().solid_cells;
    }

    B7Box box;
    box.name = with ? "D1 box 240x177x168" : "D1 box 240x177x168 without its solids";
    box.bc = resolved->domain;
    const an::AnalysisCase& c = resolved->spec;
    box.h_ns = static_cast<uint64_t>(static_cast<double>(*c.time.step) * static_cast<double>(kNsPerSecond) + 0.5);
    box.cfg = FluidConfig{.rho = c.fluid.density, .nu = c.fluid.kinematic_viscosity,
                          .h = static_cast<float>(static_cast<double>(box.h_ns) / static_cast<double>(kNsPerSecond))};
    if (with) box.cfg.n_v = kSolidsCgIterations;
    spade::Result<MacGrid> g = make_grid(shape, box.bc);
    if (!g) return std::unexpected(g.error());
    box.grid = std::move(*g);

    if (with) {
        // The parts as LiveRun poses them: the held body, its rotors still.
        auto d1 = std::make_shared<D1Solids>();
        d1->model = q->run_model;
        spade::BodyState hover{};
        hover.pos = body.pos;
        hover.orient = body.orient;
        const std::vector<float> still(d1->model.rotors.size(), 0.0f);
        spade::Result<vh::PosedParts> posed =
            vh::posed_parts(d1->model, hover, still, still, vh::PartFilter::rigid_only);
        if (!posed) return std::unexpected(posed.error());
        d1->solids.emplace(d1->model, std::move(*posed), origin);
        spade::Result<std::vector<SolidPart>> parts = d1->solids->parts();
        if (!parts) return std::unexpected(parts.error());
        d1->parts = std::move(*parts);
        box.set_box_solids = [d1](FluidScratch& s, const MacGrid& grid, const DomainBc& bc) {
            return set_solids(s, grid, bc, d1->parts, d1->solids->sampler());
        };
    }
    spade::Result<FluidScratch> s = box_scratch(box);
    if (!s) return std::unexpected(s.error());
    if (with) {
        const auto cells = static_cast<uint32_t>(
            std::count_if(s->regions.solid.begin(), s->regions.solid.end(), [](uint8_t m) { return m != 0u; }));
        if (s->solids.run_line != live_line || cells != live_cells) {
            return std::unexpected(spade::Error{spade::Code::internal, "the D1 box's solids aren't LiveRun's: '" +
                                                                           s->solids.run_line + "', " +
                                                                           std::to_string(cells) + " solid cells, "
                                                                           "against '" + live_line + "', " +
                                                                           std::to_string(live_cells)});
        }
    }
    box.solids = s->solids;
    box.solid_cells = s->regions.solid;

    for (vh::RotorRow& r : rows) r.omega = r.omega_cmd;
    box.discs.resize(rows.size());
    if (spade::Result<void> d = vh::rotor_discs(rows, body, origin, shape, box.discs); !d) {
        return std::unexpected(d.error());
    }
    box.about = body.pos - origin;  // the body's centre of mass, in the grid's frame
    float thrust = 0.0f;
    for (const ActuatorDisc& d : box.discs) thrust += d.thrust;
    const std::string solids_text =
        with ? "the reference quad's rigid parts as solids, LiveRun's own (" + live_line + ")"
             : std::string("no solids until G3 (the quad's parts left out; GpuFluid takes solids with Physics' hooks)");
    const std::string solve = with ? "CG floor " + std::to_string(kSolidsCgIterations)
                                   : "n_v " + std::to_string(box.cfg.n_v) + " (plan A's V-cycles)";
    box.config = airflow_test::formatted([&](char* b, std::size_t n) {
        return std::snprintf(b, n,
                             "D1: %ux%ux%u at dx 4 mm (kD1Grid, tests/d1_case.hpp), wind (%.1f, %.1f, %.1f) m/s; "
                             "%s; held at hover, its %zu rotors as actuator discs at their commanded hover speed, "
                             "%.0f rad/s, %.3f N in all; rho %.4g, nu %.4e, h %.2f ms, %s",
                             shape.nx, shape.ny, shape.nz, static_cast<double>(box.bc.ambient.x),
                             static_cast<double>(box.bc.ambient.y), static_cast<double>(box.bc.ambient.z),
                             solids_text.c_str(), box.discs.size(),
                             rows.empty() ? 0.0 : static_cast<double>(rows[0].omega_cmd), static_cast<double>(thrust),
                             static_cast<double>(box.cfg.rho), static_cast<double>(box.cfg.nu),
                             static_cast<double>(box.cfg.h) * 1000.0, solve.c_str());
    });
    return box;
}

// What the comparison found: the solids once, then the values of steps 1 to
// N. A step's values are compared in the order u, v, w, p, its health, its
// wall load, each array in index order, so the first differing value is the
// first in that order. Only what the candidate reports is compared.
struct Comparison {
    std::string candidate;
    Reports reports;
    uint32_t steps = 0;
    uint64_t compared = 0;
    uint64_t differing = 0;
    uint64_t solids_compared = 0;
    uint64_t solids_differing = 0;
    uint32_t first_step = 0;            // 0: none differ
    uint64_t first_step_differing = 0;  // the values differing at it
    std::string first_field;            // u, v, w, p, a health field, force or moment
    std::size_t first_index = 0;        // the sample, or the component
    double first_gap = 0.0;             // |candidate - CPU| there
    std::array<double, 4> largest_gap{};  // u, v, w and p: the largest |candidate - CPU| of the run
    double cpu_speed = 0.0;             // the CPU's largest |v| at step N: the box is not still
};

inline void tally(Comparison& c, uint32_t step, const char* field, std::size_t index, bool differs, double gap) {
    ++c.compared;
    if (!differs) return;
    ++c.differing;
    if (c.first_step == 0) {
        c.first_step = step;
        c.first_field = field;
        c.first_index = index;
        c.first_gap = gap;
    }
    if (step == c.first_step) ++c.first_step_differing;
}

[[nodiscard]] inline bool bits_differ(float a, float b) {
    return std::bit_cast<uint32_t>(a) != std::bit_cast<uint32_t>(b);
}

// One field's samples by their bits; a sample one side lacks differs.
inline void tally_field(Comparison& c, uint32_t step, const char* field, const std::vector<float>& cpu,
                        const std::vector<float>& cand, double& largest) {
    const std::size_t n = std::max(cpu.size(), cand.size());
    for (std::size_t i = 0; i < n; ++i) {
        if (i >= cpu.size() || i >= cand.size()) {
            tally(c, step, field, i, true, std::numeric_limits<double>::infinity());
            continue;
        }
        const bool differs = bits_differ(cpu[i], cand[i]);
        const double gap = std::fabs(static_cast<double>(cand[i]) - static_cast<double>(cpu[i]));
        if (differs) largest = std::max(largest, gap);
        tally(c, step, field, i, differs, gap);
    }
}

// The fault first (Physics' review): a step that faults on one side reads at
// that step even when both counts agree. Its quantity, faces and index count
// only where either side faulted. A field the candidate doesn't report is
// skipped.
inline void tally_health(Comparison& c, uint32_t step, const FluidHealth& cpu, const FluidHealth& cand) {
    const Reports& r = c.reports;
    if (r.fault) {
        tally(c, step, "fault", 0, cpu.fault != cand.fault, cpu.fault != cand.fault ? 1.0 : 0.0);
        if (cpu.fault != FluidFault::none || cand.fault != FluidFault::none) {
            tally(c, step, "fault quantity", 0, cpu.quantity != cand.quantity,
                  cpu.quantity != cand.quantity ? 1.0 : 0.0);
            tally(c, step, "fault faces", 0, cpu.faces != cand.faces, cpu.faces != cand.faces ? 1.0 : 0.0);
            tally(c, step, "fault index", 0, cpu.index != cand.index, cpu.index != cand.index ? 1.0 : 0.0);
        }
    }
    if (r.unconverged) {
        tally(c, step, "unconverged", 0, cpu.unconverged != cand.unconverged,
              cpu.unconverged != cand.unconverged ? 1.0 : 0.0);
    }
    if (r.pressure_iterations) {
        tally(c, step, "pressure_iterations", 0, cpu.pressure_iterations != cand.pressure_iterations,
              std::fabs(static_cast<double>(cand.pressure_iterations) - static_cast<double>(cpu.pressure_iterations)));
    }
    if (r.pressure_residual) {
        tally(c, step, "pressure_residual", 0, bits_differ(cpu.pressure_residual, cand.pressure_residual),
              std::fabs(static_cast<double>(cand.pressure_residual) - static_cast<double>(cpu.pressure_residual)));
    }
    if (r.stalled) tally(c, step, "stalled", 0, cpu.stalled != cand.stalled, cpu.stalled != cand.stalled ? 1.0 : 0.0);
    if (r.capped) tally(c, step, "capped", 0, cpu.capped != cand.capped, cpu.capped != cand.capped ? 1.0 : 0.0);
}

inline void tally_load(Comparison& c, uint32_t step, const WallLoad& cpu, const WallLoad& cand) {
    for (glm::length_t a = 0; a < 3; ++a) {
        const auto i = static_cast<std::size_t>(a);
        tally(c, step, "force", i, bits_differ(cpu.force[a], cand.force[a]),
              std::fabs(static_cast<double>(cand.force[a]) - static_cast<double>(cpu.force[a])));
    }
    for (glm::length_t a = 0; a < 3; ++a) {
        const auto i = static_cast<std::size_t>(a);
        tally(c, step, "moment", i, bits_differ(cpu.moment[a], cand.moment[a]),
              std::fabs(static_cast<double>(cand.moment[a]) - static_cast<double>(cpu.moment[a])));
    }
}

// One solid array by its elements' bits; an element one side lacks differs.
template <class T>
void tally_solid_array(Comparison& c, const std::vector<T>& cpu, const std::vector<T>& cand) {
    const std::size_t n = std::max(cpu.size(), cand.size());
    c.solids_compared += n;
    for (std::size_t i = 0; i < n; ++i) {
        if (i >= cpu.size() || i >= cand.size() || std::memcmp(&cpu[i], &cand[i], sizeof(T)) != 0) {
            ++c.solids_differing;
        }
    }
}

// The wall links' floats, 12 to a link.
[[nodiscard]] inline std::vector<float> link_floats(const std::vector<WallLinks>& links) {
    std::vector<float> out;
    out.reserve(links.size() * 12u);
    for (const WallLinks& l : links) {
        out.insert(out.end(), l.inv_f.begin(), l.inv_f.end());
        out.insert(out.end(), l.wall.begin(), l.wall.end());
    }
    return out;
}

// The arrays the step reads: per component theta, the wall velocity, phi,
// the link index, the free-slip layer and its two fill lists; then the wall
// links, and whether there is any solid.
inline void tally_solids(Comparison& c, const SolidField& cpu, const SolidField& cand) {
    for (std::size_t a = 0; a < 3; ++a) {
        tally_solid_array(c, cpu.theta[a], cand.theta[a]);
        tally_solid_array(c, cpu.wall[a], cand.wall[a]);
        tally_solid_array(c, cpu.phi[a], cand.phi[a]);
        tally_solid_array(c, cpu.link[a], cand.link[a]);
        tally_solid_array(c, cpu.layer[a], cand.layer[a]);
        tally_solid_array(c, cpu.fill1[a], cand.fill1[a]);
        tally_solid_array(c, cpu.fill2[a], cand.fill2[a]);
    }
    tally_solid_array(c, link_floats(cpu.links), link_floats(cand.links));
    ++c.solids_compared;
    if (cpu.any != cand.any) ++c.solids_differing;
}

// The candidate against the CPU's run of the same box in LiveRun's step
// order, its solids built from the box's parts by set_solids() as LiveRun
// builds them, for `steps` steps from the box's initial state. Refused: a
// refusal on either side, or a CPU step that faults (a frozen run would
// compare equal and prove nothing).
[[nodiscard]] inline spade::Result<Comparison> compare_with_cpu(const B7Box& box, const MakeStepper& make,
                                                                uint32_t steps) {
    spade::Result<FluidScratch> s = box_scratch(box);
    if (!s) return std::unexpected(s.error());
    CpuStandIn cpu(box.grid, box.bc, box.cfg, std::move(*s));
    spade::Result<std::unique_ptr<FluidStepper>> made = make(box.grid, box.bc, box.cfg, &box.solids);
    if (!made) return std::unexpected(made.error());
    FluidStepper& cand = **made;

    Comparison c;
    c.candidate = cand.name();
    c.reports = cand.reports();
    c.steps = steps;
    if (c.reports.solids) {
        SolidField cpu_solids;
        SolidField cand_solids;
        if (spade::Result<void> r = cpu.read_solids(cpu_solids); !r) return std::unexpected(r.error());
        if (spade::Result<void> r = cand.read_solids(cand_solids); !r) return std::unexpected(r.error());
        tally_solids(c, cpu_solids, cand_solids);
    }

    MacGrid a;
    MacGrid b;
    for (uint32_t step = 1; step <= steps; ++step) {
        const spade::Result<FluidHealth> ha = cpu.step(box.discs);
        if (!ha) return std::unexpected(ha.error());
        if (ha->fault != FluidFault::none) {
            return std::unexpected(spade::Error{spade::Code::internal, "the CPU's run faulted at step " +
                                                                           std::to_string(step) + ": " +
                                                                           describe(*ha, box.grid.shape)});
        }
        const spade::Result<FluidHealth> hb = cand.step(box.discs);
        if (!hb) return std::unexpected(hb.error());
        if (spade::Result<void> r = cpu.read(a); !r) return std::unexpected(r.error());
        if (spade::Result<void> r = cand.read(b); !r) return std::unexpected(r.error());
        tally_field(c, step, "u", a.u, b.u, c.largest_gap[0]);
        tally_field(c, step, "v", a.v, b.v, c.largest_gap[1]);
        tally_field(c, step, "w", a.w, b.w, c.largest_gap[2]);
        tally_field(c, step, "p", a.p, b.p, c.largest_gap[3]);
        tally_health(c, step, *ha, *hb);
        if (c.reports.wall_load) {
            const spade::Result<WallLoad> la = cpu.wall_load(box.about);
            if (!la) return std::unexpected(la.error());
            const spade::Result<WallLoad> lb = cand.wall_load(box.about);
            if (!lb) return std::unexpected(lb.error());
            tally_load(c, step, *la, *lb);
        }
    }
    for (const float x : a.v) c.cpu_speed = std::max(c.cpu_speed, std::fabs(static_cast<double>(x)));
    return c;
}

// The candidate's times on the box.
struct Timing {
    double step_ms = 0.0;     // the median of M steps after W warm-up
    double film_s = 0.0;      // 0.50 s of the box's time, published at each frame
    uint32_t film_steps = 0;  // the steps that took
};

// The step time with nothing read back between the timed steps; then, from
// the initial state again, the film's 0.50 s, published at each frame as
// LiveRun's advance_frame() does: frame k at the first step at or past
// round(k / kFramesPerSecond) s.
[[nodiscard]] inline spade::Result<Timing> time_candidate(const B7Box& box, const MakeStepper& make,
                                                          const RunPlan& plan) {
    using clock = std::chrono::steady_clock;
    Timing t;
    if (plan.timed > 0) {
        spade::Result<std::unique_ptr<FluidStepper>> made = make(box.grid, box.bc, box.cfg, &box.solids);
        if (!made) return std::unexpected(made.error());
        std::vector<double> ms;
        for (uint32_t k = 0; k < plan.warm + plan.timed; ++k) {
            const clock::time_point t0 = clock::now();
            const spade::Result<FluidHealth> h = (*made)->step(box.discs);
            const clock::time_point t1 = clock::now();
            if (!h) return std::unexpected(h.error());
            if (k >= plan.warm) ms.push_back(std::chrono::duration<double, std::milli>(t1 - t0).count());
        }
        std::sort(ms.begin(), ms.end());
        t.step_ms = ms[ms.size() / 2];
    }
    if (!plan.film) return t;
    spade::Result<std::unique_ptr<FluidStepper>> made = make(box.grid, box.bc, box.cfg, &box.solids);
    if (!made) return std::unexpected(made.error());
    const GridShape& s = box.grid.shape;
    const PublishInput in{
        .origin = -0.5f * s.dx * glm::vec3(static_cast<float>(s.nx), static_cast<float>(s.ny), static_cast<float>(s.nz)),
        .solid = box.solid_cells,
        .periodic = periodic_axes(box.bc),
        .theta = {box.solids.theta[0], box.solids.theta[1], box.solids.theta[2]}};
    spade::FlowVolume vol;
    uint64_t steps = 0;
    const clock::time_point t0 = clock::now();
    for (uint64_t k = 1; k <= kFilmFrames; ++k) {
        const uint64_t end_ns = (k * kNsPerSecond + an::kFramesPerSecond / 2) / an::kFramesPerSecond;
        while (steps * box.h_ns < end_ns) {
            if (const spade::Result<FluidHealth> h = (*made)->step(box.discs); !h) return std::unexpected(h.error());
            ++steps;
        }
        if (spade::Result<void> r = (*made)->publish(in, vol); !r) return std::unexpected(r.error());
    }
    t.film_s = std::chrono::duration<double>(clock::now() - t0).count();
    t.film_steps = static_cast<uint32_t>(steps);
    return t;
}

// What measure 1 compares: everything, or what the candidate reports with
// the rest named not compared.
[[nodiscard]] inline std::string compared_values(const Reports& r) {
    const std::vector<std::string> left_out = report_names(r, false);
    if (left_out.empty()) return "u, v, w and p, each step's health and wall load";
    std::vector<std::string> health = report_names(r, true);
    std::erase_if(health, [](const std::string& n) { return n == "the solids"; });
    return "u, v, w and p" + (health.empty() ? std::string() : ", " + joined(health)) +
           "; not compared: " + joined(left_out);
}

// B7's record block as the run prints it. The test reads it back through the
// record reader, so a format slip fails here and not at the paste. The
// verdicts are the call site's. The solids' measure is there when they were
// compared, the step time's when steps were timed, the film's when it ran.
[[nodiscard]] inline std::string b7_block(const std::string& box, const std::string& config,
                                          const std::string& build, const RunPlan& plan, const Comparison& c,
                                          const Timing& t, bool bits_within, bool solids_within, bool time_within) {
    const std::string film = plan.film ? airflow_test::formatted([&](char* b, std::size_t n) {
        return std::snprintf(b, n, "0.50 s of the box's time in %u steps, published at every 1/30 s", t.film_steps);
    })
                                       : std::string("the film's 0.50 s not run on this box");
    const std::string timed =
        plan.timed > 0 ? airflow_test::formatted([&](char* b, std::size_t n) {
            return std::snprintf(b, n, "the step time the median of %u steps after %u warm-up", plan.timed,
                                 plan.warm);
        })
                       : std::string("the step not timed");
    const std::string first =
        c.first_step == 0 ? std::string("none differ") : airflow_test::formatted([&](char* b, std::size_t n) {
            return std::snprintf(b, n,
                                 "%s at index %zu first, |gap| %.3e; the largest |gap| u %.3e, v %.3e, w %.3e, "
                                 "p %.3e",
                                 c.first_field.c_str(), c.first_index, c.first_gap, c.largest_gap[0],
                                 c.largest_gap[1], c.largest_gap[2], c.largest_gap[3]);
        });
    const char* bit_equal =
        "tolerance: 0, bit-equal (TD-14 as amended 2026-10-22: on the device of record, every kernel exact and both "
        "twins flush)\n";
    const std::string values = compared_values(c.reports);
    std::string block = airflow_test::formatted([&](char* b, std::size_t n) {
        return std::snprintf(
            b, n,
            "=== validation record: B7 acceptance, %s, %s against the CPU ===\n"
            "path: %s, %s\n"
            "config: %s; steps 1 to %u compared from the initial state; %s; %s\n"
            "measure: the values differing from the CPU's run over steps 1 to %u (%s) = %llu of %llu\n"
            "reference: the CPU's run of the same box, LiveRun's step order (engine/analysis/live_run.cpp)\n"
            "%s"
            "verdict: %s\n",
            box.c_str(), c.candidate.c_str(), c.candidate.c_str(), build.c_str(), config.c_str(), c.steps,
            timed.c_str(), film.c_str(), c.steps, values.c_str(), static_cast<unsigned long long>(c.differing),
            static_cast<unsigned long long>(c.compared), bit_equal, bits_within ? "within" : "MISS");
    });
    if (c.reports.solids) {
        block += airflow_test::formatted([&](char* b, std::size_t n) {
            return std::snprintf(b, n,
                                 "measure: the solids at init differing (the solid field's arrays; the CPU has no "
                                 "packed form) = %llu of %llu\n"
                                 "reference: the CPU's solid field for the same box, built as LiveRun builds it "
                                 "(set_solids())\n"
                                 "%s"
                                 "verdict: %s\n",
                                 static_cast<unsigned long long>(c.solids_differing),
                                 static_cast<unsigned long long>(c.solids_compared), bit_equal,
                                 solids_within ? "within" : "MISS");
        });
    }
    block += airflow_test::formatted([&](char* b, std::size_t n) {
        return std::snprintf(b, n,
                             "measure: the first differing step, 0 if none (%s) = %u\n"
                             "reference: the CPU's run of the same box\n"
                             "tolerance: none (report only)\n"
                             "verdict: report only\n",
                             first.c_str(), c.first_step);
    });
    if (plan.timed > 0) {
        block += airflow_test::formatted([&](char* b, std::size_t n) {
            return std::snprintf(b, n,
                                 "measure: the candidate's step time, the median of %u steps after %u warm-up = "
                                 "%.1f ms\n"
                                 "reference: the plan's expectation, about 95 ms at the D1 box "
                                 "(docs/design/physics/plans/2026-10-22-airflow-gpu-solver-plan.md, B7)\n"
                                 "tolerance: <= 190 ms (the plan's 2x profiling trigger, B7, fixed before the run)\n"
                                 "verdict: %s\n",
                                 plan.timed, plan.warm, t.step_ms, time_within ? "within" : "MISS");
        });
    }
    if (!plan.film) return block;
    return block + airflow_test::formatted([&](char* b, std::size_t n) {
               return std::snprintf(b, n,
                                    "measure: the wall time for 0.50 s of the box's time (%u steps), publishing at "
                                    "the film's frame times = %.2f s\n"
                                    "reference: the plan's estimate, about 2 minutes at the D1 box (B7)\n"
                                    "tolerance: none (report only)\n"
                                    "verdict: report only\n",
                                    t.film_steps, t.film_s);
           });
}

// The harness on one box: the comparison, the times, and the block judged.
struct B7Run {
    Comparison c;
    Timing t;
    std::string block;
};

[[nodiscard]] inline spade::Result<B7Run> run_b7(const B7Box& box, const MakeStepper& make,
                                                 const RunPlan& plan = kFullPlan) {
    spade::Result<Comparison> c = compare_with_cpu(box, make, plan.compared);
    if (!c) return std::unexpected(c.error());
    spade::Result<Timing> t = time_candidate(box, make, plan);
    if (!t) return std::unexpected(t.error());
    B7Run run{std::move(*c), *t, {}};
    run.block = b7_block(box.name, box.config, airflow_test::build_line(), plan, run.c, run.t, run.c.differing == 0u,
                         run.c.solids_differing == 0u, run.t.step_ms <= kStepBoundMs);
    return run;
}

// The values a step compares for a candidate reporting `r`: u, v, w and p,
// one per health field reported (a fault's details only where one faulted),
// and the wall load's 6 components.
[[nodiscard]] inline uint64_t values_per_step(const MacGrid& g, const Reports& r) {
    const std::vector<std::string> on = report_names(r, true);
    const auto health = static_cast<uint64_t>(std::count_if(
        on.begin(), on.end(), [](const std::string& n) { return n != "the wall load" && n != "the solids"; }));
    return g.u.size() + g.v.size() + g.w.size() + g.p.size() + health + (r.wall_load ? 6u : 0u);
}

[[nodiscard]] inline bool validation_enabled() {
#if defined(_MSC_VER)
#pragma warning(push)
#pragma warning(disable : 4996)
#endif
    return std::getenv("SPADE_AIRFLOW_VALIDATION") != nullptr;
#if defined(_MSC_VER)
#pragma warning(pop)
#endif
}

}  // namespace b7
