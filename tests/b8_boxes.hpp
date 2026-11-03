#pragma once

// B8's GPU records (the GPU solver plan, B8; the lead's ruling of 2026-11-03
// on Test/Docs' gaps 1 and 7): the validation cases as B7's harness runs
// them, so each case's record gets a block of GpuFluid against the CPU's run
// of the same box, bit for bit on the device of record, with the step time a
// reading. D1's is B7's own box (b7_harness.hpp's d1_box()). Here:
//
// - S1, the sphere in a uniform stream, as run_s1() sets it up
//   (test_airflow_solids_step.cpp, P3's checks), at R/dx 6 and 12;
// - S3, the sheet across a periodic channel, as
//   S3ASheetAcrossAChannelStopsItsFlow sets it up, on planes 16 and 17. The
//   CPU's record is one projection; here it is the whole step, which takes
//   that projection after the step's own advection;
// - C8q, the reference quad in still air, from its one builder
//   (c8q_case.hpp), its discs held at the commanded hover speed as D1's are.
//
// Each config line says where its case is set up, so a review can hold the
// two side by side. Needs the sandbox (c8q_case.hpp).

#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <glm/glm.hpp>

#include "b7_harness.hpp"
#include "c8q_case.hpp"

namespace b7 {

// B8's plan: 32 steps compared, the step timed (its median a reading), no
// film (that is D1's alone).
inline constexpr RunPlan kRecordPlan{.compared = kComparedSteps, .warm = kWarmSteps, .timed = kTimedSteps,
                                     .film = false};
inline constexpr const char* kRecordTitle = "B8, the whole step on the device";

[[nodiscard]] inline uint64_t whole_ns(float h) {
    return static_cast<uint64_t>(static_cast<double>(h) * static_cast<double>(kNsPerSecond) + 0.5);
}

// S1 at R/dx `n`: 8n cells a side at dx = 0.06 m / n, a sphere R 0.06 m
// centred on a cell corner at the box's centre, free slip, inviscid, the
// inflow U 1 m/s on -x and the other faces open, U h / dx 0.1, the CG's floor
// n_v 16, the interior extrapolated; u = U at the start. No discs. The wall
// load about the sphere's centre.
[[nodiscard]] inline spade::Result<B7Box> s1_box(uint32_t n) {
    B7Box box;
    const double R = 0.06;
    const float dx = static_cast<float>(R / n);
    const uint32_t cells = 8u * n;
    const GridShape shape{cells, cells, cells, dx};
    box.title = kRecordTitle;
    box.name = "S1 sphere in a uniform stream, R/dx " + std::to_string(n) + ", " + std::to_string(cells) + "^3";
    box.bc.ambient = glm::vec3(1.0f, 0.0f, 0.0f);
    const float U = 1.0f;
    const float cfl = 0.1f;
    box.cfg = FluidConfig{.rho = 1.225f, .nu = 0.0f, .h = cfl * dx / U, .n_v = 16,
                          .interior = SolidInterior::extrapolate};
    box.h_ns = whole_ns(box.cfg.h);
    spade::Result<MacGrid> g = make_grid(shape, box.bc);
    if (!g) return std::unexpected(g.error());
    box.grid = std::move(*g);
    std::fill(box.grid.u.begin(), box.grid.u.end(), U);
    const float mid = 0.5f * static_cast<float>(cells) * dx;
    auto scene = std::make_shared<Scene>();
    scene->shapes.push_back(Shape{.name = "sphere", .kind = Kind::sphere, .centre = glm::vec3(mid),
                                  .radius = static_cast<float>(R), .min_feature = static_cast<float>(2.0 * R)});
    box.set_box_solids = [scene](FluidScratch& s, const MacGrid& grid, const DomainBc& bc) {
        return set_solids(s, grid, bc, scene->parts(), scene->sampler());
    };
    spade::Result<FluidScratch> s = box_scratch(box);
    if (!s) return std::unexpected(s.error());
    box.solids = s->solids;
    box.solid_cells = s->regions.solid;
    box.about = glm::vec3(mid);
    box.config = airflow_test::formatted([&](char* b, std::size_t len) {
        return std::snprintf(b, len,
                             "S1 as run_s1() sets it up (tests/test_airflow_solids_step.cpp, P3's checks): %u^3 at dx "
                             "%.4g m, R 0.06 m centred on a cell corner, free slip, inviscid, inflow U 1 m/s on -x, "
                             "other faces open, u = U at the start, U h/dx 0.1 (h %.6g s), CG floor %u, cap %u, the "
                             "interior extrapolated; solids: %s",
                             cells, static_cast<double>(dx), static_cast<double>(box.cfg.h), box.cfg.n_v,
                             box.cfg.cg_cap, box.solids.run_line.c_str());
    });
    return box;
}

// S3's channel with its sheet on face plane `plane`: 32 x 8 x 8 at dx 1/32
// m, every axis periodic, a sheet 0.25 dx thick across the section, rho
// 1.225, inviscid, h 1 ms, the CG's floor n_v 32; u = U = 1 m/s at the
// start. The wall load about the sheet's centre.
[[nodiscard]] inline spade::Result<B7Box> s3_box(uint32_t plane) {
    B7Box box;
    const float dx = 1.0f / 32.0f;
    const GridShape shape{32, 8, 8, dx};
    box.title = kRecordTitle;
    box.name = "S3 sheet across a channel, plane " + std::to_string(plane) + (plane % 2u == 0u ? " (even)" : " (odd)");
    box.bc.face.fill(FaceBc::periodic);
    const float U = 1.0f;
    box.cfg = FluidConfig{.rho = 1.225f, .nu = 0.0f, .h = 0.001f, .n_v = 32};
    box.h_ns = whole_ns(box.cfg.h);
    spade::Result<MacGrid> g = make_grid(shape, box.bc);
    if (!g) return std::unexpected(g.error());
    box.grid = std::move(*g);
    std::fill(box.grid.u.begin(), box.grid.u.end(), U);
    struct Sheet {
        Scene scene;
        std::vector<SolidPart> parts;
    };
    auto sheet = std::make_shared<Sheet>();
    const float x = static_cast<float>(plane) * dx;
    sheet->scene.shapes.push_back(Shape{.name = "sheet", .kind = Kind::box, .centre = {x, 4.0f * dx, 4.0f * dx},
                                        .half = {0.125f * dx, 20.0f * dx, 20.0f * dx}, .min_feature = 0.25f * dx});
    sheet->parts = sheet->scene.parts();
    sheet->parts[0].extent = glm::vec3(0.25f * dx, 8.0f * dx, 8.0f * dx);
    box.set_box_solids = [sheet](FluidScratch& s, const MacGrid& grid, const DomainBc& bc) {
        return set_solids(s, grid, bc, sheet->parts, sheet->scene.sampler());
    };
    spade::Result<FluidScratch> s = box_scratch(box);
    if (!s) return std::unexpected(s.error());
    box.solids = s->solids;
    box.solid_cells = s->regions.solid;
    box.about = glm::vec3(x, 0.125f, 0.125f);
    box.config = airflow_test::formatted([&](char* b, std::size_t len) {
        return std::snprintf(b, len,
                             "S3 as S3ASheetAcrossAChannelStopsItsFlow sets it up (tests/test_airflow_solids_step.cpp): "
                             "32x8x8 periodic, dx 1/32 m, a sheet 0.25 dx thick on plane %u, rho 1.225, U 1.0 at the "
                             "start, inviscid, h 0.0010, the whole step (the CPU's record is one projection), CG floor "
                             "%u, cap %u; solids: %s",
                             plane, box.cfg.n_v, box.cfg.cg_cap, box.solids.run_line.c_str());
    });
    return box;
}

// C8q's box (c8q_case.hpp, as flown): the reference quad, its parts cleared,
// held level at hover in still air, 128^3 in a 0.975 m box (R/dx 10), h the
// case's dx/8 in whole ns as LiveRun steps it, plan A's V-cycles. LiveRun
// takes one step for its rotors' rows and held body (live_hover()); the
// discs are those rows at their commanded hover speed, held constant, as
// D1's are, so this is C8q's box, not its run (its rotors spin up from the
// hover throttle).
[[nodiscard]] inline spade::Result<B7Box> c8q_box() {
    namespace vh = spade::vehicles;
    spade::Result<spade::testing::C8qAir> q = spade::testing::c8q_air();
    if (!q) return std::unexpected(q.error());
    const GridShape& shape = q->resolved.base;
    const glm::vec3 origin = -0.5f * shape.dx *
                             glm::vec3(static_cast<float>(shape.nx), static_cast<float>(shape.ny),
                                       static_cast<float>(shape.nz));
    spade::Result<LiveHover> live = live_hover(q->resolved, q->model, q->hover.throttle);
    if (!live) return std::unexpected(live.error());

    B7Box box;
    box.title = kRecordTitle;
    box.name = "C8q reference quad, " + std::to_string(shape.nx) + "^3";
    box.bc = q->resolved.domain;
    const an::AnalysisCase& c = q->resolved.spec;
    box.h_ns = whole_ns(*c.time.step);
    box.cfg = FluidConfig{.rho = c.fluid.density, .nu = c.fluid.kinematic_viscosity,
                          .h = static_cast<float>(static_cast<double>(box.h_ns) / static_cast<double>(kNsPerSecond))};
    spade::Result<MacGrid> g = make_grid(shape, box.bc);
    if (!g) return std::unexpected(g.error());
    box.grid = std::move(*g);
    spade::Result<FluidScratch> s = box_scratch(box);
    if (!s) return std::unexpected(s.error());
    box.solids = s->solids;
    box.solid_cells = s->regions.solid;

    for (vh::RotorRow& r : live->rows) r.omega = r.omega_cmd;
    box.discs.resize(live->rows.size());
    if (spade::Result<void> d = vh::rotor_discs(live->rows, live->body, origin, shape, box.discs); !d) {
        return std::unexpected(d.error());
    }
    box.about = live->body.pos - origin;
    float thrust = 0.0f;
    for (const ActuatorDisc& d : box.discs) thrust += d.thrust;
    const double r_dx = box.discs.empty() ? 0.0 : static_cast<double>(box.discs[0].radius / shape.dx);
    box.config = airflow_test::formatted([&](char* b, std::size_t len) {
        return std::snprintf(b, len,
                             "C8q from its one builder (tests/c8q_case.hpp, as flown): the library's reference quad, "
                             "its parts cleared, held level at the box's centre in still air, no floor; %ux%ux%u at dx "
                             "%.3f mm (R/dx %.2f) in a %.3f m box; its %zu rotors as actuator discs at their commanded "
                             "hover speed, %.0f rad/s, %.3f N in all, held constant; rho %.4g, nu %.4e, h %.6f s, n_v "
                             "%u (plan A's V-cycles)",
                             shape.nx, shape.ny, shape.nz, static_cast<double>(shape.dx) * 1000.0, r_dx,
                             static_cast<double>(spade::testing::kC8qBoxM), box.discs.size(),
                             live->rows.empty() ? 0.0 : static_cast<double>(live->rows[0].omega_cmd),
                             static_cast<double>(thrust), static_cast<double>(box.cfg.rho),
                             static_cast<double>(box.cfg.nu), static_cast<double>(box.cfg.h), box.cfg.n_v);
    });
    return box;
}

}  // namespace b7
