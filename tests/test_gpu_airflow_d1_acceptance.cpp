// B7's acceptance harness (tests/b7_harness.hpp) proved on CPU stand-ins:
// LiveRun's step through the seam against the CPU's own run, to the bit, in
// the stand-in box and the real D1 box, and a nudged run read as a MISS at
// its first wrong step. Rendering's GpuFluid runs the same harness in
// test_gpu_airflow_d1_device.cpp. The runs are opt-in
// (SPADE_AIRFLOW_VALIDATION), about 20 s each in release, the D1 one a few
// minutes; the record's host test runs always.
#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <limits>
#include <span>
#include <string>
#include <utility>

#include "b7_harness.hpp"

namespace {

using namespace b7;

// The MISS control: the CPU stand-in with one value wrong from step s0. After
// step s0's fluid step one v sample is put 1 ulp up, a last bit gone wrong in
// the step's output, so the run first differs at step s0, in that sample
// alone; the later steps carry it on.
class NudgedCpuStandIn : public CpuStandIn {
  public:
    NudgedCpuStandIn(const MacGrid& g, const DomainBc& bc, const FluidConfig& cfg, FluidScratch s, uint32_t at_step,
                     std::size_t v_sample)
        : CpuStandIn(g, bc, cfg, std::move(s)), at_step_(at_step), v_sample_(v_sample) {}

    [[nodiscard]] static MakeStepper maker(uint32_t at_step, std::size_t v_sample) {
        return [=](const MacGrid& g, const DomainBc& bc, const FluidConfig& cfg,
                   const SolidField* solids) -> spade::Result<std::unique_ptr<FluidStepper>> {
            spade::Result<FluidScratch> s = scratch_with(g, bc, cfg, solids);
            if (!s) return std::unexpected(s.error());
            return std::make_unique<NudgedCpuStandIn>(g, bc, cfg, std::move(*s), at_step, v_sample);
        };
    }

    [[nodiscard]] std::string name() const override {
        return "nudged CPU stand-in (one v sample 1 ulp up after step " + std::to_string(at_step_) + ")";
    }

    [[nodiscard]] spade::Result<FluidHealth> step(std::span<const ActuatorDisc> discs) override {
        spade::Result<FluidHealth> health = CpuStandIn::step(discs);
        if (steps_ == at_step_) {
            grid_.v[v_sample_] = std::nextafter(grid_.v[v_sample_], std::numeric_limits<float>::infinity());
        }
        return health;
    }

  private:
    uint32_t at_step_ = 0;
    std::size_t v_sample_ = 0;
};

// The harness proved on the CPU stand-in: LiveRun's step through the seam,
// its solids set from the built field, against the CPU's own run, its solids
// from set_solids(), to the bit. Its step time is a CPU's, so that verdict is
// printed, not asserted.
TEST(B7Acceptance, TheCpuStandInIsBitEqualToTheCpu) {
    if (!validation_enabled()) GTEST_SKIP() << "set SPADE_AIRFLOW_VALIDATION=1 to run the B7 stand-in (~20 s)";
    const spade::Result<B7Box> box = stand_in_box();
    ASSERT_TRUE(box.has_value()) << box.error().context;
    const spade::Result<B7Run> run = run_b7(*box, &CpuStandIn::create);
    ASSERT_TRUE(run.has_value()) << run.error().context;
    std::fputs(run->block.c_str(), stdout);
    const std::string physical = physical_line(*run);
    std::fputs(physical.c_str(), stdout);
    EXPECT_TRUE(run->physical) << physical;
    EXPECT_LE(run->t.film_peak, run->bound) << "the film's flow stays physical too";
    const auto record = airflow_test::read_as_record(run->block, "b7-acceptance.md");
    ASSERT_TRUE(record.has_value()) << record.error().context;
    ASSERT_EQ(record->size(), 1u);
    ASSERT_EQ(record->at(0).measures.size(), 5u);
    EXPECT_EQ(record->at(0).measures[0].verdict, an::Verdict::within);
    EXPECT_EQ(record->at(0).measures[1].verdict, an::Verdict::within);
    // Health: the fault, unconverged and the four counts (no fault, so not its
    // details); the wall load's 6 components.
    const MacGrid& g = box->grid;
    EXPECT_EQ(values_per_step(g, Reports{}), g.u.size() + g.v.size() + g.w.size() + g.p.size() + 6u + 6u);
    EXPECT_EQ(run->c.compared, kComparedSteps * values_per_step(g, Reports{}));
    EXPECT_EQ(run->c.differing, 0u);
    EXPECT_GT(run->c.solids_compared, 0u);
    EXPECT_EQ(run->c.solids_differing, 0u);
    EXPECT_EQ(run->c.first_step, 0u);
    EXPECT_GT(run->c.cpu_speed, 0.1) << "the disc must move the air, or the comparison proves nothing";
    EXPECT_EQ(run->t.film_steps, 100u);
}

// The harness on the real D1 box, the one the GPU run takes: the CPU stand-in
// against the CPU for 2 steps, one step timed (a CPU's, so its verdict is
// printed, not asserted), the film left out. A few minutes in release.
TEST(B7Acceptance, TheCpuStandInIsBitEqualToTheCpuInTheD1Box) {
    if (!validation_enabled()) GTEST_SKIP() << "set SPADE_AIRFLOW_VALIDATION=1 to run the B7 stand-in in D1 (~3 min)";
    const spade::Result<B7Box> box = d1_box();
    ASSERT_TRUE(box.has_value()) << box.error().context;
    ASSERT_EQ(box->discs.size(), 4u);
    const spade::Result<B7Run> run = run_b7(*box, &CpuStandIn::create, kD1StandInPlan);
    ASSERT_TRUE(run.has_value()) << run.error().context;
    std::fputs(run->block.c_str(), stdout);
    const std::string physical = physical_line(*run);
    std::fputs(physical.c_str(), stdout);
    EXPECT_TRUE(run->physical) << physical;
    const auto record = airflow_test::read_as_record(run->block, "b7-acceptance.md");
    ASSERT_TRUE(record.has_value()) << record.error().context;
    ASSERT_EQ(record->size(), 1u);
    ASSERT_EQ(record->at(0).measures.size(), 4u) << "the film is left out";
    EXPECT_EQ(record->at(0).measures[0].verdict, an::Verdict::within);
    EXPECT_EQ(record->at(0).measures[1].verdict, an::Verdict::within);
    EXPECT_EQ(run->c.compared, kD1StandInPlan.compared * values_per_step(box->grid, Reports{}));
    EXPECT_EQ(run->c.differing, 0u);
    EXPECT_EQ(run->c.solids_differing, 0u);
    EXPECT_EQ(run->c.first_step, 0u);
    EXPECT_GT(run->c.cpu_speed, 1.0) << "the wind and the discs must move the air";
}

// The MISS control: one v sample 1 ulp up after step s0's fluid step, on the
// disc's axis 2 cells below it. The comparison must find it at step s0, in
// that sample and nothing else there, and the block must read MISS.
TEST(B7Acceptance, ANudgedRunIsAMissAtItsFirstStep) {
    if (!validation_enabled()) GTEST_SKIP() << "set SPADE_AIRFLOW_VALIDATION=1 to run the B7 stand-in (~20 s)";
    const spade::Result<B7Box> box = stand_in_box();
    ASSERT_TRUE(box.has_value()) << box.error().context;
    const std::size_t sample = v_index(box->grid.shape, 16, 28, 11);
    ASSERT_LT(sample, box->grid.v.size());
    const spade::Result<B7Run> run = run_b7(*box, NudgedCpuStandIn::maker(kNudgeStep, sample));
    ASSERT_TRUE(run.has_value()) << run.error().context;
    std::fputs(run->block.c_str(), stdout);
    const std::string physical = physical_line(*run);
    std::fputs(physical.c_str(), stdout);
    EXPECT_TRUE(run->physical) << physical;
    const auto record = airflow_test::read_as_record(run->block, "b7-acceptance.md");
    ASSERT_TRUE(record.has_value()) << record.error().context;
    ASSERT_EQ(record->size(), 1u);
    ASSERT_EQ(record->at(0).measures.size(), 5u);
    EXPECT_EQ(record->at(0).measures[0].verdict, an::Verdict::miss);
    EXPECT_EQ(record->at(0).measures[1].verdict, an::Verdict::within);
    EXPECT_EQ(record->at(0).measures[2].number, static_cast<double>(kNudgeStep));
    EXPECT_GT(run->c.differing, 0u);
    EXPECT_EQ(run->c.solids_differing, 0u);
    EXPECT_EQ(run->c.first_step, kNudgeStep);
    EXPECT_EQ(run->c.first_field, "v");
    EXPECT_EQ(run->c.first_index, sample);
    EXPECT_EQ(run->c.first_step_differing, 1u) << "step s0 differs in the nudged sample alone";
    EXPECT_GT(run->c.first_gap, 0.0);
    EXPECT_LT(run->c.first_gap, 1.0e-6) << "one ulp of a velocity near 1 m/s";
}

}  // namespace

// The block reads as one validation record of five measures: measures 1, 2
// and 4 judged, within on a bit-equal run inside the step bound and MISS on
// one with differing values past it; 3 and 5 report only; four measures for
// a plan without the film. A candidate that reports less names the rest "not
// compared" in measure 1 and, without the solids, drops their measure.
TEST(B7AcceptanceRecord, TheBlockReadsAsAValidationRecord) {
    Comparison within;
    within.candidate = "a test candidate";
    within.steps = 32;
    within.compared = 4000000;
    within.solids_compared = 600000;
    const Timing fast{.step_ms = 80.0, .film_s = 110.0, .film_steps = 1300};
    Comparison miss = within;
    miss.differing = 12345;
    miss.solids_differing = 7;
    miss.first_step = 5;
    miss.first_field = "v";
    miss.first_index = 4321;
    miss.first_gap = 1.192e-7;
    const Timing slow{.step_ms = 250.0, .film_s = 330.0, .film_steps = 1300};
    const std::array<an::Verdict, 5> judged_within = {an::Verdict::within, an::Verdict::within,
                                                      an::Verdict::report_only, an::Verdict::within,
                                                      an::Verdict::report_only};
    const std::array<an::Verdict, 5> judged_miss = {an::Verdict::miss, an::Verdict::miss, an::Verdict::report_only,
                                                    an::Verdict::miss, an::Verdict::report_only};
    const auto a = airflow_test::read_as_record(
        b7_block("a test box", "a test config", "a test build", kFullPlan, within, fast, true, true, true),
        "b7-test.md");
    ASSERT_TRUE(a.has_value()) << a.error().context;
    ASSERT_EQ(a->size(), 1u);
    ASSERT_EQ(a->at(0).measures.size(), 5u);
    for (std::size_t m = 0; m < 5; ++m) {
        EXPECT_EQ(a->at(0).measures[m].verdict, judged_within[m]) << a->at(0).measures[m].quantity;
    }
    EXPECT_EQ(a->at(0).measures[2].number, 0.0) << "no differing step reads 0";
    EXPECT_EQ(a->at(0).measures[0].quantity.find("not compared"), std::string::npos);
    const auto b = airflow_test::read_as_record(
        b7_block("a test box", "a test config", "a test build", kFullPlan, miss, slow, false, false, false),
        "b7-test.md");
    ASSERT_TRUE(b.has_value()) << b.error().context;
    ASSERT_EQ(b->size(), 1u);
    ASSERT_EQ(b->at(0).measures.size(), 5u);
    for (std::size_t m = 0; m < 5; ++m) {
        EXPECT_EQ(b->at(0).measures[m].verdict, judged_miss[m]) << b->at(0).measures[m].quantity;
    }
    EXPECT_EQ(b->at(0).measures[2].number, 5.0);
    EXPECT_EQ(b->at(0).measures[3].number, 250.0);
    // A plan without the film: four measures, the step time last.
    const auto d = airflow_test::read_as_record(
        b7_block("a test box", "a test config", "a test build", kD1StandInPlan, within, fast, true, true, true),
        "b7-test.md");
    ASSERT_TRUE(d.has_value()) << d.error().context;
    ASSERT_EQ(d->size(), 1u);
    ASSERT_EQ(d->at(0).measures.size(), 4u);
    EXPECT_EQ(d->at(0).measures[3].number, 80.0);

    // GpuFluid at G1: pressure_iterations alone beside u, v, w and p. The rest
    // is named not compared, and with no solids' measure the full plan has
    // four measures, the short plan (no timing, no film) two.
    Comparison gpu = within;
    gpu.reports = Reports{.fault = false, .unconverged = false, .pressure_iterations = true,
                          .pressure_residual = false, .stalled = false, .capped = false, .wall_load = false,
                          .solids = false};
    const std::string named = "u, v, w and p, pressure_iterations; not compared: fault, unconverged, "
                              "pressure_residual, stalled, capped, the wall load, the solids";
    const auto e = airflow_test::read_as_record(
        b7_block("a test box", "a test config", "a test build", kFullPlan, gpu, fast, true, true, true),
        "b7-test.md");
    ASSERT_TRUE(e.has_value()) << e.error().context;
    ASSERT_EQ(e->size(), 1u);
    ASSERT_EQ(e->at(0).measures.size(), 4u) << "no solids' measure";
    EXPECT_NE(e->at(0).measures[0].quantity.find(named), std::string::npos) << e->at(0).measures[0].quantity;
    EXPECT_EQ(e->at(0).measures[0].verdict, an::Verdict::within);
    EXPECT_EQ(e->at(0).measures[1].verdict, an::Verdict::report_only);
    EXPECT_EQ(e->at(0).measures[2].number, 80.0);
    EXPECT_EQ(e->at(0).measures[3].verdict, an::Verdict::report_only);
    const auto f = airflow_test::read_as_record(
        b7_block("a test box", "a test config", "a test build", kShortPlan, gpu, Timing{}, true, true, true),
        "b7-test.md");
    ASSERT_TRUE(f.has_value()) << f.error().context;
    ASSERT_EQ(f->size(), 1u);
    ASSERT_EQ(f->at(0).measures.size(), 2u) << "the values and the first differing step";
    EXPECT_EQ(f->at(0).measures[1].number, 0.0);
    const MacGrid one{};
    EXPECT_EQ(values_per_step(one, gpu.reports), 1u) << "pressure_iterations alone, no wall load";

    // A film whose flow diverged reads provisional, its time still the value.
    const Timing diverged{.step_ms = 35.0, .film_s = 45.0, .film_steps = 1316, .film_peak = 8.8e23};
    const auto h = airflow_test::read_as_record(
        b7_block("a test box", "a test config", "a test build", kFullPlan, gpu, diverged, true, true, true, true),
        "b7-test.md");
    ASSERT_TRUE(h.has_value()) << h.error().context;
    ASSERT_EQ(h->size(), 1u);
    ASSERT_EQ(h->at(0).measures.size(), 4u);
    EXPECT_NE(h->at(0).measures[3].quantity.find("provisional (diverged flow"), std::string::npos)
        << h->at(0).measures[3].quantity;
    EXPECT_EQ(h->at(0).measures[3].number, 45.0);
    EXPECT_EQ(e->at(0).measures[3].quantity.find("provisional"), std::string::npos);
}

// The physical check: the flow's peak over u, v and w, infinite past a NaN;
// the bound from momentum theory (the stand-in's disc has v_h 1 m/s and no
// wind: 4 x 2 m/s); its line PASS, FAIL, and FAIL known-open.
TEST(B7AcceptanceRecord, ThePhysicalCheckBoundsTheFlowAbove) {
    MacGrid g;
    g.u = {1.0f, -3.0f};
    g.v = {2.0f};
    EXPECT_EQ(peak_speed(g), 3.0);
    g.w = {std::numeric_limits<float>::quiet_NaN()};
    EXPECT_EQ(peak_speed(g), std::numeric_limits<double>::infinity());
    const spade::Result<B7Box> box = stand_in_box(BoxSolids::none);
    ASSERT_TRUE(box.has_value()) << box.error().context;
    EXPECT_NEAR(physical_bound(*box), 8.0, 1.0e-5);
    B7Run run;
    run.c.steps = 2;
    run.c.cpu_peak = 88.7;
    run.bound = 55.1;
    const std::string open = "known-open: D1 without solids, plan A's top-face blow-up (Physics' triage 1)";
    EXPECT_EQ(physical_line(run, open).rfind("physical: FAIL, " + open +
                                                 "; the CPU's largest |u|, |v| or |w| over steps 1 to 2 = 88.7 m/s "
                                                 "against 55.1 m/s (4 x (2 v_h + the wind))",
                                             0),
              0u)
        << physical_line(run, open);
    run.c.cpu_peak = 5.0;
    run.physical = true;
    EXPECT_EQ(physical_line(run).rfind("physical: PASS; ", 0), 0u) << physical_line(run);
}
