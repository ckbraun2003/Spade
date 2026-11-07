// ---------------------------------------------------------------------------
// test_lbm_cost.cpp -- the LBM tier's cost and coverage functions
// (engine/physics/lbm/lbm_cost.hpp; L1, design section 8):
//
//   LbmCost       lbm_cost's arithmetic, lbm_seconds over the recorded rows,
//                 and the rows pinned to the records' numbers
//   LbmCoverage   the coverage table: what lbm_covers accepts and refuses
//   LbmUnits      the lattice units of a physical case
// ---------------------------------------------------------------------------
#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <filesystem>
#include <limits>
#include <string>
#include <vector>

#include "analysis/validation.hpp"
#include "physics/lbm/lbm_cost.hpp"

namespace {

namespace an = spade::analysis;
namespace lbm = spade::physics::lbm;
namespace fs = std::filesystem;

}  // namespace

TEST(LbmCost, TheCostIsNodesStepsAndBytes) {
    const lbm::LbmCost c = lbm::lbm_cost(128, 64, 32, 1000);
    EXPECT_EQ(c.nodes, uint64_t{128} * 64 * 32);
    EXPECT_EQ(c.steps, uint64_t{1000});
    EXPECT_EQ(c.node_updates, uint64_t{128} * 64 * 32 * 1000);
    EXPECT_EQ(c.bytes, uint64_t{128} * 64 * 32 * 120);
    EXPECT_EQ(lbm::kLbmBytesPerNode, 120u);
    // The size the plan states: one buffer of 108 B a node holds f (27 x 4); u is the other 12.
    EXPECT_EQ(lbm::kLbmBytesPerNode, 27u * 4u + 3u * 4u);
}

TEST(LbmCost, TheSecondsAreTheUpdatesOverTheRecordedRate) {
    const lbm::LbmCost c = lbm::lbm_cost(64, 64, 64, 2037);  // C0b at 64^3
    const auto one = lbm::lbm_seconds(c, lbm::LbmRung::r0, lbm::kCpuOneThread);
    const auto six = lbm::lbm_seconds(c, lbm::LbmRung::r0, lbm::kCpuSixThreads);
    ASSERT_TRUE(one.has_value());
    ASSERT_TRUE(six.has_value());
    EXPECT_NEAR(*one, 64.0 * 64 * 64 * 2037 / 0.792e6, 1e-6);
    EXPECT_NEAR(*six, 64.0 * 64 * 64 * 2037 / 3.652e6, 1e-6);
    EXPECT_GT(*one, *six);
    // 535 million updates take about 11 minutes on one thread (the stability map's per-point time).
    EXPECT_GT(*one, 600.0);
    EXPECT_LT(*one, 720.0);
}

TEST(LbmCost, NoRowMeansNoNumber) {
    const lbm::LbmCost c = lbm::lbm_cost(64, 64, 64, 100);
    // Two threads, four threads and the device have no row: nothing is extrapolated.
    EXPECT_FALSE(lbm::lbm_seconds(c, lbm::LbmRung::r0, lbm::LbmDevice{lbm::LbmDevice::Kind::cpu, 2}).has_value());
    EXPECT_FALSE(lbm::lbm_seconds(c, lbm::LbmRung::r0, lbm::LbmDevice{lbm::LbmDevice::Kind::cpu, 4}).has_value());
    EXPECT_FALSE(lbm::lbm_seconds(c, lbm::LbmRung::r0, lbm::LbmDevice{lbm::LbmDevice::Kind::vulkan, 1}).has_value());
}

TEST(LbmCost, EveryRowNamesItsMachineDateAndBoxState) {
    for (const lbm::LbmCostRow& row : lbm::kLbmCostRows) {
        EXPECT_GT(row.mlups, 0.0);
        EXPECT_NE(std::string(row.machine), "");
        EXPECT_NE(std::string(row.box_state), "");
        EXPECT_EQ(std::string(row.date).size(), std::size_t{10}) << row.date;  // yyyy-mm-dd
        EXPECT_NE(std::string(row.record_identity), "");
    }
    // One row a (rung, device): lbm_cost_row's first match is the only one.
    for (std::size_t a = 0; a < std::size(lbm::kLbmCostRows); ++a) {
        for (std::size_t b = a + 1; b < std::size(lbm::kLbmCostRows); ++b) {
            EXPECT_FALSE(lbm::kLbmCostRows[a].rung == lbm::kLbmCostRows[b].rung &&
                         lbm::kLbmCostRows[a].device == lbm::kLbmCostRows[b].device);
        }
    }
}

// The table's numbers ARE the records': each row's rate equals the measure it names, read from the records folder.
// Mutation: a row's rate edited (0.792 to 0.79 in one place, say).
TEST(LbmCost, TheRowsAreTheRecordsNumbers) {
    const spade::Result<std::vector<an::ValidationRecord>> r =
        an::validation_report(fs::path(SPADE_TESTS_DIR) / "data" / "airflow" / "records");
    ASSERT_TRUE(r.has_value()) << r.error().context;
    for (const lbm::LbmCostRow& row : lbm::kLbmCostRows) {
        const an::ValidationRecord* found = nullptr;
        for (const an::ValidationRecord& rec : *r) {
            if (rec.source == row.record_file && rec.identity == row.record_identity) found = &rec;
        }
        ASSERT_NE(found, nullptr) << row.record_file << ": no record \"" << row.record_identity << "\"";
        const an::ValidationMeasure* measure = nullptr;
        for (const an::ValidationMeasure& m : found->measures) {
            if (m.quantity == row.record_quantity) measure = &m;
        }
        ASSERT_NE(measure, nullptr) << found->identity << ": no measure \"" << row.record_quantity << "\"";
        EXPECT_EQ(measure->unit, "MLUPS") << found->identity;
        EXPECT_EQ(measure->number, row.mlups) << found->identity;
        EXPECT_EQ(found->date, std::string(row.date)) << found->identity << ": the row's date is the record's";
    }
}

TEST(LbmCoverage, R0IsStatedUpTo0p275AtAnyViscosity) {
    const lbm::LbmCoverage* c = lbm::lbm_coverage(lbm::LbmRung::r0);
    ASSERT_NE(c, nullptr);
    EXPECT_EQ(c->max_u_lat, 0.275);
    EXPECT_EQ(c->edge_row_u_lat, 0.254);
    EXPECT_EQ(c->first_break, 0.30);
    // The five map viscosities, down to 0, at the stated edge and at L5's nominal row.
    for (const double nu : {0.0, 1e-6, 1e-5, 1e-4, 1e-3}) {
        EXPECT_TRUE(lbm::lbm_covers(lbm::LbmRung::r0, 0.275, nu).covered) << nu;
        EXPECT_TRUE(lbm::lbm_covers(lbm::LbmRung::r0, 0.254, nu).covered) << nu;
        EXPECT_TRUE(lbm::lbm_covers(lbm::LbmRung::r0, 0.196, nu).covered) << nu;  // L5's run, min(0.254, 0.275 / 1.4)
    }
}

TEST(LbmCoverage, APointAboveTheEdgeIsRefusedNamingThePointAndTheRung) {
    // 0.28 sits between the stated edge and the first break at 0.30: not covered either.
    for (const double u : {0.2751, 0.28, 0.30, 0.4}) {
        const lbm::LbmCovered v = lbm::lbm_covers(lbm::LbmRung::r0, u, 1e-5);
        EXPECT_FALSE(v.covered) << u;
        EXPECT_NE(v.why.find("R0"), std::string::npos) << v.why;
        EXPECT_NE(v.why.find("u_lat " + std::to_string(u)), std::string::npos) << v.why;
        EXPECT_NE(v.why.find("nu_lat"), std::string::npos) << v.why;
        EXPECT_NE(v.why.find(std::to_string(0.275)), std::string::npos) << v.why;
    }
}

TEST(LbmCoverage, NonsenseIsRefusedNotCovered) {
    const double nan = std::numeric_limits<double>::quiet_NaN();
    EXPECT_FALSE(lbm::lbm_covers(lbm::LbmRung::r0, nan, 1e-5).covered);
    EXPECT_FALSE(lbm::lbm_covers(lbm::LbmRung::r0, 0.1, nan).covered);
    EXPECT_FALSE(lbm::lbm_covers(lbm::LbmRung::r0, 0.0, 1e-5).covered);
    EXPECT_FALSE(lbm::lbm_covers(lbm::LbmRung::r0, -0.1, 1e-5).covered);
    EXPECT_FALSE(lbm::lbm_covers(lbm::LbmRung::r0, 0.1, -1e-6).covered);
}

// The coverage numbers are the design's section 8 table.
TEST(LbmCoverage, TheTableIsTheDesignsSection8) {
    // docs/design/physics/plans/2026-11-04-l1-lbm-design.md, "The coverage table": R0, max u_lat 0.275, edge row 0.254.
    const lbm::LbmCoverage& c = lbm::kLbmCoverage[0];
    EXPECT_EQ(c.rung, lbm::LbmRung::r0);
    EXPECT_EQ(c.max_u_lat, 0.275);
    EXPECT_EQ(c.edge_row_u_lat, 0.254);
}

TEST(LbmUnits, ADurationBecomesStepsAndViscosityBecomesNuLat) {
    // Re 1600 Taylor-Green on a 0.1 m cell: u_ref 1 m/s, u_lat 0.1 -> dt = 0.01 m / (1 m/s) = 0.01 s.
    const double dt = lbm::lbm_dt(1.0, 0.1, 0.1);
    EXPECT_NEAR(dt, 0.01, 1e-15);
    EXPECT_NEAR(lbm::lbm_u_lat(1.0, dt, 0.1), 0.1, 1e-15);
    EXPECT_NEAR(lbm::lbm_nu_lat(1.4607e-5, dt, 0.1), 1.4607e-5 * 0.01 / 0.01, 1e-18);  // nu dt / dx^2
    EXPECT_EQ(lbm::lbm_steps(1.0, dt), uint64_t{100});
    EXPECT_EQ(lbm::lbm_steps(1.005, dt), uint64_t{101});  // a partial step is a step
    EXPECT_EQ(lbm::lbm_steps(0.0, dt), uint64_t{0});
}
