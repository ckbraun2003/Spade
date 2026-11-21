// ---------------------------------------------------------------------------
// test_airflow_fpv_grid.cpp -- the 5-inch FPV's fine grid in the film harness (tests/d1_case.hpp): the X500's cell and rules at 1.25 mm, in a box sized
// for the OpenFrame-5F's stretched-X hubs (the lead, 2026-11-21: Cameron chose it; Interface measured the hubs from Arm.step, record
// docs/design/interface/records/2026-11-21-fpv-kit-sources.md: +-69.534 mm along and +-89.000 mm across, a 225.9 mm diagonal).
//
//   AirflowFpvGrid.TheBoxIsRectangularAtOnePointTwoFiveMillimetresAndWithinTheCaps   252 x 237 x 284 (x along the kit, z across): fewer cells than the
//                                                X500's 270 x 237 x 270; the engine's grid cap with the case's faces and solids; the device runner's 4 GiB at
//                                                234.6 B a cell
//   AirflowFpvGrid.TheHoldLeavesOneAndAHalfRadiiAboveTheHubsInWholeRows   d1_fpv_hold(hub height): 1.5 R of open air above the hubs, the kernel's
//                                                reach and 4 cells of room, about 0.2 m of wake below, a whole number of rows
//   AirflowFpvGrid.TheBoxHoldsTheOpenFramesHubsWithTheX500sSideRoom   R + 6 dx past each hub axis and 0.0165 m of room, along and across; the X500's
//                                                square 270-cell box would leave 8.75 mm across (why the box is not square)
// ---------------------------------------------------------------------------

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <filesystem>

#include <glm/glm.hpp>

#include "d1_case.hpp"
#include "phantom_case.hpp"
#include "physics/airflow/fluid_step.hpp"
#include "physics/airflow/mac_grid.hpp"

namespace {

namespace af = spade::physics::airflow;
using spade::testing::d1_fpv_hold;
using spade::testing::kD1FpvGrid;
using spade::testing::kD1FpvRotorRadius;
using spade::testing::kD1X500Grid;

constexpr double kGiB = 1073741824.0;

TEST(AirflowFpvGrid, TheBoxIsRectangularAtOnePointTwoFiveMillimetresAndWithinTheCaps) {
    EXPECT_EQ(kD1FpvGrid.nx, 252u) << "along the kit: 2 x (0.069534 + R + 6 dx + 0.0165) = 0.3141 m";
    EXPECT_EQ(kD1FpvGrid.ny, 237u);
    EXPECT_EQ(kD1FpvGrid.nz, 284u) << "across the kit: 2 x (0.089 + R + 6 dx + 0.0165) = 0.353 m";
    EXPECT_EQ(kD1FpvGrid.dx, 0.00125f);
    EXPECT_EQ(kD1FpvGrid.nx % 2u, 0u) << "an even n: an odd one puts the drone's centre on a cell centre (L12)";
    EXPECT_EQ(kD1FpvGrid.nz % 2u, 0u);
    EXPECT_LT(static_cast<uint64_t>(kD1FpvGrid.nx) * kD1FpvGrid.ny * kD1FpvGrid.nz,
              static_cast<uint64_t>(kD1X500Grid.nx) * kD1X500Grid.ny * kD1X500Grid.nz)
        << "fewer cells than the X500's box, so no more a step";
    EXPECT_EQ(2.0f * kD1FpvGrid.dx, kD1X500Grid.dx) << "half the cell";
    EXPECT_NEAR(kD1FpvRotorRadius / kD1FpvGrid.dx, 50.8, 0.01) << "R/dx 50.8, the X500's";
    const auto made = spade::testing::still_air_case(std::filesystem::path(SPADE_ASSETS_DIR) / "library/drones/fpv_5inch.model.yaml",
                                                     "assets/library/drones/fpv_5inch.model.yaml", kD1FpvGrid);
    ASSERT_TRUE(made.has_value()) << made.error().context;
    const auto& bc = made->resolved.domain;
    const uint64_t fluid = af::fluid_bytes(kD1FpvGrid, bc);
    const uint64_t solids = af::solid_bytes(kD1FpvGrid, bc);
    const double cells = static_cast<double>(kD1FpvGrid.nx) * kD1FpvGrid.ny * kD1FpvGrid.nz;
    std::printf("[ fpv     ] %ux%ux%u at 1.25 mm: %.1f M cells; fluid_bytes %.3f GiB, solid_bytes %.3f GiB, together %.3f GiB against the cap %.3f GiB; "
                "the device runner's 234.6 B a cell: %.3f GiB of 4\n",
                kD1FpvGrid.nx, kD1FpvGrid.ny, kD1FpvGrid.nz, cells / 1.0e6, static_cast<double>(fluid) / kGiB, static_cast<double>(solids) / kGiB,
                static_cast<double>(fluid + solids) / kGiB, static_cast<double>(af::kGridByteCap) / kGiB, cells * 234.6 / kGiB);
    const auto ok = af::check_grid(kD1FpvGrid, bc, true);
    EXPECT_TRUE(ok.has_value()) << (ok ? "" : ok.error().context);
    EXPECT_LE(cells * 234.6, 4.0 * kGiB) << "the device runner's own 4 GiB (compute::vram_cap), measured at 234.6 B a cell";
}

// The hub height is the kit's, above the drone's design origin (the held point): the hold is a function of it.
TEST(AirflowFpvGrid, TheHoldLeavesOneAndAHalfRadiiAboveTheHubsInWholeRows) {
    const float dx = kD1FpvGrid.dx;
    const float height = static_cast<float>(kD1FpvGrid.ny) * dx;
    const float reach = kD1FpvRotorRadius + 6.0f * dx;
    EXPECT_NEAR(d1_fpv_hold(0.0f).y, 42.0f * dx, 1.0e-6f) << "hubs at the origin: 42 rows above the box's centre";
    for (const float hub : {0.0f, 0.012f, 0.0334f, 0.0607f}) {
        const glm::vec3 hold = d1_fpv_hold(hub);
        EXPECT_EQ(hold.x, 0.0f);
        EXPECT_EQ(hold.z, 0.0f);
        const float rows = hold.y / dx;
        EXPECT_NEAR(rows, std::round(rows), 1.0e-3f) << "a whole number of rows, so the drone keeps its cell alignment (hub " << hub << ")";
        const float top_margin = 0.5f * height - hold.y - hub;
        EXPECT_GE(top_margin, 1.5f * kD1FpvRotorRadius - 1.0e-6f) << "1.5 R of open air above the hubs (hub " << hub << ")";
        EXPECT_LT(top_margin, 1.5f * kD1FpvRotorRadius + dx + 1.0e-6f) << "and not a row more than that (hub " << hub << ")";
        EXPECT_GE(top_margin, reach + 4.0f * dx) << "the kernel's reach and 4 cells of room (hub " << hub << ")";
        const float below = 0.5f * height + hold.y + hub;
        EXPECT_GE(below, 0.20f - 2.0f * dx) << "about 0.2 m of wake below the hubs (hub " << hub << ")";
        std::printf("[ fpv     ] hub %.4f m above the origin: hold y %.5f m (%.0f rows), hubs %.4f m under the top face, %.4f m of wake below\n",
                    static_cast<double>(hub), static_cast<double>(hold.y), static_cast<double>(rows), static_cast<double>(top_margin),
                    static_cast<double>(below));
    }
}

// The side rule the X500 box was sized by (kD1X500Grid's comment): R + 6 dx from the axis and the phantom's 0.0165 m past it. The OpenFrame-5F's
// motor axes, from Arm.step (Interface): along (the kit's forward, grid x) +-0.069534 m, across (grid z) +-0.089000 m.
TEST(AirflowFpvGrid, TheBoxHoldsTheOpenFramesHubsWithTheX500sSideRoom) {
    const float dx = kD1FpvGrid.dx;
    const float reach = kD1FpvRotorRadius + 6.0f * dx;
    const float room_x = 0.5f * static_cast<float>(kD1FpvGrid.nx) * dx - (0.069534f + reach);
    const float room_z = 0.5f * static_cast<float>(kD1FpvGrid.nz) * dx - (0.089000f + reach);
    std::printf("[ fpv     ] the OpenFrame-5F's hubs: %.4f m of room past the kernel's reach along, %.4f m across (the rule asks 0.0165 m)\n",
                static_cast<double>(room_x), static_cast<double>(room_z));
    EXPECT_GE(room_x, 0.0165f - 1.0e-4f);
    EXPECT_GE(room_z, 0.0165f - 1.0e-4f);
    EXPECT_LT(room_x, 0.0165f + 2.0f * dx) << "and not a cell more than it needs";
    EXPECT_LT(room_z, 0.0165f + 2.0f * dx);
    const float room_square = 0.5f * static_cast<float>(kD1X500Grid.nx) * dx - (0.089000f + reach);
    EXPECT_NEAR(room_square, 0.00875f, 1.0e-4f) << "the X500's 270 cells across would leave 8.75 mm: the reason the box is not square";
    EXPECT_LT(room_square, 0.0165f);
}

}  // namespace
