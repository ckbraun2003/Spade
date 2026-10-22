// The default library's propeller tables and blade definitions (Task L4a of
// docs/design/core/plans/2026-10-21-default-drone-library-plan.md):
// tests/library_props.gen.hpp, which tools/library_props/resample_uiuc.py
// writes from the hand-downloaded UIUC files (ruling 3: never committed).
//
// What is pinned here, on both platforms and with no Python:
//   - THE HEADER IS THE TOOL'S OWN: the FNV-1a 64 the header stamps for the
//     tool and its props table, CRLF read as LF, is what those files hash to
//     in the source tree now. Editing either without re-running the tool
//     fails here, naming the file (the lead, 2026-10-22). The tool's own
//     tests are tools/library_props/test_resample_uiuc.py, run by hand.
//   - EVERY TABLE STARTS AT ITS STATIC POINT: J = 0 holds ct_static and
//     cq_static (above 0), then J = k/20 for k = 1, 2, ..., every value
//     finite; a static-only prop has no tables.
//   - NO TABLE EXTENDS PAST ITS RUN: the last J is at or below the run's last
//     measured J, and within one step of it.
//   - THE BLADES: strictly increasing stations in (0, 1] from the hub
//     cut-out, every chord above 0 (the part file's blade rules).

#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <string>

#include "library_props.gen.hpp"

namespace {

namespace fs = std::filesystem;
using spade::test::library_props::kProps;
using spade::test::library_props::Prop;

[[nodiscard]] fs::path tool_dir() {
    return fs::path(SPADE_TESTS_DIR) / ".." / "tools" / "library_props";
}

// FNV-1a 64 of the file's bytes, CRLF read as LF: the tool's hash_file().
[[nodiscard]] bool fnv1a64_of(const fs::path& path, std::uint64_t& out) {
    std::ifstream in(path, std::ios::binary);
    if (!in) return false;
    const std::string body((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
    std::uint64_t h = 0xcbf29ce484222325ull;
    for (std::size_t i = 0; i < body.size(); ++i) {
        if (body[i] == '\r' && i + 1 < body.size() && body[i + 1] == '\n') continue;
        h ^= static_cast<unsigned char>(body[i]);
        h *= 0x100000001b3ull;
    }
    out = h;
    return true;
}

}  // namespace

TEST(LibraryProps, TheHeaderIsTheToolsOwn) {
    struct Stamp {
        const char* file;
        std::uint64_t stamped;
    };
    for (const Stamp s : {Stamp{"resample_uiuc.py", spade::test::library_props::kToolHash},
                          Stamp{"props.json", spade::test::library_props::kPropsHash}}) {
        std::uint64_t now = 0;
        ASSERT_TRUE(fnv1a64_of(tool_dir() / s.file, now)) << "cannot read " << (tool_dir() / s.file).string();
        EXPECT_EQ(now, s.stamped) << "tools/library_props/" << s.file
                                  << " changed since tests/library_props.gen.hpp was generated: re-run "
                                     "resample_uiuc.py (and its tests) and commit the header with it";
    }
}

TEST(LibraryProps, EveryPropTableStartsAtItsStaticPoint) {
    ASSERT_EQ(kProps.size(), 5u);
    for (const Prop& p : kProps) {
        SCOPED_TRACE(std::string(p.id));
        EXPECT_GT(p.ct_static, 0.0);
        EXPECT_GT(p.cq_static, 0.0);
        EXPECT_EQ(p.ct_table.size(), p.cq_table.size());
        if (p.ct_table.empty()) {
            // Static only: no run, so nothing measured past J = 0.
            EXPECT_EQ(p.run_rpm, 0.0);
            continue;
        }
        EXPECT_EQ(p.ct_table[0][0], 0.0);
        EXPECT_EQ(p.ct_table[0][1], p.ct_static);
        EXPECT_EQ(p.cq_table[0][0], 0.0);
        EXPECT_EQ(p.cq_table[0][1], p.cq_static);
        // Past its zero-thrust J a prop windmills, so a measured C_T may be
        // negative there (mit_5x4's is, from J 0.80); the chain takes any
        // finite value (propeller.cpp's check_points).
        for (std::size_t k = 0; k < p.ct_table.size(); ++k) {
            EXPECT_EQ(p.ct_table[k][0], static_cast<double>(k) / 20) << k;
            EXPECT_EQ(p.cq_table[k][0], p.ct_table[k][0]) << k;
            EXPECT_TRUE(std::isfinite(p.ct_table[k][1])) << k;
            EXPECT_TRUE(std::isfinite(p.cq_table[k][1])) << k;
        }
    }
}

TEST(LibraryProps, NoTableExtendsPastItsRun) {
    for (const Prop& p : kProps) {
        SCOPED_TRACE(std::string(p.id));
        if (p.ct_table.empty()) continue;
        const double last = p.ct_table.back()[0];
        EXPECT_LE(last, p.last_measured_j);
        EXPECT_LT(p.last_measured_j - last, 0.05);
        EXPECT_GT(p.first_measured_j, 0.0);
    }
}

TEST(LibraryProps, EachBladeRunsFromItsHubInsideTheDisc) {
    for (const Prop& p : kProps) {
        SCOPED_TRACE(std::string(p.id));
        ASSERT_GE(p.r_over_r.size(), 2u);
        EXPECT_EQ(p.r_over_r.size(), p.c_over_r.size());
        EXPECT_EQ(p.r_over_r.size(), p.beta_deg.size());
        EXPECT_EQ(p.r_over_r[0], p.hub_cutout);
        EXPECT_GE(p.hub_cutout, 0.0);
        EXPECT_LE(p.r_over_r.back(), 1.0);
        for (std::size_t i = 0; i < p.r_over_r.size(); ++i) {
            EXPECT_GT(p.c_over_r[i], 0.0) << i;
            if (i > 0) {
                EXPECT_GT(p.r_over_r[i], p.r_over_r[i - 1]) << i;
            }
        }
        EXPECT_FALSE(p.inputs.empty());
    }
}
