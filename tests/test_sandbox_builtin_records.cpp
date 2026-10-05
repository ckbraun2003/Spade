// The built-in scenes' physics records, pinned to the viewer's.
//
// sandbox/builtin_records.hpp copies, for each scene file in assets/scenes/,
// the per-world physics records world file v2 cannot hold (the seed, the
// turbulence, the contact and grid parameters), so the sandbox can run the
// files as the viewer runs its scenes without linking the viewer (SL2b). This
// test is what keeps the copy honest: every entry must equal the record
// spade_viewer's own make_scene() builds, field by field, and every scene file
// must have one. Both go when world file v3 carries the records.

#include <gtest/gtest.h>

#include <cstdint>
#include <cstring>
#include <filesystem>
#include <optional>
#include <string>
#include <vector>

#include "../sandbox/builtin_records.hpp"
#include "bridge.hpp"
#include "setup.hpp"

namespace {

namespace fs = std::filesystem;
using spade::sandbox::builtin_records;
using spade::sandbox::BuiltinRecords;

struct ViewerScene {
    const char* name;
    uint32_t lanes;
};
// Every viewer scene, with its world count (test_viewer_successors.cpp's list).
const std::vector<ViewerScene> kViewerScenes = {{"drop", 1},  {"bounce", 4}, {"shower", 1}, {"gate", 1},
                                                {"hover", 1}, {"wind", 1},   {"flight", 1}, {"swarm", 4}};

[[nodiscard]] std::string stem_of(const ViewerScene& s, uint32_t lane) {
    return s.lanes == 1u ? std::string(s.name) : std::string(s.name) + "_lane_" + std::to_string(lane);
}

}  // namespace

TEST(SandboxBuiltinRecords, EachEntryIsTheViewersRecordFieldByField) {
    for (const ViewerScene& s : kViewerScenes) {
        const std::optional<spade::viewer::Scene> scene = spade::viewer::make_scene(s.name);
        ASSERT_TRUE(scene.has_value()) << s.name;
        ASSERT_EQ(scene->worlds.worlds.size(), s.lanes) << s.name;
        for (uint32_t lane = 0; lane < s.lanes; ++lane) {
            const std::string stem = stem_of(s, lane);
            const std::optional<BuiltinRecords> mine = builtin_records(stem);
            ASSERT_TRUE(mine.has_value()) << "no records for " << stem;
            const spade::WorldInstanceDesc& theirs = scene->worlds.worlds[lane];
            EXPECT_EQ(mine->seed, theirs.seed) << stem;
            EXPECT_EQ(mine->turbulence, theirs.turbulence) << stem;
            // Whole records, every field and reserved word: both are std430
            // rows of floats with no padding (contacts.hpp and grid.hpp assert
            // their sizes), so the bytes are the fields.
            EXPECT_EQ(std::memcmp(&mine->contacts, &theirs.contacts, sizeof theirs.contacts), 0)
                << stem << ": contact parameters differ (restitution " << mine->contacts.restitution_e << " vs "
                << theirs.contacts.restitution_e << ", radius " << mine->contacts.proxy_radius << " vs "
                << theirs.contacts.proxy_radius << ")";
            EXPECT_EQ(std::memcmp(&mine->grid, &theirs.grid, sizeof theirs.grid), 0)
                << stem << ": grid parameters differ (cell " << mine->grid.cell_size << " vs "
                << theirs.grid.cell_size << ")";
        }
    }
}

// The step the viewer runs its scenes at, which the scene files assume.
TEST(SandboxBuiltinRecords, TheStepIsTheViewers) {
    EXPECT_EQ(spade::sandbox::kBuiltinStepNs, spade::viewer::kStepDtNs);
    EXPECT_EQ(spade::sandbox::kBuiltinSubsteps, spade::viewer::kSubsteps);
}

// Every scene file in assets/scenes/ has records, so none runs with defaults
// by accident.
TEST(SandboxBuiltinRecords, EverySceneFileHasRecords) {
    uint32_t files = 0;
    for (const fs::directory_entry& e : fs::directory_iterator(fs::path(SPADE_ASSETS_DIR) / "scenes")) {
        const std::string name = e.path().filename().string();
        constexpr std::string_view kSuffix = ".scene.yaml";
        if (name.size() <= kSuffix.size() || name.substr(name.size() - kSuffix.size()) != kSuffix) continue;
        ++files;
        const std::string stem = name.substr(0, name.size() - kSuffix.size());
        EXPECT_TRUE(builtin_records(stem).has_value()) << "no records for " << name;
    }
    EXPECT_EQ(files, 14u) << "assets/scenes/ holds another set of scenes: update this test and the records";
}

TEST(SandboxBuiltinRecords, AnUnknownSceneHasNone) {
    EXPECT_FALSE(builtin_records("").has_value());
    EXPECT_FALSE(builtin_records("bounce").has_value());  // a many-world scene is named by its lanes
    EXPECT_FALSE(builtin_records("bounce_lane_4").has_value());
    EXPECT_FALSE(builtin_records("my_scene").has_value());
}
