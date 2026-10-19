// The grid's state as bytes and as a file (physics/airflow/grid_io.hpp): the
// one serializer LiveRun's save and flow_movie's cache call (TD-9). u, v, w and
// p are everything a later fluid step reads that is not rebuilt, so a run
// restored from them continues bit for bit.
#include <gtest/gtest.h>

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

#include "physics/airflow/fluid_step.hpp"
#include "physics/airflow/grid_io.hpp"
#include "physics/airflow/mac_grid.hpp"
#include "unique_temp.hpp"

namespace {

using namespace spade::physics::airflow;
using spade::Result;

struct Lcg {
    uint32_t state = 12345u;
    float next() noexcept {
        state = state * 1664525u + 1013904223u;
        return static_cast<float>(state >> 8) * (2.0f / 16777216.0f) - 1.0f;
    }
};

[[nodiscard]] bool same_bits(const std::vector<float>& a, const std::vector<float>& b) {
    return a.size() == b.size() && std::memcmp(a.data(), b.data(), a.size() * sizeof(float)) == 0;
}

[[nodiscard]] bool same_grid(const MacGrid& a, const MacGrid& b) {
    return same_bits(a.u, b.u) && same_bits(a.v, b.v) && same_bits(a.w, b.w) && same_bits(a.p, b.p);
}

[[nodiscard]] MacGrid filled(const GridShape& shape, uint32_t seed) {
    MacGrid g = *make_grid(shape, DomainBc{});
    Lcg rng{seed};
    for (std::vector<float>* v : {&g.u, &g.v, &g.w, &g.p}) {
        for (float& x : *v) x = rng.next();
    }
    return g;
}

TEST(AirflowGridIo, ABlobRestoresTheGridBitForBit) {
    const GridShape shape{6, 5, 4, 0.1f};
    const MacGrid a = filled(shape, 7u);
    const std::vector<std::byte> blob = grid_to_bytes(a);
    EXPECT_EQ(blob.size(), kGridBlobHeaderBytes + 4u * (a.u.size() + a.v.size() + a.w.size() + a.p.size()));
    MacGrid b = *make_grid(shape, DomainBc{});
    const Result<void> ok = grid_from_bytes(blob, b);
    ASSERT_TRUE(ok.has_value()) << ok.error().context;
    EXPECT_TRUE(same_grid(a, b));
}

// A lid-driven box with viscosity, stepped 6 times, saved, and stepped 6 more;
// a fresh grid and a fresh scratch restored from the save and stepped 6 times
// match it bit for bit: FluidScratch holds nothing a later step reads.
// Mutation: zero p on load (the projection's warm start), and they differ.
TEST(AirflowGridIo, ARestoredRunContinuesBitForBit) {
    const GridShape shape{12, 10, 8, 0.1f};
    DomainBc bc;
    bc.face.fill(FaceBc::wall);
    bc.wall_velocity[kYPlus] = glm::vec3(1.0f, 0.0f, 0.0f);
    const FluidConfig cfg{.rho = 1.225f, .nu = 0.01f, .h = 0.01f, .n_v = 2};
    MacGrid a = *make_grid(shape, bc);
    FluidScratch sa = *make_fluid_scratch(a, bc, cfg);
    for (int k = 0; k < 6; ++k) ASSERT_EQ(fluid_step(a, bc, cfg, sa).fault, FluidFault::none);
    const std::vector<std::byte> saved = grid_to_bytes(a);
    for (int k = 0; k < 6; ++k) ASSERT_EQ(fluid_step(a, bc, cfg, sa).fault, FluidFault::none);

    MacGrid b = *make_grid(shape, bc);
    ASSERT_TRUE(grid_from_bytes(saved, b).has_value());
    FluidScratch sb = *make_fluid_scratch(b, bc, cfg);
    for (int k = 0; k < 6; ++k) ASSERT_EQ(fluid_step(b, bc, cfg, sb).fault, FluidFault::none);
    EXPECT_TRUE(same_grid(a, b));
}

// Each refusal names what differs, and leaves the grid it would restore into
// untouched.
TEST(AirflowGridIo, ARefusalNamesWhatDiffersAndWritesNothing) {
    const GridShape shape{6, 5, 4, 0.1f};
    const std::vector<std::byte> blob = grid_to_bytes(filled(shape, 7u));
    const MacGrid before = filled(shape, 99u);
    const auto refused = [&](std::vector<std::byte> bytes, const GridShape& into_shape, const char* says) {
        MacGrid into = into_shape.nx == shape.nx && into_shape.ny == shape.ny && into_shape.nz == shape.nz &&
                               into_shape.dx == shape.dx
                           ? before
                           : *make_grid(into_shape, DomainBc{});
        const MacGrid untouched = into;
        const Result<void> r = grid_from_bytes(bytes, into);
        ASSERT_FALSE(r.has_value()) << says;
        EXPECT_NE(r.error().context.find(says), std::string::npos) << r.error().context;
        EXPECT_TRUE(same_grid(into, untouched)) << says;
    };
    refused(std::vector<std::byte>(blob.begin(), blob.begin() + 10), shape, "shorter than its");
    std::vector<std::byte> foreign = blob;
    foreign[0] = std::byte{'X'};
    refused(foreign, shape, "not an airflow grid");
    std::vector<std::byte> later = blob;
    later[8] = std::byte{2};  // the version, little-endian, after the 8-byte magic
    refused(later, shape, "version 2");
    refused(blob, GridShape{6, 5, 5, 0.1f}, "6x5x4");
    refused(blob, GridShape{6, 5, 4, 0.2f}, "dx");
    refused(std::vector<std::byte>(blob.begin(), blob.end() - 4), shape, "bytes");
}

TEST(AirflowGridIo, AFileSavesAndLoadsWithItsStep) {
    const spade::test::TempDir temp("airflow_grid_io");
    const std::filesystem::path& dir = temp.path();
    const std::filesystem::path path = dir / "state.grid";
    const GridShape shape{6, 5, 4, 0.1f};
    const MacGrid a = filled(shape, 7u);
    ASSERT_TRUE(save_grid(path, a, 1234u).has_value());
    EXPECT_FALSE(std::filesystem::exists(dir / "state.grid.tmp")) << "the temporary is renamed, not left";
    MacGrid b = *make_grid(shape, DomainBc{});
    const Result<uint64_t> step = load_grid(path, b);
    ASSERT_TRUE(step.has_value()) << step.error().context;
    EXPECT_EQ(*step, 1234u);
    EXPECT_TRUE(same_grid(a, b));

    ASSERT_TRUE(save_grid(path, filled(shape, 8u), 1235u).has_value()) << "a save replaces the last";
    EXPECT_EQ(*load_grid(path, b), 1235u);

    const Result<uint64_t> missing = load_grid(dir / "nothing.grid", b);
    ASSERT_FALSE(missing.has_value());
    EXPECT_NE(missing.error().context.find("nothing.grid"), std::string::npos) << missing.error().context;
    {
        std::ofstream junk(dir / "junk.grid", std::ios::binary);
        junk << "not a grid file at all";
    }
    const Result<uint64_t> junk = load_grid(dir / "junk.grid", b);
    ASSERT_FALSE(junk.has_value());
    EXPECT_NE(junk.error().context.find("junk.grid"), std::string::npos) << junk.error().context;
}

}  // namespace
