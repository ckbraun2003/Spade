// The airflow solver core's grid (physics plan 2026-10-05-airflow-solver-core-plan.md,
// Task 1): a MAC grid, its staggered arrays and the domain faces' configuration.
#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <memory>
#include <new>
#include <string>
#include <vector>

#include "physics/airflow/boundaries.hpp"
#include "physics/airflow/mac_grid.hpp"

namespace {

using spade::Code;
using spade::physics::airflow::DomainBc;
using spade::physics::airflow::FaceBc;
using spade::physics::airflow::GridShape;
using spade::physics::airflow::make_grid;

TEST(AirflowGrid, SizesItsArraysByTheStaggeredLayout) {
    const auto g = make_grid(GridShape{4, 3, 2, 0.1f}, DomainBc{});
    ASSERT_TRUE(g.has_value()) << g.error().context;
    EXPECT_EQ(g->u.size(), 5u * 3u * 2u) << "x-faces: (nx + 1) ny nz";
    EXPECT_EQ(g->v.size(), 4u * 4u * 2u) << "y-faces: nx (ny + 1) nz";
    EXPECT_EQ(g->w.size(), 4u * 3u * 3u) << "z-faces: nx ny (nz + 1)";
    EXPECT_EQ(g->p.size(), 24u);
    for (float x : g->u) EXPECT_EQ(x, 0.0f);
    for (float x : g->p) EXPECT_EQ(x, 0.0f);
}

TEST(AirflowGrid, IndexesEveryArrayWithIInnermost) {
    const GridShape s{4, 3, 2, 0.1f};
    using namespace spade::physics::airflow;
    EXPECT_EQ(u_index(s, 4, 2, 1), 4u + 5u * (2u + 3u * 1u));
    EXPECT_EQ(v_index(s, 3, 3, 1), 3u + 4u * (3u + 4u * 1u));
    EXPECT_EQ(w_index(s, 3, 2, 2), 3u + 4u * (2u + 3u * 2u));
    EXPECT_EQ(cell_index(s, 3, 2, 1), 3u + 4u * (2u + 3u * 1u));
}

TEST(AirflowGrid, VisitsEachComponentsSamplesAtTheirPositions) {
    const GridShape s{2, 1, 1, 0.5f};
    std::vector<float> xs;
    spade::physics::airflow::for_each_sample(s, 0, [&](std::size_t idx, glm::vec3 x) {
        EXPECT_EQ(idx, xs.size());
        xs.push_back(x.x);
        EXPECT_EQ(x.y, 0.25f) << "a u-face sits mid-cell in y";
        EXPECT_EQ(x.z, 0.25f);
    });
    EXPECT_EQ(xs, (std::vector<float>{0.0f, 0.5f, 1.0f})) << "u-faces at i dx, i = 0..nx";
}

TEST(AirflowGrid, RefusesAShapeItCannotStep) {
    const auto refused = [](GridShape s, DomainBc bc, const char* what) {
        const auto g = make_grid(s, bc);
        if (g.has_value()) return testing::AssertionFailure() << "accepted";
        if (g.error().code != Code::invalid_argument) return testing::AssertionFailure() << "code";
        if (g.error().context.find(what) == std::string::npos) {
            return testing::AssertionFailure() << g.error().context;
        }
        return testing::AssertionSuccess();
    };
    EXPECT_TRUE(refused({0, 4, 4, 0.1f}, {}, "dimension"));
    EXPECT_TRUE(refused({4, 4, 4, 0.0f}, {}, "dx"));
    EXPECT_TRUE(refused({4, 4, 4, std::numeric_limits<float>::quiet_NaN()}, {}, "dx"));
    DomainBc unpaired;
    unpaired.face[0] = FaceBc::periodic;
    EXPECT_TRUE(refused({4, 4, 4, 0.1f}, unpaired, "x+"));
    DomainBc odd;
    odd.face[0] = odd.face[1] = FaceBc::periodic;
    EXPECT_TRUE(refused({3, 4, 4, 0.1f}, odd, "even"));
    DomainBc slab;
    slab.face[4] = slab.face[5] = FaceBc::periodic;
    EXPECT_TRUE(make_grid({4, 4, 1, 0.1f}, slab).has_value()) << "a periodic axis of size 1 is a quasi-2D slab";
}

// The lead's fold-in (2026-10-05): the cell cap alone let a 1024^3 grid
// (about 16 GiB of faces and cells) through to std::vector::assign, which
// throws bad_alloc out of a function that promises a Result (L6). The bytes
// are counted in 64 bits and refused past the stated host cap (16 GiB, O193).
TEST(AirflowGrid, CountsItsBytesInSixtyFourBits) {
    using spade::physics::airflow::grid_bytes;
    const uint64_t n = 1024;
    const uint64_t faces = 3u * (n + 1u) * n * n;
    EXPECT_EQ(grid_bytes({1024, 1024, 1024, 0.01f}), 4u * (faces + n * n * n));
    EXPECT_EQ(grid_bytes({4, 3, 2, 0.1f}), 4u * (30u + 32u + 36u + 24u));
}

TEST(AirflowGrid, RefusesAGridPastTheByteCapBeforeAllocating) {
    const auto g = make_grid({1100, 1100, 1100, 0.01f}, {});
    ASSERT_FALSE(g.has_value()) << "accepted a grid of about 21 GB (past the 16 GiB host cap)";
    EXPECT_EQ(g.error().code, Code::capacity_exceeded);
    const std::string& why = g.error().context;
    EXPECT_NE(why.find(std::to_string(spade::physics::airflow::grid_bytes({1100, 1100, 1100, 0.01f}))),
              std::string::npos)
        << why;
    EXPECT_NE(why.find(std::to_string(spade::physics::airflow::kGridByteCap)), std::string::npos) << why;
}

using spade::physics::airflow::FaceKind;
using spade::physics::airflow::MacGrid;

[[nodiscard]] MacGrid filled(spade::physics::airflow::GridShape s, const DomainBc& bc, float value) {
    MacGrid g = *make_grid(s, bc);
    for (float& x : g.u) x = value;
    for (float& x : g.v) x = value;
    for (float& x : g.w) x = value;
    return g;
}

TEST(AirflowBoundaries, AWallSetsItsNormalVelocity) {
    DomainBc bc;
    bc.face.fill(FaceBc::wall);
    bc.wall_velocity[spade::physics::airflow::kYPlus] = glm::vec3(1.0f, 0.0f, 0.0f);  // a lid: tangential only
    bc.wall_velocity[spade::physics::airflow::kXMinus] = glm::vec3(0.25f, 0.0f, 0.0f);  // a piston
    MacGrid g = filled({4, 4, 4, 0.1f}, bc, 7.0f);
    spade::physics::airflow::apply_velocity_boundaries(g, bc);
    const auto& s = g.shape;
    using namespace spade::physics::airflow;
    for (uint32_t k = 0; k < 4; ++k) {
        for (uint32_t i = 0; i < 4; ++i) {
            EXPECT_EQ(g.v[v_index(s, i, 4, k)], 0.0f) << "the lid moves along x, so its normal velocity is 0";
            EXPECT_EQ(g.v[v_index(s, i, 0, k)], 0.0f);
            EXPECT_EQ(g.v[v_index(s, i, 2, k)], 7.0f) << "an interior face is untouched";
        }
        for (uint32_t j = 0; j < 4; ++j) EXPECT_EQ(g.u[u_index(s, 0, j, k)], 0.25f) << "the piston's normal velocity";
    }
}

TEST(AirflowBoundaries, TheAmbientEntersThroughAnOpenFaceAndLeavesThroughAnOutlet) {
    DomainBc bc;
    bc.ambient = glm::vec3(2.0f, 0.0f, 0.0f);
    EXPECT_EQ(face_kind(bc, spade::physics::airflow::kXMinus), FaceKind::fixed_velocity) << "inflow";
    EXPECT_EQ(face_kind(bc, spade::physics::airflow::kXPlus), FaceKind::pressure_outlet);
    EXPECT_EQ(face_kind(bc, spade::physics::airflow::kYMinus), FaceKind::pressure_outlet) << "still across it";
    MacGrid g = filled({4, 2, 2, 0.1f}, bc, 3.0f);
    using namespace spade::physics::airflow;
    g.u[u_index(g.shape, 3, 1, 1)] = 5.0f;
    apply_velocity_boundaries(g, bc);
    EXPECT_EQ(g.u[u_index(g.shape, 0, 1, 1)], 2.0f) << "the inflow takes the ambient";
    EXPECT_EQ(g.u[u_index(g.shape, 4, 1, 1)], 3.0f) << "the outlet's face is left as it was (the projection sets it, L90)";
}

// L90 (the design table's outflow row, plans/2026-10-05-airflow-design.md section 2, amended). Until then an outlet copied its neighbour and refused
// backflow ("zero normal gradient, with backflow clamped to the ambient"; no instability was ever recorded for it). That wiped the air a rotor draws
// in through the open faces once a step, so the box kept a smaller share of an impulse the smaller h was (AirflowMomentumAccount); copying without the
// refusal let the backflow grow. An outlet's faces are now left as the projection sets them (p = 0 on the face), whatever their direction.
TEST(AirflowBoundaries, AnOutletsFacesAreLeftToTheProjection) {
    DomainBc bc;  // still air: every open face is an outlet
    MacGrid g = filled({4, 2, 2, 0.1f}, bc, 0.0f);
    using namespace spade::physics::airflow;
    g.u[u_index(g.shape, 1, 0, 0)] = 1.5f;   // flowing into the box through x-
    g.u[u_index(g.shape, 3, 0, 0)] = -1.5f;  // flowing into the box through x+
    g.u[u_index(g.shape, 0, 0, 0)] = 0.25f;
    g.u[u_index(g.shape, 4, 0, 0)] = -0.25f;
    apply_velocity_boundaries(g, bc);
    EXPECT_EQ(g.u[u_index(g.shape, 0, 0, 0)], 0.25f) << "neither copied nor refused";
    EXPECT_EQ(g.u[u_index(g.shape, 4, 0, 0)], -0.25f);
}

TEST(AirflowBoundaries, APeriodicAxisCopiesItsFirstFaceToItsLast) {
    DomainBc bc;
    bc.face.fill(FaceBc::periodic);
    MacGrid g = filled({4, 2, 2, 0.1f}, bc, 0.0f);
    using namespace spade::physics::airflow;
    g.u[u_index(g.shape, 0, 1, 1)] = 4.0f;
    g.u[u_index(g.shape, 4, 1, 1)] = -9.0f;
    apply_velocity_boundaries(g, bc);
    EXPECT_EQ(g.u[u_index(g.shape, 4, 1, 1)], 4.0f);
}

// Review Focus 5. With nx = 1 the x- outlet reads face 1 and the x+ outlet
// reads face nx - 1 = 0: both in range. Still air, so both are outlets.
TEST(AirflowBoundaries, AOneCellWideAxisReadsNothingOutOfRange) {
    DomainBc bc;
    MacGrid g = filled({1, 3, 1, 0.1f}, bc, 0.0f);
    using namespace spade::physics::airflow;
    ASSERT_EQ(g.u.size(), 2u * 3u * 1u);
    g.u[u_index(g.shape, 1, 2, 0)] = 0.5f;
    apply_velocity_boundaries(g, bc);
    // Both outlets' faces are left alone (L90): face 0 and face 1 keep what they held.
    EXPECT_EQ(g.u[u_index(g.shape, 0, 2, 0)], 0.0f);
    EXPECT_EQ(g.u[u_index(g.shape, 1, 2, 0)], 0.5f);
}

}  // namespace

// O193 (Oversight's second condition on the 16 GiB policy number): a CPU run that passes validation but cannot get its memory fails BY NAME, the grid and the
// bytes, and never throws out of a function that promises a Result (L6). The failure is FORCED by an injectable allocator that refuses past a limit, so the
// test never relies on the machine running out of memory.
namespace {

template <class T>
struct LimitedAlloc {
    using value_type = T;
    std::size_t limit_bytes = 0;
    LimitedAlloc() = default;
    explicit LimitedAlloc(std::size_t limit) : limit_bytes(limit) {}
    template <class U>
    LimitedAlloc(const LimitedAlloc<U>& other) noexcept : limit_bytes(other.limit_bytes) {}
    T* allocate(std::size_t n) {
        if (n * sizeof(T) > limit_bytes) throw std::bad_alloc();
        return std::allocator<T>{}.allocate(n);
    }
    void deallocate(T* p, std::size_t n) noexcept { std::allocator<T>{}.deallocate(p, n); }
    template <class U>
    bool operator==(const LimitedAlloc<U>& other) const noexcept {
        return limit_bytes == other.limit_bytes;
    }
};

}  // namespace

TEST(AirflowGrid, AnAllocationThatFailsIsRefusedByNameAndNeverThrown) {
    std::vector<float, LimitedAlloc<float>> v{LimitedAlloc<float>(1024)};
    spade::Result<void> r;
    EXPECT_NO_THROW(r = spade::physics::airflow::assign_zeroed(v, 1000, "airflow grid 10 x 10 x 10, array 'u'"));
    ASSERT_FALSE(r.has_value()) << "1000 floats is 4000 bytes, past the allocator's 1024";
    EXPECT_EQ(r.error().code, spade::Code::capacity_exceeded);
    const std::string& why = r.error().context;
    EXPECT_NE(why.find("out of memory"), std::string::npos) << why;
    EXPECT_NE(why.find("airflow grid 10 x 10 x 10, array 'u'"), std::string::npos) << "the grid is named: " << why;
    EXPECT_NE(why.find("4000"), std::string::npos) << "the bytes are named: " << why;
    EXPECT_TRUE(v.empty()) << "a refused allocation leaves the array empty and usable";
}

TEST(AirflowGrid, AnAllocationThatFitsIsFilledWithZero) {
    std::vector<float, LimitedAlloc<float>> v{LimitedAlloc<float>(1024)};
    v.assign(3, 7.0f);
    const spade::Result<void> r = spade::physics::airflow::assign_zeroed(v, 100, "airflow grid 5 x 5 x 4, array 'p'");
    ASSERT_TRUE(r.has_value()) << r.error().context;
    ASSERT_EQ(v.size(), 100u);
    for (const float x : v) ASSERT_EQ(x, 0.0f);
}
