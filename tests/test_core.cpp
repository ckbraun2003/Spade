#include <gtest/gtest.h>

#include <iterator>
#include <string>
#include <type_traits>

#include "core/error.hpp"
#include "core/ids.hpp"
#include "core/time.hpp"

namespace {

// Phantom tag types -- never instantiated, only used to parameterize Handle.
struct BodyTag {};
struct WorldTag {};

}  // namespace

// ---------------------------------------------------------------------------
// Handle<Tag>: null, equality, generation-bump invalidation.
// ---------------------------------------------------------------------------

TEST(Handle, DefaultConstructedIsNull) {
    spade::Handle<BodyTag> h{};
    EXPECT_TRUE(h.is_null());
    EXPECT_EQ(h.index, 0u);
    EXPECT_EQ(h.generation, 0u);
}

TEST(Handle, KNullIsNull) {
    EXPECT_TRUE(spade::kNull<BodyTag>.is_null());
    EXPECT_EQ(spade::Handle<BodyTag>{}, spade::kNull<BodyTag>);
}

TEST(Handle, NonZeroGenerationIsNotNull) {
    spade::Handle<BodyTag> h{.index = 3, .generation = 1};
    EXPECT_FALSE(h.is_null());
}

TEST(Handle, EqualityComparesBothFields) {
    spade::Handle<BodyTag> a{.index = 5, .generation = 1};
    spade::Handle<BodyTag> b{.index = 5, .generation = 1};
    spade::Handle<BodyTag> c{.index = 5, .generation = 2};
    spade::Handle<BodyTag> d{.index = 6, .generation = 1};

    EXPECT_EQ(a, b);
    EXPECT_NE(a, c);
    EXPECT_NE(a, d);
}

TEST(Handle, GenerationBumpInvalidatesStaleHandle) {
    // Simulates what a pool does on despawn/respawn of the same slot: the
    // index is reused, the generation is bumped. A handle captured before
    // the bump must no longer equal the handle a caller would be issued
    // after it -- that inequality IS the "is this handle stale" check,
    // since there is no registry at this layer to ask.
    spade::Handle<BodyTag> issued_first{.index = 2, .generation = 1};
    spade::Handle<BodyTag> issued_after_respawn{.index = 2, .generation = 2};

    EXPECT_NE(issued_first, issued_after_respawn);
    EXPECT_FALSE(issued_first.is_null());
    EXPECT_FALSE(issued_after_respawn.is_null());
}

TEST(Handle, DifferentTagsAreDistinctTypes) {
    // Compile-time check: a BodyTag handle and a WorldTag handle are
    // unrelated types, so mixing them up is a build error, not a runtime
    // bug. Nothing to assert at runtime; this documents the property so a
    // change to Handle can't silently drop it.
    static_assert(!std::is_same_v<spade::Handle<BodyTag>, spade::Handle<WorldTag>>);
    SUCCEED();
}

// ---------------------------------------------------------------------------
// Error / Result round-trip.
// ---------------------------------------------------------------------------

TEST(Result, SuccessRoundTrip) {
    spade::Result<int> r = 42;
    ASSERT_TRUE(r.has_value());
    EXPECT_EQ(*r, 42);
}

TEST(Result, FailureCarriesErrorCodeAndContext) {
    spade::Result<int> r = std::unexpected(spade::Error{spade::Code::not_found, "world.yaml"});

    ASSERT_FALSE(r.has_value());
    EXPECT_EQ(r.error().code, spade::Code::not_found);
    EXPECT_EQ(r.error().context, "world.yaml");
}

TEST(Result, MoveOnlyPayloadRoundTrips) {
    spade::Result<std::string> r = std::string("payload");
    ASSERT_TRUE(r.has_value());
    EXPECT_EQ(*r, "payload");
}

TEST(Error, EqualityComparesCodeAndContext) {
    spade::Error a{spade::Code::invalid_argument, "x"};
    spade::Error b{spade::Code::invalid_argument, "x"};
    spade::Error c{spade::Code::invalid_argument, "y"};
    spade::Error d{spade::Code::internal, "x"};

    EXPECT_EQ(a, b);
    EXPECT_NE(a, c);
    EXPECT_NE(a, d);
}

TEST(Error, AllCodesAreDistinct) {
    // Enumerates the full closed set from the brief so an accidental
    // duplicate value (e.g. a copy-paste of an existing enumerator) fails
    // loudly instead of silently aliasing two error kinds.
    constexpr spade::Code kCodes[] = {
        spade::Code::invalid_argument, spade::Code::capacity_exceeded, spade::Code::not_found,
        spade::Code::io_error,         spade::Code::schema_mismatch,   spade::Code::internal,
    };
    for (size_t i = 0; i < std::size(kCodes); ++i) {
        for (size_t j = i + 1; j < std::size(kCodes); ++j) {
            EXPECT_NE(kCodes[i], kCodes[j]) << "codes at index " << i << " and " << j << " alias";
        }
    }
}

// ---------------------------------------------------------------------------
// Tick arithmetic.
// ---------------------------------------------------------------------------

TEST(Tick, DefaultIsZero) {
    spade::Tick t{};
    EXPECT_EQ(t.value, 0u);
}

TEST(Tick, PreAndPostIncrement) {
    spade::Tick t{.value = 5};

    spade::Tick after_pre = ++t;
    EXPECT_EQ(t.value, 6u);
    EXPECT_EQ(after_pre.value, 6u);

    spade::Tick before_post = t++;
    EXPECT_EQ(t.value, 7u);
    EXPECT_EQ(before_post.value, 6u);
}

TEST(Tick, AddAndSubtractSteps) {
    spade::Tick t{.value = 10};

    EXPECT_EQ((t + 5).value, 15u);
    EXPECT_EQ((t - 3).value, 7u);
}

TEST(Tick, DifferenceBetweenTicksIsStepCount) {
    spade::Tick a{.value = 100};
    spade::Tick b{.value = 40};

    EXPECT_EQ(a - b, 60u);
}

TEST(Tick, OrderingIsByValue) {
    spade::Tick a{.value = 1};
    spade::Tick b{.value = 2};

    EXPECT_LT(a, b);
    EXPECT_LE(a, a);
    EXPECT_GT(b, a);
    EXPECT_GE(b, b);
    EXPECT_EQ(a, spade::Tick{.value = 1});
}
