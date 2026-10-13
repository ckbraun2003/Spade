// render/transfer_function.hpp: the colour maps against their sources and
// their perceptual properties, extinction, validation, and the range rules
// (fixed by default, auto as an announced opt-in, diverging maps symmetric).
// Test-only double and std::pow; this file feeds no golden.

#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "core/flow_volume.hpp"
#include "render/transfer_function.hpp"

namespace {

using spade::render::apply_transfer;
using spade::render::ColourMapId;
using spade::render::colour_map_at;
using spade::render::colour_map_table;
using spade::render::FlowQuantity;
using spade::render::prepare_transfer;
using spade::render::resolve_range;
using spade::render::TransferFunction;
using spade::render::validate_transfer_function;

// Relative luminance (Rec. 709) of a display colour: a check of a map's
// lightness ordering, not a value the renderer uses.
[[nodiscard]] double luminance(const glm::vec3& c) {
    const auto lin = [](double v) { return v <= 0.04045 ? v / 12.92 : std::pow((v + 0.055) / 1.055, 2.4); };
    return 0.2126 * lin(c.x) + 0.7152 * lin(c.y) + 0.0722 * lin(c.z);
}

void expect_entry(ColourMapId id, size_t i, float r, float g, float b) {
    const glm::vec3 c = colour_map_table(id)[i];
    EXPECT_FLOAT_EQ(c.x, r) << "entry " << i;
    EXPECT_FLOAT_EQ(c.y, g) << "entry " << i;
    EXPECT_FLOAT_EQ(c.z, b) << "entry " << i;
}

// Spot values read from matplotlib 3.10.8's own tables (colour_maps.cpp's
// provenance), at both ends and the middle.
TEST(RenderTransferFunction, ColourMapsMatchTheirSources) {
    expect_entry(ColourMapId::viridis, 0, 0.267004f, 0.004874f, 0.329415f);
    expect_entry(ColourMapId::viridis, 128, 0.127568f, 0.566949f, 0.550556f);
    expect_entry(ColourMapId::viridis, 255, 0.993248f, 0.906157f, 0.143936f);
    expect_entry(ColourMapId::inferno, 0, 0.001462f, 0.000466f, 0.013866f);
    expect_entry(ColourMapId::inferno, 128, 0.735683f, 0.215906f, 0.330245f);
    expect_entry(ColourMapId::inferno, 255, 0.988362f, 0.998364f, 0.644924f);
    expect_entry(ColourMapId::cool_warm, 0, 0.2298057f, 0.29871797f, 0.75368315f);
    expect_entry(ColourMapId::cool_warm, 128, 0.86742764f, 0.8643766f, 0.86260246f);
    expect_entry(ColourMapId::cool_warm, 255, 0.70567316f, 0.01555616f, 0.15023281f);
}

// What makes these maps fit for measurement: a sequential map's lightness
// rises at every entry, so no band reads as a feature; the diverging map's
// rises to its centre and falls after it, ending where it began.
TEST(RenderTransferFunction, SequentialMapsBrightenAndTheDivergingMapPeaksAtItsCentre) {
    for (const ColourMapId id : {ColourMapId::viridis, ColourMapId::inferno}) {
        const auto& t = colour_map_table(id);
        for (size_t i = 1; i < t.size(); ++i) {
            ASSERT_GT(luminance(t[i]), luminance(t[i - 1])) << "map " << static_cast<int>(id) << ", entry " << i;
        }
    }
    const auto& cw = colour_map_table(ColourMapId::cool_warm);
    for (size_t i = 1; i <= 127; ++i) ASSERT_GT(luminance(cw[i]), luminance(cw[i - 1])) << "entry " << i;
    for (size_t i = 128; i < cw.size(); ++i) ASSERT_LT(luminance(cw[i]), luminance(cw[i - 1])) << "entry " << i;
    EXPECT_NEAR(luminance(cw.front()), luminance(cw.back()), 1e-3);
}

TEST(RenderTransferFunction, AMapInterpolatesBetweenEntriesAndClamps) {
    const auto& t = colour_map_table(ColourMapId::viridis);
    const glm::vec3 a = t[10], b = t[11];
    const float u = (10.0f + 0.25f) / 255.0f;
    const glm::vec3 got = colour_map_at(ColourMapId::viridis, u);
    EXPECT_NEAR(got.x, a.x + (b.x - a.x) * 0.25f, 1e-6f);
    EXPECT_NEAR(got.y, a.y + (b.y - a.y) * 0.25f, 1e-6f);
    EXPECT_NEAR(got.z, a.z + (b.z - a.z) * 0.25f, 1e-6f);
    EXPECT_EQ(colour_map_at(ColourMapId::viridis, -3.0f), t.front());
    EXPECT_EQ(colour_map_at(ColourMapId::viridis, 7.0f), t.back());
    EXPECT_EQ(colour_map_at(ColourMapId::viridis, 1.0f), t.back());
}

[[nodiscard]] spade::FlowVolumeHeader header_with(uint32_t channel, float lo, float hi) {
    spade::FlowVolumeHeader h;
    h.nx = h.ny = h.nz = 1;
    h.cell = 1.0f;
    h.channel_min[channel] = lo;
    h.channel_max[channel] = hi;
    return h;
}

// sigma is piecewise linear in the value, exact at its points, and held
// past the ends; the colour follows the range.
TEST(RenderTransferFunction, ExtinctionIsPiecewiseLinearAndHeldPastItsEnds) {
    TransferFunction tf;
    tf.quantity = FlowQuantity::vorticity;
    tf.colours = ColourMapId::inferno;
    tf.range_min = 0.0f;
    tf.range_max = 100.0f;
    tf.extinction = {{10.0f, 0.0f}, {20.0f, 4.0f}, {60.0f, 2.0f}};
    ASSERT_TRUE(validate_transfer_function(tf).has_value());
    const auto p = prepare_transfer(tf, header_with(spade::kFlowVorticity, -5.0f, 999.0f));
    EXPECT_FLOAT_EQ(apply_transfer(p, 0.0f).sigma, 0.0f);
    EXPECT_FLOAT_EQ(apply_transfer(p, 10.0f).sigma, 0.0f);
    EXPECT_FLOAT_EQ(apply_transfer(p, 15.0f).sigma, 2.0f);
    EXPECT_FLOAT_EQ(apply_transfer(p, 20.0f).sigma, 4.0f);
    EXPECT_FLOAT_EQ(apply_transfer(p, 40.0f).sigma, 3.0f);
    EXPECT_FLOAT_EQ(apply_transfer(p, 60.0f).sigma, 2.0f);
    EXPECT_FLOAT_EQ(apply_transfer(p, 1.0e4f).sigma, 2.0f);
    EXPECT_EQ(apply_transfer(p, 0.0f).colour, colour_map_at(ColourMapId::inferno, 0.0f));
    EXPECT_EQ(apply_transfer(p, 50.0f).colour, colour_map_at(ColourMapId::inferno, 0.5f));
    EXPECT_EQ(apply_transfer(p, 250.0f).colour, colour_map_at(ColourMapId::inferno, 1.0f));
}

TEST(RenderTransferFunction, SmokeUsesItsOneColour) {
    TransferFunction tf;
    tf.quantity = FlowQuantity::dye;
    tf.constant_colour = glm::vec3(0.8f, 0.8f, 0.85f);
    ASSERT_TRUE(validate_transfer_function(tf).has_value());
    const auto p = prepare_transfer(tf, header_with(spade::kFlowDye, 0.0f, 1.0f));
    EXPECT_EQ(apply_transfer(p, 0.1f).colour, glm::vec3(0.8f, 0.8f, 0.85f));
    EXPECT_EQ(apply_transfer(p, 0.9f).colour, glm::vec3(0.8f, 0.8f, 0.85f));
}

// Q5: the fixed range by default, whatever the header says; auto only on
// request, and then it says so.
TEST(RenderTransferFunction, TheRangeIsFixedUnlessAutoIsAskedForAndThenAnnounced) {
    TransferFunction tf;
    tf.quantity = FlowQuantity::vorticity;
    tf.range_min = 0.0f;
    tf.range_max = 50.0f;
    const auto h = header_with(spade::kFlowVorticity, 2.0f, 80.0f);
    const auto fixed = resolve_range(tf, h);
    EXPECT_FALSE(fixed.from_header);
    EXPECT_EQ(fixed.min, 0.0f);
    EXPECT_EQ(fixed.max, 50.0f);

    tf.auto_range = true;
    const auto chosen = resolve_range(tf, h);
    EXPECT_TRUE(chosen.from_header);
    EXPECT_EQ(chosen.min, 2.0f);
    EXPECT_EQ(chosen.max, 80.0f);
}

// A diverging map keeps zero at its neutral centre; speed runs from 0 to the
// largest |(u, v, w)| the per-channel ranges allow.
TEST(RenderTransferFunction, AutoRangesKeepZeroCentredAndBoundSpeed) {
    TransferFunction pressure;
    pressure.quantity = FlowQuantity::pressure;
    pressure.colours = ColourMapId::cool_warm;
    pressure.auto_range = true;
    const auto p = resolve_range(pressure, header_with(spade::kFlowPressure, -3.0f, 12.0f));
    EXPECT_EQ(p.min, -12.0f);
    EXPECT_EQ(p.max, 12.0f);
    const auto prepared = prepare_transfer(pressure, header_with(spade::kFlowPressure, -3.0f, 12.0f));
    EXPECT_EQ(apply_transfer(prepared, 0.0f).colour, colour_map_at(ColourMapId::cool_warm, 0.5f));

    TransferFunction speed;
    speed.auto_range = true;
    spade::FlowVolumeHeader h = header_with(spade::kFlowU, -3.0f, 1.0f);
    h.channel_min[spade::kFlowV] = 0.0f;
    h.channel_max[spade::kFlowV] = 4.0f;
    const auto s = resolve_range(speed, h);
    EXPECT_EQ(s.min, 0.0f);
    EXPECT_FLOAT_EQ(s.max, 5.0f);  // |(-3, 4, 0)|
}

// lambda2-hat ranges over its own channel, and on a diverging map keeps
// zero, the vortex boundary, at the neutral centre.
TEST(RenderTransferFunction, Lambda2HatRangesOverItsOwnChannel) {
    TransferFunction l2;
    l2.quantity = FlowQuantity::lambda2_hat;
    l2.auto_range = true;
    const auto plain = resolve_range(l2, header_with(spade::kFlowLambda2Hat, -3.0f, 0.5f));
    EXPECT_EQ(plain.min, -3.0f);
    EXPECT_EQ(plain.max, 0.5f);
    l2.colours = ColourMapId::cool_warm;
    const auto centred = resolve_range(l2, header_with(spade::kFlowLambda2Hat, -3.0f, 0.5f));
    EXPECT_EQ(centred.min, -3.0f);
    EXPECT_EQ(centred.max, 3.0f);
}

// An empty span maps every value to the bottom, never divides by zero.
TEST(RenderTransferFunction, AnEmptyAutoRangeMapsToTheBottom) {
    TransferFunction tf;
    tf.quantity = FlowQuantity::dye;
    tf.colours = ColourMapId::inferno;
    tf.auto_range = true;
    const auto p = prepare_transfer(tf, header_with(spade::kFlowDye, 0.5f, 0.5f));
    EXPECT_EQ(p.inv_span, 0.0f);
    EXPECT_EQ(apply_transfer(p, 0.5f).colour, colour_map_at(ColourMapId::inferno, 0.0f));
}

TEST(RenderTransferFunction, AMalformedTransferFunctionIsRefused) {
    const auto refused = [](const TransferFunction& tf, const char* what) {
        const auto r = validate_transfer_function(tf);
        ASSERT_FALSE(r.has_value()) << what << " must be refused";
        EXPECT_EQ(r.error().code, spade::Code::invalid_argument) << what;
    };
    const float nan = std::numeric_limits<float>::quiet_NaN();
    ASSERT_TRUE(validate_transfer_function(TransferFunction{}).has_value());
    {
        TransferFunction tf;
        tf.range_max = tf.range_min;
        refused(tf, "an empty fixed range");
    }
    {
        TransferFunction tf;
        tf.range_min = nan;
        refused(tf, "a NaN fixed range");
    }
    {
        TransferFunction tf;
        tf.extinction = {{0.0f, 1.0f}};
        refused(tf, "one extinction point");
    }
    {
        TransferFunction tf;
        tf.extinction.assign(9, {});
        for (size_t i = 0; i < 9; ++i) tf.extinction[i].value = static_cast<float>(i);
        refused(tf, "nine extinction points");
    }
    {
        TransferFunction tf;
        tf.extinction = {{1.0f, 1.0f}, {1.0f, 2.0f}};
        refused(tf, "values not strictly ascending");
    }
    {
        TransferFunction tf;
        tf.extinction = {{0.0f, -1.0f}, {1.0f, 2.0f}};
        refused(tf, "a negative sigma");
    }
    {
        TransferFunction tf;
        tf.extinction = {{0.0f, 1.0f}, {1.0f, std::numeric_limits<float>::infinity()}};
        refused(tf, "an infinite sigma");
    }
    {
        TransferFunction tf;
        tf.constant_colour = glm::vec3(1.5f, 0.0f, 0.0f);
        refused(tf, "a constant colour past 1");
    }
    {
        TransferFunction tf;
        tf.auto_range = true;
        tf.range_min = nan;  // ignored under auto_range
        EXPECT_TRUE(validate_transfer_function(tf).has_value()) << "auto_range does not read the fixed range";
    }
}

}  // namespace
