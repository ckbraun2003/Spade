// core/flow_volume.hpp: the airflow render volume's contract. Its cell
// order, its channel lookup, and its validation (L6).

#include <gtest/gtest.h>

#include <cstdint>
#include <limits>

#include "core/flow_volume.hpp"

namespace {

using spade::FlowVolume;

// A 2 x 3 x 4 volume whose every half is its own (cell, channel) code.
[[nodiscard]] FlowVolume make_volume() {
    FlowVolume volume;
    volume.header.origin = glm::vec3(-1.0f, 0.0f, 2.0f);
    volume.header.cell = 0.25f;
    volume.header.nx = 2;
    volume.header.ny = 3;
    volume.header.nz = 4;
    const size_t cells = 2u * 3u * 4u;
    volume.v.resize(cells * 4u);
    volume.s.resize(cells * 4u);
    for (size_t c = 0; c < cells; ++c) {
        for (uint32_t ch = 0; ch < 4u; ++ch) {
            volume.v[c * 4u + ch] = static_cast<uint16_t>(c * 8u + ch);
            volume.s[c * 4u + ch] = static_cast<uint16_t>(c * 8u + 4u + ch);
        }
    }
    return volume;
}

TEST(CoreFlowVolume, CellsRunIFastestThenJThenK) {
    const FlowVolume volume = make_volume();
    EXPECT_EQ(spade::flow_cell_index(volume.header, 0, 0, 0), 0u);
    EXPECT_EQ(spade::flow_cell_index(volume.header, 1, 0, 0), 1u);
    EXPECT_EQ(spade::flow_cell_index(volume.header, 0, 1, 0), 2u);
    EXPECT_EQ(spade::flow_cell_index(volume.header, 0, 0, 1), 6u);
    EXPECT_EQ(spade::flow_cell_index(volume.header, 1, 2, 3), 23u);
}

TEST(CoreFlowVolume, EachChannelReadsItsTextureAndLane) {
    const FlowVolume volume = make_volume();
    for (size_t cell : {size_t{0}, size_t{7}, size_t{23}}) {
        for (uint32_t ch = 0; ch < spade::kFlowChannelCount; ++ch) {
            EXPECT_EQ(spade::flow_half(volume, cell, ch), static_cast<uint16_t>(cell * 8u + ch))
                << "cell " << cell << ", channel " << ch;
        }
    }
}

TEST(CoreFlowVolume, AWellFormedVolumeValidates) {
    const spade::Result<void> ok = spade::validate_flow_volume(make_volume());
    EXPECT_TRUE(ok.has_value()) << ok.error().context;
}

// L6: each malformation is refused by name, never drawn in part.
TEST(CoreFlowVolume, AMalformedVolumeIsRefused) {
    const auto refused = [](const FlowVolume& volume, const char* what) {
        const spade::Result<void> r = spade::validate_flow_volume(volume);
        ASSERT_FALSE(r.has_value()) << what << " must be refused";
        EXPECT_EQ(r.error().code, spade::Code::invalid_argument) << what;
    };
    const float nan = std::numeric_limits<float>::quiet_NaN();
    {
        FlowVolume v = make_volume();
        v.header.nz = 0;
        v.v.clear();
        v.s.clear();
        refused(v, "a zero dimension");
    }
    {
        FlowVolume v = make_volume();
        v.header.cell = 0.0f;
        refused(v, "a zero cell");
    }
    {
        FlowVolume v = make_volume();
        v.header.cell = nan;
        refused(v, "a NaN cell");
    }
    {
        FlowVolume v = make_volume();
        v.header.origin.y = std::numeric_limits<float>::infinity();
        refused(v, "an infinite origin");
    }
    {
        FlowVolume v = make_volume();
        v.v.pop_back();
        refused(v, "a short V texture");
    }
    {
        FlowVolume v = make_volume();
        v.s.push_back(0);
        refused(v, "a long S texture");
    }
    {
        FlowVolume v = make_volume();
        v.header.channel_min[spade::kFlowPressure] = 2.0f;
        v.header.channel_max[spade::kFlowPressure] = 1.0f;
        refused(v, "a channel whose min is past its max");
    }
    {
        FlowVolume v = make_volume();
        v.header.channel_max[spade::kFlowQHat] = nan;
        refused(v, "a channel range that is not finite");
    }
}

}  // namespace
