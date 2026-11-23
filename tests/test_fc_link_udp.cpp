// tests/test_fc_link_udp.cpp -- the companion link's UDP channel (tools/fc_link/udp.hpp), two channels on the loopback interface with
// system-picked ports, in one process.
#include <gtest/gtest.h>

#include <chrono>
#include <vector>

#include "../tools/fc_link/udp.hpp"

namespace {

using namespace spade::fc_link;
using namespace std::chrono_literals;

const std::vector<uint8_t> kHello = {0xfd, 0x01, 0x02, 0x03};

}  // namespace

TEST(FcLinkUdp, AChannelOnPortZeroSaysWhichPortItGot) {
    auto a = UdpChannel::open(0);
    ASSERT_TRUE(a.has_value()) << a.error().context;
    EXPECT_NE((*a)->local_port(), 0);
    EXPECT_EQ((*a)->peer_port(), 0) << "no peer yet";
}

TEST(FcLinkUdp, ADatagramGoesBothWaysExactlyAndOneReadIsOneDatagram) {
    auto a = UdpChannel::open(0);
    ASSERT_TRUE(a.has_value());
    auto b = UdpChannel::open(0, (*a)->local_port());
    ASSERT_TRUE(b.has_value());
    EXPECT_EQ((*b)->peer_port(), (*a)->local_port());

    const std::vector<uint8_t> second = {9, 8};
    ASSERT_TRUE((*b)->write(kHello).has_value());
    ASSERT_TRUE((*b)->write(second).has_value());
    auto r1 = (*a)->read_some(2s);
    ASSERT_TRUE(r1.has_value());
    EXPECT_EQ(*r1, kHello) << "the first datagram, whole";
    auto r2 = (*a)->read_some(2s);
    ASSERT_TRUE(r2.has_value());
    EXPECT_EQ(*r2, second) << "the second datagram, not joined to the first";

    EXPECT_EQ((*a)->peer_port(), (*b)->local_port()) << "a learned its peer from the first datagram it heard";
    ASSERT_TRUE((*a)->write(kHello).has_value());
    auto back = (*b)->read_some(2s);
    ASSERT_TRUE(back.has_value());
    EXPECT_EQ(*back, kHello);
}

TEST(FcLinkUdp, WritingWithNoPeerKnownRefusesByName) {
    auto a = UdpChannel::open(0);
    ASSERT_TRUE(a.has_value());
    auto w = (*a)->write(kHello);
    ASSERT_FALSE(w.has_value());
    EXPECT_NE(w.error().context.find("no peer"), std::string::npos) << w.error().context;
}

TEST(FcLinkUdp, ReadingWithNothingSentReturnsEmptyAfterTheTimeout) {
    auto a = UdpChannel::open(0);
    ASSERT_TRUE(a.has_value());
    const auto t0 = std::chrono::steady_clock::now();
    auto r = (*a)->read_some(60ms);
    const auto waited = std::chrono::steady_clock::now() - t0;
    ASSERT_TRUE(r.has_value()) << r.error().context;
    EXPECT_TRUE(r->empty());
    EXPECT_GE(waited, 50ms);
    EXPECT_LT(waited, 2s);
}

TEST(FcLinkUdp, AnEmptyWriteIsRefusedAndAPortInUseIsNamed) {
    auto a = UdpChannel::open(0);
    ASSERT_TRUE(a.has_value());
    auto b = UdpChannel::open(0, (*a)->local_port());
    ASSERT_TRUE(b.has_value());
    EXPECT_FALSE((*b)->write({}).has_value()) << "nothing to send is not a datagram";
    auto twice = UdpChannel::open((*a)->local_port());
    ASSERT_FALSE(twice.has_value());
    EXPECT_NE(twice.error().context.find("udp"), std::string::npos) << twice.error().context;
}
