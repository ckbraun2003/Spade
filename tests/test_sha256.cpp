#include <gtest/gtest.h>

#include <string>

#include "world/sha256.hpp"

// SHA-256 (FIPS 180-4) for the geo map cache's file hashes (the SP-5 note, section 4). The vectors are the standard ones
// (the empty string, "abc", the two-block message, one million "a") and the lengths around a block boundary, whose
// digests were produced by Python's hashlib.

namespace {

using spade::sha256_hex;

TEST(Sha256, TheStandardVectors) {
    EXPECT_EQ(sha256_hex(""), "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855");
    EXPECT_EQ(sha256_hex("abc"), "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad");
    EXPECT_EQ(sha256_hex("abcdbcdecdefdefgefghfghighijhijkijkljklmklmnlmnomnopnopq"),
              "248d6a61d20638b8e5c026930c3e6039a33ce45964ff2167f6ecedd419db06c1");
    EXPECT_EQ(sha256_hex(std::string(1000000, 'a')), "cdc76e5c9914fb9281a1c7e284d73e67f1809a48a497200e046d39ccc7112cd0");
}

TEST(Sha256, TheLengthsAroundABlockBoundary) {
    struct Case {
        std::size_t n;
        const char* hex;
    };
    const Case cases[] = {
        {55, "9f4390f8d30c2dd92ec9f095b65e2b9ae9b0a925a5258e241c9f1e910f734318"},
        {56, "b35439a4ac6f0948b6d6f9e3c6af0f5f590ce20f1bde7090ef7970686ec6738a"},
        {57, "f13b2d724659eb3bf47f2dd6af1accc87b81f09f59f2b75e5c0bed6589dfe8c6"},
        {63, "7d3e74a05d7db15bce4ad9ec0658ea98e3f06eeecf16b4c6fff2da457ddc2f34"},
        {64, "ffe054fe7ae0cb6dc65c3af9b61d5209f439851db43d0ba5997337df154668eb"},
        {65, "635361c48bb9eab14198e76ea8ab7f1a41685d6ad62aa9146d301d4f17eb0ae0"},
        {119, "31eba51c313a5c08226adf18d4a359cfdfd8d2e816b13f4af952f7ea6584dcfb"},
        {120, "2f3d335432c70b580af0e8e1b3674a7c020d683aa5f73aaaedfdc55af904c21c"},
        {128, "6836cf13bac400e9105071cd6af47084dfacad4e5e302c94bfed24e013afb73e"},
    };
    for (const Case& c : cases) EXPECT_EQ(sha256_hex(std::string(c.n, 'a')), c.hex) << "length " << c.n;
}

TEST(Sha256, ABinaryStringWithNulBytes) {
    // 256 bytes, every value once: the digest is pinned (hashlib, bytes(range(256))).
    std::string all;
    for (int i = 0; i < 256; ++i) all.push_back(static_cast<char>(i));
    EXPECT_EQ(sha256_hex(all), "40aff2e9d2d8922e47afd4648e6967497158785fbd1da870e7110266bf944880");
}

}  // namespace
