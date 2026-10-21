// test_core_utf8.cpp -- core/utf8.hpp: is_utf8() takes well-formed UTF-8
// (RFC 3629) and nothing else.

#include <gtest/gtest.h>

#include "core/utf8.hpp"

TEST(CoreUtf8, WellFormedTextOnly) {
    EXPECT_TRUE(spade::is_utf8(""));
    EXPECT_TRUE(spade::is_utf8("cube 2"));
    EXPECT_TRUE(spade::is_utf8("\xC3\xA9"));          // U+00E9
    EXPECT_TRUE(spade::is_utf8("\xE2\x82\xAC"));      // U+20AC
    EXPECT_TRUE(spade::is_utf8("\xF0\x9F\x9A\x81"));  // U+1F681
    EXPECT_TRUE(spade::is_utf8("\xF4\x8F\xBF\xBF"));  // U+10FFFF, the last
    EXPECT_FALSE(spade::is_utf8("\x80"));              // a continuation byte alone
    EXPECT_FALSE(spade::is_utf8("\xFF"));              // never in UTF-8
    EXPECT_FALSE(spade::is_utf8("\xC3"));              // cut short
    EXPECT_FALSE(spade::is_utf8("a\xE2\x82"));         // cut short
    EXPECT_FALSE(spade::is_utf8("\xE2\x28\xA1"));      // a broken continuation
    EXPECT_FALSE(spade::is_utf8("\xC0\xAF"));          // '/', overlong
    EXPECT_FALSE(spade::is_utf8("\xE0\x80\xAF"));      // '/', overlong
    EXPECT_FALSE(spade::is_utf8("\xED\xA0\x80"));      // U+D800, a surrogate
    EXPECT_FALSE(spade::is_utf8("\xF4\x90\x80\x80"));  // past U+10FFFF
}
