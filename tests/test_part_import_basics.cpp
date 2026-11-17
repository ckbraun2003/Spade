#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <limits>
#include <set>
#include <string>
#include <vector>

#include "../tools/part_import/report.hpp"
#include "../tools/part_import/store.hpp"
#include "../tools/part_import/transform.hpp"
#include "world/mesh.hpp"
#include "world/mesh_mass.hpp"

// spade_part_import's pure parts (L88; docs/design/interface/plans/2026-11-17-part-import-design.md): the stable codes and the JSON the
// tool prints, the part frame (axes and scale), the unit guess and the cache key.

namespace {

using namespace spade;
using namespace spade::partimport;

// ---- codes ---------------------------------------------------------------------------------------------------------------------

TEST(PartImportCodes, EveryCodeOfTheDesignNoteHasItsExitCodeAndAdvice) {
    const std::vector<std::pair<std::string_view, int>> expected = {
        {"ok", 0},
        {"store_io", 1}, {"file_unreadable", 1}, {"internal", 1},
        {"usage", 2}, {"unit_required", 2}, {"unit_not_allowed", 2}, {"axes_invalid", 2}, {"axes_mirror", 2}, {"scale_invalid", 2},
        {"object_not_found", 2}, {"deflection_required", 2}, {"store_required", 2}, {"store_in_git_tree", 2},
        {"format_unknown", 3}, {"format_unavailable", 3}, {"external_buffers", 3}, {"step_unavailable", 3}, {"converter_failed", 3},
        {"file_invalid", 4}, {"mesh_refused", 4},
        // make_mesh_diagnosed's kinds (Core), the names its refusal_name() gives
        {"empty", 4}, {"bad_indices", 4}, {"non_finite", 4}, {"unit_invalid", 4}, {"coincident_vertices", 4}, {"degenerate_triangle", 4},
        {"duplicate_triangle", 4}, {"not_closed", 4}, {"not_manifold", 4}, {"inconsistent_winding", 4}, {"inside_out", 4},
        {"self_intersecting", 4}, {"shells_not_nested", 4}, {"too_large", 4}, {"too_many_shells", 4},
        {"cancelled", 5},
        {"cross_check_flagged", 6},
    };
    EXPECT_EQ(all_codes().size(), expected.size());
    std::set<std::string_view> seen;
    for (const CodeInfo& c : all_codes()) EXPECT_TRUE(seen.insert(c.code).second) << "a code twice: " << c.code;
    for (const auto& [code, exit] : expected) {
        const CodeInfo& info = code_info(code);
        EXPECT_EQ(info.code, code);
        EXPECT_EQ(info.exit, exit) << code;
        if (code != "ok") {
            EXPECT_FALSE(info.advice.empty()) << code << " has no advice";
        }
    }
}

TEST(PartImportCodes, AStringThatIsNoCodeIsTheInternalOne) {
    EXPECT_EQ(code_info("no_such_code").code, "internal");
    EXPECT_EQ(code_info("no_such_code").exit, kExitIo);
}

TEST(PartImportCodes, TheAdviceForAnOpenMeshSaysToExportAClosedSolid) {
    EXPECT_NE(std::string(code_info("not_closed").advice).find("closed solid"), std::string::npos);
}

// ---- JSON ----------------------------------------------------------------------------------------------------------------------

TEST(PartImportJson, StringsAreEscapedAndUtf8PassesThrough) {
    EXPECT_EQ(json_quote("plain"), "\"plain\"");
    EXPECT_EQ(json_quote("a\"b\\c\n\t\r"), "\"a\\\"b\\\\c\\n\\t\\r\"");
    EXPECT_EQ(json_quote(std::string("\x01\x1f", 2)), "\"\\u0001\\u001f\"");
    EXPECT_EQ(json_quote("caf\xc3\xa9"), "\"caf\xc3\xa9\"");
}

TEST(PartImportJson, NumbersAreTheShortestRoundTripAndNonFiniteIsNull) {
    EXPECT_EQ(json_number(1.0), "1");
    EXPECT_EQ(json_number(-23.4), "-23.4");
    EXPECT_EQ(json_number(0.1), "0.1");
    EXPECT_EQ(json_number(std::numeric_limits<double>::quiet_NaN()), "null");
    EXPECT_EQ(json_number(std::numeric_limits<double>::infinity()), "null");
}

TEST(PartImportJson, AnObjectKeepsItsKeysInOrderOnOneLine) {
    JsonObject o;
    o.str("event", "result").num("volume", 1.5).integer("n", -3).boolean("ok", true).null("x").raw("box", "[1,2]").str("q", "a\"b");
    EXPECT_EQ(o.line(), "{\"event\":\"result\",\"volume\":1.5,\"n\":-3,\"ok\":true,\"x\":null,\"box\":[1,2],\"q\":\"a\\\"b\"}");
    EXPECT_EQ(JsonObject{}.line(), "{}");
}

// ---- axes and scale ------------------------------------------------------------------------------------------------------------

TEST(PartImportAxes, TheIdentityAndASimpleRotationParseAndWriteBackAsTheyWereGiven) {
    const auto id = parse_axes("x,y,z");
    ASSERT_TRUE(id.has_value()) << id.error().message;
    EXPECT_TRUE(id->identity());
    EXPECT_EQ(id->text(), "x,y,z");
    const auto r = parse_axes(" -Y , x , z ");
    ASSERT_TRUE(r.has_value()) << r.error().message;
    EXPECT_FALSE(r->identity());
    EXPECT_EQ(r->text(), "-y,x,z");
    EXPECT_EQ(apply_axes(*r, {1.5, 2.25, -3.125}), (glm::dvec3{-2.25, 1.5, -3.125}));
}

TEST(PartImportAxes, OfTheFortyEightSignedPermutationsTwentyFourAreRotationsAndTwentyFourMirrorsAreRefused) {
    int ok = 0, mirror = 0;
    const std::array<std::array<int, 3>, 6> perms = {{{0, 1, 2}, {0, 2, 1}, {1, 0, 2}, {1, 2, 0}, {2, 0, 1}, {2, 1, 0}}};
    for (const auto& p : perms) {
        for (int bits = 0; bits < 8; ++bits) {
            std::string text;
            for (int i = 0; i < 3; ++i) {
                if (i != 0) text += ',';
                if ((bits >> i) & 1) text += '-';
                text += "xyz"[p[i]];
            }
            const auto a = parse_axes(text);
            if (a.has_value()) {
                ++ok;
                EXPECT_EQ(a->text(), text);
                // the three columns, mapped: the determinant of the map is +1, whatever the text says
                const glm::dvec3 c0 = apply_axes(*a, {1, 0, 0}), c1 = apply_axes(*a, {0, 1, 0}), c2 = apply_axes(*a, {0, 0, 1});
                EXPECT_EQ(glm::dot(glm::cross(c0, c1), c2), 1.0) << text;
            } else {
                EXPECT_EQ(a.error().code, "axes_mirror") << text << ": " << a.error().message;
                ++mirror;
            }
        }
    }
    EXPECT_EQ(ok, 24);
    EXPECT_EQ(mirror, 24);
}

TEST(PartImportAxes, TextThatIsNotThreeDistinctAxesIsInvalid) {
    for (const char* bad : {"", "x,y", "x,y,z,x", "x,x,z", "a,y,z", "--x,y,z", "x y z", "x,,z", "+x,y,z", "x;y;z", "-,y,z", "xy,z,x"}) {
        const auto a = parse_axes(bad);
        ASSERT_FALSE(a.has_value()) << "'" << bad << "'";
        EXPECT_EQ(a.error().code, "axes_invalid") << bad;
        EXPECT_FALSE(a.error().message.empty());
    }
}

TEST(PartImportAxes, AScaleIsAFiniteNumberAboveZero) {
    EXPECT_EQ(parse_scale("1").value(), 1.0);
    EXPECT_EQ(parse_scale("0.001").value(), 0.001);
    EXPECT_EQ(parse_scale("2.5e-3").value(), 2.5e-3);
    for (const char* bad : {"", "0", "-1", "abc", "nan", "inf", "1x", " "}) {
        const auto s = parse_scale(bad);
        ASSERT_FALSE(s.has_value()) << "'" << bad << "'";
        EXPECT_EQ(s.error().code, "scale_invalid");
    }
}

// A closed box of sides sx, sy, sz with a corner at the origin, in metres: 8 vertices, 12 triangles, counter-clockwise from outside.
std::shared_ptr<const TriangleMesh> box(double sx, double sy, double sz) {
    MeshInput in;
    in.unit = MeshUnit::metre;
    for (int i = 0; i < 8; ++i) in.positions.push_back({(i & 1) ? sx : 0.0, (i & 2) ? sy : 0.0, (i & 4) ? sz : 0.0});
    // vertex i has x = bit 0, y = bit 1, z = bit 2
    in.indices = {0, 2, 3, 0, 3, 1,   // z = 0, facing -z
                  4, 5, 7, 4, 7, 6,   // z = sz, facing +z
                  0, 1, 5, 0, 5, 4,   // y = 0, facing -y
                  2, 6, 7, 2, 7, 3,   // y = sy, facing +y
                  0, 4, 6, 0, 6, 2,   // x = 0, facing -x
                  1, 3, 7, 1, 7, 5};  // x = sx, facing +x
    auto m = make_mesh(in, "box");
    EXPECT_TRUE(m.has_value()) << (m.has_value() ? "" : m.error().context);
    return m.has_value() ? *m : nullptr;
}

struct Box3 {
    glm::vec3 lo{std::numeric_limits<float>::max()};
    glm::vec3 hi{-std::numeric_limits<float>::max()};
};
Box3 bounds(const TriangleMesh& m) {
    Box3 b;
    for (const glm::vec3& p : m.positions()) {
        b.lo = glm::min(b.lo, p);
        b.hi = glm::max(b.hi, p);
    }
    return b;
}

TEST(PartImportTransform, TheIdentityAndAScaleOfOneGiveTheSameMesh) {
    const auto m = box(2, 3, 5);
    ASSERT_TRUE(m);
    const auto same = transform_mesh(m, Axes{}, 1.0);
    ASSERT_TRUE(same.has_value()) << same.error().message;
    EXPECT_EQ(same->get(), m.get());
}

TEST(PartImportTransform, ARotationPermutesTheBoxExactlyAndKeepsTheVolumeAndTheWinding) {
    const auto m = box(2, 3, 5);
    ASSERT_TRUE(m);
    const auto axes = parse_axes("-y,x,z");
    ASSERT_TRUE(axes.has_value());
    const auto t = transform_mesh(m, *axes, 1.0);
    ASSERT_TRUE(t.has_value()) << t.error().message;
    const Box3 b = bounds(**t);
    EXPECT_EQ(b.lo, (glm::vec3{-3, 0, 0}));  // part x = -source y, part y = source x, part z = source z
    EXPECT_EQ(b.hi, (glm::vec3{0, 2, 5}));
    EXPECT_EQ((*t)->triangle_count(), 12u);
    EXPECT_NEAR(mesh_mass_properties(**t).volume, 30.0, 1e-9);  // positive: the winding stayed outward
    EXPECT_NE((*t)->content_hash(), m->content_hash());
}

TEST(PartImportTransform, AScaleMultipliesOnceAndTheVolumeGoesAsTheCube) {
    const auto m = box(2, 3, 5);
    ASSERT_TRUE(m);
    const auto t = transform_mesh(m, Axes{}, 0.5);
    ASSERT_TRUE(t.has_value()) << t.error().message;
    const Box3 b = bounds(**t);
    EXPECT_EQ(b.lo, (glm::vec3{0, 0, 0}));
    EXPECT_EQ(b.hi, (glm::vec3{1.0f, 1.5f, 2.5f}));
    EXPECT_NEAR(mesh_mass_properties(**t).volume, 3.75, 1e-9);
}

TEST(PartImportTransform, AxesThenScaleInOneStep) {
    const auto m = box(2, 3, 5);
    ASSERT_TRUE(m);
    const auto t = transform_mesh(m, parse_axes("z,x,y").value(), 2.0);  // part x = source z, y = source x, z = source y
    ASSERT_TRUE(t.has_value()) << t.error().message;
    EXPECT_EQ(bounds(**t).hi, (glm::vec3{10, 4, 6}));
}

// ---- the unit guess ------------------------------------------------------------------------------------------------------------

TEST(PartImportUnitGuess, MillimetresWhenTheLongestSideIsFiveToFiveThousandMetresWhenItIsMuchSmaller) {
    const auto mm = guess_unit({250.0, 40.0, 12.0});
    ASSERT_TRUE(mm.has_value());
    EXPECT_EQ(mm->unit, "mm");
    EXPECT_FALSE(mm->basis.empty());
    const auto m = guess_unit({0.25, 0.04, 0.012});
    ASSERT_TRUE(m.has_value());
    EXPECT_EQ(m->unit, "m");
    EXPECT_EQ(guess_unit({5.0, 1.0, 1.0})->unit, "mm");     // 5 is the first millimetre length
    EXPECT_EQ(guess_unit({4.999, 1.0, 1.0})->unit, "m");
    EXPECT_EQ(guess_unit({0.005, 0.001, 0.001})->unit, "m");  // 0.005 is the first metre length
    EXPECT_EQ(guess_unit({4999.0, 1.0, 1.0})->unit, "mm");
}

TEST(PartImportUnitGuess, NothingOutsideBothBandsOrForAnEmptyBox) {
    EXPECT_FALSE(guess_unit({0.004, 0.001, 0.001}).has_value());
    EXPECT_FALSE(guess_unit({5000.0, 1.0, 1.0}).has_value());
    EXPECT_FALSE(guess_unit({0.0, 0.0, 0.0}).has_value());
    EXPECT_FALSE(guess_unit({std::numeric_limits<double>::quiet_NaN(), 1.0, 1.0}).has_value());
}

// ---- the cache key -------------------------------------------------------------------------------------------------------------

TEST(PartImportKey, TheTextAndTheKeyArePinned) {
    KeyInputs a;
    a.converter = "mesh-1";
    a.unit = "mm";
    EXPECT_EQ(cache_key_text(a), "spade-part-import-key-1;converter=mesh-1;tolerance=0;angle=0;unit=mm;axes=x,y,z;scale=1;object=");
    EXPECT_EQ(cache_key(a), "fce0a49e1e33eb5c");
    KeyInputs b;
    b.converter = "step-OCCT 7.9.3 cadquery-ocp 7.9.3.1.1";
    b.tolerance_m = 0.0001;
    b.angle_rad = 0.5;
    b.axes = parse_axes("-y,x,z").value();
    b.scale = 0.001;
    b.object = "arm_left";
    EXPECT_EQ(cache_key(b), "f35be0a647de9509");
}

TEST(PartImportKey, EachInputChangesTheKeyAndTheSameInputsGiveTheSame) {
    KeyInputs base;
    base.converter = "mesh-1";
    base.unit = "mm";
    const std::string key = cache_key(base);
    EXPECT_EQ(key.size(), 16u);
    EXPECT_EQ(cache_key(base), key);
    std::set<std::string> keys = {key};
    const auto vary = [&](auto&& change) {
        KeyInputs k = base;
        change(k);
        EXPECT_TRUE(keys.insert(cache_key(k)).second) << "an input did not change the key";
    };
    vary([](KeyInputs& k) { k.converter = "mesh-2"; });
    vary([](KeyInputs& k) { k.tolerance_m = 1e-4; });
    vary([](KeyInputs& k) { k.angle_rad = 0.5; });
    vary([](KeyInputs& k) { k.unit = "cm"; });
    vary([](KeyInputs& k) { k.axes = parse_axes("y,-x,z").value(); });
    vary([](KeyInputs& k) { k.scale = 2.0; });
    vary([](KeyInputs& k) { k.object = "arm"; });
    EXPECT_EQ(keys.size(), 8u);
}

}  // namespace
