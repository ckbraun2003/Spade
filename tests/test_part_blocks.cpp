// The version-2 part blocks and their one validator (L88; scene/part_blocks.hpp; the Spec B revision proposal, Revision 2).
//
// What is pinned here:
//   - A VALID DOCUMENT has no issue and reads back field for field; JSON reads as YAML with the same result; every key outside the
//     blocks (KAT's) is ignored.
//   - EVERY ISSUE IS REPORTED, not only the first, each with a code, the part's id, the field's dotted path and a text; an unknown
//     key inside a block is refused by its path.
//   - THE RULES BY BLOCK: geometry (the sha256, the unit for stl and obj only, the axes a proper rotation, the scale, the fallback),
//     mass_properties (a positive mass, a tensor only with a centre of mass, symmetric, positive definite, the triangle inequality),
//     install_points (unique names, the closed kind list, spin on motor points only, a unit quaternion), source, material.
//   - EVERY DECLARED FIELD IS CITED: mass, com, inertia and each install point, in the library's citation form.
//   - THE READER AND THE VALIDATOR AGREE: read_part_blocks fails exactly when validate_part_blocks reports, with the first issue.

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <string>
#include <string_view>
#include <vector>

#include "core/error.hpp"
#include "scene/part_blocks.hpp"

namespace {

using spade::Code;
using spade::scene::PartIssue;
namespace sc = spade::scene;

const std::string kSha(64, 'a');

// A valid document (YAML). Each test breaks one thing by replacing a piece of it.
[[nodiscard]] std::string valid() {
    return std::string(R"(id: "pix6c_fc"
provides: [cap.sensor.imu]
limits: {current_max: 1.0}
geometry:
  source: {format: step, sha256: ")") + kSha + R"(", object: "Pixhawk 6C"}
  to_part: {axes: [x, y, z], scale: 1.0}
  fallback: {kind: box, size: [0.084, 0.044, 0.016]}
mass_properties:
  mass: 0.059
  com: [0.0, 0.0, 0.008]
  inertia: [1.1e-5, 0, 0, 0, 2.2e-5, 0, 0, 0, 3.1e-5]
install_points:
  - {name: base, kind: fc, pos: [0, 0, 0], orient: [1, 0, 0, 0]}
  - {name: imu, kind: imu, pos: [0.01, 0, 0.004], orient: [1, 0, 0, 0]}
source: {id: "holybro_pixhawk6c", url: "https://example.org/pixhawk6c", license: "CC-BY-4.0", retrieved: "2026-11-17"}
material: "matte_plastic"
sources:
  mass_properties.mass: {source: "holybro_pixhawk6c", grade: published, note: "spec sheet"}
  mass_properties.com: {source: "", grade: assumed, note: "board centre, 8 mm above the base"}
  mass_properties.inertia: {source: "", grade: assumed, note: "a box of the board's size at its mass"}
  install_points.base: {source: "", grade: assumed, note: "the board's bottom face centre"}
  install_points.imu: {source: "holybro_pixhawk6c", grade: published, note: "drawing"}
  provenance.other: {kind: kat}
)";
}

[[nodiscard]] std::string with(std::string text, std::string_view from, std::string_view to) {
    const std::size_t at = text.find(from);
    EXPECT_NE(at, std::string::npos) << "the test's own text lacks: " << from;
    if (at != std::string::npos) text.replace(at, from.size(), to);
    return text;
}

[[nodiscard]] bool has_field(const std::vector<PartIssue>& issues, std::string_view field) {
    return std::any_of(issues.begin(), issues.end(), [&](const PartIssue& i) { return i.field == field; });
}

[[nodiscard]] std::string show(const std::vector<PartIssue>& issues) {
    std::string out;
    for (const PartIssue& i : issues) out += "[" + i.field + ": " + i.message + "] ";
    return out;
}

TEST(PartBlocks, TheValidExampleHasNoIssueAndReadsBackFieldForField) {
    const std::vector<PartIssue> issues = sc::validate_part_blocks(valid());
    EXPECT_TRUE(issues.empty()) << show(issues);
    const spade::Result<sc::PartBlocks> r = sc::read_part_blocks(valid());
    ASSERT_TRUE(r) << r.error().context;
    EXPECT_EQ(r->id, "pix6c_fc");
    ASSERT_TRUE(r->geometry);
    EXPECT_EQ(r->geometry->format, sc::GeometryFormat::step);
    EXPECT_EQ(r->geometry->sha256, kSha);
    EXPECT_EQ(r->geometry->object, "Pixhawk 6C");
    EXPECT_EQ(r->geometry->axes[0], 1);
    EXPECT_EQ(r->geometry->scale, 1.0);
    ASSERT_TRUE(r->geometry->fallback);
    EXPECT_EQ(r->geometry->fallback->kind, sc::GeometryFallback::Kind::box);
    EXPECT_EQ(r->geometry->fallback->size.size(), 3u);
    ASSERT_TRUE(r->mass_properties);
    EXPECT_EQ(r->mass_properties->mass, 0.059);
    ASSERT_TRUE(r->mass_properties->com);
    EXPECT_EQ(r->mass_properties->com->z, 0.008);
    ASSERT_TRUE(r->mass_properties->inertia);
    EXPECT_EQ((*r->mass_properties->inertia)[2][2], 3.1e-5);
    ASSERT_EQ(r->install_points.size(), 2u);
    EXPECT_EQ(r->install_points[0].name, "base");
    EXPECT_EQ(r->install_points[0].kind, "fc");
    EXPECT_EQ(r->install_points[1].pos.x, 0.01);
    ASSERT_TRUE(r->source);
    EXPECT_EQ(r->source->license, "CC-BY-4.0");
    EXPECT_EQ(r->material, "matte_plastic");
    EXPECT_EQ(r->sources.size(), 5u) << "the five cited paths of the blocks; KAT's provenance.other is not read";
    EXPECT_EQ(r->sources.at("mass_properties.mass").grade, sc::Grade::published);
}

TEST(PartBlocks, JsonReadsAsYamlWithTheSameResult) {
    const std::string json =
        std::string(R"({"id": "m1", "name": "KAT's own key", "mass": 99, "geometry": {"source": {"format": "stl", "sha256": ")") + kSha +
        R"(", "unit": "millimetre"}}, "mass_properties": {"mass": 0.04}, "source": {"id": "s", "url": "https://x.org/a", "license": "CC0-1.0", "retrieved": "2026-11-17"},
 "sources": {"mass_properties.mass": {"source": "s", "grade": "published", "note": "sheet"}}})";
    const std::vector<PartIssue> issues = sc::validate_part_blocks(json);
    EXPECT_TRUE(issues.empty()) << show(issues);
    const spade::Result<sc::PartBlocks> r = sc::read_part_blocks(json);
    ASSERT_TRUE(r) << r.error().context;
    EXPECT_EQ(r->geometry->unit, spade::MeshUnit::millimetre);
    EXPECT_EQ(r->mass_properties->mass, 0.04) << "the top-level `mass` is KAT's, not the block's";
}

TEST(PartBlocks, AnUnknownKeyInsideABlockIsRefusedByItsPathAndEveryIssueIsReported) {
    std::string text = with(valid(), "scale: 1.0}", "scale: 1.0, shear: 0.1}");
    text = with(text, "mass: 0.059", "mass: -1\n  mystery: 3");
    const std::vector<PartIssue> issues = sc::validate_part_blocks(text);
    EXPECT_TRUE(has_field(issues, "geometry.to_part.shear")) << show(issues);
    EXPECT_TRUE(has_field(issues, "mass_properties.mystery")) << show(issues);
    EXPECT_TRUE(has_field(issues, "mass_properties.mass")) << show(issues);
    EXPECT_GE(issues.size(), 3u) << "every issue, not only the first";
    for (const PartIssue& i : issues) {
        EXPECT_EQ(i.code, Code::invalid_argument);
        EXPECT_EQ(i.part_id, "pix6c_fc");
        EXPECT_FALSE(i.message.empty());
    }
}

TEST(PartBlocks, ADocumentThatIsNotAMappingOrHasNoBlockIsOneIssueWithNoField) {
    for (const char* text : {"", "[1, 2]", "id: x", "{not: yaml: at all"}) {
        const std::vector<PartIssue> issues = sc::validate_part_blocks(text);
        ASSERT_EQ(issues.size(), 1u) << text << " " << show(issues);
        EXPECT_EQ(issues[0].code, Code::invalid_argument);
    }
    const std::vector<PartIssue> no_id = sc::validate_part_blocks("material: \"x\"");
    EXPECT_TRUE(has_field(no_id, "id")) << show(no_id);
}

TEST(PartBlocks, GeometryRules) {
    EXPECT_TRUE(has_field(sc::validate_part_blocks(with(valid(), kSha, "ABCDEF")), "geometry.source.sha256"));
    EXPECT_TRUE(has_field(sc::validate_part_blocks(with(valid(), "format: step", "format: dwg")), "geometry.source.format"));
    // unit: required for stl and obj, refused for the formats that declare their own
    EXPECT_TRUE(has_field(sc::validate_part_blocks(with(valid(), "format: step", "format: stl")), "geometry.source.unit"));
    EXPECT_TRUE(has_field(sc::validate_part_blocks(with(valid(), "object:", "unit: metre, object:")), "geometry.source.unit"));
    EXPECT_TRUE(sc::validate_part_blocks(with(with(valid(), "format: step", "format: obj"), "object:", "unit: inch, object:")).empty());
    // axes: a permutation with signs, determinant +1
    EXPECT_TRUE(sc::validate_part_blocks(with(valid(), "axes: [x, y, z]", "axes: [x, -z, y]")).empty()) << "a rotation about x";
    EXPECT_TRUE(has_field(sc::validate_part_blocks(with(valid(), "axes: [x, y, z]", "axes: [x, y, -z]")), "geometry.to_part.axes"))
        << "a mirror";
    EXPECT_TRUE(has_field(sc::validate_part_blocks(with(valid(), "axes: [x, y, z]", "axes: [x, x, z]")), "geometry.to_part.axes"));
    EXPECT_TRUE(has_field(sc::validate_part_blocks(with(valid(), "axes: [x, y, z]", "axes: [x, y]")), "geometry.to_part.axes"));
    EXPECT_TRUE(has_field(sc::validate_part_blocks(with(valid(), "scale: 1.0}", "scale: 0}")), "geometry.to_part.scale"));
    EXPECT_TRUE(has_field(sc::validate_part_blocks(with(valid(), "scale: 1.0}", "scale: .inf}")), "geometry.to_part.scale"));
    // fallback
    EXPECT_TRUE(has_field(sc::validate_part_blocks(with(valid(), "kind: box", "kind: sphere")), "geometry.fallback.kind"));
    EXPECT_TRUE(has_field(sc::validate_part_blocks(with(valid(), "size: [0.084, 0.044, 0.016]", "size: [0.084, 0.044]")),
                          "geometry.fallback.size"));
    EXPECT_TRUE(has_field(sc::validate_part_blocks(with(valid(), "0.084, 0.044, 0.016", "0.084, -0.044, 0.016")), "geometry.fallback.size"));
    // geometry needs the source record
    const std::string no_source = with(valid(), "source: {id: \"holybro_pixhawk6c\", url: \"https://example.org/pixhawk6c\", license: \"CC-BY-4.0\", retrieved: \"2026-11-17\"}\n", "");
    EXPECT_TRUE(has_field(sc::validate_part_blocks(no_source), "source")) << show(sc::validate_part_blocks(no_source));
}

TEST(PartBlocks, MassPropertiesRules) {
    EXPECT_TRUE(has_field(sc::validate_part_blocks(with(valid(), "mass: 0.059", "mass: 0")), "mass_properties.mass"));
    EXPECT_TRUE(has_field(sc::validate_part_blocks(with(valid(), "mass: 0.059", "mass: .nan")), "mass_properties.mass"));
    EXPECT_TRUE(has_field(sc::validate_part_blocks(with(valid(), "com: [0.0, 0.0, 0.008]", "com: [0.0, 0.0]")), "mass_properties.com"));
    // a tensor needs a centre of mass
    const std::string no_com = with(valid(), "  com: [0.0, 0.0, 0.008]\n", "");
    EXPECT_TRUE(has_field(sc::validate_part_blocks(no_com), "mass_properties.inertia")) << show(sc::validate_part_blocks(no_com));
    // symmetric, positive definite, the triangle inequality
    EXPECT_TRUE(has_field(sc::validate_part_blocks(with(valid(), "[1.1e-5, 0, 0, 0, 2.2e-5, 0, 0, 0, 3.1e-5]",
                                                         "[1.1e-5, 1e-6, 0, 0, 2.2e-5, 0, 0, 0, 3.1e-5]")),
                          "mass_properties.inertia")) << "asymmetric";
    EXPECT_TRUE(has_field(sc::validate_part_blocks(with(valid(), "[1.1e-5, 0, 0, 0, 2.2e-5, 0, 0, 0, 3.1e-5]",
                                                         "[1.1e-5, 0, 0, 0, -2.2e-5, 0, 0, 0, 3.1e-5]")),
                          "mass_properties.inertia")) << "not positive definite";
    EXPECT_TRUE(has_field(sc::validate_part_blocks(with(valid(), "[1.1e-5, 0, 0, 0, 2.2e-5, 0, 0, 0, 3.1e-5]",
                                                         "[1.0e-5, 0, 0, 0, 1.0e-5, 0, 0, 0, 5.0e-5]")),
                          "mass_properties.inertia")) << "Izz above Ixx + Iyy cannot be a body's";
    EXPECT_TRUE(has_field(sc::validate_part_blocks(with(valid(), "3.1e-5]", "3.1e-5, 0]")), "mass_properties.inertia")) << "ten numbers";
    // mass only is a valid level
    const std::string mass_only = with(with(valid(), "  com: [0.0, 0.0, 0.008]\n", ""), "  inertia: [1.1e-5, 0, 0, 0, 2.2e-5, 0, 0, 0, 3.1e-5]\n", "");
    const std::string mass_only_no_cites = with(with(mass_only, "  mass_properties.com: {source: \"\", grade: assumed, note: \"board centre, 8 mm above the base\"}\n", ""),
                                                "  mass_properties.inertia: {source: \"\", grade: assumed, note: \"a box of the board's size at its mass\"}\n", "");
    EXPECT_TRUE(sc::validate_part_blocks(mass_only_no_cites).empty()) << show(sc::validate_part_blocks(mass_only_no_cites));
}

TEST(PartBlocks, InstallPointRules) {
    EXPECT_TRUE(has_field(sc::validate_part_blocks(with(valid(), "name: imu,", "name: base,")), "install_points[1].name")) << "a duplicate";
    EXPECT_TRUE(has_field(sc::validate_part_blocks(with(valid(), "name: base,", "name: \"\",")), "install_points[0].name"));
    EXPECT_TRUE(has_field(sc::validate_part_blocks(with(valid(), "kind: fc", "kind: flux_capacitor")), "install_points[0].kind"));
    EXPECT_TRUE(has_field(sc::validate_part_blocks(with(valid(), "kind: fc,", "kind: fc, spin: 1,")), "install_points[0].spin"))
        << "spin on a point that is not a motor's";
    for (const std::string_view kind : sc::kInstallPointKinds) {
        const std::string text = with(valid(), "kind: fc", "kind: " + std::string(kind));
        EXPECT_FALSE(has_field(sc::validate_part_blocks(text), "install_points[0].kind")) << kind;
    }
    const std::string motor = with(valid(), "kind: fc,", "kind: motor, spin: -1,");
    EXPECT_TRUE(sc::validate_part_blocks(motor).empty()) << show(sc::validate_part_blocks(motor));
    EXPECT_TRUE(has_field(sc::validate_part_blocks(with(motor, "spin: -1", "spin: 2")), "install_points[0].spin"));
    EXPECT_TRUE(has_field(sc::validate_part_blocks(with(valid(), "orient: [1, 0, 0, 0]}\n  - {name: imu", "orient: [0, 0, 0, 0]}\n  - {name: imu")),
                          "install_points[0].orient")) << "a zero quaternion";
    EXPECT_TRUE(has_field(sc::validate_part_blocks(with(valid(), "orient: [1, 0, 0, 0]}\n  - {name: imu", "orient: [2, 0, 0, 0]}\n  - {name: imu")),
                          "install_points[0].orient")) << "not a unit quaternion";
    EXPECT_TRUE(has_field(sc::validate_part_blocks(with(valid(), "pos: [0, 0, 0]", "pos: [0, 0]")), "install_points[0].pos"));
}

TEST(PartBlocks, SourceAndMaterialRules) {
    EXPECT_TRUE(has_field(sc::validate_part_blocks(with(valid(), "retrieved: \"2026-11-17\"", "retrieved: \"17 Nov 2026\"")), "source.retrieved"));
    EXPECT_TRUE(has_field(sc::validate_part_blocks(with(valid(), "license: \"CC-BY-4.0\"", "license: \"\"")), "source.license"));
    EXPECT_TRUE(has_field(sc::validate_part_blocks(with(valid(), "url: \"https://example.org/pixhawk6c\"", "url: \"\"")), "source.url"));
    EXPECT_TRUE(has_field(sc::validate_part_blocks(with(valid(), "material: \"matte_plastic\"", "material: \"\"")), "material"));
    EXPECT_TRUE(has_field(sc::validate_part_blocks(with(valid(), "material: \"matte_plastic\"", "material: \"" + std::string(65, 'm') + "\"")), "material"));
}

TEST(PartBlocks, EveryDeclaredFieldIsCitedInTheLibrarysForm) {
    const std::string no_mass_cite = with(valid(), "  mass_properties.mass: {source: \"holybro_pixhawk6c\", grade: published, note: \"spec sheet\"}\n", "");
    EXPECT_TRUE(has_field(sc::validate_part_blocks(no_mass_cite), "mass_properties.mass")) << show(sc::validate_part_blocks(no_mass_cite));
    const std::string no_point_cite = with(valid(), "  install_points.imu: {source: \"holybro_pixhawk6c\", grade: published, note: \"drawing\"}\n", "");
    EXPECT_TRUE(has_field(sc::validate_part_blocks(no_point_cite), "install_points.imu"));
    EXPECT_TRUE(has_field(sc::validate_part_blocks(with(valid(), "grade: published, note: \"spec sheet\"", "grade: guessed, note: \"x\"")),
                          "mass_properties.mass")) << "a grade that is not one of the six";
    EXPECT_TRUE(has_field(sc::validate_part_blocks(with(valid(), "{source: \"holybro_pixhawk6c\", grade: published, note: \"spec sheet\"}",
                                                         "{source: \"\", grade: published, note: \"spec sheet\"}")),
                          "mass_properties.mass")) << "a grade other than assumed cites a source";
    EXPECT_TRUE(has_field(sc::validate_part_blocks(with(valid(), "grade: assumed, note: \"board centre, 8 mm above the base\"", "grade: assumed, note: \"\"")),
                          "mass_properties.com")) << "an assumed value carries a note";
}

TEST(PartBlocks, TheReaderFailsExactlyWhenTheValidatorReportsAndNamesTheFirstIssue) {
    EXPECT_TRUE(sc::read_part_blocks(valid()));
    const std::string bad = with(with(valid(), "mass: 0.059", "mass: -1"), "kind: fc", "kind: nope");
    const std::vector<PartIssue> issues = sc::validate_part_blocks(bad);
    ASSERT_GE(issues.size(), 2u);
    const spade::Result<sc::PartBlocks> r = sc::read_part_blocks(bad);
    ASSERT_FALSE(r);
    EXPECT_EQ(r.error().code, issues[0].code);
    EXPECT_NE(r.error().context.find("part 'pix6c_fc'"), std::string::npos) << r.error().context;
    EXPECT_NE(r.error().context.find(issues[0].field), std::string::npos) << r.error().context;
}

// ---- L88l: the blocks of a part whose solid is in the part store (meshes, shapes) and the install point's adjust range ----

const std::string kShaB(64, 'b');

[[nodiscard]] std::string frame_part() {
    return std::string(R"(id: "x500_frame"
mass_properties: {mass: 0.610}
install_points:
  - {name: top_plate, kind: fc, pos: [0, 0, 0.0343], orient: [1, 0, 0, 0], adjust_range: {pos_min: [-0.01, -0.02, 0], pos_max: [0.01, 0.02, 0.005], yaw_deg: [-5, 5], pitch_deg: [0, 0], roll_deg: [-1.5, 1]}}
  - {name: mast_base, kind: gnss, pos: [-0.0577, 0.0405, 0.025], orient: [1, 0, 0, 0]}
meshes:
  - {store: {sha256: ")") + kSha + R"(", file: "x500-body.gltf"}, hash: "0123456789abcdef", role: visual}
  - {store: {sha256: ")" + kShaB + R"(", file: "plate.spmesh"}}
shapes:
  - {kind: box, size: [0.133, 0.045, 0.0335]}
  - {kind: cylinder, size: [0.005, 0.14], pos: [0, 0, 0.07], orient: [0.7071067811865476, 0.7071067811865476, 0, 0]}
sources:
  mass_properties.mass: {source: "holybro_x500_v2", grade: published, note: "kit page"}
  install_points.top_plate: {source: "", grade: assumed, note: "read off the frame mesh"}
  install_points.mast_base: {source: "", grade: assumed, note: "a layout choice"}
)";
}

TEST(PartBlocks, MeshesShapesAndAnAdjustRangeReadBackFieldForField) {
    const std::vector<PartIssue> issues = sc::validate_part_blocks(frame_part());
    EXPECT_TRUE(issues.empty()) << show(issues);
    const spade::Result<sc::PartBlocks> r = sc::read_part_blocks(frame_part());
    ASSERT_TRUE(r) << r.error().context;
    ASSERT_EQ(r->meshes.size(), 2u);
    EXPECT_EQ(r->meshes[0].sha256, kSha);
    EXPECT_EQ(r->meshes[0].file, "x500-body.gltf");
    EXPECT_EQ(r->meshes[0].hash, "0123456789abcdef");
    EXPECT_TRUE(r->meshes[0].visual);
    EXPECT_EQ(r->meshes[1].sha256, kShaB);
    EXPECT_EQ(r->meshes[1].hash, "") << "no hash declared";
    EXPECT_FALSE(r->meshes[1].visual) << "the default role is solid";
    ASSERT_EQ(r->shapes.size(), 2u);
    EXPECT_EQ(r->shapes[0].kind, sc::PartShape::Kind::box);
    ASSERT_EQ(r->shapes[0].size.size(), 3u);
    EXPECT_EQ(r->shapes[0].size[1], 0.045);
    EXPECT_EQ(r->shapes[0].pos.z, 0.0) << "centred on the part frame's origin by default";
    EXPECT_EQ(r->shapes[0].orient.w, 1.0);
    EXPECT_EQ(r->shapes[1].kind, sc::PartShape::Kind::cylinder);
    ASSERT_EQ(r->shapes[1].size.size(), 2u);
    EXPECT_EQ(r->shapes[1].size[1], 0.14);
    EXPECT_EQ(r->shapes[1].pos.z, 0.07);
    EXPECT_EQ(r->shapes[1].orient.x, 0.7071067811865476);
    ASSERT_EQ(r->install_points.size(), 2u);
    ASSERT_TRUE(r->install_points[0].adjust_range);
    const sc::AdjustRange& a = *r->install_points[0].adjust_range;
    EXPECT_EQ(a.pos_min.y, -0.02);
    EXPECT_EQ(a.pos_max.z, 0.005);
    EXPECT_EQ(a.yaw_deg[0], -5.0);
    EXPECT_EQ(a.yaw_deg[1], 5.0);
    EXPECT_EQ(a.pitch_deg[1], 0.0);
    EXPECT_EQ(a.roll_deg[0], -1.5);
    EXPECT_FALSE(r->install_points[1].adjust_range) << "no range declared: none";
    ASSERT_TRUE(r->mass_properties);
    EXPECT_EQ(r->mass_properties->mass, 0.610);
    EXPECT_FALSE(r->geometry) << "meshes are not `geometry`: no source record is needed";
}

TEST(PartBlocks, MeshRules) {
    const auto field = [](const std::string& text, std::string_view f) { return has_field(sc::validate_part_blocks(text), f); };
    EXPECT_TRUE(field(with(frame_part(), kSha, "ABCDEF"), "meshes[0].store.sha256"));
    EXPECT_TRUE(field(with(frame_part(), "x500-body.gltf", "dir/x500-body.gltf"), "meshes[0].store.file")) << "a directory part";
    EXPECT_TRUE(field(with(frame_part(), "x500-body.gltf", "..\\\\x500-body.gltf"), "meshes[0].store.file")) << "a backslash";
    EXPECT_TRUE(field(with(frame_part(), "x500-body.gltf", ".."), "meshes[0].store.file"));
    EXPECT_TRUE(field(with(frame_part(), "file: \"x500-body.gltf\"", "file: \"\""), "meshes[0].store.file"));
    EXPECT_TRUE(field(with(frame_part(), "hash: \"0123456789abcdef\"", "hash: \"0123456789ABCDEF\""), "meshes[0].hash")) << "upper case";
    EXPECT_TRUE(field(with(frame_part(), "hash: \"0123456789abcdef\"", "hash: \"0123456789abcde\""), "meshes[0].hash")) << "fifteen digits";
    EXPECT_TRUE(field(with(frame_part(), "role: visual", "role: ghost"), "meshes[0].role"));
    EXPECT_TRUE(sc::validate_part_blocks(with(frame_part(), "role: visual", "role: solid")).empty());
    // the same {sha256, file} cannot be listed twice
    const std::string twice = with(with(frame_part(), kShaB, kSha), "plate.spmesh", "x500-body.gltf");
    EXPECT_TRUE(field(twice, "meshes[1].store")) << show(sc::validate_part_blocks(twice));
    EXPECT_TRUE(field(with(frame_part(), "role: visual}", "role: visual, extra: 1}"), "meshes[0].extra"));
    EXPECT_TRUE(field(with(frame_part(), "{store: {sha256: \"" + kSha + "\", file: \"x500-body.gltf\"}", "{store: {sha256: \"" + kSha + "\", file: \"x500-body.gltf\", dir: x}"),
                        "meshes[0].store.dir"));
    // an empty list is a mistake, and so is a list that is not a list
    EXPECT_TRUE(field(with(with(frame_part(), "  - {store: {sha256: \"" + kSha + "\", file: \"x500-body.gltf\"}, hash: \"0123456789abcdef\", role: visual}\n", ""),
                           "  - {store: {sha256: \"" + kShaB + "\", file: \"plate.spmesh\"}}\n", ""),
                      "meshes")) << "no mesh listed";
    EXPECT_TRUE(field(with(frame_part(), "meshes:\n", "meshes: 3\nignored:\n"), "meshes"));
}

TEST(PartBlocks, ShapeRules) {
    const auto field = [](const std::string& text, std::string_view f) { return has_field(sc::validate_part_blocks(text), f); };
    EXPECT_TRUE(field(with(frame_part(), "kind: box", "kind: sphere"), "shapes[0].kind"));
    EXPECT_TRUE(field(with(frame_part(), "size: [0.133, 0.045, 0.0335]", "size: [0.133, 0.045]"), "shapes[0].size"));
    EXPECT_TRUE(field(with(frame_part(), "size: [0.133, 0.045, 0.0335]", "size: [0.133, 0, 0.0335]"), "shapes[0].size")) << "a zero side";
    EXPECT_TRUE(field(with(frame_part(), "size: [0.133, 0.045, 0.0335]", "size: [0.133, .inf, 0.0335]"), "shapes[0].size"));
    EXPECT_TRUE(field(with(frame_part(), "size: [0.005, 0.14]", "size: [0.005, 0.14, 0.2]"), "shapes[1].size")) << "a cylinder is radius and length";
    EXPECT_TRUE(field(with(frame_part(), "size: [0.005, 0.14]", "size: [-0.005, 0.14]"), "shapes[1].size"));
    EXPECT_TRUE(field(with(frame_part(), "pos: [0, 0, 0.07]", "pos: [0, 0]"), "shapes[1].pos"));
    EXPECT_TRUE(field(with(frame_part(), "orient: [0.7071067811865476, 0.7071067811865476, 0, 0]", "orient: [2, 0, 0, 0]"), "shapes[1].orient"));
    EXPECT_TRUE(field(with(frame_part(), "kind: box, size", "kind: box, colour: red, size"), "shapes[0].colour"));
    EXPECT_TRUE(field(with(frame_part(), "shapes:\n", "shapes: []\nignored:\n"), "shapes")) << "an empty list";
}

TEST(PartBlocks, AdjustRangeRules) {
    const auto field = [](const std::string& text, std::string_view f) { return has_field(sc::validate_part_blocks(text), f); };
    EXPECT_TRUE(field(with(frame_part(), "yaw_deg: [-5, 5]", "yaw_deg: [5, -5]"), "install_points[0].adjust_range.yaw_deg")) << "min above max";
    EXPECT_TRUE(field(with(frame_part(), "yaw_deg: [-5, 5]", "yaw_deg: [-5]"), "install_points[0].adjust_range.yaw_deg"));
    EXPECT_TRUE(field(with(frame_part(), "yaw_deg: [-5, 5]", "yaw_deg: [-5, .nan]"), "install_points[0].adjust_range.yaw_deg"));
    EXPECT_TRUE(field(with(frame_part(), "yaw_deg: [-5, 5]", "yaw_deg: [-400, 5]"), "install_points[0].adjust_range.yaw_deg")) << "more than 360 degrees";
    EXPECT_TRUE(field(with(frame_part(), "pos_min: [-0.01, -0.02, 0]", "pos_min: [0.02, -0.02, 0]"), "install_points[0].adjust_range.pos_min"))
        << "a pos_min above its pos_max";
    EXPECT_TRUE(field(with(frame_part(), "pos_max: [0.01, 0.02, 0.005]", "pos_max: [0.01, 0.02]"), "install_points[0].adjust_range.pos_max"));
    EXPECT_TRUE(field(with(frame_part(), "roll_deg: [-1.5, 1]}", "roll_deg: [-1.5, 1], twist: [0, 1]}"), "install_points[0].adjust_range.twist"));
    EXPECT_TRUE(field(with(frame_part(), "adjust_range: {", "adjust_range: 3, ignored: {"), "install_points[0].adjust_range")) << "not a mapping";
    // every key is optional: a part may allow yaw only
    const std::string yaw_only =
        with(frame_part(), "adjust_range: {pos_min: [-0.01, -0.02, 0], pos_max: [0.01, 0.02, 0.005], yaw_deg: [-5, 5], pitch_deg: [0, 0], roll_deg: [-1.5, 1]}",
             "adjust_range: {yaw_deg: [-5, 5]}");
    EXPECT_TRUE(sc::validate_part_blocks(yaw_only).empty()) << show(sc::validate_part_blocks(yaw_only));
    const spade::Result<sc::PartBlocks> r = sc::read_part_blocks(yaw_only);
    ASSERT_TRUE(r);
    ASSERT_TRUE(r->install_points[0].adjust_range);
    EXPECT_EQ(r->install_points[0].adjust_range->yaw_deg[1], 5.0);
    EXPECT_EQ(r->install_points[0].adjust_range->pos_max.x, 0.0) << "a missing key is no range";
    EXPECT_EQ(r->install_points[0].adjust_range->roll_deg[0], 0.0);
}

}  // namespace
