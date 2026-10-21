// The library's part files (Task L2 of docs/design/core/plans/2026-10-21-
// default-drone-library-plan.md; scene/part_file.hpp): a motor, a propeller,
// an ESC or a battery as a file of its own, every field cited.

#include <gtest/gtest.h>

#include <array>
#include <bit>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <initializer_list>
#include <optional>
#include <string>
#include <string_view>
#include <variant>
#include <vector>

#include "core/error.hpp"
#include "core/path_text.hpp"
#include "scene/library_sources.hpp"
#include "scene/part_file.hpp"
#include "unique_temp.hpp"
#include "vehicles/airframe_compile.hpp"
#include "vehicles/composite_inertia.hpp"

namespace {

namespace fs = std::filesystem;

using spade::Code;
using spade::scene::BatteryPart;
using spade::scene::BladeGeometry;
using spade::scene::EscPart;
using spade::scene::FieldSource;
using spade::scene::Grade;
using spade::scene::MotorPart;
using spade::scene::PartFile;
using spade::scene::PartKind;
using spade::scene::PartSources;
using spade::scene::PropPart;
using spade::scene::SourceRef;
using spade::scene::SourceRegistry;
using spade::vehicles::BlockShape;
using spade::vehicles::PartShape;

[[nodiscard]] bool contains(std::string_view haystack, std::string_view needle) {
    return haystack.find(needle) != std::string_view::npos;
}

template <typename T>
void expect_refused(const spade::Result<T>& r, Code code, std::initializer_list<std::string_view> needles) {
    ASSERT_FALSE(r.has_value());
    EXPECT_EQ(r.error().code, code) << r.error().context;
    for (const std::string_view needle : needles) {
        EXPECT_TRUE(contains(r.error().context, needle)) << "missing '" << needle << "' in: " << r.error().context;
    }
}

[[nodiscard]] uint64_t bits(double v) { return std::bit_cast<uint64_t>(v); }

// `text` with `from` replaced by `to` (once; it must occur).
[[nodiscard]] std::string with(std::string text, std::string_view from, std::string_view to) {
    const std::size_t at = text.find(from);
    EXPECT_NE(at, std::string::npos) << "'" << from << "' not in:\n" << text;
    if (at != std::string::npos) text.replace(at, from.size(), to);
    return text;
}

// One citation per field, the first `measured` from UIUC, the rest assumed.
[[nodiscard]] PartSources cite(std::initializer_list<std::string_view> fields) {
    PartSources out;
    bool first = true;
    for (const std::string_view f : fields) {
        out.push_back(FieldSource{std::string(f), first ? SourceRef{"uiuc_propdb", Grade::measured, ""}
                                                        : SourceRef{"", Grade::assumed, "typical"}});
        first = false;
    }
    return out;
}

// Values chosen to break a lossy writer: thirds, tenths, a tiny value, -0.
constexpr double kThird = 1.0 / 3.0;
constexpr double kTiny = kThird * 1e-9;

[[nodiscard]] MotorPart a_motor() {
    MotorPart m;
    m.block.kv = 960.0;
    m.block.resistance = 0.1;
    m.block.no_load_current = 0.5;
    m.block.no_load_voltage = 10.0;
    m.block.current_max = 20.0;
    m.block.pole_pairs = 7.0;
    m.block.inductance = 0.0;
    m.block.rotor_inertia = 1e-05;
    m.block.stator = "2312";
    m.block.mass = 0.055;
    m.block.shape = BlockShape{PartShape::cylinder, {0.014, 0.025, 0.0}, {}};
    m.sources = PartSources{
        {"kv", SourceRef{"retail_phantom3_parts", Grade::retailer, ""}},
        {"resistance", SourceRef{"", Grade::assumed, "typical of a 2312 motor"}},
        {"no_load_current", SourceRef{"", Grade::assumed, "typical"}},
        {"no_load_voltage", SourceRef{"", Grade::assumed, "typical"}},
        {"current_max", SourceRef{"", Grade::assumed, "typical"}},
        {"pole_pairs", SourceRef{"", Grade::assumed, "12N14P"}},
        {"inductance", SourceRef{"", Grade::assumed, "dropped"}},
        {"rotor_inertia", SourceRef{"", Grade::assumed, "typical"}},
        {"mass", SourceRef{"retail_phantom3_parts", Grade::retailer, "55 g"}},
        {"shape", SourceRef{"", Grade::assumed, "a 28 mm bell"}},
    };
    return m;
}

// a_motor()'s text, the format pinned.
constexpr std::string_view kMotorText = R"(part_version: 1
motor:
  kv: 960
  resistance: 0.1
  no_load_current: 0.5
  no_load_voltage: 10
  current_max: 20
  pole_pairs: 7
  inductance: 0
  rotor_inertia: 1e-05
  stator: "2312"
  mass: 0.055
  shape: {kind: cylinder, size: [0.014, 0.025, 0]}
sources:
  kv: {source: "retail_phantom3_parts", grade: retailer}
  resistance: {grade: assumed, note: "typical of a 2312 motor"}
  no_load_current: {grade: assumed, note: "typical"}
  no_load_voltage: {grade: assumed, note: "typical"}
  current_max: {grade: assumed, note: "typical"}
  pole_pairs: {grade: assumed, note: "12N14P"}
  inductance: {grade: assumed, note: "dropped"}
  rotor_inertia: {grade: assumed, note: "typical"}
  mass: {source: "retail_phantom3_parts", grade: retailer, note: "55 g"}
  shape: {grade: assumed, note: "a 28 mm bell"}
)";

[[nodiscard]] PropPart a_prop() {
    PropPart p;
    p.block.diameter = 0.254;
    p.block.pitch = 0.127;
    p.block.blades = 2;
    p.block.inertia = 0.0;
    p.block.ct_static = 0.1105;
    p.block.cq_static = kThird * 0.01;
    p.block.ct_table = {{0.0, 0.1105}, {0.05, 0.1}, {0.1, kThird * 0.3}};
    p.block.cq_table = {{0.0, 0.0071}, {0.1, kTiny}};
    p.block.mass = 0.013;
    p.blade = BladeGeometry{{0.15, 0.5, 1.0}, {0.12, 0.18, 0.06}, {38.1, 20.0, -0.0}, 0.1, "assumed: a thin plate"};
    p.sources = cite({"diameter", "pitch", "blades", "inertia", "ct_static", "cq_static", "ct_table", "cq_table",
                      "mass", "blade.r_over_r", "blade.c_over_r", "blade.beta_deg", "blade.hub_cutout",
                      "blade.airfoil"});
    return p;
}

[[nodiscard]] EscPart an_esc() {
    EscPart e;
    e.block.current_continuous = 45.0;
    e.block.current_burst = 55.0;
    e.block.current_total = 0.0;
    e.block.channels = 4;
    e.block.on_resistance = 0.0015;
    e.block.braking = true;
    e.block.protocol = "DShot600";
    e.block.mass = 0.012;
    e.sources = cite({"current_continuous", "current_burst", "current_total", "channels", "on_resistance",
                      "braking", "mass"});
    return e;
}

[[nodiscard]] BatteryPart a_battery() {
    BatteryPart b;
    b.block.cells_series = 4;
    b.block.cells_parallel = 1;
    b.block.cell_capacity = 4.48;
    b.block.cell_resistance = 0.004;
    b.block.cell_voltage_nominal = 3.8;
    b.block.cell_voltage_full = 4.35;
    b.block.cell_voltage_cutoff = 3.0;
    b.block.ocv_table = {{0.0, 3.0}, {0.5, 3.8}, {1.0, 4.35}};
    b.block.c_rating = 8.0;
    b.block.polarization_resistance = 0.0;
    b.block.polarization_capacitance = 0.0;
    b.block.mass = 0.365;
    b.block.shape = BlockShape{PartShape::tensor, {}, {{1e-4, 0.0, 0.0}, {0.0, 2e-4, 0.0}, {0.0, 0.0, kThird}}};
    b.sources = cite({"cells_series", "cells_parallel", "cell_capacity", "cell_resistance", "cell_voltage_nominal",
                      "cell_voltage_full", "cell_voltage_cutoff", "ocv_table", "c_rating", "polarization_resistance",
                      "polarization_capacitance", "mass", "shape"});
    return b;
}

void expect_same_table(const std::vector<std::array<double, 2>>& a, const std::vector<std::array<double, 2>>& b,
                       std::string_view what) {
    ASSERT_EQ(a.size(), b.size()) << what;
    for (std::size_t i = 0; i < a.size(); ++i) {
        EXPECT_EQ(bits(a[i][0]), bits(b[i][0])) << what << "[" << i << "][0]";
        EXPECT_EQ(bits(a[i][1]), bits(b[i][1])) << what << "[" << i << "][1]";
    }
}

void expect_same_shape(const std::optional<BlockShape>& a, const std::optional<BlockShape>& b) {
    ASSERT_EQ(a.has_value(), b.has_value());
    if (!a) return;
    EXPECT_EQ(a->shape, b->shape);
    for (int i = 0; i < 3; ++i) {
        EXPECT_EQ(bits(a->size[i]), bits(b->size[i])) << "size " << i;
        for (int j = 0; j < 3; ++j) EXPECT_EQ(bits(a->tensor[i][j]), bits(b->tensor[i][j])) << i << j;
    }
}

void expect_same_sources(const PartSources& a, const PartSources& b) {
    ASSERT_EQ(a.size(), b.size());
    for (std::size_t i = 0; i < a.size(); ++i) {
        EXPECT_EQ(a[i].field, b[i].field) << i;
        EXPECT_EQ(a[i].ref.source, b[i].ref.source) << a[i].field;
        EXPECT_EQ(a[i].ref.grade, b[i].ref.grade) << a[i].field;
        EXPECT_EQ(a[i].ref.note, b[i].ref.note) << a[i].field;
    }
}

[[nodiscard]] SourceRegistry a_registry() {
    const spade::Result<SourceRegistry> r = spade::scene::sources_from_yaml(R"(sources_version: 1
sources:
  - {id: uiuc_propdb, citation: "UIUC", retrieved: 2026-10-21, license: facts_cite, license_evidence: "facts"}
  - {id: retail_phantom3_parts, citation: "shops", retrieved: 2026-10-21, license: facts_cite, license_evidence: "facts"}
  - {id: apc_data_files, citation: "APC", retrieved: 2026-10-21, license: not_usable, license_evidence: "prohibited"}
)");
    EXPECT_TRUE(r.has_value()) << r.error().context;
    return r.has_value() ? *r : SourceRegistry{};
}

void write_text(const fs::path& path, std::string_view text) {
    std::ofstream out(path, std::ios::binary);
    out << text;
}

}  // namespace

TEST(PartFile, KindsHaveOneSpellingAndOneSuffix) {
    EXPECT_EQ(spade::scene::part_kind_name(PartKind::motor), "motor");
    EXPECT_EQ(spade::scene::part_kind_name(PartKind::prop), "prop");
    EXPECT_EQ(spade::scene::part_kind_name(PartKind::esc), "esc");
    EXPECT_EQ(spade::scene::part_kind_name(PartKind::battery), "battery");
    EXPECT_EQ(spade::scene::part_suffix(PartKind::motor), ".motor.yaml");
    EXPECT_EQ(spade::scene::part_suffix(PartKind::prop), ".prop.yaml");
    EXPECT_EQ(spade::scene::part_suffix(PartKind::esc), ".esc.yaml");
    EXPECT_EQ(spade::scene::part_suffix(PartKind::battery), ".battery.yaml");
}

TEST(PartFile, TheMotorTextIsCanonical) {
    const spade::Result<std::string> text = spade::scene::motor_to_yaml(a_motor());
    ASSERT_TRUE(text.has_value()) << text.error().context;
    EXPECT_EQ(*text, kMotorText);
}

TEST(PartFile, PartFilesRoundTripBitForBit) {
    {
        const MotorPart m = a_motor();
        const spade::Result<std::string> text = spade::scene::motor_to_yaml(m);
        ASSERT_TRUE(text.has_value()) << text.error().context;
        const spade::Result<MotorPart> back = spade::scene::motor_from_yaml(*text);
        ASSERT_TRUE(back.has_value()) << back.error().context;
        EXPECT_EQ(bits(back->block.kv), bits(m.block.kv));
        EXPECT_EQ(bits(back->block.resistance), bits(m.block.resistance));
        EXPECT_EQ(bits(back->block.rotor_inertia), bits(m.block.rotor_inertia));
        EXPECT_EQ(bits(back->block.mass), bits(m.block.mass));
        EXPECT_EQ(back->block.stator, "2312");
        expect_same_shape(m.block.shape, back->block.shape);
        expect_same_sources(m.sources, back->sources);
        const spade::Result<std::string> again = spade::scene::motor_to_yaml(*back);
        ASSERT_TRUE(again.has_value()) << again.error().context;
        EXPECT_EQ(*again, *text);
    }
    {
        const PropPart p = a_prop();
        const spade::Result<std::string> text = spade::scene::prop_to_yaml(p);
        ASSERT_TRUE(text.has_value()) << text.error().context;
        const spade::Result<PropPart> back = spade::scene::prop_from_yaml(*text);
        ASSERT_TRUE(back.has_value()) << back.error().context;
        EXPECT_EQ(bits(back->block.diameter), bits(p.block.diameter));
        EXPECT_EQ(bits(back->block.cq_static), bits(p.block.cq_static));
        EXPECT_EQ(back->block.blades, 2u);
        expect_same_table(p.block.ct_table, back->block.ct_table, "ct_table");
        expect_same_table(p.block.cq_table, back->block.cq_table, "cq_table");
        ASSERT_TRUE(back->blade.has_value());
        ASSERT_EQ(back->blade->beta_deg.size(), 3u);
        EXPECT_EQ(bits(back->blade->beta_deg[2]), bits(-0.0)) << "-0 survives";
        EXPECT_EQ(bits(back->blade->hub_cutout), bits(0.1));
        EXPECT_EQ(back->blade->airfoil, "assumed: a thin plate");
        EXPECT_FALSE(back->block.shape.has_value());
        expect_same_sources(p.sources, back->sources);
        const spade::Result<std::string> again = spade::scene::prop_to_yaml(*back);
        ASSERT_TRUE(again.has_value()) << again.error().context;
        EXPECT_EQ(*again, *text);
    }
    {
        const EscPart e = an_esc();
        const spade::Result<std::string> text = spade::scene::esc_to_yaml(e);
        ASSERT_TRUE(text.has_value()) << text.error().context;
        const spade::Result<EscPart> back = spade::scene::esc_from_yaml(*text);
        ASSERT_TRUE(back.has_value()) << back.error().context;
        EXPECT_EQ(back->block.channels, 4u);
        EXPECT_TRUE(back->block.braking);
        EXPECT_EQ(back->block.protocol, "DShot600");
        EXPECT_EQ(bits(back->block.on_resistance), bits(0.0015));
        expect_same_sources(e.sources, back->sources);
        const spade::Result<std::string> again = spade::scene::esc_to_yaml(*back);
        ASSERT_TRUE(again.has_value()) << again.error().context;
        EXPECT_EQ(*again, *text);
    }
    {
        const BatteryPart b = a_battery();
        const spade::Result<std::string> text = spade::scene::battery_to_yaml(b);
        ASSERT_TRUE(text.has_value()) << text.error().context;
        const spade::Result<BatteryPart> back = spade::scene::battery_from_yaml(*text);
        ASSERT_TRUE(back.has_value()) << back.error().context;
        EXPECT_EQ(back->block.cells_series, 4u);
        EXPECT_EQ(bits(back->block.cell_capacity), bits(4.48));
        expect_same_table(b.block.ocv_table, back->block.ocv_table, "ocv_table");
        expect_same_shape(b.block.shape, back->block.shape);
        expect_same_sources(b.sources, back->sources);
        const spade::Result<std::string> again = spade::scene::battery_to_yaml(*back);
        ASSERT_TRUE(again.has_value()) << again.error().context;
        EXPECT_EQ(*again, *text);
    }
}

TEST(PartFile, CitedFieldsAreTheBlocksKeysLessItsIdentityStrings) {
    const std::vector<std::string> motor = spade::scene::cited_fields(PartFile{a_motor()});
    EXPECT_EQ(motor, (std::vector<std::string>{"kv", "resistance", "no_load_current", "no_load_voltage",
                                               "current_max", "pole_pairs", "inductance", "rotor_inertia", "mass",
                                               "shape"}));
    PropPart bare = a_prop();
    bare.blade.reset();
    EXPECT_EQ(spade::scene::cited_fields(PartFile{bare}),
              (std::vector<std::string>{"diameter", "pitch", "blades", "inertia", "ct_static", "cq_static",
                                        "ct_table", "cq_table", "mass"}));
    EXPECT_EQ(spade::scene::cited_fields(PartFile{a_prop()}).size(), 14u);
    EXPECT_EQ(spade::scene::cited_fields(PartFile{an_esc()}),
              (std::vector<std::string>{"current_continuous", "current_burst", "current_total", "channels",
                                        "on_resistance", "braking", "mass"}));
    EXPECT_EQ(spade::scene::cited_fields(PartFile{a_battery()}).size(), 13u);
}

TEST(PartFile, AFieldWithoutASourceIsRefusedByPath) {
    const std::string text(kMotorText);
    expect_refused(spade::scene::motor_from_yaml(with(text, "  kv: {source: \"retail_phantom3_parts\", grade: retailer}\n", "")),
                   Code::invalid_argument, {"motor.kv", "no citation", "sources"});
    // A citation of a field the part does not have.
    expect_refused(spade::scene::motor_from_yaml(
                       with(text, "  shape: {kind: cylinder, size: [0.014, 0.025, 0]}\n", "")),
                   Code::invalid_argument, {"sources.shape", "the motor has no shape"});
    expect_refused(spade::scene::motor_from_yaml(with(text, "  mass: {source:", "  thrust: {grade: assumed, "
                                                                                 "note: \"x\"}\n  mass: {source:")),
                   Code::invalid_argument, {"sources.thrust"});
    // The writer refuses the same, so what it writes reads back.
    MotorPart missing = a_motor();
    missing.sources.erase(missing.sources.begin() + 3);
    expect_refused(spade::scene::motor_to_yaml(missing), Code::invalid_argument, {"motor.no_load_voltage", "no citation"});
    PropPart no_airfoil = a_prop();
    no_airfoil.sources.pop_back();
    expect_refused(spade::scene::prop_to_yaml(no_airfoil), Code::invalid_argument, {"prop.blade.airfoil", "no citation"});
}

TEST(PartFile, ACitationIsGivenOrItsKeyLeftOut) {
    const std::string text(kMotorText);
    expect_refused(spade::scene::motor_from_yaml(with(text, "note: \"typical of a 2312 motor\"", "note: \"\"")),
                   Code::invalid_argument, {"sources.resistance.note", "leave the key out"});
    expect_refused(spade::scene::motor_from_yaml(with(text, "source: \"retail_phantom3_parts\", grade: retailer}",
                                                      "source: \"\", grade: retailer}")),
                   Code::invalid_argument, {"sources.kv.source", "leave the key out"});
    expect_refused(spade::scene::motor_from_yaml(with(text, "grade: retailer}", "grade: guessed}")),
                   Code::invalid_argument, {"sources.kv.grade", "'guessed'", "measured"});
    expect_refused(spade::scene::motor_from_yaml(with(text, "grade: retailer}", "grade: retailer, page: 4}")),
                   Code::invalid_argument, {"unknown key 'page'", "sources.kv", "part schema v1"});
}

TEST(PartFile, UnknownKeysAreRefusedBlamedOnTheVersion) {
    const std::string text(kMotorText);
    expect_refused(spade::scene::motor_from_yaml(with(text, "  kv: 960\n", "  kv: 960\n  kv_max: 1000\n")),
                   Code::invalid_argument, {"unknown key 'kv_max'", "motor", "part schema v1"});
    expect_refused(spade::scene::motor_from_yaml(with(text, "part_version: 1", "part_version: 2")),
                   Code::schema_mismatch, {"part file is schema version 2", "reads version 1 only"});
    // The version first: a later version's keys are not reported as unknown.
    expect_refused(spade::scene::motor_from_yaml(with(text, "part_version: 1", "part_version: 2\nmaker: \"x\"")),
                   Code::schema_mismatch, {"schema version 2"});
    expect_refused(spade::scene::motor_from_yaml(with(text, "  resistance: 0.1\n", "")), Code::invalid_argument,
                   {"missing required key 'resistance'", "motor"});
    // A file of another kind.
    expect_refused(spade::scene::prop_from_yaml(text), Code::invalid_argument, {"missing required key 'prop'"});
    expect_refused(spade::scene::motor_from_yaml(with(text, "  kv: 960\n", "  kv: inf\n")), Code::invalid_argument,
                   {"motor.kv", "not finite"});
}

TEST(PartFile, ShapesAndBladesAreChecked) {
    const std::string text(kMotorText);
    expect_refused(spade::scene::motor_from_yaml(
                       with(text, "{kind: cylinder, size: [0.014, 0.025, 0]}", "{kind: point}")),
                   Code::invalid_argument, {"motor.shape", "point", "leave the key out"});
    expect_refused(spade::scene::motor_from_yaml(
                       with(text, "{kind: cylinder, size: [0.014, 0.025, 0]}", "{kind: cone, size: [1, 1, 1]}")),
                   Code::invalid_argument, {"motor.shape.kind", "'cone'", "sphere", "tensor"});
    expect_refused(spade::scene::motor_from_yaml(
                       with(text, "{kind: cylinder, size: [0.014, 0.025, 0]}", "{kind: cylinder, size: [1, 1]}")),
                   Code::invalid_argument, {"motor.shape.size"});

    const auto refused_blade = [](void (*edit)(BladeGeometry&), std::initializer_list<std::string_view> needles) {
        PropPart p = a_prop();
        edit(*p.blade);
        expect_refused(spade::scene::prop_to_yaml(p), Code::invalid_argument, needles);
    };
    refused_blade([](BladeGeometry& b) { b.c_over_r.pop_back(); }, {"prop.blade", "3 stations", "c_over_r", "2"});
    refused_blade([](BladeGeometry& b) { b.r_over_r = {0.15}; b.c_over_r = {0.1}; b.beta_deg = {10.0}; },
                  {"prop.blade", "at least 2"});
    refused_blade([](BladeGeometry& b) { b.r_over_r[1] = 0.15; }, {"prop.blade.r_over_r", "strictly increasing"});
    refused_blade([](BladeGeometry& b) { b.r_over_r[2] = 1.01; }, {"prop.blade.r_over_r", "(0, 1]"});
    refused_blade([](BladeGeometry& b) { b.c_over_r[0] = 0.0; }, {"prop.blade.c_over_r[0]", "above 0"});
    refused_blade([](BladeGeometry& b) { b.hub_cutout = 0.2; }, {"prop.blade.hub_cutout", "0.2", "0.15"});
    refused_blade([](BladeGeometry& b) { b.hub_cutout = -0.1; }, {"prop.blade.hub_cutout", "at least 0"});
    refused_blade([](BladeGeometry& b) { b.airfoil.clear(); }, {"prop.blade.airfoil", "must not be empty"});
}

TEST(PartFile, APartHasNoMount) {
    const spade::Result<std::string> text = spade::scene::esc_to_yaml(an_esc());
    ASSERT_TRUE(text.has_value()) << text.error().context;
    EXPECT_FALSE(contains(*text, "mount")) << *text;
    EscPart placed = an_esc();
    placed.block.mount.position = {0.0, 0.01, 0.0};
    expect_refused(spade::scene::esc_to_yaml(placed), Code::invalid_argument, {"esc.mount", "the drone's"});
    BatteryPart turned = a_battery();
    turned.block.mount.orientation = {0.0, 1.0, 0.0, 0.0};
    expect_refused(spade::scene::battery_to_yaml(turned), Code::invalid_argument, {"battery.mount", "the drone's"});
}

TEST(PartFile, TheRegistryDecidesEachCitation) {
    const SourceRegistry registry = a_registry();
    EXPECT_TRUE(spade::scene::check_part_sources(PartFile{a_motor()}, registry).has_value());
    MotorPart apc = a_motor();
    apc.sources[0].ref.source = "apc_data_files";
    expect_refused(spade::scene::check_part_sources(PartFile{apc}, registry), Code::invalid_argument,
                   {"motor.kv", "'apc_data_files'", "not_usable"});
    PropPart unlisted = a_prop();
    unlisted.sources[6].ref = SourceRef{"nasa_tm_2018", Grade::measured, ""};
    expect_refused(spade::scene::check_part_sources(PartFile{unlisted}, registry), Code::invalid_argument,
                   {"prop.ct_table", "'nasa_tm_2018'", "does not list"});
}

TEST(PartFile, LoadPartFileReadsByItsSuffix) {
    const spade::test::TempDir dir("part-file-load");
    const SourceRegistry registry = a_registry();
    const fs::path motor = dir.path() / "m2312.motor.yaml";
    write_text(motor, kMotorText);
    const spade::Result<PartFile> loaded = spade::scene::load_part_file(motor, registry);
    ASSERT_TRUE(loaded.has_value()) << loaded.error().context;
    ASSERT_TRUE(std::holds_alternative<MotorPart>(*loaded));
    EXPECT_EQ(std::get<MotorPart>(*loaded).block.kv, 960.0);

    const fs::path wrong = dir.path() / "m2312.prop.yaml";
    write_text(wrong, kMotorText);
    expect_refused(spade::scene::load_part_file(wrong, registry), Code::invalid_argument,
                   {spade::path_text(wrong), "missing required key 'prop'"});

    const fs::path other = dir.path() / "m2312.yaml";
    write_text(other, kMotorText);
    expect_refused(spade::scene::load_part_file(other, registry), Code::invalid_argument,
                   {spade::path_text(other), ".motor.yaml", ".battery.yaml"});

    const fs::path cites_apc = dir.path() / "apc.motor.yaml";
    write_text(cites_apc, with(std::string(kMotorText), "kv: {source: \"retail_phantom3_parts\"",
                               "kv: {source: \"apc_data_files\""));
    expect_refused(spade::scene::load_part_file(cites_apc, registry), Code::invalid_argument,
                   {spade::path_text(cites_apc), "motor.kv", "'apc_data_files'"});

    expect_refused(spade::scene::load_part_file(dir.path() / "absent.motor.yaml", registry), Code::io_error,
                   {"absent.motor.yaml"});
}
