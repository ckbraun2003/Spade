// The default library's part entries (Task L4 of docs/design/core/plans/
// 2026-10-21-default-drone-library-plan.md): the 22 part files under
// assets/library/parts/ -- five propellers, six motors, six ESCs (one per
// drone) and five batteries -- and their generator.
//
// What is pinned here:
//   - THE FILES ARE THE GENERATOR'S: each committed assets/library/parts/
//     <id>.<kind>.yaml is part_to_yaml() of the PartFile built here, byte for
//     byte. With SPADE_LIBRARY_WRITE set the same test WRITES them into the
//     source tree instead, and passes, saying so (test_library.cpp's
//     pattern). They are only ever written that way, never by hand.
//   - EVERY PART IS IN THE INDEX: library.yaml has an entry whose file is the
//     part's, in Interface's category for its kind (INT-14: motors, props,
//     ESCs, batteries), with its thumbnail (N14b), and it loads with
//     load_library_part() against the library's sources.
//   - THE PROPS ARE THE HEADER'S: each prop file's static point, tables and
//     blade stations are tests/library_props.gen.hpp's (L4a), bit for bit,
//     and their citations carry the header's grade note verbatim. The data
//     below copies them from kProps; it never computes one.
//   - THE GRADES ARE AS RULED: a value the plan does not give is `assumed`,
//     its basis in the note (Physics' rulings on L4's part numbers,
//     2026-10-22). The competition parts are `assumed`, since Kat's fit was
//     withdrawn (the lead, 2026-10-22), and no `fitted` grade cites it.
// The values, grades and notes are the plan's and its Rulings', and, for what
// neither gives, Core's L4 draft as Physics reviewed it. Notes are ASCII.
//
// LINE ENDINGS. part_to_yaml() writes LF, and the comparison reads the file
// in binary, so assets/.gitattributes gives the four part suffixes `text
// eol=lf`, as it does the object files.

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <bit>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <system_error>
#include <type_traits>
#include <utility>
#include <variant>
#include <vector>

#include <glm/vec3.hpp>

#include "core/error.hpp"
#include "core/path_text.hpp"
#include "library_props.gen.hpp"
#include "scene/library.hpp"
#include "scene/library_sources.hpp"
#include "scene/part_file.hpp"
#include "vehicles/airframe_compile.hpp"
#include "vehicles/composite_inertia.hpp"

namespace {

namespace fs = std::filesystem;
namespace sc = spade::scene;
using spade::scene::BatteryPart;
using spade::scene::BladeGeometry;
using spade::scene::EscPart;
using spade::scene::FieldSource;
using spade::scene::Grade;
using spade::scene::Library;
using spade::scene::LibraryEntry;
using spade::scene::LibraryKind;
using spade::scene::MotorPart;
using spade::scene::PartFile;
using spade::scene::PartKind;
using spade::scene::PartSources;
using spade::scene::PropPart;
using spade::scene::SourceRef;
using spade::scene::SourceRegistry;
using spade::test::library_props::kProps;
using spade::test::library_props::Prop;
using spade::vehicles::BlockShape;
using spade::vehicles::PartShape;

template <class T>
[[nodiscard]] testing::AssertionResult IsOk(const char* expr, const spade::Result<T>& r) {
    if (r) return testing::AssertionSuccess();
    return testing::AssertionFailure() << expr << " failed: [" << static_cast<int>(r.error().code) << "] "
                                       << r.error().context;
}
#define ASSERT_OK(expr) ASSERT_PRED_FORMAT1(IsOk, expr)
#define EXPECT_OK(expr) EXPECT_PRED_FORMAT1(IsOk, expr)

[[nodiscard]] std::uint64_t bits(double v) { return std::bit_cast<std::uint64_t>(v); }

[[nodiscard]] bool read_bytes(const fs::path& path, std::string& out) {
    std::ifstream in(path, std::ios::binary);
    if (!in) return false;
    out.assign(std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>());
    return true;
}

// The committed library, in the SOURCE tree: the generator writes here.
[[nodiscard]] fs::path library_dir() {
    return fs::path(SPADE_ASSETS_DIR) / "library";
}

// SPADE_LIBRARY_WRITE set: write the library's part files instead of
// comparing them. A read-only null check before any thread starts, as
// env_gate_is_set() in test_fp32_math.cpp.
[[nodiscard]] bool library_write_asked() {
#if defined(_MSC_VER)
#pragma warning(push)
#pragma warning(disable : 4996)
#endif
    return std::getenv("SPADE_LIBRARY_WRITE") != nullptr;
#if defined(_MSC_VER)
#pragma warning(pop)
#endif
}

// The line holding byte `at` of `text` (or "<end of file>").
[[nodiscard]] std::string line_at(const std::string& text, std::size_t at) {
    if (at >= text.size()) return "<end of file>";
    std::size_t from = 0;
    if (at > 0) {
        const std::size_t newline = text.rfind('\n', at - 1);
        if (newline != std::string::npos) from = newline + 1;
    }
    const std::size_t end = text.find('\n', at);
    return text.substr(from, (end == std::string::npos ? text.size() : end) - from);
}

// Why the committed bytes are not the generator's, and what to do about it.
[[nodiscard]] std::string mismatch(const fs::path& path, const std::string& committed, const std::string& generated) {
    std::size_t i = 0;
    while (i < committed.size() && i < generated.size() && committed[i] == generated[i]) ++i;
    std::size_t line = 1;
    for (std::size_t j = 0; j < i; ++j) {
        if (committed[j] == '\n') ++line;
    }
    std::string out = spade::path_text(path) + " (" + std::to_string(committed.size()) +
                      " bytes) is not the generator's text (" + std::to_string(generated.size()) +
                      " bytes): first difference at byte " + std::to_string(i) + ", line " + std::to_string(line) +
                      ".\n  committed: " + line_at(committed, i) + "\n  generated: " + line_at(generated, i) + "\n";
    if (committed.find('\r') != std::string::npos) {
        out += "The committed file holds CR bytes: a checkout converted its line endings. assets/.gitattributes "
               "must give *.motor.yaml, *.prop.yaml, *.esc.yaml and *.battery.yaml `text eol=lf` (check with `git "
               "check-attr -a` on the path).\n";
    }
    out += "The file is generated from this test's part table; never edit it by hand. To move it: (1) name the "
           "cause (TD-1): the table, tests/library_props.gen.hpp or the part-file format; (2) regenerate: set "
           "SPADE_LIBRARY_WRITE=1 and run ctest -R LibraryParts.TheCommittedFilesAreTheGeneratorsText; (3) it is "
           "final only once a fresh (-NoSeed) Docker gcc leg reproduces it (TD-12).";
    return out;
}

// ---------------------------------------------------------------------------
// THE PART TABLE (L4): every part's values and citations. Each PART_FIELD is
// one row of the draft's table (field, value, grade, source, note), in the
// block's key order; the identity strings (stator, protocol) take no
// citation. Shapes are part-local, y up: a motor is a cylinder along +Y
// (size.x the radius, size.y the length), the rest boxes of full extents. A
// prop has no shape: the thumbnail draws its disc.
// ---------------------------------------------------------------------------

// Physics' bell estimate, the 2207s' (the lead's TD-1 word, 2026-10-22); basic_quad's default moves with it
constexpr double kRotorInertia2306 = 2.7e-6;

// The sources cited (assets/library/sources.yaml).
constexpr std::string_view kNasa = "nasa_ames_2016";
constexpr std::string_view kDji = "dji_phantom3";
constexpr std::string_view kPhantomParts = "retail_phantom3_parts";
constexpr std::string_view kIrisPlus = "retail_iris_plus";
constexpr std::string_view kUiuc = "uiuc_propdb";
constexpr std::string_view kCfDatasheet = "bitcraze_cf21_datasheet";
constexpr std::string_view kCfThrust = "bitcraze_pwm_thrust";
constexpr std::string_view kCfMotor = "retail_cf_motor";
constexpr std::string_view kFpvParts = "retail_fpv_parts";
constexpr std::string_view kKatDatapack = "kat_datapack";

// The notes many rows share.
constexpr std::string_view kWithdrawn =
    "Kat's fit (datapack 7.2) was withdrawn on 2026-10-21 for APC-derived inputs; re-run not scheduled (Kat, "
    "2026-10-22)";
constexpr std::string_view kEstimatedInertia = "0: the engine estimates m D^2/12";
constexpr std::string_view kAirfoil = "unknown for every prop (P-b); Physics picks the polar";
constexpr std::string_view kNoLoadVoltage = "the usual no-load test voltage";
constexpr std::string_view kPolePairs = "12N14P";
constexpr std::string_view kInductance = "not modelled";
constexpr std::string_view kOneParallel = "one cell per series step";
constexpr std::string_view kLipoNominal = "LiPo nominal";
constexpr std::string_view kLipoFull = "LiPo full";
constexpr std::string_view kLipoCutoff = "a usual LiPo cutoff";
constexpr std::string_view kLipoOcv = "the chain's current LiPo shape (ruling 8)";
constexpr std::string_view kPolarization = "not modelled (0)";
constexpr std::string_view kNoTotal = "no board total (0)";

[[nodiscard]] SourceRef cite(Grade grade, std::string_view source, std::string_view note = {}) {
    return SourceRef{std::string(source), grade, std::string(note)};
}

[[nodiscard]] SourceRef assumed(std::string_view note) { return SourceRef{"", Grade::assumed, std::string(note)}; }

// A competition value's basis, then the withdrawal (the lead, 2026-10-22).
[[nodiscard]] std::string withdrawn(std::string_view basis) {
    return std::string(basis) + "; " + std::string(kWithdrawn);
}

template <class T>
void set_field(T& slot, std::type_identity_t<T> value, PartSources& sources, std::string_view field, SourceRef ref) {
    slot = std::move(value);
    sources.push_back(FieldSource{std::string(field), std::move(ref)});
}

// The block's field `name` set and cited under its own name, so a citation
// can never name another field than the one it was written beside.
#define PART_FIELD(part, name, value, ref) set_field((part).block.name, (value), (part).sources, #name, (ref))

[[nodiscard]] BlockShape cylinder(double radius, double length) {
    BlockShape s;
    s.shape = PartShape::cylinder;
    s.size = glm::dvec3(radius, length, 0.0);
    return s;
}

[[nodiscard]] BlockShape box(double x, double y, double z) {
    BlockShape s;
    s.shape = PartShape::box;
    s.size = glm::dvec3(x, y, z);
    return s;
}

[[nodiscard]] std::vector<std::array<double, 2>> lipo_ocv() {
    return {{0.0, 3.3}, {0.2, 3.6}, {0.5, 3.75}, {0.8, 3.95}, {1.0, 4.2}};
}

// The Phantom's LiHV pack: the LiPo shape lifted by 0.15 x SoC (Physics, Q6).
[[nodiscard]] std::vector<std::array<double, 2>> lihv_ocv() {
    return {{0.0, 3.3}, {0.2, 3.63}, {0.5, 3.825}, {0.8, 4.07}, {1.0, 4.35}};
}

// -- Props ------------------------------------------------------------------

// The header's prop with this id.
[[nodiscard]] const Prop& header_prop(std::string_view id) {
    const auto it = std::find_if(kProps.begin(), kProps.end(), [&](const Prop& p) { return p.id == id; });
    if (it == kProps.end()) {
        ADD_FAILURE() << "tests/library_props.gen.hpp has no prop '" << id << "'";
        return kProps.front();
    }
    return *it;
}

[[nodiscard]] std::vector<std::array<double, 2>> table_of(std::span<const std::array<double, 2>> table) {
    return std::vector<std::array<double, 2>>(table.begin(), table.end());
}

// The static point and the tables, copied from the header, each cited to
// UIUC with the header's grade note verbatim.
void set_coefficients(PropPart& part, const Prop& p, Grade static_grade, Grade table_grade) {
    PART_FIELD(part, ct_static, p.ct_static, cite(static_grade, kUiuc, p.note));
    PART_FIELD(part, cq_static, p.cq_static, cite(static_grade, kUiuc, p.note));
    PART_FIELD(part, ct_table, table_of(p.ct_table), cite(table_grade, kUiuc, p.note));
    PART_FIELD(part, cq_table, table_of(p.cq_table), cite(table_grade, kUiuc, p.note));
}

// The blade, its stations and hub cut-out copied from the header; the airfoil
// is unknown (Physics, Q8).
void set_blade(PropPart& part, const Prop& p, const SourceRef& stations, const SourceRef& hub) {
    BladeGeometry b;
    b.r_over_r.assign(p.r_over_r.begin(), p.r_over_r.end());
    b.c_over_r.assign(p.c_over_r.begin(), p.c_over_r.end());
    b.beta_deg.assign(p.beta_deg.begin(), p.beta_deg.end());
    b.hub_cutout = p.hub_cutout;
    b.airfoil = "unknown";
    part.blade = std::move(b);
    for (const char* field : {"blade.r_over_r", "blade.c_over_r", "blade.beta_deg"}) {
        part.sources.push_back(FieldSource{field, stations});
    }
    part.sources.push_back(FieldSource{"blade.hub_cutout", hub});
    part.sources.push_back(FieldSource{"blade.airfoil", assumed(kAirfoil)});
}

[[nodiscard]] std::uint32_t blades_of(const Prop& p) { return static_cast<std::uint32_t>(p.blades); }

[[nodiscard]] PropPart uiuc_apce_10x5() {
    const Prop& p = header_prop("uiuc_apce_10x5");
    PropPart part;
    PART_FIELD(part, diameter, 0.2388, cite(Grade::measured, kNasa, "9.4 in"));
    PART_FIELD(part, pitch, 0.127,
               cite(Grade::retailer, kPhantomParts, "5.0 in, as UIUC's 10x5; the 9450's listings give 4.5 or 5.0 in"));
    PART_FIELD(part, blades, blades_of(p), cite(Grade::measured, kUiuc));
    PART_FIELD(part, inertia, 0.0, assumed(kEstimatedInertia));
    set_coefficients(part, p, Grade::derived, Grade::derived);
    PART_FIELD(part, mass, 0.013, assumed("typical 9-10 in moulded prop"));
    set_blade(part, p, cite(Grade::derived, kUiuc, "UIUC geometry at 9.4 in"),
              cite(Grade::derived, kUiuc, "the first station"));
    return part;
}

[[nodiscard]] PropPart uiuc_apce_9x4_5() {
    const Prop& p = header_prop("uiuc_apce_9x4.5");
    PropPart part;
    PART_FIELD(part, diameter, 0.2438, cite(Grade::measured, kNasa, "9.6 in"));
    PART_FIELD(part, pitch, 0.1143, cite(Grade::retailer, kIrisPlus, "4.5 in, the 9.5 x 4.5 props"));
    PART_FIELD(part, blades, blades_of(p), cite(Grade::measured, kUiuc));
    PART_FIELD(part, inertia, 0.0, assumed(kEstimatedInertia));
    set_coefficients(part, p, Grade::derived, Grade::derived);
    PART_FIELD(part, mass, 0.012, assumed("typical 9.5 in plastic prop"));
    set_blade(part, p, cite(Grade::derived, kUiuc, "UIUC geometry at 9.6 in"),
              cite(Grade::derived, kUiuc, "the first station"));
    return part;
}

[[nodiscard]] PropPart uiuc_mit_5x4() {
    const Prop& p = header_prop("uiuc_mit_5x4");
    PropPart part;
    PART_FIELD(part, diameter, 0.127, cite(Grade::measured, kUiuc, "Micro Invent 5x4"));
    PART_FIELD(part, pitch, 0.1016, cite(Grade::measured, kUiuc, "its name, 5x4"));
    PART_FIELD(part, blades, blades_of(p), cite(Grade::measured, kUiuc, "3-Blade heading"));
    PART_FIELD(part, inertia, 0.0, assumed(kEstimatedInertia));
    set_coefficients(part, p, Grade::measured, Grade::measured);
    PART_FIELD(part, mass, 0.004, assumed("typical 5 in tri-blade, 3.5-4.5 g"));
    set_blade(part, p, cite(Grade::measured, kUiuc, "UIUC geometry, resampled"),
              cite(Grade::measured, kUiuc, "the first station"));
    return part;
}

// Static only: its tables are empty, so their grade is assumed (flat over J).
[[nodiscard]] PropPart uiuc_cfnq_45t1() {
    const Prop& p = header_prop("uiuc_cfnq_45t1");
    PropPart part;
    PART_FIELD(part, diameter, 0.04699,
               cite(Grade::measured, kUiuc, "1.85 in, UIUC's, as its coefficients were reduced; Luis & Le Ny give "
                                            "0.04626"));
    PART_FIELD(part, pitch, 0.0, assumed("not known; identity only"));
    PART_FIELD(part, blades, blades_of(p), cite(Grade::measured, kUiuc));
    PART_FIELD(part, inertia, 0.0, assumed(kEstimatedInertia));
    set_coefficients(part, p, Grade::measured, Grade::assumed);
    PART_FIELD(part, mass, 0.0002, assumed("47 mm moulded two-blade"));
    set_blade(part, p, cite(Grade::measured, kUiuc, "UIUC geometry"),
              cite(Grade::measured, kUiuc, "the first station"));
    return part;
}

// A stand-in (the lead, 2026-10-22): one grade, assumed, citing UIUC.
[[nodiscard]] PropPart triblade_6inch_scaled() {
    const Prop& p = header_prop("triblade_6inch_scaled");
    PropPart part;
    PART_FIELD(part, diameter, 0.1524, cite(Grade::retailer, kFpvParts, "HQProp DP 6x3.5x3"));
    PART_FIELD(part, pitch, 0.0889, cite(Grade::retailer, kFpvParts, "3.5 in"));
    PART_FIELD(part, blades, blades_of(p), cite(Grade::retailer, kFpvParts));
    PART_FIELD(part, inertia, 0.0, assumed(kEstimatedInertia));
    set_coefficients(part, p, Grade::assumed, Grade::assumed);
    PART_FIELD(part, mass, 0.0064, cite(Grade::retailer, kFpvParts, "~6.4 g"));
    const SourceRef stations = cite(Grade::assumed, kUiuc, "stand-in: mit_5x4's stations at 6 in");
    set_blade(part, p, stations, stations);
    return part;
}

// -- Motors -----------------------------------------------------------------

[[nodiscard]] MotorPart phantom3_2312_960kv() {
    MotorPart m;
    PART_FIELD(m, kv, 960.0, cite(Grade::retailer, kPhantomParts));
    PART_FIELD(m, resistance, 0.1, assumed("typical 2312 ~960 KV"));
    PART_FIELD(m, no_load_current, 0.5, assumed("typical 2312 ~960 KV, at 10 V"));
    PART_FIELD(m, no_load_voltage, 10.0, assumed(kNoLoadVoltage));
    PART_FIELD(m, current_max, 15.0, assumed("typical 2312 on 4S"));
    PART_FIELD(m, pole_pairs, 7.0, assumed(kPolePairs));
    PART_FIELD(m, inductance, 0.0, assumed(kInductance));
    PART_FIELD(m, rotor_inertia, 4e-6, assumed("22 g bell at 13.5 mm"));
    m.block.stator = "2312";
    PART_FIELD(m, mass, 0.056, assumed("typical 2312, 50-60 g"));
    PART_FIELD(m, shape, cylinder(0.014, 0.025), assumed("28 x 25 mm bell (23 mm stator, 12 mm tall)"));
    return m;
}

[[nodiscard]] MotorPart iris_920kv() {
    MotorPart m;
    PART_FIELD(m, kv, 920.0, cite(Grade::retailer, kIrisPlus));
    PART_FIELD(m, resistance, 0.12, assumed("typical 2212 920 KV"));
    PART_FIELD(m, no_load_current, 0.4, assumed("typical 2212 920 KV, at 10 V"));
    PART_FIELD(m, no_load_voltage, 10.0, assumed(kNoLoadVoltage));
    PART_FIELD(m, current_max, 15.0, assumed("typical 2212 on 3S"));
    PART_FIELD(m, pole_pairs, 7.0, assumed(kPolePairs));
    PART_FIELD(m, inductance, 0.0, assumed(kInductance));
    PART_FIELD(m, rotor_inertia, 3e-6, assumed("18 g bell at 13 mm"));
    m.block.stator = "2212";  // assumed: no listing names it (an identity string takes no citation)
    PART_FIELD(m, mass, 0.055, assumed("typical 2212 with leads"));
    PART_FIELD(m, shape, cylinder(0.014, 0.024), assumed("28 x 24 mm bell"));
    return m;
}

[[nodiscard]] MotorPart fpv_2207_1855kv() {
    MotorPart m;
    PART_FIELD(m, kv, 1855.0, cite(Grade::retailer, kFpvParts, "XING2 2207"));
    PART_FIELD(m, resistance, 0.05, cite(Grade::retailer, kFpvParts, "50 mohm"));
    PART_FIELD(m, no_load_current, 1.2, assumed("typical 2207 ~1800 KV, at 10 V"));
    PART_FIELD(m, no_load_voltage, 10.0, assumed(kNoLoadVoltage));
    PART_FIELD(m, current_max, 35.08,
               cite(Grade::retailer, kFpvParts, "the listing's peak; no continuous figure listed"));
    PART_FIELD(m, pole_pairs, 7.0, assumed(kPolePairs));
    PART_FIELD(m, inductance, 0.0, assumed(kInductance));
    PART_FIELD(m, rotor_inertia, 2.2e-6, assumed("14 g bell at 12.5 mm"));
    m.block.stator = "2207";
    PART_FIELD(m, mass, 0.0316, cite(Grade::retailer, kFpvParts, "31.6 g"));
    PART_FIELD(m, shape, cylinder(0.014, 0.02), assumed("28 x 20 mm bell (22 mm stator, 7 mm tall)"));
    return m;
}

// Brushed DC: the chain's equations are its own (Physics, Q4).
[[nodiscard]] MotorPart cf_7x16_coreless() {
    MotorPart m;
    PART_FIELD(m, kv, 14000.0, cite(Grade::retailer, kCfMotor));
    PART_FIELD(m, resistance, 1.6,
               cite(Grade::fitted, kCfThrust,
                    "fitted so full duty at 3.7 V gives about 24,300 rpm at 1.19 A against Bitcraze's top row, "
                    "23,882 rpm"));
    PART_FIELD(m, no_load_current, 0.06, assumed("typical 7x16 coreless, 50-100 mA"));
    PART_FIELD(m, no_load_voltage, 3.7, assumed("1S nominal"));
    PART_FIELD(m, current_max, 1.5, assumed("above its ~1.2 A full-throttle draw"));
    PART_FIELD(m, pole_pairs, 0.0, assumed("brushed DC: no inductance term (P-c)"));
    PART_FIELD(m, inductance, 0.0, assumed(kInductance));
    PART_FIELD(m, rotor_inertia, 6e-9, assumed("0.6 g cup and shaft at 3.2 mm"));
    m.block.stator = "7x16 coreless";
    PART_FIELD(m, mass, 0.0028, assumed("typical 7x16 coreless, 2.5-3.3 g"));
    PART_FIELD(m, shape, cylinder(0.0035, 0.016), cite(Grade::derived, kCfDatasheet, "the 7 x 16 mm can"));
    return m;
}

// basic_quad's default motor (engine/vehicles/basic_quad.cpp), its notes
// basic_quad_assumptions()'.
[[nodiscard]] MotorPart fpv_2306_1700kv() {
    MotorPart m;
    PART_FIELD(m, kv, 1700.0, cite(Grade::retailer, kFpvParts, "EMAX E3 2306"));
    PART_FIELD(m, resistance, 0.06, assumed("typical of a 2306 motor"));
    PART_FIELD(m, no_load_current, 1.0, assumed("typical of a 2306 motor"));
    PART_FIELD(m, no_load_voltage, 10.0, assumed(kNoLoadVoltage));
    PART_FIELD(m, current_max, 40.0, assumed("typical of a 2306 motor"));
    PART_FIELD(m, pole_pairs, 7.0, assumed(kPolePairs));
    PART_FIELD(m, inductance, 0.0, assumed(kInductance));
    PART_FIELD(m, rotor_inertia, kRotorInertia2306,
               assumed("the same bell estimate as the 2207s (Physics, 2026-10-22)"));
    m.block.stator = "2306";
    PART_FIELD(m, mass, 0.032, assumed("typical of a 2306 motor"));
    PART_FIELD(m, shape, cylinder(0.0145, 0.019), assumed("a 29 x 19 mm bell"));
    return m;
}

// Nominal datasheet values, all assumed: Kat's fit was withdrawn.
[[nodiscard]] MotorPart comp_2207_2450kv() {
    MotorPart m;
    PART_FIELD(m, kv, 2450.0, cite(Grade::assumed, kKatDatapack, withdrawn("Kat's candidate set")));
    PART_FIELD(m, resistance, 0.045, assumed(withdrawn("a typical 2207 2450 KV datasheet value")));
    PART_FIELD(m, no_load_current, 1.9, assumed("typical 2207 ~2500 KV, at 10 V"));
    PART_FIELD(m, no_load_voltage, 10.0, assumed(kNoLoadVoltage));
    PART_FIELD(m, current_max, 40.0, assumed("typical 2207 2450 KV"));
    PART_FIELD(m, pole_pairs, 7.0, assumed(kPolePairs));
    PART_FIELD(m, inductance, 0.0, assumed(kInductance));
    PART_FIELD(m, rotor_inertia, 2.2e-6, assumed("14 g bell at 12.5 mm"));
    m.block.stator = "2207";
    PART_FIELD(m, mass, 0.034, assumed("typical 2207 with leads"));
    PART_FIELD(m, shape, cylinder(0.014, 0.02), assumed("28 x 20 mm bell"));
    return m;
}

// -- ESCs, one per drone (the lead, 2026-10-22); the plan gives no ESC number

// The Phantom's and the Iris+'s four arm ESCs, written as one block.
[[nodiscard]] EscPart four_20a_escs(bool braking, std::string_view braking_note) {
    EscPart e;
    PART_FIELD(e, current_continuous, 20.0, assumed("four 20 A ESCs; covers 15 A"));
    PART_FIELD(e, current_burst, 25.0, assumed("four 20 A ESCs; covers 15 A"));
    PART_FIELD(e, current_total, 0.0, assumed(kNoTotal));
    PART_FIELD(e, channels, 4u, assumed("four boards as one block"));
    PART_FIELD(e, on_resistance, 0.006, assumed("typical 20 A ESC"));
    PART_FIELD(e, braking, braking, assumed(braking_note));
    e.block.protocol = "PWM";
    PART_FIELD(e, mass, 0.028, assumed("four x ~7 g"));
    PART_FIELD(e, shape, box(0.04, 0.008, 0.04), assumed("the four as one block"));
    return e;
}

[[nodiscard]] EscPart phantom3_4x20a() { return four_20a_escs(true, "synchronous-rectified drive"); }

[[nodiscard]] EscPart iris_4x20a() { return four_20a_escs(false, "2014-era ESC, no active braking"); }

// basic_quad's default ESC, its notes basic_quad_assumptions()', plus a
// board. Three drones' files are this one (fpv, basic, comp: 35.08 A, 40 A
// and 40 A covered).
[[nodiscard]] EscPart four_in_one_45a() {
    EscPart e;
    PART_FIELD(e, current_continuous, 45.0, assumed("a 45 A 4-in-1 ESC"));
    PART_FIELD(e, current_burst, 55.0, assumed("a 45 A 4-in-1 ESC"));
    PART_FIELD(e, current_total, 0.0, assumed(kNoTotal));
    PART_FIELD(e, channels, 4u, assumed("one 4-in-1 board"));
    PART_FIELD(e, on_resistance, 0.003, assumed("typical of a 4-in-1 ESC"));
    PART_FIELD(e, braking, true, assumed("active braking, as most FPV firmware runs"));
    e.block.protocol = "DShot600";
    PART_FIELD(e, mass, 0.012, assumed("a typical 4-in-1 ESC"));
    PART_FIELD(e, shape, box(0.036, 0.008, 0.036), assumed("a 30.5 mm-mount board; at least 8 mm (solids rule)"));
    return e;
}

// Mass 0: its FETs are on the flight board, in the remainder, and compile
// adds an ESC only above 0 (Physics, Q9).
[[nodiscard]] EscPart cf_brushed_4ch() {
    EscPart e;
    PART_FIELD(e, current_continuous, 2.0, assumed("the board's motor FETs; covers 1.5 A"));
    PART_FIELD(e, current_burst, 2.5, assumed("the board's motor FETs; covers 1.5 A"));
    PART_FIELD(e, current_total, 0.0, assumed(kNoTotal));
    PART_FIELD(e, channels, 4u, assumed("four drives, one board"));
    PART_FIELD(e, on_resistance, 0.05, assumed("small logic-level MOSFET"));
    PART_FIELD(e, braking, false, assumed("low-side brushed drive"));
    e.block.protocol = "brushed PWM";
    PART_FIELD(e, mass, 0.0, assumed("on the flight board: in the remainder"));
    PART_FIELD(e, shape, box(0.01, 0.002, 0.01), assumed("the FETs' patch (drawn only)"));
    return e;
}

// -- Batteries --------------------------------------------------------------

// A LiPo cell's voltages and OCV, assumed.
void set_lipo_cell(BatteryPart& b) {
    PART_FIELD(b, cell_voltage_nominal, 3.7, assumed(kLipoNominal));
    PART_FIELD(b, cell_voltage_full, 4.2, assumed(kLipoFull));
    PART_FIELD(b, cell_voltage_cutoff, 3.3, assumed(kLipoCutoff));
    PART_FIELD(b, ocv_table, lipo_ocv(), assumed(kLipoOcv));
}

void set_no_polarization(BatteryPart& b) {
    PART_FIELD(b, polarization_resistance, 0.0, assumed(kPolarization));
    PART_FIELD(b, polarization_capacitance, 0.0, assumed(kPolarization));
}

// DJI's LiHV pack (Physics, Q6).
[[nodiscard]] BatteryPart phantom3_4s_4480() {
    BatteryPart b;
    PART_FIELD(b, cells_series, 4u, cite(Grade::published, kDji));
    PART_FIELD(b, cells_parallel, 1u, assumed(kOneParallel));
    PART_FIELD(b, cell_capacity, 4.48, cite(Grade::published, kDji));
    PART_FIELD(b, cell_resistance, 0.008, assumed("typical 4.5 Ah low-rate cell"));
    PART_FIELD(b, cell_voltage_nominal, 3.8, cite(Grade::derived, kDji, "15.2 V / 4"));
    PART_FIELD(b, cell_voltage_full, 4.35, assumed("LiHV full"));
    PART_FIELD(b, cell_voltage_cutoff, 3.3, assumed(kLipoCutoff));
    PART_FIELD(b, ocv_table, lihv_ocv(), assumed("the LiPo shape lifted to LiHV's 4.35 V full"));
    PART_FIELD(b, c_rating, 15.0, assumed("camera-drone pack; 67 A covers 4 x 15 A"));
    set_no_polarization(b);
    PART_FIELD(b, mass, 0.365, cite(Grade::published, kDji));
    PART_FIELD(b, shape, box(0.12, 0.04, 0.05), assumed("cased pack, ~1.5 g/cm^3"));
    return b;
}

[[nodiscard]] BatteryPart iris_3s_5100() {
    BatteryPart b;
    PART_FIELD(b, cells_series, 3u, cite(Grade::retailer, kIrisPlus));
    PART_FIELD(b, cells_parallel, 1u, assumed(kOneParallel));
    PART_FIELD(b, cell_capacity, 5.1, cite(Grade::retailer, kIrisPlus));
    PART_FIELD(b, cell_resistance, 0.008, assumed("typical 5 Ah low-rate cell"));
    set_lipo_cell(b);
    PART_FIELD(b, c_rating, 8.0, cite(Grade::retailer, kIrisPlus, "8C"));
    set_no_polarization(b);
    PART_FIELD(b, mass, 0.38, assumed("typical 3S 5 Ah, 360-410 g"));
    PART_FIELD(b, shape, box(0.145, 0.03, 0.045), assumed("145 x 30 x 45 mm"));
    return b;
}

// basic_quad's default pack, its notes basic_quad_assumptions()'; also the
// basic quad's. No pack listing is registered, so assumed (the lead,
// 2026-10-22).
[[nodiscard]] BatteryPart fpv_6s_1300() {
    BatteryPart b;
    PART_FIELD(b, cells_series, 6u, assumed("6S; no pack listing cited"));
    PART_FIELD(b, cells_parallel, 1u, assumed(kOneParallel));
    PART_FIELD(b, cell_capacity, 1.3, assumed("1300 mAh"));
    PART_FIELD(b, cell_resistance, 0.004, assumed("typical of a 1300 mAh cell"));
    set_lipo_cell(b);
    PART_FIELD(b, c_rating, 75.0,
               assumed("typical of a 1300 mAh FPV pack; 97.5 A binds slightly at full throttle"));
    set_no_polarization(b);
    PART_FIELD(b, mass, 0.24, assumed("the plan's ~240 g; no listing cited"));
    PART_FIELD(b, shape, box(0.075, 0.03, 0.035), assumed("a 75 x 30 x 35 mm pack"));
    return b;
}

[[nodiscard]] BatteryPart cf_1s_250() {
    BatteryPart b;
    PART_FIELD(b, cells_series, 1u, cite(Grade::published, kCfDatasheet));
    PART_FIELD(b, cells_parallel, 1u, assumed(kOneParallel));
    PART_FIELD(b, cell_capacity, 0.25, cite(Grade::published, kCfDatasheet));
    PART_FIELD(b, cell_resistance, 0.08, assumed("typical 250 mAh cell"));
    set_lipo_cell(b);
    PART_FIELD(b, c_rating, 20.0, assumed("5 A covers ~4.8 A"));
    set_no_polarization(b);
    PART_FIELD(b, mass, 0.0072, assumed("typical 1S 250 mAh with lead"));
    PART_FIELD(b, shape, box(0.03, 0.007, 0.018), assumed("30 x 7 x 18 mm"));
    return b;
}

// Every field assumed: Kat's fit was withdrawn.
[[nodiscard]] BatteryPart comp_4s() {
    BatteryPart b;
    PART_FIELD(b, cells_series, 4u, cite(Grade::assumed, kKatDatapack, withdrawn("Kat's candidate set")));
    PART_FIELD(b, cells_parallel, 1u, assumed(kOneParallel));
    PART_FIELD(b, cell_capacity, 1.5, assumed("a 4S 1500 racing pack"));
    PART_FIELD(b, cell_resistance, 0.0056, assumed("fresh 1500 mAh racing cell"));
    set_lipo_cell(b);
    PART_FIELD(b, c_rating, 120.0, assumed("racing pack; 180 A covers 4 x 40 A"));
    set_no_polarization(b);
    PART_FIELD(b, mass, 0.19, assumed("typical 4S 1500"));
    PART_FIELD(b, shape, box(0.088, 0.033, 0.035), assumed("88 x 33 x 35 mm"));
    return b;
}

#undef PART_FIELD

struct PartRow {
    std::string id;  // the file's name before its suffix
    PartFile part;
};

// The 22 parts, by kind in the plan's order.
[[nodiscard]] std::vector<PartRow> part_table() {
    std::vector<PartRow> rows;
    rows.push_back({"uiuc_apce_10x5", uiuc_apce_10x5()});
    rows.push_back({"uiuc_apce_9x4.5", uiuc_apce_9x4_5()});
    rows.push_back({"uiuc_mit_5x4", uiuc_mit_5x4()});
    rows.push_back({"uiuc_cfnq_45t1", uiuc_cfnq_45t1()});
    rows.push_back({"triblade_6inch_scaled", triblade_6inch_scaled()});
    rows.push_back({"phantom3_2312_960kv", phantom3_2312_960kv()});
    rows.push_back({"iris_920kv", iris_920kv()});
    rows.push_back({"fpv_2207_1855kv", fpv_2207_1855kv()});
    rows.push_back({"cf_7x16_coreless", cf_7x16_coreless()});
    rows.push_back({"fpv_2306_1700kv", fpv_2306_1700kv()});
    rows.push_back({"comp_2207_2450kv", comp_2207_2450kv()});
    rows.push_back({"phantom3_4x20a", phantom3_4x20a()});
    rows.push_back({"iris_4x20a", iris_4x20a()});
    rows.push_back({"fpv_4in1_45a", four_in_one_45a()});
    rows.push_back({"cf_brushed_4ch", cf_brushed_4ch()});
    rows.push_back({"basic_4in1_45a", four_in_one_45a()});
    rows.push_back({"comp_4in1_45a", four_in_one_45a()});
    rows.push_back({"phantom3_4s_4480", phantom3_4s_4480()});
    rows.push_back({"iris_3s_5100", iris_3s_5100()});
    rows.push_back({"fpv_6s_1300", fpv_6s_1300()});
    rows.push_back({"cf_1s_250", cf_1s_250()});
    rows.push_back({"comp_4s", comp_4s()});
    return rows;
}

// The row's file as the index spells it: parts/<id>.<kind>.yaml.
[[nodiscard]] std::string index_file(const PartRow& row) {
    return "parts/" + row.id + std::string(sc::part_suffix(sc::part_kind(row.part)));
}

[[nodiscard]] fs::path path_of(const PartRow& row) { return library_dir() / fs::path(index_file(row)); }

[[nodiscard]] LibraryKind library_kind(PartKind kind) {
    switch (kind) {
        case PartKind::motor: return LibraryKind::motor;
        case PartKind::prop: return LibraryKind::prop;
        case PartKind::esc: return LibraryKind::esc;
        case PartKind::battery: return LibraryKind::battery;
    }
    return LibraryKind::model;
}

// Interface's category for each part kind (INT-14).
[[nodiscard]] std::string_view category_of(PartKind kind) {
    switch (kind) {
        case PartKind::motor: return "motors";
        case PartKind::prop: return "props";
        case PartKind::esc: return "ESCs";
        case PartKind::battery: return "batteries";
    }
    return "";
}

[[nodiscard]] const PartSources& sources_of(const PartFile& part) {
    return std::visit([](const auto& p) -> const PartSources& { return p.sources; }, part);
}

[[nodiscard]] const SourceRef* citation(const PartSources& sources, std::string_view field) {
    for (const FieldSource& s : sources) {
        if (s.field == field) return &s.ref;
    }
    return nullptr;
}

[[nodiscard]] spade::Result<SourceRegistry> library_sources() {
    return sc::load_sources(library_dir() / "sources.yaml");
}

void expect_same_table(const std::vector<std::array<double, 2>>& file, std::span<const std::array<double, 2>> header,
                       std::string_view what) {
    ASSERT_EQ(file.size(), header.size()) << what;
    for (std::size_t i = 0; i < file.size(); ++i) {
        EXPECT_EQ(bits(file[i][0]), bits(header[i][0])) << what << "[" << i << "][0]";
        EXPECT_EQ(bits(file[i][1]), bits(header[i][1])) << what << "[" << i << "][1]";
    }
}

void expect_same_list(const std::vector<double>& file, std::span<const double> header, std::string_view what) {
    ASSERT_EQ(file.size(), header.size()) << what;
    for (std::size_t i = 0; i < file.size(); ++i) {
        EXPECT_EQ(bits(file[i]), bits(header[i])) << what << "[" << i << "]";
    }
}

}  // namespace

// ===========================================================================
// The part files, and their generator
// ===========================================================================

// The golden, and with SPADE_LIBRARY_WRITE set, its generator.
TEST(LibraryParts, TheCommittedFilesAreTheGeneratorsText) {
    const bool write = library_write_asked();
    std::size_t written = 0;
    const std::vector<PartRow> rows = part_table();
    ASSERT_EQ(rows.size(), 22u) << "5 props, 6 motors, 6 ESCs (one per drone) and 5 batteries";
    for (const PartRow& row : rows) {
        SCOPED_TRACE(row.id);
        const spade::Result<std::string> text = sc::part_to_yaml(row.part);
        ASSERT_OK(text);
        const fs::path path = path_of(row);

        if (write) {
            std::error_code ec;
            fs::create_directories(path.parent_path(), ec);
            ASSERT_FALSE(ec) << spade::path_text(path.parent_path()) << ": " << ec.message();
            {
                std::ofstream out(path, std::ios::binary | std::ios::trunc);
                out.write(text->data(), static_cast<std::streamsize>(text->size()));
                out.close();
                ASSERT_TRUE(out) << "SPADE_LIBRARY_WRITE: could not write " << spade::path_text(path);
            }
            std::string back;
            ASSERT_TRUE(read_bytes(path, back)) << spade::path_text(path);
            ASSERT_TRUE(back == *text) << "SPADE_LIBRARY_WRITE: " << spade::path_text(path)
                                       << " does not read back as written";
            std::printf("SPADE_LIBRARY_WRITE: wrote %zu bytes to %s.\n", text->size(), spade::path_text(path).c_str());
            ++written;
            continue;
        }

        std::string committed;
        ASSERT_TRUE(read_bytes(path, committed))
            << spade::path_text(path) << " is missing (TD-5). Generate it: set SPADE_LIBRARY_WRITE=1 and run ctest -R "
            << "LibraryParts.TheCommittedFilesAreTheGeneratorsText.";
        EXPECT_TRUE(committed == *text) << mismatch(path, committed, *text);
    }
    if (write) {
        std::printf("SPADE_LIBRARY_WRITE: wrote %zu part files. Review the diff, name its cause (TD-1), and have a "
                    "fresh Docker gcc leg reproduce them (TD-12).\n",
                    written);
        RecordProperty("written", static_cast<int>(written));
        SUCCEED() << "wrote " << written << " part files";
    }
}

// Each part is listed in the index, in its kind's category with its
// thumbnail, and loads against the library's sources.
TEST(LibraryParts, EveryPartIsInTheIndexAndLoads) {
    const spade::Result<Library> library = sc::load_library(library_dir() / "library.yaml");
    ASSERT_OK(library);
    const spade::Result<SourceRegistry> registry = sc::load_library_sources(*library);
    ASSERT_OK(registry);
    for (const PartRow& row : part_table()) {
        const std::string file = index_file(row);
        SCOPED_TRACE(file);
        const LibraryEntry* entry = nullptr;
        for (const LibraryEntry& e : library->entries) {
            if (e.file == file) entry = &e;
        }
        if (entry == nullptr) {
            ADD_FAILURE() << "library.yaml has no entry whose file is \"" << file << "\"";
            continue;
        }
        const PartKind kind = sc::part_kind(row.part);
        EXPECT_EQ(entry->kind, library_kind(kind));
        EXPECT_EQ(entry->category, category_of(kind)) << "INT-14: Interface's category for the part kind";
        EXPECT_FALSE(entry->thumbnail.empty()) << "N14b: every entry has its thumbnail";
        const spade::Result<PartFile> part = sc::load_library_part(*library, *entry, *registry);
        EXPECT_OK(part);
        if (part) {
            EXPECT_EQ(sc::part_kind(*part), kind);
        }
    }
}

// Each prop file's numbers are the L4a header's, bit for bit, and its
// coefficients' citations carry the header's grade note verbatim.
TEST(LibraryParts, EachPropsTablesAreTheHeadersBitForBit) {
    const spade::Result<SourceRegistry> registry = library_sources();
    ASSERT_OK(registry);
    for (const Prop& p : kProps) {
        SCOPED_TRACE(std::string(p.id));
        const fs::path path = library_dir() / "parts" / (std::string(p.id) + ".prop.yaml");
        const spade::Result<PartFile> file = sc::load_part_file(path, *registry);
        EXPECT_OK(file);
        if (!file) continue;
        const PropPart* prop = std::get_if<PropPart>(&*file);
        ASSERT_NE(prop, nullptr);
        EXPECT_EQ(prop->block.blades, static_cast<std::uint32_t>(p.blades));
        EXPECT_EQ(bits(prop->block.ct_static), bits(p.ct_static)) << "ct_static";
        EXPECT_EQ(bits(prop->block.cq_static), bits(p.cq_static)) << "cq_static";
        expect_same_table(prop->block.ct_table, p.ct_table, "ct_table");
        expect_same_table(prop->block.cq_table, p.cq_table, "cq_table");
        ASSERT_TRUE(prop->blade.has_value()) << "every L4a prop carries its blade";
        expect_same_list(prop->blade->r_over_r, p.r_over_r, "blade.r_over_r");
        expect_same_list(prop->blade->c_over_r, p.c_over_r, "blade.c_over_r");
        expect_same_list(prop->blade->beta_deg, p.beta_deg, "blade.beta_deg");
        EXPECT_EQ(bits(prop->blade->hub_cutout), bits(p.hub_cutout)) << "blade.hub_cutout";
        for (const char* field : {"ct_static", "cq_static", "ct_table", "cq_table"}) {
            const SourceRef* ref = citation(prop->sources, field);
            ASSERT_NE(ref, nullptr) << field;
            EXPECT_EQ(ref->source, kUiuc) << field;
            EXPECT_EQ(ref->note, p.note) << field << ": the header's grade note, verbatim";
        }
    }
}

// A value the plan does not give is assumed, and the few the rulings grade
// otherwise are as ruled: a sanity table checked against the files.
TEST(LibraryParts, NoAssumedNumberIsGradedAsMeasured) {
    struct GradeRow {
        std::string_view id;
        std::string_view field;  // "*": every field the file cites
        Grade grade;
    };
    const std::vector<GradeRow> expected{
        // The plan gives no ESC number.
        {"phantom3_4x20a", "*", Grade::assumed},
        {"iris_4x20a", "*", Grade::assumed},
        {"fpv_4in1_45a", "*", Grade::assumed},
        {"cf_brushed_4ch", "*", Grade::assumed},
        {"basic_4in1_45a", "*", Grade::assumed},
        {"comp_4in1_45a", "*", Grade::assumed},
        // Kat's fit was withdrawn; no pack listing is registered (the lead, 2026-10-22).
        {"comp_2207_2450kv", "*", Grade::assumed},
        {"comp_4s", "*", Grade::assumed},
        {"fpv_6s_1300", "*", Grade::assumed},
        // Physics' rulings on L4's part numbers (2026-10-22).
        {"cf_7x16_coreless", "resistance", Grade::fitted},
        {"fpv_2207_1855kv", "current_max", Grade::retailer},
        {"fpv_2306_1700kv", "rotor_inertia", Grade::assumed},
        {"phantom3_4s_4480", "cell_voltage_nominal", Grade::derived},
        {"phantom3_4s_4480", "cell_voltage_full", Grade::assumed},
        {"phantom3_4s_4480", "ocv_table", Grade::assumed},
        // A static-only prop's empty tables, and the stand-in's.
        {"uiuc_cfnq_45t1", "ct_table", Grade::assumed},
        {"uiuc_cfnq_45t1", "cq_table", Grade::assumed},
        {"triblade_6inch_scaled", "ct_table", Grade::assumed},
    };
    const spade::Result<SourceRegistry> registry = library_sources();
    ASSERT_OK(registry);
    std::vector<std::pair<std::string, PartFile>> loaded;
    for (const PartRow& row : part_table()) {
        spade::Result<PartFile> file = sc::load_part_file(path_of(row), *registry);
        EXPECT_OK(file) << row.id;
        if (file) loaded.emplace_back(row.id, std::move(*file));
    }
    ASSERT_EQ(loaded.size(), 22u) << "every part file loads";

    for (const GradeRow& g : expected) {
        SCOPED_TRACE(std::string(g.id) + "." + std::string(g.field));
        const auto it = std::find_if(loaded.begin(), loaded.end(), [&](const auto& l) { return l.first == g.id; });
        ASSERT_NE(it, loaded.end());
        const PartSources& sources = sources_of(it->second);
        if (g.field == "*") {
            for (const FieldSource& s : sources) {
                EXPECT_EQ(sc::grade_name(s.ref.grade), sc::grade_name(g.grade)) << s.field;
            }
        } else {
            const SourceRef* ref = citation(sources, g.field);
            ASSERT_NE(ref, nullptr);
            EXPECT_EQ(sc::grade_name(ref->grade), sc::grade_name(g.grade));
        }
    }

    // Across every part: only UIUC's and NASA's numbers are measured; every
    // motor's inductance is assumed, not modelled; every airfoil is assumed;
    // and Kat's withdrawn datapack is cited only as assumed, saying so.
    for (const auto& [id, part] : loaded) {
        SCOPED_TRACE(id);
        for (const FieldSource& s : sources_of(part)) {
            if (s.ref.grade == Grade::measured) {
                EXPECT_TRUE(s.ref.source == kUiuc || s.ref.source == kNasa) << s.field << " cites " << s.ref.source;
            }
            if (s.ref.source == kKatDatapack) {
                EXPECT_EQ(sc::grade_name(s.ref.grade), "assumed")
                    << s.field << ": no fitted grade cites withdrawn work";
                EXPECT_TRUE(s.ref.note.ends_with(kWithdrawn)) << s.field << ": " << s.ref.note;
            }
        }
        if (sc::part_kind(part) == PartKind::motor) {
            const SourceRef* ref = citation(sources_of(part), "inductance");
            ASSERT_NE(ref, nullptr);
            EXPECT_EQ(sc::grade_name(ref->grade), "assumed") << "inductance";
            EXPECT_EQ(ref->note, kInductance);
        }
        if (sc::part_kind(part) == PartKind::prop) {
            const SourceRef* ref = citation(sources_of(part), "blade.airfoil");
            ASSERT_NE(ref, nullptr);
            EXPECT_EQ(sc::grade_name(ref->grade), "assumed") << "blade.airfoil";
        }
    }
}
