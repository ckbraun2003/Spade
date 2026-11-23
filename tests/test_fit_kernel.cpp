// test_fit_kernel.cpp -- a polar DECLARES the kernel width (in cells) it was fitted at (O207, Cameron's "new setting"; the lead and Oversight's
// ruling of 2026-11-23): a case that asks for none runs at its polar's declared kernel (else the line's default 2), a case that asks for a
// different one is refused by name, and the polar's numbers agree in EVERY file that carries them.
//
//   - the optional part-file key `blade.fit_kernel_cells` (round trip, refusals by field name, absent = every byte as before);
//   - the resolver physics/airflow/rotor_lines.hpp resolve_line_kernel() and LineRotorSpec::eps_cells reaching the LineRotor;
//   - THE AGREEMENT TEST: the library's fits (tools/library_props/polar_fits.json, through its generated header) against every part file
//     and constant that copies a polar's numbers, each polar's declared kernel against its fit's cells, and the film's own copy (the
//     film's polar is its own fit: alpha_0, cd_scale and the 2.97 cells of its refit at R/dx 51, one table below) against the film's
//     files. The dated RECORDS and PLANS under docs/ that quote the numbers are history, not copies: they are excluded by name below.
#include <gtest/gtest.h>

#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <limits>
#include <string>
#include <vector>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include "library_polar_fits.gen.hpp"
#include "physics/airflow/rotor_lines.hpp"
#include "scene/library_sources.hpp"
#include "scene/model_sources.hpp"
#include "scene/part_file.hpp"
#include "scene/prop_line.hpp"

namespace {

namespace af = spade::physics::airflow;
namespace sc = spade::scene;
namespace fs = std::filesystem;
namespace fits = spade::test::library_polar_fits;

[[nodiscard]] std::string read_all(const fs::path& p) {
    std::ifstream in(p, std::ios::binary);
    return {std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>()};
}
[[nodiscard]] fs::path repo(const char* rel) { return fs::path(SPADE_REPO_DIR) / rel; }
[[nodiscard]] bool contains(const std::string& text, const std::string& what) { return text.find(what) != std::string::npos; }

[[nodiscard]] const fits::Fit& fit_of(std::string_view id) {
    for (const fits::Fit& f : fits::kFits) {
        if (f.id == id) return f;
    }
    ADD_FAILURE() << "no fit for " << id;
    return fits::kFits[0];
}

[[nodiscard]] sc::PropPart prop_of(const fs::path& file) {
    const auto part = sc::prop_from_yaml(read_all(file));
    EXPECT_TRUE(part.has_value()) << file.string() << ": " << (part ? "" : part.error().context);
    return part ? *part : sc::PropPart{};
}

// A polar number of a part file's text, by its key, in either schema (the X500 rotors are blocks-only version 2 files, the rest version 1):
// the first "<key>: <number>" line. NaN when the key is absent.
[[nodiscard]] double number_of(const std::string& text, const std::string& key) {
    const std::string needle = " " + key + ": ";
    const std::size_t at = text.find(needle);
    if (at == std::string::npos) return std::numeric_limits<double>::quiet_NaN();
    return std::strtod(text.c_str() + at + needle.size(), nullptr);
}

[[nodiscard]] sc::PropPart library_5x4() { return prop_of(fs::path(SPADE_ASSETS_DIR) / "library/parts/uiuc_mit_5x4.prop.yaml"); }

// ---- the part file's key ----------------------------------------------------------------------------------------------------------

TEST(FitKernel, AKernelRoundTripsBitForBitAndIsCited) {
    sc::PropPart p = library_5x4();
    ASSERT_TRUE(p.blade.has_value());
    p.blade->fit_kernel_cells = 2.83;
    bool cited = false;
    for (const sc::FieldSource& s : p.sources) cited = cited || s.field == "blade.fit_kernel_cells";
    if (!cited) p.sources.push_back(sc::FieldSource{"blade.fit_kernel_cells", sc::SourceRef{"", sc::Grade::fitted, "the kernel the polar was fitted at"}});
    const auto text = sc::prop_to_yaml(p);
    ASSERT_TRUE(text.has_value()) << text.error().context;
    EXPECT_TRUE(contains(*text, "    fit_kernel_cells: 2.83\n")) << *text;
    EXPECT_TRUE(contains(*text, "  blade.fit_kernel_cells: ")) << "its citation";
    const auto back = sc::prop_from_yaml(*text);
    ASSERT_TRUE(back.has_value()) << back.error().context;
    ASSERT_TRUE(back->blade->fit_kernel_cells.has_value());
    EXPECT_EQ(*back->blade->fit_kernel_cells, 2.83);
    const auto again = sc::prop_to_yaml(*back);
    ASSERT_TRUE(again.has_value());
    EXPECT_EQ(*again, *text);
}

TEST(FitKernel, ARefusedKernelIsNamedByItsField) {
    const auto expect_refused = [](const sc::PropPart& p, const char* needle) {
        const auto text = sc::prop_to_yaml(p);
        ASSERT_FALSE(text.has_value()) << needle;
        EXPECT_NE(text.error().context.find(needle), std::string::npos) << text.error().context;
    };
    sc::PropPart base = library_5x4();
    bool cited = false;
    for (const sc::FieldSource& s : base.sources) cited = cited || s.field == "blade.fit_kernel_cells";
    if (!cited) base.sources.push_back(sc::FieldSource{"blade.fit_kernel_cells", sc::SourceRef{"", sc::Grade::fitted, "x"}});
    for (const double bad : {0.99, 3.01, 0.0, -1.0, std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::infinity()}) {
        sc::PropPart p = base;
        p.blade->fit_kernel_cells = bad;
        expect_refused(p, "fit_kernel_cells");
    }
    for (const double edge : {1.0, 3.0}) {  // the range's own ends are accepted
        sc::PropPart p = base;
        p.blade->fit_kernel_cells = edge;
        EXPECT_TRUE(sc::prop_to_yaml(p).has_value()) << edge;
    }
    sc::PropPart no_polar = base;
    no_polar.blade->alpha_0_deg.reset();
    no_polar.blade->cd_scale.reset();
    no_polar.blade->fit_kernel_cells = 2.0;
    expect_refused(no_polar, "fit_kernel_cells");  // a kernel with no polar to be the kernel of
}

// A part with no kernel is every byte it was: the writer's text of an unchanged part equals its file, and its pinned hash is the one the
// library drone's sources pin (assets/library/drones/basic_quad_6inch.sources.yaml: parts/triblade_6inch_scaled.prop.yaml 82082d320c34e718).
TEST(FitKernel, APartWithNoKernelKeepsEveryByteAndItsPinnedHash) {
    const fs::path file = fs::path(SPADE_ASSETS_DIR) / "library/parts/triblade_6inch_scaled.prop.yaml";
    const std::string text = read_all(file);
    EXPECT_EQ(sc::pinned_file_hash(text), 0x82082d320c34e718ull) << "this part is untouched by the declaration";
    const auto part = sc::prop_from_yaml(text);
    ASSERT_TRUE(part.has_value());
    EXPECT_FALSE(part->blade->fit_kernel_cells.has_value());
    const auto again = sc::prop_to_yaml(*part);
    ASSERT_TRUE(again.has_value());
    EXPECT_EQ(*again, text) << "the writer adds nothing for an absent kernel";
}

TEST(FitKernel, TheLinePropellerCarriesTheDeclaredKernelOrZero) {
    sc::PropPart p = library_5x4();
    p.blade->fit_kernel_cells = 2.83;
    const auto declared = sc::line_propeller(p);
    ASSERT_TRUE(declared.has_value()) << declared.error().context;
    EXPECT_EQ(declared->fit_kernel_cells, 2.83f);
    sc::PropPart none = library_5x4();
    none.blade->fit_kernel_cells.reset();
    const auto undeclared = sc::line_propeller(none);
    ASSERT_TRUE(undeclared.has_value());
    EXPECT_EQ(undeclared->fit_kernel_cells, 0.0f);
}

// ---- the resolver and the spec ----------------------------------------------------------------------------------------------------

TEST(FitKernel, TheKernelAKindOfCaseRunsAtIsDecidedByTheResolver) {
    const auto at = [](float case_cells, float fit_cells) { return af::resolve_line_kernel(case_cells, fit_cells); };
    ASSERT_TRUE(at(0.0f, 0.0f).has_value());
    EXPECT_EQ(*at(0.0f, 0.0f), 2.0f) << "neither: the line's own default";
    EXPECT_EQ(*at(0.0f, 2.83f), 2.83f) << "no case kernel: the polar's declared one";
    EXPECT_EQ(*at(3.0f, 0.0f), 3.0f) << "no declaration: the case's";
    EXPECT_EQ(*at(2.97f, 2.97f), 2.97f) << "equal: fine";
    const auto clash = at(2.0f, 2.83f);
    ASSERT_FALSE(clash.has_value()) << "an explicit kernel that is not the polar's is refused";
    EXPECT_NE(clash.error().context.find("kernel"), std::string::npos) << clash.error().context;
    EXPECT_NE(clash.error().context.find("2.83"), std::string::npos) << "naming the polar's: " << clash.error().context;
    EXPECT_NE(clash.error().context.find('2'), std::string::npos);
    for (const float bad : {0.5f, 3.5f, -1.0f, std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::infinity()}) {
        EXPECT_FALSE(at(bad, 0.0f).has_value()) << "case kernel " << bad;
        EXPECT_FALSE(at(0.0f, bad).has_value()) << "declared kernel " << bad;
    }
}

TEST(FitKernel, TheSpecsKernelReachesTheLineAndTheRangeIsRefusedByName) {
    af::BladeShape blade;
    blade.blades = 2;
    blade.radius = 0.0762f;
    blade.stations = {{0.012f, 0.012f, 30.0f}, {0.04f, 0.011f, 20.0f}, {0.0762f, 0.006f, 12.0f}};
    const af::PolarTable polar = af::estimated_polar();
    const af::DiscBody body{.com_world = glm::vec3(0.0f), .orient_world = glm::quat(1.0f, 0.0f, 0.0f, 0.0f)};
    const af::GridShape grid{64, 64, 64, 0.01f};
    af::LineRotorSpec spec{.omega = 100.0f, .blade = &blade, .polar = &polar};
    const auto def = af::line_from_rotor(spec, body, glm::vec3(0.0f), grid);
    ASSERT_TRUE(def.has_value()) << def.error().context;
    EXPECT_EQ(def->eps_cells, 2.0f) << "absent is the line's own default, as before";
    spec.eps_cells = 2.97f;
    const auto set = af::line_from_rotor(spec, body, glm::vec3(0.0f), grid);
    ASSERT_TRUE(set.has_value()) << set.error().context;
    EXPECT_EQ(set->eps_cells, 2.97f);
    // The part file's range for a declared kernel and the line's own are ONE range: the ends are accepted by both, and the first value past
    // either end is refused by both (the part file's side is in AKernelRoundTrips... and ARefusedKernelIsNamedByItsField: 1.0 and 3.0 in, 0.99 and 3.01 out).
    for (const float edge : {1.0f, 3.0f}) {
        spec.eps_cells = edge;
        EXPECT_TRUE(af::line_from_rotor(spec, body, glm::vec3(0.0f), grid).has_value()) << edge;
    }
    for (const float bad : {0.99f, 3.01f, std::numeric_limits<float>::quiet_NaN()}) {
        spec.eps_cells = bad;
        const auto refused = af::line_from_rotor(spec, body, glm::vec3(0.0f), grid);
        ASSERT_FALSE(refused.has_value()) << bad;
        EXPECT_NE(refused.error().context.find("eps_cells"), std::string::npos) << refused.error().context;
    }
}

// ---- THE AGREEMENT TEST -----------------------------------------------------------------------------------------------------------

// Every library part file that carries a fit holds the fit's numbers (bit for bit) and declares the fit's cells.
TEST(FitKernel, EveryLibraryPartHoldsItsFitAndDeclaresItsKernel) {
    for (const fits::Fit& f : fits::kFits) {
        const sc::PropPart p = prop_of(fs::path(SPADE_ASSETS_DIR) / "library/parts" / (std::string(f.id) + ".prop.yaml"));
        ASSERT_TRUE(p.blade.has_value()) << f.id;
        ASSERT_TRUE(p.blade->alpha_0_deg.has_value()) << f.id;
        EXPECT_EQ(*p.blade->alpha_0_deg, f.alpha_0_deg) << f.id;
        EXPECT_EQ(*p.blade->cd_scale, f.cd_scale) << f.id;
        ASSERT_TRUE(p.blade->fit_kernel_cells.has_value()) << f.id << " declares the kernel it was fitted at";
        EXPECT_EQ(*p.blade->fit_kernel_cells, f.fit_kernel_cells) << f.id;
        EXPECT_NE(std::string(f.fit).find(std::to_string(f.fit_kernel_cells).substr(0, 4) + " cells"), std::string::npos)
            << f.id << ": the fit's own text names the cells the table declares: " << f.fit;
    }
}

// The copies of the 10x5's polar (the X500's rotors and props use it, scaled): the same numbers and the same declared kernel, in every file.
TEST(FitKernel, TheCopiesOfTheApce10x5sPolarAgreeAndDeclareItsKernel) {
    const fits::Fit& f = fit_of("uiuc_apce_10x5");
    const std::vector<fs::path> copies = {
        fs::path(SPADE_ASSETS_DIR) / "library/parts/x500/x500_rotor_front_cw.prop.yaml",
        fs::path(SPADE_ASSETS_DIR) / "library/parts/x500/x500_rotor_front_ccw.prop.yaml",
        fs::path(SPADE_ASSETS_DIR) / "library/parts/x500/x500_rotor_rear_cw.prop.yaml",
        fs::path(SPADE_ASSETS_DIR) / "library/parts/x500/x500_rotor_rear_ccw.prop.yaml",
        repo("tools/flow_movie/data/x500/parts/x500_10in_apc10x5.prop.yaml"),
        repo("tests/data/library_contract/roots/quad/parts/p.prop.yaml"),
    };
    for (const fs::path& file : copies) {
        const std::string text = read_all(file);
        ASSERT_FALSE(text.empty()) << file.string();
        EXPECT_EQ(number_of(text, "alpha_0_deg"), f.alpha_0_deg) << file.string();
        EXPECT_EQ(number_of(text, "cd_scale"), f.cd_scale) << file.string();
        EXPECT_EQ(number_of(text, "fit_kernel_cells"), f.fit_kernel_cells) << file.string() << " declares the kernel the polar was fitted at";
    }
    // a hand-written test's sample value is the table's, so a re-fit that moves the table is noticed here too
    EXPECT_TRUE(contains(read_all(repo("tests/test_part_file.cpp")), "alpha_0_deg: -3.0645\\n")) << "tests/test_part_file.cpp's sample is the 10x5's";
    EXPECT_EQ(f.alpha_0_deg, -3.0645);
}

// What the agreement test does NOT hold, and why (the lead's ruling: dated records are history). These quote a polar's numbers as they
// were fitted on their day and are corrected by a dated note, never rewritten:
//   docs/design/physics/plans/2026-11-17-b1-polar-fits.md (the fit's own plan), docs/design/physics/plans/2026-11-21-openframe5f-kit-model.md,
//   docs/design/test-docs/2026-11-16-l72d-width-choice-bands.md, docs/design/test-docs/records/2026-11-18-cpu-vs-vulkan-rtx3060ti.md.

}  // namespace
