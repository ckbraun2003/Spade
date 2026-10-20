// The reference quad as a library model file (N4 of the editor end-goal
// design, docs/design/interface/plans/2026-10-19-editor-end-goal-design.md
// §3): assets/library/drones/reference_quad.model.yaml, the default library's
// one drone. The editor's library, Kat's builder and the analysis case load
// this file; the fixture it comes from (tests/reference_airframe.hpp) stays a
// test fixture.
//
// What is pinned here:
//   - THE GOLDEN, and its GENERATOR. The committed file is model_to_yaml() of
//     compile_airframe(reference_quad()).model, its propulsion set to
//     airframe_propulsion_chain(reference_quad()) (the chain is the model's,
//     N5a, so the text nests it in the model block), byte for byte. With
//     SPADE_LIBRARY_WRITE set, the same test WRITES the file into the source
//     tree instead of comparing, and passes, saying so. The file is only ever
//     written that way, never by hand. A moved byte needs a named cause before
//     it is regenerated (TD-1), and a regenerated file is final only once a
//     fresh Docker gcc leg reproduces it (TD-12).
//   - THE FILE LOADS BACK with the fixture's model, parts and propulsion
//     identities, and with PINNED ones, so a fixture change shows here even
//     when the file is regenerated with it. Its parts are SDF parts, so no
//     .spmesh sidecar sits beside it.
//   - ITS CHAIN AND ITS MODEL BELONG TOGETHER: the file's chain, at the fit's
//     hover duty, gives the fit's hover omega, and at that omega the file's
//     rotors carry the file's body.
//
// LINE ENDINGS. model_to_yaml() writes LF, and the comparison reads the file
// in binary. assets/.gitattributes gives *.model.yaml `text eol=lf`, as it
// does the scene and world files, so a Windows checkout (core.autocrlf=true)
// and the leg's git archive both give LF bytes.

#include <gtest/gtest.h>

#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <optional>
#include <string>
#include <system_error>

#include "core/error.hpp"
#include "core/path_text.hpp"
#include "reference_airframe.hpp"
#include "scene/model_file.hpp"
#include "vehicles/airframe_compile.hpp"
#include "vehicles/model_identity.hpp"
#include "vehicles/model_type.hpp"
#include "vehicles/propulsion_steady.hpp"

namespace {

namespace fs = std::filesystem;
using spade::scene::ModelFile;

template <class T>
[[nodiscard]] testing::AssertionResult IsOk(const char* expr, const spade::Result<T>& r) {
    if (r) return testing::AssertionSuccess();
    return testing::AssertionFailure() << expr << " failed: [" << static_cast<int>(r.error().code) << "] "
                                       << r.error().context;
}
#define ASSERT_OK(expr) ASSERT_PRED_FORMAT1(IsOk, expr)

template <class T>
[[nodiscard]] std::string why(const spade::Result<T>& r) {
    return r ? std::string("<succeeded>") : r.error().context;
}

// The committed file, in the SOURCE tree: the generator writes here.
[[nodiscard]] fs::path library_quad_path() {
    return fs::path(SPADE_ASSETS_DIR) / "library" / "drones" / "reference_quad.model.yaml";
}

// SPADE_LIBRARY_WRITE set: write the library file instead of comparing it.
// A read-only null check before any thread starts, as env_gate_is_set() in
// test_fp32_math.cpp.
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

[[nodiscard]] bool read_bytes(const fs::path& path, std::string& out) {
    std::ifstream in(path, std::ios::binary);
    if (!in) return false;
    out.assign(std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>());
    return true;
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
[[nodiscard]] std::string mismatch(const std::string& committed, const std::string& generated) {
    std::size_t i = 0;
    while (i < committed.size() && i < generated.size() && committed[i] == generated[i]) ++i;
    std::size_t line = 1;
    for (std::size_t j = 0; j < i; ++j) {
        if (committed[j] == '\n') ++line;
    }
    std::string out = spade::path_text(library_quad_path()) + " (" + std::to_string(committed.size()) +
                      " bytes) is not the generator's text (" + std::to_string(generated.size()) +
                      " bytes): first difference at byte " + std::to_string(i) + ", line " + std::to_string(line) +
                      ".\n  committed: " + line_at(committed, i) + "\n  generated: " + line_at(generated, i) + "\n";
    if (committed.find('\r') != std::string::npos) {
        out += "The committed file holds CR bytes: a checkout converted its line endings. assets/.gitattributes "
               "must give *.model.yaml `text eol=lf` (check with `git check-attr -a` on the path).\n";
    }
    out += "The file is generated from compile_airframe(reference_quad()) + airframe_propulsion_chain(); never "
           "edit it by hand. To move it: (1) name the cause (TD-1): the fixture, the airframe compile, the chain "
           "or the model-file format; (2) regenerate: set SPADE_LIBRARY_WRITE=1 and run ctest -R "
           "LibraryQuad.TheCommittedFileIsTheGeneratorsText; re-pin LibraryQuad's identities with that cause; "
           "(3) it is final only once a fresh (-NoSeed) Docker gcc leg reproduces it (TD-12).";
    return out;
}

// The generator's input: the fixture, compiled, with its chain as the model's
// (N5a), as ModelFile.TheReferenceQuadRoundTripsWithItsIdentitiesAndItsBytes
// builds it.
[[nodiscard]] std::optional<ModelFile> generated_model_file() {
    const auto compiled = spade::vehicles::compile_airframe(spade::vehicles::fixtures::reference_quad());
    EXPECT_TRUE(compiled.has_value()) << why(compiled);
    const auto chain = spade::vehicles::airframe_propulsion_chain(spade::vehicles::fixtures::reference_quad());
    EXPECT_TRUE(chain.has_value()) << why(chain);
    if (!compiled || !chain) return std::nullopt;
    ModelFile file{compiled->model};
    file.model.propulsion = *chain;
    return file;
}

// THE PINS. Read off the fixture's compile and chain: regression pins, so a
// fixture change shows here even when the file is regenerated with it.
// PLACEHOLDERS until GREEN measures them.
constexpr uint64_t kPinnedModelIdentity = 0x0ull;
constexpr uint64_t kPinnedPartsIdentity = 0x0ull;
constexpr uint64_t kPinnedPropulsionIdentity = 0x0ull;

}  // namespace

// The golden, and with SPADE_LIBRARY_WRITE set, its generator.
TEST(LibraryQuad, TheCommittedFileIsTheGeneratorsText) {
    const std::optional<ModelFile> file = generated_model_file();
    ASSERT_TRUE(file.has_value());
    const spade::Result<std::string> text = spade::scene::model_to_yaml(*file);
    ASSERT_OK(text);
    const fs::path path = library_quad_path();

    if (library_write_asked()) {
        std::error_code ec;
        fs::create_directories(path.parent_path(), ec);
        ASSERT_FALSE(ec) << spade::path_text(path.parent_path()) << ": " << ec.message();
        {
            std::ofstream out(path, std::ios::binary | std::ios::trunc);
            out.write(text->data(), static_cast<std::streamsize>(text->size()));
            out.close();
            ASSERT_TRUE(out) << "SPADE_LIBRARY_WRITE: could not write " << spade::path_text(path);
        }
        std::string written;
        ASSERT_TRUE(read_bytes(path, written)) << spade::path_text(path);
        ASSERT_TRUE(written == *text) << "SPADE_LIBRARY_WRITE: " << spade::path_text(path)
                                      << " does not read back as written";
        std::printf(
            "SPADE_LIBRARY_WRITE: wrote %zu bytes to %s. Review the diff, name its cause (TD-1), re-pin "
            "LibraryQuad's identities, and have a fresh Docker gcc leg reproduce it (TD-12).\n",
            text->size(), spade::path_text(path).c_str());
        RecordProperty("written", spade::path_text(path));
        SUCCEED() << "wrote " << spade::path_text(path);
        return;
    }

    std::string committed;
    ASSERT_TRUE(read_bytes(path, committed))
        << spade::path_text(path) << " is missing (TD-5). Generate it: set SPADE_LIBRARY_WRITE=1 and run ctest -R "
        << "LibraryQuad.TheCommittedFileIsTheGeneratorsText.";
    EXPECT_TRUE(committed == *text) << mismatch(committed, *text);
}

// The committed file loads, with the fixture's identities and the pinned ones.
TEST(LibraryQuad, TheCommittedFileLoadsWithThePinnedIdentities) {
    const fs::path path = library_quad_path();
    const spade::Result<ModelFile> loaded = spade::scene::load_model_file(path);
    ASSERT_OK(loaded);
    ASSERT_TRUE(loaded->model.propulsion.has_value()) << "the library quad carries its chain";
    const std::optional<ModelFile> fixture = generated_model_file();
    ASSERT_TRUE(fixture.has_value());

    // SDF parts only, so the file needs no .spmesh sidecar beside it.
    ASSERT_FALSE(loaded->model.parts.empty()) << "the compiled quad has parts, so parts_identity is not vacuous";
    for (const spade::vehicles::PartDesc& part : loaded->model.parts) {
        EXPECT_TRUE(part.mesh == nullptr) << part.name
                                          << " is a mesh part: its sidecar must be committed beside the file";
        EXPECT_FALSE(part.shape.nodes.empty()) << part.name << " has no SDF shape";
    }

    const uint64_t model = spade::vehicles::model_identity(loaded->model);
    const uint64_t parts = spade::vehicles::parts_identity(loaded->model);
    const uint64_t propulsion = spade::vehicles::propulsion_identity(*loaded->model.propulsion);
    EXPECT_EQ(model, spade::vehicles::model_identity(fixture->model)) << std::hex << model;
    EXPECT_EQ(parts, spade::vehicles::parts_identity(fixture->model)) << std::hex << parts;
    EXPECT_EQ(propulsion, spade::vehicles::propulsion_identity(*fixture->model.propulsion)) << std::hex << propulsion;
    EXPECT_EQ(model, kPinnedModelIdentity) << std::hex << "measured 0x" << model;
    EXPECT_EQ(parts, kPinnedPartsIdentity) << std::hex << "measured 0x" << parts;
    EXPECT_EQ(propulsion, kPinnedPropulsionIdentity) << std::hex << "measured 0x" << propulsion;

    // Opened and saved unedited, the file writes back its own bytes.
    std::string committed;
    ASSERT_TRUE(read_bytes(path, committed));
    const spade::Result<std::string> again = spade::scene::model_to_yaml(*loaded);
    ASSERT_OK(again);
    EXPECT_TRUE(*again == committed) << mismatch(committed, *again);
}

// The file's chain and the file's model belong together, by
// AirframeCompile.TheThrottleMapAtTheHoverDutyHoversTheFittedRotor's measure:
// the file's chain at the fit's hover duty gives the fit's hover omega, and at
// that omega the file's rotors carry the file's body.
TEST(LibraryQuad, TheFilesChainAtTheHoverDutyHoversTheFilesRotors) {
    const spade::vehicles::AirframeSpec spec = spade::vehicles::fixtures::reference_quad();
    const auto compiled = spade::vehicles::compile_airframe(spec);
    ASSERT_TRUE(compiled.has_value()) << why(compiled);
    const spade::vehicles::AirframeFit& fit = compiled->fit;
    const spade::Result<ModelFile> loaded = spade::scene::load_model_file(library_quad_path());
    ASSERT_OK(loaded);
    ASSERT_TRUE(loaded->model.propulsion.has_value());

    // The operating point is the spec's (the file carries no air density or
    // state of charge): still air at full charge.
    const auto r = spade::vehicles::rotor_omega_for_duty(*loaded->model.propulsion, fit.hover_duty, spec.air_density,
                                                         spec.state_of_charge);
    ASSERT_TRUE(r.has_value()) << why(r);
    EXPECT_EQ(r->omega_cmd, static_cast<float>(fit.hover_omega));
    EXPECT_EQ(r->flags, fit.solver_flags);

    // Each of the file's rotors, at omega_cmd, gives the fit's hover thrust
    // (float k_T and omega: under 3e-7 relative), and together they are the
    // file's body's weight.
    const spade::vehicles::ModelType& m = loaded->model;
    ASSERT_EQ(m.rotors.size(), spec.rotors.size());
    const double w = static_cast<double>(r->omega_cmd);
    double total = 0.0;
    for (std::size_t i = 0; i < m.rotors.size(); ++i) {
        const double thrust = static_cast<double>(m.rotors[i].thrust_coeff) * w * w;
        EXPECT_NEAR(thrust, fit.hover_thrust, 1e-6 * fit.hover_thrust) << "rotor " << i;
        total += thrust;
    }
    const double weight = static_cast<double>(m.body.mass) * spec.gravity;
    EXPECT_NEAR(total, weight, 1e-6 * weight);
}
