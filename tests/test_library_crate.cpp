// The Crate as a library model (L82a Q1, L82d): assets/library/bodies/crate.model.yaml, the library's first body with a contact shape.
// A box, 1 kg, 0.2 m across: one box part, the inertia of the solid box, a box collision shape, no rotors. It is a model used as a
// vehicle, so a user can place a box from the Library panel and drop it (docs/design/interface/plans/2026-11-16-l82-contact-shapes-in-
// the-editor-draft.md). Its proxy sphere is inert by L82d (a model with a shape takes the shape instead), so its own proxy_radius is 0.
//
// What is pinned: the committed file and its sidecar are the generator's text (SPADE_LIBRARY_WRITE=1 writes them, as test_library_quad.cpp
// does); the file loads with its shape; a Crate dropped with a tilt tumbles onto a face and rests there.
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <string>
#include <string_view>
#include <vector>

#include <glm/geometric.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/vec3.hpp>

#include "core/error.hpp"
#include "core/path_text.hpp"
#include "physics/rigid_contact.hpp"
#include "rigid_contact_support.hpp"
#include "scene/library.hpp"
#include "scene/library_sources.hpp"
#include "scene/model_file.hpp"
#include "scene/model_sources.hpp"
#include "sim/simulation.hpp"
#include "vehicles/model_type.hpp"
#include "world/sdf.hpp"

namespace {

namespace fs = std::filesystem;
namespace sc = spade::scene;
using namespace spade::test::rigid;

constexpr std::string_view kCrateFile = "bodies/crate.model.yaml";
constexpr std::string_view kCrateSources = "bodies/crate.sources.yaml";
constexpr float kHalf = 0.1f;  // 0.2 m across

[[nodiscard]] fs::path library_dir() { return fs::path(SPADE_ASSETS_DIR) / "library"; }

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

// The generator's input: a box of 1 kg and 0.2 m, with the box's inertia, one box part and a box shape.
[[nodiscard]] spade::vehicles::ModelType crate_model() {
    spade::vehicles::ModelType m;
    m.name = "crate";
    m.version = 1;
    m.param_schema_id = 1;
    m.body.mass = 1.0f;
    const float side2 = (2.0f * kHalf) * (2.0f * kHalf);
    const float inertia = m.body.mass * (side2 + side2) / 12.0f;  // a solid cube: m (a^2 + a^2) / 12 about each axis
    m.body.inertia_diag = glm::vec3(inertia);
    // proxy_radius stays 0: a model with a shape does not use it (physics/rigid_contact.hpp).

    spade::vehicles::PartDesc part;
    part.name = "box";
    part.role = "body";
    part.shape.transforms.push_back(spade::SdfTransform{});
    spade::SdfNode node{};
    node.kind = static_cast<uint32_t>(spade::SdfPrim::box);
    node.op = static_cast<uint32_t>(spade::SdfOp::none);
    node.params = glm::vec4(kHalf, kHalf, kHalf, 0.0f);
    part.shape.nodes = {node};
    part.min_feature = 2.0f * kHalf;
    m.parts.push_back(std::move(part));

    m.shape = ph::box_shape(glm::vec3(kHalf));
    return m;
}

[[nodiscard]] spade::Result<std::string> generated_model_text() {
    return sc::model_to_yaml(sc::ModelFile{crate_model()});
}

[[nodiscard]] spade::Result<std::string> generated_sources_text(const std::string& model_bytes) {
    sc::ModelSources s;
    s.model = sc::PinnedFile{std::string(kCrateFile), sc::pinned_file_hash(model_bytes)};
    s.built_by = "crate_model() (tests/test_library_crate.cpp)";
    // Every number is a choice, so each is graded assumed (the editor marks an assumed tile).
    const glm::vec3 inertia = crate_model().body.inertia_diag;
    s.inputs.push_back(sc::ModelInput{"body.mass", std::vector<double>{1.0},
                                      sc::SourceRef{"", sc::Grade::assumed, "1 kg by choice: a generic test prop, not a real object"},
                                      std::nullopt});
    s.inputs.push_back(sc::ModelInput{"body.inertia_diag",
                                      std::vector<double>{inertia.x, inertia.y, inertia.z},
                                      sc::SourceRef{"", sc::Grade::assumed, "a solid cube of 1 kg, 0.2 m across: m (a^2 + a^2) / 12"},
                                      std::nullopt});
    s.caveat = "a generic test prop with no citation: a 1 kg box, 0.2 m across, with the inertia of a solid cube and a box "
               "contact shape, for dropping and tumbling in the editor; it is not a real object";
    return sc::model_sources_to_yaml(s);
}

// Compare the committed file to the generator's text; with SPADE_LIBRARY_WRITE set, write it instead (and pass, saying so).
void expect_generated(const fs::path& path, const std::string& text) {
    if (library_write_asked()) {
        std::error_code ec;
        fs::create_directories(path.parent_path(), ec);
        ASSERT_FALSE(ec) << spade::path_text(path.parent_path()) << ": " << ec.message();
        {
            std::ofstream out(path, std::ios::binary | std::ios::trunc);
            out.write(text.data(), static_cast<std::streamsize>(text.size()));
            out.close();
            ASSERT_TRUE(out) << "SPADE_LIBRARY_WRITE: could not write " << spade::path_text(path);
        }
        std::string written;
        ASSERT_TRUE(read_bytes(path, written)) << spade::path_text(path);
        ASSERT_TRUE(written == text) << "SPADE_LIBRARY_WRITE: " << spade::path_text(path) << " does not read back as written";
        std::printf("SPADE_LIBRARY_WRITE: wrote %zu bytes to %s.\n", text.size(), spade::path_text(path).c_str());
        SUCCEED() << "wrote " << spade::path_text(path);
        return;
    }
    std::string committed;
    ASSERT_TRUE(read_bytes(path, committed)) << spade::path_text(path)
                                             << " is missing. Generate it: SPADE_LIBRARY_WRITE=1 ctest -R LibraryCrate.";
    EXPECT_TRUE(committed == text) << spade::path_text(path) << " is not the generator's text; never edit it by hand. Regenerate: "
                                   << "SPADE_LIBRARY_WRITE=1 ctest -R LibraryCrate.";
}

}  // namespace

TEST(LibraryCrate, TheCommittedFileIsTheGeneratorsText) {
    const spade::Result<std::string> text = generated_model_text();
    ASSERT_TRUE(text.has_value()) << text.error().context;
    expect_generated(library_dir() / fs::path(kCrateFile), *text);
}

TEST(LibraryCrate, TheCommittedSidecarIsTheGeneratorsText) {
    std::string model;
    ASSERT_TRUE(read_bytes(library_dir() / fs::path(kCrateFile), model)) << "generate the model file first";
    const spade::Result<std::string> text = generated_sources_text(model);
    ASSERT_TRUE(text.has_value()) << text.error().context;
    expect_generated(library_dir() / fs::path(kCrateSources), *text);
}

TEST(LibraryCrate, TheFileLoadsWithItsBoxShapeAndAPlainSidecar) {
    const spade::Result<sc::ModelFile> loaded = sc::load_model_file(library_dir() / fs::path(kCrateFile));
    ASSERT_TRUE(loaded.has_value()) << loaded.error().context;
    const spade::vehicles::ModelType& m = loaded->model;
    EXPECT_EQ(m.body.mass, 1.0f);
    EXPECT_TRUE(m.rotors.empty()) << "a crate has no rotors";
    ASSERT_EQ(m.parts.size(), 1u);
    ASSERT_TRUE(m.shape.has_value());
    EXPECT_EQ(m.shape->kind, ph::ShapeKind::box);
    EXPECT_EQ(m.shape->half_extents, glm::vec3(kHalf));
    EXPECT_EQ(m.proxy_radius, 0.0f);

    const spade::Result<sc::ModelSources> sources = sc::load_model_sources(library_dir() / fs::path(kCrateSources));
    ASSERT_TRUE(sources.has_value()) << sources.error().context;
    EXPECT_NE(sources->caveat.find("no citation"), std::string::npos) << sources->caveat;
    EXPECT_TRUE(sources->parts.empty());
    ASSERT_EQ(sources->inputs.size(), 2u);
    for (const sc::ModelInput& in : sources->inputs) EXPECT_EQ(sc::grade_name(in.ref.grade), "assumed") << in.path;
}

// The whole point: a Crate placed from the Library and dropped with a tilt tumbles onto a face and rests there. The model's own proxy
// radius would have frozen it on its corner (L82d); here it comes to rest with a world axis up, at the half side above the floor.
TEST(LibraryCrate, ACrateDroppedWithATiltRestsOnAFace) {
    const spade::Result<sc::ModelFile> loaded = sc::load_model_file(library_dir() / fs::path(kCrateFile));
    ASSERT_TRUE(loaded.has_value()) << loaded.error().context;

    Simulation sim = make(tumbling_set());
    const auto id = sim.register_model(loaded->model);
    ASSERT_TRUE(id.has_value()) << id.error().context;
    spade::VehicleSpawn at;
    at.pos = glm::vec3(0.0f, 0.5f, 0.0f);
    at.orient = glm::angleAxis(0.6f, glm::normalize(glm::vec3(1.0f, 0.0f, 1.0f)));
    const auto ref = sim.spawn(0, *id, at);
    ASSERT_TRUE(ref.has_value()) << ref.error().context;
    ASSERT_TRUE(sim.flush_structural().has_value());

    ASSERT_TRUE(sim.step(4000).has_value());
    const BodyState end = body(sim, ref->body);
    EXPECT_LT(glm::length(end.vel), 0.05f) << "still moving";
    EXPECT_LT(glm::length(end.omega_body), 0.05f) << "still turning";
    // On a face: one of the three body axes is within a degree of the world's up.
    const glm::quat q = end.orient;
    float best = 0.0f;
    for (const glm::vec3 axis : {glm::vec3(1, 0, 0), glm::vec3(0, 1, 0), glm::vec3(0, 0, 1)}) {
        best = std::max(best, std::abs((q * axis).y));
    }
    EXPECT_GT(best, std::cos(glm::radians(1.0f))) << "resting on a corner or an edge: best axis alignment " << best;
    EXPECT_NEAR(end.pos.y, kHalf, 0.01f) << "the centre sits half a side above the floor";
}
