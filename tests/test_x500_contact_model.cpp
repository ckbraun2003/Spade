// ---------------------------------------------------------------------------
// test_x500_contact_model.cpp -- O140 phase 1 item 4 + O150 video #2: the X500 standing on its twelve foot points as a MODEL FILE the editor loads.
//
// tests/test_x500_ground_contact.cpp builds the hull in memory from tools/flow_movie/data/x500/x500.model.yaml. That file is model_version 1 (no `collision:` key) and the
// posted film pins it, so it is not changed. This test writes and pins its sibling, tools/flow_movie/data/x500/x500_contact.model.yaml (model_version 2, the version that
// reads `collision:`): the same model, named x500_contact, with the hull of tests/x500_contact_hull.hpp as its shape. The Library panel's "Load from file" and a scene file read
// it through scene::load_model_file, and compose()/instantiate() add the rigid_contact module because a model has a shape, so nothing in engine/ or sandbox/ changes.
//
// WHAT IS PINNED, and how it is built like tests/test_library_crate.cpp: the committed file is the generator's text (SPADE_LIBRARY_WRITE=1 writes it); the original file
// is unchanged (a hash of its text with line ends normalised, since a CRLF checkout and an LF one must agree); the new file loads with its hull and differs from the
// original in the hull and the name only; composed and instantiated the way the editor runs a scene, it rests on its feet at the numbers of X500GroundContact.
//
//   X500ContactModel.TheOriginalFileIsUnchanged                       fnv1a64 and length of x500.model.yaml (CR removed), and no `collision` in it: the film's pin cannot move by accident
//   X500ContactModel.TheCommittedFileIsTheGeneratorsText              the committed x500_contact.model.yaml equals the generator's text
//   X500ContactModel.TheFileLoadsWithItsHullAndNothingElseChanged     twelve pad points, skin 9.5 mm, friction 0.6, restitution 0.1, 16 sweeps; stripped of the hull and renamed, it is the original
//   X500ContactModel.ComposedAndInstantiatedItRestsOnItsFeet           the editor's route (compose, instantiate): centre of mass 0.18987 m +-1.5 mm, no walking, no spin, rest speed within PHY-8
//   X500ContactModel.TheEditorsPlacedAtHeightIsTheRestHeight          physics::lowest_contact_point at the design origin, level (what Library placement uses) against the hull's 0.2153 m
// ---------------------------------------------------------------------------

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <optional>
#include <string>
#include <string_view>
#include <utility>

#include <glm/geometric.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/vec3.hpp>

#include "compute/backend.hpp"
#include "core/error.hpp"
#include "core/path_text.hpp"
#include "core/rng.hpp"
#include "physics/contact_sets.hpp"
#include "physics/rigid_contact.hpp"
#include "scene/compose.hpp"
#include "scene/model_file.hpp"
#include "scene/scene_file.hpp"
#include "sim/design_frame.hpp"
#include "sim/simulation.hpp"
#include "sim/world_set.hpp"
#include "vehicles/model_type.hpp"
#include "world/builder.hpp"
#include "world/world_file.hpp"
#include "x500_contact_hull.hpp"

namespace {

namespace fs = std::filesystem;
namespace sc = spade::scene;
namespace x500 = spade::test::x500;

constexpr std::string_view kOriginalFile = "tools/flow_movie/data/x500/x500.model.yaml";
constexpr std::string_view kContactFile = "tools/flow_movie/data/x500/x500_contact.model.yaml";
constexpr std::string_view kContactName = "x500_contact";

// x500.model.yaml as committed (the posted film pins it): fnv1a64 and byte count of its text with every CR removed, measured on the file at master 2fb01194.
constexpr uint64_t kOriginalFnv = 0x5b15f1186943676cULL;
constexpr std::size_t kOriginalBytes = 12798;

constexpr float kG = 9.80665f;
constexpr uint64_t kDtNs = 500'000;          // the tick the autopilot link runs at (fc-physics 5.1)
constexpr double kComY = -0.0254301;         // x500.model.yaml com_offset.y
constexpr float kLowest = -0.2153f;          // the pads' bottom faces, skin included, in the design frame
constexpr float kRestHeight = static_cast<float>(kComY - kLowest);  // 0.18987 m: the centre of mass over the plane at rest
constexpr float kDesignOriginAtRest = -kLowest;                      // 0.2153 m

[[nodiscard]] fs::path repo_file(std::string_view relative) { return fs::path(SPADE_REPO_DIR) / fs::path(std::string(relative)); }

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

// The file's text with every CR removed: the committed text is LF (git's index), a Windows checkout may hold CRLF, and both are the same file.
[[nodiscard]] bool read_text(const fs::path& path, std::string& out) {
    std::ifstream in(path, std::ios::binary);
    if (!in) return false;
    out.assign(std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>());
    std::erase(out, '\r');
    return true;
}

[[nodiscard]] sc::ModelFile load_original() {
    const spade::Result<sc::ModelFile> loaded = sc::load_model_file(repo_file(kOriginalFile));
    EXPECT_TRUE(loaded.has_value()) << (loaded ? "" : loaded.error().context);
    return loaded ? *loaded : sc::ModelFile{};
}

// The generator: the original model, standing on its feet, named x500_contact. model_to_yaml writes model_version 2 because the model has a shape.
[[nodiscard]] spade::Result<std::string> generated_text() {
    sc::ModelFile file = load_original();
    file.model = x500::with_feet(std::move(file.model));
    file.model.name = std::string(kContactName);
    return sc::model_to_yaml(file);
}

}  // namespace

// ===========================================================================
// THE ORIGINAL IS NOT TOUCHED
// ===========================================================================

TEST(X500ContactModel, TheOriginalFileIsUnchanged) {
    std::string text;
    ASSERT_TRUE(read_text(repo_file(kOriginalFile), text)) << spade::path_text(repo_file(kOriginalFile));
    EXPECT_EQ(text.size(), kOriginalBytes) << "x500.model.yaml changed length: the posted film pins it";
    EXPECT_EQ(spade::rng::fnv1a64(text), kOriginalFnv) << "x500.model.yaml changed: the posted film pins it. Add to the new file, not this one.";
    EXPECT_EQ(text.find("collision"), std::string::npos) << "the original is model_version 1: it has no collision block";
}

// ===========================================================================
// THE COMMITTED FILE IS THE GENERATOR'S TEXT
// ===========================================================================

TEST(X500ContactModel, TheCommittedFileIsTheGeneratorsText) {
    const spade::Result<std::string> text = generated_text();
    ASSERT_TRUE(text.has_value()) << text.error().context;
    const fs::path path = repo_file(kContactFile);
    if (library_write_asked()) {
        {
            std::ofstream out(path, std::ios::binary | std::ios::trunc);
            out.write(text->data(), static_cast<std::streamsize>(text->size()));
            out.close();
            ASSERT_TRUE(out) << "SPADE_LIBRARY_WRITE: could not write " << spade::path_text(path);
        }
        std::string written;
        ASSERT_TRUE(read_text(path, written)) << spade::path_text(path);
        ASSERT_TRUE(written == *text) << "SPADE_LIBRARY_WRITE: " << spade::path_text(path) << " does not read back as written";
        std::printf("SPADE_LIBRARY_WRITE: wrote %zu bytes to %s.\n", text->size(), spade::path_text(path).c_str());
        SUCCEED() << "wrote " << spade::path_text(path);
        return;
    }
    std::string committed;
    ASSERT_TRUE(read_text(path, committed)) << spade::path_text(path) << " is missing. Generate it: SPADE_LIBRARY_WRITE=1 spade_tests --gtest_filter=X500ContactModel.*";
    EXPECT_TRUE(committed == *text) << spade::path_text(path) << " is not the generator's text; never edit it by hand. Regenerate: SPADE_LIBRARY_WRITE=1 spade_tests "
                                    << "--gtest_filter=X500ContactModel.*";
}

// ===========================================================================
// IT LOADS, WITH THE HULL, AND IS OTHERWISE THE ORIGINAL
// ===========================================================================

TEST(X500ContactModel, TheFileLoadsWithItsHullAndNothingElseChanged) {
    const spade::Result<sc::ModelFile> loaded = sc::load_model_file(repo_file(kContactFile));
    ASSERT_TRUE(loaded.has_value()) << loaded.error().context;
    const spade::vehicles::ModelType& m = loaded->model;
    EXPECT_EQ(m.name, kContactName);
    ASSERT_TRUE(m.shape.has_value());
    EXPECT_EQ(m.shape->kind, spade::physics::ShapeKind::hull);
    EXPECT_EQ(m.shape->radius, x500::kSkin) << "the hull's skin";
    EXPECT_EQ(m.shape->friction, x500::kFriction);
    EXPECT_EQ(m.shape->restitution, x500::kRestitution);
    EXPECT_EQ(m.shape->iterations, x500::kSweeps);
    ASSERT_EQ(m.shape->points.size(), 12u);

    // The points are the pad table's, in the design frame (back from the body frame the file holds them in), to the file's own float spelling.
    const glm::quat d2p = glm::normalize(m.design_to_principal);
    float lowest = 1.0f;
    for (std::size_t i = 0; i < 12; ++i) {
        const glm::vec3 design = glm::inverse(d2p) * m.shape->points[i] + m.com_offset;
        const glm::vec3 want = x500::pad_points_design()[i];
        EXPECT_NEAR(design.x, want.x, 1.0e-6f) << "point " << i;
        EXPECT_NEAR(design.y, want.y, 1.0e-6f) << "point " << i;
        EXPECT_NEAR(design.z, want.z, 1.0e-6f) << "point " << i;
        lowest = std::min(lowest, design.y);
    }
    EXPECT_NEAR(lowest - x500::kSkin, kLowest, 1.0e-6f) << "the lowest of the drone, skin included";

    // Stripped of the hull and renamed, it is the original, field for field: the writer's text of the one equals the writer's text of the other.
    sc::ModelFile stripped = *loaded;
    stripped.model.shape.reset();
    stripped.model.name = load_original().model.name;
    const spade::Result<std::string> a = sc::model_to_yaml(stripped);
    const spade::Result<std::string> b = sc::model_to_yaml(load_original());
    ASSERT_TRUE(a.has_value()) << a.error().context;
    ASSERT_TRUE(b.has_value()) << b.error().context;
    EXPECT_TRUE(*a == *b) << "the contact file differs from x500.model.yaml in more than the hull and the name";
}

// ===========================================================================
// IT RESTS ON ITS FEET, THE EDITOR'S WAY
// ===========================================================================

// The editor builds a scene (a world with a floor, the model, a vehicle), compose()s it and instantiate()s it: instantiate adds the rigid_contact module because a model has a
// shape. A plain scene file takes the same route. The vehicle is spawned a centimetre above its rest, level, and left to fall on its own feet.
TEST(X500ContactModel, ComposedAndInstantiatedItRestsOnItsFeet) {
    const spade::Result<sc::ModelFile> loaded = sc::load_model_file(repo_file(kContactFile));
    ASSERT_TRUE(loaded.has_value()) << loaded.error().context;

    spade::Environment env;
    env.gravity = glm::vec3(0.0f, -kG, 0.0f);
    env.wind = glm::vec3(0.0f);
    env.air_density = 1.225f;
    const spade::Result<spade::WorldDesc> world =
        spade::WorldBuilder{}.name("ground").environment(env).capacities(spade::Capacities{1, 1, 1, 1}).plane(glm::vec3(0.0f, 1.0f, 0.0f), 0.0f).build();
    ASSERT_TRUE(world.has_value()) << world.error().context;

    sc::SceneDesc scene;
    scene.name = "x500_on_the_ground";
    scene.world.file = "ground.world.yaml";
    scene.world.hash = sc::world_hash(*world).value();
    scene.models.push_back(loaded->model);
    sc::SceneVehicle vehicle;
    vehicle.name = "drone";
    vehicle.model = loaded->model.name;
    vehicle.start.pos = glm::vec3(0.0f, kDesignOriginAtRest + 0.01f, 0.0f);  // the DESIGN origin's pose, as a scene writes it
    vehicle.start.orient = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
    vehicle.start.rotor_omega = 0.0f;
    scene.vehicles.push_back(vehicle);

    const spade::Result<sc::ComposedScene> composed = sc::compose(scene, *world);
    ASSERT_TRUE(composed.has_value()) << composed.error().context;

    spade::WorldInstanceDesc instance;
    instance.seed = 11;
    instance.turbulence = spade::dryden_params(spade::TurbulenceLevel::none);
    instance.contacts.restitution_e = 0.0f;
    instance.contacts.friction_mu = 0.0f;
    instance.contacts.proxy_radius = 0.0f;
    spade::Result<sc::SceneRun> run = sc::instantiate(*composed, instance, kDtNs, 1, spade::compute::BackendDesc{.kind = spade::compute::BackendKind::cpu});
    ASSERT_TRUE(run.has_value()) << run.error().context;

    ASSERT_TRUE(run->sim.step(4000).has_value());  // two seconds
    const auto body = run->sim.body(run->vehicles[0].body);
    ASSERT_TRUE(body.has_value());
    const spade::BodyState& b = **body;
    EXPECT_NEAR(b.pos.y, kRestHeight, 1.5e-3f) << "rest height of the centre of mass (the skin sinks about 1 mm under load)";
    EXPECT_NEAR(glm::length(glm::vec2(b.pos.x, b.pos.z)), 0.0f, 5.0e-3f) << "it has not walked";
    EXPECT_LT(std::fabs(b.vel.y), 5.5e-3f) << "rest velocity (PHY-8: about -g dt = -4.9e-3 m/s at 500 us)";
    EXPECT_LT(glm::length(b.omega_body), 0.05f) << "no spin";
}

// ===========================================================================
// WHERE THE EDITOR PLACES IT
// ===========================================================================

// The Library panel puts an item's origin rest_height() over the floor: physics::lowest_contact_point() of the model at its design frame's origin, level and still (the L9
// contact set). For a model with a shape the sim ignores that set (the shape is its one ground response), so this is only the height the editor starts the drone at; it
// should be the height it rests at, or the drone drops or pops on the first step. The model is the ORIGINAL's, whose parts are the contact file's.
TEST(X500ContactModel, TheEditorsPlacedAtHeightIsTheRestHeight) {
    const spade::vehicles::ModelType model = load_original().model;
    const spade::FrameState b = spade::to_body_state(spade::FrameState{glm::vec3(0.0f), glm::quat(1.0f, 0.0f, 0.0f, 0.0f), glm::vec3(0.0f), glm::vec3(0.0f)},
                                                     glm::normalize(model.design_to_principal), model.com_offset);
    spade::BodyState body{};
    body.pos = b.pos;
    body.orient = b.orient;
    const std::optional<float> low = spade::physics::lowest_contact_point(model, body);
    ASSERT_TRUE(low.has_value()) << "the model has no contact set";
    EXPECT_NEAR(-*low, kDesignOriginAtRest, 2.0e-3f) << "the editor would place the origin " << -*low << " m over the floor; the drone rests with it " << kDesignOriginAtRest << " m over";
}
