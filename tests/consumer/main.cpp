// Out-of-tree find_package(spade) consumer smoke (S5 Task 8, extended by the
// Task 5/8 integration follow-up: the yaml-cpp consumer-link closure).
//
// This is deliberately NOT a test of Spade's physics -- test_m1b_bar.cpp and
// the rest of tests/ already do that, in-tree, against spade_tests
// linked directly to the engine's build-tree targets. What this program
// proves is narrower and different: that find_package(spade CONFIG REQUIRED)
// resolves from an INSTALLED tree (spadeConfig.cmake + spadeTargets.cmake,
// engine/CMakeLists.txt), that spade::sim/spade::vehicles link and run
// correctly once installed, and that the consumer's own glm and yaml-cpp
// obligations (see cmake/spadeConfig.cmake.in) are satisfiable the way
// that file documents. See CMakeLists.txt in this directory for how it is
// configured against a scratch install prefix.
//
// The self-run Docker leg (TD-11) runs this program through
// scripts/consumer-smoke.sh, against a prefix installed with SPADE_VULKAN OFF
// and again with it ON. Before that leg existed (2026-09-18 to 2026-10-03),
// nothing ran it.
//
// THE WORLD-FILE ROUND TRIP BELOW IS LOAD-BEARING, NOT DECORATION: the
// original version of this file built a WorldDesc directly via WorldBuilder
// and never called into world/world_file.hpp at all. A static archive only
// extracts the .obj members it needs to resolve an otherwise-undefined
// symbol, so world_file.obj -- and therefore spade_world's yaml-cpp
// dependency -- was NEVER PULLED from the installed spade_world.lib, and this
// smoke passed while the yaml-cpp consumer-link gap sat there undetected
// (the defect Task 5's opus review surfaced). Calling save_world_file()/
// load_world_file() here is what actually forces the linker to need
// yaml-cpp's symbols, which is what makes this program's link outcome mean
// anything about that closure.
//
// Returns 0 on success, non-zero (with a message on stderr) on any failure --
// that exit code is the thing CI actually checks.

#include <cstddef>
#include <cstdio>
#include <filesystem>
#include <span>
#include <string>
#include <string_view>
#include <vector>

#include <glm/vec4.hpp>
#include <glm/vec3.hpp>
#include <glm/gtc/quaternion.hpp>

#include "analysis/case.hpp"
#include "analysis/launch.hpp"
#include "render/scene.hpp"
#include "render_parts/render_parts.hpp"
#include "scene/compose.hpp"
#include "scene/model_file.hpp"
#include "scene/scene_file.hpp"
#include "sim/simulation.hpp"
#include "sim/world_set.hpp"
#include "world/builder.hpp"
#include "world/medium.hpp"
#include "world/mesh_import.hpp"
#include "world/world_file.hpp"
#include "../unique_temp.hpp"  // the repo's per-process temp names; std-only

#if SPADE_CONSUMER_HAS_RENDER_GL
int check_render_gl();  // gl_check.cpp
#endif

int main() {
    // One world: a ground plane, nothing else -- enough for a body to exist
    // and step under gravity.
    const spade::Result<spade::WorldDesc> authored = spade::WorldBuilder()
                                                          .name("consumer_smoke")
                                                          .environment(spade::Environment{})
                                                          .capacities(spade::Capacities{4, 4, 1, 1})
                                                          .plane(glm::vec3(0.0f, 1.0f, 0.0f), 0.0f)
                                                          .build();
    if (!authored) {
        std::fprintf(stderr, "WorldBuilder::build failed: %s\n", authored.error().context.c_str());
        return 1;
    }

    // Round-trip it through a real world file -- see the header comment above
    // for why this call is the whole point of this integration follow-up.
    const std::filesystem::path world_file_path =
        spade::test::unique_temp_path("consumer_smoke_world.yaml");

    if (const spade::Result<void> saved = spade::save_world_file(*authored, world_file_path); !saved) {
        std::fprintf(stderr, "save_world_file failed: %s\n", saved.error().context.c_str());
        return 1;
    }

    const spade::Result<spade::WorldDesc> world = spade::load_world_file(world_file_path);
    std::error_code cleanup_ec;
    std::filesystem::remove(world_file_path, cleanup_ec);  // best-effort; not load-bearing
    if (!world) {
        std::fprintf(stderr, "load_world_file failed: %s\n", world.error().context.c_str());
        return 1;
    }

    spade::WorldInstanceDesc prototype;
    prototype.world = *world;  // the ROUND-TRIPPED desc, not the one WorldBuilder produced directly
    prototype.turbulence = spade::dryden_params(spade::TurbulenceLevel::light);
    prototype.contacts.restitution_e = 0.0f;
    prototype.contacts.friction_mu = 0.0f;
    prototype.contacts.proxy_radius = 0.1f;
    prototype.grid.cell_size = 0.5f;

    // replicate(): the training-fleet constructor (sim/world_set.hpp) -- two
    // worlds from one prototype, each with its own derived rng root. This is
    // the "world_set" half of the smoke: a real consumer builds a fleet, not
    // one lone world.
    const spade::WorldSetDesc set = spade::replicate(prototype, /*count=*/2, /*scene_seed=*/0xC0FFEEULL);

    spade::Result<spade::Simulation> sim = spade::Simulation::create(set, /*dt_ns=*/2'000'000, /*substeps=*/2);
    if (!sim) {
        std::fprintf(stderr, "Simulation::create failed: %s\n", sim.error().context.c_str());
        return 1;
    }

    spade::BodySpawn body;
    body.pos = glm::vec3(0.0f, 5.0f, 0.0f);
    body.mass = 1.0f;
    body.inv_inertia_diag = glm::vec3(10.0f);
    const spade::Result<spade::BodyRef> ref = sim->spawn(/*world_index=*/0, body);
    if (!ref) {
        std::fprintf(stderr, "Simulation::spawn failed: %s\n", ref.error().context.c_str());
        return 1;
    }

    if (const spade::Result<void> stepped = sim->step(100); !stepped) {
        std::fprintf(stderr, "Simulation::step failed: %s\n", stepped.error().context.c_str());
        return 1;
    }

    const spade::Result<const spade::BodyState*> state = sim->body(*ref);
    if (!state) {
        std::fprintf(stderr, "Simulation::body failed: %s\n", state.error().context.c_str());
        return 1;
    }

    std::printf(
        "consumer smoke OK: world_file_round_trip_name=\"%s\" tick=%llu world_count=%u "
        "pos=(%.6f, %.6f, %.6f) vel=(%.6f, %.6f, %.6f)\n",
        world->name.c_str(), static_cast<unsigned long long>(sim->tick().value), sim->world_count(),
        static_cast<double>((*state)->pos.x), static_cast<double>((*state)->pos.y),
        static_cast<double>((*state)->pos.z), static_cast<double>((*state)->vel.x),
        static_cast<double>((*state)->vel.y), static_cast<double>((*state)->vel.z));

    // spade::scene, installed: an asset at the identity pose leaves its
    // collider's transform unchanged (compose_transform's short-cut (a)).
    spade::SdfTransform collider{};
    collider.world_to_local[3] = glm::vec4(1.0f, 2.0f, 3.0f, 1.0f);
    collider.scale = 2.0f;
    const spade::SdfTransform posed = spade::scene::compose_transform(collider, spade::SdfTransform{});
    if (!(posed.world_to_local == collider.world_to_local && posed.scale == collider.scale)) {
        std::fprintf(stderr, "spade::scene::compose_transform changed a collider at the identity pose\n");
        return 1;
    }
    std::printf("spade::scene OK: compose_transform kept the collider at the identity pose\n");

    // The scene file, installed. Parsing a scene pulls scene_file.o out of the
    // installed spade_scene archive, and with it spade_scene's own need for
    // yaml-cpp: the world-file round trip's reason, above, one target up.
    constexpr const char* kScene =
        "scene_version: 1\n"
        "name: \"consumer_smoke\"\n"
        "world: {file: \"consumer_smoke.world.yaml\", hash: \"0x0000000000000001\"}\n"
        "materials: []\n"
        "models: []\n"
        "assets: [{name: \"marker\", pose: {position: [0, 0, 0], orientation: [1, 0, 0, 0], scale: 1},\n"
        "          collider: {sdf: {transforms: [], nodes: [], node_materials: []}},\n"
        "          visual: {mesh_ref: \"marker\", material: \"default\"}}]\n"
        "vehicles: []\n";
    const spade::Result<spade::scene::SceneDesc> scene = spade::scene::scene_from_yaml(kScene);
    if (!scene) {
        std::fprintf(stderr, "spade::scene::scene_from_yaml failed: %s\n", scene.error().context.c_str());
        return 1;
    }
    const spade::Result<std::string> scene_text = spade::scene::scene_to_yaml(*scene);
    if (!scene_text || !spade::scene::scene_from_yaml(*scene_text)) {
        std::fprintf(stderr, "spade::scene::scene_to_yaml did not round-trip the scene\n");
        return 1;
    }
    std::printf("spade::scene OK: a scene file round-tripped through scene_from_yaml and scene_to_yaml\n");

    // The model file, installed: one model and its propulsion chain, as Kat's
    // builder writes one. It shares the scene's model block through
    // scene/detail/, which is not installed, so this is what proves the
    // installed header needs nothing from there.
    spade::scene::ModelFile model_file;
    model_file.model.name = "consumer_quad";
    model_file.model.body.mass = 1.0f;
    model_file.model.body.inertia_diag = glm::vec3(0.01f);
    spade::vehicles::PropulsionChain chain;
    chain.kv = 256.56340004316644;
    chain.resistance = 0.075;
    chain.diameter = 0.1524;
    chain.table.j_max = 0.8f;
    chain.table.ct = {0.12f, 0.04f};
    chain.table.cq = {0.008f, 0.004f};
    chain.cells_series = 4;
    chain.cells_parallel = 1;
    chain.ocv_table = {3.3f, 4.2f};
    model_file.propulsion = chain;
    const spade::Result<std::string> model_text = spade::scene::model_to_yaml(model_file);
    if (!model_text) {
        std::fprintf(stderr, "spade::scene::model_to_yaml failed: %s\n", model_text.error().context.c_str());
        return 1;
    }
    const spade::Result<spade::scene::ModelFile> model_back = spade::scene::model_from_yaml(*model_text);
    if (!model_back || !model_back->propulsion ||
        spade::scene::model_to_yaml(*model_back).value_or("") != *model_text) {
        std::fprintf(stderr, "spade::scene::model_from_yaml did not round-trip the model file\n");
        return 1;
    }
    std::printf("spade::scene OK: a model file round-tripped through model_to_yaml and model_from_yaml\n");

    // spade::analysis (INT-6): a case through its file, as Kat's builder
    // reads and writes one. Calling into case_file.cpp is what pulls it from
    // the archive (the note at the top of this file).
    spade::analysis::AnalysisCase analysis_case;
    analysis_case.controls.armed = true;
    analysis_case.controls.throttle = 0.5f;
    const spade::Result<std::string> case_text = spade::analysis::case_to_yaml(analysis_case);
    if (!case_text) {
        std::fprintf(stderr, "spade::analysis::case_to_yaml failed: %s\n", case_text.error().context.c_str());
        return 1;
    }
    const spade::Result<spade::analysis::AnalysisCase> case_back = spade::analysis::case_from_yaml(*case_text);
    if (!case_back || spade::analysis::case_to_yaml(*case_back).value_or("") != *case_text) {
        std::fprintf(stderr, "spade::analysis did not round-trip a case\n");
        return 1;
    }
    std::printf("spade::analysis OK: a case round-tripped through case_to_yaml and case_from_yaml\n");

    // spade::render_parts (RND-9): a model's part posed and bound into a render
    // scene, as Kat's host draws a drone. Calling prepare_parts() and
    // bind_parts() pulls them from the installed archive.
    {
        spade::vehicles::ModelType model;
        model.name = "consumer_parts";
        spade::vehicles::PartDesc hull;
        hull.name = "hull";
        hull.shape.transforms.push_back(spade::SdfTransform{});
        spade::SdfNode box{};
        box.kind = static_cast<uint32_t>(spade::SdfPrim::box);
        box.params = glm::vec4(0.05f, 0.02f, 0.05f, 0.0f);
        hull.shape.nodes.push_back(box);
        hull.min_feature = 0.001f;
        model.parts.push_back(hull);
        spade::BodyState body{};
        body.orient = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
        const auto posed = spade::vehicles::posed_parts(model, body, {}, {}, spade::vehicles::PartFilter::all);
        const auto prepared = spade::render_parts::prepare_parts(model);
        spade::render::RenderScene drawn;
        drawn.materials.push_back(spade::render::Material{});
        if (!posed || !prepared || !spade::render_parts::bind_parts(*prepared, *posed, {}, drawn) ||
            drawn.statics.size() != 1u) {
            std::fprintf(stderr, "spade::render_parts did not bind a posed part\n");
            return 1;
        }
    }
    std::printf("spade::render_parts OK: a posed part bound into a render scene\n");

    // spade::mesh_import, installed. Reading a glTF pulls the reader's objects
    // out of the installed spade_mesh_import archive (linking alone pulls
    // none): a closed tetrahedron, its buffer a base64 data: URI holding four
    // VEC3 FLOAT positions (48 bytes) and twelve UNSIGNED_SHORT indices.
    constexpr std::string_view kTetrahedron = R"({"asset": {"version": "2.0"}, "scene": 0,
        "scenes": [{"nodes": [0]}], "nodes": [{"name": "tetra", "mesh": 0}],
        "meshes": [{"primitives": [{"attributes": {"POSITION": 0}, "indices": 1}]}],
        "accessors": [{"bufferView": 0, "componentType": 5126, "count": 4, "type": "VEC3"},
                      {"bufferView": 1, "componentType": 5123, "count": 12, "type": "SCALAR"}],
        "bufferViews": [{"buffer": 0, "byteOffset": 0, "byteLength": 48},
                        {"buffer": 0, "byteOffset": 48, "byteLength": 24}],
        "buffers": [{"byteLength": 72, "uri": "data:application/octet-stream;base64,AAAAAAAAAAAAAAAAAACAPwAAAAAAAAAAAAAAAAAAgD8AAAAAAAAAAAAAAAAAAIA/AAACAAEAAAABAAMAAAADAAIAAQACAAMA"}]})";
    const spade::Result<std::vector<spade::MeshObject>> objects =
        spade::read_gltf_bytes(std::as_bytes(std::span(kTetrahedron.data(), kTetrahedron.size())), {});
    if (!objects) {
        std::fprintf(stderr, "spade::read_gltf_bytes failed: %s\n", objects.error().context.c_str());
        return 1;
    }
    if (objects->size() != 1 || (*objects)[0].mesh->triangle_count() != 4) {
        std::fprintf(stderr, "spade::read_gltf_bytes read %zu objects, not one tetrahedron of 4 triangles\n",
                     objects->size());
        return 1;
    }
    std::printf("spade::mesh_import OK: a glTF tetrahedron read as one closed mesh of 4 triangles\n");

    // The installed analysis runner (INT-7): spade::analysis_runner names the
    // file the install put in bin/. launch_run() takes it as the runner, then
    // refuses a folder that holds no case; the call pulls launch.cpp and the
    // process code from the archive.
    const std::filesystem::path runner = SPADE_ANALYSIS_RUNNER;
    if (!std::filesystem::is_regular_file(runner)) {
        std::fprintf(stderr, "the installed spade_analysis_runner is missing: %s\n", runner.string().c_str());
        return 1;
    }
    spade::analysis::LaunchOptions dry_run;
    dry_run.kind = spade::analysis::RunKind::dry_run;
    const spade::Result<spade::analysis::RunRecord> launched =
        spade::analysis::launch_run(runner, spade::test::unique_temp_path("consumer_smoke_no_run"), dry_run);
    if (launched || launched.error().context.find("case.yaml") == std::string::npos) {
        std::fprintf(stderr, "spade::analysis::launch_run did not refuse a folder without a case\n");
        return 1;
    }
    std::printf("spade::analysis OK: the installed runner is in bin/, and launch_run refused a folder without a case\n");

#if SPADE_CONSUMER_HAS_RENDER_GL
    // The optional spade::render_gl, when this prefix installed it (gl_check.cpp).
    if (check_render_gl() != 0) {
        return 1;
    }
#endif
    return 0;
}
