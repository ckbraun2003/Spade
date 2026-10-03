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

#include <cstdio>
#include <filesystem>

#include <glm/vec4.hpp>
#include <glm/vec3.hpp>

#include "scene/compose.hpp"
#include "sim/simulation.hpp"
#include "sim/world_set.hpp"
#include "world/builder.hpp"
#include "world/medium.hpp"
#include "world/world_file.hpp"

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
        std::filesystem::temp_directory_path() / "spade_consumer_smoke_world.yaml";

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

#if SPADE_CONSUMER_HAS_RENDER_GL
    // The optional spade::render_gl, when this prefix installed it (gl_check.cpp).
    if (check_render_gl() != 0) {
        return 1;
    }
#endif
    return 0;
}
