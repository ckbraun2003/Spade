# Floor is 3.28, not the 3.15 this used to declare: the true minimum this
# tree already requires, set by two features actually in use --
# BUILD_LOCAL_INTERFACE genexes (CMake 3.24+, engine/CMakeLists.txt's
# per-target warnings/fp-strict links) and vendor/CMakeLists.txt's
# FetchContent_Declare(... EXCLUDE_FROM_ALL) calls (CMake 3.28+, also why
# CMakePresets.json pins cmakeMinimumRequired to 3.28). Declaring the real
# floor here means an older CMake fails at this line with a clear message
# instead of mid-configure with a confusing one.
cmake_minimum_required(VERSION 3.28)

# Version 0.2.0 (Addendum A11(i), engine design spec S15, signed 2026-08-10):
# charter S5's "Spade's own version" promise, discharged at the S4-M1B claim
# (Task 21 close-out). This is Spade-the-project's own release number, not a
# data-format tag, and neither of those is touched by this bump: the
# snapshot format has its own real, existing version today
# (state/snapshot.hpp's kSnapshotVersion, currently 1), and the eventual
# world-FILE format's version is an S5 concept -- the YAML world file does
# not exist yet, so there is no "world_version" symbol anywhere in this tree
# to separately version.
project(Spade VERSION 0.2.0 LANGUAGES CXX C)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_compile_definitions(GLM_ENABLE_EXPERIMENTAL)

include(FetchContent)

# v1 (src/, include/, examples/, assets/) is frozen -- this option only
# decides whether it gets built, never how. THE ONE EXCEPTION: Task 3's
# dead-code deletions -- e.g. include/Spade/Core/Engine.hpp's
# EnableBruteForceNewtonianGravity, a method whose shader existed but whose
# C++ body was never implemented, deleted outright rather than kept as
# permanent dead weight (this is the exception clause README.md's v1
# section points back to). Default ON so today's behavior is unchanged;
# Task 4's Linux CI configures with this OFF to build/test the v2 engine
# tree (engine/, tests/) on its own.
option(SPADE_BUILD_V1 "Build the v1 GL engine (src/, include/, examples/, assets/)" ON)

# SPADE_BUILD_SANDBOX (SL15a, Plan C task C0): gates spade_sandbox, the
# engine's reference application. DEFAULT ON, WHICH CI CONFIGURES OFF -- the
# same pattern SPADE_BUILD_V1 uses today and that CI already exercises.
#
# ⚠⚠ THIS FLAG IS *INTRODUCED* ALONGSIDE SPADE_BUILD_V1. IT DOES NOT MOVE
# GLFW OR IMGUI OFF THAT GATE. SL14a specifies that move -- and SL14a is the
# v1 QUARANTINE, which is correctly BLOCKED on SL8's one open transfer-register
# row (SPH fluid). Executing the move here would be executing a quarantine that
# must not execute, while v1 is still the only implementation of the fluid
# solver. So vendor/CMakeLists.txt is untouched by C0 and the two gates coexist
# until Plan B closes that row.
#
# Getting that wrong is not a small cost: the GLFW fetch sits inside the V1
# gate because a bare Linux runner has no X11/Wayland dev packages and
# FetchContent_MakeAvailable(glfw) fails its own configure without them. Moving
# it would fail ALL SIX workflow configurations at CONFIGURE time. Done this
# way the added CI cost is ZERO, because C0 is headless and links no windowing
# library at all -- the Presenter's GL implementation (task C2) is what first
# needs one, and it will need its own fetch under THIS gate, not a moved one.
option(SPADE_BUILD_SANDBOX "Build spade_sandbox, the reference application (SL15a)" ON)

# S6 Task 1: gates the spade_compute TARGET only (engine/CMakeLists.txt) --
# not the vendor Vulkan-Headers/volk/slang FetchContent calls, which fetch
# unconditionally regardless of this option (vendor/CMakeLists.txt's own
# comments explain why: the fetch is cheap, and the existing pattern in this
# tree gates consuming targets, not FetchContent calls -- see
# SPADE_BUILD_V1's GLFW/ImGui/GLAD entries there). Default ON so the compute
# scaffold builds and its tests register (skipping on a device-less runner)
# out of the box; SPADE_VULKAN=OFF is the escape hatch for an environment
# that cannot even fetch Vulkan-Headers/volk (e.g. no network) -- this
# task's report proves an OFF-configure still configures, builds, and
# installs the rest of the tree cleanly, with spade::compute simply absent.
option(SPADE_VULKAN "Build the v2 Vulkan compute backend (engine/compute/vulkan, spade_compute)" ON)

# ------------------------------------------------------------------------------
# SPADE_RENDER_GL -- v2's GPU RENDER backend (engine/render_gl, spade_render_gl).
#
# DEFAULT ON, because the user ruled it: "Rendering should default to using the
# computer gpu, most computers have one, the cpu is the fallback." raster_cpu is
# not replaced -- it keeps three jobs (the no-GPU fallback, the headless/CI
# surface, and the reference any future tolerance band is measured against).
#
# ⚠ THIS IS A SEPARATE MODULE AND NOT PART OF spade_render, ON PURPOSE.
# spade_render is installed, embeddable and has NO windowing or GL dependency;
# folding GL into it would hand that dependency to every consumer of the engine.
# An optional GPU-backed module behind its own flag is exactly the shape
# SPADE_VULKAN already uses for spade_compute, so this applies the established
# pattern rather than inventing one.
#
# OFF gives a build with no GL anywhere in v2 -- which is what a headless
# consumer, and the Linux gate, actually want.
option(SPADE_RENDER_GL "Build the v2 OpenGL render backend (engine/render_gl, spade_render_gl)" ON)

# Function to help with target aliases for consistency
function(create_target_alias target alias)
    if(TARGET ${target} AND NOT TARGET ${alias})
        add_library(${alias} ALIAS ${target})
    endif()
endfunction()

# Output directories
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

# Add vendor dependencies before our modules
add_subdirectory(vendor)

if(SPADE_BUILD_V1)
    # Add source and examples
    add_subdirectory(src)
    add_subdirectory(examples)

    # Pre-existing v1 defect surfaced by this task's build verification, not
    # introduced by it: include/Spade/Core/Engine.hpp includes <windows.h>
    # directly, after <glm/glm.hpp>/<glm/gtc/quaternion.hpp>, so windows.h's
    # unguarded min/max macros shadow calls inside GLM's own headers (e.g.
    # quaternion_exponential.inl) and the v1 target fails to compile. Fixed
    # here (root CMakeLists.txt, not a frozen v1 source) rather than in
    # Engine.hpp itself, and as a target-scoped PUBLIC definition (not a
    # directory-scoped add_compile_definitions) so it reaches Spade + its
    # Sandbox consumer without leaking into the sibling engine/tests trees,
    # which don't need it (v2 doesn't include windows.h).
    target_compile_definitions(Spade PUBLIC NOMINMAX)

    # Copy Assets to Binary Directory
    file(COPY ${CMAKE_SOURCE_DIR}/assets DESTINATION ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})
endif()

# v2 engine tree.
add_subdirectory(engine)

# The reference application. AFTER engine/, because it links spade:: aliases
# that engine/ defines.
if(SPADE_BUILD_SANDBOX)
    add_subdirectory(sandbox)
endif()

enable_testing()
add_subdirectory(tests)
