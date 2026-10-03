# ------------------------------------------------------------------------------
# The sandbox, built against an INSTALLED Spade: the SL2b guard
# (docs/design/interface/plans/2026-10-03-sl2b-guard-plan.md).
#
#   cmake -S sandbox/standalone -B <dir> -DCMAKE_PREFIX_PATH=<installed Spade>
#
# SL2b: the sandbox may include only headers the engine installs and link only
# spade:: targets. In the in-tree build that cannot be checked, because
# spade::core exports the whole engine/ directory as an include root. Here
# Spade comes only from find_package(spade): its include root is the prefix's
# include/, and only spadeTargets exist. An include of an uninstalled header,
# or a link to an unexported target, fails this build.
#
# Third-party dependencies come from the repository's own vendor/, so GLFW,
# ImGui, glm and yaml-cpp cannot drift from the in-tree pins. Spade itself is
# never taken from the source tree. The sandbox's own CMakeLists.txt is added
# unchanged.
#
# Deliberately NOT copied from the root CMakeLists.txt: its global
# GLM_ENABLE_EXPERIMENTAL. A consumer gets only what spadeConfig.cmake gives
# it; if an installed header needs more, that is a finding about the library.
# ------------------------------------------------------------------------------
cmake_minimum_required(VERSION 3.28)
project(spade_sandbox_standalone LANGUAGES CXX C)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
include(FetchContent)

# vendor/ reads these as the root's options. The sandbox needs GLFW and ImGui;
# it needs no v1, no glad and no Vulkan of its own.
set(SPADE_BUILD_V1 OFF)
set(SPADE_BUILD_SANDBOX ON)
set(SPADE_RENDER_GL OFF)
set(SPADE_VULKAN OFF)
add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/../../vendor ${CMAKE_BINARY_DIR}/vendor)

# glm::glm and yaml-cpp::yaml-cpp now exist (vendor/), which spadeConfig.cmake
# requires before it is found.
find_package(spade CONFIG REQUIRED)

# The GPU path is the installed spade::render_gl, when the prefix has it.
if(TARGET spade::render_gl)
    set(SPADE_RENDER_GL ON)
endif()

# Mirror the in-tree build: if the tree that produced this prefix built the
# sandbox's GPU path, the package must offer it too. Otherwise this build would
# quietly drop the GPU path and pass, and hide exactly the SL2b gap it exists
# to find. consumer-smoke.sh passes the source tree's SPADE_RENDER_GL here.
option(SPADE_SANDBOX_EXPECT_GPU "The in-tree sandbox links spade::render_gl" OFF)
if(SPADE_SANDBOX_EXPECT_GPU AND NOT TARGET spade::render_gl)
    message(FATAL_ERROR
        "SL2b: the in-tree sandbox links spade::render_gl, but the installed package "
        "(${spade_DIR}) has no spade::render_gl. The sandbox reaches a module Spade "
        "does not install. Install it (engine/CMakeLists.txt), then run again.")
endif()
message(STATUS "spade_sandbox (standalone): spade from ${spade_DIR}; GPU path "
               "${SPADE_RENDER_GL}; window backend ${SPADE_GLFW_HAS_BACKEND}")

add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/.. ${CMAKE_BINARY_DIR}/sandbox)
