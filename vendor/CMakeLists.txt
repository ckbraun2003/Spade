
# ------------------------------------------------------------------------------
# GLFW (v1 only -- the v2 engine has no windowing/GL surface yet). Gated
# behind SPADE_BUILD_V1: a bare Linux CI runner has no X11/Wayland dev
# packages installed, and FetchContent_MakeAvailable(glfw) would otherwise
# fail its own CMake configure trying to find them, for a target v2 never
# links.
# ------------------------------------------------------------------------------
if(SPADE_BUILD_V1)
FetchContent_Declare(
    glfw
    GIT_REPOSITORY https://github.com/glfw/glfw.git
    GIT_TAG        3.4
)
set(GLFW_BUILD_DOCS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(GLFW_INSTALL OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(glfw)
endif()

# ------------------------------------------------------------------------------
# GLM -- unconditional: spade_core's math_ops.hpp (v2) exposes glm types in
# its public signatures (see spade/engine/CMakeLists.txt), so v2 needs GLM
# even with SPADE_BUILD_V1=OFF.
# ------------------------------------------------------------------------------
FetchContent_Declare(
    glm
    GIT_REPOSITORY https://github.com/g-truc/glm.git
    GIT_TAG        1.0.1
)
set(GLM_TEST_ENABLE OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(glm)

# ------------------------------------------------------------------------------
# ImGui (v1 only -- the v1 viewer's UI layer; nothing in v2 links it).
# Pinned to the v1.91.5-docking release tag (was tracking the floating
# "docking" branch, which is not reproducible across builds). Depends on the
# glfw target declared above, so it stays inside the same SPADE_BUILD_V1
# guard.
# ------------------------------------------------------------------------------
if(SPADE_BUILD_V1)
FetchContent_Declare(
    imgui
    GIT_REPOSITORY https://github.com/ocornut/imgui.git
    GIT_TAG        v1.91.5-docking
    EXCLUDE_FROM_ALL
)
FetchContent_MakeAvailable(imgui)

if(NOT TARGET imgui)
    # Manually define ImGui target
    add_library(imgui STATIC
        ${imgui_SOURCE_DIR}/imgui.cpp
        ${imgui_SOURCE_DIR}/imgui_demo.cpp
        ${imgui_SOURCE_DIR}/imgui_draw.cpp
        ${imgui_SOURCE_DIR}/imgui_tables.cpp
        ${imgui_SOURCE_DIR}/imgui_widgets.cpp
        ${imgui_SOURCE_DIR}/backends/imgui_impl_glfw.cpp
        ${imgui_SOURCE_DIR}/backends/imgui_impl_opengl3.cpp
    )
    target_include_directories(imgui PUBLIC
        ${imgui_SOURCE_DIR}
        ${imgui_SOURCE_DIR}/backends
    )

    # ImGui needs usage of these libraries, so we link them *to* ImGui or provide include paths
    # Just linking glfw here because the backend uses it.
    target_link_libraries(imgui PUBLIC glfw)
endif()
endif()

# ------------------------------------------------------------------------------
# GLAD (v1 only -- the v1 GL loader; v2 has no GL surface).
# Using libigl-glad as a source for pre-generated glad files.
# Pinned to the commit that "master" resolved to on this box (was tracking
# the floating "master" branch, which is not reproducible across builds);
# see task-3-report.md for provenance.
# ------------------------------------------------------------------------------
if(SPADE_BUILD_V1)
FetchContent_Declare(
    glad
    GIT_REPOSITORY https://github.com/libigl/libigl-glad.git
    GIT_TAG        651a425101365aa6e8504988ef9bb363d066c5ee
)
FetchContent_MakeAvailable(glad)
endif()

# ------------------------------------------------------------------------------
# GoogleTest (v2 engine test suite) -- unconditional, needed with
# SPADE_BUILD_V1=OFF.
# ------------------------------------------------------------------------------
FetchContent_Declare(
    googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG        v1.15.2
)
set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

# ------------------------------------------------------------------------------
# google-benchmark (spade_bench, spade/tests/bench/bench_core.cpp, links this
# now; EXCLUDE_FROM_ALL just means benchmark's own internal targets --
# its bundled gtest copy, its `benchmark_main`-less variants, etc. -- still
# don't get pulled into the default `all` target just because the library
# itself does)
# ------------------------------------------------------------------------------
FetchContent_Declare(
    benchmark
    GIT_REPOSITORY https://github.com/google/benchmark.git
    GIT_TAG        v1.9.1
    EXCLUDE_FROM_ALL
)
set(BENCHMARK_ENABLE_TESTING OFF CACHE BOOL "" FORCE)
set(BENCHMARK_ENABLE_INSTALL OFF CACHE BOOL "" FORCE)
set(BENCHMARK_ENABLE_GTEST_TESTS OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(benchmark)

# ------------------------------------------------------------------------------
# yaml-cpp: the versioned YAML world file's parser (engine D7). As of S5 Task 5
# it IS linked -- PRIVATE into spade_world (engine/CMakeLists.txt), whose
# world/world_file.cpp is its only consumer; no yaml-cpp type appears in any
# Spade header. EXCLUDE_FROM_ALL stays and still earns its place: it keeps
# yaml-cpp's own auxiliary targets out of the default `all` target, while the
# library itself is now built on demand because a target depends on it.
# ------------------------------------------------------------------------------
# 0.8.0's own CMakeLists.txt declares cmake_minimum_required(VERSION 2.8.12),
# which current CMake (compatibility below 3.5 removed) rejects outright.
# interface/cmake/FetchDependencies.cmake carries the identical fix for the
# identical dependency; mirrored here rather than re-derived.
set(CMAKE_POLICY_VERSION_MINIMUM 3.5 CACHE STRING "" FORCE)
FetchContent_Declare(
    yaml-cpp
    GIT_REPOSITORY https://github.com/jbeder/yaml-cpp.git
    GIT_TAG        0.8.0
    EXCLUDE_FROM_ALL
)
set(YAML_CPP_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(YAML_CPP_BUILD_TOOLS OFF CACHE BOOL "" FORCE)
set(YAML_CPP_BUILD_CONTRIB OFF CACHE BOOL "" FORCE)
set(YAML_CPP_INSTALL OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(yaml-cpp)
