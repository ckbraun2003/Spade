
# ------------------------------------------------------------------------------
# GLFW -- v1's viewer AND (Plan C task C2) the v2 sandbox's windowed TargetSink.
#
# ⚠ THIS BLOCK USED TO BE GATED ON SPADE_BUILD_V1 ALONE, and its stated reason
# was: "a bare Linux CI runner has no X11/Wayland dev packages installed, and
# FetchContent_MakeAvailable(glfw) would otherwise fail its own CMake
# configure." THAT REASON WAS TRUE AND THE CONCLUSION DRAWN FROM IT WAS NOT.
#
# MEASURED 2026-09-18 on the real CI image (kat-ci-linux:local, which has
# X11=no GL=no EGL=no), three probes each paired with a control that had to
# fail:
#
#   GLFW with GLFW_BUILD_X11=OFF GLFW_BUILD_WAYLAND=OFF  -> configure 0,
#       build 0, libglfw3.a linked.  CONTROL with X11=ON -> FAILED at
#       find_package, which is what proves the probe could see the absence.
#   imgui_impl_opengl3.cpp standalone                     -> status 0. Its
#       bundled loader (SL11's claim) confirmed BY COMPILATION.
#   imgui_impl_glfw.cpp with -DGLFW_INCLUDE_NONE          -> status 0.
#       CONTROL without the macro -> FAILED: glfw3.h:241 pulls <GL/gl.h>.
#
# So GLFW builds on a bare image with NO dev packages at all, and the fetch
# never had to be gated. ⭐ A CONFIGURE FAILURE UNDER THE DEFAULT SETTINGS WAS
# READ AS A DEPENDENCY ON THE PLATFORM, when it was a dependency on a DEFAULT
# -- and the default is ours to set.
#
# ⚠ AND THE BACKEND CHOICE IS DETECTED, NOT ASSUMED. Hardcoding the backends
# OFF would make every LINUX DEVELOPER's window silently unopenable to buy a CI
# green. X11 is probed; if it is there the backend is built, and the sandbox is
# told either way so it can REFUSE with the real reason rather than failing
# obscurely at glfwInit().
# ------------------------------------------------------------------------------
if(SPADE_BUILD_V1 OR SPADE_BUILD_SANDBOX)
FetchContent_Declare(
    glfw
    GIT_REPOSITORY https://github.com/glfw/glfw.git
    GIT_TAG        3.4
)
set(GLFW_BUILD_DOCS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(GLFW_INSTALL OFF CACHE BOOL "" FORCE)

if(UNIX AND NOT APPLE)
    find_package(X11 QUIET)
    if(X11_FOUND)
        set(SPADE_GLFW_HAS_BACKEND 1 CACHE INTERNAL "GLFW can open a window on this platform")
        message(STATUS "spade: X11 found -- GLFW builds its X11 backend, windows can open")
    else()
        set(SPADE_GLFW_HAS_BACKEND 0 CACHE INTERNAL "GLFW can open a window on this platform")
        message(STATUS "spade: no X11 dev headers -- GLFW builds WITHOUT a window backend. "
                       "The windowed sink will COMPILE and LINK; it will refuse at runtime, by design.")
    endif()
    set(GLFW_BUILD_X11 ${X11_FOUND} CACHE BOOL "" FORCE)
    set(GLFW_BUILD_WAYLAND OFF CACHE BOOL "" FORCE)
else()
    # Win32 needs no dev packages -- the backend is part of the SDK.
    set(SPADE_GLFW_HAS_BACKEND 1 CACHE INTERNAL "GLFW can open a window on this platform")
endif()

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
# ImGui -- v1's viewer UI layer AND (Plan C task C2) the v2 sandbox's windowed
# TargetSink. Pinned to the v1.91.5-docking release tag (was tracking the
# floating "docking" branch, which is not reproducible across builds). Depends
# on the glfw target declared above, so it follows the same guard.
# ------------------------------------------------------------------------------
if(SPADE_BUILD_V1 OR SPADE_BUILD_SANDBOX)
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

    # ⚠ PRIVATE, NOT PUBLIC, AND THE DISTINCTION IS LOAD-BEARING.
    # GLFW_INCLUDE_NONE stops glfw3.h pulling <GL/gl.h> (glfw3.h:241), which is
    # what lets imgui_impl_glfw.cpp compile on an image with no GL headers --
    # measured, with a control that failed without it. ImGui's OpenGL3 backend
    # brings its own loader, so nothing here needs the system header.
    #
    # PUBLIC would propagate the macro to every consumer, including v1's viewer,
    # which includes glfw3.h expecting GL declarations to arrive with it. That
    # would fix this target by breaking a target that is not in question -- the
    # header-fanout lesson from task C1, arriving one layer up in the build
    # graph instead of in a source file.
    target_compile_definitions(imgui PRIVATE GLFW_INCLUDE_NONE)
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

# ------------------------------------------------------------------------------
# nlohmann/json (header-only): render/gltf.cpp's glTF 2.0 parser (S7a Task
# R4). Same v3.12.0 pin the repo ROOT CMakeLists.txt already uses for its own
# separate build (runtime/, editor/) -- this is a DIFFERENT CMake project
# (spade/ configures standalone, spade/CMakePresets.json's own binaryDir),
# so that pin buys nothing here automatically and has to be declared again;
# keeping the version identical is deliberate, not load-bearing, so the two
# trees' JSON behavior never has a reason to drift. No EXCLUDE_FROM_ALL: the
# upstream CMakeLists.txt produces a single INTERFACE target
# (nlohmann_json::nlohmann_json) with nothing compiled and no test/tool
# targets to keep out of the default `all` target, unlike benchmark/yaml-cpp
# above.
# ------------------------------------------------------------------------------
set(JSON_BuildTests OFF CACHE BOOL "" FORCE)
FetchContent_Declare(
    nlohmann_json
    GIT_REPOSITORY https://github.com/nlohmann/json.git
    GIT_TAG        v3.12.0
    GIT_SHALLOW    TRUE
)
FetchContent_MakeAvailable(nlohmann_json)

# ------------------------------------------------------------------------------
# Vulkan-Headers (S6 Task 1) -- the v2 compute backend's device-layer types
# (engine/compute/vulkan/context.*). Fetched UNCONDITIONALLY, not gated on
# SPADE_VULKAN: the fetch itself is cheap (a header-only git checkout, no
# compiled TUs), and the existing pattern in this file is to gate the
# *consuming target*, not the FetchContent call -- see GLM's own comment
# above (fetched unconditionally even though only v1 needs it standalone).
# engine/CMakeLists.txt's spade_compute target, the actual consumer, is what
# `if(SPADE_VULKAN)`-gates.
#
# PINNED VERSION AND WHY IT IS NOT "v1.3.*"-NAMED, DESPITE THIS TASK'S BRIEF
# ASKING FOR "an exact v1.3.* tag": KhronosGroup/Vulkan-Headers tagged
# "v1.3.NNN" through v1.3.302 (2024-11-21), then switched entirely to
# "vulkan-sdk-X.Y.Z.W" tags once Vulkan 1.4 shipped -- there has been no new
# "v1.3.*" tag since. Every "vulkan-sdk-*" tag is CUMULATIVE (it carries the
# full VK_API_VERSION_1_0..1_4 surface; nothing 1.3-era is removed). The
# coordinator's own dispatch for this task anticipated exactly this
# situation and ruled on it directly (not a quote from global-constraints.md,
# which does not contain this sentence): "Vulkan-Headers can be newer than
# the driver's 1.3.215 -- target the 1.3 API in code." Pinning the STALE
# (21-month-old) v1.3.302 tag over the current cumulative one would trade
# real freshness for a naming convention this upstream repo itself
# abandoned. This program's own code only calls the VK_API_VERSION_1_3
# surface (engine/compute/vulkan/context.cpp's VkApplicationInfo::apiVersion),
# so "target the 1.3 API in code" holds regardless of which cumulative tag
# supplies the headers.
#
# Chosen via `gh api repos/KhronosGroup/Vulkan-Headers/tags` (current stable
# HEAD tag at pin time, 2026-08-13).
# Release: https://github.com/KhronosGroup/Vulkan-Headers/releases/tag/vulkan-sdk-1.4.357.0
# ------------------------------------------------------------------------------
FetchContent_Declare(
    vulkan-headers
    GIT_REPOSITORY https://github.com/KhronosGroup/Vulkan-Headers.git
    GIT_TAG        vulkan-sdk-1.4.357.0
)
set(VULKAN_HEADERS_ENABLE_TESTS OFF CACHE BOOL "" FORCE)
set(VULKAN_HEADERS_ENABLE_INSTALL OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(vulkan-headers)

# ------------------------------------------------------------------------------
# volk (S6 Task 1) -- the dynamic Vulkan function-pointer loader
# engine/compute/vulkan/context.cpp calls (volkInitialize/volkLoadInstanceOnly/
# volkLoadDevice); no vulkan-1.lib/libvulkan.so is linked anywhere in this
# tree, so a box or CI runner with no Vulkan loader installed still links,
# and simply reports Code::unavailable / vulkan_available() == false at
# runtime rather than failing to link at all.
#
# "MATCHING tag" (this task's brief): zeux/volk tags its own releases 1:1
# against the identical vulkan-sdk-X.Y.Z.W string Vulkan-Headers uses above
# (verified empirically -- both repos' `gh api .../tags` listings are
# identical) -- "vulkan-sdk-1.4.357.0" here names the exact volk release cut
# against the exact Vulkan-Headers commit fetched above, so the two
# dependencies' header surfaces cannot drift apart.
#
# VOLK_PULL_IN_VULKAN (volk's own CMakeLists.txt, default ON) auto-detects
# the in-tree `Vulkan-Headers` target declared just above (it checks `TARGET
# Vulkan-Headers` once no VULKAN_SDK env var or find_package(Vulkan) result
# is available) and adds its include directory to both the `volk` and
# `volk_headers` targets -- this Declare/MakeAvailable pair MUST stay after
# vulkan-headers's above for that detection to see the target.
#
# Release: https://github.com/zeux/volk/releases/tag/vulkan-sdk-1.4.357.0
# ------------------------------------------------------------------------------
FetchContent_Declare(
    volk
    GIT_REPOSITORY https://github.com/zeux/volk.git
    GIT_TAG        vulkan-sdk-1.4.357.0
)
FetchContent_MakeAvailable(volk)

# ------------------------------------------------------------------------------
# Slang prebuilt compiler (S6 Task 1) -- the shader-compile-time toolchain a
# LATER task (T3 onward, per this task's brief) invokes to turn .slang
# source into SPIR-V. Nothing in Spade LINKS slang as a library, so this
# fetches a PREBUILT RELEASE ARCHIVE (URL + URL_HASH), unlike every other
# entry in this file, which builds a GIT_REPOSITORY checkout from source:
# slang's own build is a multi-hour LLVM-adjacent undertaking wholly out of
# scope for a per-configure fetch, and upstream already publishes the
# compiler binaries this tree needs.
#
# Pinned to v2026.14.1 -- the release `gh api
# repos/shader-slang/slang/releases/latest` resolved to at pin time
# (2026-08-13, published 2026-07-30).
# Release: https://github.com/shader-slang/slang/releases/tag/v2026.14.1
#
# Per-platform archive + hash. shader-slang publishes no separate checksum
# manifest for a release, so each SHA256 below was computed locally
# (`sha256sum`) from the asset downloaded directly off this release's GitHub
# URL -- the same integrity guarantee URL_HASH exists to encode, just
# computed rather than copied.
#   windows-x64: slang-2026.14.1-windows-x86_64.zip
#     SHA256 5ed0a59d650a0af0aca45d5db4e083b3d8fb5cea05748747dd95dfbe9c580658
#   linux-x64:   slang-2026.14.1-linux-x86_64.tar.gz (the glibc-generic
#     build, not the -glibc-2.27/-2.28-suffixed variants -- matches both
#     this box's own x86_64 dev loop and .github/workflows/spade.yml's
#     ubuntu-latest runner without pinning to a specific glibc ABI)
#     SHA256 21f2d7847385a770e569fb61b1507a7794d742d97850bce0432bff0032ca005f
#
# Unconditional fetch, same reasoning as Vulkan-Headers/volk above. The
# archive has no CMakeLists.txt at its root, so FetchContent_MakeAvailable
# populates ${slang_SOURCE_DIR} (bin/, lib/, include/) WITHOUT calling
# add_subdirectory -- exactly what a prebuilt-binary vendor drop needs; a
# later task's CMake code is what will actually reference
# ${slang_SOURCE_DIR}/bin/slangc(.exe) to compile SPIR-V, and this task
# leaves the variable there unconsumed, correctly out of its own scope.
#
# SPADE_SLANGC (S6 Task 3) is that later task's consumption of it, and it is
# defined HERE, immediately after the fetch that produces the binary, rather
# than in cmake/SpadeSlang.cmake where it is used: ${slang_SOURCE_DIR} is a
# FetchContent output variable, only meaningful in the scope that called
# FetchContent_MakeAvailable, and re-deriving the archive's internal layout
# (bin/slangc) at the consumer would put knowledge of the vendor drop's shape
# in two files. CACHE INTERNAL, so it survives into every subdirectory scope
# (engine/, tests/) without being a user-facing knob -- the compiler is pinned
# by the URL_HASH above, and pointing this at a different slangc would silently
# change the SPIR-V the layout checks were reflected from.
# ------------------------------------------------------------------------------
if(WIN32)
    FetchContent_Declare(
        slang
        URL      https://github.com/shader-slang/slang/releases/download/v2026.14.1/slang-2026.14.1-windows-x86_64.zip
        URL_HASH SHA256=5ed0a59d650a0af0aca45d5db4e083b3d8fb5cea05748747dd95dfbe9c580658
    )
else()
    FetchContent_Declare(
        slang
        URL      https://github.com/shader-slang/slang/releases/download/v2026.14.1/slang-2026.14.1-linux-x86_64.tar.gz
        URL_HASH SHA256=21f2d7847385a770e569fb61b1507a7794d742d97850bce0432bff0032ca005f
    )
endif()
FetchContent_MakeAvailable(slang)

if(WIN32)
    set(_spade_slangc "${slang_SOURCE_DIR}/bin/slangc.exe")
else()
    set(_spade_slangc "${slang_SOURCE_DIR}/bin/slangc")
endif()
if(NOT EXISTS "${_spade_slangc}")
    message(FATAL_ERROR
        "The slang prebuilt was fetched but no slangc was found at ${_spade_slangc}. "
        "The release archive's layout changed, or the fetch is incomplete.")
endif()
set(SPADE_SLANGC "${_spade_slangc}" CACHE INTERNAL
    "The pinned slangc (slang v2026.14.1 prebuilt) that compiles engine/shaders/**")
unset(_spade_slangc)
