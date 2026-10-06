# Spade v2 engine tree. Modules live one-per-subdirectory (core/, state/,
# world/, physics/, sim/, sensors/, vehicles/ -- see README.md's module map
# for the full, current list), each its own CMake target so dronesim can
# depend on exactly what it needs via add_subdirectory or find_package(spade).

# ------------------------------------------------------------------------------
# Shared warnings-as-errors flags. Defined once here so tests/ and every
# compiled module (spade_core below, future ones) opt in without repeating
# the compiler-specific flag lists.
# ------------------------------------------------------------------------------
add_library(spade_warnings INTERFACE)
target_compile_options(spade_warnings INTERFACE
    $<$<CXX_COMPILER_ID:MSVC>:/W4 /WX>
    $<$<NOT:$<CXX_COMPILER_ID:MSVC>>:-Wall -Wextra -Wpedantic -Werror>
)

# ------------------------------------------------------------------------------
# spade_fp_strict: -ffp-contract=off, compiler-guarded to non-MSVC, on every
# target whose pinned fp32 op order is part of the CPU<->GPU determinism
# contract (D1/D11; global constraint "fp32 only in engine state & math") --
# a follow-up from Task 9's review (bit-parity must not be optimizer-
# discretionary). MSVC's default here is /fp:precise (this whole tree's
# compile mode -- no /fp:fast anywhere in it), which never fuses a multiply-
# add on its own, so this flag changes NOTHING on this box; gcc/clang default
# to FAST contraction (-ffp-contract=fast) even under -Wall -Wextra -Wpedantic
# -Werror, silently letting the optimizer fuse a*b+c into one rounding step
# where the pinned op order says two. That is exactly the kind of last-bit
# divergence the golden digest corpus (tests/golden/scenarios/*.scenario.yaml)
# exists to catch,
# and until .github/workflows/spade.yml's Linux job runs for the first time
# at this task's PR, it has never actually been tested on a non-MSVC
# compiler -- see this task's report for that disposition.
#
# SCOPE, DECIDED AND JUSTIFIED HERE RATHER THAN LEFT IMPLICIT: every compiled
# target that evaluates pinned-order fp32 EXPRESSIONS feeding the digest.
# Linked (BUILD_LOCAL_INTERFACE, alongside spade_warnings) into spade_core
# (math_ops.cpp: the exp-map orientation update), spade_world (sdf.cpp/
# builder.cpp/medium.cpp -- both the SDF evaluator's and the Dryden filter's
# own file comments call out a pinned op order), spade_physics (integrator/
# contacts/grid/forces.cpp -- the substep passes themselves, the narrowest
# reading of "physics-critical" and the task's stated minimum), spade_sim
# (schedule.cpp's pass orchestration, sensors/imu.cpp's noise/bias synthesis,
# sim/*.cpp's wrench application -- all pinned-order fp32 arithmetic that
# folds into the digest same as physics/'s own), and spade_vehicles (rotor.cpp/
# quadrotor.cpp/model_type.cpp -- momentum-theory inflow and RPM lag, again
# pinned-order fp32). NOT spade_state: arenas.cpp/registry.cpp/snapshot.cpp
# copy and account for BYTES (memcpy, offsets, counters) and evaluate no
# floating-point expression a contraction could touch, so linking this in
# would be a no-op wearing the label of coverage.
#
# spade_tests AND spade_bench ARE ALSO LINKED (tests/CMakeLists.txt) --
# an earlier version of this comment excluded them, on a justification that
# does not survive contact with what those TUs actually do. Test code
# RE-DERIVES REFERENCE ARITHMETIC of its own: test_dryden.cpp and
# test_rotor.cpp carry EXPECT_FLOAT_EQ assertions (25 of them in
# test_rotor.cpp alone) at 4-ULP tolerance, computed from the same pinned
# fp32 formulas the library re-implements independently as ground truth --
# if the test's OWN computation contracts on gcc/clang while the library's
# does not, a "wrong" value could still pass a 4-ULP-wide assertion for the
# wrong reason, or a genuinely correct library value could fail against a
# reference the compiler quietly perturbed. Test/bench code also COMPILES
# HEADER-ONLY ENGINE CODE INLINE INTO ITS OWN TU -- core/rng.hpp's
# next_gauss, sensors/rings.hpp, state/layout.hpp are all header-only and
# therefore compiled under spade_tests'/spade_bench's OWN flags, not the
# library's, wherever a test includes them directly. Both are real fp32
# expressions the harnesses evaluate themselves, not merely calls into a
# pre-built library -- guarding the five libraries above does not guard
# either of these. This is exactly the risk the T9-C4 ledger note named
# before this PR's Linux job ever ran gcc for the first time. The one
# remaining, correctly-justified exclusion is spade_viewer: presentation,
# gated behind SPADE_BUILD_V1, and outside the CPU twin's own determinism
# contract -- it renders what Simulation computed, it does not recompute
# any of it.
# ------------------------------------------------------------------------------
add_library(spade_fp_strict INTERFACE)
target_compile_options(spade_fp_strict INTERFACE
    $<$<NOT:$<CXX_COMPILER_ID:MSVC>>:-ffp-contract=off>
)

# ------------------------------------------------------------------------------
# spade_core: typed handles (ids.hpp), Error/Result (error.hpp), the Tick step
# counter (time.hpp), (Task 2) math_ops and fp32_math -- the first compiled
# engine TUs, which is why this is a STATIC library rather than INTERFACE:
# ids/error/time stay header-only, but math_ops.cpp needs somewhere to compile
# into. The spade::core alias and install/export set are unchanged across that
# switch so existing/future consumers see no difference at the target-name
# level.
#
# fp32_math.cpp is deliberately a COMPILED TU rather than a header-only
# addition next to rng.hpp, even though its only caller today is rng.hpp's
# header-inline next_gauss(). Its whole purpose is that its arithmetic is
# bit-reproducible, and a header would be recompiled under whatever fp flags
# each consuming target happens to carry; compiled here, it is built exactly
# once, under spade_fp_strict's -ffp-contract=off, and every consumer links the
# same object. The polynomials inside are Horner chains -- precisely the
# `a*b+c` shape a contracting compiler fuses -- so this is not a stylistic
# preference.
# ------------------------------------------------------------------------------
add_library(spade_core STATIC
    core/math_ops.cpp
    core/fp32_math.cpp
)
add_library(spade::core ALIAS spade_core)

# EXPORT_NAME (Task 8): install(EXPORT spadeTargets ... NAMESPACE spade::)
# further down this file names each exported target after its RAW CMake target
# (spade_core), not this ALIAS -- an ALIAS is purely in-tree and install(EXPORT)
# does not see it. Without this property the installed, out-of-tree target
# would be spade::spade_core, not spade::core, silently breaking every
# find_package(spade) consumer's target_link_libraries(... spade::core) call
# (this exact gap is what tests/consumer/'s out-of-tree smoke caught).
# Setting EXPORT_NAME here makes the exported name match this in-tree alias
# exactly, so one name works both in-tree and installed. Same on every other
# exported target below (spade_world/state/physics/sim/vehicles) -- see this
# comment, not repeated at each site.
set_target_properties(spade_core PROPERTIES EXPORT_NAME core)

target_include_directories(spade_core PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    $<INSTALL_INTERFACE:include>
)

# Pinned via compile feature (not the ambient CMAKE_CXX_STANDARD variable):
# the v1-frozen root CMakeLists.txt sets CMAKE_CXX_STANDARD 20 for src/ and
# examples/, and that non-cache set() shadows this subdirectory's inherited
# value too. A target-level requirement is immune to that shadowing and
# propagates to anything that links spade::core.
target_compile_features(spade_core PUBLIC cxx_std_23)

# math_ops.hpp exposes glm::vec3/quat/mat3 in its public signatures, so glm
# is a PUBLIC dependency (consumers including the header need its types).
# glm is fetched unconditionally in vendor/CMakeLists.txt (added before the
# SPADE_BUILD_V1 gate in the root CMakeLists.txt), so it is available here
# even when v1 (SPADE_BUILD_V1=OFF) is not built.
#
# Wrapped in $<BUILD_INTERFACE:...>: glm is a vendored FetchContent target,
# not itself installed/exported under spadeTargets, so plain PUBLIC would
# make install(EXPORT spadeTargets ...) below fail ("requires target glm
# that is not in any export set" -- CMake requires every target reachable
# from an exported target's link interface to itself be exportable, an
# IMPORTED target, or excluded from the install interface like this).
# $<BUILD_INTERFACE:...> keeps glm's include dirs flowing to in-tree
# consumers (spade_tests, future engine modules) while leaving it out of the
# installed package interface; header-only glm has no compiled symbols an
# out-of-tree consumer would need linked anyway. A real find_package(spade)
# consumer needing glm's headers would need to find_package(glm) itself --
# fine for now, nothing outside this build tree consumes the export yet.
target_link_libraries(spade_core PUBLIC $<BUILD_INTERFACE:glm>)

# /W4 /WX applies to spade_core's own compilation (math_ops.cpp) without
# exporting the warning policy to consumers linking spade::core -- PRIVATE,
# not PUBLIC/INTERFACE. $<BUILD_LOCAL_INTERFACE:...> (not plain PRIVATE):
# for a STATIC library, CMake still requires PRIVATE-linked targets to be
# exportable (a static archive doesn't resolve its own dependencies, so
# they normally must appear on a consumer's link line too), and
# spade_warnings -- compile-options-only, no symbols -- has nothing to
# install/export. BUILD_LOCAL_INTERFACE (CMake 3.24+) keeps it applied
# in-tree and drops it from the installed export, which is exactly what a
# warnings-as-errors target should do.
target_link_libraries(spade_core PRIVATE $<BUILD_LOCAL_INTERFACE:spade_warnings> $<BUILD_LOCAL_INTERFACE:spade_fp_strict>)

install(TARGETS spade_core
    EXPORT spadeTargets
    ARCHIVE DESTINATION lib
    INCLUDES DESTINATION include
)

install(DIRECTORY core/ DESTINATION include/core FILES_MATCHING PATTERN "*.hpp")

# ------------------------------------------------------------------------------
# spade_world: the analytic SDF scene program (world/sdf.*) plus the
# WorldBuilder and its WorldDesc product (world/builder.*). STATIC rather than
# INTERFACE because it has translation units -- the SDF evaluator lives in one
# .cpp so its pinned fp32 op order sits in a single, reviewable place.
#
# Every link entry below is genex-wrapped, and that is load bearing:
# install(EXPORT) refuses an export set whose members require targets no
# export set owns, and GLM (FetchContent, not an installed package) and
# spade_warnings/spade_fp_strict (flags-only INTERFACE targets) are both
# outside spadeTargets. The genex CHOICE differs by keyword and matches every
# other target in this file: $<BUILD_INTERFACE:glm::glm> on the PUBLIC entry,
# because glm's headers must still reach other IN-TREE consumers that link
# spade::world transitively (add_subdirectory -- what dronesim uses today);
# $<BUILD_LOCAL_INTERFACE:...> (CMake 3.24+) on the two PRIVATE entries,
# because spade_warnings'/spade_fp_strict's flags (/W4 /WX,
# -ffp-contract=off) are this target's own compilation discipline, not
# something any consumer -- in-tree or installed -- should inherit just by
# linking spade::world. A STATIC library records even PRIVATE links into its
# interface as $<LINK_ONLY:...> for transitive linking, so plain PRIVATE
# alone would not keep either genex-wrapped case out of the installed
# export; the genexes do.
# ------------------------------------------------------------------------------
add_library(spade_world STATIC
    world/sdf.cpp
    world/builder.cpp
    world/medium.cpp
    world/world_file.cpp
)
add_library(spade::world ALIAS spade_world)
set_target_properties(spade_world PROPERTIES EXPORT_NAME world)

# PUBLIC spade::state (Task 8): world/medium.hpp's sample() takes a
# spade::WorldParams -- the RUNTIME per-world row from state/layout.hpp, not
# the authoring-side Environment -- so the dependency is in the public
# signature, not an implementation detail. spade_state is defined further down
# this file; that is fine, CMake resolves link items at generate time.
# The direction (world -> state, never state -> world) is deliberate: state
# knows nothing about worlds.
#
# yaml-cpp (S5 Task 5): world/world_file.cpp's parser only. PRIVATE and
# $<BUILD_INTERFACE:...>-wrapped, and the two choices answer different
# questions.
#   PRIVATE, because world/world_file.hpp's surface is paths, std::string and
#   WorldDesc -- no yaml-cpp type crosses it, so no consumer needs its headers.
#   $<BUILD_INTERFACE:...>, for the same install(EXPORT) reason as glm above:
#   yaml-cpp is a FetchContent target in no export set, and a STATIC library
#   records even its PRIVATE links into the interface as $<LINK_ONLY:...>, so
#   an unwrapped entry would fail generation outright.
# THE ONE PLACE THIS DIFFERS FROM glm: yaml-cpp is a compiled archive, not a
# header-only one, so an OUT-OF-TREE consumer that links the INSTALLED
# spade_world.lib and calls world_from_yaml() does need yaml-cpp's objects on
# its final link line -- and $<BUILD_INTERFACE:...> deliberately does not put
# them there. In-tree consumers (spade_tests, spade_viewer, dronesim via
# add_subdirectory -- everything that exists today) are unaffected: the genex
# is live for them.
#
# THE INSTALLED-CONSUMER GAP THIS LEAVES, AND WHY IT IS NOT CLOSED HERE (S5
# Task 5/8 integration follow-up, surfaced by Task 5's opus review). The
# obvious-looking fix -- add a SECOND, $<INSTALL_INTERFACE:yaml-cpp::yaml-cpp>
# entry beside the one above, so the exported spadeTargets.cmake records the
# name for a consumer to resolve -- DOES NOT WORK, and this is recorded here
# because it is exactly the fix a future reader will also reach for first.
# Verified empirically (this integration follow-up's own build failed with
# it) and confirmed against CMake's own documented behaviour (CMake Discourse,
# a Kitware maintainer, on this exact scenario): install(EXPORT) resolves ANY
# name appearing in an exported target's INTERFACE_LINK_LIBRARIES -- INCLUDING
# one that only appears via $<INSTALL_INTERFACE:...> -- against every target
# CMake knows about in THIS project, genex or not, and rejects it if that name
# resolves to a real (non-IMPORTED) target not in any export set. Since
# yaml-cpp::yaml-cpp genuinely IS such a target here (FetchContent, i.e.
# add_subdirectory + add_library, not find_package), the $<INSTALL_INTERFACE:>
# wrapper does not exempt it -- only $<BUILD_INTERFACE:...>/
# $<BUILD_LOCAL_INTERFACE:...>, which HIDE it from the installed interface
# entirely, do. (The documented workaround for "FetchContent dependency
# leaking into an install export" is literally to wrap it in
# $<BUILD_INTERFACE:...>, the opposite direction from what would be needed
# here.) The rejected install(TARGETS yaml-cpp) alternative (making yaml-cpp
# itself installable/exportable, avoiding this whole problem) was already
# ruled out for this program (third-party redistribution, SBOM/version-clash
# surface).
#
# THE ACTUAL FIX therefore lives one layer up, in the CONSUMER's own CMake run
# rather than in this export: cmake/spadeConfig.cmake.in, AFTER
# confirming yaml-cpp::yaml-cpp exists (its own explicit check, mirroring
# glm's), does `set_property(TARGET spade::world APPEND PROPERTY
# INTERFACE_LINK_LIBRARIES yaml-cpp::yaml-cpp)` -- legal precisely because
# that runs inside find_package(spade)'s OWN config-processing, on an already
# fully-formed IMPORTED target, entirely outside Spade's own install(EXPORT)
# validation. The observable effect for a consumer is identical to what a
# working install-interface entry would have produced: linking spade::sim (or
# anything above spade::world) pulls yaml-cpp::yaml-cpp onto their final link
# line transitively. tests/consumer/ is the worked example that calls
# the loader and proves the closure.
target_link_libraries(spade_world
    PUBLIC
        spade::core
        spade::state
        $<BUILD_INTERFACE:glm::glm>
    PRIVATE
        $<BUILD_INTERFACE:yaml-cpp::yaml-cpp>
        $<BUILD_LOCAL_INTERFACE:spade_warnings>
        $<BUILD_LOCAL_INTERFACE:spade_fp_strict>
)

target_include_directories(spade_world PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    $<INSTALL_INTERFACE:include>
)

install(TARGETS spade_world
    EXPORT spadeTargets
    ARCHIVE DESTINATION lib
    INCLUDES DESTINATION include
)

# world/detail/ is internal: yaml_text.hpp names yaml-cpp types, which no installed
# header may expose (the yaml-cpp note above).
install(DIRECTORY world/ DESTINATION include/world FILES_MATCHING PATTERN "*.hpp"
    PATTERN "detail" EXCLUDE)

# ------------------------------------------------------------------------------
# spade_state: the state backbone (Task 6) -- layout.hpp's shared PODs with
# their static_asserted std430 offsets, the world-partitioned ArenaSet, and
# the StateRegistry that Task 7's snapshot walks -- plus that snapshot itself
# (state/snapshot.*: the versioned registry-walk blob and its file IO, which is
# the engine's replay/determinism contract). STATIC because arenas.cpp,
# registry.cpp and snapshot.cpp are compiled TUs; layout.hpp stays header-only.
#
# PUBLIC spade::core: state/*.hpp expose spade::Result/spade::Error in their
# signatures and layout.hpp exposes glm types, both of which reach consumers
# through spade_core's own PUBLIC interface.
# ------------------------------------------------------------------------------
add_library(spade_state STATIC
    state/arenas.cpp
    state/registry.cpp
    state/snapshot.cpp
)
add_library(spade::state ALIAS spade_state)
set_target_properties(spade_state PROPERTIES EXPORT_NAME state)

target_include_directories(spade_state PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    $<INSTALL_INTERFACE:include>
)

# Same reasoning as spade_core above: pinned per target, not via the ambient
# CMAKE_CXX_STANDARD the v1-frozen root CMakeLists.txt sets to 20.
target_compile_features(spade_state PUBLIC cxx_std_23)

target_link_libraries(spade_state PUBLIC spade::core)

# $<BUILD_LOCAL_INTERFACE:...>, not plain PRIVATE -- see the long note on
# spade_core's warnings link above: an installed STATIC library records its
# PRIVATE link dependencies in the export interface as $<LINK_ONLY:...>, and
# install(EXPORT) then rejects spade_warnings for not being in any export set.
target_link_libraries(spade_state PRIVATE $<BUILD_LOCAL_INTERFACE:spade_warnings>)

install(TARGETS spade_state
    EXPORT spadeTargets
    ARCHIVE DESTINATION lib
    INCLUDES DESTINATION include
)

install(DIRECTORY state/ DESTINATION include/state FILES_MATCHING PATTERN "*.hpp")

# ------------------------------------------------------------------------------
# spade_objects (24th spec, SL3-SL5): the composition half of the ECS engine
# design section 4 ratified -- object identity, component attachment, and the
# behavior registry. Holds NO simulation state (SL3): components reference the
# SoA slots spade_state already owns, which is what keeps the snapshot and
# parity estates byte-unchanged.
# ------------------------------------------------------------------------------
add_library(spade_objects STATIC
    objects/behavior.cpp
    objects/behaviors/kinematic_mover.cpp
    objects/graph.cpp
    objects/serialize.cpp
)
add_library(spade::objects ALIAS spade_objects)
set_target_properties(spade_objects PROPERTIES EXPORT_NAME objects)

target_include_directories(spade_objects PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    $<INSTALL_INTERFACE:include>
)

target_compile_features(spade_objects PUBLIC cxx_std_23)

target_link_libraries(spade_objects PUBLIC spade::core)
# spade::state PRIVATE (Plan A Task 9): objects/behaviors/kinematic_mover.cpp
# writes BodyState. PRIVATE because no spade_objects HEADER names a state type
# -- kinematic_mover.hpp exposes only glm and objects/behavior.hpp -- so this is
# a compilation dependency, not part of this target's surface. Not
# BUILD_LOCAL_INTERFACE-wrapped, unlike the spade_warnings/nlohmann links below:
# spade::state IS in this package's export set, so naming it here is exactly
# what an installed consumer's final link needs.
#
# physics/schedule.hpp is used HEADER-ONLY by that same TU (SubstepContext and
# WorldSubstepView are read, no function declared there is called), so no link
# on spade_sim is created and the objects <-> sim relationship stays one-way at
# link time: spade_sim links spade::objects, never the reverse.
target_link_libraries(spade_objects PRIVATE spade::state)
# nlohmann_json is PRIVATE and BUILD_LOCAL_INTERFACE-wrapped for the same
# reason spade_warnings is: serialize.cpp uses it, no nlohmann type appears in
# serialize.hpp's signature, and an unwrapped link would put a target this
# package does not export into spadeTargets.cmake -- breaking find_package()
# for a consumer that has no nlohmann_json target defined.
target_link_libraries(spade_objects PRIVATE
    $<BUILD_LOCAL_INTERFACE:spade_warnings>
    $<BUILD_LOCAL_INTERFACE:nlohmann_json::nlohmann_json>
)

install(TARGETS spade_objects
    EXPORT spadeTargets
    ARCHIVE DESTINATION lib
    INCLUDES DESTINATION include
)

install(DIRECTORY objects/ DESTINATION include/objects FILES_MATCHING PATTERN "*.hpp")

# ------------------------------------------------------------------------------
# spade_render (S7a Task 0 + Task R1): the Spade Rendering & Scene
# Representation program's module -- the render target/camera contract
# (render/target.hpp: RenderTarget, Camera, DrawMode, RenderOptions, Aabb, the
# kNoMaterial/kNoMesh sentinels, and validate_target()) plus, since Task R1,
# render/scene.hpp+.cpp: RenderScene and scene_from_world()/update_dynamics(),
# the WorldDesc -> renderer-view bridge. STATIC because both TUs compile real
# code -- target.cpp's validate_target(), and scene.cpp's
# scene_from_world()/update_dynamics() plus the two world-bounds/ground-plane
# heuristics ported from dronesim/spade/raster.cpp (kept file-local, not part
# of this target's public surface).
#
# Modeled on spade_world above, verbatim in structure -- STATIC lib,
# spade::render ALIAS, EXPORT_NAME, cxx_std_23 compile feature, PUBLIC engine
# deps, PRIVATE $<BUILD_LOCAL_INTERFACE:spade_warnings>/spade_fp_strict.
#
# PUBLIC spade::core: target.hpp's Result<void> return and Camera's
# glm::vec3/glm::quat public members reach consumers through spade_core's own
# PUBLIC interface (Result/Error from core/error.hpp; glm from spade_core's
# own PUBLIC $<BUILD_INTERFACE:glm> link) -- the same "public API exposes it,
# so the link is PUBLIC" reasoning every other target in this file uses, and
# the same precedent spade_state's own comment above sets for not re-adding
# glm here directly.
#
# PUBLIC spade::world + spade::state: Task R1's scene_from_world() takes a
# `const spade::WorldDesc&` (world/builder.hpp) and stores a
# `const spade::SdfProgram*` (world/sdf.hpp) in its own public RenderScene
# result -- both in scene.hpp's own public signature -- and, transitively
# through spade::world, spade::state comes along the same way spade_world's
# own PUBLIC spade::state link does above.
#
# $<BUILD_LOCAL_INTERFACE:...> on warnings/fp_strict -- same reasoning as
# every other target in this file (spade_core's own comment above): an
# installed STATIC library must not leak flags-only INTERFACE targets into
# its export set. spade_fp_strict specifically: scene.cpp's world-bounds
# helper evaluates no pinned-order fp32 expression feeding the determinism
# digest either (presentation-only, PA-4/PA-5), but tessellate/csg_mesh TUs a
# later S7a task adds to this SAME target will, so the flag stays wired from
# the target's first TU rather than revisited per file.
#
# render/shadow.cpp (S7a Task R7): the single-sun shadow map -- depth-only
# orthographic rasterisation fitted to RenderScene::bounds, cached on
# RenderScene::static_shadow (scene.cpp calls it from scene_from_world()) and
# sampled per pixel from raster_cpu.cpp. PRIVATE to this target's own build
# (no new external dependency): both TUs already need each other's headers
# (scene.hpp includes shadow.hpp for the ShadowMap field type; shadow.cpp
# includes scene.hpp for RenderScene's complete definition), so it joins the
# same STATIC target rather than becoming a separate library.
#
# render/gltf.cpp (S7a Task R4) joins this target's sources: the glTF 2.0
# mesh loader, PRIVATE to spade_render (nlohmann::json never appears in
# gltf.hpp's signature -- Result<MeshData>/std::filesystem::path/
# std::string_view only). LINKED VIA $<BUILD_LOCAL_INTERFACE:...>, same as
# spade_warnings/spade_fp_strict below, and for the identical reason even
# though nlohmann_json::nlohmann_json is header-only (no compiled object code
# at all, unlike yaml-cpp): CMake's install(EXPORT spadeTargets ...) rejects
# ANY plain PRIVATE-linked target that is not itself in an export set,
# regardless of whether that target compiles anything -- proved empirically
# ("requires target \"nlohmann_json\" that is not in any export set", this
# task's own first configure attempt) before reaching for the genex rather
# than assuming header-only made it exempt. BUILD_LOCAL_INTERFACE scopes the
# usage requirement to building spade_render's own sources and drops it from
# the exported interface entirely, exactly like the flags-only targets below.
#
# render/raymarch.cpp (S7a Task R8): the exact SDF reference path -- sphere-
# traces RenderScene::sdf directly for DrawMode::raymarch, called from
# raster_cpu.cpp's render(). No new external dependency: it calls
# world/sdf.hpp's eval()/gradient() (already reached transitively through
# this target's PUBLIC spade::world link, below) and render/scene.hpp's
# shared shade_vertex_color()/sky_gradient_color(), joining this target
# rather than becoming a separate library for the same reason shadow.cpp
# did -- it and raster_cpu.cpp already need each other's headers
# (raster_cpu.cpp includes raymarch.hpp to dispatch to it).
#
# render/agreement.cpp (S7a Task R9): the RS4 visual/physics agreement
# harness -- compare_silhouettes()/assert_no_material_matches_sky(), a pure
# post-process over two already-rendered RenderTarget frames (one shaded, one
# raymarched). No new external dependency: it calls render/scene.hpp's shared
# to_byte() and reads world/builder.hpp's MaterialDesc (already reached
# transitively through this target's PUBLIC spade::world link, below, for the
# SAME reason raymarch.cpp's SdfProgram reference is). Joins this target
# rather than becoming a separate library, matching every other single-file
# addition to spade_render above -- nothing about it needs a build boundary
# raymarch.cpp/shadow.cpp/gltf.cpp did not already establish.
# ------------------------------------------------------------------------------
add_library(spade_render STATIC
    render/target.cpp
    render/scene.cpp
    render/tessellate.cpp
    render/csg_mesh.cpp
    render/raster_cpu.cpp
    render/gltf.cpp
    render/shadow.cpp
    render/raymarch.cpp
    render/agreement.cpp
)
add_library(spade::render ALIAS spade_render)
set_target_properties(spade_render PROPERTIES EXPORT_NAME render)

target_include_directories(spade_render PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    $<INSTALL_INTERFACE:include>
)

target_compile_features(spade_render PUBLIC cxx_std_23)

target_link_libraries(spade_render PUBLIC spade::core spade::world spade::state)
target_link_libraries(spade_render PRIVATE
    $<BUILD_LOCAL_INTERFACE:spade_warnings>
    $<BUILD_LOCAL_INTERFACE:spade_fp_strict>
    $<BUILD_LOCAL_INTERFACE:nlohmann_json::nlohmann_json>
)

install(TARGETS spade_render
    EXPORT spadeTargets
    ARCHIVE DESTINATION lib
    INCLUDES DESTINATION include
)

install(DIRECTORY render/ DESTINATION include/render FILES_MATCHING PATTERN "*.hpp")

# ------------------------------------------------------------------------------
# spade_render_gl: v2's GPU RENDER backend, and the thing v2 has never had.
#
# ⭐ WHY IT IS A SEPARATE MODULE AND NOT SOURCES INSIDE spade_render:
# spade_render is INSTALLED, embeddable, and has no windowing or GL dependency
# of any kind. Folding GL into it would hand that dependency to every consumer
# of the engine -- dronesim, the SDK, the Linux gate -- for a capability most of
# them do not want. An optional GPU-backed module behind its own option is the
# shape spade_compute already uses behind SPADE_VULKAN, so this is the
# established pattern applied rather than a new one invented.
#
# ⚠ DELIBERATELY NOT INSTALLED AND NOT EXPORTED, FOR NOW. SL2a states a release
# identity of ELEVEN library modules, and quietly making it twelve is a change
# to a signed claim rather than a build detail. In-tree consumers (spade_sandbox)
# link it directly; an out-of-tree consumer wanting the GPU path is a decision
# for whoever amends SL2a, not a side effect of this file.
#
# The MEASUREMENT that motivates it, on one box, 2026-09-18: v1's GL path
# renders a near-empty scene at ~950 fps at 1280x720 where v2's raster_cpu
# renders a ground plane at 11.5 fps at 900x560 -- 1.83x fewer pixels and ~82x
# slower, about 150x per pixel.
# ------------------------------------------------------------------------------
if(SPADE_RENDER_GL)
add_library(spade_render_gl STATIC
    render_gl/gl_renderer.cpp
)
add_library(spade::render_gl ALIAS spade_render_gl)

target_include_directories(spade_render_gl PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    $<INSTALL_INTERFACE:include>
)

# No glad LINK: gl_renderer.cpp loads its GL entry points into a private table
# through the caller's GlProcLoader, so the archive exports no glad symbol to
# clash with a consumer's own loader. It takes glad's HEADER for the GL types
# and enums only: a PRIVATE build-tree include, never part of the interface.
# Linking the glad target, even PRIVATE, would put $<LINK_ONLY:glad> in the
# interface, and install(EXPORT) refuses a non-exported target there.
target_include_directories(spade_render_gl PRIVATE
    $<BUILD_INTERFACE:$<TARGET_PROPERTY:glad,INTERFACE_INCLUDE_DIRECTORIES>>
)

# glm is PUBLIC because the header's signatures use it, wrapped in
# $<BUILD_INTERFACE:...> for the install(EXPORT) reason spade_core's own glm
# link gives. spade::render brings the scene/target types the API is
# expressed in.
target_link_libraries(spade_render_gl PUBLIC spade::core spade::render $<BUILD_INTERFACE:glm>)
target_link_libraries(spade_render_gl PRIVATE
    $<BUILD_LOCAL_INTERFACE:spade_warnings>
)
target_compile_features(spade_render_gl PUBLIC cxx_std_23)

# Installed and exported as spade::render_gl, an optional module present when
# SPADE_RENDER_GL is on (the lead, 2026-10-03). The editor's GL path (RND-4)
# must be reachable from an installed Spade: SL2b lets the sandbox use only what
# the engine installs, and a consumer like Kat's editor needs the same path.
# Its interface holds only exported targets, so install(EXPORT) accepts it.
# tests/consumer links it beside a second glad, and consumer-smoke.sh checks
# that the archive defines and needs no glad symbol.
set_target_properties(spade_render_gl PROPERTIES EXPORT_NAME render_gl)
install(TARGETS spade_render_gl
    EXPORT spadeTargets
    ARCHIVE DESTINATION lib
    INCLUDES DESTINATION include
)
install(DIRECTORY render_gl/ DESTINATION include/render_gl FILES_MATCHING PATTERN "*.hpp")
endif()

# ------------------------------------------------------------------------------
# spade_physics: the per-substep dynamics passes -- Integrate (Task 9: one
# substep of symplectic Euler over a world's body span, with the specific-force
# capture the IMU pass reads), CollisionStatic (Task 11: sphere-proxy vs
# world-SDF contact generation and impulse response), CollisionDynamic (Task 12,
# physics/grid.*: the world-keyed sorted-grid broad phase ported from v1 plus the
# mass-weighted pair response), and the ForceElements pass's force-element tables
# and DragBody (Task 15, physics/forces.*). STATIC because all four are compiled
# TUs, and deliberately so: their op orders are the CPU<->GPU parity contract
# (D1/D11), so each lives in exactly one reviewable place rather than being
# inlined from a header where a consumer's optimizer settings could reassociate
# it.
#
# PUBLIC spade::state: integrator.hpp and forces.hpp take BodyState/WorldParams
# by span and reference in their signatures, so consumers need layout.hpp --
# and through spade_state's own PUBLIC link, spade::core (math_ops, used in
# integrator.cpp) and glm's headers come along.
#
# PUBLIC spade::world: forces.hpp's apply_drag() takes `const Medium&`
# (world/medium.hpp) in its public signature -- the same "public API exposes
# it, so the link is PUBLIC" reasoning spade_world itself uses for spade::state
# above. This is the one new edge Task 15 adds to the module graph: physics
# now depends on world, joining world's existing dependency on state; state
# itself still knows nothing about either.
#
# PUBLIC spade::world: contacts.hpp takes an SdfProgram by const reference in
# its signature, so consumers need world/sdf.hpp. The direction (physics ->
# world, never world -> physics) is deliberate -- the world knows nothing about
# the passes that query it -- and it is acyclic: spade_world links state and
# core only.
#
# $<BUILD_LOCAL_INTERFACE:...> on the warnings link, not plain PRIVATE -- see
# the long note on spade_core's warnings link above: an installed STATIC library
# records its PRIVATE link dependencies in the export interface as
# $<LINK_ONLY:...>, and install(EXPORT) then rejects spade_warnings for not
# being in any export set.
# ------------------------------------------------------------------------------
add_library(spade_physics STATIC
    physics/integrator.cpp
    physics/contacts.cpp
    physics/grid.cpp
    physics/forces.cpp
)
add_library(spade::physics ALIAS spade_physics)
set_target_properties(spade_physics PROPERTIES EXPORT_NAME physics)

target_include_directories(spade_physics PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    $<INSTALL_INTERFACE:include>
)

# Same reasoning as spade_core/spade_state above: pinned per target, not via the
# ambient CMAKE_CXX_STANDARD the v1-frozen root CMakeLists.txt sets to 20.
target_compile_features(spade_physics PUBLIC cxx_std_23)

target_link_libraries(spade_physics PUBLIC spade::state spade::world)
target_link_libraries(spade_physics PRIVATE $<BUILD_LOCAL_INTERFACE:spade_warnings> $<BUILD_LOCAL_INTERFACE:spade_fp_strict>)

install(TARGETS spade_physics
    EXPORT spadeTargets
    ARCHIVE DESTINATION lib
    INCLUDES DESTINATION include
)

install(DIRECTORY physics/ DESTINATION include/physics FILES_MATCHING PATTERN "*.hpp")

# ------------------------------------------------------------------------------
# spade_sim: the integration point (Task 13) -- the fixed per-substep PASS
# SCHEDULE (physics/schedule.*, spec section 3's eight passes as data rather than
# as a call sequence) and the Simulation/WorldSet layer that owns the arenas, the
# tick, the substep loop and the structural queue (sim/*).
#
# WHY schedule.* COMPILES HERE RATHER THAN INTO spade_physics. The schedule is
# not a physics pass; it is the ORDER of the physics passes, and the only thing
# that runs it is Simulation. Splitting it into spade_physics would give that
# target a file no other consumer of it can use, and would put the "who owns the
# substep loop" answer in two places. The HEADER still lives under physics/ (it is
# where a reader looks for the pass sequence, and physics/integrator.hpp,
# contacts.hpp and grid.hpp all quote that sequence in their own headers), so it
# is installed by the existing install(DIRECTORY physics/ ...) rule above -- the
# header's residency and its compilation unit's target are deliberately allowed
# to differ here, and this comment is the record of that.
#
# PUBLIC spade::physics: sim/simulation.hpp exposes physics::WorldSubstepView,
# GridScratch, ContactParams, GridParams and DragBodyRow in its own public
# surface, and physics itself carries spade::world (the SdfProgram and Medium
# a world instance holds) and spade::state (the arenas and the snapshot) through
# its PUBLIC interface -- so one link edge brings the whole lower stack, exactly
# as the module graph intends. Nothing new is added to that graph: sim sits
# strictly above physics, and nothing below it knows sim exists.
#
# PUBLIC spade::vehicles (Task 18): sim/simulation.hpp exposes ModelType,
# RotorDesc and RotorRow in its own public surface (the model registry, the
# structural queue's rotor op, and the rotor accessor), and physics/schedule.hpp
# -- which compiles into THIS target, see above -- carries a RotorRow span in
# WorldSubstepView and calls vehicles::apply_rotors() from the ForceElements
# pass, which is where spec §3's "rotors -> drag -> lift surfaces" lives. The
# edge is sim -> vehicles -> physics, still strictly downward and still
# acyclic; vehicles knows nothing about sim.
#
# ALSO COMPILED HERE: sensors/imu.cpp (Task 19) -- the SensorSynthesis pass's
# IMU synthesis, plus the header-only output rings (sensors/rings.hpp). It joins
# THIS target rather than getting one of its own for the same reason
# physics/schedule.cpp does: the only thing that runs a sensor pass is the
# schedule, which only Simulation runs, and spade_sim already carries every
# dependency it has (state for BodyState, core for rng::Stream, physics for
# body_flags). A spade_sensors target would be a library with exactly one
# consumer in the same directory.
#
# NOT LISTED HERE: engine/testing/. It holds header-only TEST SUPPORT --
# replay.hpp (the determinism digest and the scenario harness) and, since S5
# Task 7, scenario_file.hpp (the scenario-file loader) -- included by
# tests/ only, and deliberately absent from every target and every
# install rule. Note that every install(DIRECTORY ...) above names ONE module
# directory (core/, world/, state/, physics/, sim/, sensors/, vehicles/), so
# nothing sweeps testing/ by accident; adding a header there ships nothing.
# A consumer of the installed package has no business with any of it.
#
# $<BUILD_LOCAL_INTERFACE:...> on the warnings link, not plain PRIVATE -- see the
# long note on spade_core's warnings link above.
# ------------------------------------------------------------------------------
add_library(spade_sim STATIC
    physics/schedule.cpp
    sim/module_schedule.cpp
    sim/standard_modules.cpp
    sim/design_frame.cpp
    # objects/spawn_helpers.cpp (SL9e, Plan A Task 11) -- HEADER DIRECTORY AND
    # COMPILING TARGET DELIBERATELY DIFFER, exactly as physics/schedule.cpp's
    # do a line above, and here it is not a preference but the only option:
    # spawn_in_sphere/spawn_in_cube call Simulation::spawn/despawn/
    # body_capacity/live_body_count, so the TU needs spade_sim's symbols -- and
    # spade_sim already links spade::objects (Task 8, for the behavior
    # registry). Compiling it into spade_objects would close that into a target
    # cycle. The header stays under objects/ because SL9e groups it with the
    # object model, and a consumer needs spade::sim to call it anyway: the API
    # takes a Simulation&.
    objects/spawn_helpers.cpp
    sensors/gnss.cpp
    sensors/imu.cpp
    sim/simulation.cpp
    sim/world_set.cpp
)
add_library(spade::sim ALIAS spade_sim)
set_target_properties(spade_sim PROPERTIES EXPORT_NAME sim)

target_include_directories(spade_sim PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    $<INSTALL_INTERFACE:include>
)

# Same reasoning as spade_core/spade_state/spade_physics above: pinned per
# target, not via the ambient CMAKE_CXX_STANDARD the v1-frozen root
# CMakeLists.txt sets to 20.
target_compile_features(spade_sim PUBLIC cxx_std_23)

# spade::objects (24th spec SL6, Plan A Task 8): physics/schedule.hpp's
# SubstepContext carries a `const objects::BehaviorRegistry*` and schedule.cpp
# calls run_slot() on it. PUBLIC because the pointer is in a public header's
# struct, so a consumer that attaches a registry needs the target too. No cycle:
# spade_objects links only spade::core, and objects/behavior.hpp forward-
# declares SubstepContext rather than including schedule.hpp.
target_link_libraries(spade_sim PUBLIC spade::physics spade::vehicles spade::objects)
target_link_libraries(spade_sim PRIVATE $<BUILD_LOCAL_INTERFACE:spade_warnings> $<BUILD_LOCAL_INTERFACE:spade_fp_strict>)

install(TARGETS spade_sim
    EXPORT spadeTargets
    ARCHIVE DESTINATION lib
    INCLUDES DESTINATION include
)

install(DIRECTORY sim/ DESTINATION include/sim FILES_MATCHING PATTERN "*.hpp")

# sensors/*.hpp are part of spade_sim's public surface (sim/simulation.hpp
# exposes ImuSensorRow, ImuSample and the poll result), so they install
# alongside sim/ even though the header directory and the compiling target
# differ -- the same deliberate split physics/schedule.hpp already has.
install(DIRECTORY sensors/ DESTINATION include/sensors FILES_MATCHING PATTERN "*.hpp")

# ------------------------------------------------------------------------------
# spade_vehicles: the model-type layer (spec section 6) -- engine asset classes
# one tier above shapes and meshes. Task 17 ships its first member,
# vehicles/rotor.* (RotorElement: thrust/torque curves, RPM lag, momentum-theory
# inflow, SDF-sampled ground effect -- D5); Task 18's Quadrotor ModelType joins
# it here. Spec section 3's tree comment is the rule this target enforces:
# "nothing vehicle-specific below here", i.e. the dependency arrow points DOWN
# from vehicles into physics/world/state and never back.
#
# WHY A ROTOR IS NOT IN spade_physics EVEN THOUGH IT IS A FORCE ELEMENT. The
# split is by what the model KNOWS: physics/ knows rigid bodies and fields;
# vehicles/ knows that some bodies are machines with actuators, commands and
# per-actuator state. Spec section 6 lists RotorElement under the model-type
# layer for that reason, and the coordinator's resolution gives this directory
# its own target accordingly.
#
# PUBLIC spade::state + spade::world: vehicles/rotor.hpp takes BodyState and
# WorldParams (state/layout.hpp) by span/reference and `const Medium&` /
# `const SdfProgram&` (world/medium.hpp, world/sdf.hpp) in its public
# signature, so a consumer including the header needs all three headers -- the
# same "public API exposes it, so the link is PUBLIC" reasoning spade_world
# uses for spade::state.
#
# PUBLIC spade::physics -- PROMOTED FROM PRIVATE BY TASK 18, and the reason is
# worth recording rather than just doing. Task 17 linked it privately because
# only rotor.cpp needed it (body_flags::active, from physics/integrator.hpp)
# and nothing about physics appeared in this target's public surface. Task 18's
# vehicles/model_type.hpp changes that: a DragBodyDesc names its element mode
# with physics::drag_mode:: values, so the header itself includes
# physics/forces.hpp and a consumer needs it. The alternative -- re-declaring
# the two mode constants here -- would duplicate a wire-level enumeration
# across a module boundary, which is precisely the v1 defect (D9) the layout
# discipline exists to prevent. The dependency arrow still points the right
# way: vehicles sits ABOVE physics, physics knows nothing about vehicles, and
# the graph stays acyclic.
# ------------------------------------------------------------------------------
add_library(spade_vehicles STATIC
    vehicles/rotor.cpp
    vehicles/rotor_wake.cpp
    vehicles/model_type.cpp
    vehicles/model_identity.cpp
    vehicles/quadrotor.cpp
    vehicles/motor.cpp
    vehicles/battery.cpp
    vehicles/propeller.cpp
    vehicles/composite_inertia.cpp
    vehicles/propulsion_steady.cpp
    vehicles/airframe_compile.cpp
)
add_library(spade::vehicles ALIAS spade_vehicles)
set_target_properties(spade_vehicles PROPERTIES EXPORT_NAME vehicles)

target_include_directories(spade_vehicles PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    $<INSTALL_INTERFACE:include>
)

# Same reasoning as spade_core/spade_state/spade_physics/spade_sim above:
# pinned per target, not via the ambient CMAKE_CXX_STANDARD the v1-frozen root
# CMakeLists.txt sets to 20.
target_compile_features(spade_vehicles PUBLIC cxx_std_23)

target_link_libraries(spade_vehicles PUBLIC spade::state spade::world spade::physics)
target_link_libraries(spade_vehicles PRIVATE $<BUILD_LOCAL_INTERFACE:spade_warnings> $<BUILD_LOCAL_INTERFACE:spade_fp_strict>)

install(TARGETS spade_vehicles
    EXPORT spadeTargets
    ARCHIVE DESTINATION lib
    INCLUDES DESTINATION include
)

install(DIRECTORY vehicles/ DESTINATION include/vehicles FILES_MATCHING PATTERN "*.hpp")

# ------------------------------------------------------------------------------
# spade_scene: composing a scene (a world reference plus assets and vehicles)
# into a runnable world -- the drone-builder joint spec, section 3, approved by
# the user 2026-10-03. Core owns the module's meaning (the world model);
# Interface implements it (docs/design/interface/plans/
# 2026-10-03-scene-composer-plan.md). Installed and exported, because Kat
# composes scenes too and links only installed targets (SL2b).
#
# It links spade::world and spade::sim; vehicles::ModelType reaches it
# through sim. spade_fp_strict, because the composed transforms land in the
# SDF program that config_hash folds.
#
# scene/scene_file.cpp is Core's schema (SCN-001..009, the drone-builder plan's
# Task D): it parses and emits YAML, so it links yaml-cpp PRIVATE and
# $<BUILD_INTERFACE:...>-wrapped, for spade_world's reasons (the yaml-cpp note
# above). An installed consumer gets yaml-cpp through spade::world, which
# spadeConfig.cmake gives it; tests/consumer parses a scene to prove that link.
# ------------------------------------------------------------------------------
add_library(spade_scene STATIC
    scene/compose.cpp
    scene/scene_file.cpp
)
add_library(spade::scene ALIAS spade_scene)
set_target_properties(spade_scene PROPERTIES EXPORT_NAME scene)
target_include_directories(spade_scene PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(spade_scene
    PUBLIC spade::world spade::sim
    PRIVATE
        $<BUILD_INTERFACE:yaml-cpp::yaml-cpp>
        $<BUILD_LOCAL_INTERFACE:spade_warnings>
        $<BUILD_LOCAL_INTERFACE:spade_fp_strict>
)
target_compile_features(spade_scene PUBLIC cxx_std_23)

install(TARGETS spade_scene
    EXPORT spadeTargets
    ARCHIVE DESTINATION lib
    INCLUDES DESTINATION include
)
install(DIRECTORY scene/ DESTINATION include/scene FILES_MATCHING PATTERN "*.hpp")

# ------------------------------------------------------------------------------
# spade_compute (S6 Task 1): the v2 Vulkan compute backend scaffold --
# compute/backend.hpp's BackendKind/BackendDesc descriptors (no Vulkan
# header; any consumer can use them without pulling in Vulkan at all) and
# compute/vulkan/context.* (VulkanContext: headless instance + device + one
# compute queue + one transfer queue, RAII, volk-loaded). STATIC, one
# compiled TU today (vulkan/context.cpp), matching every other module in
# this file.
#
# GATED ENTIRELY behind SPADE_VULKAN (CMakeLists.txt, default ON) --
# unlike vendor/CMakeLists.txt's Vulkan-Headers/volk/slang FetchContent
# calls, which fetch unconditionally (see that file's own comments), this is
# the one place SPADE_VULKAN actually removes something from the build
# graph: with it OFF, this whole block -- target, install rules, everything
# -- is absent, and spadeTargets.cmake/spadeConfig.cmake simply never
# mention spade::compute. Proven by this task's OFF-configure verification
# (task report): configure + build + install all succeed with
# SPADE_VULKAN=OFF and no spade_compute anywhere in the result.
#
# PUBLIC spade::state: this task's own interface note -- device-side mirrors
# of registered state (a later task) take BodyState/WorldParams
# (state/layout.hpp) shapes, so a consumer of spade::compute needs those
# layout types on its include path from day one, even though this task's two
# headers (backend.hpp, vulkan/context.hpp) don't reference them yet.
#
# PRIVATE volk + Vulkan::Headers, $<BUILD_INTERFACE:...>-wrapped: this
# target's one translation unit (vulkan/context.cpp) is the sole place in
# the whole v2 tree that calls into Vulkan (the S6 dependency rule, spec
# section 2: "nothing outside engine/compute/ includes a Vulkan header").
# PRIVATE, not PUBLIC, because VulkanContext's public surface
# (device_name()/api_version()/compute_queue_family()/instance()/etc.)
# exposes Vk* types and plain scalars in ITS OWN header
# (compute/vulkan/context.hpp) directly -- but nothing about spade_compute's
# OWN target-level API (what a consumer gets merely by linking spade::compute
# without including that header) requires volk/Vulkan-Headers, so neither
# dependency needs to reach a consumer's link line by DEFAULT.
# $<BUILD_INTERFACE:...>, not $<BUILD_LOCAL_INTERFACE:...>: the same
# install(EXPORT) reasoning as spade_world's yaml-cpp link above applies
# here even more directly -- volk is a COMPILED static archive (volk.c), not
# flags-only, and a STATIC library records even PRIVATE links into its
# interface as $<LINK_ONLY:...>; an unwrapped or BUILD_LOCAL_INTERFACE-only
# entry would make install(EXPORT spadeTargets ...) reject `volk`/
# `Vulkan-Headers` for not being in any export set. $<BUILD_INTERFACE:...>
# keeps both live for in-tree consumers -- tests/CMakeLists.txt directly
# links `volk`/`Vulkan::Headers` itself for exactly this reason, mirroring
# how it already directly links `yaml-cpp::yaml-cpp` for
# engine/testing/scenario_file.hpp -- while leaving the installed export
# exactly as bare as spade_world's yaml-cpp precedent leaves it, FOR HEADERS
# (an out-of-tree consumer that includes compute/vulkan/context.hpp directly
# still owes itself the same fetch spadeConfig.cmake.in already documents for
# glm/yaml-cpp, since context.hpp names Vk*/volk types in its own public
# signature).
#
# THIS WAS NOT THE WHOLE STORY, AND THE GAP IT LEFT WAS REAL (T11 fix-wave,
# review F-I1; corrected here, not merely re-diagnosed). This comment
# originally called closing volk's SYMBOL gap "future integration work" on
# the assumption that only a consumer directly including context.hpp would
# ever need volk resolved at link time. That assumption was wrong: S6 Task 5
# made spade_sim link spade::compute PUBLIC and call
# compute::VulkanBackend::create/upload/step/readback UNCONDITIONALLY
# (BackendKind is a RUNTIME branch, not a compile-time one -- see spade_sim's
# own link comment below), so context.cpp/state_mirror.cpp/step_recorder.cpp/
# backend.cpp/timestamps.cpp compile real vk*/volk* calls into every
# SPADE_VULKAN=ON build regardless of which backend a given run actually
# selects. Those calls are UNRESOLVED SYMBOLS as far as libspade_compute.a's
# own object files are concerned, and the PRIVATE + BUILD_INTERFACE wrap
# above -- correct for HEADERS -- also means install(EXPORT) never lets
# `volk`'s symbols reach an installed consumer's final link. So ANY
# out-of-tree consumer that links spade::sim (i.e. essentially every real
# consumer, whether or not it ever constructs a vulkan-backed Simulation)
# failed to link with ~100 undefined vk*/volk* references -- proven by the
# gcc-13 Linux gate's install+consumer-smoke step (task-11-fixwave-report.md).
#
# THE CLOSURE (below, after the PRIVATE link block): rather than exporting
# volk's own FetchContent target into spadeTargets (install(EXPORT) allows
# this in principle, but it would make a first-party-looking installed
# dependency out of a vendored one -- not what glm/yaml-cpp's own precedent
# chose) or pushing a "fetch volk yourself" obligation onto every consumer
# the way the glm/yaml-cpp spadeConfig.cmake.in closures do (right for those
# two: glm's types and yaml-cpp's calls are NAMED in Spade's own public API;
# volk's are not -- no installed Spade header outside compute/vulkan/
# context.hpp ever names anything volk.h declares, and context.hpp's own gap
# is the HEADERS one above, already closed the glm/yaml-cpp way), volk.c --
# its entire implementation, one translation unit -- is compiled DIRECTLY as
# an ADDITIONAL spade_compute SOURCE. The object code then lives INSIDE
# libspade_compute.a itself, so an installed consumer's final link resolves
# every vk*/volk* symbol from the archive it already links, with no separate
# volk archive and no consumer-side obligation at all. This is strictly a
# SYMBOL fix, not a HEADERS one -- a consumer that includes context.hpp
# directly still owes itself volk.h/Vulkan-Headers on its OWN include path,
# exactly as the paragraph above already says.
#
# $<BUILD_LOCAL_INTERFACE:...> on warnings/fp_strict -- same reasoning as
# every other target in this file (spade_core's own comment above): an
# installed STATIC library must not leak flags-only INTERFACE targets into
# its export set.
# ------------------------------------------------------------------------------
#
# S6 TASK 3 ADDITIONS (the D9 pipeline), all inside the same SPADE_VULKAN gate:
#
#   * compute/layout_check.cpp -- the TU that compiles the GENERATED
#     static_asserts (layout_check.gen.hpp) comparing every C++ state-row field
#     offset against the offset slangc reports for the matching field of
#     engine/shaders/shared/layouts.slang. Layout drift becomes a build failure
#     of THIS target, which is the whole D9 mechanism.
#   * spade_slang_generated_layouts() -- adds the slangc reflection command,
#     the generator command, and the generated-header include directory (a
#     BUILD_INTERFACE entry: those headers live only in the build tree and are
#     never installed).
#   * spade_slang_kernel() -- compiles a kernel to SPIR-V and embeds it as a
#     uint32_t[] header, so nothing loads a .spv at runtime (the no-CWD rule
#     applies to shaders too). Used for a kernel with no local-size knob to
#     serve; fp32_math_probe is the only one (probe_runner.cpp dispatches it on
#     its own terms, outside the schedule).
#   * spade_slang_kernel_variants() -- the same, ONCE PER SIZE in
#     SPADE_SLANG_WORKGROUP_SIZES {32, 64, 128}, embedded as one header holding
#     all three plus a variant table (S6 Task 9b). Every kernel the schedule
#     dispatches uses this one, because that is what makes the A7
#     BackendDesc::workgroup_size knob reach a real pipeline instead of a
#     literal `[numthreads(64,1,1)]` baked at slangc time.
#
# The functions live in cmake/SpadeSlang.cmake; see that file for the
# generation-mechanism decision and for why -fp-mode precise is on every kernel
# compile. include()d HERE rather than from the root CMakeLists.txt because it
# FATAL_ERRORs without SPADE_SLANGC, which only exists once vendor/ has fetched
# the slang prebuilt -- and because a SPADE_VULKAN=OFF configure must never
# even look for slangc or for a Python interpreter.
#
if(SPADE_VULKAN)
include(${PROJECT_SOURCE_DIR}/cmake/SpadeSlang.cmake)

#
# S6 TASK 4 ADDITION: compute/vulkan/probe_runner.cpp -- the one-dispatch
# helper the fp32_math parity batteries run their kernel through (and that
# Task 7 reuses for the sort-isolation tests). Inside the same SPADE_VULKAN
# gate and inside engine/compute/vulkan/ for the same dependency reason as
# context.cpp: it is the second (and, for S6, last) translation unit that
# calls into Vulkan.
#
add_library(spade_compute STATIC
    compute/layout_check.cpp
    compute/vulkan/context.cpp
    compute/vulkan/probe_runner.cpp
)

add_library(spade::compute ALIAS spade_compute)
set_target_properties(spade_compute PROPERTIES EXPORT_NAME compute)

target_include_directories(spade_compute PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    $<INSTALL_INTERFACE:include>
)

# Same reasoning as every other module above: pinned per target, not via the
# ambient CMAKE_CXX_STANDARD the v1-frozen root CMakeLists.txt sets to 20.
target_compile_features(spade_compute PUBLIC cxx_std_23)

target_link_libraries(spade_compute PUBLIC spade::state)
target_link_libraries(spade_compute PRIVATE
    $<BUILD_INTERFACE:volk>
    $<BUILD_INTERFACE:Vulkan::Headers>
    $<BUILD_LOCAL_INTERFACE:spade_warnings>
    $<BUILD_LOCAL_INTERFACE:spade_fp_strict>
)

# T11 fix-wave (review F-I1) -- THE INSTALL-EXPORT CLOSURE. See this target's
# own comment above (the corrected "THIS WAS NOT THE WHOLE STORY" paragraph)
# for the full finding: the PRIVATE + $<BUILD_INTERFACE:...> volk link two
# lines up is correct for HEADERS but, by the same install(EXPORT) rule that
# makes it correct, never lets volk's SYMBOLS reach an installed consumer's
# final link -- and every SPADE_VULKAN=ON build needs them there, because
# spade_sim links spade::compute PUBLIC and unconditionally compiles vk*
# calls into this target's TUs regardless of which backend a given run
# selects.
#
# volk's entire implementation is one translation unit -- zeux/volk's own
# CMakeLists.txt: `add_library(volk STATIC volk.h volk.c)` -- so rather than
# exporting volk's own FetchContent target into spadeTargets (turns a
# vendored dependency into an apparently first-party installed one; glm/
# yaml-cpp's own precedent chose not to) or a spadeConfig.cmake.in
# fetch-it-yourself closure like glm/yaml-cpp use (right for those two
# because glm's types and yaml-cpp's calls are NAMED in Spade's own public
# API; volk's are not), volk.c compiles DIRECTLY as an additional
# spade_compute SOURCE. Its object code then lives INSIDE libspade_compute.a
# itself, so an installed consumer's final link resolves vk*/volk* symbols
# from the archive it already links -- no separate volk archive, no
# consumer-side action, and no change to what any consumer's own compile
# line needs (still nothing, by design: nothing about spade_compute's
# TARGET-LEVEL API requires volk/Vulkan-Headers, exactly as this target's
# own PUBLIC-vs-PRIVATE note above already establishes).
#
# get_target_property(... volk SOURCE_DIR), not a new vendor/CMakeLists.txt
# CACHE variable: `volk` (linked PRIVATE just above, for the include
# directories this file's TUs -- and now volk.c itself -- still need) is
# already visible here by target name; SOURCE_DIR is a standard read-only
# CMake target property (the directory add_library(volk ...) was called
# from), so no new cross-directory-scope plumbing is needed the way
# vendor/CMakeLists.txt's SPADE_SLANGC precedent needed one (that value is
# needed BEFORE this SPADE_VULKAN block is even entered, which is not true
# here).
#
# tests/CMakeLists.txt's own direct `volk` link (its host-side
# test_compute_context.cpp/probe_runner.hpp need volk.h on their own include
# path -- see that file's comment) is UNTOUCHED and still correct: a static
# archive only extracts the member that resolves a symbol still undefined at
# that point in the link, so whichever of spade_compute's own bundled
# volk.c.o or the separately-linked libvolk.a's copy is reached first
# resolves every vk*/volk* symbol, and the other is never even considered --
# volk.c compiling into two targets in the same build is not a duplicate-
# symbol conflict, it is ordinary static-archive linking.
get_target_property(_spade_volk_source_dir volk SOURCE_DIR)
target_sources(spade_compute PRIVATE "${_spade_volk_source_dir}/volk.c")
unset(_spade_volk_source_dir)

install(TARGETS spade_compute
    EXPORT spadeTargets
    ARCHIVE DESTINATION lib
    INCLUDES DESTINATION include
)

install(DIRECTORY compute/ DESTINATION include/compute FILES_MATCHING PATTERN "*.hpp")

# The D9 generated headers and the SPIR-V embeddings, attached AFTER the
# install rules above on purpose: both functions add a PUBLIC
# $<BUILD_INTERFACE:...> include directory, which install(TARGETS ...) must not
# see as an installed usage requirement -- generated headers exist only in this
# build tree and are regenerated from the .slang sources on every build.
#
# ORDER MATTERS BETWEEN THE TWO: spade_slang_generated_layouts() must come
# first, because compute/layout_check.cpp includes layout_check.gen.hpp and
# bindings.gen.hpp, and both custom targets are wired as add_dependencies() of
# spade_compute -- which is what stops ninja compiling that TU before the
# generator has produced its inputs.
spade_slang_generated_layouts(spade_compute)

# pipeline_smoke IS GONE (S6 Task 8), and its absence is the deliverable rather
# than a tidy-up. It was the D9 pipeline's SMOKE KERNEL -- scaffolding that gave
# the compile/embed/float-controls half of the pipeline a real module to run
# over before any physics kernel existed, and that Task 5 then reused as the
# step recorder's copy-through stub for every un-ported schedule slot. Task 8
# ports the last two (MediumUpdate and SensorSynthesis), so no slot binds a stub
# any more; the two INERT slots (Gravity and Publish, empty functions on the CPU
# too) recorded NO DISPATCH AT ALL rather than a no-op one; both passes have
# since been removed (module-API plan, stage 1). This task's report records the
# accompanying retirement of spirv_scan.hpp's NO_OP profile, which had no other
# instance.

# fp32_math_probe (S6 Task 4): the evaluation kernel for the fp32_math parity
# batteries. Compiling it is also what compiles engine/shaders/fp32_math.slang
# -- the Slang port of core/fp32_math.cpp -- since a Slang module only reaches
# SPIR-V through a kernel that imports it. That is deliberate rather than
# incidental: the float-controls gate (SlangSpirv.FloatControlsPinned, EXACT
# profile) scans the EMBEDDED module, so the ported kernels' NoContraction
# decorations and their absence of OpFDiv/sqrt are checked on the real compiled
# artifact, not on a separately-compiled copy of it.
spade_slang_kernel(spade_compute fp32_math_probe)

# denorm_probe: a standing device record (banded-parity plan T6). It began as
# M1 of the NVIDIA fp32-denormal measurement
# (docs/design/core/plans/2026-10-04-nvidia-denorm-measurement-plan.md) and is
# now compiled in every build, so every gate log records what the device does
# with fp32 subnormals per op class when no mode is requested (P3);
# tests/test_gpu_denorm_probe.cpp dispatches it through probe_runner.
spade_slang_kernel(spade_compute denorm_probe)

# ------------------------------------------------------------------------------
# S6 TASK 6 -- kernels wave A. Three real physics kernels replacing the stub:
# forces_drag (today's drag.forces), collision_static (static_contact.resolve)
# and integrate (integrate.integrate). This section describes what TASK 6 itself
# added, as a historical delta; compute/vulkan/step_recorder.hpp's recipe table
# is the current account of which kernel each pass records.
#
# engine/shaders/sdf_eval.slang is NOT listed here and does not need to be: it
# is a MODULE, not a kernel, and reaches SPIR-V only through
# collision_static.slang's `import sdf_eval` -- the same arrangement
# fp32_math.slang already has through fp32_math_probe. slangc's -depfile carries
# the transitive import graph, so editing it recompiles the kernel that imports
# it without this file naming it (cmake/SpadeSlang.cmake's DEPFILE note).
#
# Registered in tests/test_slang_layouts.cpp's kSpirvModules[] under the PARITY
# profile -- P1 (NoContraction on every contractable op), P2 (no sum-of-products
# opcode) and P3 (no fp32 denormal mode requested), but NOT exact's E1/E2: these are
# downstream kernels and the global constraint allows them div and sqrt, which
# is exactly why CPU<->GPU here is banded rather than bit-identical.
# ------------------------------------------------------------------------------
spade_slang_kernel_variants(spade_compute forces_drag)
spade_slang_kernel_variants(spade_compute collision_static)
spade_slang_kernel_variants(spade_compute integrate)

# ------------------------------------------------------------------------------
# S6 TASK 7 -- kernels wave B. The CollisionDynamic slot, which is the one
# schedule slot that is a CHAIN rather than a single dispatch: the sorted-grid
# broad phase's three stages (physics/grid.cpp's build / sort / resolve, v1's
# GridBuild.comp / BitonicSort.comp / GridOffsets.comp + GridCollision.comp).
#
#   grid_build         one key per active, in-range body; sentinels elsewhere
#   grid_sort          ONE compare-exchange stage of a bitonic network; the
#                      host unrolls the (k, j) double loop into the recorded
#                      command buffer, so this single module is dispatched once
#                      per stage with its stage in a push constant
#   collision_dynamic  the Gauss-Seidel sweep, one thread per WORLD (its own
#                      header carries the parity argument for why the sweep
#                      cannot be parallel within a world)
#
# All three are registered in tests/test_slang_layouts.cpp's kSpirvModules[]
# under the PARITY profile, exactly like wave A's.
# ------------------------------------------------------------------------------
spade_slang_kernel_variants(spade_compute grid_build)
spade_slang_kernel_variants(spade_compute grid_sort)
spade_slang_kernel_variants(spade_compute collision_dynamic)

# collision_gather -- the same pass as a Jacobi gather, one thread per ENTRY
# rather than one per world, plus the fill dispatch that takes its
# start-of-iteration shadow. Its own file rather than two more entry points in
# collision_dynamic.slang: SlangSpirv.EveryCompiledVariantIsScanned requires
# two workgroup-size variants of a module to differ in EXACTLY ONE word, and a
# module with three entry points has three LocalSize declarations. The guard is
# the estate's only automatic detector of lane-dependent codegen -- the one
# L306 was reverted by -- so the file splits and the guard stays exact.
spade_slang_kernel_variants(spade_compute collision_fill)
spade_slang_kernel_variants(spade_compute collision_gather)

# ------------------------------------------------------------------------------
# S6 TASK 8 -- kernels wave C, which completed the GPU chain.
#
#   medium_update  the dryden.advance pass: one thread per WORLD, advancing that
#                  world's Dryden gust filter one substep (the filter is a
#                  sequential recurrence within a world; worlds are the parallel
#                  axis)
#   rotors         the rotor.forces pass -- recorded ahead of forces_drag
#                  because drag.forces declares `after rotor.forces`, and that
#                  order is "a numerical contract and not a preference" (both
#                  accumulate into the same two float3 accumulators;
#                  sim/standard_modules.cpp)
#   sensor_imu     the imu.synthesize pass: one thread per SENSOR slot, a THIRD
#                  dispatch grid, writing sensor rows and ring samples
#
# FOUR MODULES, NOT LISTED HERE AND NOT NEEDING TO BE, for sdf_eval.slang's
# reason above -- a Slang module reaches SPIR-V only through a kernel that
# imports it, and slangc's -depfile carries the transitive graph:
#
#   u64.slang      exact 64-bit unsigned arithmetic on uint2 lanes (this device
#                  has no shaderInt64 -- see engine/testing/spirv_scan.hpp's
#                  rule P4)
#   rng.slang      the port of core/rng.hpp's splitmix64 / next_float /
#                  next_gauss
#   dryden.slang   the port of world/medium.cpp's gust filter, imported by
#                  medium_update (advance) AND by both force kernels (sample)
#   mirrors.slang  the shared glm expression mirrors + effective_proxy_radius
#
# All three kernels are registered in tests/test_slang_layouts.cpp's
# kSpirvModules[] under the PARITY profile, like waves A and B.
# ------------------------------------------------------------------------------
spade_slang_kernel_variants(spade_compute medium_update)
spade_slang_kernel_variants(spade_compute rotors)
spade_slang_kernel_variants(spade_compute sensor_imu)
# The gnss.synthesize pass, over the GNSS slot space. Declared with variants for
# the same reason sensor_imu is: it is dispatched per sensor slot, so
# BackendDesc::workgroup_size must select a module compiled for that size rather
# than a fallback.
spade_slang_kernel_variants(spade_compute sensor_gnss)
# Module-API stage 3: the two field providers' kernels, one thread per WORLD.
# field_environment copies gravity and density into each world's field sample
# row; field_dryden writes the wind (mean wind plus the gust, after
# medium_update). Variants for the same reason as every per-dispatch kernel.
spade_slang_kernel_variants(spade_compute field_environment)
spade_slang_kernel_variants(spade_compute field_dryden)

# T11 fix-wave (review W7b): must come AFTER every spade_slang_kernel()/
# spade_slang_kernel_variants() call above (fp32_math_probe through field_dryden)
# -- see SpadeSlang.cmake's
#
# This comment USED TO SAY "ELEVEN in total" and "moved kCompiledSpirvKernelCount
# 10 -> 11". The generated value is 13 today. The count was restated here and
# then moved twice more without anyone touching this line.
# DO NOT REPLACE IT WITH 13. A CURRENT VALUE RESTATED IN PROSE IS GUARANTEED TO
# ROT; ONLY AN INVARIANT IS SAFE TO WRITE DOWN -- and the invariant is the
# ORDERING, which is the whole reason this comment exists. The number has exactly
# one home (the generated header) and exactly one checker
# (test_slang_layouts.cpp's SlangSpirv.EveryCompiledVariantIsScanned).
# own function comment
# for why the ordering is load-bearing. Writes kernel_manifest.gen.hpp's
# kCompiledSpirvKernelCount, which tests/test_slang_layouts.cpp asserts
# kSpirvModules[]'s size against.
spade_slang_write_kernel_manifest(spade_compute)

# ------------------------------------------------------------------------------
# S6 TASK 5 ADDITIONS -- state mirror + step recorder + the VulkanBackend seam.
# A single, clearly-delimited target_sources() block (appending to the
# add_library(spade_compute ...) call above rather than editing its own file
# list) because this file conflicts ADDITIVELY across S6 lanes (program note:
# "both add sources... coordinator resolves at merge"); keeping this task's
# additions in one place at the tail of the SPADE_VULKAN block is what makes
# that merge mechanical.
#
#   compute/vulkan/state_mirror.cpp  -- the device buffer table (every
#     registered walk entry + the derived buffers), upload()/readback().
#   compute/vulkan/step_recorder.cpp -- the per-substep step command buffer,
#     record-once-per-shape / submit-per-step.
#   compute/vulkan/backend.cpp       -- VulkanBackend (compute/vulkan/
#     backend.hpp, the Vulkan-FREE seam sim/simulation.hpp/.cpp includes):
#     composes the two above behind a PIMPL.
#
# S6 TASK 10 ADDITION: compute/vulkan/timestamps.cpp -- PassTimestamps, the
# per-pass GPU timing query-pool wrapper step_recorder.cpp now records into
# its one reusable command buffer. Listed here rather than in a new block of
# its own for the same reason state_mirror.cpp/step_recorder.cpp/backend.cpp
# already share one: all four are additive TUs inside the same SPADE_VULKAN
# gate, and step_recorder.cpp is this new file's only caller.
# ------------------------------------------------------------------------------
target_sources(spade_compute PRIVATE
    compute/vulkan/state_mirror.cpp
    compute/vulkan/step_recorder.cpp
    compute/vulkan/backend.cpp
    compute/vulkan/timestamps.cpp
)

# spade_sim needs spade::compute (S6 Task 5): Simulation
# (sim/simulation.hpp/.cpp) owns a std::unique_ptr<compute::VulkanBackend> and
# calls its create/upload/step/readback from simulation.cpp. PUBLIC, not
# PRIVATE: simulation.hpp's own public surface now names
# compute::BackendDesc (Simulation::create()'s new trailing parameter,
# DEFAULTED so every pre-Task-5 call site is unaffected -- see create()'s own
# doc comment) directly in a public function signature, so a consumer
# including sim/simulation.hpp needs compute/backend.hpp on its include path
# too, the same "public API exposes it, so the link is PUBLIC" reasoning
# every other target in this file already uses. compute/backend.hpp itself
# stays Vulkan-free (this task's design, matching this file's own Task-1
# note on that header), so this link does not put a Vulkan header on
# spade_sim's OWN public include surface -- only spade::compute's compiled
# objects onto the link line of whatever links spade::sim.
#
# SPADE_VULKAN=OFF -- gap closed, S6 Task 5 review round 1 finding I1. The
# ON branch above links spade_sim to spade::compute for the real
# VulkanBackend definitions (compute/vulkan/backend.cpp, inside
# spade_compute); this branch gives it the SAME symbols from a stub TU
# instead, compiled directly into spade_sim itself (no spade_compute target
# exists at all when this option is OFF, so there is nothing to link
# against). sim/simulation.cpp calls compute::VulkanBackend::create/upload/
# step/readback UNCONDITIONALLY -- the cpu/vulkan split is a RUNTIME branch
# on BackendDesc::kind, not a compile-time #ifdef -- so every executable
# that links spade::sim needs SOME definition of those symbols regardless of
# this option, or the link fails even though nothing at runtime would ever
# have taken the vulkan path. See compute/vulkan/backend_stub.cpp's own
# header for the full mechanism (its create() always returns
# Code::unavailable, matching VulkanContext::create()'s own report for "no
# Vulkan present" -- from a caller's perspective the two situations are the
# identical fact).
target_link_libraries(spade_sim PUBLIC spade::compute)
else()
target_sources(spade_sim PRIVATE compute/vulkan/backend_stub.cpp)
# sim/simulation.hpp includes these three Vulkan-free headers whatever this
# option says, and the ON branch installs them with the rest of compute/. With
# it OFF they must still ship, or the installed simulation.hpp cannot compile.
install(FILES compute/backend.hpp compute/grid_entry.hpp compute/step_params.hpp
    DESTINATION include/compute)
endif()

install(EXPORT spadeTargets
    FILE spadeTargets.cmake
    NAMESPACE spade::
    DESTINATION lib/cmake/spade
)

# ------------------------------------------------------------------------------
# spadeConfig.cmake / spadeConfigVersion.cmake (Task 8: the kat build wiring) --
# what makes `find_package(spade CONFIG REQUIRED)` work for an OUT-OF-TREE
# consumer (tests/consumer/). .github/workflows/spade.yml's install+consume
# CI step, which used to exercise this, was deleted 2026-09-18; scripts/test.ps1
# is this repo's gate now. install(EXPORT) above only emits spadeTargets.cmake, the raw target
# definitions; CMake's config mode looks for a file named spadeConfig.cmake (or
# spade<Version>Config.cmake) to find its way TO that file at all, and for a
# spadeConfigVersion.cmake to perform the compatibility check a versioned
# find_package(spade) call makes.
#
# configure_package_config_file, not a hand-rolled @PACKAGE_INIT@ substitution:
# it is what defines PACKAGE_PREFIX_DIR and opens the PACKAGE_INIT block
# ../cmake/spadeConfig.cmake.in starts with, which is what keeps every path
# inside that file relocatable to wherever a consumer's CMAKE_PREFIX_PATH
# happens to point rather than baked in as THIS build's absolute prefix.
#
# SameMajorVersion, version ${PROJECT_VERSION} (project(Spade VERSION 0.2.0),
# CMakeLists.txt): any 0.x.y request is accepted -- the same "no
# stability promise yet" posture 0.x carries everywhere else in this tree --
# and a 1.0.0 request would be rejected once the project actually reaches 1.0.
# ------------------------------------------------------------------------------
include(CMakePackageConfigHelpers)

configure_package_config_file(
    ${PROJECT_SOURCE_DIR}/cmake/spadeConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/spadeConfig.cmake
    INSTALL_DESTINATION lib/cmake/spade
)

write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/spadeConfigVersion.cmake
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY SameMajorVersion
)

install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/spadeConfig.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/spadeConfigVersion.cmake
    DESTINATION lib/cmake/spade
)

# ------------------------------------------------------------------------------
# spade_viewer_scenes (INT-4): the viewer's eight scenes and their setup, as a
# v1-free library. Built in EVERY configuration, not only with SPADE_BUILD_V1:
# the trajectory guard in spade_tests replays these scenes against the goldens
# in tests/golden/viewer/, and the Docker leg runs that guard with v1 off. The
# scenes feed golden digests, so they compile under spade_fp_strict like every
# other digest-feeding target (TD-3). Not installed: this is tool code, which
# the v1 harvest ports into successor scenes.
# ------------------------------------------------------------------------------
add_library(spade_viewer_scenes STATIC
    tools/viewer/scenes.cpp
    tools/viewer/setup.cpp
)
target_compile_features(spade_viewer_scenes PUBLIC cxx_std_23)
target_include_directories(spade_viewer_scenes PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/tools/viewer)
target_link_libraries(spade_viewer_scenes
    PUBLIC spade::sim
    PRIVATE $<BUILD_LOCAL_INTERFACE:spade_warnings> $<BUILD_LOCAL_INTERFACE:spade_fp_strict>
)

# ------------------------------------------------------------------------------
# spade_viewer (Task 14): the strangler viewer -- v1's frozen OpenGL renderer
# fed by v2 Simulation state, one demo scene at a time
# (scripts/demo.ps1). A TOOL, not a library: not installed, not
# exported, and guarded by SPADE_BUILD_V1 because it links v1's GL stack (the
# `Spade` target from src/, only built when SPADE_BUILD_V1=ON -- root
# CMakeLists.txt). With SPADE_BUILD_V1=OFF this whole block -- and therefore
# the viewer -- is absent from the build graph; nothing else here depends on
# it existing.
# ------------------------------------------------------------------------------
if(SPADE_BUILD_V1)
add_executable(spade_viewer
    tools/viewer/main.cpp
    tools/viewer/bridge.cpp
    tools/viewer/trajectory.cpp
)

# Same reasoning as spade_core/spade_state/... above: pinned per target, not
# via the ambient CMAKE_CXX_STANDARD the v1-frozen root CMakeLists.txt sets to
# 20 for src/ and examples/ -- spade_viewer's own sources need C++23 (v2's
# Result<T>/std::expected).
target_compile_features(spade_viewer PUBLIC cxx_std_23)

# Spade: v1's GL engine (glad/GLFW/glm/windows.h and the render loop this tool
# drives). spade::sim: the v2 Simulation this tool bridges FROM -- brings
# spade::physics/spade::world/spade::state/spade::core transitively, and glm's
# include dir a second time (harmless; same vendored target either way).
target_link_libraries(spade_viewer PRIVATE
    Spade
    spade::sim
    spade_viewer_scenes
)

# The trajectory mode reports process memory (GetProcessMemoryInfo).
if(WIN32)
    target_link_libraries(spade_viewer PRIVATE psapi)
endif()

# /W4 /WX for the whole target, EXCEPT tools/viewer/bridge.cpp below. Verified
# by this target's first build: bridge.cpp -- the one TU that also includes
# v1's own headers (Spade/Spade.hpp) -- does not survive /WX. The failure is
# entirely a vendor/v1 header ordering issue, not anything in this task's own
# code: glad.h (vendor/CMakeLists.txt's libigl-glad fetch) '#define's
# APIENTRY, Engine.hpp then includes <windows.h> AFTER glad.h, and
# minwindef.h's own APIENTRY definition collides -- warning C4005 "macro
# redefinition", fatal under /WX. v1's own `Spade` target never hits this
# because it links no warnings flags at all (src/CMakeLists.txt).
#
# Per the coordinator's resolution: isolate the relaxation to bridge.cpp
# ALONE rather than dropping /W4 /WX for the whole target. main.cpp and
# scenes.cpp are v1-free and compile clean under the same policy every other
# v2 target uses; only /WX- is added for bridge.cpp (via the source-level
# COMPILE_OPTIONS below, appended after -- and so overriding -- the target's
# own /WX), not a warning-level drop, so bridge.cpp's warnings still print
# during a build, they simply do not fail it.
target_link_libraries(spade_viewer PRIVATE $<BUILD_LOCAL_INTERFACE:spade_warnings>)
# S5 T9 /WX ticket: the /WX- override that used to sit here is REMOVED --
# bridge.cpp now includes <windows.h> before anything that reaches glad.h
# transitively (see bridge.cpp's own file-top comment for the mechanism), and
# that alone lets bridge.cpp survive full /W4 /WX like every other v2 TU in
# this target. Literal before/after build output is in this task's report.
endif()
