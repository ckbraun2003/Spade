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
# spade_tests AND spade_bench ARE ALSO LINKED (spade/tests/CMakeLists.txt) --
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
# (this exact gap is what spade/tests/consumer/'s out-of-tree smoke caught).
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
# rather than in this export: spade/cmake/spadeConfig.cmake.in, AFTER
# confirming yaml-cpp::yaml-cpp exists (its own explicit check, mirroring
# glm's), does `set_property(TARGET spade::world APPEND PROPERTY
# INTERFACE_LINK_LIBRARIES yaml-cpp::yaml-cpp)` -- legal precisely because
# that runs inside find_package(spade)'s OWN config-processing, on an already
# fully-formed IMPORTED target, entirely outside Spade's own install(EXPORT)
# validation. The observable effect for a consumer is identical to what a
# working install-interface entry would have produced: linking spade::sim (or
# anything above spade::world) pulls yaml-cpp::yaml-cpp onto their final link
# line transitively. spade/tests/consumer/ is the worked example that calls
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

install(DIRECTORY world/ DESTINATION include/world FILES_MATCHING PATTERN "*.hpp")

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
# spade/tests/ only, and deliberately absent from every target and every
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

target_link_libraries(spade_sim PUBLIC spade::physics spade::vehicles)
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
    vehicles/model_type.cpp
    vehicles/quadrotor.cpp
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

install(EXPORT spadeTargets
    FILE spadeTargets.cmake
    NAMESPACE spade::
    DESTINATION lib/cmake/spade
)

# ------------------------------------------------------------------------------
# spadeConfig.cmake / spadeConfigVersion.cmake (Task 8: the kat build wiring) --
# what makes `find_package(spade CONFIG REQUIRED)` work for an OUT-OF-TREE
# consumer (spade/tests/consumer/, .github/workflows/spade.yml's install+consume
# CI step). install(EXPORT) above only emits spadeTargets.cmake, the raw target
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
# spade/CMakeLists.txt): any 0.x.y request is accepted -- the same "no
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
# spade_viewer (Task 14): the strangler viewer -- v1's frozen OpenGL renderer
# fed by v2 Simulation state, one demo scene at a time
# (spade/scripts/demo.ps1). A TOOL, not a library: not installed, not
# exported, and guarded by SPADE_BUILD_V1 because it links v1's GL stack (the
# `Spade` target from spade/src, only built when SPADE_BUILD_V1=ON -- root
# CMakeLists.txt). With SPADE_BUILD_V1=OFF this whole block -- and therefore
# the viewer -- is absent from the build graph; nothing else here depends on
# it existing.
# ------------------------------------------------------------------------------
if(SPADE_BUILD_V1)
add_executable(spade_viewer
    tools/viewer/main.cpp
    tools/viewer/bridge.cpp
    tools/viewer/scenes.cpp
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
)

# /W4 /WX for the whole target, EXCEPT tools/viewer/bridge.cpp below. Verified
# by this target's first build: bridge.cpp -- the one TU that also includes
# v1's own headers (Spade/Spade.hpp) -- does not survive /WX. The failure is
# entirely a vendor/v1 header ordering issue, not anything in this task's own
# code: glad.h (vendor/CMakeLists.txt's libigl-glad fetch) '#define's
# APIENTRY, Engine.hpp then includes <windows.h> AFTER glad.h, and
# minwindef.h's own APIENTRY definition collides -- warning C4005 "macro
# redefinition", fatal under /WX. v1's own `Spade` target never hits this
# because it links no warnings flags at all (spade/src/CMakeLists.txt).
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
