# ==============================================================================
# READ THIS BEFORE ADDING A TEST HERE. Two facts about where these tests run,
# neither visible from inside a test file.
#
# ------------------------------------------------------------------------------
# 1. WHAT RUNS THIS SUITE
# ------------------------------------------------------------------------------
# The root CMakeLists.txt adds this directory, and scripts/test.ps1
# (`ctest -L spade`) is the gate: green on both presets
# (docs/design/test-docs/02-build-and-gate.md). There is no hosted CI. Every
# TEST() is registered as its own ctest case below, so never run
# spade_tests.exe directly.
#
# Consumers build against an INSTALLED Spade through
# `find_package(spade CONFIG)`, never against this tree; tests/consumer/ is
# the worked example, and proves something no test here can: that the
# installed tree is usable from outside. Nothing runs it today
# (docs/design/backlog.md).
#
# ------------------------------------------------------------------------------
# 2. GPU COVERAGE IS DEVELOPER-MACHINE-ONLY. SAY SO WHEN YOU CITE IT.
# ------------------------------------------------------------------------------
# Tests in Gpu* suites get the `gpu` label (AppendSpadeLabels.cmake) and open
# with `if (!vulkan_available()) GTEST_SKIP();` (test-docs TD-6). No hosted CI
# job this suite ever ran on had a GPU, so no gate has ever observed one of
# them pass or fail; they run where a developer has a device. And
# `--no-tests=error` only fires on ZERO REGISTERED tests, so a battery that
# always skips is invisible to red/green. They are still the only check on
# cross-backend determinism, which SL18 requires.
#
# THE EIGHT THAT ALWAYS RUN ARE NOT AN OVERSIGHT, AND THEY ARE WHY THIS IS A
# DECLARED LIMIT RATHER THAN A HOLE. These sites in the GPU test files are
# deliberately NOT device-gated; each is a CPU-side half that needs no device:
#   test_compute_context.cpp  ComputeBackendAvailability.ForcedUnavailableReturnsUnavailable
#   test_gpu_invariance.cpp   WorkgroupSizeContract.AllowedSetMatchesCompiledVariants
#                             WorkgroupSizeContract.UnsupportedSizeIsRejectedByName
#                             BackendSeamReproof.CpuPathCorpusDigestsUnchangedAtTip
#   test_gpu_parity.cpp       ParityCorpus.EveryCorpusScenarioIsInTheParitySet
#                             ParityGeometry.HeterogeneousGeometrySetMatchesSoloRuns
#                             ParityChaos.ShowerPileAmplifiesOneUlpOnTheCpuAlone
#   test_gpu_state_mirror.cpp BackendKnobInvariance.CpuBackendReproducesTodaysDigestsAcrossTheFullCorpus
#
# ParityCorpus.EveryCorpusScenarioIsInTheParitySet is the load-bearing one: it
# is a MEMBERSHIP FLOOR over the parity corpus, so a device-less run still
# catches a scenario silently dropping OUT of the set the GPU tests would
# compare. What such a run cannot see is whether the two backends AGREE; what
# it can still see is that the question is being asked about every scenario.
# Keep that property if you touch these files.
#
# If you add a GPU-gated test: it is on you to run it on a machine with a
# device, and to say in the commit that you did. Nothing downstream will.
# ==============================================================================

add_executable(spade_tests
    test_core.cpp
    test_math.cpp
    test_fp32_math.cpp
    test_path_text.cpp
    test_sdf.cpp
    test_state.cpp
    test_snapshot.cpp
    test_rng_medium.cpp
    test_integrator.cpp
    test_contacts.cpp
    test_grid.cpp
    test_forces.cpp
    test_dryden.cpp
    test_rotor.cpp
    test_rotor_wake.cpp
    test_motor.cpp
    test_battery.cpp
    test_propeller.cpp
    test_composite_inertia.cpp
    test_propulsion_steady.cpp
    test_airframe_compile.cpp
    test_quadrotor.cpp
    test_imu.cpp
    test_gnss.cpp
    test_world_file.cpp
    test_scene_file.cpp
    test_determinism.cpp
    test_sim_medium.cpp
    test_module_schedule.cpp
    test_design_frame.cpp
    test_model_registry.cpp
    test_m1b_bar.cpp
    test_render_target.cpp
    test_render_scene.cpp
    test_render_tessellate.cpp
    test_render_raster.cpp
    test_render_shading.cpp
    test_render_gltf.cpp
    test_render_csg.cpp
    test_render_csg_raster.cpp
    test_render_shadow.cpp
    test_render_raymarch.cpp
    test_render_agreement.cpp
    test_render_agreement_matrix.cpp
    test_render_field.cpp
    test_objects_graph.cpp
    test_objects_component.cpp
    test_objects_determinism.cpp
    test_objects_serialize.cpp
    test_objects_behavior.cpp
    test_render_velocity.cpp
    test_objects_spawn_helpers.cpp
    test_transfer_register.cpp
    # INT-4: replays spade_viewer's eight scenes (spade_viewer_scenes, v1-free)
    # against the trajectory goldens in golden/viewer/. One case per scene.
    test_viewer_trajectories.cpp
    # Composer task 5: assets/worlds/ and assets/scenes/ hold the viewer scenes
    # as files; each runs beside the viewer's own setup, row for row.
    test_viewer_successors.cpp
    # The gate scene's lobs pass through its ring (the live smoke's finding).
    test_viewer_gate.cpp
    # spade::scene's composer: compose_transform, compose, compose_file and
    # instantiate (the drone-builder joint spec, section 3).
    test_scene_compose.cpp
)

# test_compute_context.cpp (S6 Task 1) -- includes compute/vulkan/context.hpp
# directly, which needs volk on ITS OWN include path (see engine/CMakeLists.txt's
# spade_compute target comment: its volk/Vulkan::Headers links are PRIVATE +
# BUILD_INTERFACE-wrapped, which reaches spade_compute's OWN compilation but
# does not propagate to a consumer like this executable just by linking
# spade::compute -- the identical reason this target already links
# yaml-cpp::yaml-cpp directly below for engine/testing/scenario_file.hpp).
# Compiled ONLY when SPADE_VULKAN built spade_compute at all -- with it OFF
# there is no VulkanContext to test and no volk/Vulkan-Headers target to link
# against.
#
# test_slang_layouts.cpp (S6 Task 3) joins it under the same gate for a related
# but distinct reason: it includes the D9 GENERATED headers
# (layout_check.gen.hpp, bindings.gen.hpp, and one <kernel>.spv.gen.hpp for
# every compiled kernel -- it named pipeline_smoke's too until S6 Task 8 retired
# that stub), which cmake/SpadeSlang.cmake only generates when spade_compute
# exists. It reaches
# them through spade::compute's PUBLIC $<BUILD_INTERFACE:...> include directory
# -- and through the add_dependencies() edge on the generator targets, which is
# what orders the generation before this TU compiles. Unlike
# test_compute_context.cpp it executes NOTHING on a device and never skips.
#
# test_gpu_fp32_math.cpp (S6 Task 4) joins them under the same gate: it
# includes compute/vulkan/probe_runner.hpp (volk, like test_compute_context.cpp)
# AND the generated fp32_math_probe.spv.gen.hpp (like test_slang_layouts.cpp),
# so it needs both reasons above at once. Its GpuFp32Math.* suites execute on
# the device and each begins with the mandated
# `if (!vulkan_available()) GTEST_SKIP()`.
#
# test_gpu_state_mirror.cpp (S6 Task 5) joins the same gate: it exercises
# compute::VulkanBackend (state_mirror.cpp/step_recorder.cpp/backend.cpp,
# compiled into spade_compute only when SPADE_VULKAN is ON) directly, plus
# sim/simulation.hpp's vulkan path. Its A7 cpu-leg test
# (BackendKnobInvariance.*) is host-only and does not need a device, but the
# TU as a whole still needs spade::compute's headers on its include path, so
# it is gated here rather than split across two files.
#
# test_gpu_parity.cpp (S6 Task 6) joins the same gate, for the union of the
# reasons above: it drives sim/simulation.hpp's vulkan path (spade::compute's
# headers), reads back a compute::StepWitness, and its GpuParity* suites execute
# on the device behind the mandated `if (!vulkan_available()) GTEST_SKIP()`.
#
# THREE of its cases are deliberately HOST-ONLY and never skip:
#
#   ParityCorpus.*    the corpus completeness table -- reads a directory and a
#                     list of names, never a device
#   ParityGeometry.*  (S6 Task 6b) the heterogeneous-worlds solo-run digest
#                     lock, a pure CPU-backend comparison
#   ParityChaos.*     (S6 Task 7) the shower pile's one-ulp amplification
#                     control, two CPU runs
#
# THE COUNT HAS MOVED TWICE AND THIS COMMENT NOW SAYS WHY, since it going stale
# is the review minor (T7 M5) S6 Task 8 was asked to close. It read THREE
# through Task 6b; Task 7 added ParityChaos and made it four without updating
# the line; Task 8 deleted ParityGate -- the unported-pass refusal, which had
# nothing left to refuse once every schedule slot got a kernel -- and it is
# three again, for a different three.
#
# So a CI runner with no GPU still exercises the parts that do not need one.
# The `Gpu` NAME PREFIX is what attaches the ctest "gpu" label
# (tests/AppendSpadeLabels.cmake's `^Gpu[^.]*\.` rule), which is exactly why
# these three do not carry it -- see test_gpu_parity.cpp's own section 0.
#
# test_gpu_invariance.cpp (S6 Task 9) joins the same gate for the same union of
# reasons: it drives sim/simulation.hpp's vulkan path with varying BackendDesc
# values, reads back compute::GridEntryRow via vulkan_grid_entries(), and its
# GpuInvarianceTest/GpuGridSortDeepTest suites execute on the device behind the
# mandated `if (!vulkan_available()) GTEST_SKIP()`. ONE case,
# BackendSeamReproof.CpuPathCorpusDigestsUnchangedAtTip, is host-only and never
# skips -- the cpu-path corpus-digest re-proof needs no device, exactly like
# ParityCorpus/ParityGeometry/ParityChaos above.
#
# test_gpu_module_schedule.cpp (module-API plan, stage 2) checks that the
# Vulkan step records the compiled schedule's passes. It includes
# compute/vulkan/context.hpp for vulkan_available(), so it lives here rather
# than in test_module_schedule.cpp, which builds with SPADE_VULKAN off too.
if(SPADE_VULKAN)
    target_sources(spade_tests PRIVATE
        test_compute_context.cpp
        test_slang_layouts.cpp
        test_gpu_fp32_math.cpp
        test_gpu_state_mirror.cpp
        test_gpu_parity.cpp
        test_gpu_invariance.cpp
        test_gpu_module_schedule.cpp
    )
endif()

# -----------------------------------------------------------------------
# Plan C task C1: SL10's same-path invariant across the TargetSink seam.
#
# GATED ON SPADE_BUILD_SANDBOX BECAUSE THE FLAG MUST STAY A REAL OFF SWITCH.
# ../sandbox/target_sink.hpp does not exist as a compiled thing when the
# sandbox is off, and a test suite that failed to configure at OFF would make
# the flag a lie -- C0 introduced it precisely so the target can be omitted
# entirely.
#
# ⚠⚠ AND A GATED TEST THAT VANISHES QUIETLY IS THE 61-GPU-TEST DEFECT AGAIN:
# `--no-tests=error` only fires on ZERO registered tests, so a battery that
# disappears at a flag is invisible to red/green. THE COUNT IS THE EVIDENCE --
# assert the count, never the rate, because a missing test reads exactly like a
# passing one.
#
# THE INVARIANT, RATHER THAN TWO NUMBERS: the ON total minus the OFF total is
# EXACTLY the cases registered by THE SANDBOX TEST SOURCES LISTED BELOW, and
# nothing else moves with this flag. That stays true when someone adds a test;
# a pair of baked totals would not.
#
# ⚠ AND THE SENTENCE ABOVE USED TO NAME ONE FILE. C2 added a second, which
# turned a correct invariant into a false one WITHOUT ANY TEST GOING RED --
# the count still balances, and only the prose was wrong. A claim that names a
# MEMBER goes stale when the SET grows; a claim that names the SET does not.
# This is the same repair shape as every other stale count in this estate, and
# it is here because the file that hosts the rule is not exempt from it.
#
# (An earlier draft of this comment stated "825 at ON and 821 at OFF" -- both
# WRITTEN BEFORE EITHER WAS MEASURED, and both already wrong: ON measured 829.
# A number in a comment is a claim, and this one was a guess wearing a
# measurement's clothes. It is also the third time in one night this realm
# wrote something that named a MOMENT rather than a CONDITION and went stale
# immediately; the fix is always the same shape.)
#
# ⚠ A SOURCE-LEVEL REACH INTO ../sandbox, AND IT IS THE THIRD IN THIS FILE.
# The alternative was to make the sandbox a library so the test could LINK it,
# which would have added a twelfth target to a release identity SL2a states as
# eleven library modules, and changed the answer to "how many targets are
# there" for everyone downstream. An include path costs less and claims less.
# The header is header-only by design so no link is needed at all.
#
# This is what SL15b ("headless is the test surface") requires structurally
# rather than by discipline: an executable's internals are not reachable from
# a test, so a seam that lives only inside spade_sandbox cannot be asserted by
# anything, however carefully it is written.
# -----------------------------------------------------------------------
if(SPADE_BUILD_SANDBOX)
    target_sources(spade_tests PRIVATE
        test_sandbox_target_sink.cpp
        # C2. The two pieces of the window that are NOT the window: the single
        # channel-order statement both sinks call, and OrbitCamera. Neither
        # needs a display, which is the whole reason they are out here -- a
        # windowed path can be COMPILED on both platforms and EXECUTED on
        # neither gate, so anything that stays inside the window is untestable
        # by construction.
        #
        # This file also carries L301's discharge: a KNOWN-ANSWER case whose
        # expectations are hand-written literals rather than a second
        # expression of the mapping under test.
        test_sandbox_camera_and_channels.cpp
        # The builder: placement, picking, dragging, delete/duplicate, the
        # UI-capture gate and the upload contract. All of it decided by pure
        # functions in sandbox/builder_scene.hpp, so all of it runs with no
        # display -- which is the entire reason those functions are not in the
        # window loop.
        test_sandbox_builder.cpp
        # The drone sim box: the test stand's pin, the attitude controller and
        # mixer (checked against the engine, not only against their own
        # formulas), the accumulator cap and the rebuild carry.
        test_sandbox_drone.cpp
        # The drone's view: the air field (medium + rotor wakes), the heatmap
        # palette and slice, and the drone's real-size parts -- including one
        # exact-pixel check through the CPU rasterizer.
        test_sandbox_drone_view.cpp
        # The live smoke's verdict machinery (journal, declared-step ledger,
        # expiring known-open entries, inject hooks), its artefact names, the
        # ffmpeg command, the title cards and the PNG encoder. The tour itself
        # needs a display and runs from scripts/live-smoke.ps1, not ctest.
        test_sandbox_live_smoke.cpp
        # The built-in scenes' physics records (a bridge until world file v3),
        # pinned to spade_viewer's make_scene() records field by field.
        test_sandbox_builtin_records.cpp
        # What the live smoke's scene main functions measure: bounce's
        # rebounds, gate's ring crossings.
        test_sandbox_scene_checks.cpp
    )
    target_include_directories(spade_tests PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../sandbox)
endif()

# GlRenderer on a real GL context (SR-13 back-face culling first). GPU-gated:
# its suite is GpuGlRenderer, so it carries the `gpu` label, runs on the
# development box and skips by name where no display or GL 4.3 context exists
# (TD-13). It needs spade_render_gl and the glfw target, which is fetched only
# when the sandbox or v1 is built.
#
# GLFW_INCLUDE_NONE: glad declares GL here, and glfw3.h must not pull in
# <GL/gl.h> beside it. glad is linked here, not through render_gl: the test
# calls GL itself, and render_gl keeps its own private function table.
if(SPADE_RENDER_GL AND TARGET glfw)
    target_sources(spade_tests PRIVATE test_gpu_gl_renderer.cpp)
    target_link_libraries(spade_tests PRIVATE spade::render_gl glfw glad)
    target_compile_definitions(spade_tests PRIVATE GLFW_INCLUDE_NONE)
endif()

# spade_fp_strict (plain link, not BUILD_LOCAL_INTERFACE -- spade_tests is an
# executable, not an installed/exported target, so there is no export-set
# concern to guard against): test TUs re-derive reference arithmetic of their
# own (test_dryden.cpp, test_rotor.cpp's 25 EXPECT_FLOAT_EQ assertions at
# 4-ULP tolerance) and compile header-only engine code inline (core/rng.hpp's
# next_gauss, sensors/rings.hpp, state/layout.hpp) -- fp32 expressions that
# feed a determinism-sensitive assertion just as much as anything inside the
# five libraries this target links. Without this, gcc/clang would contract
# those expressions while the libraries themselves do not (engine/
# CMakeLists.txt's spade_fp_strict comment), a mismatch this PR's first-ever
# Linux/gcc CI run would otherwise hit blind.
#
# yaml-cpp (S5 Task 7): engine/testing/scenario_file.hpp parses the committed
# scenario corpus (tests/golden/scenarios/*.scenario.yaml) and is compiled INTO
# THIS TARGET -- it is header-only test support, like replay.hpp beside it, and
# is not part of any library. spade::world links yaml-cpp PRIVATE (its parser
# is world_file.cpp's alone and no yaml-cpp type crosses its API), so nothing
# propagates the headers here and this target has to name it itself.
#
# nlohmann_json (S7a Task R4): test_render_gltf.cpp builds/mutates glTF JSON
# documents programmatically for its malformed-input cases rather than
# hand-editing fragile raw string literals, and independently re-parses the
# committed gate-ring.gltf fixture to pin its accessor counts. render/gltf.cpp
# also uses nlohmann_json, but PRIVATE to spade_render (it never appears in
# gltf.hpp's signature), so -- exactly like yaml-cpp above -- nothing
# propagates it here and this target names it itself. INTERFACE-only
# (header-only, no compiled archive), unlike yaml-cpp, so there is no
# yaml-cpp-style final-link-line concern either way.
#
# NOT a packaging change: spade_tests is an executable that never leaves the
# build tree, so there is no export set, no install rule and no consumer to
# consider. Test support never ships.
target_link_libraries(spade_tests PRIVATE
    spade::core
    spade::world
    spade::state
    spade::objects
    spade::render
    spade::physics
    spade::vehicles
    spade::sim
    spade_warnings
    spade_fp_strict
    yaml-cpp::yaml-cpp
    nlohmann_json::nlohmann_json
    GTest::gtest_main
    # The viewer's scenes and setup, for test_viewer_trajectories.cpp. Built in
    # every configuration and v1-free (engine/CMakeLists.txt).
    spade_viewer_scenes
    spade::scene
)

# spade::compute + volk + Vulkan::Headers (S6 Task 1): same
# not-a-packaging-change posture as yaml-cpp above -- spade_tests is an
# executable that never leaves the build tree, so no export-set concern
# applies to a plain (non-genex) link here, unlike engine/CMakeLists.txt's
# spade_compute target. Guarded by SPADE_VULKAN alongside
# test_compute_context.cpp's conditional target_sources() above: neither the
# source file nor these three targets exist when it is OFF.
if(SPADE_VULKAN)
    target_link_libraries(spade_tests PRIVATE
        spade::compute
        volk
        Vulkan::Headers
    )
    # Build-ORDER edge onto the D9 generators (S6 Task 3). The generated
    # headers' include DIRECTORY already arrives with spade::compute's PUBLIC
    # $<BUILD_INTERFACE:...> usage requirement; this is the separate guarantee
    # that they have been WRITTEN before test_slang_layouts.cpp compiles. See
    # cmake/SpadeSlang.cmake's spade_slang_attach_generated() for why it is
    # stated rather than inferred.
    spade_slang_attach_generated(spade_tests)
endif()

# The golden corpus -- tests/golden/scenarios/*.scenario.yaml (the determinism
# scenarios, each carrying its own expected digest) and
# tests/golden/worlds/*.world.yaml (the worlds they name) -- located by an
# absolute path baked in at configure time rather than by a CWD-relative one.
# gtest_discover_tests below does set WORKING_DIRECTORY, but the no-CWD rule
# is the right posture for a data file a test MUST find: a
# compile definition cannot be defeated by running the exe from elsewhere.
#
# SPADE_ENGINE_DIR / SPADE_TESTS_DIR (Task 20's test_m1b_bar.cpp): the same
# discipline, for the M1B "fixed-step"/"headless" canaries' source-tree scan
# -- an absolute path baked in at configure time, not a CWD-relative guess.
#
# SPADE_ENGINE_DIR is NORMALIZED (cmake_path(NORMALIZE), not a bare
# "${CMAKE_CURRENT_SOURCE_DIR}/../engine" concatenation): an un-normalized
# path bakes a literal "tests/../engine" segment into the compiled string,
# which means the string itself contains a "tests" path COMPONENT even
# though it resolves to engine/. test_m1b_bar.cpp's source-tree scanner
# filters by path component name (skipping "tools" today); if a future skip
# list ever added "tests" -- an entirely reasonable thing to want to
# exclude -- an un-normalized SPADE_ENGINE_DIR would match on that spurious
# component and silently skip the ENTIRE scan, not just a real tests/
# subdirectory under engine/ (there is none). Normalizing removes the ".."
# before it can be baked in, so the string the scanner sees never contains a
# component that isn't a real directory on the path to engine/.
cmake_path(SET SPADE_ENGINE_DIR_NORMALIZED NORMALIZE "${CMAKE_CURRENT_SOURCE_DIR}/../engine")
# The same normalization, for the content test_viewer_successors.cpp reads.
cmake_path(SET SPADE_ASSETS_DIR_NORMALIZED NORMALIZE "${CMAKE_CURRENT_SOURCE_DIR}/../assets")

target_compile_definitions(spade_tests PRIVATE
    SPADE_GOLDEN_DIR="${CMAKE_CURRENT_SOURCE_DIR}/golden"
    SPADE_ENGINE_DIR="${SPADE_ENGINE_DIR_NORMALIZED}"
    SPADE_TESTS_DIR="${CMAKE_CURRENT_SOURCE_DIR}"
    SPADE_ASSETS_DIR="${SPADE_ASSETS_DIR_NORMALIZED}"
    # Where opt-in test output goes: the build tree, never the source tree.
    # Writers: RasterGolden's frame dump (SPADE_RENDER_DUMP_FRAMES) and
    # AgreementMatrix's measurements (SPADE_AGREEMENT_MEASURE).
    SPADE_TEST_OUTPUT_DIR="${CMAKE_CURRENT_BINARY_DIR}/test-output"
)

# The measurement build's inverted rule P3 (SPADE_MEASURE_UNPINNED_DENORMS, the
# top-level CMakeLists.txt): SlangSpirv.FloatControlsPinned reads it here, on
# spade_tests only, so a default build's targets are untouched.
if(SPADE_MEASURE_UNPINNED_DENORMS)
    target_compile_definitions(spade_tests PRIVATE SPADE_MEASURE_UNPINNED_DENORMS=1)
    # M1's probe (GpuDenormProbe): its kernel exists only in this build.
    if(SPADE_VULKAN)
        target_sources(spade_tests PRIVATE test_gpu_denorm_probe.cpp)
    endif()
endif()

include(GoogleTest)
# WORKING_DIRECTORY is gtest_discover_tests()'s OWN top-level keyword, not a
# PROPERTIES entry (S5 T9 ticket B, first of two root causes -- see
# AppendSpadeLabels.cmake for the second): it used to sit inside PROPERTIES,
# which cmake_parse_arguments -- used internally by gtest_discover_tests() to
# split its own arguments -- has no concept of "inside PROPERTIES" as a
# nested scope for. It recognizes ITS OWN keywords (WORKING_DIRECTORY among
# them, see CMake's own Modules/GoogleTest.cmake) WHEREVER they appear in the
# argument list, so the old call's `PROPERTIES LABELS <labels>
# WORKING_DIRECTORY ${CMAKE_SOURCE_DIR} TIMEOUT 60` had its own
# WORKING_DIRECTORY token silently reinterpreted as gtest_discover_tests()'s
# TOP-LEVEL WORKING_DIRECTORY argument, prematurely terminating the
# PROPERTIES capture right there -- everything after it (TIMEOUT and 60)
# fell into cmake_parse_arguments's UNPARSED_ARGUMENTS bucket, which
# gtest_discover_tests() never reads, and was silently dropped. Moving
# WORKING_DIRECTORY to its correct, dedicated top-level slot (this call)
# removes that collision; TIMEOUT below (a single scalar value, no embedded
# semicolon) now reaches CTest intact -- proof is in this task's report.
#
# LABELS is deliberately NOT listed here even though it has none of
# WORKING_DIRECTORY's collision problem -- see AppendSpadeLabels.cmake,
# included below, for the SECOND, independent bug that made it necessary to
# set LABELS a different way.
# NO_PRETTY_VALUES: by default GoogleTestAddTests.cmake registers a value-
# parameterized test under GetParam()'s PRINTED VALUE instead of the name its
# TestParamInfo generator assigns, so `ctest -N`/`ctest -R` would show a
# different name from the one --gtest_filter selects. This makes CTest use the
# real name everywhere. No TEST_P suite exists today (the one that needed this,
# a KAT-world agreement matrix, left this suite on 2026-10-01); the flag stays
# so the next one registers correctly without anyone rediscovering why.
gtest_discover_tests(spade_tests
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    NO_PRETTY_VALUES
    # DISCOVERY_TIMEOUT is NOT the same knob as PROPERTIES TIMEOUT below.
    # TIMEOUT 60 bounds how long each discovered test may RUN; this bounds how
    # long `spade_tests --gtest_list_tests` may take to ENUMERATE them, and its
    # CMake default is a mere 5 seconds. S7a's render suite grew this one
    # binary by ~750 tests across ten new test_render_*.cpp files, and on a
    # GitHub windows-latest runner the enumeration crossed that 5s line --
    # "Result: Process terminated due to timeout" at link time, so the job
    # died before a single test ran. It stayed invisible locally because this
    # box enumerates faster than the shared runner.
    DISCOVERY_TIMEOUT 180
    PROPERTIES
        TIMEOUT 60
)

# SECOND ROOT CAUSE, and why LABELS is set here instead of above (S5 T9
# ticket B). Proved with a standalone repro script (this task's report has
# the transcript) rather than by reading GoogleTestAddTests.cmake alone:
# `PROPERTIES LABELS "spade;gpu"` builds a CORRECTLY escaped list value
# (`LABELS;spade\;gpu`, the `\;` meaning "one element containing a literal
# semicolon") all the way through gtest_discover_tests()'s own
# cmake_parse_arguments call, through the -D-forwarded cache variable
# GoogleTestAddTests.cmake's script-mode block re-reads, and into that
# module's OWN oneValueArgs re-capture -- the escape survives every one of
# those hops intact. It is LOST at the very last step:
# GoogleTestAddTests.cmake's write_test_to_file() forwards it into
# set_tests_properties() via `add_command(set_tests_properties ...
# ${arg_TEST_PROPERTIES})`, an UNQUOTED variable expansion inside a function
# call's argument list -- and CMake's unquoted-expansion argument-splitting
# rule (distinct from list()/foreach(IN LISTS)'s own escape-aware iteration)
# splits on EVERY literal `;` it finds, `\;` or not. "spade;gpu" -- meant to
# survive as one PROPERTIES value -- flattens into two separate, unpaired
# tokens, and the generated script ends up calling
# `set_tests_properties(... LABELS spade gpu)`: LABELS pairs with "spade"
# only, "gpu" dangles with no value and is silently dropped by
# set_tests_properties. This is a real limitation of GoogleTestAddTests.cmake
# itself (upstream CMake, not this project's code) -- no keyword reordering
# on our side can fix a bug inside its own internal forwarding.
#
# THE WORKAROUND: GoogleTest.cmake's own docs name the escape hatch --
# "If more fine-grained test control is needed, custom content may be
# provided through an external CTest script using the TEST_INCLUDE_FILES
# directory property. The set of discovered tests is made accessible to such
# a script via the <target>_TESTS variable." AppendSpadeLabels.cmake is that
# script: CTest evaluates TEST_INCLUDE_FILES entries in append order, so it
# runs AFTER gtest_discover_tests()'s own generated include (appended first,
# inside that function) has already `set(spade_tests_TESTS ...)` -- a REAL
# CMake list built by this module's own list-construction code, not
# reconstructed through the broken unquoted-expansion path -- and calls
# set_tests_properties() with a QUOTED "spade;gpu" literal written directly in
# THIS repo's own source text, which never needs the escape trick that
# breaks: a quoted argument is never list-split, regardless of what
# characters it contains.
set_property(DIRECTORY APPEND PROPERTY TEST_INCLUDE_FILES
    "${CMAKE_CURRENT_SOURCE_DIR}/AppendSpadeLabels.cmake"
)

# ------------------------------------------------------------------------------
# spade_bench: manual google-benchmark harness (bench/bench_core.cpp,
# bench/bench_sim.cpp). Deliberately NOT passed to gtest_discover_tests/
# add_test -- see either file's header comment for the "not ctest, run by
# hand" policy, in force until S6.
#
# spade::sim (Task 21): bench_sim.cpp benchmarks Simulation::step() itself, so
# it needs the whole stack sim carries PUBLICly -- physics, world, state,
# vehicles, core. spade::core stays listed too even though sim now pulls it
# in transitively: bench_core.cpp's own dependency (math_ops.hpp) predates
# sim's, and this target's link list should say what each file in it actually
# uses rather than rely on one file's transitive closure covering another's.
# ------------------------------------------------------------------------------
add_executable(spade_bench
    bench/bench_core.cpp
    bench/bench_sim.cpp
    bench/bench_render.cpp
)

# spade_fp_strict here too, same reasoning as spade_tests above: bench_core.cpp
# calls spade::math::integrate_orientation directly (header-only math_ops.hpp
# inlines into this TU) and bench_sim.cpp builds scenario state inline --
# both are fp32 expressions this executable evaluates itself, not just calls
# into a pre-built library for.
# spade::render (base-package phase W6c, 2026-09-08): bench_render.cpp calls
# spade::render::render() directly to measure the CPU rasterizer's per-pixel
# fill cost -- the render side of the MV3 render-scale-cap question, whose
# present side the editor realm measured as neutral (1.03x). Listed explicitly
# rather than leaned on transitively, per this target's own stated convention
# that the link list should say what each file in it actually uses.
target_link_libraries(spade_bench PRIVATE
    spade::core
    spade::sim
    spade::render
    spade_warnings
    spade_fp_strict
    benchmark::benchmark
)

# NO spade::compute/volk/Vulkan::Headers LINK HERE, DELIBERATELY (S6 Task 10).
# bench_sim.cpp's new GPU families name compute::BackendDesc/BackendKind/
# PassDurationsNs -- all three Vulkan-FREE (compute/backend.hpp, already
# reachable via spade::sim's own PUBLIC link) -- and skip gracefully by
# catching Simulation::create()'s Code::unavailable rather than calling
# compute::vulkan_available() directly, so this file never includes a Vulkan
# header at all (see bench_sim.cpp's own header comment, "THE THIRD AND
# FOURTH SWEEPS", for why that distinction matters: spade_bench compiles
# bench_sim.cpp UNCONDITIONALLY, unlike spade_tests's GPU-touching test files,
# which are added to that target's sources only inside the SPADE_VULKAN guard
# immediately above -- so an unconditional Vulkan-header include here would
# break the SPADE_VULKAN=OFF configuration).
