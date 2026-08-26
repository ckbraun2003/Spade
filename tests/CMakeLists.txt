add_executable(spade_tests
    test_core.cpp
    test_math.cpp
    test_fp32_math.cpp
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
    test_quadrotor.cpp
    test_imu.cpp
    test_world_file.cpp
    test_determinism.cpp
    test_m1b_bar.cpp
    test_render_target.cpp
    test_render_scene.cpp
    test_render_tessellate.cpp
    test_render_raster.cpp
    test_render_shading.cpp
    test_render_gltf.cpp
    test_render_csg.cpp
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
if(SPADE_VULKAN)
    target_sources(spade_tests PRIVATE
        test_compute_context.cpp
        test_slang_layouts.cpp
        test_gpu_fp32_math.cpp
        test_gpu_state_mirror.cpp
        test_gpu_parity.cpp
        test_gpu_invariance.cpp
    )
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
    spade::render
    spade::physics
    spade::vehicles
    spade::sim
    spade_warnings
    spade_fp_strict
    yaml-cpp::yaml-cpp
    nlohmann_json::nlohmann_json
    GTest::gtest_main
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
# gtest_discover_tests below does set WORKING_DIRECTORY, but the kat testing
# spec's no-CWD rule is the right posture for a data file a test MUST find: a
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
target_compile_definitions(spade_tests PRIVATE
    SPADE_GOLDEN_DIR="${CMAKE_CURRENT_SOURCE_DIR}/golden"
    SPADE_ENGINE_DIR="${SPADE_ENGINE_DIR_NORMALIZED}"
    SPADE_TESTS_DIR="${CMAKE_CURRENT_SOURCE_DIR}"
)

include(GoogleTest)
# WORKING_DIRECTORY is gtest_discover_tests()'s OWN top-level keyword, not a
# PROPERTIES entry (S5 T9 ticket B, first of two root causes -- see
# AppendSpadeLabels.cmake for the second): it used to sit inside PROPERTIES,
# which cmake_parse_arguments -- used internally by gtest_discover_tests() to
# split its own arguments -- has no concept of "inside PROPERTIES" as a
# nested scope for. It recognizes ITS OWN keywords (WORKING_DIRECTORY among
# them, see CMake's own Modules/GoogleTest.cmake) WHEREVER they appear in the
# argument list, so the old call's `PROPERTIES LABELS "spade;T0"
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
gtest_discover_tests(spade_tests
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    PROPERTIES
        TIMEOUT 60
)

# SECOND ROOT CAUSE, and why LABELS is set here instead of above (S5 T9
# ticket B). Proved with a standalone repro script (this task's report has
# the transcript) rather than by reading GoogleTestAddTests.cmake alone:
# `PROPERTIES LABELS "spade;T0"` builds a CORRECTLY escaped list value
# (`LABELS;spade\;T0`, the `\;` meaning "one element containing a literal
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
# splits on EVERY literal `;` it finds, `\;` or not. "spade;T0" -- meant to
# survive as one PROPERTIES value -- flattens into two separate, unpaired
# tokens, and the generated script ends up calling
# `set_tests_properties(... LABELS spade T0)`: LABELS pairs with "spade"
# only, "T0" dangles with no value and is silently dropped by
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
# set_tests_properties() with a QUOTED "spade;T0" literal written directly in
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
)

# spade_fp_strict here too, same reasoning as spade_tests above: bench_core.cpp
# calls spade::math::integrate_orientation directly (header-only math_ops.hpp
# inlines into this TU) and bench_sim.cpp builds scenario state inline --
# both are fp32 expressions this executable evaluates itself, not just calls
# into a pre-built library for.
target_link_libraries(spade_bench PRIVATE
    spade::core
    spade::sim
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
