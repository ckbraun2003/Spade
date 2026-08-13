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
)

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
# NOT a packaging change: spade_tests is an executable that never leaves the
# build tree, so there is no export set, no install rule and no consumer to
# consider. Test support never ships.
target_link_libraries(spade_tests PRIVATE
    spade::core
    spade::world
    spade::state
    spade::physics
    spade::vehicles
    spade::sim
    spade_warnings
    spade_fp_strict
    yaml-cpp::yaml-cpp
    GTest::gtest_main
)

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
