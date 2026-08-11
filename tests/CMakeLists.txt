add_executable(spade_tests
    test_core.cpp
    test_math.cpp
    test_fp32_math.cpp
    test_ecs.cpp
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
target_link_libraries(spade_tests PRIVATE
    spade::core
    spade::world
    spade::state
    spade::physics
    spade::vehicles
    spade::sim
    spade_warnings
    spade_fp_strict
    GTest::gtest_main
)

# The golden determinism corpus (tests/golden/*.digest), located by absolute
# path baked in at configure time rather than by a CWD-relative path.
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
gtest_discover_tests(spade_tests
    PROPERTIES
        LABELS "spade;T0"
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
        TIMEOUT 60
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
