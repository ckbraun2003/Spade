# CTest TEST_INCLUDE_FILES script (S5 T9 ticket B) -- sets the "spade" and
# "T0" ctest labels on every test spade_tests's gtest_discover_tests() call
# (CMakeLists.txt, this directory) registered. See that call's own comment
# for WHY this lives in a separate file instead of gtest_discover_tests()'s
# own PROPERTIES keyword: PROPERTIES's LABELS value needs two items in one
# property, and GoogleTestAddTests.cmake's own internal forwarding (upstream
# CMake, not this project) silently flattens and drops the second one.
#
# ORDERING THIS RELIES ON: CTest evaluates the directory's TEST_INCLUDE_FILES
# list in append order. gtest_discover_tests(spade_tests ...) appends ITS OWN
# generated include first (inside that function, before CMakeLists.txt's own
# `set_property(... APPEND PROPERTY TEST_INCLUDE_FILES ...)` call that adds
# THIS file runs), so by the time this file is included, spade_tests_TESTS
# already holds the discovered list -- gtest_discover_tests()'s default
# TEST_LIST name, "<target>_TESTS" (this call site never overrides TEST_LIST).
if(spade_tests_TESTS)
    set_tests_properties(${spade_tests_TESTS} PROPERTIES LABELS "spade;T0")
endif()

# S5 final-review fix wave (I2): CMakeLists.txt's gtest_discover_tests() call
# above sets TIMEOUT 60 for all 436 discovered tests -- correct for the suite
# as a whole, but it also caps Fp32Exp.FullDomainSweepEveryFloatArgument, the
# env-gated (SPADE_FULL_EXP_SWEEP=1) full-domain sweep that test_fp32_math.cpp
# documents at 90-127 s in Release and several times that in Debug. That test
# is a GTEST_SKIP() no-op (~0.02 s) on every normal run, so it never costs the
# 60 s budget by default -- but its one sanctioned way to actually RUN
# (test.ps1 -> ctest, per its own header comment; the DISABLED_ prefix and a
# direct gtest-binary invocation are both deliberately closed off) inherits
# the blanket 60 s cap and gets killed mid-sweep. A per-test override, applied
# here rather than in the PROPERTIES block above, raises its budget without
# loosening the 60 s cap that keeps every other test honest.
if(spade_tests_TESTS)
    list(FIND spade_tests_TESTS "Fp32Exp.FullDomainSweepEveryFloatArgument" _spade_full_exp_sweep_index)
    if(NOT _spade_full_exp_sweep_index EQUAL -1)
        set_tests_properties(Fp32Exp.FullDomainSweepEveryFloatArgument PROPERTIES TIMEOUT 900)
    endif()
    unset(_spade_full_exp_sweep_index)
endif()
