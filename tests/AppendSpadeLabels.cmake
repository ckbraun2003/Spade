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

# T11 fix-wave (F-I1 review W7a): ParityChaos.ShowerPileAmplifiesOneUlpOnTheCpuAlone
# is a host-only CPU<->CPU ulp-perturbation control (not Gpu*-prefixed, so it
# never gets the device TIMEOUT 180 override below) that measures 53.38 s
# nominal in Release against the blanket 60 s cap -- ~1.1x headroom, the same
# TIMEOUT-kills-a-legitimately-slow-test failure mode S5 already hit once.
# Same mechanism, byte-for-byte, as the Fp32Exp override immediately above:
# a per-test budget raised where it is genuinely needed, not the blanket cap
# loosened for everything.
if(spade_tests_TESTS)
    list(FIND spade_tests_TESTS "ParityChaos.ShowerPileAmplifiesOneUlpOnTheCpuAlone" _spade_parity_chaos_index)
    if(NOT _spade_parity_chaos_index EQUAL -1)
        set_tests_properties(ParityChaos.ShowerPileAmplifiesOneUlpOnTheCpuAlone PROPERTIES TIMEOUT 180)
    endif()
    unset(_spade_parity_chaos_index)
endif()

# gpu label (S6 Task 1) -- NAME-PREFIX RULE: any registered test whose ctest
# name (gtest_discover_tests's own "Suite.Test" convention) starts with
# "Gpu" -- i.e. its test suite is Gpu* ("Gpu*.*") -- gets "gpu" appended to
# its label set, on top of the blanket "spade;T0" every test above already
# carries. These are the DEVICE-EXECUTING tests (global constraint: "Every
# device-executing test begins if (!compute::vulkan_available())
# GTEST_SKIP()"); "gpu" is what lets a caller select/exclude them
# (`ctest -L gpu` / `-LE gpu`) without re-deriving the naming convention
# itself every time. scripts/test.ps1 stays unchanged -- its `-L spade`
# selection still runs everything (gpu-labeled tests carry "spade" too); the
# skip-not-fail behavior on a device-less box/CI runner is what does the
# actual gating, not this label.
#
# A SECOND, SEPARATE set_tests_properties() call -- not
# `set_property(... APPEND PROPERTY LABELS "gpu")` layered onto the blanket
# call above -- for the identical reason the Fp32Exp TIMEOUT override just
# above is its own separate call rather than folded into the first: this
# whole file is TEST_INCLUDE_FILES content, re-evaluated by CTest on EVERY
# invocation within a build tree, not a one-time configure step. APPEND
# semantics would re-append "gpu" onto the SAME already-registered test's
# LABELS property on each subsequent run, growing "spade;T0;gpu" into
# "spade;T0;gpu;gpu;gpu;..." across repeated invocations -- a real bug this
# file's own header comment on the FIRST set_tests_properties() call
# documents CTest's LABELS handling is fragile enough to warrant sidestepping
# outright, not just here. Computing the FULL label string ("spade;T0;gpu")
# once below and passing it to set_tests_properties(), which ASSIGNS
# (replaces) rather than appends, produces the identical result on the 1st
# and the 100th run in the same build tree.
if(spade_tests_TESTS)
    set(_spade_gpu_tests "")
    foreach(_spade_test_name ${spade_tests_TESTS})
        if(_spade_test_name MATCHES "^Gpu[^.]*\\.")
            list(APPEND _spade_gpu_tests "${_spade_test_name}")
        endif()
    endforeach()
    if(_spade_gpu_tests)
        set_tests_properties(${_spade_gpu_tests} PROPERTIES LABELS "spade;T0;gpu")

        # TIMEOUT 180 on the device-executing set (S6 Task 4), same posture and
        # same mechanism as the Fp32Exp.FullDomainSweep override above: a
        # per-test budget raised where it is genuinely needed, rather than the
        # blanket 60 s cap loosened for everything.
        #
        # WHY THESE NEED IT. A GPU test's wall time is not under its own
        # control in the way a host test's is. Two of them stand behind that:
        #
        #   * THE DEBUG PRESET. GpuFp32Math's batteries compare millions of
        #     arguments, and the HOST half of every comparison -- the CPU
        #     kernel each one is checked against -- runs unoptimised there.
        #     Measured on this box, unloaded: 30.0 s worst
        #     (Sin32MatchesTheHostBitForBitOverEveryBoxMullerAngle, 16,777,216
        #     arguments) against Release's 1.1 s. Half the 60 s cap, spent
        #     almost entirely in std-library-free host arithmetic.
        #   * THE DEVICE IS SHARED. S6's lanes may run correctness tests
        #     concurrently (Vulkan isolates contexts, so this is allowed and
        #     expected); dispatches then queue behind each other. A 2x margin
        #     is not a margin under contention -- it is a flake waiting for the
        #     one CI-less afternoon when two lanes overlap.
        #
        # 180 s is 6x the measured Debug worst case, and it still FAILS rather
        # than hangs: run_probe()'s own fence wait gives up after 10 s per
        # dispatch (probe_runner.cpp's kFenceTimeoutNs), so a genuinely hung
        # device reports a named error long before this budget expires. This
        # raises the ceiling on legitimate slowness only.
        set_tests_properties(${_spade_gpu_tests} PROPERTIES TIMEOUT 180)
    endif()
    unset(_spade_gpu_tests)
    unset(_spade_test_name)
endif()
