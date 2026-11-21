# fc_label_check.cmake -- the `fc` label rule (TD-17), asked of ctest itself.
#
#   cmake -DCTEST=<ctest> -DBUILD=<build tree> -P fc_label_check.cmake
#
# The FcLive* suites (tests that need a real autopilot) carry the label `fc` and NOT `spade`, so `-L spade` -- scripts/test.ps1 and the Docker leg -- leaves them
# out. This lists, with -N (nothing runs), the tests under each label and checks:
#   - the `fc` set is not empty and holds the sentinel FcLiveHarness test (a rule that selects nothing would pass for ever: TD-5);
#   - no FcLive test is in the `spade` set, and none is in `gpu`;
#   - the unit suite of the same work (FcRunFolder) IS in the `spade` set, so the gate still runs it.

if(NOT CTEST OR NOT BUILD)
    message(FATAL_ERROR "fc_label_check: pass -DCTEST=<ctest> -DBUILD=<build tree>")
endif()

function(spade_tests_under label out_var)
    execute_process(COMMAND "${CTEST}" --test-dir "${BUILD}" -N -L "${label}"
                    OUTPUT_VARIABLE listing ERROR_VARIABLE err RESULT_VARIABLE rc)
    if(NOT rc EQUAL 0)
        message(FATAL_ERROR "fc_label_check: ctest -N -L ${label} exited ${rc}: ${err}")
    endif()
    string(REGEX MATCHALL "Test +#[0-9]+: [^\r\n]+" lines "${listing}")
    set(names "")
    foreach(line IN LISTS lines)
        string(REGEX REPLACE "^Test +#[0-9]+: " "" name "${line}")
        string(STRIP "${name}" name)
        list(APPEND names "${name}")
    endforeach()
    set(${out_var} "${names}" PARENT_SCOPE)
endfunction()

spade_tests_under(fc fc_tests)
spade_tests_under(spade spade_tests_set)
spade_tests_under(gpu gpu_tests)

set(sentinel "FcLiveHarness.TheRunnerHandsTheTestsItsRunAndFolder")
if(NOT sentinel IN_LIST fc_tests)
    message(FATAL_ERROR "fc_label_check: ${sentinel} is not under the label fc (the rule selects nothing, or the test is gone)")
endif()

foreach(name IN LISTS spade_tests_set)
    if(name MATCHES "^FcLive")
        message(FATAL_ERROR "fc_label_check: ${name} is under the label spade, so the gates would run it without an autopilot")
    endif()
endforeach()
foreach(name IN LISTS gpu_tests)
    if(name MATCHES "^FcLive")
        message(FATAL_ERROR "fc_label_check: ${name} is under the label gpu")
    endif()
endforeach()

set(unit "FcRunFolder.Sha256KnownAnswers")
if(NOT unit IN_LIST spade_tests_set)
    message(FATAL_ERROR "fc_label_check: ${unit} is not under the label spade, so the gate would not run the run-folder checker")
endif()

list(LENGTH fc_tests n_fc)
list(LENGTH spade_tests_set n_spade)
message(STATUS "fc_label_check: ${n_fc} tests under fc, ${n_spade} under spade, none of the fc ones in spade or gpu")
