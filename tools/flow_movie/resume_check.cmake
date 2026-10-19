# tools/flow_movie/resume_check.cmake -- GpuFlowMovie.ResumedRunMatchesUnbroken.
#
#   cmake -DFLOW_MOVIE=<flow_movie> -DOUT=<dir> -P resume_check.cmake
#
# The solver state's proof (TD-9; the lead, 2026-10-19): the smoke film,
# stopped by its frame budget and continued from its state, caches the same
# frames, byte for byte, as one unbroken run. The state's grid is grid_io's
# blob ("SPADEGRD" after the state's frame/step header), and a state from
# before grid_io is refused, naming --resimulate. Skips where flow_movie
# exits 77 (no GL).

set(film --video smoke --seconds 0.2 --simulate-only)
file(REMOVE_RECURSE "${OUT}")

macro(flow_movie dir expected)
    execute_process(COMMAND "${FLOW_MOVIE}" ${film} ${ARGN} --out "${OUT}/${dir}"
                    RESULT_VARIABLE rc OUTPUT_VARIABLE said ERROR_VARIABLE said_err)
    if(rc EQUAL 77)
        message("[ resume ] skipped: flow_movie has no GL here. ${said_err}")
        return()
    endif()
    if(NOT rc EQUAL ${expected})
        message(FATAL_ERROR "flow_movie ${ARGN} into ${dir} exited ${rc}, not ${expected}:\n${said}${said_err}")
    endif()
endmacro()

flow_movie(whole 0)
flow_movie(resumed 3 --frame-budget 2)
set(state "${OUT}/resumed/cache/smoke/state.bin")
if(NOT EXISTS "${state}")
    message(FATAL_ERROR "the budget stop kept no solver state at ${state}")
endif()
file(READ "${state}" state_magic LIMIT 8)
file(READ "${state}" grid_magic OFFSET 20 LIMIT 8)
if(NOT state_magic STREQUAL "SPSTATE2" OR NOT grid_magic STREQUAL "SPADEGRD")
    message(FATAL_ERROR "the state is not flow_movie's header around grid_io's blob: "
                        "\"${state_magic}\" then \"${grid_magic}\" at byte 20")
endif()
flow_movie(resumed 0)

file(GLOB frames RELATIVE "${OUT}/whole/cache/smoke" "${OUT}/whole/cache/smoke/frame-*.bin")
list(LENGTH frames n)
if(NOT n EQUAL 6)
    message(FATAL_ERROR "the unbroken run cached ${n} frames, not 6")
endif()
foreach(f IN LISTS frames)
    execute_process(COMMAND "${CMAKE_COMMAND}" -E compare_files "${OUT}/whole/cache/smoke/${f}"
                            "${OUT}/resumed/cache/smoke/${f}" RESULT_VARIABLE differs)
    if(NOT differs EQUAL 0)
        message(FATAL_ERROR "${f}: the resumed run's frame is not the unbroken run's")
    endif()
endforeach()

flow_movie(old 3 --frame-budget 2)
file(WRITE "${OUT}/old/cache/smoke/state.bin" "SPFRAME1 a state from before grid_io")
flow_movie(old 1)
if(NOT said_err MATCHES "--resimulate")
    message(FATAL_ERROR "a state from before grid_io must be refused naming --resimulate:\n${said_err}")
endif()

message("[ resume ] ${n} frames: stopped after 2 and continued, each byte-identical to the unbroken run's; "
        "the state is grid_io's blob; an older state is refused by name")
