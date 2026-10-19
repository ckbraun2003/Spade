# tools/flow_movie/resume_check.cmake -- GpuFlowMovie.*ResumedRunMatchesUnbroken.
#
#   cmake -DFLOW_MOVIE=<flow_movie> -DOUT=<dir> -DVIDEO=smoke|case-smoke -P resume_check.cmake
#
# The resume proof (TD-9; the lead, 2026-10-19): a film stopped by its frame
# budget and continued from its save caches the same frames, byte for byte,
# as one unbroken run.
#   smoke: the solver's state is flow_movie's frame/step header around
#     grid_io's blob ("SPADEGRD" at byte 20), and a state from before grid_io
#     is refused, naming --resimulate.
#   case-smoke: the powered quad through LiveRun's save/restore (live.save),
#     over 12 frames stopped after 8: the showcase program arms at 0.2 s
#     (frame 6), so the save carries spinning rotors. run.txt, each frame's
#     step and run_label(), matches too.
# smoke runs 6 frames stopped after 2.
# Skips where flow_movie exits 77 (no GL).

if(VIDEO STREQUAL "smoke")
    set(seconds 0.2)
    set(expected_frames 6)
    set(budget 2)
else()
    set(seconds 0.4)
    set(expected_frames 12)
    set(budget 8)
endif()
set(film --video ${VIDEO} --seconds ${seconds} --simulate-only)
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

set(whole "${OUT}/whole/cache/${VIDEO}")
set(resumed "${OUT}/resumed/cache/${VIDEO}")
flow_movie(whole 0)
flow_movie(resumed 3 --frame-budget ${budget})
if(VIDEO STREQUAL "smoke")
    set(state "${resumed}/state.bin")
    if(NOT EXISTS "${state}")
        message(FATAL_ERROR "the budget stop kept no solver state at ${state}")
    endif()
    file(READ "${state}" state_magic LIMIT 8)
    file(READ "${state}" grid_magic OFFSET 20 LIMIT 8)
    if(NOT state_magic STREQUAL "SPSTATE2" OR NOT grid_magic STREQUAL "SPADEGRD")
        message(FATAL_ERROR "the state is not flow_movie's header around grid_io's blob: "
                            "\"${state_magic}\" then \"${grid_magic}\" at byte 20")
    endif()
else()
    if(NOT EXISTS "${resumed}/live.save")
        message(FATAL_ERROR "the budget stop kept no LiveRun save at ${resumed}/live.save")
    endif()
endif()
flow_movie(resumed 0)

file(GLOB frames RELATIVE "${whole}" "${whole}/frame-*.bin")
list(LENGTH frames n)
if(NOT n EQUAL expected_frames)
    message(FATAL_ERROR "the unbroken run cached ${n} frames, not ${expected_frames}")
endif()
set(compared ${frames})
if(NOT VIDEO STREQUAL "smoke")
    list(APPEND compared run.txt)
    if(EXISTS "${resumed}/live.save")
        message(FATAL_ERROR "a finished film kept its LiveRun save")
    endif()
endif()
foreach(f IN LISTS compared)
    execute_process(COMMAND "${CMAKE_COMMAND}" -E compare_files "${whole}/${f}" "${resumed}/${f}"
                    RESULT_VARIABLE differs)
    if(NOT differs EQUAL 0)
        message(FATAL_ERROR "${f}: the resumed run's is not the unbroken run's")
    endif()
endforeach()

if(VIDEO STREQUAL "smoke")
    flow_movie(old 3 --frame-budget 2)
    file(WRITE "${OUT}/old/cache/smoke/state.bin" "SPFRAME1 a state from before grid_io")
    flow_movie(old 1)
    if(NOT said_err MATCHES "--resimulate")
        message(FATAL_ERROR "a state from before grid_io must be refused naming --resimulate:\n${said_err}")
    endif()
endif()

message("[ resume ] ${VIDEO}: ${n} frames, stopped after ${budget} and continued, byte-identical to the unbroken run")
