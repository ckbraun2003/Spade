# tools/flow_movie/keep_cache_check.cmake -- GpuFlowMovie.CaseRenderKeepsItsCache.
#
#   cmake -DFLOW_MOVIE=<flow_movie> -DOUT=<dir> -P keep_cache_check.cmake
#
# A LiveRun film's cache is the only copy of its run's frames: D1's 9.4 h run
# lost its frames to its own render (2026-10-22), before the arm-top samples
# were taken from them. So a render keeps a LiveRun film's cache unless
# --drop-cache is given:
#   1. case-smoke rendered: its cache is still there, every frame in it;
#   2. rendered again with --drop-cache: it draws from that cache, then
#      removes it;
#   3. smoke, a film that isn't LiveRun's, still drops its cache after its
#      render, as before (--keep-cache keeps it).
# Skips where flow_movie exits 77 (no GL).

file(REMOVE_RECURSE "${OUT}")

macro(flow_movie dir)
    execute_process(COMMAND "${FLOW_MOVIE}" ${ARGN} --out "${OUT}/${dir}"
                    RESULT_VARIABLE rc OUTPUT_VARIABLE said ERROR_VARIABLE said_err)
    if(rc EQUAL 77)
        message("[ keep cache ] skipped: flow_movie has no GL here. ${said_err}")
        return()
    endif()
    if(NOT rc EQUAL 0)
        message(FATAL_ERROR "flow_movie ${ARGN} into ${dir} exited ${rc}:\n${said}${said_err}")
    endif()
endmacro()

set(live "${OUT}/live/cache/case-smoke")
flow_movie(live --video case-smoke)
file(GLOB frames "${live}/frame-*.bin")
list(LENGTH frames n)
if(n EQUAL 0)
    message(FATAL_ERROR "case-smoke's render removed its cache (${live}); a LiveRun film's is kept:\n${said}")
endif()
if(NOT said MATCHES "kept the cache")
    message(FATAL_ERROR "case-smoke's render kept its cache without saying so:\n${said}")
endif()

flow_movie(live --video case-smoke --drop-cache)
if(NOT said MATCHES "using the ${n} cached frames")
    message(FATAL_ERROR "the --drop-cache render didn't draw from the kept cache:\n${said}")
endif()
if(EXISTS "${live}")
    message(FATAL_ERROR "--drop-cache left the cache at ${live}")
endif()

flow_movie(fast --video smoke)
if(EXISTS "${OUT}/fast/cache/smoke")
    message(FATAL_ERROR "smoke, not a LiveRun film, kept its cache without --keep-cache")
endif()

message("[ keep cache ] case-smoke kept its ${n} frames, --drop-cache removed them, smoke dropped its own")
