# tools/flow_movie: the offline airflow recorder (Cameron's videos, relayed
# 2026-10-13). It runs the CPU solver, publishes at fixed simulated times,
# draws each frame on GL at 1920x1080 at full flow quality and pipes it to
# ffmpeg. The root CMakeLists adds it only where GL and a window backend are
# built.
add_executable(flow_movie
    main.cpp
    cases.hpp
    quad_case.cpp
    validation_cases.cpp
    cache.hpp
    cache.cpp
    recorder.hpp
    recorder.cpp
    views.hpp
    views.cpp
    live_case.hpp
    live_case.cpp
    arm_pressure.hpp
    arm_pressure.cpp
    subnormal_scan.hpp
    subnormal_scan.cpp
    frame_check.hpp)
target_compile_features(flow_movie PRIVATE cxx_std_23)
# spade::analysis: the validation reel reads the V1 records through
# Interface's one reader (TD-9).
target_link_libraries(flow_movie PRIVATE
    spade::core spade::physics spade::render spade::render_gl spade::analysis spade::scene spade::render_parts
    spade::vehicles glfw glad)
# GLFW_INCLUDE_NONE: glad declares GL, and glfw3.h must not pull in <GL/gl.h>.
# SPADE_TESTS_DIR, the tests' name, which the gcc check defines too: the V1
# records sit under tests/.
target_compile_definitions(flow_movie PRIVATE GLFW_INCLUDE_NONE
    SPADE_TESTS_DIR="${CMAKE_SOURCE_DIR}/tests"
    SPADE_ASSETS_DIR="${CMAKE_SOURCE_DIR}/assets")

if(WIN32)
    target_compile_definitions(flow_movie PRIVATE NOMINMAX WIN32_LEAN_AND_MEAN)
endif()
# flow_movie.map beside the exe: --subnormal-locate names fault addresses
# from it (the release build keeps no PDB).
if(MSVC)
    target_link_options(flow_movie PRIVATE /MAP)
endif()

# --gpu (plan B, B7's G2): a LiveRun film's fluid on GpuFluid, from the
# airflow chain runner, which the root CMakeLists adds before this directory
# when SPADE_VULKAN is on.
if(TARGET spade_airflow_runner)
    target_sources(flow_movie PRIVATE gpu_live.hpp gpu_live.cpp)
    target_link_libraries(flow_movie PRIVATE spade_airflow_runner)
    target_compile_definitions(flow_movie PRIVATE SPADE_FLOW_MOVIE_GPU=1)
endif()

# The smoke test: two frames of a small case through GL, ffmpeg and the
# captions, at 640x360. A box with no display or no ffmpeg skips (77).
add_test(NAME GpuFlowMovie.SmokeRendersTwoFrames
    COMMAND flow_movie --video smoke --out "${CMAKE_CURRENT_BINARY_DIR}/smoke")
set_tests_properties(GpuFlowMovie.SmokeRendersTwoFrames PROPERTIES
    LABELS "spade;gpu"
    SKIP_RETURN_CODE 77
    PASS_REGULAR_EXPRESSION "smoke: wrote"
    TIMEOUT 120)

# The solver state's proof (TD-9): the smoke film stopped by its frame budget
# and continued caches the same frames, byte for byte, as one unbroken run;
# its state is grid_io's blob. Skips as the smoke above does.
add_test(NAME GpuFlowMovie.ResumedRunMatchesUnbroken
    COMMAND "${CMAKE_COMMAND}" "-DFLOW_MOVIE=$<TARGET_FILE:flow_movie>" "-DOUT=${CMAKE_CURRENT_BINARY_DIR}/resume"
            -DVIDEO=smoke -P "${CMAKE_CURRENT_SOURCE_DIR}/resume_check.cmake")
set_tests_properties(GpuFlowMovie.ResumedRunMatchesUnbroken PROPERTIES
    LABELS "spade;gpu"
    SKIP_REGULAR_EXPRESSION "\\[ resume \\] skipped"
    PASS_REGULAR_EXPRESSION "\\[ resume \\] smoke: 6 frames"
    TIMEOUT 120)

# The same through LiveRun's save/restore: the powered case smoke, stopped
# after it arms (its rotors spinning) and continued, caches the same frames
# and run labels as one unbroken run.
add_test(NAME GpuFlowMovie.CaseResumedRunMatchesUnbroken
    COMMAND "${CMAKE_COMMAND}" "-DFLOW_MOVIE=$<TARGET_FILE:flow_movie>" "-DOUT=${CMAKE_CURRENT_BINARY_DIR}/case-resume"
            -DVIDEO=case-smoke -P "${CMAKE_CURRENT_SOURCE_DIR}/resume_check.cmake")
set_tests_properties(GpuFlowMovie.CaseResumedRunMatchesUnbroken PROPERTIES
    LABELS "spade;gpu"
    SKIP_REGULAR_EXPRESSION "\\[ resume \\] skipped"
    PASS_REGULAR_EXPRESSION "\\[ resume \\] case-smoke: 12 frames"
    TIMEOUT 120)

# A LiveRun film's cache outlives its render unless --drop-cache is given (D1
# lost its run's frames to its render, 2026-10-22); a film that isn't
# LiveRun's still drops its own.
add_test(NAME GpuFlowMovie.CaseRenderKeepsItsCache
    COMMAND "${CMAKE_COMMAND}" "-DFLOW_MOVIE=$<TARGET_FILE:flow_movie>" "-DOUT=${CMAKE_CURRENT_BINARY_DIR}/keep-cache"
            -P "${CMAKE_CURRENT_SOURCE_DIR}/keep_cache_check.cmake")
set_tests_properties(GpuFlowMovie.CaseRenderKeepsItsCache PROPERTIES
    LABELS "spade;gpu"
    SKIP_REGULAR_EXPRESSION "\\[ keep cache \\] skipped"
    PASS_REGULAR_EXPRESSION "\\[ keep cache \\] case-smoke kept its"
    TIMEOUT 120)

# The --case smoke: two frames of the powered reference quad through LiveRun
# and the recorder (D1's path), at 640x360. Skips as the smoke above does.
# --drop-cache: each run simulates afresh, never drawing a kept cache from an
# earlier build.
add_test(NAME GpuFlowMovie.CaseSmokeRendersTwoFrames
    COMMAND flow_movie --video case-smoke --drop-cache --out "${CMAKE_CURRENT_BINARY_DIR}/case-smoke")
set_tests_properties(GpuFlowMovie.CaseSmokeRendersTwoFrames PROPERTIES
    LABELS "spade;gpu"
    SKIP_RETURN_CODE 77
    PASS_REGULAR_EXPRESSION "case-smoke: wrote"
    TIMEOUT 120)

# --gpu (B7's G2): two frames of D1's no-parts film with its fluid on
# GpuFluid, at D1's 4 mm on a 112x80x112 box that holds the rotors' discs,
# each frame checked for a non-finite sample before it is cached; and a
# program that leaves the hover (the case smoke's showcase) refused, since the
# GPU film's discs never change. Skip as the smoke above does.
if(TARGET spade_airflow_runner)
    add_test(NAME GpuFlowMovie.GpuD1RendersTwoFrames
        COMMAND flow_movie --video d1-hover --gpu --grid 112x80x112 --seconds 0.0667 --drop-cache
                --out "${CMAKE_CURRENT_BINARY_DIR}/d1-gpu")
    set_tests_properties(GpuFlowMovie.GpuD1RendersTwoFrames PROPERTIES
        LABELS "spade;gpu"
        SKIP_RETURN_CODE 77
        PASS_REGULAR_EXPRESSION "steps on the GPU in .*d1-hover-gpu: wrote"
        TIMEOUT 300)
    add_test(NAME GpuFlowMovie.GpuRefusesAMovingProgram
        COMMAND flow_movie --video case-smoke --gpu --simulate-only --out "${CMAKE_CURRENT_BINARY_DIR}/smoke-gpu")
    set_tests_properties(GpuFlowMovie.GpuRefusesAMovingProgram PROPERTIES
        LABELS "spade;gpu"
        SKIP_RETURN_CODE 77
        PASS_REGULAR_EXPRESSION "--gpu holds the rotors at the program's first controls"
        TIMEOUT 120)
endif()
