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
    live_case.cpp)
target_compile_features(flow_movie PRIVATE cxx_std_23)
# spade::analysis: the validation reel reads the V1 records through
# Interface's one reader (TD-9).
target_link_libraries(flow_movie PRIVATE
    spade::core spade::physics spade::render spade::render_gl spade::analysis spade::scene spade::render_parts
    glfw glad)
# GLFW_INCLUDE_NONE: glad declares GL, and glfw3.h must not pull in <GL/gl.h>.
# SPADE_TESTS_DIR, the tests' name, which the gcc check defines too: the V1
# records sit under tests/.
target_compile_definitions(flow_movie PRIVATE GLFW_INCLUDE_NONE
    SPADE_TESTS_DIR="${CMAKE_SOURCE_DIR}/tests"
    SPADE_ASSETS_DIR="${CMAKE_SOURCE_DIR}/assets")

# The build stamp every card names: the commit the binary is built from, and
# whether the tree had uncommitted changes, taken at build time (never asked
# of git when the binary runs, which could name a later commit).
set(FLOW_MOVIE_STAMP "${CMAKE_CURRENT_BINARY_DIR}/generated/flow_movie_identity.hpp")
add_custom_target(flow_movie_identity
    COMMAND "${CMAKE_COMMAND}" "-DSOURCE_DIR=${CMAKE_SOURCE_DIR}" "-DOUT=${FLOW_MOVIE_STAMP}"
            -P "${CMAKE_CURRENT_SOURCE_DIR}/stamp_identity.cmake"
    BYPRODUCTS "${FLOW_MOVIE_STAMP}"
    COMMENT "flow_movie: stamping the build's commit"
    VERBATIM)
add_dependencies(flow_movie flow_movie_identity)
target_include_directories(flow_movie PRIVATE "${CMAKE_CURRENT_BINARY_DIR}/generated")
if(WIN32)
    target_compile_definitions(flow_movie PRIVATE NOMINMAX WIN32_LEAN_AND_MEAN)
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
            -P "${CMAKE_CURRENT_SOURCE_DIR}/resume_check.cmake")
set_tests_properties(GpuFlowMovie.ResumedRunMatchesUnbroken PROPERTIES
    LABELS "spade;gpu"
    SKIP_REGULAR_EXPRESSION "\\[ resume \\] skipped"
    PASS_REGULAR_EXPRESSION "\\[ resume \\] 6 frames"
    TIMEOUT 120)

# The --case smoke: two frames of Interface's tiny LiveRun case through the
# recorder (D1's path), at 640x360. Skips as the smoke above does.
add_test(NAME GpuFlowMovie.CaseSmokeRendersTwoFrames
    COMMAND flow_movie --video case-smoke --out "${CMAKE_CURRENT_BINARY_DIR}/case-smoke")
set_tests_properties(GpuFlowMovie.CaseSmokeRendersTwoFrames PROPERTIES
    LABELS "spade;gpu"
    SKIP_RETURN_CODE 77
    PASS_REGULAR_EXPRESSION "case-smoke: wrote"
    TIMEOUT 120)
