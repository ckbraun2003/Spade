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
    lambda2_fill.hpp
    lambda2_fill.cpp
    recorder.hpp
    recorder.cpp
    views.hpp
    views.cpp)
target_compile_features(flow_movie PRIVATE cxx_std_23)
# spade::analysis: the validation reel reads the V1 records through
# Interface's one reader (TD-9).
target_link_libraries(flow_movie PRIVATE
    spade::core spade::physics spade::render spade::render_gl spade::analysis glfw glad)
# GLFW_INCLUDE_NONE: glad declares GL, and glfw3.h must not pull in <GL/gl.h>.
# SPADE_ENGINE_DIR and SPADE_TESTS_DIR are the tests' names, which the gcc
# check defines too: git reads the build's commit from the engine's folder,
# and the V1 records sit under tests/.
target_compile_definitions(flow_movie PRIVATE GLFW_INCLUDE_NONE
    SPADE_ENGINE_DIR="${CMAKE_SOURCE_DIR}/engine"
    SPADE_TESTS_DIR="${CMAKE_SOURCE_DIR}/tests")
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
