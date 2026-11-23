# spade_camera_frames (O150, video 4): the engine camera's CPU reference frames of one pose of a world file -- the pinhole and the KB4 fisheye -- written as PNGs.
# The library holds what the tests drive in-process (tests/CMakeLists.txt compiles camera_frames.cpp into spade_tests); the exe is main.cpp over it. A tool run by hand,
# its output a video's footage: not part of the editor package. CPU only: no GL, no GPU.
add_library(spade_camera_frames_lib STATIC camera_frames.cpp)
target_link_libraries(spade_camera_frames_lib PUBLIC spade::core spade::world spade::render PRIVATE spade_warnings)
target_compile_features(spade_camera_frames_lib PUBLIC cxx_std_23)

add_executable(spade_camera_frames main.cpp)
target_link_libraries(spade_camera_frames PRIVATE spade_camera_frames_lib spade_warnings)
