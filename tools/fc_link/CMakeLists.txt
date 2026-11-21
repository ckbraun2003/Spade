# spade_fc_link (O140): the autopilot links -- the MAVLink 2 codec, the frame conversions and the PX4 link -- as a static library outside engine/
# (L7) on Spade's public headers; the include root is tools/, so a user writes #include "fc_link/mavlink.hpp". Build-tree only, not installed.
add_library(spade_fc_link STATIC mavlink.cpp frames.cpp px4_link.cpp)
target_include_directories(spade_fc_link PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/..)
target_link_libraries(spade_fc_link PUBLIC spade::core PRIVATE spade_warnings)
target_compile_features(spade_fc_link PUBLIC cxx_std_23)
