# spade_fc_link (O140): the autopilot links -- the MAVLink 2 codec, the frame conversions and the PX4 link -- as a static library outside engine/
# (L7) on Spade's public headers; the include root is tools/, so a user writes #include "fc_link/mavlink.hpp". Build-tree only, not installed.
add_library(spade_fc_link STATIC mavlink.cpp frames.cpp px4_link.cpp tcp.cpp autopilot_map.cpp autopilot_link.cpp run_support.cpp)
target_include_directories(spade_fc_link PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/..)
# spade::vehicles: autopilot_map.hpp takes a vehicles::PropulsionChain and calls rotor_omega_for_duty().
target_link_libraries(spade_fc_link PUBLIC spade::core spade::vehicles spade::world PRIVATE spade_warnings)
target_compile_features(spade_fc_link PUBLIC cxx_std_23)
if(WIN32)
    target_link_libraries(spade_fc_link PUBLIC ws2_32)  # Winsock, for tcp.cpp
endif()
