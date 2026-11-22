# spade_fc_host (O140): the lockstep host, a static library outside engine/ (L7) on Spade's public headers; the include root is tools/, so a user
# writes #include "fc_host/lockstep.hpp". Build-tree only, not installed. fc_link includes fc_host; fc_host never includes fc_link.
add_library(spade_fc_host STATIC lockstep.cpp)
target_include_directories(spade_fc_host PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/..)
target_link_libraries(spade_fc_host PUBLIC spade::core spade::sim spade::vehicles PRIVATE spade_warnings)
target_compile_features(spade_fc_host PUBLIC cxx_std_23)
