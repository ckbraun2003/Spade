# spade_part_stand_in (L88l): stand-in `shapes` for a library part whose meshes are in the local part store (stand_in.hpp has the rule). The
# library holds what the tests drive in-process (tests/CMakeLists.txt compiles stand_in.cpp into spade_tests); the exe is main.cpp over it.
# A tool run by hand, its output committed: it is not part of the editor package.
add_library(spade_part_stand_in_lib STATIC stand_in.cpp)
target_link_libraries(spade_part_stand_in_lib PUBLIC spade::scene spade::world spade::render PRIVATE nlohmann_json::nlohmann_json spade_warnings)
target_compile_features(spade_part_stand_in_lib PUBLIC cxx_std_23)

add_executable(spade_part_stand_in main.cpp)
target_link_libraries(spade_part_stand_in PRIVATE spade_part_stand_in_lib spade_warnings)
