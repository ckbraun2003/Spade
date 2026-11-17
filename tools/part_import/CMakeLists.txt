# spade_part_import (L88, L88b; docs/design/interface/plans/2026-11-17-part-import-design.md): the part import tool, a CAD file to the local
# part store. The library holds everything the tests drive in-process (tests/CMakeLists.txt compiles the same sources into spade_tests);
# the exe is main.cpp over it.
add_library(spade_part_import_lib STATIC report.cpp transform.cpp store.cpp options.cpp run.cpp process.cpp)
target_link_libraries(spade_part_import_lib PUBLIC spade::mesh_import spade::world PRIVATE nlohmann_json::nlohmann_json spade_warnings)
target_compile_features(spade_part_import_lib PUBLIC cxx_std_23)
