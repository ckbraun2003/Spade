# spade_mesh_pack (CORE-6 D; the STEP converter plan, Task S1): a mesh file
# spade_mesh_import reads, to .spmesh sidecars. Build-tree only (L23 5): no
# install rule. run() lives in mesh_pack.cpp, which spade_tests compiles too
# (tests/test_mesh_pack.cpp), so the tool is tested without a process.
add_executable(spade_mesh_pack main.cpp mesh_pack.hpp mesh_pack.cpp)
target_link_libraries(spade_mesh_pack PRIVATE spade::mesh_import spade_warnings)
target_compile_features(spade_mesh_pack PRIVATE cxx_std_23)
