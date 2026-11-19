# spade_library (L88l, S4; docs/design/core/plans/2026-11-17-spade-library-json-contract.md): the library's C ABI, JSON in and JSON out,
# for a host that loads it by path (KAT through DynLib; the SDK through the CLI). The shared library is OUTPUT_NAME spade_library
# (spade_library.dll, libspade_library.so), installed as the component `library_c` and nowhere else: it is not in the editor package.
# The tests compile the same sources into spade_tests (tests/CMakeLists.txt) and drive the ABI's functions directly, as part_import's do.
set(SPADE_LIBRARY_C_SOURCES library_json.cpp c_api.cpp)

add_library(spade_library_c SHARED ${SPADE_LIBRARY_C_SOURCES})
set_target_properties(spade_library_c PROPERTIES
    OUTPUT_NAME spade_library
    CXX_VISIBILITY_PRESET hidden
    VISIBILITY_INLINES_HIDDEN ON)
target_compile_definitions(spade_library_c PRIVATE SPADE_LIBRARY_BUILD_DLL)
target_link_libraries(spade_library_c PRIVATE spade::scene nlohmann_json::nlohmann_json spade_warnings)
target_compile_features(spade_library_c PUBLIC cxx_std_23)

# The C ABI's component: `cmake --install <build> --component library_c` puts the library and its header under <prefix>. EXCLUDE_FROM_ALL: a plain
# `cmake --install` (the SDK's) does not carry it, so an install after a build of only the SDK's targets does not look for it.
install(TARGETS spade_library_c EXCLUDE_FROM_ALL
    RUNTIME DESTINATION bin COMPONENT library_c
    LIBRARY DESTINATION lib COMPONENT library_c)
install(FILES spade_library.h DESTINATION include COMPONENT library_c EXCLUDE_FROM_ALL)
