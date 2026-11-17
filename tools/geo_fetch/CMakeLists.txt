# spade_geo_fetch (L85, geo G2, SP-5; docs/design/interface/plans/2026-11-17-geo-map-fetch-design.md): the map fetch, a separate tool
# outside engine/ (the engine never touches the network). It asks USGS 3DEP, USGS NAIP and NOAA NGS and writes the local map cache
# that world/map_cache.hpp reads. The library holds everything the tests drive (tests/CMakeLists.txt compiles the same sources into
# spade_tests with a fake network); the HTTP layer is WinHTTP on Windows and "no HTTP layer on this platform yet" elsewhere.
if(WIN32)
    set(SPADE_GEO_FETCH_HTTP http_winhttp.cpp)
else()
    set(SPADE_GEO_FETCH_HTTP http_none.cpp)
endif()
add_library(spade_geo_fetch_lib STATIC
    geo_services.cpp raster_formats.cpp cache_store.cpp fetch.cpp cli.cpp ${SPADE_GEO_FETCH_HTTP})
target_link_libraries(spade_geo_fetch_lib PUBLIC spade::world PRIVATE spade_warnings spade_version)
target_compile_features(spade_geo_fetch_lib PUBLIC cxx_std_23)
if(WIN32)
    target_link_libraries(spade_geo_fetch_lib PUBLIC winhttp)
endif()
