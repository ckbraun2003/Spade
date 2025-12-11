# Rasterizer Demo
add_executable(SandboxRaster main_raster.cpp)
target_link_libraries(SandboxRaster PRIVATE Spade Psapi)

# Ray Tracer Demo
add_executable(SandboxRT main_rt.cpp)
target_link_libraries(SandboxRT PRIVATE Spade Psapi)
