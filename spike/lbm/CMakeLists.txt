# ------------------------------------------------------------------------------
# spade_lbm_chain (L1, the LBM tier's GPU twin): the LBM kernels' buffer list and,
# from T2, its chain, in its own target beside spade_airflow_runner (which runs
# its dispatches). Not linked into Simulation, not installed, not exported.
# Vulkan only: the root CMakeLists.txt adds this directory when SPADE_VULKAN is on.
#
# The kernels (engine/shaders/kernels/lbm_*.slang) are registered HERE, as the
# airflow ones are on spade_airflow_runner, so spade_compute's generated headers
# and test_slang_layouts.cpp's kernel count are unchanged; tests/test_gpu_lbm.cpp
# scans these modules itself. Every LBM kernel is NAN_PRESERVE (rule P6, as the
# airflow kernels are; Core confirms at the scan review).
# ------------------------------------------------------------------------------
add_library(spade_lbm_chain STATIC
    lbm_chain.hpp
    lbm_chain.cpp
)
target_include_directories(spade_lbm_chain PUBLIC "${PROJECT_SOURCE_DIR}")
target_compile_features(spade_lbm_chain PUBLIC cxx_std_23)
target_link_libraries(spade_lbm_chain
    PUBLIC spade_airflow_runner spade::compute
    PRIVATE spade_warnings spade_fp_strict
)

include("${PROJECT_SOURCE_DIR}/cmake/SpadeSlang.cmake")

# T1: the lattice tables written out for the bit comparison with
# engine/physics/lbm/lattice.hpp (tests/test_gpu_lbm.cpp).
spade_slang_kernel_variants(spade_lbm_chain lbm_lattice_dump NAN_PRESERVE)
