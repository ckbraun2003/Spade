# ------------------------------------------------------------------------------
# spade_airflow_runner (plan B, task B1): the airflow chain runner, a spike and
# bench harness in its own target (the chain-runner design's G1). It is not
# linked into Simulation, not installed and not exported; plan B's tests, its
# benchmark and the sandbox's demonstration harness link it. Vulkan only: the
# root CMakeLists.txt adds this directory when SPADE_VULKAN is on.
#
# Includes are spelled from the repository root ("spike/airflow/..."), so no
# spike header can be mistaken for an engine one.
# ------------------------------------------------------------------------------
add_library(spade_airflow_runner STATIC
    chain_runner.hpp
    chain_runner.cpp
)
target_include_directories(spade_airflow_runner PUBLIC "${PROJECT_SOURCE_DIR}")
target_compile_features(spade_airflow_runner PUBLIC cxx_std_23)
# spade::physics for kGridByteCap (physics/airflow/mac_grid.hpp), the budget's
# ceiling. volk and the Vulkan headers PUBLIC: chain_runner.hpp names Vulkan
# handles.
target_link_libraries(spade_airflow_runner
    PUBLIC spade::compute spade::physics volk Vulkan::Headers
    PRIVATE spade_warnings spade_fp_strict
)

# SpadeSlang.cmake's paths, profile and workgroup sizes are variables of the
# directory that includes it (engine/ does), so this directory includes it too;
# including it defines functions and sets variables, and creates no target.
include("${PROJECT_SOURCE_DIR}/cmake/SpadeSlang.cmake")

# The runner's test kernels (engine/shaders/kernels/airflow_*.slang), compiled
# at every workgroup size in SPADE_SLANG_WORKGROUP_SIZES. Registered on THIS
# target, after engine/'s kernel manifest is written, so spade_compute's
# generated headers and test_slang_layouts.cpp's kernel count are unchanged;
# tests/test_gpu_airflow_runner.cpp scans these three modules itself.
spade_slang_kernel_variants(spade_airflow_runner airflow_copy)
spade_slang_kernel_variants(spade_airflow_runner airflow_add_one)
spade_slang_kernel_variants(spade_airflow_runner airflow_tile_sum)
