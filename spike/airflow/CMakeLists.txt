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
    projection_chain.hpp
    projection_chain.cpp
    solids_chain.hpp
    solids_chain.cpp
    publish_chain.hpp
    publish_chain.cpp
)
target_include_directories(spade_airflow_runner PUBLIC "${PROJECT_SOURCE_DIR}")
target_compile_features(spade_airflow_runner PUBLIC cxx_std_23)
# spade::compute for the context, Core's limit and budget checks
# (compute/device_limits.hpp) and the generated bindings.gen.hpp, whose section
# F constants kAirflowSetBuffers is pinned to. volk and the Vulkan headers
# PUBLIC: chain_runner.hpp names Vulkan handles. spade::physics for B2's
# projection chain, which lays out plan A's multigrid levels and writes the
# CPU's own projection floats (projection_chain.hpp).
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
# B5.0b's test-only probe: airflow_tile_sum's last pass with loads issued
# ahead, and a two-level order, measured by an opt-in test; no chain uses it.
spade_slang_kernel_variants(spade_airflow_runner airflow_tile_sum_probe)

# B2's projection kernels, the CPU's multigrid V-cycle and its divergence and
# gradient (tests/test_gpu_airflow_projection.cpp; AirflowRunnerSpirv scans
# them with the three above). They import mg_levels.slang and
# airflow_params_layout.slang, which -depfile tracks.
spade_slang_kernel_variants(spade_airflow_runner airflow_divergence)
spade_slang_kernel_variants(spade_airflow_runner airflow_gradient_subtract)
spade_slang_kernel_variants(spade_airflow_runner airflow_mg_smooth)
spade_slang_kernel_variants(spade_airflow_runner airflow_mg_residual)
spade_slang_kernel_variants(spade_airflow_runner airflow_mg_restrict)
spade_slang_kernel_variants(spade_airflow_runner airflow_mg_prolong)
spade_slang_kernel_variants(spade_airflow_runner airflow_remove_mean)

# B3's advection kernel, advect.cpp term for term
# (tests/test_gpu_airflow_advection.cpp).
spade_slang_kernel_variants(spade_airflow_runner airflow_advection)

# B3's velocity conditions and implicit viscosity, boundaries.cpp and
# viscous.cpp term for term (tests/test_gpu_airflow_viscous.cpp; the lead's
# grant, 2026-10-22). airflow_viscous imports fp32_math.slang.
spade_slang_kernel_variants(spade_airflow_runner airflow_velocity_bc)
spade_slang_kernel_variants(spade_airflow_runner airflow_viscous)

# B6's publish kernels, publish_render_volume's channels and their binary16
# pack (tests/test_gpu_airflow_publish.cpp).
spade_slang_kernel_variants(spade_airflow_runner airflow_publish_fields)
spade_slang_kernel_variants(spade_airflow_runner airflow_pack_f16)

# B4's rotor discs, spread_actuator() as a gather
# (tests/test_gpu_airflow_actuator.cpp).
spade_slang_kernel_variants(spade_airflow_runner airflow_actuator_gather)

# B5's solids projection (Physics; tests/test_gpu_airflow_solids.cpp): the
# CG's dots (pass 0; airflow_tile_sum ends them), its max-norm and its
# quotients (div32_ftz), the aperture divergence and gradient, and the
# operator over a CG vector (mg_levels.slang's stencil_from()). They import
# airflow_pcg.slang, which -depfile tracks.
spade_slang_kernel_variants(spade_airflow_runner airflow_tile_dot)
spade_slang_kernel_variants(spade_airflow_runner airflow_pcg_measure)
spade_slang_kernel_variants(spade_airflow_runner airflow_pcg_scalars)
spade_slang_kernel_variants(spade_airflow_runner airflow_solid_divergence)
spade_slang_kernel_variants(spade_airflow_runner airflow_solid_gradient)
spade_slang_kernel_variants(spade_airflow_runner airflow_pcg_apply)
# B5.2b: the symmetric V(2,2) preconditioner's full-weighting restriction
# (mg_levels.slang's levels; B2's smoother, residual and prolongation do the rest).
spade_slang_kernel_variants(spade_airflow_runner airflow_mg_restrict_full)

# B5.3 and B5.5 (Physics; tests/test_gpu_airflow_solids.cpp): the closed
# regions' means and the CG's gauge, what a closed face holds (both interior
# modes; div32_ftz), and the wall load.
spade_slang_kernel_variants(spade_airflow_runner airflow_region_mean)
spade_slang_kernel_variants(spade_airflow_runner airflow_solid_velocities)
spade_slang_kernel_variants(spade_airflow_runner airflow_wall_load)

# B5.4 (Physics; tests/test_gpu_airflow_solids.cpp): the solids CG's vector
# work (no division); its decisions are airflow_pcg_scalars'.
spade_slang_kernel_variants(spade_airflow_runner airflow_pcg_update)

# chain_runner.hpp includes bindings.gen.hpp: compile after every generator
# (SpadeSlang.cmake's spade_slang_attach_generated() says why an explicit
# build-order edge, and not the link to spade::compute).
spade_slang_attach_generated(spade_airflow_runner)
