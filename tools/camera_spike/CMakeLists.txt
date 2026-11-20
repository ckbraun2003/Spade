# tools/camera_spike: the camera sensor's core spike (docs/design/rendering/2026-11-19-camera-sensor-design.md, section 8, V1 to V11).
# A standalone Vulkan ray-query benchmark: it links no engine library, changes no engine API and is not installed. It measures what the
# design needs before the technique is committed to (throughput at KAT's two P1 operating points, the lattice against the per-env
# TLAS, the export path, determinism, and a CPU oracle). Build-tree only, behind SPADE_VULKAN.
#
# THE SHADER IS NOT BUILT WITH SpadeSlang.cmake's spade_slang_kernel(): that function compiles every kernel with -fp-mode precise (the
# parity half of D9), which would measure a slower, contraction-free kernel than the camera would ship as (the camera is banded, not
# parity). So slangc runs here with its defaults, once per variant: L (one lattice TLAS), E128 and E1024 (an array of TLAS, one an env).
if(NOT SPADE_SLANGC)
    message(STATUS "camera_spike: SPADE_SLANGC is not set; the camera spike is not built")
    return()
endif()

set(_cs_shader "${CMAKE_CURRENT_SOURCE_DIR}/camera_spike.slang")
set(_cs_tex "${CMAKE_CURRENT_SOURCE_DIR}/tex.slang")  # the texture filter, included by the kernel and the probe
set(_cs_out "${CMAKE_CURRENT_BINARY_DIR}")
set(_cs_spv)
foreach(_variant IN ITEMS L E128 E1024)
    if(_variant STREQUAL "L")
        set(_defs)
    elseif(_variant STREQUAL "E128")
        set(_defs -D MODE_E=1 -D NUM_TLAS=128)
    else()
        set(_defs -D MODE_E=1 -D NUM_TLAS=1024)
    endif()
    set(_spv "${_cs_out}/camera_spike_${_variant}.spv")
    add_custom_command(
        OUTPUT "${_spv}"
        COMMAND "${SPADE_SLANGC}" "${_cs_shader}" -target spirv -profile spirv_1_5 -entry main -stage compute ${_defs} -o "${_spv}"
        DEPENDS "${_cs_shader}" "${_cs_tex}"
        COMMENT "slangc: camera_spike.slang -> camera_spike_${_variant}.spv"
        VERBATIM)
    list(APPEND _cs_spv "${_spv}")
endforeach()
# the texture-filter probe (V11): the filter alone, for the CPU-against-GPU check
set(_cs_probe "${CMAKE_CURRENT_SOURCE_DIR}/texprobe.slang")
set(_cs_probe_spv "${_cs_out}/texprobe.spv")
add_custom_command(
    OUTPUT "${_cs_probe_spv}"
    COMMAND "${SPADE_SLANGC}" "${_cs_probe}" -target spirv -profile spirv_1_5 -entry main -stage compute -o "${_cs_probe_spv}"
    DEPENDS "${_cs_probe}" "${_cs_tex}"
    COMMENT "slangc: texprobe.slang -> texprobe.spv"
    VERBATIM)
list(APPEND _cs_spv "${_cs_probe_spv}")
add_custom_target(camera_spike_shaders DEPENDS ${_cs_spv})

# volk.c compiles into this target with the win32 platform macro, as spade_compute does for its own use: the external-memory and
# semaphore handle calls (V5) are win32 entry points that volk only declares under VK_USE_PLATFORM_WIN32_KHR.
get_target_property(_cs_volk_dir volk SOURCE_DIR)
add_executable(camera_spike main.cpp "${_cs_volk_dir}/volk.c")
add_dependencies(camera_spike camera_spike_shaders)
target_compile_features(camera_spike PRIVATE cxx_std_23)
target_link_libraries(camera_spike PRIVATE $<BUILD_INTERFACE:volk> Vulkan::Headers $<BUILD_LOCAL_INTERFACE:spade_warnings>)
target_compile_definitions(camera_spike PRIVATE CAMERA_SPIKE_SPV_DIR="${_cs_out}")
if(WIN32)
    target_compile_definitions(camera_spike PRIVATE VK_USE_PLATFORM_WIN32_KHR NOMINMAX WIN32_LEAN_AND_MEAN)
endif()
unset(_cs_volk_dir)
