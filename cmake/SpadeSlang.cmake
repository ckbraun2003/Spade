# ==============================================================================
# SpadeSlang.cmake -- the D9 shader build. BUILD FUNCTIONS ONLY: this file
# declares no target and compiles nothing on its own; engine/CMakeLists.txt
# calls the two functions below from inside its SPADE_VULKAN block.
#
#   spade_slang_generated_layouts(<target> ...)
#       slangc reflection over shaders/shared/bindings.slang -> reflection JSON
#       -> layout_check.gen.hpp + bindings.gen.hpp, wired onto <target>'s
#       include path.
#
#   spade_slang_kernel(<target> <name> ...)
#       shaders/kernels/<name>.slang -> <name>.spv -> <name>.spv.gen.hpp
#       (uint32_t[] embedding), wired onto <target>'s include path.
#
#   spade_slang_kernel_variants(<target> <name> ...)
#       the same, ONCE PER LOCAL SIZE in SPADE_SLANG_WORKGROUP_SIZES -- the
#       A7 workgroup_size knob's compiled half (S6 Task 9b).
#
# ------------------------------------------------------------------------------
# GENERATION MECHANISM: PYTHON, NOT CMAKE SCRIPT MODE -- the decision the brief
# required be recorded here, with its reasons rather than as an assertion.
#
# CMake 3.19+ does have string(JSON ...), so "consume the reflection JSON in
# cmake -P script mode" was a real option and was rejected on four counts:
#
#   1. THE INPUT IS RECURSIVE. A reflected struct's fields may themselves be
#      structs (RngStream inside DrydenState and ImuSensorRow; ContactParams
#      and GridParams inside PassParams), to arbitrary depth. string(JSON) has
#      no iteration primitive -- every array walk is a LENGTH query plus an
#      index loop plus a GET per member -- and CMake functions cannot recurse
#      cleanly over a document without threading a path prefix string through
#      every call. The identical walk is eight lines of Python.
#   2. ERROR REPORTING. This generator's whole value is FAILING LOUDLY on an
#      unclaimed struct, a directive naming a struct nothing reflects, or an
#      unexpected binding kind. string(JSON) reports a missing member by
#      halting configure with a message about a JSON path; Python reports it
#      with the struct's name and what the author should do about it.
#   3. WHEN IT RUNS. CMake script mode invoked as a custom command is a
#      separate cmake process per build anyway, so it buys no configure-time
#      integration over a python process -- the supposed "no extra dependency"
#      advantage is the only real one, and see 4.
#   4. PYTHON IS ALREADY PRESENT WHERE THIS REPOSITORY BUILDS (this box, and
#      the hosted runners while they existed), and find_package(Python3
#      COMPONENTS Interpreter) is confined to the SPADE_VULKAN branch -- so a SPADE_VULKAN=OFF configure,
#      the documented escape hatch for an environment that cannot even fetch
#      the toolchain, never looks for it.
#
# There is therefore exactly ONE reflection-consuming generator in the tree:
# engine/shaders/tools/gen_layout_check.py. (engine/shaders/tools/
# embed_spirv.py is a different job -- bytes to a C array, no reflection
# involved -- in the same language, invoked by the same interpreter; see its
# own header comment.)
#
# ------------------------------------------------------------------------------
# WHY THE GENERATED HEADERS LIVE IN THE BUILD TREE AND ARE NEVER COMMITTED.
# A committed generated header is a hand-maintained duplicate wearing a
# disclaimer: it goes stale the first time someone edits a .slang file without
# re-running the generator, and the staleness is invisible in review. Generated
# into the build tree, they cannot be stale -- ninja rebuilds them from the
# .slang sources whenever those change (the reflection command lists the shared
# modules in DEPENDS; the kernel commands use slangc's own -depfile, which
# reports the transitive import graph). Nothing is added to .gitignore: build
# directories are already ignored wholesale.
#
# ------------------------------------------------------------------------------
# WHY -fp-mode precise IS ON EVERY KERNEL COMPILE (the parity half of D9).
# Global constraint: "FMA/contraction pinned OFF on parity paths ... Slang
# side: precise qualifiers / NoContraction on parity-path arithmetic, enforced
# by the build-gated SPIR-V check". Measured on slang v2026.14.1 (this task's
# report has the transcript): the `precise` KEYWORD alone emits nothing on the
# direct SPIR-V path -- a kernel written with `precise float m = a*b;` produces
# an OpFMul carrying no decoration at all -- while -fp-mode precise decorates
# EVERY contractable float op in the module with NoContraction, which is the
# module-level equivalent the constraint allows and the CPU side's own
# /fp:precise + -ffp-contract=off analogue. Removing this flag makes
# SlangSpirv.FloatControlsPinned fail immediately; that is the intended
# tripwire, not an accident of flag ordering.
#
# ------------------------------------------------------------------------------
# WHY NO KERNEL REQUESTS AN fp32 DENORMAL MODE (amended 2026-10-05).
# Vulkan leaves fp32 denormal handling IMPLEMENTATION-DEFINED unless a module
# requests a mode (SPV_KHR_float_controls). Requesting a mode the device does
# not support (VkPhysicalDeviceFloatControlsProperties) is a VULKAN VALID-USAGE
# VIOLATION, i.e. UNDEFINED BEHAVIOUR, not a reported error. So no compile here
# passes a -denorm-mode flag, and every device runs the kernels legally with
# its own default.
#
# HISTORY. From S6 Task 4 to 2026-10-05, every kernel was compiled
# -denorm-mode-fp32 preserve, after a MEASURED finding:
#   - The Intel Iris Plus (driver 0x0019484D) flushes denormals to zero by
#     default, while advertising both behaviours.
#   - The CPU twin (x86 SSE, default MXCSR) preserves.
#   - So log32's pre-scale, exp32's subnormal tail and sin32 of a subnormal
#     diverged.
# That pin needed shaderDenormPreserveFloat32, and VulkanContext refused any
# device without it. The RTX 3060 Ti has neither mode, so it was refused. The
# user's ruling of 2026-10-05 ended bit-exact CPU<->GPU parity, and option (b)
# of docs/design/core/plans/2026-10-05-banded-parity-plan.md removed the flag.
#
# WHAT IT COSTS, AND WHAT COVERS IT.
#   - Both devices on record flush by default, so a GPU result can differ
#     from the CPU's at subnormal magnitudes (below 1.18e-38).
#   - fp32_math's subnormal paths are integer forms that do not depend on the
#     mode (the plan's T1), so that library stays exact on every device
#     measured.
#   - Everything else is banded against the CPU (TD-14).
#
# THE TRIPWIRE. engine/testing/spirv_scan.hpp's rule P3: a module that declares
# a DenormPreserve or DenormFlushToZero mode or capability fails
# SlangSpirv.FloatControlsPinned before any device runs it. That covers a flag
# put back here and a future slangc default alike.
#
# WHAT IS *NOT* PINNED, stated rather than left unsaid: the other two
# float-controls execution modes, signed-zero/Inf/NaN preservation and
# round-to-nearest-even rounding. slangc v2026.14.1 exposes no flag for either
# (`slangc -h` lists only -denorm-mode-fp16/32/64, and no float-controls entry
# appears in `slangc -h capability`). The Iris Plus advertised both as
# supported and honoured them in practice:
# GpuFp32Math.EdgeCasesAreBitIdenticalToTheHost compares +-0, +-inf and NaN
# through all four fp32_math kernels and matched bit for bit. So the exposure
# is a missing PIN, not a missing behaviour. Recorded here for whoever next
# revisits the toolchain pins.
# ==============================================================================

if(NOT SPADE_SLANGC)
    message(FATAL_ERROR
        "SpadeSlang.cmake requires SPADE_SLANGC (the pinned slangc executable). "
        "vendor/CMakeLists.txt sets it after fetching the slang prebuilt.")
endif()

find_package(Python3 REQUIRED COMPONENTS Interpreter)

set(SPADE_SLANG_SHADERS_DIR "${PROJECT_SOURCE_DIR}/engine/shaders")
set(SPADE_SLANG_TOOLS_DIR "${PROJECT_SOURCE_DIR}/engine/shaders/tools")
set(SPADE_SLANG_SHARED_DIR "${PROJECT_SOURCE_DIR}/engine/shaders/shared")
set(SPADE_SLANG_KERNELS_DIR "${PROJECT_SOURCE_DIR}/engine/shaders/kernels")

# The shared modules every compile imports. Listed explicitly rather than
# globbed: the reflection command below cannot use slangc's -depfile (slangc
# emits one only when it actually generates output, and the reflection pass
# runs with -no-codegen, verified on v2026.14.1), so this list IS the
# reflection step's dependency edge. Two files today; a third shared module
# belongs here the moment it exists.
set(SPADE_SLANG_SHARED_MODULES
    "${SPADE_SLANG_SHARED_DIR}/layouts.slang"
    "${SPADE_SLANG_SHARED_DIR}/bindings.slang"
)

# The SPIR-V flavour every compile targets. glsl_450 yields SPIR-V 1.3, which
# every Vulkan 1.1+ device accepts -- including this box's Iris Plus at Vulkan
# 1.3.215 -- and asks for nothing newer than the kernels need. Pinned here, in
# one place, so no kernel can be compiled against a different profile than the
# module the layout checks were reflected from.
set(SPADE_SLANG_PROFILE "glsl_450")

# ------------------------------------------------------------------------------
# THE LOCAL SIZES EVERY SCHEDULE KERNEL IS COMPILED AT (S6 Task 9b).
#
# MUST MATCH compute/backend.hpp's kSupportedWorkgroupSizes EXACTLY, and the
# match is TESTED rather than trusted (tests/test_gpu_invariance.cpp's
# WorkgroupSizeContract.AllowedSetMatchesCompiledVariants): a size this list
# compiles but that header rejects is a wasted build; a size that header admits
# but this list does not compile is a value that validates and then fails at
# pipeline creation, which is strictly worse.
#
# WHY A LIST HERE AT ALL, rather than one compile and a specialization constant.
# MEASURED on the pinned slangc v2026.14.1 (Task 9b's report carries the
# transcript): a `[vk::constant_id(0)] const int` driving [numthreads] does
# compile, and it does keep the -fp-mode pin -- but slangc emits
# it as `OpExecutionModeId <entry> LocalSizeId ...`, NOT as the SpecId-decorated
# `BuiltIn WorkgroupSize` composite. Vulkan gates LocalSizeId behind the
# maintenance4 feature for any SPIR-V below 1.6 (these modules are 1.3), so that
# route would mean enabling a device feature and refusing every device without
# it -- see compute/vulkan/context.cpp's deliberate "NO pEnabledFeatures,
# STILL" note -- to buy a TUNING knob. Three compiles per kernel cost build time
# and nothing else, and each variant is independently scanned by the SPIR-V
# policy gate.
set(SPADE_SLANG_WORKGROUP_SIZES 32 64 128)

# ------------------------------------------------------------------------------
# spade_slang_generated_layouts(<target>)
#
# Adds the reflection + generation commands, makes <target> depend on their
# outputs, and puts the generated-header directory on <target>'s include path
# as a BUILD_INTERFACE entry (build tree only -- these headers are never
# installed, so an installed consumer must never see the directory in its
# usage requirements).
#
# PUBLIC on the include directory, deliberately: spade_tests includes
# layout_check.gen.hpp and bindings.gen.hpp itself (its compiling IS half the
# test), and it reaches them by linking spade::compute rather than by
# re-deriving the build path.
# ------------------------------------------------------------------------------
function(spade_slang_generated_layouts target)
    set(_gen_dir "${CMAKE_CURRENT_BINARY_DIR}/generated/spade_slang")
    set(_reflection "${_gen_dir}/bindings.reflection.json")
    set(_layout_hpp "${_gen_dir}/layout_check.gen.hpp")
    set(_bindings_hpp "${_gen_dir}/bindings.gen.hpp")

    # Step 1: slangc reflects bindings.slang WITHOUT generating code.
    # -no-codegen type-checks and lays the module out, then reports every
    # global shader parameter in it -- even though no entry point references
    # any of them. That is what lets the registry be a declaration-only file:
    # reflecting a KERNEL instead would make each binding's presence in the
    # generated header depend on whether some kernel happened to read it.
    add_custom_command(
        OUTPUT "${_reflection}"
        COMMAND "${CMAKE_COMMAND}" -E make_directory "${_gen_dir}"
        COMMAND "${SPADE_SLANGC}"
                "${SPADE_SLANG_SHARED_DIR}/bindings.slang"
                -target spirv
                -profile ${SPADE_SLANG_PROFILE}
                -no-codegen
                -I "${SPADE_SLANG_SHARED_DIR}"
                -reflection-json "${_reflection}"
        DEPENDS ${SPADE_SLANG_SHARED_MODULES}
        COMMENT "slangc: reflecting shaders/shared/bindings.slang"
        VERBATIM
    )

    # Step 2: the generator turns that record into the two headers. It depends
    # on the .slang sources as well as on the JSON because it reads the @cpp
    # and @set DIRECTIVES straight out of them -- a directive edited without
    # any layout changing must still regenerate.
    add_custom_command(
        OUTPUT "${_layout_hpp}" "${_bindings_hpp}"
        COMMAND "${Python3_EXECUTABLE}"
                "${SPADE_SLANG_TOOLS_DIR}/gen_layout_check.py"
                --reflection "${_reflection}"
                --layouts "${SPADE_SLANG_SHARED_DIR}/layouts.slang"
                --bindings "${SPADE_SLANG_SHARED_DIR}/bindings.slang"
                --layout-out "${_layout_hpp}"
                --bindings-out "${_bindings_hpp}"
        DEPENDS "${_reflection}"
                "${SPADE_SLANG_TOOLS_DIR}/gen_layout_check.py"
                ${SPADE_SLANG_SHARED_MODULES}
        COMMENT "gen_layout_check.py: layout_check.gen.hpp + bindings.gen.hpp"
        VERBATIM
    )

    # A custom target, not a bare source-file dependency: generated headers are
    # not listed in any target's SOURCES (nothing compiles them directly --
    # compute/layout_check.cpp #includes them), so ninja needs an explicit
    # ordering edge or it will happily compile that TU before the generator has
    # run. The dependency is on the TARGET so every consumer that links
    # spade::compute inherits the ordering too.
    add_custom_target(spade_slang_layouts_gen DEPENDS "${_layout_hpp}" "${_bindings_hpp}")
    add_dependencies(${target} spade_slang_layouts_gen)
    set_property(GLOBAL APPEND PROPERTY SPADE_SLANG_GENERATOR_TARGETS spade_slang_layouts_gen)

    target_include_directories(${target} PUBLIC "$<BUILD_INTERFACE:${_gen_dir}>")
endfunction()

# ------------------------------------------------------------------------------
# spade_slang_attach_generated(<target>)
#
# Orders <target>'s own compilation after every generator this file registered.
# The generated-header INCLUDE PATH already reaches any target that links
# spade::compute (the functions above add it PUBLIC), but a build-ORDER edge is
# a separate question, and one worth answering explicitly rather than relying on
# a generator's transitive-order-dependency behaviour: spade_tests compiles
# test_slang_layouts.cpp, which #includes three generated headers, and an
# object-compile racing the generator is precisely the kind of failure that
# reproduces once in ten clean builds and never on the box you debug it on.
#
# Callable from any directory scope; the target list is a GLOBAL property, set
# by the producer functions above.
# ------------------------------------------------------------------------------
function(spade_slang_attach_generated target)
    get_property(_generators GLOBAL PROPERTY SPADE_SLANG_GENERATOR_TARGETS)
    if(NOT _generators)
        message(FATAL_ERROR
            "spade_slang_attach_generated(${target}): no Slang generators have been "
            "declared yet. Call it after engine/CMakeLists.txt's "
            "spade_slang_generated_layouts()/spade_slang_kernel() calls.")
    endif()
    add_dependencies(${target} ${_generators})
endfunction()

# ------------------------------------------------------------------------------
# spade_slang_kernel(<target> <name>)
#
# Compiles engine/shaders/kernels/<name>.slang to SPIR-V and embeds the result
# as `spade::compute::gen::kSpv_<name>[]` in <name>.spv.gen.hpp.
#
# The .spv itself IS a build output (the brief asks for it), and is also the
# artifact a human disassembles when a float-controls failure needs
# explaining; the embedded header is what the code and the tests actually
# consume, so nothing ever loads a file at runtime.
# ------------------------------------------------------------------------------
function(spade_slang_kernel target name)
    set(_gen_dir "${CMAKE_CURRENT_BINARY_DIR}/generated/spade_slang")
    set(_src "${SPADE_SLANG_KERNELS_DIR}/${name}.slang")
    set(_spv "${_gen_dir}/${name}.spv")
    set(_hpp "${_gen_dir}/${name}.spv.gen.hpp")
    set(_dep "${_gen_dir}/${name}.spv.d")

    if(NOT EXISTS "${_src}")
        message(FATAL_ERROR "spade_slang_kernel: no such kernel source: ${_src}")
    endif()

    # DEPFILE carries slangc's own transitive import graph (kernel + every
    # module it imports), so editing layouts.slang recompiles every kernel that
    # reaches it without this file listing the graph by hand.
    #
    # TWO -I PATHS (S6 Task 4). shaders/shared/ holds the modules EVERY kernel
    # needs (layouts, bindings); shaders/ itself holds the shared modules that
    # are not part of the binding/layout contract -- fp32_math.slang is the
    # first, and it belongs there rather than in shared/ precisely because
    # shared/ is the D9 reflection surface: SPADE_SLANG_SHARED_MODULES above is
    # the reflection step's dependency edge, and adding a pure math module to
    # it would re-run slangc's reflection over bindings.slang every time a
    # polynomial coefficient changed. Kernels reach it as `import fp32_math;`.
    add_custom_command(
        OUTPUT "${_spv}"
        COMMAND "${CMAKE_COMMAND}" -E make_directory "${_gen_dir}"
        COMMAND "${SPADE_SLANGC}"
                "${_src}"
                -target spirv
                -profile ${SPADE_SLANG_PROFILE}
                -fp-mode precise
                -I "${SPADE_SLANG_SHARED_DIR}"
                -I "${SPADE_SLANG_SHADERS_DIR}"
                -o "${_spv}"
                -depfile "${_dep}"
        DEPENDS "${_src}" ${SPADE_SLANG_SHARED_MODULES}
        DEPFILE "${_dep}"
        COMMENT "slangc: ${name}.slang -> SPIR-V (fp-mode precise)"
        VERBATIM
    )

    add_custom_command(
        OUTPUT "${_hpp}"
        COMMAND "${Python3_EXECUTABLE}"
                "${SPADE_SLANG_TOOLS_DIR}/embed_spirv.py"
                --input "${_spv}"
                --output "${_hpp}"
                --symbol "${name}"
        DEPENDS "${_spv}" "${SPADE_SLANG_TOOLS_DIR}/embed_spirv.py"
        COMMENT "embed_spirv.py: ${name}.spv -> ${name}.spv.gen.hpp"
        VERBATIM
    )

    add_custom_target(spade_slang_kernel_${name} DEPENDS "${_hpp}")
    add_dependencies(${target} spade_slang_kernel_${name})
    set_property(GLOBAL APPEND PROPERTY SPADE_SLANG_GENERATOR_TARGETS spade_slang_kernel_${name})
    set_property(GLOBAL APPEND PROPERTY SPADE_SLANG_KERNEL_NAMES ${name})

    target_include_directories(${target} PUBLIC "$<BUILD_INTERFACE:${_gen_dir}>")
endfunction()

# ------------------------------------------------------------------------------
# spade_slang_kernel_variants(<target> <name>)
#
# spade_slang_kernel() above, but compiled ONCE PER SIZE in
# SPADE_SLANG_WORKGROUP_SIZES, and embedded as ONE header holding all of them:
#
#   <name>.wg32.spv  \
#   <name>.wg64.spv   >-- <name>.spv.gen.hpp, with kSpvVariants_<name>
#   <name>.wg128.spv /
#
# THIS IS WHAT MAKES BackendDesc::workgroup_size A REAL KNOB (S6 Task 9b). A
# kernel's local size is compiled into its SPIR-V as `OpExecutionMode <entry>
# LocalSize N 1 1`; through Task 9 all nine schedule kernels spelled a literal
# `[numthreads(64,1,1)]`, so the knob reached nothing and T9's A7 sweep re-ran
# one configuration three times. The kernels now spell
# `[numthreads(SPADE_WG,1,1)]` and this function supplies SPADE_WG.
#
# NO `#ifndef SPADE_WG` FALLBACK IN THE KERNEL SOURCES, DELIBERATELY. A default
# would mean that dropping the -D below -- for one size, in one branch of this
# loop -- yields a 64-wide module that step_recorder.cpp then dispatches on a
# grid divided by 32 or 128. That is a Vulkan valid-usage violation whose
# symptom is wrong physics rather than an error, and it is exactly the class of
# silent mismatch this file's -fp-mode and denormal notes exist to prevent
# elsewhere. Without a fallback, the same slip is a slangc "undefined identifier
# SPADE_WG" at build time.
#
# WHY THE 64 VARIANT IS NOT A NEW ARTIFACT: `-DSPADE_WG=64` over
# `[numthreads(SPADE_WG,1,1)]` produces a byte-identical .spv to the literal
# `[numthreads(64,1,1)]` compile it replaces (verified per kernel; Task 9b's
# report carries the sha256 table). The respelling is a build parameterization,
# not an arithmetic change, which is what lets the T8 parity tables stay
# untouched.
# ------------------------------------------------------------------------------
function(spade_slang_kernel_variants target name)
    set(_gen_dir "${CMAKE_CURRENT_BINARY_DIR}/generated/spade_slang")
    set(_src "${SPADE_SLANG_KERNELS_DIR}/${name}.slang")
    set(_hpp "${_gen_dir}/${name}.spv.gen.hpp")

    if(NOT EXISTS "${_src}")
        message(FATAL_ERROR "spade_slang_kernel_variants: no such kernel source: ${_src}")
    endif()

    set(_spv_outputs "")
    set(_embed_args "")

    foreach(_wg IN LISTS SPADE_SLANG_WORKGROUP_SIZES)
        set(_spv "${_gen_dir}/${name}.wg${_wg}.spv")
        set(_dep "${_gen_dir}/${name}.wg${_wg}.spv.d")

        # Identical to spade_slang_kernel()'s compile in every respect except
        # the -D: same profile, same -fp-mode precise, the same absence of any
        # -denorm-mode flag, same two -I paths, same -depfile. The pins are
        # per-compile flags, so EVERY variant carries them and the SPIR-V
        # policy gate checks every variant independently.
        add_custom_command(
            OUTPUT "${_spv}"
            COMMAND "${CMAKE_COMMAND}" -E make_directory "${_gen_dir}"
            COMMAND "${SPADE_SLANGC}"
                    "${_src}"
                    -target spirv
                    -profile ${SPADE_SLANG_PROFILE}
                    -fp-mode precise
                    -DSPADE_WG=${_wg}
                    -I "${SPADE_SLANG_SHARED_DIR}"
                    -I "${SPADE_SLANG_SHADERS_DIR}"
                    -o "${_spv}"
                    -depfile "${_dep}"
            DEPENDS "${_src}" ${SPADE_SLANG_SHARED_MODULES}
            DEPFILE "${_dep}"
            COMMENT "slangc: ${name}.slang -> SPIR-V (local size ${_wg}, fp-mode precise)"
            VERBATIM
        )

        list(APPEND _spv_outputs "${_spv}")
        list(APPEND _embed_args --variant "${_wg}=${_spv}")
    endforeach()

    # ONE embed invocation for the whole set, so the generated header's variant
    # table is written by the same pass that writes the arrays -- a table
    # assembled from separately-generated headers could disagree with them.
    #
    # _wg_list is COMMA-joined for the COMMENT below, not interpolated
    # directly: ${SPADE_SLANG_WORKGROUP_SIZES} is a CMake list, whose internal
    # representation IS semicolon-separated, so a bare ${...} substitution
    # inside the string prints the raw separator too -- "wg{32;64;128}", which
    # reads as if a single literal file named that were produced (it is not;
    # ${_spv_outputs} above is the three separate per-variant files this
    # target actually depends on). Cosmetic only (S6 hygiene, T9b-M5): no
    # build behavior depends on this string, only the ninja/msbuild progress
    # line a human reads.
    string(REPLACE ";" "," _wg_list "${SPADE_SLANG_WORKGROUP_SIZES}")
    add_custom_command(
        OUTPUT "${_hpp}"
        COMMAND "${Python3_EXECUTABLE}"
                "${SPADE_SLANG_TOOLS_DIR}/embed_spirv.py"
                ${_embed_args}
                --output "${_hpp}"
                --symbol "${name}"
        DEPENDS ${_spv_outputs} "${SPADE_SLANG_TOOLS_DIR}/embed_spirv.py"
        COMMENT "embed_spirv.py: ${name}.wg{${_wg_list}}.spv -> ${name}.spv.gen.hpp"
        VERBATIM
    )

    add_custom_target(spade_slang_kernel_${name} DEPENDS "${_hpp}")
    add_dependencies(${target} spade_slang_kernel_${name})
    set_property(GLOBAL APPEND PROPERTY SPADE_SLANG_GENERATOR_TARGETS spade_slang_kernel_${name})
    set_property(GLOBAL APPEND PROPERTY SPADE_SLANG_KERNEL_NAMES ${name})

    target_include_directories(${target} PUBLIC "$<BUILD_INTERFACE:${_gen_dir}>")
endfunction()

# ------------------------------------------------------------------------------
# spade_slang_write_kernel_manifest(<target>)
#
# T11 fix-wave (whole-plan review W7b, XS option): closes the gap
# tests/test_slang_layouts.cpp's EveryCompiledVariantIsScanned documents out
# loud -- kScheduleKernels there and kSpirvModules[] in the same file were
# both HAND-MAINTAINED literals, so that test's own ASSERT_EQ cross-checked
# one hand count against another, not against CMake's own authoritative list
# of what it actually compiled. A tenth kernel added to this file's
# spade_slang_kernel()/spade_slang_kernel_variants() calls, whose author
# edited neither hand literal, escaped every scanner in that file silently.
#
# CALL ONCE, AFTER EVERY spade_slang_kernel()/spade_slang_kernel_variants()
# CALL FOR <target> HAS RUN. engine/CMakeLists.txt calls this immediately
# after its last kernel declaration (S6 Task 8's sensor_imu, the tail of the
# SPADE_VULKAN kernel block). Called earlier, this would silently undercount
# rather than error -- CMake has no "no more kernels will be declared" event
# to hook -- so the ordering is load-bearing and is stated here rather than
# defended with a runtime check the accumulation itself cannot perform.
#
# Writes kernel_manifest.gen.hpp declaring
# spade::compute::gen::kCompiledSpirvKernelCount, a plain compile-time
# constant equal to the number of DISTINCT kernel names either producer
# function above has appended to the SPADE_SLANG_KERNEL_NAMES GLOBAL
# property. test_slang_layouts.cpp asserts std::size(kSpirvModules) against
# this constant instead of against a second hand-typed literal -- a kernel
# added to engine/CMakeLists.txt without a matching kSpirvModules[] row now
# fails a real cross-check against CMake's own list, not one hand count
# against another.
#
# COUNT ONLY, deliberately -- not the NAME-BIJECTION check the review's
# larger "S" option describes (asserting kSpirvModules[] covers CMake's list
# bijectively, not just matches its cardinality). This is the cheapest
# correct mechanism for the failure mode named above; the stronger form is
# ticketed for S7 rather than attempted here.
# ------------------------------------------------------------------------------
function(spade_slang_write_kernel_manifest target)
    set(_gen_dir "${CMAKE_CURRENT_BINARY_DIR}/generated/spade_slang")
    set(_hpp "${_gen_dir}/kernel_manifest.gen.hpp")

    get_property(_names GLOBAL PROPERTY SPADE_SLANG_KERNEL_NAMES)
    if(_names)
        list(REMOVE_DUPLICATES _names)
    endif()
    list(LENGTH _names _count)

    file(GENERATE OUTPUT "${_hpp}" CONTENT
"// GENERATED by spade_slang_write_kernel_manifest() (cmake/SpadeSlang.cmake).
// Do not edit; not committed (build tree only, like every other .gen.hpp in
// this directory -- see this file's own \"WHY THE GENERATED HEADERS LIVE IN
// THE BUILD TREE\" note).
#pragma once
#include <cstddef>

namespace spade::compute::gen {

// The number of DISTINCT kernel names spade_slang_kernel()/
// spade_slang_kernel_variants() registered for this target, as of the last
// configure. tests/test_slang_layouts.cpp's EveryCompiledVariantIsScanned
// asserts kSpirvModules[]'s size against this rather than a hand-typed
// literal (S6 Task 11 fix-wave, review W7b).
inline constexpr std::size_t kCompiledSpirvKernelCount = ${_count};

}  // namespace spade::compute::gen
")

    target_include_directories(${target} PUBLIC "$<BUILD_INTERFACE:${_gen_dir}>")
endfunction()
