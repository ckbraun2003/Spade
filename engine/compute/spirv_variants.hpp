#pragma once

// ---------------------------------------------------------------------------
// spirv_variants.hpp (S6 Task 9b) -- how ONE kernel source reaches the device
// as SEVERAL compiled modules, one per legal `BackendDesc::workgroup_size`.
//
// THE PROBLEM THIS SOLVES. A compute kernel's local size is baked into its
// SPIR-V by `[numthreads(...)]` -- it is an `OpExecutionMode <entry> LocalSize
// N 1 1` in the module's header, decided by slangc, not by the host. So a
// backend knob that claims to vary the local size has exactly two ways to be
// real: a specialization constant, or one compiled module per size. Task 9b
// measured the first and took the second (that task's report carries the
// slangc transcript and the rejection); this header is the second's data
// structure.
//
// WHAT THE BUILD PRODUCES. cmake/SpadeSlang.cmake's
// spade_slang_kernel_variants() compiles each compute kernel (twelve today)
// once per size in {32, 64, 128} with `-DSPADE_WG=<size>`, and
// engine/shaders/tools/embed_spirv.py emits ONE header per kernel holding all
// three embedded arrays plus a `kSpvVariants_<name>` of this type. A kernel
// with no knob to serve (fp32_math_probe, which probe_runner.cpp dispatches on
// its own terms) gets a set of ONE, at its literal 64 -- so every consumer,
// including tests/test_slang_layouts.cpp's SPIR-V policy scanner, walks the
// same shape and no module can be scanned-by-accident-omitted.
//
// DELIBERATELY VULKAN-FREE, like compute/backend.hpp beside it (S6 dependency
// rule, spec section 2): a variant set is a table of byte arrays and the sizes
// they were compiled for. compute/vulkan/step_recorder.cpp is what turns one
// into a VkShaderModule.
// ---------------------------------------------------------------------------

#include <cstddef>
#include <cstdint>
#include <span>

namespace spade::compute {

// One compiled module: the local size it was compiled for, and its words.
//
// RAW POINTER + COUNT RATHER THAN A std::span MEMBER, deliberately: these are
// aggregate-initialized inside a generated header at namespace scope as
// `inline constexpr`, and a pointer/size pair keeps that initialization a
// plain brace list with no constructor involved. `code()` hands out the span
// at the call site, which is where it is actually wanted.
struct SpirvVariant {
    uint32_t workgroup_size = 0;
    const uint32_t* words = nullptr;
    std::size_t word_count = 0;

    [[nodiscard]] constexpr std::span<const uint32_t> code() const noexcept {
        return std::span<const uint32_t>(words, word_count);
    }
};

// Every compiled module for one kernel source, keyed by local size.
struct SpirvVariantSet {
    const char* name = nullptr;  // the kernel source's own name, for diagnostics
    const SpirvVariant* variants = nullptr;
    std::size_t count = 0;

    [[nodiscard]] constexpr std::span<const SpirvVariant> all() const noexcept {
        return std::span<const SpirvVariant>(variants, count);
    }

    // The variant compiled for `workgroup_size`, or nullptr if this kernel has
    // none. NULLPTR RATHER THAN A DEFAULT OR THE NEAREST SIZE: a caller that
    // dispatched a 64-wide module on a grid divided by 32 would be committing a
    // Vulkan valid-usage violation (and, worse, one that mostly "works"), so the
    // only safe answer to "no such variant" is an error the caller must handle
    // -- see compute/vulkan/step_recorder.cpp's create(), which turns it into a
    // named Code::invalid_argument naming this set's `name`.
    [[nodiscard]] constexpr const SpirvVariant* for_size(uint32_t workgroup_size) const noexcept {
        for (std::size_t i = 0; i < count; ++i) {
            if (variants[i].workgroup_size == workgroup_size) return &variants[i];
        }
        return nullptr;
    }
};

}  // namespace spade::compute
