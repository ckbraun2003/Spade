#pragma once

// ---------------------------------------------------------------------------
// probe_runner.hpp -- ONE DISPATCH, START TO FINISH. Hand it a compiled SPIR-V
// module, a buffer of floats to read and a buffer of floats to fill, and it
// creates the pipeline, uploads, dispatches, fences, reads back and tears
// everything down again. It is the smallest thing that can run a kernel on the
// device, and that is the whole design goal: a parity test's evidence must not
// depend on machinery the test itself cannot see through.
//
// KERNEL-AGNOSTIC ON PURPOSE. Task 4 uses it for the fp32_math batteries and
// Task 7 reuses it for the sort-isolation tests, so it knows nothing about
// either -- no mode flag, no kernel-specific push value, no float semantics.
// A probe kernel that needs a selector puts it in its own argument buffer
// (engine/shaders/kernels/fp32_math_probe.slang does exactly that, in args[0],
// and says so).
//
// NOT A DISPATCHER. T5 owns the real pass dispatch path -- persistent buffers,
// the state descriptor set, per-substep submission, no per-call pipeline
// creation. This function creates and destroys EVERYTHING on every call, which
// costs a few milliseconds and buys total isolation between dispatches; that
// trade is right for a test harness and wrong for a simulation loop. Nothing
// in the engine calls it.
//
// ---------------------------------------------------------------------------
// THE CONTRACT A PROBE KERNEL MUST MEET, in full:
//
//   * ONE descriptor set, set 0, with exactly two bindings, both
//     VK_DESCRIPTOR_TYPE_STORAGE_BUFFER:
//         binding 0  the ARGUMENT buffer   (StructuredBuffer<float>)
//         binding 1  the RESULT buffer     (RWStructuredBuffer<float>)
//     This set 0 is the PROBE's own layout on the probe's own pipeline. It is
//     unrelated to -- and must never be confused with -- the state binding
//     registry's set 0 (engine/shaders/shared/bindings.slang), which is the
//     ABI T5's dispatch path writes descriptors for. Two different pipelines
//     may both use set 0; they simply have different layouts.
//
//   * ONE push-constant block of two uints, `{ arg_count, out_count }`, both
//     ELEMENT counts (not bytes), visible to the compute stage at offset 0.
//
//   * `[numthreads(kProbeWorkgroupSize, 1, 1)]`, one thread per RESULT
//     element, and its own `if (i >= out_count) return;` guard -- the dispatch
//     is rounded up to whole workgroups, so the tail threads are real.
//
// The fp32_math probe's compiled SPIR-V is checked against the first two of
// those by GpuFp32Math.ProbeKernelDeclaresTheRunnerBindings, which reads the
// binding decorations straight out of the module rather than trusting this
// comment.
//
// ---------------------------------------------------------------------------
// MEMORY. Both buffers are allocated from a HOST_VISIBLE + HOST_COHERENT heap,
// preferring one that is also DEVICE_LOCAL -- which on this program's
// correctness device (an integrated Intel Iris Plus, where host and device
// share the same physical memory) is the normal case, and makes upload and
// readback plain memcpys with no staging buffer and no transfer submission.
// On a discrete GPU the same code still works through the PCIe-visible heap;
// it is simply slower, which a probe does not care about.
//
// SIZE. `args` and `out` are the caller's chunking decision, and the caller is
// expected to make one: this program's device has ~1 GB of shared memory and
// the global constraint asks for <= 64 MB per dispatch. run_probe() does not
// chunk on the caller's behalf -- it would have to invent a per-kernel
// argument-to-result correspondence to do so -- but it DOES reject a dispatch
// whose workgroup count exceeds the device's own limit, with
// Code::invalid_argument, rather than letting the driver fail it late.
// ---------------------------------------------------------------------------

#include <cstddef>
#include <cstdint>
#include <span>

#include "compute/vulkan/context.hpp"
#include "core/error.hpp"

namespace spade::compute {

// The local size every probe kernel declares, and the divisor run_probe() uses
// to size the dispatch. Stated here, once, because the two must agree and the
// kernel cannot read this header.
inline constexpr uint32_t kProbeWorkgroupSize = 64;

// The probe descriptor-set layout, as the host writes it. A probe kernel
// declares the same two indices in set 0.
inline constexpr uint32_t kProbeSet = 0;
inline constexpr uint32_t kProbeArgBinding = 0;
inline constexpr uint32_t kProbeOutBinding = 1;

// Runs `spirv`'s single compute entry point over `args`, filling `out`.
//
// One thread per `out` element; `out.size()` rounded up to whole workgroups.
// Both element counts reach the kernel as push constants so it can bound-check
// its own reads and writes.
//
// Returns:
//   * Code::invalid_argument -- `spirv` is not a whole number of 32-bit words
//     or is shorter than a SPIR-V header; `out` is empty; the dispatch would
//     exceed the device's maxComputeWorkGroupCount[0].
//   * Code::internal -- any Vulkan call failed, or the fence timed out. The
//     `context` string names the call.
//
// SYNCHRONOUS: it does not return until the dispatch has completed and `out`
// holds the device's results. Every resource it created is destroyed before
// it returns, on the failure paths as well as the success one.
[[nodiscard]] Result<void> run_probe(VulkanContext& ctx, std::span<const std::byte> spirv,
                                     std::span<const float> args, std::span<float> out);

}  // namespace spade::compute
