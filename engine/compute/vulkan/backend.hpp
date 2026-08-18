#pragma once

// ---------------------------------------------------------------------------
// backend.hpp (S6 Task 5) -- the Vulkan backend SEAM: the one type
// sim/simulation.hpp holds and calls, and DELIBERATELY the second file (after
// compute/backend.hpp) in engine/compute/ with zero Vulkan dependency of its
// own -- no <volk.h>, no Vk* type anywhere in this header. That is not a
// stylistic choice, it is what makes it legal for sim/simulation.hpp/.cpp
// (outside engine/compute/) to include this header at all under the S6
// dependency rule ("nothing outside engine/compute/ includes a Vulkan
// header", spec section 2): if this header pulled in <volk.h> transitively,
// every translation unit that includes sim/simulation.hpp would too.
//
// THE PIMPL SPLIT THAT MAKES THIS POSSIBLE. VulkanBackend's private state
// (the VulkanContext, the StateMirror's device/staging buffers, the
// StepRecorder's command buffer and pipeline -- all genuinely Vulkan-shaped)
// lives in a forward-declared `Impl` defined only in backend.cpp, which DOES
// include <volk.h> (transitively, via state_mirror.hpp/step_recorder.hpp) --
// legally, because backend.cpp lives inside engine/compute/vulkan/, inside
// the boundary the dependency rule draws. Every public method here is
// declared, not defined, so this header alone never requires a complete
// Vulkan type.
//
// WHY THIS FILE EXISTS SEPARATELY FROM state_mirror.hpp/step_recorder.hpp
// (this task's two brief-listed files). Those two are Vulkan-touching by
// design -- they own the actual device buffers and command buffer -- and
// living under compute/vulkan/ they are allowed to include <volk.h>
// directly, which is exactly what a header sim/simulation.hpp could safely
// include must never do. VulkanBackend is the seam that composes them behind
// a Vulkan-free door, the same role compute/backend.hpp already plays for
// BackendKind/BackendDesc/StepShape one directory up.
// ---------------------------------------------------------------------------

#include <cstddef>
#include <cstdint>
#include <memory>
#include <span>

#include "compute/backend.hpp"
#include "core/error.hpp"
#include "state/arenas.hpp"

namespace spade::compute {

// ---------------------------------------------------------------------------
// VulkanBackend -- owned by Simulation when BackendDesc::kind == vulkan (S6
// Task 5's brief, verbatim interface). Move-only for the same reason every
// other RAII resource owner in this tree is (VulkanContext, ArenaSet): it
// owns real device resources with a real teardown order, so copying would
// either double-free or need reference counting nothing here uses.
// ---------------------------------------------------------------------------
class VulkanBackend {
public:
    // Builds a headless Vulkan context, the device-buffer mirror for the
    // full registered walk (StateMirror, sized from `shape`), and the
    // per-substep dispatch chain recorder (StepRecorder) bound to that mirror's
    // descriptor set. Fails wherever any of the three does:
    //   - Code::unavailable   -- no Vulkan loader/device (VulkanContext), or a
    //     device that cannot run the requested local size (its
    //     maxComputeWorkGroupInvocations / maxComputeWorkGroupSize[0] limits).
    //   - Code::invalid_argument -- desc.device_index out of range, or
    //     desc.workgroup_size outside compute/backend.hpp's
    //     kSupportedWorkgroupSizes (S6 Task 9b). The latter is checked FIRST,
    //     before any device is touched, so the diagnosis is the same with or
    //     without a Vulkan device present -- see create()'s own comment.
    //   - Code::capacity_exceeded -- a buffer or command-buffer allocation
    //     did not fit (VK_ERROR_OUT_OF_DEVICE_MEMORY / _HOST_MEMORY, or an
    //     absurd `shape` that overflows a VkDeviceSize computation); the
    //     CollisionDynamic key array exceeding kMaxGridEntries; or a shape whose
    //     widest dispatch grid exceeds maxComputeWorkGroupCount[0] at the
    //     requested local size (S6 Task 9b -- and note that a SMALLER
    //     workgroup_size needs MORE groups, so this one is size-dependent).
    //   - Code::internal -- any other Vulkan API failure, including
    //     VK_ERROR_DEVICE_LOST if the device is lost during setup.
    //
    // NO RunParams ARGUMENT AS OF S6 TASK 6b: Task 6 threaded world 0's
    // ContactParams/GridParams through here for PassParams's push constant;
    // Task 6b's PassParams decision tree found no kernel ever read them and
    // removed the fields (and RunParams itself -- see compute/backend.hpp's
    // doc comment on its removal), so this factory has nothing left to
    // receive from a caller for that purpose. Per-world material still
    // reaches the device -- see upload_contact_params()/upload_grid_params()
    // below, both PER-WORLD, uploaded once by Simulation::create() after this
    // call succeeds.
    [[nodiscard]] static Result<std::unique_ptr<VulkanBackend>> create(const BackendDesc& desc,
                                                                        const StepShape& shape);

    // Declared, not defaulted, here: Impl is only forward-declared in this
    // header (see the file header note above), and an implicitly-defined
    // special member touching a unique_ptr<Impl> would need Impl COMPLETE at
    // the point it is defined. Defining these four as `= default` in
    // backend.cpp -- where Impl's definition is visible -- is what makes
    // that legal; Simulation's own destructor/move members need the
    // identical treatment for the identical reason (sim/simulation.hpp/.cpp).
    ~VulkanBackend();
    VulkanBackend(const VulkanBackend&) = delete;
    VulkanBackend& operator=(const VulkanBackend&) = delete;
    VulkanBackend(VulkanBackend&&) noexcept;
    VulkanBackend& operator=(VulkanBackend&&) noexcept;

    // Uploads every registered array (the full 18-entry walk: the nine
    // arrays plus their nine `.slot_to_world` siblings -- state/arenas.hpp's
    // ArenaSet::registry()) from `arenas` into this backend's device
    // buffers, staging through host-visible memory. Fails with
    // invalid_argument if `arenas`' shape (per-array elem_size/world_count/
    // capacity_per_world) does not match what this backend was created for,
    // and with capacity_exceeded/internal on the underlying Vulkan failure
    // taxonomy `create()` documents.
    [[nodiscard]] Result<void> upload(const ArenaSet& arenas);

    // The per-world Dryden turbulence CONFIGURATION (binding 13,
    // `dryden_params` -- bindings.slang section C). NOT part of the verbatim
    // upload()/readback() contract: dryden_params is derived, backend-
    // internal storage, not a registered array (global constraint: "S6 adds
    // NO register_array call"), and its source data
    // (WorldConfig::turbulence, sim/simulation.hpp) lives on WorldConfig, not
    // in the ArenaSet upload()/readback() exchange with. It is CONFIG,
    // immutable for a Simulation's lifetime (world_set.hpp's
    // WorldInstanceDesc note), so Simulation::create() calls this exactly
    // once, right after VulkanBackend::create() succeeds, and step() never
    // calls it again. See this task's report for why this extra method
    // exists beside the brief's four verbatim ones.
    [[nodiscard]] Result<void> upload_dryden_params(std::span<const std::byte> params_bytes,
                                                      uint32_t elem_size, uint32_t world_count);

    // ---------------------------------------------------------------------
    // S6 Task 6's two additional CONFIG uploads and its one diagnostic
    // readback. All three follow upload_dryden_params()'s precedent exactly:
    // derived, backend-internal storage (no register_array is added
    // anywhere), sourced from Simulation's per-world WorldConfig rather than
    // from the ArenaSet, uploaded EXACTLY ONCE by Simulation::create() and
    // never touched by step().
    // ---------------------------------------------------------------------

    // The flattened per-world SDF programs (bindings 14-16) -- what
    // CollisionStatic walks. All three buffers in one call because they are
    // one object: the ranges buffer indexes the other two, so a partial
    // upload would leave the device holding a self-inconsistent program.
    // Build the bytes with compute/sdf_program.hpp's flatten_sdf_programs().
    [[nodiscard]] Result<void> upload_sdf_program(std::span<const std::byte> node_bytes,
                                                    std::span<const std::byte> transform_bytes,
                                                    std::span<const std::byte> range_bytes);

    // The PER-WORLD contact material (binding 19). Per world, not per run:
    // the golden corpus's `bounce` is a four-rung restitution ladder, and
    // CollisionStatic reads its own world's row exactly as the CPU pass reads
    // `w.contacts`.
    [[nodiscard]] Result<void> upload_contact_params(std::span<const std::byte> params_bytes,
                                                       uint32_t elem_size, uint32_t world_count);

    // The PER-WORLD broad-phase grid config (binding 20, S6 Task 6b -- the
    // checkpoint-1 heterogeneous-worlds ruling). Same shape and reasoning as
    // upload_contact_params() immediately above.
    [[nodiscard]] Result<void> upload_grid_params(std::span<const std::byte> params_bytes, uint32_t elem_size,
                                                    uint32_t world_count);

    // The StepWitness row (binding 18) the Integrate kernel published on its
    // most recent dispatch -- what closes S6 Task 5's deferred "prove a kernel
    // can READ the per-step buffer" half. Raw bytes, exactly
    // sizeof(compute::StepWitness); see compute/step_params.hpp for what each
    // field proves. Costs one device->host copy and one queue submit, so it is
    // a diagnostic call, not a step-path one.
    [[nodiscard]] Result<void> read_step_witness(std::span<std::byte> out_bytes);

    // The CollisionDynamic key array (binding 21, S6 Task 7) as the last
    // CollisionDynamic pass left it -- SORTED, since the sweep is the chain's
    // last dispatch and never reorders. Raw bytes, exactly
    // grid_entries_byte_size(); a diagnostic path (one device->host copy, one
    // submit), never called by step(). tests/test_gpu_parity.cpp's GpuGridSort
    // cases are its only consumer: they compare these keys element for element
    // against std::sort + physics::grid_entry_less over the same bodies, which
    // is what checks the sort NETWORK rather than the physics it feeds.
    [[nodiscard]] Result<void> read_grid_entries(std::span<std::byte> out_bytes);

    // Bytes in that buffer, so a caller sizes its span without recomputing
    // compute/grid_entry.hpp's grid_domain_of().
    [[nodiscard]] std::size_t grid_entries_byte_size() const noexcept;

    // Submits `n` steps of the ALREADY-RECORDED per-substep dispatch chain --
    // the SAME command buffer every time, never re-recorded (S6 Task 5
    // review round 1, finding C1) -- starting at `first_tick` (today:
    // Simulation::tick_'s value before this call). Before each submit,
    // writes StepParams{.tick = first_tick + step_index} into a
    // persistently mapped, host-visible buffer (round 1, finding C2) --
    // see last_written_tick() below and compute/vulkan/step_recorder.hpp's
    // StepParams doc comment for why this exists and what it does not yet
    // do (bind to a descriptor, be read by a kernel). Fails with
    // Code::internal mapping VK_ERROR_DEVICE_LOST from the queue submit or
    // the fence wait, or Code::capacity_exceeded if a per-step allocation
    // (none today, reserved for a future dynamic path) fails.
    [[nodiscard]] Result<void> step(uint64_t n, uint64_t first_tick);

    // The tick most recently written into the per-step host-visible params
    // buffer (compute/vulkan/step_recorder.hpp's StepParams) -- what this
    // task's C2 fix test reads to prove the host-side write genuinely
    // happens once per submit with the right value, without needing a
    // kernel to consume it (GPU-visible consumption of this buffer is
    // deliberately deferred to Task 6). 0 before the first step().
    [[nodiscard]] uint64_t last_written_tick() const noexcept;

    // Reads every registered array's device buffer back into `arenas`,
    // staging through host-visible memory -- the exact reverse of upload().
    // NEVER writes a partially-read or zero-filled array on failure: the
    // first per-array mismatch or Vulkan failure aborts before any
    // destination byte for that array (or any array after it) is touched,
    // and returns the mapped error rather than the silently-wrong state the
    // brief's fault-path requirement forbids.
    [[nodiscard]] Result<void> readback(ArenaSet& arenas);

    // Instrumentation the round-trip/dirty-tracking tests read (this task's
    // brief: "assert only the re-upload happened via an instrumentation
    // counter"). Counts successful upload() calls only -- a failed upload
    // does not advance it, so a test can tell "uploaded once" from "tried
    // and failed twice".
    [[nodiscard]] uint64_t upload_count() const noexcept;

    // S6 Task 10: per-pass GPU timing for the MOST RECENTLY COMPLETED step()
    // call -- forwards straight to compute/vulkan/step_recorder.hpp's
    // StepRecorder::pass_durations_ns(), which is where the "unconditional,
    // skip-gracefully" posture and the "safe to call any time after step()
    // returns" contract are actually documented. A diagnostic call, like
    // read_step_witness()/read_grid_entries() above: never on the step path
    // itself, purely additive to this class's existing surface.
    [[nodiscard]] Result<PassDurationsNs> read_pass_durations_ns() const;

private:
    VulkanBackend();

    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace spade::compute
