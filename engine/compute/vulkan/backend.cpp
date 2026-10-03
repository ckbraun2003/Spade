#include "compute/vulkan/backend.hpp"

#include <string>
#include <utility>

#include "compute/vulkan/context.hpp"
#include "compute/vulkan/state_mirror.hpp"
#include "compute/vulkan/step_recorder.hpp"

namespace spade::compute {

// ---------------------------------------------------------------------------
// Impl -- the genuinely Vulkan-shaped state backend.hpp's forward declaration
// hides from every consumer outside engine/compute/. Owns, in construction/
// destruction order: the headless context, the device-buffer mirror (bound
// to that context's device), and the step recorder (bound to the mirror's
// descriptor set). unique_ptr member order is declaration order is
// destruction order (reverse), so listing them context-then-mirror-then-
// recorder here tears down recorder first, then mirror, then context --
// exactly the dependency order each was built in.
// ---------------------------------------------------------------------------
struct VulkanBackend::Impl {
    std::unique_ptr<VulkanContext> ctx;
    std::unique_ptr<StateMirror> mirror;
    std::unique_ptr<StepRecorder> recorder;
};

VulkanBackend::VulkanBackend() : impl_(std::make_unique<Impl>()) {}
VulkanBackend::~VulkanBackend() = default;
VulkanBackend::VulkanBackend(VulkanBackend&&) noexcept = default;
VulkanBackend& VulkanBackend::operator=(VulkanBackend&&) noexcept = default;

Result<std::unique_ptr<VulkanBackend>> VulkanBackend::create(const BackendDesc& desc, const StepShape& shape,
                                                             std::span<const GpuPass> passes) {
    // ARGUMENT VALIDATION BEFORE RESOURCE ACQUISITION (S6 Task 9b). Checked
    // ahead of VulkanContext::create() deliberately, and the ordering is a
    // contract rather than a style preference:
    //
    //   * A wrong workgroup_size is wrong on a box with no Vulkan device at
    //     all, so answering it with Code::unavailable ("no device") would tell
    //     the caller the wrong thing on every CI runner. Validating first means
    //     the diagnosis does not depend on the environment.
    //   * It is CHEAP and TOTAL -- a lookup in a three-element compiled-in
    //     array -- so there is nothing to gain by deferring it past an
    //     instance, a device and two allocators.
    //
    // Code::invalid_argument, matching desc.device_index's out-of-range
    // rejection one layer down (context.cpp): the caller asked for something
    // this build cannot provide, which is different from the resource being
    // architecturally absent. The message names the allowed set from the ONE
    // place it is defined (compute/backend.hpp) so it can never go stale.
    if (!workgroup_size_supported(desc.workgroup_size)) {
        return std::unexpected(Error{Code::invalid_argument,
                                     "BackendDesc::workgroup_size " +
                                         std::to_string(desc.workgroup_size) +
                                         " is not one of the compiled " + supported_workgroup_sizes_text() +
                                         ". A kernel's local size is compiled into its SPIR-V, so a "
                                         "backend can only offer the sizes the build compiled a variant "
                                         "for (cmake/SpadeSlang.cmake)."});
    }

    // A pass with no GPU kernel, checked before any device is touched for the
    // same reason: it is wrong on every machine. (StepRecorder::create() keeps
    // the closer guard.)
    for (const GpuPass& pass : passes) {
        if (pass.recipe == GpuRecipe::none) {
            return std::unexpected(Error{Code::invalid_argument, "VulkanBackend::create: pass '" + pass.name +
                                                                     "' has no GPU kernel (GpuRecipe::none)"});
        }
    }

    Result<std::unique_ptr<VulkanContext>> ctx = VulkanContext::create(desc);
    if (!ctx) return std::unexpected(ctx.error());

    Result<std::unique_ptr<StateMirror>> mirror = StateMirror::create(**ctx, shape);
    if (!mirror) return std::unexpected(mirror.error());

    // The recorder is handed the mirror's step-params mapping rather than
    // allocating its own (S6 Task 6): a descriptor set is written once, from
    // buffers that already exist, so a buffer created after the mirror could
    // never be bound. See StateMirror::step_params_mapped().
    Result<std::unique_ptr<StepRecorder>> recorder =
        StepRecorder::create(**ctx, shape, (*mirror)->descriptor_set_layout(), (*mirror)->descriptor_set(),
                             (*mirror)->step_params_mapped(), desc.workgroup_size, passes);
    if (!recorder) return std::unexpected(recorder.error());

    // std::unique_ptr<VulkanBackend>(new VulkanBackend()): the default
    // constructor is private (construction is only ever valid through this
    // factory), matching VulkanContext::create()'s identical precedent, so
    // make_unique -- which requires a public constructor -- cannot be used
    // here either.
    auto self = std::unique_ptr<VulkanBackend>(new VulkanBackend());
    self->impl_->ctx = std::move(*ctx);
    self->impl_->mirror = std::move(*mirror);
    self->impl_->recorder = std::move(*recorder);
    return self;
}

Result<void> VulkanBackend::upload(const ArenaSet& arenas) { return impl_->mirror->upload(arenas); }

Result<void> VulkanBackend::upload_dryden_params(std::span<const std::byte> params_bytes, uint32_t elem_size,
                                                   uint32_t world_count) {
    return impl_->mirror->upload_dryden_params(params_bytes, elem_size, world_count);
}

Result<void> VulkanBackend::upload_sdf_program(std::span<const std::byte> node_bytes,
                                                std::span<const std::byte> transform_bytes,
                                                std::span<const std::byte> range_bytes) {
    return impl_->mirror->upload_sdf_program(node_bytes, transform_bytes, range_bytes);
}

Result<void> VulkanBackend::upload_contact_params(std::span<const std::byte> params_bytes, uint32_t elem_size,
                                                   uint32_t world_count) {
    return impl_->mirror->upload_contact_params(params_bytes, elem_size, world_count);
}

Result<void> VulkanBackend::upload_grid_params(std::span<const std::byte> params_bytes, uint32_t elem_size,
                                                uint32_t world_count) {
    return impl_->mirror->upload_grid_params(params_bytes, elem_size, world_count);
}

Result<void> VulkanBackend::read_step_witness(std::span<std::byte> out_bytes) {
    return impl_->mirror->read_step_witness(out_bytes);
}

Result<void> VulkanBackend::read_grid_entries(std::span<std::byte> out_bytes) {
    return impl_->mirror->read_grid_entries(out_bytes);
}

std::size_t VulkanBackend::grid_entries_byte_size() const noexcept {
    return impl_->mirror->grid_entries_byte_size();
}

Result<void> VulkanBackend::step(uint64_t n, uint64_t first_tick) { return impl_->recorder->submit(n, first_tick); }

Result<void> VulkanBackend::readback(ArenaSet& arenas) { return impl_->mirror->readback(arenas); }

uint64_t VulkanBackend::upload_count() const noexcept { return impl_->mirror->upload_count(); }

uint64_t VulkanBackend::last_written_tick() const noexcept { return impl_->recorder->last_written_tick(); }

Result<PassDurationsNs> VulkanBackend::read_pass_durations_ns() const {
    return impl_->recorder->pass_durations_ns();
}

RecordedChain VulkanBackend::recorded_chain() const noexcept { return impl_->recorder->recorded_chain(); }

uint32_t VulkanBackend::bound_binding_count() const noexcept { return impl_->mirror->bound_binding_count(); }

}  // namespace spade::compute
