#include "compute/vulkan/backend.hpp"

// ---------------------------------------------------------------------------
// backend_stub.cpp (S6 Task 5 review round 1, finding I1) -- the
// SPADE_VULKAN=OFF fallback definition of spade::compute::VulkanBackend.
//
// WHY THIS FILE EXISTS. sim/simulation.cpp calls
// compute::VulkanBackend::create/upload/upload_dryden_params/step/readback
// UNCONDITIONALLY -- the routing between the cpu and vulkan paths is a
// RUNTIME branch on BackendDesc::kind (a value), not a compile-time #ifdef,
// so every one of those call expressions is compiled into simulation.cpp's
// object code regardless of SPADE_VULKAN, and the linker must resolve every
// one of them for ANY executable that links spade::sim. compute/vulkan/
// backend.cpp -- the REAL implementation -- is compiled into spade_compute,
// which engine/CMakeLists.txt's SPADE_VULKAN gate makes NOT EXIST AT ALL
// when that option is OFF. Without this file, a SPADE_VULKAN=OFF build of
// spade_sim would compile cleanly (compute/vulkan/backend.hpp is Vulkan-free
// and physically present in the source tree either way) and then fail to
// LINK any executable that links spade::sim -- exactly the regression this
// file closes.
//
// engine/CMakeLists.txt compiles this file into spade_sim ONLY in the
// SPADE_VULKAN=OFF branch (the mirror image of the ON branch's
// target_sources(spade_compute ... compute/vulkan/backend.cpp ...)), so this
// file and the real backend.cpp are NEVER compiled into the same program --
// there is exactly one definition of each VulkanBackend method in any given
// build, satisfying the ODR without needing this file to be conditionally
// excluded by a preprocessor guard of its own.
//
// create() ALWAYS RETURNS Code::unavailable, which is not a special case
// invented for this file: it is the EXACT code
// compute::VulkanContext::create() already reports when no Vulkan loader or
// physical device is present (core/error.hpp's Code::unavailable doc
// comment: "a requested resource is architecturally absent from this
// process/environment"). From a caller's perspective "Vulkan support was
// not compiled into this build" and "a Vulkan loader is not installed" are
// the SAME fact -- the vulkan path is not available in this process, full
// stop -- so this file reports the identical code a caller already has to
// handle for the ON-but-no-device case, rather than inventing a second one.
//
// EVERY OTHER METHOD IS UNREACHABLE AT RUNTIME (create() always fails
// before any VulkanBackend is ever constructed) but still needs a
// DEFINITION here: simulation.cpp's compiled text calls all of them
// (guarded by `if (vulkan_backend_)` at runtime, which is always false in
// this build since create() never succeeds -- but the CALL EXPRESSIONS
// themselves are compiled regardless of that runtime guard), so the linker
// needs a symbol for each one to resolve against, reachable or not.
//
// Impl IS DEFINED HERE, EMPTY -- not merely forward-declared the way
// backend.hpp itself leaves it. unique_ptr<Impl>'s DESTRUCTOR (which
// VulkanBackend's own destructor, move constructor and move-assignment
// operator -- all `= default` below -- each implicitly generate a call to)
// requires Impl to be COMPLETE at the point it is COMPILED, regardless of
// whether the pointer it destroys is ever non-null at runtime: `delete ptr`
// needs sizeof(Impl) to be known even inside the `if (ptr)` branch that a
// null pointer never actually takes. This Impl is entirely distinct from
// backend.cpp's real one (different fields, different file, and -- per the
// note above -- never compiled into the same program as it), so there is no
// collision to reconcile.
// ---------------------------------------------------------------------------

namespace spade::compute {

struct VulkanBackend::Impl {};

VulkanBackend::VulkanBackend() = default;
VulkanBackend::~VulkanBackend() = default;
VulkanBackend::VulkanBackend(VulkanBackend&&) noexcept = default;
VulkanBackend& VulkanBackend::operator=(VulkanBackend&&) noexcept = default;

Result<std::unique_ptr<VulkanBackend>> VulkanBackend::create(const BackendDesc&, const StepShape&,
                                                             std::span<const GpuPass>) {
    return std::unexpected(
        Error{Code::unavailable,
              "the vulkan compute backend was not compiled into this build (SPADE_VULKAN=OFF)"});
}

Result<void> VulkanBackend::upload(const ArenaSet&) {
    return std::unexpected(Error{Code::unavailable, "VulkanBackend::upload: unreachable (SPADE_VULKAN=OFF)"});
}

Result<void> VulkanBackend::upload_dryden_params(std::span<const std::byte>, uint32_t, uint32_t) {
    return std::unexpected(
        Error{Code::unavailable, "VulkanBackend::upload_dryden_params: unreachable (SPADE_VULKAN=OFF)"});
}

Result<void> VulkanBackend::upload_sdf_program(std::span<const std::byte>, std::span<const std::byte>,
                                                std::span<const std::byte>) {
    return std::unexpected(
        Error{Code::unavailable, "VulkanBackend::upload_sdf_program: unreachable (SPADE_VULKAN=OFF)"});
}

Result<void> VulkanBackend::upload_contact_params(std::span<const std::byte>, uint32_t, uint32_t) {
    return std::unexpected(
        Error{Code::unavailable, "VulkanBackend::upload_contact_params: unreachable (SPADE_VULKAN=OFF)"});
}

Result<void> VulkanBackend::upload_grid_params(std::span<const std::byte>, uint32_t, uint32_t) {
    return std::unexpected(
        Error{Code::unavailable, "VulkanBackend::upload_grid_params: unreachable (SPADE_VULKAN=OFF)"});
}

Result<void> VulkanBackend::read_step_witness(std::span<std::byte>) {
    return std::unexpected(
        Error{Code::unavailable, "VulkanBackend::read_step_witness: unreachable (SPADE_VULKAN=OFF)"});
}

Result<void> VulkanBackend::read_grid_entries(std::span<std::byte>) {
    return std::unexpected(
        Error{Code::unavailable, "VulkanBackend::read_grid_entries: unreachable (SPADE_VULKAN=OFF)"});
}

// 0 rather than an error: this is a plain accessor with no Result to carry a
// diagnosis, and a zero-byte key array is the honest answer for a build with
// no device at all.
std::size_t VulkanBackend::grid_entries_byte_size() const noexcept { return 0; }

Result<void> VulkanBackend::read_field_samples(std::span<std::byte>) {
    return std::unexpected(
        Error{Code::unavailable, "VulkanBackend::read_field_samples: unreachable (SPADE_VULKAN=OFF)"});
}

std::size_t VulkanBackend::field_samples_byte_size() const noexcept { return 0; }

Result<void> VulkanBackend::step(uint64_t, uint64_t) {
    return std::unexpected(Error{Code::unavailable, "VulkanBackend::step: unreachable (SPADE_VULKAN=OFF)"});
}

Result<void> VulkanBackend::readback(ArenaSet&) {
    return std::unexpected(Error{Code::unavailable, "VulkanBackend::readback: unreachable (SPADE_VULKAN=OFF)"});
}

uint64_t VulkanBackend::upload_count() const noexcept { return 0; }

uint64_t VulkanBackend::last_written_tick() const noexcept { return 0; }

// S6 Task 10: same "compiled regardless of the runtime guard" reasoning as
// every Result-returning stub above -- Simulation::vulkan_pass_durations_ns()
// calls this unconditionally inside its own `if (vulkan_backend_)` guard,
// which is always false in this build (create() never succeeds), but the
// call expression itself still needs a symbol to link against.
Result<PassDurationsNs> VulkanBackend::read_pass_durations_ns() const {
    return std::unexpected(
        Error{Code::unavailable, "VulkanBackend::read_pass_durations_ns: unreachable (SPADE_VULKAN=OFF)"});
}

RecordedChain VulkanBackend::recorded_chain() const noexcept { return {}; }

uint32_t VulkanBackend::bound_binding_count() const noexcept { return 0; }

}  // namespace spade::compute
