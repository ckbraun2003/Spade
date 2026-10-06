#pragma once

// Backend descriptors ONLY -- deliberately the one file in engine/compute/
// with zero Vulkan dependency (S6 dependency rule, spec section 2: "nothing
// outside engine/compute/ includes a Vulkan header"; this header lets a
// consumer name/configure a backend without needing Vulkan on its include
// path at all). vulkan/context.hpp is the file that actually touches Vulkan.

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace spade::compute {

// Which compute backend a caller targets. `cpu` names today's existing
// engine path (spade_physics/spade_sim's host-side passes) as a first-class
// value so BackendDesc can select either uniformly; `vulkan` is what
// engine/compute/vulkan/ implements starting this task.
enum class BackendKind : uint32_t {
    cpu = 0,
    vulkan = 1,
};

// ---------------------------------------------------------------------------
// THE LEGAL VALUES OF BackendDesc::workgroup_size (S6 Task 9b), stated HERE
// because this is the knob's point of definition and the allowed set is part
// of what the knob MEANS -- not a private detail of whichever file happens to
// validate it.
//
// IT IS A CLOSED SET, AND THAT IS A BUILD FACT RATHER THAN A PREFERENCE. A
// kernel's local size is compiled into its SPIR-V (`OpExecutionMode <entry>
// LocalSize N 1 1`), so the only sizes a backend can offer are the sizes the
// BUILD compiled a module for: cmake/SpadeSlang.cmake's
// spade_slang_kernel_variants() compiles each compute kernel (twelve today)
// once per entry in this array, and compute/spirv_variants.hpp is the table
// that results. Adding a size here without adding it there would produce a
// value that validates and then fails at pipeline creation, so the two lists
// are pinned to each other by tests/test_gpu_invariance.cpp's
// WorkgroupSizeContract.AllowedSetMatchesCompiledVariants.
//
// WHY THESE THREE. 64 is the default and the size every kernel was authored,
// measured and parity-banded at through Tasks 6-8; 32 and 128 bracket it by a
// factor of two either way, which is what makes the A7 sweep a real test of
// the "no workgroup-shared reduction whose result order depends on local size"
// kernel design constraint rather than a re-run of one configuration. All
// three are within every Vulkan implementation's guaranteed minimum
// maxComputeWorkGroupInvocations (128), so the set costs no portability --
// though compute/vulkan/step_recorder.cpp checks the actual device limit
// anyway rather than trusting that guarantee.
inline constexpr std::array<uint32_t, 3> kSupportedWorkgroupSizes = {32u, 64u, 128u};

[[nodiscard]] inline constexpr bool workgroup_size_supported(uint32_t workgroup_size) noexcept {
    for (const uint32_t supported : kSupportedWorkgroupSizes) {
        if (supported == workgroup_size) return true;
    }
    return false;
}

// The allowed set as message text, e.g. "workgroup sizes {32, 64, 128}". Lives
// beside the set itself so that every rejection message names the SAME list
// this header defines -- a hand-written "{32, 64, 128}" in an error string is a
// second copy that goes stale the moment the array grows, and the whole point
// of a named error is that it tells the caller what to pass instead.
[[nodiscard]] inline std::string supported_workgroup_sizes_text() {
    std::string text = "workgroup sizes {";
    for (std::size_t i = 0; i < kSupportedWorkgroupSizes.size(); ++i) {
        if (i != 0) text += ", ";
        text += std::to_string(kSupportedWorkgroupSizes[i]);
    }
    text += "}";
    return text;
}

// Per-run backend configuration. Plain aggregate (no user-declared
// constructors), matching every authored *Desc type in state/layout.hpp.
//
// workgroup_size and device_index are both A7 knobs (global constraint:
// every backend knob owes a bit-invariance test -- same device, knob
// varied, GPU results must stay bit-identical, and BackendKind::cpu must
// reproduce today's CPU digests exactly).
//
// BOTH ARE LIVE AS OF S6 TASK 9b. device_index has been read by
// VulkanContext::create() since Task 5. workgroup_size was INERT through Task
// 9 -- declared here, plumbed through Simulation, and consumed by nothing,
// because all nine kernels baked `[numthreads(64,1,1)]` at slangc time -- which
// made T9's A7 sweep a test of the plumbing rather than of the kernels. Task 9b
// closed that: the nine kernels spell `[numthreads(SPADE_WG,1,1)]`, the build
// compiles each at every size in kSupportedWorkgroupSizes above,
// VulkanBackend::create() rejects anything outside that set, and
// compute/vulkan/step_recorder.cpp both SELECTS the matching compiled module
// and divides its dispatch grids by the SAME value. A value outside the set is
// a Code::invalid_argument naming the set, never a silent fallback to 64.
// ⚠⚠ THE DEFAULT IS `cpu` AND THAT IS NOT A PLACEHOLDER -- IT IS THE FASTER
// BACKEND IN EVERY CONFIGURATION ANYONE HAS MEASURED. Recorded here, beside
// the field, because this is where a reader decides what to pass.
//
// Measured 2026-09-18, spade_bench, one box (Intel Iris Plus, driver
// 3.3.0-31.0.101.2125), wall-clock per step, no render involved:
//
//   1 world x N bodies      CPU          vulkan        vulkan slower by
//     10                    0.0141 ms     1.424 ms        101x
//    100                    0.2297 ms    30.09   ms       131x
//   1000                    3.834  ms   284.9    ms        74x
//
//   N worlds x quad scene   CPU          vulkan        vulkan slower by
//     1                     0.0071 ms    15.14 ms        2139x
//     4                     0.0276 ms    14.30 ms         519x
//    16                     0.1139 ms    18.61 ms         163x
//    64                     0.6169 ms    15.48 ms          25x
//
// ⛔ THERE IS NO CROSSOVER. Do not read the 64-world row as "it wins at
// scale" -- it is still 25x behind, and the trend is a FIXED ~15 ms/step
// floor rather than a slope that would eventually cross. A threshold implying
// a crossover would be inventing a number nobody has found.
//
// TWO INDEPENDENT COSTS, and which dominates depends on the scene:
//   * collision_dynamic maps ONE THREAD PER WORLD (kernels/
//     collision_dynamic.slang, `const uint world = tid.x;`), so a
//     single-world scene runs its whole sweep on ONE LANE. Its own header
//     states this and calls the cost "REAL AND REPORTED RATHER THAN HIDDEN".
//     Dominates single-world-many-body.
//   * A ~14 ms/step fixed overhead, neither CPU work nor GPU kernel time --
//     the GPU step is flat from 1 to 64 worlds while its kernels sum to under
//     1 ms. Dominates everything else.
//
// ⭐ THE BACKEND IS BUILT FOR MANY WORLDS WITH FEW BODIES, AND EVERY SURFACE
// THIS PRODUCT ACTUALLY RUNS IS ONE WORLD WITH MANY BODIES. It is not broken
// and it was never hidden; nothing routinely ran it, so it degraded without
// anyone being wrong about it.
//
// ⚠ THIS IS DOCUMENTATION OF A MEASUREMENT, NOT A FIX. `vulkan` REMAINS FULLY
// SELECTABLE -- a backend that cannot be chosen on demand cannot be measured
// again, and whoever improves it needs to be able to run it.
// WHAT A VULKAN RUN USED (CORE-5; L6, announced and never silent). No kernel
// requests an fp32 denormal mode (SPIR-V rule P3), so every Vulkan 1.1 device
// is admitted and runs with its own denormal default, and its results are
// banded against the CPU (TD-14), not bit-identical. The context records the
// device, the driver and the float controls here, so every run can say which
// device and driver produced it. Plain data, no Vulkan types: a front end on
// the CPU backend can include it too.
struct DeviceReport {
    std::string device_name;
    uint32_t vendor_id = 0;
    uint32_t device_id = 0;
    uint32_t api_version = 0;     // VK_MAKE_API_VERSION-encoded
    uint32_t driver_version = 0;  // vendor-encoded; driver_info is its readable form
    // VkPhysicalDeviceDriverProperties and VkPhysicalDeviceFloatControlsProperties
    // are Vulkan 1.2 core; below it they are not queryable and stay empty / false.
    bool float_controls_queryable = false;
    std::string driver_name;
    std::string driver_info;
    bool denorm_preserve_f32 = false;
    bool denorm_flush_to_zero_f32 = false;
};

// One line for logs, gate output and front ends, e.g. "NVIDIA GeForce RTX 3060
// Ti, driver NVIDIA 572.83 (0x8F14C000), Vulkan 1.4.303; fp32 denormals: no
// mode requested, device default (preserve supported: no, flush-to-zero
// supported: no)".
[[nodiscard]] inline std::string describe(const DeviceReport& report) {
    static constexpr char kHex[] = "0123456789ABCDEF";
    std::string hex = "0x00000000";
    for (int i = 0; i < 8; ++i) hex[9 - i] = kHex[(report.driver_version >> (4 * i)) & 0xFu];
    const uint32_t v = report.api_version;
    std::string out = report.device_name + ", driver ";
    if (!report.driver_name.empty() || !report.driver_info.empty()) {
        out += report.driver_name;
        if (!report.driver_name.empty() && !report.driver_info.empty()) out += ' ';
        out += report.driver_info + " ";
    }
    out += "(" + hex + "), Vulkan " + std::to_string((v >> 22) & 0x7Fu) + "." +
           std::to_string((v >> 12) & 0x3FFu) + "." + std::to_string(v & 0xFFFu) +
           "; fp32 denormals: no mode requested, device default ";
    if (!report.float_controls_queryable) return out + "(float controls not queryable below Vulkan 1.2)";
    return out + "(preserve supported: " + (report.denorm_preserve_f32 ? "yes" : "no") +
           ", flush-to-zero supported: " + (report.denorm_flush_to_zero_f32 ? "yes" : "no") + ")";
}

struct BackendDesc {
    BackendKind kind = BackendKind::cpu;
    uint32_t workgroup_size = 64;
    uint32_t device_index = 0;
};

// ---------------------------------------------------------------------------
// StepShape (S6 Task 5) -- the fixed shape a VulkanBackend is created for:
// world/body/element/sensor capacities plus the two scalars (substeps, h) and
// the one config flag (batch_dynamic_collision) the schedule needs. Mirrors
// Simulation::create()'s VALIDATED WorldSetLayout (sim/world_set.hpp) plus the
// substep decomposition create() itself derives -- see sim/simulation.cpp's
// create(), which is the one place all seven values are already in scope
// together and is where a Simulation on the vulkan path builds one of these
// to hand to VulkanBackend::create().
//
// DELIBERATELY DOES NOT CARRY ContactParams/GridParams. Those are per-WORLD
// material/solver records (physics/contacts.hpp, physics/grid.hpp) that reach
// the device through the PER-WORLD `contact_params`/`grid_params` buffers
// (bindings.slang bindings 19/20), never through the shape a backend is
// created for. Folding them into StepShape would make two distinct
// Simulations that happen to share every capacity but disagree on
// restitution report DIFFERENT shapes for no shape-related reason.
//
// A backend's buffers, descriptor set and dispatch grid are sized from this
// struct ONCE, at VulkanBackend::create() time, and never resized -- exactly
// like ArenaSet's own fixed-capacity contract (state/arenas.hpp). Reshaping a
// running Simulation is out of scope for S6 (a fresh Simulation is the
// documented way to change shape today).
// ---------------------------------------------------------------------------
// The two SDF extents (S6 Task 6) ARE shape, unlike the material record below:
// they size two device buffers, exactly like body_capacity sizes the body
// mirror, and they are fixed for a Simulation's life because the per-world
// SdfProgram is configuration that never changes after create(). Both are
// TOTALS OVER THE WHOLE SET (every world's nodes concatenated), because that is
// what the buffers hold -- see compute/sdf_program.hpp for the flattening.
//
// THE REGISTERED WALK IS SHAPE TOO (module-API stage 4). `arrays` holds one
// StateArrayShape per registry walk entry -- each array and then its
// `.slot_to_world` map, in walk order -- and the state mirror makes exactly one
// device buffer per element. compute/ holds no list of array names or sizes:
// Simulation::create() fills this from the module set's declarations
// (sim/simulation.hpp's state_array_shapes()), so a module's array reaches the
// device without a change here. Every size is checked again against the live
// registry on every upload and readback.
struct StateArrayShape {
    std::string name{};
    uint32_t elem_size = 0;
    uint32_t capacity_per_world = 0;
};

// A DECLARED SCRATCH is shape too (module-API stage 4, Task 7b; sim/module.hpp's
// ScratchDecl): rows a module's passes share within a substep, which are not
// registered state. `scratch` holds one ScratchShape per declared scratch, as
// data, so compute/ keeps no list of them either. The state mirror makes a
// derived buffer for each one that names a binding -- world_count *
// capacity_per_world rows of elem_size bytes, zero-filled at create() and
// bound at `binding` -- and none for one with kNoBinding, which is CPU-only.
// A scratch's buffer is never uploaded or read back.
inline constexpr uint32_t kNoBinding = 0xFFFF'FFFFu;
struct ScratchShape {
    std::string name{};
    uint32_t elem_size = 0;           // the declared row size: the CPU rows' and the buffer's stride
    uint32_t capacity_per_world = 0;  // per_body: the body capacity; per_world: 1
    uint32_t binding = kNoBinding;    // a bindings.slang binding, or kNoBinding: no device buffer
};

struct StepShape {
    uint32_t world_count = 0;
    uint32_t body_capacity = 0;
    uint32_t element_capacity = 0;
    uint32_t sensor_capacity = 0;
    uint32_t substeps = 0;
    float h = 0.0f;
    bool batch_dynamic_collision = false;
    uint32_t sdf_node_count = 0;       // total SDF nodes across every world
    uint32_t sdf_transform_count = 0;  // total SDF transforms across every world
    std::vector<StateArrayShape> arrays{};  // one per walk entry, maps included, in walk order
    std::vector<ScratchShape> scratch{};    // every declared scratch, in the schedule's order (Task 7b)
};

// A built-in kernel the step recorder knows how to dispatch: a module pass
// names one to say "this is what my CPU function does, on the GPU". The pass
// may only name the recipe of its own CPU function (sim/module.hpp's
// builtin_cpu_for), so the GPU never runs a different experiment from the CPU.
// The two behavior recipes dispatch nothing: with no registry attached the
// behavior passes do nothing on either backend, and with one attached the
// Vulkan step refuses (CORE-1).
enum class GpuRecipe : uint8_t {
    none = 0,  // no GPU kernel: the pass runs only on the CPU
    behaviors_kinematic,
    medium_update,
    rotors,
    drag,
    behaviors_force,
    collision_static,
    collision_dynamic,
    integrate,
    sensor_imu,
    sensor_gnss,
    environment_sample,  // the field providers (module-API stage 3): each
    dryden_sample,       // writes every world's field sample row
};

// One pass of the GPU chain, in schedule order: "<module>.<pass>" and its
// recipe. sim/ derives the list from the compiled schedule; compute/ records it.
struct GpuPass {
    std::string name{};
    GpuRecipe recipe = GpuRecipe::none;
};

// ---------------------------------------------------------------------------
// PassDurationsNs (S6 Task 10; named per pass since module-API stage 2) --
// per-pass GPU timing, one entry per pass of the recorded GPU chain, in
// schedule order, named "<module>.<pass>" (compute/vulkan/timestamps.hpp).
// Nanoseconds, summed across every substep of the most recently completed
// step.
//
// WHY THIS LIVES HERE AND NOT IN compute/step_params.hpp, THE OTHER
// Vulkan-free header sim/simulation.hpp already includes. StepParams/
// StepWitness there are CPU<->GPU WIRE CONTRACTS -- mirrored verbatim in
// engine/shaders/shared/layouts.slang, checked by the generated
// layout_check.gen.hpp static_asserts, bytes that genuinely cross the device
// boundary. PassDurationsNs crosses no such boundary: it is a HOST-SIDE
// reduction of raw timestamp query ticks (compute/vulkan/timestamps.cpp's
// read_durations_ns()), never seen by a kernel, never mirrored in Slang. It
// belongs beside StepShape instead -- the general Vulkan-free "value that
// crosses the compute::/sim:: seam by value" home compute/backend.hpp's own
// file header already claims for BackendDesc/StepShape, and the SAME
// Vulkan-free-header trick vulkan_step_witness()/vulkan_grid_entries()
// (sim/simulation.hpp) already use to return a Vulkan-adjacent result from a
// header with zero Vulkan dependency of its own.
//
// `supported == false` means the device/queue could not time compute work at
// all (compute/vulkan/timestamps.hpp's skip-gracefully posture) -- `passes`
// is then EMPTY, never a list of zeros that could be read as "measured zero".
// The two behavior passes dispatch nothing, so their entries report a
// genuinely MEASURED ~0 ns whenever `supported` is true.
//
// LOOK A PASS UP WITH find(), WHICH HAS NO DEFAULT. A pass that is not in the
// list (a renamed module, say) and a pass that took no time must not look
// alike (TD-5), so there is deliberately no lookup that returns 0 for a
// missing name.
// ---------------------------------------------------------------------------
struct PassDuration {
    std::string pass{};  // "<module>.<pass>"
    double ns = 0.0;
};

struct PassDurationsNs {
    bool supported = false;
    std::vector<PassDuration> passes{};  // schedule order; empty when !supported

    [[nodiscard]] std::optional<double> find(std::string_view pass) const noexcept {
        for (const PassDuration& p : passes) {
            if (p.pass == pass) return p.ns;
        }
        return std::nullopt;
    }

    // ⛔ THERE IS NO host_fence_wait_ns FIELD HERE, AND THE ABSENCE IS A
    // RULING. L307 (2) added one -- a host-side wall-clock timer around vkWaitForFences in
    // StepRecorder::submit() -- to size the host stall the params ring
    // removes. M1B.FixedStepNoWallClockSymbolsInEngineSource went red:
    // engine source may not name a wall clock at all, because it carries a
    // fixed-step determinism charter.
    //
    // The guard is a SYMBOL scan. It cannot tell a diagnostic counter from
    // a timestep input, and THAT IS THE POINT rather than a limitation --
    // the next such timer in here might not be diagnostic. The instrument was
    // removed; the guard was not weakened.
    //
    // ⚠ AND IT MEASURED THE WRONG THING ANYWAY, WHICH IS THE PART WORTH
    // KEEPING: the host blocks in that fence WHILE THE GPU WORKS, so the
    // wait is 93.8% / 99.0% / 99.9% of the step at 10 / 100 / 1000 bodies
    // and is dominated by useful execution, not stall. At 1000 bodies it
    // came out BELOW the GPU's own pass total -- i.e. no host bubble to
    // recover at all. Anyone re-reaching for this: the quantity the ring
    // removes is the GAP BETWEEN STEPS on the DEVICE timeline, not the host
    // wait, and measuring it needs the per-slot query pool the ring itself
    // introduces.

    // L307 (2): how many individual pass samples in this readback exceeded
    // kImplausibleSampleNs. NON-ZERO MEANS THE NUMBERS ABOVE ARE NOT A
    // MEASUREMENT, and a caller that prints a duration without checking this
    // is printing whatever the driver left in the undefined bits.
    //
    // It exists because masking by timestampValidBits removes the old
    // `end > start` guard, which used to discard unexplainable samples in
    // silence. Discarding them is what let a 119.8 ms collision_dynamic sit
    // beside a 69.3 ms wall-clock step for a session without anybody being
    // able to see that the instrument, not the kernel, was the problem.
    // An instrument that cannot report its own failure produces a confident
    // number, and a confident number gets acted on.
    uint32_t implausible_samples = 0;
};

// What the Vulkan step recorder put into its one recorded command buffer, as
// counted while recording it. A diagnostic, like PassDurationsNs: tests read
// it to check that every adjacent pair of dispatches has a barrier between
// them, which a parity run on a forgiving driver cannot see, and that the
// chain follows the compiled schedule.
struct RecordedChain {
    uint32_t dispatches = 0;   // vkCmdDispatch calls, across every substep
    uint32_t barriers = 0;     // vkCmdPipelineBarrier calls between them
    uint32_t sort_stages = 0;  // bitonic stages per collision_dynamic recipe
    std::vector<std::string> passes{};  // each GPU pass once, "<module>.<pass>", as record() walked them
};

// ---------------------------------------------------------------------------
// RunParams -- REMOVED (S6 Task 6b, checkpoint-1 heterogeneous-worlds
// ruling). Task 6 introduced this struct to carry world 0's ContactParams/
// GridParams into PassParams's push constant, on the theory that a future
// batched CollisionDynamic kernel would read them as one per-dispatch
// material for the whole batch. Task 6b's PassParams decision tree grepped
// every kernel and step_recorder.cpp and found no reader anywhere -- the
// push-constant fields were write-only, world 0's values copied in and never
// read back out -- so both the PassParams fields (layouts.slang,
// compute/vulkan/step_recorder.hpp) and this struct, their sole source, are
// removed together: a carrier with nothing left to carry is not a smaller
// version of the mechanism, it is the same dead weight one layer up. T7's
// CollisionDynamic kernel will define its own per-batch (or, per this task's
// own ruling, more likely per-world) consumption path when it lands, not
// revive this one.
// ---------------------------------------------------------------------------
// The validation-message error sink (S6 Task 5, spec section 11's fault-path
// requirement). ONE global callback, registered by a caller (today: the gpu
// test fixture, tests/test_gpu_state_mirror.cpp) and invoked by
// compute/vulkan/context.cpp's debug messenger whenever the
// VK_LAYER_KHRONOS_validation layer reports a message on a debug build where
// the layer is present -- "a validation message during any GPU test is a
// test FAILURE" (this task's brief), which the fixture enforces by wiring the
// sink to ADD_FAILURE().
//
// DELIBERATELY VULKAN-FREE, like the rest of this file: the signature
// (std::string_view in, nothing Vulkan-shaped) is generic enough that any
// future backend could route its own diagnostics through the same seam, and
// a caller that only wants to register a sink never needs a Vulkan header on
// its include path to do it.
//
// HEADER-ONLY, deliberately, matching backend.hpp's own standing note that
// it is "the one file in engine/compute/ with zero Vulkan dependency" --
// adding a .cpp here just to host two tiny functions and one atomic would
// break that property for no benefit; an inline variable (C++17) is exactly
// as safe and needs no new translation unit.
//
// THREAD SAFETY: the store/load pair is a plain relaxed atomic, not a full
// memory-ordering protocol -- the engine is externally synchronized to one
// caller thread (every class in this tree says so), but a validation
// callback can fire from a driver-internal thread the caller does not
// control, so the pointer itself must not be torn or racily read; relaxed
// ordering is sufficient because the payload is a single pointer-sized value
// with no associated data the reader needs synchronized against it.
// ---------------------------------------------------------------------------
using ErrorSink = void (*)(std::string_view) noexcept;

namespace detail {
inline std::atomic<ErrorSink> g_error_sink{nullptr};
}  // namespace detail

// Registers `sink` as the current error sink; pass nullptr to clear it (the
// fixture's TearDown does this, so a later, unrelated test never inherits a
// prior test's ADD_FAILURE()-wired callback).
inline void set_error_sink(ErrorSink sink) noexcept {
    detail::g_error_sink.store(sink, std::memory_order_relaxed);
}

// Invoked by compute/vulkan/context.cpp's debug messenger trampoline (and by
// any future fault path that wants to report through the same seam) with the
// message text. A no-op when no sink is registered -- the common case on a
// release build or a test that never called set_error_sink().
inline void invoke_error_sink(std::string_view message) noexcept {
    if (const ErrorSink sink = detail::g_error_sink.load(std::memory_order_relaxed); sink != nullptr) {
        sink(message);
    }
}

}  // namespace spade::compute
