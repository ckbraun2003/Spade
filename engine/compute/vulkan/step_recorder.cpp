#include "compute/vulkan/step_recorder.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstring>
#include <iterator>
#include <string>
#include <utility>
#include <vector>

#include "bindings.gen.hpp"
#include "collision_dynamic.spv.gen.hpp"
#include "collision_static.spv.gen.hpp"
#include "forces_drag.spv.gen.hpp"
#include "grid_build.spv.gen.hpp"
#include "grid_sort.spv.gen.hpp"
#include "integrate.spv.gen.hpp"
#include "medium_update.spv.gen.hpp"
#include "rotors.spv.gen.hpp"
#include "sensor_gnss.spv.gen.hpp"
#include "sensor_imu.spv.gen.hpp"

namespace spade::compute {

// See state_mirror.cpp's identical comment: no `namespace gen = ...` alias
// here -- this file is already inside namespace spade::compute, where `gen`
// is already reachable as bindings.gen.hpp's nested namespace.

namespace {

// Same taxonomy as state_mirror.cpp's map_vk_error -- duplicated rather than
// shared through a third header because it is four lines and every caller
// already has <core/error.hpp> and <volk.h> in scope; a shared helper header
// for this alone would be one more file to open to see four lines of code.
[[nodiscard]] Error map_vk_error(VkResult result, std::string_view op) {
    switch (result) {
        case VK_ERROR_DEVICE_LOST:
            return Error{Code::internal, "Vulkan device lost during " + std::string(op)};
        case VK_ERROR_OUT_OF_DEVICE_MEMORY:
        case VK_ERROR_OUT_OF_HOST_MEMORY:
            return Error{Code::capacity_exceeded,
                         "Vulkan allocation failed (VkResult " + std::to_string(static_cast<int>(result)) +
                             ") during " + std::string(op)};
        default:
            return Error{Code::internal, "Vulkan call failed (VkResult " +
                                              std::to_string(static_cast<int>(result)) + ") during " +
                                              std::string(op)};
    }
}

// ---------------------------------------------------------------------------
// THE DIVISOR IS THE LIVE A7 KNOB AS OF S6 TASK 9b -- it used to be a
// `constexpr uint32_t kWorkgroupSize = 64` here, matching the literal
// `[numthreads(64,1,1)]` all nine kernels then spelled, with a comment
// explaining that BackendDesc::workgroup_size could not be read because a
// kernel's local size is baked into its compiled SPIR-V.
//
// It still is baked in -- that fact did not change, the BUILD did.
// cmake/SpadeSlang.cmake now compiles every one of these nine kernels once per
// size in compute/backend.hpp's kSupportedWorkgroupSizes, so there IS a module
// compiled for whatever the caller asked for, and create() below selects it.
// `workgroup_size_` is therefore the single value that both chose the pipelines
// and divides the grids: a kernel dispatched on a grid divided by a number
// other than its own local size is a Vulkan valid-usage violation whose symptom
// is wrong physics, not an error, so the two halves must come from one field
// and they do.
// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------
// FOUR DISPATCH EXTENTS, AND EACH IS THE ARRAY ITS KERNELS OWN A ROW OF.
//
//   PER BODY SLOT (world_count * body_capacity) -- rotors, forces_drag,
//     collision_static, integrate. The two force kernels dispatch one thread
//     per BODY and fold that body's ELEMENTS serially rather than dispatching
//     per element, for the race/determinism/workgroup-invariance reasons their
//     own headers set out at length.
//   PER KEY-ARRAY ENTRY (S6 Task 7) -- grid_build and grid_sort, over the
//     power-of-two-padded domain compute/grid_entry.hpp sizes.
//   PER WORLD -- collision_dynamic (its whole sweep is sequential within a
//     world) and, as of S6 Task 8, medium_update (the Dryden filter is a
//     sequential recurrence within a world). Both kernels' headers carry the
//     same shape of parity argument; they SHARE this grid because they happen
//     to have the same extent, not because they are related.
//   PER SENSOR SLOT (S6 Task 8, world_count * sensor_capacity) -- sensor_imu.
//     A world set may size its sensor capacity independently of its body
//     capacity, so this is genuinely a fourth extent and not the body grid
//     under another name.
//
// Hence groups_for() below rather than one hardcoded grid.
// ---------------------------------------------------------------------------
[[nodiscard]] uint32_t groups_for(uint64_t threads, uint32_t workgroup_size) {
    return static_cast<uint32_t>((threads + workgroup_size - 1) / workgroup_size);
}

[[nodiscard]] uint32_t dispatch_groups_for(const StepShape& shape, uint32_t workgroup_size) {
    return groups_for(static_cast<uint64_t>(shape.world_count) * static_cast<uint64_t>(shape.body_capacity),
                      workgroup_size);
}

[[nodiscard]] uint32_t sensor_groups_for(const StepShape& shape, uint32_t workgroup_size) {
    return groups_for(static_cast<uint64_t>(shape.world_count) *
                          static_cast<uint64_t>(shape.sensor_capacity),
                      workgroup_size);
}

// ---------------------------------------------------------------------------
// Can this device actually run a local size of `workgroup_size` (S6 Task 9b)?
//
// CHECKED IN CODE RATHER THAN LEFT TO THE DRIVER, for exactly the reason
// context.cpp checks shaderDenormPreserveFloat32 there and says so at length:
// exceeding maxComputeWorkGroupInvocations or maxComputeWorkGroupSize[0] is a
// Vulkan VALID-USAGE violation, i.e. undefined behaviour, NOT a guaranteed
// VK_ERROR_* from vkCreateComputePipelines -- and this project enables
// validation layers only in Debug and only if present. Both limits have a
// guaranteed minimum of 128 in every Vulkan implementation, so
// kSupportedWorkgroupSizes cannot exceed them on a conformant device; this
// check is what turns "cannot" into a named error instead of a trust
// assumption.
[[nodiscard]] bool device_supports_local_size(VkPhysicalDevice physical_device, uint32_t workgroup_size,
                                              uint32_t& out_max_invocations, uint32_t& out_max_size_x) {
    VkPhysicalDeviceProperties properties{};
    vkGetPhysicalDeviceProperties(physical_device, &properties);
    out_max_invocations = properties.limits.maxComputeWorkGroupInvocations;
    out_max_size_x = properties.limits.maxComputeWorkGroupSize[0];
    return workgroup_size <= out_max_invocations && workgroup_size <= out_max_size_x;
}

// ---------------------------------------------------------------------------
// And can it dispatch that many GROUPS (S6 Task 9b review, minor M2)?
//
// THIS IS THE LIMIT THAT ACTUALLY SCALES WITH THIS TASK. The two limits above
// bound the local size, which this change moves between 32 and 128 -- both far
// inside every implementation's guaranteed minimum, so neither can realistically
// fire. maxComputeWorkGroupCount[0] bounds the OTHER side of the same division:
// group count is ceil(extent / local size), so HALVING the workgroup size
// DOUBLES it. A shape that dispatched N groups at the old fixed 64 dispatches 2N
// at 32.
//
// The guaranteed minimum is 65535 while compute/grid_entry.hpp's kMaxGridEntries
// ceiling is 2^26, so a large shape can exceed this -- which was ALREADY true at
// 64 and is a pre-existing exposure this task WIDENS BY 2x rather than
// introduces. Stated that way deliberately: the honest description is not "new
// hazard" but "existing hazard, one factor of two closer, and now checked".
//
// compute/vulkan/probe_runner.cpp:174 already had the house pattern for exactly
// this check on its own single dispatch; this is the same check for the recorded
// chain's four grids, and it belongs beside the local-size gate because the two
// are the two halves of one division.
//
// A HOST-SIDE TEST FOR THE REJECTING BRANCH, ASSESSED AND DEFERRED (S6
// hygiene; T9b review M2-followup). This function is a two-line query-then-
// compare, and in isolation would be trivial to test: read the box's OWN
// real maxComputeWorkGroupCount[0] once, then call this with group_count ==
// (that value + 1) to force `false` deterministically, on any device,
// without needing to actually construct a StepRecorder shape that reaches
// the limit. That is NOT constructible today without a code change, though,
// for one reason: this function is TU-private (the anonymous namespace
// opened above), so no test outside this file can call it directly, and the
// smallest-diff bar for a hygiene pass does not cover exporting a new
// internal-only entry point (this file has no precedent for one -- grepped).
// The alternative -- reaching this branch through StepRecorder::create()'s
// public API with a genuinely oversized shape -- is very likely UNREACHABLE
// on real hardware: kMaxGridEntries (2^26, compute/grid_entry.hpp) divided
// by the smallest supported workgroup_size (32) is the largest group_count
// this engine can ever ask for, ~2.1M, and Vulkan's GUARANTEED MINIMUM for
// maxComputeWorkGroupCount[0] is only 65535 (so this branch is not dead by
// the spec) but any real GPU driver, including this box's Iris, is expected
// to report far more (commonly UINT32_MAX-class) -- meaning a shape big
// enough to trip it through the public API would also very likely first
// trip kMaxGridEntries's own, unrelated ceiling. Flagged here rather than
// guessed at with an untested assumption about this box's actual reported
// limit.
//
// T11 FINAL TRIAGE (whole-plan review W7c): RESOLVED to (b) -- this analysis
// is ACCEPTED ON THE RECORD as the reason no test exists for the rejecting
// branch. No test-visibility seam was added. The two things that would have
// justified (a) instead -- a real path to constructing the rejecting branch
// through StepRecorder::create()'s public API, or a precedent elsewhere in
// this file for exporting a TU-private helper for tests -- are both absent
// (grepped, per the paragraph above), and this comment's own arithmetic
// already IS the artifact a reviewer would want from a test: it shows the
// branch is reachable per the Vulkan spec's guaranteed minimum but not
// through any shape this engine can actually construct.
[[nodiscard]] bool device_supports_group_count(VkPhysicalDevice physical_device, uint32_t group_count,
                                               uint32_t& out_max_group_count) {
    VkPhysicalDeviceProperties properties{};
    vkGetPhysicalDeviceProperties(physical_device, &properties);
    out_max_group_count = properties.limits.maxComputeWorkGroupCount[0];
    return group_count <= out_max_group_count;
}

// ---------------------------------------------------------------------------
// INTERIM (module-API plan, stage 1, 2026-10-02): physics/schedule.cpp's
// kSchedule is gone. The CPU runs the compiled module schedule
// (sim/module.hpp, sim/standard_modules.cpp), and this table mirrors the
// STANDARD set's compiled order, which
// Schedule.TheCompiledStandardSetFollowsTheGpuRecordersOrder pins. Stage 2
// derives the GPU chain from the schedule and deletes this table; until then,
// read "kSchedule" below as "the compiled standard set", and the CPU's empty
// Gravity and Publish passes as removed.
//
// Schedule slot -> what it records. THE ORDER IS physics/schedule.cpp's
// kSchedule, verbatim, and these two tables are the one place the
// correspondence is written down; step_recorder.hpp's header comment is their
// prose form.
//
// TWO SLOTS RECORD NOTHING, AND `kNoDispatch` SAYS SO BY NAME (S6 Task 8) --
// where through Task 7 they bound a stub pipeline that provably did nothing.
// Gravity and Publish are INERT BY DESIGN on both backends (schedule.cpp's own
// pass_gravity()/pass_publish() are empty functions; gravity is applied inside
// Integrate), so the honest recording for them is no commands at all. They stay
// in the table so the correspondence with spec section 3's eight names remains
// literal.
//
// TWO SLOTS ARE CHAINS, which a one-pipeline-per-slot table cannot express;
// record() special-cases both by index, and their entries here name the chain's
// FIRST pipeline so the table stays a faithful "what does this slot start with"
// and never silently reads as "this slot is one dispatch".
// ---------------------------------------------------------------------------
// ---------------------------------------------------------------------------
// THIS TABLE MODELS SPEC SECTION 3'S EIGHT PASSES, NOT THE CPU SCHEDULE'S TEN.
//
// physics/schedule.cpp's kSchedule carries TEN slots as of the 24th spec's SL6
// (Plan A Task 7): §3's eight, plus BehaviorsKinematic (before ForceElements)
// and BehaviorsForce (after it). Neither is represented here, and that is
// deliberate rather than an oversight -- SL6 refuses a behavior without a
// record_gpu half from a GPU-authoritative world outright, so what the GPU
// shape should be is the behavior registry's decision to make, not a pair of
// no-dispatch slots added ahead of it. Adding them now would also cost two
// PassDurationsNs fields and two timestamp query slots per substep to measure
// two things that record nothing.
//
// THE SLOT INDICES BELOW ARE THIS TABLE'S, THEREFORE, AND NO LONGER THE CPU
// SCHEDULE'S. In kSchedule, ForceElements is index 2 and CollisionDynamic is 5.
//
// WHAT KEEPS THE DIVERGENCE HONEST: test_determinism.cpp's
// Schedule.RemovingTheBehaviorSlotsLeavesSpecSectionThreeExactly, which asserts
// that striking the Behaviors* slots out of kSchedule leaves §3's eight names
// in §3's order. That is precisely the property this table assumes, and until
// it was written nothing checked it -- the recorder never reads kSchedule, so
// the schedule grew from eight to ten with every GPU test green.
// ---------------------------------------------------------------------------
constexpr uint32_t kNoDispatch = 0xFFFFFFFFu;

constexpr uint32_t kForceElementsSlot = 1;     // rotors -> forces_drag (S6 Task 8)
constexpr uint32_t kSensorSynthesisSlot = 6;   // sensor_imu -> sensor_gnss (GPU-sensor leg)
constexpr uint32_t kCollisionDynamicSlot = 4;  // grid_build -> grid_sort xS -> collision_dynamic

constexpr std::array<uint32_t, 8> kPassPipeline = {
    StepRecorder::kPipelineMedium,            // 0 MediumUpdate      -- per-WORLD grid
    StepRecorder::kPipelineRotors,            // 1 ForceElements     -- CHAIN; see kForceElementsSlot
    kNoDispatch,                              // 2 Gravity           -- inert by design
    StepRecorder::kPipelineCollision,         // 3 CollisionStatic
    StepRecorder::kPipelineGridBuild,         // 4 CollisionDynamic  -- CHAIN; see kCollisionDynamicSlot
    StepRecorder::kPipelineIntegrate,         // 5 Integrate
    StepRecorder::kPipelineSensorImu,         // 6 SensorSynthesis   -- CHAIN; see kSensorSynthesisSlot
    kNoDispatch,                              // 7 Publish           -- inert by design
};

// Which dispatch EXTENT each slot runs over, indexed identically. A second
// table beside the first, rather than a switch inside record(), because "which
// extent does this pass run over" is exactly the kind of per-slot fact that
// belongs next to "which kernel does it bind" -- and because Task 8 made it a
// genuine four-way choice rather than "the body grid, except slot 4".
enum class PassGrid : uint32_t { body, world, sensor, grid_entry, none };

constexpr std::array<PassGrid, 8> kPassGrid = {
    PassGrid::world,       // 0 MediumUpdate     -- one thread per world
    PassGrid::body,        // 1 ForceElements    -- BOTH chain members are per-body
    PassGrid::none,        // 2 Gravity
    PassGrid::body,        // 3 CollisionStatic
    PassGrid::grid_entry,  // 4 CollisionDynamic -- the chain's first two stages; the sweep is per-world
    PassGrid::body,        // 5 Integrate
    PassGrid::sensor,      // 6 SensorSynthesis  -- one thread per sensor slot
    PassGrid::none,        // 7 Publish
};

// log2 of a power of two, for the bitonic stage count. `segment` comes from
// grid_domain_of() and is a power of two >= 1 by construction.
[[nodiscard]] uint32_t log2_pow2(uint32_t segment) {
    uint32_t bits = 0;
    while ((1u << bits) < segment) ++bits;
    return bits;
}

}  // namespace

// ---------------------------------------------------------------------------
// create
// ---------------------------------------------------------------------------

Result<std::unique_ptr<StepRecorder>> StepRecorder::create(VulkanContext& ctx, const StepShape& shape,
                                                             VkDescriptorSetLayout set_layout,
                                                             VkDescriptorSet set, void* step_params_mapped,
                                                             uint32_t workgroup_size) {
    if (step_params_mapped == nullptr) {
        return std::unexpected(Error{Code::invalid_argument,
                                     "StepRecorder::create: step_params_mapped is null (StateMirror owns "
                                     "that buffer and must have mapped it before this call)"});
    }

    // The A7 knob, validated BEFORE anything is allocated (S6 Task 9b).
    // VulkanBackend::create() checks the same value against the same predicate
    // ahead of building a context at all -- this is the second, closer guard,
    // kept because StepRecorder is independently constructible and the selection
    // it protects (which compiled variant becomes each pipeline) happens here.
    //
    // IT IS DEFENCE IN DEPTH, AND IT IS UNTESTED -- said here so a reader does
    // not go hunting for the case that covers it (Task 9b review, minor M6).
    // Nothing reaches StepRecorder::create() except through VulkanBackend::
    // create(), which already rejected every unsupported size, so this branch is
    // UNREACHABLE on today's only path and the suite exercises the outer guard
    // instead (tests/test_gpu_invariance.cpp's
    // WorkgroupSizeContract.UnsupportedSizeIsRejectedByName). It stays because a
    // future direct constructor of this class would otherwise select a pipeline
    // variant that does not exist -- and make_pipeline() below would then be the
    // thing reporting it, one layer further from the caller's mistake.
    if (!workgroup_size_supported(workgroup_size)) {
        return std::unexpected(Error{Code::invalid_argument,
                                     "StepRecorder::create: workgroup_size " +
                                         std::to_string(workgroup_size) + " is not one of the compiled " +
                                         supported_workgroup_sizes_text()});
    }

    uint32_t max_invocations = 0;
    uint32_t max_size_x = 0;
    if (!device_supports_local_size(ctx.physical_device(), workgroup_size, max_invocations, max_size_x)) {
        return std::unexpected(Error{
            Code::unavailable,
            "physical device '" + std::string(ctx.device_name()) + "' cannot run a local size of " +
                std::to_string(workgroup_size) + " (maxComputeWorkGroupInvocations=" +
                std::to_string(max_invocations) + ", maxComputeWorkGroupSize[0]=" +
                std::to_string(max_size_x) +
                "). Requesting one anyway would be a Vulkan valid-usage violation, not a reported error."});
    }

    auto self = std::unique_ptr<StepRecorder>(new StepRecorder());
    self->device_ = ctx.device();
    self->queue_ = ctx.compute_queue();
    self->set_ = set;
    self->shape_ = shape;
    self->step_params_mapped_ = step_params_mapped;
    self->workgroup_size_ = workgroup_size;
    self->dispatch_groups_x_ = dispatch_groups_for(shape, workgroup_size);

    // S6 Task 7: the CollisionDynamic chain's own shape, decided ONCE here for
    // the same reason every other shape-derived value in this class is -- the
    // command buffer is recorded once and never re-recorded, so anything the
    // recording depends on has to be fixed before it runs. `ok == false` means
    // the shape asks for a key array this backend refuses to allocate; report
    // it here, at the one place the shape is turned into resources, rather
    // than letting the allocator discover it (compute/grid_entry.hpp).
    self->grid_ = grid_domain_of(shape);
    if (!self->grid_.ok) {
        return std::unexpected(Error{Code::capacity_exceeded,
                                     "StepRecorder::create: the CollisionDynamic key array for this shape "
                                     "would exceed compute/grid_entry.hpp's kMaxGridEntries ceiling"});
    }
    self->grid_groups_x_ = groups_for(self->grid_.entry_count, workgroup_size);
    self->world_groups_x_ = groups_for(shape.world_count, workgroup_size);
    self->sensor_groups_x_ = sensor_groups_for(shape, workgroup_size);

    // S6 Task 10: per-pass GPU timing, created unconditionally (see this
    // class's header comment on pass_durations_ns()). PassTimestamps::create()
    // itself never fails on a capability gap -- only a genuine Vulkan failure
    // allocating the (tiny) query pool on a device that DOES claim support
    // propagates here, exactly like every other resource this factory builds.
    Result<std::unique_ptr<PassTimestamps>> timestamps = PassTimestamps::create(ctx, shape.substeps);
    if (!timestamps) {
        return std::unexpected(timestamps.error());
    }
    self->timestamps_ = std::move(*timestamps);

    // All FOUR grids against maxComputeWorkGroupCount[0] -- checked here, after
    // every one is known, because the recording below emits dispatches on all of
    // them and vkCmdDispatch does not report an over-limit group count, it is
    // simply invalid usage. The widest one is the only one that can fail, so it
    // is the only one that needs naming.
    const uint32_t widest_grid = std::max({self->dispatch_groups_x_, self->grid_groups_x_,
                                           self->world_groups_x_, self->sensor_groups_x_});
    uint32_t max_group_count = 0;
    if (!device_supports_group_count(ctx.physical_device(), widest_grid, max_group_count)) {
        return std::unexpected(Error{
            Code::capacity_exceeded,
            "StepRecorder::create: this shape needs " + std::to_string(widest_grid) +
                " workgroups at local size " + std::to_string(workgroup_size) + ", but device '" +
                std::string(ctx.device_name()) + "' allows at most " + std::to_string(max_group_count) +
                " (maxComputeWorkGroupCount[0]). Note that group count is ceil(extent / local size), so a "
                "SMALLER workgroup_size needs MORE groups for the same shape -- 64 or 128 may fit where "
                "32 does not."});
    }

    // The pipeline LAYOUT first: all four pipelines share it (see the class
    // note in the header), so it is created once, before any of them.
    VkPushConstantRange push_range{};
    push_range.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
    push_range.offset = gen::kPushConstantOffset;
    push_range.size = gen::kPushConstantSize;

    VkPipelineLayoutCreateInfo layout_info{};
    layout_info.sType = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO;
    layout_info.setLayoutCount = 1;
    layout_info.pSetLayouts = &set_layout;
    layout_info.pushConstantRangeCount = 1;
    layout_info.pPushConstantRanges = &push_range;
    if (VkResult r = vkCreatePipelineLayout(self->device_, &layout_info, nullptr, &self->pipeline_layout_);
        r != VK_SUCCESS) {
        return std::unexpected(map_vk_error(r, "vkCreatePipelineLayout"));
    }

    // ONE VARIANT SET PER PIPELINE SLOT (S6 Task 9b). Each entry names the
    // kernel's whole compiled family -- all three local sizes -- and
    // make_pipeline() picks the one matching workgroup_size_. Written as a table
    // rather than as nine near-identical if-statements because the previous
    // shape had every call site repeating the same four-line error dance, and
    // because the slot->kernel correspondence is the fact worth being able to
    // read in one place (kPassPipeline above maps SCHEDULE slots onto these).
    const struct {
        PipelineSlot slot;
        const SpirvVariantSet& variants;
    } kPipelineSources[] = {
        {kPipelineDrag, gen::kSpvVariants_forces_drag},
        {kPipelineCollision, gen::kSpvVariants_collision_static},
        {kPipelineIntegrate, gen::kSpvVariants_integrate},
        {kPipelineGridBuild, gen::kSpvVariants_grid_build},
        {kPipelineGridSort, gen::kSpvVariants_grid_sort},
        {kPipelineCollisionDynamic, gen::kSpvVariants_collision_dynamic},
        // S6 Task 8's three, completing the schedule.
        {kPipelineMedium, gen::kSpvVariants_medium_update},
        {kPipelineRotors, gen::kSpvVariants_rotors},
        {kPipelineSensorImu, gen::kSpvVariants_sensor_imu},
        {kPipelineSensorGnss, gen::kSpvVariants_sensor_gnss},
    };
    static_assert(std::size(kPipelineSources) == kPipelineCount,
                  "every PipelineSlot needs exactly one kernel variant set");

    for (const auto& source : kPipelineSources) {
        if (Result<void> made = self->make_pipeline(source.variants, self->shaders_[source.slot],
                                                     self->pipelines_[source.slot]);
            !made) {
            return std::unexpected(made.error());
        }
    }

    VkCommandPoolCreateInfo pool_info{};
    pool_info.sType = VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO;
    // No RESET_COMMAND_BUFFER_BIT (S6 Task 5 review round 1, C1 fix): this
    // command buffer is recorded exactly once (record(), below) and never
    // reset for the life of this object, so the pool never needs to support
    // per-buffer reset -- omitting the flag is a correctness statement, not
    // merely an optimization: it is what would make an accidental future
    // vkResetCommandBuffer() call fail loudly (VUID-vkResetCommandBuffer-
    // commandBuffer-... requires the pool to have been created with the
    // flag) instead of silently reintroducing the per-step re-record this
    // fix removes.
    pool_info.queueFamilyIndex = ctx.compute_queue_family();
    if (VkResult r = vkCreateCommandPool(self->device_, &pool_info, nullptr, &self->pool_); r != VK_SUCCESS) {
        return std::unexpected(map_vk_error(r, "vkCreateCommandPool (StepRecorder)"));
    }

    VkCommandBufferAllocateInfo cmd_info{};
    cmd_info.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO;
    cmd_info.commandPool = self->pool_;
    cmd_info.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
    cmd_info.commandBufferCount = 1;
    if (VkResult r = vkAllocateCommandBuffers(self->device_, &cmd_info, &self->cmd_); r != VK_SUCCESS) {
        return std::unexpected(map_vk_error(r, "vkAllocateCommandBuffers (StepRecorder)"));
    }

    VkFenceCreateInfo fence_info{};
    fence_info.sType = VK_STRUCTURE_TYPE_FENCE_CREATE_INFO;
    if (VkResult r = vkCreateFence(self->device_, &fence_info, nullptr, &self->fence_); r != VK_SUCCESS) {
        return std::unexpected(map_vk_error(r, "vkCreateFence (StepRecorder)"));
    }

    // Records the fixed dispatch chain ONCE -- see record()'s own comment.
    if (Result<void> recorded = self->record(); !recorded) {
        return std::unexpected(recorded.error());
    }

    return self;
}

Result<void> StepRecorder::make_pipeline(const SpirvVariantSet& variants, VkShaderModule& out_module,
                                           VkPipeline& out_pipeline) {
    // THE SELECTION (S6 Task 9b). A null answer means the build did not compile
    // this kernel for the size create() accepted -- i.e.
    // cmake/SpadeSlang.cmake's SPADE_SLANG_WORKGROUP_SIZES and
    // compute/backend.hpp's kSupportedWorkgroupSizes have drifted apart. That is
    // a build-configuration defect rather than a caller error, and it must be a
    // loud one HERE, because the alternative -- falling back to some other
    // variant -- would dispatch a module on a grid divided by a different
    // number and produce wrong physics silently.
    const SpirvVariant* variant = variants.for_size(workgroup_size_);
    if (variant == nullptr) {
        return std::unexpected(Error{
            Code::invalid_argument,
            std::string("StepRecorder: kernel '") + variants.name + "' has no variant compiled for local size " +
                std::to_string(workgroup_size_) +
                ". cmake/SpadeSlang.cmake's SPADE_SLANG_WORKGROUP_SIZES must list every size "
                "compute/backend.hpp's kSupportedWorkgroupSizes admits."});
    }

    VkShaderModuleCreateInfo shader_info{};
    shader_info.sType = VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO;
    shader_info.codeSize = variant->word_count * sizeof(uint32_t);
    shader_info.pCode = variant->words;
    if (VkResult r = vkCreateShaderModule(device_, &shader_info, nullptr, &out_module); r != VK_SUCCESS) {
        return std::unexpected(map_vk_error(r, "vkCreateShaderModule (StepRecorder)"));
    }

    VkPipelineShaderStageCreateInfo stage_info{};
    stage_info.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
    stage_info.stage = VK_SHADER_STAGE_COMPUTE_BIT;
    stage_info.module = out_module;
    // Entry point name: "main", regardless of what the Slang function is
    // called. Verified against every embedded module's own OpEntryPoint
    // literal (tests/test_slang_layouts.cpp asserts it for each): slangc's
    // direct-to-SPIR-V compile names the sole compute entry point "main", so
    // vkCreateComputePipelines's pName must match THAT. Getting this wrong
    // once already produced a pipeline the driver could not run (S6 Task 5's
    // own SEH-crash regression, round 1).
    stage_info.pName = "main";

    VkComputePipelineCreateInfo pipeline_info{};
    pipeline_info.sType = VK_STRUCTURE_TYPE_COMPUTE_PIPELINE_CREATE_INFO;
    pipeline_info.stage = stage_info;
    pipeline_info.layout = pipeline_layout_;
    if (VkResult r =
            vkCreateComputePipelines(device_, VK_NULL_HANDLE, 1, &pipeline_info, nullptr, &out_pipeline);
        r != VK_SUCCESS) {
        return std::unexpected(map_vk_error(r, "vkCreateComputePipelines"));
    }
    return {};
}

// ---------------------------------------------------------------------------
// sort_stage_count -- how many bitonic (k, j) stages one CollisionDynamic pass
// records. The textbook network over a power-of-two domain of size S runs
// log2(S) outer merges whose inner loops have 1, 2, ... log2(S) stages, hence
// the triangular number. 0 for S == 1 (a single-entry segment is sorted).
//
// Stated as its own function rather than inlined into record() because
// record()'s barrier bookkeeping needs the TOTAL dispatch count BEFORE it
// emits the first one, and a second hand-derived copy of this arithmetic is
// exactly the sort of thing that goes wrong silently (one missing barrier at
// the end of the chain would be invisible until a driver reordered).
// ---------------------------------------------------------------------------
uint32_t StepRecorder::sort_stage_count() const noexcept {
    const uint32_t bits = log2_pow2(grid_.segment);
    return bits * (bits + 1u) / 2u;
}

// ---------------------------------------------------------------------------
// record -- ONCE, from create(). Never called again.
// ---------------------------------------------------------------------------

Result<void> StepRecorder::record() {
    VkCommandBufferBeginInfo begin{};
    begin.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
    // NOT ONE_TIME_SUBMIT: that flag tells the driver this buffer will be
    // submitted exactly once and then reset/freed, which is not true --
    // submit() below resubmits this SAME recording arbitrarily many times
    // without ever resetting it (S6 Task 5 review round 1, C1 fix).
    if (VkResult r = vkBeginCommandBuffer(cmd_, &begin); r != VK_SUCCESS) {
        return std::unexpected(map_vk_error(r, "vkBeginCommandBuffer (StepRecorder record)"));
    }

    // The descriptor set is bound ONCE for the whole chain: every pipeline
    // shares one layout, so a pipeline bind does not invalidate it.
    vkCmdBindDescriptorSets(cmd_, VK_PIPELINE_BIND_POINT_COMPUTE, pipeline_layout_, 0, 1, &set_, 0, nullptr);

    // S6 Task 10: reset every per-pass timing query slot ONCE, before the
    // first dispatch -- a no-op when timestamps_ is unsupported on this
    // device (PassTimestamps::record_reset()'s own doc comment).
    timestamps_->record_reset(cmd_);

    // The per-substep chain, `shape_.substeps` times, walking spec section 3's
    // eight passes slot by slot (MediumUpdate .. Publish) -- which is kSchedule
    // MINUS SL6's two behavior slots; see kNoDispatch's comment above. Each slot
    // records what kPassPipeline/kPassGrid name for it, EXCEPT the two CHAIN
    // slots (1 ForceElements and 4 CollisionDynamic) and the two INERT ones
    // (2 Gravity and 7 Publish), which record nothing at all.
    //
    // A memory barrier after every dispatch except the very last one of the
    // WHOLE recorded chain: that covers the inter-pass hazard within one
    // substep, the inter-STAGE hazard inside the sort chain, the rotors->drag
    // hazard inside slot 1 (both write the same two accumulators, in that
    // order), and the inter-substep hazard between one substep's last dispatch
    // and the next substep's MediumUpdate -- uniformly, rather than
    // special-casing any of them.
    //
    // EVERY PassParams FIELD HERE IS SHAPE OR DISPATCH DATA, NOT STEP DATA
    // (what makes recording this once, rather than per-step, correct):
    // world_count .. sensor_capacity/h and the three grid_* lanes come from
    // `shape_`/`grid_`, fixed for this object's life; `substep` cycles
    // 0..substeps-1 IDENTICALLY on every step (it is a property of WHICH
    // DISPATCH this is within the fixed chain, not of which step is executing);
    // and sort_k/sort_j are likewise a property of which STAGE a dispatch is.
    // The one thing that genuinely varies per submit, the tick, does not travel
    // through a push constant at all (compute/step_params.hpp). PassParams
    // carried `contact`/`grid` from `run_` through S6 Task 6; Task 6b's
    // PassParams decision tree removed both fields (no kernel ever read them)
    // along with `run_` itself -- see step_recorder.hpp's PassParams doc
    // comment for the ruling, and note that S6 Task 7's five new lanes are on
    // the right side of it (shape/dispatch, never material).
    //
    // BUILT AS A LIST, THEN RECORDED. The walk below appends each dispatch and
    // each timestamp mark to `commands` in order; the loop after it records
    // them, with a barrier after every dispatch except the last one in the
    // list. "Which dispatch is last" is therefore read off what was actually
    // emitted. Until 2026-10-01 it came from a hand tally of dispatches per
    // substep that went stale when SensorSynthesis became a two-kernel chain,
    // and the last `substeps` barriers of every recording were silently
    // dropped. Barrier placement relative to the timestamp marks is unchanged,
    // so the per-pass timings mean what they meant before.
    struct Command {
        bool is_dispatch = false;
        uint32_t pipeline = 0;  // dispatch: PipelineSlot
        PassParams params{};    // dispatch: its push constant
        uint32_t groups = 0;    // dispatch: group count
        uint32_t substep = 0;   // mark: which substep
        uint32_t boundary = 0;  // mark: which boundary of that substep
    };
    std::vector<Command> commands;

    PassParams base{};
    base.world_count = shape_.world_count;
    base.body_capacity = shape_.body_capacity;
    base.element_capacity = shape_.element_capacity;
    base.sensor_capacity = shape_.sensor_capacity;
    base.h = shape_.h;
    base.grid_entry_count = grid_.entry_count;
    base.grid_segment = grid_.segment;
    base.grid_segment_slots = grid_.segment_slots;

    const auto emit = [&](uint32_t pipeline, const PassParams& params, uint32_t groups) {
        commands.push_back(Command{.is_dispatch = true, .pipeline = pipeline, .params = params, .groups = groups});
    };
    const auto mark = [&](uint32_t substep, uint32_t boundary) {
        commands.push_back(Command{.substep = substep, .boundary = boundary});
    };

    // The extent each PassGrid names, resolved once. A degenerate shape yields
    // zeros here, and a vkCmdDispatch with a zero group count is legal and
    // executes nothing -- see the chain note below for why recording it anyway
    // is the right answer.
    const auto groups_of = [&](PassGrid grid) -> uint32_t {
        switch (grid) {
            case PassGrid::body: return dispatch_groups_x_;
            case PassGrid::world: return world_groups_x_;
            case PassGrid::sensor: return sensor_groups_x_;
            case PassGrid::grid_entry: return grid_groups_x_;
            case PassGrid::none: break;
        }
        return 0u;
    };

    for (uint32_t s = 0; s < shape_.substeps; ++s) {
        // S6 Task 10: "start of this substep" mark -- boundary 0 of this
        // substep's 9-slot block. A no-op when timestamps_ is unsupported.
        mark(s, 0);

        for (uint32_t pass = 0; pass < 8u; ++pass) {
            PassParams params = base;
            params.substep = s;

            if (kPassPipeline[pass] == kNoDispatch) {
                // The two INERT slots (S6 Task 8). Nothing is recorded -- not
                // a stub dispatch, not a barrier, not a pipeline bind. See
                // kPassPipeline's comment: the CPU's pass_gravity() and
                // pass_publish() are empty functions, so an empty recording is
                // the faithful mirror and a provably-inert dispatch was not.
            } else if (pass == kForceElementsSlot) {
                // -----------------------------------------------------------
                // THE ForceElements CHAIN (S6 Task 8) -- physics/schedule.cpp's
                // pass_force_elements(), which calls apply_rotors() and THEN
                // apply_drag() and says at the call site why the order is "a
                // numerical contract and not a preference": both accumulate
                // into the same two float3 accumulators, and fp32 addition is
                // not associative. The barrier `emit` places between them is
                // what makes drag's read of `force_acc` see rotors' write.
                //
                // BOTH MEMBERS RUN ON THE BODY GRID, which is why this chain
                // needs no grid bookkeeping of its own -- unlike
                // CollisionDynamic's, whose three stages span two different
                // extents.
                // -----------------------------------------------------------
                emit(kPipelineRotors, params, dispatch_groups_x_);
                emit(kPipelineDrag, params, dispatch_groups_x_);
            } else if (pass == kSensorSynthesisSlot) {
                // -----------------------------------------------------------
                // THE SensorSynthesis CHAIN -- physics/schedule.cpp's
                // pass_sensor_synthesis(), which calls synthesize_imu() and
                // THEN synthesize_gnss().
                //
                // AND THE ORDER IS NOT A CONTRACT HERE, WHICH IS THE OPPOSITE
                // OF THE ForceElements CHAIN ABOVE AND WORTH SAYING SO NOBODY
                // INFERS THE RULE FROM THE SHAPE. Rotors-then-drag is a
                // numerical contract because both accumulate into the same
                // float3 accumulators and fp32 addition is not associative.
                // These two share nothing: disjoint arrays, per-row rng streams
                // seeded under different domain tags, and `bodies` read-only by
                // both. state_digest folds the registry walk in REGISTRATION
                // order rather than dispatch order, so swapping these two lines
                // moves no digest either.
                //
                // The barrier `emit` places between them is therefore NOT
                // load-bearing here, unlike in the chain above. It is left in
                // place because it costs one pipeline barrier per substep and
                // removing it would make this the one dispatch pair in the file
                // whose safety depends on an argument rather than on a barrier.
                //
                // BOTH MEMBERS RUN ON THE SENSOR GRID: one sensor_capacity
                // sizes both arenas, so a single extent covers the pair.
                // -----------------------------------------------------------
                emit(kPipelineSensorImu, params, groups_of(PassGrid::sensor));
                emit(kPipelineSensorGnss, params, groups_of(PassGrid::sensor));
            } else if (pass != kCollisionDynamicSlot) {
                emit(kPassPipeline[pass], params, groups_of(kPassGrid[pass]));
            } else {
                // -----------------------------------------------------------
                // THE CollisionDynamic CHAIN -- physics/grid.cpp's five-stage
                // shape, as three kernels:
                //
                //   1. grid_build          one thread per key-array entry
                //   2. grid_sort  x stages one thread per entry, one dispatch
                //                          per (k, j) stage of the bitonic
                //                          network
                //   3. collision_dynamic   one thread per WORLD (the sweep is
                //                          sequential within a world -- that
                //                          kernel's header has the parity
                //                          argument)
                //
                // RECORDED ONCE, like everything else in this function. The
                // stage loop below is a HOST-side unroll of grid_sort.slang's
                // textbook double loop, so the network is baked into the
                // command buffer and nothing about it is decided per submit. A
                // multi-dispatch pass with barriers between its stages is
                // still a recording (spec section 9's record-once discipline
                // is about not re-recording per step, not about a
                // one-dispatch-per-pass shape).
                //
                // A DEGENERATE SHAPE RECORDS THE SAME CHAIN WITH EMPTY GRIDS
                // rather than a different chain: a world set with no worlds
                // (or no body capacity) yields grid_groups_x_ == 0 /
                // world_groups_x_ == 0, and vkCmdDispatch with a zero group
                // count is legal and executes nothing. Recording it anyway
                // keeps the recorded shape a pure function of `shape_` with no
                // special case to get wrong, and costs a handful of commands
                // that never run.
                // -----------------------------------------------------------
                emit(kPipelineGridBuild, params, grid_groups_x_);

                for (uint32_t k = 2; k <= grid_.segment; k <<= 1) {
                    for (uint32_t j = k >> 1; j > 0; j >>= 1) {
                        PassParams stage = params;
                        stage.sort_k = k;
                        stage.sort_j = j;
                        emit(kPipelineGridSort, stage, grid_groups_x_);
                    }
                }

                emit(kPipelineCollisionDynamic, params, world_groups_x_);
            }

            // S6 Task 10: "slot `pass` just finished" mark -- boundary
            // `pass + 1` of this substep's block, common to every branch
            // above (including the inert one, whose mark simply brackets zero
            // intervening commands and so reads back as a measured ~0 ns
            // rather than an unmeasured one). A no-op when unsupported.
            mark(s, pass + 1);
        }
    }

    // Record the list. The last dispatch is found from the list itself.
    std::size_t last_dispatch = 0;
    for (std::size_t i = 0; i < commands.size(); ++i) {
        if (commands[i].is_dispatch) last_dispatch = i;
    }
    chain_ = RecordedChain{.sort_stages = sort_stage_count()};
    for (std::size_t i = 0; i < commands.size(); ++i) {
        const Command& c = commands[i];
        if (!c.is_dispatch) {
            timestamps_->record_mark(cmd_, c.substep, c.boundary);
            continue;
        }
        vkCmdBindPipeline(cmd_, VK_PIPELINE_BIND_POINT_COMPUTE, pipelines_[c.pipeline]);
        vkCmdPushConstants(cmd_, pipeline_layout_, VK_SHADER_STAGE_COMPUTE_BIT, gen::kPushConstantOffset,
                            gen::kPushConstantSize, &c.params);
        vkCmdDispatch(cmd_, c.groups, 1, 1);
        ++chain_.dispatches;
        if (i < last_dispatch) {
            VkMemoryBarrier barrier{};
            barrier.sType = VK_STRUCTURE_TYPE_MEMORY_BARRIER;
            barrier.srcAccessMask = VK_ACCESS_SHADER_WRITE_BIT;
            barrier.dstAccessMask = VK_ACCESS_SHADER_READ_BIT | VK_ACCESS_SHADER_WRITE_BIT;
            vkCmdPipelineBarrier(cmd_, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
                                  0, 1, &barrier, 0, nullptr, 0, nullptr);
            ++chain_.barriers;
        }
    }

    if (VkResult r = vkEndCommandBuffer(cmd_); r != VK_SUCCESS) {
        return std::unexpected(map_vk_error(r, "vkEndCommandBuffer (StepRecorder record)"));
    }
    return {};
}

// ---------------------------------------------------------------------------
// submit -- n times, the SAME already-recorded command buffer. No
// vkBeginCommandBuffer/vkEndCommandBuffer here at all (S6 Task 5 review
// round 1, C1 fix).
// ---------------------------------------------------------------------------

Result<void> StepRecorder::submit(uint64_t n, uint64_t first_tick) {
    for (uint64_t step_i = 0; step_i < n; ++step_i) {
        // StepParams (C2): the ONE thing that genuinely varies per submit.
        // Written host-side into the persistently mapped, HOST_COHERENT
        // buffer -- no explicit flush, no command-buffer involvement at all,
        // which is exactly why this can change between submits of an
        // otherwise-frozen recording. Two 32-bit words rather than a uint64
        // store, so the device side needs no Int64 capability and neither
        // side assumes a byte order (compute/step_params.hpp).
        const StepParams params = step_params_from_tick(first_tick + step_i);
        std::memcpy(step_params_mapped_, &params, sizeof(params));

        if (VkResult r = vkResetFences(device_, 1, &fence_); r != VK_SUCCESS) {
            return std::unexpected(map_vk_error(r, "vkResetFences (StepRecorder)"));
        }

        VkSubmitInfo submit_info{};
        submit_info.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
        submit_info.commandBufferCount = 1;
        submit_info.pCommandBuffers = &cmd_;
        if (VkResult r = vkQueueSubmit(queue_, 1, &submit_info, fence_); r != VK_SUCCESS) {
            return std::unexpected(map_vk_error(r, "vkQueueSubmit (StepRecorder)"));
        }

        // ⛔ DO NOT TIME THIS WAIT FROM HERE. An earlier draft of L307 (2)
        // wrapped it in a host-side wall-clock timer to size the stall, and
        // M1B.FixedStepNoWallClockSymbolsInEngineSource went red: engine
        // source carries a fixed-step determinism charter that forbids
        // wall-clock symbols outright. The guard is a SYMBOL scan and cannot
        // tell a diagnostic counter from a timestep input -- which is the
        // point of it, not a limitation, because the next use might not be
        // diagnostic. Measure the bubble from the GPU timeline or from the
        // caller, never from in here.
        if (VkResult r = vkWaitForFences(device_, 1, &fence_, VK_TRUE, UINT64_MAX); r != VK_SUCCESS) {
            return std::unexpected(map_vk_error(r, "vkWaitForFences (StepRecorder)"));
        }
    }

    return {};
}

uint64_t StepRecorder::last_written_tick() const noexcept {
    if (step_params_mapped_ == nullptr) return 0;
    StepParams params{};
    std::memcpy(&params, step_params_mapped_, sizeof(params));
    return step_params_tick(params);
}

// S6 Task 10: forwards straight to PassTimestamps -- see that class's
// read_durations_ns() for what "most recently completed submit" means and
// why it is safe to call any time after submit() has returned.
Result<PassDurationsNs> StepRecorder::pass_durations_ns() const { return timestamps_->read_durations_ns(); }

// ---------------------------------------------------------------------------
// teardown
// ---------------------------------------------------------------------------

void StepRecorder::destroy() noexcept {
    if (device_ == VK_NULL_HANDLE) return;  // never fully constructed

    // step_params_mapped_ is NON-OWNING as of S6 Task 6 -- StateMirror owns
    // that buffer and its mapping, and outlives this object (backend.cpp's
    // Impl declares the mirror before the recorder, so the recorder is
    // destroyed first). Nothing to free here; just forget it.
    step_params_mapped_ = nullptr;

    if (fence_ != VK_NULL_HANDLE) {
        vkDestroyFence(device_, fence_, nullptr);
        fence_ = VK_NULL_HANDLE;
    }
    if (pool_ != VK_NULL_HANDLE) {
        vkDestroyCommandPool(device_, pool_, nullptr);  // frees cmd_ implicitly
        pool_ = VK_NULL_HANDLE;
        cmd_ = VK_NULL_HANDLE;
    }
    for (uint32_t i = 0; i < kPipelineCount; ++i) {
        if (pipelines_[i] != VK_NULL_HANDLE) {
            vkDestroyPipeline(device_, pipelines_[i], nullptr);
            pipelines_[i] = VK_NULL_HANDLE;
        }
        if (shaders_[i] != VK_NULL_HANDLE) {
            vkDestroyShaderModule(device_, shaders_[i], nullptr);
            shaders_[i] = VK_NULL_HANDLE;
        }
    }
    if (pipeline_layout_ != VK_NULL_HANDLE) {
        vkDestroyPipelineLayout(device_, pipeline_layout_, nullptr);
        pipeline_layout_ = VK_NULL_HANDLE;
    }

    device_ = VK_NULL_HANDLE;
}

StepRecorder::~StepRecorder() { destroy(); }

}  // namespace spade::compute
