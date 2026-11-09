#include "compute/vulkan/step_recorder.hpp"

#include <algorithm>
#include <cstddef>
#include <cstring>
#include <iterator>
#include <string>
#include <utility>
#include <vector>

#include "bindings.gen.hpp"
#include "compute/admit.hpp"
#include "compute/layout_classes.hpp"

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
// The device-limit checks of a created recorder (local size, group counts) are compute::admit()'s (stage 7 Task A1).

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
                                                             uint32_t workgroup_size,
                                                             std::span<const GpuPass> passes,
                                                             std::span<const SetOneSet> set1_sets,
                                                             std::span<const GpuPass> rebuilds) {
    // A pass with no recipe has no GPU kernel: recording the rest would run a
    // different experiment from the CPU, silently (L6). Refused before any
    // resource is built. A rebuild hook is a pass too.
    for (const std::span<const GpuPass> list : {passes, rebuilds}) {
        for (const GpuPass& pass : list) {
            if (pass.recipe == GpuRecipe::none && pass.custom == nullptr) {
                return std::unexpected(Error{Code::invalid_argument, "StepRecorder::create: pass '" + pass.name +
                                                                         "' has no GPU kernel (GpuRecipe::none)"});
            }
        }
    }

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

    auto self = std::unique_ptr<StepRecorder>(new StepRecorder());
    self->device_ = ctx.device();
    self->queue_ = ctx.compute_queue();
    self->set_ = set;
    self->shape_ = shape;
    self->step_params_mapped_ = step_params_mapped;
    self->workgroup_size_ = workgroup_size;
    self->passes_.assign(passes.begin(), passes.end());
    self->rebuilds_.assign(rebuilds.begin(), rebuilds.end());
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

    self->limits_ = ctx.device_report().limits;
    self->device_name_ = std::string(ctx.device_name());

    // THE RECIPES (stage 7 Task B1): each pass's own, or its built-in's. The kernels are deduplicated by variant set, in the
    // order they first appear, so the standard set builds the twelve it always did and a set that never names a recipe builds
    // none of its kernels. A kernel naming set-1 bindings is counted by admit() as set 0's thirty plus its subset
    // (kernel_layout()), and gets its own pipeline layout below. The rebuild hooks' recipes (Task C3) are resolved the same way.
    AdmitShape asked;
    const auto resolve = [&](const GpuPass& pass, std::vector<const RecipeDesc*>& recipes,
                             std::vector<std::vector<uint32_t>>& all_slots) -> Result<void> {
        const RecipeDesc* desc = pass.custom != nullptr ? pass.custom : builtin_recipe(pass.recipe);
        if (desc == nullptr || desc->record == nullptr) {
            return std::unexpected(Error{Code::internal, "StepRecorder::create: pass '" + pass.name + "' has no recipe"});
        }
        recipes.push_back(desc);
        std::vector<uint32_t> slots;
        for (const RecipeKernel& k : desc->kernels) {
            if (k.variants == nullptr) {
                return std::unexpected(Error{Code::invalid_argument, "StepRecorder::create: recipe '" +
                                                                         std::string(desc->name) + "' (pass '" + pass.name +
                                                                         "') names a kernel with no compiled variants"});
            }
            Result<AdmitLayout> layout = kernel_layout(k.variants->name, k.set1_bindings);
            if (!layout) return std::unexpected(layout.error());
            asked.layouts.push_back(std::move(*layout));
            std::vector<uint32_t> key = normalized_set1_subset(k.set1_bindings);
            std::size_t at = 0;
            while (at < self->kernels_.size() &&
                   !(self->kernels_[at].variants == k.variants && self->kernels_[at].set1_key == key)) {
                ++at;
            }
            if (at == self->kernels_.size()) {
                self->kernels_.push_back(Kernel{.variants = k.variants, .set1_key = std::move(key)});
            }
            slots.push_back(static_cast<uint32_t>(at));
        }
        all_slots.push_back(std::move(slots));
        return {};
    };
    for (const GpuPass& pass : self->rebuilds_) {
        if (Result<void> ok = resolve(pass, self->rebuild_recipes_, self->rebuild_kernels_); !ok) {
            return std::unexpected(ok.error());
        }
    }
    for (const GpuPass& pass : self->passes_) {
        if (Result<void> ok = resolve(pass, self->pass_recipes_, self->pass_kernels_); !ok) {
            return std::unexpected(ok.error());
        }
        if (pass.every_steps > 1) {
            if (self->cadence_ != 1 && self->cadence_ != pass.every_steps) {
                return std::unexpected(Error{Code::invalid_argument,
                                             "StepRecorder::create: pass '" + pass.name + "' runs every " +
                                                 std::to_string(pass.every_steps) + " steps, another every " +
                                                 std::to_string(self->cadence_) + "; the set has one cadence"});
            }
            self->cadence_ = pass.every_steps;
        }
    }

    // THE RECORDINGS (stage 7 Task B2): recording 0 holds every pass; recording 1, only with a cadence, the passes that run
    // every step. Each has its own timing pool, named for the passes it holds.
    const uint32_t recording_count = self->cadence_ > 1 ? 2u : 1u;
    self->recordings_.resize(recording_count);
    for (uint32_t r = 0; r < recording_count; ++r) {
        Recording& rec = self->recordings_[r];
        std::vector<std::string> pass_names;
        for (uint32_t i = 0; i < self->passes_.size(); ++i) {
            if (r == 1 && self->passes_[i].every_steps > 1) continue;
            rec.passes.push_back(i);
            pass_names.push_back(self->passes_[i].name);
        }
        // S6 Task 10: per-pass GPU timing, created unconditionally. PassTimestamps::create() never fails on a capability
        // gap, only on a genuine Vulkan failure.
        Result<std::unique_ptr<PassTimestamps>> timestamps =
            PassTimestamps::create(ctx, shape.substeps, std::move(pass_names));
        if (!timestamps) return std::unexpected(timestamps.error());
        rec.timestamps = std::move(*timestamps);
    }

    // The kernels' local size and ALL FOUR grids against the device's limits, by name, through compute::admit() (stage 7
    // Task A1). Checked here, after every grid is known, because the recording below emits dispatches on all of them and
    // vkCmdDispatch does not report an over-limit group count, it is simply invalid usage; a local size past
    // maxComputeWorkGroupInvocations or maxComputeWorkGroupSize is the same. The widest grid is the only one that can
    // fail, and the refusal names which.
    {
        asked.kernels.push_back(AdmitKernel{.name = "workgroup_size " + std::to_string(workgroup_size),
                                            .local = {workgroup_size, 1, 1}});
        asked.dispatches = {AdmitDispatch{.pass = "bodies", .groups = {self->dispatch_groups_x_, 1, 1}},
                            AdmitDispatch{.pass = "grid", .groups = {self->grid_groups_x_, 1, 1}},
                            AdmitDispatch{.pass = "worlds", .groups = {self->world_groups_x_, 1, 1}},
                            AdmitDispatch{.pass = "sensors", .groups = {self->sensor_groups_x_, 1, 1}}};
        if (Result<void> ok = admit(asked, ctx.device_report().limits, ctx.device_name()); !ok) {
            Error e = ok.error();
            e.context = "StepRecorder::create: " + e.context +
                        ". Note that group count is ceil(extent / local size), so a SMALLER workgroup_size needs MORE "
                        "groups for the same shape -- 64 or 128 may fit where 32 does not.";
            return std::unexpected(std::move(e));
        }
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

    // THE LAYOUT CLASSES (stage 7 Task C2; plan decision 1). A kernel with no set-1 subset keeps the one set-0 layout above,
    // exactly as every standard kernel always did. A kernel naming a subset gets a layout [set 0, the set-1 layout the
    // mirror built for exactly that subset], one per distinct subset, shared by the kernels naming it. A subset the mirror
    // did not build is a defect of the caller's (backend.cpp derives both from the same recipes), named.
    for (Kernel& kernel : self->kernels_) {
        kernel.layout = self->pipeline_layout_;
        if (kernel.set1_key.empty()) continue;
        const SetOneSet* one = nullptr;
        for (const SetOneSet& candidate : set1_sets) {
            if (candidate.bindings == kernel.set1_key) one = &candidate;
        }
        if (one == nullptr) {
            return std::unexpected(Error{Code::internal, "StepRecorder::create: kernel '" +
                                                             std::string(kernel.variants->name) +
                                                             "' names a set-1 subset the state mirror built no set for"});
        }
        kernel.set1 = one->set;
        const Kernel* shared = nullptr;
        for (const Kernel& other : self->kernels_) {
            if (&other == &kernel) break;
            if (other.set1_key == kernel.set1_key) shared = &other;
        }
        if (shared != nullptr) {
            kernel.layout = shared->layout;
            continue;
        }
        const VkDescriptorSetLayout both[] = {set_layout, one->layout};
        VkPipelineLayoutCreateInfo one_layout_info{};
        one_layout_info.sType = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO;
        one_layout_info.setLayoutCount = 2;
        one_layout_info.pSetLayouts = both;
        one_layout_info.pushConstantRangeCount = 1;
        one_layout_info.pPushConstantRanges = &push_range;
        VkPipelineLayout made = VK_NULL_HANDLE;
        if (VkResult r = vkCreatePipelineLayout(self->device_, &one_layout_info, nullptr, &made); r != VK_SUCCESS) {
            return std::unexpected(map_vk_error(r, "vkCreatePipelineLayout (set 0 + set 1)"));
        }
        self->set1_layouts_.push_back(made);
        kernel.layout = made;
    }

    // One pipeline per distinct kernel (stage 7 Task B1); make_pipeline() picks the variant matching workgroup_size_.
    for (Kernel& kernel : self->kernels_) {
        if (Result<void> made = self->make_pipeline(*kernel.variants, ctx.device_report(), kernel.layout, kernel.shader,
                                                    kernel.pipeline);
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
    // The step's recordings, then (Task C3) the one on-demand rebuild recording when the set has rebuild hooks.
    const bool has_rebuild = !self->rebuilds_.empty();
    const uint32_t buffer_count = recording_count + (has_rebuild ? 1u : 0u);
    cmd_info.commandBufferCount = buffer_count;
    {
        std::vector<VkCommandBuffer> cmds(buffer_count, VK_NULL_HANDLE);
        if (VkResult r = vkAllocateCommandBuffers(self->device_, &cmd_info, cmds.data()); r != VK_SUCCESS) {
            return std::unexpected(map_vk_error(r, "vkAllocateCommandBuffers (StepRecorder)"));
        }
        for (uint32_t r = 0; r < recording_count; ++r) self->recordings_[r].cmd = cmds[r];
        if (has_rebuild) self->rebuild_cmd_ = cmds[recording_count];
    }

    VkFenceCreateInfo fence_info{};
    fence_info.sType = VK_STRUCTURE_TYPE_FENCE_CREATE_INFO;
    if (VkResult r = vkCreateFence(self->device_, &fence_info, nullptr, &self->fence_); r != VK_SUCCESS) {
        return std::unexpected(map_vk_error(r, "vkCreateFence (StepRecorder)"));
    }
    if (has_rebuild) {
        if (VkResult r = vkCreateFence(self->device_, &fence_info, nullptr, &self->rebuild_fence_); r != VK_SUCCESS) {
            return std::unexpected(map_vk_error(r, "vkCreateFence (StepRecorder rebuild)"));
        }
    }

    // Records the fixed dispatch chain ONCE -- see record()'s own comment.
    for (uint32_t r = 0; r < recording_count; ++r) {
        if (Result<void> recorded = self->record(r); !recorded) return std::unexpected(recorded.error());
    }
    if (has_rebuild) {
        if (Result<void> recorded = self->record_rebuild(); !recorded) return std::unexpected(recorded.error());
    }

    return self;
}

Result<void> StepRecorder::make_pipeline(const SpirvVariantSet& variants, const DeviceReport& report,
                                           VkPipelineLayout layout, VkShaderModule& out_module,
                                           VkPipeline& out_pipeline) {
    // The selection, the shader module and the pipeline: dispatch_chain.hpp's
    // make_compute_pipeline, shared with the airflow chain runner (its design
    // note §1). The messages name "StepRecorder", as they always have.
    return make_compute_pipeline(device_, layout, variants, workgroup_size_, report, "StepRecorder", out_module,
                                 out_pipeline);
}

// The context every recipe starts from (substep 0; record() sets each substep), shared by the step's recordings and the
// rebuild recording.
RecipeContext StepRecorder::base_context() const {
    RecipeContext context;
    context.base.world_count = shape_.world_count;
    context.base.body_capacity = shape_.body_capacity;
    context.base.element_capacity = shape_.element_capacity;
    context.base.sensor_capacity = shape_.sensor_capacity;
    context.base.h = shape_.h;
    context.base.grid_entry_count = grid_.entry_count;
    context.base.grid_segment = grid_.segment;
    context.base.grid_segment_slots = grid_.segment_slots;
    context.base.switches = shape_.switches;  // module API stage 6: which optional bindings hold a real array
    context.workgroup_size = workgroup_size_;
    context.bodies_groups = dispatch_groups_x_;
    context.grid_groups = grid_groups_x_;
    context.worlds_groups = world_groups_x_;
    context.sensors_groups = sensor_groups_x_;
    context.point_rows = shape_.point_rows;
    return context;
}

// One entry per kernel: the pipeline layout and set-1 set a kernel with a set-1 subset binds before each dispatch; a null
// layout for a kernel with none, which binds nothing (the chain's set 0 is bound once at the head of each recording).
std::vector<PipelineSetBinding> StepRecorder::set_bindings() const {
    std::vector<PipelineSetBinding> binds(kernels_.size());
    for (std::size_t i = 0; i < kernels_.size(); ++i) {
        if (kernels_[i].set1_key.empty()) continue;
        binds[i] = PipelineSetBinding{.layout = kernels_[i].layout, .set_index = 1, .set = kernels_[i].set1};
    }
    return binds;
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

Result<void> StepRecorder::record(uint32_t recording) {
    Recording& rec = recordings_[recording];
    VkCommandBuffer cmd = rec.cmd;

    VkCommandBufferBeginInfo begin{};
    begin.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
    // NOT ONE_TIME_SUBMIT: submit() resubmits this SAME recording arbitrarily many times without ever resetting it.
    if (VkResult r = vkBeginCommandBuffer(cmd, &begin); r != VK_SUCCESS) {
        return std::unexpected(map_vk_error(r, "vkBeginCommandBuffer (StepRecorder record)"));
    }

    // The descriptor set is bound ONCE for the whole chain: every pipeline shares one layout.
    vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_COMPUTE, pipeline_layout_, 0, 1, &set_, 0, nullptr);

    // S6 Task 10: reset every per-pass timing query slot ONCE, before the first dispatch (a no-op when unsupported).
    rec.timestamps->record_reset(cmd);

    // The per-substep chain, `shape_.substeps` times, walking this recording's passes in schedule order. Each pass records
    // what its recipe's function appends (compute/recipe.hpp; the built-ins are compute/vulkan/builtin_recipes.cpp, the old
    // switch moved unchanged). THE CADENCE (stage 7 Task B2): a pass with every_steps > 1 is in recording 0 only, and only
    // in substep 0 -- the same rule as sim/cadence.hpp's due() at a tick that is a multiple of the cadence. Its timing marks
    // are still recorded in the other substeps, bracketing nothing, so the marks per substep stay the pass count.
    //
    // A memory barrier after every dispatch except the very last one of the WHOLE recorded chain (CORE-2), read off the
    // list of what was emitted (dispatch_chain.hpp).
    //
    // EVERY PassParams FIELD HERE IS SHAPE OR DISPATCH DATA, NOT STEP DATA, which is what makes recording once correct:
    // `substep` cycles 0..substeps-1 identically on every step, the sort lanes name which stage a dispatch is, and the one
    // thing that varies per submit, the tick, does not travel through a push constant at all (compute/step_params.hpp).
    static_assert(sizeof(PassParams) <= kMaxChainPushBytes, "PassParams must fit a chain command's push block");
    static_assert(sizeof(PassParams) == gen::kPushConstantSize, "PassParams must be exactly the push range");
    static_assert(gen::kPushConstantOffset + gen::kPushConstantSize <= kMaxChainPushBytes,
                  "the push range must fit a chain command's push block");
    std::vector<ChainCommand> commands;

    RecipeContext context = base_context();

    std::vector<std::string> walked;  // the pass names as the walk visits them, once, for recorded_chain()
    walked.reserve(rec.passes.size());
    AdmitShape emitted;               // every emitted dispatch, against the device's group-count limits

    for (uint32_t s = 0; s < shape_.substeps; ++s) {
        // S6 Task 10: "start of this substep" mark -- boundary 0 of this substep's block.
        commands.push_back(ChainCommand{.mark_a = s, .mark_b = 0});

        for (uint32_t li = 0; li < rec.passes.size(); ++li) {
            const uint32_t i = rec.passes[li];
            const GpuPass& pass = passes_[i];
            const bool recorded_now = pass.every_steps <= 1 || s == 0;
            if (recorded_now) {
                context.base.substep = s;
                RecipeEmitter emitter;
                pass_recipes_[i]->record(emitter, context);
                for (const EmittedDispatch& d : emitter.dispatches()) {
                    if (d.kernel >= pass_kernels_[i].size()) {
                        return std::unexpected(Error{Code::internal, "StepRecorder::record: recipe '" +
                                                                         std::string(pass_recipes_[i]->name) +
                                                                         "' dispatched kernel " + std::to_string(d.kernel) +
                                                                         " of " + std::to_string(pass_kernels_[i].size())});
                    }
                    ChainCommand c{.is_dispatch = true, .pipeline = pass_kernels_[i][d.kernel], .groups = d.groups};
                    std::memcpy(c.push.data(), &d.params, sizeof(PassParams));
                    commands.push_back(c);
                    emitted.dispatches.push_back(AdmitDispatch{.pass = pass.name, .groups = d.groups});
                }
            }
            // S6 Task 10: "pass li just finished" mark -- boundary li + 1, common to every recipe (including the two that
            // record nothing, which then read back as a measured ~0 ns rather than an unmeasured one).
            commands.push_back(ChainCommand{.mark_a = s, .mark_b = li + 1});
            if (s == 0) walked.push_back(pass.name);
        }
    }

    // The group counts of what was actually emitted, by pass and axis (stage 7 Task A1/B1): a module's own recipe computes
    // its own groups, and a count past the device's limit is invalid usage, not a reported error.
    if (Result<void> ok = admit(emitted, limits_, device_name_); !ok) {
        Error e = ok.error();
        e.context = "StepRecorder::record: " + e.context;
        return std::unexpected(std::move(e));
    }

    std::vector<VkPipeline> pipelines;
    pipelines.reserve(kernels_.size());
    for (const Kernel& kernel : kernels_) pipelines.push_back(kernel.pipeline);

    rec.chain = RecordedChain{.sort_stages = sort_stage_count()};
    rec.chain.passes = std::move(walked);
    const std::vector<PipelineSetBinding> set_binds = set_bindings();
    const ChainCounts counts = record_dispatch_chain(
        cmd, pipeline_layout_, pipelines, commands, gen::kPushConstantOffset, gen::kPushConstantSize,
        [&rec](VkCommandBuffer c, uint32_t substep, uint32_t boundary) {
            rec.timestamps->record_mark(c, substep, boundary);
        },
        set_binds);
    rec.chain.dispatches = counts.dispatches;
    rec.chain.barriers = counts.barriers;

    if (VkResult r = vkEndCommandBuffer(cmd); r != VK_SUCCESS) {
        return std::unexpected(map_vk_error(r, "vkEndCommandBuffer (StepRecorder record)"));
    }
    return {};
}

// ---------------------------------------------------------------------------
// record_rebuild -- ONCE, from create(), when the set has rebuild hooks (stage 7 Task C3). One emitter walk per rebuild
// pass, in order; the chain's barrier rule (CORE-2) puts a barrier between adjacent dispatches and none after the last.
// No timing marks and no cadence: the recording is submitted on demand (submit_rebuild()), at create() and at restore().
// Every emitted dispatch's group counts are admitted like any other.
// ---------------------------------------------------------------------------
Result<void> StepRecorder::record_rebuild() {
    VkCommandBuffer cmd = rebuild_cmd_;
    VkCommandBufferBeginInfo begin{};
    begin.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
    // NOT ONE_TIME_SUBMIT: the same recording is resubmitted at every restore().
    if (VkResult r = vkBeginCommandBuffer(cmd, &begin); r != VK_SUCCESS) {
        return std::unexpected(map_vk_error(r, "vkBeginCommandBuffer (StepRecorder rebuild)"));
    }
    vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_COMPUTE, pipeline_layout_, 0, 1, &set_, 0, nullptr);

    RecipeContext context = base_context();
    context.base.substep = 0;
    std::vector<ChainCommand> commands;
    AdmitShape emitted;
    for (std::size_t i = 0; i < rebuilds_.size(); ++i) {
        RecipeEmitter emitter;
        rebuild_recipes_[i]->record(emitter, context);
        for (const EmittedDispatch& d : emitter.dispatches()) {
            if (d.kernel >= rebuild_kernels_[i].size()) {
                return std::unexpected(Error{Code::internal, "StepRecorder::record_rebuild: recipe '" +
                                                                 std::string(rebuild_recipes_[i]->name) +
                                                                 "' dispatched kernel " + std::to_string(d.kernel) +
                                                                 " of " + std::to_string(rebuild_kernels_[i].size())});
            }
            ChainCommand c{.is_dispatch = true, .pipeline = rebuild_kernels_[i][d.kernel], .groups = d.groups};
            std::memcpy(c.push.data(), &d.params, sizeof(PassParams));
            commands.push_back(c);
            emitted.dispatches.push_back(AdmitDispatch{.pass = rebuilds_[i].name, .groups = d.groups});
        }
    }
    if (Result<void> ok = admit(emitted, limits_, device_name_); !ok) {
        Error e = ok.error();
        e.context = "StepRecorder::record_rebuild: " + e.context;
        return std::unexpected(std::move(e));
    }

    std::vector<VkPipeline> pipelines;
    pipelines.reserve(kernels_.size());
    for (const Kernel& kernel : kernels_) pipelines.push_back(kernel.pipeline);
    const std::vector<PipelineSetBinding> set_binds = set_bindings();
    (void)record_dispatch_chain(cmd, pipeline_layout_, pipelines, commands, gen::kPushConstantOffset,
                                gen::kPushConstantSize, [](VkCommandBuffer, uint32_t, uint32_t) {}, set_binds);

    if (VkResult r = vkEndCommandBuffer(cmd); r != VK_SUCCESS) {
        return std::unexpected(map_vk_error(r, "vkEndCommandBuffer (StepRecorder rebuild)"));
    }
    return {};
}

Result<void> StepRecorder::submit_rebuild() {
    if (rebuild_cmd_ == VK_NULL_HANDLE) return {};  // no rebuild recipes: nothing to submit, and not counted
    if (VkResult r = vkResetFences(device_, 1, &rebuild_fence_); r != VK_SUCCESS) {
        return std::unexpected(map_vk_error(r, "vkResetFences (StepRecorder rebuild)"));
    }
    VkSubmitInfo submit_info{};
    submit_info.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
    submit_info.commandBufferCount = 1;
    submit_info.pCommandBuffers = &rebuild_cmd_;
    if (VkResult r = vkQueueSubmit(queue_, 1, &submit_info, rebuild_fence_); r != VK_SUCCESS) {
        return std::unexpected(map_vk_error(r, "vkQueueSubmit (StepRecorder rebuild)"));
    }
    if (VkResult r = vkWaitForFences(device_, 1, &rebuild_fence_, VK_TRUE, UINT64_MAX); r != VK_SUCCESS) {
        return std::unexpected(map_vk_error(r, "vkWaitForFences (StepRecorder rebuild)"));
    }
    ++on_demand_count_;
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

        // THE RECORDING IS A FUNCTION OF THE TICK ALONE (stage 7 Task B2): the one holding every pass on a step whose tick is
        // a multiple of the cadence, the other on the rest -- so a restore lands on the same cadence.
        const uint64_t tick = first_tick + step_i;
        const uint32_t which = cadence_ > 1 && tick % cadence_ != 0 ? 1u : 0u;
        last_recording_ = which;
        last_tick_ = tick;

        VkSubmitInfo submit_info{};
        submit_info.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
        submit_info.commandBufferCount = 1;
        submit_info.pCommandBuffers = &recordings_[which].cmd;
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
Result<PassDurationsNs> StepRecorder::pass_durations_ns() const {
    Result<PassDurationsNs> d = recordings_[last_recording_].timestamps->read_durations_ns();
    if (d) {
        d->recording = last_recording_;  // which recording ran in the most recently completed submit, and its tick
        d->tick = last_tick_;
    }
    return d;
}

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
    if (rebuild_fence_ != VK_NULL_HANDLE) {
        vkDestroyFence(device_, rebuild_fence_, nullptr);
        rebuild_fence_ = VK_NULL_HANDLE;
    }
    if (pool_ != VK_NULL_HANDLE) {
        vkDestroyCommandPool(device_, pool_, nullptr);  // frees the recordings' command buffers implicitly
        pool_ = VK_NULL_HANDLE;
    }
    recordings_.clear();  // the timing pools
    for (Kernel& kernel : kernels_) {
        if (kernel.pipeline != VK_NULL_HANDLE) vkDestroyPipeline(device_, kernel.pipeline, nullptr);
        if (kernel.shader != VK_NULL_HANDLE) vkDestroyShaderModule(device_, kernel.shader, nullptr);
    }
    kernels_.clear();
    for (VkPipelineLayout layout : set1_layouts_) vkDestroyPipelineLayout(device_, layout, nullptr);
    set1_layouts_.clear();
    if (pipeline_layout_ != VK_NULL_HANDLE) {
        vkDestroyPipelineLayout(device_, pipeline_layout_, nullptr);
        pipeline_layout_ = VK_NULL_HANDLE;
    }

    device_ = VK_NULL_HANDLE;
}

StepRecorder::~StepRecorder() { destroy(); }

}  // namespace spade::compute
