#pragma once

// ---------------------------------------------------------------------------
// step_recorder.hpp (S6 Task 5; kernels wave A wired in by Task 6) -- the step
// command buffer: PassParams (the push-constant block every dispatch carries)
// and StepRecorder (the object that dispatches spec section 3's 8-pass chain
// once per step, stage-scoped barriers between passes, submit-per-step).
//
// INTERIM (module-API plan, stage 1, 2026-10-02): physics/schedule.cpp's
// kSchedule is gone. The CPU runs the compiled module schedule
// (sim/module.hpp, sim/standard_modules.cpp), and this table mirrors the
// STANDARD set's compiled order, which
// Schedule.TheCompiledStandardSetFollowsTheGpuRecordersOrder pins. Stage 2
// derives the GPU chain from the schedule and deletes this table; until then,
// read "kSchedule" below as "the compiled standard set", and the CPU's empty
// Gravity and Publish passes as removed.
//
// "SECTION 3's EIGHT", not "the schedule's ten": physics/schedule.cpp's
// kSchedule grew two inert SL6 behavior slots (Plan A Task 7) that this class
// deliberately does not model. step_recorder.cpp's kNoDispatch comment carries
// the reasoning and names the test that keeps the divergence deliberate.
//
// ---------------------------------------------------------------------------
// ALL EIGHT SLOTS ARE REAL AS OF S6 TASK 8, AND THE STUB IS RETIRED. Spec
// section 3's eight passes -- kSchedule's ten less its two behavior slots --
// bind, in order (slot numbers below are THIS table's, not kSchedule's):
//
//   0 MediumUpdate      medium_update    -- REAL (S6 Task 8). One thread per
//                                          WORLD: the Dryden filter is a
//                                          sequential recurrence within a world
//                                          and worlds are the parallel axis.
//   1 ForceElements     rotors           -- REAL, and the SECOND slot that is a
//                       + forces_drag      CHAIN rather than a single dispatch.
//                                          The order is physics/schedule.cpp's
//                                          and is a NUMERICAL contract, not a
//                                          preference: both kernels accumulate
//                                          into the same two float3
//                                          accumulators (that file says so at
//                                          the call site).
//   2 Gravity           -- NO DISPATCH -- INERT BY DESIGN, on BOTH backends:
//                                          gravity is applied inside Integrate
//                                          (physics/integrator.hpp), and the
//                                          CPU's pass_gravity() is an empty
//                                          function for exactly this reason.
//                                          The slot is kept in this table so
//                                          the correspondence with spec section
//                                          3's eight names stays literal, which
//                                          is the same choice schedule.cpp made
//                                          and documented -- but nothing is
//                                          recorded for it.
//   3 CollisionStatic   collision_static -- REAL (sphere proxy vs world SDF)
//   4 CollisionDynamic  grid_build       -- REAL as of S6 Task 7: build the
//                       + grid_sort xS     sorted-grid keys, run the bitonic
//                       + collision_dynamic network over them one stage per
//                                          dispatch, then sweep.
//   5 Integrate         integrate        -- REAL
//   6 SensorSynthesis   sensor_imu       -- REAL (S6 Task 8). One thread per
//                                          SENSOR slot -- a third dispatch
//                                          grid, over the array this pass
//                                          actually writes.
//   7 Publish           -- NO DISPATCH -- inert on the CPU twin too
//
// THE STUB KERNEL IS GONE (S6 Task 8), AND ITS ABSENCE IS A DELIVERABLE. Slots
// 2 and 7 used to dispatch pipeline_smoke -- Task 3's smoke kernel, respelled
// by Task 5's review round 2 into a literal bit copy so that dispatching it
// changed nothing. With the last two real passes ported, no slot needs a
// placeholder, and a dispatch that provably does nothing is worse than no
// dispatch: it costs a pipeline, a bind, a push constant, a barrier and a grid
// per substep, and it invites a reader to ask what it is for. So the two inert
// slots now record NOTHING, and engine/shaders/kernels/pipeline_smoke.slang,
// its CMake entry, its kSpirvModules row and engine/testing/spirv_scan.hpp's
// NO_OP profile (whose only instance it was) are deleted with it. This task's
// report carries that decision and flags it for review.
//
// WHAT THIS MEANS FOR A CALLER, STATED PLAINLY: a vulkan-backed Simulation now
// runs the WHOLE substep -- turbulence, rotors, drag, static and dynamic
// contact, integration and IMU synthesis. sim/simulation.cpp's
// check_vulkan_unported_config()/check_vulkan_unported_state() gate is
// therefore deleted rather than shrunk: there is no unported pass left for it
// to refuse a world set on behalf of.
// ---------------------------------------------------------------------------

#include <cstddef>
#include <cstdint>
#include <memory>

#include <volk.h>

#include "compute/backend.hpp"
#include "compute/grid_entry.hpp"
#include "compute/spirv_variants.hpp"
#include "compute/step_params.hpp"
#include "compute/vulkan/context.hpp"
#include "compute/vulkan/timestamps.hpp"
#include "core/error.hpp"

namespace spade::compute {

// ---------------------------------------------------------------------------
// PassParams -- the C++ mirror of shaders/shared/layouts.slang's PassParams,
// the per-dispatch push-constant block (bindings.slang: `[[vk::push_constant]]
// ConstantBuffer<PassParams> pass_params`). THIS is the C++ struct
// layouts.slang's own header comment asked for: "the host will need a C++
// struct to write into the push-constant range, and the moment that struct
// exists it should gain @cpp directives here like every other mirror" -- S6
// Task 5 added exactly those two directives above layouts.slang's PassParams,
// so gen_layout_check.py emits the same per-field static_asserts for it as for
// every other mirrored struct (layout_check.gen.hpp, compiled into
// compute/layout_check.cpp).
//
// FIELD ORDER IS THE CONTRACT, exactly as layouts.slang's own header says for
// every mirror: do not reorder.
//
// NO `contact`/`grid` FIELDS AS OF S6 TASK 6b (checkpoint-1 ruling). S6 Task 6
// added `physics::ContactParams contact`/`physics::GridParams grid` here,
// world 0's records, on the theory that a future batched CollisionDynamic
// kernel would read them as one per-dispatch material for the whole batch.
// Task 6b's PassParams decision tree grepped every kernel plus this file's own
// record() below and found no reader anywhere -- record() WROTE `run_.contact`/
// `run_.grid` into the two fields every dispatch, but nothing ever read them
// back out of the push constant -- so both fields (and their `_pad0`/`_pad1`
// alignment lanes, no longer needed once the alignas(16) members they existed
// to align are gone) are removed, along with `run_` and the RunParams struct
// that fed them (compute/backend.hpp). CollisionStatic already reads a
// PER-WORLD `contact_params` buffer instead (bindings.slang binding 19,
// unaffected by this removal); T7's CollisionDynamic gets a PER-WORLD
// `grid_params` buffer (binding 20, S6 Task 6b) for the identical reason: a
// world set may be heterogeneous, so a single per-dispatch record cannot serve
// it. See engine/shaders/kernels/collision_static.slang's header for the
// established precedent this ruling extends.
//
// NO `tick` FIELD HERE. PassParams is push-constant-delivered, and push
// constants are baked into the command buffer at RECORD time (see
// StepRecorder's own header note below on why this class now records ONCE
// per shape) -- a value that must vary on every one of the n resubmits of an
// already-recorded buffer cannot live in a push constant at all. The tick
// therefore does NOT go here; see StepParams below for where it actually
// lives (coordinator ruling, S6 Task 5 review round 1, finding C2).
// ---------------------------------------------------------------------------
// S6 TASK 7 ADDED FIVE UINT LANES, and every one of them is SHAPE or DISPATCH
// data -- which is what makes them legal here where the removed `contact`/
// `grid` were not. The three `grid_*` lanes are compute/grid_entry.hpp's
// GridDomain, i.e. a pure function of StepShape (how many key-array entries,
// how big one independently-sorted segment is, how many body slots map into
// one segment); the two `sort_*` lanes name WHICH STAGE of the bitonic network
// a dispatch is, and are the first PassParams fields that differ from dispatch
// to dispatch inside the recorded chain -- which is exactly what a push
// constant baked at record time can express and a per-submit value cannot.
// NEITHER GROUP IS A PHYSICS PARAMETER: cell_size still comes from
// `grid_params[world]` and the material from `contact_params[world]`, per
// world, per the checkpoint-1 ruling above.
struct PassParams {
    uint32_t world_count = 0;
    uint32_t body_capacity = 0;
    uint32_t element_capacity = 0;
    uint32_t sensor_capacity = 0;
    float h = 0.0f;
    uint32_t substep = 0;
    uint32_t grid_entry_count = 0;
    uint32_t grid_segment = 0;
    uint32_t grid_segment_slots = 0;
    uint32_t sort_k = 0;
    uint32_t sort_j = 0;
};

// ---------------------------------------------------------------------------
// StepParams MOVED to compute/step_params.hpp (S6 Task 6), and the reason is
// mechanical rather than aesthetic: it is now MIRRORED in
// engine/shaders/shared/layouts.slang, so the generated layout_check.gen.hpp
// #includes whatever header declares it -- and that generated header is
// compiled by compute/layout_check.cpp. Declaring it in THIS file would have
// pulled <volk.h> into that translation unit to reach an eight-byte POD.
// compute/step_params.hpp is Vulkan-free, carries the same struct with the
// same contract (plus the two-word tick spelling the device side needs), and
// is included below so every existing user of this header still sees the type.
//
// Its other half -- StepWitness, what the Integrate kernel publishes to prove
// it READ the buffer -- lives beside it there. S6 Task 5's round-1 ruling
// deferred "GPU-visible consumption" to Task 6; that is what closed it.
// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------
// StepRecorder -- ONE command buffer, recorded ONCE per shape (at create()
// time) and resubmitted verbatim on every submit() call.
//
// "RECORD ONCE PER SHAPE", CORRECTED (S6 Task 5 review round 1, finding C1).
// Every object this class owns is shape-invariant and now genuinely built
// exactly once, in create(): the compute pipeline, the pipeline layout, the
// command pool, the dispatch grid size, AND -- as of this fix -- the
// recorded command buffer itself. Round 1's earlier design re-recorded the
// identical per-substep dispatch chain inside submit()'s per-step loop, which
// bought nothing (every re-recording produced byte-identical commands) and
// mis-stated what "record once" means. What made that seem necessary was
// treating PassParams::substep and the step's tick as both needing to vary
// per submit -- but `substep` cycles 0..substeps-1 IDENTICALLY every step
// (it is shape data, not step data), so it can be, and now is, baked into
// the one-time recording via vkCmdPushConstants exactly like every other
// PassParams field. Only the tick genuinely varies per submit, and it no
// longer goes through a push constant at all (see StepParams above).
// submit() therefore does no vkBeginCommandBuffer/vkEndCommandBuffer at all
// -- only vkQueueSubmit of the SAME already-recorded VkCommandBuffer,
// exactly n times, which is what "n steps = n submits of the same command
// buffer" now means literally, not merely at the object-identity level the
// previous revision settled for.
//
// STAGE-SCOPED BARRIERS, MINIMAL (spec section 9). One VkMemoryBarrier
// (whole-memory SHADER_WRITE -> SHADER_READ|WRITE, COMPUTE_SHADER ->
// COMPUTE_SHADER) between each adjacent pair of dispatches, and none after the
// last -- rather than a per-buffer barrier set: "minimal, not per-dispatch"
// reads as one coarse dependency per PASS BOUNDARY, not one finely-scoped
// barrier per buffer per dispatch.
//
// "ONE DISPATCH PER SCHEDULE SLOT" HAS NOT BEEN TRUE SINCE S6 TASK 7, AND IS
// FURTHER FROM IT AFTER TASK 8. The tally, per substep:
//
//     slot 0  MediumUpdate      1                    (per-WORLD grid)
//     slot 1  ForceElements     2                    rotors, then drag
//     slot 2  Gravity           0                    inert -- nothing recorded
//     slot 3  CollisionStatic   1
//     slot 4  CollisionDynamic  2 + sort_stage_count()
//     slot 5  Integrate         1
//     slot 6  SensorSynthesis   2                    sensor_imu, sensor_gnss (per-SENSOR grid)
//     slot 7  Publish           0                    inert -- nothing recorded
//                              ---
//                               9 + sort_stage_count()
//
// This table is documentation, not an input: record() finds the last dispatch
// from what it actually emitted, and recorded_chain() reports the counts
// (tests/test_gpu_state_mirror.cpp checks both against this tally).
//
// The barrier rule above is unchanged and covers all three kinds of hazard --
// between passes, between the sort chain's stages, and between rotors and drag
// inside slot 1 -- by the same clause, rather than special-casing any of them.
// ---------------------------------------------------------------------------
// TEN PIPELINES, ONE LAYOUT (four at S6 Task 6, three more at Task 7, three
// more at Task 8, less the retired stub, plus sensor_gnss). Every kernel this class dispatches is
// a distinct VkShaderModule/VkPipeline pair created from the SAME
// VkPipelineLayout -- they share one descriptor set layout (StateMirror's) and
// one push-constant range, because they read the same pool of bindings. The
// recorded chain binds whichever pipeline a dispatch needs before it; pipeline
// binds are cheap and recorded once, so the chain gains one vkCmdBindPipeline
// per dispatch and nothing else.
// ---------------------------------------------------------------------------
// THREE DISPATCH GRIDS, NOT ONE (S6 Task 8 adds the fourth kind of extent).
// Most passes run one thread per BODY SLOT; the CollisionDynamic chain runs its
// build and sort stages per KEY-ARRAY ENTRY and its sweep per WORLD;
// MediumUpdate runs per WORLD (reusing the sweep's grid); and SensorSynthesis
// runs per SENSOR SLOT, which is its own extent because `imu_sensors` has its
// own per-world capacity. See dispatch_groups_for()/groups_for() in the .cpp.
// ---------------------------------------------------------------------------
class StepRecorder {
public:
    // Every kernel this class dispatches. PUBLIC only so step_recorder.cpp's
    // file-scope slot->pipeline table (kPassPipeline, the one place the
    // schedule/pipeline correspondence is written down) can name them; nothing
    // outside this translation unit has any business with these values.
    //
    // kPipelineStub IS GONE (S6 Task 8): no slot binds a placeholder any more,
    // and the two inert slots record no dispatch at all. See this header's slot
    // table for the argument.
    enum PipelineSlot : uint32_t {
        kPipelineDrag = 0,       // forces_drag
        kPipelineCollision = 1,  // collision_static
        kPipelineIntegrate = 2,  // integrate
        // S6 Task 7's three: the CollisionDynamic slot is a CHAIN, so it needs
        // three pipelines rather than one. Kept in this enum (rather than a
        // second one) because they share the same layout, the same descriptor
        // set and the same push-constant range as every other kernel here.
        kPipelineGridBuild = 3,        // grid_build
        kPipelineGridSort = 4,         // grid_sort
        kPipelineCollisionDynamic = 5, // collision_dynamic
        // S6 Task 8's three, completing the schedule.
        kPipelineMedium = 6,     // medium_update
        kPipelineRotors = 7,     // rotors
        kPipelineSensorImu = 8,  // sensor_imu
        // The GPU-sensor leg's one addition: SensorSynthesis became a CHAIN
        // when a second sensor kind arrived, exactly as physics/schedule.hpp
        // predicted it would ("a second batched call HERE ... and not a second
        // pass"). Appended rather than inserted so no existing enumerator moves.
        kPipelineSensorGnss = 9, // sensor_gnss
        kPipelineCount = 10,
    };

    // NO RunParams ARGUMENT AS OF S6 TASK 6b: PassParams no longer carries
    // ContactParams/GridParams (its own doc comment above has the ruling), so
    // there is nothing left for a caller to pass in for that purpose.
    //
    // `workgroup_size` (S6 Task 9b) IS THE A7 KNOB, ARRIVING LIVE. It does two
    // things here and they must be the SAME number or the recording is invalid:
    // it SELECTS which compiled variant of each kernel becomes a pipeline
    // (compute/spirv_variants.hpp), and it is the DIVISOR every dispatch grid
    // below is computed with. Passed explicitly rather than folded into
    // StepShape because it is backend CONFIGURATION, not shape -- two
    // Simulations with identical capacities and different workgroup sizes have
    // the same shape, and StepShape's own doc comment draws that line for
    // ContactParams/GridParams already.
    //
    // Rejected with Code::invalid_argument if it is not in
    // compute/backend.hpp's kSupportedWorkgroupSizes (no kernel variant was
    // compiled for it) or if the device's maxComputeWorkGroupInvocations /
    // maxComputeWorkGroupSize[0] cannot run it.
    [[nodiscard]] static Result<std::unique_ptr<StepRecorder>> create(VulkanContext& ctx,
                                                                        const StepShape& shape,
                                                                        VkDescriptorSetLayout set_layout,
                                                                        VkDescriptorSet set,
                                                                        void* step_params_mapped,
                                                                        uint32_t workgroup_size);

    ~StepRecorder();
    StepRecorder(const StepRecorder&) = delete;
    StepRecorder& operator=(const StepRecorder&) = delete;
    // Not movable: only ever held by unique_ptr (VulkanBackend::Impl). A
    // hand-written move had to name every member and silently skipped one.
    StepRecorder(StepRecorder&&) = delete;
    StepRecorder& operator=(StepRecorder&&) = delete;

    // Submits the ALREADY-RECORDED per-substep dispatch chain `n` times (one
    // submit per step, the SAME VkCommandBuffer every time -- no
    // vkResetCommandBuffer, no re-recording). Before EACH submit, writes
    // StepParams{.tick = first_tick + step_index} into the persistently
    // mapped, HOST_COHERENT step-params buffer (no explicit flush needed)
    // and blocks (fence wait) until that submit completes before moving to
    // the next -- so last_written_tick() below always reflects exactly the
    // value the most recently completed submit carried.
    [[nodiscard]] Result<void> submit(uint64_t n, uint64_t first_tick);

    // The tick most recently written into the step-params buffer -- what
    // Task 5's C2 fix test reads to prove "the host-side write happens per
    // submit with the right value". Safe to read with no synchronization:
    // only the CPU ever writes this buffer (nothing on the GPU writes back to
    // it), so re-reading a HOST_COHERENT mapping the CPU itself just wrote
    // needs no barrier or fence wait beyond the one submit() already
    // performed. 0 before the first submit().
    //
    // ITS DEVICE-SIDE COUNTERPART IS StepWitness (compute/step_params.hpp),
    // read back through StateMirror::read_step_witness(): this accessor says
    // what the host WROTE, the witness says what a kernel READ, and only the
    // two together close S6 Task 5's deferred half.
    [[nodiscard]] uint64_t last_written_tick() const noexcept;

    // ---------------------------------------------------------------------
    // S6 TASK 10 -- per-pass GPU timing (compute/vulkan/timestamps.hpp's
    // PassTimestamps), created unconditionally alongside every other
    // shape-derived object in create() and recorded into the SAME command
    // buffer this class already records once. "Unconditionally" means there
    // is no separate opt-in knob here: a device/queue that cannot time
    // compute work makes PassTimestamps itself report !supported() (its own
    // class comment's skip-gracefully posture) and every recording call a
    // no-op, so nothing about this class's public surface or existing
    // callers changes -- see this task's report for why that is the
    // "zero-cost when off" contract rather than a caller-visible toggle.
    //
    // Returns the durations of the SUBMIT most recently completed by
    // submit() above -- read the query pool's own doc comment
    // (PassTimestamps::read_durations_ns()) for why that is a safe thing to
    // read any time after submit() has returned.
    [[nodiscard]] Result<PassDurationsNs> pass_durations_ns() const;

    // What record() put into the command buffer (compute/backend.hpp).
    [[nodiscard]] RecordedChain recorded_chain() const noexcept { return chain_; }

private:
    StepRecorder() = default;
    void destroy() noexcept;

    // Records the fixed per-substep dispatch chain into `cmd_` ONCE -- called
    // from create(), never again. Binds the descriptor set once, then for
    // each substep/pass pair binds that slot's pipeline, pushes a PassParams
    // (substep baked in, shape- and run-constant otherwise) and dispatches,
    // with a barrier between every adjacent pair short of the very last
    // dispatch.
    //
    // NO LONGER 8 DISPATCHES PER SUBSTEP as of S6 Task 7: the CollisionDynamic
    // slot expands into 2 + sort_stages() dispatches (build, one per bitonic
    // stage, sweep). The chain is still recorded exactly once for the object's
    // life -- a multi-dispatch pass with barriers between its stages is still
    // a recording, and every stage's (k, j) is shape data baked into its own
    // push constant.
    [[nodiscard]] Result<void> record();

    // How many bitonic stages the recorded chain carries for one
    // CollisionDynamic pass: log2(segment) * (log2(segment) + 1) / 2, i.e. the
    // (k, j) pairs of grid_sort.slang's textbook double loop. 0 when the
    // segment holds a single entry (nothing to sort).
    [[nodiscard]] uint32_t sort_stage_count() const noexcept;

    // Builds one VkShaderModule + VkPipeline from the variant of `variants`
    // compiled for workgroup_size_ (S6 Task 9b -- it used to take one module's
    // words directly, because there was only ever one). Fails with
    // Code::invalid_argument, naming the kernel, if that set has no such
    // variant: dispatching a module compiled for one local size on a grid
    // divided by another is a Vulkan valid-usage violation whose symptom is
    // wrong physics rather than an error, so it must never be reachable.
    //
    // Every pipeline shares pipeline_layout_ (see the class note above).
    [[nodiscard]] Result<void> make_pipeline(const SpirvVariantSet& variants,
                                              VkShaderModule& out_module, VkPipeline& out_pipeline);

    // No VkPhysicalDevice member as of S6 Task 6: this class allocates no
    // memory of its own any more (StateMirror owns the step-params buffer, the
    // one allocation that used to need a physical device here), so keeping the
    // handle would be a field nothing reads.
    VkDevice device_ = VK_NULL_HANDLE;       // non-owning; VulkanContext outlives this object
    VkQueue queue_ = VK_NULL_HANDLE;         // non-owning
    VkPipelineLayout pipeline_layout_ = VK_NULL_HANDLE;
    VkCommandPool pool_ = VK_NULL_HANDLE;
    VkCommandBuffer cmd_ = VK_NULL_HANDLE;
    VkFence fence_ = VK_NULL_HANDLE;
    VkDescriptorSet set_ = VK_NULL_HANDLE;   // non-owning; StateMirror owns the pool/set

    // Index order is PipelineSlot's, which is what kPassPipeline maps a
    // schedule slot onto.
    VkShaderModule shaders_[kPipelineCount] = {};
    VkPipeline pipelines_[kPipelineCount] = {};

    // StepParams (S6 Task 5 review round 1, C2): host-visible, persistently
    // mapped, written fresh before every submit. The BUFFER is now owned by
    // StateMirror (which is what makes it bindable -- see that class's
    // step_params_mapped() note); this is a non-owning pointer to its
    // mapping, and this class remains the only writer.
    void* step_params_mapped_ = nullptr;

    StepShape shape_{};

    // The A7 knob, resolved (S6 Task 9b). THE SINGLE SOURCE OF BOTH HALVES:
    // every pipeline above was built from the kernel variant compiled for this
    // value, and every *_groups_x_ below is ceil(extent / this value). They
    // cannot drift because neither is written anywhere else.
    uint32_t workgroup_size_ = 0;

    uint32_t dispatch_groups_x_ = 0;

    // S6 Task 7: the CollisionDynamic chain's own shape. `grid_` is
    // compute/grid_entry.hpp's GridDomain for `shape_` -- how many key-array
    // entries there are and how they are partitioned -- and the two group
    // counts are the dispatch grids for a per-ENTRY kernel (build, sort) and a
    // per-WORLD one (the sweep), neither of which is the per-body-slot grid
    // every other pass uses.
    GridDomain grid_{};
    uint32_t grid_groups_x_ = 0;   // ceil(grid_.entry_count / workgroup_size_)
    uint32_t world_groups_x_ = 0;  // ceil(shape_.world_count / workgroup_size_)

    // S6 Task 8: SensorSynthesis's own extent. NOT derivable from either grid
    // above -- `imu_sensors` has its own per-world capacity, which a world set
    // may size independently of its body capacity (a 100-body world with one
    // IMU is the corpus's own shape), so a pass dispatched over the body grid
    // would either miss sensors or run threads with no row to own.
    uint32_t sensor_groups_x_ = 0;  // ceil(world_count * sensor_capacity / workgroup_size_)

    // S6 Task 10: per-pass GPU timing, never null after a successful create()
    // (a device that cannot time compute work still gets a valid, merely
    // !supported() object -- see PassTimestamps's own class comment).
    std::unique_ptr<PassTimestamps> timestamps_;

    // Counted by record(), never written anywhere else.
    RecordedChain chain_{};
};

}  // namespace spade::compute
