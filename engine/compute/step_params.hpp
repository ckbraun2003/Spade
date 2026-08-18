#pragma once

// ---------------------------------------------------------------------------
// step_params.hpp (S6 Task 6) -- the two PER-STEP records that travel between
// the host and the device through a storage buffer rather than through the
// per-dispatch push constant, plus the one record that travels back.
//
// WHY THIS FILE EXISTS SEPARATELY FROM compute/vulkan/step_recorder.hpp, where
// StepParams was born (S6 Task 5, review round 1's finding C2). Both structs
// below are now MIRRORED in engine/shaders/shared/layouts.slang and therefore
// carry @cpp-type/@cpp-header directives, which means the generated header
// layout_check.gen.hpp #includes whatever file declares them -- and that header
// is compiled by compute/layout_check.cpp. step_recorder.hpp includes <volk.h>;
// pulling Vulkan into the layout-check TU purely to reach an 8-byte POD would
// have been a dependency nobody wanted to explain. This header names no Vulkan
// type at all, exactly like compute/backend.hpp one directory up, so the
// generated asserts cost nothing.
//
// THE SPLIT BETWEEN THESE AND PassParams, restated because it is the whole
// reason the buffer exists. PassParams (compute/vulkan/step_recorder.hpp) is a
// PUSH CONSTANT: its values are baked into the command buffer at RECORD time,
// which happens exactly once per shape, so nothing in it may vary across the n
// resubmits of that one recording. StepParams is written by the host into a
// HOST_COHERENT storage buffer immediately before each vkQueueSubmit, so it
// CAN vary -- the recorded command buffer references the buffer's slot, never
// its contents.
// ---------------------------------------------------------------------------

#include <cstddef>
#include <cstdint>
#include <type_traits>

namespace spade::compute {

// ---------------------------------------------------------------------------
// StepParams -- what the host writes before every submit. One field today: the
// tick of the step about to execute.
//
// TWO 32-BIT WORDS, NOT ONE uint64_t, AND THAT IS A DELIBERATE ABI CHOICE
// rather than an accident of transcription. Task 5's original struct spelled
// this `uint64_t tick`, which was correct for a buffer NOTHING on the device
// read. Task 6 is the first consumer, and a Slang mirror spelled `uint64_t`
// compiles to a SPIR-V module that requires OpCapability Int64: a device
// without shaderInt64 could not create the pipeline at all, and this program's
// correctness device (Intel Iris Plus) is not a device to gamble a hard
// requirement on for the sake of one counter. Two uint32 lanes need no
// capability beyond baseline Vulkan 1.0, and -- because they are written and
// read as WORDS rather than reinterpreted from a 64-bit store -- they carry no
// host-endianness assumption either. The two helpers below are the only
// sanctioned way to cross between this representation and a uint64_t tick.
//
// GROWTH: "whatever T6-T8 need later" (the C2 ruling's own words) appends
// fields here. The Slang mirror and the generated static_asserts make an
// append that forgets one side a build failure.
// ---------------------------------------------------------------------------
struct StepParams {
    uint32_t tick_lo = 0;  // low 32 bits of the step's tick
    uint32_t tick_hi = 0;  // high 32 bits
};

[[nodiscard]] inline constexpr StepParams step_params_from_tick(uint64_t tick) noexcept {
    return StepParams{static_cast<uint32_t>(tick & 0xFFFFFFFFu),
                      static_cast<uint32_t>(tick >> 32)};
}

[[nodiscard]] inline constexpr uint64_t step_params_tick(const StepParams& params) noexcept {
    return (static_cast<uint64_t>(params.tick_hi) << 32) | static_cast<uint64_t>(params.tick_lo);
}

static_assert(std::is_standard_layout_v<StepParams>);
static_assert(std::is_trivially_copyable_v<StepParams>);
static_assert(sizeof(StepParams) == 8, "StepParams is a CPU<->GPU wire contract");
static_assert(offsetof(StepParams, tick_lo) == 0);
static_assert(offsetof(StepParams, tick_hi) == 4);
static_assert(sizeof(StepParams::tick_lo) + sizeof(StepParams::tick_hi) == sizeof(StepParams),
              "StepParams has implicit padding: every byte must belong to a named field");

// ---------------------------------------------------------------------------
// StepWitness -- THE FAR SIDE OF THE StepParams MECHANISM, and the reason this
// task can claim the mechanism is proven rather than merely wired.
//
// Task 5 proved the HOST half of the per-step buffer: the write happens, once
// per submit, with the right value (StepRecorder::last_written_tick()). It
// explicitly deferred the DEVICE half -- "wiring it into bindings.slang and
// proving a kernel can read it is Task 6's job, the first task with a real
// consumer".
//
// TASK 6 HAD NO PHYSICAL CONSUMER, AND SAID SO PLAINLY. None of wave A's four
// passes needed the tick: Integrate, the drag half of ForceElements and
// CollisionStatic are all pure functions of (state, params, h), and Gravity is
// inert. Rather than leave the mechanism half-proven -- or invent a fake
// dependency inside a physics kernel, which would be worse -- the Integrate
// kernel publishes what it read into this dedicated, backend-internal witness
// row, and a test asserts the value. That was the "parity-support path" reading
// the brief sanctioned, declared rather than disguised.
//
// THE PHYSICAL CONSUMER ARRIVED AT S6 TASK 8, exactly where that note predicted
// it would: engine/shaders/kernels/sensor_imu.slang reads step_params and
// stamps `tick` into every emitted ImuSample, which
// tests/test_gpu_parity.cpp's quad_hover case then compares against the CPU's
// stamps BIT-EXACTLY (an integer row -- QuantityKind::bits). So the buffer is
// now proven twice over and by two different instruments: the witness says a
// kernel READ it, and 128 matching sample tick-stamps say a kernel USED it.
// THE WITNESS STAYS regardless -- it is the only check that survives a future
// scenario with no sensors in it.
//
// WHAT THE THREE FIELDS PROVE, EACH SEPARATELY:
//   * tick_lo/tick_hi -- the host's PER-SUBMIT write reached the device and was
//     read by a real kernel. Only a fresh read can produce the last step's tick
//     after n submits of one frozen command buffer.
//   * substep -- the PER-DISPATCH push constant (PassParams::substep, baked at
//     record time) is live in the same kernel. After a full step the witness
//     holds substeps-1, i.e. the LAST substep's dispatch wrote last, which also
//     pins the recorded chain's ordering.
//
// NOT REGISTERED STATE (global constraint: "S6 adds NO register_array call").
// This is backend-internal derived storage in exactly the category
// dryden_params already occupies -- it is never snapshotted, never digested,
// and a cpu-backend Simulation has none.
//
// WRITTEN BY EXACTLY ONE THREAD (global dispatch thread 0) of the Integrate
// dispatch, so it is a per-item write with no cross-lane reduction and no
// dependence on the workgroup size -- the shape Task 9's {32,64,128}
// invariance proof needs by construction.
// ---------------------------------------------------------------------------
struct StepWitness {
    uint32_t tick_lo = 0;  // low 32 bits of the tick the last dispatch read
    uint32_t tick_hi = 0;  // high 32 bits
    uint32_t substep = 0;  // PassParams::substep of the dispatch that wrote last
    uint32_t _r0 = 0;      // reserved; must stay 0
};

[[nodiscard]] inline constexpr uint64_t step_witness_tick(const StepWitness& w) noexcept {
    return (static_cast<uint64_t>(w.tick_hi) << 32) | static_cast<uint64_t>(w.tick_lo);
}

static_assert(std::is_standard_layout_v<StepWitness>);
static_assert(std::is_trivially_copyable_v<StepWitness>);
static_assert(sizeof(StepWitness) == 16, "StepWitness is a CPU<->GPU wire contract");
static_assert(offsetof(StepWitness, tick_lo) == 0);
static_assert(offsetof(StepWitness, tick_hi) == 4);
static_assert(offsetof(StepWitness, substep) == 8);
static_assert(offsetof(StepWitness, _r0) == 12);
static_assert(sizeof(StepWitness::tick_lo) + sizeof(StepWitness::tick_hi) +
                      sizeof(StepWitness::substep) + sizeof(StepWitness::_r0) ==
                  sizeof(StepWitness),
              "StepWitness has implicit padding: every byte must belong to a named field");

}  // namespace spade::compute
