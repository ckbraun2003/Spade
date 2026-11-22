#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <memory>
#include <span>
#include <string>
#include <vector>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include "core/error.hpp"
#include "sensors/rings.hpp"
#include "sim/simulation.hpp"
#include "vehicles/model_type.hpp"

// ---------------------------------------------------------------------------
// fc_host/lockstep.hpp -- the lockstep host (docs/design/core/plans/2026-11-21-fc-core.md section 2). One vehicle of one Simulation, driven the way a
// flight computer's simulator link drives it: advance the world to a time, take the sensor samples that time produced, hand the commands back.
//
//   * THE CLOCK. sim_time_ns() = tick() * dt_ns(), the one clock; nothing else counts time. A sample stamped tick k is the state at the END of step
//     k, so its time is (k + 1) * dt_ns.
//   * WHOLE TICKS. A sensor's period (rate_divider substeps) must be a whole number of ticks, because a sample is stamped with the tick only: a
//     shorter period would put two samples on one stamp. open() refuses a sensor that is not, naming it and its numbers.
//   * NOTHING DROPPED. take_sensors() refuses a poll that lost samples (the rings are 64 deep); advance_to() steps tick by tick, so a caller that
//     advances a whole sensor period at a time cannot lose one.
//   * NO WALL CLOCK, NO RANDOMNESS OF ITS OWN. The host never reads a clock and draws from no stream: the same world, seed and command stream give
//     the same samples, which is what replay (fc_host/replay) relies on.
//
// The host is in-process C++ on the public engine headers; fc_link includes it and it never includes fc_link. It needs no engine call and no C ABI.
// ---------------------------------------------------------------------------

namespace spade::fc_host {

// The sensor kinds a host can bind; the numbers are sensors::sensor_kind's.
enum class SensorKind : uint32_t { imu = 1, gnss = 2, baro = 4, mag = 5 };

// One sensor the host reads, by the ref its spawn returned.
struct SensorBinding {
    SensorKind kind = SensorKind::imu;
    ImuSensorRef imu{};    // kind imu
    GnssSensorRef gnss{};  // kind gnss
    RowRef row{};          // kind baro or mag (Simulation::attach_row)

    [[nodiscard]] static SensorBinding of(ImuSensorRef ref) noexcept;
    [[nodiscard]] static SensorBinding of(GnssSensorRef ref) noexcept;
    [[nodiscard]] static SensorBinding baro(RowRef ref) noexcept;
    [[nodiscard]] static SensorBinding mag(RowRef ref) noexcept;
};

// The largest engine sample (ImuSample and GnssFix are 48 bytes; BaroSample and MagSample 32).
inline constexpr std::size_t kMaxSampleBytes = 48;

// One new sample of one bound sensor.
struct Sample {
    SensorKind kind = SensorKind::imu;
    uint32_t binding = 0;  // the index in the span passed to open()
    uint64_t tick = 0;     // the engine's stamp
    uint64_t time_ns = 0;  // (tick + 1) * dt_ns
    uint64_t index = 0;    // the ring index (1, 2, ... per sensor), sensors::SampleIndex
    uint32_t size = 0;     // bytes of `bytes` in use: the engine's sample struct, as stored
    std::array<std::byte, kMaxSampleBytes> bytes{};
};

// The vehicle's ground truth at the current tick.
struct Truth {
    glm::vec3 pos{0.0f};
    glm::vec3 vel{0.0f};
    glm::quat orient{1.0f, 0.0f, 0.0f, 0.0f};  // body to world, glm order (w, x, y, z as constructed)
    glm::vec3 omega_body{0.0f};
};

struct HostConfig {
    // Whole ticks between command() and the first tick that sees it. 0: the next tick. A real link adds its own latency on top.
    uint64_t command_latency_ticks = 0;
};

// ---------------------------------------------------------------------------
// THE TRACE (docs/design/core/plans/2026-11-21-fc-core.md section 4). Every tick the host records the vehicle's state at the END of the tick, the
// rotors' applied command, actual shaft speed and blade azimuth, and the sensor samples that tick produced, and folds them into a rolling hash. The
// trace is the host's own account of the run: the same world, seed and applied commands give the same bytes, which is what replay checks.
//
// trace.bin v1, little-endian, no padding, every offset named here (Rendering's reader is written against this):
//   header, 192 bytes: 0 "SPTR"; 4 u32 version 1; 8 u32 header_size; 12 u32 flags 0; 16 u64 dt_ns; 24 u32 substeps; 28 u32 rotor_count; 32 u64 tick_count;
//     40 u64 first_tick; 48 u32 tick_record_size; 52 u32 sample_header_size (16); 56 u64 tick_section_offset; 64 u64 sample_section_offset; 72 u64 sample_count;
//     80 u32 payload bytes of an imu / gnss / baro / mag sample (80..96); 96 char[64] trace_hash (lower-case hex); 160..192 zero.
//   tick record (76 + 12 n bytes, n = rotor_count): 0 u64 tick; 8 u64 time_ns = (tick + 1) * dt_ns; 16 f32[3] pos; 28 f32[3] vel; 40 f32[4] orient (w, x, y, z);
//     56 f32[3] omega_body; 68 f32[n] rotor_cmd (applied); 68+4n u32 sample_first; 72+4n u32 sample_count; 76+4n f32[n] omega (actual shaft speed, rad/s);
//     76+8n f32[n] psi (blade azimuth, rad in [0, 2 pi), physics/blade_azimuth.hpp's rule with h = dt, held on the host, outside the registered state).
//   sample section: sample_count records of {u32 kind, u32 binding, u32 index, u32 0} + the engine sample struct's bytes (the kind's payload size).
//   trailer, the last 64 bytes: lower-case hex SHA-256 of every byte before it.
// trace_hash: h0 = sha256_hex("spade.trace.v1"); h_k = sha256_hex(h_{k-1} + tick record bytes + that tick's sample records' bytes); the last h.
// commands.bin v1: 0 "SPCM"; 4 u32 version 1; 8 u32 rotor_count; 12 u32 0; 16 u64 count; then count records {u64 tick, f32 omega[n]}: the command the rotors
// were given BEFORE that tick's step (after any latency), in tick order. It is the input of a replay.
// ---------------------------------------------------------------------------
inline constexpr uint32_t kTraceVersion = 1;
inline constexpr uint32_t kTraceHeaderBytes = 192;
inline constexpr uint32_t kTraceTrailerBytes = 64;
inline constexpr uint32_t kTickRecordFixedBytes = 76;
inline constexpr uint32_t kSampleHeaderBytes = 16;
inline constexpr uint32_t kCommandsHeaderBytes = 24;
[[nodiscard]] inline constexpr uint32_t tick_record_bytes(uint32_t rotor_count) noexcept { return kTickRecordFixedBytes + 12u * rotor_count; }

class LockstepHost {
public:
    // Binds `vehicle` and `sensors` of `sim`. The Simulation must outlive the host. Refusals, each naming what is wrong: a null or stale vehicle ref;
    // a sensor ref that is not live; a sensor whose period is not a whole number of ticks; a binding kind the host does not know.
    [[nodiscard]] static Result<LockstepHost> open(Simulation& sim, const VehicleRef& vehicle, std::span<const SensorBinding> sensors,
                                                   const HostConfig& config = {});

    [[nodiscard]] uint64_t tick() const noexcept;     // the Simulation's tick counter
    [[nodiscard]] uint64_t dt_ns() const noexcept;    // one tick
    [[nodiscard]] uint64_t sim_time_ns() const noexcept;  // tick() * dt_ns()

    // Steps tick by tick until sim_time_ns() == t_ns. Releases the commands whose time has come before every tick. Refuses a t_ns that is in the past
    // or not on a tick boundary.
    [[nodiscard]] Result<void> advance_to(uint64_t t_ns);

    // Every new sample of every bound sensor since the last call, ordered by (tick, binding), each with its time. Refuses a poll that lost samples.
    [[nodiscard]] Result<std::vector<Sample>> take_sensors();

    [[nodiscard]] Result<Truth> truth() const;

    // Queues a rotor command, rad/s per rotor in the model's declaration order. Refuses a wrong count, a non-finite or a negative value.
    [[nodiscard]] Result<void> command(std::span<const float> omega);

    // The trace so far (above): ticks recorded, the rolling hash (64 hex), trace.bin and commands.bin as bytes. The host polls every bound sensor after
    // every tick itself, so the trace holds each sample once whenever the caller takes them, and a late take_sensors() loses nothing.
    [[nodiscard]] uint64_t trace_ticks() const noexcept;
    [[nodiscard]] std::string trace_hash() const;
    [[nodiscard]] Result<std::vector<std::byte>> trace_bytes() const;
    [[nodiscard]] std::vector<std::byte> commands_bytes() const;

    // REPLAY: steps `ticks` ticks applying the commands of a commands.bin at the ticks it names, with no autopilot and no queue; the trace it records is
    // the live run's, byte for byte, when the world and seed are the same. Refuses a file that is short, has another magic or version, another rotor count,
    // a record in the past, or ticks out of order, naming what.
    [[nodiscard]] Result<void> replay_commands(std::span<const std::byte> commands, uint64_t ticks);

    // The vehicle's model, for a link that maps outputs to rotors.
    [[nodiscard]] const vehicles::ModelType& model() const noexcept;

private:
    LockstepHost() = default;

    [[nodiscard]] Result<void> apply_now(std::span<const float> omega);  // set the rotors before the coming step and log it for commands.bin
    [[nodiscard]] Result<void> step_and_record();                       // one tick, then its trace record
    [[nodiscard]] Result<void> poll_all(std::vector<Sample>& out);      // every bound sensor since its cursor

    // A command waiting for its release tick (the tick whose step is the first to see it).
    struct Pending {
        uint64_t release_tick = 0;
        std::vector<float> omega;
    };

    Simulation* sim_ = nullptr;
    VehicleRef vehicle_{};
    HostConfig config_{};
    const vehicles::ModelType* model_ = nullptr;
    std::vector<SensorBinding> sensors_;
    std::vector<sensors::SampleIndex> since_;  // per binding: the newest index take_sensors() has returned
    std::deque<Pending> pending_;
    std::vector<Sample> inbox_;   // samples polled after each tick, waiting for take_sensors()
    struct Recorded;              // the trace (lockstep.cpp)
    std::shared_ptr<Recorded> recorded_;
};

}  // namespace spade::fc_host
