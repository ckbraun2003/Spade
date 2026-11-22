#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>
#include <vector>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include "core/error.hpp"
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
    uint32_t index = 0;    // the ring index (1, 2, ... per sensor)
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

    // The vehicle's model, for a link that maps outputs to rotors.
    [[nodiscard]] const vehicles::ModelType& model() const noexcept;

private:
    LockstepHost() = default;

    Simulation* sim_ = nullptr;
    VehicleRef vehicle_{};
    HostConfig config_{};
    std::vector<SensorBinding> sensors_;
};

}  // namespace spade::fc_host
