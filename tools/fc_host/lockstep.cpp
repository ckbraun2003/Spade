#include "fc_host/lockstep.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <string>
#include <utility>

#include "sensors/baro.hpp"
#include "sensors/gnss.hpp"
#include "sensors/imu.hpp"
#include "sensors/kinds.hpp"
#include "sensors/mag.hpp"

namespace spade::fc_host {

namespace {

[[nodiscard]] Error invalid(std::string what) { return Error{Code::invalid_argument, "fc_host: " + std::move(what)}; }

[[nodiscard]] const char* kind_name(SensorKind kind) noexcept {
    switch (kind) {
        case SensorKind::imu: return "imu";
        case SensorKind::gnss: return "gnss";
        case SensorKind::baro: return "baro";
        case SensorKind::mag: return "mag";
    }
    return "unknown";
}

// A bound sensor's emission period in substeps. Refuses a ref that names no live sensor of its kind.
[[nodiscard]] Result<uint32_t> rate_divider_of(const Simulation& sim, const SensorBinding& b, std::size_t index) {
    const std::string who = "sensor " + std::to_string(index) + " (" + kind_name(b.kind) + ")";
    switch (b.kind) {
        case SensorKind::imu: {
            const Result<const sensors::ImuSensorRow*> row = sim.imu_sensor(b.imu);
            if (!row) return std::unexpected(invalid(who + ": " + row.error().context));
            if ((*row)->kind != sensors::sensor_kind::imu) return std::unexpected(invalid(who + " is not live"));
            return (*row)->rate_divider;
        }
        case SensorKind::gnss: {
            const Result<std::span<const sensors::GnssSensorRow>> rows = sim.arenas().array(sim.gnss_sensors_array());
            if (!rows || b.gnss.slot >= rows->size()) return std::unexpected(invalid(who + ": no such slot"));
            const sensors::GnssSensorRow& row = (*rows)[b.gnss.slot];
            if (row.kind != sensors::sensor_kind::gnss) return std::unexpected(invalid(who + " is not live"));
            return row.rate_divider;
        }
        case SensorKind::baro: {
            const Result<std::span<const sensors::BaroSensorRow>> rows = sim.module_rows<sensors::BaroSensorRow>("baro_sensors", b.row.world_index);
            if (!rows) return std::unexpected(invalid(who + ": " + rows.error().context));
            if (b.row.slot >= rows->size() || (*rows)[b.row.slot].kind != sensors::sensor_kind::baro) return std::unexpected(invalid(who + " is not live"));
            return (*rows)[b.row.slot].rate_divider;
        }
        case SensorKind::mag: {
            const Result<std::span<const sensors::MagSensorRow>> rows = sim.module_rows<sensors::MagSensorRow>("mag_sensors", b.row.world_index);
            if (!rows) return std::unexpected(invalid(who + ": " + rows.error().context));
            if (b.row.slot >= rows->size() || (*rows)[b.row.slot].kind != sensors::sensor_kind::mag) return std::unexpected(invalid(who + " is not live"));
            return (*rows)[b.row.slot].rate_divider;
        }
    }
    return std::unexpected(invalid(who + ": a kind the host does not know"));
}

// Appends the samples of one poll to `out`, moves `since` to the newest returned, and returns how many samples the ring lost.
template <class T>
[[nodiscard]] uint64_t collect(const sensors::PollResult<T>& poll, SensorKind kind, uint32_t binding, uint64_t dt_ns, sensors::SampleIndex& since,
                               std::vector<Sample>& out) {
    static_assert(sizeof(T) <= kMaxSampleBytes);
    for (const T& s : poll.samples) {
        Sample sample;
        sample.kind = kind;
        sample.binding = binding;
        sample.tick = s.tick;
        sample.time_ns = (s.tick + 1) * dt_ns;
        sample.index = s.index;
        sample.size = static_cast<uint32_t>(sizeof(T));
        std::memcpy(sample.bytes.data(), &s, sizeof(T));
        out.push_back(sample);
    }
    since = poll.next_since;
    return poll.dropped;
}

// One baro or mag sensor's ring, by its module's array names.
template <class Sample_, class Row>
[[nodiscard]] Result<sensors::PollResult<Sample_>> poll_module(const Simulation& sim, const char* rows_name, const char* ring_name, const RowRef& ref,
                                                               sensors::SampleIndex since, std::span<Sample_> out) {
    const Result<std::span<const Row>> rows = sim.module_rows<Row>(rows_name, ref.world_index);
    const Result<std::span<const Sample_>> ring = sim.module_rows<Sample_>(ring_name, ref.world_index);
    if (!rows || !ring) return std::unexpected(invalid(std::string("the '") + rows_name + "' module is not in this world"));
    const std::span<const Sample_> window = ring->subspan(std::size_t{ref.slot} * sensors::kRingDepth, sensors::kRingDepth);
    return sensors::ring_poll<Sample_>(window, (*rows)[ref.slot].last_index, since, out);
}

}  // namespace

SensorBinding SensorBinding::of(ImuSensorRef ref) noexcept {
    SensorBinding b;
    b.kind = SensorKind::imu;
    b.imu = ref;
    return b;
}
SensorBinding SensorBinding::of(GnssSensorRef ref) noexcept {
    SensorBinding b;
    b.kind = SensorKind::gnss;
    b.gnss = ref;
    return b;
}
SensorBinding SensorBinding::baro(RowRef ref) noexcept {
    SensorBinding b;
    b.kind = SensorKind::baro;
    b.row = ref;
    return b;
}
SensorBinding SensorBinding::mag(RowRef ref) noexcept {
    SensorBinding b;
    b.kind = SensorKind::mag;
    b.row = ref;
    return b;
}

Result<LockstepHost> LockstepHost::open(Simulation& sim, const VehicleRef& vehicle, std::span<const SensorBinding> sensors, const HostConfig& config) {
    if (vehicle.is_null()) return std::unexpected(invalid("open: the vehicle ref is null"));
    if (const Result<const BodyState*> body = sim.body(vehicle.body); !body) {
        return std::unexpected(invalid("open: the vehicle is not live (" + body.error().context + ")"));
    }
    const Result<const vehicles::ModelType*> model = sim.model(vehicle.model);
    if (!model) return std::unexpected(invalid("open: the vehicle's model is not registered (" + model.error().context + ")"));

    const uint32_t substeps = sim.substeps();
    for (std::size_t i = 0; i < sensors.size(); ++i) {
        const Result<uint32_t> divider = rate_divider_of(sim, sensors[i], i);
        if (!divider) return std::unexpected(divider.error());
        if (*divider == 0 || *divider % substeps != 0) {
            return std::unexpected(invalid("open: sensor " + std::to_string(i) + " (" + kind_name(sensors[i].kind) + "): rate_divider " +
                                           std::to_string(*divider) + " is not a whole number of ticks (a tick is " + std::to_string(substeps) +
                                           " substeps), and a sample is stamped with the tick only"));
        }
    }

    LockstepHost host;
    host.sim_ = &sim;
    host.vehicle_ = vehicle;
    host.config_ = config;
    host.model_ = *model;
    host.sensors_.assign(sensors.begin(), sensors.end());
    host.since_.assign(sensors.size(), 0);
    return host;
}

uint64_t LockstepHost::tick() const noexcept { return sim_->tick().value; }
uint64_t LockstepHost::dt_ns() const noexcept { return sim_->dt_ns(); }
uint64_t LockstepHost::sim_time_ns() const noexcept { return tick() * dt_ns(); }

Result<void> LockstepHost::advance_to(uint64_t t_ns) {
    const uint64_t dt = dt_ns();
    if (t_ns < sim_time_ns()) {
        return std::unexpected(invalid("advance_to: " + std::to_string(t_ns) + " ns is in the past (now " + std::to_string(sim_time_ns()) + " ns)"));
    }
    if (t_ns % dt != 0) {
        return std::unexpected(invalid("advance_to: " + std::to_string(t_ns) + " ns is not on a tick boundary (dt " + std::to_string(dt) + " ns)"));
    }
    const uint64_t ticks = (t_ns - sim_time_ns()) / dt;
    for (uint64_t k = 0; k < ticks; ++k) {
        // Release every command whose time has come, oldest first: the last one released wins, as the last write to a row does.
        while (!pending_.empty() && pending_.front().release_tick <= tick()) {
            if (Result<void> r = sim_->set_rotor_commands(vehicle_, pending_.front().omega); !r) return r;
            pending_.pop_front();
        }
        if (Result<void> r = sim_->step(1); !r) return r;
    }
    return {};
}

Result<std::vector<Sample>> LockstepHost::take_sensors() {
    std::vector<Sample> out;
    const uint64_t dt = dt_ns();
    uint64_t dropped = 0;
    std::size_t dropped_at = 0;
    for (std::size_t i = 0; i < sensors_.size(); ++i) {
        const SensorBinding& b = sensors_[i];
        const uint32_t binding = static_cast<uint32_t>(i);
        uint64_t lost = 0;
        switch (b.kind) {
            case SensorKind::imu: {
                std::array<sensors::ImuSample, sensors::kRingDepth> buf;
                const Result<ImuPoll> poll = sim_->poll_imu(b.imu, since_[i], buf);
                if (!poll) return std::unexpected(invalid("take_sensors: sensor " + std::to_string(i) + ": " + poll.error().context));
                lost = collect(*poll, b.kind, binding, dt, since_[i], out);
                break;
            }
            case SensorKind::gnss: {
                std::array<sensors::GnssFix, sensors::kRingDepth> buf;
                const Result<GnssPoll> poll = sim_->poll_gnss(b.gnss, since_[i], buf);
                if (!poll) return std::unexpected(invalid("take_sensors: sensor " + std::to_string(i) + ": " + poll.error().context));
                lost = collect(*poll, b.kind, binding, dt, since_[i], out);
                break;
            }
            case SensorKind::baro: {
                std::array<sensors::BaroSample, sensors::kRingDepth> buf;
                const Result<sensors::PollResult<sensors::BaroSample>> poll =
                    poll_module<sensors::BaroSample, sensors::BaroSensorRow>(*sim_, "baro_sensors", "baro_ring", b.row, since_[i], buf);
                if (!poll) return std::unexpected(poll.error());
                lost = collect(*poll, b.kind, binding, dt, since_[i], out);
                break;
            }
            case SensorKind::mag: {
                std::array<sensors::MagSample, sensors::kRingDepth> buf;
                const Result<sensors::PollResult<sensors::MagSample>> poll =
                    poll_module<sensors::MagSample, sensors::MagSensorRow>(*sim_, "mag_sensors", "mag_ring", b.row, since_[i], buf);
                if (!poll) return std::unexpected(poll.error());
                lost = collect(*poll, b.kind, binding, dt, since_[i], out);
                break;
            }
        }
        if (lost != 0 && dropped == 0) dropped_at = i;
        dropped += lost;
    }
    if (dropped != 0) {
        // The cursors have moved past what was lost, so the next call carries on from the ring's oldest resident sample.
        return std::unexpected(invalid("take_sensors: dropped " + std::to_string(dropped) + " sample(s) (first at sensor " + std::to_string(dropped_at) +
                                       "): the 64-deep ring wrapped between two polls; advance in whole sensor periods"));
    }
    std::stable_sort(out.begin(), out.end(), [](const Sample& a, const Sample& b) { return a.tick != b.tick ? a.tick < b.tick : a.binding < b.binding; });
    return out;
}

Result<Truth> LockstepHost::truth() const {
    const Result<const BodyState*> body = sim_->body(vehicle_.body);
    if (!body) return std::unexpected(invalid("truth: " + body.error().context));
    Truth t;
    t.pos = (*body)->pos;
    t.vel = (*body)->vel;
    t.orient = (*body)->orient;
    t.omega_body = (*body)->omega_body;
    return t;
}

Result<void> LockstepHost::command(std::span<const float> omega) {
    if (omega.size() != model_->rotors.size()) {
        return std::unexpected(invalid("command: " + std::to_string(omega.size()) + " values for " + std::to_string(model_->rotors.size()) + " rotors"));
    }
    for (std::size_t i = 0; i < omega.size(); ++i) {
        if (!std::isfinite(omega[i]) || omega[i] < 0.0f) {
            return std::unexpected(invalid("command: rotor " + std::to_string(i) + " is " + std::to_string(omega[i]) + " (must be finite and >= 0)"));
        }
    }
    Pending p;
    p.release_tick = tick() + config_.command_latency_ticks;
    p.omega.assign(omega.begin(), omega.end());
    pending_.push_back(std::move(p));
    return {};
}

// RED stubs: the trace API compiles and says it is not built.
uint64_t LockstepHost::trace_ticks() const noexcept { return 0; }
std::string LockstepHost::trace_hash() const { return {}; }
Result<std::vector<std::byte>> LockstepHost::trace_bytes() const { return std::unexpected(Error{Code::unavailable, "fc_host: trace_bytes is not built yet"}); }
std::vector<std::byte> LockstepHost::commands_bytes() const { return {}; }
Result<void> LockstepHost::replay_commands(std::span<const std::byte>, uint64_t) {
    return std::unexpected(Error{Code::unavailable, "fc_host: replay_commands is not built yet"});
}

const vehicles::ModelType& LockstepHost::model() const noexcept { return *model_; }

}  // namespace spade::fc_host
