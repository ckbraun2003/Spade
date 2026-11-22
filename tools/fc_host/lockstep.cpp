#include "fc_host/lockstep.hpp"

#include <algorithm>
#include <bit>
#include <cmath>
#include <cstring>
#include <string>
#include <string_view>
#include <utility>

#include "core/rng.hpp"
#include "physics/blade_azimuth.hpp"
#include "sensors/baro.hpp"
#include "sensors/gnss.hpp"
#include "sensors/imu.hpp"
#include "sensors/kinds.hpp"
#include "sensors/mag.hpp"
#include "world/sha256.hpp"

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

// Little-endian writers, one byte at a time: the file's bytes never depend on the host's byte order.
void put_u32(std::vector<std::byte>& out, uint32_t v) {
    for (int i = 0; i < 4; ++i) out.push_back(static_cast<std::byte>((v >> (8 * i)) & 0xFFu));
}
void put_u64(std::vector<std::byte>& out, uint64_t v) {
    for (int i = 0; i < 8; ++i) out.push_back(static_cast<std::byte>((v >> (8 * i)) & 0xFFu));
}
void put_f32(std::vector<std::byte>& out, float v) { put_u32(out, std::bit_cast<uint32_t>(v)); }

[[nodiscard]] uint32_t get_u32(std::span<const std::byte> in, std::size_t at) {
    uint32_t v = 0;
    for (int i = 3; i >= 0; --i) v = (v << 8) | static_cast<uint32_t>(std::to_integer<uint8_t>(in[at + static_cast<std::size_t>(i)]));
    return v;
}
[[nodiscard]] uint64_t get_u64(std::span<const std::byte> in, std::size_t at) {
    uint64_t v = 0;
    for (int i = 7; i >= 0; --i) v = (v << 8) | static_cast<uint64_t>(std::to_integer<uint8_t>(in[at + static_cast<std::size_t>(i)]));
    return v;
}

[[nodiscard]] std::string_view text_of(std::span<const std::byte> b) { return std::string_view(reinterpret_cast<const char*>(b.data()), b.size()); }

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

[[nodiscard]] Result<void> check_sigmas(const BootConfig& c, const char* who) {
    for (const float sigma : {c.sigma_accel, c.sigma_gyro, c.sigma_pressure_pa, c.sigma_mag_ut}) {
        if (!std::isfinite(sigma) || sigma < 0.0f) return std::unexpected(invalid(std::string(who) + ": a sigma must be finite and >= 0 (got " + std::to_string(sigma) + ")"));
    }
    return {};
}

constexpr std::string_view kBootStreamTag = "spade.fc_host.boot";
constexpr std::string_view kHashSeedText = "spade.trace.v1";
constexpr std::string_view kTraceMagic = "SPTR";
constexpr std::string_view kCommandsMagic = "SPCM";

}  // namespace

// The trace as recorded: the tick records and the sample records as the file will hold them, the chain hash, the blade azimuths and the commands
// applied. Shared by copies of the host (a host is movable, never copied in practice).
struct LockstepHost::Recorded {
    uint32_t rotor_count = 0;
    uint64_t dt_ns = 0;
    uint32_t substeps = 0;
    uint64_t first_tick = 0;
    uint64_t ticks = 0;
    uint64_t samples = 0;
    std::vector<std::byte> tick_bytes;
    std::vector<std::byte> sample_bytes;
    std::string hash;
    std::vector<float> psi;
    std::vector<std::byte> command_bytes;
    uint64_t commands = 0;
    bool boot_noted = false;
    uint64_t boot_steps = 0;
    uint64_t boot_end_time_usec = 0;
    BootConfig boot;
    std::vector<uint64_t> periods_ticks;
    uint64_t world_seed = 0;
};

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
    std::vector<uint64_t> periods_ticks;
    for (std::size_t i = 0; i < sensors.size(); ++i) {
        const Result<uint32_t> divider = rate_divider_of(sim, sensors[i], i);
        if (!divider) return std::unexpected(divider.error());
        if (*divider == 0 || *divider % substeps != 0) {
            return std::unexpected(invalid("open: sensor " + std::to_string(i) + " (" + kind_name(sensors[i].kind) + "): rate_divider " +
                                           std::to_string(*divider) + " is not a whole number of ticks (a tick is " + std::to_string(substeps) +
                                           " substeps), and a sample is stamped with the tick only"));
        }
        periods_ticks.push_back(*divider / substeps);
    }
    const Result<const WorldParams*> world = sim.world_params(vehicle.body.world_index);
    if (!world) return std::unexpected(invalid("open: " + world.error().context));

    LockstepHost host;
    host.sim_ = &sim;
    host.vehicle_ = vehicle;
    host.config_ = config;
    host.model_ = *model;
    host.sensors_.assign(sensors.begin(), sensors.end());
    host.since_.assign(sensors.size(), 0);
    host.recorded_ = std::make_shared<Recorded>();
    Recorded& rec = *host.recorded_;
    rec.rotor_count = static_cast<uint32_t>((*model)->rotors.size());
    rec.dt_ns = sim.dt_ns();
    rec.substeps = substeps;
    rec.first_tick = sim.tick().value;
    rec.hash = sha256_hex(kHashSeedText);
    rec.periods_ticks = std::move(periods_ticks);
    rec.world_seed = (*world)->seed;
    rec.psi.assign(rec.rotor_count, 0.0f);
    return host;
}

uint64_t LockstepHost::tick() const noexcept { return sim_->tick().value; }
uint64_t LockstepHost::dt_ns() const noexcept { return sim_->dt_ns(); }
uint64_t LockstepHost::sim_time_ns() const noexcept { return tick() * dt_ns(); }

// Applies `omega` to the rotors before the coming tick's step, and logs it for commands.bin.
Result<void> LockstepHost::apply_now(std::span<const float> omega) {
    if (Result<void> r = sim_->set_rotor_commands(vehicle_, omega); !r) return r;
    Recorded& rec = *recorded_;
    put_u64(rec.command_bytes, tick());
    for (const float v : omega) put_f32(rec.command_bytes, v);
    ++rec.commands;
    return {};
}

// Steps ONE tick (the commands for it already applied), then records it: the state at the end of the tick, the rotors, the samples it produced.
Result<void> LockstepHost::step_and_record() {
    Recorded& rec = *recorded_;
    const uint64_t this_tick = tick();
    if (Result<void> r = sim_->step(1); !r) return r;

    // THE SAMPLES OF THIS TICK, polled now so the trace never depends on when the caller takes them.
    std::vector<Sample> fresh;
    if (Result<void> r = poll_all(fresh); !r) return r;

    // THE ROTORS: the applied command and the shaft speed after the step, and the blade azimuth by physics/blade_azimuth.hpp's one rule.
    const Result<std::span<const vehicles::RotorRow>> rows = sim_->arenas().array(sim_->rotors_array());
    if (!rows) return std::unexpected(invalid("step: " + rows.error().context));
    const float h = static_cast<float>(static_cast<double>(rec.dt_ns) * 1e-9);
    std::vector<float> cmd(rec.rotor_count, 0.0f);
    std::vector<float> omega(rec.rotor_count, 0.0f);
    for (uint32_t i = 0; i < rec.rotor_count; ++i) {
        const vehicles::RotorRow& row = (*rows)[vehicle_.rotor_slots[i]];
        cmd[i] = row.omega_cmd;
        omega[i] = row.omega;
        if (row.enabled != 0u) {
            if (!physics::advance_blade_azimuth(rec.psi[i], row.omega, h)) {
                return std::unexpected(invalid("step: rotor " + std::to_string(i) + ": the blade azimuth refused (omega " + std::to_string(row.omega) + ")"));
            }
        }
    }

    const Result<Truth> truth = this->truth();
    if (!truth) return std::unexpected(truth.error());

    // The sample records of this tick, then the tick record that points at them.
    std::vector<std::byte> sample_records;
    for (const Sample& s : fresh) {
        put_u32(sample_records, static_cast<uint32_t>(s.kind));
        put_u32(sample_records, s.binding);
        put_u32(sample_records, static_cast<uint32_t>(s.index));
        put_u32(sample_records, 0u);
        sample_records.insert(sample_records.end(), s.bytes.begin(), s.bytes.begin() + s.size);
    }
    std::vector<std::byte> tick_record;
    put_u64(tick_record, this_tick);
    put_u64(tick_record, (this_tick + 1) * rec.dt_ns);
    put_f32(tick_record, truth->pos.x);
    put_f32(tick_record, truth->pos.y);
    put_f32(tick_record, truth->pos.z);
    put_f32(tick_record, truth->vel.x);
    put_f32(tick_record, truth->vel.y);
    put_f32(tick_record, truth->vel.z);
    put_f32(tick_record, truth->orient.w);
    put_f32(tick_record, truth->orient.x);
    put_f32(tick_record, truth->orient.y);
    put_f32(tick_record, truth->orient.z);
    put_f32(tick_record, truth->omega_body.x);
    put_f32(tick_record, truth->omega_body.y);
    put_f32(tick_record, truth->omega_body.z);
    for (const float v : cmd) put_f32(tick_record, v);
    put_u32(tick_record, static_cast<uint32_t>(rec.samples));
    put_u32(tick_record, static_cast<uint32_t>(fresh.size()));
    for (const float v : omega) put_f32(tick_record, v);
    for (const float v : rec.psi) put_f32(tick_record, v);

    std::string chained = rec.hash;
    chained.append(text_of(tick_record));
    chained.append(text_of(sample_records));
    rec.hash = sha256_hex(chained);

    rec.tick_bytes.insert(rec.tick_bytes.end(), tick_record.begin(), tick_record.end());
    rec.sample_bytes.insert(rec.sample_bytes.end(), sample_records.begin(), sample_records.end());
    rec.samples += fresh.size();
    ++rec.ticks;
    inbox_.insert(inbox_.end(), fresh.begin(), fresh.end());
    return {};
}

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
            if (Result<void> r = apply_now(pending_.front().omega); !r) return r;
            pending_.pop_front();
        }
        if (Result<void> r = step_and_record(); !r) return r;
    }
    return {};
}

Result<void> LockstepHost::poll_all(std::vector<Sample>& out) {
    const uint64_t dt = dt_ns();
    for (std::size_t i = 0; i < sensors_.size(); ++i) {
        const SensorBinding& b = sensors_[i];
        const uint32_t binding = static_cast<uint32_t>(i);
        uint64_t lost = 0;
        switch (b.kind) {
            case SensorKind::imu: {
                std::array<sensors::ImuSample, sensors::kRingDepth> buf;
                const Result<ImuPoll> poll = sim_->poll_imu(b.imu, since_[i], buf);
                if (!poll) return std::unexpected(invalid("poll: sensor " + std::to_string(i) + ": " + poll.error().context));
                lost = collect(*poll, b.kind, binding, dt, since_[i], out);
                break;
            }
            case SensorKind::gnss: {
                std::array<sensors::GnssFix, sensors::kRingDepth> buf;
                const Result<GnssPoll> poll = sim_->poll_gnss(b.gnss, since_[i], buf);
                if (!poll) return std::unexpected(invalid("poll: sensor " + std::to_string(i) + ": " + poll.error().context));
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
        // One tick cannot wrap a 64-deep ring; if it did, something other than this host stepped the world.
        if (lost != 0) {
            return std::unexpected(Error{Code::internal, "fc_host: sensor " + std::to_string(i) + " lost " + std::to_string(lost) +
                                                             " sample(s) inside one tick: the Simulation was stepped outside the host"});
        }
    }
    return {};
}

Result<std::vector<Sample>> LockstepHost::take_sensors() {
    std::vector<Sample> out;
    out.swap(inbox_);
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

uint64_t LockstepHost::trace_ticks() const noexcept { return recorded_->ticks; }
std::string LockstepHost::trace_hash() const { return recorded_->hash; }

Result<std::vector<std::byte>> LockstepHost::trace_bytes() const {
    const Recorded& rec = *recorded_;
    std::vector<std::byte> out;
    out.reserve(kTraceHeaderBytes + rec.tick_bytes.size() + rec.sample_bytes.size() + kTraceTrailerBytes);
    for (const char c : kTraceMagic) out.push_back(static_cast<std::byte>(c));
    put_u32(out, kTraceVersion);
    put_u32(out, kTraceHeaderBytes);
    put_u32(out, 0u);
    put_u64(out, rec.dt_ns);
    put_u32(out, rec.substeps);
    put_u32(out, rec.rotor_count);
    put_u64(out, rec.ticks);
    put_u64(out, rec.first_tick);
    put_u32(out, tick_record_bytes(rec.rotor_count));
    put_u32(out, kSampleHeaderBytes);
    put_u64(out, kTraceHeaderBytes);
    put_u64(out, kTraceHeaderBytes + rec.tick_bytes.size());
    put_u64(out, rec.samples);
    put_u32(out, static_cast<uint32_t>(sizeof(sensors::ImuSample)));
    put_u32(out, static_cast<uint32_t>(sizeof(sensors::GnssFix)));
    put_u32(out, static_cast<uint32_t>(sizeof(sensors::BaroSample)));
    put_u32(out, static_cast<uint32_t>(sizeof(sensors::MagSample)));
    for (const char c : rec.hash) out.push_back(static_cast<std::byte>(c));
    put_u64(out, rec.boot_steps);
    put_u64(out, rec.boot_end_time_usec);
    put_u64(out, rec.boot_noted ? rec.boot.seed : 0u);
    while (out.size() < kTraceHeaderBytes) out.push_back(std::byte{0});
    out.insert(out.end(), rec.tick_bytes.begin(), rec.tick_bytes.end());
    out.insert(out.end(), rec.sample_bytes.begin(), rec.sample_bytes.end());
    const std::string trailer = sha256_hex(text_of(out));
    for (const char c : trailer) out.push_back(static_cast<std::byte>(c));
    return out;
}

std::vector<std::byte> LockstepHost::commands_bytes() const {
    const Recorded& rec = *recorded_;
    std::vector<std::byte> out;
    for (const char c : kCommandsMagic) out.push_back(static_cast<std::byte>(c));
    put_u32(out, 1u);
    put_u32(out, rec.rotor_count);
    put_u32(out, 0u);
    put_u64(out, rec.commands);
    out.insert(out.end(), rec.command_bytes.begin(), rec.command_bytes.end());
    return out;
}

Result<void> LockstepHost::replay_commands(std::span<const std::byte> commands, uint64_t ticks) {
    const uint32_t n = recorded_->rotor_count;
    if (commands.size() < kCommandsHeaderBytes) return std::unexpected(invalid("replay: the commands file is short (" + std::to_string(commands.size()) + " bytes)"));
    if (text_of(commands.first(4)) != kCommandsMagic) return std::unexpected(invalid("replay: the commands file has the wrong magic (not SPCM)"));
    if (get_u32(commands, 4) != 1u) return std::unexpected(invalid("replay: commands.bin version " + std::to_string(get_u32(commands, 4)) + " (this build reads 1)"));
    if (get_u32(commands, 8) != n) {
        return std::unexpected(invalid("replay: the commands file has " + std::to_string(get_u32(commands, 8)) + " rotors, the vehicle has " + std::to_string(n)));
    }
    const uint64_t count = get_u64(commands, 16);
    const std::size_t record = 8u + 4u * n;
    if (commands.size() != kCommandsHeaderBytes + count * record) {
        return std::unexpected(invalid("replay: the commands file is truncated or has trailing bytes (" + std::to_string(commands.size()) + " bytes for " +
                                       std::to_string(count) + " records of " + std::to_string(record) + ")"));
    }
    uint64_t previous = 0;
    for (uint64_t i = 0; i < count; ++i) {
        const uint64_t at = get_u64(commands, kCommandsHeaderBytes + i * record);
        if (at < tick()) return std::unexpected(invalid("replay: record " + std::to_string(i) + " is for tick " + std::to_string(at) + ", already past (now " + std::to_string(tick()) + ")"));
        if (i > 0 && at < previous) return std::unexpected(invalid("replay: record " + std::to_string(i) + " is out of tick order"));
        previous = at;
    }

    uint64_t next = 0;
    std::vector<float> omega(n);
    const uint64_t end = tick() + ticks;
    while (tick() < end) {
        while (next < count && get_u64(commands, kCommandsHeaderBytes + next * record) <= tick()) {
            for (uint32_t r = 0; r < n; ++r) omega[r] = std::bit_cast<float>(get_u32(commands, kCommandsHeaderBytes + next * record + 8 + 4 * r));
            if (Result<void> res = apply_now(omega); !res) return res;
            ++next;
        }
        if (Result<void> res = step_and_record(); !res) return res;
    }
    return {};
}

Result<BootReadings> LockstepHost::boot_readings(uint64_t boot_step, const BootConfig& config) const {
    if (tick() != 0) {
        return std::unexpected(invalid("boot_readings: the world is held only before tick 0 (this host is at tick " + std::to_string(tick()) + ")"));
    }
    if (Result<void> r = check_sigmas(config, "boot_readings"); !r) return std::unexpected(r.error());
    const Result<const BodyState*> body = sim_->body(vehicle_.body);
    if (!body) return std::unexpected(invalid("boot_readings: " + body.error().context));
    const Result<const WorldParams*> world = sim_->world_params(vehicle_.body.world_index);
    if (!world) return std::unexpected(invalid("boot_readings: " + world.error().context));
    const glm::quat orient = (*body)->orient;

    // The noise: a stream of its own, a pure function of (seed, boot_step), drawn in a fixed order whether or not a sensor is bound.
    rng::Stream stream = rng::make_stream(config.seed, kBootStreamTag, boot_step);
    glm::vec3 accel_noise;
    glm::vec3 gyro_noise;
    glm::vec3 mag_noise;
    for (int i = 0; i < 3; ++i) accel_noise[i] = stream.next_gauss();
    for (int i = 0; i < 3; ++i) gyro_noise[i] = stream.next_gauss();
    const float pressure_noise = stream.next_gauss();
    for (int i = 0; i < 3; ++i) mag_noise[i] = stream.next_gauss();

    BootReadings out;
    out.boot_step = boot_step;
    out.accel_body = glm::conjugate(orient) * (-(*world)->gravity) + config.sigma_accel * accel_noise;
    out.gyro_body = config.sigma_gyro * gyro_noise;
    out.position = (*body)->pos;
    out.velocity = (*body)->vel;
    for (const SensorBinding& b : sensors_) {
        if (b.kind == SensorKind::baro && !out.has_baro) {
            const Result<std::span<const sensors::BaroSensorRow>> rows = sim_->module_rows<sensors::BaroSensorRow>("baro_sensors", b.row.world_index);
            if (!rows || b.row.slot >= rows->size()) return std::unexpected(invalid("boot_readings: the barometer row is gone"));
            const sensors::BaroSensorRow& row = (*rows)[b.row.slot];
            const float height = row.world_origin_altitude_m + ((*body)->pos + orient * row.mount_pos).y;
            const sensors::BaroTruth truth = sensors::baro_truth(height, row.sea_level_pressure_pa, row.sea_level_temperature_k, row.lapse_k_per_m);
            out.has_baro = true;
            out.pressure_pa = truth.pressure_pa + config.sigma_pressure_pa * pressure_noise;
            out.temperature_c = truth.temperature_k - 273.15f;
        } else if (b.kind == SensorKind::mag && !out.has_mag) {
            const Result<std::span<const sensors::MagSensorRow>> rows = sim_->module_rows<sensors::MagSensorRow>("mag_sensors", b.row.world_index);
            if (!rows || b.row.slot >= rows->size()) return std::unexpected(invalid("boot_readings: the magnetometer row is gone"));
            const sensors::MagSensorRow& row = (*rows)[b.row.slot];
            out.has_mag = true;
            out.mag_body_ut = sensors::mag_truth(orient, row.mount_orient, row.field_world_ut) + config.sigma_mag_ut * mag_noise;
        }
    }
    return out;
}

Result<void> LockstepHost::note_boot(uint64_t boot_steps, uint64_t boot_end_time_usec, const BootConfig& config) {
    if (tick() != 0) return std::unexpected(invalid("note_boot: the boot ends at tick 0 (this host is at tick " + std::to_string(tick()) + ")"));
    if (Result<void> r = check_sigmas(config, "note_boot"); !r) return r;
    Recorded& rec = *recorded_;
    rec.boot_noted = true;
    rec.boot_steps = boot_steps;
    rec.boot_end_time_usec = boot_end_time_usec;
    rec.boot = config;
    return {};
}

uint32_t LockstepHost::substeps() const noexcept { return recorded_->substeps; }
std::span<const uint64_t> LockstepHost::sensor_periods_ticks() const noexcept { return recorded_->periods_ticks; }
uint64_t LockstepHost::world_seed() const noexcept { return recorded_->world_seed; }
const HostConfig& LockstepHost::config() const noexcept { return config_; }
std::optional<BootNote> LockstepHost::boot_note() const {
    const Recorded& rec = *recorded_;
    if (!rec.boot_noted) return std::nullopt;
    return BootNote{rec.boot_steps, rec.boot_end_time_usec, rec.boot};
}

const vehicles::ModelType& LockstepHost::model() const noexcept { return *model_; }

}  // namespace spade::fc_host
