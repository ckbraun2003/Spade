#include "fc_host/lockstep.hpp"

#include <string>

// RED stubs: the interface compiles and every call says it is not built, so the first test run fails on behaviour.

namespace spade::fc_host {

namespace {
[[nodiscard]] Error not_built(const char* what) { return Error{Code::unavailable, std::string("fc_host: ") + what + " is not built yet"}; }
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

Result<LockstepHost> LockstepHost::open(Simulation&, const VehicleRef&, std::span<const SensorBinding>, const HostConfig&) {
    return std::unexpected(not_built("open"));
}
uint64_t LockstepHost::tick() const noexcept { return 0; }
uint64_t LockstepHost::dt_ns() const noexcept { return 0; }
uint64_t LockstepHost::sim_time_ns() const noexcept { return 0; }
Result<void> LockstepHost::advance_to(uint64_t) { return std::unexpected(not_built("advance_to")); }
Result<std::vector<Sample>> LockstepHost::take_sensors() { return std::unexpected(not_built("take_sensors")); }
Result<Truth> LockstepHost::truth() const { return std::unexpected(not_built("truth")); }
Result<void> LockstepHost::command(std::span<const float>) { return std::unexpected(not_built("command")); }
const vehicles::ModelType& LockstepHost::model() const noexcept {
    static const vehicles::ModelType empty{};
    return empty;
}

}  // namespace spade::fc_host
