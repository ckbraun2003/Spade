// camera_host.cpp -- 3.3's CameraHost. THIS COMMIT IS THE RED'S STUB: every call refuses, so the tests fail on behaviour; the GREEN commit builds it.
#include "sim/camera_host.hpp"

namespace spade {

namespace {
Error not_built() { return Error{Code::invalid_argument, "camera host: not built yet"}; }
}  // namespace

struct CameraHost::Impl {};

Result<CameraHost> CameraHost::open(Simulation&) { return std::unexpected(not_built()); }
Result<uint32_t> CameraHost::add_camera(BodyRef, const CameraSpec&) { return std::unexpected(not_built()); }
Result<void> CameraHost::reset_camera(uint32_t, const CameraReset&) { return std::unexpected(not_built()); }
Result<void> CameraHost::set_label(BodyRef, uint32_t) { return std::unexpected(not_built()); }
Result<void> CameraHost::set_keypoints(BodyRef, std::span<const glm::vec3>) { return std::unexpected(not_built()); }
Result<uint32_t> CameraHost::label_of(BodyRef) const { return std::unexpected(not_built()); }
Result<std::span<const glm::vec3>> CameraHost::keypoints_of(BodyRef) const { return std::unexpected(not_built()); }
Result<StepReport> CameraHost::step(uint64_t) { return std::unexpected(not_built()); }
Result<RotorReading> CameraHost::rotors(const VehicleRef&) const { return std::unexpected(not_built()); }
Result<void> CameraHost::set_rotor_info(const VehicleRef&, std::span<const RotorInfo>) { return std::unexpected(not_built()); }
std::vector<std::byte> CameraHost::save() const { return {}; }
Result<void> CameraHost::load(std::span<const std::byte>) { return std::unexpected(not_built()); }
int64_t CameraHost::sample_time_ns(uint64_t) const noexcept { return 0; }

}  // namespace spade
