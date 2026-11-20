#include "sensors/camera.hpp"

namespace spade::sensors {

// RED stubs (Phase 3.1): the behaviour is not built yet.
bool camera_advance(CameraSensorRow& /*row*/) noexcept { return false; }

std::string camera_row_issue(const CameraSensorRow& /*row*/) { return {}; }

}  // namespace spade::sensors
