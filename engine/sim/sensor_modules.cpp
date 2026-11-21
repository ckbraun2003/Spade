// sim/sensor_modules.cpp -- RED stub: the opt-in barometer and magnetometer modules (filled in by the GREEN commit).
#include "sim/module.hpp"

namespace spade::modules {

ModuleDesc baro_module() { return {.name = "baro"}; }
ModuleDesc mag_module() { return {.name = "mag"}; }

}  // namespace spade::modules
