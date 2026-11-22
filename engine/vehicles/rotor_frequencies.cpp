#include "vehicles/rotor_frequencies.hpp"

#include <cstdio>
#include <limits>

namespace spade::vehicles {

namespace {

constexpr double kTwoPi = 6.28318530717958647692;
constexpr uint32_t kMaxBlades = 16;
constexpr uint32_t kMaxPolePairs = 64;
constexpr uint32_t kCommutationsPerElectricalCycle = 6;  // six-step commutation

[[nodiscard]] bool finite(double v) noexcept {
    return v == v && v != std::numeric_limits<double>::infinity() && v != -std::numeric_limits<double>::infinity();
}

[[nodiscard]] const char* assumed_tag(bool assumed) noexcept {
    return assumed ? ", assumed" : "";
}

}  // namespace

Result<DerivedFrequencies> derive_frequencies(const FrequencyInputs& in) {
    if (!finite(in.omega) || in.omega < 0.0) {
        return std::unexpected(Error{Code::invalid_argument, "rotor_frequencies: omega must be finite and not negative"});
    }
    if (in.blades < 1 || in.blades > kMaxBlades) {
        return std::unexpected(Error{Code::invalid_argument, "rotor_frequencies: blades must be 1 to 16"});
    }
    if (in.pole_pairs > kMaxPolePairs) {
        return std::unexpected(Error{Code::invalid_argument, "rotor_frequencies: pole_pairs must be 0 to 64"});
    }
    if (!finite(in.pwm_hz) || in.pwm_hz < 0.0) {
        return std::unexpected(Error{Code::invalid_argument, "rotor_frequencies: pwm_hz must be finite and not negative"});
    }
    DerivedFrequencies f;
    f.shaft_hz = in.omega / kTwoPi;
    f.blade_pass_hz = static_cast<double>(in.blades) * f.shaft_hz;
    f.electrical_hz = static_cast<double>(in.pole_pairs) * f.shaft_hz;
    f.commutation_hz = static_cast<double>(kCommutationsPerElectricalCycle) * f.electrical_hz;
    f.pwm_hz = in.pwm_hz;
    f.blades = in.blades;
    f.pole_pairs = in.pole_pairs;
    f.blade_pass_assumed = in.blades_assumed;
    f.electrical_assumed = in.pole_pairs_assumed;
    f.pwm_assumed = in.pwm_assumed;
    return f;
}

void harmonics(double fundamental_hz, std::span<double> out) noexcept {
    for (std::size_t k = 0; k < out.size(); ++k) out[k] = static_cast<double>(k + 1) * fundamental_hz;
}

std::string frequency_readout(const DerivedFrequencies& f) {
    char line[160];
    std::string text;
    std::snprintf(line, sizeof line, "shaft %.2f Hz\n", f.shaft_hz);
    text += line;
    std::snprintf(line, sizeof line, "blade pass %.2f Hz (%u blades%s)\n", f.blade_pass_hz, f.blades, assumed_tag(f.blade_pass_assumed));
    text += line;
    if (f.pole_pairs == 0) {
        text += "electrical none (a brushed motor)\n";
    } else {
        std::snprintf(line, sizeof line, "electrical %.2f Hz (%u pole pairs%s)\n", f.electrical_hz, f.pole_pairs,
                      assumed_tag(f.electrical_assumed));
        text += line;
        std::snprintf(line, sizeof line, "commutation %.2f Hz (%u per electrical cycle%s)\n", f.commutation_hz,
                      kCommutationsPerElectricalCycle, assumed_tag(f.electrical_assumed));
        text += line;
    }
    if (f.pwm_hz > 0.0) {
        std::snprintf(line, sizeof line, "pwm %.0f Hz%s\n", f.pwm_hz, f.pwm_assumed ? " (assumed)" : "");
        text += line;
    } else {
        text += "pwm not set\n";
    }
    return text;
}

}  // namespace spade::vehicles
