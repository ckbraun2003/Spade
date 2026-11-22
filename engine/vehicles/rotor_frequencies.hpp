#pragma once

#include <cstdint>
#include <span>
#include <string>

#include "core/error.hpp"

// ---------------------------------------------------------------------------
// rotor_frequencies.hpp -- the frequencies a spinning rotor makes, derived from its shaft speed (O169; the first step toward sound and RF
// analysis: nothing here is measured or simulated sound, only the lines a spectrum of this rotor would have).
//
//   shaft       omega / 2 pi, Hz
//   blade pass  blades x shaft: the rotor's loudest tone's fundamental (its harmonics are the multiples)
//   electrical  pole pairs x shaft: the motor's electrical frequency (0 for a brushed motor, which has no pole pairs)
//   commutation 6 x electrical: a six-step ESC commutates six times per electrical cycle (it is the sine-drive or field-oriented ESC's
//               carrier that differs; this is the six-step figure)
//   pwm         the ESC's switching frequency, an INPUT (no model computes it)
//
// EVERY INPUT THAT IS NOT A MEASURED OR PUBLISHED FIGURE IS LABELLED ASSUMED, and each derived value carries the label of the inputs it comes
// from: a blade count read from the part file is `given`, a pole-pair count of 7 (the library's 12N14P) is `assumed`, an ESC switching
// frequency nobody wrote down is `assumed`. The readout text says so on the same line.
//
// Pure functions: no state, no clock, no libm. `omega` is a rotor row's `omega` (rad/s), or any shaft speed.
// ---------------------------------------------------------------------------
namespace spade::vehicles {

struct FrequencyInputs {
    double omega = 0.0;              // shaft speed, rad/s: finite, >= 0
    uint32_t blades = 0;             // propeller blade count: 1 to 16
    bool blades_assumed = true;      // true unless the count came from a graded part file
    uint32_t pole_pairs = 0;         // motor pole pairs: 0 for a brushed motor (no electrical frequency), else up to 64
    bool pole_pairs_assumed = true;  // true unless the count came from a graded part file
    double pwm_hz = 0.0;             // the ESC's switching frequency, Hz: finite, >= 0; 0 is "not set"
    bool pwm_assumed = true;         // true unless the figure came from a graded part file
};

struct DerivedFrequencies {
    double shaft_hz = 0.0;
    double blade_pass_hz = 0.0;
    double electrical_hz = 0.0;   // 0 when the motor has no pole pairs
    double commutation_hz = 0.0;  // 6 x electrical_hz
    double pwm_hz = 0.0;          // as given; 0 when not set
    uint32_t blades = 0;
    uint32_t pole_pairs = 0;
    bool blade_pass_assumed = true;
    bool electrical_assumed = true;  // the commutation frequency carries the same label
    bool pwm_assumed = true;
};

// The derived frequencies, or invalid_argument naming the field: omega not finite or negative, blades outside 1 to 16, pole_pairs above 64,
// pwm_hz not finite or negative.
[[nodiscard]] Result<DerivedFrequencies> derive_frequencies(const FrequencyInputs& in);

// out[k] = (k + 1) x fundamental_hz: the harmonics of a fundamental, allocation-free.
void harmonics(double fundamental_hz, std::span<double> out) noexcept;

// A readout, one line per figure, each assumed input named as such:
//   shaft 155.00 Hz
//   blade pass 465.00 Hz (3 blades, assumed)
//   electrical 1085.00 Hz (7 pole pairs, assumed)          or  electrical none (a brushed motor)
//   commutation 6510.00 Hz (6 per electrical cycle, assumed)  (omitted for a brushed motor)
//   pwm 24000 Hz (assumed)                                  or  pwm not set
[[nodiscard]] std::string frequency_readout(const DerivedFrequencies& f);

}  // namespace spade::vehicles
