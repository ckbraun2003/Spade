// q_look.hpp -- THE NASA LOOK (the lead, 2026-11-17: Cameron wants the multirotor flow figures of NASA Ames, OVERFLOW: Q-criterion
// iso-surfaces coloured by vorticity magnitude, blue low to magenta high, crisp thin tubes and sheets).
//
// A published frame's q_hat channel is sqrt(Q) with its sign (core/flow_volume.hpp), so an iso level L on it is Q = L^2. The look is
// a LADDER of three levels: each a fraction of the frame's largest q_hat (the thin cores at the top, the sheets low), each with its
// own opacity (the cores near solid, the outer sheets see-through so what is inside shows), every one coloured by the vorticity
// channel on the blue-to-magenta map over the FIXED range 0 .. vorticity_max x the frame's largest |omega| (the colours of a
// frame mean what they mean in the next one's, within one clip's solve).
#pragma once

#include <array>
#include <charconv>
#include <cmath>
#include <cstddef>
#include <format>
#include <optional>
#include <string>
#include <string_view>
#include <system_error>
#include <vector>

#include "core/error.hpp"
#include "core/flow_volume.hpp"
#include "render/flow_look.hpp"
#include "render/flow_render.hpp"
#include "render/transfer_function.hpp"

namespace flow_movie {

struct QLook {
    std::array<float, 3> level{0.50f, 0.25f, 0.10f};   // fractions of the frame's largest q_hat, strictly falling, each in (0, 1]
    std::array<float, 3> opacity{0.95f, 0.60f, 0.30f};  // each in (0, 1]
    float vorticity_max = 0.5f;                        // the map's top: this fraction of the frame's largest |omega|, in (0, 1]
    // The cloud (--vorticity-cloud): the density at the top of the range in 1/m (above 0, at most 1000), the fraction of the range below which
    // the air is clear (0 up to, not including, 1), and the thinning near the drone: radius as a multiple of the drone's own reach (Drone::reach_m; 0 off, at most 2), the factor at its centre.
    float sigma = 30.0f;
    float onset = 0.15f;
    float thin_r = 0.0f;
    float thin_floor = 0.3f;
    // The vorticity the map's top is a share of, in 1/s: 0 takes each frame's own largest |omega| (the default, and every film before O144's second); above 0 it is the CLIP's largest, read
    // from the cache's headers before drawing (--fixed-cloud-range), so a colour means the same vorticity in every frame and the cloud does not step with the frame's peak.
    float vorticity_peak = 0.0f;
};

// The ladder's iso-surfaces, highest level first; none when the frame has no Q above zero or no vorticity to colour by.
[[nodiscard]] inline std::vector<spade::render::IsoSurface> q_isos(const spade::FlowVolume& v, const QLook& look) {
    const float q_max = v.header.channel_max[spade::kFlowQHat];
    const float vort_max = v.header.channel_max[spade::kFlowVorticity];
    if (!(std::isfinite(q_max) && q_max > 0.0f && std::isfinite(vort_max) && vort_max > 0.0f)) return {};
    spade::render::TransferFunction colours;
    colours.quantity = spade::render::FlowQuantity::vorticity;
    colours.colours = spade::render::ColourMapId::blue_magenta;
    colours.auto_range = false;
    colours.range_min = 0.0f;
    colours.range_max = (look.vorticity_peak > 0.0f ? look.vorticity_peak : vort_max) * look.vorticity_max;
    std::vector<spade::render::IsoSurface> isos;
    for (std::size_t i = 0; i < look.level.size(); ++i) {
        spade::render::IsoSurface iso;
        iso.quantity = spade::render::FlowQuantity::q_hat;
        iso.value = q_max * look.level[i];
        iso.opacity = look.opacity[i];
        iso.colour_by = colours;
        isos.push_back(std::move(iso));
    }
    return isos;
}

// THE CLOUD (Cameron: "fluffy cloud-like air"): |omega| as a soft volume on the same map and over the same range, no surfaces; none
// when the frame has no vorticity. No absorption below an onset of the range, up to a modest density at its top.
[[nodiscard]] inline std::optional<spade::render::TransferFunction> vorticity_cloud(const spade::FlowVolume& v, const QLook& look) {
    const float vort_max = v.header.channel_max[spade::kFlowVorticity];
    if (!(std::isfinite(vort_max) && vort_max > 0.0f)) return std::nullopt;
    return spade::render::volume_transfer(spade::render::FlowQuantity::vorticity, spade::render::ColourMapId::blue_magenta,
                                          (look.vorticity_peak > 0.0f ? look.vorticity_peak : vort_max) * look.vorticity_max, look.onset, look.sigma);
}

// The caption's words for the swirls the pressure film shows with --vorticity-cloud (O144, Oversight's terms): how they are drawn, and that they are the simulated air's own vorticity, not a
// measurement. The rule and its threshold are the cloud's own, so a changed --q-look changes the words.
[[nodiscard]] inline std::string vortex_note(const QLook& look) {
    if (look.vorticity_peak > 0.0f) {  // --fixed-cloud-range: absolute, the same in every frame
        const double top = static_cast<double>(look.vorticity_peak) * static_cast<double>(look.vorticity_max);
        return std::format("The swirls round the rotors are the simulated air's own vorticity (the local spin of the air), drawn as a soft cloud from blue (low) to magenta (high): clear where the "
                           "vorticity is under {:g}% of the map's range ({:.0f} per second), which tops out at {:g}% of the clip's largest vorticity ({:.0f} per second; the clip's largest is "
                           "{:.0f} per second), the same in every frame, up to a density of {:g} per metre at its top, so only the strongest swirls show, and weaker frames show fewer, dimmer "
                           "swirls; the cloud is the simulated air, not a measurement, and the parts and blades keep their pressure colour under it.",
                           static_cast<double>(look.onset) * 100.0, top * static_cast<double>(look.onset), static_cast<double>(look.vorticity_max) * 100.0, top,
                           static_cast<double>(look.vorticity_peak), static_cast<double>(look.sigma));
    }
    return std::format("The swirls round the rotors are the simulated air's own vorticity (the local spin of the air), drawn as a soft cloud from blue (low) to magenta (high): clear where the "
                       "vorticity is under {:g}% of the map's range, which tops out at {:g}% of each frame's largest vorticity, up to a density of {:g} per metre at its top, so only the strongest "
                       "swirls show; the cloud is the simulated air, not a measurement, and the parts and blades keep their pressure colour under it.",
                       static_cast<double>(look.onset) * 100.0, static_cast<double>(look.vorticity_max) * 100.0, static_cast<double>(look.sigma));
}

// "key=value,key=value" over QLook's defaults (--q-look). Keys: l1 l2 l3 (the levels, each in (0, 1], strictly falling), o1 o2 o3
// (the opacities, each in (0, 1]), vmax (in (0, 1]). Empty: the defaults. An unknown key, a missing '=', key or value, a value that
// is not a finite number, one out of its range, or levels that do not fall: refused, naming it.
[[nodiscard]] inline spade::Result<QLook> parse_q_look(std::string_view text) {
    const auto bad = [](const std::string& why) {
        return std::unexpected(spade::Error{spade::Code::invalid_argument, "q look: " + why});
    };
    const auto trim = [](std::string_view v) {
        while (!v.empty() && (v.front() == ' ' || v.front() == '\t')) v.remove_prefix(1);
        while (!v.empty() && (v.back() == ' ' || v.back() == '\t')) v.remove_suffix(1);
        return v;
    };
    QLook look;
    if (trim(text).empty()) return look;
    std::size_t at = 0;
    while (at <= text.size()) {
        const std::size_t comma = text.find(',', at);
        const std::string_view item = trim(text.substr(at, comma == std::string_view::npos ? std::string_view::npos : comma - at));
        at = comma == std::string_view::npos ? text.size() + 1u : comma + 1u;
        if (item.empty()) return bad("an empty item (a stray comma)");
        const std::size_t eq = item.find('=');
        if (eq == std::string_view::npos) return bad("'" + std::string(item) + "' has no '=': write key=value");
        const std::string key(trim(item.substr(0, eq)));
        const std::string_view value = trim(item.substr(eq + 1u));
        if (key.empty()) return bad("an empty key in '" + std::string(item) + "'");
        if (value.empty()) return bad("'" + key + "' has no value");
        float number = 0.0f;
        const auto [end, ec] = std::from_chars(value.data(), value.data() + value.size(), number);
        if (ec != std::errc{} || end != value.data() + value.size() || !std::isfinite(number)) {
            return bad("'" + key + "' is not a number: '" + std::string(value) + "'");
        }
        float* slot = nullptr;
        if (key == "l1") slot = &look.level[0];
        else if (key == "l2") slot = &look.level[1];
        else if (key == "l3") slot = &look.level[2];
        else if (key == "o1") slot = &look.opacity[0];
        else if (key == "o2") slot = &look.opacity[1];
        else if (key == "o3") slot = &look.opacity[2];
        else if (key == "vmax") slot = &look.vorticity_max;
        else if (key == "sigma") slot = &look.sigma;
        else if (key == "onset") slot = &look.onset;
        else if (key == "thin_r") slot = &look.thin_r;
        else if (key == "thin_floor") slot = &look.thin_floor;
        if (slot == nullptr) return bad("unknown key '" + key + "' (l1 l2 l3 o1 o2 o3 vmax sigma onset thin_r thin_floor)");
        const bool inclusive_zero = key == "onset" || key == "thin_r" || key == "thin_floor";
        const float top = key == "sigma" ? 1000.0f : key == "thin_r" ? 2.0f : 1.0f;
        const bool onset_open = key == "onset";  // 0 up to, not including, 1
        const bool in_range = onset_open ? (number >= 0.0f && number < 1.0f)
                                         : (inclusive_zero ? (number >= 0.0f && number <= top) : (number > 0.0f && number <= top));
        if (!in_range) {
            return bad("'" + key + "' = " + std::string(value) + " is out of its range (" + (inclusive_zero ? "0 up to " : "above 0, at most ") +
                       (onset_open ? std::string("but not 1") : std::to_string(top)) + ")");
        }
        *slot = number;
    }
    if (!(look.level[0] > look.level[1] && look.level[1] > look.level[2])) {
        return bad("the levels l1 > l2 > l3 must fall (got " + std::to_string(look.level[0]) + ", " + std::to_string(look.level[1]) +
                   ", " + std::to_string(look.level[2]) + ")");
    }
    return look;
}

}  // namespace flow_movie
