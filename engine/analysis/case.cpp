// The analysis case's own checks and resolving it (see the header).

#include "analysis/case.hpp"

#include <algorithm>
#include <cmath>
#include <format>
#include <string>
#include <utility>

#include <glm/glm.hpp>

#include "core/rng.hpp"
#include "physics/airflow/fluid_step.hpp"
#include "vehicles/model_identity.hpp"

namespace spade::analysis {

namespace {

[[nodiscard]] std::string num(float v) { return std::format("{:.6g}", v); }
[[nodiscard]] std::string mm(float metres) { return std::format("{:.3g} mm", metres * 1000.0f); }
[[nodiscard]] std::string idx(std::string_view what, std::size_t i) {
    return std::string(what) + "[" + std::to_string(i) + "]";
}
[[nodiscard]] std::string hex64(uint64_t v) { return std::format("0x{:016x}", v); }

[[nodiscard]] std::unexpected<Error> refuse(std::string field, std::string why) {
    return std::unexpected(Error{Code::invalid_argument, std::move(field) + ": " + std::move(why)});
}

[[nodiscard]] bool finite_at_least(float v, float lo) { return std::isfinite(v) && v >= lo; }
[[nodiscard]] bool positive(float v) { return std::isfinite(v) && v > 0.0f; }

[[nodiscard]] std::string_view model_word(RotorModel m) {
    switch (m) {
        case RotorModel::disc: return "the disc";
        case RotorModel::line: return "the actuator line";
        case RotorModel::blade_resolved: return "blade-resolved";
    }
    return "the rotor model";
}

[[nodiscard]] Result<void> check_blade(const BladeDefinition& b, const std::string& at) {
    if (b.blade_count == 0) return refuse(at + ".blade_count", "a rotor needs at least one blade");
    if (!positive(b.radius)) return refuse(at + ".radius", num(b.radius) + " m must be above 0");
    if (!finite_at_least(b.hub_radius, 0.0f) || b.hub_radius >= b.radius) {
        return refuse(at + ".hub_radius", num(b.hub_radius) + " m must be at least 0 and below the radius " +
                                              num(b.radius) + " m");
    }
    if (!std::isfinite(b.collective_deg)) return refuse(at + ".collective_deg", "must be finite");
    if (b.stations.empty()) return refuse(at + ".stations", "a blade needs at least one station");
    for (std::size_t i = 0; i < b.stations.size(); ++i) {
        const BladeStation& s = b.stations[i];
        const std::string here = at + "." + idx("stations", i);
        if (!positive(s.r_over_radius) || s.r_over_radius > 1.0f) {
            return refuse(here + ".r_over_radius", num(s.r_over_radius) + " must be in (0, 1]");
        }
        if (i > 0 && s.r_over_radius <= b.stations[i - 1].r_over_radius) {
            return refuse(here + ".r_over_radius", num(s.r_over_radius) + " must be above " + idx("stations", i - 1) +
                                                       "'s " + num(b.stations[i - 1].r_over_radius) +
                                                       ": stations run from hub to tip");
        }
        if (!positive(s.chord)) return refuse(here + ".chord", num(s.chord) + " m must be above 0");
        if (!std::isfinite(s.twist_deg)) return refuse(here + ".twist_deg", "must be finite");
        if (s.airfoil.empty()) return refuse(here + ".airfoil", "must name an airfoil");
    }
    return {};
}

[[nodiscard]] Result<void> check_airfoil(const Airfoil& a, const std::string& at) {
    for (std::size_t i = 0; i < a.polars.size(); ++i) {
        const Polar& p = a.polars[i];
        const std::string here = at + "." + idx("polars", i);
        if (!positive(p.reynolds)) return refuse(here + ".reynolds", num(p.reynolds) + " must be above 0");
        if (p.alpha_deg.empty()) return refuse(here + ".alpha_deg", "a polar needs at least one angle");
        for (std::size_t k = 0; k < p.alpha_deg.size(); ++k) {
            if (!std::isfinite(p.alpha_deg[k])) return refuse(here + ".alpha_deg", "[" + std::to_string(k) + "] must be finite");
            if (k > 0 && p.alpha_deg[k] <= p.alpha_deg[k - 1]) {
                return refuse(here + ".alpha_deg", "must increase: [" + std::to_string(k) + "] is " +
                                                       num(p.alpha_deg[k]) + " after " + num(p.alpha_deg[k - 1]));
            }
        }
        const auto same_length = [&](const std::vector<float>& v, const char* key) -> Result<void> {
            if (v.size() != p.alpha_deg.size()) {
                return refuse(here + "." + key, std::to_string(v.size()) + " values for " +
                                                    std::to_string(p.alpha_deg.size()) + " angles");
            }
            for (const float x : v) {
                if (!std::isfinite(x)) return refuse(here + "." + key, "must be finite");
            }
            return {};
        };
        if (Result<void> r = same_length(p.cl, "cl"); !r) return r;
        if (Result<void> r = same_length(p.cd, "cd"); !r) return r;
    }
    return {};
}

}  // namespace

Result<void> validate_case(const AnalysisCase& c) {
    const Controls& k = c.controls;
    if (!std::isfinite(k.throttle) || k.throttle < 0.0f || k.throttle > 1.0f) {
        return refuse("controls.throttle", num(k.throttle) + " must be in [0, 1]");
    }
    if (!std::isfinite(k.pitch_deg) || !std::isfinite(k.roll_deg) || !std::isfinite(k.yaw_deg)) {
        return refuse("controls", "the attitude must be finite");
    }
    const Flight& f = c.flight;
    if (!finite_at_least(f.speed, 0.0f)) return refuse("flight.speed", num(f.speed) + " m/s must be at least 0");
    if (f.kind == FlightKind::hover && f.speed != 0.0f) {
        return refuse("flight.speed", "hover has no speed; a wind on a hovering drone is hover_in_wind");
    }
    if (f.kind != FlightKind::hover && f.speed == 0.0f) {
        return refuse("flight.speed", "forward flight and hover in a wind need a speed above 0");
    }
    if (!finite_at_least(f.floor_height, 0.0f)) {
        return refuse("flight.floor_height", num(f.floor_height) + " m must be at least 0 (0 for no floor)");
    }
    if (!positive(c.fluid.density)) return refuse("fluid.density", num(c.fluid.density) + " kg/m3 must be above 0");
    if (!positive(c.fluid.kinematic_viscosity)) {
        return refuse("fluid.kinematic_viscosity", num(c.fluid.kinematic_viscosity) + " m2/s must be above 0");
    }
    const GridSetup& g = c.grid;
    if (!finite_at_least(g.box, 0.0f)) return refuse("grid.box", num(g.box) + " m must be at least 0 (0 for the default)");
    if (!finite_at_least(g.spacing, 0.0f)) {
        return refuse("grid.spacing", num(g.spacing) + " m must be at least 0 (0 for the default)");
    }
    if (g.box > 0.0f && g.spacing >= g.box) {
        return refuse("grid.spacing", num(g.spacing) + " m must be below the box " + num(g.box) + " m");
    }
    if (g.nests > kMaxNests) {
        return refuse("grid.nests", std::to_string(g.nests) + " levels; the solver nests at most " +
                                        std::to_string(kMaxNests));
    }
    for (std::size_t i = 0; i < c.blades.size(); ++i) {
        const std::string at = idx("blades", i);
        if (c.blades[i].name.empty()) return refuse(at + ".name", "a blade needs a name its rotors use");
        for (std::size_t j = 0; j < i; ++j) {
            if (c.blades[j].name == c.blades[i].name) {
                return refuse(at + ".name", "'" + c.blades[i].name + "' is " + idx("blades", j) + "'s name too");
            }
        }
        if (Result<void> r = check_blade(c.blades[i], at); !r) return r;
    }
    for (std::size_t i = 0; i < c.airfoils.size(); ++i) {
        const std::string at = idx("airfoils", i);
        if (c.airfoils[i].id.empty()) return refuse(at + ".id", "an airfoil needs the id its stations use");
        for (std::size_t j = 0; j < i; ++j) {
            if (c.airfoils[j].id == c.airfoils[i].id) {
                return refuse(at + ".id", "'" + c.airfoils[i].id + "' is " + idx("airfoils", j) + "'s id too");
            }
        }
        if (Result<void> r = check_airfoil(c.airfoils[i], at); !r) return r;
    }
    for (std::size_t i = 0; i < c.rotors.size(); ++i) {
        const std::string& blade = c.rotors[i].blade;
        const bool known = blade.empty() || std::any_of(c.blades.begin(), c.blades.end(),
                                                        [&](const BladeDefinition& b) { return b.name == blade; });
        if (!known) return refuse(idx("rotors", i) + ".blade", "no blade is named '" + blade + "'");
    }
    const TimeSetup& t = c.time;
    if (!positive(t.duration)) return refuse("time.duration", num(t.duration) + " s must be above 0");
    if (!finite_at_least(t.startup, 0.0f) || t.startup >= t.duration) {
        return refuse("time.startup", num(t.startup) + " s must be at least 0 and below the duration " +
                                          num(t.duration) + " s");
    }
    if (!finite_at_least(t.step, 0.0f)) return refuse("time.step", num(t.step) + " s must be at least 0 (0 for the solver's rule)");
    if (!finite_at_least(c.outputs.history_every, 0.0f)) return refuse("outputs.history_every", "must be at least 0");
    if (!finite_at_least(c.outputs.snapshot_every, 0.0f)) return refuse("outputs.snapshot_every", "must be at least 0");
    return {};
}

float reach_of(const vehicles::ModelType& model) noexcept {
    float reach = model.proxy_radius;
    for (const vehicles::RotorDesc& r : model.rotors) reach = std::max(reach, glm::length(r.local_pos) + r.radius);
    return reach;
}

std::expected<ResolvedCase, std::vector<CaseIssue>> resolve(const AnalysisCase& c, const vehicles::ModelType& model) {
    std::vector<CaseIssue> issues;
    const auto issue = [&](std::string field, std::string message) {
        issues.push_back(CaseIssue{std::move(field), std::move(message)});
    };
    // A refusal's context is "field: why".
    const auto from_error = [&](const Error& e, std::string_view fallback) {
        const std::size_t colon = e.context.find(": ");
        if (colon == std::string::npos) {
            issue(std::string(fallback), e.context);
        } else {
            issue(e.context.substr(0, colon), e.context.substr(colon + 2));
        }
    };
    if (Result<void> ok = validate_case(c); !ok) from_error(ok.error(), "case");
    if (Result<void> ok = model.validate(); !ok) issue("model", ok.error().context);
    if (!issues.empty()) return std::unexpected(std::move(issues));

    const uint64_t identity = vehicles::model_identity(model);
    if (identity != c.model.identity) {
        issue("model.identity", "the model changed since the case pinned it (case " + hex64(c.model.identity) +
                                    ", model " + hex64(identity) + "); re-pin the case to the model");
    }
    if (c.rotors.size() != model.rotors.size()) {
        issue("rotors", "the case sets " + std::to_string(c.rotors.size()) + " rotors; the model has " +
                            std::to_string(model.rotors.size()));
    }

    ResolvedCase r;
    r.spec = c;
    GridSetup& g = r.spec.grid;
    if (g.box == 0.0f) {
        g.box = kDefaultBoxReaches * reach_of(model);
        r.box_source = Source::default_value;
    }
    if (g.spacing == 0.0f) {
        g.spacing = g.box / static_cast<float>(kDefaultCellsAcross);
        r.spacing_source = Source::default_value;
    }
    r.finest_spacing = g.spacing / static_cast<float>(1u << g.nests);
    const std::string finest = "the finest cell is " + mm(r.finest_spacing) + " (" + mm(g.spacing) + " base, " +
                               std::to_string(g.nests) + " nests)";

    // The base grid, and whether the solver can hold it.
    const auto cells = static_cast<uint32_t>(std::ceil(g.box / g.spacing * (1.0f - 1e-6f)));
    r.base = physics::airflow::GridShape{cells, cells, cells, g.spacing};
    r.base_bytes = physics::airflow::fluid_bytes(r.base);
    if (r.base_bytes > physics::airflow::kGridByteCap) {
        const double scale = std::cbrt(static_cast<double>(physics::airflow::kGridByteCap) /
                                       static_cast<double>(r.base_bytes));
        const auto most = static_cast<float>(std::floor(static_cast<double>(cells) * scale));
        issue("grid.spacing", "a " + std::to_string(cells) + "^3 base grid needs " +
                                  std::format("{:.3g}", static_cast<double>(r.base_bytes) / double(1u << 30)) +
                                  " GiB; the solver holds " +
                                  std::to_string(physics::airflow::kGridByteCap >> 30) +
                                  " GiB: a spacing of at least " + mm(g.box / std::max(most, 1.0f)) +
                                  " fits, or a smaller box");
    }

    // Each rotor, at the finest cell.
    for (std::size_t i = 0; i < std::min(c.rotors.size(), model.rotors.size()); ++i) {
        const RotorSetup& s = c.rotors[i];
        const vehicles::RotorDesc& rd = model.rotors[i];
        ResolvedRotor rr;
        rr.model = s.model;
        rr.radius = rd.radius;
        rr.cells_across_radius = rd.radius / r.finest_spacing;
        const std::string at = idx("rotors", i);
        const BladeDefinition* blade = nullptr;
        for (const BladeDefinition& b : c.blades) {
            if (b.name == s.blade) blade = &b;
        }
        if (blade != nullptr) {
            if (std::abs(blade->radius - rd.radius) > 1e-6f * std::max(1.0f, rd.radius)) {
                issue(at + ".blade", "blade '" + blade->name + "' has a radius of " + num(blade->radius) +
                                         " m; rotor " + std::to_string(i) + "'s radius is " + num(rd.radius) + " m");
            }
            for (const BladeStation& st : blade->stations) {
                const auto polars = std::find_if(c.airfoils.begin(), c.airfoils.end(),
                                                 [&](const Airfoil& a) { return a.id == st.airfoil; });
                if (polars == c.airfoils.end() || polars->polars.empty()) rr.estimated_polars = true;
            }
        }
        if (s.model != RotorModel::disc && blade == nullptr) {
            issue(at + ".blade", std::string(model_word(s.model)) + " needs a blade definition");
        }
        if (s.model == RotorModel::line && rr.cells_across_radius < kLineCellsPerRadius) {
            issue(at, "the actuator line needs R/dx >= 20: R " + mm(rd.radius) + " needs a cell of at most " +
                          mm(rd.radius / kLineCellsPerRadius) + "; " + finest);
        }
        // Blade-resolved's thickness rule (>= 2 cells) comes with P5's
        // lofting, which knows the airfoil's thickness.
        if (s.model == RotorModel::blade_resolved && blade != nullptr) {
            float chord = INFINITY;
            for (const BladeStation& st : blade->stations) chord = std::min(chord, st.chord);
            if (chord / r.finest_spacing < kBladeCellsPerChord) {
                issue(at, "blade-resolved needs 32 cells across the chord: the narrowest chord " + mm(chord) +
                              " needs a cell of at most " + mm(chord / kBladeCellsPerChord) + "; " + finest);
            }
        }
        r.rotors.push_back(rr);
    }

    // Each wall part, at the finest cell: refused, never made porous (P2).
    for (std::size_t k = 0; k < model.parts.size(); ++k) {
        const vehicles::PartDesc& p = model.parts[k];
        if (p.surface.kind != vehicles::PartSurface::Kind::wall) continue;
        if (p.min_feature < kWallCells * r.finest_spacing) {
            issue(idx("parts", k), "'" + p.name + "' declares a " + mm(p.min_feature) +
                                       " smallest feature; a wall needs 2 cells across it, so a cell of at most " +
                                       mm(p.min_feature / kWallCells) + "; " + finest);
        }
    }

    if (!issues.empty()) return std::unexpected(std::move(issues));
    const Result<std::string> text = case_to_yaml(r.spec);
    if (!text) {
        issue("case", text.error().context);
        return std::unexpected(std::move(issues));
    }
    r.hash = rng::fnv1a64(*text);
    return r;
}

}  // namespace spade::analysis
