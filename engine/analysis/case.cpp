// The analysis case's own checks and resolving it (see the header).

#include "analysis/case.hpp"

#include <algorithm>
#include <cmath>
#include <format>
#include <numbers>
#include <string>
#include <utility>

#include <glm/glm.hpp>

#include "core/fp32_math.hpp"
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

[[nodiscard]] bool at_least(float v, float lo) { return std::isfinite(v) && v >= lo; }
[[nodiscard]] bool positive(float v) { return std::isfinite(v) && v > 0.0f; }

[[nodiscard]] std::string_view model_word(RotorModel m) {
    switch (m) {
        case RotorModel::disc: return "the disc";
        case RotorModel::line: return "the actuator line";
        case RotorModel::blade_resolved: return "blade-resolved";
    }
    return "the rotor model";
}

// The drone box's faces (Physics' plan A conditions). Every face open, the
// ambient the air's velocity past the drone: forward flight moves the air
// AND the ground at -v along where the air comes from (the floor a rolling
// road), hover in a wind moves only the air (the floor still). sin32 and
// cos32, not libm (TD-3).
[[nodiscard]] physics::airflow::DomainBc drone_box_domain(const Flight& f) {
    namespace af = physics::airflow;
    af::DomainBc bc;
    if (f.kind != FlightKind::hover) {
        const float h = f.heading_deg * (std::numbers::pi_v<float> / 180.0f);
        const glm::vec3 from(math::cos32(h), 0.0f, -math::sin32(h));  // + heading: from the left (-Z)
        bc.ambient = -f.speed * from;
    }
    if (f.floor_height) {
        bc.face[af::kYMinus] = af::FaceBc::wall;
        bc.wall_velocity[af::kYMinus] = f.kind == FlightKind::forward ? bc.ambient : glm::vec3(0.0f);
    }
    return bc;
}

// The problems, in field order.
class Issues {
  public:
    void add(std::string field, std::string message) {
        list_.push_back(CaseIssue{std::move(field), std::move(message)});
    }
    [[nodiscard]] std::vector<CaseIssue> take() { return std::move(list_); }

  private:
    std::vector<CaseIssue> list_;
};

void check_blade(const BladeDefinition& b, const std::string& at, Issues& out) {
    if (b.blade_count == 0) out.add(at + ".blade_count", "a rotor needs at least one blade");
    if (!at_least(b.hub_radius, 0.0f)) out.add(at + ".hub_radius", num(b.hub_radius) + " m must be at least 0");
    if (!std::isfinite(b.collective_deg)) out.add(at + ".collective_deg", "must be finite");
    if (b.stations.empty()) out.add(at + ".stations", "a blade needs at least one station");
    for (std::size_t i = 0; i < b.stations.size(); ++i) {
        const BladeStation& s = b.stations[i];
        const std::string here = at + "." + idx("stations", i);
        if (!positive(s.r_over_radius) || s.r_over_radius > 1.0f) {
            out.add(here + ".r_over_radius", num(s.r_over_radius) + " must be in (0, 1]");
        } else if (i > 0 && s.r_over_radius <= b.stations[i - 1].r_over_radius) {
            out.add(here + ".r_over_radius", num(s.r_over_radius) + " must be above " + idx("stations", i - 1) +
                                                 "'s " + num(b.stations[i - 1].r_over_radius) +
                                                 ": stations run from hub to tip");
        }
        if (!positive(s.chord)) out.add(here + ".chord", num(s.chord) + " m must be above 0");
        if (!std::isfinite(s.twist_deg)) out.add(here + ".twist_deg", "must be finite");
        if (s.airfoil.empty()) out.add(here + ".airfoil", "must name an airfoil");
    }
}

void check_airfoil(const Airfoil& a, const std::string& at, Issues& out) {
    for (std::size_t i = 0; i < a.polars.size(); ++i) {
        const Polar& p = a.polars[i];
        const std::string here = at + "." + idx("polars", i);
        if (!positive(p.reynolds)) out.add(here + ".reynolds", num(p.reynolds) + " must be above 0");
        if (p.alpha_deg.empty()) out.add(here + ".alpha_deg", "a polar needs at least one angle");
        for (std::size_t k = 0; k < p.alpha_deg.size(); ++k) {
            if (!std::isfinite(p.alpha_deg[k])) {
                out.add(here + ".alpha_deg", "[" + std::to_string(k) + "] must be finite");
                break;
            }
            if (k > 0 && p.alpha_deg[k] <= p.alpha_deg[k - 1]) {
                out.add(here + ".alpha_deg", "must increase: [" + std::to_string(k) + "] is " + num(p.alpha_deg[k]) +
                                                 " after " + num(p.alpha_deg[k - 1]));
                break;
            }
        }
        const auto same_length = [&](const std::vector<float>& v, const char* key) {
            if (v.size() != p.alpha_deg.size()) {
                out.add(here + "." + key,
                        std::to_string(v.size()) + " values for " + std::to_string(p.alpha_deg.size()) + " angles");
            } else if (!std::all_of(v.begin(), v.end(), [](float x) { return std::isfinite(x); })) {
                out.add(here + "." + key, "must be finite");
            }
        };
        same_length(p.cl, "cl");
        same_length(p.cd, "cd");
    }
}

}  // namespace

std::vector<CaseIssue> case_issues(const AnalysisCase& c) {
    Issues out;
    const Controls& k = c.controls;
    if (!std::isfinite(k.throttle) || k.throttle < 0.0f || k.throttle > 1.0f) {
        out.add("controls.throttle", num(k.throttle) + " must be in [0, 1]");
    }
    if (!std::isfinite(k.pitch_deg)) out.add("controls.pitch_deg", "must be finite");
    if (!std::isfinite(k.roll_deg)) out.add("controls.roll_deg", "must be finite");
    if (!std::isfinite(k.yaw_deg)) out.add("controls.yaw_deg", "must be finite");

    const Flight& f = c.flight;
    if (!at_least(f.speed, 0.0f)) {
        out.add("flight.speed", num(f.speed) + " m/s must be at least 0");
    } else if (f.kind == FlightKind::hover && f.speed != 0.0f) {
        out.add("flight.speed", "hover has no speed; a wind on a hovering drone is hover_in_wind");
    } else if (f.kind != FlightKind::hover && f.speed == 0.0f) {
        out.add("flight.speed", "forward flight and hover in a wind need a speed above 0");
    }
    if (!std::isfinite(f.heading_deg)) out.add("flight.heading_deg", "must be finite");
    if (f.floor_height && !positive(*f.floor_height)) {
        out.add("flight.floor_height", num(*f.floor_height) + " m must be above 0; leave it out for no floor");
    }

    if (!positive(c.fluid.density)) out.add("fluid.density", num(c.fluid.density) + " kg/m3 must be above 0");
    if (!positive(c.fluid.kinematic_viscosity)) {
        out.add("fluid.kinematic_viscosity", num(c.fluid.kinematic_viscosity) + " m2/s must be above 0");
    }

    const GridSetup& g = c.grid;
    if (g.box && !positive(*g.box)) out.add("grid.box", num(*g.box) + " m must be above 0; leave it out for the default");
    if (g.spacing && !positive(*g.spacing)) {
        out.add("grid.spacing", num(*g.spacing) + " m must be above 0; leave it out for the default");
    } else if (g.spacing && g.box && positive(*g.box) && *g.spacing >= *g.box) {
        out.add("grid.spacing", num(*g.spacing) + " m must be below the box " + num(*g.box) + " m");
    }
    if (g.nests > kMaxNests) {
        out.add("grid.nests", std::to_string(g.nests) + " levels; the solver nests at most " + std::to_string(kMaxNests));
    }

    for (std::size_t i = 0; i < c.rotors.size(); ++i) {
        const std::string& blade = c.rotors[i].blade;
        const bool known = blade.empty() || std::any_of(c.blades.begin(), c.blades.end(),
                                                        [&](const BladeDefinition& b) { return b.name == blade; });
        if (!known) out.add(idx("rotors", i) + ".blade", "no blade is named '" + blade + "'");
    }
    for (std::size_t i = 0; i < c.blades.size(); ++i) {
        const std::string at = idx("blades", i);
        if (c.blades[i].name.empty()) out.add(at + ".name", "a blade needs a name its rotors use");
        for (std::size_t j = 0; j < i; ++j) {
            if (!c.blades[i].name.empty() && c.blades[j].name == c.blades[i].name) {
                out.add(at + ".name", "'" + c.blades[i].name + "' is " + idx("blades", j) + "'s name too");
                break;
            }
        }
        check_blade(c.blades[i], at, out);
    }
    for (std::size_t i = 0; i < c.airfoils.size(); ++i) {
        const std::string at = idx("airfoils", i);
        if (c.airfoils[i].id.empty()) out.add(at + ".id", "an airfoil needs the id its stations use");
        for (std::size_t j = 0; j < i; ++j) {
            if (!c.airfoils[i].id.empty() && c.airfoils[j].id == c.airfoils[i].id) {
                out.add(at + ".id", "'" + c.airfoils[i].id + "' is " + idx("airfoils", j) + "'s id too");
                break;
            }
        }
        check_airfoil(c.airfoils[i], at, out);
    }

    const TimeSetup& t = c.time;
    if (!positive(t.duration)) out.add("time.duration", num(t.duration) + " s must be above 0");
    if (!at_least(t.startup, 0.0f) || (positive(t.duration) && t.startup >= t.duration)) {
        out.add("time.startup", num(t.startup) + " s must be at least 0 and below the duration " + num(t.duration) + " s");
    }
    if (t.step && !positive(*t.step)) out.add("time.step", num(*t.step) + " s must be above 0; leave it out for the solver's rule");
    if (c.outputs.history_every && !positive(*c.outputs.history_every)) {
        out.add("outputs.history_every", "must be above 0; leave it out for every step");
    }
    if (c.outputs.snapshot_every && !positive(*c.outputs.snapshot_every)) {
        out.add("outputs.snapshot_every", "must be above 0; leave it out for none");
    }
    return out.take();
}

Result<void> validate_case(const AnalysisCase& c) {
    const std::vector<CaseIssue> issues = case_issues(c);
    if (issues.empty()) return {};
    return std::unexpected(Error{Code::invalid_argument, issues.front().field + ": " + issues.front().message});
}

std::expected<ResolvedCase, std::vector<CaseIssue>> resolve(const AnalysisCase& c, const vehicles::ModelType& model) {
    Issues out;
    for (CaseIssue& i : case_issues(c)) out.add(std::move(i.field), std::move(i.message));
    if (Result<void> ok = model.validate(); !ok) out.add("model", ok.error().context);
    if (std::vector<CaseIssue> early = out.take(); !early.empty()) return std::unexpected(std::move(early));

    const uint64_t identity = vehicles::model_identity(model);
    if (identity != c.model.identity) {
        out.add("model.identity", "the model changed since the case pinned it (case " + hex64(c.model.identity) +
                                      ", model " + hex64(identity) + "); re-pin the case to the model");
    }
    const uint64_t parts = vehicles::parts_identity(model);
    if (parts != c.model.parts) {
        out.add("model.parts", "the model's parts changed since the case pinned them (case " + hex64(c.model.parts) +
                                   ", model " + hex64(parts) + "); re-pin the case to the model");
    }
    if (c.rotors.size() != model.rotors.size()) {
        out.add("rotors", "the case sets " + std::to_string(c.rotors.size()) + " rotors; the model has " +
                              std::to_string(model.rotors.size()));
    }

    ResolvedCase r;
    r.spec = c;
    GridSetup& g = r.spec.grid;
    if (!g.box) {
        g.box = kDefaultBoxReaches * vehicles::model_reach(model);
        r.box_source = Source::default_value;
    }
    if (!g.spacing) {
        g.spacing = *g.box / static_cast<float>(kDefaultCellsAcross);
        r.spacing_source = Source::default_value;
    }
    const float box = *g.box;
    const float spacing = *g.spacing;
    r.finest_spacing = spacing / static_cast<float>(1u << g.nests);
    const std::string finest = "the finest cell is " + mm(r.finest_spacing) + " (" + mm(spacing) + " base, " +
                               std::to_string(g.nests) + " nests)";

    // The base grid, and whether the solver can hold it.
    const auto cells = static_cast<uint32_t>(std::ceil(box / spacing * (1.0f - 1e-6f)));
    r.base = physics::airflow::GridShape{cells, cells, cells, spacing};
    r.domain = drone_box_domain(r.spec.flight);
    r.base_bytes = physics::airflow::fluid_bytes(r.base, r.domain);
    if (r.base_bytes > physics::airflow::kGridByteCap) {
        const double scale =
            std::cbrt(static_cast<double>(physics::airflow::kGridByteCap) / static_cast<double>(r.base_bytes));
        const auto most = static_cast<float>(std::floor(static_cast<double>(cells) * scale));
        out.add("grid.spacing", "a " + std::to_string(cells) + "^3 base grid needs " +
                                    std::format("{:.3g}", static_cast<double>(r.base_bytes) / double(1u << 30)) +
                                    " GiB; the solver holds " + std::to_string(physics::airflow::kGridByteCap >> 30) +
                                    " GiB: a spacing of at least " + mm(box / std::max(most, 1.0f)) +
                                    " fits, or a smaller box");
    }

    // Each rotor, at the finest cell. A blade takes its rotor's radius.
    for (std::size_t i = 0; i < std::min(c.rotors.size(), model.rotors.size()); ++i) {
        const RotorSetup& s = c.rotors[i];
        const vehicles::RotorDesc& rd = model.rotors[i];
        ResolvedRotor rr;
        rr.model = s.model;
        rr.radius = rd.radius;
        rr.radius_source = Source::derived;
        rr.cells_across_radius = rd.radius / r.finest_spacing;
        const std::string at = idx("rotors", i);
        const BladeDefinition* blade = nullptr;
        for (const BladeDefinition& b : c.blades) {
            if (!s.blade.empty() && b.name == s.blade) blade = &b;
        }
        if (blade != nullptr) {
            if (blade->hub_radius >= rd.radius) {
                out.add(at + ".blade", "blade '" + blade->name + "' has a hub of " + num(blade->hub_radius) +
                                           " m; rotor " + std::to_string(i) + "'s radius is " + num(rd.radius) +
                                           " m, and the hub must be inside it");
            }
            for (const BladeStation& st : blade->stations) {
                const auto foil = std::find_if(c.airfoils.begin(), c.airfoils.end(),
                                               [&](const Airfoil& a) { return a.id == st.airfoil; });
                if (foil == c.airfoils.end() || foil->polars.empty()) rr.estimated_polars = true;
            }
        }
        if (s.model != RotorModel::disc && blade == nullptr) {
            out.add(at + ".blade", std::string(model_word(s.model)) + " needs a blade definition");
        }
        if (s.model == RotorModel::line && rr.cells_across_radius < kLineCellsPerRadius) {
            out.add(at, "the actuator line needs R/dx >= 20: R " + mm(rd.radius) + " needs a cell of at most " +
                            mm(rd.radius / kLineCellsPerRadius) + "; " + finest);
        }
        // Blade-resolved's thickness rule (>= 2 cells) comes with P5's
        // lofting, which knows the airfoil's thickness.
        if (s.model == RotorModel::blade_resolved && blade != nullptr) {
            float chord = INFINITY;
            for (const BladeStation& st : blade->stations) chord = std::min(chord, st.chord);
            if (chord / r.finest_spacing < kBladeCellsPerChord) {
                out.add(at, "blade-resolved needs 32 cells across the chord: the narrowest chord " + mm(chord) +
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
            out.add(idx("parts", k), "'" + p.name + "' declares a " + mm(p.min_feature) +
                                         " smallest feature; a wall needs 2 cells across it, so a cell of at most " +
                                         mm(p.min_feature / kWallCells) + "; " + finest);
        }
    }

    if (std::vector<CaseIssue> issues = out.take(); !issues.empty()) return std::unexpected(std::move(issues));
    const Result<std::string> text = case_to_yaml(r.spec);
    if (!text) return std::unexpected(std::vector<CaseIssue>{CaseIssue{"case", text.error().context}});
    r.hash = rng::fnv1a64(*text);
    return r;
}

}  // namespace spade::analysis
