#pragma once

#include <algorithm>
#include <array>
#include <charconv>
#include <concepts>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <filesystem>
#include <fstream>
#include <initializer_list>
#include <iterator>
#include <memory>
#include <string>
#include <string_view>
#include <system_error>
#include <utility>
#include <vector>

#include <yaml-cpp/yaml.h>

#include <glm/gtc/quaternion.hpp>
#include <glm/vec3.hpp>

#include "core/error.hpp"
#include "core/validate.hpp"
#include "sim/simulation.hpp"
#include "sim/world_set.hpp"
#include "testing/replay.hpp"
#include "vehicles/quadrotor.hpp"
#include "world/world_file.hpp"

// ---------------------------------------------------------------------------
// THE SCENARIO FILE -- the determinism corpus, as DATA (S5, spec Addendum A
// section 15's "the golden corpus becomes data").
//
// TEST SUPPORT, NOT ENGINE API, exactly like engine/testing/replay.hpp beside
// it: header-only, deliberately NOT installed, NOT exported, and not compiled
// into any shipped target (engine/CMakeLists.txt installs headers per
// DIRECTORY -- core/, world/, state/, physics/, sim/, sensors/, vehicles/ --
// and testing/ is in none of them). It is included by tests/ only, which
// is why it may use std::function, std::string and yaml-cpp freely.
//
// ---------------------------------------------------------------------------
// WHAT PROBLEM IT SOLVES
//
// S1-S4 built its four golden scenarios as C++ builder lambdas in
// tests/test_determinism.cpp, with their digests in tests/golden/*.digest. That
// pairing worked, and it made two things impossible:
//
//   * A SCENARIO COULD NOT BE READ WITHOUT READING C++. The thing a golden
//     digest pins -- which worlds, at which seeds, under which materials, with
//     which spawns and which inputs -- was spread across a lambda, a helper
//     and a comment. A reviewer asking "what exactly does bounce run?" had to
//     reconstruct it.
//   * A SCENARIO COULD NOT BE AUTHORED WITHOUT RECOMPILING. Which means no
//     tool, no editor and no training harness could ever produce one.
//
// A scenario file is that description, in one document, next to the world file
// it names and next to the digest it produces. `expected_digest` lives IN the
// file for the same reason: a scenario and its golden are one artifact, and
// splitting them across two files was one more thing to keep in step.
//
// ---------------------------------------------------------------------------
// SCHEMA v1, IN FULL (the top level, in order)
//
//   scenario_version: 1        # the ONE upgrade path; see the strictness note
//   name: "bounce"             # the scenario's identity; matches the filename
//   world: "../worlds/bounce.world.yaml"
//   instances: [ ... ]         # ONE ENTRY PER WORLD, each resolving `world`
//   step: {dt_ns, substeps, steps}
//   models: [ {quadrotor: {...}} ]        # OPTIONAL; omit for body-only runs
//   spawns: [ {world, body: {...}} | {world, vehicle: {...}} ]
//   inputs: [ {tick, wrench: {...}} | {tick, rotor_commands: {...}} ]
//   expected_digest: "0x...."  # the committed golden
//
// EVERY KEY IS REQUIRED except `models` (a scenario with no vehicles has no
// models to declare), and an UNKNOWN KEY IS AN ERROR AT EVERY LEVEL -- the
// world file's posture, adopted verbatim and for its reason: a v1 reader must
// never silently drop a field a later version added. `scenario_version` is the
// one upgrade path, and it is checked BEFORE the unknown-key sweep so that a
// v2 file is answered with "this build reads version 1" rather than with a
// true but useless complaint about the first key it did not recognize.
//
// ---------------------------------------------------------------------------
// THE FIVE THINGS WORTH KNOWING BEFORE WRITING ONE
//
// 1. `world` IS RELATIVE TO THE SCENARIO FILE'S OWN DIRECTORY, which is the
//    only rule that needs no out-of-band knowledge of where a corpus lives:
//    the loader resolves it against `path.parent_path()`, so the committed
//    corpus writes "../worlds/<name>.world.yaml" and a scenario anywhere else
//    works the same way. The file is loaded ONCE, however many instances name
//    it, and every instance shares the resulting WorldDesc.
//
// 2. `instances` CARRIES EXPLICIT PER-INSTANCE SEEDS, which is why this loader
//    builds a WorldSetDesc directly instead of going through world_set_from()
//    /replicate(). Those exist to DERIVE a fleet's seeds from one scene seed,
//    and the seed-derivation authority doctrine says nothing else may re-spell
//    that formula. A scenario file does not derive anything: it PINS the exact
//    seed, material, grid and turbulence of every world, because a golden
//    digest is only meaningful against an exactly-pinned set -- and because
//    heterogeneous instances are the point (bounce's four worlds differ in
//    restitution, and that is what puts CollisionDynamic on its per-world
//    branch).
//
// 3. FLOATS ARE 9-SIGNIFICANT-DIGIT %.9g FORMS, parsed with std::from_chars on
//    the raw scalar string -- never yaml-cpp's as<float>(), never strtof.
//    world_file.cpp's long note is the authority; the short version is that 9
//    digits is exactly FLT_DECIMAL_DIG (every binary32 survives the round
//    trip) and that [charconv] is the one conversion the standard defines with
//    NO locale dependence, while as<float>() runs through an iostream and
//    strtof through LC_NUMERIC. The Qt editor host WILL call setlocale.
//
// 4. `turbulence` TAKES A LEVEL NAME OR A FULL RECORD. "none" / "light" /
//    "moderate" / "severe" mean exactly dryden_params(TurbulenceLevel::x) --
//    the same factory a builder scenario called -- and a mapping spells the
//    eight DrydenParams fields out for a world that wants something else.
//
// 5. `rotor_omega: hover` MEANS vehicles::hover_command(params) AT STANDARD
//    GRAVITY: the common shaft speed at which THIS airframe's four rotors
//    produce exactly its own weight. It is the model's own function of its own
//    parameters, not a number transcribed into the file, so retuning an
//    airframe retunes its trim. A world flying at non-standard gravity is not
//    covered by the shorthand -- write the explicit scalar there, because
//    deriving the magnitude from the world's gravity vector would mean
//    length(g), i.e. a square root of a squared float, which can land an ulp
//    off the value the caller wrote.
//
// ---------------------------------------------------------------------------
// THE INPUT SCRIPT IS STATE-FIRST, AND THAT IS WHAT MAKES IT REPLAYABLE
//
// An input names its target by SPAWN INDEX -- "the body/vehicle the third
// `spawns` entry created" -- and the dispatcher turns that into a live ref at
// the moment it fires, through Simulation::body_ref_at() or
// Simulation::vehicle_ref_at(). It never holds a ref captured at setup,
// because a ref captured before a snapshot is stale after a restore (the
// restore rewinds the generation counters with everything else). That is the
// property replay.hpp's Scenario comment asks of an input script, and it is
// what lets a scenario be resumed from an arbitrary tick.
//
// Entries need not be sorted by tick; every entry whose tick matches is
// applied, IN FILE ORDER, before that tick's step. File order is the contract:
// two commands to the same rotor in one tick resolve last-one-wins, and which
// one is last must be a property of the document.
// ---------------------------------------------------------------------------

namespace spade::testing {

// The schema version this build reads. A file carrying any other value is
// rejected rather than guessed at -- version gating IS the upgrade path, which
// is precisely why unknown keys are an error.
inline constexpr uint32_t kScenarioFileVersion = 1;

// ---------------------------------------------------------------------------
// One `spawns` entry, parsed. Both shapes in one struct rather than a variant:
// a test reads `data->spawns[1].body` and the alternative would make that a
// std::get_if dance for no gain.
//
// `local_body_slot` and `vehicle_ordinal` are DERIVED at load time and are what
// the input dispatcher addresses through -- see the state-first note above.
// Body slots are allocated lowest-free-first in call order, so an entry's
// world-local slot is simply how many earlier entries (of either kind) targeted
// the same world; its vehicle ordinal is how many earlier VEHICLE entries did.
// ---------------------------------------------------------------------------
struct ScenarioSpawn {
    enum class Kind { body, vehicle };

    Kind kind = Kind::body;
    uint32_t world = 0;

    // kind == body
    BodySpawn body{};
    std::vector<DragElementSpawn> drag_elements;

    // kind == vehicle
    uint32_t model_index = 0;  // index into ScenarioData::models
    VehicleSpawn vehicle{};    // rotor_omega already resolved ("hover" -> the trim)

    uint32_t local_body_slot = 0;
    uint32_t vehicle_ordinal = 0;  // meaningful only when kind == vehicle
};

// One `inputs` entry, parsed.
struct ScenarioInput {
    enum class Kind { wrench, rotor_commands };

    Kind kind = Kind::wrench;
    uint64_t tick = 0;
    uint32_t spawn_index = 0;  // which `spawns` entry this addresses

    // kind == wrench
    glm::vec3 force{0.0f};   // world frame, N
    glm::vec3 torque{0.0f};  // body frame, N m

    // kind == rotor_commands
    std::vector<float> omega;  // rad/s, one per rotor, in the model's declaration order
};

// ---------------------------------------------------------------------------
// The whole parsed document. Held by shared_ptr behind LoadedScenario so the
// three Scenario closures and any test reading the data see ONE immutable copy.
// ---------------------------------------------------------------------------
struct ScenarioData {
    std::string name;
    std::filesystem::path world_path;  // `world`, resolved against the scenario file's directory

    // One entry per `instances` entry, every one carrying the SAME resolved
    // WorldDesc. This is what scenario.build() hands back.
    WorldSetDesc worlds;

    uint64_t dt_ns = 0;
    uint32_t substeps = 1;
    uint64_t steps = 0;

    std::vector<vehicles::ModelType> models;
    std::vector<ScenarioSpawn> spawns;
    std::vector<ScenarioInput> inputs;

    uint64_t expected_digest = 0;
};

// What scenario_from_yaml() returns: a replay.hpp Scenario ready to run, and
// the data it was built from, for a test that wants to re-spawn one of its
// bodies somewhere else without transcribing it.
struct LoadedScenario {
    Scenario scenario;
    std::shared_ptr<const ScenarioData> data;
};

namespace scenario_detail {

// ===========================================================================
// THE PARSE PRIMITIVES.
//
// These are world/world_file.cpp's, deliberately: same shapes, same
// diagnostics, same locale-free number path. They are COPIED rather than
// shared because world_file.cpp's live in an anonymous namespace inside a .cpp
// -- sharing them would mean exporting a parsing toolkit from the engine's
// public surface, or installing a test header, and neither is worth it for
// nine small functions. If a third parser ever appears, THAT is the moment to
// promote them to a real header; two is not.
// ===========================================================================

template <std::integral T>
[[nodiscard]] inline std::string dec(T value) {
    char buf[24];
    const std::to_chars_result r = std::to_chars(buf, buf + sizeof(buf), value, 10);
    return std::string(buf, r.ec == std::errc{} ? r.ptr : buf);
}

// std::filesystem::path::string() can throw on a path the native narrow
// encoding cannot represent, and nothing may be thrown out of a Spade-shaped
// API. world_file.cpp's helper, verbatim.
[[nodiscard]] inline std::string path_text(const std::filesystem::path& path) noexcept {
    try {
        return path.string();
    } catch (...) {
        return "<unprintable path>";
    }
}

[[nodiscard]] inline std::string mark_of(const YAML::Node& node) {
    const YAML::Mark mark = node.Mark();
    if (mark.is_null()) return {};
    return " (line " + dec(mark.line + 1) + ", column " + dec(mark.column + 1) + ")";
}

[[nodiscard]] inline Error at(const YAML::Node& node, std::string message) {
    return Error{Code::invalid_argument, std::move(message) + mark_of(node)};
}

// A mapping with EVERY key accounted for. Unknown keys and duplicate keys are
// both rejected: the first would let a v2 field pass unnoticed through a v1
// reader, the second is a silent last-one-wins in every YAML library there is.
[[nodiscard]] inline Result<void> check_map(const YAML::Node& node, std::string_view what,
                                            std::initializer_list<std::string_view> allowed) {
    if (!node.IsMap()) {
        return std::unexpected(at(node, std::string(what) + " must be a mapping"));
    }
    std::vector<std::string> seen;
    seen.reserve(allowed.size());
    for (YAML::const_iterator it = node.begin(); it != node.end(); ++it) {
        if (!it->first.IsScalar()) {
            return std::unexpected(at(it->first, "non-scalar key in " + std::string(what)));
        }
        const std::string key = it->first.Scalar();
        if (std::find(allowed.begin(), allowed.end(), std::string_view(key)) == allowed.end()) {
            return std::unexpected(at(it->first, "unknown key '" + key + "' in " + std::string(what) +
                                                     " -- scenario schema v" +
                                                     dec(kScenarioFileVersion) +
                                                     " does not define it"));
        }
        if (std::find(seen.begin(), seen.end(), key) != seen.end()) {
            return std::unexpected(
                at(it->first, "duplicate key '" + key + "' in " + std::string(what)));
        }
        seen.push_back(key);
    }
    return {};
}

[[nodiscard]] inline Result<YAML::Node> field(const YAML::Node& map, const char* key,
                                              std::string_view what) {
    const YAML::Node child = map[key];
    if (!child.IsDefined() || child.IsNull()) {
        return std::unexpected(
            at(map, "missing required key '" + std::string(key) + "' in " + std::string(what)));
    }
    return child;
}

[[nodiscard]] inline bool has(const YAML::Node& map, const char* key) {
    const YAML::Node child = map[key];
    return child.IsDefined() && !child.IsNull();
}

[[nodiscard]] inline Result<std::string> scalar(const YAML::Node& node, std::string_view what) {
    if (!node.IsScalar()) {
        return std::unexpected(at(node, std::string(what) + " must be a scalar"));
    }
    return node.Scalar();
}

// node.Scalar() then from_chars, NEVER node.as<float>(). See point 3 of this
// header's note, and world_file.cpp's long-form version of it.
[[nodiscard]] inline Result<float> parse_float(const YAML::Node& node, std::string_view what) {
    const Result<std::string> text = scalar(node, what);
    if (!text) return std::unexpected(text.error());
    std::string_view body(*text);
    // from_chars rejects a leading '+'; the corpus never writes one, but a hand
    // editor may, and refusing "+1.5" would be pedantry with no payoff.
    if (!body.empty() && body.front() == '+') body.remove_prefix(1);

    float value = 0.0f;
    const std::from_chars_result r = std::from_chars(body.data(), body.data() + body.size(), value);
    if (r.ec == std::errc::result_out_of_range) {
        return std::unexpected(at(node, std::string(what) + ": '" + *text +
                                            "' is outside the range of a 32-bit float"));
    }
    if (r.ec != std::errc{} || r.ptr != body.data() + body.size()) {
        return std::unexpected(
            at(node, std::string(what) + ": '" + *text + "' is not a decimal number"));
    }
    if (!finite(value)) {
        return std::unexpected(at(node, std::string(what) + ": '" + *text +
                                            "' is not finite; an infinity or a NaN in a scenario is "
                                            "always an authoring error"));
    }
    return value;
}

template <std::unsigned_integral T>
[[nodiscard]] inline Result<T> parse_uint(const YAML::Node& node, std::string_view what) {
    const Result<std::string> text = scalar(node, what);
    if (!text) return std::unexpected(text.error());
    std::string_view body(*text);
    if (!body.empty() && body.front() == '+') body.remove_prefix(1);

    // Hex is accepted for `seed` and for `expected_digest`, which are the two
    // fields a human reads as bit patterns rather than as quantities; every
    // other integer in the schema is a count and is written in decimal. The
    // prefix decides the base, so nothing is ambiguous.
    //
    // THE ONE DELIBERATE DIVERGENCE FROM world_file.cpp's parse_uint(), worth
    // stating rather than leaving for a future diff to wonder about: that
    // copy is decimal-only (base 10, unconditionally) and its "not a
    // non-negative decimal integer" message has no hex clause. That is not a
    // missed feature over there -- world_file.cpp's own `seed` field
    // (Environment::seed) is a real round-trip: world_to_yaml()'s emit()
    // always writes it with dec(), never a hex literal, so world_from_yaml()
    // never needs to read one back. A scenario file is never written by this
    // engine (scenario_file.hpp is read-only test support, "never installed
    // or shipped" -- docs/dev/testing.md), so its `seed`/`expected_digest`
    // fields are hand-authored, and hex is the natural spelling for a value a
    // human reads as bits (a digest, a seed transcribed from a debug print)
    // -- hence the accommodation exists on THIS side of the copy and not the
    // other. If a future edit to either parse_uint ever needs to bring the
    // two back in sync, this is the one place they are meant to disagree.
    int base = 10;
    if (body.size() > 2 && body[0] == '0' && (body[1] == 'x' || body[1] == 'X')) {
        base = 16;
        body.remove_prefix(2);
    }

    T value = 0;
    const std::from_chars_result r =
        std::from_chars(body.data(), body.data() + body.size(), value, base);
    if (r.ec == std::errc::result_out_of_range) {
        return std::unexpected(at(node, std::string(what) + ": '" + *text + "' does not fit in " +
                                            dec(sizeof(T) * 8) + " unsigned bits"));
    }
    if (r.ec != std::errc{} || r.ptr != body.data() + body.size()) {
        return std::unexpected(at(node, std::string(what) + ": '" + *text +
                                            "' is not a non-negative decimal or 0x-hex integer"));
    }
    return value;
}

template <std::size_t N>
[[nodiscard]] inline Result<std::array<float, N>> parse_floats(const YAML::Node& node,
                                                               std::string_view what) {
    if (!node.IsSequence()) {
        return std::unexpected(
            at(node, std::string(what) + " must be a sequence of " + dec(N) + " numbers"));
    }
    if (node.size() != N) {
        return std::unexpected(at(node, std::string(what) + " must hold exactly " + dec(N) +
                                            " numbers, found " + dec(node.size())));
    }
    std::array<float, N> out{};
    for (std::size_t i = 0; i < N; ++i) {
        const Result<float> value = parse_float(node[i], std::string(what) + "[" + dec(i) + "]");
        if (!value) return std::unexpected(value.error());
        out[i] = *value;
    }
    return out;
}

// --- composed field readers; `what` is the containing mapping's dotted path --

[[nodiscard]] inline Result<float> float_field(const YAML::Node& map, const char* key,
                                               std::string_view what) {
    const Result<YAML::Node> node = field(map, key, what);
    if (!node) return std::unexpected(node.error());
    return parse_float(*node, std::string(what) + "." + key);
}

template <std::unsigned_integral T>
[[nodiscard]] inline Result<T> uint_field(const YAML::Node& map, const char* key,
                                          std::string_view what) {
    const Result<YAML::Node> node = field(map, key, what);
    if (!node) return std::unexpected(node.error());
    return parse_uint<T>(*node, std::string(what) + "." + key);
}

template <std::size_t N>
[[nodiscard]] inline Result<std::array<float, N>> floats_field(const YAML::Node& map,
                                                               const char* key,
                                                               std::string_view what) {
    const Result<YAML::Node> node = field(map, key, what);
    if (!node) return std::unexpected(node.error());
    return parse_floats<N>(*node, std::string(what) + "." + key);
}

[[nodiscard]] inline Result<std::string> string_field(const YAML::Node& map, const char* key,
                                                      std::string_view what) {
    const Result<YAML::Node> node = field(map, key, what);
    if (!node) return std::unexpected(node.error());
    return scalar(*node, std::string(what) + "." + key);
}

// Optional field readers -- for the handful of keys whose absence means "the
// engine's default", each named in the schema comment where it is read.
[[nodiscard]] inline Result<glm::vec3> vec3_field(const YAML::Node& map, const char* key,
                                                  std::string_view what) {
    const Result<std::array<float, 3>> v = floats_field<3>(map, key, what);
    if (!v) return std::unexpected(v.error());
    return glm::vec3((*v)[0], (*v)[1], (*v)[2]);
}

// [w, x, y, z] in the file, which is glm::quat's constructor order. NOT
// re-normalized here: the spawn path normalizes, and silently fixing a file
// would hide an authoring error the engine is willing to report.
[[nodiscard]] inline Result<glm::quat> quat_field(const YAML::Node& map, const char* key,
                                                  std::string_view what) {
    const Result<std::array<float, 4>> q = floats_field<4>(map, key, what);
    if (!q) return std::unexpected(q.error());
    return glm::quat((*q)[0], (*q)[1], (*q)[2], (*q)[3]);
}

// ===========================================================================
// Block parsers
// ===========================================================================

[[nodiscard]] inline Result<DrydenParams> parse_turbulence(const YAML::Node& node,
                                                           std::string_view what) {
    // The level shorthand: exactly dryden_params(TurbulenceLevel::x), the same
    // factory a builder scenario called, so a migrated scenario says what its
    // predecessor said rather than a transcription of what that produced.
    if (node.IsScalar()) {
        const std::string level = node.Scalar();
        if (level == "none") return dryden_params(TurbulenceLevel::none);
        if (level == "light") return dryden_params(TurbulenceLevel::light);
        if (level == "moderate") return dryden_params(TurbulenceLevel::moderate);
        if (level == "severe") return dryden_params(TurbulenceLevel::severe);
        return std::unexpected(at(node, std::string(what) + ": unknown turbulence level '" + level +
                                            "' (expected none, light, moderate, severe, or a "
                                            "mapping of the DrydenParams fields)"));
    }

    constexpr std::string_view kFields[] = {"scale_u", "scale_v", "scale_w", "sigma_u",
                                            "sigma_v", "sigma_w", "reference_airspeed"};
    if (Result<void> r = check_map(node, what,
                                   {kFields[0], kFields[1], kFields[2], kFields[3], kFields[4],
                                    kFields[5], kFields[6]});
        !r) {
        return std::unexpected(r.error());
    }
    DrydenParams params;  // _reserved0 stays 0; the file never carries it
    float* const targets[] = {&params.scale_u, &params.scale_v, &params.scale_w, &params.sigma_u,
                              &params.sigma_v, &params.sigma_w, &params.reference_airspeed};
    for (std::size_t i = 0; i < std::size(kFields); ++i) {
        const Result<float> value = float_field(node, std::string(kFields[i]).c_str(), what);
        if (!value) return std::unexpected(value.error());
        *targets[i] = *value;
    }
    return params;
}

[[nodiscard]] inline Result<physics::ContactParams> parse_contacts(const YAML::Node& node,
                                                                   std::string_view what) {
    if (Result<void> r = check_map(
            node, what, {"restitution_e", "friction_mu", "baumgarte_beta", "slop", "proxy_radius"});
        !r) {
        return std::unexpected(r.error());
    }
    physics::ContactParams contacts;  // the three reserved lanes stay 0
    const char* const keys[] = {"restitution_e", "friction_mu", "baumgarte_beta", "slop",
                                "proxy_radius"};
    float* const targets[] = {&contacts.restitution_e, &contacts.friction_mu,
                              &contacts.baumgarte_beta, &contacts.slop, &contacts.proxy_radius};
    for (std::size_t i = 0; i < std::size(keys); ++i) {
        const Result<float> value = float_field(node, keys[i], what);
        if (!value) return std::unexpected(value.error());
        *targets[i] = *value;
    }
    return contacts;
}

[[nodiscard]] inline Result<physics::GridParams> parse_grid(const YAML::Node& node,
                                                            std::string_view what) {
    if (Result<void> r = check_map(node, what, {"cell_size"}); !r) {
        return std::unexpected(r.error());
    }
    const Result<float> cell_size = float_field(node, "cell_size", what);
    if (!cell_size) return std::unexpected(cell_size.error());
    physics::GridParams grid;  // the three reserved lanes stay 0
    grid.cell_size = *cell_size;
    return grid;
}

[[nodiscard]] inline Result<std::vector<DragElementSpawn>> parse_drag_elements(
    const YAML::Node& node, std::string_view what) {
    if (!node.IsSequence()) {
        return std::unexpected(at(node, std::string(what) + " must be a sequence"));
    }
    std::vector<DragElementSpawn> elements;
    elements.reserve(node.size());
    for (std::size_t i = 0; i < node.size(); ++i) {
        const std::string where = std::string(what) + "[" + dec(i) + "]";
        const YAML::Node entry = node[i];
        if (Result<void> r =
                check_map(entry, where, {"mode", "area", "coeffs", "local_pos", "local_orient"});
            !r) {
            return std::unexpected(r.error());
        }
        const Result<std::string> mode = string_field(entry, "mode", where);
        if (!mode) return std::unexpected(mode.error());

        DragElementSpawn drag;
        // NAMES, never the underlying integers -- world_file.cpp's rule for
        // SdfPrim, applied to drag_mode for its reason: an integer in a text
        // format is a promise the enumeration never gets reordered.
        if (*mode == "quadratic") {
            drag.mode = physics::drag_mode::quadratic;
        } else if (*mode == "componentwise") {
            drag.mode = physics::drag_mode::componentwise;
        } else {
            return std::unexpected(at(entry["mode"], where + ".mode: unknown drag mode '" + *mode +
                                                         "' (expected quadratic or componentwise)"));
        }
        const Result<float> area = float_field(entry, "area", where);
        if (!area) return std::unexpected(area.error());
        const Result<glm::vec3> coeffs = vec3_field(entry, "coeffs", where);
        if (!coeffs) return std::unexpected(coeffs.error());
        const Result<glm::vec3> local_pos = vec3_field(entry, "local_pos", where);
        if (!local_pos) return std::unexpected(local_pos.error());
        const Result<glm::quat> local_orient = quat_field(entry, "local_orient", where);
        if (!local_orient) return std::unexpected(local_orient.error());

        drag.area = *area;
        drag.coeffs = *coeffs;
        drag.local_pos = *local_pos;
        drag.local_orient = *local_orient;
        elements.push_back(drag);
    }
    return elements;
}

[[nodiscard]] inline Result<vehicles::QuadrotorParams> parse_quadrotor(const YAML::Node& node,
                                                                       std::string_view what) {
    if (Result<void> r = check_map(node, what,
                                   {"name", "param_schema_id", "visual_ref", "mass", "inertia_diag",
                                    "arm_length", "rotor_height", "proxy_radius", "rotors",
                                    "spin_dirs", "drag", "imu"});
        !r) {
        return std::unexpected(r.error());
    }

    vehicles::QuadrotorParams params;
    const Result<std::string> name = string_field(node, "name", what);
    if (!name) return std::unexpected(name.error());
    params.name = *name;
    const Result<uint32_t> schema_id = uint_field<uint32_t>(node, "param_schema_id", what);
    if (!schema_id) return std::unexpected(schema_id.error());
    params.param_schema_id = *schema_id;
    const Result<std::string> visual_ref = string_field(node, "visual_ref", what);
    if (!visual_ref) return std::unexpected(visual_ref.error());
    params.visual_ref = *visual_ref;

    const Result<float> mass = float_field(node, "mass", what);
    if (!mass) return std::unexpected(mass.error());
    params.mass = *mass;
    const Result<glm::vec3> inertia = vec3_field(node, "inertia_diag", what);
    if (!inertia) return std::unexpected(inertia.error());
    params.inertia_diag = *inertia;
    const Result<float> arm_length = float_field(node, "arm_length", what);
    if (!arm_length) return std::unexpected(arm_length.error());
    params.arm_length = *arm_length;
    const Result<float> rotor_height = float_field(node, "rotor_height", what);
    if (!rotor_height) return std::unexpected(rotor_height.error());
    params.rotor_height = *rotor_height;
    const Result<float> proxy_radius = float_field(node, "proxy_radius", what);
    if (!proxy_radius) return std::unexpected(proxy_radius.error());
    params.proxy_radius = *proxy_radius;

    const Result<YAML::Node> rotors = field(node, "rotors", what);
    if (!rotors) return std::unexpected(rotors.error());
    if (!rotors->IsSequence() || rotors->size() != vehicles::kQuadrotorRotorCount) {
        return std::unexpected(at(*rotors, std::string(what) + ".rotors must be a sequence of " +
                                               dec(vehicles::kQuadrotorRotorCount) +
                                               " rotor calibrations"));
    }
    for (std::size_t i = 0; i < vehicles::kQuadrotorRotorCount; ++i) {
        const std::string where = std::string(what) + ".rotors[" + dec(i) + "]";
        const YAML::Node entry = (*rotors)[i];
        if (Result<void> r =
                check_map(entry, where, {"tau", "radius", "thrust_coeff", "torque_coeff"});
            !r) {
            return std::unexpected(r.error());
        }
        const char* const keys[] = {"tau", "radius", "thrust_coeff", "torque_coeff"};
        float* const targets[] = {&params.rotors[i].tau, &params.rotors[i].radius,
                                  &params.rotors[i].thrust_coeff, &params.rotors[i].torque_coeff};
        for (std::size_t k = 0; k < std::size(keys); ++k) {
            const Result<float> value = float_field(entry, keys[k], where);
            if (!value) return std::unexpected(value.error());
            *targets[k] = *value;
        }
    }

    const Result<std::array<float, vehicles::kQuadrotorRotorCount>> spins =
        floats_field<vehicles::kQuadrotorRotorCount>(node, "spin_dirs", what);
    if (!spins) return std::unexpected(spins.error());
    params.spin_dirs = *spins;

    const Result<YAML::Node> drag_node = field(node, "drag", what);
    if (!drag_node) return std::unexpected(drag_node.error());
    // The airframe's one drag body, spelled with the same keys a spawn's drag
    // element uses -- one sequence entry, because DragBodyDesc IS one element.
    const Result<std::vector<DragElementSpawn>> drag =
        parse_drag_elements(*drag_node, std::string(what) + ".drag");
    if (!drag) return std::unexpected(drag.error());
    if (drag->size() != 1) {
        return std::unexpected(at(*drag_node, std::string(what) +
                                                  ".drag must hold exactly one element (a "
                                                  "quadrotor carries one bluff-body drag element "
                                                  "at the COM)"));
    }
    params.drag.mode = (*drag)[0].mode;
    params.drag.area = (*drag)[0].area;
    params.drag.coeffs = (*drag)[0].coeffs;
    params.drag.local_pos = (*drag)[0].local_pos;
    params.drag.local_orient = (*drag)[0].local_orient;

    const Result<YAML::Node> imu_node = field(node, "imu", what);
    if (!imu_node) return std::unexpected(imu_node.error());
    const std::string imu_what = std::string(what) + ".imu";
    if (Result<void> r = check_map(*imu_node, imu_what,
                                   {"mount_pos", "mount_orient", "rate_divider", "sigma_a",
                                    "sigma_g", "sigma_ba", "sigma_bg"});
        !r) {
        return std::unexpected(r.error());
    }
    const Result<glm::vec3> mount_pos = vec3_field(*imu_node, "mount_pos", imu_what);
    if (!mount_pos) return std::unexpected(mount_pos.error());
    const Result<glm::quat> mount_orient = quat_field(*imu_node, "mount_orient", imu_what);
    if (!mount_orient) return std::unexpected(mount_orient.error());
    const Result<uint32_t> rate_divider = uint_field<uint32_t>(*imu_node, "rate_divider", imu_what);
    if (!rate_divider) return std::unexpected(rate_divider.error());
    params.imu.mount_pos = *mount_pos;
    params.imu.mount_orient = *mount_orient;
    params.imu.rate_divider = *rate_divider;
    const char* const sigma_keys[] = {"sigma_a", "sigma_g", "sigma_ba", "sigma_bg"};
    float* const sigma_targets[] = {&params.imu.sigma_a, &params.imu.sigma_g, &params.imu.sigma_ba,
                                    &params.imu.sigma_bg};
    for (std::size_t i = 0; i < std::size(sigma_keys); ++i) {
        const Result<float> value = float_field(*imu_node, sigma_keys[i], imu_what);
        if (!value) return std::unexpected(value.error());
        *sigma_targets[i] = *value;
    }
    return params;
}

}  // namespace scenario_detail

// ---------------------------------------------------------------------------
// scenario_from_yaml -- parse a scenario file and adapt it to a runnable
// replay.hpp Scenario.
//
// Codes: io_error (the scenario file or the world file it names cannot be
// read); schema_mismatch (the file states a `scenario_version` this build does
// not read -- the one failure a caller may answer by upgrading rather than by
// editing, so it is distinguishable without matching on a string, exactly as
// world_from_yaml() reports the same situation); invalid_argument for every
// other rejection. A world file's own diagnostics come through verbatim, with
// the path load_world_file() already prefixed.
// ---------------------------------------------------------------------------
[[nodiscard]] inline Result<LoadedScenario> scenario_from_yaml(const std::filesystem::path& path) {
    using namespace scenario_detail;

    std::string text;
    try {
        std::ifstream in(path, std::ios::binary);
        if (!in) {
            return std::unexpected(
                Error{Code::io_error, "cannot open scenario file '" + path_text(path) + "'"});
        }
        text.assign(std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>());
        if (in.bad()) {
            return std::unexpected(
                Error{Code::io_error, "failed reading scenario file '" + path_text(path) + "'"});
        }
    } catch (const std::exception& e) {
        return std::unexpected(Error{Code::io_error, "failed reading scenario file '" +
                                                         path_text(path) + "': " + e.what()});
    }

    // Each declared model's hover trim, computed from its own QuadrotorParams
    // as the models are parsed, so that a `rotor_omega: hover` spawn below can
    // resolve without keeping the params themselves in the scenario data.
    std::vector<float> hover_speeds;

    // The whole parse, wrapped: yaml-cpp reports every syntax failure by
    // throwing, and nothing may leave a Spade-shaped API that way.
    const auto parse = [&]() -> Result<ScenarioData> {
        const YAML::Node root = YAML::Load(text);
        if (!root.IsMap()) {
            return std::unexpected(at(
                root, "a scenario file's top level must be a mapping with a 'scenario_version' key"));
        }

        // VERSION FIRST, before the unknown-key sweep -- world_file.cpp's rule,
        // for its reason: a v2 file is full of keys a v1 reader does not know,
        // and "unknown key 'foo'" would be a true but useless answer to a
        // question whose real answer is "this build does not read version 2".
        const Result<YAML::Node> version_node = field(root, "scenario_version", "scenario");
        if (!version_node) return std::unexpected(version_node.error());
        const Result<uint32_t> version =
            parse_uint<uint32_t>(*version_node, "scenario.scenario_version");
        if (!version) return std::unexpected(version.error());
        if (*version != kScenarioFileVersion) {
            return std::unexpected(Error{Code::schema_mismatch,
                                         "scenario file is schema version " + dec(*version) +
                                             ", this build reads version " +
                                             dec(kScenarioFileVersion) + " only" +
                                             mark_of(*version_node)});
        }

        if (Result<void> r = check_map(root, "scenario",
                                       {"scenario_version", "name", "world", "instances", "step",
                                        "models", "spawns", "inputs", "expected_digest"});
            !r) {
            return std::unexpected(r.error());
        }

        ScenarioData data;

        const Result<std::string> name = string_field(root, "name", "scenario");
        if (!name) return std::unexpected(name.error());
        data.name = *name;

        // --- world -------------------------------------------------------
        const Result<std::string> world_rel = string_field(root, "world", "scenario");
        if (!world_rel) return std::unexpected(world_rel.error());
        data.world_path = path.parent_path() / std::filesystem::path(*world_rel);
        const Result<WorldDesc> world = load_world_file(data.world_path);
        if (!world) return std::unexpected(world.error());

        // --- step --------------------------------------------------------
        const Result<YAML::Node> step_node = field(root, "step", "scenario");
        if (!step_node) return std::unexpected(step_node.error());
        if (Result<void> r = check_map(*step_node, "scenario.step", {"dt_ns", "substeps", "steps"});
            !r) {
            return std::unexpected(r.error());
        }
        const Result<uint64_t> dt_ns = uint_field<uint64_t>(*step_node, "dt_ns", "scenario.step");
        if (!dt_ns) return std::unexpected(dt_ns.error());
        const Result<uint32_t> substeps =
            uint_field<uint32_t>(*step_node, "substeps", "scenario.step");
        if (!substeps) return std::unexpected(substeps.error());
        const Result<uint64_t> steps = uint_field<uint64_t>(*step_node, "steps", "scenario.step");
        if (!steps) return std::unexpected(steps.error());
        data.dt_ns = *dt_ns;
        data.substeps = *substeps;
        data.steps = *steps;

        // --- instances ---------------------------------------------------
        const Result<YAML::Node> instances = field(root, "instances", "scenario");
        if (!instances) return std::unexpected(instances.error());
        if (!instances->IsSequence() || instances->size() == 0) {
            return std::unexpected(
                at(*instances, "scenario.instances must be a non-empty sequence (one entry per "
                               "world, each resolving the same world file)"));
        }
        data.worlds.worlds.reserve(instances->size());
        for (std::size_t i = 0; i < instances->size(); ++i) {
            const std::string what = "scenario.instances[" + dec(i) + "]";
            const YAML::Node entry = (*instances)[i];
            if (Result<void> r = check_map(entry, what, {"seed", "contacts", "grid", "turbulence"});
                !r) {
                return std::unexpected(r.error());
            }
            const Result<uint64_t> seed = uint_field<uint64_t>(entry, "seed", what);
            if (!seed) return std::unexpected(seed.error());
            const Result<YAML::Node> contacts_node = field(entry, "contacts", what);
            if (!contacts_node) return std::unexpected(contacts_node.error());
            const Result<physics::ContactParams> contacts =
                parse_contacts(*contacts_node, what + ".contacts");
            if (!contacts) return std::unexpected(contacts.error());
            const Result<YAML::Node> grid_node = field(entry, "grid", what);
            if (!grid_node) return std::unexpected(grid_node.error());
            const Result<physics::GridParams> grid = parse_grid(*grid_node, what + ".grid");
            if (!grid) return std::unexpected(grid.error());
            const Result<YAML::Node> turbulence_node = field(entry, "turbulence", what);
            if (!turbulence_node) return std::unexpected(turbulence_node.error());
            const Result<DrydenParams> turbulence =
                parse_turbulence(*turbulence_node, what + ".turbulence");
            if (!turbulence) return std::unexpected(turbulence.error());

            WorldInstanceDesc instance;
            instance.world = *world;  // ONE resolved world, shared by every instance
            instance.seed = *seed;
            instance.contacts = *contacts;
            instance.grid = *grid;
            instance.turbulence = *turbulence;
            data.worlds.worlds.push_back(std::move(instance));
        }
        const uint32_t world_count = static_cast<uint32_t>(data.worlds.worlds.size());

        // --- models (optional) -------------------------------------------
        if (has(root, "models")) {
            const YAML::Node models = root["models"];
            if (!models.IsSequence()) {
                return std::unexpected(at(models, "scenario.models must be a sequence"));
            }
            for (std::size_t i = 0; i < models.size(); ++i) {
                const std::string what = "scenario.models[" + dec(i) + "]";
                const YAML::Node entry = models[i];
                // One key, naming the model-type CONSTRUCTOR. A second kind
                // joins this list rather than growing a discriminator field.
                if (Result<void> r = check_map(entry, what, {"quadrotor"}); !r) {
                    return std::unexpected(r.error());
                }
                const Result<YAML::Node> quad_node = field(entry, "quadrotor", what);
                if (!quad_node) return std::unexpected(quad_node.error());
                const Result<vehicles::QuadrotorParams> params =
                    parse_quadrotor(*quad_node, what + ".quadrotor");
                if (!params) return std::unexpected(params.error());
                // make_quadrotor() validates; its message comes through
                // verbatim rather than re-worded, so an author sees the
                // model-type layer's own complaint.
                const Result<vehicles::ModelType> model = vehicles::make_quadrotor(*params);
                if (!model) {
                    return std::unexpected(
                        Error{model.error().code, what + ": " + model.error().context});
                }
                data.models.push_back(*model);
                hover_speeds.push_back(vehicles::hover_command(*params));
            }
        }

        // --- spawns -------------------------------------------------------
        const Result<YAML::Node> spawns = field(root, "spawns", "scenario");
        if (!spawns) return std::unexpected(spawns.error());
        if (!spawns->IsSequence()) {
            return std::unexpected(at(*spawns, "scenario.spawns must be a sequence"));
        }
        std::vector<uint32_t> next_local_slot(world_count, 0);
        std::vector<uint32_t> next_vehicle_ordinal(world_count, 0);
        for (std::size_t i = 0; i < spawns->size(); ++i) {
            const std::string what = "scenario.spawns[" + dec(i) + "]";
            const YAML::Node entry = (*spawns)[i];
            if (Result<void> r = check_map(entry, what, {"world", "body", "vehicle"}); !r) {
                return std::unexpected(r.error());
            }
            const bool is_body = has(entry, "body");
            const bool is_vehicle = has(entry, "vehicle");
            if (is_body == is_vehicle) {
                return std::unexpected(at(entry, what + " must carry exactly one of 'body' (a bare "
                                                       "rigid body) or 'vehicle' (an instance of a "
                                                       "declared model)"));
            }
            const Result<uint32_t> world_index = uint_field<uint32_t>(entry, "world", what);
            if (!world_index) return std::unexpected(world_index.error());
            if (*world_index >= world_count) {
                return std::unexpected(at(entry, what + ".world: " + dec(*world_index) +
                                                     " is outside this scenario's " +
                                                     dec(world_count) + " worlds"));
            }

            ScenarioSpawn spawn;
            spawn.world = *world_index;
            spawn.local_body_slot = next_local_slot[*world_index]++;

            if (is_body) {
                const std::string body_what = what + ".body";
                const YAML::Node body_node = entry["body"];
                if (Result<void> r = check_map(body_node, body_what,
                                               {"pos", "orient", "vel", "omega_body", "mass",
                                                "inv_inertia_diag", "drag_elements"});
                    !r) {
                    return std::unexpected(r.error());
                }
                const Result<glm::vec3> pos = vec3_field(body_node, "pos", body_what);
                if (!pos) return std::unexpected(pos.error());
                const Result<glm::quat> orient = quat_field(body_node, "orient", body_what);
                if (!orient) return std::unexpected(orient.error());
                const Result<glm::vec3> vel = vec3_field(body_node, "vel", body_what);
                if (!vel) return std::unexpected(vel.error());
                const Result<glm::vec3> omega = vec3_field(body_node, "omega_body", body_what);
                if (!omega) return std::unexpected(omega.error());
                const Result<float> mass = float_field(body_node, "mass", body_what);
                if (!mass) return std::unexpected(mass.error());
                const Result<glm::vec3> inv_inertia =
                    vec3_field(body_node, "inv_inertia_diag", body_what);
                if (!inv_inertia) return std::unexpected(inv_inertia.error());

                spawn.kind = ScenarioSpawn::Kind::body;
                spawn.body.pos = *pos;
                spawn.body.orient = *orient;
                spawn.body.vel = *vel;
                spawn.body.omega_body = *omega;
                spawn.body.mass = *mass;
                spawn.body.inv_inertia_diag = *inv_inertia;

                // OPTIONAL, and the one key in the schema whose absence means
                // "none": most bodies carry no force elements at all, and
                // requiring `drag_elements: []` on all 114 of the corpus's
                // bare spawns would be ceremony, not clarity.
                if (has(body_node, "drag_elements")) {
                    const Result<std::vector<DragElementSpawn>> drag =
                        parse_drag_elements(body_node["drag_elements"],
                                            body_what + ".drag_elements");
                    if (!drag) return std::unexpected(drag.error());
                    spawn.drag_elements = *drag;
                }
            } else {
                const std::string vehicle_what = what + ".vehicle";
                const YAML::Node vehicle_node = entry["vehicle"];
                if (Result<void> r = check_map(vehicle_node, vehicle_what,
                                               {"model", "pos", "orient", "vel", "omega_body",
                                                "rotor_omega"});
                    !r) {
                    return std::unexpected(r.error());
                }
                const Result<uint32_t> model_index =
                    uint_field<uint32_t>(vehicle_node, "model", vehicle_what);
                if (!model_index) return std::unexpected(model_index.error());
                if (*model_index >= data.models.size()) {
                    return std::unexpected(at(vehicle_node,
                                              vehicle_what + ".model: " + dec(*model_index) +
                                                  " is outside this scenario's " +
                                                  dec(data.models.size()) + " declared models"));
                }
                const Result<glm::vec3> pos = vec3_field(vehicle_node, "pos", vehicle_what);
                if (!pos) return std::unexpected(pos.error());
                const Result<glm::quat> orient = quat_field(vehicle_node, "orient", vehicle_what);
                if (!orient) return std::unexpected(orient.error());
                const Result<glm::vec3> vel = vec3_field(vehicle_node, "vel", vehicle_what);
                if (!vel) return std::unexpected(vel.error());
                const Result<glm::vec3> omega = vec3_field(vehicle_node, "omega_body", vehicle_what);
                if (!omega) return std::unexpected(omega.error());

                // `hover` or an explicit scalar. ONE scalar, never four:
                // VehicleSpawn::rotor_omega is a single common shaft speed by
                // design (see its header note -- a symmetric airframe's trim
                // IS one speed, and an asymmetric one has no common-speed trim
                // to spawn into), so four-per-rotor spawn speeds are not a
                // thing the engine can express. A scenario that wants unequal
                // speeds spawns in trim and commands them, which is exactly
                // what an `inputs` rotor_commands entry at tick 0 does.
                const Result<YAML::Node> omega_node =
                    field(vehicle_node, "rotor_omega", vehicle_what);
                if (!omega_node) return std::unexpected(omega_node.error());
                float rotor_omega = 0.0f;
                if (omega_node->IsScalar() && omega_node->Scalar() == "hover") {
                    rotor_omega = hover_speeds[*model_index];
                } else {
                    const Result<float> explicit_omega =
                        parse_float(*omega_node, vehicle_what + ".rotor_omega");
                    if (!explicit_omega) return std::unexpected(explicit_omega.error());
                    rotor_omega = *explicit_omega;
                }

                spawn.kind = ScenarioSpawn::Kind::vehicle;
                spawn.model_index = *model_index;
                spawn.vehicle.pos = *pos;
                spawn.vehicle.orient = *orient;
                spawn.vehicle.vel = *vel;
                spawn.vehicle.omega_body = *omega;
                spawn.vehicle.rotor_omega = rotor_omega;
                spawn.vehicle_ordinal = next_vehicle_ordinal[*world_index]++;
            }
            data.spawns.push_back(std::move(spawn));
        }

        // --- inputs -------------------------------------------------------
        const Result<YAML::Node> inputs = field(root, "inputs", "scenario");
        if (!inputs) return std::unexpected(inputs.error());
        if (!inputs->IsSequence()) {
            return std::unexpected(at(*inputs, "scenario.inputs must be a sequence"));
        }
        for (std::size_t i = 0; i < inputs->size(); ++i) {
            const std::string what = "scenario.inputs[" + dec(i) + "]";
            const YAML::Node entry = (*inputs)[i];
            if (Result<void> r = check_map(entry, what, {"tick", "wrench", "rotor_commands"}); !r) {
                return std::unexpected(r.error());
            }
            const bool is_wrench = has(entry, "wrench");
            const bool is_command = has(entry, "rotor_commands");
            if (is_wrench == is_command) {
                return std::unexpected(at(entry, what + " must carry exactly one of 'wrench' or "
                                                       "'rotor_commands'"));
            }
            const Result<uint64_t> tick = uint_field<uint64_t>(entry, "tick", what);
            if (!tick) return std::unexpected(tick.error());
            if (*tick >= data.steps) {
                // An input at or past the last tick never fires. Silently
                // ignoring it would let a scenario carry a script nobody
                // notices is dead.
                return std::unexpected(at(entry, what + ".tick: " + dec(*tick) +
                                                     " is at or past the run's " + dec(data.steps) +
                                                     " steps, so it would never be applied"));
            }

            ScenarioInput input;
            input.tick = *tick;
            if (is_wrench) {
                const std::string wrench_what = what + ".wrench";
                const YAML::Node wrench = entry["wrench"];
                if (Result<void> r = check_map(wrench, wrench_what, {"body", "force", "torque"});
                    !r) {
                    return std::unexpected(r.error());
                }
                const Result<uint32_t> target = uint_field<uint32_t>(wrench, "body", wrench_what);
                if (!target) return std::unexpected(target.error());
                if (*target >= data.spawns.size()) {
                    return std::unexpected(at(wrench, wrench_what + ".body: " + dec(*target) +
                                                          " is outside this scenario's " +
                                                          dec(data.spawns.size()) + " spawns"));
                }
                const Result<glm::vec3> force = vec3_field(wrench, "force", wrench_what);
                if (!force) return std::unexpected(force.error());
                const Result<glm::vec3> torque = vec3_field(wrench, "torque", wrench_what);
                if (!torque) return std::unexpected(torque.error());

                input.kind = ScenarioInput::Kind::wrench;
                input.spawn_index = *target;
                input.force = *force;
                input.torque = *torque;
            } else {
                const std::string command_what = what + ".rotor_commands";
                const YAML::Node command = entry["rotor_commands"];
                if (Result<void> r = check_map(command, command_what, {"vehicle", "omega"}); !r) {
                    return std::unexpected(r.error());
                }
                const Result<uint32_t> target =
                    uint_field<uint32_t>(command, "vehicle", command_what);
                if (!target) return std::unexpected(target.error());
                if (*target >= data.spawns.size()) {
                    return std::unexpected(at(command, command_what + ".vehicle: " + dec(*target) +
                                                           " is outside this scenario's " +
                                                           dec(data.spawns.size()) + " spawns"));
                }
                if (data.spawns[*target].kind != ScenarioSpawn::Kind::vehicle) {
                    return std::unexpected(at(command, command_what + ".vehicle: spawn " +
                                                           dec(*target) +
                                                           " is a bare body, which has no rotors"));
                }
                const YAML::Node omega_node = command["omega"];
                if (!omega_node.IsSequence()) {
                    return std::unexpected(
                        at(omega_node, command_what + ".omega must be a sequence"));
                }
                const std::size_t expected =
                    data.models[data.spawns[*target].model_index].rotors.size();
                if (omega_node.size() != expected) {
                    return std::unexpected(at(omega_node, command_what + ".omega must hold exactly " +
                                                              dec(expected) +
                                                              " commands (one per rotor the model "
                                                              "declares), found " +
                                                              dec(omega_node.size())));
                }
                std::vector<float> omega;
                omega.reserve(expected);
                for (std::size_t k = 0; k < expected; ++k) {
                    const Result<float> value =
                        parse_float(omega_node[k], command_what + ".omega[" + dec(k) + "]");
                    if (!value) return std::unexpected(value.error());
                    omega.push_back(*value);
                }

                input.kind = ScenarioInput::Kind::rotor_commands;
                input.spawn_index = *target;
                input.omega = std::move(omega);
            }
            data.inputs.push_back(std::move(input));
        }

        // --- expected_digest ----------------------------------------------
        const Result<uint64_t> digest = uint_field<uint64_t>(root, "expected_digest", "scenario");
        if (!digest) return std::unexpected(digest.error());
        data.expected_digest = *digest;

        return data;
    };

    Result<ScenarioData> data = Result<ScenarioData>{};
    try {
        data = parse();
    } catch (const YAML::Exception& e) {
        return std::unexpected(Error{Code::invalid_argument, std::string(e.what())});
    } catch (const std::exception& e) {
        return std::unexpected(
            Error{Code::internal, std::string("scenario file parse failed: ") + e.what()});
    }
    if (!data) {
        // The parse diagnostics carry a YAML mark; only this layer knows which
        // file they came from. A world-file error already carries its own path
        // and is left alone.
        return std::unexpected(Error{data.error().code, path_text(path) + ": " + data.error().context});
    }

    std::shared_ptr<const ScenarioData> shared = std::make_shared<const ScenarioData>(*std::move(data));

    LoadedScenario loaded;
    loaded.data = shared;
    loaded.scenario.name = shared->name;
    loaded.scenario.dt_ns = shared->dt_ns;
    loaded.scenario.substeps = shared->substeps;
    loaded.scenario.steps = shared->steps;

    loaded.scenario.build = [shared]() -> Result<WorldSetDesc> { return shared->worlds; };

    loaded.scenario.setup = [shared](Simulation& sim) -> Result<void> {
        // Models first, in declaration order: a ModelTypeId is an index into
        // THIS Simulation's registration order, and the registry is
        // configuration a restore does not rebuild -- so a scenario re-run
        // into a fresh Simulation must register the same models in the same
        // order to obtain the same ids. Doing it here means every path that
        // creates a Simulation for this scenario does so by construction.
        std::vector<ModelTypeId> ids;
        ids.reserve(shared->models.size());
        for (const vehicles::ModelType& model : shared->models) {
            const Result<ModelTypeId> id = sim.register_model(model);
            if (!id) return std::unexpected(id.error());
            ids.push_back(*id);
        }

        for (const ScenarioSpawn& spawn : shared->spawns) {
            if (spawn.kind == ScenarioSpawn::Kind::body) {
                const Result<BodyRef> ref = sim.spawn(spawn.world, spawn.body);
                if (!ref) return std::unexpected(ref.error());
                for (const DragElementSpawn& drag : spawn.drag_elements) {
                    const Result<DragElementRef> element = sim.add_drag_element(*ref, drag);
                    if (!element) return std::unexpected(element.error());
                }
            } else {
                const Result<VehicleRef> ref =
                    sim.spawn(spawn.world, ids[spawn.model_index], spawn.vehicle);
                if (!ref) return std::unexpected(ref.error());
            }
        }
        return {};
    };

    loaded.scenario.input = [shared](Simulation& sim, Tick tick) -> Result<void> {
        for (const ScenarioInput& input : shared->inputs) {
            if (input.tick != tick.value) continue;
            const ScenarioSpawn& target = shared->spawns[input.spawn_index];
            if (input.kind == ScenarioInput::Kind::wrench) {
                // STATE-FIRST: the ref is re-derived here, every tick, rather
                // than captured at setup -- see this header's note on why that
                // is what makes the script replayable across a restore.
                const Result<BodyRef> ref = sim.body_ref_at(target.world, target.local_body_slot);
                if (!ref) return std::unexpected(ref.error());
                if (Result<void> r = sim.apply_wrench(*ref, input.force, input.torque); !r) {
                    return r;
                }
            } else {
                const Result<VehicleRef> ref =
                    sim.vehicle_ref_at(target.world, target.vehicle_ordinal);
                if (!ref) return std::unexpected(ref.error());
                if (Result<void> r = sim.set_rotor_commands(*ref, input.omega); !r) return r;
            }
        }
        return {};
    };

    return loaded;
}

}  // namespace spade::testing
