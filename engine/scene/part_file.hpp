// ---------------------------------------------------------------------------
// scene/part_file.hpp -- the library's part files: a motor, a propeller, an
// ESC or a battery as a file of its own, with a source for every field
// (spade::scene; Task L2 of docs/design/core/plans/2026-10-21-default-drone-
// library-plan.md).
//
// A part file is one of the airframe builder's blocks (vehicles/
// airframe_compile.hpp: MotorBlock, PropBlock, EscBlock, BatteryBlock), key
// for key, plus `sources`: the citation of each field (scene/
// library_sources.hpp). A drone (L5) is built from part files and compiled
// with compile_airframe(), so the blocks' own rules (a positive kv, a table's
// increasing J, ...) are checked there and only there (TD-9). A part file
// checks what is its own: its keys, its numbers' text, its blade block and
// its sources.
//
// THE TEXT, top-level keys in this order:
//   part_version: 1
//   <kind>:    motor | prop | esc | battery, the file's kind (its suffix,
//              *.motor.yaml, *.prop.yaml, *.esc.yaml or *.battery.yaml, says
//              the same; load_part_file() refuses the two disagreeing). The
//              block's keys in declaration order, every one required:
//     motor:   kv, resistance, no_load_current, no_load_voltage, current_max,
//              pole_pairs, inductance, rotor_inertia, stator, mass, [shape]
//     prop:    diameter, pitch, blades, inertia, ct_static, cq_static,
//              ct_table, cq_table, mass, [shape], [blade]
//     esc:     current_continuous, current_burst, current_total, channels,
//              on_resistance, braking, protocol, mass, [shape]
//     battery: cells_series, cells_parallel, cell_capacity, cell_resistance,
//              cell_voltage_nominal, cell_voltage_full, cell_voltage_cutoff,
//              ocv_table, c_rating, polarization_resistance,
//              polarization_capacitance, mass, [shape]
//   sources:   a mapping from each cited field's path to its citation,
//              {source: <id>, grade: <grade>, note: "<text>"}, in the
//              block's key order.
//
// NUMBERS. Doubles in their shortest round-trip form (yaml_text::
// format_double), so every one reads back bit for bit; counts (blades,
// channels, cells_series, cells_parallel) as decimal integers; braking as true
// or false. A table (ct_table, cq_table, ocv_table) is a sequence of
// [x, value] pairs, one per line; `[]` for none.
//
// SHAPE (optional; leave the key out for none, which is a point mass, or a
// propeller's thin disc): {kind: sphere | box | cylinder | tube, size: [x, y,
// z]} with vehicles::BlockShape's meaning per kind (vehicles/
// composite_inertia.hpp), or {kind: tensor, tensor: [9 numbers, row-major]}.
// `point` is refused: it is what leaving the key out means. The member a kind
// does not use (`size` on a tensor, `tensor` on the others) has no key, so the
// writer refuses one that is set (any bit, -0 too) rather than drop it: every
// struct it accepts reads back the same.
//
// BLADE (a propeller only, optional): the blade definition for the actuator
// line (P-b), {r_over_r: [...], c_over_r: [...], beta_deg: [...], hub_cutout:
// <r/R>, airfoil: "<text>"}. The three lists have one value per station, at
// least 2; r/R is strictly increasing in (0, 1] and starts at or above the hub
// cut-out, which is at least 0; every chord is above 0; the airfoil names the
// section (its polar is the actuator line's, Physics'). The blade count is the
// propeller's own `blades`, written once.
//
// NO MOUNT. An ESC's and a battery's mount is where a drone places them, so it
// is the drone's (L5), not the part's: the text has no mount, the emitter
// writes none, and the reader leaves the identity.
//
// SOURCES. Every field cites, by its path ("kv", "ct_table", "blade.airfoil"),
// except the identity strings `stator` and `protocol`; `shape` cites as one,
// since its kind and size are one description. So the cited fields are the
// block's keys, less those two, plus `shape` when the part has one and the
// five blade keys when it has a blade. `sources` holds exactly those, no more
// and no fewer: a field without a citation is refused by its path, and so is
// a citation of a field the part does not have, or of an identity string
// ("the motor's field 'stator' takes no citation"). In a citation, `source` is
// left out for an assumed value that cites none, and `note` is left out when
// there is none; an empty one is refused (leave the key out). The registry's
// rules (a listed, usable source; a note on a derived or an assumed value)
// are check_source_ref()'s, run by check_part_sources().
//
// Unknown, missing and duplicate keys are refused at every level, by the
// dotted path, and from the readers with the line and column. An unknown key
// in the top level, the block, a shape, the blade or a citation is blamed on
// "part schema v1"; an unknown key in `sources` is a citation of something
// the part does not cite, refused as SOURCES says. A part_version this build
// does not read is schema_mismatch, read before any other key.
// ---------------------------------------------------------------------------
#pragma once

#include <cstdint>
#include <filesystem>
#include <optional>
#include <string>
#include <string_view>
#include <variant>
#include <vector>

#include "core/error.hpp"
#include "scene/library_sources.hpp"
#include "vehicles/airframe_compile.hpp"

namespace spade::scene {

// The part-file schema this build reads (the only one so far).
inline constexpr uint32_t kPartVersion = 1;

enum class PartKind : uint32_t {
    motor = 0,
    prop = 1,
    esc = 2,
    battery = 3,
};

// The kind's key in the text ("motor") and its file suffix (".motor.yaml").
[[nodiscard]] std::string_view part_kind_name(PartKind kind) noexcept;
[[nodiscard]] std::string_view part_suffix(PartKind kind) noexcept;

// One field's citation; a part's are in its block's key order.
struct FieldSource {
    std::string field{};  // the field's path: "kv", "ct_table", "blade.airfoil"
    SourceRef ref{};
};
using PartSources = std::vector<FieldSource>;

// A propeller's blade definition (BLADE above).
struct BladeGeometry {
    std::vector<double> r_over_r{};
    std::vector<double> c_over_r{};
    std::vector<double> beta_deg{};
    double hub_cutout = 0.0;  // r/R
    std::string airfoil{};
};

struct MotorPart {
    vehicles::MotorBlock block{};
    PartSources sources{};
};

struct PropPart {
    vehicles::PropBlock block{};
    std::optional<BladeGeometry> blade{};
    PartSources sources{};
};

struct EscPart {
    vehicles::EscBlock block{};  // block.mount is the drone's; a part file leaves it the identity
    PartSources sources{};
};

struct BatteryPart {
    vehicles::BatteryBlock block{};  // block.mount, as EscPart's
    PartSources sources{};
};

using PartFile = std::variant<MotorPart, PropPart, EscPart, BatteryPart>;

[[nodiscard]] PartKind part_kind(const PartFile& part) noexcept;

// The fields `part` must cite, in order (SOURCES above).
[[nodiscard]] std::vector<std::string> cited_fields(const PartFile& part);

// Part -> canonical text (LF, the key order above). invalid_argument for a
// part the text cannot carry: a non-finite number, a missing or an extra
// citation, an empty explicit source or note, a point shape, or a blade that
// breaks BLADE's rules; the same rules the readers apply, so what is written
// reads back.
[[nodiscard]] Result<std::string> motor_to_yaml(const MotorPart& part);
[[nodiscard]] Result<std::string> prop_to_yaml(const PropPart& part);
[[nodiscard]] Result<std::string> esc_to_yaml(const EscPart& part);
[[nodiscard]] Result<std::string> battery_to_yaml(const BatteryPart& part);
[[nodiscard]] Result<std::string> part_to_yaml(const PartFile& part);

// Text -> part, no disk and no registry. schema_mismatch for a part_version
// this build does not read; invalid_argument for every other rule of THE
// TEXT, with its dotted path and its line and column. The readers check the
// citations' shape, not the registry: check_part_sources() does.
[[nodiscard]] Result<MotorPart> motor_from_yaml(std::string_view text);
[[nodiscard]] Result<PropPart> prop_from_yaml(std::string_view text);
[[nodiscard]] Result<EscPart> esc_from_yaml(std::string_view text);
[[nodiscard]] Result<BatteryPart> battery_from_yaml(std::string_view text);
[[nodiscard]] Result<PartFile> part_from_yaml(std::string_view text, PartKind kind);

// Every citation against the registry (check_source_ref(), each named by its
// field's path, "<kind>.<field>"). The first refusal comes back.
[[nodiscard]] Result<void> check_part_sources(const PartFile& part, const SourceRegistry& registry);

// Reads `path` (its kind by its suffix; another suffix is invalid_argument),
// runs part_from_yaml(), then check_part_sources() against `registry`. io_error
// when it cannot be read; every refusal is prefixed with the path.
[[nodiscard]] Result<PartFile> load_part_file(const std::filesystem::path& path, const SourceRegistry& registry);

}  // namespace spade::scene
