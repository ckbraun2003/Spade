// ---------------------------------------------------------------------------
// scene/model_sources.hpp -- a library drone's sources sidecar: where every
// input of one model file came from (spade::scene; Task L5 of docs/design/
// core/plans/2026-10-21-default-drone-library-plan.md).
//
// A library drone is compiled from part files (scene/part_file.hpp) and a few
// inputs of its own: a layout, a body, a mass target. Its model file (scene/
// model_file.hpp) carries the compiled numbers, not where they came from, so
// each drone of the default library has a sidecar beside it,
// drones/<name>.sources.yaml next to drones/<name>.model.yaml. The sidecar
// pins the model and the parts it was built from by their hashes, and lists
// every input with its value and its citation (scene/library_sources.hpp's
// SourceRef and grades, TD-9), what the model does not carry, the checks its
// tests make, and its assumptions. The default library's tests write each
// sidecar with its model (SPADE_LIBRARY_WRITE) and check it with
// check_model_sources().
//
// WHO REQUIRES ONE (the lead's ruling, 2026-10-22). The committed default
// library's drones: tests/test_library_drones.cpp refuses each library drone
// it builds whose sidecar is missing or stale. It checks those drones by name,
// not by walking the index, and the reference quad has no sidecar.
// NOT load_library() or check_library_contents(): a project's own library,
// written by Save to library, has model entries without sidecars, and they
// keep loading.
//
// THE TEXT, after a generated-file comment, top-level keys in this order:
//   model_sources_version: 1
//   model:        {file: "<path>", hash: "<hash>"}: the model file
//   built_by:     non-empty: what built the spec ("vehicles::
//                 basic_quad_airframe()")
//   parts:        a mapping from a part kind to {file: "<path>", hash:
//                 "<hash>"}: the part files the drone was built from, in the
//                 order prop, motor, esc, battery, each kind at most once;
//                 `{}` for none
//   inputs:       a mapping from each input's field path to its value and
//                 citation (INPUTS), in the generator's order; `{}` for none
//   not_carried:  a sequence of non-empty strings, none twice: what the
//                 source has and the model leaves out; `[]` for none
//   checks:       a sequence, one mapping per check (CHECKS); `[]` for none
//   assumptions:  a sequence of non-empty strings, none twice; `[]` for none
//   caveat:       OPTIONAL, non-empty: what holds for the whole drone ("fitted
//                 to a simulator, not hardware"); leave the key out for none
//
// PATHS AND HASHES. A path is library-relative, spelled as the library index
// spells the entry (scene/library.hpp). A hash is pinned_file_hash() of the
// file's bytes: FNV-1a 64 with CRLF read as LF, so a checkout's line endings
// do not change it (tools/library_props' hash_file is the same function). It
// is written as 16 lowercase hex digits, quoted, with no 0x.
//
// INPUTS. Each key is the input's field path in the spec or the params the
// drone is built from ("mass", "body_size", "rotors[0].hub.position"): dotted
// segments, each a letter or '_' and then letters, digits and '_' (lower-case
// ASCII), each optionally indexed ("[0]"). The writer quotes the key. Its
// value is a mapping, keys in this order:
//   value:   a number, or a vector input's numbers in the field's own order
//            ([x, y, z] for a size or a position, [w, x, y, z] for an
//            orientation) as a flow sequence of at least one number. One
//            number in a sequence stays a vector: InputValue keeps the two
//            apart. A table (ct_table) is not an input's value; it is a part's
//            field, cited in its part file.
//   source:  the registry id; left out for an assumed value that cites none
//   grade:   one of the six (library_sources.hpp's GRADES)
//   note:    left out for none
//   kat:     OPTIONAL: Kat's own provenance of a value taken from Kat's files,
//            in Kat's words, {source: "<word>", confidence: "<word>", date:
//            "YYYY-MM-DD"}, the date left out when Kat's is undated
//
// CHECKS. Each a mapping, keys in this order:
//   test:         non-empty: the test that makes the check
//   reference:    the cited number the test compares against
//   unit:         non-empty: the reference's unit ("m/s^2", "N"); "1" for a
//                 pure number
//   source, grade, note: the reference's citation, as an input's; the note is
//                 the check's own
//   band:         [lo, hi], lo <= hi: the band the test holds the relative
//                 error, (value - reference) / reference, to ([-0.02, 0.02]
//                 is +-2%). Required unless report_only; a report-only check
//                 may record one.
//   report_only:  true when the test reports the number and holds it to no
//                 band; false otherwise
//
// NUMBERS. Doubles in their shortest round-trip form (yaml_text::
// format_double), so every value reads back bit for bit; every one finite.
// Every string is double-quoted; a grade and report_only are bare words.
//
// CITATIONS. An input's and a check's citation keep check_source_ref()'s
// rules. The reader and the writer apply those that need no registry: every
// grade but assumed cites a source, and a derived or an assumed value has a
// note. check_model_sources() applies the registry's: a listed, usable source.
// A source or a note that is there must not be empty (leave the key out).
//
// Unknown, missing and duplicate keys are refused at every level, blamed on
// "model sources schema v1", by the dotted path ("inputs.mass.grade",
// "checks[1].band"), and from the reader with the line and column. A
// model_sources_version this build does not read is schema_mismatch, read
// before any other key.
//
// WHAT check_model_sources() CHECKS, against a library and its registry:
//   - model.file is a model entry of the index, and its bytes hash to
//     model.hash, so a model cannot be regenerated without its sidecar;
//   - each parts.<kind> is a <kind> part entry of the index, its bytes hash
//     to its hash, and it loads (load_library_part(), its own citations);
//   - every input's and every check's citation passes check_source_ref().
// The first refusal comes back, naming the sidecar's field. It does not
// compare the values with the spec's: the drone's own test does, since only
// it builds the spec.
// ---------------------------------------------------------------------------
#pragma once

#include <array>
#include <cstdint>
#include <filesystem>
#include <optional>
#include <string>
#include <string_view>
#include <variant>
#include <vector>

#include "core/error.hpp"
#include "scene/library.hpp"          // Library
#include "scene/library_sources.hpp"  // SourceRef, SourceRegistry
#include "scene/part_file.hpp"        // PartKind

namespace spade::scene {

// The model-sources schema this build reads (the only one so far).
inline constexpr uint32_t kModelSourcesVersion = 1;

// A file the sidecar pins: its path, as the index spells it, and
// pinned_file_hash() of its bytes.
struct PinnedFile {
    std::string file{};
    uint64_t hash = 0;
};

// A part file the drone was built from, pinned as a PinnedFile is.
struct PinnedPart {
    PartKind kind = PartKind::prop;
    std::string file{};
    uint64_t hash = 0;
};

// Kat's provenance of a value from Kat's files, in Kat's words (Kat's JSON).
struct KatProvenance {
    std::string source{};      // "tuned", "derived", "assumed"
    std::string confidence{};  // "low", "medium", "high"
    std::string date{};        // YYYY-MM-DD; empty when Kat's is undated
};

// An input's value: a number, or a vector input's numbers in the field's own
// order (INPUTS above).
using InputValue = std::variant<double, std::vector<double>>;

// One input of the drone, in the order the generator gives.
struct ModelInput {
    std::string path{};  // the field path: "mass", "rotors[0].hub.position"
    InputValue value{};
    SourceRef ref{};
    std::optional<KatProvenance> kat{};
};

// One check a test makes against a cited number.
struct ModelCheck {
    std::string test{};
    double reference = 0.0;
    std::string unit{};                           // "1" for a pure number
    SourceRef ref{};                              // the reference's citation; ref.note is the check's note
    std::optional<std::array<double, 2>> band{};  // [lo, hi] on the relative error; none when report-only
    bool report_only = false;
};

// A sidecar, key for key (THE TEXT above).
struct ModelSources {
    PinnedFile model{};
    std::string built_by{};
    std::vector<PinnedPart> parts{};  // in the order prop, motor, esc, battery
    std::vector<ModelInput> inputs{};
    std::vector<std::string> not_carried{};
    std::vector<ModelCheck> checks{};
    std::vector<std::string> assumptions{};
    std::string caveat{};  // empty when there is none
};

// FNV-1a 64 of `bytes` with every CRLF read as LF (PATHS AND HASHES): what a
// sidecar pins a file by. The generator hashes the text it writes with it.
[[nodiscard]] uint64_t pinned_file_hash(std::string_view bytes) noexcept;

// ModelSources -> canonical text (LF; the generated-file comment, then the key
// order above). invalid_argument, naming the field, for a sidecar the text
// cannot carry or the reader would refuse: an empty model.file or built_by,
// parts out of order or a kind twice, an input path that is not a field path
// or is listed twice, a non-finite number, an empty vector value, a citation
// that breaks CITATIONS, a Kat date that is not YYYY-MM-DD, an empty or a
// repeated string in not_carried or assumptions, a check with an empty test
// or unit, a band with lo above hi, or a banded check with no band. What it
// writes reads back equal.
[[nodiscard]] Result<std::string> model_sources_to_yaml(const ModelSources& sources);

// Text -> ModelSources, no disk and no registry. schema_mismatch for a
// model_sources_version this build does not read; invalid_argument for every
// other rule of THE TEXT, with its dotted path and its line and column.
[[nodiscard]] Result<ModelSources> model_sources_from_yaml(std::string_view text);

// Reads the sidecar at `path` and runs model_sources_from_yaml(). io_error
// when it cannot be read; a parse diagnostic comes back prefixed with the
// path.
[[nodiscard]] Result<ModelSources> load_model_sources(const std::filesystem::path& path);

// The sidecar against `library` and its `registry` (WHAT check_model_sources()
// CHECKS above), after the writer's rules. The first refusal comes back,
// naming the sidecar's field: invalid_argument for a file the index does not
// list as that kind, a stale hash, or a citation the registry refuses;
// io_error for a listed file that cannot be read; a part's own refusal keeps
// its code, after "parts.<kind>: ".
[[nodiscard]] Result<void> check_model_sources(const Library& library, const SourceRegistry& registry,
                                               const ModelSources& sources);

}  // namespace spade::scene
