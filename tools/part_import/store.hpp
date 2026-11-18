#pragma once

// ---------------------------------------------------------------------------
// part_import/store.hpp -- the local part store's names (design note, section 3). <store>/<sha256>/<original file name> is the source;
// the converted sidecars are under <store>/<sha256>/<key>/, where <key> names everything that changes a sidecar.
// ---------------------------------------------------------------------------

#include <chrono>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <vector>

#include "transform.hpp"
#include "world/mesh_file.hpp"

namespace spade::partimport {

// Everything that changes a sidecar: the converter's version (a number for the mesh readers, the OpenCASCADE and wheel versions for
// STEP), the tessellation tolerance (m) and angle (rad) (STEP; 0 for the others), the unit (empty when the file declares its own),
// the axes, the scale and the object selection (empty: every object).
struct KeyInputs {
    std::string converter;
    double tolerance_m = 0.0;
    double angle_rad = 0.0;
    std::string unit;
    Axes axes;
    double scale = 1.0;
    std::string object;
};

// The canonical text of the inputs: "spade-part-import-key-1;converter=...;tolerance=...;angle=...;unit=...;axes=...;scale=...;
// object=..." (one line), numbers as json_number writes them.
[[nodiscard]] std::string cache_key_text(const KeyInputs& in);
// The first 16 hex digits of that text's sha256.
[[nodiscard]] std::string cache_key(const KeyInputs& in);

// A file name from an object's name, as spade_mesh_pack makes one: every character a file name may not hold on Windows or Linux
// (control characters, < > : " / \ | ? *) becomes '_', trailing dots and spaces go, an empty name is "object", and a Windows device name
// (CON, PRN, AUX, NUL, COM1-9, LPT1-9, with or without an extension) gets a leading '_'.
[[nodiscard]] std::string safe_file_name(std::string_view name);
// The names made safe, then unique ignoring ASCII case: a later duplicate gets "-2", "-3", ... in order.
[[nodiscard]] std::vector<std::string> unique_file_names(const std::vector<std::string>& names);

// One object to store: its sidecar's mesh and provenance.
struct StoredObject {
    std::string name;
    std::shared_ptr<const TriangleMesh> mesh;
    MeshProvenance provenance;
    std::string cross_check_json;  // optional: a JSON object kept in import.json (see SidecarRecord)
};
struct SidecarRecord {
    std::string object;
    std::string file;          // the sidecar's name in the entry directory
    std::string content_hash;  // the mesh's 64-bit content hash, 16 hex digits
    std::uint32_t triangles = 0;
    double volume_m3 = 0.0;
    // The sidecar is also a store file of its own, <store>/<store_sha256>/<file>: the SHA-256 of ITS bytes, so a record can name it as
    // store: {sha256, file} (the library contract's "Store files"). Empty until the store writes it.
    std::string store_sha256;
    // The converter's cross-check of this object as one JSON object (STEP imports), or empty: kept so a cache hit reports it, and a
    // flagged object is flagged every time, not only on the run that made it.
    std::string cross_check_json;
    bool operator==(const SidecarRecord&) const = default;
};
// An entry: the record import.json holds.
struct EntryRecord {
    std::string sha256;       // of the source file
    std::string key;
    std::string key_text;     // cache_key_text(): what the key names, so a person can read it
    std::string converter;
    std::string source_name;  // the source file's name, as stored beside the entry
    std::vector<SidecarRecord> sidecars;
    std::filesystem::path dir;  // <store>/<sha256>/<key>; not part of import.json
    bool operator==(const EntryRecord&) const = default;
};

[[nodiscard]] std::filesystem::path entry_dir(const std::filesystem::path& store, std::string_view sha256, std::string_view key);

// Writes the source file and the entry. <store>/<sha256>/<safe source name> holds the source bytes (kept when already there); the entry
// is built in <store>/<sha256>/.tmp-<token>-<key>/ (one <object>.spmesh each, import.json LAST) and renamed to <key> in one step, so an
// entry exists whole or not at all. An incomplete directory at the key is replaced; a complete one is kept. Code on failure: store_io.
[[nodiscard]] Expected<EntryRecord> write_entry(const std::filesystem::path& store, std::string_view source_bytes,
                                                std::string_view source_name, const KeyInputs& inputs,
                                                std::span<const StoredObject> objects, std::string_view token);

// The complete entry for this source and key, or nothing: import.json must be there and read, name this key, and every sidecar it lists
// must load with the content hash it records.
[[nodiscard]] std::optional<EntryRecord> find_entry(const std::filesystem::path& store, std::string_view sha256, std::string_view key);

// Removes every <store>/<sha256>/.tmp-* directory last written more than `max_age` ago (a killed run's leftovers); returns how many.
std::size_t remove_stale_temps(const std::filesystem::path& store, std::chrono::seconds max_age);

}  // namespace spade::partimport
