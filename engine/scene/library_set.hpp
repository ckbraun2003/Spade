// ---------------------------------------------------------------------------
// scene/library_set.hpp -- N library roots opened together: the v3 index and
// entry layout, namespaced ids, aliases and the structured refusals
// (spade::scene). First slice of the library work (L88i); the shapes are the
// contract's, docs/design/core/plans/2026-11-17-spade-library-json-contract.md
// section 2 (the refusal object) and section 3 (the index and entry layout).
//
// A ROOT is a folder with library.yaml. Version 1 and 2 roots (scene/library.hpp)
// read as they always did, with DERIVED ids; a version 3 root names its own
// per-kind index files, aliases and dependencies. The host names the roots it
// loads, with absolute paths, in the order it wants them listed; nothing is read
// from the environment and nothing is searched for. A root that is absent, a
// dependency that is not opened, an id that names a root the entry does not
// depend on: each is refused by name, never skipped.
//
// REFUSALS are LibraryRefusal, the contract's refusal object: a stable `code`
// string, a message for people, and the root, path, id and field it is about.
// open_libraries() refuses as "open_failed" with one LibraryRefusal per
// problem in `issues`: it never opens part of a set.
//
// This header reads no entry's contents (a part's mass, a world's size: the
// peek, a later piece) and computes no hash (a later piece). It reads indexes.
// ---------------------------------------------------------------------------
#pragma once

#include <array>
#include <cstdint>
#include <expected>
#include <filesystem>
#include <map>
#include <optional>
#include <span>
#include <string>
#include <vector>

namespace spade::scene {

// The contract's refusal object (section 2). Empty strings stand for null.
struct LibraryRefusal {
    std::string code{};     // stable: "open_failed", "root_unavailable", "bad_index", ...
    std::string message{};  // for people; may change
    std::string root{};
    std::string path{};     // a file or folder path
    std::string id{};       // a full id, "<root>:<local id>"
    std::string field{};    // a dotted field path, "entries[2].file"
    std::vector<LibraryRefusal> issues{};
};

template <class T>
using LibraryResult = std::expected<T, LibraryRefusal>;

// A root the host asks for: its name ([a-z][a-z0-9_]*), the folder holding
// library.yaml (absolute), and whether the host lets writes land there.
struct LibraryRootSpec {
    std::string name{};
    std::filesystem::path path{};
    bool writable = false;
};

// A pose in the drone's design frame (+X forward, +Y left, +Z up; metres): a position and a unit quaternion [w, x, y, z].
struct SetPose {
    std::array<double, 3> pos{};
    std::array<double, 4> quat{1.0, 0.0, 0.0, 0.0};
};

// A per-slot adjustment, applied in the install point's own frame: a position offset and rotations in degrees, right-handed
// about the frame's axes (yaw about +Z, pitch about +Y, roll about +X), composed as Rz(yaw) * Ry(pitch) * Rx(roll).
struct SetAdjust {
    std::array<double, 3> pos{};
    double yaw_deg = 0.0;
    double pitch_deg = 0.0;
    double roll_deg = 0.0;
};

// One slot of a drone's parts_map (the contract's section 5): a part, and where it goes -- SLOTTED (a parent slot's
// install point, its pose derived) or FREE (a full pose), never both.
struct PartsMapSlot {
    std::string slot{};
    std::string part{};          // a full id
    std::string hash{};          // "sha256:<64 hex>", or empty for none
    std::string parent{};        // the slot key of the part that carries the install point; empty for a free slot
    std::string install_point{}; // its name on that part; empty for a free slot
    std::optional<SetPose> placement{};  // a free slot's full pose
    std::optional<SetAdjust> adjust{};   // a slotted slot's adjustment
    uint32_t count = 1;
};

// One entry of an opened set, as its index spells it, with its full id.
struct SetEntry {
    std::string id{};        // "<root>:<local_id>"
    std::string root{};
    std::string local_id{};  // "part.motor.fpv_2306", "drone.x500", "world.gate_corridor"
    // model, object, part, drone, world, mesh, or opaque (an entry with `type`).
    std::string kind{};
    std::string part_kind{};  // motor, prop, esc, battery, frame, payload for a part; else empty
    std::string type{};       // an opaque entry's declared type; else empty
    std::string name{};
    std::string category{};
    std::string rel_file{};   // as written; empty for a drone with a parts map only
    std::filesystem::path file{};       // the root's path joined with rel_file; empty if none
    std::string rel_thumbnail{};        // as written; empty for none
    std::filesystem::path thumbnail{};  // joined; empty for none
    std::vector<std::string> tags{};
    std::vector<std::string> requires_ids{};  // full ids
    std::vector<std::string> files{};         // companion files, relative, as written
    std::string generated_recipe{};           // generated_from.recipe; empty for none
    std::string generated_hash{};             // generated_from.hash; empty for none
    bool has_parts_map = false;               // a drone entry that carries a parts_map
    // The parts_map as canonical JSON (object keys sorted, no whitespace, numbers as the shortest decimal that
    // reads back), for the entry hash; empty without a parts_map. Its meaning is a later piece's (find).
    std::string parts_map_json{};
    std::vector<PartsMapSlot> parts_map{};  // the same parts_map, structured and validated
    // A file in the local store (L88b), named by `store: {sha256, file}` INSTEAD of `file`: it lives at
    // <store>/<sha256>/<file>. rel_file is empty then, and `file` is that resolved path when the host gave a store
    // to open_libraries(), else empty. Nothing is read from the store when the set is opened.
    bool in_store = false;
    std::string store_sha256{};  // 64 lower-case hex of the file's exact bytes
    std::string store_file{};    // a plain file name
};

// One opened root.
struct SetRoot {
    std::string name{};
    std::filesystem::path path{};
    uint32_t library_version = 0;       // 1, 2 or 3
    std::vector<std::string> depends{};
    std::size_t entry_count = 0;
    bool writable = false;
    std::map<std::string, std::string> aliases{};  // this root's own alias table: alias -> full id (a v3 root's `aliases`)
};

// Several roots opened together: roots and entries in the host's order, each
// root's entries in its index order; the merged alias table (old string -> full id).
struct LibrarySet {
    std::vector<SetRoot> roots{};
    std::vector<SetEntry> entries{};
    std::map<std::string, std::string> aliases{};
    std::filesystem::path store{};  // the host's local store, absolute; empty if none was given
};

// A YAML file as canonical JSON (object keys sorted, no whitespace, numbers as the shortest decimal that reads back):
// what a part's record is in find. io_error if the file cannot be read or parsed.
[[nodiscard]] LibraryResult<std::string> yaml_file_to_canonical_json(const std::filesystem::path& file);

// The newest root version this build reads.
inline constexpr uint32_t kLibrarySetVersion = 3;

// Opens `roots`, in order. Refused as "open_failed" (issues: one per problem,
// every root checked): relative_path, duplicate_root, root_unavailable (folder
// or index absent or unreadable), root_name_mismatch (a v3 index's name is not
// the one given), dependency_missing, dependency_cycle, bad_index (unknown,
// missing or duplicate key; a bad id, name or tag; a duplicate id; a kind that
// the id and the file extension disagree on; a missing file or thumbnail; a
// `requires` that is not a full id of an existing entry), foreign_root_reference
// (an id, in `requires` or `aliases`, that names a root the entry's root does not
// depend on; in a root with no dependencies, any other root), bare_id (a
// `requires` or alias target without "<root>:") and alias_conflict (an alias that
// is also an id, or that two roots both define).
//
// `store` is the host's local store folder (L88b), absolute, or empty for none: where an entry's `store:` file is looked
// up. A relative one is refused (relative_path). An absent store, or one without a file an entry names, does not stop
// the open: that is `store_file_missing`, reported when the entry is listed or found.
[[nodiscard]] LibraryResult<LibrarySet> open_libraries(std::span<const LibraryRootSpec> roots,
                                                       const std::filesystem::path& store = {});

}  // namespace spade::scene
