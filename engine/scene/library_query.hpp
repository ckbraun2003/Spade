// ---------------------------------------------------------------------------
// scene/library_query.hpp -- listing an opened library set: filters, the peek
// fields, and the computed entry hash (spade::scene). Slice S3a of the library
// work (L88i); the shapes are the contract's section 3 (the hash) and section 4
// (`list`), docs/design/core/plans/2026-11-17-spade-library-json-contract.md.
//
// list_entries() walks the set in the host's root order and each root's index
// order, keeps the entries a ListFilter accepts, and returns, for each, its
// computed hash and its peek: the few facts a panel shows without loading the
// entry (a part's mass, a model's rotor count, a world's version, the install
// kinds a part carries). The peek reads the entry's own file once and reads no
// other; an unreadable file gives no peek and a `peek_failed` warning, and the
// entry still lists.
//
// THE HASH is SHA-256 over text lines "<rel path>\t<sha256 hex of those bytes>\n"
// for the entry's file and each companion in `files`, sorted by relative path
// bytewise, plus, for a drone that has a parts_map, the line
// "parts_map\t<sha256 hex of its canonical JSON>\n"; written "sha256:<64 hex>".
// A text file's content has CR LF read as LF (extensions .yaml .yml .json .gltf .md .txt .csv, ASCII case-folded;
// a lone CR stays); every other file is hashed by its exact bytes, so a Windows checkout with autocrlf hashes
// the same as an LF one. It is not transitive and is never written in an index.
// ---------------------------------------------------------------------------
#pragma once

#include <cstddef>
#include <cstdint>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "scene/library_set.hpp"
#include "world/mesh.hpp"

namespace spade::scene {

// What the panel shows without loading an entry. Every field is empty or
// nullopt where it does not apply to the entry's kind.
struct EntryPeek {
    std::optional<double> mass_kg{};          // a part's own mass, or a model's body mass; none if undeclared
    std::vector<std::string> provides{};      // a part's `provides` ids (part_version 2 keys; empty before)
    std::map<std::string, double> limits{};   // a part's `limits` (same)
    std::optional<uint32_t> rotor_count{};    // a model's or drone's rotors
    std::optional<uint32_t> world_version{};  // a world's world_version
    std::vector<std::string> install_kinds{}; // sorted, de-duplicated kinds of a part's install points
};

struct ListFilter {
    std::vector<std::string> roots{};  // names; empty: every opened root. A name not opened is root_not_loaded.
    std::string kind{};                // empty, or model|object|part|drone|world|mesh|opaque
    std::string part_kind{};           // empty, or motor|prop|esc|battery|frame|payload (any lower-case name is accepted)
    std::string type{};                // empty, or an opaque entry's exact type string
    std::string text{};                // empty, or an ASCII case-folded substring of name, category or a tag
    std::vector<std::string> tags{};   // every tag listed must be on the entry
    std::size_t offset = 0;
    std::optional<std::size_t> limit{};
};

struct ListedEntry {
    const SetEntry* entry = nullptr;  // points into the LibrarySet passed to list_entries()
    std::string hash{};               // "sha256:<64 hex>"; empty if a file could not be read (a warning says so)
    std::optional<EntryPeek> peek{};  // none: unreadable (a warning says so)
};

struct ListPage {
    std::vector<ListedEntry> rows{};
    std::size_t total = 0;  // the entries the filter accepts, before offset and limit
    std::size_t offset = 0;
    // Per-entry problems that do not stop the list: "peek_failed" and "io_error", each with the entry's id and path.
    std::vector<LibraryRefusal> warnings{};
};

struct FindOptions {
    std::optional<std::string> expect_hash{};  // the caller's pin, "sha256:<64 hex>"
    bool with_record = false;                  // a part's file as canonical JSON
};

struct FindResult {
    const SetEntry* entry = nullptr;      // points into the LibrarySet passed to find_entry()
    std::string hash{};                   // the entry's current hash
    std::optional<bool> hash_matches{};   // the pin against `hash`; none when no pin was given
    std::string record_json{};            // a part file's own mapping as canonical JSON, when asked; else empty
    std::vector<LibraryRefusal> warnings{};  // "hash_mismatch" when a pin was given and differs
};

// One entry by full id. Refused: bare_id, root_not_loaded (the id's root is not open), entry_not_found, invalid_request
// (an expect_hash that is not "sha256:" and 64 lower-case hex), store_file_missing (a store entry whose file is absent;
// `path` is where it looked), io_error (a file that cannot be read for the hash). A hash that differs from the pin is a
// warning, not a refusal: the entry still returns.
[[nodiscard]] LibraryResult<FindResult> find_entry(const LibrarySet& set, std::string_view id, const FindOptions& options);

// The set's entries, filtered and paged. Refused: root_not_loaded (a filter root that is not open) and
// invalid_request (a kind that is not one of the seven).
[[nodiscard]] LibraryResult<ListPage> list_entries(const LibrarySet& set, const ListFilter& filter);

// The entry's computed hash. io_error (with the entry's id and the file's path) if a file cannot be read.
[[nodiscard]] LibraryResult<std::string> entry_hash(const LibrarySet& set, const SetEntry& entry);

// The entry's peek: nullopt when its file cannot be read or parsed (the reason in *why, when given).
[[nodiscard]] std::optional<EntryPeek> peek_entry(const SetEntry& entry, std::string* why = nullptr);

// resolve (L88l): an alias to its full id, read from the opened set's alias tables (one per root, in each root's library.yaml); the
// alias is any non-empty string. `root`, when not empty, limits the lookup to that root's own table. The result's `root` is the
// resolved id's own root, which need not be the table's. Refused: invalid_request (an empty alias; field alias), root_not_loaded
// (`root` is not open; field root), entry_not_found (no table has it; field alias). An id is not an alias.
struct ResolveResult {
    std::string id{};
    std::string root{};
};
[[nodiscard]] LibraryResult<ResolveResult> resolve_alias(const LibrarySet& set, std::string_view alias, std::string_view root = {});

// hash_text (L88l): "sha256:" and the 64 lower-case hex of the SHA-256 of `text`'s bytes exactly as given (no newline translation),
// for a host that pins a document it holds in memory.
[[nodiscard]] std::string hash_text(std::string_view text);

// One object of a part's solid, read from the store (L88l): the part record's `meshes` entry and its mesh, rebuilt by load_mesh_file() (a sidecar's
// bytes are never trusted), in the part's own frame, metres.
struct PartMeshObject {
    std::string sha256{};  // the store folder
    std::string file{};    // the store file
    bool visual = false;   // role: visual (drawn only)
    std::shared_ptr<const TriangleMesh> mesh{};  // null for a drawing-only object that is not an .spmesh (a presentation glTF): the renderer loads that file
};

// A part's objects, in the order its record lists them, each read from `<store>/<sha256>/<file>` (so Rendering draws compose_placement x part_meshes).
// An empty list is a part that has only shapes. Refused: bare_id, root_not_loaded, entry_not_found as find_entry; invalid_entry (the entry is not a part,
// or its record's blocks do not read: the message is the reader's), store_file_missing (field store: no store was given, or the store lacks that file; the
// object's file in path), invalid_entry again (path: the file is no readable .spmesh). Nothing else is read; with no store, `list` and `find` still work.
[[nodiscard]] LibraryResult<std::vector<PartMeshObject>> part_meshes(const LibrarySet& set, std::string_view id);

}  // namespace spade::scene
