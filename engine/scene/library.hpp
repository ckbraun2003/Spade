// ---------------------------------------------------------------------------
// scene/library.hpp -- Spade's default library: its index, library.yaml, and
// the model and object files it lists (spade::scene; N3 of the editor
// end-goal design, docs/design/interface/plans/2026-10-19-editor-end-goal-
// design.md §3).
//
// The library is a folder -- assets/library/ in the repository, installed
// with Spade (N21) so Kat's builder shows the same defaults -- and its index,
// library.yaml, is HAND-WRITTEN: one entry per item, in the order the editor
// shows them. It is not a folder walk, so the order is explicit and a test
// can load every entry. Core owns the format and this reader; Interface owns
// the categories and what the editor shows (the Library panel, N14).
//
// THE TEXT, top-level keys in this order:
//   library_version: 1 or 2
//   entries:   a sequence, one mapping per entry, in display order:
//     name:      non-empty; unique across the library ignoring (ASCII) case
//     category:  non-empty free text; the editor groups by it, in index order
//     file:      the entry's file, relative to library.yaml's directory: the
//                scene's relative-path rule (forward slashes, never absolute,
//                no ':'; scene/detail/relative_path.hpp, TD-9), no '..'
//                segment, and unique ignoring case. Its extension decides its
//                kind: *.model.yaml is a model file (scene/model_file.hpp),
//                *.object.yaml an object file (scene/object_file.hpp), and,
//                from version 2, *.motor.yaml, *.prop.yaml, *.esc.yaml and
//                *.battery.yaml a part file (scene/part_file.hpp). Anything
//                else is refused, and so is a part file in a version 1 index.
//     thumbnail: OPTIONAL, an image beside the entry, by file's path rules.
//                Leave the key out for none.
// Unknown, missing and duplicate keys are refused at every level, blamed on
// "library schema v<the index's version>", with the dotted path
// ("entries[2].file") and the line and column, as the scene's are. A
// library_version this build does not read is schema_mismatch, read before
// any other key. Version 2 adds only the part kinds, so a version 1 index
// reads as it always did.
//
// THE SOURCES. A library with part entries cites its sources from the
// registry beside its index, sources.yaml (scene/library_sources.hpp):
// load_library_sources() reads it, and load_library_part() checks a part's
// citations against it.
//
// PLACING COPIES the entry in (EDT-012): the editor loads the entry's file
// (load_library_model / load_library_object) and places it with add_model() or
// add_object(), so a scene never refers to the library.
//
// CASE. Windows' file systems ignore case and Linux's do not, so two paths
// that differ only in case are one file on one and two on the other. So an
// entry's file is unique ignoring case, and across the library's objects no
// two visual mesh_refs may differ only in case (check_library_contents(),
// Core's promise to Rendering: a ref names the same file everywhere). Names
// are unique ignoring case so the panel never shows two that read alike.
// ---------------------------------------------------------------------------
#pragma once

#include <cstdint>
#include <filesystem>
#include <string>
#include <string_view>
#include <vector>

#include "core/error.hpp"
#include "scene/library_sources.hpp"  // SourceRegistry
#include "scene/model_file.hpp"       // ModelFile
#include "scene/object_file.hpp"      // ObjectFile
#include "scene/part_file.hpp"        // PartFile

namespace spade::scene {

// The newest library-index schema this build reads; it reads every version
// from 1 to this one.
inline constexpr uint32_t kLibraryVersion = 2;

// What an entry's file holds, by its extension.
enum class LibraryKind : uint32_t {
    model = 0,    // *.model.yaml: a drone, placed with add_model()
    object = 1,   // *.object.yaml: anything else, placed with add_object()
    motor = 2,    // *.motor.yaml: a part (version 2), built into a drone
    prop = 3,     // *.prop.yaml
    esc = 4,      // *.esc.yaml
    battery = 5,  // *.battery.yaml
};

// One entry, as the index spells it.
struct LibraryEntry {
    std::string name{};
    std::string category{};
    // Relative to Library::dir, forward slashes, as written.
    std::string file{};
    // Relative to Library::dir, as written; empty when the entry has none.
    std::string thumbnail{};
    LibraryKind kind = LibraryKind::model;
};

// A read index: its entries in index order, and the folder they are relative
// to.
struct Library {
    // The index's directory (load_library); empty from library_from_yaml(),
    // which reads no disk, and then the entries resolve against the working
    // directory.
    std::filesystem::path dir{};
    std::vector<LibraryEntry> entries{};
};

// Text -> Library: the index alone, no disk. Codes: schema_mismatch for a
// library_version this build does not read; invalid_argument for everything
// else -- every rule of THE TEXT above, each with its dotted path and its
// line and column. `dir` is empty.
[[nodiscard]] Result<Library> library_from_yaml(std::string_view text);

// Reads the index at `path` and runs library_from_yaml(), then checks the
// disk: each entry's file exists, and a thumbnail the entry NAMES exists (the
// panel never shows a blank tile for a typo). dir is the index's directory
// ("." for a bare file name). io_error when the index cannot be read, and for
// a missing file or thumbnail, naming the entry and the path; a parse
// diagnostic comes back prefixed with the index's path. It reads no entry's
// contents: check_library_contents() does.
[[nodiscard]] Result<Library> load_library(const std::filesystem::path& path);

// The entry's model file, read with load_model_file() from library.dir /
// entry.file. Refused by name (invalid_argument) for an object entry; the
// loader's own refusal keeps its code, after "library entry '<name>': ".
[[nodiscard]] Result<ModelFile> load_library_model(const Library& library, const LibraryEntry& entry);

// The entry's object file, read with load_object_file() from library.dir /
// entry.file. Refused by name (invalid_argument) for a model entry; the
// loader's own refusal keeps its code, after "library entry '<name>': ".
[[nodiscard]] Result<ObjectFile> load_library_object(const Library& library, const LibraryEntry& entry);

// The library's sources registry, library.dir / "sources.yaml", read with
// load_sources().
[[nodiscard]] Result<SourceRegistry> load_library_sources(const Library& library);

// The entry's part file, read with load_part_file() from library.dir /
// entry.file and checked against `registry`. Refused by name
// (invalid_argument) for a model or an object entry; the loader's own refusal
// keeps its code, after "library entry '<name>': ".
[[nodiscard]] Result<PartFile> load_library_part(const Library& library, const LibraryEntry& entry,
                                                 const SourceRegistry& registry);

// Loads EVERY entry (the loaders above), in index order, and checks the one
// rule across entries: no two visual mesh_refs of the library's objects may
// differ only in case ("props/Ball" and "props/ball"), because they are one
// file on Windows. The same ref twice is one mesh, and fine. A library with
// part entries reads its sources registry once, first. The first refusal
// comes back with its code: a loader's, or invalid_argument naming both
// entries and both refs. The index test calls it, and so will the install
// step (N21).
[[nodiscard]] Result<void> check_library_contents(const Library& library);

}  // namespace spade::scene
