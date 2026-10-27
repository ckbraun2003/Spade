// Save to library (EDT-020; INT-14 Q1a, the lead 2026-10-22): a scene's
// vehicle (its model, its propulsion chain with it) or asset (with the scene
// materials it names) saved into the PROJECT library, <the scene's
// folder>/library/, as a model or object file and its thumbnail, and listed
// in that folder's library.yaml, made on the first save.
//
// The default library (assets/library) is a pinned repo asset and is never
// written: a built-in scene has no project library. A name the index already
// has (ignoring case), or a file already where the item would go, takes the
// next of <name>_2, <name>_3, ...; nothing is ever written over. The index is
// replaced whole with Core's library_to_yaml() text (write_atomically()), after
// the item's files, so a kill leaves the old index or the new one. A refusal
// leaves the folder as it was.
#pragma once

#include <algorithm>
#include <cctype>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <system_error>
#include <vector>

#include "core/error.hpp"
#include "editor_files.hpp"
#include "editor_library.hpp"
#include "library_thumbnail.hpp"
#include "png_writer.hpp"
#include "scene/library.hpp"
#include "scene/model_file.hpp"
#include "scene/object_file.hpp"
#include "scene/scene_file.hpp"

namespace spade::sandbox::editor {

// What Save to library wrote.
struct LibrarySaved {
    std::string name;                 // the entry's: the vehicle's or asset's, or the next free <name>_<n>
    std::filesystem::path file;       // the model or object file
    std::filesystem::path thumbnail;  // its thumbnail; empty when it could not be drawn
    std::string no_thumbnail;         // why not, render_thumbnail()'s words; empty when drawn
};

namespace detail {

[[nodiscard]] inline Error save_refusal(std::string why) {
    return Error{Code::invalid_argument, "Save to library: " + std::move(why)};
}

// The built-ins' folder: assets/scenes/ beside assets/worlds/ (the File menu's
// rule, list_sources()).
[[nodiscard]] inline bool builtin_scene_folder(const std::filesystem::path& dir) {
    std::error_code ec;
    return !dir.empty() && dir.filename() == "scenes" && std::filesystem::is_directory(dir.parent_path() / "worlds", ec);
}

[[nodiscard]] inline bool same_ignoring_case(std::string_view a, std::string_view b) {
    return std::equal(a.begin(), a.end(), b.begin(), b.end(), [](char x, char y) {
        return std::tolower(static_cast<unsigned char>(x)) == std::tolower(static_cast<unsigned char>(y));
    });
}

// A file stem for `name`: lower case, and anything but a letter, a digit, '-'
// or '_' as '_'.
[[nodiscard]] inline std::string file_stem(std::string_view name) {
    std::string out;
    for (const char c : name) {
        const auto u = static_cast<unsigned char>(c);
        out += (std::isalnum(u) || c == '-' || c == '_') ? static_cast<char>(std::tolower(u)) : '_';
    }
    return out.empty() ? std::string("item") : out;
}

// The scene materials `asset` names, in the scene's order: what its object
// file carries (a world's own are named, not listed).
[[nodiscard]] inline std::vector<MaterialDesc> materials_named(const scene::SceneDesc& s, const scene::SceneAsset& a) {
    std::vector<MaterialDesc> out;
    for (const MaterialDesc& m : s.materials) {
        const bool named = m.name == a.visual.material ||
                           std::find(a.collider_materials.begin(), a.collider_materials.end(), m.name) !=
                               a.collider_materials.end();
        if (named) out.push_back(m);
    }
    return out;
}

}  // namespace detail

// Where Save to library writes for the scene at `scene_path`: its folder's
// library/. Refused by name for an unsaved scene and for a built-in one.
[[nodiscard]] inline Result<std::filesystem::path> project_library_dir(const std::filesystem::path& scene_path) {
    if (scene_path.empty()) {
        return std::unexpected(detail::save_refusal(
            "save the scene first: a saved item goes into its folder's library/"));
    }
    const std::filesystem::path dir = scene_path.parent_path();
    if (detail::builtin_scene_folder(dir)) {
        return std::unexpected(detail::save_refusal(
            "'" + scene_path.filename().string() +
            "' is a built-in scene, and its library is the default library, which is never written; save the "
            "scene to your own folder first (Save As)"));
    }
    return dir / "library";
}

// Saves the scene's vehicle or asset `name` into the library at `dir` (made
// when absent), with its thumbnail drawn on the CPU renderer from
// `mesh_roots`, and a drone's mesh parts' sidecars beside its file (its
// placed parts name them by their content). An item the renderer can't
// draw is saved without a thumbnail, and LibrarySaved says why. Refused by name, writing nothing: a name the scene
// has no vehicle or asset of; a model or object Core's writer refuses; an
// index that won't read; and a write that fails (what it wrote is removed).
[[nodiscard]] inline Result<LibrarySaved> save_to_library(const std::filesystem::path& dir,
                                                          const scene::SceneDesc& scene, std::string_view name,
                                                          std::span<const std::filesystem::path> mesh_roots) {
    namespace fs = std::filesystem;

    // The item: its file's text, its kind and its panel item (for the thumbnail).
    std::string text;
    LibraryItem item;
    scene::SceneDesc sidecars;  // a saved model alone, for its mesh parts' sidecars
    scene::LibraryKind kind = scene::LibraryKind::model;
    std::string category;
    std::string extension;
    const auto vehicle = std::find_if(scene.vehicles.begin(), scene.vehicles.end(),
                                      [&](const scene::SceneVehicle& v) { return v.name == name; });
    const auto asset = std::find_if(scene.assets.begin(), scene.assets.end(),
                                    [&](const scene::SceneAsset& a) { return a.name == name; });
    if (vehicle != scene.vehicles.end()) {
        const auto model = std::find_if(scene.models.begin(), scene.models.end(),
                                        [&](const vehicles::ModelType& m) { return m.name == vehicle->model; });
        if (model == scene.models.end()) {
            return std::unexpected(detail::save_refusal("vehicle '" + std::string(name) + "' flies model '" +
                                                        vehicle->model + "', which the scene doesn't have"));
        }
        // Only what Core would place back (TD-9), as the library places it
        // (add_model_with_meshes()): a model it refuses is not saved, and the
        // refusal is Core's. Placed into a copy of this scene under a free
        // name, so the scene it would become is judged whole. The file is
        // that placed copy under the model's own name: each mesh part names
        // its sidecar by its content ("imported/<h>.spmesh"), written beside
        // the model file below (the lead, 2026-10-27; L7c1's props).
        scene::SceneDesc trial = scene;
        std::string trial_name = model->name + "_placed";
        while (std::any_of(trial.models.begin(), trial.models.end(),
                           [&](const vehicles::ModelType& m) { return m.name == trial_name; })) {
            trial_name += "_";
        }
        if (Result<void> placed = scene::add_model_with_meshes(trial, scene::ModelFile{*model}, trial_name); !placed) {
            return std::unexpected(detail::save_refusal("'" + std::string(name) +
                                                        "' would not place back from the library, so it was not "
                                                        "saved: " + placed.error().context));
        }
        const auto copy = std::find_if(trial.models.begin(), trial.models.end(),
                                       [&](const vehicles::ModelType& m) { return m.name == trial_name; });
        scene::ModelFile file{copy != trial.models.end() ? *copy : *model};
        file.model.name = model->name;
        sidecars.models.push_back(file.model);
        Result<std::string> t = scene::model_to_yaml(file);
        if (!t) return std::unexpected(detail::save_refusal("'" + std::string(name) + "': " + t.error().context));
        text = std::move(*t);
        item = detail::model_item(file);
        kind = scene::LibraryKind::model;
        category = "drones";
        extension = ".model.yaml";
    } else if (asset != scene.assets.end()) {
        scene::ObjectFile file;
        file.object = *asset;
        file.materials = detail::materials_named(scene, *asset);
        Result<std::string> t = scene::object_to_yaml(file);
        if (!t) return std::unexpected(detail::save_refusal("'" + std::string(name) + "': " + t.error().context));
        text = std::move(*t);
        item = detail::object_item(file);
        kind = scene::LibraryKind::object;
        category = "objects";
        extension = ".object.yaml";
    } else {
        return std::unexpected(
            detail::save_refusal("the scene has no vehicle or asset named '" + std::string(name) + "'"));
    }

    // The index as it is, read without the disk (an entry whose file is gone
    // is the panel's to name, not a reason to refuse a save).
    const fs::path index = dir / "library.yaml";
    std::error_code ec;
    scene::Library library;
    if (fs::exists(index, ec)) {
        std::ifstream in(index, std::ios::binary);
        if (!in) return std::unexpected(detail::save_refusal("its index '" + index.generic_string() + "' can't be read"));
        const std::string old{std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>()};
        Result<scene::Library> read = scene::library_from_yaml(old);
        if (!read) {
            return std::unexpected(detail::save_refusal("its index '" + index.generic_string() +
                                                        "' won't read, so nothing was saved: " + read.error().context));
        }
        library = std::move(*read);
    }
    library.dir = dir;

    // The first free name: not in the index (ignoring case), no file there.
    const auto taken = [&](const std::string& n, const std::string& stem) {
        for (const scene::LibraryEntry& e : library.entries) {
            if (detail::same_ignoring_case(e.name, n) || detail::same_ignoring_case(e.file, stem + extension)) {
                return true;
            }
        }
        std::error_code e;
        return fs::exists(dir / (stem + extension), e) || fs::exists(dir / "thumbnails" / (stem + ".png"), e);
    };
    std::string chosen(name);
    std::string stem = detail::file_stem(chosen);
    for (int n = 2; taken(chosen, stem); ++n) {
        if (n > 9999) return std::unexpected(detail::save_refusal("no free name for '" + std::string(name) + "'"));
        chosen = std::string(name) + "_" + std::to_string(n);
        stem = detail::file_stem(chosen);
    }
    item.name = chosen;

    LibrarySaved saved;
    saved.name = chosen;
    saved.file = dir / (stem + extension);
    std::vector<uint8_t> png_bytes;
    if (Result<png::Image> drawn = render_thumbnail(item, mesh_roots); drawn) {
        png_bytes = png::encode_rgba(drawn->rgba.data(), drawn->width, drawn->height, false);
        saved.thumbnail = dir / "thumbnails" / (stem + ".png");
    } else {
        saved.no_thumbnail = drawn.error().context;
    }

    scene::LibraryEntry entry;
    entry.name = chosen;
    entry.category = category;
    entry.file = stem + extension;
    entry.thumbnail = saved.thumbnail.empty() ? std::string() : "thumbnails/" + stem + ".png";
    entry.kind = kind;
    library.entries.push_back(entry);
    Result<std::string> index_text = scene::library_to_yaml(library);
    if (!index_text) return std::unexpected(detail::save_refusal(index_text.error().context));

    // Write: a model's sidecars, the item, its thumbnail, then the index. A
    // failure removes what this save made (a sidecar already there is the
    // same content, its name being the content's, and stays).
    const bool had_dir = fs::exists(dir, ec);
    std::vector<fs::path> made;
    std::vector<fs::path> made_folders;
    const auto undo = [&](const Error& e) -> Result<LibrarySaved> {
        std::error_code ignored;
        for (const fs::path& p : made) fs::remove(p, ignored);
        for (const fs::path& f : made_folders) fs::remove(f, ignored);  // only when empty
        if (!saved.thumbnail.empty()) fs::remove(dir / "thumbnails", ignored);  // only when empty
        if (!had_dir) fs::remove(dir, ignored);                                 // only when empty
        return std::unexpected(detail::save_refusal(e.context));
    };
    fs::create_directories(saved.thumbnail.empty() ? dir : dir / "thumbnails", ec);
    if (ec) return undo(Error{Code::io_error, "can't make '" + dir.generic_string() + "': " + ec.message()});
    std::vector<fs::path> fresh;
    for (const vehicles::ModelType& m : sidecars.models) {
        for (const vehicles::PartDesc& part : m.parts) {
            if (!part.mesh || part.mesh_file.empty()) continue;
            const fs::path p = dir / part.mesh_file;
            if (!fs::exists(p, ec) && std::find(fresh.begin(), fresh.end(), p) == fresh.end()) {
                if (!fs::exists(p.parent_path(), ec)) made_folders.push_back(p.parent_path());
                fresh.push_back(p);
            }
        }
    }
    const Result<void> meshes = scene::write_placed_meshes(sidecars, dir);
    for (const fs::path& p : fresh) {
        if (fs::exists(p, ec)) made.push_back(p);
    }
    if (!meshes) return undo(meshes.error());
    if (Result<void> w = write_atomically(saved.file, text); !w) return undo(w.error());
    made.push_back(saved.file);
    if (!saved.thumbnail.empty()) {
        if (!png::write_file(saved.thumbnail, png_bytes)) {
            return undo(Error{Code::io_error, "can't write '" + saved.thumbnail.generic_string() + "'"});
        }
        made.push_back(saved.thumbnail);
    }
    if (Result<void> w = write_atomically(index, *index_text); !w) return undo(w.error());
    return saved;
}

}  // namespace spade::sandbox::editor
