// The editor's files: open, save, save as. Display-free (SL15b); header-only,
// like the sandbox's other display-free pieces, so spade_tests runs this code
// itself.
//
// EVERY SAVE IS ATOMIC (EDT-006). The text goes to "<target>.tmp-spade" beside
// the target, and only a complete file is renamed over it. A save that cannot
// complete removes the temporary file, leaves the target as it was, and leaves
// the document dirty with its path unchanged; the error names the file, what
// failed, and what to check.
//
// A scene names its world by a path relative to itself and pins it by hash
// (SCN-001). Saving a scene therefore:
//   1. saves its world first when the world is dirty, so the scene never pins
//      a world that is not on disk;
//   2. rewrites the world path relative to where the scene is going, so Save
//      As elsewhere still finds it;
//   3. re-pins the world's hash when it changed, but only if the scene still
//      composes with that world; otherwise compose()'s reason is returned and
//      the scene is not written;
//   4. commits any change it made to the document, so it can be undone.
// An unedited scene saves back byte for byte (EDT-005).
//
// THE SCENE FOLLOWS ITS WORLD (EDT-016). Saving the world on its own
// (save_world_and_follow) re-pins the open scene by the same rule as step 3,
// as an undoable edit, and lists the other scene files in the project that
// name the world but do not pin its new hash. It never changes them: the user
// re-pins each with repin_scene_file(), which is refused if that scene would
// not compose. compose_conflict() shows the open pair's conflict live
// (EDT-017).
//
// docs/design/interface/plans/2026-10-05-editor-design.md §6-§7;
// 2026-10-05-editor-plan.md Tasks 5 and 6.

#pragma once

#include <algorithm>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <ios>
#include <string>
#include <string_view>
#include <system_error>
#include <utility>
#include <vector>

#include "editor_document.hpp"
#include "editor_scene_edits.hpp"
#include "scene/compose.hpp"
#include "scene/scene_file.hpp"
#include "world/world_file.hpp"

namespace spade::sandbox::editor {

inline constexpr std::string_view kTempSuffix = ".tmp-spade";

struct OpenedScene {
    SceneDocument scene;
    WorldDocument world;
};

// Writes `text` to `target` whole or not at all (see the file comment).
[[nodiscard]] inline Result<void> write_atomically(const std::filesystem::path& target, std::string_view text) {
    std::filesystem::path tmp = target;
    tmp += kTempSuffix;
    const auto fail = [&](const std::string& why) -> Result<void> {
        std::error_code ignored;
        std::filesystem::remove(tmp, ignored);
        return std::unexpected(Error{Code::io_error, "save " + target.string() + ": " + why +
                                                         "; nothing was written over it"});
    };
    {
        std::ofstream out(tmp, std::ios::binary | std::ios::trunc);
        if (!out) {
            return fail("cannot create " + tmp.filename().string() +
                        " beside it; check that the folder exists and can be written");
        }
        out.write(text.data(), static_cast<std::streamsize>(text.size()));
        out.flush();
        if (!out) return fail("writing " + tmp.filename().string() + " failed; check the free space");
    }
    std::error_code ec;
    std::filesystem::rename(tmp, target, ec);
    if (ec) {
        return fail("cannot replace it (" + ec.message() +
                    "); check that it is not read-only, not a folder, and not open in another program");
    }
    return {};
}

// The world's path relative to a scene file's folder, with forward slashes,
// as a scene file spells it (SCN-001).
[[nodiscard]] inline Result<std::string> relative_world_path(const std::filesystem::path& scene_dir,
                                                             const std::filesystem::path& world_file) {
    std::error_code ec;
    const std::filesystem::path from =
        std::filesystem::absolute(scene_dir.empty() ? std::filesystem::path(".") : scene_dir, ec).lexically_normal();
    const std::filesystem::path to = std::filesystem::absolute(world_file, ec).lexically_normal();
    const std::filesystem::path rel = ec ? std::filesystem::path() : to.lexically_relative(from);
    if (rel.empty()) {
        return std::unexpected(Error{Code::invalid_argument,
                                     "the world " + to.string() + " cannot be reached from " + from.string() +
                                         " by a relative path (another drive?); a scene names its world by a "
                                         "relative path (SCN-001), so keep the two on one drive"});
    }
    return rel.generic_string();
}

[[nodiscard]] inline Result<WorldDocument> open_world(const std::filesystem::path& world_file) {
    Result<WorldDesc> w = load_world_file(world_file);
    if (!w) return std::unexpected(w.error());
    return make_world_document(std::move(*w), world_file);
}

// Opens a scene file and the world it names.
[[nodiscard]] inline Result<OpenedScene> open_scene(const std::filesystem::path& scene_file) {
    Result<scene::SceneDesc> s = scene::load_scene_file(scene_file);
    if (!s) return std::unexpected(s.error());
    const std::filesystem::path world_file = (scene_file.parent_path() / s->world.file).lexically_normal();
    Result<WorldDesc> w = load_world_file(world_file);
    if (!w) {
        return std::unexpected(Error{w.error().code, "open " + scene_file.string() + ": its world '" +
                                                         s->world.file + "' could not be opened (" +
                                                         w.error().context +
                                                         "); restore the world file, or point the scene at "
                                                         "another world"});
    }
    return OpenedScene{make_scene_document(std::move(*s), scene_file),
                       make_world_document(std::move(*w), world_file)};
}

// Writes the world to `target`; on success that is the document's path and
// the document is clean.
[[nodiscard]] inline Result<void> save_world_as(WorldDocument& world, const std::filesystem::path& target) {
    const Result<std::string> text = world_to_yaml(world.desc());
    if (!text) return std::unexpected(text.error());
    if (Result<void> written = write_atomically(target, *text); !written) return written;
    world.path = target;
    world.mark_saved();
    return {};
}

[[nodiscard]] inline Result<void> save_world(WorldDocument& world) {
    if (world.path.empty()) {
        return std::unexpected(Error{Code::invalid_argument, "save the world '" + world.desc().name +
                                                                 "': it has no file yet; use Save As to choose one"});
    }
    return save_world_as(world, world.path);
}

// Writes the scene to `target`, after its world (see the file comment); on
// success that is the scene document's path and both documents are clean.
[[nodiscard]] inline Result<void> save_scene_as(SceneDocument& scene, WorldDocument& world,
                                                const std::filesystem::path& target) {
    if (world.path.empty()) {
        return std::unexpected(Error{Code::invalid_argument, "save " + target.string() + ": its world '" +
                                                                 world.desc().name +
                                                                 "' has no file yet; save the world with Save As "
                                                                 "first, so the scene can name it"});
    }
    if (world.dirty()) {
        if (Result<void> saved = save_world(world); !saved) return saved;
    }
    const Result<std::string> rel = relative_world_path(target.parent_path(), world.path);
    if (!rel) return std::unexpected(rel.error());
    const Result<uint64_t> hash = scene::world_hash(world.desc());
    if (!hash) return std::unexpected(hash.error());

    scene::SceneDesc next = scene.desc();
    const bool repin = next.world.hash != *hash;
    const bool moved = next.world.file != *rel;
    next.world.file = *rel;
    next.world.hash = *hash;
    if (repin) {
        if (const Result<scene::ComposedScene> composed = scene::compose(next, world.desc()); !composed) {
            return std::unexpected(Error{composed.error().code,
                                         "save " + target.string() + ": the scene does not compose with its world "
                                         "as saved (" + composed.error().context +
                                         "); the scene was not written. Resolve the conflict, then save again"});
        }
    }
    const Result<std::string> text = scene::scene_to_yaml(next);
    if (!text) return std::unexpected(text.error());
    if (Result<void> written = write_atomically(target, *text); !written) return written;
    if (repin || moved) {
        // validate_scene() already accepted `next` inside scene_to_yaml().
        if (Result<void> committed = scene.commit(std::move(next)); !committed) return committed;
    }
    scene.path = target;
    scene.mark_saved();
    return {};
}

[[nodiscard]] inline Result<void> save_scene(SceneDocument& scene, WorldDocument& world) {
    if (scene.path.empty()) {
        return std::unexpected(Error{Code::invalid_argument, "save the scene '" + scene.desc().name +
                                                                 "': it has no file yet; use Save As to choose one"});
    }
    return save_scene_as(scene, world, scene.path);
}

// What saving a world did to the scenes that name it.
struct WorldSaved {
    uint64_t old_hash = 0;      // the world's hash as it was on disk before the save; 0 if it had no file
    uint64_t new_hash = 0;      // its hash now
    bool repinned = false;      // the open scene pins new_hash (an undoable edit, saved with the scene)
    std::string why_not;        // compose()'s refusal when the open scene could not follow
    std::vector<std::filesystem::path> others;  // other scene files under the project folder that name this
                                                // world and do not pin new_hash; never changed here
};

// compose()'s refusal for the open scene and world as they would be saved
// together -- the scene re-pinned to the world's current hash -- or "" when
// they compose. Only the hash an unsaved world edit has not written yet is
// ignored; every other conflict is reported.
[[nodiscard]] inline std::string compose_conflict(const SceneDocument& scene, const WorldDocument& world) {
    const Result<uint64_t> hash = scene::world_hash(world.desc());
    if (!hash) return hash.error().context;
    scene::SceneDesc candidate = scene.desc();
    candidate.world.hash = *hash;
    const Result<scene::ComposedScene> composed = scene::compose(candidate, world.desc());
    return composed ? std::string() : composed.error().context;
}

namespace detail {

[[nodiscard]] inline bool is_scene_file(const std::filesystem::path& p) {
    const std::string name = p.filename().string();
    return name.size() > 11 && name.ends_with(".scene.yaml");
}

[[nodiscard]] inline bool same_file(const std::filesystem::path& a, const std::filesystem::path& b) {
    std::error_code ec;
    return std::filesystem::equivalent(a, b, ec) && !ec;
}

}  // namespace detail

// Saves the world, re-pins the open scene if it still composes, and lists the
// project's other scenes that name this world but no longer pin it.
[[nodiscard]] inline Result<WorldSaved> save_world_and_follow(WorldDocument& world, SceneDocument& scene,
                                                              const std::filesystem::path& project_dir) {
    WorldSaved out;
    if (const Result<WorldDesc> on_disk = load_world_file(world.path); on_disk) {
        if (const Result<uint64_t> h = scene::world_hash(*on_disk); h) out.old_hash = *h;
    }
    if (Result<void> saved = save_world(world); !saved) return std::unexpected(saved.error());
    const Result<uint64_t> hash = scene::world_hash(world.desc());
    if (!hash) return std::unexpected(hash.error());
    out.new_hash = *hash;

    if (scene.desc().world.hash == out.new_hash) {
        out.repinned = true;
    } else if (const std::string conflict = compose_conflict(scene, world); !conflict.empty()) {
        out.why_not = conflict;
    } else if (Result<void> repinned = apply(scene, RepointWorld{scene.desc().world.file, out.new_hash}); !repinned) {
        out.why_not = repinned.error().context;
    } else {
        out.repinned = true;
    }

    std::error_code ec;
    for (std::filesystem::recursive_directory_iterator it(
             project_dir, std::filesystem::directory_options::skip_permission_denied, ec), end;
         !ec && it != end; it.increment(ec)) {
        const std::filesystem::path& p = it->path();
        if (!detail::is_scene_file(p) || !it->is_regular_file()) continue;
        if (!scene.path.empty() && detail::same_file(p, scene.path)) continue;
        // A scene that does not load is not judged here; opening it says why.
        const Result<scene::SceneDesc> s = scene::load_scene_file(p);
        if (!s || s->world.hash == out.new_hash) continue;
        if (!detail::same_file((p.parent_path() / s->world.file).lexically_normal(), world.path)) continue;
        out.others.push_back(p);
    }
    std::sort(out.others.begin(), out.others.end());
    return out;
}

// Re-pins a scene file, on the user's word, to the world it names as now
// saved. Refused, with the file unchanged, if the scene would not compose
// with that world.
[[nodiscard]] inline Result<void> repin_scene_file(const std::filesystem::path& scene_file, const WorldDesc& world) {
    Result<scene::SceneDesc> s = scene::load_scene_file(scene_file);
    if (!s) return std::unexpected(s.error());
    const Result<uint64_t> hash = scene::world_hash(world);
    if (!hash) return std::unexpected(hash.error());
    s->world.hash = *hash;
    if (const Result<scene::ComposedScene> composed = scene::compose(*s, world); !composed) {
        return std::unexpected(Error{composed.error().code,
                                     "re-pin " + scene_file.string() + ": it would not compose with the saved world (" +
                                         composed.error().context +
                                         "); the file is unchanged. Open it and resolve the conflict, then save it"});
    }
    const Result<std::string> text = scene::scene_to_yaml(*s);
    if (!text) return std::unexpected(text.error());
    return write_atomically(scene_file, *text);
}

}  // namespace spade::sandbox::editor
