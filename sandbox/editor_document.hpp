// The editor's document: what the user edits, never the running simulation
// (EDT-001). Display-free (SL15b).
//
// A Document holds a description (a SceneDesc or a WorldDesc) and the
// validator its file format uses. Every change goes through commit(), which
// validates first and changes nothing when the result could not be saved
// (EDT-003). Undo and redo keep whole copies, one per committed edit: a scene
// or a world is small, and a copy cannot disagree with the edit it reverses
// (EDT-004). The history is bounded at kHistoryBound.
//
// THE SAVE POINT IS A SERIAL, NOT A FLAG. Every commit gets the next serial,
// undo and redo move between serials, and mark_saved() records the current
// one. dirty() compares them, so undoing past a save is dirty, redoing back to
// it is clean, and a new edit made after undoing past it stays dirty until it
// is saved, even if it happens to restore the same value.
//
// docs/design/interface/plans/2026-10-05-editor-design.md §3;
// 2026-10-05-editor-plan.md Task 1.

#pragma once

#include <cstddef>
#include <cstdint>
#include <deque>
#include <filesystem>
#include <functional>
#include <utility>

#include "core/error.hpp"
#include "scene/scene_file.hpp"
#include "world/builder.hpp"

namespace spade::sandbox::editor {

inline constexpr std::size_t kHistoryBound = 200;

template <class Desc>
class Document {
  public:
    using Validator = std::function<Result<void>(const Desc&)>;

    Document(Desc initial, Validator validate, std::size_t bound = kHistoryBound)
        : current_{std::move(initial), 0}, validate_(std::move(validate)), bound_(bound == 0 ? 1 : bound) {}

    [[nodiscard]] const Desc& desc() const noexcept { return current_.desc; }

    // Validates `next`; on success the current description goes onto the undo
    // history, the redo history is cleared, and `next` becomes current. On a
    // refusal nothing changes and the validator's error is returned.
    [[nodiscard]] Result<void> commit(Desc next) {
        if (validate_) {
            if (Result<void> ok = validate_(next); !ok) {
                return std::unexpected(ok.error());
            }
        }
        undo_.push_back(std::move(current_));
        if (undo_.size() > bound_) {
            undo_.pop_front();
        }
        redo_.clear();
        current_ = Entry{std::move(next), ++last_serial_};
        return {};
    }

    // False when there is nothing to undo.
    bool undo() {
        if (undo_.empty()) return false;
        redo_.push_back(std::move(current_));
        current_ = std::move(undo_.back());
        undo_.pop_back();
        return true;
    }

    // False when there is nothing to redo.
    bool redo() {
        if (redo_.empty()) return false;
        undo_.push_back(std::move(current_));
        current_ = std::move(redo_.back());
        redo_.pop_back();
        return true;
    }

    [[nodiscard]] bool can_undo() const noexcept { return !undo_.empty(); }
    [[nodiscard]] bool can_redo() const noexcept { return !redo_.empty(); }

    // The current state is what is on disk.
    void mark_saved() noexcept { saved_serial_ = current_.serial; }
    [[nodiscard]] bool dirty() const noexcept { return current_.serial != saved_serial_; }

    // Where the document was opened from or last saved to; empty for one never saved.
    std::filesystem::path path;

  private:
    struct Entry {
        Desc desc;
        uint64_t serial = 0;
    };

    Entry current_;
    std::deque<Entry> undo_;
    std::deque<Entry> redo_;
    Validator validate_;
    std::size_t bound_;
    uint64_t last_serial_ = 0;
    uint64_t saved_serial_ = 0;  // a fresh document is what it was opened from
};

using SceneDocument = Document<scene::SceneDesc>;
using WorldDocument = Document<WorldDesc>;

// A scene document, validated by the scene file's one validator (SCN-003).
[[nodiscard]] inline SceneDocument make_scene_document(scene::SceneDesc desc, std::filesystem::path path) {
    SceneDocument d(std::move(desc), [](const scene::SceneDesc& s) { return scene::validate_scene(s); });
    d.path = std::move(path);
    return d;
}

// A world document, validated by the world file's one validator.
[[nodiscard]] inline WorldDocument make_world_document(WorldDesc desc, std::filesystem::path path) {
    WorldDocument d(std::move(desc), [](const WorldDesc& w) -> Result<void> {
        if (Result<uint32_t> ok = validate_world_desc(w); !ok) return std::unexpected(ok.error());
        return {};
    });
    d.path = std::move(path);
    return d;
}

}  // namespace spade::sandbox::editor
