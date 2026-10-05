// The editor's document: a description, its validator, undo and redo, and the
// save point (editor plan Task 1; EDT-001, EDT-003, EDT-004).
//
// A Document<int> with a validator that refuses negatives pins the history's
// rules; the two real factories pin that the scene validates with
// validate_scene() and the world with validate_world_desc().

#include <gtest/gtest.h>

#include <string>

#include "../sandbox/editor_document.hpp"
#include "editor_test_support.hpp"

namespace {

using spade::Code;
using spade::Error;
using spade::Result;
using spade::sandbox::editor::Document;
using spade::sandbox::editor::make_scene_document;
using spade::sandbox::editor::make_world_document;
namespace support = spade::sandbox::editor::test;

[[nodiscard]] Document<int> counter(int start, std::size_t bound = spade::sandbox::editor::kHistoryBound) {
    return Document<int>(
        start,
        [](const int& v) -> Result<void> {
            if (v < 0) return std::unexpected(Error{Code::invalid_argument, "negative"});
            return {};
        },
        bound);
}

}  // namespace

TEST(EditorDocument, ACommitIsUndoneAndRedoneExactly) {
    Document<int> d = counter(1);
    ASSERT_TRUE(d.commit(2).has_value());
    ASSERT_TRUE(d.commit(3).has_value());
    EXPECT_TRUE(d.undo());
    EXPECT_EQ(d.desc(), 2);
    EXPECT_TRUE(d.undo());
    EXPECT_EQ(d.desc(), 1);
    EXPECT_FALSE(d.undo());  // nothing left
    EXPECT_TRUE(d.redo());
    EXPECT_TRUE(d.redo());
    EXPECT_EQ(d.desc(), 3);
    EXPECT_FALSE(d.redo());
}

TEST(EditorDocument, ARefusedCommitChangesNothingAndSaysWhy) {
    Document<int> d = counter(1);
    const Result<void> r = d.commit(-5);
    ASSERT_FALSE(r.has_value());
    EXPECT_EQ(r.error().context, "negative");
    EXPECT_EQ(d.desc(), 1);
    EXPECT_FALSE(d.can_undo());
    EXPECT_FALSE(d.dirty());
}

TEST(EditorDocument, ANewCommitClearsRedo) {
    Document<int> d = counter(1);
    ASSERT_TRUE(d.commit(2).has_value());
    ASSERT_TRUE(d.undo());
    EXPECT_TRUE(d.can_redo());
    ASSERT_TRUE(d.commit(4).has_value());
    EXPECT_FALSE(d.can_redo());
    EXPECT_EQ(d.desc(), 4);
}

TEST(EditorDocument, TheHistoryKeepsTheNewestBoundEntries) {
    Document<int> d = counter(1, 3);
    for (int v = 2; v <= 6; ++v) ASSERT_TRUE(d.commit(v).has_value());
    EXPECT_TRUE(d.undo());
    EXPECT_TRUE(d.undo());
    EXPECT_TRUE(d.undo());
    EXPECT_EQ(d.desc(), 3);
    EXPECT_FALSE(d.undo());  // 1 and 2 fell off the bound
}

TEST(EditorDocument, AFreshDocumentIsClean) {
    const Document<int> d = counter(1);
    EXPECT_FALSE(d.dirty());
}

// The editor plan's review focus 4.
TEST(EditorDocument, UndoingASavedEditMakesItDirtyAndRedoingMakesItClean) {
    Document<int> d = counter(1);
    ASSERT_TRUE(d.commit(2).has_value());
    EXPECT_TRUE(d.dirty());
    d.mark_saved();
    EXPECT_FALSE(d.dirty());
    ASSERT_TRUE(d.undo());
    EXPECT_TRUE(d.dirty());
    ASSERT_TRUE(d.redo());
    EXPECT_FALSE(d.dirty());
}

// Saving, undoing past it and committing something new leaves the saved state
// unreachable: the document stays dirty until it is saved again.
TEST(EditorDocument, ABranchOffTheSavedStateIsDirtyUntilSaved) {
    Document<int> d = counter(1);
    ASSERT_TRUE(d.commit(2).has_value());
    d.mark_saved();
    ASSERT_TRUE(d.undo());
    ASSERT_TRUE(d.commit(2).has_value());  // the same value, a different edit
    EXPECT_TRUE(d.dirty());
    d.mark_saved();
    EXPECT_FALSE(d.dirty());
}

TEST(EditorDocument, TheSceneDocumentValidatesWithValidateScene) {
    auto d = make_scene_document(support::sample_scene(), support::hover_scene());
    EXPECT_EQ(d.path, support::hover_scene());
    spade::scene::SceneDesc bad = d.desc();
    bad.vehicles.push_back({"stray", "no_such_model", {}});
    const Result<void> r = d.commit(bad);
    ASSERT_FALSE(r.has_value());
    EXPECT_NE(r.error().context.find("no_such_model"), std::string::npos) << r.error().context;
    EXPECT_EQ(support::text_of(d.desc()), support::text_of(support::sample_scene()));
}

TEST(EditorDocument, TheWorldDocumentValidatesWithValidateWorldDesc) {
    auto d = make_world_document(support::sample_world(), support::hover_world());
    spade::WorldDesc bad = d.desc();
    bad.capacities.bodies = 0;
    const Result<void> r = d.commit(bad);
    ASSERT_FALSE(r.has_value());
    EXPECT_NE(r.error().context.find("bodies"), std::string::npos) << r.error().context;
    EXPECT_EQ(d.desc().capacities.bodies, support::sample_world().capacities.bodies);
}
