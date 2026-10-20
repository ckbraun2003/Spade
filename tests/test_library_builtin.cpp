// The default library's four primitives are the editor's built-in ones (N3):
// assets/library/primitives/{box,sphere,cylinder,capsule}.object.yaml collide
// exactly as sandbox builtin_library()'s items do -- the same SdfPrim kind and
// params, node for node, on the same identity transform, at the same pose --
// so a box placed from the library is the box the editor always placed.
//
// What differs, by design: the library's object names its one material,
// library_<name>, on every node, where the built-in item names none (the
// palette's default).
//
// Built only with the sandbox (tests/CMakeLists.txt's SPADE_BUILD_SANDBOX
// list): it reaches ../sandbox/editor_library.hpp by source, as the editor's
// own tests do.

#include <gtest/gtest.h>

#include <cstring>
#include <filesystem>
#include <string>
#include <string_view>
#include <vector>

#include "../sandbox/editor_library.hpp"
#include "core/error.hpp"
#include "scene/library.hpp"
#include "scene/object_file.hpp"
#include "world/builder.hpp"
#include "world/sdf.hpp"

namespace {

namespace ed = spade::sandbox::editor;
namespace fs = std::filesystem;

template <class T>
[[nodiscard]] testing::AssertionResult IsOk(const char* expr, const spade::Result<T>& r) {
    if (r) return testing::AssertionSuccess();
    return testing::AssertionFailure() << expr << " failed: [" << static_cast<int>(r.error().code) << "] "
                                       << r.error().context;
}
#define ASSERT_OK(expr) ASSERT_PRED_FORMAT1(IsOk, expr)

}  // namespace

TEST(LibraryBuiltin, TheLibrarysPrimitivesCollideExactlyAsTheEditorsBuiltInOnes) {
    const spade::Result<spade::scene::Library> library =
        spade::scene::load_library(fs::path(SPADE_ASSETS_DIR) / "library" / "library.yaml");
    ASSERT_OK(library);
    const std::vector<ed::LibraryItem> builtin = ed::builtin_library();

    // The library entry's name, and the built-in item's.
    struct Pair {
        std::string_view entry;
        std::string_view item;
    };
    for (const Pair& p : {Pair{"Box", "box"}, Pair{"Sphere", "sphere"}, Pair{"Cylinder", "cylinder"},
                          Pair{"Capsule", "capsule"}}) {
        SCOPED_TRACE(std::string(p.entry));
        const spade::scene::LibraryEntry* entry = nullptr;
        for (const spade::scene::LibraryEntry& e : library->entries) {
            if (e.name == p.entry) entry = &e;
        }
        ASSERT_NE(entry, nullptr) << "the index lists it";
        const ed::LibraryItem* item = nullptr;
        for (const ed::LibraryItem& i : builtin) {
            if (i.name == p.item) item = &i;
        }
        ASSERT_NE(item, nullptr) << "the editor ships it";
        ASSERT_TRUE(item->asset.has_value());

        const spade::Result<spade::scene::ObjectFile> object = spade::scene::load_library_object(*library, *entry);
        ASSERT_OK(object);
        const spade::SdfProgram& ours = object->object.collider;
        const spade::SdfProgram& theirs = item->asset->collider;

        // Node for node, bit for bit: kind, op, transform, padding and params.
        ASSERT_EQ(ours.nodes.size(), theirs.nodes.size());
        EXPECT_EQ(std::memcmp(ours.nodes.data(), theirs.nodes.data(), ours.nodes.size() * sizeof(spade::SdfNode)), 0);
        ASSERT_EQ(ours.transforms.size(), theirs.transforms.size());
        EXPECT_EQ(std::memcmp(ours.transforms.data(), theirs.transforms.data(),
                              ours.transforms.size() * sizeof(spade::SdfTransform)),
                  0);
        EXPECT_TRUE(ours.node_materials.empty());
        EXPECT_TRUE(theirs.node_materials.empty());
        EXPECT_EQ(std::memcmp(&object->object.pose, &item->asset->pose, sizeof(spade::SdfPose)), 0);

        // Both collider-only; the library's names its own material.
        EXPECT_TRUE(object->object.visual.mesh_ref.empty());
        EXPECT_TRUE(item->asset->visual.mesh_ref.empty());
        EXPECT_TRUE(item->asset->collider_materials.empty());
        EXPECT_EQ(object->object.collider_materials,
                  std::vector<std::string>(ours.nodes.size(), "library_" + std::string(p.item)));
    }
}
