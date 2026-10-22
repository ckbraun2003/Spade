// The library's thumbnails (INT-14, N14b; the lead's Q2a): one committed PNG
// per entry of the default library, kThumbnailSize square, drawn on the CPU
// renderer by render_thumbnail() (sandbox/library_thumbnail.hpp).
//
// Each entry's owner adds its `thumbnail: "thumbnails/<file stem>.png"` key
// and regenerates in the same commit (Core's L4/L5 entries too; the lead,
// 2026-10-22):
//     SPADE_LIBRARY_WRITE=1 ctest -R LibraryThumbnails
// writes each named PNG into the SOURCE tree's assets/library. Without the
// variable the test checks each one: it decodes (png_reader.hpp), it is the
// size, and it is not one colour. Never its bytes: CPU pixels may differ
// between MSVC and gcc.

#include <gtest/gtest.h>

#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <string>
#include <vector>

#include "../sandbox/editor_library.hpp"
#include "../sandbox/end_goal_checks.hpp"
#include "../sandbox/library_thumbnail.hpp"
#include "../sandbox/png_reader.hpp"
#include "../sandbox/png_writer.hpp"
#include "scene/library.hpp"
#include "scene/model_file.hpp"
#include "scene/object_file.hpp"

namespace {

namespace fs = std::filesystem;
namespace ed = spade::sandbox::editor;
namespace eg = spade::sandbox::end_goal;
namespace png = spade::sandbox::png;
namespace sc = spade::scene;

// The committed library, in the SOURCE tree: the generator writes here.
[[nodiscard]] fs::path library_dir() { return fs::path(SPADE_ASSETS_DIR) / "library"; }

// SPADE_LIBRARY_WRITE set: write the thumbnails instead of checking them, as
// test_library.cpp's generator does.
[[nodiscard]] bool library_write_asked() {
#if defined(_MSC_VER)
#pragma warning(push)
#pragma warning(disable : 4996)
#endif
    return std::getenv("SPADE_LIBRARY_WRITE") != nullptr;
#if defined(_MSC_VER)
#pragma warning(pop)
#endif
}

// The index alone, its dir set: load_library() would refuse a thumbnail the
// generator has yet to write.
[[nodiscard]] spade::Result<sc::Library> the_index() {
    std::ifstream in(library_dir() / "library.yaml", std::ios::binary);
    if (!in) return std::unexpected(spade::Error{spade::Code::io_error, "library.yaml can't be read"});
    const std::string text{std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>()};
    spade::Result<sc::Library> library = sc::library_from_yaml(text);
    if (library) library->dir = library_dir();
    return library;
}

// The entry, loaded as the panel loads it.
[[nodiscard]] spade::Result<ed::LibraryItem> item_of(const sc::Library& library, const sc::LibraryEntry& e) {
    ed::LibraryItem item;
    if (e.kind == sc::LibraryKind::model) {
        spade::Result<sc::ModelFile> f = sc::load_library_model(library, e);
        if (!f) return std::unexpected(f.error());
        item = ed::detail::model_item(*f);
    } else if (e.kind == sc::LibraryKind::object) {
        spade::Result<sc::ObjectFile> f = sc::load_library_object(library, e);
        if (!f) return std::unexpected(f.error());
        item = ed::detail::object_item(*f);
    } else {
        item.kind = ed::LibraryItem::Kind::part;
    }
    item.name = e.name;
    return item;
}

[[nodiscard]] eg::PixelsView view_of(const png::Image& image) {
    return eg::PixelsView{image.width, image.height, std::size_t{image.width} * 4u, image.rgba};
}

}  // namespace

TEST(LibraryThumbnails, EveryEntryOfTheDefaultLibraryHasItsOwnThumbnail) {
    const spade::Result<sc::Library> library = the_index();
    ASSERT_TRUE(library.has_value()) << library.error().context;
    ASSERT_FALSE(library->entries.empty());
    const std::vector<fs::path> mesh_roots{library_dir() / "meshes"};
    const bool write = library_write_asked();
    for (const sc::LibraryEntry& e : library->entries) {
        SCOPED_TRACE(e.name);
        const std::string stem = fs::path(e.file).filename().string();
        if (e.thumbnail.empty()) {
            ADD_FAILURE() << "no thumbnail: add thumbnail: \"thumbnails/"
                          << stem.substr(0, stem.find('.')) << ".png\" to its entry, then regenerate";
            continue;
        }
        const fs::path file = library->dir / e.thumbnail;
        if (write) {
            const spade::Result<ed::LibraryItem> item = item_of(*library, e);
            ASSERT_TRUE(item.has_value()) << item.error().context;
            const spade::Result<png::Image> drawn = ed::render_thumbnail(*item, mesh_roots);
            ASSERT_TRUE(drawn.has_value()) << drawn.error().context;
            std::error_code ec;
            fs::create_directories(file.parent_path(), ec);
            ASSERT_TRUE(png::write_file(file, png::encode_rgba(drawn->rgba.data(), drawn->width, drawn->height, false)))
                << file.generic_string();
        }
        const spade::Result<png::Image> image = png::read_file(file);
        ASSERT_TRUE(image.has_value()) << image.error().context;
        EXPECT_EQ(image->width, ed::kThumbnailSize);
        EXPECT_EQ(image->height, ed::kThumbnailSize);
        const eg::PixelsView view = view_of(*image);
        const double varied = eg::differ_fraction(view, eg::dominant_colour(view));
        EXPECT_TRUE(eg::differs(varied)) << file.generic_string() << " is one colour: a blank, not the item";
    }
}

// A thumbnail is the item alone, the same every time it is drawn, and a part
// (drawn in its drone) has none of its own.
TEST(LibraryThumbnails, AThumbnailIsTheSameEachTimeAndAPartHasNone) {
    const spade::Result<sc::Library> library = the_index();
    ASSERT_TRUE(library.has_value()) << library.error().context;
    const std::vector<fs::path> mesh_roots{library_dir() / "meshes"};
    const spade::Result<ed::LibraryItem> first = item_of(*library, library->entries.front());
    ASSERT_TRUE(first.has_value()) << first.error().context;
    const spade::Result<png::Image> a = ed::render_thumbnail(*first, mesh_roots);
    const spade::Result<png::Image> b = ed::render_thumbnail(*first, mesh_roots);
    ASSERT_TRUE(a.has_value()) << a.error().context;
    ASSERT_TRUE(b.has_value()) << b.error().context;
    EXPECT_EQ(a->rgba, b->rgba);

    ed::LibraryItem part;
    part.kind = ed::LibraryItem::Kind::part;
    part.name = "2312 motor";
    const spade::Result<png::Image> none = ed::render_thumbnail(part, mesh_roots);
    ASSERT_FALSE(none.has_value());
    EXPECT_NE(none.error().context.find("'2312 motor'"), std::string::npos) << none.error().context;
    EXPECT_NE(none.error().context.find("drawn in its drone"), std::string::npos) << none.error().context;
}
