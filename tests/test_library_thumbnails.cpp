// The library's thumbnails (INT-14, N14b; the lead's Q2a): one committed PNG
// per entry of the default library, kThumbnailSize square, drawn on the CPU
// renderer by render_thumbnail() (sandbox/library_thumbnail.hpp). A part is
// drawn from its own geometry, so Core's L4 part entries carry one too.
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
#include <string_view>
#include <utility>
#include <vector>

#include <glm/glm.hpp>

#include "../sandbox/editor_library.hpp"
#include "../sandbox/end_goal_checks.hpp"
#include "../sandbox/library_thumbnail.hpp"
#include "../sandbox/png_reader.hpp"
#include "../sandbox/png_writer.hpp"
#include "scene/library.hpp"
#include "scene/library_sources.hpp"
#include "scene/model_file.hpp"
#include "scene/object_file.hpp"
#include "scene/part_file.hpp"
#include "vehicles/airframe_compile.hpp"

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
        const spade::Result<sc::SourceRegistry> registry = sc::load_library_sources(library);
        if (!registry) return std::unexpected(registry.error());
        spade::Result<sc::PartFile> f = sc::load_library_part(library, e, *registry);
        if (!f) return std::unexpected(f.error());
        item.kind = ed::LibraryItem::Kind::part;
        item.part = std::move(*f);
    }
    item.name = e.name;
    return item;
}

// The thumbnail an entry without one should name: its file's name less the
// trailing ".<kind>.yaml", so a name with a dot of its own keeps it.
[[nodiscard]] std::string suggested_thumbnail(const std::string& file) {
    const std::string stem = fs::path(file).filename().string();
    return "thumbnails/" + stem.substr(0, stem.find('.')) + ".png";
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
        if (e.thumbnail.empty()) {
            ADD_FAILURE() << "no thumbnail: add thumbnail: \"" << suggested_thumbnail(e.file)
                          << "\" to its entry, then regenerate";
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

// A thumbnail is the same every time it is drawn.
TEST(LibraryThumbnails, AThumbnailIsTheSameEachTimeItIsDrawn) {
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
}

namespace {

[[nodiscard]] ed::LibraryItem part_item(std::string name, sc::PartFile part) {
    ed::LibraryItem item;
    item.kind = ed::LibraryItem::Kind::part;
    item.name = std::move(name);
    item.part = std::move(part);
    return item;
}

[[nodiscard]] spade::vehicles::BlockShape shape(spade::vehicles::PartShape kind, glm::dvec3 size) {
    spade::vehicles::BlockShape s;
    s.shape = kind;
    s.size = size;
    return s;
}

}  // namespace

// A part (Core's L4: motors, props, ESCs, batteries) is drawn from its own
// geometry: its file's shape, as vehicles::BlockShape sizes it, or a prop's
// disc, its diameter, when it has none (Core, 2026-10-22). A part with no
// geometry (no shape, or a tensor) is refused by name, never skipped.
TEST(LibraryThumbnails, APartIsDrawnFromItsOwnGeometryAndOneWithoutAnyIsRefused) {
    namespace vh = spade::vehicles;
    const std::vector<fs::path> mesh_roots{library_dir() / "meshes"};
    sc::MotorPart motor;
    motor.block.shape = shape(vh::PartShape::cylinder, glm::dvec3(0.014, 0.025, 0.0));
    sc::PropPart prop;
    prop.block.diameter = 0.24;
    prop.block.blades = 2;
    sc::BatteryPart battery;
    battery.block.shape = shape(vh::PartShape::box, glm::dvec3(0.13, 0.03, 0.045));
    for (const ed::LibraryItem& item :
         {part_item("2312 motor", motor), part_item("9450 prop", prop), part_item("4S battery", battery)}) {
        SCOPED_TRACE(item.name);
        const spade::Result<png::Image> drawn = ed::render_thumbnail(item, mesh_roots);
        ASSERT_TRUE(drawn.has_value()) << drawn.error().context;
        EXPECT_EQ(drawn->width, ed::kThumbnailSize);
        const eg::PixelsView view = view_of(*drawn);
        EXPECT_TRUE(eg::differs(eg::differ_fraction(view, eg::dominant_colour(view)))) << "a blank";
    }

    sc::EscPart shapeless;
    sc::MotorPart tensor;
    tensor.block.shape = shape(vh::PartShape::tensor, glm::dvec3(0.0));
    for (const ed::LibraryItem& item : {part_item("bare ESC", shapeless), part_item("tensor motor", tensor)}) {
        SCOPED_TRACE(item.name);
        const spade::Result<png::Image> none = ed::render_thumbnail(item, mesh_roots);
        ASSERT_FALSE(none.has_value());
        EXPECT_NE(none.error().context.find("'" + item.name + "'"), std::string::npos) << none.error().context;
        EXPECT_NE(none.error().context.find("no geometry to draw"), std::string::npos) << none.error().context;
    }
}

// Core's L4 part entries are drawn from their own files. The default library
// has a part entry of each kind; one of each, loaded as the generator loads it
// (item_of()), draws through render_thumbnail() to an image that is not a
// blank, and its committed PNG is there, the size, and not one colour.
TEST(LibraryThumbnails, ADefaultLibraryPartIsDrawnFromItsFile) {
    const spade::Result<sc::Library> library = the_index();
    ASSERT_TRUE(library.has_value()) << library.error().context;
    const std::vector<fs::path> mesh_roots{library_dir() / "meshes"};
    struct Kind {
        sc::LibraryKind kind;
        const char* name;
    };
    for (const Kind k : {Kind{sc::LibraryKind::prop, "prop"}, Kind{sc::LibraryKind::motor, "motor"},
                         Kind{sc::LibraryKind::esc, "ESC"}, Kind{sc::LibraryKind::battery, "battery"}}) {
        SCOPED_TRACE(k.name);
        const sc::LibraryEntry* entry = nullptr;
        for (const sc::LibraryEntry& e : library->entries) {
            if (e.kind == k.kind) {
                entry = &e;
                break;
            }
        }
        if (entry == nullptr) {
            ADD_FAILURE() << "the default library has no " << k.name << " entry (Core's L4 part entries)";
            continue;
        }
        SCOPED_TRACE(entry->name);
        const spade::Result<ed::LibraryItem> item = item_of(*library, *entry);
        ASSERT_TRUE(item.has_value()) << item.error().context;
        ASSERT_TRUE(item->part.has_value()) << "a part entry's item carries its file";
        const spade::Result<png::Image> drawn = ed::render_thumbnail(*item, mesh_roots);
        ASSERT_TRUE(drawn.has_value()) << drawn.error().context;
        EXPECT_EQ(drawn->width, ed::kThumbnailSize);
        EXPECT_EQ(drawn->height, ed::kThumbnailSize);
        const eg::PixelsView drawn_view = view_of(*drawn);
        EXPECT_TRUE(eg::differs(eg::differ_fraction(drawn_view, eg::dominant_colour(drawn_view)))) << "a blank";

        ASSERT_FALSE(entry->thumbnail.empty()) << "N14b: every entry has its thumbnail";
        const fs::path file = library->dir / entry->thumbnail;
        const spade::Result<png::Image> image = png::read_file(file);
        ASSERT_TRUE(image.has_value()) << image.error().context;
        EXPECT_EQ(image->width, ed::kThumbnailSize);
        EXPECT_EQ(image->height, ed::kThumbnailSize);
        const eg::PixelsView view = view_of(*image);
        EXPECT_TRUE(eg::differs(eg::differ_fraction(view, eg::dominant_colour(view))))
            << file.generic_string() << " is one colour: a blank, not the part";
    }
}

// The hint for a missing thumbnail names the entry's file less its trailing
// ".<kind>.yaml" only (it used to cut at the first dot).
TEST(LibraryThumbnails, TheHintNamesTheFileLessItsKindAndYaml) {
    EXPECT_EQ(suggested_thumbnail("drones/reference_quad.model.yaml"), "thumbnails/reference_quad.png");
    EXPECT_EQ(suggested_thumbnail("props/apc_10x4.5.prop.yaml"), "thumbnails/apc_10x4.5.png");
    EXPECT_EQ(suggested_thumbnail("objects/crate.v2.object.yaml"), "thumbnails/crate.v2.png");
}
