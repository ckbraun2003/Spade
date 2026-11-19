#pragma once

// ---------------------------------------------------------------------------
// tests/library_index.hpp -- the default root's index, alone, as a scene::Library (L88l).
//
// assets/library is a version-3 root: library.yaml names the root and index.yaml lists the entries. scene::library_from_yaml() reads only the version
// 1 and 2 text, and scene::load_library() checks the disk (a thumbnail a generator has yet to write is refused), so the generators and the tests that
// need "the index, no disk check" read index.yaml here: the entries of the kinds LibraryKind knows, in the file's order, name, category, file,
// thumbnail and kind as written (the kind by the file's extension), `dir` set to the folder.
// ---------------------------------------------------------------------------

#include <filesystem>
#include <string>

#include <yaml-cpp/yaml.h>

#include "core/error.hpp"
#include "scene/library.hpp"

namespace spade::test {

[[nodiscard]] inline Result<scene::Library> default_root_index(const std::filesystem::path& dir) {
    YAML::Node doc;
    try {
        doc = YAML::LoadFile((dir / "index.yaml").string());
    } catch (const YAML::Exception& e) {
        return std::unexpected(Error{Code::io_error, std::string("index.yaml can't be read: ") + e.what()});
    }
    struct Suffix {
        const char* text;
        scene::LibraryKind kind;
    };
    static const Suffix kSuffixes[] = {{".model.yaml", scene::LibraryKind::model},   {".object.yaml", scene::LibraryKind::object},
                                       {".motor.yaml", scene::LibraryKind::motor},   {".prop.yaml", scene::LibraryKind::prop},
                                       {".esc.yaml", scene::LibraryKind::esc},       {".battery.yaml", scene::LibraryKind::battery}};
    scene::Library library;
    library.dir = dir;
    for (const YAML::Node& e : doc["entries"]) {
        scene::LibraryEntry out;
        out.name = e["name"].as<std::string>();
        out.category = e["category"].as<std::string>();
        out.file = e["file"].as<std::string>();
        if (e["thumbnail"]) out.thumbnail = e["thumbnail"].as<std::string>();
        bool known = false;
        for (const Suffix& s : kSuffixes) {
            const std::string t = s.text;
            if (out.file.size() > t.size() && out.file.compare(out.file.size() - t.size(), t.size(), t) == 0) {
                out.kind = s.kind;
                known = true;
            }
        }
        if (known) library.entries.push_back(std::move(out));
    }
    return library;
}

}  // namespace spade::test
