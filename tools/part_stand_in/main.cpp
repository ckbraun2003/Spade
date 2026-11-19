// spade_part_stand_in (L88l): the program around add_stand_in(). For each part file named, it reads the part's store meshes and rewrites the file
// with its generated stand-in block (stand_in.hpp has what that is and the rule it serves). Exit 0: every file done; 1: a file was refused (the
// others are still done); 2: a bad command line.
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <iterator>
#include <string>
#include <vector>

#include "stand_in.hpp"

#if defined(_MSC_VER)
#pragma warning(push)
#pragma warning(disable : 4996)  // std::getenv: read once, before anything else runs
#endif
namespace {
[[nodiscard]] std::string environment(const char* name) {
    const char* v = std::getenv(name);
    return v != nullptr ? v : "";
}
}  // namespace
#if defined(_MSC_VER)
#pragma warning(pop)
#endif

int main(int argc, char** argv) {
    namespace fs = std::filesystem;
    spade::stand_in::Options options;
    fs::path store = environment("SPADE_PART_STORE");
    std::vector<fs::path> files;
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        if ((a == "--store" || a == "--skip-material") && i + 1 < argc) {
            const std::string v = argv[++i];
            if (a == "--store") store = v;
            else options.skip_materials.push_back(v);
        } else if (!a.empty() && a[0] == '-') {
            std::cerr << "spade_part_stand_in: unknown option '" << a << "'\n";
            return 2;
        } else {
            files.emplace_back(a);
        }
    }
    if (files.empty() || store.empty()) {
        std::cerr << "usage: spade_part_stand_in [--store <dir>] [--skip-material <name ending>]... <part.yaml>...\n"
                     "  --store <dir>            the part store (default: $SPADE_PART_STORE)\n"
                     "  --skip-material <end>    a glTF material whose name ends so is left out (a bell hidden in the drawing); applies to every file named\n"
                     "writes each file in place: its own text kept, a generated block of one axis box per store object appended (run again, it is replaced).\n";
        return 2;
    }
    int status = 0;
    for (const fs::path& file : files) {
        std::ifstream in(file, std::ios::binary);
        if (!in) {
            std::cerr << file.string() << ": cannot be read\n";
            status = 1;
            continue;
        }
        const std::string text((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
        const auto result = spade::stand_in::add_stand_in(text, store, options);
        if (!result) {
            std::cerr << file.string() << ": " << result.error() << "\n";
            status = 1;
            continue;
        }
        if (result->boxes == 0) {
            std::cout << file.string() << ": no store meshes, left as it is\n";
            continue;
        }
        if (result->text != text) {
            std::ofstream out(file, std::ios::binary | std::ios::trunc);
            out << result->text;
            if (!out) {
                std::cerr << file.string() << ": cannot be written\n";
                status = 1;
                continue;
            }
        }
        std::cout << file.string() << ": " << result->boxes << " boxes (" << result->skipped << " objects under the limit)\n";
    }
    return status;
}
