// tools/mesh_pack/main.cpp -- spade_mesh_pack's entry point: the arguments as
// paths (wmain on Windows, so a wide argument survives), then run().
#include <cstdio>
#include <filesystem>
#include <vector>

#include "mesh_pack.hpp"

namespace {

int pack(const std::vector<std::filesystem::path>& args) {
    const spade::mesh_pack::Outcome outcome = spade::mesh_pack::run(args);
    std::fwrite(outcome.out.data(), 1, outcome.out.size(), stdout);
    std::fwrite(outcome.err.data(), 1, outcome.err.size(), stderr);
    return outcome.exit_code;
}

}  // namespace

#ifdef _WIN32
int wmain(int argc, wchar_t** argv) { return pack(std::vector<std::filesystem::path>(argv + 1, argv + argc)); }
#else
int main(int argc, char** argv) { return pack(std::vector<std::filesystem::path>(argv + 1, argv + argc)); }
#endif
