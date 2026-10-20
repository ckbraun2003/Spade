#include "scene/library.hpp"

#include <string>

// N3, RED: the reader's shape only. Every call is refused, so the library's
// tests compile and fail on behaviour; GREEN implements them.

namespace spade::scene {
namespace {

[[nodiscard]] Error not_built() {
    return Error{Code::internal, "the library reader is not built yet (N3, RED)"};
}

}  // namespace

Result<Library> library_from_yaml(std::string_view) {
    return std::unexpected(not_built());
}

Result<Library> load_library(const std::filesystem::path&) {
    return std::unexpected(not_built());
}

Result<ModelFile> load_library_model(const Library&, const LibraryEntry&) {
    return std::unexpected(not_built());
}

Result<ObjectFile> load_library_object(const Library&, const LibraryEntry&) {
    return std::unexpected(not_built());
}

Result<void> check_library_contents(const Library&) {
    return std::unexpected(not_built());
}

}  // namespace spade::scene
