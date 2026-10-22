// RED (Task L5): the model-sources sidecar's API, declared and not yet
// implemented. Every function links and refuses, so tests/
// test_model_sources.cpp fails on behaviour; the implementation replaces this
// file.
#include "scene/model_sources.hpp"

#include <cstdint>
#include <expected>
#include <filesystem>
#include <string>
#include <string_view>

namespace spade::scene {
namespace {

[[nodiscard]] Error not_implemented() { return Error{Code::invalid_argument, "model_sources: not implemented"}; }

}  // namespace

// Not a Result, so it cannot refuse: 0 is no FNV-1a 64 the tests expect.
uint64_t pinned_file_hash(std::string_view) noexcept { return 0; }

Result<std::string> model_sources_to_yaml(const ModelSources&) { return std::unexpected(not_implemented()); }

Result<ModelSources> model_sources_from_yaml(std::string_view) { return std::unexpected(not_implemented()); }

Result<ModelSources> load_model_sources(const std::filesystem::path&) { return std::unexpected(not_implemented()); }

Result<void> check_model_sources(const Library&, const SourceRegistry&, const ModelSources&) {
    return std::unexpected(not_implemented());
}

}  // namespace spade::scene
