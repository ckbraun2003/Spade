#include "scene/model_file.hpp"

#include <string>
#include <string_view>

namespace spade::scene {

// RED stubs: every call is refused.

Result<void> validate_model_file(const ModelFile& file) {
    static_cast<void>(file);
    return std::unexpected(Error{Code::internal, "model file: not implemented"});
}

Result<std::string> model_to_yaml(const ModelFile& file) {
    static_cast<void>(file);
    return std::unexpected(Error{Code::internal, "model file: not implemented"});
}

Result<ModelFile> model_from_yaml(std::string_view text, const std::filesystem::path& base_dir) {
    static_cast<void>(text);
    static_cast<void>(base_dir);
    return std::unexpected(Error{Code::internal, "model file: not implemented"});
}

Result<ModelFile> load_model_file(const std::filesystem::path& path) {
    static_cast<void>(path);
    return std::unexpected(Error{Code::internal, "model file: not implemented"});
}

}  // namespace spade::scene
