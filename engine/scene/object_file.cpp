#include "scene/object_file.hpp"

#include <string>
#include <string_view>

namespace spade::scene {
namespace {

// RED: every call refuses until the object file is written.
[[nodiscard]] Error not_yet() {
    return Error{Code::internal, "the object file is not implemented yet"};
}

}  // namespace

Result<void> validate_object_file(const ObjectFile& /*file*/) {
    return std::unexpected(not_yet());
}

Result<std::string> object_to_yaml(const ObjectFile& /*file*/) {
    return std::unexpected(not_yet());
}

Result<ObjectFile> object_from_yaml(std::string_view /*text*/) {
    return std::unexpected(not_yet());
}

Result<ObjectFile> load_object_file(const std::filesystem::path& /*path*/) {
    return std::unexpected(not_yet());
}

Result<void> add_object(SceneDesc& /*scene*/, const ObjectFile& /*object*/, std::string_view /*name*/,
                        const SdfPose& /*pose*/) {
    return std::unexpected(not_yet());
}

}  // namespace spade::scene
