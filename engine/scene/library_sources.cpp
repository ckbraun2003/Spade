#include "scene/library_sources.hpp"

#include <filesystem>
#include <string>
#include <string_view>

namespace spade::scene {

std::string_view license_name(License license) noexcept {
    (void)license;
    return {};
}

std::string_view grade_name(Grade grade) noexcept {
    (void)grade;
    return {};
}

Result<Grade> parse_grade(std::string_view text, std::string_view what) {
    (void)text, (void)what;
    return Grade::assumed;
}

const Source* SourceRegistry::find(std::string_view id) const noexcept {
    (void)id;
    return nullptr;
}

Result<SourceRegistry> sources_from_yaml(std::string_view text) {
    (void)text;
    return SourceRegistry{};
}

Result<SourceRegistry> load_sources(const std::filesystem::path& path) {
    (void)path;
    return SourceRegistry{};
}

Result<void> check_source_ref(const SourceRegistry& registry, const SourceRef& ref, std::string_view field) {
    (void)registry, (void)ref, (void)field;
    return {};
}

}  // namespace spade::scene
