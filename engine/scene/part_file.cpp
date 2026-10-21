#include "scene/part_file.hpp"

#include <filesystem>
#include <string>
#include <string_view>
#include <vector>

namespace spade::scene {

std::string_view part_kind_name(PartKind kind) noexcept {
    (void)kind;
    return {};
}

std::string_view part_suffix(PartKind kind) noexcept {
    (void)kind;
    return {};
}

PartKind part_kind(const PartFile& part) noexcept { return static_cast<PartKind>(part.index()); }

std::vector<std::string> cited_fields(const PartFile& part) {
    (void)part;
    return {};
}

Result<std::string> motor_to_yaml(const MotorPart& part) {
    (void)part;
    return std::string{};
}

Result<std::string> prop_to_yaml(const PropPart& part) {
    (void)part;
    return std::string{};
}

Result<std::string> esc_to_yaml(const EscPart& part) {
    (void)part;
    return std::string{};
}

Result<std::string> battery_to_yaml(const BatteryPart& part) {
    (void)part;
    return std::string{};
}

Result<std::string> part_to_yaml(const PartFile& part) {
    (void)part;
    return std::string{};
}

Result<MotorPart> motor_from_yaml(std::string_view text) {
    (void)text;
    return MotorPart{};
}

Result<PropPart> prop_from_yaml(std::string_view text) {
    (void)text;
    return PropPart{};
}

Result<EscPart> esc_from_yaml(std::string_view text) {
    (void)text;
    return EscPart{};
}

Result<BatteryPart> battery_from_yaml(std::string_view text) {
    (void)text;
    return BatteryPart{};
}

Result<PartFile> part_from_yaml(std::string_view text, PartKind kind) {
    (void)text, (void)kind;
    return PartFile{};
}

Result<void> check_part_sources(const PartFile& part, const SourceRegistry& registry) {
    (void)part, (void)registry;
    return {};
}

Result<PartFile> load_part_file(const std::filesystem::path& path, const SourceRegistry& registry) {
    (void)path, (void)registry;
    return PartFile{};
}

}  // namespace spade::scene
