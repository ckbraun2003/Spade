#include "scene/part_spec.hpp"

#include <array>
#include <exception>
#include <string>
#include <utility>

#include <yaml-cpp/yaml.h>

namespace spade::scene {

Result<std::optional<PartFile>> part_spec_from_text(std::string_view part_text) {
    YAML::Node root;
    try {
        root = YAML::Load(std::string(part_text));
    } catch (const std::exception& e) {
        return std::unexpected(Error{Code::invalid_argument, std::string("the part text is not valid YAML: ") + e.what()});
    }
    if (!root.IsMap()) {
        return std::optional<PartFile>{};
    }
    const YAML::Node spec = root["spec"];
    if (!spec.IsDefined()) {
        return std::optional<PartFile>{};
    }
    if (!spec.IsMap()) {
        return std::unexpected(Error{Code::invalid_argument, "spec: must be a mapping holding one motor, prop, esc or battery block and its sources"});
    }
    static constexpr std::array<PartKind, 4> kKinds = {PartKind::motor, PartKind::prop, PartKind::esc, PartKind::battery};
    std::optional<PartKind> kind;
    std::size_t found = 0;
    for (const PartKind k : kKinds) {
        if (spec[std::string(part_kind_name(k))].IsDefined()) {
            kind = k;
            ++found;
        }
    }
    if (found != 1 || !kind) {
        return std::unexpected(Error{Code::invalid_argument, "spec: must hold exactly one of motor, prop, esc and battery (it holds " + std::to_string(found) + ")"});
    }
    // The spec's body is a v1 part file's: the same text, the part_version line put back.
    const std::string v1 = "part_version: " + std::to_string(kPartVersion) + "\n" + YAML::Dump(spec) + "\n";
    Result<PartFile> part = part_from_yaml(v1, *kind);
    if (!part) {
        return std::unexpected(Error{part.error().code, "spec: " + part.error().context});
    }
    return std::optional<PartFile>(std::move(*part));
}

}  // namespace spade::scene
