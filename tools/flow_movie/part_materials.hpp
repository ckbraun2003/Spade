// tools/flow_movie/part_materials.hpp -- a role-to-material table for a library drone's parts (the lead, 2026-11-17; Cameron: "the
// drone should look fairly realistic").
//
// Today every non-rotor part of a library model draws one light grey: render_parts::bind_parts has three materials, by KIND (body,
// rotor, porous), and never reads a part's role (the engine does not: model_part.hpp, "a free label"). So a battery, a camera and
// a pair of legs look like the shell. This is the tool's own display rule, after the parts are bound: each draw item's part
// ("<object>/<part name>") is looked up in the model, and a part whose role the table knows gets that role's flat colour. Nothing
// about the flow or the physics reads a role. An unknown or empty role keeps the item's material.
//
// The look is lambert and flat (render::Material has a base colour and a shading mode, no specular or metalness): "metal" is a
// mid-grey, "glossy" is dark. The colours are the tool's to tune; a model sets only the role.
#pragma once

#include <array>
#include <cstddef>
#include <map>
#include <optional>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include <glm/glm.hpp>

#include "render/scene.hpp"

namespace flow_movie {

struct RoleLook {
    std::string_view role;
    glm::vec3 colour;
};

inline constexpr std::array<RoleLook, 14> kRoleLooks{{
    {"body", {0.86f, 0.87f, 0.90f}},     // the shell: matte pale plastic
    {"plate", {0.10f, 0.10f, 0.12f}},    // frame plates: dark carbon
    {"arm", {0.22f, 0.23f, 0.26f}},      // arms: dark grey
    {"motor", {0.55f, 0.56f, 0.60f}},    // motors: brushed metal, a mid-grey
    {"prop", {0.08f, 0.08f, 0.10f}},     // blades: dark carbon
    {"battery", {0.16f, 0.20f, 0.28f}},  // the pack: darker blue-grey
    {"strap", {0.03f, 0.03f, 0.035f}},   // the battery strap: black rubber
    {"camera", {0.05f, 0.05f, 0.06f}},   // near-black
    {"gimbal", {0.07f, 0.07f, 0.08f}},
    {"lens", {0.02f, 0.03f, 0.05f}},     // glass, dark
    {"gear", {0.72f, 0.73f, 0.76f}},     // legs: matte light grey
    {"esc", {0.14f, 0.15f, 0.17f}},
    {"gps", {0.20f, 0.20f, 0.22f}},
    {"antenna", {0.03f, 0.03f, 0.03f}},
}};

// The colour of `role` (exact, lower case), or nothing for one the table does not know, empty included.
[[nodiscard]] inline std::optional<glm::vec3> role_colour(std::string_view role) {
    for (const RoleLook& look : kRoleLooks) {
        if (look.role == role) return look.colour;
    }
    return std::nullopt;
}

// Gives the draw items from `first_item` on the material of their part's role. `part_roles`: the model's parts, name then role. An
// item's part label is "<object>/<part name>"; the name after the first '/' is looked up. A role gets one material, appended to
// scene.materials the first time it is used; an item whose part is not listed, whose role is unknown or empty, or whose label is
// not a label keeps its material. Returns the number of items changed.
[[nodiscard]] inline std::size_t apply_role_materials(spade::render::RenderScene& scene, std::size_t first_item,
                                                      const std::vector<std::pair<std::string, std::string>>& part_roles) {
    std::map<std::string, uint32_t, std::less<>> made;  // role -> its material's index in the scene
    std::size_t changed = 0;
    for (std::size_t i = first_item; i < scene.statics.size(); ++i) {
        spade::render::DrawItem& item = scene.statics[i];
        if (item.part_label >= scene.labels.size()) continue;
        const std::string& label = scene.labels[item.part_label];
        const std::size_t slash = label.find('/');
        if (slash == std::string::npos) continue;
        const std::string_view part = std::string_view(label).substr(slash + 1);
        const std::string* role = nullptr;
        for (const auto& [name, r] : part_roles) {
            if (name == part) {
                role = &r;
                break;
            }
        }
        if (role == nullptr) continue;
        const std::optional<glm::vec3> colour = role_colour(*role);
        if (!colour) continue;
        auto it = made.find(*role);
        if (it == made.end()) {
            scene.materials.push_back(spade::render::Material{.base_color = glm::vec4(*colour, 1.0f)});
            it = made.emplace(*role, static_cast<uint32_t>(scene.materials.size() - 1u)).first;
        }
        item.material_override = it->second;
        ++changed;
    }
    return changed;
}

}  // namespace flow_movie
