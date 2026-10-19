// The editor's docked panels (editor spec §13; plan Task 10). See the header.
//
// Every panel is flat: sections fold, properties are aligned rows, lists are
// rows on the panel (§13, checked by test_sandbox_ui_conventions.cpp). Style
// lives in ui_theme.hpp alone.

#include "editor_panels_ui.hpp"

#if SPADE_SANDBOX_HAS_GL

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include "drone_sim.hpp"
#include "editor_library.hpp"
#include "editor_display.hpp"
#include "editor_model_params.hpp"
#include "editor_panels.hpp"
#include "ui_theme.hpp"

namespace spade::sandbox {

namespace {

namespace ed = editor;
using Element = vehicles::ModelIssue::Element;

constexpr float kRadToDeg = 57.2957795f;

// ---- widgets that report once, on release (INT-2) -------------------------
//
// While a widget is held, the panel shows its own copy of the value, so a
// drag moves smoothly even though the document has not changed yet; the edit
// is queued once, when the widget is let go.
struct Held {
    ImGuiID id = 0;
    float v[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    uint32_t u = 0;
    std::string text;
};

Held& held() {
    static Held h;
    return h;
}

bool drag_floats(const char* id, const float* current, int n, float speed, float* out, const char* format = "%.3f") {
    Held& h = held();
    const ImGuiID key = ImGui::GetID(id);
    // Shown as it prints: a value that rounds to zero shows as 0, never -0.
    const int decimals = ed::decimals_of(format);
    float v[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    for (int i = 0; i < n; ++i) v[i] = h.id == key ? h.v[i] : ed::shown(current[i], decimals);
    ImGui::SetNextItemWidth(-FLT_MIN);
    ImGui::DragScalarN(id, ImGuiDataType_Float, v, n, speed, nullptr, nullptr, format);
    if (ImGui::IsItemActive()) {
        h.id = key;
        std::copy(v, v + n, h.v);
    }
    const bool done = ImGui::IsItemDeactivatedAfterEdit();
    if (done) std::copy(v, v + n, out);
    if (!ImGui::IsItemActive() && h.id == key) h.id = 0;
    return done;
}

bool drag_u32(const char* id, uint32_t current, uint32_t& out) {
    Held& h = held();
    const ImGuiID key = ImGui::GetID(id);
    uint32_t v = h.id == key ? h.u : current;
    ImGui::SetNextItemWidth(-FLT_MIN);
    ImGui::DragScalar(id, ImGuiDataType_U32, &v, 0.2f);
    if (ImGui::IsItemActive()) {
        h.id = key;
        h.u = v;
    }
    const bool done = ImGui::IsItemDeactivatedAfterEdit();
    if (done) out = v;
    if (!ImGui::IsItemActive() && h.id == key) h.id = 0;
    return done;
}

bool edit_text(const char* id, const std::string& current, std::string& out) {
    Held& h = held();
    const ImGuiID key = ImGui::GetID(id);
    char buf[256] = {};
    const std::string& shown = h.id == key ? h.text : current;
    std::snprintf(buf, sizeof buf, "%s", shown.c_str());
    ImGui::SetNextItemWidth(-FLT_MIN);
    ImGui::InputText(id, buf, sizeof buf);
    if (ImGui::IsItemActive()) {
        h.id = key;
        h.text = buf;
    }
    const bool done = ImGui::IsItemDeactivatedAfterEdit();
    if (done) out = buf;
    if (!ImGui::IsItemActive() && h.id == key) h.id = 0;
    return done;
}

// Yaw, pitch and roll in degrees, in the drone readouts' order (Y, Z, X).
bool drag_attitude(const char* id, const glm::quat& current, glm::quat& out) {
    const AttitudeTarget a = attitude_from_quat(current);
    const float deg[3] = {a.yaw * kRadToDeg, a.pitch * kRadToDeg, a.roll * kRadToDeg};
    float next[3];
    if (!drag_floats(id, deg, 3, 0.5f, next, "%.1f")) return false;
    out = attitude_quat(AttitudeTarget{next[0] / kRadToDeg, next[1] / kRadToDeg, next[2] / kRadToDeg});
    return true;
}

bool vec3_row(const char* label, const char* id, const glm::vec3& current, float speed, glm::vec3& out,
              const char* format = "%.3f") {
    ui::property_row(label);
    float next[3];
    if (!drag_floats(id, &current.x, 3, speed, next, format)) return false;
    out = glm::vec3(next[0], next[1], next[2]);
    return true;
}

bool float_row(const char* label, const char* id, float current, float speed, float& out, const char* format = "%.3f") {
    ui::property_row(label);
    return drag_floats(id, &current, 1, speed, &out, format);
}

// A colour as three channels and a swatch, held like a drag. No picker popup:
// its edits would not arrive as one release.
bool colour_row(const char* label, const char* id, const glm::vec3& current, glm::vec3& out) {
    ui::property_row(label);
    Held& h = held();
    const ImGuiID key = ImGui::GetID(id);
    float c[3] = {current.r, current.g, current.b};
    if (h.id == key) std::copy(h.v, h.v + 3, c);
    ImGui::ColorEdit3(id, c, ImGuiColorEditFlags_Float | ImGuiColorEditFlags_NoPicker);
    if (ImGui::IsItemActive()) {
        h.id = key;
        std::copy(c, c + 3, h.v);
    }
    const bool done = ImGui::IsItemDeactivatedAfterEdit();
    if (done) out = glm::vec3(c[0], c[1], c[2]);
    if (!ImGui::IsItemActive() && h.id == key) h.id = 0;
    return done;
}

// ---- the toolbar ------------------------------------------------------------

void toolbar(ed::EditorPanels& p) {
    if (!ImGui::Begin(ui::kToolbar, nullptr, ui::kPanelFlags | ImGuiWindowFlags_NoScrollbar)) {
        ImGui::End();
        return;
    }
    const auto press = [&p](const char* label, ed::Command c, bool enabled = true) {
        ImGui::BeginDisabled(!enabled);
        if (ImGui::Button(label)) p.commands.push_back(c);
        ImGui::EndDisabled();
        ImGui::SameLine();
    };
    press(p.running ? "Stop" : "Run", p.running ? ed::Command::stop : ed::Command::run);
    press(p.paused ? "Resume" : "Pause", p.paused ? ed::Command::resume : ed::Command::pause, p.running);
    press("Step", ed::Command::step, p.running);
    ImGui::AlignTextToFramePadding();
    ImGui::TextDisabled("tick %llu", static_cast<unsigned long long>(p.tick));
    ImGui::SameLine(0.0f, 6.0f * ui::kBase);
    press("Undo", ed::Command::undo, p.can_undo);
    press("Redo", ed::Command::redo, p.can_redo);
    ImGui::SameLine(0.0f, 4.0f * ui::kBase);
    // The gizmo: its tool (W, E) and snapping (Ctrl flips it while held).
    if (ImGui::RadioButton("Move", p.tool == ed::GizmoTool::move)) p.tool = ed::GizmoTool::move;
    ImGui::SameLine();
    if (ImGui::RadioButton("Rotate", p.tool == ed::GizmoTool::rotate)) p.tool = ed::GizmoTool::rotate;
    ImGui::SameLine();
    ImGui::Checkbox("Snap", &p.snap);
    ImGui::SameLine(0.0f, 4.0f * ui::kBase);
    press("Save", ed::Command::save);
    // The live smoke opens and closes the menu by request, as a click would.
    const bool open_request = p.file_menu.has_value() && *p.file_menu;
    const bool close_request = p.file_menu.has_value() && !*p.file_menu;
    p.file_menu.reset();
    if (ImGui::Button("File") || open_request) ImGui::OpenPopup("file");
    if (ImGui::BeginPopup("file")) {
        if (close_request) ImGui::CloseCurrentPopup();
        // The scene picker (SL13): the built-in scenes, then the saved ones.
        if (!p.sources.empty()) {
            ImGui::TextDisabled("Scenes");
            for (std::size_t i = 0; i < p.sources.size(); ++i) {
                const ed::SceneSource& source = p.sources[i];
                ImGui::PushID(static_cast<int>(i));
                const std::string label =
                    source.name + (source.kind == ed::SceneSource::Kind::saved ? "  (saved)" : "");
                if (ImGui::Selectable(label.c_str())) {
                    p.path = source.file.string();
                    p.commands.push_back(ed::Command::open);
                }
                ImGui::PopID();
            }
            ui::gap();
        }
        ImGui::TextDisabled("Scene file");
        std::string path = p.path;
        if (edit_text("##path", p.path, path)) p.path = path;
        if (ImGui::Button("Open")) {
            p.commands.push_back(ed::Command::open);
            ImGui::CloseCurrentPopup();
        }
        ImGui::SameLine();
        if (ImGui::Button("Save As")) {
            p.commands.push_back(ed::Command::save_as);
            ImGui::CloseCurrentPopup();
        }
        ImGui::EndPopup();
    }
    ImGui::SameLine(0.0f, 6.0f * ui::kBase);
    ImGui::AlignTextToFramePadding();
    ImGui::TextUnformatted(p.title.c_str());
    ImGui::End();
}

// ---- the hierarchy ----------------------------------------------------------

void hierarchy(ed::EditorPanels& p) {
    if (!ImGui::Begin(ui::kHierarchy, nullptr, ui::kPanelFlags)) {
        ImGui::End();
        return;
    }
    for (std::size_t i = 0; i < p.hierarchy.size(); ++i) {
        const ed::HierarchyRow& row = p.hierarchy[i];
        ImGui::PushID(static_cast<int>(i));
        // The scene and the world are headings of a kind: their settings, not objects.
        const std::string label = row.target.kind == ed::Selection::Kind::scene   ? "Scene  " + row.label
                                  : row.target.kind == ed::Selection::Kind::world ? "World  " + row.label
                                                                                  : "    " + row.label;
        if (ui::list_row(label.c_str(), p.selection == row.target)) p.selection = row.target;
        ImGui::PopID();
    }
    ImGui::End();
}

// ---- the inspector ----------------------------------------------------------

// Which of a part's rows to draw: the common ones, or the model's Advanced
// fold (its internals, and its principal axes as a raw quaternion).
enum class Rows { common, advanced };

void model_param_rows(ed::EditorPanels& p, const std::string& vehicle, const vehicles::ModelType& m, Element element,
                      std::size_t index, Rows rows) {
    for (const ed::ModelParam& param : ed::model_params()) {
        if (param.element != element) continue;
        const ed::ParamDisplay display = ed::param_display(element, param.field);
        // An orientation reads as angles; the model's own also as a raw
        // quaternion, in Advanced, for whoever needs the exact numbers.
        const bool raw_quaternion =
            rows == Rows::advanced && param.kind == ed::ParamKind::quat && element == Element::model;
        if (rows == Rows::common && display.advanced) continue;
        if (rows == Rows::advanced && !display.advanced && !raw_quaternion) continue;
        const Result<ed::ParamValue> value = ed::get_model_param(m, element, index, param.field);
        if (!value) continue;
        const std::string field(param.field);
        const std::string id = "##" + field + (raw_quaternion ? "-raw" : "");
        std::string label = display.label;
        if (param.kind == ed::ParamKind::quat) label += raw_quaternion ? " (w x y z)" : " (\xC2\xB0)";
        ImGui::PushID(static_cast<int>(index));
        ui::property_row(label.c_str());
        const auto queue = [&](ed::ParamValue v) {
            p.pending.push_back(ed::SetModelParam{vehicle, element, index, field, std::move(v)});
        };
        switch (param.kind) {
            case ed::ParamKind::scalar: {
                float out = 0.0f;
                if (drag_floats(id.c_str(), &std::get<float>(*value), 1, 0.001f, &out, "%.5g")) queue(out);
                break;
            }
            case ed::ParamKind::count: {
                uint32_t out = 0;
                if (drag_u32(id.c_str(), std::get<uint32_t>(*value), out)) queue(out);
                break;
            }
            case ed::ParamKind::vec3: {
                const glm::vec3& v = std::get<glm::vec3>(*value);
                float out[3];
                if (drag_floats(id.c_str(), &v.x, 3, 0.001f, out, "%.4g")) queue(glm::vec3(out[0], out[1], out[2]));
                break;
            }
            case ed::ParamKind::quat: {
                const glm::quat& q = std::get<glm::quat>(*value);
                if (raw_quaternion) {
                    const float wxyz[4] = {q.w, q.x, q.y, q.z};
                    float out[4];
                    if (drag_floats(id.c_str(), wxyz, 4, 0.001f, out, "%.3f")) {
                        queue(glm::quat(out[0], out[1], out[2], out[3]));
                    }
                } else {
                    glm::quat out;
                    if (drag_attitude(id.c_str(), q, out)) queue(out);
                }
                break;
            }
            case ed::ParamKind::text: {
                std::string out;
                if (edit_text(id.c_str(), std::get<std::string>(*value), out)) queue(out);
                break;
            }
        }
        ImGui::PopID();
    }
}

void inspect_vehicle(ed::EditorPanels& p, const scene::SceneVehicle& v) {
    const vehicles::ModelType* model = nullptr;
    for (const vehicles::ModelType& m : p.scene->models) {
        if (m.name == v.model) model = &m;
    }
    ui::title(v.name.c_str());
    if (model != nullptr) ImGui::TextDisabled("%s  flying %s", ed::vehicle_kind_of(*model).c_str(), model->name.c_str());
    if (ui::section("Start", true) && ui::begin_properties("start")) {
        VehicleSpawn next = v.start;
        bool changed = vec3_row("Position (m)", "##pos", v.start.pos, 0.01f, next.pos);
        ui::property_row("Attitude (\xC2\xB0)");
        changed |= drag_attitude("##att", v.start.orient, next.orient);
        changed |= vec3_row("Velocity (m/s)", "##vel", v.start.vel, 0.01f, next.vel, "%.2f");
        changed |= float_row("Rotor speed", "##rotor", v.start.rotor_omega, 1.0f, next.rotor_omega, "%.0f rad/s");
        if (changed) p.pending.push_back(ed::SetVehicleStart{v.name, next});
        ui::end_properties();
    }
    if (model == nullptr) return;
    if (ui::section("Model", true) && ui::begin_properties("model")) {
        model_param_rows(p, v.name, *model, Element::model, 0, Rows::common);
        ui::end_properties();
    }
    // The parts and the IMU's noise are advanced, folded (§13).
    for (const Element e : {Element::rotor, Element::drag_body, Element::imu_mount}) {
        const std::size_t n = ed::detail::part_count(*model, e);
        if (n == 0) continue;
        const std::string heading = std::string(ed::detail::parts_word(e)) + " (" + std::to_string(n) + ")";
        if (!ui::section(heading.c_str(), false)) continue;
        for (std::size_t i = 0; i < n; ++i) {
            ImGui::TextDisabled("%s", ed::part_name(e, i).c_str());
            const std::string table = heading + std::to_string(i);
            if (ui::begin_properties(table.c_str())) {
                model_param_rows(p, v.name, *model, e, i, Rows::common);
                ui::end_properties();
            }
        }
    }
    // The internals, folded (§13's progressive disclosure).
    if (ui::section("Advanced", false) && ui::begin_properties("advanced")) {
        model_param_rows(p, v.name, *model, Element::model, 0, Rows::advanced);
        ui::end_properties();
    }
}

void inspect_asset(ed::EditorPanels& p, const scene::SceneAsset& a) {
    ui::title(a.name.c_str());
    if (ui::section("Transform", true) && ui::begin_properties("transform")) {
        SdfPose next = a.pose;
        bool changed = vec3_row("Position (m)", "##pos", a.pose.position, 0.01f, next.position);
        ui::property_row("Rotation (\xC2\xB0)");
        changed |= drag_attitude("##rot", a.pose.rotation, next.rotation);
        float scale = a.pose.scale;
        if (float_row("Scale", "##scale", a.pose.scale, 0.01f, scale)) {
            next.scale = scale;
            changed = true;
        }
        if (changed) p.pending.push_back(ed::SetAssetPose{a.name, next});
        ui::end_properties();
    }
    if (ui::section("Material", true) && ui::begin_properties("material")) {
        // The world's palette, then the scene's; choosing another queues `edit`.
        const auto combo = [&](const char* label, const char* id, const std::string& current, const char* shown,
                               const auto& edit) {
            ui::property_row(label);
            if (!ImGui::BeginCombo(id, shown)) return;
            const auto option = [&](const std::string& name) {
                if (ImGui::Selectable(name.c_str(), name == current) && name != current) {
                    p.pending.push_back(edit(name));
                }
            };
            for (const MaterialDesc& m : p.world->materials) option(m.name);
            for (const MaterialDesc& m : p.scene->materials) option(m.name);
            ImGui::EndCombo();
        };
        if (!a.visual.mesh_ref.empty()) {
            combo("Visual", "##material", a.visual.material,
                  a.visual.material.empty() ? "default" : a.visual.material.c_str(),
                  [&](const std::string& name) { return ed::SceneEdit{ed::SetAssetMaterial{a.name, name}}; });
        }
        if (!a.collider.nodes.empty()) {
            // An asset without a visual draws as its collider, in these materials.
            const std::string first = a.collider_materials.empty() ? std::string() : a.collider_materials.front();
            const bool mixed = std::any_of(a.collider_materials.begin(), a.collider_materials.end(),
                                           [&](const std::string& m) { return m != first; });
            combo("Collider", "##collider_material", mixed ? std::string() : first,
                  mixed ? "mixed" : (first.empty() ? "default" : first.c_str()),
                  [&](const std::string& name) { return ed::SceneEdit{ed::SetColliderMaterial{a.name, name}}; });
        }
        ui::end_properties();
    }
    if (ui::section("Collider", false) && ui::begin_properties("collider")) {
        ui::value_row("Nodes", "%zu", a.collider.nodes.size());
        ui::value_row("Transforms", "%zu", a.collider.transforms.size());
        ui::end_properties();
    }
}

void inspect_scene(ed::EditorPanels& p) {
    const scene::SceneDesc& s = *p.scene;
    ui::title(s.name.c_str());
    if (ui::section("World", true) && ui::begin_properties("world ref")) {
        ui::value_row("File", "%s", s.world.file.c_str());
        ui::value_row("Hash", "0x%016llx", static_cast<unsigned long long>(s.world.hash));
        ui::end_properties();
    }
    if (ui::section("Materials", true)) {
        if (ui::begin_properties("materials")) {
            for (const MaterialDesc& m : s.materials) {
                ui::value_row(m.name.c_str(), "%.2f  %.2f  %.2f", m.base_color.r, m.base_color.g, m.base_color.b);
            }
            ui::end_properties();
        }
        if (ImGui::Button("Add material")) {
            MaterialDesc m;
            m.name = "material";
            p.pending.push_back(ed::AddMaterial{m});
        }
    }
    if (ui::section("Spare slots", false) && ui::begin_properties("spare")) {
        ui::value_row("Bodies", "%u", s.spare.bodies);
        ui::value_row("Force elements", "%u", s.spare.force_elements);
        ui::value_row("Sensors", "%u", s.spare.sensors);
        ui::value_row("Contacts", "%u", s.spare.contacts);
        ui::end_properties();
    }
}

void inspect_world(ed::EditorPanels& p) {
    const WorldDesc& w = *p.world;
    ui::title(w.name.c_str());
    if (ui::section("Environment", true) && ui::begin_properties("environment")) {
        Environment next = w.environment;
        bool changed = vec3_row("Gravity (m/s\xC2\xB2)", "##gravity", w.environment.gravity, 0.01f, next.gravity, "%.2f");
        changed |= vec3_row("Wind (m/s)", "##wind", w.environment.wind, 0.05f, next.wind, "%.2f");
        changed |= float_row("Air density", "##rho", w.environment.air_density, 0.001f, next.air_density, "%.4f kg/m\xC2\xB3");
        changed |= float_row("Temperature", "##temp", w.environment.temperature_k, 0.1f, next.temperature_k, "%.2f K");
        if (changed) p.pending_world.push_back(ed::SetEnvironment{next});
        ui::end_properties();
    }
    if (ui::section("Capacities", false) && ui::begin_properties("capacities")) {
        Capacities next = w.capacities;
        bool changed = false;
        ui::property_row("Bodies");
        changed |= drag_u32("##bodies", w.capacities.bodies, next.bodies);
        ui::property_row("Force elements");
        changed |= drag_u32("##force", w.capacities.force_elements, next.force_elements);
        ui::property_row("Sensors");
        changed |= drag_u32("##sensors", w.capacities.sensors, next.sensors);
        ui::property_row("Contacts");
        changed |= drag_u32("##contacts", w.capacities.contacts, next.contacts);
        if (changed) p.pending_world.push_back(ed::SetCapacities{next});
        ui::end_properties();
    }
    if (ui::section("Lighting", true) && ui::begin_properties("lighting")) {
        LightingDesc next = w.lighting;
        bool changed = vec3_row("Sun direction", "##sundir", w.lighting.sun_direction, 0.01f, next.sun_direction);
        changed |= colour_row("Sun colour", "##suncol", w.lighting.sun_color, next.sun_color);
        changed |= float_row("Sun intensity", "##sunint", w.lighting.sun_intensity, 0.01f, next.sun_intensity);
        changed |= colour_row("Ambient", "##ambient", w.lighting.ambient_color, next.ambient_color);
        if (changed) p.pending_world.push_back(ed::SetLighting{next});
        ui::end_properties();
    }
    if (ui::section("Sky", false) && ui::begin_properties("sky")) {
        LightingDesc next = w.lighting;
        bool changed = colour_row("Zenith", "##zenith", w.lighting.sky_zenith, next.sky_zenith);
        changed |= colour_row("Horizon", "##horizon", w.lighting.sky_horizon, next.sky_horizon);
        if (changed) p.pending_world.push_back(ed::SetLighting{next});
        ui::end_properties();
    }
    if (ui::section("Spawn points", false)) {
        if (ui::begin_properties("spawns")) {
            for (const SpawnPoint& sp : w.spawns) {
                ImGui::PushID(sp.name.c_str());
                SpawnPoint next = sp;
                if (vec3_row((sp.name + " (m)").c_str(), "##spawn", sp.position, 0.01f, next.position)) {
                    p.pending_world.push_back(ed::SetSpawn{sp.name, next});
                }
                ImGui::PopID();
            }
            ui::end_properties();
        }
        if (ImGui::Button("Add spawn point")) p.pending_world.push_back(ed::AddSpawn{SpawnPoint{}});
    }
    if (ui::section("Props", false) && ui::begin_properties("props")) {
        for (std::size_t i = 0; i < w.props.size(); ++i) {
            ui::value_row(std::to_string(i).c_str(), "%s", w.props[i].mesh_ref.c_str());
        }
        ui::end_properties();
    }
    // EDT-018: what the sandbox keeps for the world, which its file cannot hold yet.
    if (p.records != nullptr && ui::section("Not saved with the world", false)) {
        ImGui::TextDisabled("Kept by the sandbox until world file v3 carries them.");
        if (ui::begin_properties("records")) {
            ui::value_row("Gusts", "%.2f  %.2f  %.2f m/s", p.records->turbulence.sigma_u, p.records->turbulence.sigma_v,
                          p.records->turbulence.sigma_w);
            ui::value_row("Restitution", "%.2f", p.records->contacts.restitution_e);
            ui::value_row("Friction", "%.2f", p.records->contacts.friction_mu);
            ui::value_row("Contact proxy", "%.3f m", p.records->contacts.proxy_radius);
            ui::value_row("Grid cell", "%.3f m", p.records->grid.cell_size);
            ui::end_properties();
        }
    }
}

void inspector(ed::EditorPanels& p) {
    if (!ImGui::Begin(ui::kInspector, nullptr, ui::kPanelFlags)) {
        ImGui::End();
        return;
    }
    if (!p.status.empty()) {
        ImGui::PushTextWrapPos(0.0f);
        ui::status_text(p.status);
        ImGui::PopTextWrapPos();
        ui::gap();
    }
    if (p.scene == nullptr || p.world == nullptr) {
        ImGui::End();
        return;
    }
    switch (p.selection.kind) {
        case ed::Selection::Kind::none: ImGui::TextDisabled("Select an object, the scene or the world."); break;
        case ed::Selection::Kind::scene: inspect_scene(p); break;
        case ed::Selection::Kind::world: inspect_world(p); break;
        case ed::Selection::Kind::object: {
            bool found = false;
            for (const scene::SceneVehicle& v : p.scene->vehicles) {
                if (v.name == p.selection.name) {
                    inspect_vehicle(p, v);
                    found = true;
                }
            }
            for (const scene::SceneAsset& a : p.scene->assets) {
                if (!found && a.name == p.selection.name) inspect_asset(p, a);
            }
            if (ImGui::Button("Duplicate")) p.pending.push_back(ed::DuplicateObject{p.selection.name});
            ImGui::SameLine();
            if (ImGui::Button("Delete")) p.pending.push_back(ed::RemoveObject{p.selection.name});
            break;
        }
    }
    ImGui::End();
}

// ---- the library ------------------------------------------------------------

void library(ed::EditorPanels& p) {
    if (!ImGui::Begin(ui::kLibrary, nullptr, ui::kPanelFlags)) {
        ImGui::End();
        return;
    }
    // Vehicle models by the kind their model makes them, then the assets (§4).
    std::vector<std::string> kinds;
    for (const ed::LibraryItem& item : p.library) {
        if (item.kind != ed::LibraryItem::Kind::model || !item.model) continue;
        const std::string k = ed::vehicle_kind_of(*item.model);
        if (std::find(kinds.begin(), kinds.end(), k) == kinds.end()) kinds.push_back(k);
    }
    const auto row = [&p](std::size_t i) {
        ImGui::PushID(static_cast<int>(i));
        const bool armed = p.armed && *p.armed == i;
        if (ui::list_row(p.library[i].name.c_str(), armed)) {
            if (armed) {
                p.armed.reset();
            } else {
                p.armed = i;
            }
        }
        ImGui::PopID();
    };
    for (const std::string& k : kinds) {
        if (!ui::section(k.c_str(), true)) continue;
        for (std::size_t i = 0; i < p.library.size(); ++i) {
            const ed::LibraryItem& item = p.library[i];
            if (item.kind == ed::LibraryItem::Kind::model && item.model && ed::vehicle_kind_of(*item.model) == k) row(i);
        }
    }
    if (ui::section("Assets", true)) {
        for (std::size_t i = 0; i < p.library.size(); ++i) {
            if (p.library[i].kind == ed::LibraryItem::Kind::asset) row(i);
        }
    }
    if (p.armed) {
        ui::gap();
        ImGui::TextDisabled("Click in the view to place %s.", p.library[*p.armed].name.c_str());
    }
    ImGui::End();
}

}  // namespace

void draw_editor_panels(editor::EditorPanels& panels) {
    toolbar(panels);
    hierarchy(panels);
    inspector(panels);
    library(panels);
}

}  // namespace spade::sandbox

#else

namespace spade::sandbox {
void draw_editor_panels(editor::EditorPanels&) {}
}  // namespace spade::sandbox

#endif
