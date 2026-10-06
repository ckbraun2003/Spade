// The sandbox's one UI theme and the helpers every panel is built from: the
// editor design's §13 "Look and feel" (EDT-021), after the user's direction of
// 2026-10-06 for a sleek, consolidated, Unity/Unreal-style UI.
//
// THE ONLY FILE IN sandbox/ THAT SETS STYLE. Colours, style variables and
// fonts are set here and nowhere else; tests/test_sandbox_ui_conventions.cpp
// scans for it. A panel is built from:
//   - section(): a collapsing header, instead of a boxed group or separator;
//   - begin_properties() / property_row() / end_properties(): aligned
//     label/value rows, the label in a fixed left column;
//   - list_row(): a selectable row, the accent marking the selection;
//   - status_text(): a refusal or a state, in the warning colour.
// The panels dock (build_layout()); the HUD is the one floating window.
//
// Needs an ImGui context; not part of spade_tests (it draws). It is compiled
// only into the window (gl_target_sink.cpp).

#pragma once

#include <cstdarg>
#include <filesystem>
#include <initializer_list>
#include <string>
#include <string_view>
#include <system_error>

#include <imgui.h>
#include <imgui_internal.h>

namespace spade::sandbox::ui {

// ---- the scales ----
inline constexpr float kBase = 4.0f;          // every gap is a multiple of this
inline constexpr float kFontPixels = 15.0f;   // one size for body text and headers
inline constexpr float kLabelWidth = 30.0f * kBase;  // the property label column

// ---- the panels, by window name (DockBuilder docks them by name) ----
inline constexpr const char* kHierarchy = "Hierarchy";
inline constexpr const char* kInspector = "Inspector";
inline constexpr const char* kDrone = "Drone";

// ---- the fonts ----
inline constexpr std::string_view kFontRegular = "Inter-Regular.ttf";
inline constexpr std::string_view kFontSemiBold = "Inter-SemiBold.ttf";

struct Fonts {
    ImFont* body = nullptr;
    ImFont* heading = nullptr;
    std::string missing;  // what could not be loaded, for the announcement; empty when loaded
};

[[nodiscard]] inline Fonts& fonts() {
    static Fonts f;
    return f;
}

// Adds the UI font from `dir` to the current context's atlas. Call after
// CreateContext() and before the first frame. On failure records what is
// missing and leaves ImGui's built-in font, which the caller must announce
// (L6): never a silent fallback.
inline bool load_fonts(const std::filesystem::path& dir) {
    Fonts& f = fonts();
    const std::filesystem::path regular = dir / kFontRegular;
    const std::filesystem::path semibold = dir / kFontSemiBold;
    std::error_code ec;
    // Checked first: AddFontFromFileTTF() asserts on a missing file in a debug build.
    if (!std::filesystem::is_regular_file(regular, ec) || !std::filesystem::is_regular_file(semibold, ec)) {
        f.missing = "the UI font is missing from " + dir.string() + " (" + std::string(kFontRegular) + ", " +
                    std::string(kFontSemiBold) + "); rebuild spade_sandbox, which copies assets/fonts beside it";
        return false;
    }
    ImGuiIO& io = ImGui::GetIO();
    ImFontConfig config;
    config.OversampleH = 2;
    config.OversampleV = 1;
    f.body = io.Fonts->AddFontFromFileTTF(regular.string().c_str(), kFontPixels, &config);
    f.heading = io.Fonts->AddFontFromFileTTF(semibold.string().c_str(), kFontPixels, &config);
    if (f.body == nullptr || f.heading == nullptr) {
        f.missing = "the UI font in " + dir.string() + " could not be read; rebuild spade_sandbox";
        f.body = f.heading = nullptr;
        return false;
    }
    io.FontDefault = f.body;
    f.missing.clear();
    return true;
}

// ---- the palette: neutral greys, one accent, one warning ----
inline constexpr ImVec4 kAccent{0.29f, 0.56f, 0.89f, 1.00f};
inline constexpr ImVec4 kWarning{0.95f, 0.71f, 0.29f, 1.00f};

inline void apply_theme() {
    ImGuiStyle& s = ImGui::GetStyle();
    s = ImGuiStyle();
    ImGui::StyleColorsDark(&s);

    s.WindowPadding = ImVec2(2.0f * kBase, 2.0f * kBase);
    s.FramePadding = ImVec2(1.5f * kBase, kBase);
    s.ItemSpacing = ImVec2(2.0f * kBase, 1.5f * kBase);
    s.ItemInnerSpacing = ImVec2(1.5f * kBase, kBase);
    s.CellPadding = ImVec2(kBase, 0.75f * kBase);
    s.IndentSpacing = 3.0f * kBase;
    s.ScrollbarSize = 2.5f * kBase;
    s.GrabMinSize = 2.0f * kBase;

    s.WindowRounding = 0.0f;
    s.ChildRounding = 0.0f;
    s.FrameRounding = 3.0f;
    s.PopupRounding = 3.0f;
    s.ScrollbarRounding = 3.0f;
    s.GrabRounding = 3.0f;
    s.TabRounding = 3.0f;

    s.WindowBorderSize = 0.0f;
    s.ChildBorderSize = 0.0f;
    s.FrameBorderSize = 0.0f;
    s.PopupBorderSize = 1.0f;  // a popup floats over a panel and needs an edge
    s.TabBorderSize = 0.0f;
    s.TabBarBorderSize = 0.0f;
    s.TabBarOverlineSize = 0.0f;
    s.DockingSeparatorSize = 2.0f;

    s.WindowTitleAlign = ImVec2(0.0f, 0.5f);
    s.WindowMenuButtonPosition = ImGuiDir_None;

    ImVec4* c = s.Colors;
    const ImVec4 bg{0.125f, 0.130f, 0.140f, 1.00f};
    const ImVec4 bg_dark{0.100f, 0.104f, 0.112f, 1.00f};
    const ImVec4 frame{0.175f, 0.180f, 0.195f, 1.00f};
    const ImVec4 frame_hover{0.215f, 0.222f, 0.240f, 1.00f};
    const ImVec4 frame_active{0.245f, 0.253f, 0.273f, 1.00f};
    const ImVec4 line{0.200f, 0.206f, 0.222f, 1.00f};
    const ImVec4 accent_soft{kAccent.x, kAccent.y, kAccent.z, 0.35f};

    c[ImGuiCol_Text] = ImVec4(0.870f, 0.875f, 0.890f, 1.00f);
    c[ImGuiCol_TextDisabled] = ImVec4(0.520f, 0.530f, 0.560f, 1.00f);
    c[ImGuiCol_WindowBg] = bg;
    c[ImGuiCol_ChildBg] = ImVec4(0.0f, 0.0f, 0.0f, 0.0f);
    c[ImGuiCol_PopupBg] = bg_dark;
    c[ImGuiCol_Border] = line;
    c[ImGuiCol_BorderShadow] = ImVec4(0.0f, 0.0f, 0.0f, 0.0f);
    c[ImGuiCol_FrameBg] = frame;
    c[ImGuiCol_FrameBgHovered] = frame_hover;
    c[ImGuiCol_FrameBgActive] = frame_active;
    c[ImGuiCol_TitleBg] = bg_dark;
    c[ImGuiCol_TitleBgActive] = bg_dark;
    c[ImGuiCol_TitleBgCollapsed] = bg_dark;
    c[ImGuiCol_MenuBarBg] = bg_dark;
    c[ImGuiCol_ScrollbarBg] = ImVec4(0.0f, 0.0f, 0.0f, 0.0f);
    c[ImGuiCol_ScrollbarGrab] = frame_hover;
    c[ImGuiCol_ScrollbarGrabHovered] = frame_active;
    c[ImGuiCol_ScrollbarGrabActive] = kAccent;
    c[ImGuiCol_CheckMark] = kAccent;
    c[ImGuiCol_SliderGrab] = ImVec4(0.420f, 0.430f, 0.460f, 1.00f);
    c[ImGuiCol_SliderGrabActive] = kAccent;
    c[ImGuiCol_Button] = frame;
    c[ImGuiCol_ButtonHovered] = frame_hover;
    c[ImGuiCol_ButtonActive] = accent_soft;
    // Header* paints collapsing headers AND selected rows; the headers stay
    // neutral here and list_row() lends the accent to a selection.
    c[ImGuiCol_Header] = ImVec4(0.155f, 0.160f, 0.172f, 1.00f);
    c[ImGuiCol_HeaderHovered] = frame_hover;
    c[ImGuiCol_HeaderActive] = frame_active;
    c[ImGuiCol_Separator] = line;
    c[ImGuiCol_SeparatorHovered] = accent_soft;
    c[ImGuiCol_SeparatorActive] = kAccent;
    c[ImGuiCol_ResizeGrip] = ImVec4(0.0f, 0.0f, 0.0f, 0.0f);
    c[ImGuiCol_ResizeGripHovered] = accent_soft;
    c[ImGuiCol_ResizeGripActive] = kAccent;
    c[ImGuiCol_Tab] = bg_dark;
    c[ImGuiCol_TabHovered] = frame_hover;
    c[ImGuiCol_TabSelected] = bg;
    c[ImGuiCol_TabSelectedOverline] = kAccent;
    c[ImGuiCol_TabDimmed] = bg_dark;
    c[ImGuiCol_TabDimmedSelected] = bg;
    c[ImGuiCol_TabDimmedSelectedOverline] = ImVec4(0.0f, 0.0f, 0.0f, 0.0f);
    c[ImGuiCol_DockingPreview] = accent_soft;
    c[ImGuiCol_DockingEmptyBg] = ImVec4(0.0f, 0.0f, 0.0f, 0.0f);
    c[ImGuiCol_TableHeaderBg] = bg_dark;
    c[ImGuiCol_TableBorderStrong] = line;
    c[ImGuiCol_TableBorderLight] = line;
    c[ImGuiCol_TableRowBg] = ImVec4(0.0f, 0.0f, 0.0f, 0.0f);
    c[ImGuiCol_TableRowBgAlt] = ImVec4(0.0f, 0.0f, 0.0f, 0.0f);
    c[ImGuiCol_TextSelectedBg] = accent_soft;
    c[ImGuiCol_NavCursor] = kAccent;
}

// ---- building panels ----

// A collapsing section, its title in the semibold face. `open` sets only the
// first-seen state: common sections open, advanced ones folded (§13).
inline bool section(const char* label, bool open) {
    if (fonts().heading != nullptr) ImGui::PushFont(fonts().heading);
    const bool shown = ImGui::CollapsingHeader(label, open ? ImGuiTreeNodeFlags_DefaultOpen : ImGuiTreeNodeFlags_None);
    if (fonts().heading != nullptr) ImGui::PopFont();
    return shown;
}

// The selection's name at the top of the inspector, in the semibold face.
inline void title(const char* text) {
    if (fonts().heading != nullptr) ImGui::PushFont(fonts().heading);
    ImGui::TextUnformatted(text);
    if (fonts().heading != nullptr) ImGui::PopFont();
}

// Two columns, no borders: the label column fixed, the value column the rest.
// The HUD's short labels take kHudLabelWidth.
inline constexpr float kHudLabelWidth = 16.0f * kBase;

inline bool begin_properties(const char* id, float label_width = kLabelWidth) {
    if (!ImGui::BeginTable(id, 2, ImGuiTableFlags_SizingStretchProp)) return false;
    ImGui::TableSetupColumn("label", ImGuiTableColumnFlags_WidthFixed, label_width);
    ImGui::TableSetupColumn("value", ImGuiTableColumnFlags_WidthStretch);
    return true;
}

inline void end_properties() { ImGui::EndTable(); }

// Starts a row: the label on the left, then the next widget fills the value
// column. Give that widget a hidden label ("##wind").
inline void property_row(const char* label) {
    ImGui::TableNextRow();
    ImGui::TableSetColumnIndex(0);
    ImGui::AlignTextToFramePadding();
    ImGui::TextDisabled("%s", label);
    ImGui::TableSetColumnIndex(1);
    ImGui::SetNextItemWidth(-FLT_MIN);
}

// A read-only row: the label, then formatted text.
inline void value_row(const char* label, const char* format, ...) IM_FMTARGS(2);
inline void value_row(const char* label, const char* format, ...) {
    property_row(label);
    va_list args;
    va_start(args, format);
    ImGui::TextV(format, args);
    va_end(args);
}

// A list row; a selected row takes the accent.
inline bool list_row(const char* label, bool selected) {
    if (selected) {
        ImGui::PushStyleColor(ImGuiCol_Header, ImVec4(kAccent.x, kAccent.y, kAccent.z, 0.45f));
        ImGui::PushStyleColor(ImGuiCol_HeaderHovered, ImVec4(kAccent.x, kAccent.y, kAccent.z, 0.55f));
    }
    const bool clicked = ImGui::Selectable(label, selected);
    if (selected) ImGui::PopStyleColor(2);
    return clicked;
}

// A refusal or a state the user must see (INT-2), wrapped, in the warning colour.
inline void status_text(std::string_view text) {
    ImGui::PushStyleColor(ImGuiCol_Text, kWarning);
    ImGui::PushTextWrapPos(0.0f);
    ImGui::TextUnformatted(text.data(), text.data() + text.size());
    ImGui::PopTextWrapPos();
    ImGui::PopStyleColor();
}

// Vertical room between groups, instead of a separator.
inline void gap() { ImGui::Dummy(ImVec2(0.0f, kBase)); }

// The separator between items on one line: a middle dot, spelled in UTF-8
// bytes because the sources are not compiled as UTF-8.
inline constexpr const char* kDot = "  \xC2\xB7  ";

[[nodiscard]] inline std::string joined(std::initializer_list<std::string_view> items) {
    std::string out;
    for (const std::string_view item : items) {
        if (!out.empty()) out += kDot;
        out += item;
    }
    return out;
}

// ---- the docked layout ----

enum class Layout { none, builder, drone };

// The default layout for the scene on screen: the hierarchy on the left and
// the inspector on the right (the builder), or the drone's panel on the right.
// The centre is the viewport, which lets the scene show and take input through.
inline void build_layout(ImGuiID dockspace, Layout layout) {
    const ImGuiViewport* viewport = ImGui::GetMainViewport();
    ImGui::DockBuilderRemoveNode(dockspace);
    // Two enums (one private), so combined as ints: mixing enum types in a
    // bitwise operation is deprecated in C++20, and gcc refuses it.
    ImGuiDockNodeFlags flags = ImGuiDockNodeFlags_DockSpace;
    flags |= ImGuiDockNodeFlags_PassthruCentralNode;
    ImGui::DockBuilderAddNode(dockspace, flags);
    ImGui::DockBuilderSetNodeSize(dockspace, viewport->WorkSize);
    ImGuiID centre = dockspace;
    if (layout == Layout::builder) {
        const ImGuiID left = ImGui::DockBuilderSplitNode(centre, ImGuiDir_Left, 0.20f, nullptr, &centre);
        const ImGuiID right = ImGui::DockBuilderSplitNode(centre, ImGuiDir_Right, 0.30f, nullptr, &centre);
        ImGui::DockBuilderDockWindow(kHierarchy, left);
        ImGui::DockBuilderDockWindow(kInspector, right);
    } else if (layout == Layout::drone) {
        const ImGuiID right = ImGui::DockBuilderSplitNode(centre, ImGuiDir_Right, 0.30f, nullptr, &centre);
        ImGui::DockBuilderDockWindow(kDrone, right);
    }
    ImGui::DockBuilderFinish(dockspace);
}

// The viewport: the dockspace's central node, or the whole work area.
inline void viewport_rect(ImGuiID dockspace, ImVec2& pos, ImVec2& size) {
    if (const ImGuiDockNode* centre = ImGui::DockBuilderGetCentralNode(dockspace)) {
        pos = centre->Pos;
        size = centre->Size;
        return;
    }
    const ImGuiViewport* viewport = ImGui::GetMainViewport();
    pos = viewport->WorkPos;
    size = viewport->WorkSize;
}

// The panels' window flags: docked, no collapse arrow.
inline constexpr ImGuiWindowFlags kPanelFlags = ImGuiWindowFlags_NoCollapse;

// The HUD: the one floating window, translucent, out of the way.
inline constexpr ImGuiWindowFlags kHudFlags = ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_AlwaysAutoResize |
                                              ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoFocusOnAppearing |
                                              ImGuiWindowFlags_NoNav | ImGuiWindowFlags_NoDocking |
                                              ImGuiWindowFlags_NoMove;
inline constexpr float kHudAlpha = 0.72f;

}  // namespace spade::sandbox::ui
