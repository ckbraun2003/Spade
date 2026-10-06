// The editor's command line. Display-free (SL15b): main() hands it every
// argument after --edit, and it never opens a window.
//
//   spade_sandbox --edit <scene.yaml> [--apply <edits.txt>] [--save <scene.yaml>]
//                 [--save-world <world.yaml>] [--run <steps>]
//
// It opens the scene and its world, then applies the edits file. Each line of
// that file is one edit in the edit text (editor_edit_text.hpp); "#" comments
// and blank lines are skipped. Then:
// - --save-world writes the world there;
// - --save writes the scene there, after a dirty world, which goes where it
//   was opened unless --save-world moved it (editor_files.hpp);
// - --run steps the result, from the documents, with no window.
//
// Exit codes: 0 done (the last line says "done: applied N edits, ran S
// steps"); 1 an edit, a file, a save or the run was refused, with the edits
// file's name and line where one is at fault, and nothing saved after the
// failure; 2 bad arguments.
//
// docs/design/interface/plans/2026-10-05-editor-plan.md Task 8.

#pragma once

#include <charconv>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <optional>
#include <ostream>
#include <string>
#include <string_view>
#include <system_error>
#include <variant>
#include <vector>

#include "editor_edit_text.hpp"
#include "editor_files.hpp"
#include "editor_run.hpp"

namespace spade::sandbox::editor {

inline constexpr std::string_view kEditCliUsage =
    "usage: spade_sandbox --edit <scene.yaml> [--apply <edits.txt>] [--save <scene.yaml>] "
    "[--save-world <world.yaml>] [--run <steps>]";

struct EditCliArgs {
    std::filesystem::path scene;
    std::filesystem::path apply;       // empty: no edits
    std::filesystem::path save;        // empty: the scene is not saved
    std::filesystem::path save_world;  // empty: a dirty world is saved where it was opened, with the scene
    uint32_t run = 0;                  // steps; 0: no run
};

[[nodiscard]] inline Result<EditCliArgs> parse_edit_cli(const std::vector<std::string>& args) {
    const auto bad = [](const std::string& why) {
        return std::unexpected(Error{Code::invalid_argument, why + "; " + std::string(kEditCliUsage)});
    };
    if (args.empty() || args[0].starts_with("--")) return bad("--edit needs a scene file first");
    EditCliArgs a;
    a.scene = args[0];
    for (std::size_t i = 1; i < args.size(); ++i) {
        const std::string& flag = args[i];
        if (flag != "--apply" && flag != "--save" && flag != "--save-world" && flag != "--run") {
            return bad("unknown argument '" + flag + "'");
        }
        if (i + 1 >= args.size()) return bad(flag + " needs a value");
        const std::string& value = args[++i];
        if (flag == "--apply") a.apply = value;
        if (flag == "--save") a.save = value;
        if (flag == "--save-world") a.save_world = value;
        if (flag == "--run") {
            const char* e = value.data() + value.size();
            const std::from_chars_result r = std::from_chars(value.data(), e, a.run);
            if (r.ec != std::errc() || r.ptr != e) return bad("--run takes a whole number of steps, not '" + value + "'");
        }
    }
    return a;
}

[[nodiscard]] inline int run_edit_cli(const EditCliArgs& a, std::ostream& out, std::ostream& err) {
    const auto fail = [&](const std::string& why) {
        err << "spade_sandbox --edit: " << why << "\n";
        return 1;
    };
    Result<OpenedScene> opened = open_scene(a.scene);
    if (!opened) return fail(opened.error().context);
    const BuiltinRecords records = records_for(a.scene);  // the scene as opened, wherever it is saved
    out << "opened " << a.scene.generic_string() << " with " << opened->world.path.generic_string() << "\n";

    std::size_t applied = 0;
    if (!a.apply.empty()) {
        std::ifstream in(a.apply, std::ios::binary);
        if (!in) return fail("cannot read the edits file " + a.apply.string() + "; check the path");
        std::string line;
        std::size_t number = 0;
        while (std::getline(in, line)) {
            ++number;
            if (!line.empty() && line.back() == '\r') line.pop_back();
            const std::size_t first = line.find_first_not_of(" \t");
            if (first == std::string::npos || line[first] == '#') continue;
            const std::string where = a.apply.filename().string() + ":" + std::to_string(number) + ": ";
            const Result<AnyEdit> edit = parse_any_edit(line);
            if (!edit) return fail(where + edit.error().context);
            const Result<void> done = std::holds_alternative<SceneEdit>(*edit)
                                          ? apply(opened->scene, std::get<SceneEdit>(*edit))
                                          : apply(opened->world, std::get<WorldEdit>(*edit));
            if (!done) return fail(where + done.error().context);
            ++applied;
        }
    }

    if (!a.save_world.empty()) {
        if (const Result<void> s = save_world_as(opened->world, a.save_world); !s) return fail(s.error().context);
        out << "saved " << a.save_world.generic_string() << "\n";
    }
    if (!a.save.empty()) {
        const bool world_dirty = opened->world.dirty();
        if (const Result<void> s = save_scene_as(opened->scene, opened->world, a.save); !s) {
            return fail(s.error().context);
        }
        if (world_dirty) out << "saved " << opened->world.path.generic_string() << "\n";
        out << "saved " << a.save.generic_string() << "\n";
    }

    if (a.run > 0) {
        Result<EditorRun> run = EditorRun::start(opened->scene.desc(), opened->world.desc(), records);
        if (!run) return fail(run.error().context);
        if (const Result<void> s = run->step(a.run); !s) return fail(s.error().context);
        for (const scene::SceneVehicle& v : opened->scene.desc().vehicles) {
            if (const std::optional<FrameState> st = run->state_of(v.name)) {
                out << "vehicle " << v.name << " at " << st->pos.x << " " << st->pos.y << " " << st->pos.z << "\n";
            }
        }
    }
    out << "done: applied " << applied << " edits, ran " << a.run << " steps\n";
    return 0;
}

// The whole command line: every argument after --edit. Bad arguments exit 2.
[[nodiscard]] inline int edit_cli(const std::vector<std::string>& args, std::ostream& out, std::ostream& err) {
    const Result<EditCliArgs> a = parse_edit_cli(args);
    if (!a) {
        err << "spade_sandbox --edit: " << a.error().context << "\n";
        return 2;
    }
    return run_edit_cli(*a, out, err);
}

}  // namespace spade::sandbox::editor
