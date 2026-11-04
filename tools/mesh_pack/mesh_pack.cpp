// tools/mesh_pack/mesh_pack.cpp -- spade_mesh_pack's run(). See mesh_pack.hpp
// for the command line, the provenance, the file names and the exit codes.
//
// The order: the options (a usage error reads nothing); the input's format,
// --unit and --tolerance; the read, all objects or a refusal; the file names;
// then the directory, and each sidecar in the reader's order.
#include "mesh_pack.hpp"

#include <array>
#include <charconv>
#include <cmath>
#include <exception>
#include <expected>
#include <new>
#include <optional>
#include <set>
#include <string>
#include <string_view>
#include <system_error>
#include <vector>

#include "core/error.hpp"
#include "core/path_text.hpp"
#include "world/mesh.hpp"
#include "world/mesh_file.hpp"
#include "world/mesh_import.hpp"

namespace spade::mesh_pack {
namespace {

namespace fs = std::filesystem;

constexpr std::string_view kUsage =
    "usage: spade_mesh_pack IN --out DIR [--unit U] [--tolerance LEN] [--source TEXT]\n"
    "  IN   a .gltf, .glb, .stl, .obj or .3mf file\n"
    "  U    m, mm, cm, um, in or ft: the unit an STL or OBJ is in (they carry none)\n"
    "  LEN  a number and a unit, such as 0.1mm: the converter's tolerance, recorded\n"
    "  TEXT the source named in each sidecar's provenance (default: IN's file name)\n";

constexpr std::string_view kUnits = "m, mm, cm, um, in or ft";

[[nodiscard]] Outcome usage(const std::string& why) {
    return Outcome{2, {}, "spade_mesh_pack: " + why + "\n" + std::string(kUsage)};
}

[[nodiscard]] Outcome refused(int code, const std::string& why) {
    return Outcome{code, {}, "spade_mesh_pack: " + why + "\n"};
}

[[nodiscard]] int exit_for(Code code) noexcept {
    if (code == Code::unavailable) return 3;
    if (code == Code::io_error || code == Code::internal) return 1;
    return 4;
}

[[nodiscard]] char ascii_lower(char c) noexcept {
    return c >= 'A' && c <= 'Z' ? static_cast<char>(c - 'A' + 'a') : c;
}

[[nodiscard]] std::string lower(std::string_view s) {
    std::string out(s);
    for (char& c : out) c = ascii_lower(c);
    return out;
}

[[nodiscard]] std::optional<MeshUnit> unit_named(std::string_view u) noexcept {
    if (u == "m") return MeshUnit::metre;
    if (u == "mm") return MeshUnit::millimetre;
    if (u == "cm") return MeshUnit::centimetre;
    if (u == "um") return MeshUnit::micrometre;
    if (u == "in") return MeshUnit::inch;
    if (u == "ft") return MeshUnit::foot;
    return std::nullopt;
}

enum class Format { gltf, stl, obj, threemf };

[[nodiscard]] std::optional<Format> format_of(const std::string& extension) noexcept {
    if (extension == ".gltf" || extension == ".glb") return Format::gltf;
    if (extension == ".stl") return Format::stl;
    if (extension == ".obj") return Format::obj;
    if (extension == ".3mf") return Format::threemf;
    return std::nullopt;
}

// A length with its unit ("0.1mm"), in metres, or the reason it is not one.
[[nodiscard]] std::expected<double, std::string> length_in_metres(std::string_view text) {
    std::size_t split = text.size();
    while (split > 0 && ((text[split - 1] >= 'a' && text[split - 1] <= 'z'))) --split;
    const std::string quoted = "'" + std::string(text) + "'";
    if (split == text.size()) return std::unexpected(quoted + " has no unit (" + std::string(kUnits) + ")");
    const std::optional<MeshUnit> unit = unit_named(text.substr(split));
    if (!unit) {
        return std::unexpected(quoted + ": '" + std::string(text.substr(split)) + "' is not " + std::string(kUnits));
    }
    double value = 0.0;
    const char* const first = text.data();
    const char* const last = text.data() + split;
    const auto [ptr, ec] = std::from_chars(first, last, value);
    if (split == 0 || ec != std::errc{} || ptr != last) return std::unexpected(quoted + " is not a number and a unit");
    if (!std::isfinite(value) || value < 0.0) return std::unexpected(quoted + " is not finite and >= 0");
    return value * mesh_unit_scale(*unit);
}

[[nodiscard]] fs::path from_utf8(const std::string& s) {
    return fs::path(std::u8string(s.begin(), s.end()));
}

[[nodiscard]] bool device_name(std::string_view stem) {
    const std::string s = lower(stem);
    for (const std::string_view d : {"con", "prn", "aux", "nul"}) {
        if (s == d) return true;
    }
    return s.size() == 4 && (s.starts_with("com") || s.starts_with("lpt")) && s[3] >= '1' && s[3] <= '9';
}

// An object's name made a safe file name (mesh_pack.hpp, FILE NAMES).
[[nodiscard]] std::string safe_name(std::string_view name) {
    std::string s;
    for (const char c : name) {
        const auto u = static_cast<unsigned char>(c);
        const bool bad = u < 0x20 || u == 0x7F || std::string_view("<>:\"/\\|?*").find(c) != std::string_view::npos;
        s += bad ? '_' : c;
    }
    while (!s.empty() && (s.back() == '.' || s.back() == ' ')) s.pop_back();
    if (s.empty()) s = "object";
    if (device_name(std::string_view(s).substr(0, s.find('.')))) s.insert(0, "_");
    return s;
}

[[nodiscard]] Result<std::vector<MeshObject>> read(Format format, const fs::path& in, MeshUnit unit) {
    switch (format) {
        case Format::gltf:
            return read_gltf(in);
        case Format::stl:
            return read_stl(in, unit);
        case Format::obj:
            return read_obj(in, unit);
        case Format::threemf:
            return read_3mf(in);
    }
    return std::unexpected(Error{Code::internal, "spade_mesh_pack: no reader for the format"});
}

[[nodiscard]] Outcome pack(std::span<const fs::path> args) {
    std::optional<fs::path> in;
    std::optional<fs::path> out;
    std::optional<std::string> unit_text;
    std::optional<std::string> tolerance_text;
    std::optional<std::string> source;
    for (std::size_t i = 0; i < args.size(); ++i) {
        const std::string arg = path_text(args[i]);
        if (!arg.starts_with("--")) {
            if (in) return usage("one input only; '" + arg + "' is a second");
            in = args[i];
            continue;
        }
        const bool known = arg == "--out" || arg == "--unit" || arg == "--tolerance" || arg == "--source";
        if (!known) return usage("unknown option '" + arg + "'");
        if (i + 1 >= args.size()) return usage(arg + " needs a value");
        const fs::path& value = args[++i];
        const auto set = [&](auto& slot, auto v) -> bool {
            if (slot) return false;
            slot = std::move(v);
            return true;
        };
        const bool fresh = arg == "--out"         ? set(out, value)
                           : arg == "--unit"      ? set(unit_text, path_text(value))
                           : arg == "--tolerance" ? set(tolerance_text, path_text(value))
                                                  : set(source, path_text(value));
        if (!fresh) return usage(arg + " is given twice");
    }
    if (!in) return usage("no input file");
    if (!out) return usage("--out DIR is required");

    const std::string file = path_text(in->filename());
    const std::string extension = lower(path_text(in->extension()));
    const std::optional<Format> format = format_of(extension);
    if (!format) return usage("'" + extension + "' is not .gltf, .glb, .stl, .obj or .3mf (" + file + ")");
    MeshUnit unit = MeshUnit::unspecified;
    if (unit_text) {
        const std::optional<MeshUnit> named = unit_named(*unit_text);
        if (!named) return usage("--unit '" + *unit_text + "' is not " + std::string(kUnits));
        if (*format == Format::gltf || *format == Format::threemf) {
            return usage(file + " declares its own unit; --unit is for STL and OBJ");
        }
        unit = *named;
    }
    double tolerance = 0.0;
    if (tolerance_text) {
        const auto metres = length_in_metres(*tolerance_text);
        if (!metres) return usage("--tolerance " + metres.error());
        tolerance = *metres;
    }

    const Result<std::vector<MeshObject>> objects = read(*format, *in, unit);
    if (!objects) return refused(exit_for(objects.error().code), objects.error().context);

    std::vector<std::string> names;
    std::set<std::string> taken;
    for (const MeshObject& o : *objects) {
        const std::string base = safe_name(o.name);
        std::string name = base;
        for (int k = 2; taken.contains(lower(name)); ++k) name = base + "-" + std::to_string(k);
        taken.insert(lower(name));
        names.push_back(name);
    }

    std::error_code ec;
    fs::create_directories(*out, ec);
    if (ec || !fs::is_directory(*out)) {
        return refused(1, "cannot make the output directory '" + path_text(*out) + "'" +
                              (ec ? ": " + ec.message() : std::string(": a file is there")));
    }
    Outcome done;
    const std::string named_source = source ? *source : file;
    for (std::size_t i = 0; i < objects->size(); ++i) {
        const MeshObject& o = (*objects)[i];
        const fs::path target = *out / from_utf8(names[i] + ".spmesh");
        const MeshProvenance provenance{named_source + ": " + o.name, o.unit, tolerance};
        if (const Result<void> saved = save_mesh_file(target, *o.mesh, provenance); !saved) {
            return refused(exit_for(saved.error().code), saved.error().context + " (" + std::to_string(i) + " of " +
                                                             std::to_string(objects->size()) + " written)");
        }
        done.out += o.name + " -> " + path_text(target) + "\n";
    }
    done.out += "wrote " + std::to_string(objects->size()) + " mesh" + (objects->size() == 1 ? "" : "es") + " to " +
                path_text(*out) + "\n";
    return done;
}

}  // namespace

Outcome run(std::span<const std::filesystem::path> args) {
    try {
        return pack(args);
    } catch (const std::bad_alloc&) {
        return refused(1, "out of memory");
    } catch (const std::exception& e) {
        return refused(1, std::string("unexpected failure: ") + e.what());
    }
}

}  // namespace spade::mesh_pack
