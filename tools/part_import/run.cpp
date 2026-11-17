#include "run.hpp"

#include <algorithm>
#include <cctype>
#include <format>
#include <fstream>
#include <iterator>
#include <optional>
#include <ostream>
#include <span>

#include <nlohmann/json.hpp>

#include "options.hpp"
#include "report.hpp"
#include "store.hpp"
#include "transform.hpp"
#include "world/map_cache.hpp"
#include "world/mesh_import.hpp"
#include "world/sha256.hpp"

namespace spade::partimport {

namespace fs = std::filesystem;

namespace {

[[nodiscard]] std::string lower(std::string s) {
    for (char& c : s) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    return s;
}

[[nodiscard]] std::optional<std::string> format_of(const fs::path& path) {
    const std::string ext = lower(path.extension().string());
    if (ext == ".stl") return "stl";
    if (ext == ".obj") return "obj";
    if (ext == ".glb") return "glb";
    if (ext == ".gltf") return "gltf";
    if (ext == ".3mf") return "3mf";
    if (ext == ".step" || ext == ".stp") return "step";
    return std::nullopt;
}

[[nodiscard]] const char* unit_text(MeshUnit u) {
    switch (u) {
        case MeshUnit::metre: return "m";
        case MeshUnit::millimetre: return "mm";
        case MeshUnit::centimetre: return "cm";
        case MeshUnit::micrometre: return "um";
        case MeshUnit::inch: return "in";
        case MeshUnit::foot: return "ft";
        case MeshUnit::unspecified: break;
    }
    return "";
}

[[nodiscard]] std::optional<MeshUnit> parse_unit(const std::string& text) {
    for (const MeshUnit u : {MeshUnit::metre, MeshUnit::millimetre, MeshUnit::centimetre, MeshUnit::micrometre, MeshUnit::inch, MeshUnit::foot}) {
        if (text == unit_text(u)) return u;
    }
    return std::nullopt;
}

void stage(std::ostream& out, const char* name, const char* state) {
    out << JsonObject{}.str("event", "stage").str("stage", name).str("state", state).line() << "\n";
}

// The result line, last: the fixed keys, then the mode's payload. Returns the exit code.
int result(std::ostream& out, const std::string& mode, const Problem* problem, const JsonObject& payload) {
    const std::string code = problem ? problem->code : "ok";
    const CodeInfo& info = code_info(code);
    JsonObject o;
    o.str("event", "result").str("mode", mode).boolean("ok", problem == nullptr).integer("exit", info.exit).str("code", info.code);
    o.str("message", problem ? problem->message : "").str("advice", problem ? info.advice : "");
    o.merge(payload);
    out << o.line() << "\n";
    return info.exit;
}

[[nodiscard]] Problem reader_problem(const Error& e) {
    switch (e.code) {
        case Code::unavailable: return {"format_unavailable", e.context};
        case Code::io_error: return {"file_unreadable", e.context};
        case Code::capacity_exceeded: return {"too_large", e.context};
        case Code::internal: return {"internal", e.context};
        default: return {"file_invalid", e.context};  // until make_mesh_diagnosed names the gate's kinds
    }
}

// A .gltf that names a file (a uri not starting "data:") in its buffers or images: not self-contained, so not storable as one file.
[[nodiscard]] bool gltf_has_external_uri(std::string_view text) {
    const nlohmann::json j = nlohmann::json::parse(text, nullptr, false);
    if (j.is_discarded() || !j.is_object()) return false;
    for (const char* key : {"buffers", "images"}) {
        const auto it = j.find(key);
        if (it == j.end() || !it->is_array()) continue;
        for (const nlohmann::json& e : *it) {
            const auto uri = e.is_object() ? e.find("uri") : e.end();
            if (e.is_object() && uri != e.end() && uri->is_string() && uri->get<std::string>().rfind("data:", 0) != 0) return true;
        }
    }
    return false;
}

[[nodiscard]] Expected<std::vector<MeshObject>> read_objects(const std::string& format, const fs::path& path, const std::string& bytes,
                                                             MeshUnit unit) {
    Result<std::vector<MeshObject>> r = std::unexpected(Error{Code::internal, "no reader"});
    if (format == "stl") r = read_stl(path, unit);
    if (format == "obj") r = read_obj(path, unit);
    if (format == "glb" || format == "gltf") r = read_gltf_bytes(std::as_bytes(std::span<const char>(bytes)), fs::path());
    if (format == "3mf") r = read_3mf(path);
    if (!r) return std::unexpected(reader_problem(r.error()));
    return std::move(*r);
}

[[nodiscard]] std::string hex16(std::uint64_t v) { return std::format("{:016x}", v); }

[[nodiscard]] std::string array3(const glm::vec3& v) {
    return "[" + json_number(v.x) + "," + json_number(v.y) + "," + json_number(v.z) + "]";
}

}  // namespace

int run_part_import(const std::vector<std::string>& args, const RunEnv& env, std::ostream& out, std::ostream& err) {
    const Expected<Options> parsed = parse_options(args);
    if (!parsed) {
        err << usage_text();
        return result(out, "none", &parsed.error(), {});
    }
    const Options& opts = *parsed;
    if (opts.help) {
        err << usage_text();
        return result(out, "help", nullptr, {});
    }
    const auto cancelled = [&] { return env.cancelled && env.cancelled(); };

    if (opts.mode == Mode::probe) {
        // The process layer comes with the STEP route; until then the probe reports what it can see without running anything.
        JsonObject step;
        step.boolean("available", false).null("python").str("reason", "the STEP converter is not looked for yet");
        JsonObject payload;
        payload.raw("step", step.line());
        return result(out, "probe", nullptr, payload);
    }

    const bool importing = opts.mode == Mode::import;
    const std::string mode_name = importing ? "import" : "inspect";
    const auto fail = [&](Problem p, const JsonObject& extra = {}) { return result(out, mode_name, &p, extra); };

    // ---- the options, before any file is read ---------------------------------------------------------------------------------
    const fs::path path(opts.file);
    const std::optional<std::string> format = format_of(path);
    if (!format) {
        return fail({"format_unknown", "'" + opts.file + "' is not a STEP, 3MF, glTF, glb, STL or OBJ file (by its extension)"});
    }
    const bool declares_unit = *format != "stl" && *format != "obj";
    std::optional<MeshUnit> unit;
    if (opts.unit) {
        unit = parse_unit(*opts.unit);
        if (!unit) return fail({"usage", "--unit '" + *opts.unit + "' is not one of m, mm, cm, um, in, ft"});
        if (declares_unit) return fail({"unit_not_allowed", "--unit is for STL and OBJ; a ." + *format + " file declares its own unit"});
    }
    if (importing && !declares_unit && !unit) {
        return fail({"unit_required", "a ." + *format + " file carries no unit; give --unit (m, mm, cm, um, in or ft)"});
    }
    Axes axes;
    if (opts.axes) {
        const auto a = parse_axes(*opts.axes);
        if (!a) return fail(a.error());
        axes = *a;
    }
    double scale = 1.0;
    if (opts.scale) {
        const auto s = parse_scale(*opts.scale);
        if (!s) return fail(s.error());
        scale = *s;
    }
    if (*format == "step") {
        if (importing && !opts.deflection) {
            return fail({"deflection_required", "a STEP import needs --deflection with a unit, for example 0.1mm"});
        }
        return fail({"step_unavailable", "no STEP converter was found (OpenCASCADE through step_convert.py)"});
    }
    fs::path store;
    if (importing) {
        if (opts.store) {
            store = *opts.store;
        } else if (const char* e = env.getenv ? env.getenv("SPADE_PART_STORE") : nullptr; e != nullptr && *e != '\0') {
            store = e;
        } else {
            return fail({"store_required", "no part store: give --store DIR, or set SPADE_PART_STORE"});
        }
        if (inside_git_work_tree(store)) {
            return fail({"store_in_git_tree", "the store " + store.string() + " is inside a git work tree"});
        }
    }

    // ---- read ---------------------------------------------------------------------------------------------------------------
    if (cancelled()) return fail({"cancelled", "cancelled before the file was read"});
    stage(out, "read", "start");
    std::string bytes;
    {
        std::ifstream in(path, std::ios::binary);
        if (!in) return fail({"file_unreadable", "cannot read '" + opts.file + "'"});
        bytes.assign(std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>());
    }
    const std::string sha = sha256_hex(bytes);
    stage(out, "read", "done");
    JsonObject base;
    base.str("file", opts.file).integer("bytes", static_cast<std::int64_t>(bytes.size())).str("sha256", sha).str("format", *format);
    if (cancelled()) return fail({"cancelled", "cancelled after the file was read"}, base);

    KeyInputs key_in;
    key_in.converter = kMeshConverterVersion;
    key_in.unit = (declares_unit || !opts.unit) ? std::string() : *opts.unit;
    key_in.axes = axes;
    key_in.scale = scale;
    key_in.object = opts.object.value_or(std::string());
    const std::string source_name = path.filename().string();
    const auto import_payload = [&](const EntryRecord& rec, bool cached) {
        JsonObject p;
        p.boolean("cached", cached).str("sha256", rec.sha256).str("key", rec.key).str("converter", rec.converter);
        p.str("store", store.string()).str("entry", rec.dir.string()).str("source", (store / rec.sha256 / rec.source_name).string());
        std::string objects;
        for (const SidecarRecord& s : rec.sidecars) {
            if (!objects.empty()) objects.push_back(',');
            objects += JsonObject{}
                           .str("name", s.object)
                           .str("sidecar", (rec.dir / s.file).string())
                           .str("content_hash", s.content_hash)
                           .integer("triangles", s.triangles)
                           .num("volume_m3", s.volume_m3)
                           .line();
        }
        p.raw("objects", "[" + objects + "]");
        return p;
    };
    if (importing) {
        if (const auto hit = find_entry(store, sha, cache_key(key_in))) return result(out, "import", nullptr, import_payload(*hit, true));
    }

    // ---- check: read the objects, pick, put them in the part frame ------------------------------------------------------------
    stage(out, "check", "start");
    JsonObject refused = base;
    refused.str("gate", "refused").raw("objects", "[]");
    if (*format == "gltf" && gltf_has_external_uri(bytes)) {
        return fail({"external_buffers", "the glTF refers to files beside it, so it is not one self-contained file"}, refused);
    }
    Expected<std::vector<MeshObject>> read = read_objects(*format, path, bytes, unit.value_or(MeshUnit::metre));
    if (!read) return fail(read.error(), refused);
    std::vector<MeshObject> objects = std::move(*read);
    if (opts.object) {
        std::vector<MeshObject> picked;
        for (const MeshObject& o : objects) {
            if (o.name == *opts.object) picked.push_back(o);
        }
        if (picked.empty()) {
            std::string names;
            for (const MeshObject& o : objects) names += (names.empty() ? "'" : ", '") + o.name + "'";
            return fail({"object_not_found", "the file has no object named '" + *opts.object + "'; it has " + names}, base);
        }
        objects = std::move(picked);
    }
    glm::vec3 file_lo(1e30f), file_hi(-1e30f);  // the objects' box in the file's own numbers, for the unit guess
    for (const MeshObject& o : objects) {
        for (const glm::vec3& p : o.mesh->positions()) {
            file_lo = glm::min(file_lo, p);
            file_hi = glm::max(file_hi, p);
        }
    }
    for (MeshObject& o : objects) {
        auto moved = transform_mesh(o.mesh, axes, scale);
        if (!moved) return fail(moved.error(), refused);
        o.mesh = *moved;
    }
    stage(out, "check", "done");
    if (cancelled()) return fail({"cancelled", "cancelled after the check"}, base);

    // ---- inspect -------------------------------------------------------------------------------------------------------------
    if (!importing) {
        std::optional<std::string> declared;
        if (*format == "glb" || *format == "gltf") declared = "m";
        if (*format == "3mf" && !objects.empty()) declared = unit_text(objects.front().unit);
        const std::optional<std::string> used = declares_unit ? declared : (unit ? std::optional<std::string>(unit_text(*unit)) : std::nullopt);
        JsonObject unit_json;
        if (declared) unit_json.str("declared", *declared); else unit_json.null("declared");
        if (used) unit_json.str("used", *used); else unit_json.null("used");
        const auto guess = used ? std::nullopt : guess_unit(glm::dvec3(file_hi - file_lo));
        if (guess) {
            unit_json.raw("guess", JsonObject{}.str("unit", guess->unit).str("basis", guess->basis).line());
        } else {
            unit_json.null("guess");
        }
        std::string list;
        for (const MeshObject& o : objects) {
            glm::vec3 lo(1e30f), hi(-1e30f);
            for (const glm::vec3& p : o.mesh->positions()) {
                lo = glm::min(lo, p);
                hi = glm::max(hi, p);
            }
            if (!list.empty()) list.push_back(',');
            list += JsonObject{}
                        .str("name", o.name)
                        .boolean("ok", true)
                        .integer("triangles", o.mesh->triangle_count())
                        .integer("vertices", static_cast<std::int64_t>(o.mesh->positions().size()))
                        .integer("shells", o.mesh->shell_count())
                        .raw("bbox", JsonObject{}.raw("min", array3(lo)).raw("max", array3(hi)).line())
                        .num("volume", o.mesh->mass_properties().volume)
                        .str("content_hash", hex16(o.mesh->content_hash()))
                        .line();
        }
        JsonObject payload = base;
        payload.raw("unit", unit_json.line())
            .str("measures_in", used ? "m" : "file")
            .str("frame", (axes.identity() && scale == 1.0) ? "file" : "part")
            .str("gate", "passed")
            .str("depth", "full")
            .raw("objects", "[" + list + "]");
        return result(out, "inspect", nullptr, payload);
    }

    // ---- import --------------------------------------------------------------------------------------------------------------
    stage(out, "write", "start");
    (void)remove_stale_temps(store, env.stale_after);
    std::vector<StoredObject> stored;
    for (const MeshObject& o : objects) {
        stored.push_back({o.name, o.mesh, MeshProvenance{source_name + ": " + o.name, o.unit, 0.0}});
    }
    const Expected<EntryRecord> rec = write_entry(store, bytes, source_name, key_in, stored, env.token);
    if (!rec) return fail(rec.error(), base);
    stage(out, "write", "done");
    return result(out, "import", nullptr, import_payload(*rec, false));
}

}  // namespace spade::partimport
