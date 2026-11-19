#include "run.hpp"

#include <algorithm>
#include <cctype>
#include <format>
#include <fstream>
#include <iterator>
#include <map>
#include <optional>
#include <ostream>
#include <span>

#include <nlohmann/json.hpp>

#include "options.hpp"
#include "report.hpp"
#include "step.hpp"
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

// What the tool says about a file it does not read: the format's name where it is known, and what to convert to.
[[nodiscard]] std::string unsupported_format_message(const fs::path& path) {
    static const std::pair<const char*, const char*> kKnown[] = {
        {".dae", "COLLADA"},   {".fbx", "FBX"},      {".blend", "a Blender file"}, {".ply", "PLY"},     {".3ds", "3ds (3D Studio)"},
        {".iges", "IGES"},     {".igs", "IGES"},     {".skp", "a SketchUp file"},  {".usd", "USD"},     {".usda", "USD"},
        {".usdc", "USD"},      {".usdz", "USD"},     {".x3d", "X3D"},              {".wrl", "VRML"},    {".dxf", "DXF"},
        {".amf", "AMF"},       {".off", "OFF"},      {".x", "DirectX .x"},         {".max", "a 3ds Max file"}, {".ma", "a Maya file"},
        {".mb", "a Maya file"}, {".c4d", "a Cinema 4D file"}, {".sldprt", "a SolidWorks part"}, {".ipt", "an Inventor part"},
        {".prt", "a CAD part (NX, Creo)"}, {".3dm", "Rhino 3dm"}, {".dwg", "DWG"},
    };
    const std::string ext = lower(path.extension().string());
    const std::string convert = "convert it to glb, OBJ, STL, 3MF or STEP";
    const std::string file = "'" + path.string() + "'";
    for (const auto& [e, name] : kKnown) {
        if (ext == e) return file + " is " + name + " (" + ext + "), which this tool does not read; " + convert;
    }
    if (ext.empty()) {
        return file + " has no extension, so its format is unknown; this tool reads STEP, 3MF, glTF, glb, STL and OBJ (by extension); " + convert;
    }
    return file + " has the extension '" + ext + "', which is not a format this tool reads (STEP, 3MF, glTF, glb, STL and OBJ, by extension); " +
           convert;
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

// One object the gate refused: make_mesh's message and the kind's name (make_mesh_diagnosed's refusal_name(), the stable code).
struct GateRefusal {
    std::string message;
    std::string code;
};

// The objects of a file in the order the reader builds them. A refused object (make_mesh's refusal) has a null mesh and its refusal in
// `refusal` (parallel to `objects`); everything else about the file is one problem.
struct Candidates {
    std::vector<MeshObject> objects;
    std::vector<std::optional<GateRefusal>> refusal;
    std::vector<std::optional<SurfaceColour>> colour;  // parallel to `objects`: what the reader found (a STEP's come from the converter)
};

[[nodiscard]] Expected<Candidates> to_candidates(Result<std::vector<MeshObjectOutcome>> r) {
    if (!r) return std::unexpected(reader_problem(r.error()));
    Candidates c;
    for (MeshObjectOutcome& o : *r) {
        c.colour.push_back(o.colour);
        if (o.mesh) {
            c.objects.push_back(MeshObject{o.name, std::move(*o.mesh), o.unit});
            c.refusal.emplace_back(std::nullopt);
        } else {
            c.objects.push_back(MeshObject{o.name, nullptr, o.unit});
            c.refusal.emplace_back(GateRefusal{o.mesh.error().context, o.kind ? refusal_name(*o.kind) : "mesh_refused"});
        }
    }
    return c;
}

[[nodiscard]] Expected<Candidates> read_objects(const std::string& format, const fs::path& path, const std::string& bytes, MeshUnit unit) {
    Result<std::vector<MeshObjectOutcome>> r = std::unexpected(Error{Code::internal, "no reader"});
    if (format == "stl") r = read_stl_objects(path, unit);
    if (format == "obj") r = read_obj_objects(path, unit);
    if (format == "glb" || format == "gltf") r = read_gltf_bytes_objects(std::as_bytes(std::span<const char>(bytes)), fs::path());
    if (format == "3mf") r = read_3mf_objects(path);
    return to_candidates(std::move(r));
}

// A part the converter left out because it holds no solid (a surface or a shell): named, with a code, never a gate refusal.
[[nodiscard]] std::string skipped_object(const std::string& name) {
    return JsonObject{}
        .str("name", name)
        .boolean("ok", false)
        .str("code", "no_solid")
        .str("message", "'" + name + "' holds no solid (a surface, shell or wire), so it is left out; the rest of the file is read")
        .line();
}

// The flagged cross-check as a problem: the converter's mesh volume and the B-rep's differ by more than 1% for these objects.
[[nodiscard]] Problem flagged_problem_for(const std::vector<std::string>& flagged_names) {
    std::string names;
    for (const std::string& n : flagged_names) names += (names.empty() ? "" : ", ") + n;
    return Problem{"cross_check_flagged", "the mesh volume differs from the B-rep's by more than 1% for " + names +
                                              "; use a smaller --deflection or --angle"};
}

[[nodiscard]] std::string hex16(std::uint64_t v) { return std::format("{:016x}", v); }

[[nodiscard]] std::string array3(const glm::vec3& v) {
    return "[" + json_number(v.x) + "," + json_number(v.y) + "," + json_number(v.z) + "]";
}

// The converter's boxes are doubles and stay doubles (a float would put 1e-9 of noise on a box in metres).
[[nodiscard]] std::string array3(const glm::dvec3& v) {
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
    if (opts.help) {  // a request, not a usage problem: plain text on standard output, no JSON line, nothing on standard error
        out << usage_text();
        return 0;
    }
    const auto cancelled = [&] { return env.cancelled && env.cancelled(); };

    if (opts.mode == Mode::probe) {
        const ToolSearch found = find_step_tools(opts, env);
        std::string searched = "[";
        for (const std::string& where : found.searched) searched += (searched.size() > 1 ? "," : "") + json_quote(where);
        searched += "]";
        const auto unavailable = [&](const std::string& why, const std::string& python) {
            JsonObject step;
            step.boolean("available", false);
            if (python.empty()) step.null("python"); else step.str("python", python);
            step.raw("searched", searched).str("reason", why).str("install", step_install_text());
            JsonObject payload;
            payload.raw("step", step.line());
            const Problem p{"step_unavailable", why};
            return result(out, "probe", &p, payload);
        };
        if (!found.tools) return unavailable("no python with step_convert.py was found", "");
        const ConverterOutcome c = run_converter(*found.tools, {"--probe"}, env);
        if (c.killed) {
            const Problem p{"cancelled", "cancelled while the converter ran"};
            return result(out, "probe", &p, {});
        }
        if (!c.started || c.exit != 0) {
            return unavailable(c.started ? "the converter did not answer --probe (exit " + std::to_string(c.exit) + "): " + c.err : c.start_error,
                               found.tools->python);
        }
        const Expected<StepProbe> pr = parse_probe(c.out);
        if (!pr) return unavailable(pr.error().message, found.tools->python);
        if (!pr->ocp) return unavailable(pr->reason.empty() ? "OpenCASCADE is not installed for that python" : pr->reason, found.tools->python);
        JsonObject step;
        step.boolean("available", true).str("python", pr->python).str("python_path", found.tools->python);
        step.str("occt", pr->occt).str("wheel", pr->wheel).str("converter", found.tools->converter.string());
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
        return fail({"format_unknown", unsupported_format_message(path)});
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
    const bool is_step = *format == "step";
    const bool fast_inspect = is_step && !importing && !opts.deep;
    double deflection_m = 0.0, angle_rad = 0.0;
    if (is_step) {
        if (!fast_inspect && !opts.deflection) {
            return fail({"deflection_required", std::string("a STEP ") + (importing ? "import" : "deep inspect") +
                                                    " needs --deflection with a unit, for example 0.1mm"});
        }
        if (opts.deflection) {
            const auto d = parse_length_m(*opts.deflection, "--deflection");
            if (!d) return fail(d.error());
            deflection_m = *d;
        }
        const auto a = parse_angle(opts.angle);
        if (!a) return fail(a.error());
        angle_rad = *a;
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

    ToolSearch search;  // the converter is only needed when something must be converted: a cache hit runs without it
    if (is_step) search = find_step_tools(opts, env);
    const auto no_converter = [&]() {
        std::string where;
        for (const std::string& w : search.searched) where += (where.empty() ? "" : "; ") + w;
        return Problem{"step_unavailable", "no STEP converter was found (python with step_convert.py); looked at: " + where};
    };

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
    {  // the picks name the entry whatever the order they were given in (one pick: its own name, as before)
        std::vector<std::string> sorted = opts.objects;
        std::sort(sorted.begin(), sorted.end());
        for (const std::string& n : sorted) key_in.object += (key_in.object.empty() ? "" : "\x1f") + n;
    }
    if (is_step) {
        key_in.converter = kStepConverterVersion;
        key_in.tolerance_m = deflection_m;
        key_in.angle_rad = angle_rad;
    }
    std::map<std::string, std::string> cross_by_name;  // object name -> its cross-check as JSON, for the objects the converter measured
    const std::string source_name = path.filename().string();
    const auto import_payload = [&](const EntryRecord& rec, bool cached) {
        JsonObject p;
        p.boolean("cached", cached).str("sha256", rec.sha256).str("key", rec.key).str("converter", rec.converter);
        p.str("store", store.string()).str("entry", rec.dir.string()).str("source", (store / rec.sha256 / rec.source_name).string());
        std::string objects;
        for (const SidecarRecord& s : rec.sidecars) {
            if (!objects.empty()) objects.push_back(',');
            JsonObject o;
            o.str("name", s.object).str("sidecar", (rec.dir / s.file).string()).str("content_hash", s.content_hash);
            o.integer("triangles", s.triangles).num("volume_m3", s.volume_m3);
            o.raw("store_ref", JsonObject{}.str("sha256", s.store_sha256).str("file", s.file).line());
            if (!s.cross_check_json.empty()) o.raw("cross_check", s.cross_check_json);
            o.raw("colour", colour_json(s.colour)).integer("colours", colour_count(s.colour));
            objects += o.line();
        }
        p.raw("objects", "[" + objects + "]");
        if (!rec.skipped.empty()) {
            std::string skipped_list;
            for (const std::string& name : rec.skipped) skipped_list += (skipped_list.empty() ? "" : ",") + skipped_object(name);
            p.raw("skipped", "[" + skipped_list + "]");
        }
        return p;
    };
    if (importing) {
        if (const auto hit = find_entry(store, sha, cache_key(key_in))) {
            std::vector<std::string> flagged_on_hit;  // the flag was kept with the entry: a flagged import is flagged every time
            for (const SidecarRecord& s : hit->sidecars) {
                const nlohmann::json x = nlohmann::json::parse(s.cross_check_json.empty() ? "null" : s.cross_check_json, nullptr, false);
                if (x.is_object() && x.value("flagged", false)) flagged_on_hit.push_back(s.object);
            }
            if (!flagged_on_hit.empty()) {
                const Problem p = flagged_problem_for(flagged_on_hit);
                return result(out, "import", &p, import_payload(*hit, true));
            }
            return result(out, "import", nullptr, import_payload(*hit, true));
        }
    }

    // ---- a STEP's fast inspect: the converter's own solids, boxes and B-rep volumes; nothing is tessellated, so no gate ----------------
    if (fast_inspect) {
        if (!search.tools) return fail(no_converter(), base);
        stage(out, "convert", "start");
        const ConverterOutcome c = run_converter(*search.tools, {path.string(), "--inspect"}, env);
        if (c.killed) return fail({"cancelled", "cancelled while the converter ran"}, base);
        if (!c.started || c.exit != 0) return fail(converter_problem(c), base);
        const Expected<StepInspect> info = parse_inspect(c.out);
        if (!info) return fail(info.error(), base);
        stage(out, "convert", "done");
        std::string list;
        for (const StepSolid& sol : info->solids) {
            glm::dvec3 lo(1e300), hi(-1e300);
            for (int corner = 0; corner < 8; ++corner) {
                const glm::dvec3 p((corner & 1) ? sol.hi.x : sol.lo.x, (corner & 2) ? sol.hi.y : sol.lo.y, (corner & 4) ? sol.hi.z : sol.lo.z);
                const glm::dvec3 q = apply_axes(axes, p) * scale;
                lo = glm::min(lo, q);
                hi = glm::max(hi, q);
            }
            if (!list.empty()) list.push_back(',');
            list += JsonObject{}
                        .str("name", sol.name)
                        .str("gate", "not_checked")
                        .raw("bbox", JsonObject{}.raw("min", array3(lo)).raw("max", array3(hi)).line())
                        .num("volume", sol.volume_m3 * scale * scale * scale)
                        .raw("colour", colour_json(sol.colour))
                        .integer("colours", colour_count(sol.colour))
                        .line();
        }
        for (const std::string& name : info->skipped) {
            if (!list.empty()) list.push_back(',');
            list += skipped_object(name);
        }
        JsonObject unit_json;
        unit_json.str("declared", info->unit).str("used", info->unit).null("guess");
        JsonObject payload = base;
        payload.raw("unit", unit_json.line())
            .str("measures_in", "m")
            .str("frame", (axes.identity() && scale == 1.0) ? "file" : "part")
            .str("gate", "not_checked")
            .str("depth", "fast")
            .str("occt", info->occt)
            .str("wheel", info->wheel)
            .raw("objects", "[" + list + "]");
        return result(out, "inspect", nullptr, payload);
    }

    // ---- check: read the objects, pick, put them in the part frame ------------------------------------------------------------
    stage(out, "check", "start");
    JsonObject refused = base;
    refused.str("gate", "refused").raw("objects", "[]");
    if (*format == "gltf" && gltf_has_external_uri(bytes)) {
        return fail({"external_buffers", "the glTF refers to files beside it, so it is not one self-contained file"}, refused);
    }
    std::optional<EmitReport> emitted;
    const auto convert_step = [&]() -> Expected<Candidates> {
        if (!search.tools) return std::unexpected(no_converter());
        std::error_code ec;
        const fs::path work = fs::temp_directory_path(ec) / ("spade_part_import_" + env.token);
        fs::remove_all(work, ec);
        fs::create_directories(work, ec);
        if (ec) return std::unexpected(Problem{"store_io", "cannot make the temporary directory " + work.string()});
        struct Cleanup {
            fs::path dir;
            ~Cleanup() {
                std::error_code e;
                fs::remove_all(dir, e);
            }
        } cleanup{work};
        const fs::path model = work / "model.3mf";
        std::vector<std::string> a = {path.string(), "--emit-3mf", model.string(), "--deflection", *opts.deflection, "--angle", json_number(angle_rad)};
        for (const std::string& pick : opts.objects) {
            a.push_back("--solid");
            a.push_back(pick);
        }
        stage(out, "convert", "start");
        const ConverterOutcome c = run_converter(*search.tools, a, env);
        if (c.killed) return std::unexpected(Problem{"cancelled", "cancelled while the converter ran"});
        if (!c.started || (c.exit != 0 && c.exit != 5)) return std::unexpected(converter_problem(c));
        Expected<EmitReport> report = parse_emit(c.out);
        if (!report) return std::unexpected(report.error());
        emitted = std::move(*report);
        stage(out, "convert", "done");
        return to_candidates(read_3mf_objects(model));
    };
    Expected<Candidates> read = is_step ? convert_step() : read_objects(*format, path, bytes, unit.value_or(MeshUnit::metre));
    if (!read) return fail(read.error(), read.error().code == "cancelled" ? base : refused);
    std::map<std::string, std::optional<SurfaceColour>> colour_by_name;  // each object's colour: the reader's, or the converter's for a STEP
    for (std::size_t k = 0; k < read->objects.size(); ++k) colour_by_name[read->objects[k].name] = read->colour[k];
    if (emitted) {
        for (const ObjectColour& c : emitted->colours) colour_by_name[c.name] = c.colour;
    }
    const auto colour_of = [&](const std::string& name) {
        const auto it = colour_by_name.find(name);
        return it == colour_by_name.end() ? std::optional<SurfaceColour>() : it->second;
    };
    std::vector<MeshObject> objects = std::move(read->objects);
    std::vector<std::optional<GateRefusal>> refusal = std::move(read->refusal);
    if (!opts.objects.empty()) {
        std::vector<MeshObject> picked;
        std::vector<std::optional<GateRefusal>> picked_refusal;
        for (std::size_t k = 0; k < objects.size(); ++k) {
            if (std::find(opts.objects.begin(), opts.objects.end(), objects[k].name) == opts.objects.end()) continue;
            picked.push_back(objects[k]);
            picked_refusal.push_back(refusal[k]);
        }
        for (const std::string& pick : opts.objects) {
            const bool found = std::any_of(objects.begin(), objects.end(), [&](const MeshObject& o) { return o.name == pick; });
            if (found) continue;
            std::string names;
            for (const MeshObject& o : objects) names += (names.empty() ? "'" : ", '") + o.name + "'";
            return fail({"object_not_found", "the file has no object named '" + pick + "'; it has " + names}, base);
        }
        objects = std::move(picked);
        refusal = std::move(picked_refusal);
    }
    std::size_t refused_count = 0;
    GateRefusal first_refusal;
    for (const std::optional<GateRefusal>& r : refusal) {
        if (!r) continue;
        if (refused_count++ == 0) first_refusal = *r;
    }
    // The gate's refusal: the first refused object's kind as the code and its message, the count when several.
    const Problem gate_problem{first_refusal.code, refused_count > 1 ? first_refusal.message + " (and " + std::to_string(refused_count - 1) +
                                                                           " more objects refused)"
                                                                     : first_refusal.message};
    if (importing && refused_count > 0) return fail(gate_problem, refused);
    std::vector<std::string> flagged_names;
    std::string cross_list;
    if (emitted) {
        for (const CrossCheck& x : emitted->checks) {
            const std::string one = JsonObject{}
                                        .str("name", x.name)
                                        .num("brep_volume", x.brep_volume)
                                        .num("mesh_volume", x.mesh_volume)
                                        .num("difference", x.difference)
                                        .boolean("flagged", x.flagged)
                                        .line();
            cross_by_name[x.name] = one;
            cross_list += (cross_list.empty() ? "" : ",") + one;
            if (x.flagged) flagged_names.push_back(x.name);
        }
    }
    const auto flagged_problem = [&]() { return flagged_problem_for(flagged_names); };
    glm::vec3 file_lo(1e30f), file_hi(-1e30f);  // the objects' box in the file's own numbers, for the unit guess
    for (const MeshObject& o : objects) {
        if (!o.mesh) continue;
        for (const glm::vec3& p : o.mesh->positions()) {
            file_lo = glm::min(file_lo, p);
            file_hi = glm::max(file_hi, p);
        }
    }
    for (MeshObject& o : objects) {
        if (!o.mesh) continue;
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
        if ((*format == "3mf" || is_step) && !objects.empty()) declared = unit_text(objects.front().unit);
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
        for (std::size_t k = 0; k < objects.size(); ++k) {
            const MeshObject& o = objects[k];
            if (!list.empty()) list.push_back(',');
            if (refusal[k]) {  // this object did not pass the gate
                list += JsonObject{}.str("name", o.name).boolean("ok", false).str("code", refusal[k]->code).str("message", refusal[k]->message).line();
                continue;
            }
            glm::vec3 lo(1e30f), hi(-1e30f);
            for (const glm::vec3& p : o.mesh->positions()) {
                lo = glm::min(lo, p);
                hi = glm::max(hi, p);
            }
            list += JsonObject{}
                        .str("name", o.name)
                        .boolean("ok", true)
                        .integer("triangles", o.mesh->triangle_count())
                        .integer("vertices", static_cast<std::int64_t>(o.mesh->positions().size()))
                        .integer("shells", o.mesh->shell_count())
                        .raw("bbox", JsonObject{}.raw("min", array3(lo)).raw("max", array3(hi)).line())
                        .num("volume", o.mesh->mass_properties().volume)
                        .str("content_hash", hex16(o.mesh->content_hash()))
                        .raw("colour", colour_json(colour_of(o.name)))
                        .integer("colours", colour_count(colour_of(o.name)))
                        .line();
        }
        JsonObject payload = base;
        payload.raw("unit", unit_json.line())
            .str("measures_in", used ? "m" : "file")
            .str("frame", (axes.identity() && scale == 1.0) ? "file" : "part")
            .str("gate", refused_count > 0 ? "refused" : "passed")
            .str("depth", is_step ? "deep" : "full")
            .raw("objects", "[" + list + "]");
        if (emitted) payload.raw("cross_check", "[" + cross_list + "]");
        if (emitted && !emitted->skipped.empty()) {
            std::string skipped_list;
            for (const std::string& name : emitted->skipped) skipped_list += (skipped_list.empty() ? "" : ",") + skipped_object(name);
            payload.raw("skipped", "[" + skipped_list + "]");
        }
        if (refused_count > 0) return result(out, "inspect", &gate_problem, payload);
        if (!flagged_names.empty()) {
            const Problem p = flagged_problem();
            return result(out, "inspect", &p, payload);
        }
        return result(out, "inspect", nullptr, payload);
    }

    // ---- import --------------------------------------------------------------------------------------------------------------
    stage(out, "write", "start");
    (void)remove_stale_temps(store, env.stale_after);
    std::vector<StoredObject> stored;
    for (const MeshObject& o : objects) {
        const auto cross = cross_by_name.find(o.name);
        stored.push_back({o.name, o.mesh, MeshProvenance{source_name + ": " + o.name, o.unit, key_in.tolerance_m},
                          cross == cross_by_name.end() ? std::string() : cross->second, colour_of(o.name)});
    }
    const std::vector<std::string> no_solid = emitted ? emitted->skipped : std::vector<std::string>();
    const Expected<EntryRecord> rec = write_entry(store, bytes, source_name, key_in, stored, env.token, no_solid);
    if (!rec) return fail(rec.error(), base);
    stage(out, "write", "done");
    if (!flagged_names.empty()) {
        const Problem p = flagged_problem();
        return result(out, "import", &p, import_payload(*rec, false));
    }
    return result(out, "import", nullptr, import_payload(*rec, false));
}

}  // namespace spade::partimport
