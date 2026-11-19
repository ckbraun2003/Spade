#include "library_json.hpp"
#include "mesh_info.hpp"

#include <array>
#include <cmath>
#include <exception>
#include <filesystem>
#include <initializer_list>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <nlohmann/json.hpp>

#include "physics/forces.hpp"
#include "scene/library_compile.hpp"
#include "scene/library_place.hpp"
#include "scene/library_query.hpp"
#include "scene/library_set.hpp"
#include "scene/library_validate.hpp"
#include "scene/model_file.hpp"
#include "vehicles/airframe_compile.hpp"

namespace spade::libc {

namespace fs = std::filesystem;
namespace sc = spade::scene;
using Json = nlohmann::json;

class Library {
public:
    sc::LibrarySet set{};
};

void LibraryDeleter::operator()(Library* lib) const { delete lib; }

namespace {

// ----- text <-> path -------------------------------------------------------------------------------------------------------------------

std::string utf8(const fs::path& p) {
    const std::u8string s = p.generic_u8string();  // forward slashes on every platform, so a response reads the same on all of them
    return std::string(reinterpret_cast<const char*>(s.data()), s.size());
}

fs::path path_of(const std::string& text) {
    return fs::path(std::u8string(reinterpret_cast<const char8_t*>(text.data()), text.size()));
}

Json str_or_null(const std::string& s) { return s.empty() ? Json(nullptr) : Json(s); }

// ----- refusals ------------------------------------------------------------------------------------------------------------------------

sc::LibraryRefusal refusal(std::string code, std::string message, std::string field = {}) {
    sc::LibraryRefusal r;
    r.code = std::move(code);
    r.message = std::move(message);
    r.field = std::move(field);
    return r;
}

// Thrown while a request is being read; caught in the dispatcher and answered as a refusal.
struct Refused {
    sc::LibraryRefusal refusal;
};

Json refusal_json(const sc::LibraryRefusal& r, const char* severity = "error") {
    Json issues = Json::array();
    for (const sc::LibraryRefusal& i : r.issues) issues.push_back(refusal_json(i, severity));
    return Json{{"code", r.code},
                {"severity", severity},
                {"message", r.message},
                {"root", str_or_null(r.root)},
                {"path", r.path.empty() ? Json(nullptr) : Json(utf8(path_of(r.path)))},
                {"id", str_or_null(r.id)},
                {"field", str_or_null(r.field)},
                {"issues", std::move(issues)}};
}

Json warnings_json(const std::vector<sc::LibraryRefusal>& warnings) {
    Json out = Json::array();
    for (const sc::LibraryRefusal& w : warnings) out.push_back(refusal_json(w, "warning"));
    return out;
}

Json envelope(std::string_view op) {
    return Json{{"format", std::string(kFormat)}, {"minor", kMinor}, {"op", std::string(op)}};
}

Answer ok_answer(std::string_view op, Json result, const std::vector<sc::LibraryRefusal>& warnings = {}) {
    Json out = envelope(op);
    out["ok"] = true;
    out["result"] = std::move(result);
    out["warnings"] = warnings_json(warnings);
    return {0, out.dump()};
}

Answer refused_answer(std::string_view op, const sc::LibraryRefusal& r) {
    Json out = envelope(op);
    out["ok"] = false;
    out["refusal"] = refusal_json(r);
    return {1, out.dump()};
}

// ----- reading a request ---------------------------------------------------------------------------------------------------------------

[[noreturn]] void refuse(std::string code, std::string message, std::string field = {}) {
    throw Refused{refusal(std::move(code), std::move(message), std::move(field))};
}

std::string join(std::string_view where, std::string_view key) { return where.empty() ? std::string(key) : std::string(where) + "." + std::string(key); }

// The request must be an object whose keys are all among `keys` (and "format", where `format_ok`).
void only_keys(const Json& obj, std::initializer_list<const char*> keys, const std::string& where, bool format_ok) {
    for (auto it = obj.begin(); it != obj.end(); ++it) {
        bool known = format_ok && it.key() == "format";
        for (const char* k : keys) known = known || it.key() == k;
        if (!known) refuse("invalid_request", "unknown key '" + it.key() + "'", join(where, it.key()));
    }
}

// The member `key`, or null when it is absent or JSON null.
const Json* member(const Json& obj, const char* key) {
    const auto it = obj.find(key);
    return (it == obj.end() || it->is_null()) ? nullptr : &*it;
}

const Json& need(const Json& obj, const char* key, std::string_view where) {
    if (const Json* m = member(obj, key)) return *m;
    refuse("invalid_request", std::string("missing key '") + key + "'", join(where, key));
}

std::string as_string(const Json& v, const std::string& field) {
    if (!v.is_string()) refuse("invalid_request", "'" + field + "' must be a string", field);
    return v.get<std::string>();
}

std::string opt_string(const Json& obj, const char* key, const std::string& where) {
    const Json* m = member(obj, key);
    return m == nullptr ? std::string() : as_string(*m, join(where, key));
}

bool opt_bool(const Json& obj, const char* key, const std::string& where, bool fallback) {
    const Json* m = member(obj, key);
    if (m == nullptr) return fallback;
    if (!m->is_boolean()) refuse("invalid_request", std::string("'") + key + "' must be true or false", join(where, key));
    return m->get<bool>();
}

std::optional<std::size_t> opt_count(const Json& obj, const char* key, const std::string& where) {
    const Json* m = member(obj, key);
    if (m == nullptr) return std::nullopt;
    if (!m->is_number_unsigned() && !(m->is_number_integer() && m->get<long long>() >= 0)) {
        refuse("invalid_request", std::string("'") + key + "' must be a whole number, 0 or more", join(where, key));
    }
    return static_cast<std::size_t>(m->get<unsigned long long>());
}

std::vector<std::string> opt_strings(const Json& obj, const char* key, const std::string& where) {
    std::vector<std::string> out;
    const Json* m = member(obj, key);
    if (m == nullptr) return out;
    if (!m->is_array()) refuse("invalid_request", std::string("'") + key + "' must be a list of strings", join(where, key));
    for (std::size_t i = 0; i < m->size(); ++i) out.push_back(as_string((*m)[i], join(where, key) + "[" + std::to_string(i) + "]"));
    return out;
}

// The request text as an object; a "format" it carries must name a major this library speaks.
Json parse_request(std::string_view text) {
    Json j = Json::parse(text.begin(), text.end(), nullptr, /*allow_exceptions=*/false);
    if (j.is_discarded() || !j.is_object()) refuse("invalid_request", "the request is not a JSON object");
    if (const Json* f = member(j, "format")) {
        if (!f->is_string()) refuse("invalid_request", "'format' must be a string", "format");
        const std::string s = f->get<std::string>();
        constexpr std::string_view kPrefix = "spade.library/";
        bool supported = false;
        if (s.rfind(kPrefix, 0) == 0) {
            const std::string rest = s.substr(kPrefix.size());
            const std::string major = rest.substr(0, rest.find('.'));
            supported = major == "1";
        }
        if (!supported) refuse("unsupported_format", "this library speaks " + std::string(kFormat) + ", not '" + s + "'", "format");
    }
    return j;
}

// ----- numbers and poses ---------------------------------------------------------------------------------------------------------------

double number(const Json& v, const std::string& field) {
    if (!v.is_number()) refuse("invalid_request", "'" + field + "' must be a number", field);
    const double d = v.get<double>();
    if (!std::isfinite(d)) refuse("invalid_request", "'" + field + "' must be a finite number", field);
    return d;
}

template <std::size_t N>
std::array<double, N> numbers(const Json& v, const std::string& field) {
    if (!v.is_array() || v.size() != N) refuse("invalid_request", "'" + field + "' must be a list of " + std::to_string(N) + " numbers", field);
    std::array<double, N> out{};
    for (std::size_t i = 0; i < N; ++i) out[i] = number(v[i], field + "[" + std::to_string(i) + "]");
    return out;
}

sc::SetPose read_pose(const Json& v, const std::string& field) {
    if (!v.is_object()) refuse("invalid_request", "'" + field + "' must be {pos, quat}", field);
    only_keys(v, {"pos", "quat"}, field, false);
    sc::SetPose p;
    p.pos = numbers<3>(need(v, "pos", field), field + ".pos");
    p.quat = numbers<4>(need(v, "quat", field), field + ".quat");
    return p;
}

sc::SetAdjust read_adjust(const Json& v, const std::string& field) {
    if (!v.is_object()) refuse("invalid_request", "'" + field + "' must be an object", field);
    only_keys(v, {"pos", "yaw_deg", "pitch_deg", "roll_deg"}, field, false);
    sc::SetAdjust a;
    if (const Json* m = member(v, "pos")) a.pos = numbers<3>(*m, field + ".pos");
    if (const Json* m = member(v, "yaw_deg")) a.yaw_deg = number(*m, field + ".yaw_deg");
    if (const Json* m = member(v, "pitch_deg")) a.pitch_deg = number(*m, field + ".pitch_deg");
    if (const Json* m = member(v, "roll_deg")) a.roll_deg = number(*m, field + ".roll_deg");
    return a;
}

sc::AdjustRange read_adjust_range(const Json& v, const std::string& field) {
    if (!v.is_object()) refuse("invalid_request", "'" + field + "' must be an object", field);
    only_keys(v, {"pos_min", "pos_max", "yaw_deg", "pitch_deg", "roll_deg"}, field, false);
    sc::AdjustRange r;
    const auto lo = numbers<3>(need(v, "pos_min", field), field + ".pos_min");
    const auto hi = numbers<3>(need(v, "pos_max", field), field + ".pos_max");
    r.pos_min = {lo[0], lo[1], lo[2]};
    r.pos_max = {hi[0], hi[1], hi[2]};
    r.yaw_deg = numbers<2>(need(v, "yaw_deg", field), field + ".yaw_deg");
    r.pitch_deg = numbers<2>(need(v, "pitch_deg", field), field + ".pitch_deg");
    r.roll_deg = numbers<2>(need(v, "roll_deg", field), field + ".roll_deg");
    return r;
}

Json pose_json(const sc::SetPose& p) {
    return Json{{"pos", Json::array({p.pos[0], p.pos[1], p.pos[2]})}, {"quat", Json::array({p.quat[0], p.quat[1], p.quat[2], p.quat[3]})}};
}

Json adjust_json(const sc::SetAdjust& a) {
    return Json{{"pos", Json::array({a.pos[0], a.pos[1], a.pos[2]})},
                {"yaw_deg", a.yaw_deg},
                {"pitch_deg", a.pitch_deg},
                {"roll_deg", a.roll_deg}};
}

// ----- the entry object ----------------------------------------------------------------------------------------------------------------

Json peek_json(const sc::EntryPeek& p) {
    Json limits = Json::object();
    for (const auto& [k, v] : p.limits) limits[k] = v;
    return Json{{"mass_kg", p.mass_kg ? Json(*p.mass_kg) : Json(nullptr)},
                {"provides", p.provides},
                {"limits", std::move(limits)},
                {"rotor_count", p.rotor_count ? Json(*p.rotor_count) : Json(nullptr)},
                {"world_version", p.world_version ? Json(*p.world_version) : Json(nullptr)},
                {"install_kinds", p.install_kinds}};
}

Json entry_json(const sc::SetEntry& e, const std::string& hash, const std::optional<sc::EntryPeek>& peek) {
    Json store_file = nullptr;
    if (e.in_store) {
        const bool have_path = !e.file.empty();
        std::error_code ec;
        store_file = Json{{"sha256", e.store_sha256},
                          {"file", e.store_file},
                          {"path", have_path ? Json(utf8(e.file)) : Json(nullptr)},
                          {"present", have_path && fs::exists(e.file, ec)}};
    }
    Json generated = nullptr;
    if (!e.generated_recipe.empty()) generated = Json{{"recipe", e.generated_recipe}, {"hash", str_or_null(e.generated_hash)}};
    return Json{{"id", e.id},
                {"root", e.root},
                {"local_id", e.local_id},
                {"kind", e.kind},
                {"part_kind", str_or_null(e.part_kind)},
                {"type", str_or_null(e.type)},
                {"name", e.name},
                {"category", e.category},
                {"tags", e.tags},
                {"file", e.file.empty() ? Json(nullptr) : Json(utf8(e.file))},
                {"rel_file", str_or_null(e.rel_file)},
                {"thumbnail", e.thumbnail.empty() ? Json(nullptr) : Json(utf8(e.thumbnail))},
                {"requires", e.requires_ids},
                {"files", e.files},
                {"store_file", std::move(store_file)},
                {"hash", str_or_null(hash)},
                {"generated_from", std::move(generated)},
                {"peek", peek ? peek_json(*peek) : Json(nullptr)}};
}

Json slot_json(const sc::PartsMapSlot& s) {
    return Json{{"slot", s.slot},
                {"part", s.part},
                {"hash", str_or_null(s.hash)},
                {"parent", str_or_null(s.parent)},
                {"install_point", str_or_null(s.install_point)},
                {"placement", s.placement ? pose_json(*s.placement) : Json(nullptr)},
                {"adjust", s.adjust ? adjust_json(*s.adjust) : Json(nullptr)},
                {"count", s.count}};
}

// ----- the ops -------------------------------------------------------------------------------------------------------------------------

Json roots_json(const sc::LibrarySet& set) {
    Json roots = Json::array();
    for (const sc::SetRoot& r : set.roots) {
        roots.push_back(Json{{"name", r.name},
                             {"path", utf8(r.path)},
                             {"library_version", r.library_version},
                             {"depends", r.depends},
                             {"entries", r.entry_count},
                             {"writable", r.writable}});
    }
    return Json{{"roots", std::move(roots)},
                {"entry_count", set.entries.size()},
                {"store", set.store.empty() ? Json(nullptr) : Json(utf8(set.store))}};
}

Answer op_list(const Library& lib, const Json& req) {
    only_keys(req, {"roots", "kind", "part_kind", "type", "text", "tags", "offset", "limit"}, "", true);
    sc::ListFilter f;
    f.roots = opt_strings(req, "roots", "");
    f.kind = opt_string(req, "kind", "");
    f.part_kind = opt_string(req, "part_kind", "");
    f.type = opt_string(req, "type", "");
    f.text = opt_string(req, "text", "");
    f.tags = opt_strings(req, "tags", "");
    f.offset = opt_count(req, "offset", "").value_or(0);
    f.limit = opt_count(req, "limit", "");
    auto page = sc::list_entries(lib.set, f);
    if (!page) return refused_answer("list", page.error());
    Json entries = Json::array();
    for (const sc::ListedEntry& row : page->rows) entries.push_back(entry_json(*row.entry, row.hash, row.peek));
    return ok_answer("list", Json{{"entries", std::move(entries)}, {"total", page->total}, {"offset", page->offset}}, page->warnings);
}

Answer op_find(const Library& lib, const Json& req) {
    only_keys(req, {"id", "expect_hash", "with"}, "", true);
    const std::string id = as_string(need(req, "id", ""), "id");
    sc::FindOptions options;
    if (const Json* m = member(req, "expect_hash")) options.expect_hash = as_string(*m, "expect_hash");
    bool want_parts_map = false;
    const std::vector<std::string> with = opt_strings(req, "with", "");
    for (std::size_t i = 0; i < with.size(); ++i) {
        if (with[i] == "record") {
            options.with_record = true;
        } else if (with[i] == "parts_map") {
            want_parts_map = true;
        } else {
            refuse("invalid_request", "'with' takes record and parts_map, not '" + with[i] + "'", "with[" + std::to_string(i) + "]");
        }
    }
    auto found = sc::find_entry(lib.set, id, options);
    if (!found) return refused_answer("find", found.error());
    const sc::SetEntry& entry = *found->entry;
    std::vector<sc::LibraryRefusal> warnings = found->warnings;
    std::string why;
    std::optional<sc::EntryPeek> peek = sc::peek_entry(entry, &why);
    if (!peek) {
        sc::LibraryRefusal w = refusal("peek_failed", why.empty() ? "the entry's header could not be read" : why);
        w.root = entry.root;
        w.id = entry.id;
        w.path = utf8(entry.file);
        warnings.push_back(std::move(w));
    }
    Json drone = nullptr;
    if (want_parts_map && entry.kind == "drone") {
        Json slots = Json::array();
        for (const sc::PartsMapSlot& s : entry.parts_map) slots.push_back(slot_json(s));
        drone = Json{{"frame", "design/1"}, {"parts_map", std::move(slots)}, {"measured", nullptr}};
    }
    Json record = nullptr;
    if (options.with_record && !found->record_json.empty()) record = Json::parse(found->record_json);
    return ok_answer("find",
                     Json{{"entry", entry_json(entry, found->hash, peek)},
                          {"hash_matches", found->hash_matches ? Json(*found->hash_matches) : Json(nullptr)},
                          {"drone", std::move(drone)},
                          {"record", std::move(record)}},
                     warnings);
}

Answer op_resolve(const Library& lib, const Json& req) {
    only_keys(req, {"alias", "root"}, "", true);
    const std::string alias = as_string(need(req, "alias", ""), "alias");
    const std::string root = opt_string(req, "root", "");
    auto r = sc::resolve_alias(lib.set, alias, root);
    if (!r) return refused_answer("resolve", r.error());
    return ok_answer("resolve", Json{{"id", r->id}, {"root", r->root}});
}

Answer op_hash_text(const Json& req) {
    only_keys(req, {"text"}, "", true);
    return ok_answer("hash_text", Json{{"hash", sc::hash_text(as_string(need(req, "text", ""), "text"))}});
}

Answer op_validate(const Json& req) {
    only_keys(req, {"as", "text", "name"}, "", true);
    sc::ValidateOptions options;
    options.as = as_string(need(req, "as", ""), "as");
    const std::string text = as_string(need(req, "text", ""), "text");
    (void)opt_string(req, "name", "");
    auto result = sc::validate_text(text, options);
    if (!result) return refused_answer("validate", result.error());
    Json issues = Json::array();
    for (const sc::LibraryIssue& i : result->issues) {
        issues.push_back(Json{{"code", i.code},
                              {"severity", i.severity},
                              {"kind", str_or_null(i.kind)},
                              {"index", i.index},
                              {"id", str_or_null(i.id)},
                              {"path", str_or_null(i.path)},
                              {"line", i.line ? Json(*i.line) : Json(nullptr)},
                              {"column", i.column ? Json(*i.column) : Json(nullptr)},
                              {"message", i.message}});
    }
    return ok_answer("validate", Json{{"valid", result->valid}, {"issues", std::move(issues)}});
}

// One request item. A problem inside the item is that item's refusal, so one bad slot does not hide the others; an item that is not an
// object, or has no `key`, cannot be answered by itself and refuses the call.
Answer op_compose_placement(const Json& req) {
    only_keys(req, {"items"}, "", true);
    const Json& list = need(req, "items", "");
    if (!list.is_array()) refuse("invalid_request", "'items' must be a list", "items");
    std::vector<sc::PlacementItem> items;
    std::vector<std::optional<sc::LibraryRefusal>> bad(list.size());
    for (std::size_t i = 0; i < list.size(); ++i) {
        const std::string where = "items[" + std::to_string(i) + "]";
        const Json& v = list[i];
        if (!v.is_object()) refuse("invalid_request", "'" + where + "' must be an object", where);
        sc::PlacementItem item;
        item.key = as_string(need(v, "key", where), where + ".key");
        try {
            only_keys(v, {"key", "parent", "install_point", "adjust", "adjust_range", "part_to_mount"}, "", false);
            if (const Json* m = member(v, "parent")) item.parent = read_pose(*m, "parent");
            item.install_point = read_pose(need(v, "install_point", ""), "install_point");
            if (const Json* m = member(v, "adjust")) item.adjust = read_adjust(*m, "adjust");
            if (const Json* m = member(v, "adjust_range")) item.adjust_range = read_adjust_range(*m, "adjust_range");
            if (const Json* m = member(v, "part_to_mount")) item.part_to_mount = read_pose(*m, "part_to_mount");
        } catch (const Refused& r) {
            bad[i] = r.refusal;
        }
        items.push_back(std::move(item));
    }
    // Compose the good items together, in request order, and put each answer back in its place.
    std::vector<sc::PlacementItem> good;
    for (std::size_t i = 0; i < items.size(); ++i) {
        if (!bad[i]) good.push_back(items[i]);
    }
    const std::vector<sc::PlacedItem> placed = sc::compose_placement(good);
    Json out = Json::array();
    std::size_t next = 0;
    for (std::size_t i = 0; i < items.size(); ++i) {
        if (bad[i]) {
            out.push_back(Json{{"key", items[i].key}, {"pose", nullptr}, {"refusal", refusal_json(*bad[i])}});
            continue;
        }
        const sc::PlacedItem& p = placed[next++];
        out.push_back(Json{{"key", p.key},
                           {"pose", p.pose ? pose_json(*p.pose) : Json(nullptr)},
                           {"refusal", p.refusal ? refusal_json(*p.refusal) : Json(nullptr)}});
    }
    return ok_answer("compose_placement", Json{{"items", std::move(out)}});
}

// mesh_file_info: a file by absolute `path`, or by the `id` of an entry that has one; `unit` names the unit of an STL or OBJ file, which declares none.
Answer op_mesh_file_info(const Library& lib, const Json& req) {
    only_keys(req, {"path", "id", "unit"}, "", true);
    const std::string path = opt_string(req, "path", "");
    const std::string id = opt_string(req, "id", "");
    if (path.empty() == id.empty()) refuse("invalid_request", "give exactly one of `path` and `id`", "path");
    fs::path file;
    if (!path.empty()) {
        file = path_of(path);
        if (!file.is_absolute()) refuse("relative_path", "the path must be absolute", "path");
    } else {
        auto found = sc::find_entry(lib.set, id, {});
        if (!found) return refused_answer("mesh_file_info", found.error());
        if (found->entry->file.empty()) refuse("invalid_request", "the entry has no file", "id");
        file = found->entry->file;
    }
    auto info = mesh_file_info(file, opt_string(req, "unit", ""));
    if (!info) return refused_answer("mesh_file_info", info.error());
    return ok_answer("mesh_file_info", std::move(*info));
}

const char* provenance_name(vehicles::Provenance p) {
    switch (p) {
        case vehicles::Provenance::from_parts: return "from_parts";
        case vehicles::Provenance::fitted: return "fitted";
        case vehicles::Provenance::estimated: return "estimated";
        case vehicles::Provenance::given: return "given";
    }
    return "given";
}

// A DragSpec as the compile takes it: {mode: "quadratic" | "componentwise", coeffs: [3], area?}.
vehicles::DragSpec read_drag(const Json& v, const std::string& field) {
    if (!v.is_object()) refuse("invalid_request", "'" + field + "' must be {mode, coeffs, area}", field);
    only_keys(v, {"mode", "area", "coeffs"}, field, false);
    vehicles::DragSpec d;
    const std::string mode = as_string(need(v, "mode", field), join(field, "mode"));
    if (mode == "quadratic") {
        d.mode = physics::drag_mode::quadratic;
    } else if (mode == "componentwise") {
        d.mode = physics::drag_mode::componentwise;
    } else {
        refuse("invalid_request", "'mode' must be \"quadratic\" or \"componentwise\"", join(field, "mode"));
    }
    if (const Json* m = member(v, "area")) d.area = number(*m, join(field, "area"));
    const auto c = numbers<3>(need(v, "coeffs", field), join(field, "coeffs"));
    d.coeffs = {c[0], c[1], c[2]};
    return d;
}

// compile: a drone entry by `id` and the `options.drag` the compile cannot know; the model Spade would write, the composite and the fit.
Answer op_compile(const Library& lib, const Json& req) {
    only_keys(req, {"id", "options"}, "", true);
    const std::string id = as_string(need(req, "id", ""), "id");
    const Json* options = member(req, "options");
    if (options != nullptr) {
        if (!options->is_object()) refuse("invalid_request", "'options' must be an object", "options");
        only_keys(*options, {"drag"}, "options", false);
    }
    const Json* drag = options != nullptr ? member(*options, "drag") : nullptr;
    if (drag == nullptr) refuse("invalid_request", "options.drag is required: a part's mass is a tensor, which gives no area to estimate a drag from", "options.drag");
    sc::CompileOptions compile_options;
    compile_options.drag = read_drag(*drag, "options.drag");
    const auto compiled = sc::compile_drone(lib.set, id, compile_options);
    if (!compiled) return refused_answer("compile", compiled.error());
    const vehicles::CompiledAirframe& c = compiled->compiled;
    sc::ModelFile file;
    file.model = c.model;
    const auto yaml = sc::model_to_yaml(file);
    if (!yaml) return refused_answer("compile", refusal("compile_refused", yaml.error().context));
    const glm::dmat3& t = c.inertia.inertia_design;
    Json tensor = Json::array();
    for (int r = 0; r < 3; ++r) tensor.push_back(Json::array({t[0][r], t[1][r], t[2][r]}));
    const vehicles::AirframeFit& f = c.fit;
    Json provenance = {{"mass_and_inertia", provenance_name(f.mass_and_inertia)},
                       {"mounts", provenance_name(f.mounts)},
                       {"rotor", provenance_name(f.rotor)},
                       {"rotor_inertia", provenance_name(f.rotor_inertia_source)},
                       {"drag", provenance_name(f.drag)},
                       {"proxy_radius", provenance_name(f.proxy_radius)},
                       {"sensors", provenance_name(f.sensors)}};
    return ok_answer("compile",
                     Json{{"model_yaml", *yaml},
                          {"inertia",
                           {{"mass_kg", static_cast<double>(c.inertia.mass)},
                            {"com", Json::array({static_cast<double>(c.inertia.center_of_mass.x), static_cast<double>(c.inertia.center_of_mass.y),
                                                 static_cast<double>(c.inertia.center_of_mass.z)})},
                            {"tensor", std::move(tensor)}}},
                          {"fit",
                           {{"hover_duty", f.hover_duty},
                            {"hover_omega", f.hover_omega},
                            {"hover_thrust", f.hover_thrust},
                            {"thrust_coeff", f.thrust_coeff},
                            {"torque_coeff", f.torque_coeff},
                            {"tau", f.tau},
                            {"rotor_inertia", f.rotor_inertia},
                            {"provenance", std::move(provenance)}}}});
}

// Ops the contract names and this build does not answer yet ([shape]), and ops it does not name.
bool is_known_op(std::string_view op) {
    for (std::string_view s : {"list", "find", "resolve", "hash_text", "validate", "compose_placement", "mesh_file_info", "reload", "create_root",
                               "add_entry", "update_entry", "remove_entry", "compile", "evaluate"}) {
        if (op == s) return true;
    }
    return false;
}

}  // namespace

Answer misuse(std::string_view op) {
    Answer a = refused_answer(op, refusal("invalid_request", "no library is open"));
    a.rc = 2;
    return a;
}

Answer open_library(std::string_view request_json, LibraryPtr& out) {
    out.reset();
    try {
        const Json req = parse_request(request_json);
        only_keys(req, {"roots", "store"}, "", true);
        const Json& roots = need(req, "roots", "");
        if (!roots.is_array()) refuse("invalid_request", "'roots' must be a list", "roots");
        std::vector<sc::LibraryRootSpec> specs;
        for (std::size_t i = 0; i < roots.size(); ++i) {
            const std::string where = "roots[" + std::to_string(i) + "]";
            const Json& r = roots[i];
            if (!r.is_object()) refuse("invalid_request", "'" + where + "' must be an object", where);
            only_keys(r, {"name", "path", "writable"}, where, false);
            sc::LibraryRootSpec spec;
            spec.name = as_string(need(r, "name", where), where + ".name");
            spec.path = path_of(as_string(need(r, "path", where), where + ".path"));
            spec.writable = opt_bool(r, "writable", where, false);
            specs.push_back(std::move(spec));
        }
        fs::path store;
        if (const Json* s = member(req, "store")) {
            const std::string text = as_string(*s, "store");
            if (text.empty()) {
                sc::LibraryRefusal bad = refusal("relative_path", "the store must be an absolute path", "store");
                return refused_answer("open", bad);
            }
            store = path_of(text);
        }
        auto set = sc::open_libraries(specs, store);
        if (!set) return refused_answer("open", set.error());
        auto lib = std::make_unique<Library>();
        lib->set = std::move(*set);
        Json result = roots_json(lib->set);
        out = LibraryPtr(lib.release());
        return ok_answer("open", std::move(result));
    } catch (const Refused& r) {
        return refused_answer("open", r.refusal);
    } catch (const std::exception& e) {
        return refused_answer("open", refusal("internal_error", e.what()));
    }
}

Answer call_library(const Library& lib, std::string_view op, std::string_view request_json) {
    try {
        if (!is_known_op(op)) return refused_answer(op, refusal("unknown_op", "no such op '" + std::string(op) + "'"));
        const Json req = parse_request(request_json);
        if (op == "list") return op_list(lib, req);
        if (op == "find") return op_find(lib, req);
        if (op == "resolve") return op_resolve(lib, req);
        if (op == "hash_text") return op_hash_text(req);
        if (op == "validate") return op_validate(req);
        if (op == "compose_placement") return op_compose_placement(req);
        if (op == "mesh_file_info") return op_mesh_file_info(lib, req);
        if (op == "compile") return op_compile(lib, req);
        return refused_answer(op, refusal("not_implemented", "'" + std::string(op) + "' is not built in this version"));
    } catch (const Refused& r) {
        return refused_answer(op, r.refusal);
    } catch (const std::exception& e) {
        return refused_answer(op, refusal("internal_error", e.what()));
    }
}

}  // namespace spade::libc
