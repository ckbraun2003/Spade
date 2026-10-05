#include "sim/module.hpp"

#include <algorithm>
#include <functional>
#include <map>
#include <string>
#include <tuple>
#include <utility>

#include "core/rng.hpp"
#include "sim/world_set.hpp"  // kWorldSeedDomainTag

namespace spade::modules {
namespace {

[[nodiscard]] Error invalid(std::string context) { return Error{Code::invalid_argument, std::move(context)}; }

struct Node {
    std::size_t module = 0;  // index in the set
    std::size_t decl = 0;    // index in that module's passes
    const PassDecl* pass = nullptr;
    std::string full;        // "<module>.<pass>"
};

// Two passes touching one quantity need an explicit edge when both write, or
// when one writes and the other accumulates.
[[nodiscard]] bool needs_edge(Access a, Access b) noexcept {
    return (a == Access::write && b != Access::read) || (b == Access::write && a != Access::read);
}

// "field.<name>" -> "<name>"; empty for any other quantity.
constexpr std::string_view kFieldPrefix = "field.";
[[nodiscard]] std::string_view field_name_of(std::string_view q) noexcept {
    return q.starts_with(kFieldPrefix) ? q.substr(kFieldPrefix.size()) : std::string_view{};
}

using FieldOwners = std::map<std::string, std::size_t, std::less<>>;  // field name -> declaring module

[[nodiscard]] bool is_core_quantity(std::string_view q) noexcept {
    for (const std::string_view core : kCoreQuantities) {
        if (q == core) return true;
    }
    return false;
}

// "<module>.<name>" split at its first '.'; false if either half is empty.
[[nodiscard]] bool split_quantity(std::string_view q, std::string_view& owner, std::string_view& name) noexcept {
    const std::size_t dot = q.find('.');
    if (dot == std::string_view::npos || dot == 0 || dot + 1 == q.size()) return false;
    owner = q.substr(0, dot);
    name = q.substr(dot + 1);
    return true;
}

[[nodiscard]] const ModuleDesc* find_module(std::span<const ModuleDesc> modules, std::string_view name) noexcept {
    for (const ModuleDesc& m : modules) {
        if (m.name == name) return &m;
    }
    return nullptr;
}

[[nodiscard]] bool declares_array(const ModuleDesc& m, std::string_view name) noexcept {
    for (const ArrayDecl& a : m.state) {
        if (a.name == name) return true;
    }
    return false;
}

[[nodiscard]] bool declares_table(const ModuleDesc& m, std::string_view name) noexcept {
    for (const ConfigTableDecl& t : m.tables) {
        if (t.name == name) return true;
    }
    return false;
}

// "<module>.<table>" for a table a module in the set declares (Task 7).
[[nodiscard]] bool names_table(std::string_view q, std::span<const ModuleDesc> modules) noexcept {
    std::string_view owner;
    std::string_view name;
    if (!split_quantity(q, owner, name)) return false;
    const ModuleDesc* m = find_module(modules, owner);
    return m != nullptr && declares_table(*m, name);
}

[[nodiscard]] bool known_quantity(std::string_view q, std::span<const ModuleDesc> modules,
                                  const FieldOwners& fields) noexcept {
    if (is_core_quantity(q)) return true;
    if (q.starts_with(kFieldPrefix)) return fields.contains(field_name_of(q));
    std::string_view owner;
    std::string_view name;
    if (!split_quantity(q, owner, name)) return false;
    const ModuleDesc* m = find_module(modules, owner);
    if (m == nullptr) return false;
    // A stateful module's quantities name its arrays or its tables (stage 4),
    // so a misspelt array cannot drop a hazard and a misspelt table cannot bind
    // nothing. A module with neither keeps free tokens, as in stage 1.
    if (m->state.empty() && m->tables.empty()) return true;
    return declares_array(*m, name) || declares_table(*m, name);
}

// OPTIONAL ACCESSES (Task 6; module.hpp's QuantityAccess). Legal only as a
// read of "<module>.<name>". A core quantity and a field are never absent, so
// optional would mean nothing there. When the module is in the set, <name>
// must be one of its arrays or tables (Task 7): an optional read binds an
// array, a table or nothing, so a present module's misspelt name cannot bind
// nothing in silence.
[[nodiscard]] Result<void> check_optional(const std::string& pass, const QuantityAccess& qa,
                                          std::span<const ModuleDesc> modules) {
    const std::string where = pass + ": optional access to '" + std::string(qa.quantity) + "'";
    if (qa.access != Access::read) {
        return std::unexpected(invalid(where + ": only a read may be optional; a write or an accumulation of "
                                               "what may be absent would change state no ordering accounts for"));
    }
    std::string_view owner;
    std::string_view name;
    if (is_core_quantity(qa.quantity) || qa.quantity.starts_with(kFieldPrefix) ||
        !split_quantity(qa.quantity, owner, name)) {
        return std::unexpected(invalid(where + ": optional is for <module>.<array> of a module the set may not "
                                               "hold; a core quantity or a field is never absent"));
    }
    if (const ModuleDesc* m = find_module(modules, owner);
        m != nullptr && !declares_array(*m, name) && !declares_table(*m, name)) {
        return std::unexpected(invalid(where + ": module '" + std::string(owner) +
                                       "' is in the set and declares no array or table '" + std::string(name) +
                                       "'"));
    }
    return {};
}

// An access that binds nothing and orders nothing: an optional read of a
// module the set does not hold.
[[nodiscard]] bool absent_access(const QuantityAccess& qa, std::span<const ModuleDesc> modules) noexcept {
    std::string_view owner;
    std::string_view name;
    return qa.optional && split_quantity(qa.quantity, owner, name) && find_module(modules, owner) == nullptr;
}

// What one declared access binds (module.hpp's CompiledBinding): the array a
// "<module>.<array>" quantity names, or the table a "<module>.<table>" one
// does, whichever module owns it; or nothing. Core quantities are checked
// first, as known_quantity() checks them. Arrays and tables share one
// namespace (compile_tables), so at most one of the two loops can match.
[[nodiscard]] CompiledBinding binding_of(const QuantityAccess& qa, const std::vector<CompiledArray>& arrays,
                                         const std::vector<CompiledTable>& tables) noexcept {
    std::string_view owner;
    std::string_view name;
    if (is_core_quantity(qa.quantity) || qa.quantity.starts_with(kFieldPrefix) ||
        !split_quantity(qa.quantity, owner, name)) {
        return {};
    }
    for (std::size_t i = 0; i < arrays.size(); ++i) {
        if (arrays[i].module == owner && arrays[i].name == name) {
            return CompiledBinding{BindingKind::array, static_cast<uint32_t>(i)};
        }
    }
    for (std::size_t i = 0; i < tables.size(); ++i) {
        if (tables[i].module == owner && tables[i].name == name) {
            return CompiledBinding{BindingKind::table, static_cast<uint32_t>(i)};
        }
    }
    return {};
}

[[nodiscard]] bool is_core_array(std::string_view name) noexcept {
    for (const std::string_view core : kCoreArrays) {
        if (name == core) return true;
    }
    return false;
}

[[nodiscard]] bool is_legacy_array(std::string_view name) noexcept {
    for (const std::string_view legacy : kLegacyWalkArrays) {
        if (name == legacy) return true;
    }
    return false;
}

[[nodiscard]] bool is_attached(Extent e) noexcept {
    return e == Extent::per_body || e == Extent::per_element || e == Extent::per_sensor;
}

[[nodiscard]] std::string_view extent_name(Extent e) noexcept {
    switch (e) {
        case Extent::per_world: return "per_world";
        case Extent::per_body: return "per_body";
        case Extent::per_element: return "per_element";
        case Extent::per_sensor: return "per_sensor";
        case Extent::per_row: return "per_row";
    }
    return "unknown";
}

// THE MODULE ARRAY TABLE (stage 4), in registration order: the legacy
// modules' arrays first, then every other module's -- each pass in set order,
// and each module's arrays in declaration order. Owners resolve by name after
// both passes, so an owner may be declared by any module in the set.
[[nodiscard]] Result<std::vector<CompiledArray>> compile_arrays(std::span<const ModuleDesc> modules) {
    std::vector<CompiledArray> out;
    std::vector<std::string_view> owners;  // parallel to `out`: each array's declared owner
    std::map<std::string, std::size_t, std::less<>> by_name;
    for (const bool legacy : {true, false}) {
        for (const ModuleDesc& mod : modules) {
            if (mod.legacy_walk != legacy) continue;
            for (const ArrayDecl& a : mod.state) {
                const std::string where =
                    "array '" + std::string(a.name) + "' (module '" + std::string(mod.name) + "')";
                if (a.name.empty() || a.name.find('.') != std::string_view::npos) {
                    return std::unexpected(invalid(where + ": an array needs a name with no '.'"));
                }
                if (is_core_array(a.name)) {
                    return std::unexpected(invalid(where + ": the core registers '" + std::string(a.name) +
                                                   "' itself; a module cannot declare it"));
                }
                if (const auto it = by_name.find(a.name); it != by_name.end()) {
                    if (out[it->second].module == mod.name) {
                        return std::unexpected(invalid(where + ": declared twice"));
                    }
                    return std::unexpected(invalid("array '" + std::string(a.name) + "' is declared by module '" +
                                                   out[it->second].module + "' and module '" +
                                                   std::string(mod.name) + "'; an array has one owner"));
                }
                if (a.elem_size == 0) {
                    return std::unexpected(invalid(where + ": elem_size is 0; a row has a size"));
                }
                if (static_cast<uint8_t>(a.extent) > static_cast<uint8_t>(Extent::per_row)) {
                    return std::unexpected(invalid(where + ": unknown extent"));
                }
                if (a.extent == Extent::per_row && a.depth == 0) {
                    return std::unexpected(invalid(where + ": a per_row array needs a depth of at least 1"));
                }
                if (a.extent != Extent::per_row && (!a.owner.empty() || a.depth != 1)) {
                    return std::unexpected(invalid(where + ": only a per_row array takes an owner or a depth"));
                }
                // ROW INITS (Task 4). An attached row (per_body, per_element or
                // per_sensor) is written only by its module's init, at the
                // boundary attach_row() queued it for, so it needs one; a
                // slot-allocated one starts with the uint32_t body_slot the
                // despawn cascade reads. A per_row array may carry an init too:
                // the plan's vehicle-row hook (Task 7) initializes rows owned
                // by rotors. A per_world row is never queued, so an init, a
                // validate or a spawn size there would never be used.
                if (a.spawn_size > kMaxSpawnBytes) {
                    return std::unexpected(invalid(where + ": spawn_size " + std::to_string(a.spawn_size) +
                                                   " exceeds kMaxSpawnBytes (" + std::to_string(kMaxSpawnBytes) +
                                                   "); the structural queue carries the record by value"));
                }
                if (is_attached(a.extent)) {
                    if (a.init == nullptr) {
                        return std::unexpected(invalid(where + ": an attached array (" +
                                                       std::string(extent_name(a.extent)) +
                                                       ") needs an init; attach_row runs it at the step boundary"));
                    }
                    if (a.extent != Extent::per_body && a.elem_size < sizeof(uint32_t)) {
                        return std::unexpected(invalid(where + ": a " + std::string(extent_name(a.extent)) +
                                                       " row starts with its uint32_t body_slot, so it is at "
                                                       "least 4 bytes"));
                    }
                } else if (a.extent == Extent::per_world &&
                           (a.spawn_size != 0 || a.init != nullptr || a.validate != nullptr)) {
                    return std::unexpected(invalid(where + ": a per_world row is never attached or queued, so it "
                                                           "takes no spawn size, init or validate"));
                }
                if (mod.legacy_walk && !is_legacy_array(a.name)) {
                    return std::unexpected(invalid(
                        where + ": the module carries the legacy walk marker, which registers before "
                                "replay_config and is only for the arrays that predate it (drag_bodies, dryden, "
                                "imu_sensors, imu_ring, rotors)"));
                }
                if (!mod.legacy_walk && is_legacy_array(a.name)) {
                    return std::unexpected(invalid(
                        where + ": an array that predates replay_config registers before it, so its module "
                                "must carry the legacy walk marker; without it the walk would move"));
                }
                by_name.emplace(std::string(a.name), out.size());
                out.push_back(CompiledArray{std::string(mod.name), std::string(a.name), a.elem_size, a.extent,
                                            kNoArray, a.depth, mod.legacy_walk, a.spawn_size, a.init,
                                            a.validate});
                owners.push_back(a.owner);
            }
        }
    }
    for (std::size_t i = 0; i < out.size(); ++i) {
        CompiledArray& c = out[i];
        if (c.extent != Extent::per_row) continue;
        const std::string where = "array '" + c.name + "' (module '" + c.module + "')";
        const auto it = by_name.find(owners[i]);
        if (it == by_name.end()) {
            return std::unexpected(invalid(where + ": its owner '" + std::string(owners[i]) +
                                           "' is not an array any module in the set declares"));
        }
        const Extent owner_extent = out[it->second].extent;
        if (owner_extent == Extent::per_world || owner_extent == Extent::per_row) {
            return std::unexpected(invalid(where + ": its owner '" + std::string(owners[i]) + "' is " +
                                           std::string(extent_name(owner_extent)) +
                                           "; a per_row array's owner holds rows (per_body, per_element or "
                                           "per_sensor)"));
        }
        c.owner = static_cast<uint32_t>(it->second);
    }
    return out;
}

// THE STREAM TABLE (stage 4, Task 5), in set order and then each module's
// declaration order: the order create() and reseed() derive the streams in.
// For the standard set that is dryden, imu, gnss -- the order reseed()'s
// hand-written blocks ran in before the declarations replaced them.
[[nodiscard]] Result<std::vector<CompiledStream>> compile_streams(std::span<const ModuleDesc> modules,
                                                                  const std::vector<CompiledArray>& arrays) {
    std::vector<CompiledStream> out;
    std::map<std::string, std::size_t, std::less<>> by_tag;  // tag -> its entry in `out`
    for (const ModuleDesc& mod : modules) {
        for (const StreamDecl& s : mod.streams) {
            if (s.tag.empty()) {
                return std::unexpected(invalid("module '" + std::string(mod.name) + "': the stream on array '" +
                                               std::string(s.array) + "' needs a tag"));
            }
            const std::string where = "stream '" + std::string(s.tag) + "' (module '" + std::string(mod.name) + "')";
            if (s.tag == kWorldSeedDomainTag) {
                return std::unexpected(invalid(where + ": the tag is reserved; every world's own seed is derived "
                                                       "under it (kWorldSeedDomainTag)"));
            }
            if (const auto it = by_tag.find(s.tag); it != by_tag.end()) {
                if (out[it->second].module == mod.name) {
                    return std::unexpected(invalid(where + ": declared twice"));
                }
                return std::unexpected(invalid("stream '" + std::string(s.tag) + "' is declared by module '" +
                                               out[it->second].module + "' and module '" + std::string(mod.name) +
                                               "'; a tag names one stream, or two modules would draw the same "
                                               "numbers"));
            }
            if (s.reseed == nullptr) {
                return std::unexpected(invalid(where + ": no reseed function, so reseed() could not re-derive it"));
            }
            const auto array = std::ranges::find(arrays, s.array, &CompiledArray::name);
            if (array == arrays.end() || array->module != mod.name) {
                return std::unexpected(invalid(where + ": its array '" + std::string(s.array) +
                                               "' is not one this module declares"));
            }
            // Liveness is the arena's: a per_world row is always live, and a
            // slot-allocated row while its map names the world. A per_body or
            // per_row row has no liveness of its own to walk.
            if (array->extent != Extent::per_world && array->extent != Extent::per_element &&
                array->extent != Extent::per_sensor) {
                return std::unexpected(invalid(where + ": its array '" + array->name + "' is " +
                                               std::string(extent_name(array->extent)) +
                                               "; a stream lives in a per_world, per_element or per_sensor array, "
                                               "whose liveness the arena keeps"));
            }
            by_tag.emplace(std::string(s.tag), out.size());
            out.push_back(CompiledStream{std::string(mod.name), std::string(s.tag),
                                         static_cast<uint32_t>(array - arrays.begin()), s.reseed});
        }
    }
    return out;
}

// THE CONFIGURATION TABLES (Task 7; module.hpp's ConfigTableDecl), in set
// order and then each module's declaration order: the order Simulation builds
// them in. A table's name shares the array namespace, so "<module>.<name>"
// names one thing, whichever kind it is.
[[nodiscard]] Result<std::vector<CompiledTable>> compile_tables(std::span<const ModuleDesc> modules,
                                                                const std::vector<CompiledArray>& arrays) {
    std::vector<CompiledTable> out;
    for (const ModuleDesc& mod : modules) {
        for (const ConfigTableDecl& t : mod.tables) {
            if (t.name.empty() || t.name.find('.') != std::string_view::npos) {
                return std::unexpected(invalid("module '" + std::string(mod.name) + "': table '" +
                                               std::string(t.name) + "': a table needs a name with no '.'"));
            }
            const std::string where = "table '" + std::string(t.name) + "' (module '" + std::string(mod.name) + "')";
            if (is_core_array(t.name)) {
                return std::unexpected(invalid(where + ": '" + std::string(t.name) +
                                               "' is a core array's name; a table shares the array namespace"));
            }
            if (const auto array = std::ranges::find(arrays, t.name, &CompiledArray::name); array != arrays.end()) {
                return std::unexpected(invalid(where + ": module '" + array->module +
                                               "' declares an array of that name; a table shares the array "
                                               "namespace, so <module>.<name> names one thing"));
            }
            if (const auto twice = std::ranges::find(out, t.name, &CompiledTable::name); twice != out.end()) {
                if (twice->module == mod.name) return std::unexpected(invalid(where + ": declared twice"));
                return std::unexpected(invalid("table '" + std::string(t.name) + "' is declared by module '" +
                                               twice->module + "' and module '" + std::string(mod.name) +
                                               "'; a table has one owner"));
            }
            if (t.build == nullptr) {
                return std::unexpected(
                    invalid(where + ": no build function, so register_model() could not build it"));
            }
            out.push_back(CompiledTable{std::string(mod.name), std::string(t.name), t.build});
        }
    }
    return out;
}

// The built-in fields' fixed shapes (module.hpp). The GPU row depends on them.
struct BuiltinField {
    std::string_view name;
    FieldKind kind;
    std::string_view unit;
    uint32_t offset;
};
constexpr BuiltinField kBuiltinFields[] = {
    {"gravity", FieldKind::vec3, "m/s^2", kFieldGravityOffset},
    {"density", FieldKind::scalar, "kg/m^3", kFieldDensityOffset},
    {"wind", FieldKind::vec3, "m/s", kFieldWindOffset},
};

[[nodiscard]] const BuiltinField* builtin_field(std::string_view name) noexcept {
    for (const BuiltinField& b : kBuiltinFields) {
        if (b.name == name) return &b;
    }
    return nullptr;
}

[[nodiscard]] uint64_t fold_byte(uint64_t h, uint8_t b) noexcept {
    h ^= b;
    return h * rng::kFnv1aPrime;
}

[[nodiscard]] uint64_t fold_str(uint64_t h, std::string_view s) noexcept {
    for (const char c : s) h = fold_byte(h, static_cast<uint8_t>(c));
    return h;
}

[[nodiscard]] uint64_t fold_u32_le(uint64_t h, uint32_t v) noexcept {
    for (int i = 0; i < 4; ++i) h = fold_byte(h, static_cast<uint8_t>(v >> (8 * i)));
    return h;
}

}  // namespace

Result<CompiledSchedule> compile_schedule(std::span<const ModuleDesc> modules) {
    std::vector<Node> nodes;
    std::map<std::string, std::size_t, std::less<>> by_name;

    // THE FIELD REGISTRY (stage 3), built first so that a pass's "field.<name>"
    // access resolves against it. One provider per field: this stage has one
    // region, the whole world (spec section 6).
    FieldOwners field_owner;
    for (std::size_t m = 0; m < modules.size(); ++m) {
        const ModuleDesc& mod = modules[m];
        for (const FieldDecl& f : mod.fields) {
            const std::string where = "field '" + std::string(f.name) + "' (module '" + std::string(mod.name) + "')";
            if (f.name.empty() || f.name.find('.') != std::string_view::npos) {
                return std::unexpected(
                    invalid("module '" + std::string(mod.name) + "': a field needs a name with no '.'"));
            }
            if (static_cast<uint8_t>(f.kind) > static_cast<uint8_t>(FieldKind::bands)) {
                return std::unexpected(invalid(where + ": unknown kind"));
            }
            if (f.kind == FieldKind::bands && (f.bands == 0 || f.bands > kMaxFieldBands)) {
                return std::unexpected(invalid(where + ": a band field needs 1 to " +
                                               std::to_string(kMaxFieldBands) + " bands, not " +
                                               std::to_string(f.bands)));
            }
            if (f.kind != FieldKind::bands && f.bands != 0) {
                return std::unexpected(invalid(where + ": only a band field takes a band count"));
            }
            if (const BuiltinField* b = builtin_field(f.name); b != nullptr && (f.kind != b->kind || f.unit != b->unit)) {
                return std::unexpected(invalid(where + " is built in: it must be " +
                                               (b->kind == FieldKind::vec3 ? "vec3" : "scalar") + " in " +
                                               std::string(b->unit)));
            }
            if (const auto it = field_owner.find(f.name); it != field_owner.end()) {
                return std::unexpected(invalid("field '" + std::string(f.name) + "' is declared by module '" +
                                               std::string(modules[it->second].name) + "' and module '" +
                                               std::string(mod.name) + "'; a field has one provider"));
            }
            field_owner.emplace(std::string(f.name), m);
        }
    }

    for (std::size_t m = 0; m < modules.size(); ++m) {
        const ModuleDesc& mod = modules[m];
        if (mod.name.empty() || mod.name.find('.') != std::string_view::npos) {
            return std::unexpected(invalid("module " + std::to_string(m) + ": a module needs a name with no '.'"));
        }
        if (mod.name == "field") {
            return std::unexpected(invalid("module 'field': the name is reserved for the field quantities"));
        }
        for (std::size_t k = 0; k < m; ++k) {
            if (modules[k].name == mod.name) {
                return std::unexpected(invalid("module '" + std::string(mod.name) + "' appears twice in the set"));
            }
        }
        for (std::size_t d = 0; d < mod.passes.size(); ++d) {
            const PassDecl& p = mod.passes[d];
            std::string full = std::string(mod.name) + "." + std::string(p.name);
            // A '.' in a pass name would make "<module>.<pass>", the name an
            // edge uses, ambiguous.
            if (p.name.empty() || p.name.find('.') != std::string_view::npos || p.cpu == nullptr) {
                return std::unexpected(invalid(full + ": a pass needs a name with no '.' and a CPU function"));
            }
            if (static_cast<std::size_t>(p.phase) >= kPhaseCount) {
                return std::unexpected(invalid(full + ": unknown phase"));
            }
            if (p.gpu != compute::GpuRecipe::none && p.cpu != builtin_cpu_for(p.gpu)) {
                return std::unexpected(
                    invalid("pass '" + full + "' names a built-in GPU kernel but carries another CPU function"));
            }
            if (by_name.contains(full)) {
                return std::unexpected(invalid(full + ": declared twice"));
            }
            for (const QuantityAccess& qa : p.access) {
                if (qa.optional) {
                    // A read, so never a field write: nothing below applies.
                    if (Result<void> ok = check_optional(full, qa, modules); !ok) return std::unexpected(ok.error());
                    continue;
                }
                if (!known_quantity(qa.quantity, modules, field_owner)) {
                    return std::unexpected(invalid(full + ": unknown quantity '" + std::string(qa.quantity) +
                                                   "' (a core quantity, field.<name> for a field a module "
                                                   "declares, or <module>.<name> for a module in the set, "
                                                   "where <name> is one of its arrays or tables if it declares "
                                                   "any)"));
                }
                // A table changes only in register_model() (Task 7), so no
                // pass writes or accumulates it -- its own module's included.
                if (qa.access != Access::read && names_table(qa.quantity, modules)) {
                    return std::unexpected(invalid(full + (qa.access == Access::write ? ": writes" : ": accumulates") +
                                                   " table '" + std::string(qa.quantity) +
                                                   "'; a configuration table changes only in register_model(), "
                                                   "so a pass may only read it"));
                }
                if (const std::string_view field = field_name_of(qa.quantity);
                    !field.empty() && qa.access != Access::read) {
                    if (field_owner.find(field)->second != m) {
                        return std::unexpected(invalid(full + ": writes " + std::string(qa.quantity) +
                                                       ", which module '" +
                                                       std::string(modules[field_owner.find(field)->second].name) +
                                                       "' provides; only a field's provider writes it"));
                    }
                    // A provider writes in Fields, before any reader (spec
                    // section 6). A later write would leave an earlier-phase
                    // reader the previous substep's row: zeros on the first
                    // step, another timeline's after restore().
                    if (p.phase != Phase::fields) {
                        return std::unexpected(invalid(full + ": writes " + std::string(qa.quantity) +
                                                       " outside the Fields phase; a provider writes its field "
                                                       "in Fields, before any reader"));
                    }
                }
            }
            by_name.emplace(full, nodes.size());
            nodes.push_back(Node{m, d, &p, std::move(full)});
        }
    }

    Result<std::vector<CompiledArray>> arrays = compile_arrays(modules);
    if (!arrays) return std::unexpected(arrays.error());
    Result<std::vector<CompiledStream>> streams = compile_streams(modules, *arrays);
    if (!streams) return std::unexpected(streams.error());
    Result<std::vector<CompiledTable>> tables = compile_tables(modules, *arrays);
    if (!tables) return std::unexpected(tables.error());

    // Every declared field has a pass in its provider module that writes it.
    for (const auto& [field, m] : field_owner) {
        const std::string q = std::string(kFieldPrefix) + field;
        bool written = false;
        for (const PassDecl& p : modules[m].passes) {
            for (const QuantityAccess& qa : p.access) written = written || (qa.quantity == q && qa.access == Access::write);
        }
        if (!written) {
            return std::unexpected(invalid("module '" + std::string(modules[m].name) + "' provides " + q +
                                           " but none of its passes writes it"));
        }
    }

    // The registry's layout: the built-ins at their fixed places, then every
    // other field in set order, from the end of the built-in prefix.
    std::vector<CompiledField> fields;
    for (const BuiltinField& b : kBuiltinFields) {
        if (const auto it = field_owner.find(b.name); it != field_owner.end()) {
            fields.push_back(CompiledField{std::string(b.name), b.kind, b.kind == FieldKind::vec3 ? 3u : 1u,
                                           b.offset, std::string(b.unit)});
        }
    }
    uint32_t field_stride = kFieldBuiltinFloats;
    for (const ModuleDesc& mod : modules) {
        for (const FieldDecl& f : mod.fields) {
            if (builtin_field(f.name) != nullptr) continue;
            const uint32_t count = f.kind == FieldKind::bands ? f.bands : (f.kind == FieldKind::vec3 ? 3u : 1u);
            fields.push_back(CompiledField{std::string(f.name), f.kind, count, field_stride, std::string(f.unit)});
            field_stride += count;
        }
    }

    const std::size_t n = nodes.size();
    std::vector<std::vector<std::size_t>> preds(n);              // preds[i]: must run before i
    std::vector<std::vector<bool>> edge_path(n, std::vector<bool>(n, false));  // by explicit edges only

    for (std::size_t i = 0; i < n; ++i) {
        for (const std::string_view target : nodes[i].pass->after) {
            const auto it = by_name.find(target);
            if (it == by_name.end()) {
                return std::unexpected(invalid(nodes[i].full + ": edge to '" + std::string(target) +
                                               "', which no module in the set declares"));
            }
            const Node& t = nodes[it->second];
            if (t.pass->phase > nodes[i].pass->phase) {
                return std::unexpected(invalid(nodes[i].full + ": edge to '" + t.full + "', which runs in a later phase"));
            }
            if (t.pass->phase == nodes[i].pass->phase) {
                preds[i].push_back(it->second);
                edge_path[it->second][i] = true;
            }
        }
        // `before`, the mirror (Task 6): the target runs after this pass. A
        // target in a later phase already does; one in an earlier phase never
        // can.
        for (const std::string_view target : nodes[i].pass->before) {
            const auto it = by_name.find(target);
            if (it == by_name.end()) {
                return std::unexpected(invalid(nodes[i].full + ": before edge to '" + std::string(target) +
                                               "', which no module in the set declares"));
            }
            const Node& t = nodes[it->second];
            if (t.pass->phase < nodes[i].pass->phase) {
                return std::unexpected(
                    invalid(nodes[i].full + ": before edge to '" + t.full + "', which runs in an earlier phase"));
            }
            if (t.pass->phase == nodes[i].pass->phase) {
                preds[it->second].push_back(i);
                edge_path[i][it->second] = true;
            }
        }
    }
    for (std::size_t k = 0; k < n; ++k) {
        for (std::size_t i = 0; i < n; ++i) {
            if (!edge_path[i][k]) continue;
            for (std::size_t j = 0; j < n; ++j) {
                if (edge_path[k][j]) edge_path[i][j] = true;
            }
        }
    }

    for (std::size_t i = 0; i < n; ++i) {
        for (std::size_t j = i + 1; j < n; ++j) {
            const PassDecl& a = *nodes[i].pass;
            const PassDecl& b = *nodes[j].pass;
            if (a.phase != b.phase) continue;
            if (a.placement != b.placement) {
                if (a.placement < b.placement) {
                    preds[j].push_back(i);
                } else {
                    preds[i].push_back(j);
                }
            }
            for (const QuantityAccess& qa : a.access) {
                // An optional read of an absent module is no hazard (Task 6).
                // Nothing else can name that module -- it would be an unknown
                // quantity -- so this only says so rather than relying on it.
                if (absent_access(qa, modules)) continue;
                for (const QuantityAccess& qb : b.access) {
                    if (absent_access(qb, modules)) continue;
                    if (qa.quantity != qb.quantity) continue;
                    if (needs_edge(qa.access, qb.access)) {
                        // A different placement already orders the pair.
                        if (a.placement == b.placement && !edge_path[i][j] && !edge_path[j][i]) {
                            return std::unexpected(invalid(nodes[i].full + " and " + nodes[j].full + " both write '" +
                                                           std::string(qa.quantity) +
                                                           "' with no edge between them"));
                        }
                    } else if (qa.access == Access::read && qb.access != Access::read) {
                        preds[i].push_back(j);
                    } else if (qb.access == Access::read && qa.access != Access::read) {
                        preds[j].push_back(i);
                    }
                }
            }
        }
    }

    std::vector<std::size_t> indegree(n, 0);
    std::vector<std::vector<std::size_t>> succs(n);
    for (std::size_t i = 0; i < n; ++i) {
        for (const std::size_t p : preds[i]) {
            succs[p].push_back(i);
            ++indegree[i];
        }
    }
    const auto key = [&](std::size_t i) {
        return std::tuple(static_cast<int>(nodes[i].pass->phase), nodes[i].module, nodes[i].decl);
    };

    CompiledSchedule out;
    out.passes.reserve(n);
    std::vector<bool> done(n, false);
    for (std::size_t emitted = 0; emitted < n; ++emitted) {
        std::size_t best = n;
        for (std::size_t i = 0; i < n; ++i) {
            if (!done[i] && indegree[i] == 0 && (best == n || key(i) < key(best))) best = i;
        }
        if (best == n) {
            std::string names;
            for (std::size_t i = 0; i < n; ++i) {
                if (!done[i]) names += " " + nodes[i].full;
            }
            return std::unexpected(invalid("these passes form a cycle:" + names));
        }
        done[best] = true;
        for (const std::size_t s : succs[best]) --indegree[s];
        const Node& nd = nodes[best];
        // One binding per declared access, in declaration order, so a pass
        // indexes SubstepContext::state by its own access list.
        std::vector<CompiledBinding> bindings;
        bindings.reserve(nd.pass->access.size());
        for (const QuantityAccess& qa : nd.pass->access) bindings.push_back(binding_of(qa, *arrays, *tables));
        out.passes.push_back(CompiledPass{std::string(modules[nd.module].name), std::string(nd.pass->name),
                                          nd.pass->phase, nd.pass->cpu, nd.pass->gpu, std::move(bindings)});
    }

    // The recipe is not folded: it is bound to the CPU function (checked above),
    // and the identity spells names, versions and order, not functions.
    uint64_t h = rng::kFnv1aOffsetBasis;
    for (const ModuleDesc& m : modules) {
        h = fold_str(h, m.name);
        h = fold_byte(h, 0);
        h = fold_u32_le(h, m.version);
    }
    h = fold_byte(h, 1);
    for (const CompiledPass& p : out.passes) {
        h = fold_str(h, p.module);
        h = fold_byte(h, '.');
        h = fold_str(h, p.pass);
        h = fold_byte(h, 0);
        h = fold_byte(h, static_cast<uint8_t>(p.phase));
    }
    out.identity = h;
    out.fields = std::move(fields);
    out.field_stride = field_stride;
    out.arrays = std::move(*arrays);
    out.streams = std::move(*streams);
    out.tables = std::move(*tables);
    // The vehicle-spawn hooks, in set order (Task 7): the order spawn() calls
    // them in, checking every request against the vehicle.
    for (const ModuleDesc& mod : modules) {
        if (mod.vehicle_rows != nullptr) {
            out.vehicle_rows.push_back(CompiledVehicleRows{std::string(mod.name), mod.vehicle_rows});
        }
    }
    return out;
}

std::vector<compute::GpuPass> gpu_passes(const CompiledSchedule& schedule) {
    std::vector<compute::GpuPass> out;
    out.reserve(schedule.passes.size());
    for (const CompiledPass& p : schedule.passes) out.push_back({p.module + "." + p.pass, p.gpu});
    return out;
}

std::vector<std::string> walk_order(const CompiledSchedule& schedule) {
    // kCoreArrays is the three head arrays, then replay_config.
    const std::span<const std::string_view> core(kCoreArrays);
    std::vector<std::string> out(core.begin(), core.end() - 1);
    for (const CompiledArray& a : schedule.arrays) {
        if (a.legacy_walk) out.push_back(a.name);
    }
    out.emplace_back(core.back());
    for (const CompiledArray& a : schedule.arrays) {
        if (!a.legacy_walk) out.push_back(a.name);
    }
    return out;
}

}  // namespace spade::modules
