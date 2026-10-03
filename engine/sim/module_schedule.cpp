#include "sim/module.hpp"

#include <functional>
#include <map>
#include <string>
#include <tuple>
#include <utility>

#include "core/rng.hpp"

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

[[nodiscard]] bool known_quantity(std::string_view q, std::span<const ModuleDesc> modules,
                                  const FieldOwners& fields) noexcept {
    for (const std::string_view core : kCoreQuantities) {
        if (q == core) return true;
    }
    if (q.starts_with(kFieldPrefix)) return fields.contains(field_name_of(q));
    const std::size_t dot = q.find('.');
    if (dot == std::string_view::npos || dot == 0 || dot + 1 == q.size()) return false;
    const std::string_view owner = q.substr(0, dot);
    for (const ModuleDesc& m : modules) {
        if (m.name == owner) return true;
    }
    return false;
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
            if (p.name.empty() || p.cpu == nullptr) {
                return std::unexpected(invalid(full + ": a pass needs a name and a CPU function"));
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
                if (!known_quantity(qa.quantity, modules, field_owner)) {
                    return std::unexpected(invalid(full + ": unknown quantity '" + std::string(qa.quantity) +
                                                   "' (a core quantity, field.<name> for a field a module "
                                                   "declares, or <module>.<name> for a module in the set)"));
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
                for (const QuantityAccess& qb : b.access) {
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
        out.passes.push_back(CompiledPass{std::string(modules[nd.module].name), std::string(nd.pass->name),
                                          nd.pass->phase, nd.pass->cpu, nd.pass->gpu});
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
    return out;
}

std::vector<compute::GpuPass> gpu_passes(const CompiledSchedule& schedule) {
    std::vector<compute::GpuPass> out;
    out.reserve(schedule.passes.size());
    for (const CompiledPass& p : schedule.passes) out.push_back({p.module + "." + p.pass, p.gpu});
    return out;
}

}  // namespace spade::modules
