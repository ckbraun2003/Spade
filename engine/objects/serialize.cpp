// serialize.cpp -- see serialize.hpp for the two format decisions that matter
// (components by registered name; parent links by index, never by name).

#include "objects/serialize.hpp"

#include <array>
#include <cstdint>
#include <optional>
#include <unordered_map>
#include <vector>

#include <nlohmann/json.hpp>

namespace spade::objects {
namespace {

using nlohmann::json;

[[nodiscard]] Error bad(std::string context) {
    return Error{Code::invalid_argument, std::move(context)};
}

// ---------------------------------------------------------------------------
// Reading primitives. Every one names the offending key, because the whole
// value of erroring instead of skipping is that the message tells the author
// which line to look at.
// ---------------------------------------------------------------------------

[[nodiscard]] Result<const json*> object_field(const json& j, const char* key,
                                               std::string_view what) {
    const auto it = j.find(key);
    if (it == j.end()) return std::unexpected(bad(std::string(what) + ": missing '" + key + "'"));
    return &*it;
}

[[nodiscard]] Result<float> float_at(const json& j, std::string_view what) {
    if (!j.is_number()) return std::unexpected(bad(std::string(what) + ": expected a number"));
    return j.get<float>();
}

[[nodiscard]] Result<uint32_t> u32_field(const json& j, const char* key, std::string_view what) {
    const Result<const json*> f = object_field(j, key, what);
    if (!f) return std::unexpected(f.error());
    if (!(*f)->is_number_unsigned()) {
        return std::unexpected(
            bad(std::string(what) + "." + key + ": expected a non-negative integer"));
    }
    return (*f)->get<uint32_t>();
}

[[nodiscard]] Result<float> float_field(const json& j, const char* key, std::string_view what) {
    const Result<const json*> f = object_field(j, key, what);
    if (!f) return std::unexpected(f.error());
    return float_at(**f, std::string(what) + "." + key);
}

[[nodiscard]] Result<bool> bool_field(const json& j, const char* key, std::string_view what) {
    const Result<const json*> f = object_field(j, key, what);
    if (!f) return std::unexpected(f.error());
    if (!(*f)->is_boolean()) {
        return std::unexpected(bad(std::string(what) + "." + key + ": expected a boolean"));
    }
    return (*f)->get<bool>();
}

// A wrong-length array is the failure this exists for: [1,2] read as a vec3
// would otherwise leave z holding whatever the default was, which is a
// silently different scene rather than a rejected file.
template <std::size_t N>
[[nodiscard]] Result<std::array<float, N>> floats_field(const json& j, const char* key,
                                                        std::string_view what) {
    const Result<const json*> f = object_field(j, key, what);
    if (!f) return std::unexpected(f.error());
    const json& node = **f;
    if (!node.is_array() || node.size() != N) {
        return std::unexpected(bad(std::string(what) + "." + key + ": expected " +
                                   std::to_string(N) + " numbers, got " +
                                   (node.is_array() ? std::to_string(node.size())
                                                    : std::string("a non-array"))));
    }
    std::array<float, N> out{};
    for (std::size_t i = 0; i < N; ++i) {
        const Result<float> v = float_at(node[i], std::string(what) + "." + key);
        if (!v) return std::unexpected(v.error());
        out[i] = *v;
    }
    return out;
}

// ---------------------------------------------------------------------------
// Components. Keys come from component_type_name(), so the vocabulary this
// file writes is the enum's, not a second copy of it.
// ---------------------------------------------------------------------------

template <class T>
[[nodiscard]] std::string_view key_of() {
    return component_type_name(component_type_id<T>());
}

void write_components(const ObjectGraph& g, ObjectId id, json& out) {
    // Read through component<T>() -- which consults the mask -- so a component
    // that was detached is absent here rather than written from a store slot
    // that still holds its old value.
    if (g.component<TransformComponent>(id)) {
        out[std::string(key_of<TransformComponent>())] = json::object();
    }
    if (const auto* c = g.component<BodyComponent>(id)) {
        out[std::string(key_of<BodyComponent>())] = {{"world_index", c->world_index},
                                                     {"body_slot", c->body_slot}};
    }
    if (const auto* c = g.component<MeshComponent>(id)) {
        out[std::string(key_of<MeshComponent>())] = {{"draw_item", c->draw_item}};
    }
    if (const auto* c = g.component<MaterialComponent>(id)) {
        out[std::string(key_of<MaterialComponent>())] = {{"material_index", c->material_index}};
    }
    if (const auto* c = g.component<ColliderComponent>(id)) {
        out[std::string(key_of<ColliderComponent>())] = {{"sphere_radius", c->sphere_radius}};
    }
    if (const auto* c = g.component<SensorComponent>(id)) {
        out[std::string(key_of<SensorComponent>())] = {{"sensor_slot", c->sensor_slot}};
    }
    if (const auto* c = g.component<ForceElementComponent>(id)) {
        out[std::string(key_of<ForceElementComponent>())] = {{"element_slot", c->element_slot}};
    }
    if (const auto* c = g.component<CameraComponent>(id)) {
        out[std::string(key_of<CameraComponent>())] = {{"fov_degrees", c->fov_degrees},
                                                       {"near_plane", c->near_plane},
                                                       {"far_plane", c->far_plane},
                                                       {"active", c->active}};
    }
    if (const auto* c = g.component<BehaviorComponent>(id)) {
        out[std::string(key_of<BehaviorComponent>())] = {{"behavior_index", c->behavior_index}};
    }
    if (const auto* c = g.component<FluidComponent>(id)) {
        out[std::string(key_of<FluidComponent>())] = {{"rest_density", c->rest_density},
                                                      {"stiffness", c->stiffness},
                                                      {"viscosity", c->viscosity}};
    }
}

// Dispatch on the REGISTERED id rather than on the string, so an added
// component type that forgets a case here is a missing switch arm the compiler
// warns about (/W4 /WX) rather than a name that silently falls through.
[[nodiscard]] Result<void> read_component(ObjectGraph& g, ObjectId id, ComponentTypeId type,
                                          const json& body, std::string_view what) {
    const std::string where = std::string(what) + "." + std::string(component_type_name(type));
    if (!body.is_object()) return std::unexpected(bad(where + ": expected an object"));

    switch (type) {
        case ComponentTypeId::transform:
            g.attach<TransformComponent>(id, TransformComponent{});
            return {};
        case ComponentTypeId::body: {
            const Result<uint32_t> w = u32_field(body, "world_index", where);
            if (!w) return std::unexpected(w.error());
            const Result<uint32_t> s = u32_field(body, "body_slot", where);
            if (!s) return std::unexpected(s.error());
            g.attach<BodyComponent>(id, BodyComponent{.world_index = *w, .body_slot = *s});
            return {};
        }
        case ComponentTypeId::mesh: {
            const Result<uint32_t> d = u32_field(body, "draw_item", where);
            if (!d) return std::unexpected(d.error());
            g.attach<MeshComponent>(id, MeshComponent{.draw_item = *d});
            return {};
        }
        case ComponentTypeId::material: {
            const Result<uint32_t> m = u32_field(body, "material_index", where);
            if (!m) return std::unexpected(m.error());
            g.attach<MaterialComponent>(id, MaterialComponent{.material_index = *m});
            return {};
        }
        case ComponentTypeId::collider: {
            const Result<float> r = float_field(body, "sphere_radius", where);
            if (!r) return std::unexpected(r.error());
            g.attach<ColliderComponent>(id, ColliderComponent{.sphere_radius = *r});
            return {};
        }
        case ComponentTypeId::sensor: {
            const Result<uint32_t> s = u32_field(body, "sensor_slot", where);
            if (!s) return std::unexpected(s.error());
            g.attach<SensorComponent>(id, SensorComponent{.sensor_slot = *s});
            return {};
        }
        case ComponentTypeId::force_element: {
            const Result<uint32_t> e = u32_field(body, "element_slot", where);
            if (!e) return std::unexpected(e.error());
            g.attach<ForceElementComponent>(id, ForceElementComponent{.element_slot = *e});
            return {};
        }
        case ComponentTypeId::camera: {
            const Result<float> fov = float_field(body, "fov_degrees", where);
            if (!fov) return std::unexpected(fov.error());
            const Result<float> np = float_field(body, "near_plane", where);
            if (!np) return std::unexpected(np.error());
            const Result<float> fp = float_field(body, "far_plane", where);
            if (!fp) return std::unexpected(fp.error());
            const Result<bool> on = bool_field(body, "active", where);
            if (!on) return std::unexpected(on.error());
            g.attach<CameraComponent>(id, CameraComponent{.fov_degrees = *fov,
                                                          .near_plane = *np,
                                                          .far_plane = *fp,
                                                          .active = *on});
            return {};
        }
        case ComponentTypeId::behavior: {
            const Result<uint32_t> b = u32_field(body, "behavior_index", where);
            if (!b) return std::unexpected(b.error());
            g.attach<BehaviorComponent>(id, BehaviorComponent{.behavior_index = *b});
            return {};
        }
        case ComponentTypeId::fluid: {
            const Result<float> d = float_field(body, "rest_density", where);
            if (!d) return std::unexpected(d.error());
            const Result<float> k = float_field(body, "stiffness", where);
            if (!k) return std::unexpected(k.error());
            const Result<float> v = float_field(body, "viscosity", where);
            if (!v) return std::unexpected(v.error());
            g.attach<FluidComponent>(
                id, FluidComponent{.rest_density = *d, .stiffness = *k, .viscosity = *v});
            return {};
        }
    }
    return std::unexpected(bad(where + ": unhandled component type"));
}

}  // namespace

Result<std::string> to_json(const ObjectGraph& graph) {
    // Slot index -> position in the objects array. Built first, because a
    // parent may sit at a HIGHER slot than its child and the link is written
    // as an array position.
    std::unordered_map<uint32_t, std::size_t> position;
    std::vector<ObjectId> order;
    graph.for_each([&](ObjectId id) {
        position.emplace(id.index, order.size());
        order.push_back(id);
    });

    json objects = json::array();
    for (const ObjectId id : order) {
        const Object* o = graph.get(id);
        json entry;
        entry["name"] = o->name;

        std::int64_t parent_position = -1;  // root
        if (!o->parent.is_null()) {
            const auto it = position.find(o->parent.index);
            // Dangling: the named parent is not a live object of this graph.
            // See serialize.hpp -- refusing here is the whole point.
            if (it == position.end() || !graph.alive(o->parent)) {
                return std::unexpected(bad("object '" + o->name +
                                           "' names a parent that is not a live object of this "
                                           "graph; re-root it explicitly before saving"));
            }
            parent_position = static_cast<std::int64_t>(it->second);
        }
        entry["parent"] = parent_position;

        entry["position"] = {o->position.x, o->position.y, o->position.z};
        // (w, x, y, z) -- glm's quat CONSTRUCTOR order, which is not its
        // storage order. Spelled out field by field so the file's order is
        // stated here rather than inherited from whatever glm packs.
        entry["orientation"] = {o->orientation.w, o->orientation.x, o->orientation.y,
                                o->orientation.z};
        entry["scale"] = {o->scale.x, o->scale.y, o->scale.z};

        json components = json::object();
        write_components(graph, id, components);
        entry["components"] = components;

        objects.push_back(std::move(entry));
    }

    json doc;
    doc["objects"] = std::move(objects);
    return doc.dump(2);
}

Result<ObjectGraph> from_json(std::string_view text) {
    json doc = json::parse(text, nullptr, /*allow_exceptions=*/false);
    if (doc.is_discarded()) return std::unexpected(bad("document is not valid JSON"));
    if (!doc.is_object()) return std::unexpected(bad("document: expected an object"));

    const auto objects_it = doc.find("objects");
    if (objects_it == doc.end() || !objects_it->is_array()) {
        return std::unexpected(bad("document: missing 'objects' array"));
    }

    ObjectGraph graph;
    std::vector<ObjectId> created;
    created.reserve(objects_it->size());

    // Pass 1: every object, without its parent link. Positions in `created`
    // mirror positions in the array, which is what the second pass resolves
    // against -- so a parent may appear after its child in the file.
    for (std::size_t i = 0; i < objects_it->size(); ++i) {
        const json& entry = (*objects_it)[i];
        const std::string what = "objects[" + std::to_string(i) + "]";
        if (!entry.is_object()) return std::unexpected(bad(what + ": expected an object"));

        const Result<const json*> name = object_field(entry, "name", what);
        if (!name) return std::unexpected(name.error());
        if (!(*name)->is_string()) return std::unexpected(bad(what + ".name: expected a string"));

        const ObjectId id = graph.create((*name)->get<std::string>());
        created.push_back(id);
        Object* o = graph.get(id);

        const Result<std::array<float, 3>> p = floats_field<3>(entry, "position", what);
        if (!p) return std::unexpected(p.error());
        o->position = glm::vec3((*p)[0], (*p)[1], (*p)[2]);

        const Result<std::array<float, 4>> q = floats_field<4>(entry, "orientation", what);
        if (!q) return std::unexpected(q.error());
        o->orientation = glm::quat((*q)[0], (*q)[1], (*q)[2], (*q)[3]);  // (w, x, y, z)

        const Result<std::array<float, 3>> s = floats_field<3>(entry, "scale", what);
        if (!s) return std::unexpected(s.error());
        o->scale = glm::vec3((*s)[0], (*s)[1], (*s)[2]);

        const Result<const json*> comps = object_field(entry, "components", what);
        if (!comps) return std::unexpected(comps.error());
        if (!(*comps)->is_object()) {
            return std::unexpected(bad(what + ".components: expected an object"));
        }
        for (const auto& [key, body] : (*comps)->items()) {
            const std::optional<ComponentTypeId> type = component_type_id_from_name(key);
            // An unknown name is an ERROR, not a skip: silently dropping it
            // loads a scene quietly missing behaviour its author put there.
            if (!type) {
                return std::unexpected(
                    bad(what + ".components: unknown component name '" + key + "'"));
            }
            if (Result<void> r = read_component(graph, id, *type, body, what + ".components"); !r) {
                return std::unexpected(r.error());
            }
        }
    }

    // Pass 2: parent links, by array position.
    for (std::size_t i = 0; i < objects_it->size(); ++i) {
        const json& entry = (*objects_it)[i];
        const std::string what = "objects[" + std::to_string(i) + "]";
        const Result<const json*> parent = object_field(entry, "parent", what);
        if (!parent) return std::unexpected(parent.error());
        if (!(*parent)->is_number_integer()) {
            return std::unexpected(bad(what + ".parent: expected an integer index, or -1 for a root"));
        }
        const std::int64_t at = (*parent)->get<std::int64_t>();
        if (at < 0) continue;  // root
        if (static_cast<std::size_t>(at) >= created.size()) {
            return std::unexpected(bad(what + ".parent: index " + std::to_string(at) +
                                       " is out of range for " + std::to_string(created.size()) +
                                       " objects"));
        }
        if (static_cast<std::size_t>(at) == i) {
            return std::unexpected(bad(what + ".parent: an object cannot be its own parent"));
        }
        graph.get(created[i])->parent = created[static_cast<std::size_t>(at)];
    }

    return graph;
}

ObjectId find_by_name(const ObjectGraph& graph, std::string_view name) noexcept {
    ObjectId found{};
    graph.for_each([&](ObjectId id) {
        if (!found.is_null()) return;
        if (graph.get(id)->name == name) found = id;
    });
    return found;
}

}  // namespace spade::objects
