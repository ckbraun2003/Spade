// ---------------------------------------------------------------------------
// scene/compose.cpp -- see compose.hpp.
//
// Compiled under spade_fp_strict, like every digest-feeding target: the
// composed transforms land in the SDF program, which config_hash folds, so gcc
// must not contract these products into FMA.
// ---------------------------------------------------------------------------
#include "scene/compose.hpp"

#include "core/path_text.hpp"

#include <cstddef>
#include <exception>
#include <limits>
#include <string>
#include <utility>

#include "world/world_file.hpp"  // load_world_file

namespace spade::scene {
namespace {

// Value comparison on purpose: -0 == 0, so a transform built analytically from
// a zero position, whose translation is -0, counts as the identity.
bool is_identity(const SdfTransform& t) {
    return t.world_to_local == glm::mat4(1.0f) && t.scale == 1.0f;
}

[[nodiscard]] Error invalid(std::string context) {
    return Error{Code::invalid_argument, std::move(context)};
}

// The same error, its context prefixed: the code is the cause's, not ours.
[[nodiscard]] Error prefixed(const Error& error, const std::string& prefix) {
    return Error{error.code, prefix + error.context};
}

// A hash as the scene file spells it: "0x" and 16 lowercase hex digits.
[[nodiscard]] std::string hex64(uint64_t value) {
    constexpr char kDigits[] = "0123456789abcdef";
    std::string out = "0x";
    for (int shift = 60; shift >= 0; shift -= 4) {
        out += kDigits[(value >> shift) & 0xFu];
    }
    return out;
}

// The one palette entry `name` refers to (SCN-009: a reference is a name). A
// name the palette holds twice is not a name, so it is refused like one the
// palette does not hold. Scene materials never repeat a world name (step 3),
// so a repeat can only be the world's own.
[[nodiscard]] Result<uint32_t> resolve_material(const std::vector<MaterialDesc>& palette,
                                                const std::string& name, const std::string& user) {
    const std::size_t none = palette.size();
    std::size_t found = none;
    for (std::size_t i = 0; i < palette.size(); ++i) {
        if (palette[i].name != name) {
            continue;
        }
        if (found != none) {
            return std::unexpected(invalid(user + " names material '" + name +
                                           "', which the world's palette holds twice (indices " +
                                           std::to_string(found) + " and " + std::to_string(i) +
                                           "); rename one of them in the world file"));
        }
        found = i;
    }
    if (found == none) {
        return std::unexpected(invalid(user + " names material '" + name +
                                       "', which neither the world's palette nor the scene's materials "
                                       "hold; add it to the scene's materials or correct the name"));
    }
    return static_cast<uint32_t>(found);
}

// Joins one asset's collider to the composed program (compose.hpp, step 4).
// `materialized` says whether `sdf.node_materials` is full length yet.
[[nodiscard]] Result<void> join_collider(SdfProgram& sdf, bool& materialized, const SceneAsset& asset,
                                         const std::vector<MaterialDesc>& palette) {
    const std::string user = "asset '" + asset.name + "'";
    const Result<SdfTransform> pose = transform_of(asset.pose);
    if (!pose) {
        return std::unexpected(prefixed(pose.error(), user + ": "));
    }

    std::vector<uint32_t> named;
    named.reserve(asset.collider_materials.size());
    for (const std::string& name : asset.collider_materials) {
        const Result<uint32_t> index = resolve_material(palette, name, "a collider node of " + user);
        if (!index) {
            return std::unexpected(index.error());
        }
        named.push_back(*index);
    }

    const std::size_t base = sdf.transforms.size();
    if (base + asset.collider.transforms.size() > std::numeric_limits<uint32_t>::max()) {
        return std::unexpected(Error{Code::capacity_exceeded,
                                     user + ": the composed SDF program would hold more than 2^32 - 1 transforms"});
    }
    for (const SdfTransform& t : asset.collider.transforms) {
        sdf.transforms.push_back(compose_transform(t, *pose));
    }

    // The first named node fills the array to full length: every node before
    // it uses the palette's default, index 0, which is what an empty array
    // meant for them.
    if (!named.empty() && !materialized) {
        sdf.node_materials.assign(sdf.nodes.size(), 0u);
        materialized = true;
    }

    const bool joins = !sdf.nodes.empty();
    for (std::size_t k = 0; k < asset.collider.nodes.size(); ++k) {
        SdfNode node = asset.collider.nodes[k];
        // Only a primitive indexes the transforms. An operator node's transform
        // is unused and must stay 0 (SdfProgram::validate).
        if (node.op == static_cast<uint32_t>(SdfOp::none)) {
            node.transform += static_cast<uint32_t>(base);
        }
        sdf.nodes.push_back(node);
        if (materialized) {
            sdf.node_materials.push_back(named.empty() ? 0u : named[k]);
        }
    }
    if (joins) {
        // WorldBuilder::union_()'s node, byte for byte.
        SdfNode join{};
        join.op = static_cast<uint32_t>(SdfOp::union_);
        sdf.nodes.push_back(join);
        if (materialized) {
            sdf.node_materials.push_back(0u);  // an operator's material is never read
        }
    }
    return {};
}

// One composed capacity field (SCN-007): the world's count, plus what the
// scene's vehicles need, plus the scene's spare.
[[nodiscard]] Result<uint32_t> capacity(const char* field, uint64_t world, uint64_t need, uint64_t spare) {
    const uint64_t total = world + need + spare;
    if (total > std::numeric_limits<uint32_t>::max()) {
        return std::unexpected(Error{
            Code::capacity_exceeded,
            std::string("the composed capacity '") + field + "' would be " + std::to_string(total) +
                " (the world's " + std::to_string(world) + ", plus " + std::to_string(need) +
                " for the vehicles, plus " + std::to_string(spare) +
                " spare), beyond 2^32 - 1; lower the scene's spare count"});
    }
    return static_cast<uint32_t>(total);
}

}  // namespace

SdfTransform compose_transform(const SdfTransform& collider, const SdfTransform& asset_pose) {
    if (is_identity(asset_pose)) {
        return collider;  // short-cut (a)
    }
    if (is_identity(collider)) {
        return asset_pose;  // short-cut (b)
    }
    SdfTransform out{};
    out.world_to_local = collider.world_to_local * asset_pose.world_to_local;
    out.scale = collider.scale * asset_pose.scale;
    return out;
}

Result<ComposedScene> compose(const SceneDesc& scene, const WorldDesc& world) {
    // 1. The one validator (SCN-003). The reader ran it too, but a SceneDesc
    //    can also be built in memory.
    if (Result<void> r = validate_scene(scene); !r) {
        return std::unexpected(r.error());
    }
    const std::string where = "scene '" + scene.name + "': ";

    // 2. The world hash, on every path (SCN-001).
    const Result<uint64_t> hash = world_hash(world);
    if (!hash) {
        return std::unexpected(prefixed(hash.error(), where + "its world does not validate: "));
    }
    if (*hash != scene.world.hash) {
        return std::unexpected(invalid(
            where + "it pins its world '" + scene.world.file + "' at hash " + hex64(scene.world.hash) +
            ", but the world given ('" + world.name + "') hashes to " + hex64(*hash) +
            ". The world changed after the scene was written: re-pin the scene to the world as it now "
            "loads, or point it at the world it was made for"));
    }

    ComposedScene out{};
    out.world = world;
    WorldDesc& composed = out.world;

    // 3. Materials, after the world's palette (SCN-004).
    for (const MaterialDesc& m : scene.materials) {
        for (std::size_t i = 0; i < world.materials.size(); ++i) {
            if (world.materials[i].name == m.name) {
                return std::unexpected(invalid(where + "material '" + m.name +
                                               "' reuses the name of the world's material " + std::to_string(i) +
                                               "; names must be unique across the world's palette and the "
                                               "scene's materials, so rename the scene's"));
            }
        }
        composed.materials.push_back(m);
    }

    // 4. Assets, in scene order (SCN-005).
    bool materialized = !composed.sdf.node_materials.empty();
    for (const SceneAsset& asset : scene.assets) {
        if (!asset.collider.nodes.empty()) {
            if (Result<void> r = join_collider(composed.sdf, materialized, asset, composed.materials); !r) {
                return std::unexpected(prefixed(r.error(), where));
            }
        }
        if (!asset.visual.mesh_ref.empty()) {
            const Result<uint32_t> material =
                resolve_material(composed.materials, asset.visual.material, "the visual of asset '" + asset.name + "'");
            if (!material) {
                return std::unexpected(prefixed(material.error(), where));
            }
            composed.props.push_back(PropDesc{asset.visual.mesh_ref, asset.pose, *material});
        }
    }

    // 5. Models in the models: section's order, and each vehicle's model name
    //    as an index into them.
    out.models = scene.models;
    uint64_t need_force_elements = 0;
    uint64_t need_sensors = 0;
    for (const SceneVehicle& v : scene.vehicles) {
        std::size_t model = 0;
        while (model < scene.models.size() && scene.models[model].name != v.model) {
            ++model;
        }
        if (model == scene.models.size()) {
            // validate_scene() refuses this first; kept so an index can never be
            // out of range.
            return std::unexpected(invalid(where + "vehicle '" + v.name + "' names unknown model '" + v.model + "'"));
        }
        const vehicles::ModelType& m = scene.models[model];
        need_force_elements += m.rotors.size() + m.drag_bodies.size();
        need_sensors += m.imu_mounts.size();
        out.vehicles.push_back(VehiclePlacement{v.name, static_cast<uint32_t>(model), v.start});
    }

    // 6. Capacities (SCN-007).
    const Capacities& have = world.capacities;
    const Result<uint32_t> bodies = capacity("bodies", have.bodies, scene.vehicles.size(), scene.spare.bodies);
    if (!bodies) {
        return std::unexpected(prefixed(bodies.error(), where));
    }
    const Result<uint32_t> force_elements =
        capacity("force_elements", have.force_elements, need_force_elements, scene.spare.force_elements);
    if (!force_elements) {
        return std::unexpected(prefixed(force_elements.error(), where));
    }
    const Result<uint32_t> sensors = capacity("sensors", have.sensors, need_sensors, scene.spare.sensors);
    if (!sensors) {
        return std::unexpected(prefixed(sensors.error(), where));
    }
    const Result<uint32_t> contacts = capacity("contacts", have.contacts, 0, scene.spare.contacts);
    if (!contacts) {
        return std::unexpected(prefixed(contacts.error(), where));
    }
    composed.capacities = Capacities{*bodies, *force_elements, *sensors, *contacts};

    // 7. The one validation every WorldDesc passes.
    if (const Result<uint32_t> depth = validate_world_desc(composed); !depth) {
        return std::unexpected(prefixed(depth.error(), where + "the composed world does not validate: "));
    }
    return out;
}

Result<ComposedScene> compose_file(const std::filesystem::path& scene_file) {
    const Result<SceneDesc> scene = load_scene_file(scene_file);
    if (!scene) {
        return std::unexpected(scene.error());  // load_scene_file names the file
    }
    const std::string where = path_text(scene_file) + ": ";

    std::filesystem::path world_file;
    try {
        world_file = scene_file.parent_path() / std::filesystem::path(scene->world.file);
    } catch (const std::exception& e) {
        return std::unexpected(invalid(where + "its world '" + scene->world.file +
                                       "' is not a path this platform can open: " + e.what()));
    }
    const Result<WorldDesc> world = load_world_file(world_file);
    if (!world) {
        return std::unexpected(prefixed(world.error(), where + "its world '" + scene->world.file + "' did not load: "));
    }

    Result<ComposedScene> composed = compose(*scene, *world);
    if (!composed) {
        return std::unexpected(prefixed(composed.error(), where));
    }
    return composed;
}

Result<SceneRun> instantiate(const ComposedScene& scene, WorldInstanceDesc instance, uint64_t dt_ns,
                             uint32_t substeps, const compute::BackendDesc& backend) {
    instance.world = scene.world;
    WorldSetDesc set{};
    set.worlds.push_back(std::move(instance));

    Result<Simulation> created = Simulation::create(set, dt_ns, substeps, backend);
    if (!created) {
        return std::unexpected(prefixed(created.error(), "instantiate: the simulation was not created: "));
    }
    Simulation& sim = *created;

    // A fresh Simulation, so models[i] gets ModelTypeId i + 1.
    std::vector<ModelTypeId> ids;
    ids.reserve(scene.models.size());
    for (const vehicles::ModelType& m : scene.models) {
        const Result<ModelTypeId> id = sim.register_model(m);
        if (!id) {
            return std::unexpected(prefixed(id.error(), "instantiate: model '" + m.name + "' did not register: "));
        }
        ids.push_back(*id);
    }

    std::vector<VehicleRef> refs;
    refs.reserve(scene.vehicles.size());
    for (const VehiclePlacement& v : scene.vehicles) {
        if (v.model >= ids.size()) {
            return std::unexpected(invalid("instantiate: vehicle '" + v.name + "' names model " +
                                           std::to_string(v.model) + ", but the composed scene has " +
                                           std::to_string(ids.size()) + " model(s)"));
        }
        // The start exactly as the scene wrote it, in the design frame:
        // spawn() converts it (DBP-45, DBP-46).
        const Result<VehicleRef> ref = sim.spawn(0, ids[v.model], v.start);
        if (!ref) {
            return std::unexpected(prefixed(ref.error(), "instantiate: vehicle '" + v.name + "' did not spawn: "));
        }
        refs.push_back(*ref);
    }

    if (const Result<void> flushed = sim.flush_structural(); !flushed) {
        return std::unexpected(prefixed(flushed.error(), "instantiate: the spawns did not flush: "));
    }
    return SceneRun{std::move(sim), std::move(refs)};
}

}  // namespace spade::scene
