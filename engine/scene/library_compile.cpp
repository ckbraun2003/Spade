#include "scene/library_compile.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <iterator>
#include <map>
#include <string>
#include <utility>
#include <variant>
#include <vector>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include "scene/library_place.hpp"
#include "scene/library_query.hpp"
#include "scene/part_blocks.hpp"
#include "scene/part_spec.hpp"
#include "vehicles/composite_inertia.hpp"
#include "world/mesh.hpp"

namespace spade::scene {

namespace {

using vehicles::PartInertia;

LibraryRefusal refuse(std::string code, std::string message, const SetEntry* entry = nullptr, std::string field = {}) {
    LibraryRefusal r;
    r.code = std::move(code);
    r.message = std::move(message);
    if (entry != nullptr) {
        r.root = entry->root;
        r.id = entry->id;
    }
    r.field = std::move(field);
    return r;
}

struct Pose {
    glm::dvec3 p{0.0};
    glm::dquat q{1.0, 0.0, 0.0, 0.0};
};

// The one frame change: the library's design frame (+X forward, +Y left, +Z up) to the AirframeSpec's (+X forward, +Y up, +Z right), Rx(-90 deg):
// (x, y, z) -> (x, z, -y).
const glm::dquat kToSpade{0.70710678118654757, -0.70710678118654757, 0.0, 0.0};

glm::dvec3 to_spade(const glm::dvec3& v) { return {v.x, v.z, -v.y}; }
glm::dquat to_spade(const glm::dquat& q) { return glm::normalize(kToSpade * q); }

bool read_text(const std::filesystem::path& path, std::string& out) {
    std::ifstream in(path, std::ios::binary);
    if (!in) return false;
    out.assign(std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>());
    return true;
}

// One part of the drone, read once: its record, its spec and (a part with meshes) its objects.
struct PartData {
    const SetEntry* entry = nullptr;
    PartBlocks blocks{};
    std::optional<PartFile> spec{};
    std::vector<PartMeshObject> objects{};
    double mass = 0.0;
};

const PartInstallPoint* point_of(const PartBlocks& blocks, const std::string& name) {
    for (const PartInstallPoint& ip : blocks.install_points) {
        if (ip.name == name) return &ip;
    }
    return nullptr;
}

SetPose to_set_pose(const Pose& p) { return SetPose{{p.p.x, p.p.y, p.p.z}, {p.q.w, p.q.x, p.q.y, p.q.z}}; }
Pose from_set_pose(const SetPose& s) { return Pose{{s.pos[0], s.pos[1], s.pos[2]}, glm::dquat(s.quat[0], s.quat[1], s.quat[2], s.quat[3])}; }

LibraryResult<PartData> read_part(const LibrarySet& set, const std::string& id, bool need_meshes) {
    LibraryResult<FindResult> found = find_entry(set, id, {});
    if (!found) return std::unexpected(found.error());
    PartData d;
    d.entry = found->entry;
    if (d.entry->kind != "part" || d.entry->file.empty()) {
        return std::unexpected(refuse("invalid_entry", "a slot's part must be a part entry with a file", d.entry));
    }
    std::string text;
    if (!read_text(d.entry->file, text)) {
        return std::unexpected(refuse("io_error", "the part's record cannot be read: " + d.entry->file.generic_string(), d.entry));
    }
    Result<PartBlocks> blocks = read_part_blocks(text);
    if (!blocks) return std::unexpected(refuse("invalid_entry", blocks.error().context, d.entry));
    d.blocks = std::move(*blocks);
    Result<std::optional<PartFile>> spec = part_spec_from_text(text);
    if (!spec) return std::unexpected(refuse("invalid_entry", spec.error().context, d.entry, "spec"));
    d.spec = std::move(*spec);
    if (d.blocks.mass_properties && d.blocks.mass_properties->mass > 0.0) d.mass = d.blocks.mass_properties->mass;
    if (need_meshes && !d.blocks.meshes.empty()) {
        LibraryResult<std::vector<PartMeshObject>> objects = part_meshes(set, id);
        if (!objects) return std::unexpected(objects.error());
        d.objects = std::move(*objects);
    }
    return d;
}

// ----- the composite of a set of records, in double (the remainder's arithmetic) ---------------------------------------------------------------

// Mass, centre of mass and the tensor about it, design frame.
struct Lump {
    double mass = 0.0;
    glm::dvec3 com{0.0};
    glm::dmat3 tensor{0.0};
};

// The inertia of a point mass `m` at `r` from the reference point.
glm::dmat3 parallel_axis(double m, const glm::dvec3& r) {
    glm::dmat3 t(0.0);
    const double r2 = glm::dot(r, r);
    for (int c = 0; c < 3; ++c) {
        for (int k = 0; k < 3; ++k) t[c][k] = m * ((c == k ? r2 : 0.0) - r[c] * r[k]);
    }
    return t;
}

Lump lump_of(const std::vector<PartInertia>& records) {
    Lump out;
    for (const PartInertia& r : records) {
        out.mass += r.mass;
        out.com += r.mass * r.position;
    }
    out.com /= out.mass;
    for (const PartInertia& r : records) {
        const glm::dmat3 rot = glm::mat3_cast(glm::normalize(r.orientation));
        out.tensor += rot * vehicles::part_inertia_local(r) * glm::transpose(rot) + parallel_axis(r.mass, r.position - out.com);
    }
    return out;
}

// The eigenvalues of a symmetric 3 x 3, ascending: cyclic Jacobi sweeps, square roots only (no libm transcendental: the engine's sources are scanned).
std::array<double, 3> principal_moments(const glm::dmat3& m) {
    double a[3][3];
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) a[i][j] = 0.5 * (m[i][j] + m[j][i]);
    }
    for (int sweep = 0; sweep < 32; ++sweep) {
        const double off = a[0][1] * a[0][1] + a[0][2] * a[0][2] + a[1][2] * a[1][2];
        const double diag = a[0][0] * a[0][0] + a[1][1] * a[1][1] + a[2][2] * a[2][2];
        if (off <= 1e-34 * diag || off == 0.0) break;
        for (int p = 0; p < 2; ++p) {
            for (int q = p + 1; q < 3; ++q) {
                if (a[p][q] == 0.0) continue;
                const double theta = (a[q][q] - a[p][p]) / (2.0 * a[p][q]);
                const double t = (theta >= 0.0 ? 1.0 : -1.0) / (std::abs(theta) + std::sqrt(theta * theta + 1.0));
                const double c = 1.0 / std::sqrt(t * t + 1.0);
                const double s = t * c;
                const double apq = a[p][q];
                a[p][p] -= t * apq;
                a[q][q] += t * apq;
                a[p][q] = a[q][p] = 0.0;
                for (int r = 0; r < 3; ++r) {
                    if (r == p || r == q) continue;
                    const double arp = a[r][p];
                    const double arq = a[r][q];
                    a[r][p] = a[p][r] = c * arp - s * arq;
                    a[r][q] = a[q][r] = s * arp + c * arq;
                }
            }
        }
    }
    std::array<double, 3> v = {a[0][0], a[1][1], a[2][2]};
    std::sort(v.begin(), v.end());
    return v;
}

std::string names_of(const std::vector<FlowPart>& parts) {
    std::string out;
    for (const FlowPart& p : parts) out += (out.empty() ? "" : ", ") + p.name;
    return out;
}

// The flow parts' records and ONE remainder record, replacing `records` (the all-parts mass records, design frame). Refused: invalid_request, field
// `options.flow_parts[...]`.
LibraryResult<void> apply_flow_parts(const std::vector<FlowPart>& parts, std::vector<PartInertia>& records, const SetEntry& drone) {
    const auto bad = [&](std::string message, std::string field) {
        return std::unexpected(refuse("invalid_request", std::move(message), &drone, std::move(field)));
    };
    std::vector<PartInertia> flow;
    for (std::size_t i = 0; i < parts.size(); ++i) {
        const FlowPart& p = parts[i];
        const std::string where = "options.flow_parts[" + std::to_string(i) + "]";
        if (p.name.empty()) return bad(where + ": a flow part needs a name", where + ".name");
        if (p.name == "remainder") return bad(where + ": 'remainder' is the compile's own record", where + ".name");
        for (std::size_t j = 0; j < i; ++j) {
            if (parts[j].name == p.name) return bad(where + ": the name '" + p.name + "' is used twice", where + ".name");
        }
        if (!(p.mass > 0.0) || !std::isfinite(p.mass)) return bad(where + " '" + p.name + "': the mass must be finite and above 0", where + ".mass");
        if (p.shape != vehicles::PartShape::box && p.shape != vehicles::PartShape::sphere && p.shape != vehicles::PartShape::cylinder &&
            p.shape != vehicles::PartShape::tube) {
            return bad(where + " '" + p.name + "': a flow part is a box, sphere, cylinder or tube (a point or a tensor is no solid)", where + ".shape");
        }
        const bool two = p.shape == vehicles::PartShape::cylinder || p.shape == vehicles::PartShape::tube;
        const bool three = p.shape == vehicles::PartShape::box;
        const bool ok = std::isfinite(p.size.x) && p.size.x > 0.0 && (!two || (std::isfinite(p.size.y) && p.size.y > 0.0)) &&
                        (!three || (std::isfinite(p.size.y) && p.size.y > 0.0 && std::isfinite(p.size.z) && p.size.z > 0.0));
        if (!ok) return bad(where + " '" + p.name + "': a size that is not finite and above 0", where + ".size");
        if (!std::isfinite(p.pos.x) || !std::isfinite(p.pos.y) || !std::isfinite(p.pos.z) || glm::dot(p.quat, p.quat) <= 0.0) {
            return bad(where + " '" + p.name + "': a pose that is not finite", where + ".pos");
        }
        PartInertia r;
        r.mass = p.mass;
        r.position = to_spade(p.pos);
        r.orientation = to_spade(p.quat);
        r.shape = p.shape;
        r.size = p.size;
        r.geometry.name = p.name;
        r.geometry.role = p.role;
        flow.push_back(r);
    }
    const Lump all = lump_of(records);
    const Lump held = lump_of(flow);
    const double rest = all.mass - held.mass;
    if (rest < -1e-9 * all.mass) {
        return bad("the flow parts [" + names_of(parts) + "] hold more mass than the drone (" + std::to_string(held.mass) + " of " + std::to_string(all.mass) + " kg)",
                   "options.flow_parts");
    }
    if (rest <= 1e-9 * all.mass) {  // they take all of it: no zero record
        records = std::move(flow);
        return {};
    }
    // the remainder: its mass, its centre where the all-parts centre of mass balances the flow parts', its tensor what completes the all-parts tensor about the
    // all-parts centre of mass (tests/x500_model.hpp kit_remainder, the same arithmetic)
    const glm::dvec3 centre = (all.mass * all.com - held.mass * held.com) / rest;
    const glm::dmat3 held_at_all = held.tensor + parallel_axis(held.mass, held.com - all.com);
    glm::dmat3 tensor = all.tensor - held_at_all - parallel_axis(rest, centre - all.com);
    for (int c = 0; c < 3; ++c) {
        for (int k = c + 1; k < 3; ++k) tensor[c][k] = tensor[k][c] = 0.5 * (tensor[c][k] + tensor[k][c]);
    }
    const std::array<double, 3> pm = principal_moments(tensor);
    const double tol = 1e-9 * std::max(pm[2], 0.0);
    if (pm[0] < -tol || pm[0] + pm[1] < pm[2] - tol) {
        return bad("the flow parts [" + names_of(parts) + "] leave a remainder that is not a physical body: principal moments " + std::to_string(pm[0]) + ", " +
                       std::to_string(pm[1]) + ", " + std::to_string(pm[2]) + " kg m^2 (the smallest must not be below zero, and the two smaller must sum to at least the largest)",
                   "options.flow_parts");
    }
    PartInertia rem;
    rem.mass = rest;
    rem.position = centre;
    rem.shape = vehicles::PartShape::tensor;
    rem.tensor = tensor;
    rem.geometry.name = "remainder";
    rem.geometry.role = "remainder";
    flow.push_back(rem);
    records = std::move(flow);
    return {};
}

// The mass records of one part placed at `pose` (library frame), in the AirframeSpec's frame.
LibraryResult<void> add_records(const PartData& part, const Pose& pose, std::vector<PartInertia>& out) {
    if (part.mass <= 0.0) {
        return std::unexpected(refuse("invalid_entry", "the part has no mass_properties.mass above 0", part.entry, "mass_properties.mass"));
    }
    // meshes win (THE RULE): one tensor record per solid object, at a uniform density over the part
    double volume = 0.0;
    for (const PartMeshObject& o : part.objects) {
        if (!o.visual && o.mesh != nullptr) volume += o.mesh->mass_properties().volume;
    }
    if (volume > 0.0) {
        const double rho = part.mass / volume;
        for (const PartMeshObject& o : part.objects) {
            if (o.visual || o.mesh == nullptr) continue;
            const MeshMassProperties& mp = o.mesh->mass_properties();
            PartInertia r;
            r.mass = rho * mp.volume;
            r.position = to_spade(pose.p + pose.q * mp.centroid);
            r.orientation = to_spade(pose.q);
            r.shape = vehicles::PartShape::tensor;
            r.tensor = rho * mp.inertia;
            out.push_back(r);
        }
        return {};
    }
    // no solid meshes: the uniform boxes of its shapes
    if (!part.blocks.shapes.empty()) {
        double total = 0.0;
        for (const spade::scene::PartShape& s : part.blocks.shapes) {
            if (s.kind != spade::scene::PartShape::Kind::box || s.size.size() != 3) {
                return std::unexpected(refuse("invalid_entry", "compile reads box shapes only", part.entry, "shapes"));
            }
            total += s.size[0] * s.size[1] * s.size[2];
        }
        if (!(total > 0.0)) return std::unexpected(refuse("invalid_entry", "the shapes hold no volume", part.entry, "shapes"));
        for (const spade::scene::PartShape& s : part.blocks.shapes) {
            PartInertia r;
            r.mass = part.mass * (s.size[0] * s.size[1] * s.size[2]) / total;
            r.position = to_spade(pose.p + pose.q * s.pos);
            r.orientation = to_spade(pose.q * s.orient);
            r.shape = vehicles::PartShape::box;
            r.size = {s.size[0], s.size[1], s.size[2]};
            r.tensor = vehicles::part_inertia_local(r);  // the box's inertia, as a tensor record: mass only, like every part (a box record would be a wall)
            r.shape = vehicles::PartShape::tensor;
            r.size = glm::dvec3(0.0);
            out.push_back(r);
        }
        return {};
    }
    PartInertia r;  // a point mass at the part's origin
    r.mass = part.mass;
    r.position = to_spade(pose.p);
    r.orientation = to_spade(pose.q);
    out.push_back(r);
    return {};
}

template <class Block>
const Block* spec_block(const PartData& part) {
    if (!part.spec) return nullptr;
    return std::get_if<Block>(&*part.spec);
}

}  // namespace

LibraryResult<CompiledDrone> compile_drone(const LibrarySet& set, std::string_view id, const CompileOptions& options) {
    LibraryResult<FindResult> found = find_entry(set, id, {});
    if (!found) return std::unexpected(found.error());
    const SetEntry& drone = *found->entry;
    if (drone.kind != "drone" || drone.parts_map.empty()) {
        return std::unexpected(refuse("invalid_entry", "only a drone with a parts map compiles", &drone));
    }
    if (!options.drag) {
        return std::unexpected(refuse("invalid_request", "a drag is required: a tensor record gives the compile's estimate no area; give a DragSpec (zero for none)", &drone, "drag"));
    }

    // the parts, read once each, and the slots' poses in the library's design frame
    std::map<std::string, PartData> parts;
    std::map<std::string, Pose> placed;      // by slot key
    std::map<std::string, std::string> part_of;  // slot key -> part id
    for (const PartsMapSlot& slot : drone.parts_map) {
        if (parts.find(slot.part) == parts.end()) {
            LibraryResult<PartData> d = read_part(set, slot.part, true);
            if (!d) return std::unexpected(d.error());
            parts.emplace(slot.part, std::move(*d));
        }
        const PartData& part = parts.at(slot.part);
        PlacementItem item;
        item.key = slot.slot;
        if (slot.placement) {
            item.install_point = *slot.placement;
        } else {
            const auto parent = placed.find(slot.parent);
            if (parent == placed.end()) {
                return std::unexpected(refuse("invalid_entry", "slot '" + slot.slot + "' names a parent not placed before it: '" + slot.parent + "'", &drone, "parts_map"));
            }
            item.parent = to_set_pose(parent->second);
            const PartInstallPoint* ip = point_of(parts.at(part_of.at(slot.parent)).blocks, slot.install_point);
            if (ip == nullptr) {
                return std::unexpected(refuse("invalid_entry", "slot '" + slot.slot + "': '" + slot.parent + "' has no install point '" + slot.install_point + "'", &drone, "parts_map"));
            }
            item.install_point = SetPose{{ip->pos.x, ip->pos.y, ip->pos.z}, {ip->orient.w, ip->orient.x, ip->orient.y, ip->orient.z}};
            item.adjust = slot.adjust;
            // a part with a `seat` point stands on its parent's point by that seat: its own mount offset is the seat's inverse
            if (const PartInstallPoint* seat = point_of(part.blocks, "seat")) {
                const glm::dquat inv = glm::conjugate(seat->orient);
                const glm::dvec3 off = -(inv * seat->pos);
                item.part_to_mount = SetPose{{off.x, off.y, off.z}, {inv.w, inv.x, inv.y, inv.z}};
            }
        }
        const std::vector<PlacedItem> result = compose_placement({item});
        if (result.empty() || !result[0].pose) {
            return std::unexpected(refuse(result.empty() || !result[0].refusal ? "invalid_entry" : result[0].refusal->code, "slot '" + slot.slot + "' cannot be placed", &drone, "parts_map"));
        }
        placed[slot.slot] = from_set_pose(*result[0].pose);
        part_of[slot.slot] = slot.part;
    }

    vehicles::AirframeSpec spec;
    spec.name = drone.local_id;
    spec.drag.push_back(*options.drag);

    // the mass records: every part. A propeller is a thin disc of its radius about its spin axis (+Y), at its part's pose (the kit constants' own: its mass
    // is where the table puts the propeller, the part's origin, not on the hub), so the block's mass is 0
    for (const PartsMapSlot& slot : drone.parts_map) {
        const PartData& part = parts.at(slot.part);
        if (part.entry->part_kind == "prop") {
            const PropPart* p = spec_block<PropPart>(part);
            if (p == nullptr || !(p->block.diameter > 0.0)) {
                return std::unexpected(refuse("invalid_entry", "a propeller part needs a prop spec with a diameter", part.entry, "spec"));
            }
            if (part.mass <= 0.0) return std::unexpected(refuse("invalid_entry", "the propeller has no mass_properties.mass above 0", part.entry, "mass_properties.mass"));
            const double radius = 0.5 * p->block.diameter;
            PartInertia disc;
            disc.mass = part.mass;
            disc.position = to_spade(placed.at(slot.slot).p);
            disc.orientation = to_spade(placed.at(slot.slot).q);
            disc.shape = vehicles::PartShape::tensor;
            disc.tensor[0][0] = 0.25 * part.mass * radius * radius;
            disc.tensor[1][1] = 0.5 * part.mass * radius * radius;
            disc.tensor[2][2] = 0.25 * part.mass * radius * radius;
            spec.parts.push_back(disc);
            continue;
        }
        LibraryResult<void> ok = add_records(part, placed.at(slot.slot), spec.parts);
        if (!ok) return std::unexpected(ok.error());
    }

    // the walls, when the caller gives flow parts: they and one remainder record replace the mass records, the composite unchanged
    if (!options.flow_parts.empty()) {
        LibraryResult<void> ok = apply_flow_parts(options.flow_parts, spec.parts, drone);
        if (!ok) return std::unexpected(ok.error());
    }

    // the blocks, from the parts' specs; the motor, ESC and pack carry mass 0 (their parts' records are the mass)
    bool have_motor = false, have_prop = false, have_esc = false, have_battery = false;
    for (const PartsMapSlot& slot : drone.parts_map) {
        const PartData& part = parts.at(slot.part);
        if (!have_motor) {
            if (const MotorPart* m = spec_block<MotorPart>(part)) {
                spec.motor = m->block;
                spec.motor.mass = 0.0;
                spec.motor.shape.reset();
                have_motor = true;
            }
        }
        if (!have_prop) {
            if (const PropPart* p = spec_block<PropPart>(part)) {
                spec.prop = p->block;
                spec.prop.mass = 0.0;  // the propeller's mass is its disc record's (above)
                spec.prop.shape.reset();
                have_prop = true;
            }
        }
        if (!have_esc) {
            if (const EscPart* e = spec_block<EscPart>(part)) {
                spec.esc = e->block;
                spec.esc.mass = 0.0;
                spec.esc.shape.reset();
                have_esc = true;
            }
        }
        if (!have_battery) {
            if (const BatteryPart* b = spec_block<BatteryPart>(part)) {
                spec.battery = b->block;
                spec.battery.mass = 0.0;
                spec.battery.shape.reset();
                have_battery = true;
            }
        }
    }
    if (!have_motor) return std::unexpected(refuse("invalid_entry", "no part of the drone carries a motor spec", &drone, "parts_map"));
    if (!have_prop) return std::unexpected(refuse("invalid_entry", "no part of the drone carries a prop spec", &drone, "parts_map"));
    if (!have_esc) return std::unexpected(refuse("invalid_entry", "no part of the drone carries an esc spec", &drone, "parts_map"));
    if (!have_battery) return std::unexpected(refuse("invalid_entry", "no part of the drone carries a battery spec", &drone, "parts_map"));

    // the rotors: one per motor slot, in the map's order; the hub is its propeller's `disc` point, the spin its motor point's
    for (const PartsMapSlot& slot : drone.parts_map) {
        const PartData& part = parts.at(slot.part);
        if (part.entry->part_kind != "motor") continue;
        const PartsMapSlot* prop_slot = nullptr;
        for (const PartsMapSlot& other : drone.parts_map) {
            if (other.parent == slot.slot && parts.at(other.part).entry->part_kind == "prop") prop_slot = &other;
        }
        if (prop_slot == nullptr) return std::unexpected(refuse("invalid_entry", "motor slot '" + slot.slot + "' carries no propeller", &drone, "parts_map"));
        Pose hub = placed.at(prop_slot->slot);
        if (const PartInstallPoint* disc = point_of(parts.at(prop_slot->part).blocks, "disc")) {
            hub = Pose{hub.p + hub.q * disc->pos, hub.q * disc->orient};
        }
        int spin = 0;
        if (!slot.parent.empty()) {
            if (const PartInstallPoint* ip = point_of(parts.at(part_of.at(slot.parent)).blocks, slot.install_point)) spin = ip->spin;
        }
        vehicles::RotorSpec rotor;
        rotor.hub.position = to_spade(hub.p);
        rotor.hub.orientation = to_spade(hub.q);
        rotor.spin_dir = static_cast<double>(spin);
        spec.rotors.push_back(rotor);
    }
    if (spec.rotors.empty()) return std::unexpected(refuse("invalid_entry", "the drone has no motor slot", &drone, "parts_map"));

    Result<vehicles::CompiledAirframe> compiled = vehicles::compile_airframe(spec);
    if (!compiled) return std::unexpected(refuse("compile_refused", compiled.error().context, &drone));
    CompiledDrone out;
    out.spec = std::move(spec);
    out.compiled = std::move(*compiled);
    return out;
}

}  // namespace spade::scene
