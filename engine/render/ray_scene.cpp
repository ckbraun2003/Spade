#include "render/ray_scene.hpp"

#include <algorithm>
#include <cmath>
#include <string>
#include <utility>

#include "render/raymarch.hpp"
#include "world/sdf.hpp"

namespace spade::render {

namespace {

constexpr uint32_t kLeafTriangles = 4;           // a leaf holds at most this many
constexpr double kBoxPad = 1e-9;                  // metres: a slab test never loses a triangle that lies on its box's face
constexpr double kParallel = 1e-14;               // |det| below this is a ray parallel to the triangle
constexpr float kSubtreeBoundsMargin = 1e-3f;     // metres added round a CSG subtree's bounds before the march is clipped to them
constexpr uint32_t kNoId = 0xFFFFFFFFu;

// The part of the ray within [t0, t1] that lies inside the box, as [near, far], or false for a miss. A zero direction component is handled apart: no division, and
// the origin must lie within that slab.
[[nodiscard]] bool slab(const glm::dvec3& lo, const glm::dvec3& hi, const glm::dvec3& o, const glm::dvec3& d, double t0, double t1, double& near_t, double& far_t) {
    for (int a = 0; a < 3; ++a) {
        if (d[a] == 0.0) {
            if (o[a] < lo[a] || o[a] > hi[a]) return false;
            continue;
        }
        const double inv = 1.0 / d[a];
        double ta = (lo[a] - o[a]) * inv;
        double tb = (hi[a] - o[a]) * inv;
        if (ta > tb) std::swap(ta, tb);
        t0 = ta > t0 ? ta : t0;
        t1 = tb < t1 ? tb : t1;
        if (t0 > t1) return false;
    }
    near_t = t0;
    far_t = t1;
    return true;
}

// The scene's material index for a (override, submesh material) pair, resolved as the raster does: the override when set, else the submesh's; an index past the table
// falls back to the default material 0.
[[nodiscard]] uint32_t resolve_material(const RenderScene& scene, uint32_t override_index, uint32_t submesh_material) {
    const uint32_t index = override_index != kNoMaterial ? override_index : submesh_material;
    return index < scene.materials.size() ? index : 0u;
}

[[nodiscard]] glm::dvec3 world_point(const glm::dmat4& l2w, const glm::vec3& p) { return glm::dvec3(l2w * glm::dvec4(glm::dvec3(p), 1.0)); }

}  // namespace

Result<RayScene> RayScene::build(const RenderScene& scene) {
    RayScene rs;

    // statics that are not triangles here: a CSG subtree's item (marched exactly) and a ground plane's tessellated grid (the plane is closed form)
    std::vector<uint8_t> skip(scene.statics.size(), 0u);
    for (std::size_t i = 0; i < scene.csg_subtrees.size(); ++i) {
        const CsgSubtree& c = scene.csg_subtrees[i];
        if (c.draw_item >= scene.statics.size()) {
            return std::unexpected(Error{Code::invalid_argument, "RayScene::build: CSG subtree " + std::to_string(i) + "'s draw item " + std::to_string(c.draw_item) +
                                                                     " is not a static (there are " + std::to_string(scene.statics.size()) + ")"});
        }
        skip[c.draw_item] = 1u;
        Subtree st;
        st.csg = c;
        const DrawItem& item = scene.statics[c.draw_item];
        uint32_t submesh_material = 0u;
        if (item.mesh_index < scene.meshes.size() && !scene.meshes[item.mesh_index].submesh_material.empty()) {
            submesh_material = scene.meshes[item.mesh_index].submesh_material[0];
        }
        st.material = resolve_material(scene, item.material_override, submesh_material);
        st.label = item.object_label;
        rs.subtrees_.push_back(std::move(st));
    }
    for (const GroundPlane& g : scene.ground_planes) {
        if (g.item < scene.statics.size()) skip[g.item] = 1u;
        Ground gr;
        const glm::dvec3 n(g.normal);
        const double len = std::sqrt(glm::dot(n, n));
        gr.normal = len > 0.0 ? n / len : glm::dvec3(0.0, 1.0, 0.0);
        gr.offset = static_cast<double>(g.offset);
        gr.material = g.material < scene.materials.size() ? g.material : 0u;
        gr.label = g.label;
        rs.grounds_.push_back(gr);
    }

    // the world-space triangles of every drawn mesh item, in the build's own order
    std::vector<Triangle> tris;
    uint32_t next_id = 0;
    auto add_item = [&](const DrawItem& item, uint32_t item_index) {
        if (item.mesh_index >= scene.meshes.size()) return;
        const MeshData& mesh = scene.meshes[item.mesh_index];
        const glm::dmat4 l2w(item.local_to_world);
        const bool has_normals = mesh.normals.size() == mesh.positions.size();
        const bool has_submeshes = !mesh.submesh_first_index.empty();
        const std::size_t submesh_count =
            has_submeshes ? std::min({mesh.submesh_first_index.size(), mesh.submesh_index_count.size(), mesh.submesh_material.size()}) : std::size_t{1};
        for (std::size_t s = 0; s < submesh_count; ++s) {
            const uint32_t first = has_submeshes ? mesh.submesh_first_index[s] : 0u;
            const uint32_t count = has_submeshes ? mesh.submesh_index_count[s] : static_cast<uint32_t>(mesh.indices.size());
            if (first > mesh.indices.size() || count > mesh.indices.size() - first) continue;  // a range past the index buffer: skip it, not the mesh
            const uint32_t material = resolve_material(scene, item.material_override, has_submeshes ? mesh.submesh_material[s] : 0u);
            for (uint32_t k = 0; k + 2 < count; k += 3) {
                const uint32_t i0 = mesh.indices[first + k], i1 = mesh.indices[first + k + 1], i2 = mesh.indices[first + k + 2];
                if (i0 >= mesh.positions.size() || i1 >= mesh.positions.size() || i2 >= mesh.positions.size()) continue;
                Triangle t;
                t.a = world_point(l2w, mesh.positions[i0]);
                t.e1 = world_point(l2w, mesh.positions[i1]) - t.a;
                t.e2 = world_point(l2w, mesh.positions[i2]) - t.a;
                const glm::dvec3 g = glm::cross(t.e1, t.e2);
                const double glen = std::sqrt(glm::dot(g, g));
                if (!(glen > 0.0)) continue;  // a degenerate triangle has no surface
                const glm::vec3 geometric(g / glen);
                if (has_normals) {
                    t.n0 = transform_normal(item.local_to_world, mesh.normals[i0]);
                    t.n1 = transform_normal(item.local_to_world, mesh.normals[i1]);
                    t.n2 = transform_normal(item.local_to_world, mesh.normals[i2]);
                } else {
                    t.n0 = t.n1 = t.n2 = geometric;
                }
                t.item = item_index;
                t.index = (first + k) / 3u;
                t.material = material;
                t.id = next_id++;
                tris.push_back(t);
            }
        }
    };
    for (std::size_t i = 0; i < scene.statics.size(); ++i) {
        rs.labels_.push_back(scene.statics[i].object_label);
        if (skip[i] == 0u) add_item(scene.statics[i], static_cast<uint32_t>(i));
    }
    for (std::size_t i = 0; i < scene.dynamics.size(); ++i) {
        rs.labels_.push_back(scene.dynamics[i].object_label);
        add_item(scene.dynamics[i], static_cast<uint32_t>(scene.statics.size() + i));
    }

    // the BVH: median split on the longest centroid axis, ordered by (centroid, id), a total order, so the tree is the same on every platform
    auto centroid = [](const Triangle& t) { return t.a + (t.e1 + t.e2) * (1.0 / 3.0); };
    std::vector<Node>& nodes = rs.nodes_;
    auto build_node = [&](auto&& self, std::size_t lo, std::size_t hi) -> uint32_t {
        const uint32_t index = static_cast<uint32_t>(nodes.size());
        nodes.emplace_back();
        glm::dvec3 box_lo(std::numeric_limits<double>::infinity()), box_hi(-std::numeric_limits<double>::infinity());
        glm::dvec3 c_lo = box_lo, c_hi = box_hi;
        for (std::size_t i = lo; i < hi; ++i) {
            const Triangle& t = tris[i];
            for (const glm::dvec3& p : {t.a, t.a + t.e1, t.a + t.e2}) {
                box_lo = glm::min(box_lo, p);
                box_hi = glm::max(box_hi, p);
            }
            const glm::dvec3 c = centroid(t);
            c_lo = glm::min(c_lo, c);
            c_hi = glm::max(c_hi, c);
        }
        nodes[index].lo = box_lo - glm::dvec3(kBoxPad);
        nodes[index].hi = box_hi + glm::dvec3(kBoxPad);
        if (hi - lo <= kLeafTriangles) {
            nodes[index].first = static_cast<uint32_t>(lo);
            nodes[index].count = static_cast<uint32_t>(hi - lo);
            return index;
        }
        const glm::dvec3 span = c_hi - c_lo;
        const int axis = (span.x >= span.y && span.x >= span.z) ? 0 : (span.y >= span.z ? 1 : 2);
        std::sort(tris.begin() + static_cast<std::ptrdiff_t>(lo), tris.begin() + static_cast<std::ptrdiff_t>(hi), [&](const Triangle& a, const Triangle& b) {
            const double ca = centroid(a)[axis], cb = centroid(b)[axis];
            return ca != cb ? ca < cb : a.id < b.id;
        });
        const std::size_t mid = lo + (hi - lo) / 2;
        const uint32_t left = self(self, lo, mid);
        const uint32_t right = self(self, mid, hi);
        nodes[index].first = left;
        nodes[index].right = right;
        nodes[index].count = 0;
        return index;
    };
    if (!tris.empty()) build_node(build_node, 0, tris.size());
    rs.triangles_ = std::move(tris);
    return rs;
}

RayHit RayScene::trace(const Ray& ray) const {
    const glm::dvec3& o = ray.origin;
    const glm::dvec3& d = ray.direction;
    double best_t = ray.t_max;
    uint32_t best_id = kNoId;
    RayHit hit;
    glm::vec3 shading_normal{0.0f};

    // --- triangles ---
    if (!nodes_.empty()) {
        uint32_t stack[128];
        int sp = 0;
        stack[sp++] = 0u;
        while (sp > 0) {
            const Node& n = nodes_[stack[--sp]];
            double near_t = 0.0, far_t = 0.0;
            if (!slab(n.lo, n.hi, o, d, ray.t_min, best_t, near_t, far_t)) continue;
            if (n.count > 0) {
                for (uint32_t i = n.first; i < n.first + n.count; ++i) {
                    const Triangle& tr = triangles_[i];
                    const glm::dvec3 p = glm::cross(d, tr.e2);
                    const double det = glm::dot(tr.e1, p);
                    if (std::abs(det) < kParallel) continue;
                    const glm::dvec3 s = o - tr.a;
                    const double u = glm::dot(s, p) / det;
                    if (u < 0.0 || u > 1.0) continue;
                    const glm::dvec3 q = glm::cross(s, tr.e1);
                    const double v = glm::dot(d, q) / det;
                    if (v < 0.0 || u + v > 1.0) continue;
                    const double t = glm::dot(tr.e2, q) / det;
                    if (!(t > ray.t_min)) continue;
                    if (t < best_t || (t == best_t && tr.id < best_id)) {
                        best_t = t;
                        best_id = tr.id;
                        hit.kind = RayHitKind::triangle;
                        hit.item = tr.item;
                        hit.triangle = tr.index;
                        hit.material = tr.material;
                        hit.label = kNoLabel;  // filled below from the item
                        const glm::vec3 nn = tr.n0 * static_cast<float>(1.0 - u - v) + tr.n1 * static_cast<float>(u) + tr.n2 * static_cast<float>(v);
                        const float len = std::sqrt(glm::dot(nn, nn));
                        shading_normal = len > 0.0f ? nn / len : glm::vec3(glm::normalize(glm::cross(tr.e1, tr.e2)));
                    }
                }
            } else {
                const Node& l = nodes_[n.first];
                const Node& r = nodes_[n.right];
                double el = 0.0, er = 0.0, fl = 0.0, fr = 0.0;
                const bool hl = slab(l.lo, l.hi, o, d, ray.t_min, best_t, el, fl);
                const bool hr = slab(r.lo, r.hi, o, d, ray.t_min, best_t, er, fr);
                if (hl && hr) {
                    stack[sp++] = el <= er ? n.right : n.first;  // the farther first, so the nearer is popped first
                    stack[sp++] = el <= er ? n.first : n.right;
                } else if (hl) {
                    stack[sp++] = n.first;
                } else if (hr) {
                    stack[sp++] = n.right;
                }
            }
        }
    }

    // --- CSG subtrees: marched exactly inside their bounds ---
    uint32_t sdf_label = kNoLabel;
    uint16_t sdf_leaf_label = 0;
    glm::vec3 sdf_normal{0.0f};
    for (const Subtree& st : subtrees_) {
        const glm::dvec3 lo = glm::dvec3(st.csg.bounds.min) - glm::dvec3(static_cast<double>(kSubtreeBoundsMargin));
        const glm::dvec3 hi = glm::dvec3(st.csg.bounds.max) + glm::dvec3(static_cast<double>(kSubtreeBoundsMargin));
        double t0 = 0.0, t1 = 0.0;
        if (!slab(lo, hi, o, d, ray.t_min, best_t, t0, t1)) continue;
        if (!(t0 < t1)) continue;
        const MarchHit march = sphere_trace(st.csg.program, glm::vec3(o), glm::vec3(d), static_cast<float>(t0), static_cast<float>(t1));
        if (!march.hit) continue;
        const double t = static_cast<double>(march.t);
        if (!(t > ray.t_min) || !(t < best_t)) continue;
        best_t = t;
        best_id = kNoId - 1u;
        hit.kind = RayHitKind::sdf;
        hit.item = st.csg.draw_item;
        hit.triangle = kNoMesh;
        hit.material = st.material;
        sdf_label = st.label;
        sdf_leaf_label = spade::sdf_hit_label(st.csg.program, march.p);
        const glm::vec3 raw = spade::gradient(st.csg.program, march.p);
        const float len = std::sqrt(glm::dot(raw, raw));
        sdf_normal = len > 0.0f ? raw / len : glm::vec3(0.0f, 1.0f, 0.0f);
    }

    // --- the analytic ground: closed form, hit from the open side only ---
    uint32_t ground_label = kNoLabel;
    glm::vec3 ground_normal{0.0f};
    for (const Ground& g : grounds_) {
        const double denom = glm::dot(g.normal, d);
        if (!(denom < 0.0)) continue;  // away from or along the plane, or from inside the solid side
        const double t = (g.offset - glm::dot(g.normal, o)) / denom;
        if (!(t > ray.t_min) || !(t < best_t)) continue;
        best_t = t;
        hit.kind = RayHitKind::ground;
        hit.item = kNoMesh;
        hit.triangle = kNoMesh;
        hit.material = g.material;
        ground_label = g.label;
        ground_normal = glm::vec3(g.normal);
    }

    if (hit.kind == RayHitKind::none) return hit;
    hit.t = best_t;
    hit.point = o + d * best_t;
    glm::vec3 n;
    if (hit.kind == RayHitKind::triangle) {
        n = shading_normal;
        hit.label = hit.item < labels_.size() ? labels_[hit.item] : kNoLabel;
    } else if (hit.kind == RayHitKind::sdf) {
        n = sdf_normal;
        hit.label = sdf_label;
        hit.sdf_label = sdf_leaf_label;
    } else {
        n = ground_normal;
        hit.label = ground_label;
    }
    hit.front_face = glm::dot(glm::dvec3(n), d) < 0.0;
    hit.normal = hit.front_face ? n : -n;
    return hit;
}

}  // namespace spade::render
