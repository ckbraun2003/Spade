// Unit shapes as meshes -- a box, a sphere and a cylinder spanning
// [-0.5, 0.5], flat-shaded -- that the sandbox's views scale and place:
// drones' bodies, arms and rotors, and the editor's selection. Display-free
// (SL15b). Moved from builder_scene.hpp when the builder mode left the
// sandbox (INT-9).
//
// Built here rather than pulled from the engine's tessellator on purpose: SL2b
// limits this target to headers the engine INSTALLS. These are small, exact,
// and the sandbox's own.

#pragma once

#include <cmath>
#include <cstdint>

#include <glm/glm.hpp>

#include "render/scene.hpp"

namespace spade::sandbox {

namespace detail {

inline void push_tri(render::MeshData& m, const glm::vec3& a, const glm::vec3& b,
                     const glm::vec3& c) {
    const glm::vec3 n = glm::normalize(glm::cross(b - a, c - a));
    const uint32_t base = static_cast<uint32_t>(m.positions.size());
    m.positions.push_back(a);
    m.positions.push_back(b);
    m.positions.push_back(c);
    m.normals.push_back(n);
    m.normals.push_back(n);
    m.normals.push_back(n);
    m.indices.push_back(base);
    m.indices.push_back(base + 1u);
    m.indices.push_back(base + 2u);
}

inline void finish(render::MeshData& m) {
    m.submesh_first_index = {0u};
    m.submesh_index_count = {static_cast<uint32_t>(m.indices.size())};
    m.submesh_material = {0u};
}

}  // namespace detail

// The size parameters let a LIT part be built at its real dimensions and placed
// with rotation and translation only, so its frame stays conformal. Since
// 1cac77a both shading paths use the cofactor for a non-conformal frame, so a
// scaled box also shades correctly. The defaults are the unit meshes a view
// scales.
[[nodiscard]] inline render::MeshData make_box_mesh(const glm::vec3& half = glm::vec3(0.5f)) {
    render::MeshData m;
    const float x = half.x, y = half.y, z = half.z;
    const glm::vec3 p[8] = {
        {-x, -y, -z}, {x, -y, -z}, {x, y, -z}, {-x, y, -z},
        {-x, -y, z},  {x, -y, z},  {x, y, z},  {-x, y, z},
    };
    // Counter-clockwise seen from outside.
    const int faces[6][4] = {
        {4, 5, 6, 7},  // +z
        {1, 0, 3, 2},  // -z
        {5, 1, 2, 6},  // +x
        {0, 4, 7, 3},  // -x
        {3, 7, 6, 2},  // +y
        {0, 1, 5, 4},  // -y
    };
    for (const auto& f : faces) {
        detail::push_tri(m, p[f[0]], p[f[1]], p[f[2]]);
        detail::push_tri(m, p[f[0]], p[f[2]], p[f[3]]);
    }
    detail::finish(m);
    return m;
}

[[nodiscard]] inline render::MeshData make_sphere_mesh(uint32_t rings = 12u,
                                                       uint32_t sectors = 18u) {
    render::MeshData m;
    const float r = 0.5f;
    const float pi = 3.14159265358979323846f;
    auto at = [&](uint32_t ri, uint32_t si) {
        const float phi = pi * static_cast<float>(ri) / static_cast<float>(rings);
        const float theta = 2.0f * pi * static_cast<float>(si) / static_cast<float>(sectors);
        return glm::vec3(r * std::sin(phi) * std::cos(theta), r * std::cos(phi),
                         r * std::sin(phi) * std::sin(theta));
    };
    for (uint32_t ri = 0; ri < rings; ++ri) {
        for (uint32_t si = 0; si < sectors; ++si) {
            const glm::vec3 a = at(ri, si);
            const glm::vec3 b = at(ri + 1u, si);
            const glm::vec3 c = at(ri + 1u, si + 1u);
            const glm::vec3 d = at(ri, si + 1u);
            // Counter-clockwise seen from OUTSIDE, like the box. At the north
            // pole a == d and at the south pole b == c, so the triangle that
            // collapses there is the one skipped -- the two conditions were
            // once the other way round, which left a hole at each pole and a
            // zero-area triangle with a NaN normal in its place.
            if (ri + 1u != rings) {
                detail::push_tri(m, a, c, b);
            }
            if (ri != 0u) {
                detail::push_tri(m, a, d, c);
            }
        }
    }
    detail::finish(m);
    return m;
}

[[nodiscard]] inline render::MeshData make_cylinder_mesh(uint32_t sectors = 20u, float radius = 0.5f,
                                                         float half_height = 0.5f) {
    render::MeshData m;
    const float r = radius, h = half_height;
    const float pi = 3.14159265358979323846f;
    auto ring = [&](uint32_t si, float y) {
        const float t = 2.0f * pi * static_cast<float>(si) / static_cast<float>(sectors);
        return glm::vec3(r * std::cos(t), y, r * std::sin(t));
    };
    for (uint32_t si = 0; si < sectors; ++si) {
        const glm::vec3 a = ring(si, -h), b = ring(si + 1u, -h);
        const glm::vec3 c = ring(si + 1u, h), d = ring(si, h);
        // Counter-clockwise seen from OUTSIDE, like the box. The ring runs
        // from +X toward +Z, so the outward side triangles are (a, c, b) and
        // (a, d, c).
        detail::push_tri(m, a, c, b);
        detail::push_tri(m, a, d, c);
        detail::push_tri(m, glm::vec3(0.0f, h, 0.0f), c, d);     // top cap, normal +Y
        detail::push_tri(m, glm::vec3(0.0f, -h, 0.0f), a, b);    // bottom cap, normal -Y
    }
    detail::finish(m);
    return m;
}

}  // namespace spade::sandbox
