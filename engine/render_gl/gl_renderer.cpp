// GlRenderer -- see the header for why this exists and what it deliberately is
// not.

#include "render_gl/gl_renderer.hpp"

#include <cstddef>
#include <string>
#include <unordered_map>
#include <vector>

#include <glad/glad.h>

// ⚠ THE VENDORED GLAD IS GENERATED UP TO GL 4.1, AND SSBOs ARE 4.3. Every
// FUNCTION this file calls is 4.1 or earlier and is present; the only thing
// missing is the enum. Defined here behind an #ifndef, with the value from the
// GL registry, which has never changed.
//
// ⭐ THIRD INSTANCE TODAY OF THE SAME SPECIES: WHAT A LOADER DECLARES IS A
// PROPERTY OF HOW IT WAS GENERATED, NOT OF THE GL SPEC. ImGui's bundled loader
// is filtered to what ImGui uses (no GL_BGRA, no glTexSubImage2D); Windows'
// <GL/gl.h> stops at 1.1 (no GL_CLAMP_TO_EDGE); this glad stops at 4.1. v1 hit
// the identical wall and worked around it the same way -- Engine.cpp loads
// glDispatchCompute and friends by hand for exactly this reason.
#ifndef GL_SHADER_STORAGE_BUFFER
#define GL_SHADER_STORAGE_BUFFER 0x90D2
#endif

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtc/type_ptr.hpp>

#include "core/error.hpp"

namespace spade::render_gl {
namespace {

// ---------------------------------------------------------------------------
// The shaders, embedded. See the header: v1 loads these from a CWD-relative
// path and dies "with no window ever opening" when launched from elsewhere.
// A renderer whose correctness depends on the caller's working directory has a
// failure mode it does not need.
//
// std430 layouts are written out explicitly rather than trusted to match a C++
// struct by luck -- GpuMaterial below pads to 32 bytes on purpose, because
// vec4+uint is 20 bytes of data with 16-byte alignment and a silent mismatch
// here reads as "the colours are wrong" three files away.
// ---------------------------------------------------------------------------

constexpr const char* kVertexSrc = R"GLSL(#version 430 core
layout(location = 0) in vec3 aPos;
layout(location = 1) in vec3 aNrm;

// THE MECHANISM. Per-instance transforms live here and the CPU never touches
// one at draw time; the shader indexes by gl_InstanceID plus the base this
// mesh's batch starts at -- v1's `instanceStartIndex`, same idea.
layout(std430, binding = 0) readonly buffer Transforms { mat4 uModel[]; };

uniform uint uInstanceBase;
uniform mat4 uViewProj;

out vec3 vNormal;
flat out uint vInstance;

void main() {
    uint idx = uInstanceBase + uint(gl_InstanceID);
    mat4 m = uModel[idx];
    vec4 world = m * vec4(aPos, 1.0);
    // Use the inverse-transpose: mat3(m) tilts normals under a per-axis scale.
    // The cofactor matrix is det * inverse-transpose. Det's sign keeps a
    // mirrored normal outward, and the fragment shader normalizes.
    // render/scene.hpp's transform_normal() uses the same rule. It keeps
    // mat3(m) for conformal matrices so CPU goldens stay bit-identical.
    // GL has no goldens, so it does not need that branch.
    mat3 l = mat3(m);
    mat3 cofactor = mat3(cross(l[1], l[2]), cross(l[2], l[0]), cross(l[0], l[1]));
    vNormal = cofactor * aNrm * sign(dot(l[0], cofactor[0]));
    vInstance = idx;
    gl_Position = uViewProj * world;
}
)GLSL";

constexpr const char* kFragmentSrc = R"GLSL(#version 430 core
in vec3 vNormal;
flat in uint vInstance;

struct GpuMaterial {
    vec4 base_color;
    uint shading;
    uint pad0;
    uint pad1;
    uint pad2;
};

layout(std430, binding = 1) readonly buffer Overrides  { uint uOverride[]; };
layout(std430, binding = 2) readonly buffer MaterialSet { GpuMaterial uMaterials[]; };

uniform uint uSubmeshMaterial;
uniform uint uNoMaterial;
uniform vec3 uSunDir;
uniform vec3 uSunColor;
uniform vec3 uAmbient;
uniform float uSunIntensity;

out vec4 fragColor;

void main() {
    // DrawItem::material_override wins when set, else the submesh's own
    // material -- the same precedence raster_cpu applies, stated once here.
    uint mi = uOverride[vInstance];
    if (mi == uNoMaterial) { mi = uSubmeshMaterial; }
    GpuMaterial m = uMaterials[mi];

    vec3 base = m.base_color.rgb;
    vec3 lit;
    if (m.shading == 1u || m.shading == 2u) {
        lit = base;                       // 1 = unlit, 2 = emissive
    } else {
        vec3 n = normalize(vNormal);
        // uSunDir points FROM the scene TOWARD the sun (LightingDesc's
        // convention), so N.L is dot(n, +uSunDir) -- the same term as
        // render/scene.hpp's shade_vertex_color(). Negating it lit every
        // world-loaded scene from below on this path only.
        float ndl = max(dot(n, uSunDir), 0.0);
        lit = base * (uAmbient + uSunColor * uSunIntensity * ndl);
    }
    fragColor = vec4(lit, 1.0);
}
)GLSL";

// 32 bytes, explicitly padded. See the shader comment above.
struct GpuMaterial {
    glm::vec4 base_color{0.72f, 0.72f, 0.74f, 1.0f};
    uint32_t shading = 0;
    uint32_t pad0 = 0, pad1 = 0, pad2 = 0;
};
static_assert(sizeof(GpuMaterial) == 32, "std430 layout must match the shader's GpuMaterial");

struct GpuMesh {
    GLuint vao = 0, vbo_pos = 0, vbo_nrm = 0, ebo = 0;
    uint32_t index_count = 0;
    std::vector<uint32_t> submesh_first_index;
    std::vector<uint32_t> submesh_index_count;
    std::vector<uint32_t> submesh_material;
};

[[nodiscard]] Result<GLuint> compile(GLenum stage, const char* src, const char* what) {
    const GLuint sh = glCreateShader(stage);
    glShaderSource(sh, 1, &src, nullptr);
    glCompileShader(sh);
    GLint ok = GL_FALSE;
    glGetShaderiv(sh, GL_COMPILE_STATUS, &ok);
    if (ok != GL_TRUE) {
        GLint len = 0;
        glGetShaderiv(sh, GL_INFO_LOG_LENGTH, &len);
        std::string log(static_cast<size_t>(len > 0 ? len : 1), '\0');
        glGetShaderInfoLog(sh, len, nullptr, log.data());
        glDeleteShader(sh);
        // The driver's own log, forwarded verbatim. A shader failure reported
        // as "compile failed" sends the reader to guess; the log names a line.
        return std::unexpected(Error{Code::internal, std::string(what) + " shader: " + log});
    }
    return sh;
}

}  // namespace

// ---------------------------------------------------------------------------

struct GlRenderer::Impl {
    GLuint program = 0;
    std::vector<GpuMesh> meshes;

    GLuint ssbo_transforms = 0;
    GLuint ssbo_overrides = 0;
    GLuint ssbo_materials = 0;
    size_t transforms_capacity = 0;  // in instances, so growth is in place after the first
    size_t overrides_capacity = 0;
    size_t materials_capacity = 0;

    // Scratch, reused every frame so a frame allocates nothing steady-state.
    std::vector<glm::mat4> instance_transforms;
    std::vector<uint32_t> instance_overrides;
    std::vector<uint32_t> batch_base;   // per mesh: where its instances start
    std::vector<uint32_t> batch_count;  // per mesh: how many
    std::vector<GpuMaterial> gpu_materials;

    uint32_t last_draw_calls = 0;
    uint32_t last_instances = 0;

    std::string renderer_name;
    std::string version_string;

    GLint u_instance_base = -1, u_view_proj = -1, u_submesh_material = -1, u_no_material = -1;
    GLint u_sun_dir = -1, u_sun_color = -1, u_ambient = -1, u_sun_intensity = -1;

    ~Impl() {
        for (GpuMesh& m : meshes) {
            if (m.ebo != 0) glDeleteBuffers(1, &m.ebo);
            if (m.vbo_nrm != 0) glDeleteBuffers(1, &m.vbo_nrm);
            if (m.vbo_pos != 0) glDeleteBuffers(1, &m.vbo_pos);
            if (m.vao != 0) glDeleteVertexArrays(1, &m.vao);
        }
        if (ssbo_materials != 0) glDeleteBuffers(1, &ssbo_materials);
        if (ssbo_overrides != 0) glDeleteBuffers(1, &ssbo_overrides);
        if (ssbo_transforms != 0) glDeleteBuffers(1, &ssbo_transforms);
        if (program != 0) glDeleteProgram(program);
    }
};

GlRenderer::GlRenderer(std::unique_ptr<Impl> impl) : impl_(std::move(impl)) {}
GlRenderer::~GlRenderer() = default;

Result<std::unique_ptr<GlRenderer>> GlRenderer::create(GlProcLoader loader) {
    if (loader == nullptr) {
        return std::unexpected(Error{Code::invalid_argument, "GlRenderer::create: null proc loader"});
    }
    if (gladLoadGLLoader(reinterpret_cast<GLADloadproc>(loader)) == 0) {
        return std::unexpected(Error{Code::unavailable,
                                     "GlRenderer::create: could not load GL entry points -- is a GL "
                                     "context current on this thread?"});
    }

    auto impl = std::make_unique<Impl>();
    const auto* rend = reinterpret_cast<const char*>(glGetString(GL_RENDERER));
    const auto* vers = reinterpret_cast<const char*>(glGetString(GL_VERSION));
    impl->renderer_name = rend != nullptr ? rend : "(unknown)";
    impl->version_string = vers != nullptr ? vers : "(unknown)";

    // ⚠ REFUSED, NOT DEGRADED. The entire mechanism rests on SSBOs; without
    // them this class cannot do the thing it exists for, and a renderer that
    // quietly substitutes something slower is indistinguishable from the defect
    // it was written to fix. Code::unavailable specifically, so a caller can
    // fall back to raster_cpu by switching on it -- which is exactly what the
    // user's "GPU default, CPU fallback" ruling asks of a caller.
    GLint major = 0, minor = 0;
    glGetIntegerv(GL_MAJOR_VERSION, &major);
    glGetIntegerv(GL_MINOR_VERSION, &minor);
    if (major < 4 || (major == 4 && minor < 3)) {
        return std::unexpected(Error{
            Code::unavailable, "GlRenderer::create: needs OpenGL 4.3 for shader storage buffers; "
                               "this context reports " +
                                   impl->version_string + " on " + impl->renderer_name});
    }

    const Result<GLuint> vs = compile(GL_VERTEX_SHADER, kVertexSrc, "vertex");
    if (!vs) return std::unexpected(vs.error());
    const Result<GLuint> fs = compile(GL_FRAGMENT_SHADER, kFragmentSrc, "fragment");
    if (!fs) {
        glDeleteShader(*vs);
        return std::unexpected(fs.error());
    }

    impl->program = glCreateProgram();
    glAttachShader(impl->program, *vs);
    glAttachShader(impl->program, *fs);
    glLinkProgram(impl->program);
    glDeleteShader(*vs);
    glDeleteShader(*fs);

    GLint linked = GL_FALSE;
    glGetProgramiv(impl->program, GL_LINK_STATUS, &linked);
    if (linked != GL_TRUE) {
        GLint len = 0;
        glGetProgramiv(impl->program, GL_INFO_LOG_LENGTH, &len);
        std::string log(static_cast<size_t>(len > 0 ? len : 1), '\0');
        glGetProgramInfoLog(impl->program, len, nullptr, log.data());
        return std::unexpected(Error{Code::internal, "GlRenderer::create: link: " + log});
    }

    const GLuint p = impl->program;
    impl->u_instance_base = glGetUniformLocation(p, "uInstanceBase");
    impl->u_view_proj = glGetUniformLocation(p, "uViewProj");
    impl->u_submesh_material = glGetUniformLocation(p, "uSubmeshMaterial");
    impl->u_no_material = glGetUniformLocation(p, "uNoMaterial");
    impl->u_sun_dir = glGetUniformLocation(p, "uSunDir");
    impl->u_sun_color = glGetUniformLocation(p, "uSunColor");
    impl->u_ambient = glGetUniformLocation(p, "uAmbient");
    impl->u_sun_intensity = glGetUniformLocation(p, "uSunIntensity");

    glGenBuffers(1, &impl->ssbo_transforms);
    glGenBuffers(1, &impl->ssbo_overrides);
    glGenBuffers(1, &impl->ssbo_materials);

    return std::unique_ptr<GlRenderer>(new GlRenderer(std::move(impl)));
}

Result<void> GlRenderer::upload_scene(const render::RenderScene& scene) {
    Impl& s = *impl_;

    // Geometry uploads ONCE per mesh and lives until the mesh set changes --
    // v1's `if (meshComponent.VAO == 0)` branch, and the reason a frame costs
    // nothing per vertex.
    for (GpuMesh& m : s.meshes) {
        if (m.ebo != 0) glDeleteBuffers(1, &m.ebo);
        if (m.vbo_nrm != 0) glDeleteBuffers(1, &m.vbo_nrm);
        if (m.vbo_pos != 0) glDeleteBuffers(1, &m.vbo_pos);
        if (m.vao != 0) glDeleteVertexArrays(1, &m.vao);
    }
    s.meshes.clear();
    s.meshes.resize(scene.meshes.size());

    for (size_t i = 0; i < scene.meshes.size(); ++i) {
        const render::MeshData& src = scene.meshes[i];
        GpuMesh& dst = s.meshes[i];

        if (src.positions.size() != src.normals.size()) {
            return std::unexpected(Error{Code::invalid_argument,
                                         "GlRenderer::upload_scene: mesh " + std::to_string(i) +
                                             " has " + std::to_string(src.positions.size()) +
                                             " positions and " + std::to_string(src.normals.size()) +
                                             " normals"});
        }

        glGenVertexArrays(1, &dst.vao);
        glBindVertexArray(dst.vao);

        glGenBuffers(1, &dst.vbo_pos);
        glBindBuffer(GL_ARRAY_BUFFER, dst.vbo_pos);
        glBufferData(GL_ARRAY_BUFFER,
                     static_cast<GLsizeiptr>(src.positions.size() * sizeof(glm::vec3)),
                     src.positions.data(), GL_STATIC_DRAW);
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(glm::vec3), nullptr);

        glGenBuffers(1, &dst.vbo_nrm);
        glBindBuffer(GL_ARRAY_BUFFER, dst.vbo_nrm);
        glBufferData(GL_ARRAY_BUFFER, static_cast<GLsizeiptr>(src.normals.size() * sizeof(glm::vec3)),
                     src.normals.data(), GL_STATIC_DRAW);
        glEnableVertexAttribArray(1);
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(glm::vec3), nullptr);

        glGenBuffers(1, &dst.ebo);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, dst.ebo);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER,
                     static_cast<GLsizeiptr>(src.indices.size() * sizeof(uint32_t)),
                     src.indices.data(), GL_STATIC_DRAW);

        glBindVertexArray(0);

        dst.index_count = static_cast<uint32_t>(src.indices.size());
        dst.submesh_first_index = src.submesh_first_index;
        dst.submesh_index_count = src.submesh_index_count;
        dst.submesh_material = src.submesh_material;

        // A mesh with no declared submeshes is ONE submesh covering everything,
        // material 0. Stated here rather than special-cased at draw time, so
        // the draw loop has exactly one shape.
        if (dst.submesh_first_index.empty()) {
            dst.submesh_first_index = {0u};
            dst.submesh_index_count = {dst.index_count};
            dst.submesh_material = {0u};
        }
    }

    // The scene's palette, uploaded with the geometry: it changes with the
    // scene, not with the frame.
    s.gpu_materials.clear();
    s.gpu_materials.reserve(scene.materials.empty() ? 1u : scene.materials.size());
    for (const render::Material& m : scene.materials) {
        GpuMaterial g;
        g.base_color = m.base_color;
        g.shading = m.shading;
        s.gpu_materials.push_back(g);
    }
    if (s.gpu_materials.empty()) {
        s.gpu_materials.push_back(GpuMaterial{});  // index 0 is always valid
    }
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, s.ssbo_materials);
    glBufferData(GL_SHADER_STORAGE_BUFFER,
                 static_cast<GLsizeiptr>(s.gpu_materials.size() * sizeof(GpuMaterial)),
                 s.gpu_materials.data(), GL_STATIC_DRAW);
    s.materials_capacity = s.gpu_materials.size();
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0);

    return {};
}

Result<void> GlRenderer::draw(const render::RenderScene& scene, const render::Camera& camera,
                              const render::RenderOptions& options, uint32_t width,
                              uint32_t height) {
    Impl& s = *impl_;
    (void)options;  // draw modes, overlays and shadows are later tasks; see the header

    if (s.meshes.size() != scene.meshes.size()) {
        return std::unexpected(Error{Code::invalid_argument,
                                     "GlRenderer::draw: this scene has " +
                                         std::to_string(scene.meshes.size()) +
                                         " meshes but " + std::to_string(s.meshes.size()) +
                                         " are uploaded -- call upload_scene() first"});
    }
    if (width == 0u || height == 0u) {
        return std::unexpected(Error{Code::invalid_argument, "GlRenderer::draw: zero framebuffer size"});
    }

    // --- group by mesh_index. This IS the mechanism. -----------------------
    const size_t mesh_count = s.meshes.size();
    s.batch_count.assign(mesh_count, 0u);
    for (const render::DrawItem& d : scene.statics) {
        if (d.mesh_index < mesh_count) ++s.batch_count[d.mesh_index];
    }
    for (const render::DrawItem& d : scene.dynamics) {
        if (d.mesh_index < mesh_count) ++s.batch_count[d.mesh_index];
    }
    s.batch_base.assign(mesh_count, 0u);
    uint32_t running = 0;
    for (size_t i = 0; i < mesh_count; ++i) {
        s.batch_base[i] = running;
        running += s.batch_count[i];
    }
    const uint32_t total_instances = running;

    s.instance_transforms.assign(total_instances, glm::mat4(1.0f));
    s.instance_overrides.assign(total_instances, render::kNoMaterial);
    std::vector<uint32_t> cursor(s.batch_base.begin(), s.batch_base.end());
    const auto place = [&](const render::DrawItem& d) {
        if (d.mesh_index >= mesh_count) return;
        const uint32_t at = cursor[d.mesh_index]++;
        s.instance_transforms[at] = d.local_to_world;
        s.instance_overrides[at] = d.material_override;
    };
    for (const render::DrawItem& d : scene.statics) place(d);
    for (const render::DrawItem& d : scene.dynamics) place(d);

    // --- instance buffers: allocate once, then SubData in place ------------
    // v1's contract, and the reason a steady-state frame does no reallocation.
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, s.ssbo_transforms);
    if (total_instances > s.transforms_capacity) {
        glBufferData(GL_SHADER_STORAGE_BUFFER,
                     static_cast<GLsizeiptr>(total_instances * sizeof(glm::mat4)),
                     s.instance_transforms.data(), GL_DYNAMIC_DRAW);
        s.transforms_capacity = total_instances;
    } else if (total_instances > 0u) {
        glBufferSubData(GL_SHADER_STORAGE_BUFFER, 0,
                        static_cast<GLsizeiptr>(total_instances * sizeof(glm::mat4)),
                        s.instance_transforms.data());
    }

    glBindBuffer(GL_SHADER_STORAGE_BUFFER, s.ssbo_overrides);
    if (total_instances > s.overrides_capacity) {
        glBufferData(GL_SHADER_STORAGE_BUFFER,
                     static_cast<GLsizeiptr>(total_instances * sizeof(uint32_t)),
                     s.instance_overrides.data(), GL_DYNAMIC_DRAW);
        s.overrides_capacity = total_instances;
    } else if (total_instances > 0u) {
        glBufferSubData(GL_SHADER_STORAGE_BUFFER, 0,
                        static_cast<GLsizeiptr>(total_instances * sizeof(uint32_t)),
                        s.instance_overrides.data());
    }
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0);

    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 0, s.ssbo_transforms);
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 1, s.ssbo_overrides);
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 2, s.ssbo_materials);

    // --- camera ------------------------------------------------------------
    // The engine's camera space is right-handed with forward = -Z
    // (raster_cpu.cpp states it), which is glm's convention, so the view is the
    // inverse of the camera's own rigid transform with no axis fixup.
    const glm::mat4 cam_to_world =
        glm::translate(glm::mat4(1.0f), camera.position) * glm::mat4_cast(camera.orientation);
    const glm::mat4 view = glm::inverse(cam_to_world);
    const float aspect = static_cast<float>(width) / static_cast<float>(height);
    const glm::mat4 proj =
        glm::perspective(camera.fov_y_radians, aspect, camera.near_plane, camera.far_plane);
    const glm::mat4 view_proj = proj * view;

    // --- draw --------------------------------------------------------------
    glViewport(0, 0, static_cast<GLsizei>(width), static_cast<GLsizei>(height));
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LESS);
    glClear(GL_DEPTH_BUFFER_BIT);
    // SR-13: shaded mode culls back faces, as raster_cpu does. Front is CCW
    // seen from the camera, the same convention as the CPU's edge-function
    // sign, so an outward-wound mesh loses nothing and an inward-wound one
    // shows its winding bug here as well as on the CPU.
    glEnable(GL_CULL_FACE);
    glCullFace(GL_BACK);
    glFrontFace(GL_CCW);

    glUseProgram(s.program);
    glUniformMatrix4fv(s.u_view_proj, 1, GL_FALSE, glm::value_ptr(view_proj));
    glUniform1ui(s.u_no_material, render::kNoMaterial);
    glUniform3fv(s.u_sun_dir, 1, glm::value_ptr(scene.lighting.sun_direction));
    glUniform3fv(s.u_sun_color, 1, glm::value_ptr(scene.lighting.sun_color));
    glUniform3fv(s.u_ambient, 1, glm::value_ptr(scene.lighting.ambient_color));
    glUniform1f(s.u_sun_intensity, scene.lighting.sun_intensity);

    uint32_t draw_calls = 0;
    for (size_t i = 0; i < mesh_count; ++i) {
        const uint32_t count = s.batch_count[i];
        if (count == 0u) continue;  // a mesh nothing instances costs nothing

        const GpuMesh& m = s.meshes[i];
        glBindVertexArray(m.vao);
        glUniform1ui(s.u_instance_base, s.batch_base[i]);

        // ONE DRAW PER SUBMESH, and submeshes exist because materials differ
        // within a mesh. Still O(meshes x submeshes) and never O(objects),
        // which is the property the whole module is for.
        for (size_t sm = 0; sm < m.submesh_first_index.size(); ++sm) {
            const uint32_t first = m.submesh_first_index[sm];
            const uint32_t n = m.submesh_index_count[sm];
            if (n == 0u) continue;
            glUniform1ui(s.u_submesh_material, m.submesh_material[sm]);
            glDrawElementsInstanced(
                GL_TRIANGLES, static_cast<GLsizei>(n), GL_UNSIGNED_INT,
                reinterpret_cast<const void*>(static_cast<uintptr_t>(first) * sizeof(uint32_t)),
                static_cast<GLsizei>(count));
            ++draw_calls;
        }
    }
    glBindVertexArray(0);
    glUseProgram(0);

    s.last_draw_calls = draw_calls;
    s.last_instances = total_instances;
    return {};
}

std::vector<std::string_view> GlRenderer::unhonoured(const render::RenderOptions& options) {
    (void)options;
    return {};  // stub: the test commit fails against it
}

uint32_t GlRenderer::last_draw_calls() const noexcept { return impl_->last_draw_calls; }
uint32_t GlRenderer::last_instances() const noexcept { return impl_->last_instances; }
const std::string& GlRenderer::renderer_name() const noexcept { return impl_->renderer_name; }
const std::string& GlRenderer::version_string() const noexcept { return impl_->version_string; }

}  // namespace spade::render_gl
