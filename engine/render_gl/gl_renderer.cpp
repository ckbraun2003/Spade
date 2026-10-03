// GlRenderer -- see the header for why this exists and what it deliberately is
// not.

#include "render_gl/gl_renderer.hpp"

#include <cstddef>
#include <cstring>
#include <initializer_list>
#include <string>
#include <string_view>
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
// This renderer's own GL entry points, loaded through the caller's
// GlProcLoader into its Impl. glad's header gives the types and enums only:
// no glad_gl* pointer and no gladLoad* call is referenced, so an installed
// spade::render_gl exports no GL loader symbol to clash with a consumer's
// own (Interface's plan 513e1b8). It is also not static mutable state.
// ---------------------------------------------------------------------------
#define SPADE_GL_FUNCTIONS(X) \
    X(PFNGLGETSTRINGPROC, GetString) \
    X(PFNGLATTACHSHADERPROC, AttachShader) \
    X(PFNGLBINDBUFFERPROC, BindBuffer) \
    X(PFNGLBINDBUFFERBASEPROC, BindBufferBase) \
    X(PFNGLBINDVERTEXARRAYPROC, BindVertexArray) \
    X(PFNGLBUFFERDATAPROC, BufferData) \
    X(PFNGLBUFFERSUBDATAPROC, BufferSubData) \
    X(PFNGLCLEARPROC, Clear) \
    X(PFNGLCOMPILESHADERPROC, CompileShader) \
    X(PFNGLCREATEPROGRAMPROC, CreateProgram) \
    X(PFNGLCREATESHADERPROC, CreateShader) \
    X(PFNGLCULLFACEPROC, CullFace) \
    X(PFNGLDELETEBUFFERSPROC, DeleteBuffers) \
    X(PFNGLDELETEPROGRAMPROC, DeleteProgram) \
    X(PFNGLDELETESHADERPROC, DeleteShader) \
    X(PFNGLDELETEVERTEXARRAYSPROC, DeleteVertexArrays) \
    X(PFNGLDEPTHFUNCPROC, DepthFunc) \
    X(PFNGLDEPTHMASKPROC, DepthMask) \
    X(PFNGLDISABLEPROC, Disable) \
    X(PFNGLDRAWARRAYSPROC, DrawArrays) \
    X(PFNGLDRAWARRAYSINSTANCEDPROC, DrawArraysInstanced) \
    X(PFNGLDRAWELEMENTSINSTANCEDPROC, DrawElementsInstanced) \
    X(PFNGLENABLEPROC, Enable) \
    X(PFNGLENABLEVERTEXATTRIBARRAYPROC, EnableVertexAttribArray) \
    X(PFNGLFRONTFACEPROC, FrontFace) \
    X(PFNGLGENBUFFERSPROC, GenBuffers) \
    X(PFNGLGENVERTEXARRAYSPROC, GenVertexArrays) \
    X(PFNGLGETINTEGERVPROC, GetIntegerv) \
    X(PFNGLGETPROGRAMINFOLOGPROC, GetProgramInfoLog) \
    X(PFNGLGETPROGRAMIVPROC, GetProgramiv) \
    X(PFNGLGETSHADERINFOLOGPROC, GetShaderInfoLog) \
    X(PFNGLGETSHADERIVPROC, GetShaderiv) \
    X(PFNGLGETUNIFORMLOCATIONPROC, GetUniformLocation) \
    X(PFNGLLINKPROGRAMPROC, LinkProgram) \
    X(PFNGLPOLYGONMODEPROC, PolygonMode) \
    X(PFNGLSHADERSOURCEPROC, ShaderSource) \
    X(PFNGLUNIFORM1FPROC, Uniform1f) \
    X(PFNGLUNIFORM1UIPROC, Uniform1ui) \
    X(PFNGLUNIFORM2FPROC, Uniform2f) \
    X(PFNGLUNIFORM2UIPROC, Uniform2ui) \
    X(PFNGLUNIFORM3FVPROC, Uniform3fv) \
    X(PFNGLUNIFORMMATRIX4FVPROC, UniformMatrix4fv) \
    X(PFNGLUSEPROGRAMPROC, UseProgram) \
    X(PFNGLVERTEXATTRIBPOINTERPROC, VertexAttribPointer) \
    X(PFNGLVIEWPORTPROC, Viewport)

struct GlApi {
#define SPADE_GL_MEMBER(type, name) type name = nullptr;
    SPADE_GL_FUNCTIONS(SPADE_GL_MEMBER)
#undef SPADE_GL_MEMBER
};

// A loader hands back void*; GL wants function pointers. memcpy rather than
// a cast between object and function pointers, which pedantic gcc flags.
template <class Fn>
[[nodiscard]] Fn as_function(void* p) {
    static_assert(sizeof(Fn) == sizeof(void*), "function and object pointers differ in size here");
    Fn fn = nullptr;
    std::memcpy(&fn, &p, sizeof fn);
    return fn;
}

// Fills `gl` from `loader`. Returns the first name the loader could not
// resolve, or nullptr when every entry point resolved.
[[nodiscard]] const char* load_gl(GlApi& gl, GlProcLoader loader) {
#define SPADE_GL_LOAD(type, name)                               \
    gl.name = as_function<type>(loader("gl" #name));            \
    if (gl.name == nullptr) return "gl" #name;
    SPADE_GL_FUNCTIONS(SPADE_GL_LOAD)
#undef SPADE_GL_LOAD
    return nullptr;
}

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

constexpr const char* kGlslVersion = "#version 430 core\n";

// One GLSL port of render/scene.hpp's sky_gradient_color() and
// horizon_blend(), shared by the background and the mesh programs.
constexpr const char* kCommonSrc = R"GLSL(
uniform vec3 uSkyZenith;
uniform vec3 uSkyHorizon;
uniform float uHorizonStrength;  // 0 outside shaded mode, as raster_cpu passes it
uniform float uHorizonOnset;

vec3 sky_gradient_color(vec3 ray) {
    float len = sqrt(dot(ray, ray));
    float elevation = len > 0.0 ? ray.y / len : 1.0;
    float horizon_fraction = clamp(1.0 - elevation, 0.0, 1.0);
    return uSkyZenith + (uSkyHorizon - uSkyZenith) * horizon_fraction;
}

// SR-17a. Returns ground_color exactly at strength 0, by its form.
vec3 horizon_blend(vec3 ground_color, vec3 sky_color, float view_distance) {
    if (uHorizonStrength <= 0.0 || uHorizonOnset <= 0.0) return ground_color;
    float r = view_distance / uHorizonOnset;
    float t = (r * r) / (1.0 + r * r);
    return ground_color + (sky_color - ground_color) * (t * uHorizonStrength);
}
)GLSL";

constexpr const char* kVertexSrc = R"GLSL(
layout(location = 0) in vec3 aPos;
layout(location = 1) in vec3 aNrm;

// THE MECHANISM. Per-instance transforms live here and the CPU never touches
// one at draw time; the shader indexes by gl_InstanceID plus the base this
// mesh's batch starts at -- v1's `instanceStartIndex`, same idea.
layout(std430, binding = 0) readonly buffer Transforms { mat4 uModel[]; };

uniform uint uInstanceBase;
uniform mat4 uViewProj;

out vec3 vNormal;
out vec3 vWorld;
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
    vWorld = world.xyz;
    vInstance = idx;
    gl_Position = uViewProj * world;
}
)GLSL";

constexpr const char* kFragmentSrc = R"GLSL(
in vec3 vNormal;
in vec3 vWorld;
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
layout(std430, binding = 4) readonly buffer Speeds { float uSpeed[]; };

uniform uint uSubmeshMaterial;
uniform uint uNoMaterial;
uniform vec3 uSunDir;
uniform vec3 uSunColor;
uniform vec3 uAmbient;
uniform float uSunIntensity;
uniform uint uMode;            // render::DrawMode: 0 shaded, 1 wireframe, 3 velocity
uniform float uVelocityScale;  // RenderOptions::velocity_scale_mps
uniform vec3 uCamPos;

out vec4 fragColor;

// raster_cpu's velocity_ramp(): blue at rest, red at the scale and above.
vec3 velocity_ramp(float speed, float scale) {
    if (!(scale > 0.0)) return vec3(1.0, 0.0, 0.0);
    float u = clamp(speed / scale, 0.0, 1.0);
    return vec3(u, 0.0, 1.0 - u);
}

void main() {
    // DrawItem::material_override wins when set, else the submesh's own
    // material -- the same precedence raster_cpu applies, stated once here.
    uint mi = uOverride[vInstance];
    if (mi == uNoMaterial) { mi = uSubmeshMaterial; }
    GpuMaterial m = uMaterials[mi];

    vec3 base = m.base_color.rgb;
    // Wireframe edges take the unlit material colour, as raster_cpu draws them.
    if (uMode == 1u) {
        fragColor = vec4(base, 1.0);
        return;
    }
    // Velocity swaps the colour and keeps the material's shading, as raster_cpu does.
    if (uMode == 3u) { base = velocity_ramp(uSpeed[vInstance], uVelocityScale); }

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
    // SR-17a, per pixel along this pixel's own ray, as raster_cpu does it.
    vec3 eye_to_surface = vWorld - uCamPos;
    lit = horizon_blend(lit, sky_gradient_color(eye_to_surface), length(eye_to_surface));
    fragColor = vec4(lit, 1.0);
}
)GLSL";

// One triangle that covers the viewport, from gl_VertexID; no vertex buffer.
constexpr const char* kBackgroundVertexSrc = R"GLSL(
void main() {
    vec2 p = vec2(float((gl_VertexID << 1) & 2), float(gl_VertexID & 2)) * 2.0 - 1.0;
    gl_Position = vec4(p, 0.0, 1.0);
}
)GLSL";

// The background: a port of raster_cpu's draw_sky_and_ground_background(),
// by way of shaders/kernels/raster_background.slang. Sky in every mode; the
// analytic ground, grid and horizon term in shaded mode only (SR-17, SR-22).
// fp32, so it agrees with the fp64 CPU to a band, not to the bit.
constexpr const char* kBackgroundFragmentSrc = R"GLSL(
// 64 bytes. The colour is shaded once per plane on the host.
struct GroundPlaneGpu {
    vec3 normal;
    float offset;    // dot(p, normal) <= offset is solid
    vec3 color;
    uint is_front;   // the camera is above the plane, decided on the host in double
    vec3 grid_u;     // ground_plane_basis()
    float pad0;
    vec3 grid_v;
    float pad1;
};
layout(std430, binding = 3) readonly buffer GroundPlanes { GroundPlaneGpu uPlanes[]; };

uniform vec3 uCamPos;
uniform vec3 uRight;
uniform vec3 uUp;
uniform vec3 uForward;
uniform float uInvTanHalfFov;
uniform float uAspect;
uniform vec2 uViewport;
uniform uint uPlaneCount;  // 0 outside shaded mode (SR-22)
uniform uint uGridEnabled;
uniform vec3 uGridColor;
uniform float uGridSpacing;
uniform float uGridHalfWidth;
uniform float uGridWidthGrowth;
uniform float uGridFadeDistance;

out vec4 fragColor;

float distance_to_nearest_line(float c, float spacing) {
    float k = floor(c / spacing + 0.5);
    return abs(c - k * spacing);
}

// render/scene.hpp's ground_grid_coverage(), with the basis precomputed.
float ground_grid_coverage(vec3 hit, vec3 u, vec3 v, float view_distance) {
    float hw = uGridHalfWidth * (1.0 + view_distance * uGridWidthGrowth);
    float d = min(distance_to_nearest_line(dot(hit, u), uGridSpacing),
                  distance_to_nearest_line(dot(hit, v), uGridSpacing));
    if (d >= hw) return 0.0;
    float r = view_distance / uGridFadeDistance;
    return (1.0 - d / hw) * (1.0 / (1.0 + r * r));
}

void main() {
    // gl_FragCoord is the pixel centre with y up. For the CPU's row y, top
    // first, its y_ndc = 1 - 2 (y + 0.5) / h is this same value.
    float x_ndc = 2.0 * gl_FragCoord.x / uViewport.x - 1.0;
    float y_ndc = 2.0 * gl_FragCoord.y / uViewport.y - 1.0;
    vec3 ray = uForward + uRight * (x_ndc * uAspect / uInvTanHalfFov) + uUp * (y_ndc / uInvTanHalfFov);
    vec3 sky = sky_gradient_color(ray);

    float best_t = -1.0;
    uint best = 0u;
    for (uint i = 0u; i < uPlaneCount; ++i) {
        if (uPlanes[i].is_front == 0u) continue;  // camera at or below this plane
        float denom = dot(uPlanes[i].normal, ray);
        if (denom >= 0.0) continue;               // moving away from its front
        float t = (uPlanes[i].offset - dot(uPlanes[i].normal, uCamPos)) / denom;
        if (t > 0.0 && (best_t < 0.0 || t < best_t)) {
            best_t = t;
            best = i;
        }
    }

    vec3 color = sky;
    if (best_t > 0.0) {
        vec3 hit = uCamPos + ray * best_t;
        float view_distance = sqrt(dot(ray, ray)) * best_t;
        color = uPlanes[best].color;
        if (uGridEnabled != 0u) {
            float cov = ground_grid_coverage(hit, uPlanes[best].grid_u, uPlanes[best].grid_v, view_distance);
            color = color + (uGridColor - color) * cov;
        }
        color = horizon_blend(color, sky, view_distance);
    }
    fragColor = vec4(color, 1.0);
}
)GLSL";

// Field layers (render/field_layer.hpp): one instance per cell, six vertices
// per cell, no vertex buffer. Corners use field_cell_corner()'s expression,
// so neighbouring cells meet without cracks, as on the CPU.
constexpr const char* kFieldVertexSrc = R"GLSL(
layout(std430, binding = 5) readonly buffer FieldColours { vec4 uCellColour[]; };

uniform mat4 uViewProj;
uniform vec3 uCenter;
uniform vec3 uRight;
uniform vec3 uUp;
uniform vec2 uSize;      // width and height, metres
uniform uvec2 uCells;    // cells_u, cells_v
uniform uint uCellBase;  // this layer's first entry in uCellColour

flat out vec3 vColour;

const uvec2 kCorner[6] = uvec2[6](uvec2(0u, 0u), uvec2(1u, 0u), uvec2(1u, 1u),
                                  uvec2(0u, 0u), uvec2(1u, 1u), uvec2(0u, 1u));

void main() {
    uint cell = uint(gl_InstanceID);
    uvec2 ij = uvec2(cell % uCells.x, cell / uCells.x) + kCorner[gl_VertexID];
    float u = float(ij.x) / float(uCells.x) - 0.5;
    float v = float(ij.y) / float(uCells.y) - 0.5;
    vec3 p = uCenter + uRight * (u * uSize.x) + uUp * (v * uSize.y);
    vColour = uCellColour[uCellBase + cell].rgb;
    gl_Position = uViewProj * vec4(p, 1.0);
}
)GLSL";

// Data, not appearance: the bin colour as it is, with no light or atmosphere.
constexpr const char* kFieldFragmentSrc = R"GLSL(
flat in vec3 vColour;
out vec4 fragColor;

void main() {
    fragColor = vec4(vColour, 1.0);
}
)GLSL";

// 32 bytes, explicitly padded. See the shader comment above.
struct GpuMaterial {
    glm::vec4 base_color{0.72f, 0.72f, 0.74f, 1.0f};
    uint32_t shading = 0;
    uint32_t pad0 = 0, pad1 = 0, pad2 = 0;
};
static_assert(sizeof(GpuMaterial) == 32, "std430 layout must match the shader's GpuMaterial");

// 64 bytes: std430 packs each vec3 with the scalar after it into 16.
struct GpuGroundPlane {
    glm::vec3 normal{0.0f};
    float offset = 0.0f;
    glm::vec3 color{0.0f};
    uint32_t is_front = 0;
    glm::vec3 grid_u{0.0f};
    float pad0 = 0.0f;
    glm::vec3 grid_v{0.0f};
    float pad1 = 0.0f;
};
static_assert(sizeof(GpuGroundPlane) == 64, "std430 layout must match the shader's GroundPlaneGpu");

struct GpuMesh {
    GLuint vao = 0, vbo_pos = 0, vbo_nrm = 0, ebo = 0;
    uint32_t index_count = 0;
    std::vector<uint32_t> submesh_first_index;
    std::vector<uint32_t> submesh_index_count;
    std::vector<uint32_t> submesh_material;
};

// `parts` are concatenated in order, so one GLSL helper serves two programs.
[[nodiscard]] Result<GLuint> compile(const GlApi& gl, GLenum stage, std::initializer_list<const char*> parts, const char* what) {
    const GLuint sh = gl.CreateShader(stage);
    gl.ShaderSource(sh, static_cast<GLsizei>(parts.size()), parts.begin(), nullptr);
    gl.CompileShader(sh);
    GLint ok = GL_FALSE;
    gl.GetShaderiv(sh, GL_COMPILE_STATUS, &ok);
    if (ok != GL_TRUE) {
        GLint len = 0;
        gl.GetShaderiv(sh, GL_INFO_LOG_LENGTH, &len);
        std::string log(static_cast<size_t>(len > 0 ? len : 1), '\0');
        gl.GetShaderInfoLog(sh, len, nullptr, log.data());
        gl.DeleteShader(sh);
        // The driver's own log, forwarded verbatim. A shader failure reported
        // as "compile failed" sends the reader to guess; the log names a line.
        // `parts` are separate source strings, so a line is counted within
        // its own part on drivers that prefix the string index.
        const char* kind = stage == GL_VERTEX_SHADER ? " vertex" : " fragment";
        return std::unexpected(Error{Code::internal, std::string(what) + kind + " shader: " + log});
    }
    return sh;
}

[[nodiscard]] Result<GLuint> link(const GlApi& gl, std::initializer_list<const char*> vertex,
                                  std::initializer_list<const char*> fragment, const char* what) {
    const Result<GLuint> vs = compile(gl, GL_VERTEX_SHADER, vertex, what);
    if (!vs) return std::unexpected(vs.error());
    const Result<GLuint> fs = compile(gl, GL_FRAGMENT_SHADER, fragment, what);
    if (!fs) {
        gl.DeleteShader(*vs);
        return std::unexpected(fs.error());
    }
    const GLuint program = gl.CreateProgram();
    gl.AttachShader(program, *vs);
    gl.AttachShader(program, *fs);
    gl.LinkProgram(program);
    gl.DeleteShader(*vs);
    gl.DeleteShader(*fs);

    GLint linked = GL_FALSE;
    gl.GetProgramiv(program, GL_LINK_STATUS, &linked);
    if (linked != GL_TRUE) {
        GLint len = 0;
        gl.GetProgramiv(program, GL_INFO_LOG_LENGTH, &len);
        std::string log(static_cast<size_t>(len > 0 ? len : 1), '\0');
        gl.GetProgramInfoLog(program, len, nullptr, log.data());
        gl.DeleteProgram(program);
        return std::unexpected(Error{Code::internal, std::string("GlRenderer::create: link ") + what + ": " + log});
    }
    return program;
}

// Grows `buffer` only when `data` outgrows it, else writes in place (v1's
// contract, and the reason a steady-state frame does no reallocation).
template <class T>
void upload_in_place(const GlApi& gl, GLuint buffer, size_t& capacity, const std::vector<T>& data) {
    gl.BindBuffer(GL_SHADER_STORAGE_BUFFER, buffer);
    if (data.size() > capacity) {
        gl.BufferData(GL_SHADER_STORAGE_BUFFER, static_cast<GLsizeiptr>(data.size() * sizeof(T)), data.data(),
                     GL_DYNAMIC_DRAW);
        capacity = data.size();
    } else if (!data.empty()) {
        gl.BufferSubData(GL_SHADER_STORAGE_BUFFER, 0, static_cast<GLsizeiptr>(data.size() * sizeof(T)), data.data());
    }
}

}  // namespace

// ---------------------------------------------------------------------------

struct GlRenderer::Impl {
    // This renderer's own GL entry points; see GlApi.
    GlApi gl;
    GLuint program = 0;
    std::vector<GpuMesh> meshes;

    GLuint ssbo_transforms = 0;
    GLuint ssbo_overrides = 0;
    GLuint ssbo_materials = 0;
    GLuint ssbo_speeds = 0;
    GLuint ssbo_planes = 0;
    size_t transforms_capacity = 0;  // in instances, so growth is in place after the first
    size_t overrides_capacity = 0;
    size_t materials_capacity = 0;
    size_t speeds_capacity = 0;
    size_t planes_capacity = 0;

    // The background program draws one triangle from gl_VertexID. A core
    // profile still needs a vertex array bound, so it gets an empty one.
    GLuint background_program = 0;
    GLuint empty_vao = 0;

    // Field layers: their own program, and one colour per cell, all layers
    // end to end in one buffer.
    GLuint field_program = 0;
    GLuint ssbo_field_colours = 0;
    size_t field_colours_capacity = 0;

    // Scratch, reused every frame so a frame allocates nothing steady-state.
    std::vector<glm::mat4> instance_transforms;
    std::vector<uint32_t> instance_overrides;
    std::vector<float> instance_speeds;
    std::vector<uint32_t> batch_base;   // per mesh: where its instances start
    std::vector<uint32_t> batch_count;  // per mesh: how many
    std::vector<GpuMaterial> gpu_materials;
    std::vector<GpuGroundPlane> gpu_planes;
    std::vector<glm::vec4> field_colours;

    uint32_t last_draw_calls = 0;
    uint32_t last_instances = 0;

    std::string renderer_name;
    std::string version_string;

    GLint u_instance_base = -1, u_view_proj = -1, u_submesh_material = -1, u_no_material = -1;
    GLint u_sun_dir = -1, u_sun_color = -1, u_ambient = -1, u_sun_intensity = -1;
    GLint u_mode = -1, u_velocity_scale = -1, u_cam_pos = -1;
    GLint u_sky_zenith = -1, u_sky_horizon = -1, u_horizon_strength = -1, u_horizon_onset = -1;

    struct BackgroundUniforms {
        GLint cam_pos = -1, right = -1, up = -1, forward = -1, inv_tan_half_fov = -1, aspect = -1, viewport = -1;
        GLint plane_count = -1, grid_enabled = -1, grid_color = -1, grid_spacing = -1, grid_half_width = -1;
        GLint grid_width_growth = -1, grid_fade_distance = -1;
        GLint sky_zenith = -1, sky_horizon = -1, horizon_strength = -1, horizon_onset = -1;
    } bg;

    struct FieldUniforms {
        GLint view_proj = -1, center = -1, right = -1, up = -1, size = -1, cells = -1, cell_base = -1;
    } field;

    ~Impl() {
        for (GpuMesh& m : meshes) {
            if (m.ebo != 0) gl.DeleteBuffers(1, &m.ebo);
            if (m.vbo_nrm != 0) gl.DeleteBuffers(1, &m.vbo_nrm);
            if (m.vbo_pos != 0) gl.DeleteBuffers(1, &m.vbo_pos);
            if (m.vao != 0) gl.DeleteVertexArrays(1, &m.vao);
        }
        if (ssbo_field_colours != 0) gl.DeleteBuffers(1, &ssbo_field_colours);
        if (ssbo_planes != 0) gl.DeleteBuffers(1, &ssbo_planes);
        if (ssbo_speeds != 0) gl.DeleteBuffers(1, &ssbo_speeds);
        if (ssbo_materials != 0) gl.DeleteBuffers(1, &ssbo_materials);
        if (ssbo_overrides != 0) gl.DeleteBuffers(1, &ssbo_overrides);
        if (ssbo_transforms != 0) gl.DeleteBuffers(1, &ssbo_transforms);
        if (empty_vao != 0) gl.DeleteVertexArrays(1, &empty_vao);
        if (field_program != 0) gl.DeleteProgram(field_program);
        if (background_program != 0) gl.DeleteProgram(background_program);
        if (program != 0) gl.DeleteProgram(program);
    }
};

GlRenderer::GlRenderer(std::unique_ptr<Impl> impl) : impl_(std::move(impl)) {}
GlRenderer::~GlRenderer() = default;

Result<std::unique_ptr<GlRenderer>> GlRenderer::create(GlProcLoader loader) {
    if (loader == nullptr) {
        return std::unexpected(Error{Code::invalid_argument, "GlRenderer::create: null proc loader"});
    }
    auto impl = std::make_unique<Impl>();
    // Refused before any GL call: every pointer is checked first, and a
    // null one would crash on use.
    if (const char* missing = load_gl(impl->gl, loader); missing != nullptr) {
        return std::unexpected(Error{Code::unavailable,
                                     std::string("GlRenderer::create: the loader found no ") + missing +
                                         " -- is a GL 4.3 context current on this thread?"});
    }
    const GlApi& gl = impl->gl;
    const auto* rend = reinterpret_cast<const char*>(gl.GetString(GL_RENDERER));
    const auto* vers = reinterpret_cast<const char*>(gl.GetString(GL_VERSION));
    impl->renderer_name = rend != nullptr ? rend : "(unknown)";
    impl->version_string = vers != nullptr ? vers : "(unknown)";

    // ⚠ REFUSED, NOT DEGRADED. The entire mechanism rests on SSBOs; without
    // them this class cannot do the thing it exists for, and a renderer that
    // quietly substitutes something slower is indistinguishable from the defect
    // it was written to fix. Code::unavailable specifically, so a caller can
    // fall back to raster_cpu by switching on it -- which is exactly what the
    // user's "GPU default, CPU fallback" ruling asks of a caller.
    GLint major = 0, minor = 0;
    gl.GetIntegerv(GL_MAJOR_VERSION, &major);
    gl.GetIntegerv(GL_MINOR_VERSION, &minor);
    if (major < 4 || (major == 4 && minor < 3)) {
        return std::unexpected(Error{
            Code::unavailable, "GlRenderer::create: needs OpenGL 4.3 for shader storage buffers; "
                               "this context reports " +
                                   impl->version_string + " on " + impl->renderer_name});
    }

    const Result<GLuint> mesh_program =
        link(gl, {kGlslVersion, kCommonSrc, kVertexSrc}, {kGlslVersion, kCommonSrc, kFragmentSrc}, "mesh");
    if (!mesh_program) return std::unexpected(mesh_program.error());
    impl->program = *mesh_program;
    const Result<GLuint> background_program = link(gl, {kGlslVersion, kBackgroundVertexSrc},
                                                    {kGlslVersion, kCommonSrc, kBackgroundFragmentSrc}, "background");
    if (!background_program) return std::unexpected(background_program.error());
    impl->background_program = *background_program;
    const Result<GLuint> field_program =
        link(gl, {kGlslVersion, kFieldVertexSrc}, {kGlslVersion, kFieldFragmentSrc}, "field");
    if (!field_program) return std::unexpected(field_program.error());
    impl->field_program = *field_program;

    const GLuint p = impl->program;
    impl->u_instance_base = gl.GetUniformLocation(p, "uInstanceBase");
    impl->u_view_proj = gl.GetUniformLocation(p, "uViewProj");
    impl->u_submesh_material = gl.GetUniformLocation(p, "uSubmeshMaterial");
    impl->u_no_material = gl.GetUniformLocation(p, "uNoMaterial");
    impl->u_sun_dir = gl.GetUniformLocation(p, "uSunDir");
    impl->u_sun_color = gl.GetUniformLocation(p, "uSunColor");
    impl->u_ambient = gl.GetUniformLocation(p, "uAmbient");
    impl->u_sun_intensity = gl.GetUniformLocation(p, "uSunIntensity");
    impl->u_mode = gl.GetUniformLocation(p, "uMode");
    impl->u_velocity_scale = gl.GetUniformLocation(p, "uVelocityScale");
    impl->u_cam_pos = gl.GetUniformLocation(p, "uCamPos");
    impl->u_sky_zenith = gl.GetUniformLocation(p, "uSkyZenith");
    impl->u_sky_horizon = gl.GetUniformLocation(p, "uSkyHorizon");
    impl->u_horizon_strength = gl.GetUniformLocation(p, "uHorizonStrength");
    impl->u_horizon_onset = gl.GetUniformLocation(p, "uHorizonOnset");

    const GLuint b = impl->background_program;
    Impl::BackgroundUniforms& bg = impl->bg;
    bg.cam_pos = gl.GetUniformLocation(b, "uCamPos");
    bg.right = gl.GetUniformLocation(b, "uRight");
    bg.up = gl.GetUniformLocation(b, "uUp");
    bg.forward = gl.GetUniformLocation(b, "uForward");
    bg.inv_tan_half_fov = gl.GetUniformLocation(b, "uInvTanHalfFov");
    bg.aspect = gl.GetUniformLocation(b, "uAspect");
    bg.viewport = gl.GetUniformLocation(b, "uViewport");
    bg.plane_count = gl.GetUniformLocation(b, "uPlaneCount");
    bg.grid_enabled = gl.GetUniformLocation(b, "uGridEnabled");
    bg.grid_color = gl.GetUniformLocation(b, "uGridColor");
    bg.grid_spacing = gl.GetUniformLocation(b, "uGridSpacing");
    bg.grid_half_width = gl.GetUniformLocation(b, "uGridHalfWidth");
    bg.grid_width_growth = gl.GetUniformLocation(b, "uGridWidthGrowth");
    bg.grid_fade_distance = gl.GetUniformLocation(b, "uGridFadeDistance");
    bg.sky_zenith = gl.GetUniformLocation(b, "uSkyZenith");
    bg.sky_horizon = gl.GetUniformLocation(b, "uSkyHorizon");
    bg.horizon_strength = gl.GetUniformLocation(b, "uHorizonStrength");
    bg.horizon_onset = gl.GetUniformLocation(b, "uHorizonOnset");

    gl.GenBuffers(1, &impl->ssbo_transforms);
    gl.GenBuffers(1, &impl->ssbo_overrides);
    gl.GenBuffers(1, &impl->ssbo_materials);
    gl.GenBuffers(1, &impl->ssbo_speeds);
    gl.GenBuffers(1, &impl->ssbo_planes);
    gl.GenVertexArrays(1, &impl->empty_vao);

    const GLuint f = impl->field_program;
    Impl::FieldUniforms& fu = impl->field;
    fu.view_proj = gl.GetUniformLocation(f, "uViewProj");
    fu.center = gl.GetUniformLocation(f, "uCenter");
    fu.right = gl.GetUniformLocation(f, "uRight");
    fu.up = gl.GetUniformLocation(f, "uUp");
    fu.size = gl.GetUniformLocation(f, "uSize");
    fu.cells = gl.GetUniformLocation(f, "uCells");
    fu.cell_base = gl.GetUniformLocation(f, "uCellBase");
    gl.GenBuffers(1, &impl->ssbo_field_colours);

    return std::unique_ptr<GlRenderer>(new GlRenderer(std::move(impl)));
}

Result<void> GlRenderer::upload_scene(const render::RenderScene& scene) {
    Impl& s = *impl_;
    const GlApi& gl = s.gl;

    // Geometry uploads ONCE per mesh and lives until the mesh set changes --
    // v1's `if (meshComponent.VAO == 0)` branch, and the reason a frame costs
    // nothing per vertex.
    for (GpuMesh& m : s.meshes) {
        if (m.ebo != 0) gl.DeleteBuffers(1, &m.ebo);
        if (m.vbo_nrm != 0) gl.DeleteBuffers(1, &m.vbo_nrm);
        if (m.vbo_pos != 0) gl.DeleteBuffers(1, &m.vbo_pos);
        if (m.vao != 0) gl.DeleteVertexArrays(1, &m.vao);
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

        gl.GenVertexArrays(1, &dst.vao);
        gl.BindVertexArray(dst.vao);

        gl.GenBuffers(1, &dst.vbo_pos);
        gl.BindBuffer(GL_ARRAY_BUFFER, dst.vbo_pos);
        gl.BufferData(GL_ARRAY_BUFFER,
                     static_cast<GLsizeiptr>(src.positions.size() * sizeof(glm::vec3)),
                     src.positions.data(), GL_STATIC_DRAW);
        gl.EnableVertexAttribArray(0);
        gl.VertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(glm::vec3), nullptr);

        gl.GenBuffers(1, &dst.vbo_nrm);
        gl.BindBuffer(GL_ARRAY_BUFFER, dst.vbo_nrm);
        gl.BufferData(GL_ARRAY_BUFFER, static_cast<GLsizeiptr>(src.normals.size() * sizeof(glm::vec3)),
                     src.normals.data(), GL_STATIC_DRAW);
        gl.EnableVertexAttribArray(1);
        gl.VertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(glm::vec3), nullptr);

        gl.GenBuffers(1, &dst.ebo);
        gl.BindBuffer(GL_ELEMENT_ARRAY_BUFFER, dst.ebo);
        gl.BufferData(GL_ELEMENT_ARRAY_BUFFER,
                     static_cast<GLsizeiptr>(src.indices.size() * sizeof(uint32_t)),
                     src.indices.data(), GL_STATIC_DRAW);

        gl.BindVertexArray(0);

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
    gl.BindBuffer(GL_SHADER_STORAGE_BUFFER, s.ssbo_materials);
    gl.BufferData(GL_SHADER_STORAGE_BUFFER,
                 static_cast<GLsizeiptr>(s.gpu_materials.size() * sizeof(GpuMaterial)),
                 s.gpu_materials.data(), GL_STATIC_DRAW);
    s.materials_capacity = s.gpu_materials.size();
    gl.BindBuffer(GL_SHADER_STORAGE_BUFFER, 0);

    return {};
}

Result<void> GlRenderer::draw(const render::RenderScene& scene, const render::Camera& camera,
                              const render::RenderOptions& options, uint32_t width,
                              uint32_t height) {
    Impl& s = *impl_;
    const GlApi& gl = s.gl;

    // L6: GL has no ray-marcher, and drawing a raster frame instead would be
    // a different technique under the same name.
    if (options.mode == render::DrawMode::raymarch) {
        return std::unexpected(Error{Code::unavailable,
                                     "GlRenderer::draw: raymarch is a CPU technique (render/raymarch.hpp); "
                                     "draw it with render::render()"});
    }
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
    // A malformed field layer is refused before any state changes (L6).
    for (const render::FieldLayer& layer : scene.field_layers) {
        if (Result<void> valid = render::validate_field_layer(layer); !valid) {
            return valid;
        }
    }
    const bool shaded = options.mode == render::DrawMode::shaded;
    const bool wireframe = options.mode == render::DrawMode::wireframe;

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
    s.instance_speeds.assign(total_instances, 0.0f);
    std::vector<uint32_t> cursor(s.batch_base.begin(), s.batch_base.end());
    const auto place = [&](const render::DrawItem& d) {
        if (d.mesh_index >= mesh_count) return;
        const uint32_t at = cursor[d.mesh_index]++;
        s.instance_transforms[at] = d.local_to_world;
        s.instance_overrides[at] = d.material_override;
        s.instance_speeds[at] = d.speed_mps;
    };
    for (const render::DrawItem& d : scene.statics) place(d);
    for (const render::DrawItem& d : scene.dynamics) place(d);

    // --- ground planes: per frame, because is_front depends on the camera --
    // raster_cpu's own precompute: shaded mode only (SR-22), each plane's
    // colour shaded once by shade_vertex_color(), and "is the camera above
    // it" decided in double.
    s.gpu_planes.clear();
    if (shaded && !scene.materials.empty()) {
        for (const render::GroundPlane& gp : scene.ground_planes) {
            GpuGroundPlane g;
            g.normal = gp.normal;
            g.offset = gp.offset;
            const uint32_t mi = gp.material < scene.materials.size() ? gp.material : 0u;
            g.color = render::shade_vertex_color(scene.materials[mi], scene.lighting, gp.normal).combined;
            const double n_dot_cam = static_cast<double>(gp.normal.x) * camera.position.x +
                                     static_cast<double>(gp.normal.y) * camera.position.y +
                                     static_cast<double>(gp.normal.z) * camera.position.z;
            g.is_front = n_dot_cam > static_cast<double>(gp.offset) ? 1u : 0u;
            render::ground_plane_basis(gp.normal, g.grid_u, g.grid_v);
            s.gpu_planes.push_back(g);
        }
    }
    const auto plane_count = static_cast<GLuint>(s.gpu_planes.size());
    if (s.gpu_planes.empty()) {
        s.gpu_planes.emplace_back();  // a bound buffer always has storage; uPlaneCount says 0
    }

    // --- buffers: allocate once, then write in place -----------------------
    upload_in_place(gl, s.ssbo_transforms, s.transforms_capacity, s.instance_transforms);
    upload_in_place(gl, s.ssbo_overrides, s.overrides_capacity, s.instance_overrides);
    upload_in_place(gl, s.ssbo_speeds, s.speeds_capacity, s.instance_speeds);
    upload_in_place(gl, s.ssbo_planes, s.planes_capacity, s.gpu_planes);
    gl.BindBuffer(GL_SHADER_STORAGE_BUFFER, 0);

    gl.BindBufferBase(GL_SHADER_STORAGE_BUFFER, 0, s.ssbo_transforms);
    gl.BindBufferBase(GL_SHADER_STORAGE_BUFFER, 1, s.ssbo_overrides);
    gl.BindBufferBase(GL_SHADER_STORAGE_BUFFER, 2, s.ssbo_materials);
    gl.BindBufferBase(GL_SHADER_STORAGE_BUFFER, 3, s.ssbo_planes);
    gl.BindBufferBase(GL_SHADER_STORAGE_BUFFER, 4, s.ssbo_speeds);

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
    // raster_cpu's horizon term is shaded-mode only, on the ground and on meshes.
    const float horizon_strength = shaded ? options.horizon_blend_strength : 0.0f;
    const render::Lighting& light = scene.lighting;

    gl.Viewport(0, 0, static_cast<GLsizei>(width), static_cast<GLsizei>(height));
    gl.DepthMask(GL_TRUE);
    gl.Clear(GL_DEPTH_BUFFER_BIT);
    gl.PolygonMode(GL_FRONT_AND_BACK, GL_FILL);

    // --- background --------------------------------------------------------
    // Every pixel, before the meshes and with no depth: raster_cpu's
    // background writes no depth either, so every mesh draws over it.
    {
        const Impl::BackgroundUniforms& bg = s.bg;
        // raster_cpu's per-frame ray basis and its fp32 tangent (tan32).
        const glm::vec3 right = camera.orientation * glm::vec3(1.0f, 0.0f, 0.0f);
        const glm::vec3 up = camera.orientation * glm::vec3(0.0f, 1.0f, 0.0f);
        const glm::vec3 forward = camera.orientation * glm::vec3(0.0f, 0.0f, -1.0f);
        const auto inv_tan_half_fov =
            static_cast<float>(1.0 / render::tan32(static_cast<double>(camera.fov_y_radians) * 0.5));
        const render::GroundGridParams& grid = options.ground_grid_params;

        gl.Disable(GL_DEPTH_TEST);
        gl.DepthMask(GL_FALSE);
        gl.Disable(GL_CULL_FACE);
        gl.UseProgram(s.background_program);
        gl.Uniform3fv(bg.cam_pos, 1, glm::value_ptr(camera.position));
        gl.Uniform3fv(bg.right, 1, glm::value_ptr(right));
        gl.Uniform3fv(bg.up, 1, glm::value_ptr(up));
        gl.Uniform3fv(bg.forward, 1, glm::value_ptr(forward));
        gl.Uniform1f(bg.inv_tan_half_fov, inv_tan_half_fov);
        gl.Uniform1f(bg.aspect, aspect);
        gl.Uniform2f(bg.viewport, static_cast<float>(width), static_cast<float>(height));
        gl.Uniform1ui(bg.plane_count, plane_count);
        gl.Uniform1ui(bg.grid_enabled, (shaded && options.ground_grid) ? 1u : 0u);
        gl.Uniform3fv(bg.grid_color, 1, glm::value_ptr(grid.color));
        gl.Uniform1f(bg.grid_spacing, grid.spacing);
        gl.Uniform1f(bg.grid_half_width, grid.line_half_width);
        gl.Uniform1f(bg.grid_width_growth, grid.width_growth);
        gl.Uniform1f(bg.grid_fade_distance, grid.fade_distance);
        gl.Uniform3fv(bg.sky_zenith, 1, glm::value_ptr(light.sky_zenith));
        gl.Uniform3fv(bg.sky_horizon, 1, glm::value_ptr(light.sky_horizon));
        gl.Uniform1f(bg.horizon_strength, horizon_strength);
        gl.Uniform1f(bg.horizon_onset, options.horizon_blend_onset);
        gl.BindVertexArray(s.empty_vao);
        gl.DrawArrays(GL_TRIANGLES, 0, 3);
        gl.DepthMask(GL_TRUE);
    }

    // --- meshes ------------------------------------------------------------
    gl.Enable(GL_DEPTH_TEST);
    gl.DepthFunc(GL_LESS);
    if (wireframe) {
        // SR-13: wireframe culls nothing, so both windings draw the same edges.
        gl.Disable(GL_CULL_FACE);
        gl.PolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    } else {
        // SR-13: shaded mode culls back faces, as raster_cpu does (velocity
        // too, through the same CPU function). Front is CCW seen from the
        // camera, the CPU's edge-function sign, so an outward-wound mesh
        // loses nothing and an inward-wound one shows its winding bug here.
        gl.Enable(GL_CULL_FACE);
        gl.CullFace(GL_BACK);
        gl.FrontFace(GL_CCW);
    }

    gl.UseProgram(s.program);
    gl.UniformMatrix4fv(s.u_view_proj, 1, GL_FALSE, glm::value_ptr(view_proj));
    gl.Uniform1ui(s.u_no_material, render::kNoMaterial);
    gl.Uniform3fv(s.u_sun_dir, 1, glm::value_ptr(light.sun_direction));
    gl.Uniform3fv(s.u_sun_color, 1, glm::value_ptr(light.sun_color));
    gl.Uniform3fv(s.u_ambient, 1, glm::value_ptr(light.ambient_color));
    gl.Uniform1f(s.u_sun_intensity, light.sun_intensity);
    gl.Uniform1ui(s.u_mode, static_cast<GLuint>(options.mode));
    gl.Uniform1f(s.u_velocity_scale, options.velocity_scale_mps);
    gl.Uniform3fv(s.u_cam_pos, 1, glm::value_ptr(camera.position));
    gl.Uniform3fv(s.u_sky_zenith, 1, glm::value_ptr(light.sky_zenith));
    gl.Uniform3fv(s.u_sky_horizon, 1, glm::value_ptr(light.sky_horizon));
    gl.Uniform1f(s.u_horizon_strength, horizon_strength);
    gl.Uniform1f(s.u_horizon_onset, options.horizon_blend_onset);

    uint32_t draw_calls = 0;
    for (size_t i = 0; i < mesh_count; ++i) {
        const uint32_t count = s.batch_count[i];
        if (count == 0u) continue;  // a mesh nothing instances costs nothing

        const GpuMesh& m = s.meshes[i];
        gl.BindVertexArray(m.vao);
        gl.Uniform1ui(s.u_instance_base, s.batch_base[i]);

        // ONE DRAW PER SUBMESH, and submeshes exist because materials differ
        // within a mesh. Still O(meshes x submeshes) and never O(objects),
        // which is the property the whole module is for.
        for (size_t sm = 0; sm < m.submesh_first_index.size(); ++sm) {
            const uint32_t first = m.submesh_first_index[sm];
            const uint32_t n = m.submesh_index_count[sm];
            if (n == 0u) continue;
            gl.Uniform1ui(s.u_submesh_material, m.submesh_material[sm]);
            gl.DrawElementsInstanced(
                GL_TRIANGLES, static_cast<GLsizei>(n), GL_UNSIGNED_INT,
                reinterpret_cast<const void*>(static_cast<uintptr_t>(first) * sizeof(uint32_t)),
                static_cast<GLsizei>(count));
            ++draw_calls;
        }
    }
    gl.PolygonMode(GL_FRONT_AND_BACK, GL_FILL);

    // --- field layers ------------------------------------------------------
    // Filled in every raster mode, because a layer is data. Depth-tested
    // against the meshes, culling nothing, so both sides draw.
    if (!scene.field_layers.empty()) {
        s.field_colours.clear();
        for (const render::FieldLayer& layer : scene.field_layers) {
            const float top = render::resolved_range_max(layer);
            for (const float value : layer.values) {
                s.field_colours.emplace_back(render::field_cell_colour(layer.colour_map, value, top), 1.0f);
            }
        }
        upload_in_place(gl, s.ssbo_field_colours, s.field_colours_capacity, s.field_colours);
        gl.BindBuffer(GL_SHADER_STORAGE_BUFFER, 0);
        gl.BindBufferBase(GL_SHADER_STORAGE_BUFFER, 5, s.ssbo_field_colours);

        gl.Disable(GL_CULL_FACE);
        gl.UseProgram(s.field_program);
        gl.UniformMatrix4fv(s.field.view_proj, 1, GL_FALSE, glm::value_ptr(view_proj));
        gl.BindVertexArray(s.empty_vao);
        GLuint base = 0;
        for (const render::FieldLayer& layer : scene.field_layers) {
            gl.Uniform3fv(s.field.center, 1, glm::value_ptr(layer.center));
            gl.Uniform3fv(s.field.right, 1, glm::value_ptr(layer.right));
            gl.Uniform3fv(s.field.up, 1, glm::value_ptr(layer.up));
            gl.Uniform2f(s.field.size, layer.width, layer.height);
            gl.Uniform2ui(s.field.cells, layer.cells_u, layer.cells_v);
            gl.Uniform1ui(s.field.cell_base, base);
            const GLuint cells = layer.cells_u * layer.cells_v;
            gl.DrawArraysInstanced(GL_TRIANGLES, 0, 6, static_cast<GLsizei>(cells));
            base += cells;
        }
    }
    gl.BindVertexArray(0);
    gl.UseProgram(0);

    s.last_draw_calls = draw_calls;
    s.last_instances = total_instances;
    return {};
}

std::vector<std::string_view> GlRenderer::unhonoured(const render::RenderOptions& options) {
    // draw() refuses raymarch; no other option applies to a raymarch frame.
    if (options.mode == render::DrawMode::raymarch) {
        return {"mode"};
    }
    std::vector<std::string_view> names;
    // raster_cpu draws shadows in shaded mode only, so only there is one missing.
    if (options.shadows && options.mode == render::DrawMode::shaded) {
        names.emplace_back("shadows");
    }
    if (options.overlays) {
        names.emplace_back("overlays");
    }
    return names;
}

uint32_t GlRenderer::last_draw_calls() const noexcept { return impl_->last_draw_calls; }
uint32_t GlRenderer::last_instances() const noexcept { return impl_->last_instances; }
const std::string& GlRenderer::renderer_name() const noexcept { return impl_->renderer_name; }
const std::string& GlRenderer::version_string() const noexcept { return impl_->version_string; }

}  // namespace spade::render_gl
