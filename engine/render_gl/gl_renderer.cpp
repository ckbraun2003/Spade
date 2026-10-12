// GlRenderer -- see the header for why this exists and what it deliberately is
// not.

#include "render_gl/gl_renderer.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstring>
#include <initializer_list>
#include <limits>
#include <string>
#include <string_view>
#include <unordered_map>
#include <vector>

#include <glad/glad.h>

// ⚠ THE VENDORED GLAD IS GENERATED UP TO GL 4.1, AND SSBOs ARE 4.3. Every
// FUNCTION this file calls is 4.1 or earlier and is present, but one (below);
// for SSBOs the only thing missing is the enum. Defined here behind an
// #ifndef, with the value from the GL registry, which has never changed.
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
#ifndef GL_MAX_SHADER_STORAGE_BLOCK_SIZE
#define GL_MAX_SHADER_STORAGE_BLOCK_SIZE 0x90DE
#endif

// One 4.3 FUNCTION too, since GL shadows: glCopyImageSubData, which seeds a
// frame's shadow map from the static one. Its type, from the registry's
// signature, under a name of our own so it cannot clash with a later glad.
// create() refuses a context below 4.3, so the entry point always exists.
typedef void(APIENTRYP SpadePfnGlCopyImageSubData)(GLuint src, GLenum src_target, GLint src_level, GLint src_x,
                                                   GLint src_y, GLint src_z, GLuint dst, GLenum dst_target,
                                                   GLint dst_level, GLint dst_x, GLint dst_y, GLint dst_z,
                                                   GLsizei width, GLsizei height, GLsizei depth);

// And three for the flow march's empty-space pass (airflow plan §7.2), a
// compute shader writing an image: 4.2's glBindImageTexture and
// glMemoryBarrier, 4.3's glDispatchCompute.
typedef void(APIENTRYP SpadePfnGlBindImageTexture)(GLuint unit, GLuint texture, GLint level, GLboolean layered,
                                                   GLint layer, GLenum access, GLenum format);
typedef void(APIENTRYP SpadePfnGlDispatchCompute)(GLuint groups_x, GLuint groups_y, GLuint groups_z);
typedef void(APIENTRYP SpadePfnGlMemoryBarrier)(GLbitfield barriers);
#ifndef GL_COMPUTE_SHADER
#define GL_COMPUTE_SHADER 0x91B9
#endif
#ifndef GL_TEXTURE_FETCH_BARRIER_BIT
#define GL_TEXTURE_FETCH_BARRIER_BIT 0x00000008
#endif

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtc/type_ptr.hpp>

#include "core/error.hpp"
#include "render/raster_cpu.hpp"  // overlay_geometry(), kOverlayDepthBias
#include "render/raymarch.hpp"    // kRaymarchMaxSteps, kRaymarchSurfaceEpsilon
#include "render/shadow.hpp"      // kNoOccluder, kShadowDepthBias
#include "world/sdf.hpp"          // SdfNode, SdfTransform, kMaxSdfDepth

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
    X(PFNGLACTIVETEXTUREPROC, ActiveTexture) \
    X(PFNGLATTACHSHADERPROC, AttachShader) \
    X(PFNGLBINDBUFFERPROC, BindBuffer) \
    X(PFNGLBINDBUFFERBASEPROC, BindBufferBase) \
    X(PFNGLBINDFRAMEBUFFERPROC, BindFramebuffer) \
    X(SpadePfnGlBindImageTexture, BindImageTexture) \
    X(PFNGLBINDTEXTUREPROC, BindTexture) \
    X(PFNGLBINDVERTEXARRAYPROC, BindVertexArray) \
    X(PFNGLBLENDEQUATIONPROC, BlendEquation) \
    X(PFNGLBLENDFUNCPROC, BlendFunc) \
    X(PFNGLBUFFERDATAPROC, BufferData) \
    X(PFNGLBUFFERSUBDATAPROC, BufferSubData) \
    X(PFNGLCHECKFRAMEBUFFERSTATUSPROC, CheckFramebufferStatus) \
    X(PFNGLCLEARPROC, Clear) \
    X(PFNGLCLEARBUFFERFVPROC, ClearBufferfv) \
    X(PFNGLCOMPILESHADERPROC, CompileShader) \
    X(SpadePfnGlCopyImageSubData, CopyImageSubData) \
    X(PFNGLCREATEPROGRAMPROC, CreateProgram) \
    X(PFNGLCREATESHADERPROC, CreateShader) \
    X(PFNGLCULLFACEPROC, CullFace) \
    X(PFNGLDELETEBUFFERSPROC, DeleteBuffers) \
    X(PFNGLDELETEFRAMEBUFFERSPROC, DeleteFramebuffers) \
    X(PFNGLDELETEPROGRAMPROC, DeleteProgram) \
    X(PFNGLDELETESHADERPROC, DeleteShader) \
    X(PFNGLDELETETEXTURESPROC, DeleteTextures) \
    X(PFNGLDELETEVERTEXARRAYSPROC, DeleteVertexArrays) \
    X(PFNGLDEPTHFUNCPROC, DepthFunc) \
    X(PFNGLDEPTHMASKPROC, DepthMask) \
    X(PFNGLDISABLEPROC, Disable) \
    X(SpadePfnGlDispatchCompute, DispatchCompute) \
    X(PFNGLDRAWARRAYSPROC, DrawArrays) \
    X(PFNGLDRAWBUFFERPROC, DrawBuffer) \
    X(PFNGLDRAWARRAYSINSTANCEDPROC, DrawArraysInstanced) \
    X(PFNGLDRAWELEMENTSINSTANCEDPROC, DrawElementsInstanced) \
    X(PFNGLENABLEPROC, Enable) \
    X(PFNGLENABLEVERTEXATTRIBARRAYPROC, EnableVertexAttribArray) \
    X(PFNGLFRAMEBUFFERTEXTURE2DPROC, FramebufferTexture2D) \
    X(PFNGLFRONTFACEPROC, FrontFace) \
    X(PFNGLGENBUFFERSPROC, GenBuffers) \
    X(PFNGLGENFRAMEBUFFERSPROC, GenFramebuffers) \
    X(PFNGLGENTEXTURESPROC, GenTextures) \
    X(PFNGLGENVERTEXARRAYSPROC, GenVertexArrays) \
    X(PFNGLGETINTEGERVPROC, GetIntegerv) \
    X(PFNGLGETPROGRAMINFOLOGPROC, GetProgramInfoLog) \
    X(PFNGLGETPROGRAMIVPROC, GetProgramiv) \
    X(PFNGLGETSHADERINFOLOGPROC, GetShaderInfoLog) \
    X(PFNGLGETSHADERIVPROC, GetShaderiv) \
    X(PFNGLGETUNIFORMLOCATIONPROC, GetUniformLocation) \
    X(PFNGLLINKPROGRAMPROC, LinkProgram) \
    X(SpadePfnGlMemoryBarrier, MemoryBarrier) \
    X(PFNGLPOLYGONMODEPROC, PolygonMode) \
    X(PFNGLREADBUFFERPROC, ReadBuffer) \
    X(PFNGLPOINTSIZEPROC, PointSize) \
    X(PFNGLSHADERSOURCEPROC, ShaderSource) \
    X(PFNGLTEXIMAGE2DPROC, TexImage2D) \
    X(PFNGLTEXIMAGE3DPROC, TexImage3D) \
    X(PFNGLTEXPARAMETERIPROC, TexParameteri) \
    X(PFNGLUNIFORM1FPROC, Uniform1f) \
    X(PFNGLUNIFORM1UIPROC, Uniform1ui) \
    X(PFNGLUNIFORM2FPROC, Uniform2f) \
    X(PFNGLUNIFORM2UIPROC, Uniform2ui) \
    X(PFNGLUNIFORM3FVPROC, Uniform3fv) \
    X(PFNGLUNIFORM4FVPROC, Uniform4fv) \
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
// horizon_blend(), and of render/shadow.cpp's sample_shadow(), shared by the
// background and the mesh programs.
constexpr const char* kCommonSrc = R"GLSL(
uniform vec3 uSkyZenith;
uniform vec3 uSkyHorizon;
uniform float uHorizonStrength;  // 0 outside shaded mode, as raster_cpu passes it
uniform float uHorizonOnset;

// The sun's shadow map: the CPU's static map, or this frame's copy with the
// dynamic casters added. Unit 0. uShadowEnabled is 0 unless shaded mode asks
// for shadows and the scene has a map, as raster_cpu decides.
layout(binding = 0) uniform sampler2D uShadowMap;
uniform uint uShadowEnabled;
uniform mat4 uLightViewProj;
uniform uint uShadowSize;
uniform float uNoOccluder;    // shadow.hpp's kNoOccluder
uniform float uShadowBias;    // shadow.hpp's kShadowDepthBias

// sample_shadow(): one texel, unfiltered. Lit outside the map's footprint
// (SR-17 clause 6) and where no caster landed. 1 lit, 0 shadowed.
float sample_shadow(vec3 world) {
    if (uShadowEnabled == 0u) return 1.0;
    vec4 p = uLightViewProj * vec4(world, 1.0);
    if (p.x < -1.0 || p.x > 1.0 || p.y < -1.0 || p.y > 1.0) return 1.0;
    float size_f = float(uShadowSize);
    int last = int(uShadowSize) - 1;
    int tx = clamp(int((p.x * 0.5 + 0.5) * size_f), 0, last);
    int ty = clamp(int((p.y * 0.5 + 0.5) * size_f), 0, last);
    float occluder = texelFetch(uShadowMap, ivec2(tx, ty), 0).r;
    if (occluder == uNoOccluder) return 1.0;
    return (p.z < occluder - uShadowBias) ? 0.0 : 1.0;
}

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

// The flow volume on GL (airflow visualization plan §7.1), a port of
// render/flow_sample.cpp. V and S are GL_RGBA16UI 3D textures of raw halves.
// Trilinear is cell-centred, by nested lerps, and reads a corner only when
// its axis fraction is above 0. A read corner whose exponent bits are all
// ones (inf or NaN) is no-data. Bit tests and unpackHalf2x16 keep the
// driver's own half and NaN handling out of it. The colour maps are their 256
// stops, a row per ColourMapId, interpolated as colour_map_at() does.
constexpr const char* kFlowSampleSrc = R"GLSL(
layout(binding = 1) uniform usampler3D uFlowV;  // u, v, w, dye
layout(binding = 2) uniform usampler3D uFlowS;  // p, |omega|, q-hat, reserved
layout(binding = 3) uniform sampler2D uFlowColourTable;  // 256 x 3, a row per ColourMapId
uniform vec3 uFlowOrigin;
uniform float uFlowInvCell;  // 1 / cell, as the CPU's FlowSampler holds it

void flow_axis(float coordinate, float origin, int n, out int i0, out int i1, out float f) {
    float g = clamp((coordinate - origin) * uFlowInvCell - 0.5, 0.0, float(n - 1));
    float base = floor(g);
    i0 = int(base);
    i1 = min(i0 + 1, n - 1);
    f = g - base;
}

// False on no-data. `channel` is a FlowChannel: 0-3 in V, 4-7 in S.
bool flow_sample_channel(vec3 p, uint channel, out float value) {
    ivec3 n = textureSize(uFlowV, 0);
    int x0, x1, y0, y1, z0, z1;
    float fx, fy, fz;
    flow_axis(p.x, uFlowOrigin.x, n.x, x0, x1, fx);
    flow_axis(p.y, uFlowOrigin.y, n.y, y0, y1, fy);
    flow_axis(p.z, uFlowOrigin.z, n.z, z0, z1, fz);
    float c[8] = float[8](0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    for (int dz = 0; dz <= (fz > 0.0 ? 1 : 0); ++dz) {
        for (int dy = 0; dy <= (fy > 0.0 ? 1 : 0); ++dy) {
            for (int dx = 0; dx <= (fx > 0.0 ? 1 : 0); ++dx) {
                ivec3 cell = ivec3(dx == 1 ? x1 : x0, dy == 1 ? y1 : y0, dz == 1 ? z1 : z0);
                uvec4 t = channel < 4u ? texelFetch(uFlowV, cell, 0) : texelFetch(uFlowS, cell, 0);
                uint h = t[int(channel & 3u)];
                if ((h & 0x7C00u) == 0x7C00u) {
                    value = 0.0;
                    return false;
                }
                c[dz * 4 + dy * 2 + dx] = unpackHalf2x16(h).x;
            }
        }
    }
    float a = c[0] + (c[1] - c[0]) * fx;
    float b = c[2] + (c[3] - c[2]) * fx;
    float d = c[4] + (c[5] - c[4]) * fx;
    float e = c[6] + (c[7] - c[6]) * fx;
    float lo = a + (b - a) * fy;
    float hi = d + (e - d) * fy;
    value = lo + (hi - lo) * fz;
    return true;
}

// `quantity` is a FlowQuantity: 0 speed, then u, v, w, dye, pressure,
// vorticity and q-hat, which are FlowChannel 0 to 6.
bool flow_sample_quantity(vec3 p, uint quantity, out float value) {
    if (quantity == 0u) {
        float u, v, w;
        if (!flow_sample_channel(p, 0u, u) || !flow_sample_channel(p, 1u, v) || !flow_sample_channel(p, 2u, w)) {
            value = 0.0;
            return false;
        }
        value = sqrt(u * u + v * v + w * w);
        return true;
    }
    return flow_sample_channel(p, quantity - 1u, value);
}

vec3 flow_colour_map(float t, uint map) {
    int row = int(map);
    float x = clamp(t, 0.0, 1.0) * 255.0;
    if (!(x < 255.0)) return texelFetch(uFlowColourTable, ivec2(255, row), 0).rgb;
    int i = int(x);
    float u = x - float(i);
    vec3 a = texelFetch(uFlowColourTable, ivec2(i, row), 0).rgb;
    vec3 b = texelFetch(uFlowColourTable, ivec2(i + 1, row), 0).rgb;
    return a + (b - a) * u;
}
)GLSL";

// Surface colouring (render/flow_field.hpp), for the mesh and CSG programs.
// A fragment inside the volume's box takes its quantity's colour at its own
// world point, unlit. raster_cpu's draw_flow_surface_colouring() colours the
// point it rebuilds from depth. No-data keeps the shaded colour. Off in a
// program that never sets uFlowColouring. Needs kFlowSampleSrc before it.
constexpr const char* kFlowSurfaceSrc = R"GLSL(
uniform uint uFlowColouring;
uniform uint uFlowQuantity;
uniform uint uFlowColourMap;
uniform vec3 uFlowBoxMin;
uniform vec3 uFlowBoxMax;
uniform float uFlowRangeMin;
uniform float uFlowInvSpan;
uniform uint uFlowConstant;
uniform vec3 uFlowConstantColour;

vec3 flow_colour_surface(vec3 world, vec3 shaded) {
    if (uFlowColouring == 0u) return shaded;
    if (any(lessThan(world, uFlowBoxMin)) || any(greaterThan(world, uFlowBoxMax))) return shaded;
    float value;
    if (!flow_sample_quantity(world, uFlowQuantity, value)) return shaded;
    if (uFlowConstant != 0u) return uFlowConstantColour;
    return flow_colour_map((value - uFlowRangeMin) * uFlowInvSpan, uFlowColourMap);
}
)GLSL";

// A flow layer's prepared transfers and isos as scalars (see draw()):
// transfer k at 36 k; iso k at 144 + 44 k, its colour_by transfer 8 later.
// Shared by the march and the brick pass. Needs kFlowSampleSrc before it.
constexpr const char* kFlowParamsSrc = R"GLSL(
uniform vec4 uFlowParams[80];

const int kIsoBase = 144;
const int kIsoFloats = 44;

float P(int i) { return uFlowParams[i >> 2][i & 3]; }

// A prepared transfer at `b`: quantity, map, constant?, range min, inverse
// span, point count, -, -, constant colour, -, then 8 (value, sigma) points
// and 7 inverse widths. apply_transfer()'s colour and extinction.
vec3 transfer_colour(int b, float value) {
    if (P(b + 2) != 0.0) return vec3(P(b + 8), P(b + 9), P(b + 10));
    return flow_colour_map((value - P(b + 3)) * P(b + 4), uint(P(b + 1)));
}

float transfer_sigma(int b, float value) {
    int last = int(P(b + 5)) - 1;
    if (!(value > P(b + 12))) return P(b + 13);
    if (!(value < P(b + 12 + 2 * last))) return P(b + 13 + 2 * last);
    int i = 0;
    while (!(value < P(b + 12 + 2 * (i + 1)))) ++i;
    float u = (value - P(b + 12 + 2 * i)) * P(b + 28 + i);
    return P(b + 13 + 2 * i) + (P(b + 15 + 2 * i) - P(b + 13 + 2 * i)) * u;
}
)GLSL";

// Empty space (airflow plan §7.2, step 7e): one invocation per 8^3 brick
// takes each channel's range over the brick and a one-cell apron (every
// cell a sample inside the brick can read). The brick is empty when every
// transfer's extinction is 0 over its quantity's range, and every iso's
// level lies strictly outside its quantity's range with no no-data. There a
// step can neither emit, absorb nor cross an iso. Ranges widen by a few
// ulps of their largest magnitude, as far as a trilinear sample can round
// past its corners. Needs kFlowSampleSrc and kFlowParamsSrc before it.
constexpr const char* kFlowBrickSrc = R"GLSL(
layout(local_size_x = 4, local_size_y = 4, local_size_z = 4) in;
layout(binding = 0, r8ui) uniform writeonly uimage3D uFlowEmptyOut;
uniform uint uTransferCount;
uniform uint uIsoCount;

// False when no cell holds a value for the quantity. Speed's range is
// bounded by the velocity channels' largest magnitudes.
bool quantity_range(uint q, float mn[8], float mx[8], out float lo, out float hi) {
    lo = 0.0;
    hi = 0.0;
    if (q == 0u) {
        float s2 = 0.0;
        for (int c = 0; c < 3; ++c) {
            if (mn[c] > mx[c]) return false;
            float m = max(abs(mn[c]), abs(mx[c]));
            s2 += m * m;
        }
        hi = sqrt(s2);
    } else {
        int c = int(q) - 1;
        if (mn[c] > mx[c]) return false;
        lo = mn[c];
        hi = mx[c];
    }
    // A nested lerp can round a few ulps of its largest corner past the
    // range, never more; a range of one value interpolates exactly.
    float w = max(abs(lo), abs(hi)) * 1e-5;
    lo -= w;
    hi += w;
    return true;
}

bool quantity_has_no_data(uint q, bool nd[8]) {
    return q == 0u ? (nd[0] || nd[1] || nd[2]) : nd[int(q) - 1];
}

void main() {
    ivec3 brick = ivec3(gl_GlobalInvocationID);
    if (any(greaterThanEqual(brick, imageSize(uFlowEmptyOut)))) return;
    ivec3 n = textureSize(uFlowV, 0);
    ivec3 c0 = max(brick * 8 - 1, ivec3(0));
    ivec3 c1 = min(brick * 8 + 8, n - 1);
    float mn[8];
    float mx[8];
    bool nd[8];
    for (int c = 0; c < 8; ++c) {
        mn[c] = 3.402823466e+38;
        mx[c] = -3.402823466e+38;
        nd[c] = false;
    }
    for (int z = c0.z; z <= c1.z; ++z) {
        for (int y = c0.y; y <= c1.y; ++y) {
            for (int x = c0.x; x <= c1.x; ++x) {
                uvec4 a = texelFetch(uFlowV, ivec3(x, y, z), 0);
                uvec4 b = texelFetch(uFlowS, ivec3(x, y, z), 0);
                for (int c = 0; c < 8; ++c) {
                    uint h = c < 4 ? a[c] : b[c - 4];
                    if ((h & 0x7C00u) == 0x7C00u) {
                        nd[c] = true;
                        continue;
                    }
                    float v = unpackHalf2x16(h).x;
                    mn[c] = min(mn[c], v);
                    mx[c] = max(mx[c], v);
                }
            }
        }
    }
    bool empty = true;
    for (int k = 0; k < int(uTransferCount) && empty; ++k) {
        int b = 36 * k;
        float lo, hi;
        if (!quantity_range(uint(P(b)), mn, mx, lo, hi)) continue;  // every step there is clear
        float m = max(transfer_sigma(b, lo), transfer_sigma(b, hi));
        for (int i = 0; i < int(P(b + 5)); ++i) {
            float e = P(b + 12 + 2 * i);
            if (e > lo && e < hi) m = max(m, P(b + 13 + 2 * i));
        }
        if (m > 0.0) empty = false;
    }
    for (int k = 0; k < int(uIsoCount) && empty; ++k) {
        int b = kIsoBase + k * kIsoFloats;
        uint q = uint(P(b));
        float lo, hi;
        if (quantity_has_no_data(q, nd) || !quantity_range(q, mn, mx, lo, hi)) {
            empty = false;
        } else {
            float level = P(b + 1);
            if (!(hi < level || lo >= level)) empty = false;
        }
    }
    imageStore(uFlowEmptyOut, brick, uvec4(empty ? 1u : 0u, 0u, 0u, 0u));
}
)GLSL";

// A flow layer, ray-marched (airflow visualization plan §7.1, step 7c): a
// port of render/flow_render.cpp's march_flow_ray(), drawn on the layer's
// box's back faces (kCsgVertexSrc) with depth clamp, so every pixel the box
// covers runs one march. The ray, the box clip and the near and far limits
// are the CSG pass's. The march stops at the frame's opaque depth, which a
// depth-only copy of the opaque passes leaves in uOpaqueDepth. Half-cell
// steps by multiplication, the last one partial; iso crossings refined by
// four bisections and a linear step, nearest first; emission and absorption
// with every transfer at once; out at T < 1/256. GLSL's exp() stands in for
// exp32(). The result blends as C + T * behind (ONE, SRC_ALPHA). An opaque
// iso writes its depth; any other pixel writes back the depth it had.
// With uSkip, a step in an empty brick (7e) runs, and so does the last step
// whose midpoint is still in that brick; the steps between change nothing,
// so the frame is the unskipped march's. Needs kFlowSampleSrc and
// kFlowParamsSrc before it.
constexpr const char* kFlowVolumeFragmentSrc = R"GLSL(
uniform mat4 uViewProj;
uniform vec3 uBoxMin;
uniform vec3 uBoxMax;
uniform vec3 uCamPos;
uniform vec3 uRight;
uniform vec3 uUp;
uniform vec3 uForward;
uniform float uInvTanHalfFov;
uniform float uAspect;
uniform vec2 uViewport;
uniform float uNear;
uniform float uFar;
uniform float uProj22;  // the projection's depth terms, to undo window depth
uniform float uProj32;
layout(binding = 4) uniform sampler2D uOpaqueDepth;
uniform float uStep;
uniform float uInvStep;
uniform uint uTransferCount;
uniform uint uIsoCount;
uniform vec3 uSunDir;
uniform vec3 uSunColor;
uniform vec3 uAmbient;
uniform float uSunIntensity;
layout(binding = 5) uniform usampler3D uFlowEmpty;  // 1 per empty 8^3 brick (7e)
uniform uint uSkip;

out vec4 fragColor;

bool iso_offset(int k, vec3 origin, vec3 dir, float t, out float f) {
    int b = kIsoBase + k * kIsoFloats;
    float v;
    if (!flow_sample_quantity(origin + dir * t, uint(P(b)), v)) {
        f = 0.0;
        return false;
    }
    f = v - P(b + 1);
    return true;
}

float refine_crossing(int k, vec3 origin, vec3 dir, float ta, float fa, float tb, float fb) {
    for (int i = 0; i < 4; ++i) {  // kIsoBisections
        float tm = 0.5 * (ta + tb);
        float fm;
        if (!iso_offset(k, origin, dir, tm, fm)) break;
        if ((fa < 0.0) != (fm < 0.0)) {
            tb = tm;
            fb = fm;
        } else {
            ta = tm;
            fa = fm;
        }
    }
    float span = fa - fb;
    return span != 0.0 ? ta + (tb - ta) * (fa / span) : 0.5 * (ta + tb);
}

// shade_iso(): its colour or colour_by's, lit about the field's gradient
// (central differences one cell apart), turned toward the eye.
vec3 shade_iso(int k, vec3 q, vec3 dir, inout bool crossed_no_data) {
    int b = kIsoBase + k * kIsoFloats;
    uint quantity = uint(P(b));
    vec3 base = vec3(P(b + 4), P(b + 5), P(b + 6));
    if (P(b + 3) != 0.0) {
        float v;
        if (!flow_sample_quantity(q, uint(P(b + 8)), v)) {
            crossed_no_data = true;
        } else {
            base = transfer_colour(b + 8, v);
        }
    }
    vec3 grad = vec3(0.0);
    bool ok = true;
    for (int a = 0; a < 3 && ok; ++a) {
        vec3 e = vec3(0.0);
        e[a] = uStep;
        float hi, lo;
        bool hi_ok = flow_sample_quantity(q + e, quantity, hi);
        bool lo_ok = flow_sample_quantity(q - e, quantity, lo);
        ok = hi_ok && lo_ok;
        grad[a] = hi - lo;
    }
    float len = sqrt(grad.x * grad.x + grad.y * grad.y + grad.z * grad.z);
    vec3 normal = ok && len > 0.0 ? grad / len : -dir;
    if (dot(normal, dir) > 0.0) normal = -normal;
    float ndl = max(dot(normal, uSunDir), 0.0);
    return base * (uSunColor * (uSunIntensity * ndl) + uAmbient);
}

void main() {
    float x_ndc = 2.0 * gl_FragCoord.x / uViewport.x - 1.0;
    float y_ndc = 2.0 * gl_FragCoord.y / uViewport.y - 1.0;
    vec3 ray = uForward + uRight * (x_ndc * uAspect / uInvTanHalfFov) + uUp * (y_ndc / uInvTanHalfFov);
    vec3 dir = normalize(ray);
    float cos_theta = dot(dir, uForward);

    float t_enter = -3.402823466e+38;
    float t_exit = 3.402823466e+38;
    for (int a = 0; a < 3; ++a) {
        if (dir[a] == 0.0) {
            if (uCamPos[a] < uBoxMin[a] || uCamPos[a] > uBoxMax[a]) discard;
            continue;
        }
        float inv = 1.0 / dir[a];
        float ta = (uBoxMin[a] - uCamPos[a]) * inv;
        float tb = (uBoxMax[a] - uCamPos[a]) * inv;
        t_enter = max(t_enter, min(ta, tb));
        t_exit = min(t_exit, max(ta, tb));
    }
    if (!(t_enter <= t_exit && t_exit >= 0.0)) discard;
    float depth = texelFetch(uOpaqueDepth, ivec2(gl_FragCoord.xy), 0).r;
    float t_end = min(uFar / cos_theta, t_exit);
    if (depth < 1.0) {
        // An opaque surface hides the air behind it.
        t_end = min(t_end, (uProj32 / ((2.0 * depth - 1.0) + uProj22)) / cos_theta);
    }
    float t_start = max(uNear / cos_theta, t_enter);

    vec3 colour = vec3(0.0);
    float transmittance = 1.0;
    bool crossed_no_data = false;
    float opaque_t = -1.0;
    if (t_start < t_end) {
        float length = t_end - t_start;
        float full = floor(length * uInvStep);
        uint n_full = uint(full);
        float last = max(0.0, length - full * uStep);
        uint n = n_full + (last > 0.0 ? 1u : 0u);
        int n_isos = int(uIsoCount);
        float prev_f[4] = float[4](0.0, 0.0, 0.0, 0.0);
        float prev_t[4] = float[4](0.0, 0.0, 0.0, 0.0);
        bool prev_valid[4] = bool[4](false, false, false, false);
        uint next = 0u;
        for (uint i = 0u; i < n && opaque_t < 0.0; i = next) {
            next = i + 1u;
            float t0 = t_start + float(i) * uStep;
            float dt = i < n_full ? uStep : last;
            float tm = t0 + 0.5 * dt;
            vec3 p = uCamPos + dir * tm;
            if (uSkip != 0u && i + 2u < n_full) {
                ivec3 nb = textureSize(uFlowEmpty, 0);
                ivec3 brick = clamp(ivec3(floor((p - uFlowOrigin) * uFlowInvCell * 0.125)), ivec3(0), nb - 1);
                if (texelFetch(uFlowEmpty, brick, 0).r != 0u) {
                    vec3 bmin = uFlowOrigin + vec3(brick) * (8.0 / uFlowInvCell);
                    vec3 bmax = bmin + vec3(8.0 / uFlowInvCell);
                    float t_out = 3.402823466e+38;
                    for (int a = 0; a < 3; ++a) {
                        if (dir[a] > 0.0) t_out = min(t_out, (bmax[a] - uCamPos[a]) / dir[a]);
                        if (dir[a] < 0.0) t_out = min(t_out, (bmin[a] - uCamPos[a]) / dir[a]);
                    }
                    // The last full step whose midpoint is still in the brick.
                    float j = min(floor((t_out - t_start) * uInvStep - 0.5), float(n_full - 1u));
                    if (j > float(i) + 1.0) next = uint(j);
                }
            }

            float hit_t[4];
            int hit_k[4];
            int n_hits = 0;
            for (int k = 0; k < n_isos; ++k) {
                float f;
                if (!iso_offset(k, uCamPos, dir, tm, f)) {
                    crossed_no_data = true;
                    prev_valid[k] = false;
                    continue;
                }
                if (prev_valid[k] && (prev_f[k] < 0.0) != (f < 0.0)) {
                    hit_t[n_hits] = refine_crossing(k, uCamPos, dir, prev_t[k], prev_f[k], tm, f);
                    hit_k[n_hits] = k;
                    ++n_hits;
                }
                prev_f[k] = f;
                prev_t[k] = tm;
                prev_valid[k] = true;
            }
            for (int a = 1; a < n_hits; ++a) {
                for (int b = a; b > 0 && hit_t[b] < hit_t[b - 1]; --b) {
                    float tt = hit_t[b];
                    hit_t[b] = hit_t[b - 1];
                    hit_t[b - 1] = tt;
                    int kk = hit_k[b];
                    hit_k[b] = hit_k[b - 1];
                    hit_k[b - 1] = kk;
                }
            }
            for (int h = 0; h < n_hits && opaque_t < 0.0; ++h) {
                int b = kIsoBase + hit_k[h] * kIsoFloats;
                float opacity = P(b + 2);
                vec3 shaded = shade_iso(hit_k[h], uCamPos + dir * hit_t[h], dir, crossed_no_data);
                colour += transmittance * opacity * shaded;
                if (opacity >= 1.0) {
                    transmittance = 0.0;
                    opaque_t = hit_t[h];
                } else {
                    transmittance *= 1.0 - opacity;
                }
            }
            if (opaque_t >= 0.0 || uTransferCount == 0u) continue;

            float sigma = 0.0;
            vec3 weighted = vec3(0.0);
            bool no_data = false;
            for (int k = 0; k < int(uTransferCount); ++k) {
                int b = 36 * k;
                float v;
                if (!flow_sample_quantity(p, uint(P(b)), v)) {
                    no_data = true;
                    break;
                }
                float sk = transfer_sigma(b, v);
                sigma += sk;
                weighted += sk * transfer_colour(b, v);
            }
            if (no_data) {
                crossed_no_data = true;
                continue;
            }
            if (!(sigma > 0.0)) continue;
            float tau = exp(-sigma * dt);
            colour += transmittance * (1.0 - tau) * (weighted / sigma);
            transmittance *= tau;
            if (transmittance < 1.0 / 256.0) break;  // kFlowEarlyExitTransmittance
        }
    }
    if (transmittance == 1.0 && colour == vec3(0.0)) discard;  // nothing absorbed or emitted
    fragColor = vec4(colour, transmittance);
    if (opaque_t >= 0.0) {
        vec4 clip = uViewProj * vec4(uCamPos + dir * opaque_t, 1.0);
        gl_FragDepth = (clip.z / clip.w) * 0.5 + 0.5;
    } else {
        gl_FragDepth = depth;
    }
}
)GLSL";

// The surface shading the mesh and CSG programs share: the material table,
// the sun, velocity's ramp, the shadow lookup and the SR-17a blend, as
// raster_cpu shades a mesh fragment. Needs kCommonSrc before it.
constexpr const char* kSurfaceSrc = R"GLSL(
struct GpuMaterial {
    vec4 base_color;
    uint shading;
    uint pad0;
    uint pad1;
    uint pad2;
};

layout(std430, binding = 2) readonly buffer MaterialSet { GpuMaterial uMaterials[]; };

uniform vec3 uSunDir;
uniform vec3 uSunColor;
uniform vec3 uAmbient;
uniform float uSunIntensity;
uniform uint uMode;            // render::DrawMode: 0 shaded, 1 wireframe, 3 velocity
uniform float uVelocityScale;  // RenderOptions::velocity_scale_mps
uniform vec3 uCamPos;

// raster_cpu's velocity_ramp(): blue at rest, red at the scale and above.
vec3 velocity_ramp(float speed, float scale) {
    if (!(scale > 0.0)) return vec3(1.0, 0.0, 0.0);
    float u = clamp(speed / scale, 0.0, 1.0);
    return vec3(u, 0.0, 1.0 - u);
}

// `n` is unit. Unlit and emissive keep `base`; lambert takes the sun and
// ambient, and a shadowed point loses the sun term and keeps the ambient,
// as raster_cpu's apply_shadow() does.
vec3 shade_surface(uint shading, vec3 base, vec3 n, vec3 world) {
    vec3 lit;
    if (shading == 1u || shading == 2u) {
        lit = base;                       // 1 = unlit, 2 = emissive
    } else {
        // uSunDir points FROM the scene TOWARD the sun (LightingDesc's
        // convention), so N.L is dot(n, +uSunDir) -- the same term as
        // render/scene.hpp's shade_vertex_color(). Negating it lit every
        // world-loaded scene from below on this path only.
        float ndl = max(dot(n, uSunDir), 0.0);
        lit = base * (uAmbient + uSunColor * uSunIntensity * ndl * sample_shadow(world));
    }
    // SR-17a, per pixel along this pixel's own ray, as raster_cpu does it.
    vec3 eye_to_surface = world - uCamPos;
    return horizon_blend(lit, sky_gradient_color(eye_to_surface), length(eye_to_surface));
}
)GLSL";

constexpr const char* kFragmentSrc = R"GLSL(
in vec3 vNormal;
in vec3 vWorld;
flat in uint vInstance;

layout(std430, binding = 1) readonly buffer Overrides  { uint uOverride[]; };
layout(std430, binding = 4) readonly buffer Speeds { float uSpeed[]; };

uniform uint uSubmeshMaterial;
uniform uint uNoMaterial;

out vec4 fragColor;

void main() {
    // DrawItem::material_override wins when set, else the submesh's own
    // material -- the same precedence raster_cpu applies, stated once here.
    uint mi = uOverride[vInstance];
    if (mi == uNoMaterial) { mi = uSubmeshMaterial; }
    GpuMaterial m = uMaterials[mi];

    vec3 base = m.base_color.rgb;
    // Wireframe edges take the unlit material colour, as raster_cpu draws them.
    if (uMode == 1u) {
        fragColor = vec4(flow_colour_surface(vWorld, base), 1.0);
        return;
    }
    // Velocity swaps the colour and keeps the material's shading, as raster_cpu does.
    if (uMode == 3u) { base = velocity_ramp(uSpeed[vInstance], uVelocityScale); }
    fragColor = vec4(flow_colour_surface(vWorld, shade_surface(m.shading, base, normalize(vNormal), vWorld)), 1.0);
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
// 80 bytes. The colour is shaded once per plane on the host: `color` is
// shade_vertex_color()'s combined term and `sun` its sun term, which a
// shadowed pixel loses.
struct GroundPlaneGpu {
    vec3 normal;
    float offset;    // dot(p, normal) <= offset is solid
    vec3 color;
    uint is_front;   // the camera is above the plane, decided on the host in double
    vec3 grid_u;     // ground_plane_basis()
    float pad0;
    vec3 grid_v;
    float pad1;
    vec3 sun;
    float pad2;
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
        // Shaded, then shadowed, then gridded, then blended: raster_cpu's order.
        color = uPlanes[best].color - uPlanes[best].sun * (1.0 - sample_shadow(hit));
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

// Overlays (render::overlay_geometry(), the CPU's own lists): flat, unlit,
// depth-tested. Each vertex moves along its view ray to the depth whose
// inverse is 1/d + uDepthBias, which is raster_cpu's overlay bias with the
// screen position unchanged. glPolygonOffset would not reach GL_LINES.
constexpr const char* kOverlayVertexSrc = R"GLSL(
layout(location = 0) in vec3 aPos;
layout(location = 1) in vec3 aColour;

uniform mat4 uView;  // world to camera
uniform mat4 uProj;
uniform float uDepthBias;

out vec3 vColour;

void main() {
    vec4 pc = uView * vec4(aPos, 1.0);
    float d = -pc.z;
    if (d > 0.0) {
        pc.xyz *= 1.0 / (1.0 + uDepthBias * d);  // d' = 1 / (1/d + bias)
    }
    vColour = aColour;
    gl_Position = uProj * pc;
}
)GLSL";

constexpr const char* kOverlayFragmentSrc = R"GLSL(
in vec3 vColour;
out vec4 fragColor;

void main() {
    fragColor = vec4(vColour, 1.0);
}
)GLSL";

// Dynamic shadow casters, drawn from the sun into this frame's copy of the
// static map (render/shadow.cpp's rasterize_shadow_casters()). Each fragment
// writes its light-clip z, and GL_MAX blending keeps the larger: the CPU's
// "closer to the sun wins". Nothing is culled, and depth clamp stands in for
// the CPU's rasterizer, which clips nothing in z.
constexpr const char* kCasterVertexSrc = R"GLSL(
layout(location = 0) in vec3 aPos;
layout(std430, binding = 0) readonly buffer Transforms { mat4 uModel[]; };

uniform uint uInstanceBase;
uniform mat4 uLightViewProj;

out float vLightZ;

void main() {
    vec4 p = uLightViewProj * (uModel[uInstanceBase + uint(gl_InstanceID)] * vec4(aPos, 1.0));
    vLightZ = p.z;
    gl_Position = p;
}
)GLSL";

constexpr const char* kCasterFragmentSrc = R"GLSL(
in float vLightZ;
out vec4 fragColor;

void main() {
    fragColor = vec4(vLightZ, 0.0, 0.0, 1.0);
}
)GLSL";

// CSG subtrees, ray-marched (RS3, replacement signed 2026-10-05;
// rendering/plans/2026-10-04-b2-raymarched-csg-plan.md, step 3). Each
// subtree draws its box's back faces as a proxy: one fragment for every
// pixel the box covers, whether the camera is outside it or in. The
// fragment shader marches the subtree from the box entry, as raster_cpu's
// draw_csg_subtrees() does, and writes its own depth.
constexpr const char* kCsgVertexSrc = R"GLSL(
uniform mat4 uViewProj;
uniform vec3 uBoxMin;
uniform vec3 uBoxMax;

// The box's 12 triangles, wound counter-clockwise seen from outside. A
// corner's bits are x, y, z: 0 for the min side, 1 for the max.
const int kBoxCorner[36] = int[36](0, 4, 6, 0, 6, 2,  1, 3, 7, 1, 7, 5,  0, 1, 5, 0, 5, 4,
                                   2, 6, 7, 2, 7, 3,  0, 2, 3, 0, 3, 1,  4, 5, 7, 4, 7, 6);

void main() {
    int c = kBoxCorner[gl_VertexID];
    vec3 f = vec3(float(c & 1), float((c >> 1) & 1), float((c >> 2) & 1));
    gl_Position = uViewProj * vec4(mix(uBoxMin, uBoxMax, f), 1.0);
}
)GLSL";

// A hand port of world/sdf.cpp's eval() and sample() gradient (Q3, the
// lead's call of 2026-10-05), the same expressions in the same order.
// Banded against the CPU, not bit-exact (RND-3). The program is the
// subtree's own (csg_subtree_program()); its nodes and transforms sit in
// two buffers shared by every subtree, from uNodeBase and uTransformBase.
constexpr const char* kSdfSrc = R"GLSL(
// world/sdf.hpp's SdfNode and SdfTransform, byte for byte.
struct GpuSdfNode {
    uint kind;
    uint op;
    uint transform;
    uint pad;
    vec4 params;
};
struct GpuSdfTransform {
    mat4 world_to_local;
    float scale;
    float pad0;
    float pad1;
    float pad2;
};
layout(std430, binding = 6) readonly buffer SdfNodes { GpuSdfNode uNodes[]; };
layout(std430, binding = 7) readonly buffer SdfTransforms { GpuSdfTransform uTransforms[]; };

uniform uint uNodeBase;
uniform uint uNodeCount;
uniform uint uTransformBase;
uniform float uGradientStep;  // world/sdf.hpp's kSdfGradientStep

const int kMaxSdfDepth = 32;  // world/sdf.hpp's; gl_renderer.cpp asserts they agree
const float kSdfEmptyDistance = 3.402823466e+38;

float sgn(float x) { return x < 0.0 ? -1.0 : 1.0; }

float primitive_distance(uint kind, vec4 prm, vec3 p) {
    switch (kind) {
        case 0u:  // plane
            return dot(p, prm.xyz) - prm.w;
        case 1u:  // sphere
            return length(p) - prm.x;
        case 2u: {  // box
            vec3 q = abs(p) - prm.xyz;
            return length(max(q, 0.0)) + min(max(q.x, max(q.y, q.z)), 0.0);
        }
        case 3u: {  // cylinder
            vec2 d = vec2(length(p.xz) - prm.x, abs(p.y) - prm.y);
            return min(max(d.x, d.y), 0.0) + length(max(d, 0.0));
        }
        case 4u: {  // capsule
            vec3 q = p;
            q.y -= clamp(q.y, -prm.y, prm.y);
            return length(q) - prm.x;
        }
        case 5u: {  // torus
            vec2 q = vec2(length(p.xz) - prm.x, p.y);
            return length(q) - prm.y;
        }
        case 6u: {  // heightfield. The driver's sin, where the CPU has sin32, which also reads
                   // flat beyond |f x| >= 2^24; this one does not. Banded, not bit-exact (RND-3).
            float h = prm.w + prm.x * sin(prm.y * p.x) * sin(prm.z * p.z);
            float slope = abs(prm.x) * max(abs(prm.y), abs(prm.z));
            return (p.y - h) / sqrt(1.0 + slope * slope);
        }
    }
    return kSdfEmptyDistance;
}

vec3 central_difference(uint kind, vec4 prm, vec3 p) {
    float h = uGradientStep;
    float inv_2h = 0.5 / h;
    float dx = primitive_distance(kind, prm, vec3(p.x + h, p.y, p.z)) - primitive_distance(kind, prm, vec3(p.x - h, p.y, p.z));
    float dy = primitive_distance(kind, prm, vec3(p.x, p.y + h, p.z)) - primitive_distance(kind, prm, vec3(p.x, p.y - h, p.z));
    float dz = primitive_distance(kind, prm, vec3(p.x, p.y, p.z + h)) - primitive_distance(kind, prm, vec3(p.x, p.y, p.z - h));
    return vec3(dx * inv_2h, dy * inv_2h, dz * inv_2h);
}

vec3 primitive_gradient(uint kind, vec4 prm, vec3 p) {
    switch (kind) {
        case 0u:
            return prm.xyz;
        case 1u: {
            float len = length(p);
            return len > 0.0 ? p / len : vec3(0.0);
        }
        case 2u: {
            vec3 q = abs(p) - prm.xyz;
            float qmax = max(q.x, max(q.y, q.z));
            if (qmax > 0.0) {
                vec3 e = max(q, 0.0);
                float len = length(e);
                if (len <= 0.0) return vec3(0.0);
                return vec3(sgn(p.x), sgn(p.y), sgn(p.z)) * (e / len);
            }
            if (q.x >= q.y && q.x >= q.z) return vec3(sgn(p.x), 0.0, 0.0);
            if (q.y >= q.z) return vec3(0.0, sgn(p.y), 0.0);
            return vec3(0.0, 0.0, sgn(p.z));
        }
    }
    return central_difference(kind, prm, p);  // cylinder, capsule, torus, heightfield
}

float smooth_union_weight(float k, float a, float b) {
    return clamp(0.5 + 0.5 * (b - a) / k, 0.0, 1.0);
}

float combine_distance(uint op, vec4 prm, float a, float b) {
    switch (op) {
        case 1u: return a <= b ? a : b;    // union
        case 2u: return a >= b ? a : b;    // intersect
        case 3u: return a >= -b ? a : -b;  // subtract: a minus b
        case 4u: {                         // smooth_union
            float k = prm.x;
            if (!(k > 0.0)) return a <= b ? a : b;
            float h = smooth_union_weight(k, a, b);
            return (b + (a - b) * h) - k * h * (1.0 - h);
        }
    }
    return a;
}

vec3 combine_gradient(uint op, vec4 prm, float a, float b, vec3 ga, vec3 gb) {
    switch (op) {
        case 1u: return a <= b ? ga : gb;
        case 2u: return a >= b ? ga : gb;
        case 3u: return a >= -b ? ga : -gb;
        case 4u: {
            float k = prm.x;
            if (!(k > 0.0)) return a <= b ? ga : gb;
            float h = smooth_union_weight(k, a, b);
            return gb + (ga - gb) * h;
        }
    }
    return ga;
}

float sdf_eval(vec3 p) {
    float stack[kMaxSdfDepth];
    int sp = 0;
    for (uint i = 0u; i < uNodeCount; ++i) {
        GpuSdfNode node = uNodes[uNodeBase + i];
        if (node.op == 0u) {
            GpuSdfTransform t = uTransforms[uTransformBase + node.transform];
            vec3 local = (t.world_to_local * vec4(p, 1.0)).xyz;
            stack[sp++] = primitive_distance(node.kind, node.params, local) * t.scale;
        } else {
            float b = stack[--sp];
            float a = stack[--sp];
            stack[sp++] = combine_distance(node.op, node.params, a, b);
        }
    }
    return sp == 1 ? stack[0] : kSdfEmptyDistance;
}

// sample()'s gradient: distances and gradients on one stack.
vec3 sdf_gradient(vec3 p) {
    float dist[kMaxSdfDepth];
    vec3 grad[kMaxSdfDepth];
    int sp = 0;
    for (uint i = 0u; i < uNodeCount; ++i) {
        GpuSdfNode node = uNodes[uNodeBase + i];
        if (node.op == 0u) {
            GpuSdfTransform t = uTransforms[uTransformBase + node.transform];
            vec3 local = (t.world_to_local * vec4(p, 1.0)).xyz;
            dist[sp] = primitive_distance(node.kind, node.params, local) * t.scale;
            grad[sp] = (transpose(mat3(t.world_to_local)) * primitive_gradient(node.kind, node.params, local)) * t.scale;
            ++sp;
        } else {
            --sp;
            float b = dist[sp];
            vec3 gb = grad[sp];
            --sp;
            float a = dist[sp];
            vec3 ga = grad[sp];
            dist[sp] = combine_distance(node.op, node.params, a, b);
            grad[sp] = combine_gradient(node.op, node.params, a, b, ga, gb);
            ++sp;
        }
    }
    return sp == 1 ? grad[0] : vec3(0.0);
}
)GLSL";

// render/raymarch.cpp's sphere_trace(), marching past the solid a ray
// starts in, then raster_cpu's draw_csg_subtrees(): the ray through this
// pixel's centre, clipped to the box and to near and far, the hit's depth
// in the mesh pass's convention, and the mesh pass's shading with the SDF
// gradient as the normal. Needs kCommonSrc, kSurfaceSrc and kSdfSrc.
constexpr const char* kCsgFragmentSrc = R"GLSL(
uniform mat4 uViewProj;
uniform vec3 uBoxMin;
uniform vec3 uBoxMax;
uniform vec3 uRight;
uniform vec3 uUp;
uniform vec3 uForward;
uniform float uInvTanHalfFov;
uniform float uAspect;
uniform vec2 uViewport;
uniform float uNear;
uniform float uFar;
uniform uint uMarchMaxSteps;     // render/raymarch.hpp's kRaymarchMaxSteps
uniform float uMarchEpsilon;     // kRaymarchSurfaceEpsilon (SR-28)
uniform uint uSubtreeMaterial;   // the draw item's material
uniform float uSubtreeSpeed;     // the draw item's speed, for velocity mode

out vec4 fragColor;

bool sphere_trace(vec3 origin, vec3 dir, float t_start, float t_end, out vec3 hit) {
    float t = t_start;
    vec3 p = origin + dir * t;
    float d = sdf_eval(p);
    bool cleared_start_solid = d > 0.0;
    for (uint step = 0u; step < uMarchMaxSteps; ++step) {
        if (!cleared_start_solid && d > uMarchEpsilon) cleared_start_solid = true;
        if (cleared_start_solid) {
            if (d <= uMarchEpsilon) {
                hit = p;
                return true;
            }
            t += d;
        } else {
            t += max(abs(d), uMarchEpsilon);  // march past the solid the ray starts in
        }
        if (t > t_end) break;
        p = origin + dir * t;
        d = sdf_eval(p);
    }
    hit = p;
    return false;
}

void main() {
    // The background pass's ray, normalised, as render/raymarch's camera_ray().
    float x_ndc = 2.0 * gl_FragCoord.x / uViewport.x - 1.0;
    float y_ndc = 2.0 * gl_FragCoord.y / uViewport.y - 1.0;
    vec3 ray = uForward + uRight * (x_ndc * uAspect / uInvTanHalfFov) + uUp * (y_ndc / uInvTanHalfFov);
    vec3 dir = normalize(ray);
    float cos_theta = dot(dir, uForward);

    // The slab test, as raster_cpu's clip_ray_to_box().
    float t_enter = -3.402823466e+38;
    float t_exit = 3.402823466e+38;
    for (int a = 0; a < 3; ++a) {
        if (dir[a] == 0.0) {
            if (uCamPos[a] < uBoxMin[a] || uCamPos[a] > uBoxMax[a]) discard;
            continue;
        }
        float inv = 1.0 / dir[a];
        float ta = (uBoxMin[a] - uCamPos[a]) * inv;
        float tb = (uBoxMax[a] - uCamPos[a]) * inv;
        t_enter = max(t_enter, min(ta, tb));
        t_exit = min(t_exit, max(ta, tb));
    }
    if (!(t_enter <= t_exit && t_exit >= 0.0)) discard;
    float t_start = max(uNear / cos_theta, t_enter);
    float t_end = min(uFar / cos_theta, t_exit);
    if (!(t_start <= t_end)) discard;

    vec3 p;
    if (!sphere_trace(uCamPos, dir, t_start, t_end, p)) discard;

    vec4 clip = uViewProj * vec4(p, 1.0);
    gl_FragDepth = (clip.z / clip.w) * 0.5 + 0.5;

    GpuMaterial m = uMaterials[uSubtreeMaterial];
    vec3 base = uMode == 3u ? velocity_ramp(uSubtreeSpeed, uVelocityScale) : m.base_color.rgb;
    vec3 g = sdf_gradient(p);
    float g_len = length(g);
    vec3 n = g_len > 0.0 ? g / g_len : vec3(0.0, 1.0, 0.0);
    fragColor = vec4(flow_colour_surface(p, shade_surface(m.shading, base, n, p)), 1.0);
}
)GLSL";

// 32 bytes, explicitly padded. See the shader comment above.
struct GpuMaterial {
    glm::vec4 base_color{0.72f, 0.72f, 0.74f, 1.0f};
    uint32_t shading = 0;
    uint32_t pad0 = 0, pad1 = 0, pad2 = 0;
};
static_assert(sizeof(GpuMaterial) == 32, "std430 layout must match the shader's GpuMaterial");

// 80 bytes: std430 packs each vec3 with the scalar after it into 16.
struct GpuGroundPlane {
    glm::vec3 normal{0.0f};
    float offset = 0.0f;
    glm::vec3 color{0.0f};
    uint32_t is_front = 0;
    glm::vec3 grid_u{0.0f};
    float pad0 = 0.0f;
    glm::vec3 grid_v{0.0f};
    float pad1 = 0.0f;
    glm::vec3 sun{0.0f};  // the sun term a shadowed ground pixel loses
    float pad2 = 0.0f;
};
static_assert(sizeof(GpuGroundPlane) == 80, "std430 layout must match the shader's GroundPlaneGpu");

// kSdfSrc uploads SdfNode and SdfTransform as they are, and sizes its stacks
// by a literal; these hold the two sides together.
static_assert(sizeof(SdfNode) == 32, "kSdfSrc's GpuSdfNode is 32 bytes");
static_assert(sizeof(SdfTransform) == 80, "kSdfSrc's GpuSdfTransform is 80 bytes");
static_assert(kMaxSdfDepth == 32, "kSdfSrc's kMaxSdfDepth must match world/sdf.hpp's");

// One CSG subtree's place in the shared node and transform buffers.
struct GpuCsgSubtree {
    uint32_t node_base = 0, node_count = 0, transform_base = 0;
};

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

// `program` if it linked, else the driver's log, and the program deleted.
[[nodiscard]] Result<GLuint> linked(const GlApi& gl, GLuint program, const char* what) {
    GLint ok = GL_FALSE;
    gl.GetProgramiv(program, GL_LINK_STATUS, &ok);
    if (ok != GL_TRUE) {
        GLint len = 0;
        gl.GetProgramiv(program, GL_INFO_LOG_LENGTH, &len);
        std::string log(static_cast<size_t>(len > 0 ? len : 1), '\0');
        gl.GetProgramInfoLog(program, len, nullptr, log.data());
        gl.DeleteProgram(program);
        return std::unexpected(Error{Code::internal, std::string("GlRenderer::create: link ") + what + ": " + log});
    }
    return program;
}

[[nodiscard]] Result<GLuint> link_compute(const GlApi& gl, std::initializer_list<const char*> compute,
                                          const char* what) {
    const Result<GLuint> cs = compile(gl, GL_COMPUTE_SHADER, compute, what);
    if (!cs) return std::unexpected(cs.error());
    const GLuint program = gl.CreateProgram();
    gl.AttachShader(program, *cs);
    gl.LinkProgram(program);
    gl.DeleteShader(*cs);
    return linked(gl, program, what);
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
    return linked(gl, program, what);
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

    // Overlays: their own program, and one vertex buffer of position and
    // colour, refilled each frame from render::overlay_geometry().
    GLuint overlay_program = 0;
    GLuint overlay_vao = 0;
    GLuint overlay_vbo = 0;
    size_t overlay_capacity = 0;  // in floats
    std::vector<float> overlay_vertices;

    // The sun's shadow map (render/shadow.hpp): the CPU's static map,
    // uploaded with the scene, and a working copy that a frame with dynamic
    // casters draws them into, through `shadow_fbo`.
    GLuint shadow_static_tex = 0;
    GLuint shadow_work_tex = 0;
    GLuint shadow_fbo = 0;

    // Surface colouring's volume (V and S, raw halves) on units 1 and 2, and
    // its colour table on unit 3. Uploaded by each draw() that colours.
    GLuint flow_v_tex = 0;
    GLuint flow_s_tex = 0;
    GLuint flow_colour_tex = 0;

    // A flow layer's march (step 7c), and the frame's opaque depth it stops
    // at: a depth-only copy of the opaque passes, sized to the frame.
    GLuint flow_program = 0;
    GLuint flow_brick_program = 0;
    GLuint flow_empty_tex = 0;  // R8UI, one texel per 8^3 brick (7e)
    GLuint flow_depth_tex = 0;
    GLuint flow_depth_fbo = 0;
    uint32_t flow_depth_width = 0;
    uint32_t flow_depth_height = 0;
    uint32_t shadow_size = 0;  // 0: the scene has no map, so nothing is shadowed
    bool r32f_renderable = false;  // create() asks the driver once
    glm::mat4 shadow_light_view_proj{1.0f};
    GLuint caster_program = 0;

    // CSG subtrees, ray-marched: their own program, and every subtree's SDF
    // program end to end in two buffers, uploaded with the scene.
    GLuint csg_program = 0;
    GLuint ssbo_csg_nodes = 0;
    GLuint ssbo_csg_transforms = 0;
    std::vector<GpuCsgSubtree> csg_subtrees;  // parallel to RenderScene::csg_subtrees
    size_t max_storage_block_bytes = 0;       // GL_MAX_SHADER_STORAGE_BLOCK_SIZE

    // Scratch, reused every frame so a frame allocates nothing steady-state.
    std::vector<glm::mat4> instance_transforms;
    std::vector<uint32_t> instance_overrides;
    std::vector<float> instance_speeds;
    std::vector<uint32_t> batch_base;    // per mesh: where its instances start
    std::vector<uint32_t> batch_count;   // per mesh: how many
    std::vector<uint32_t> batch_static;  // per mesh: how many of those are statics, which come first
    std::vector<uint8_t> static_marched; // per static: 1 when a CSG subtree draws it this frame
    std::vector<GpuMaterial> gpu_materials;
    std::vector<GpuGroundPlane> gpu_planes;
    std::vector<glm::vec4> field_colours;

    uint32_t last_draw_calls = 0;
    GlRenderer::FlowQuality flow_quality;
    float last_flow_scale = 0.0f;
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

    struct OverlayUniforms {
        GLint view = -1, proj = -1, depth_bias = -1;
    } overlay;

    struct CasterUniforms {
        GLint instance_base = -1, light_view_proj = -1;
    } caster;

    struct CsgUniforms {
        GLint view_proj = -1, box_min = -1, box_max = -1, node_base = -1, node_count = -1, transform_base = -1;
        GLint gradient_step = -1, right = -1, up = -1, forward = -1, inv_tan_half_fov = -1, aspect = -1;
        GLint viewport = -1, near_plane = -1, far_plane = -1, march_max_steps = -1, march_epsilon = -1;
        GLint material = -1, speed = -1, sun_dir = -1, sun_color = -1, ambient = -1, sun_intensity = -1;
        GLint mode = -1, velocity_scale = -1, cam_pos = -1;
        GLint sky_zenith = -1, sky_horizon = -1, horizon_strength = -1, horizon_onset = -1;
    } csg;

    // kCommonSrc's shadow lookup, once per program that includes it.
    struct ShadowUniforms {
        GLint enabled = -1, light_view_proj = -1, size = -1, no_occluder = -1, bias = -1;
    } mesh_shadow, bg_shadow, csg_shadow;

    // kFlowSampleSrc's and kFlowSurfaceSrc's, in the mesh and CSG programs.
    struct FlowSurfaceUniforms {
        GLint on = -1, quantity = -1, box_min = -1, box_max = -1, origin = -1, inv_cell = -1, range_min = -1,
              inv_span = -1, constant = -1, constant_colour = -1, colour_map = -1;
    } mesh_flow, csg_flow;

    struct FlowLayerUniforms {
        GLint view_proj = -1, box_min = -1, box_max = -1, cam_pos = -1, right = -1, up = -1, forward = -1,
              inv_tan_half_fov = -1, aspect = -1, viewport = -1, near_plane = -1, far_plane = -1, proj22 = -1,
              proj32 = -1, step = -1, inv_step = -1, transfer_count = -1, iso_count = -1, sun_dir = -1,
              sun_color = -1, ambient = -1, sun_intensity = -1, params = -1, origin = -1, inv_cell = -1,
              skip = -1;
    } flow_layer;
    struct FlowBrickUniforms {
        GLint transfer_count = -1, iso_count = -1, params = -1;
    } flow_brick;

    ~Impl() {
        for (GpuMesh& m : meshes) {
            if (m.ebo != 0) gl.DeleteBuffers(1, &m.ebo);
            if (m.vbo_nrm != 0) gl.DeleteBuffers(1, &m.vbo_nrm);
            if (m.vbo_pos != 0) gl.DeleteBuffers(1, &m.vbo_pos);
            if (m.vao != 0) gl.DeleteVertexArrays(1, &m.vao);
        }
        if (shadow_fbo != 0) gl.DeleteFramebuffers(1, &shadow_fbo);
        if (shadow_work_tex != 0) gl.DeleteTextures(1, &shadow_work_tex);
        if (shadow_static_tex != 0) gl.DeleteTextures(1, &shadow_static_tex);
        if (flow_colour_tex != 0) gl.DeleteTextures(1, &flow_colour_tex);
        if (flow_s_tex != 0) gl.DeleteTextures(1, &flow_s_tex);
        if (flow_v_tex != 0) gl.DeleteTextures(1, &flow_v_tex);
        if (flow_depth_fbo != 0) gl.DeleteFramebuffers(1, &flow_depth_fbo);
        if (flow_depth_tex != 0) gl.DeleteTextures(1, &flow_depth_tex);
        if (flow_program != 0) gl.DeleteProgram(flow_program);
        if (flow_brick_program != 0) gl.DeleteProgram(flow_brick_program);
        if (flow_empty_tex != 0) gl.DeleteTextures(1, &flow_empty_tex);
        if (caster_program != 0) gl.DeleteProgram(caster_program);
        if (ssbo_csg_transforms != 0) gl.DeleteBuffers(1, &ssbo_csg_transforms);
        if (ssbo_csg_nodes != 0) gl.DeleteBuffers(1, &ssbo_csg_nodes);
        if (csg_program != 0) gl.DeleteProgram(csg_program);
        if (overlay_vbo != 0) gl.DeleteBuffers(1, &overlay_vbo);
        if (overlay_vao != 0) gl.DeleteVertexArrays(1, &overlay_vao);
        if (overlay_program != 0) gl.DeleteProgram(overlay_program);
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
        link(gl, {kGlslVersion, kCommonSrc, kVertexSrc},
             {kGlslVersion, kCommonSrc, kSurfaceSrc, kFlowSampleSrc, kFlowSurfaceSrc, kFragmentSrc}, "mesh");
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
    const Result<GLuint> overlay_program =
        link(gl, {kGlslVersion, kOverlayVertexSrc}, {kGlslVersion, kOverlayFragmentSrc}, "overlay");
    if (!overlay_program) return std::unexpected(overlay_program.error());
    impl->overlay_program = *overlay_program;
    const Result<GLuint> caster_program =
        link(gl, {kGlslVersion, kCasterVertexSrc}, {kGlslVersion, kCasterFragmentSrc}, "shadow caster");
    if (!caster_program) return std::unexpected(caster_program.error());
    impl->caster_program = *caster_program;
    const Result<GLuint> csg_program = link(gl, {kGlslVersion, kCsgVertexSrc},
                                            {kGlslVersion, kCommonSrc, kSurfaceSrc, kSdfSrc, kFlowSampleSrc,
                                             kFlowSurfaceSrc, kCsgFragmentSrc},
                                            "csg");
    if (!csg_program) return std::unexpected(csg_program.error());
    impl->csg_program = *csg_program;
    const Result<GLuint> flow_program =
        link(gl, {kGlslVersion, kCsgVertexSrc},
             {kGlslVersion, kFlowSampleSrc, kFlowParamsSrc, kFlowVolumeFragmentSrc}, "flow volume");
    if (!flow_program) return std::unexpected(flow_program.error());
    impl->flow_program = *flow_program;
    const Result<GLuint> flow_brick_program =
        link_compute(gl, {kGlslVersion, kFlowSampleSrc, kFlowParamsSrc, kFlowBrickSrc}, "flow bricks");
    if (!flow_brick_program) return std::unexpected(flow_brick_program.error());
    impl->flow_brick_program = *flow_brick_program;

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

    const GLuint o = impl->overlay_program;
    impl->overlay.view = gl.GetUniformLocation(o, "uView");
    impl->overlay.proj = gl.GetUniformLocation(o, "uProj");
    impl->overlay.depth_bias = gl.GetUniformLocation(o, "uDepthBias");
    // Position then colour, six floats a vertex. The array records the
    // buffer object, so a later BufferData that grows it needs no re-setup.
    gl.GenVertexArrays(1, &impl->overlay_vao);
    gl.GenBuffers(1, &impl->overlay_vbo);
    gl.BindVertexArray(impl->overlay_vao);
    gl.BindBuffer(GL_ARRAY_BUFFER, impl->overlay_vbo);
    constexpr GLsizei kOverlayStride = 6 * sizeof(float);
    gl.EnableVertexAttribArray(0);
    gl.VertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, kOverlayStride, nullptr);
    gl.EnableVertexAttribArray(1);
    gl.VertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, kOverlayStride, reinterpret_cast<const void*>(3 * sizeof(float)));
    gl.BindVertexArray(0);
    gl.BindBuffer(GL_ARRAY_BUFFER, 0);

    impl->caster.instance_base = gl.GetUniformLocation(impl->caster_program, "uInstanceBase");
    impl->caster.light_view_proj = gl.GetUniformLocation(impl->caster_program, "uLightViewProj");
    const auto shadow_uniforms = [&gl](GLuint program, Impl::ShadowUniforms& u) {
        u.enabled = gl.GetUniformLocation(program, "uShadowEnabled");
        u.light_view_proj = gl.GetUniformLocation(program, "uLightViewProj");
        u.size = gl.GetUniformLocation(program, "uShadowSize");
        u.no_occluder = gl.GetUniformLocation(program, "uNoOccluder");
        u.bias = gl.GetUniformLocation(program, "uShadowBias");
    };
    shadow_uniforms(impl->program, impl->mesh_shadow);
    shadow_uniforms(impl->background_program, impl->bg_shadow);
    shadow_uniforms(impl->csg_program, impl->csg_shadow);
    const auto flow_uniforms = [&gl](GLuint program, Impl::FlowSurfaceUniforms& u) {
        u.on = gl.GetUniformLocation(program, "uFlowColouring");
        u.quantity = gl.GetUniformLocation(program, "uFlowQuantity");
        u.box_min = gl.GetUniformLocation(program, "uFlowBoxMin");
        u.box_max = gl.GetUniformLocation(program, "uFlowBoxMax");
        u.origin = gl.GetUniformLocation(program, "uFlowOrigin");
        u.inv_cell = gl.GetUniformLocation(program, "uFlowInvCell");
        u.range_min = gl.GetUniformLocation(program, "uFlowRangeMin");
        u.inv_span = gl.GetUniformLocation(program, "uFlowInvSpan");
        u.constant = gl.GetUniformLocation(program, "uFlowConstant");
        u.constant_colour = gl.GetUniformLocation(program, "uFlowConstantColour");
        u.colour_map = gl.GetUniformLocation(program, "uFlowColourMap");
    };
    flow_uniforms(impl->program, impl->mesh_flow);
    flow_uniforms(impl->csg_program, impl->csg_flow);
    {
        Impl::FlowLayerUniforms& u = impl->flow_layer;
        const GLuint fp = impl->flow_program;
        u.view_proj = gl.GetUniformLocation(fp, "uViewProj");
        u.box_min = gl.GetUniformLocation(fp, "uBoxMin");
        u.box_max = gl.GetUniformLocation(fp, "uBoxMax");
        u.cam_pos = gl.GetUniformLocation(fp, "uCamPos");
        u.right = gl.GetUniformLocation(fp, "uRight");
        u.up = gl.GetUniformLocation(fp, "uUp");
        u.forward = gl.GetUniformLocation(fp, "uForward");
        u.inv_tan_half_fov = gl.GetUniformLocation(fp, "uInvTanHalfFov");
        u.aspect = gl.GetUniformLocation(fp, "uAspect");
        u.viewport = gl.GetUniformLocation(fp, "uViewport");
        u.near_plane = gl.GetUniformLocation(fp, "uNear");
        u.far_plane = gl.GetUniformLocation(fp, "uFar");
        u.proj22 = gl.GetUniformLocation(fp, "uProj22");
        u.proj32 = gl.GetUniformLocation(fp, "uProj32");
        u.step = gl.GetUniformLocation(fp, "uStep");
        u.inv_step = gl.GetUniformLocation(fp, "uInvStep");
        u.transfer_count = gl.GetUniformLocation(fp, "uTransferCount");
        u.iso_count = gl.GetUniformLocation(fp, "uIsoCount");
        u.sun_dir = gl.GetUniformLocation(fp, "uSunDir");
        u.sun_color = gl.GetUniformLocation(fp, "uSunColor");
        u.ambient = gl.GetUniformLocation(fp, "uAmbient");
        u.sun_intensity = gl.GetUniformLocation(fp, "uSunIntensity");
        u.params = gl.GetUniformLocation(fp, "uFlowParams");
        u.origin = gl.GetUniformLocation(fp, "uFlowOrigin");
        u.inv_cell = gl.GetUniformLocation(fp, "uFlowInvCell");
        u.skip = gl.GetUniformLocation(fp, "uSkip");
        Impl::FlowBrickUniforms& bu = impl->flow_brick;
        const GLuint bp = impl->flow_brick_program;
        bu.transfer_count = gl.GetUniformLocation(bp, "uTransferCount");
        bu.iso_count = gl.GetUniformLocation(bp, "uIsoCount");
        bu.params = gl.GetUniformLocation(bp, "uFlowParams");
    }
    gl.GenTextures(1, &impl->flow_empty_tex);
    gl.GenTextures(1, &impl->flow_v_tex);
    gl.GenTextures(1, &impl->flow_s_tex);
    gl.GenTextures(1, &impl->flow_colour_tex);
    gl.GenTextures(1, &impl->flow_depth_tex);
    gl.GenFramebuffers(1, &impl->flow_depth_fbo);
    {
        // Every colour map's stops, a row per ColourMapId, once.
        static_assert(static_cast<uint32_t>(render::ColourMapId::viridis) == 0u &&
                      static_cast<uint32_t>(render::ColourMapId::inferno) == 1u &&
                      static_cast<uint32_t>(render::ColourMapId::cool_warm) == 2u);
        static_assert(render::kColourMapEntries == 256 && sizeof(glm::vec3) == 3 * sizeof(float));
        std::vector<glm::vec3> rows;
        for (const render::ColourMapId id :
             {render::ColourMapId::viridis, render::ColourMapId::inferno, render::ColourMapId::cool_warm}) {
            const auto& table = render::colour_map_table(id);
            rows.insert(rows.end(), table.begin(), table.end());
        }
        gl.BindTexture(GL_TEXTURE_2D, impl->flow_colour_tex);
        gl.TexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
        gl.TexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
        gl.TexImage2D(GL_TEXTURE_2D, 0, GL_RGBA32F, static_cast<GLsizei>(render::kColourMapEntries), 3, 0, GL_RGB,
                      GL_FLOAT, rows.data());
        gl.BindTexture(GL_TEXTURE_2D, 0);
    }
    gl.GenTextures(1, &impl->shadow_static_tex);
    gl.GenTextures(1, &impl->shadow_work_tex);
    gl.GenFramebuffers(1, &impl->shadow_fbo);
    // The working shadow map stays attached to `shadow_fbo`; upload_scene()
    // only resizes it. Whether the driver renders to R32F is asked here,
    // once, so upload_scene() can refuse before it changes anything.
    {
        gl.BindTexture(GL_TEXTURE_2D, impl->shadow_work_tex);
        gl.TexImage2D(GL_TEXTURE_2D, 0, GL_R32F, 1, 1, 0, GL_RED, GL_FLOAT, nullptr);
        gl.TexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
        gl.TexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
        gl.BindTexture(GL_TEXTURE_2D, 0);
        GLint prev_draw = 0, prev_read = 0;
        gl.GetIntegerv(GL_DRAW_FRAMEBUFFER_BINDING, &prev_draw);
        gl.GetIntegerv(GL_READ_FRAMEBUFFER_BINDING, &prev_read);
        gl.BindFramebuffer(GL_FRAMEBUFFER, impl->shadow_fbo);
        gl.FramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, impl->shadow_work_tex, 0);
        impl->r32f_renderable = gl.CheckFramebufferStatus(GL_FRAMEBUFFER) == GL_FRAMEBUFFER_COMPLETE;
        gl.BindFramebuffer(GL_DRAW_FRAMEBUFFER, static_cast<GLuint>(prev_draw));
        gl.BindFramebuffer(GL_READ_FRAMEBUFFER, static_cast<GLuint>(prev_read));
    }

    const GLuint c = impl->csg_program;
    Impl::CsgUniforms& cu = impl->csg;
    cu.view_proj = gl.GetUniformLocation(c, "uViewProj");
    cu.box_min = gl.GetUniformLocation(c, "uBoxMin");
    cu.box_max = gl.GetUniformLocation(c, "uBoxMax");
    cu.node_base = gl.GetUniformLocation(c, "uNodeBase");
    cu.node_count = gl.GetUniformLocation(c, "uNodeCount");
    cu.transform_base = gl.GetUniformLocation(c, "uTransformBase");
    cu.gradient_step = gl.GetUniformLocation(c, "uGradientStep");
    cu.right = gl.GetUniformLocation(c, "uRight");
    cu.up = gl.GetUniformLocation(c, "uUp");
    cu.forward = gl.GetUniformLocation(c, "uForward");
    cu.inv_tan_half_fov = gl.GetUniformLocation(c, "uInvTanHalfFov");
    cu.aspect = gl.GetUniformLocation(c, "uAspect");
    cu.viewport = gl.GetUniformLocation(c, "uViewport");
    cu.near_plane = gl.GetUniformLocation(c, "uNear");
    cu.far_plane = gl.GetUniformLocation(c, "uFar");
    cu.march_max_steps = gl.GetUniformLocation(c, "uMarchMaxSteps");
    cu.march_epsilon = gl.GetUniformLocation(c, "uMarchEpsilon");
    cu.material = gl.GetUniformLocation(c, "uSubtreeMaterial");
    cu.speed = gl.GetUniformLocation(c, "uSubtreeSpeed");
    cu.sun_dir = gl.GetUniformLocation(c, "uSunDir");
    cu.sun_color = gl.GetUniformLocation(c, "uSunColor");
    cu.ambient = gl.GetUniformLocation(c, "uAmbient");
    cu.sun_intensity = gl.GetUniformLocation(c, "uSunIntensity");
    cu.mode = gl.GetUniformLocation(c, "uMode");
    cu.velocity_scale = gl.GetUniformLocation(c, "uVelocityScale");
    cu.cam_pos = gl.GetUniformLocation(c, "uCamPos");
    cu.sky_zenith = gl.GetUniformLocation(c, "uSkyZenith");
    cu.sky_horizon = gl.GetUniformLocation(c, "uSkyHorizon");
    cu.horizon_strength = gl.GetUniformLocation(c, "uHorizonStrength");
    cu.horizon_onset = gl.GetUniformLocation(c, "uHorizonOnset");
    gl.GenBuffers(1, &impl->ssbo_csg_nodes);
    gl.GenBuffers(1, &impl->ssbo_csg_transforms);
    // GL 4.3 guarantees 2^24 bytes; a driver reporting nothing usable gets that.
    GLint max_block = 0;
    gl.GetIntegerv(GL_MAX_SHADER_STORAGE_BLOCK_SIZE, &max_block);
    impl->max_storage_block_bytes = max_block > 0 ? static_cast<size_t>(max_block) : size_t{1} << 24u;

    return std::unique_ptr<GlRenderer>(new GlRenderer(std::move(impl)));
}

Result<void> GlRenderer::upload_scene(const render::RenderScene& scene) {
    Impl& s = *impl_;
    const GlApi& gl = s.gl;

    // --- validate everything first: a refusal changes nothing (L6) ------------
    // The scene uploaded before still draws after a refused one.
    for (size_t i = 0; i < scene.meshes.size(); ++i) {
        const render::MeshData& src = scene.meshes[i];
        if (src.positions.size() != src.normals.size()) {
            return std::unexpected(Error{Code::invalid_argument,
                                         "GlRenderer::upload_scene: mesh " + std::to_string(i) +
                                             " has " + std::to_string(src.positions.size()) +
                                             " positions and " + std::to_string(src.normals.size()) +
                                             " normals"});
        }
    }
    if (scene.static_shadow.has_value()) {
        // A map whose depth array is not size x size cannot be sampled (the
        // CPU would read past it). Refused, never drawn unshadowed in
        // silence. A size-0 map is a valid empty one: the CPU leaves every
        // pixel lit, and so does GL.
        const render::ShadowMap& map = *scene.static_shadow;
        if (map.depth.size() != static_cast<size_t>(map.size) * map.size) {
            return std::unexpected(Error{Code::invalid_argument,
                                         "GlRenderer::upload_scene: the static shadow map has " +
                                             std::to_string(map.depth.size()) + " texels for size " +
                                             std::to_string(map.size) + ", not size x size"});
        }
        // Refused, not degraded: dynamic shadows draw into R32F.
        if (map.size > 0u && !s.r32f_renderable) {
            return std::unexpected(Error{Code::unavailable,
                                         "GlRenderer::upload_scene: this driver cannot render to an R32F texture, "
                                         "which the shadows of dynamic bodies need"});
        }
    }

    // CSG subtrees' SDF programs, every one end to end. Each keeps only the
    // transforms its leaves use, renumbered from its own transform base: a
    // subtree's program carries the whole world's table (csg_subtree_program()),
    // and copying it per subtree would grow the buffer as subtrees x transforms.
    std::vector<SdfNode> nodes;
    std::vector<SdfTransform> transforms;
    std::vector<GpuCsgSubtree> subtrees;
    subtrees.reserve(scene.csg_subtrees.size());
    for (size_t i = 0; i < scene.csg_subtrees.size(); ++i) {
        const render::CsgSubtree& sub = scene.csg_subtrees[i];
        const std::string which = "GlRenderer::upload_scene: CSG subtree " + std::to_string(i);
        // Either would vanish from the frame in silence.
        if (sub.program.empty()) {
            return std::unexpected(Error{Code::invalid_argument, which + " has no program"});
        }
        if (sub.draw_item >= scene.statics.size()) {
            return std::unexpected(Error{Code::invalid_argument,
                                         which + " names draw item " + std::to_string(sub.draw_item) + ", past the " +
                                             std::to_string(scene.statics.size()) + " statics"});
        }
        // An invalid program would overrun the shader's fixed stack. validate()'s
        // own code comes through: capacity_exceeded past kMaxSdfDepth.
        if (const Result<uint32_t> valid = sub.program.validate(); !valid) {
            return std::unexpected(Error{valid.error().code, which + ": " + valid.error().context});
        }
        if (sub.program.nodes.size() > kMaxCsgSubtreeNodes) {
            return std::unexpected(Error{Code::capacity_exceeded,
                                         which + " has " + std::to_string(sub.program.nodes.size()) +
                                             " nodes; GL marches at most " + std::to_string(kMaxCsgSubtreeNodes) +
                                             " (GlRenderer::kMaxCsgSubtreeNodes); draw it with render::render()"});
        }
        const GpuCsgSubtree g{.node_base = static_cast<uint32_t>(nodes.size()),
                              .node_count = static_cast<uint32_t>(sub.program.nodes.size()),
                              .transform_base = static_cast<uint32_t>(transforms.size())};
        constexpr uint32_t kUnused = std::numeric_limits<uint32_t>::max();
        std::vector<uint32_t> renumbered(sub.program.transforms.size(), kUnused);
        for (SdfNode node : sub.program.nodes) {
            if (node.op == static_cast<uint32_t>(SdfOp::none)) {
                uint32_t& local = renumbered[node.transform];  // in range: validate() checked it
                if (local == kUnused) {
                    local = static_cast<uint32_t>(transforms.size()) - g.transform_base;
                    transforms.push_back(sub.program.transforms[node.transform]);
                }
                node.transform = local;
            }
            nodes.push_back(node);
        }
        subtrees.push_back(g);
    }
    if (nodes.empty()) nodes.emplace_back();  // a bound buffer always has storage
    if (transforms.empty()) transforms.emplace_back();
    const size_t node_bytes = nodes.size() * sizeof(SdfNode);
    const size_t transform_bytes = transforms.size() * sizeof(SdfTransform);
    if (node_bytes > s.max_storage_block_bytes || transform_bytes > s.max_storage_block_bytes) {
        return std::unexpected(Error{Code::capacity_exceeded,
                                     "GlRenderer::upload_scene: the CSG programs need " +
                                         std::to_string(std::max(node_bytes, transform_bytes)) +
                                         " bytes in one storage block; this driver's limit is " +
                                         std::to_string(s.max_storage_block_bytes) +
                                         " (GL_MAX_SHADER_STORAGE_BLOCK_SIZE)"});
    }

    // --- then upload -------------------------------------------------------------
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

    s.csg_subtrees = std::move(subtrees);
    gl.BindBuffer(GL_SHADER_STORAGE_BUFFER, s.ssbo_csg_nodes);
    gl.BufferData(GL_SHADER_STORAGE_BUFFER, static_cast<GLsizeiptr>(node_bytes), nodes.data(), GL_STATIC_DRAW);
    gl.BindBuffer(GL_SHADER_STORAGE_BUFFER, s.ssbo_csg_transforms);
    gl.BufferData(GL_SHADER_STORAGE_BUFFER, static_cast<GLsizeiptr>(transform_bytes), transforms.data(),
                  GL_STATIC_DRAW);
    gl.BindBuffer(GL_SHADER_STORAGE_BUFFER, 0);

    // The sun's shadow map, as the CPU built it: static shadows then come
    // from the same data on both paths. The working copy stays attached to
    // `shadow_fbo` (create()), for the frames that add dynamic casters.
    s.shadow_size = 0;
    if (scene.static_shadow.has_value() && scene.static_shadow->size > 0u) {
        const render::ShadowMap& map = *scene.static_shadow;
        const auto size = static_cast<GLsizei>(map.size);
        for (const GLuint tex : {s.shadow_static_tex, s.shadow_work_tex}) {
            gl.BindTexture(GL_TEXTURE_2D, tex);
            gl.TexImage2D(GL_TEXTURE_2D, 0, GL_R32F, size, size, 0, GL_RED, GL_FLOAT,
                          tex == s.shadow_static_tex ? map.depth.data() : nullptr);
            // One level, so the minifying filter must not ask for mipmaps.
            gl.TexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
            gl.TexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
            gl.TexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
            gl.TexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        }
        gl.BindTexture(GL_TEXTURE_2D, 0);
        s.shadow_size = map.size;
        s.shadow_light_view_proj = map.light_view_proj;
    }

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
    // L6: GL marches one flow layer per frame; a frame with fewer would be a
    // different picture under the same scene.
    if (scene.flow_layers.size() > 1u) {
        return std::unexpected(Error{Code::unavailable, "GlRenderer::draw: GL marches one flow layer per frame, and "
                                                        "this scene has " +
                                                            std::to_string(scene.flow_layers.size()) +
                                                            "; draw them with render::render()"});
    }
    // The flow inputs GL draws, validated before any pixel, as render() does.
    for (size_t i = 0; i < scene.flow_polylines.size(); ++i) {
        if (Result<void> valid = render::validate_flow_polyline(scene.flow_polylines[i]); !valid) {
            return std::unexpected(Error{valid.error().code, "GlRenderer::draw: flow polyline " + std::to_string(i) +
                                                                 ": " + valid.error().context});
        }
    }
    if (Result<void> valid = render::validate_flow_points(scene.flow_points); !valid) {
        return std::unexpected(Error{valid.error().code, "GlRenderer::draw: " + valid.error().context});
    }
    GLint max_3d = 0;
    gl.GetIntegerv(GL_MAX_3D_TEXTURE_SIZE, &max_3d);
    const auto fits_3d = [max_3d](const FlowVolumeHeader& h) {
        return std::max(h.nx, std::max(h.ny, h.nz)) <= static_cast<uint32_t>(std::max(max_3d, 0));
    };
    const auto too_big = [max_3d](const char* what, const FlowVolumeHeader& h) {
        return std::unexpected(Error{Code::unavailable, std::string("GlRenderer::draw: ") + what + " is " +
                                                            std::to_string(h.nx) + "x" + std::to_string(h.ny) + "x" +
                                                            std::to_string(h.nz) +
                                                            " cells, past this device's 3D texture size " +
                                                            std::to_string(max_3d)});
    };
    if (scene.flow_surface_colouring) {
        if (Result<void> valid = render::validate_flow_surface_colouring(*scene.flow_surface_colouring); !valid) {
            return std::unexpected(Error{valid.error().code, "GlRenderer::draw: " + valid.error().context});
        }
        const FlowVolumeHeader& h = scene.flow_surface_colouring->volume->header;
        if (!fits_3d(h)) return too_big("the surface colouring's volume", h);
    }
    if (!scene.flow_layers.empty()) {
        if (Result<void> valid = render::validate_flow_layer(scene.flow_layers[0]); !valid) {
            return std::unexpected(Error{valid.error().code, "GlRenderer::draw: flow layer: " + valid.error().context});
        }
        const FlowVolumeHeader& h = scene.flow_layers[0].volume->header;
        if (!fits_3d(h)) return too_big("the flow layer's volume", h);
    }
    if (s.meshes.size() != scene.meshes.size()) {
        return std::unexpected(Error{Code::invalid_argument,
                                     "GlRenderer::draw: this scene has " +
                                         std::to_string(scene.meshes.size()) +
                                         " meshes but " + std::to_string(s.meshes.size()) +
                                         " are uploaded -- call upload_scene() first"});
    }
    if (s.csg_subtrees.size() != scene.csg_subtrees.size()) {
        return std::unexpected(Error{Code::invalid_argument,
                                     "GlRenderer::draw: this scene has " + std::to_string(scene.csg_subtrees.size()) +
                                         " CSG subtrees but " + std::to_string(s.csg_subtrees.size()) +
                                         " are uploaded -- call upload_scene() first"});
    }
    // A subtree that is not the one uploaded would vanish or march another
    // program, so it is refused, as upload_scene() refuses one (L6).
    for (size_t i = 0; i < scene.csg_subtrees.size(); ++i) {
        const render::CsgSubtree& sub = scene.csg_subtrees[i];
        if (sub.draw_item >= scene.statics.size() || sub.program.nodes.size() != s.csg_subtrees[i].node_count) {
            return std::unexpected(Error{Code::invalid_argument,
                                         "GlRenderer::draw: CSG subtree " + std::to_string(i) +
                                             " is not the one uploaded -- call upload_scene() with this scene"});
        }
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
    // As raster_cpu decides: shaded and velocity frames march each CSG
    // subtree and leave its mesh out; wireframe draws the mesh (RS3).
    const bool march_csg = (shaded || options.mode == render::DrawMode::velocity) && !scene.csg_subtrees.empty();
    s.static_marched.assign(scene.statics.size(), 0u);
    if (march_csg) {
        for (const render::CsgSubtree& sub : scene.csg_subtrees) {
            s.static_marched[sub.draw_item] = 1u;  // in range: checked above
        }
    }

    // --- group by mesh_index. This IS the mechanism. -----------------------
    const size_t mesh_count = s.meshes.size();
    s.batch_count.assign(mesh_count, 0u);
    for (size_t i = 0; i < scene.statics.size(); ++i) {
        const render::DrawItem& d = scene.statics[i];
        if (d.mesh_index < mesh_count && s.static_marched[i] == 0u) ++s.batch_count[d.mesh_index];
    }
    s.batch_static.assign(s.batch_count.begin(), s.batch_count.end());
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
    for (size_t i = 0; i < scene.statics.size(); ++i) {
        if (s.static_marched[i] == 0u) place(scene.statics[i]);
    }
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
            const render::ShadedColor shade = render::shade_vertex_color(scene.materials[mi], scene.lighting, gp.normal);
            g.color = shade.combined;
            g.sun = shade.sun;
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

    // --- shadows -----------------------------------------------------------
    // As raster_cpu decides: shaded mode, shadows asked for, and a map. With
    // dynamic casters, copy the static map and draw them into the copy, so
    // the static casters' texels stay the CPU's own.
    const bool shadows = options.shadows && shaded && s.shadow_size > 0u;
    GLuint shadow_tex = s.shadow_static_tex;
    uint32_t dynamic_instances = 0;
    for (size_t i = 0; i < mesh_count; ++i) {
        dynamic_instances += s.batch_count[i] - s.batch_static[i];
    }
    if (shadows && dynamic_instances > 0u) {
        GLint prev_draw = 0, prev_read = 0;
        gl.GetIntegerv(GL_DRAW_FRAMEBUFFER_BINDING, &prev_draw);
        gl.GetIntegerv(GL_READ_FRAMEBUFFER_BINDING, &prev_read);
        const auto size = static_cast<GLsizei>(s.shadow_size);
        gl.CopyImageSubData(s.shadow_static_tex, GL_TEXTURE_2D, 0, 0, 0, 0, s.shadow_work_tex, GL_TEXTURE_2D, 0, 0, 0,
                            0, size, size, 1);
        gl.BindFramebuffer(GL_FRAMEBUFFER, s.shadow_fbo);
        gl.Viewport(0, 0, size, size);
        gl.Disable(GL_DEPTH_TEST);
        gl.DepthMask(GL_FALSE);
        gl.Disable(GL_CULL_FACE);
        gl.PolygonMode(GL_FRONT_AND_BACK, GL_FILL);
        gl.Enable(GL_BLEND);
        gl.BlendEquation(GL_MAX);
        gl.Enable(GL_DEPTH_CLAMP);
        gl.UseProgram(s.caster_program);
        gl.UniformMatrix4fv(s.caster.light_view_proj, 1, GL_FALSE, glm::value_ptr(s.shadow_light_view_proj));
        for (size_t i = 0; i < mesh_count; ++i) {
            const uint32_t dynamic = s.batch_count[i] - s.batch_static[i];
            if (dynamic == 0u || s.meshes[i].index_count == 0u) continue;
            gl.BindVertexArray(s.meshes[i].vao);
            gl.Uniform1ui(s.caster.instance_base, s.batch_base[i] + s.batch_static[i]);
            // Every triangle casts, whatever its submesh's material.
            gl.DrawElementsInstanced(GL_TRIANGLES, static_cast<GLsizei>(s.meshes[i].index_count), GL_UNSIGNED_INT,
                                     nullptr, static_cast<GLsizei>(dynamic));
        }
        gl.Disable(GL_DEPTH_CLAMP);
        gl.BlendEquation(GL_FUNC_ADD);
        gl.Disable(GL_BLEND);
        gl.BindFramebuffer(GL_DRAW_FRAMEBUFFER, static_cast<GLuint>(prev_draw));
        gl.BindFramebuffer(GL_READ_FRAMEBUFFER, static_cast<GLuint>(prev_read));
        shadow_tex = s.shadow_work_tex;
    }
    gl.ActiveTexture(GL_TEXTURE0);
    gl.BindTexture(GL_TEXTURE_2D, shadows ? shadow_tex : 0u);
    // Surface colouring (airflow plan step 7b): the volume's halves as they
    // are, and the colour maps' table. Integer and unmipmapped textures need
    // NEAREST to be complete; the shader reads them by texelFetch.
    render::PreparedTransfer flow_colours;
    if (scene.flow_surface_colouring) {
        const render::FlowSurfaceColouring& fc = *scene.flow_surface_colouring;
        const FlowVolumeHeader& h = fc.volume->header;
        const auto upload_halves = [&gl, &h](GLuint tex, const std::vector<uint16_t>& halves) {
            gl.BindTexture(GL_TEXTURE_3D, tex);
            gl.TexParameteri(GL_TEXTURE_3D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
            gl.TexParameteri(GL_TEXTURE_3D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
            gl.TexImage3D(GL_TEXTURE_3D, 0, GL_RGBA16UI, static_cast<GLsizei>(h.nx), static_cast<GLsizei>(h.ny),
                          static_cast<GLsizei>(h.nz), 0, GL_RGBA_INTEGER, GL_UNSIGNED_SHORT, halves.data());
        };
        gl.ActiveTexture(GL_TEXTURE1);
        upload_halves(s.flow_v_tex, fc.volume->v);
        gl.ActiveTexture(GL_TEXTURE2);
        upload_halves(s.flow_s_tex, fc.volume->s);
        gl.ActiveTexture(GL_TEXTURE3);
        gl.BindTexture(GL_TEXTURE_2D, s.flow_colour_tex);
        gl.ActiveTexture(GL_TEXTURE0);
        flow_colours = render::prepare_transfer(fc.colours, h);
    }
    const auto set_flow_uniforms = [&](const Impl::FlowSurfaceUniforms& u) {
        gl.Uniform1ui(u.on, scene.flow_surface_colouring ? 1u : 0u);
        if (!scene.flow_surface_colouring) return;
        const FlowVolumeHeader& h = scene.flow_surface_colouring->volume->header;
        // raster_cpu's box and sampler, in the same float arithmetic.
        const glm::vec3 extent(static_cast<float>(h.nx), static_cast<float>(h.ny), static_cast<float>(h.nz));
        const glm::vec3 box_max = h.origin + extent * h.cell;
        gl.Uniform1ui(u.quantity, static_cast<GLuint>(flow_colours.tf.quantity));
        gl.Uniform1ui(u.colour_map, static_cast<GLuint>(flow_colours.tf.colours));
        gl.Uniform3fv(u.box_min, 1, glm::value_ptr(h.origin));
        gl.Uniform3fv(u.box_max, 1, glm::value_ptr(box_max));
        gl.Uniform3fv(u.origin, 1, glm::value_ptr(h.origin));
        gl.Uniform1f(u.inv_cell, 1.0f / h.cell);
        gl.Uniform1f(u.range_min, flow_colours.range.min);
        gl.Uniform1f(u.inv_span, flow_colours.inv_span);
        gl.Uniform1ui(u.constant, flow_colours.tf.constant_colour ? 1u : 0u);
        const glm::vec3 constant = flow_colours.tf.constant_colour.value_or(glm::vec3(0.0f));
        gl.Uniform3fv(u.constant_colour, 1, glm::value_ptr(constant));
    };
    const auto set_shadow_uniforms = [&](const Impl::ShadowUniforms& u) {
        gl.Uniform1ui(u.enabled, shadows ? 1u : 0u);
        gl.UniformMatrix4fv(u.light_view_proj, 1, GL_FALSE, glm::value_ptr(s.shadow_light_view_proj));
        gl.Uniform1ui(u.size, s.shadow_size);
        gl.Uniform1f(u.no_occluder, render::kNoOccluder);
        gl.Uniform1f(u.bias, render::kShadowDepthBias);
    };

    gl.Viewport(0, 0, static_cast<GLsizei>(width), static_cast<GLsizei>(height));
    gl.DepthMask(GL_TRUE);
    gl.Clear(GL_DEPTH_BUFFER_BIT);
    gl.PolygonMode(GL_FRONT_AND_BACK, GL_FILL);

    // Every pass below writes without blending and clips in depth. Set here,
    // not assumed: the caller's state, or a dynamic-caster pass, may differ.
    gl.Disable(GL_BLEND);
    gl.Disable(GL_DEPTH_CLAMP);

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
        set_shadow_uniforms(s.bg_shadow);
        gl.BindVertexArray(s.empty_vao);
        gl.DrawArrays(GL_TRIANGLES, 0, 3);
        gl.DepthMask(GL_TRUE);
    }

    // The opaque passes: meshes, CSG subtrees and field layers. A flow layer
    // marches to their depth, so it gets them once more, depth only, first.
    uint32_t draw_calls = 0;
    const auto draw_opaque = [&]() {
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
        set_shadow_uniforms(s.mesh_shadow);
        set_flow_uniforms(s.mesh_flow);
        gl.Uniform3fv(s.u_cam_pos, 1, glm::value_ptr(camera.position));
        gl.Uniform3fv(s.u_sky_zenith, 1, glm::value_ptr(light.sky_zenith));
        gl.Uniform3fv(s.u_sky_horizon, 1, glm::value_ptr(light.sky_horizon));
        gl.Uniform1f(s.u_horizon_strength, horizon_strength);
        gl.Uniform1f(s.u_horizon_onset, options.horizon_blend_onset);

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

        // --- CSG subtrees, ray-marched -------------------------------------------
        // After every mesh, as raster_cpu draws them, depth-tested against them
        // with the depth each fragment writes. The box's back faces are the
        // proxy, and depth clamp keeps the parts beyond the far plane, so every
        // pixel the box covers runs one march.
        if (march_csg && !scene.materials.empty()) {
            const Impl::CsgUniforms& cu = s.csg;
            const glm::vec3 right = camera.orientation * glm::vec3(1.0f, 0.0f, 0.0f);
            const glm::vec3 up = camera.orientation * glm::vec3(0.0f, 1.0f, 0.0f);
            const glm::vec3 forward = camera.orientation * glm::vec3(0.0f, 0.0f, -1.0f);
            const auto inv_tan_half_fov =
                static_cast<float>(1.0 / render::tan32(static_cast<double>(camera.fov_y_radians) * 0.5));

            gl.Enable(GL_DEPTH_TEST);
            gl.DepthFunc(GL_LESS);
            gl.DepthMask(GL_TRUE);
            gl.Enable(GL_CULL_FACE);
            gl.CullFace(GL_FRONT);
            gl.FrontFace(GL_CCW);
            gl.Enable(GL_DEPTH_CLAMP);
            gl.UseProgram(s.csg_program);
            gl.BindBufferBase(GL_SHADER_STORAGE_BUFFER, 6, s.ssbo_csg_nodes);
            gl.BindBufferBase(GL_SHADER_STORAGE_BUFFER, 7, s.ssbo_csg_transforms);
            gl.UniformMatrix4fv(cu.view_proj, 1, GL_FALSE, glm::value_ptr(view_proj));
            gl.Uniform1f(cu.gradient_step, kSdfGradientStep);
            gl.Uniform3fv(cu.right, 1, glm::value_ptr(right));
            gl.Uniform3fv(cu.up, 1, glm::value_ptr(up));
            gl.Uniform3fv(cu.forward, 1, glm::value_ptr(forward));
            gl.Uniform1f(cu.inv_tan_half_fov, inv_tan_half_fov);
            gl.Uniform1f(cu.aspect, aspect);
            gl.Uniform2f(cu.viewport, static_cast<float>(width), static_cast<float>(height));
            gl.Uniform1f(cu.near_plane, camera.near_plane);
            gl.Uniform1f(cu.far_plane, camera.far_plane);
            gl.Uniform1ui(cu.march_max_steps, render::kRaymarchMaxSteps);
            gl.Uniform1f(cu.march_epsilon, render::kRaymarchSurfaceEpsilon);
            gl.Uniform3fv(cu.sun_dir, 1, glm::value_ptr(light.sun_direction));
            gl.Uniform3fv(cu.sun_color, 1, glm::value_ptr(light.sun_color));
            gl.Uniform3fv(cu.ambient, 1, glm::value_ptr(light.ambient_color));
            gl.Uniform1f(cu.sun_intensity, light.sun_intensity);
            gl.Uniform1ui(cu.mode, static_cast<GLuint>(options.mode));
            gl.Uniform1f(cu.velocity_scale, options.velocity_scale_mps);
            gl.Uniform3fv(cu.cam_pos, 1, glm::value_ptr(camera.position));
            gl.Uniform3fv(cu.sky_zenith, 1, glm::value_ptr(light.sky_zenith));
            gl.Uniform3fv(cu.sky_horizon, 1, glm::value_ptr(light.sky_horizon));
            gl.Uniform1f(cu.horizon_strength, horizon_strength);
            gl.Uniform1f(cu.horizon_onset, options.horizon_blend_onset);
            set_shadow_uniforms(s.csg_shadow);
            set_flow_uniforms(s.csg_flow);
            gl.BindVertexArray(s.empty_vao);
            for (size_t i = 0; i < scene.csg_subtrees.size(); ++i) {
                const render::CsgSubtree& sub = scene.csg_subtrees[i];
                const GpuCsgSubtree& g = s.csg_subtrees[i];
                const render::DrawItem& item = scene.statics[sub.draw_item];
                // The draw item's material, as draw_csg_subtrees() resolves it.
                const uint32_t wanted = item.material_override != render::kNoMaterial ? item.material_override : 0u;
                const uint32_t material = wanted < scene.materials.size() ? wanted : 0u;
                gl.Uniform3fv(cu.box_min, 1, glm::value_ptr(sub.bounds.min));
                gl.Uniform3fv(cu.box_max, 1, glm::value_ptr(sub.bounds.max));
                gl.Uniform1ui(cu.node_base, g.node_base);
                gl.Uniform1ui(cu.node_count, g.node_count);
                gl.Uniform1ui(cu.transform_base, g.transform_base);
                gl.Uniform1ui(cu.material, material);
                gl.Uniform1f(cu.speed, item.speed_mps);
                gl.DrawArrays(GL_TRIANGLES, 0, 36);
                ++draw_calls;
            }
            gl.Disable(GL_DEPTH_CLAMP);
            gl.CullFace(GL_BACK);
        }

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

    };
    if (!scene.flow_layers.empty()) {
        // The caller's depth may be a renderbuffer, which no shader can read
        // (airflow plan §7.1), so the march reads this copy.
        if (s.flow_depth_width != width || s.flow_depth_height != height) {
            gl.BindTexture(GL_TEXTURE_2D, s.flow_depth_tex);
            gl.TexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
            gl.TexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
            gl.TexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT32F, static_cast<GLsizei>(width),
                          static_cast<GLsizei>(height), 0, GL_DEPTH_COMPONENT, GL_FLOAT, nullptr);
            gl.BindTexture(GL_TEXTURE_2D, 0);
            s.flow_depth_width = width;
            s.flow_depth_height = height;
        }
        GLint prev_draw = 0, prev_read = 0;
        gl.GetIntegerv(GL_DRAW_FRAMEBUFFER_BINDING, &prev_draw);
        gl.GetIntegerv(GL_READ_FRAMEBUFFER_BINDING, &prev_read);
        gl.BindFramebuffer(GL_FRAMEBUFFER, s.flow_depth_fbo);
        gl.FramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, s.flow_depth_tex, 0);
        gl.DrawBuffer(GL_NONE);
        gl.ReadBuffer(GL_NONE);
        if (gl.CheckFramebufferStatus(GL_FRAMEBUFFER) == GL_FRAMEBUFFER_COMPLETE) {
            const GLfloat far_depth = 1.0f;
            gl.DepthMask(GL_TRUE);
            gl.ClearBufferfv(GL_DEPTH, 0, &far_depth);
            draw_opaque();
        }
        gl.BindFramebuffer(GL_DRAW_FRAMEBUFFER, static_cast<GLuint>(prev_draw));
        gl.BindFramebuffer(GL_READ_FRAMEBUFFER, static_cast<GLuint>(prev_read));
        draw_calls = 0;  // the frame's own passes count; the copy does not
    }
    draw_opaque();

    // --- flow layer (render/flow_render.hpp) ----------------------------------
    // After the opaque passes, as raster_cpu's draw_flow_layers(). Depth test
    // ALWAYS: the march clips itself to the opaque depth, and every pixel's
    // depth is written, the iso's where one is opaque and the old one
    // elsewhere. Blending makes C + T * behind.
    if (!scene.flow_layers.empty()) {
        const render::FlowLayer& layer = scene.flow_layers[0];
        const render::PreparedFlowLayer prepared = render::prepare_flow_layer(layer, &scene.lighting);
        const FlowVolumeHeader& h = layer.volume->header;
        const auto upload_halves = [&gl, &h](GLuint tex, const std::vector<uint16_t>& halves) {
            gl.BindTexture(GL_TEXTURE_3D, tex);
            gl.TexParameteri(GL_TEXTURE_3D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
            gl.TexParameteri(GL_TEXTURE_3D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
            gl.TexImage3D(GL_TEXTURE_3D, 0, GL_RGBA16UI, static_cast<GLsizei>(h.nx), static_cast<GLsizei>(h.ny),
                          static_cast<GLsizei>(h.nz), 0, GL_RGBA_INTEGER, GL_UNSIGNED_SHORT, halves.data());
        };
        gl.ActiveTexture(GL_TEXTURE1);
        upload_halves(s.flow_v_tex, layer.volume->v);
        gl.ActiveTexture(GL_TEXTURE2);
        upload_halves(s.flow_s_tex, layer.volume->s);
        gl.ActiveTexture(GL_TEXTURE3);
        gl.BindTexture(GL_TEXTURE_2D, s.flow_colour_tex);
        gl.ActiveTexture(GL_TEXTURE4);
        gl.BindTexture(GL_TEXTURE_2D, s.flow_depth_tex);
        gl.ActiveTexture(GL_TEXTURE0);

        // The prepared transfers and isos as kFlowVolumeFragmentSrc reads them.
        std::array<float, 320> params{};
        const auto pack_transfer = [&params](size_t b, const render::PreparedTransfer& t) {
            params[b + 0] = static_cast<float>(static_cast<uint32_t>(t.tf.quantity));
            params[b + 1] = static_cast<float>(static_cast<uint32_t>(t.tf.colours));
            params[b + 2] = t.tf.constant_colour ? 1.0f : 0.0f;
            params[b + 3] = t.range.min;
            params[b + 4] = t.inv_span;
            const size_t n = std::min(t.tf.extinction.size(), render::kMaxExtinctionPoints);
            params[b + 5] = static_cast<float>(n);
            if (t.tf.constant_colour) {
                params[b + 8] = t.tf.constant_colour->r;
                params[b + 9] = t.tf.constant_colour->g;
                params[b + 10] = t.tf.constant_colour->b;
            }
            for (size_t i = 0; i < n; ++i) {
                params[b + 12 + 2 * i] = t.tf.extinction[i].value;
                params[b + 13 + 2 * i] = t.tf.extinction[i].sigma;
            }
            for (size_t i = 0; i < t.extinction_inv_width.size(); ++i) {
                params[b + 28 + i] = t.extinction_inv_width[i];
            }
        };
        static_assert(render::kMaxFlowTransfers == 4 && render::kMaxIsoSurfaces == 4 &&
                      render::kMaxExtinctionPoints == 8);
        const size_t n_transfers = std::min(prepared.transfers.size(), render::kMaxFlowTransfers);
        const size_t n_isos = std::min(prepared.isos.size(), render::kMaxIsoSurfaces);
        for (size_t k = 0; k < n_transfers; ++k) {
            pack_transfer(36 * k, prepared.transfers[k]);
        }
        for (size_t k = 0; k < n_isos; ++k) {
            const render::PreparedIso& iso = prepared.isos[k];
            const size_t b = 144 + 44 * k;
            params[b + 0] = static_cast<float>(static_cast<uint32_t>(iso.quantity));
            params[b + 1] = iso.value;
            params[b + 2] = iso.opacity;
            params[b + 3] = iso.coloured_by ? 1.0f : 0.0f;
            params[b + 4] = iso.colour.r;
            params[b + 5] = iso.colour.g;
            params[b + 6] = iso.colour.b;
            if (iso.coloured_by) pack_transfer(b + 8, iso.colour_by);
        }

        // Empty space (7e): which 8^3 bricks no step can change, for this
        // layer's transfers and isos, into one R8UI texel each.
        const bool skip = s.flow_quality.skip_empty_space;
        if (skip) {
            const auto bricks = [](uint32_t cells) { return static_cast<GLsizei>((cells + 7u) / 8u); };
            const GLsizei bx = bricks(h.nx), by = bricks(h.ny), bz = bricks(h.nz);
            gl.ActiveTexture(GL_TEXTURE5);
            gl.BindTexture(GL_TEXTURE_3D, s.flow_empty_tex);
            gl.TexParameteri(GL_TEXTURE_3D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
            gl.TexParameteri(GL_TEXTURE_3D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
            gl.TexImage3D(GL_TEXTURE_3D, 0, GL_R8UI, bx, by, bz, 0, GL_RED_INTEGER, GL_UNSIGNED_BYTE, nullptr);
            gl.ActiveTexture(GL_TEXTURE0);
            gl.UseProgram(s.flow_brick_program);
            gl.Uniform1ui(s.flow_brick.transfer_count, static_cast<GLuint>(n_transfers));
            gl.Uniform1ui(s.flow_brick.iso_count, static_cast<GLuint>(n_isos));
            gl.Uniform4fv(s.flow_brick.params, static_cast<GLsizei>(params.size() / 4u), params.data());
            gl.BindImageTexture(0, s.flow_empty_tex, 0, GL_TRUE, 0, GL_WRITE_ONLY, GL_R8UI);
            const auto groups = [](GLsizei b) { return static_cast<GLuint>((b + 3) / 4); };
            gl.DispatchCompute(groups(bx), groups(by), groups(bz));
            gl.MemoryBarrier(GL_TEXTURE_FETCH_BARRIER_BIT);
        }

        const glm::vec3 right = camera.orientation * glm::vec3(1.0f, 0.0f, 0.0f);
        const glm::vec3 up = camera.orientation * glm::vec3(0.0f, 1.0f, 0.0f);
        const glm::vec3 forward = camera.orientation * glm::vec3(0.0f, 0.0f, -1.0f);
        const auto inv_tan_half_fov =
            static_cast<float>(1.0 / render::tan32(static_cast<double>(camera.fov_y_radians) * 0.5));
        const Impl::FlowLayerUniforms& u = s.flow_layer;
        gl.Enable(GL_DEPTH_TEST);
        gl.DepthFunc(GL_ALWAYS);
        gl.DepthMask(GL_TRUE);
        gl.Enable(GL_CULL_FACE);
        gl.CullFace(GL_FRONT);
        gl.FrontFace(GL_CCW);
        gl.Enable(GL_DEPTH_CLAMP);
        gl.Enable(GL_BLEND);
        gl.BlendEquation(GL_FUNC_ADD);
        gl.BlendFunc(GL_ONE, GL_SRC_ALPHA);
        gl.UseProgram(s.flow_program);
        gl.UniformMatrix4fv(u.view_proj, 1, GL_FALSE, glm::value_ptr(view_proj));
        gl.Uniform3fv(u.box_min, 1, glm::value_ptr(prepared.box.min));
        gl.Uniform3fv(u.box_max, 1, glm::value_ptr(prepared.box.max));
        gl.Uniform3fv(u.cam_pos, 1, glm::value_ptr(camera.position));
        gl.Uniform3fv(u.right, 1, glm::value_ptr(right));
        gl.Uniform3fv(u.up, 1, glm::value_ptr(up));
        gl.Uniform3fv(u.forward, 1, glm::value_ptr(forward));
        gl.Uniform1f(u.inv_tan_half_fov, inv_tan_half_fov);
        gl.Uniform1f(u.aspect, aspect);
        gl.Uniform2f(u.viewport, static_cast<float>(width), static_cast<float>(height));
        gl.Uniform1f(u.near_plane, camera.near_plane);
        gl.Uniform1f(u.far_plane, camera.far_plane);
        gl.Uniform1f(u.proj22, proj[2][2]);
        gl.Uniform1f(u.proj32, proj[3][2]);
        gl.Uniform1f(u.step, prepared.step);
        gl.Uniform1f(u.inv_step, prepared.inv_step);
        gl.Uniform1ui(u.transfer_count, static_cast<GLuint>(n_transfers));
        gl.Uniform1ui(u.iso_count, static_cast<GLuint>(n_isos));
        gl.Uniform3fv(u.sun_dir, 1, glm::value_ptr(light.sun_direction));
        gl.Uniform3fv(u.sun_color, 1, glm::value_ptr(light.sun_color));
        gl.Uniform3fv(u.ambient, 1, glm::value_ptr(light.ambient_color));
        gl.Uniform1f(u.sun_intensity, light.sun_intensity);
        gl.Uniform4fv(u.params, static_cast<GLsizei>(params.size() / 4u), params.data());
        gl.Uniform3fv(u.origin, 1, glm::value_ptr(h.origin));
        gl.Uniform1f(u.inv_cell, prepared.sampler.inv_cell);
        gl.Uniform1ui(u.skip, skip ? 1u : 0u);
        gl.BindVertexArray(s.empty_vao);
        gl.DrawArrays(GL_TRIANGLES, 0, 36);
        gl.BindVertexArray(0);
        gl.Disable(GL_BLEND);
        gl.BlendFunc(GL_ONE, GL_ZERO);
        gl.Disable(GL_DEPTH_CLAMP);
        gl.CullFace(GL_BACK);
        gl.DepthFunc(GL_LESS);
    }

    // --- streamlines and tracers (render/flow_lines.hpp) -----------------------
    // As raster_cpu's draw_flow_lines(): after the field layers, before the
    // overlays, depth-tested and written, unlit, with no depth bias. Each
    // segment's two vertices carry the mean of its ends' colours, so GL draws
    // the CPU's flat segment colour; tracers are GL_POINTS of size_px.
    // draw() validated them before any pixel.
    if (!scene.flow_polylines.empty() || !scene.flow_points.positions.empty()) {
        std::vector<float>& v = s.overlay_vertices;
        v.clear();
        // Colours go up as raster_cpu's own bytes (to_byte, then /255), which
        // GL's float-to-unorm8 conversion returns exactly. An arbitrary float
        // near a rounding boundary can land a level apart on the two paths.
        const auto push = [&v](const glm::vec3& p, const glm::vec3& c) {
            v.insert(v.end(), {p.x, p.y, p.z, static_cast<float>(render::to_byte(c.r)) / 255.0f,
                               static_cast<float>(render::to_byte(c.g)) / 255.0f,
                               static_cast<float>(render::to_byte(c.b)) / 255.0f});
        };
        for (const render::FlowPolyline& line : scene.flow_polylines) {
            for (size_t i = 0; i + 1u < line.points.size(); ++i) {
                const glm::vec3 c = 0.5f * (line.colours[i] + line.colours[i + 1u]);
                push(line.points[i], c);
                push(line.points[i + 1u], c);
            }
        }
        const auto line_vertices = static_cast<GLsizei>(v.size() / 6u);
        for (size_t i = 0; i < scene.flow_points.positions.size(); ++i) {
            push(scene.flow_points.positions[i], scene.flow_points.colours[i]);
        }
        const auto point_vertices = static_cast<GLsizei>(v.size() / 6u) - line_vertices;
        gl.BindBuffer(GL_ARRAY_BUFFER, s.overlay_vbo);
        if (v.size() > s.overlay_capacity) {
            gl.BufferData(GL_ARRAY_BUFFER, static_cast<GLsizeiptr>(v.size() * sizeof(float)), v.data(),
                          GL_STREAM_DRAW);
            s.overlay_capacity = v.size();
        } else {
            gl.BufferSubData(GL_ARRAY_BUFFER, 0, static_cast<GLsizeiptr>(v.size() * sizeof(float)), v.data());
        }
        gl.BindBuffer(GL_ARRAY_BUFFER, 0);
        gl.Enable(GL_DEPTH_TEST);
        gl.DepthFunc(GL_LESS);
        gl.DepthMask(GL_TRUE);
        gl.Disable(GL_CULL_FACE);
        gl.UseProgram(s.overlay_program);
        gl.UniformMatrix4fv(s.overlay.view, 1, GL_FALSE, glm::value_ptr(view));
        gl.UniformMatrix4fv(s.overlay.proj, 1, GL_FALSE, glm::value_ptr(proj));
        gl.Uniform1f(s.overlay.depth_bias, 0.0f);
        gl.BindVertexArray(s.overlay_vao);
        if (line_vertices > 0) gl.DrawArrays(GL_LINES, 0, line_vertices);
        if (point_vertices > 0) {
            gl.PointSize(static_cast<GLfloat>(scene.flow_points.size_px));
            gl.DrawArrays(GL_POINTS, line_vertices, point_vertices);
            gl.PointSize(1.0f);
        }
        gl.BindVertexArray(0);
    }

    // --- overlays ------------------------------------------------------------
    // The CPU's own lists, in its order: lines, then triangles. Depth-tested
    // and written, strictly nearer wins (setPixelIfCloser's rule), and nothing
    // is culled, as on the CPU. In every raster mode, filled.
    if (options.overlays) {
        const render::OverlayGeometry overlays = render::overlay_geometry(scene, options);
        std::vector<float>& v = s.overlay_vertices;
        v.clear();
        const auto push = [&v](const glm::dvec3& p, const std::array<uint8_t, 3>& rgb) {
            v.insert(v.end(), {static_cast<float>(p.x), static_cast<float>(p.y), static_cast<float>(p.z),
                               static_cast<float>(rgb[0]) / 255.0f, static_cast<float>(rgb[1]) / 255.0f,
                               static_cast<float>(rgb[2]) / 255.0f});
        };
        for (const render::OverlayLine& line : overlays.lines) {
            push(line.a, line.rgb);
            push(line.b, line.rgb);
        }
        for (const render::OverlayTriangle& tri : overlays.triangles) {
            push(tri.a, tri.rgb);
            push(tri.b, tri.rgb);
            push(tri.c, tri.rgb);
        }
        if (!v.empty()) {
            gl.BindBuffer(GL_ARRAY_BUFFER, s.overlay_vbo);
            if (v.size() > s.overlay_capacity) {
                gl.BufferData(GL_ARRAY_BUFFER, static_cast<GLsizeiptr>(v.size() * sizeof(float)), v.data(),
                              GL_STREAM_DRAW);
                s.overlay_capacity = v.size();
            } else {
                gl.BufferSubData(GL_ARRAY_BUFFER, 0, static_cast<GLsizeiptr>(v.size() * sizeof(float)), v.data());
            }
            gl.BindBuffer(GL_ARRAY_BUFFER, 0);

            gl.Enable(GL_DEPTH_TEST);
            gl.DepthFunc(GL_LESS);
            gl.DepthMask(GL_TRUE);
            gl.Disable(GL_CULL_FACE);
            gl.PolygonMode(GL_FRONT_AND_BACK, GL_FILL);
            gl.UseProgram(s.overlay_program);
            gl.UniformMatrix4fv(s.overlay.view, 1, GL_FALSE, glm::value_ptr(view));
            gl.UniformMatrix4fv(s.overlay.proj, 1, GL_FALSE, glm::value_ptr(proj));
            gl.Uniform1f(s.overlay.depth_bias, static_cast<float>(render::kOverlayDepthBias));
            gl.BindVertexArray(s.overlay_vao);
            const auto line_vertices = static_cast<GLsizei>(overlays.lines.size() * 2u);
            const auto triangle_vertices = static_cast<GLsizei>(overlays.triangles.size() * 3u);
            if (line_vertices > 0) gl.DrawArrays(GL_LINES, 0, line_vertices);
            if (triangle_vertices > 0) gl.DrawArrays(GL_TRIANGLES, line_vertices, triangle_vertices);
        }
    }
    gl.BindVertexArray(0);
    gl.UseProgram(0);

    s.last_draw_calls = draw_calls;
    s.last_flow_scale = scene.flow_layers.empty() ? 0.0f : 1.0f;
    s.last_instances = total_instances;
    return {};
}

std::vector<std::string_view> GlRenderer::unhonoured(const render::RenderOptions& options) {
    // draw() refuses raymarch; no other option applies to a raymarch frame.
    if (options.mode == render::DrawMode::raymarch) {
        return {"mode"};
    }
    // GL draws everything else raster_cpu draws: shadows and overlays too.
    return {};
}

uint32_t GlRenderer::last_draw_calls() const noexcept { return impl_->last_draw_calls; }
uint32_t GlRenderer::last_instances() const noexcept { return impl_->last_instances; }
void GlRenderer::set_flow_quality(const FlowQuality& quality) noexcept { impl_->flow_quality = quality; }
GlRenderer::FlowQuality GlRenderer::flow_quality() const noexcept { return impl_->flow_quality; }
float GlRenderer::last_flow_scale() const noexcept { return impl_->last_flow_scale; }
const std::string& GlRenderer::renderer_name() const noexcept { return impl_->renderer_name; }
const std::string& GlRenderer::version_string() const noexcept { return impl_->version_string; }

}  // namespace spade::render_gl
