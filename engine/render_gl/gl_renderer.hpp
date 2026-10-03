// GlRenderer -- v2's GPU render backend. The thing v2 has never had.
//
// ⭐⭐⭐ WHY THIS EXISTS, IN ONE SENTENCE: V1'S PER-FRAME CPU COST SCALES WITH
// THE NUMBER OF MESHES AND V2'S SCALES WITH THE NUMBER OF PIXELS. Measured on
// one box, 2026-09-18: v1's GL path renders a near-empty scene at ~950 fps at
// 1280x720 while v2's CPU rasteriser renders a ground plane at 11.5 fps at
// 900x560 -- 1.83x fewer pixels and ~82x slower, about 150x per pixel. The
// user's recollection of v1 at "30-60 fps" was conservative.
//
// THIS PORTS THE MECHANISM, NOT THE CODE. Four things, each of which is the
// reason for the margin, and each read out of v1's Engine.cpp rather than
// assumed:
//
//   1. ONE glDrawElementsInstanced PER DISTINCT MESH+SUBMESH, never per
//      object. A million cubes sharing a mesh is one draw call.
//   2. PER-INSTANCE DATA LIVES IN SSBOs indexed by gl_InstanceID. The CPU
//      touches no per-object transform at draw time.
//   3. GEOMETRY UPLOADS ONCE per mesh and lives until the mesh set changes.
//   4. INSTANCE BUFFERS GROW IN PLACE (glBufferSubData), never reallocate,
//      once they exist.
//
// ⚠ AND v2's DATA MODEL ALREADY FITS. RenderScene::meshes is the distinct-mesh
// list; DrawItem carries mesh_index + local_to_world + material_override and IS
// v1's per-instance record under another name. Nothing here redesigns v2's
// scene -- it groups what is already there by mesh_index.
//
// ⛔ WHAT THIS DELIBERATELY IS NOT:
//   * NOT part of spade_render. That library is installed, embeddable and has
//     no windowing or GL dependency, and it stays that way. This is a separate
//     optional module behind SPADE_RENDER_GL -- the same shape spade_compute
//     already uses behind SPADE_VULKAN, so this applies an established pattern
//     rather than inventing one.
//   * NOT a window owner. It renders into whatever framebuffer is bound and
//     knows nothing about GLFW. The caller owns the context and the present.
//   * NOT a replacement for raster_cpu. The user ruled GPU default, CPU
//     fallback; raster_cpu keeps three jobs -- the no-GPU fallback, the
//     headless/CI surface, and the reference any future tolerance band is
//     measured against.
//
// ⭐ SHADERS ARE EMBEDDED, NOT LOADED FROM DISK, AND THAT IS A FIX RATHER THAN
// A STYLE. v1 loads its shaders through a CWD-RELATIVE "assets/shaders/..."
// path, so its executables die at shader load -- "with no window ever opening"
// -- when launched from anywhere but their own directory. That trap cost a run
// on 2026-09-18 and it is documented in demo.ps1 in the exact words of the
// failure. A renderer whose correctness depends on the caller's working
// directory has a failure mode it does not need.

#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <string_view>
#include <vector>

#include "core/error.hpp"
#include "render/scene.hpp"
#include "render/target.hpp"

namespace spade::render_gl {

// How the caller hands us GL entry points. Taking a loader rather than linking
// one keeps this module free of GLFW: the sandbox passes glfwGetProcAddress,
// a Qt host would pass its own, and neither dependency reaches in here.
using GlProcLoader = void* (*)(const char* name);

class GlRenderer {
  public:
    // Requires a CURRENT GL context on the calling thread -- it queries the
    // version and compiles programs immediately, so a failure is reported here
    // rather than at the first frame.
    //
    // ⚠ NEEDS GL 4.3 (or ARB_shader_storage_buffer_object): the whole
    // mechanism rests on SSBOs. Refused with the measured version named, never
    // degraded -- a renderer that silently falls back to something slower is
    // indistinguishable from the bug this module exists to fix.
    [[nodiscard]] static Result<std::unique_ptr<GlRenderer>> create(GlProcLoader loader);

    ~GlRenderer();
    GlRenderer(const GlRenderer&) = delete;
    GlRenderer& operator=(const GlRenderer&) = delete;

    // Uploads geometry for every mesh in the scene. Call when the MESH SET
    // changes -- at scene load -- and not per frame. Idempotent: calling it
    // again re-uploads, which is what a changed mesh set needs.
    [[nodiscard]] Result<void> upload_scene(const render::RenderScene& scene);

    // One frame. Groups statics+dynamics by mesh_index, refreshes the instance
    // SSBOs in place, and issues one instanced draw per mesh+submesh.
    //
    // Honours RenderOptions as raster_cpu does, except what unhonoured()
    // names. A background pass draws the sky in every mode. In shaded mode it
    // also draws the analytic ground, the grid and the atmospheric term
    // (SR-17, SR-17a, SR-22). Meshes draw shaded, wireframe or velocity.
    // Field layers draw filled in every mode, as raster_cpu draws them.
    // Raymarch is refused with Code::unavailable: it is a CPU technique. So
    // is a malformed field layer, with Code::invalid_argument.
    //
    // Renders into the CURRENTLY BOUND framebuffer at the given size and
    // covers every pixel. It does not present: that belongs to whoever owns
    // the window. It changes depth, cull and polygon-mode state, so a caller
    // drawing its own geometry afterwards sets the state it needs.
    [[nodiscard]] Result<void> draw(const render::RenderScene& scene, const render::Camera& camera,
                                    const render::RenderOptions& options, uint32_t width,
                                    uint32_t height);

    // Names the options in `options` that draw() will not draw, so the gap is
    // announced, not silent (L6). Each name is the RenderOptions field it
    // concerns: "shadows", "overlays", or "mode" for a mode draw() refuses.
    // A caller shows the list, or refuses GL when it needs one of them.
    // Needs no GL context.
    [[nodiscard]] static std::vector<std::string_view> unhonoured(const render::RenderOptions& options);

    // ⭐ THE MECHANISM, MADE MEASURABLE RATHER THAN CLAIMED. The entire case
    // for this module is "draw calls scale with meshes, not objects", and a
    // claim like that is exactly the kind this estate has learned to assert in
    // a test. After draw(), these report what the last frame actually issued --
    // so "one draw per mesh" is checkable by a caller with no display, against
    // a scene whose instance count it chose. The background pass and each
    // field layer are one more draw apiece, and are not counted.
    [[nodiscard]] uint32_t last_draw_calls() const noexcept;
    [[nodiscard]] uint32_t last_instances() const noexcept;

    // What the driver says it is. Reported rather than assumed, because the
    // difference between a real driver and a software implementation changes
    // every performance number measured through this class.
    [[nodiscard]] const std::string& renderer_name() const noexcept;
    [[nodiscard]] const std::string& version_string() const noexcept;

  private:
    struct Impl;
    explicit GlRenderer(std::unique_ptr<Impl> impl);
    std::unique_ptr<Impl> impl_;
};

}  // namespace spade::render_gl
