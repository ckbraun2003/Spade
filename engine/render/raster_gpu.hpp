#pragma once

// ---------------------------------------------------------------------------
// raster_gpu -- the GPU rasterizer's PUBLIC, VULKAN-FREE surface. Approved by
// the user 2026-09-24 ("Yes build a gpu renderer").
//
// This header is to render/vulkan/ exactly what compute/backend.hpp is to
// compute/vulkan/: the Vulkan boundary lives one directory down, and a
// consumer that only needs to ask for a frame never sees volk. That mirroring
// is deliberate and is the reason the S6 dependency rule survives the
// addition -- see shaders/kernels/raster_background.slang's header for the
// amendment and why it is an extension of that rule rather than a breach.
//
// ===========================================================================
// ⛔⛔ STAGE 1 DRAWS THE BACKGROUND ONLY, AND THE ENTRY POINT IS NAMED FOR IT.
//
// There is deliberately NO `render()` in this header yet. A GPU `render()`
// that quietly drew the sky and the ground and no meshes would produce a
// frame that LOOKS like a rendered scene -- correct horizon, correct grid,
// correct lighting, and every drone missing -- and a caller has no way to
// tell that from a scene that genuinely has no geometry.
//
// ***A PARTIAL IMPLEMENTATION BEHIND A COMPLETE NAME IS INDISTINGUISHABLE FROM
// A COMPLETE ONE WHOSE INPUT WAS EMPTY.*** The name is the only guard that
// costs nothing, so stage 1 ships `render_background()` and `render()` does
// not exist until it can draw what it says.
//
// ===========================================================================
// WHY A PERSISTENT OBJECT AND NOT A FREE FUNCTION
//
// render/raster_cpu.hpp's render() is a free function because its only state
// is an optional scratch buffer. A GPU path cannot be: the device, the
// pipeline, the descriptor set, the parameter buffer and the readback
// staging buffer are all expensive to create and must survive across frames.
// Creating them per call would spend more time than the CPU path it replaces.
//
// DETERMINISM (raster_cpu.hpp's constraint 4) IS PRESERVED IN THE SAME FORM:
// render_background() reads only its explicit inputs plus this object's
// device resources, and the device resources carry no frame-to-frame state
// that affects a pixel -- they are allocations, not memory.
//
// ===========================================================================
// ⚠ THIS PATH IS NOT BIT-IDENTICAL TO raster_cpu.cpp AND MUST NOT BE HELD TO
// THAT STANDARD. raster_cpu.cpp computes in fp64 (its Vec3 is `double x,y,z`;
// 95 `double` against 38 `float`); a compute shader is fp32, because fp64 is
// optional in Vulkan, commonly absent, and commonly 1/32 rate where present.
//
// The evidence this path takes is render/agreement.hpp's compare_silhouettes()
// -- coverage, never colour -- against a FLAT-SKY scene, which that header
// requires and explains. ⛔ Do NOT add this path to tests/test_gpu_parity.cpp:
// those tables compare quantities that are fp32 on both sides by construction,
// and a zero band here would be red on its first run.
//
// ===========================================================================
// AVAILABILITY IS A RESULT, NOT A QUESTION ASKED IN ADVANCE
//
// create() returns Code::unavailable when there is no Vulkan loader or no
// suitable device -- the same shape compute/vulkan/backend.hpp already uses
// and documents. There is deliberately no `is_gpu_available()` free function:
// a probe that says yes and a create() that then fails are two answers to one
// question, and the estate has paid for that shape before.
//
// ⛔⛔ AND THE CALLER MUST ANNOUNCE THE FALLBACK. The user ruled that the
// viewport defaults to GPU when available; a silent drop to raster_cpu is
// indistinguishable from "the GPU default never landed" and from "this box
// has no device", and it reproduces the blocky viewport with a different
// cause and no way to tell which. Whoever calls create() and falls back owes
// a visible, durable statement of WHICH PATH DREW THE FRAME -- not a log line
// that scrolls. This header cannot enforce that; it states it because the
// obligation is created here.
// ---------------------------------------------------------------------------

#include <cstdint>
#include <memory>

#include "core/error.hpp"
#include "render/scene.hpp"
#include "render/target.hpp"

namespace spade::render {

struct GpuRasterizerDesc {
    // Which physical device, in the same enumeration order compute's
    // BackendDesc::device_index uses. A caller running both the compute
    // backend and this rasterizer should pass the SAME index: two Vulkan
    // devices in one process is legal and doubles the memory for no gain.
    uint32_t device_index = 0;

    // NO workgroup_size KNOB, DELIBERATELY. The background kernel is compiled
    // at a FIXED [numthreads(64,1,1)] and does not join the workgroup-size
    // variant family -- that family exists to serve a PARITY property of the
    // physics path (T9 pins that varying the size changes no result), and this
    // path has no parity obligation because raster_cpu.cpp is fp64.
    // A knob that cannot change the outcome is worse than no knob: it invites
    // a caller to tune something, and every value they choose is the same.

    // The largest frame this rasterizer will be asked for. Device buffers are
    // sized ONCE from this and never grown: a mid-run reallocation would make
    // frame cost depend on history, and the whole point of this path is a
    // predictable per-frame budget. A render_background() call whose target
    // exceeds it fails rather than silently reallocating.
    uint32_t max_width = 2560;
    uint32_t max_height = 1440;
};

class GpuRasterizer {
public:
    // Code::unavailable -- no Vulkan loader, no suitable device, or a device
    // that cannot satisfy the descriptor. THAT IS AN ORDINARY OUTCOME on a box
    // without a GPU and the caller is expected to fall back to raster_cpu --
    // loudly. See this header's own note on the announcement obligation.
    [[nodiscard]] static Result<std::unique_ptr<GpuRasterizer>> create(const GpuRasterizerDesc& desc);

    ~GpuRasterizer();
    GpuRasterizer(const GpuRasterizer&) = delete;
    GpuRasterizer& operator=(const GpuRasterizer&) = delete;
    GpuRasterizer(GpuRasterizer&&) noexcept;
    GpuRasterizer& operator=(GpuRasterizer&&) noexcept;

    // Draws the sky gradient, the analytic ground planes, the overlay grid and
    // the horizon fade into `target`, and NOTHING ELSE -- no meshes, no
    // dynamics, no shadows, no body markers. See this header's opening note
    // for why the name says so.
    //
    // `scene` is read for its lighting, its ground planes and its materials;
    // scene.statics and scene.dynamics are NOT read at this stage and a scene
    // carrying them is not an error -- the caller asked for a background.
    //
    // Fails with Code::invalid when target exceeds the descriptor's max_width
    // or max_height, rather than reallocating. See GpuRasterizerDesc.
    [[nodiscard]] Result<void> render_background(const RenderScene& scene, const Camera& camera,
                                                  const RenderOptions& options, RenderTarget& target);

    // The device this actually got, for the announcement the caller owes and
    // for a benchmark row that must name its hardware. baselines.json's _meta
    // already warns that this box's GPU "is not a proxy for a more capable
    // device a reader might assume"; a frame time without a device name
    // repeats that mistake with fresher numbers.
    [[nodiscard]] const char* device_name() const noexcept;

private:
    GpuRasterizer();
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace spade::render
