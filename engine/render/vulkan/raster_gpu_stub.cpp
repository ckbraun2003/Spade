// ---------------------------------------------------------------------------
// raster_gpu_stub.cpp -- the SPADE_VULKAN=OFF definition of
// spade::render::GpuRasterizer, and the exact mirror image of
// compute/vulkan/backend_stub.cpp.
//
// WHY THIS FILE EXISTS, IN THE FORM THAT MAKES IT NECESSARY RATHER THAN TIDY.
// The GPU/CPU choice this rasterizer serves is a RUNTIME one, and it has to be:
// raster_gpu.hpp and compute/vulkan/context.hpp both state that the only
// available detection shape is TRY-AND-FALL-BACK, because VulkanContext reports
// "no loader, no device" as a Code::unavailable FROM create() rather than
// through a probe a caller could ask first. So the caller that wants a GPU
// frame calls GpuRasterizer::create() UNCONDITIONALLY and falls back to
// raster_cpu on unavailable -- which means that call expression is compiled
// into its object code whatever SPADE_VULKAN is set to, and the linker must
// resolve it either way.
//
// render/vulkan/raster_gpu.cpp -- the real implementation -- is compiled only
// in the SPADE_VULKAN=ON branch, because it includes a header that includes
// volk. Without this file, an OFF build would COMPILE cleanly (raster_gpu.hpp
// is Vulkan-free and physically present in the tree either way) and then fail
// to LINK any executable containing that call. That is the regression this
// file closes, and it is the same one backend_stub.cpp closed for physics.
//
// ⚠ AND THE TWO ARE NEVER COMPILED INTO THE SAME PROGRAM, which is what makes
// two definitions of the same methods legal rather than an ODR violation:
// engine/CMakeLists.txt compiles raster_gpu.cpp in the ON branch and this file
// in the OFF branch, exactly as it already does for backend.cpp/backend_stub.cpp.
// ⛔ THAT IS A PROPERTY OF THE CMAKE GATE, NOT OF THIS FILE -- there is no
// preprocessor guard in here to enforce it, deliberately, because a guard would
// let a wrong gate produce a silently empty translation unit instead of a
// duplicate-symbol error. THE LINKER IS THE BETTER ALARM.
//
// ⛔ NOT IN ANY BUILD GRAPH YET. Like background_pass.cpp beside it, no
// CMakeLists names this file until the wiring commit lands. It is written now
// because it needs no device, no SPIR-V and no build to be correct -- unlike
// the wiring, which needs all three.
// ---------------------------------------------------------------------------

#include "render/raster_gpu.hpp"

namespace spade::render {

// Defined here, EMPTY -- not merely forward-declared the way raster_gpu.hpp
// leaves it. unique_ptr<Impl>'s destructor, which this class's own destructor
// and both defaulted move operations each implicitly instantiate, needs Impl to
// be COMPLETE where it is compiled, regardless of whether the pointer it
// destroys is ever non-null at runtime: `delete ptr` needs sizeof(Impl) even
// inside a branch a null pointer never takes.
struct GpuRasterizer::Impl {};

GpuRasterizer::GpuRasterizer() : impl_(nullptr) {}
GpuRasterizer::~GpuRasterizer() = default;
GpuRasterizer::GpuRasterizer(GpuRasterizer&&) noexcept = default;
GpuRasterizer& GpuRasterizer::operator=(GpuRasterizer&&) noexcept = default;

// Code::unavailable, and NOT a new code invented for this build. From a
// caller's side "Vulkan was not compiled in" and "this box has no Vulkan
// loader" are the same fact -- the GPU path is not available in this process --
// so this returns the identical code the ON build already reports for a
// deviceless box, and a caller needs one branch rather than two.
//
// ⛔ THE FALLBACK STILL HAS TO ANNOUNCE ITSELF, AND THIS FILE IS THE HARDEST
// CASE FOR THAT. A silent CPU fallback is indistinguishable from "the GPU
// default never landed" and from "the device is missing" -- three causes, one
// blocky viewport. The message therefore names the BUILD OPTION rather than
// only the outcome, because that is the one of the three a log line cannot
// otherwise tell apart.
Result<std::unique_ptr<GpuRasterizer>> GpuRasterizer::create(const GpuRasterizerDesc&) {
    return std::unexpected(Error{Code::unavailable,
                                 "the GPU rasterizer was not compiled into this build (SPADE_VULKAN=OFF); "
                                 "the caller is expected to fall back to render::render() and to SAY SO"});
}

// Unreachable at runtime -- create() never succeeds, so no GpuRasterizer is
// ever constructed in this build -- but it still needs a DEFINITION, because a
// caller's `if (gpu) gpu->render_background(...)` compiles the call expression
// regardless of the guard that is always false here.
Result<void> GpuRasterizer::render_background(const RenderScene&, const Camera&, const RenderOptions&,
                                              RenderTarget&) {
    return std::unexpected(Error{Code::unavailable,
                                 "GpuRasterizer::render_background: unreachable (SPADE_VULKAN=OFF)"});
}

// A LITERAL, NEVER nullptr. This accessor has no Result to carry a diagnosis,
// and the one path that would ever reach it is a caller logging the device name
// on a build that has no device -- i.e. precisely the path least likely to have
// been exercised. Returning nullptr would turn the least-tested branch into a
// crash in a log statement.
const char* GpuRasterizer::device_name() const noexcept { return "none (SPADE_VULKAN=OFF)"; }

}  // namespace spade::render
