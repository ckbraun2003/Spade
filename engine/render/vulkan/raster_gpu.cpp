// ---------------------------------------------------------------------------
// raster_gpu.cpp -- the GPU rasterizer's FACADE, and the Vulkan boundary for
// engine/render/. Structurally a mirror of compute/vulkan/backend.cpp, which
// is 135 lines for the same reason this file is short: the genuinely
// Vulkan-shaped work lives one level down (there, StateMirror + StepRecorder;
// here, BackgroundPass), and this file's whole job is construction order,
// lifetime, and forwarding.
//
// THE MIRRORING IS DELIBERATE AND IS THE RISK CONTROL. Every decision in this
// file that could have gone two ways went the way compute/vulkan/ already
// went -- validate-before-acquire, a private constructor behind a factory,
// unique_ptr member order as destruction order, Code::unavailable for "no
// device" and Code::invalid_argument for "you asked for something this build
// cannot provide". A second Vulkan consumer in one engine that invented its
// own conventions would double the surface a reader has to learn for no gain.
//
// ⚠ THE S6 DEPENDENCY RULE IS EXTENDED HERE, NOT BREACHED. compute/vulkan/
// context.hpp cites spec section 2 -- "nothing outside engine/compute/
// includes a Vulkan header" -- and in the same comment says
// "camera/rendering/FramePool is S7 (Addendum A8), out of scope for every S6
// task". The rule drew its boundary around the only Vulkan consumer that
// existed and DEFERRED rendering rather than forbidding it. Its intent is
// that a SPADE_VULKAN=OFF build still links and that almost nothing sees
// volk; both are preserved by giving render/ its own vulkan/ subdirectory
// with a Vulkan-free header above it and a stub translation unit beside it,
// exactly as compute/vulkan/backend_stub.cpp does.
// ---------------------------------------------------------------------------

#include "render/raster_gpu.hpp"

#include <string>
#include <utility>

#include "compute/backend.hpp"
#include "compute/vulkan/context.hpp"
#include "render/vulkan/background_pass.hpp"

namespace spade::render {

// ---------------------------------------------------------------------------
// Impl -- the Vulkan-shaped state raster_gpu.hpp's forward declaration hides.
// Declaration order IS construction order and reverse destruction order, so
// context-then-pass tears the pass down first and the device last, which is
// the dependency order each was built in. Same discipline, same reason, as
// VulkanBackend::Impl's context/mirror/recorder.
// ---------------------------------------------------------------------------
struct GpuRasterizer::Impl {
    std::unique_ptr<compute::VulkanContext> ctx;
    std::unique_ptr<BackgroundPass> pass;
    std::string device_name;
};

GpuRasterizer::GpuRasterizer() : impl_(std::make_unique<Impl>()) {}
GpuRasterizer::~GpuRasterizer() = default;
GpuRasterizer::GpuRasterizer(GpuRasterizer&&) noexcept = default;
GpuRasterizer& GpuRasterizer::operator=(GpuRasterizer&&) noexcept = default;

Result<std::unique_ptr<GpuRasterizer>> GpuRasterizer::create(const GpuRasterizerDesc& desc) {
    // ARGUMENT VALIDATION BEFORE RESOURCE ACQUISITION, and the ordering is the
    // same contract VulkanBackend::create() states rather than a style echo:
    // a zero or absurd frame bound is wrong on a box with no Vulkan device at
    // all, so answering it with Code::unavailable ("no device") would hand the
    // caller the wrong diagnosis on every machine without a GPU. VALIDATING
    // FIRST MEANS THE DIAGNOSIS DOES NOT DEPEND ON THE ENVIRONMENT.
    if (desc.max_width == 0 || desc.max_height == 0) {
        return std::unexpected(Error{Code::invalid_argument,
                                     "GpuRasterizerDesc max_width/max_height must both be non-zero; got " +
                                         std::to_string(desc.max_width) + "x" + std::to_string(desc.max_height) +
                                         ". Device buffers are sized ONCE from these and never grown, so a "
                                         "zero bound is not an 'unbounded' request -- it is a buffer no "
                                         "frame can fit in."});
    }

    // The context is compute's, unchanged and unwrapped. It wants a
    // BackendDesc, and the workgroup size it carries is NOT this rasterizer's
    // dispatch size: the background kernel is compiled at a fixed
    // [numthreads(64,1,1)] and never joins the variant family. 64 is passed
    // because it is a value workgroup_size_supported() accepts and because
    // passing anything else would imply a choice this path does not have.
    const compute::BackendDesc backend_desc{compute::BackendKind::vulkan, 64u, desc.device_index};

    // Code::unavailable propagates from here on a box with no loader and no
    // device. THAT IS AN ORDINARY OUTCOME, not a fault: the caller is expected
    // to fall back to raster_cpu -- AND TO SAY SO WHERE SOMEONE WILL SEE IT.
    // A silent fallback is indistinguishable from "the GPU default never
    // landed" and reproduces the blocky viewport with a different cause and no
    // way to tell which. raster_gpu.hpp states that obligation; this comment
    // exists because THIS is the line that creates it.
    Result<std::unique_ptr<compute::VulkanContext>> ctx = compute::VulkanContext::create(backend_desc);
    if (!ctx) return std::unexpected(ctx.error());

    Result<std::unique_ptr<BackgroundPass>> pass =
        BackgroundPass::create(**ctx, desc.max_width, desc.max_height);
    if (!pass) return std::unexpected(pass.error());

    // Private default constructor behind a factory -- construction is only
    // ever valid through this function, which is why make_unique (needing a
    // public constructor) cannot be used, matching VulkanContext::create()
    // and VulkanBackend::create() exactly.
    auto self = std::unique_ptr<GpuRasterizer>(new GpuRasterizer());
    self->impl_->device_name = (*ctx)->device_name();
    self->impl_->ctx = std::move(*ctx);
    self->impl_->pass = std::move(*pass);
    return self;
}

Result<void> GpuRasterizer::render_background(const RenderScene& scene, const Camera& camera,
                                               const RenderOptions& options, RenderTarget& target) {
    return impl_->pass->render(scene, camera, options, target);
}

const char* GpuRasterizer::device_name() const noexcept { return impl_->device_name.c_str(); }

}  // namespace spade::render
