#pragma once

// ---------------------------------------------------------------------------
// BackgroundPass -- the Vulkan side of stage 1 of the GPU rasterizer. Owns the
// descriptor set layout, the compute pipeline, the device buffers and the
// readback staging for shaders/kernels/raster_background.slang, and nothing
// else. raster_gpu.cpp's facade owns one of these; nothing outside
// engine/render/vulkan/ names it.
//
// THIS HEADER INCLUDES volk.h AND THAT IS THE POINT. It is render/'s Vulkan
// boundary, exactly as compute/vulkan/context.hpp is compute's: the public
// surface one directory up (render/raster_gpu.hpp) stays Vulkan-free, so a
// consumer that only wants a frame never sees a Vulkan type, and a
// SPADE_VULKAN=OFF build links against the stub instead of this.
//
// ===========================================================================
// ⛔ ONE PASS, ONE DISPATCH, ONE READBACK -- AND NO FRAME-TO-FRAME STATE THAT
// CAN AFFECT A PIXEL.
//
// render() uploads the parameter block and the ground-plane list, dispatches
// one thread per pixel, and copies the result back. The device buffers persist
// ACROSS frames (creating them per call would cost more than the CPU path this
// replaces) but they are ALLOCATIONS, NOT MEMORY: every byte the kernel reads
// is written by the same render() call that dispatches it.
//
// That is what preserves raster_cpu.hpp's determinism constraint 4 in the same
// form -- the output is a pure function of the explicit inputs. ⛔ A future
// stage that keeps a previous frame's contents on the device (temporal
// reprojection, an accumulation buffer) BREAKS THAT and must say so loudly at
// the seam rather than quietly here, because a renderer whose output depends
// on its own history cannot be compared frame-for-frame against a CPU
// reference that has none.
//
// ===========================================================================
// ⚠ SIZED ONCE, NEVER GROWN -- AND THE REFUSAL IS THE FEATURE.
//
// create() allocates for the descriptor's max_width x max_height and render()
// REFUSES a larger target rather than reallocating. A mid-run reallocation
// would make frame cost depend on history, which defeats the entire purpose of
// a path whose value is a predictable per-frame budget -- and it would do it
// invisibly, because the frame that pays for the reallocation looks exactly
// like the frame that did not.
//
// ⭐ THE SAME ARGUMENT THE ENGINE ALREADY MAKES ONE LAYER DOWN: sim/simulation.hpp
// pins FIXED CAPACITY because "a spawn past capacity is capacity_exceeded,
// never a silent grow". This is that rule applied to pixels instead of bodies,
// and it is stated here rather than inherited silently.
//
// ===========================================================================
// SET 1, AND SET 0 IS NOT TOUCHED
//
// shaders/shared/bindings.slang is the PHYSICS binding registry: nine bound
// arrays at 0..8, four .slot_to_world siblings at 9..12, DENSE and guarded by
// test_slang_layouts.cpp's BindingIndicesAreDistinctAndDense. Adding a render
// buffer there would renumber physics bindings and break the one path whose
// entire value is that it is pinned. These three bindings therefore live in
// SET 1 and need no entry in that registry at all.
// ---------------------------------------------------------------------------

#include <cstdint>
#include <memory>
#include <vector>

#include <volk.h>

#include "compute/vulkan/context.hpp"
#include "core/error.hpp"
#include "render/scene.hpp"
#include "render/target.hpp"

namespace spade::render {

class BackgroundPass {
public:
    // `ctx` is BORROWED, not owned: GpuRasterizer::Impl declares the context
    // before the pass, so the context outlives it by construction order. This
    // mirrors StepRecorder taking the StateMirror's descriptor set rather than
    // allocating its own -- one owner, one lifetime, no shared_ptr.
    //
    // Code::invalid_argument for a zero bound; Code::internal for a Vulkan
    // call that fails for a reason the caller did not cause. NOT
    // Code::unavailable -- by the time this runs a device already exists, so
    // "no device" is not a reachable answer here and returning it would tell
    // the caller to fall back for the wrong reason.
    [[nodiscard]] static Result<std::unique_ptr<BackgroundPass>> create(compute::VulkanContext& ctx,
                                                                        uint32_t max_width,
                                                                        uint32_t max_height);

    ~BackgroundPass();
    BackgroundPass(const BackgroundPass&) = delete;
    BackgroundPass& operator=(const BackgroundPass&) = delete;

    // Draws sky + ground planes + grid + horizon fade into `target`.
    //
    // Reads from `scene`: lighting, ground_planes, materials. DOES NOT READ
    // statics or dynamics -- a scene carrying them is not an error, because
    // the caller asked for a background. See render/raster_gpu.hpp for why the
    // entry point is named for what it draws rather than called render().
    //
    // Code::invalid_argument for a target exceeding the create() bounds, or a target
    // whose stride is narrower than its width.
    [[nodiscard]] Result<void> render(const RenderScene& scene, const Camera& camera,
                                      const RenderOptions& options, RenderTarget& target);

private:
    BackgroundPass() = default;

    // MIRRORS raster_background.slang's BackgroundParams FIELD FOR FIELD AND
    // IN ORDER. ⛔ THE TWO ARE A PAIR AND NOTHING COMPARES THEM: the physics
    // mirrors are pinned by generated static_asserts (bindings.gen.hpp), and
    // this struct has no such generator because it is not in that registry.
    // So the ordering here is load-bearing prose, and a field added to one
    // side without the other is a silent misread rather than a build failure.
    //
    // ⚠ THAT IS A KNOWN GAP AND IT IS NAMED RATHER THAN PAPERED OVER. The
    // cheapest real guard is a std430 layout assertion over this struct's
    // size and offsets against the Slang reflection, the same way
    // spade_slang_generated_layouts() does for the physics mirrors -- owed
    // before this path carries anything a caller depends on, and cheap once
    // the kernel is registered with the build.
    struct Params {
        float cam_pos[3];
        float inv_tan_half_fov;
        float right_world[3];
        float aspect;
        float up_world[3];
        uint32_t width;
        float forward_world[3];
        uint32_t height;

        float sun_direction[3];
        float sun_intensity;
        float sun_color[3];
        float horizon_strength;
        float ambient_color[3];
        float horizon_onset;
        float sky_zenith[3];
        uint32_t plane_count;
        float sky_horizon[3];
        uint32_t row_pitch_texels;

        float grid_color[3];
        float grid_spacing;
        float grid_line_half_width;
        float grid_width_growth;
        float grid_fade_distance;
        uint32_t grid_enabled;
    };

    // Mirrors the kernel's GroundPlaneGpu, same pairing caveat as Params.
    struct PlaneGpu {
        float normal[3];
        float offset;
        float base_color[3];
        uint32_t is_front;
        float grid_u[3];
        // The kernel's repurposed pad0: Material::shading, 0 = lambert /
        // 1 = unlit / 2 = emissive. A uint32_t where a float of padding was, so
        // sizeof(PlaneGpu) and every offset are unchanged -- the only property
        // this unguarded mirror can rely on.
        uint32_t shading_mode;
        float grid_v[3];
        float pad1;
    };

    compute::VulkanContext* ctx_ = nullptr;   // borrowed; outlives this by construction order

    // ⛔ SET 0 IS DECLARED AND EMPTY, AND IT IS NOT OPTIONAL. The kernel binds
    // its three resources at [[vk::binding(n, 1)]] -- SET 1 -- so the pipeline
    // layout must describe sets 0 AND 1 for set 1 to mean set 1. A layout with
    // one entry would bind these descriptors as SET 0 and the shader would read
    // nothing. Zero-binding layouts are legal and cost one handle.
    //
    // ⚠ AND THE STAGE-1 JUSTIFICATION FOR SET 1 WAS OVERSTATED, CORRECTED HERE
    // RATHER THAN LEFT TO LAUNDER. It said adding a render buffer to set 0 would
    // renumber the physics bindings. It would not: shaders/shared/bindings.slang
    // is a SOURCE-level registry and this kernel does not `import bindings` at
    // all (its only import is fp32_math), so its set numbering is private to its
    // own pipeline layout and could have been 0 with nothing to collide with.
    // Set 1 is kept because `binding(n, 0)` means PHYSICS by convention
    // everywhere else in this tree and a grep should not turn up a render buffer
    // wearing that spelling -- a legibility argument, which is a real one, but a
    // SMALLER one than the commit message claimed. A FALSE MECHANISM IN A
    // COMMENT IS WHAT THE NEXT IMPLEMENTER DESIGNS AGAINST.
    VkDescriptorSetLayout empty_set0_layout_ = VK_NULL_HANDLE;
    VkDescriptorSetLayout set_layout_ = VK_NULL_HANDLE;
    VkDescriptorPool descriptor_pool_ = VK_NULL_HANDLE;
    VkDescriptorSet descriptor_set_ = VK_NULL_HANDLE;
    VkPipelineLayout pipeline_layout_ = VK_NULL_HANDLE;
    VkPipeline pipeline_ = VK_NULL_HANDLE;
    VkShaderModule module_ = VK_NULL_HANDLE;

    VkCommandPool command_pool_ = VK_NULL_HANDLE;
    VkCommandBuffer command_buffer_ = VK_NULL_HANDLE;
    VkFence fence_ = VK_NULL_HANDLE;

    // binding 0 -- the pixels the kernel writes, device-local, sized for
    // max_width * max_height texels and never grown.
    VkBuffer pixels_ = VK_NULL_HANDLE;
    VkDeviceMemory pixels_memory_ = VK_NULL_HANDLE;

    // binding 1 -- the ground-plane list, host-visible, written per frame.
    VkBuffer planes_ = VK_NULL_HANDLE;
    VkDeviceMemory planes_memory_ = VK_NULL_HANDLE;
    void* planes_mapped_ = nullptr;
    uint32_t plane_capacity_ = 0;

    // binding 2 -- the parameter block, host-visible, written per frame.
    VkBuffer params_ = VK_NULL_HANDLE;
    VkDeviceMemory params_memory_ = VK_NULL_HANDLE;
    void* params_mapped_ = nullptr;

    // The readback destination. Host-visible and COHERENT so the copy is
    // visible without an explicit invalidate -- the frame is read once,
    // immediately, by the thread that submitted it.
    VkBuffer staging_ = VK_NULL_HANDLE;
    VkDeviceMemory staging_memory_ = VK_NULL_HANDLE;
    void* staging_mapped_ = nullptr;

    uint32_t max_width_ = 0;
    uint32_t max_height_ = 0;

    // Scratch reused across frames so a per-frame std::vector allocation does
    // not reappear in a path whose entire value is a predictable budget.
    std::vector<PlaneGpu> plane_scratch_;
};

}  // namespace spade::render
