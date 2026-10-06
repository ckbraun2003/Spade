// ---------------------------------------------------------------------------
// raster_background.slang -- STAGE 1 OF THE GPU RASTERIZER: the background
// pass. The compute-shader port of render/raster_cpu.cpp's
// draw_sky_and_ground_background(): sky gradient, N analytic ground planes,
// the overlay grid, and the horizon fade, one thread per pixel.
//
// Approved by the user 2026-09-24 ("Yes build a gpu renderer"). This is the
// first stage because it is the FILL-BOUND one: tests/bench/bench_render.cpp
// measures BackgroundFill and GroundPlane at ~400 ms for a 1892x1066 frame on
// this box, and raster_cpu.cpp is single-threaded (zero std::thread /
// execution::par anywhere in its 1549 lines). The blocky editor viewport is
// that budget made visible -- the render-scale cap and the nearest-neighbour
// upscale are what the editor does to buy the frame rate back.
//
// ===========================================================================
// THE ONE THING TO READ FIRST: THIS IS NOT A PARITY PORT, AND IT CANNOT BE.
//
// Every other Slang kernel in this directory mirrors a CPU path that is fp32
// on BOTH sides by construction -- that is what core/fp32_math.hpp exists for,
// and what lets tests/test_gpu_parity.cpp pin most quantities BIT-EXACT.
//
//   ⛔ render/raster_cpu.cpp IS fp64. Its `Vec3` is `double x, y, z` (:42-44),
//      and the file carries 95 occurrences of `double` against 38 of `float`.
//      The background pass computes the camera basis, the per-column and
//      per-row ray vectors, the ray/plane intersection and the hit point in
//      DOUBLE.
//
// fp64 in a compute shader is optional in Vulkan, absent on many devices, and
// commonly 1/32 rate where present. So this kernel is fp32, the two paths
// differ in the last bits BY CONSTRUCTION, and A BIT-IDENTITY CLAIM HERE WOULD
// BE A CLAIM THE HARDWARE CANNOT KEEP -- the same reason R3 refused to tighten
// the gaussian sqrt band in rng.slang rather than pretending it could.
//
// ⛔ DO NOT ADD THIS KERNEL TO tests/test_gpu_parity.cpp's tables. Those
// compare quantities that are fp32 on both sides. A zero band here would be
// red on the first run and a loose band would be a number nobody chose.
//
// ✅ THE EVIDENCE THIS PATH NEEDS ALREADY EXISTS AND IS THE RIGHT INSTRUMENT:
// render/agreement.hpp's compare_silhouettes(), built for exactly this shape
// -- a FAST path measured against an ESTABLISHED REFERENCE (it was written for
// raster_cpu vs raymarch). It is COVERAGE ONLY, NEVER COLOUR: it asks of each
// frame independently "is this pixel the sky, or not", and never compares a
// hit pixel's colour to the other frame's. Two frames agreeing on every
// silhouette edge and disagreeing on every lit pixel's exact shade report 0.0.
// That is precisely the discrimination this pass needs.
//
//   ⚠⚠ AND ITS ONE TRAP APPLIES TO US IN FULL: compare_silhouettes() takes a
//   SINGLE FLAT sky_reference_rgb and no Lighting, so it is only a correct
//   "is this the sky" test when the compared frames were rendered from a scene
//   whose sky is FLAT (sky_horizon == sky_zenith). Against a gradient sky a
//   genuine miss carries its own row's colour, reads as "covered", and THE
//   DISAGREEMENT CANCELS ITSELF OUT BEFORE THE COMPARISON EVER SEES IT --
//   silently, and in the direction that looks like success. Every agreement
//   fixture for this kernel MUST flatten the sky first. See agreement.hpp.
//
// ===========================================================================
// WHY A COMPUTE SHADER AND NOT A GRAPHICS PIPELINE
//
// The consumer is a caller-owned RenderTarget (render/target.hpp): a
// CPU-readable byte buffer with a stride, which a host blits or hands on as a
// frame. A camera sensor wants the same thing. Nothing presents to a surface.
//
// So this path needs NO swapchain, NO render pass, NO framebuffer and NO
// VkSurfaceKHR -- it needs a dispatch that writes pixels into a storage buffer
// and a readback. That is EXACTLY what engine/compute/vulkan/ already does,
// and its VulkanContext is general: device(), compute_queue(),
// transfer_queue(), compute_queue_family(), loaded through volk, and its own
// header states "No VkSurfaceKHR anywhere in this file or its .cpp".
//
// ⚠ THE S6 DEPENDENCY RULE NEEDS EXTENDING, AND THAT IS A DELIBERATE AMENDMENT
// RATHER THAN AN OVERSIGHT. context.hpp cites spec section 2 -- "nothing
// outside engine/compute/ includes a Vulkan header" -- and in the SAME comment
// explains why: "camera/rendering/FramePool is S7 (Addendum A8), out of scope
// for every S6 task". The rule drew a boundary around the only Vulkan consumer
// that existed at the time and explicitly DEFERRED rendering rather than
// forbidding it. The intent -- that a SPADE_VULKAN=OFF build still links, and
// that almost no code sees volk -- is preserved by giving the renderer its own
// `vulkan/` boundary subdirectory with a Vulkan-free public header above it
// and a stub translation unit, mirroring compute/vulkan/backend_stub.cpp
// exactly. Stated here rather than assumed, because the rule is written down.
//
// ===========================================================================
// ⛔⛔ THIS KERNEL TAKES ITS OWN DESCRIPTOR SET. IT MUST NOT JOIN SET 0.
//
// shaders/shared/bindings.slang is THE BINDING REGISTRY for the physics path,
// and the one statement of what set 0 holds: its registered arrays and the
// buffers derived from them, DENSE, and guarded by tests/test_slang_layouts.cpp's
// BindingIndicesAreDistinctAndDense / BindingRegistryCoversEveryRegisteredArray.
// No count is restated here; it changes as modules register state.
//
// Adding a render buffer to that registry would renumber physics bindings and
// break a path whose whole value is that it is pinned. These resources
// therefore live in SET 1 and the physics set is not touched at all. A render
// resource added here needs NO entry in bindings.slang, and that is the point.
//
// ===========================================================================
// N GROUND PLANES, NOT ONE -- DELIBERATELY
//
// raster_cpu.cpp loops over scene.ground_planes and precomputes a per-plane
// `front[]` flag. This kernel takes the same list as a buffer. A single-plane
// fast path would be smaller, faster to write, and correct for every world
// shipped today -- which is exactly the shape this estate keeps paying for:
// A CARDINALITY SUPPLIED BY THE CURRENT CONTENT, ENCODED AS A STRUCTURE.
// The count is a parameter here for the same reason world_count is one in
// collision_dynamic.slang.
//
// ===========================================================================
// THE PER-FRAME HOIST IS THE HOST'S, MIRRORING THE CPU'S OWN
//
// raster_cpu.cpp hoists the camera basis out of the pixel loop (its Task VQ-B,
// measured: hoisting alone took a ground-bearing frame from 77.6 ms to 31.3 ms
// at 1280x720) and builds per-column and per-row ray vectors. This kernel
// takes `right_world`, `up_world`, `forward_world`, `inv_tan_half_fov` and
// `aspect` ALREADY COMPUTED, for the same reason: they are facts about the
// frame, not about the pixel. The host computes them once, in fp64 if it
// likes, and narrows -- the narrowing is stated, not hidden.
// ---------------------------------------------------------------------------

import fp32_math;

// ---------------------------------------------------------------------------
// Set 1, binding 0 -- the destination. One packed R8G8B8A8 texel per pixel,
// row-major, `row_pitch_texels` apart. A BUFFER rather than a storage image
// because the destination is a CPU-readable linear allocation with a stride
// that kathost_frame already describes, and a storage image would add a layout
// transition and a copy to reach the same bytes.
// ---------------------------------------------------------------------------
[[vk::binding(0, 1)]] RWStructuredBuffer<uint> out_pixels;

struct GroundPlaneGpu {
    float3 normal;      // world, normalised
    float  offset;      // dot(p, normal) <= offset is solid (world/sdf.hpp)
    float3 base_color;  // the resolved material colour, already looked up
    uint   is_front;    // host-side precompute: dot(camPos, normal) > offset
    float3 grid_u;      // the plane's orthonormal basis, from ground_plane_basis()
    // WAS `float pad0`. Material::shading, 0 = lambert / 1 = unlit / 2 = emissive.
    // ⛔ A uint IN A SLOT THAT HELD PADDING: sizeof(GroundPlaneGpu) and every
    // field offset are UNCHANGED by this, which is the only property the
    // host-side mirror (background_pass.hpp's PlaneGpu) can rely on, since
    // nothing compares the two.
    uint   shading_mode;
    float3 grid_v;
    float  pad1;
};

[[vk::binding(1, 1)]] StructuredBuffer<GroundPlaneGpu> ground_planes;

struct BackgroundParams {
    // -- the per-frame camera hoist (see the header) ------------------------
    float3 cam_pos;
    float  inv_tan_half_fov;   // 1 / tan(fovY * 0.5), the CPU's `f`
    float3 right_world;
    float  aspect;             // width / height
    float3 up_world;
    uint   width;
    float3 forward_world;
    uint   height;

    // -- lighting (render/scene.hpp's Lighting) -----------------------------
    float3 sun_direction;
    float  sun_intensity;
    float3 sun_color;
    float  horizon_strength;
    float3 ambient_color;
    float  horizon_onset;
    float3 sky_zenith;
    uint   plane_count;
    float3 sky_horizon;
    uint   row_pitch_texels;

    // -- the overlay grid (render/target.hpp's GroundGridParams) ------------
    float3 grid_color;
    float  grid_spacing;
    float  grid_line_half_width;
    float  grid_width_growth;
    float  grid_fade_distance;
    uint   grid_enabled;
};

[[vk::binding(2, 1)]] ConstantBuffer<BackgroundParams> bg;

// ---------------------------------------------------------------------------
// render/scene.hpp's sky_gradient_color(), statement for statement. Elevation
// is the ray's normalised Y; `horizon_fraction` clamps to [0,1] so a ray that
// looks below the horizon gets the horizon colour rather than an extrapolation.
// ---------------------------------------------------------------------------
float3 sky_color(float3 ray_direction_world) {
    const float len = sqrt(dot(ray_direction_world, ray_direction_world));
    const float elevation = len > 0.0f ? ray_direction_world.y / len : 1.0f;
    const float horizon_fraction = clamp(1.0f - elevation, 0.0f, 1.0f);
    return bg.sky_zenith + (bg.sky_horizon - bg.sky_zenith) * horizon_fraction;
}

// render/scene.hpp -- distance from `c` to the nearest multiple of `spacing`.
// floor-based rather than fmod so it is exact at negative coordinates too.
float distance_to_nearest_line(float c, float spacing) {
    const float k = floor(c / spacing + 0.5f);
    return abs(c - k * spacing);
}

// ---------------------------------------------------------------------------
// render/scene.hpp's grid coverage. The half-width GROWS with view distance so
// a line stays roughly constant in screen space, and the whole grid fades with
// a rational falloff rather than an exponential -- cheaper, and it never
// reaches exactly zero, which is why `d >= hw` returns an EXACT zero first:
// the common case (a pixel not on a line) must be bit-identical ground colour.
// ---------------------------------------------------------------------------
float grid_coverage(float3 hit_world, float3 u, float3 v, float view_distance) {
    const float cu = dot(hit_world, u);
    const float cv = dot(hit_world, v);
    const float hw = bg.grid_line_half_width * (1.0f + view_distance * bg.grid_width_growth);
    const float du = distance_to_nearest_line(cu, bg.grid_spacing);
    const float dv = distance_to_nearest_line(cv, bg.grid_spacing);
    const float d = du < dv ? du : dv;
    if (d >= hw) return 0.0f;   // EXACT zero -- see above
    const float edge = 1.0f - d / hw;
    const float r = view_distance / bg.grid_fade_distance;
    const float fade = 1.0f / (1.0f + r * r);
    return edge * fade;
}

// render/scene.hpp's shade(), lambert half only -- the background pass never
// samples the shadow map for a ground plane, and raster_cpu DOES apply the
// static shadow to its analytic ground (apply_shadow/sample_shadow).
// ⛔ THIS SENTENCE USED TO SAY "STAGE 2 BRINGS IT ACROSS". STAGE 2 LANDED
// (cfd01879) AND DID NOT. Corrected rather than left standing, because a
// forward promise whose deadline has passed reads as a plan and is a false
// statement about the tree -- the declaration rotted while the behaviour
// stayed correct, which is the failure nothing here instruments for.
// ⚠ SO THIS IS A REAL, OPEN DIFFERENCE FROM THE CPU BACKGROUND, not a
// deferred one with an owner: with RenderOptions::shadows on and a static
// shadow map present, the CPU's ground is shadowed and this kernel's is not.
// It is COLOUR-ONLY, so compare_silhouettes() cannot see it either.
float3 shade_lambert(float3 base, float3 n_world) {
    const float n_dot_l = max(dot(n_world, bg.sun_direction), 0.0f);
    const float3 sun_term = bg.sun_color * (bg.sun_intensity * n_dot_l);
    return base * (sun_term + bg.ambient_color);
}

// render/scene.hpp's horizon fade -- a rational blend toward the sky with
// distance, 0 at the camera and approaching `strength` at range.
float3 fade_to_horizon(float3 ground_color, float3 sky, float view_distance) {
    if (bg.horizon_strength <= 0.0f || bg.horizon_onset <= 0.0f) return ground_color;
    const float r = view_distance / bg.horizon_onset;
    const float t = (r * r) / (1.0f + r * r);
    const float k = t * bg.horizon_strength;
    return ground_color + (sky - ground_color) * k;
}

// render/scene.hpp's encode_channel() -- clamp then round-half-away-from-zero
// to match std::lround, which rounds .5 AWAY from zero while HLSL/Slang
// `round()` rounds to even. floor(x + 0.5) is the exact std::lround behaviour
// for the non-negative values a clamped channel can hold.
uint encode_channel(float channel) {
    const float clamped = clamp(channel, 0.0f, 1.0f);
    return (uint)floor(clamped * 255.0f + 0.5f);
}

// ---------------------------------------------------------------------------
// PACKS B,G,R,X -- NOT R,G,B,A. render/target.hpp's PixelFormat has exactly ONE
// value, bgrx8, and raster_cpu.cpp writes it as px[0]=b, px[1]=g, px[2]=r,
// px[3]=kBgrxOpaqueByte at both of its pixel-write sites (:305, :1404). A `uint`
// stored to a buffer lands least-significant-byte-first on every device this
// engine targets, so the low 8 bits ARE px[0] -- which is BLUE.
//
// ⛔ THE FIRST VERSION OF THIS FUNCTION WAS NAMED pack_rgba AND PACKED RGBA,
// WHICH WOULD HAVE SWAPPED RED AND BLUE IN EVERY GPU FRAME. Found by reading the
// consumer while writing the host side, BEFORE this kernel ever ran -- it has
// never been compiled, so no measurement was wrong, only the code.
//
// ⭐⭐⭐ AND THE GUARD THIS PATH CHOSE CANNOT SEE IT. render/agreement.hpp's
// compare_silhouettes() is COVERAGE, NEVER COLOUR -- deliberately, because
// raster_cpu.cpp is fp64 and this kernel is fp32, so a colour comparison would be
// red on its first run for a reason nobody can fix. A CHANNEL SWAP CHANGES NO
// PIXEL'S COVERAGE: every sky pixel is still sky and every ground pixel still
// ground. AN INSTRUMENT CHOSEN FOR BEING INSENSITIVE TO AN EXPECTED DIFFERENCE IS
// ALSO INSENSITIVE TO A DEFECT WEARING THAT DIFFERENCE'S CLOTHES -- so the name
// stating the byte order is the only guard on this, and it is why the name changed
// rather than only the body.
// ---------------------------------------------------------------------------
uint pack_bgrx(float3 rgb) {
    return encode_channel(rgb.z) | (encode_channel(rgb.y) << 8) |
           (encode_channel(rgb.x) << 16) | (255u << 24);
}

[shader("compute")]
// ---------------------------------------------------------------------------
// ⛔ A FIXED LOCAL SIZE, AND THIS KERNEL DELIBERATELY DOES *NOT* JOIN THE
// WORKGROUP-SIZE VARIANT FAMILY. Register it with spade_slang_kernel(), never
// spade_slang_kernel_variants().
//
// The schedule kernels take -DSPADE_WG=<size> once per SPADE_SLANG_WORKGROUP_-
// SIZES entry so BackendDesc::workgroup_size can select a module that was
// actually compiled for it, and T9 pins that varying the size changes no
// result. THAT FAMILY EXISTS TO SERVE A PARITY PROPERTY OF THE PHYSICS PATH.
// This kernel has no parity obligation -- it cannot, because raster_cpu.cpp is
// fp64; see this file's header -- and no caller that varies its size.
//
// AND THE COST OF JOINING ANYWAY WOULD BE A STALE NAME, WHICH IS THIS
// NEIGHBOURHOOD'S OWN DOCUMENTED FAILURE MODE. tests/test_slang_layouts.cpp's
// SlangSpirv.EveryCompiledVariantIsScanned carries `kScheduleKernels = 12`,
// counting kernels that HAVE the full variant family. A render kernel joining
// it would make the count 13 while the NAME still said "schedule" -- and that
// same file records the identical species in miniature: "THE PROSE USED TO SAY
// NINE WHILE THE CONSTANT SAID 11 ... the number was maintained and the
// sentence describing it was not, so the sentence quietly became the less
// trustworthy of the two."
//
// ⚠ REGISTERING THIS KERNEL STILL MOVES A GUARDED NUMBER, BY DESIGN AND BY ONE.
// spade_slang_kernel() appends its name to CMake's global kernel property,
// which writes gen::kCompiledSpirvKernelCount, and the assert compares that
// against std::size(kSpirvModules). SO THE CMAKE REGISTRATION AND A NEW
// kSpirvModules[] ROW MUST LAND IN THE SAME COMMIT or the suite goes red --
// that is the gate doing exactly what it was built for, never an obstacle to
// route around.
// ---------------------------------------------------------------------------
[numthreads(64, 1, 1)]
void raster_background(uint3 tid : SV_DispatchThreadID) {
    const uint index = tid.x;
    const uint pixel_count = bg.width * bg.height;
    if (index >= pixel_count) {
        return;
    }
    const uint px = index % bg.width;
    const uint py = index / bg.width;

    // The CPU's per-column and per-row ray vectors, recomputed per pixel here
    // rather than precomputed into two arrays. That is the right trade on a
    // GPU -- two multiply-adds beat a dependent load of a value another thread
    // produced -- and it is arithmetically the same expression, not an
    // approximation of it. raster_cpu.cpp builds col_ray[x] and row_ray[y]
    // from the identical terms.
    const float x_ndc = 2.0f * ((float)px + 0.5f) / (float)bg.width - 1.0f;
    const float y_ndc = 1.0f - 2.0f * ((float)py + 0.5f) / (float)bg.height;
    const float3 ray = bg.forward_world +
                       bg.right_world * (x_ndc * bg.aspect / bg.inv_tan_half_fov) +
                       bg.up_world * (y_ndc / bg.inv_tan_half_fov);

    const float3 sky = sky_color(ray);

    // -- the analytic ground pass ------------------------------------------
    // Nearest FRONT-FACING plane the ray actually meets. `is_front` is the
    // host's per-plane precompute of raster_cpu's own front[] test: the camera
    // must be strictly on the side the normal points to, which is the same
    // outward-normal convention the mesh back-face cull uses. A plane the
    // camera sits at or below never contributes a hit -- the tessellated
    // ground disappears when viewed from below for the same reason.
    float best_t = -1.0f;
    uint best_plane = 0u;
    for (uint i = 0u; i < bg.plane_count; ++i) {
        const GroundPlaneGpu gp = ground_planes[i];
        if (gp.is_front == 0u) continue;

        const float denom = dot(gp.normal, ray);
        // Ray moving away from, or parallel to, the plane's front -- no hit.
        if (denom >= 0.0f) continue;

        const float t = (gp.offset - dot(gp.normal, bg.cam_pos)) / denom;
        if (t <= 0.0f) continue;
        if (best_t < 0.0f || t < best_t) {
            best_t = t;
            best_plane = i;
        }
    }

    float3 color = sky;
    if (best_t > 0.0f) {
        const GroundPlaneGpu gp = ground_planes[best_plane];
        const float3 hit = bg.cam_pos + ray * best_t;
        // View distance is the true metric distance to the hit, not `t` --
        // `ray` is not unit length (it carries the projection terms), so
        // scaling by its length is what makes the grid's width growth and
        // fade behave in metres rather than in ray-parameter units.
        const float view_distance = best_t * sqrt(dot(ray, ray));

        // shade_vertex_color()'s OWN EARLY RETURN, and note the TEST IS ON 1 AND 2,
        // NOT ON `!= 0`. The CPU is `if (shading == 1 || shading == 2) return base;`
        // and falls through to lambert for 0 OR ANY OTHER VALUE. Testing `== 0`
        // instead is its complement only across {0,1,2}, and diverges for anything
        // outside that set -- the GPU would echo raw base_color where the CPU
        // lamberts.
        // ⛔ AND "OUT OF RANGE CANNOT HAPPEN" IS NOT A FACT ABOUT THIS FILE.
        // world/builder.cpp:368 rejects shading >= kMaterialShadingCount, but that
        // validates a WorldDesc -- and a RenderScene can be HAND-BUILT, which every
        // fixture in this corpus does (render/scene.hpp says so), bypassing that
        // check entirely. So the guarantee lives in a different file at a different
        // layer and does not cover this path's real callers.
        // ⭐ A DIVERGENCE GROUNDED IN "SOMETHING UPSTREAM CURRENTLY PREVENTS IT" IS
        // A TASK, NOT A PROPERTY. Mirroring the reference's own test costs one
        // comparison and removes the dependency outright.
        float3 ground = (gp.shading_mode == 1u || gp.shading_mode == 2u)
                            ? gp.base_color
                            : shade_lambert(gp.base_color, gp.normal);
        if (bg.grid_enabled != 0u) {
            const float coverage = grid_coverage(hit, gp.grid_u, gp.grid_v, view_distance);
            if (coverage > 0.0f) {
                // ⛔ THE RAW grid_color, NEVER A SHADED ONE. raster_cpu.cpp blends
                // `grid->color` itself into the already-shaded ground
                // (`color + (grid->color - color) * cov`). An earlier draft of this
                // kernel lambert-shaded it first, which is A DIFFERENT FORMULA rather
                // than a different precision -- exactly the class of divergence that
                // fp32-vs-fp64 gets blamed for and does not cause.
                ground = ground + (bg.grid_color - ground) * coverage;
            }
        }
        color = fade_to_horizon(ground, sky, view_distance);
    }

    out_pixels[py * bg.row_pitch_texels + px] = pack_bgrx(color);
}
