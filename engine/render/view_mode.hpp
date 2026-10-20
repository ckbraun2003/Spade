// ---------------------------------------------------------------------------
// render/view_mode.hpp -- what a view shows (spade::render).
//
// A view is a base mode plus flow layers over it: the editor's per-view
// "rendering schema" (Cameron's goal, 2026-10-19; the lead approved the
// selector the same day). supports() says, before a draw, whether a backend
// can draw a spec and, if not, why, in the very phrase the draw path refuses
// with. apply_view() builds a spec into a RenderScene and RenderOptions
// through flow_look and returns a legend per layer, its range resolved by the
// function both renderers draw with. A test draws every base and layer on
// both backends and fails if supports() and the draw ever disagree.
//
// The sensor bases, depth and normals (N13; docs/design/rendering/plans/
// 2026-10-20-sensor-views-design.md), draw the geometry as data, on both
// backends, and take no flow layers.
//
// Not yet: segmentation (N13's second part), and per-camera field channels
// (the backlog's "field channels for cameras", with RND-4's Vulkan technique).
// ---------------------------------------------------------------------------
#pragma once

#include <cstdint>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <vector>

#include <glm/vec3.hpp>

#include "core/error.hpp"
#include "core/flow_volume.hpp"
#include "render/field_layer.hpp"
#include "render/flow_lines.hpp"
#include "render/flow_render.hpp"
#include "render/scene.hpp"
#include "render/target.hpp"
#include "render/transfer_function.hpp"

namespace spade::render {

// The base: how the scene itself draws (RenderOptions::mode).
enum class ViewBase : uint32_t {
    shaded = 0,     // lit meshes, CSG marched per pixel (RS3), shadows, overlays
    wireframe = 1,  // unlit edges
    velocity = 2,   // each item by its speed (SL9c)
    exact_sdf = 3,  // the exact sphere-trace of the world's SDF: the CPU only, and no flow layers
    depth = 4,      // view-space depth in metres, coloured over ViewSpec::depth; no flow layers
    normals = 5,    // the unit world-space normal n as RGB (n + 1) / 2; no flow layers
};
inline constexpr uint32_t kViewBaseCount = 6;

// A flow layer over the base, from a published FlowVolume.
enum class FlowView : uint32_t {
    volume = 0,       // an absorbing volume of one quantity
    vortex_iso = 1,   // an iso-surface (lambda2-hat, Q-hat, |w|...) coloured by speed
    streamlines = 2,  // lines from seeds, coloured by speed
    slice = 3,        // a coloured plane of one quantity
    surface_cp = 4,   // C_p on the flow walls (RND-8)
};
inline constexpr uint32_t kFlowViewCount = 5;

// Stable names, for a menu and a saved view: "shaded", "vortex_iso".
[[nodiscard]] std::string_view view_base_name(ViewBase base) noexcept;
[[nodiscard]] std::string_view flow_view_name(FlowView view) noexcept;
[[nodiscard]] std::optional<ViewBase> view_base_from_name(std::string_view name) noexcept;
[[nodiscard]] std::optional<FlowView> flow_view_from_name(std::string_view name) noexcept;

// One flow layer and its parameters; each kind reads only its own.
struct FlowViewSpec {
    FlowView kind = FlowView::volume;
    FlowQuantity quantity = FlowQuantity::speed;  // volume and slice: what shows; vortex_iso: the iso's field
    ColourMapId colours = ColourMapId::viridis;
    // The map's range: of the quantity; of speed for vortex_iso and
    // streamlines; of C_p for surface_cp. A volume absorbs from 0
    // (volume_transfer()), so it reads only range_max.
    float range_min = 0.0f;
    float range_max = 1.0f;
    float iso_value = 0.0f;         // vortex_iso: the level, in the quantity's units
    float onset = 0.25f;            // volume: absorbing from this fraction of the range...
    float sigma = 10.0f;            // ...up to this, per metre, at its top
    FieldLayer plane{};             // slice: its geometry
    std::vector<glm::vec3> seeds;   // streamlines
    StreamlineOptions lines{};      // streamlines
    float q_ref = 0.0f;             // surface_cp: C_p = (p - p_ref) / q_ref
    float p_ref = 0.0f;
};

struct ViewSpec {
    ViewBase base = ViewBase::shaded;
    std::vector<FlowViewSpec> layers;
    DepthRange depth{};  // ViewBase::depth: its range and colours (0.1-20 m, viridis)
};

enum class ViewBackend : uint32_t { cpu = 0, gl = 1 };

// What the scene and the backend have that a view may need.
struct ViewInputs {
    bool has_volume = false;     // a published FlowVolume: every flow layer needs one
    bool has_walls = false;      // a draw item with flow_wall: surface_cp needs one
    bool has_sdf = false;        // scene.sdf: exact_sdf needs it
    bool gl_flow_ready = true;   // GlRenderer::flow_ready(): GL's flow programs are compiled
};

struct ViewSupport {
    bool drawable = false;
    // Empty when drawable; else a fixed phrase that the refusal from
    // apply_view() or the backend's draw contains verbatim, fit to show.
    std::string why_not;
};

// Whether `backend` can draw `spec` given `inputs`, and if not, why: the
// first refusal the draw path would give, in its own words. It answers for
// the inputs, the composition, each layer's colours and iso value, GL's march
// and compile wait, and the budget (kMaxFlowTransfers volume layers and
// kMaxIsoSurfaces vortex_iso layers, one FlowLayer between them). Left to the
// draw path, which refuses them by name: a slice's plane and the streamline
// options, which apply_view() checks against the volume, and on GL a volume
// past the device's 3D texture size.
[[nodiscard]] ViewSupport supports(const ViewSpec& spec, ViewBackend backend, const ViewInputs& inputs);

// Streamlines traced once per generation and reused (the lead, 2026-10-19).
// Keep one across a frame's views, and from frame to frame, and pass it to
// apply_view(): views of one FlowVolume::generation with the same seeds and
// options trace once, whatever their colours. A new generation drops what it
// holds; generation 0 (unnamed) is traced every time and never kept.
class ViewCache {
  public:
    // The lines from `seeds` under `options` through `volume`, traced or
    // reused; valid until the next call. Whatever trace_streamlines()
    // refuses, it refuses.
    [[nodiscard]] Result<std::span<const Streamline>> streamlines(const FlowVolume& volume,
                                                                  std::span<const glm::vec3> seeds,
                                                                  const StreamlineOptions& options);
    // How many times streamlines() has traced: for tests and profiling.
    [[nodiscard]] uint64_t traces() const noexcept { return traces_; }

  private:
    struct Entry {
        std::vector<glm::vec3> seeds;
        StreamlineOptions options;
        std::vector<Streamline> lines;
    };
    uint64_t generation_ = 0;
    std::vector<Entry> entries_;
    std::vector<Streamline> unnamed_;
    uint64_t traces_ = 0;
};

// One layer's colour scale, for a legend.
struct ViewLegend {
    FlowView layer = FlowView::volume;
    std::string quantity;  // "speed |u|", "lambda2-hat", "C_p"
    std::string unit;      // "m/s", "1/s", "Pa", or "" for none
    ColourMapId colours = ColourMapId::viridis;
    ResolvedRange range;   // as both renderers resolve it (resolve_range())
};

// The base's own key, beside the layers' legends: depth's range and colours,
// normals' axes, and, for a sensor base, the black of a pixel no surface
// covers. A base with no key (shaded, wireframe, velocity, exact_sdf) gives
// empty text.
struct BaseLegend {
    ViewBase base = ViewBase::shaded;
    std::string quantity;  // "depth (view z)", "surface normal", or ""
    std::string unit;      // "m", or ""
    ColourMapId colours = ColourMapId::viridis;  // depth
    ResolvedRange range;                         // depth: [near_m, far_m]
    std::string key;                             // normals: which colour is which axis
    bool black_is_no_surface = false;            // a sensor base: an uncovered pixel is black
};

// `spec`'s base legend. `scene` is for segmentation's labels (N13's second
// part); depth and normals read only the spec.
[[nodiscard]] BaseLegend base_legend(const ViewSpec& spec, const RenderScene& scene);

// Builds `spec` into `scene` and `options`: the base's mode, and the layers
// over `volume` (volume and vortex_iso merged into one FlowLayer; streamlines,
// slices and C_p, on walls only, through flow_look and cp_colours). The
// view's flow content REPLACES the scene's (flow and field layers, lines,
// points, surface colouring), so views over one world each apply their own.
// Returns a legend per layer, in spec order. Refused by name, with nothing
// changed, for what supports() names as apply_view's: a missing input, layers
// on exact_sdf, a second surface_cp, streamlines without seeds, a layer's
// colours. The budget and GL's own refusals come from the draw.
[[nodiscard]] Result<std::vector<ViewLegend>> apply_view(const ViewSpec& spec, const FlowVolume* volume,
                                                         RenderScene& scene, RenderOptions& options);
// The same, its streamlines through `cache`.
[[nodiscard]] Result<std::vector<ViewLegend>> apply_view(const ViewSpec& spec, const FlowVolume* volume,
                                                         RenderScene& scene, RenderOptions& options, ViewCache& cache);

}  // namespace spade::render
