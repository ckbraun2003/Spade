#include "render/view_mode.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <span>
#include <utility>

#include "render/flow_field.hpp"
#include "render/flow_look.hpp"

namespace spade::render {

namespace {

constexpr std::array<std::string_view, kViewBaseCount> kBaseNames = {"shaded", "wireframe", "velocity", "exact_sdf",
                                                                     "depth",  "normals"};
constexpr std::array<std::string_view, kFlowViewCount> kLayerNames = {"volume", "vortex_iso", "streamlines", "slice",
                                                                      "surface_cp"};

// apply_view()'s refusals; each is the whole of a why_not and appears in its
// refusal after "apply_view: ".
constexpr std::string_view kLayersOnSdf = "exact_sdf draws no flow layers";
constexpr std::string_view kNoSdf = "exact_sdf needs the world's SDF";
constexpr std::string_view kNoVolume = "a flow layer needs a published flow volume";
constexpr std::string_view kTwoCp = "a view colours its surfaces by one C_p layer";
constexpr std::string_view kNoWalls = "C_p needs a flow wall to colour";
constexpr std::string_view kNoSeeds = "streamlines need seeds";
// The backends' own words: GlRenderer::draw() and validate_flow_layer().
constexpr std::string_view kGlNoMarch = "raymarch is a CPU technique";
constexpr std::string_view kGlCompiling = "the flow shaders are still compiling";

// The transfer a layer's colours come from, built once for supports() and
// apply_view() alike: the volume's own, an iso's or the lines' speed, a
// slice's map, or C_p's pressure range.
[[nodiscard]] Result<TransferFunction> layer_transfer(const FlowViewSpec& s) {
    switch (s.kind) {
        case FlowView::volume:
            return volume_transfer(s.quantity, s.colours, s.range_max, s.onset, s.sigma);
        case FlowView::vortex_iso:
        case FlowView::streamlines:
            return colours_over(FlowQuantity::speed, s.colours, s.range_min, s.range_max);
        case FlowView::slice:
            return colours_over(s.quantity, s.colours, s.range_min, s.range_max);
        case FlowView::surface_cp:
            return cp_colours(s.q_ref, s.p_ref, s.range_min, s.range_max, s.colours);
    }
    return std::unexpected(Error{Code::invalid_argument, "its kind is not a FlowView"});
}

[[nodiscard]] std::string layer_label(std::size_t i, FlowView kind) {
    return "layer " + std::to_string(i) + " (" + std::string(flow_view_name(kind)) + "): ";
}

// The first refusal apply_view() gives, or empty: one function, so the two
// cannot disagree.
[[nodiscard]] std::string spec_refusal(const ViewSpec& spec, bool has_volume, bool has_walls, bool has_sdf) {
    if (static_cast<uint32_t>(spec.base) >= kViewBaseCount) return "its base is not a ViewBase";
    if (spec.base == ViewBase::exact_sdf && !spec.layers.empty()) return std::string(kLayersOnSdf);
    if (spec.base == ViewBase::exact_sdf && !has_sdf) return std::string(kNoSdf);
    if (!spec.layers.empty() && !has_volume) return std::string(kNoVolume);
    const auto cps = std::count_if(spec.layers.begin(), spec.layers.end(),
                                   [](const FlowViewSpec& s) { return s.kind == FlowView::surface_cp; });
    if (cps > 1) return std::string(kTwoCp);
    if (cps == 1 && !has_walls) return std::string(kNoWalls);
    for (std::size_t i = 0; i < spec.layers.size(); ++i) {
        const FlowViewSpec& s = spec.layers[i];
        if (static_cast<uint32_t>(s.kind) >= kFlowViewCount) return "layer " + std::to_string(i) + ": its kind is not a FlowView";
        if (s.kind == FlowView::streamlines && s.seeds.empty()) return layer_label(i, s.kind) + std::string(kNoSeeds);
        if (s.kind == FlowView::vortex_iso && !std::isfinite(s.iso_value)) {
            return layer_label(i, s.kind) + "the iso value is not finite";
        }
        auto tf = layer_transfer(s);
        if (!tf) return layer_label(i, s.kind) + tf.error().context;
        if (Result<void> valid = validate_transfer_function(*tf); !valid) {
            return layer_label(i, s.kind) + valid.error().context;
        }
    }
    return {};
}

[[nodiscard]] bool scene_has_walls(const RenderScene& scene) {
    const auto wall = [](const DrawItem& item) { return item.flow_wall; };
    return std::any_of(scene.statics.begin(), scene.statics.end(), wall) ||
           std::any_of(scene.dynamics.begin(), scene.dynamics.end(), wall);
}

struct Named {
    std::string_view name;
    std::string_view unit;
};

[[nodiscard]] Named quantity_named(FlowQuantity q) {
    switch (q) {
        case FlowQuantity::speed: return {"speed |u|", "m/s"};
        case FlowQuantity::u: return {"u", "m/s"};
        case FlowQuantity::v: return {"v", "m/s"};
        case FlowQuantity::w: return {"w", "m/s"};
        case FlowQuantity::dye: return {"dye", ""};
        case FlowQuantity::pressure: return {"gauge pressure", "Pa"};
        case FlowQuantity::vorticity: return {"vorticity |w|", "1/s"};
        case FlowQuantity::q_hat: return {"Q-hat", "1/s"};
        case FlowQuantity::lambda2_hat: return {"lambda2-hat", "1/s"};
    }
    return {"", ""};
}

}  // namespace

std::string_view view_base_name(ViewBase base) noexcept {
    const auto i = static_cast<uint32_t>(base);
    return i < kViewBaseCount ? kBaseNames[i] : std::string_view{};
}

std::string_view flow_view_name(FlowView view) noexcept {
    const auto i = static_cast<uint32_t>(view);
    return i < kFlowViewCount ? kLayerNames[i] : std::string_view{};
}

std::optional<ViewBase> view_base_from_name(std::string_view name) noexcept {
    for (uint32_t i = 0; i < kViewBaseCount; ++i) {
        if (kBaseNames[i] == name) return static_cast<ViewBase>(i);
    }
    return std::nullopt;
}

std::optional<FlowView> flow_view_from_name(std::string_view name) noexcept {
    for (uint32_t i = 0; i < kFlowViewCount; ++i) {
        if (kLayerNames[i] == name) return static_cast<FlowView>(i);
    }
    return std::nullopt;
}

ViewSupport supports(const ViewSpec& spec, ViewBackend backend, const ViewInputs& inputs) {
    if (std::string why = spec_refusal(spec, inputs.has_volume, inputs.has_walls, inputs.has_sdf); !why.empty()) {
        return ViewSupport{false, std::move(why)};
    }
    // Then the draw's, in the order it checks them.
    std::size_t transfers = 0, isos = 0;
    for (const FlowViewSpec& s : spec.layers) {
        transfers += s.kind == FlowView::volume ? 1u : 0u;
        isos += s.kind == FlowView::vortex_iso ? 1u : 0u;
    }
    if (backend == ViewBackend::gl) {
        if (spec.base == ViewBase::exact_sdf) return ViewSupport{false, std::string(kGlNoMarch)};
        if (transfers + isos > 0u && !inputs.gl_flow_ready) return ViewSupport{false, std::string(kGlCompiling)};
    }
    if (transfers > kMaxFlowTransfers || isos > kMaxIsoSurfaces) {
        // validate_flow_layer()'s words, which both backends refuse in.
        return ViewSupport{false, "it has " + std::to_string(transfers) + " transfer functions and " +
                                      std::to_string(isos) + " iso-surfaces; each may be 0 to " +
                                      std::to_string(kMaxFlowTransfers)};
    }
    return ViewSupport{true, {}};
}

Result<std::span<const Streamline>> ViewCache::streamlines(const FlowVolume& volume, std::span<const glm::vec3> seeds,
                                                           const StreamlineOptions& options) {
    if (volume.generation != generation_) {
        entries_.clear();
        generation_ = volume.generation;
    }
    if (volume.generation != 0u) {
        for (const Entry& entry : entries_) {
            if (entry.options == options && std::ranges::equal(entry.seeds, seeds)) {
                return std::span<const Streamline>(entry.lines);
            }
        }
    }
    auto lines = trace_streamlines(volume, seeds, options);
    if (!lines) return std::unexpected(lines.error());
    ++traces_;
    if (volume.generation == 0u) {
        unnamed_ = std::move(*lines);
        return std::span<const Streamline>(unnamed_);
    }
    entries_.push_back(Entry{std::vector<glm::vec3>(seeds.begin(), seeds.end()), options, std::move(*lines)});
    return std::span<const Streamline>(entries_.back().lines);
}

namespace {

[[nodiscard]] Result<std::vector<ViewLegend>> apply(const ViewSpec& spec, const FlowVolume* volume,
                                                    RenderScene& scene, RenderOptions& options, ViewCache* cache) {
    const bool has_sdf = scene.sdf != nullptr && !scene.sdf->empty();
    if (std::string why = spec_refusal(spec, volume != nullptr, scene_has_walls(scene), has_sdf); !why.empty()) {
        return std::unexpected(Error{Code::invalid_argument, "apply_view: " + why});
    }
    // Built aside, so a refusal from a builder changes nothing.
    RenderScene built;
    FlowLayer marched;
    marched.volume = volume;
    std::vector<ViewLegend> legends;
    legends.reserve(spec.layers.size());
    for (std::size_t i = 0; i < spec.layers.size(); ++i) {
        const FlowViewSpec& s = spec.layers[i];
        TransferFunction tf = *layer_transfer(s);  // spec_refusal() built it once already
        ViewLegend legend;
        legend.layer = s.kind;
        legend.colours = s.colours;
        legend.range = resolve_range(tf, volume->header);
        const Named named = quantity_named(tf.quantity);
        legend.quantity = named.name;
        legend.unit = named.unit;
        Result<void> added{};
        switch (s.kind) {
            case FlowView::volume:
                marched.transfers.push_back(std::move(tf));
                break;
            case FlowView::vortex_iso: {
                IsoSurface iso;
                iso.quantity = s.quantity;
                iso.value = s.iso_value;
                iso.colour_by = std::move(tf);
                marched.isos.push_back(std::move(iso));
                break;
            }
            case FlowView::streamlines:
                if (cache == nullptr) {
                    added = add_streamlines(built, *volume, std::span<const glm::vec3>(s.seeds), s.lines, tf);
                } else if (auto lines = cache->streamlines(*volume, std::span<const glm::vec3>(s.seeds), s.lines);
                           lines) {
                    add_traced_streamlines(built, volume->header, *lines, tf);
                } else {
                    added = std::unexpected(lines.error());
                }
                break;
            case FlowView::slice:
                added = add_slice(built, *volume, s.quantity, s.plane, s.colours, s.range_min, s.range_max);
                break;
            case FlowView::surface_cp:
                legend.quantity = "C_p";
                legend.unit = "";
                legend.range.min = (legend.range.min - s.p_ref) / s.q_ref;
                legend.range.max = (legend.range.max - s.p_ref) / s.q_ref;
                built.flow_surface_colouring = FlowSurfaceColouring{.volume = volume, .colours = std::move(tf),
                                                                    .walls_only = true};
                break;
        }
        if (!added) {
            return std::unexpected(Error{added.error().code, "apply_view: " + layer_label(i, s.kind) +
                                                                 added.error().context});
        }
        legends.push_back(std::move(legend));
    }
    if (!marched.transfers.empty() || !marched.isos.empty()) built.flow_layers.push_back(std::move(marched));
    // The view's flow content replaces the scene's.
    scene.flow_layers = std::move(built.flow_layers);
    scene.field_layers = std::move(built.field_layers);
    scene.flow_polylines = std::move(built.flow_polylines);
    scene.flow_points = FlowPoints{};
    scene.flow_surface_colouring = std::move(built.flow_surface_colouring);
    switch (spec.base) {
        case ViewBase::shaded: options.mode = DrawMode::shaded; break;
        case ViewBase::wireframe: options.mode = DrawMode::wireframe; break;
        case ViewBase::velocity: options.mode = DrawMode::velocity; break;
        case ViewBase::exact_sdf: options.mode = DrawMode::raymarch; break;
    }
    return legends;
}

}  // namespace

Result<std::vector<ViewLegend>> apply_view(const ViewSpec& spec, const FlowVolume* volume, RenderScene& scene,
                                           RenderOptions& options) {
    return apply(spec, volume, scene, options, nullptr);
}

Result<std::vector<ViewLegend>> apply_view(const ViewSpec& spec, const FlowVolume* volume, RenderScene& scene,
                                           RenderOptions& options, ViewCache& cache) {
    return apply(spec, volume, scene, options, &cache);
}

// RED: a stub until the green step.
BaseLegend base_legend(const ViewSpec& spec, const RenderScene&) { return BaseLegend{.base = spec.base}; }

}  // namespace spade::render
