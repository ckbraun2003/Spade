// tests/view_mode_cases.hpp -- the view specs both backends' parity tests
// draw (render/view_mode.hpp; the lead's condition, 2026-10-19): every base,
// every flow layer, compositions, every missing input, and one past each
// budget. expect_view_parity() runs a backend's draw path on each and fails
// if supports() said otherwise, or named another reason.

#pragma once

#include <cmath>
#include <functional>
#include <string>
#include <vector>

#include <gtest/gtest.h>
#include <glm/glm.hpp>

#include "flow_synthetic.hpp"
#include "render/scene.hpp"
#include "render/tessellate.hpp"
#include "render/view_mode.hpp"
#include "world/sdf.hpp"

namespace spade::test {

// A 2 m box of air: speed 1 + x along +x, pressure 10 y, and a lambda2-hat
// well (to -4 1/s) about the centre, so every layer has something to show.
[[nodiscard]] inline FlowVolume view_mode_volume() {
    return make_flow_volume(glm::vec3(-1.0f), 0.125f, 16, 16, 16, [](glm::vec3 p) {
        FlowChannels c{};
        c[kFlowU] = 1.0f + p.x;
        c[kFlowPressure] = 10.0f * p.y;
        c[kFlowLambda2Hat] = -4.0f * std::max(0.0f, 1.0f - glm::length(p) / 0.6f);
        return c;
    });
}

// The world's SDF for exact_sdf: a sphere of 0.3 m.
[[nodiscard]] inline SdfProgram view_mode_sdf() {
    SdfProgram p;
    p.transforms.push_back(SdfTransform{});
    SdfNode sphere{};
    sphere.kind = static_cast<uint32_t>(SdfPrim::sphere);
    sphere.params = glm::vec4(0.3f, 0.0f, 0.0f, 0.0f);
    p.nodes.push_back(sphere);
    return p;
}

// A box of 0.4 m at the centre, a flow wall when `walls`; the SDF when `sdf`.
[[nodiscard]] inline render::RenderScene view_mode_scene(bool walls, const SdfProgram* sdf) {
    render::RenderScene scene;
    scene.bounds = render::Aabb{.min = glm::vec3(-2.0f), .max = glm::vec3(2.0f)};
    scene.materials = {render::Material{}};
    auto box = render::tessellate_primitive(SdfPrim::box, glm::vec4(0.2f, 0.2f, 0.2f, 0.0f), scene.bounds);
    EXPECT_TRUE(box.has_value());
    if (box) scene.meshes.push_back(std::move(*box));
    render::DrawItem item;
    item.mesh_index = 0;
    item.flow_wall = walls;
    scene.statics.push_back(item);
    scene.sdf = sdf;
    return scene;
}

[[nodiscard]] inline render::FlowViewSpec layer(render::FlowView kind) {
    render::FlowViewSpec s;
    s.kind = kind;
    switch (kind) {
        case render::FlowView::volume:
            s.quantity = render::FlowQuantity::speed;
            s.range_max = 2.0f;
            break;
        case render::FlowView::vortex_iso:
            s.quantity = render::FlowQuantity::lambda2_hat;
            s.iso_value = -2.0f;
            s.range_max = 2.0f;
            break;
        case render::FlowView::streamlines:
            s.seeds = {glm::vec3(-0.8f, 0.0f, 0.0f), glm::vec3(-0.8f, 0.3f, 0.0f)};
            s.range_max = 2.0f;
            break;
        case render::FlowView::slice:
            s.quantity = render::FlowQuantity::pressure;
            s.plane.width = 1.5f;
            s.plane.height = 1.5f;
            s.plane.cells_u = 8;
            s.plane.cells_v = 8;
            s.range_min = -10.0f;
            s.range_max = 10.0f;
            break;
        case render::FlowView::surface_cp:
            s.q_ref = 5.0f;
            s.range_min = -1.0f;
            s.range_max = 1.0f;
            break;
    }
    return s;
}

struct ViewCase {
    std::string name;
    render::ViewSpec spec;
    render::ViewInputs inputs;
};

[[nodiscard]] inline std::vector<ViewCase> view_cases() {
    using render::FlowView;
    using render::ViewBase;
    const render::ViewInputs all{.has_volume = true, .has_walls = true, .has_sdf = true, .gl_flow_ready = true};
    std::vector<ViewCase> cases;
    for (uint32_t b = 0; b < render::kViewBaseCount; ++b) {
        const auto base = static_cast<ViewBase>(b);
        cases.push_back({"base " + std::to_string(b), render::ViewSpec{base, {}}, all});
    }
    for (uint32_t v = 0; v < render::kFlowViewCount; ++v) {
        cases.push_back({"layer " + std::to_string(v), render::ViewSpec{ViewBase::shaded, {layer(static_cast<FlowView>(v))}},
                         all});
    }
    cases.push_back({"iso with lines", {ViewBase::shaded, {layer(FlowView::vortex_iso), layer(FlowView::streamlines)}}, all});
    cases.push_back({"slice and volume over wireframe", {ViewBase::wireframe, {layer(FlowView::slice), layer(FlowView::volume)}}, all});
    cases.push_back({"the full budget", {ViewBase::shaded, {}}, all});
    for (int i = 0; i < 4; ++i) cases.back().spec.layers.push_back(layer(FlowView::volume));
    for (int i = 0; i < 4; ++i) cases.back().spec.layers.push_back(layer(FlowView::vortex_iso));
    cases.back().spec.layers.push_back(layer(FlowView::surface_cp));

    render::ViewInputs no_volume = all;
    no_volume.has_volume = false;
    cases.push_back({"a layer without a volume", {ViewBase::shaded, {layer(FlowView::slice)}}, no_volume});
    render::ViewInputs no_walls = all;
    no_walls.has_walls = false;
    cases.push_back({"C_p without walls", {ViewBase::shaded, {layer(FlowView::surface_cp)}}, no_walls});
    render::ViewInputs no_sdf = all;
    no_sdf.has_sdf = false;
    cases.push_back({"exact_sdf without an SDF", {ViewBase::exact_sdf, {}}, no_sdf});
    cases.push_back({"exact_sdf with a layer", {ViewBase::exact_sdf, {layer(FlowView::slice)}}, all});
    cases.push_back({"depth with a layer", {ViewBase::depth, {layer(FlowView::volume)}}, all});
    cases.push_back({"normals with a layer", {ViewBase::normals, {layer(FlowView::streamlines)}}, all});
    cases.push_back({"depth over an empty range", {ViewBase::depth, {}}, all});
    cases.back().spec.depth = render::DepthRange{.near_m = 4.0f, .far_m = 4.0f};
    cases.push_back({"two C_p", {ViewBase::shaded, {layer(FlowView::surface_cp), layer(FlowView::surface_cp)}}, all});
    render::FlowViewSpec seedless = layer(FlowView::streamlines);
    seedless.seeds.clear();
    cases.push_back({"streamlines without seeds", {ViewBase::shaded, {seedless}}, all});
    cases.push_back({"a fifth iso", {ViewBase::shaded, {}}, all});
    for (int i = 0; i < 5; ++i) cases.back().spec.layers.push_back(layer(FlowView::vortex_iso));
    cases.push_back({"a fifth transfer", {ViewBase::shaded, {}}, all});
    for (int i = 0; i < 5; ++i) cases.back().spec.layers.push_back(layer(FlowView::volume));
    return cases;
}

// The draw path for one case: apply_view() on the case's scene, then the
// backend's `draw`; the first refusal's message, or empty when it drew.
using ViewDraw = std::function<Result<void>(const render::RenderScene&, const render::RenderOptions&)>;

[[nodiscard]] inline std::string draw_path(const ViewCase& c, const FlowVolume& volume, const SdfProgram& sdf,
                                           const ViewDraw& draw) {
    render::RenderScene scene = view_mode_scene(c.inputs.has_walls, c.inputs.has_sdf ? &sdf : nullptr);
    render::RenderOptions options;
    options.overlays = false;
    auto applied = render::apply_view(c.spec, c.inputs.has_volume ? &volume : nullptr, scene, options);
    if (!applied) return applied.error().context.empty() ? "(refused without a message)" : applied.error().context;
    auto drawn = draw(scene, options);
    if (!drawn) return drawn.error().context.empty() ? "(refused without a message)" : drawn.error().context;
    return {};
}

// For every case: supports() says drawable exactly when the draw path drew,
// and a refusal names supports()'s reason verbatim.
inline void expect_view_parity(render::ViewBackend backend, const ViewDraw& draw) {
    const FlowVolume volume = view_mode_volume();
    const SdfProgram sdf = view_mode_sdf();
    std::size_t drawn = 0, refused = 0;
    for (const ViewCase& c : view_cases()) {
        const render::ViewSupport said = render::supports(c.spec, backend, c.inputs);
        const std::string refusal = draw_path(c, volume, sdf, draw);
        EXPECT_EQ(said.drawable, refusal.empty())
            << c.name << ": supports() said " << (said.drawable ? "drawable" : "not: " + said.why_not)
            << ", the draw " << (refusal.empty() ? "drew" : "refused: " + refusal);
        if (!said.drawable && !refusal.empty()) {
            EXPECT_FALSE(said.why_not.empty()) << c.name;
            EXPECT_NE(refusal.find(said.why_not), std::string::npos)
                << c.name << ": supports() named \"" << said.why_not << "\"; the draw refused: " << refusal;
        }
        (refusal.empty() ? drawn : refused) += 1;
    }
    EXPECT_GT(drawn, 8u);
    EXPECT_GT(refused, 6u);
}

}  // namespace spade::test
