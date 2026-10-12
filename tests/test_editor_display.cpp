// How the inspector shows a model's parameters (editor plan Task 10b, the
// lead's review of the first stills).
//
// Common fields read as a person would name them, with the unit in the label
// so the number in the field is never cut off; the internals (the schema id,
// the version, the visual's path, the principal axes as a raw quaternion) are
// folded into Advanced (§13's progressive disclosure). A value that rounds to
// zero shows as 0, never -0.

#include <gtest/gtest.h>

#include <bit>
#include <cmath>
#include <cstdint>
#include <limits>
#include <string>

#include "../sandbox/editor_display.hpp"
#include "../sandbox/editor_model_params.hpp"

namespace {

namespace ed = spade::sandbox::editor;
using E = spade::vehicles::ModelIssue::Element;

}  // namespace

TEST(EditorDisplay, TheModelsCommonFieldsHaveFriendlyNamesWithUnits) {
    EXPECT_EQ(ed::param_display(E::model, "body.mass").label, "Mass (kg)");
    EXPECT_EQ(ed::param_display(E::model, "body.inertia_diag").label, "Inertia (kg\xC2\xB7m\xC2\xB2)");
    EXPECT_EQ(ed::param_display(E::model, "com_offset").label, "Centre of mass (m)");
    EXPECT_EQ(ed::param_display(E::model, "design_to_principal").label, "Principal axes");
    EXPECT_FALSE(ed::param_display(E::model, "body.mass").advanced);
    EXPECT_FALSE(ed::param_display(E::model, "design_to_principal").advanced);
    EXPECT_EQ(ed::param_display(E::rotor, "thrust_coeff").label, "Thrust coefficient (N\xC2\xB7s\xC2\xB2)");
}

TEST(EditorDisplay, TheInternalsAreAdvanced) {
    for (const char* field : {"param_schema_id", "version", "visual_ref"}) {
        EXPECT_TRUE(ed::param_display(E::model, field).advanced) << field;
    }
}

TEST(EditorDisplay, EveryParameterHasAFriendlyLabel) {
    for (const ed::ModelParam& p : ed::model_params()) {
        const ed::ParamDisplay d = ed::param_display(p.element, p.field);
        EXPECT_FALSE(d.label.empty()) << p.field;
        EXPECT_NE(d.label, std::string(p.field)) << p.field;
    }
}

TEST(EditorDisplay, AValueThatRoundsToZeroShowsAsZero) {
    EXPECT_EQ(std::bit_cast<uint32_t>(ed::shown(-0.0f, 1)), 0u);
    EXPECT_EQ(std::bit_cast<uint32_t>(ed::shown(-0.04f, 1)), 0u);  // printed "-0.0" at one decimal
    EXPECT_EQ(ed::shown(-0.06f, 1), -0.06f);                       // "-0.1": a real value, kept
    EXPECT_EQ(ed::shown(12.5f, 1), 12.5f);
    EXPECT_EQ(std::bit_cast<uint32_t>(ed::shown(-0.0004f, 3)), 0u);
    EXPECT_TRUE(std::isnan(ed::shown(std::numeric_limits<float>::quiet_NaN(), 1)));
}
