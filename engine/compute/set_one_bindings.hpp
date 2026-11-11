// compute/set_one_bindings.hpp -- the names of set 1's bindings (module API stage 7, Task C2; plan decision 2: names are the
// contract). Vulkan-free, beside optional_bindings.hpp, so sim/ reads the names and the mirror binds them.
//
// A walk array (StepShape::arrays) or a device-only scratch (ScratchShape) whose NAME is in this table is bound at that
// set-1 binding, in whichever set-1 descriptor set a kernel's subset needs (compute/layout_classes.hpp). The numbers are
// bindings.slang section F's, written here as plain constants so this header is Vulkan-free in the full sense: a SPADE_VULKAN=OFF
// build has no generated headers (VulkanOff.NoFileAnOffTargetCompilesIncludesAGeneratedHeaderUnguarded), and sim/ includes this
// file. Where the generated header exists (every Vulkan build) the static_asserts at the foot tie each number to the generated
// gen::kBinding_airflow_* constant, as spike/airflow/chain_runner.hpp ties kAirflowSetBuffers. The table lists the 28 airflow names, bindings 0 to 27.
// The stage 7 appendices are here as their batch declares them (28 points, 34 centred); the configuration image (33) is bound by
// StepShape::config_binding, so it is not a name. The LBM's (40 to 47) have their own lists.
#pragma once

#include <array>
#include <cstdint>
#include <iterator>
#include <string_view>

#include "compute/backend.hpp"

namespace spade::compute {

// The numbers of set 1 and set 0 that Vulkan-free code needs (the generated gen:: constants, which exist only in a Vulkan build, are
// asserted equal to them below).
inline constexpr uint32_t kStateBindingCount = 30;      // set 0's bindings (gen::kBindingCount_state)
inline constexpr uint32_t kAirflowConfigBinding = 33;   // the module configuration image's set-1 binding (gen::kBinding_airflow_config)
inline constexpr uint32_t kAirflowWallLoadBinding = 27;  // the last of section F's 28 airflow bindings
// The stage 7 appendices and the LBM twin's, for layout_classes.hpp's is_declared_set1 (each tied to its gen:: constant below).
inline constexpr uint32_t kAirflowPointsBinding = 28;   // the per-point rows (batch D)
inline constexpr uint32_t kAirflowCentredBinding = 34;  // the cell-centred field (batch D)
inline constexpr uint32_t kLbmFBinding = 40;
inline constexpr uint32_t kLbmUBinding = 41;
inline constexpr uint32_t kAirflowDeclaredCount = 33;   // bindings section F declares (gen::kBindingCount_airflow)

struct SetOneBinding {
    std::string_view name;
    uint32_t binding;
};

// X(name, number): section F's airflow bindings 0 to 27, in binding order.
#define SPADE_SET_ONE_BINDINGS(X) \
    X(airflow_params, 0) \
    X(airflow_u, 1) \
    X(airflow_v, 2) \
    X(airflow_w, 3) \
    X(airflow_pressure, 4) \
    X(airflow_next, 5) \
    X(airflow_advect, 6) \
    X(airflow_increments, 7) \
    X(airflow_viscous_rhs, 8) \
    X(airflow_rhs, 9) \
    X(airflow_residual, 10) \
    X(airflow_multigrid, 11) \
    X(airflow_mg_weights, 12) \
    X(airflow_apertures, 13) \
    X(airflow_wall_velocity, 14) \
    X(airflow_phi, 15) \
    X(airflow_face_tags, 16) \
    X(airflow_wall_links, 17) \
    X(airflow_region_labels, 18) \
    X(airflow_region_table, 19) \
    X(airflow_pcg, 20) \
    X(airflow_scalars, 21) \
    X(airflow_discs, 22) \
    X(airflow_partials, 23) \
    X(airflow_health, 24) \
    X(airflow_publish_v, 25) \
    X(airflow_publish_s, 26) \
    X(airflow_wall_load, 27) \
    /* The stage 7 appendices a module binds by name (batch D): the per-point rows and the cell-centred field. */ \
    X(airflow_points, 28) \
    X(airflow_centred, 34)

inline constexpr SetOneBinding kSetOneBindings[] = {
#define SPADE_SET_ONE_ROW(name, number) {#name, number},
    SPADE_SET_ONE_BINDINGS(SPADE_SET_ONE_ROW)
#undef SPADE_SET_ONE_ROW
};

// Section F's bindings 0 to airflow_wall_load come first, in binding order: entry i is binding i. The two appendices follow.
static_assert(std::size(kSetOneBindings) == kAirflowWallLoadBinding + 1 + 2,
              "kSetOneBindings lists bindings.slang section F's airflow bindings, 0 to airflow_wall_load, then points and centred");
static_assert(
    [] {
        for (uint32_t i = 0; i <= kAirflowWallLoadBinding; ++i) {
            if (kSetOneBindings[i].binding != i) return false;
        }
        return true;
    }(),
    "kSetOneBindings is in binding order: entry i is section F's binding i, for 0 to airflow_wall_load");

// The set-1 binding named `name`, or kNoBinding when the table does not hold it.
[[nodiscard]] constexpr uint32_t set_one_binding_of(std::string_view name) noexcept {
    for (const SetOneBinding& row : kSetOneBindings) {
        if (row.name == name) return row.binding;
    }
    return kNoBinding;
}

}  // namespace spade::compute

// WHERE THE GENERATED HEADER EXISTS (every SPADE_VULKAN=ON build, and gcc-check's), each number above is held to the constant
// bindings.slang generated for it: a renumbered binding fails to compile here, not on a device.
#if __has_include("bindings.gen.hpp")
#include "bindings.gen.hpp"
namespace spade::compute {
static_assert(kStateBindingCount == gen::kBindingCount_state, "set 0's binding count is bindings.slang's");
static_assert(kAirflowConfigBinding == gen::kBinding_airflow_config, "the config image's binding is bindings.slang's");
static_assert(kAirflowWallLoadBinding == gen::kBinding_airflow_wall_load, "section F's last airflow binding is bindings.slang's");
static_assert(kAirflowPointsBinding == gen::kBinding_airflow_points, "airflow_points is bindings.slang's number");
static_assert(kAirflowCentredBinding == gen::kBinding_airflow_centred, "airflow_centred is bindings.slang's number");
static_assert(kLbmFBinding == gen::kBinding_lbm_f && kLbmUBinding == gen::kBinding_lbm_u, "the LBM bindings are bindings.slang's");
static_assert(kAirflowDeclaredCount == gen::kBindingCount_airflow, "the declared airflow binding count is bindings.slang's");
#define SPADE_SET_ONE_ASSERT(name, number) \
    static_assert(gen::kBinding_##name == number, "kSetOneBindings: " #name " is not bindings.slang's number");
SPADE_SET_ONE_BINDINGS(SPADE_SET_ONE_ASSERT)
#undef SPADE_SET_ONE_ASSERT
}  // namespace spade::compute
#endif
#undef SPADE_SET_ONE_BINDINGS
