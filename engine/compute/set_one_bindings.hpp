// compute/set_one_bindings.hpp -- the names of set 1's bindings (module API stage 7, Task C2; plan decision 2: names are the
// contract). Vulkan-free, beside optional_bindings.hpp, so sim/ reads the names and the mirror binds them.
//
// A walk array (StepShape::arrays) or a device-only scratch (ScratchShape) whose NAME is in this table is bound at that
// set-1 binding, in whichever set-1 descriptor set a kernel's subset needs (compute/layout_classes.hpp). The numbers are
// the generated gen::kBinding_airflow_* constants of bindings.slang section F; the static_asserts below tie the table to
// them, as spike/airflow/chain_runner.hpp ties kAirflowSetBuffers. The table lists the 28 airflow names, bindings 0 to 27.
// The stage 7 appendices (28 to 33) are not here: the configuration image is bound by StepShape::config_binding, and the
// others by the batch that declares each row. The LBM's (40 to 47) have their own lists.
#pragma once

#include <array>
#include <cstdint>
#include <iterator>
#include <string_view>

#include "bindings.gen.hpp"
#include "compute/backend.hpp"

namespace spade::compute {

struct SetOneBinding {
    std::string_view name;
    uint32_t binding;
};

inline constexpr SetOneBinding kSetOneBindings[] = {
    {"airflow_params", gen::kBinding_airflow_params},
    {"airflow_u", gen::kBinding_airflow_u},
    {"airflow_v", gen::kBinding_airflow_v},
    {"airflow_w", gen::kBinding_airflow_w},
    {"airflow_pressure", gen::kBinding_airflow_pressure},
    {"airflow_next", gen::kBinding_airflow_next},
    {"airflow_advect", gen::kBinding_airflow_advect},
    {"airflow_increments", gen::kBinding_airflow_increments},
    {"airflow_viscous_rhs", gen::kBinding_airflow_viscous_rhs},
    {"airflow_rhs", gen::kBinding_airflow_rhs},
    {"airflow_residual", gen::kBinding_airflow_residual},
    {"airflow_multigrid", gen::kBinding_airflow_multigrid},
    {"airflow_mg_weights", gen::kBinding_airflow_mg_weights},
    {"airflow_apertures", gen::kBinding_airflow_apertures},
    {"airflow_wall_velocity", gen::kBinding_airflow_wall_velocity},
    {"airflow_phi", gen::kBinding_airflow_phi},
    {"airflow_face_tags", gen::kBinding_airflow_face_tags},
    {"airflow_wall_links", gen::kBinding_airflow_wall_links},
    {"airflow_region_labels", gen::kBinding_airflow_region_labels},
    {"airflow_region_table", gen::kBinding_airflow_region_table},
    {"airflow_pcg", gen::kBinding_airflow_pcg},
    {"airflow_scalars", gen::kBinding_airflow_scalars},
    {"airflow_discs", gen::kBinding_airflow_discs},
    {"airflow_partials", gen::kBinding_airflow_partials},
    {"airflow_health", gen::kBinding_airflow_health},
    {"airflow_publish_v", gen::kBinding_airflow_publish_v},
    {"airflow_publish_s", gen::kBinding_airflow_publish_s},
    {"airflow_wall_load", gen::kBinding_airflow_wall_load},
};

// Section F's bindings 0 to airflow_wall_load, in binding order: entry i is binding i.
static_assert(std::size(kSetOneBindings) == gen::kBinding_airflow_wall_load + 1,
              "kSetOneBindings lists bindings.slang section F's airflow bindings, 0 to airflow_wall_load");
static_assert(
    [] {
        for (uint32_t i = 0; i < std::size(kSetOneBindings); ++i) {
            if (kSetOneBindings[i].binding != i) return false;
        }
        return true;
    }(),
    "kSetOneBindings is in binding order: entry i is section F's binding i");

// The set-1 binding named `name`, or kNoBinding when the table does not hold it.
[[nodiscard]] constexpr uint32_t set_one_binding_of(std::string_view name) noexcept {
    for (const SetOneBinding& row : kSetOneBindings) {
        if (row.name == name) return row.binding;
    }
    return kNoBinding;
}

}  // namespace spade::compute
