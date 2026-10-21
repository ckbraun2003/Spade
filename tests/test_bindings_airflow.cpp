// The airflow descriptor set, set 1 (B0.3 of Core's B0 plan,
// docs/design/core/plans/2026-10-21-airflow-gpu-b0-plan.md): its own set, so
// set 0 is unchanged, with Physics' confirmed list at fixed indices that never
// move (bindings.slang section F). Read from the generated header, so a
// binding moved in bindings.slang fails here.

#include <gtest/gtest.h>

#include <cstdint>

#include "bindings.gen.hpp"

namespace gen = spade::compute::gen;

TEST(BindingsAirflow, TheAirflowSetIsSetOne) {
    EXPECT_EQ(gen::kSet_state, 0u);
    EXPECT_EQ(gen::kSet_airflow, 1u);
}

TEST(BindingsAirflow, TheStateSetIsUnchanged) {
    // Bindings 0..27 of set 0, as before set 1 existed; the mirror's census
    // (GpuStateMirrorTest.DescriptorSetBindsEveryRegistryBinding) binds them.
    EXPECT_EQ(gen::kBindingCount_state, 28u);
}

TEST(BindingsAirflow, AirflowBindingsAreTheConfirmedListFromZero) {
    EXPECT_EQ(gen::kBindingCount_airflow, 28u);
    const uint32_t list[] = {
        gen::kBinding_airflow_params,        gen::kBinding_airflow_u,
        gen::kBinding_airflow_v,             gen::kBinding_airflow_w,
        gen::kBinding_airflow_pressure,      gen::kBinding_airflow_next,
        gen::kBinding_airflow_advect,        gen::kBinding_airflow_increments,
        gen::kBinding_airflow_viscous_rhs,   gen::kBinding_airflow_rhs,
        gen::kBinding_airflow_residual,      gen::kBinding_airflow_multigrid,
        gen::kBinding_airflow_mg_weights,    gen::kBinding_airflow_apertures,
        gen::kBinding_airflow_wall_velocity, gen::kBinding_airflow_phi,
        gen::kBinding_airflow_face_tags,     gen::kBinding_airflow_wall_links,
        gen::kBinding_airflow_region_labels, gen::kBinding_airflow_region_table,
        gen::kBinding_airflow_pcg,           gen::kBinding_airflow_scalars,
        gen::kBinding_airflow_discs,         gen::kBinding_airflow_partials,
        gen::kBinding_airflow_health,        gen::kBinding_airflow_publish_v,
        gen::kBinding_airflow_publish_s,     gen::kBinding_airflow_wall_load,
    };
    for (uint32_t i = 0; i < 28; ++i) EXPECT_EQ(list[i], i) << "list position " << i;
}
