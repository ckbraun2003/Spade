// The airflow descriptor set, set 1 (B0.3 of Core's B0 plan,
// docs/design/core/plans/2026-10-21-airflow-gpu-b0-plan.md): its own set, so
// set 0 is unchanged, with Physics' confirmed list at fixed indices that never
// move (bindings.slang section F). Read from the generated header, so a
// binding moved in bindings.slang fails here.

#include <gtest/gtest.h>

#include <algorithm>
#include <cstdint>
#include <vector>

#include "bindings.gen.hpp"

namespace gen = spade::compute::gen;

TEST(BindingsAirflow, TheAirflowSetIsSetOne) {
    EXPECT_EQ(gen::kSet_state, 0u);
    EXPECT_EQ(gen::kSet_airflow, 1u);
}

TEST(BindingsAirflow, TheStateSetIsUnchanged) {
    // Bindings 0..27 of set 0, as before set 1 existed, and 28, the contact
    // sets (L10); the mirror's census
    // (GpuStateMirrorTest.DescriptorSetBindsEveryRegistryBinding) binds them.
    // ... and 29, the translation lock's rows (module API stage 6), the first optional binding.
    EXPECT_EQ(gen::kBindingCount_state, 30u);
}

TEST(BindingsAirflow, AirflowBindingsAreTheConfirmedListFromZero) {
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

// Section F's number space is shared by three owners, each with its own range of
// binding numbers and its own count of DECLARED bindings (kBindingCount_airflow
// counts declared bindings, a descriptor-pool sizing input; nothing keys on the
// highest index, and sparse numbering is legal). An owner's change is a one-number
// edit to its own row:
//   airflow   0..27   the airflow kernels' (append-only list above)
//   stage 7   28..39  module API stage 7's appendices: 28 to 33 named, 34 to 39 spare
//   lbm       40..47  the LBM tier's: lbm_f, lbm_u declared; lbm_flags (42, L2) reserved
TEST(BindingsAirflow, EachOwnersDeclaredBindingsLieInItsOwnRange) {
    struct Owner {
        const char* name;
        uint32_t first;
        uint32_t last;
        uint32_t declared;  // the count this owner has declared now
        std::vector<uint32_t> bindings;
    };
    const Owner owners[] = {
        {"airflow", 0, 27, 28,
         {gen::kBinding_airflow_params,       gen::kBinding_airflow_u,           gen::kBinding_airflow_v,
          gen::kBinding_airflow_w,            gen::kBinding_airflow_pressure,    gen::kBinding_airflow_next,
          gen::kBinding_airflow_advect,       gen::kBinding_airflow_increments,  gen::kBinding_airflow_viscous_rhs,
          gen::kBinding_airflow_rhs,          gen::kBinding_airflow_residual,    gen::kBinding_airflow_multigrid,
          gen::kBinding_airflow_mg_weights,   gen::kBinding_airflow_apertures,   gen::kBinding_airflow_wall_velocity,
          gen::kBinding_airflow_phi,          gen::kBinding_airflow_face_tags,   gen::kBinding_airflow_wall_links,
          gen::kBinding_airflow_region_labels, gen::kBinding_airflow_region_table, gen::kBinding_airflow_pcg,
          gen::kBinding_airflow_scalars,      gen::kBinding_airflow_discs,       gen::kBinding_airflow_partials,
          gen::kBinding_airflow_health,       gen::kBinding_airflow_publish_v,   gen::kBinding_airflow_publish_s,
          gen::kBinding_airflow_wall_load}},
        {"stage 7", 28, 39, 1, {gen::kBinding_airflow_config}},
        {"lbm", 40, 47, 2, {gen::kBinding_lbm_f, gen::kBinding_lbm_u}},
    };
    uint32_t total = 0;
    uint32_t previous_last = 0;
    for (const Owner& o : owners) {
        EXPECT_LT(o.first, o.last) << o.name;
        if (total != 0 || o.first != 0) {
            EXPECT_EQ(o.first, previous_last + 1) << o.name << ": ranges are consecutive";
        }
        previous_last = o.last;
        // (a) every declared binding lies in this owner's range, once
        for (const uint32_t b : o.bindings) {
            EXPECT_GE(b, o.first) << o.name << " binding " << b;
            EXPECT_LE(b, o.last) << o.name << " binding " << b;
        }
        std::vector<uint32_t> sorted = o.bindings;
        std::sort(sorted.begin(), sorted.end());
        EXPECT_EQ(std::adjacent_find(sorted.begin(), sorted.end()), sorted.end()) << o.name << ": a binding twice";
        // (b) the declared count is the row's
        EXPECT_EQ(o.bindings.size(), std::size_t{o.declared}) << o.name;
        total += o.declared;
    }
    // (c) the rows sum to the generated count, so a declared binding no row lists fails here
    EXPECT_EQ(total, gen::kBindingCount_airflow);
    EXPECT_EQ(gen::kBinding_lbm_f, 40u);
    EXPECT_EQ(gen::kBinding_lbm_u, 41u);
}
