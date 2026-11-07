// test_spirv_pin.cpp -- every existing kernel's SPIR-V is byte-identical across the stage 7 variant-build change
// (module API stage 7, Task B1; the lead's pin of 2026-11-07).
//
// Stage 7 adds SPADE_WG_X/Y/Z defines to the variant compile (cmake/SpadeSlang.cmake) so a grid kernel can be a 3D
// workgroup. A define a 1D kernel never reads must not move a byte of its SPIR-V, or a standard kernel's arithmetic
// could change silently. This table is FNV-1a 64 over each compiled variant's bytes as built from master bd207ee8,
// before the change; the test hashes what is embedded now. A PIN: green at red, and it must stay green. A kernel that
// is changed on purpose in a later task re-records its own rows in that task's commit, with the reason in the message.
#include <gtest/gtest.h>

#include <cstddef>
#include <cstdint>
#include <string>
#include <string_view>

#include "compute/spirv_variants.hpp"
#include "airflow_actuator_gather.spv.gen.hpp"
#include "airflow_add_increments.spv.gen.hpp"
#include "airflow_add_one.spv.gen.hpp"
#include "airflow_advection.spv.gen.hpp"
#include "airflow_copy.spv.gen.hpp"
#include "airflow_divergence.spv.gen.hpp"
#include "airflow_gradient_subtract.spv.gen.hpp"
#include "airflow_mg_prolong.spv.gen.hpp"
#include "airflow_mg_residual.spv.gen.hpp"
#include "airflow_mg_restrict.spv.gen.hpp"
#include "airflow_mg_restrict_full.spv.gen.hpp"
#include "airflow_mg_smooth.spv.gen.hpp"
#include "airflow_pack_f16.spv.gen.hpp"
#include "airflow_pcg_apply.spv.gen.hpp"
#include "airflow_pcg_measure.spv.gen.hpp"
#include "airflow_pcg_scalars.spv.gen.hpp"
#include "airflow_pcg_update.spv.gen.hpp"
#include "airflow_plan_a_measure.spv.gen.hpp"
#include "airflow_publish_fields.spv.gen.hpp"
#include "airflow_region_mean.spv.gen.hpp"
#include "airflow_remove_mean.spv.gen.hpp"
#include "airflow_solid_divergence.spv.gen.hpp"
#include "airflow_solid_gradient.spv.gen.hpp"
#include "airflow_solid_velocities.spv.gen.hpp"
#include "airflow_tile_dot.spv.gen.hpp"
#include "airflow_tile_sum.spv.gen.hpp"
#include "airflow_tile_sum_probe.spv.gen.hpp"
#include "airflow_velocity_bc.spv.gen.hpp"
#include "airflow_viscous.spv.gen.hpp"
#include "airflow_wall_load.spv.gen.hpp"
#include "collision_dynamic.spv.gen.hpp"
#include "collision_fill.spv.gen.hpp"
#include "collision_gather.spv.gen.hpp"
#include "collision_static.spv.gen.hpp"
#include "denorm_probe.spv.gen.hpp"
#include "field_dryden.spv.gen.hpp"
#include "field_environment.spv.gen.hpp"
#include "forces_drag.spv.gen.hpp"
#include "fp32_math_probe.spv.gen.hpp"
#include "grid_build.spv.gen.hpp"
#include "grid_sort.spv.gen.hpp"
#include "integrate.spv.gen.hpp"
#include "medium_update.spv.gen.hpp"
#include "rotors.spv.gen.hpp"
#include "sensor_gnss.spv.gen.hpp"
#include "sensor_imu.spv.gen.hpp"

namespace {

using spade::compute::SpirvVariantSet;

[[nodiscard]] uint64_t fnv1a64_words(const uint32_t* words, std::size_t count) {
    uint64_t h = 0xcbf29ce484222325ull;
    for (std::size_t i = 0; i < count; ++i) {
        for (int b = 0; b < 4; ++b) {
            h ^= static_cast<uint8_t>(words[i] >> (8 * b));
            h *= 0x100000001b3ull;
        }
    }
    return h;
}

struct Row {
    const SpirvVariantSet* set;
    const char* name;
    uint32_t workgroup_size;
    uint64_t hash;
};

const Row kRows[] = {
    {&spade::compute::gen::kSpvVariants_airflow_actuator_gather, "airflow_actuator_gather", 32, 0xa7a28fa20e0349c8ull},
    {&spade::compute::gen::kSpvVariants_airflow_actuator_gather, "airflow_actuator_gather", 64, 0xd5c906975cca98e8ull},
    {&spade::compute::gen::kSpvVariants_airflow_actuator_gather, "airflow_actuator_gather", 128, 0xd3d82509d193b228ull},
    {&spade::compute::gen::kSpvVariants_airflow_add_increments, "airflow_add_increments", 32, 0x965d899125eba947ull},
    {&spade::compute::gen::kSpvVariants_airflow_add_increments, "airflow_add_increments", 64, 0x7b5ddec1bed6fee7ull},
    {&spade::compute::gen::kSpvVariants_airflow_add_increments, "airflow_add_increments", 128, 0x3ef92492d1bc98a7ull},
    {&spade::compute::gen::kSpvVariants_airflow_add_one, "airflow_add_one", 32, 0x6cdff9ab96d5e17eull},
    {&spade::compute::gen::kSpvVariants_airflow_add_one, "airflow_add_one", 64, 0x3355e3a7176ab35eull},
    {&spade::compute::gen::kSpvVariants_airflow_add_one, "airflow_add_one", 128, 0xf262343195c0a71eull},
    {&spade::compute::gen::kSpvVariants_airflow_advection, "airflow_advection", 32, 0xbc7598c557052777ull},
    {&spade::compute::gen::kSpvVariants_airflow_advection, "airflow_advection", 64, 0x0c917296f6d3d597ull},
    {&spade::compute::gen::kSpvVariants_airflow_advection, "airflow_advection", 128, 0x582bf6d9bcb66ad7ull},
    {&spade::compute::gen::kSpvVariants_airflow_copy, "airflow_copy", 32, 0xafbbcbf43729c891ull},
    {&spade::compute::gen::kSpvVariants_airflow_copy, "airflow_copy", 64, 0xccb64e62782a14b1ull},
    {&spade::compute::gen::kSpvVariants_airflow_copy, "airflow_copy", 128, 0x26ed6c0f4f2ed3f1ull},
    {&spade::compute::gen::kSpvVariants_airflow_divergence, "airflow_divergence", 32, 0x0a9ae19292910ca6ull},
    {&spade::compute::gen::kSpvVariants_airflow_divergence, "airflow_divergence", 64, 0xa1031c309c2f8546ull},
    {&spade::compute::gen::kSpvVariants_airflow_divergence, "airflow_divergence", 128, 0x445ec1a426543906ull},
    {&spade::compute::gen::kSpvVariants_airflow_gradient_subtract, "airflow_gradient_subtract", 32, 0x88cdb88d2b4b6bbeull},
    {&spade::compute::gen::kSpvVariants_airflow_gradient_subtract, "airflow_gradient_subtract", 64, 0x57a31f251339979eull},
    {&spade::compute::gen::kSpvVariants_airflow_gradient_subtract, "airflow_gradient_subtract", 128, 0x41df1b019afd30deull},
    {&spade::compute::gen::kSpvVariants_airflow_mg_prolong, "airflow_mg_prolong", 32, 0x1ca148fc8f207b5full},
    {&spade::compute::gen::kSpvVariants_airflow_mg_prolong, "airflow_mg_prolong", 64, 0x508da48f95c55f3full},
    {&spade::compute::gen::kSpvVariants_airflow_mg_prolong, "airflow_mg_prolong", 128, 0x1017d28373935f7full},
    {&spade::compute::gen::kSpvVariants_airflow_mg_residual, "airflow_mg_residual", 32, 0x01f16f1baf6d7d2bull},
    {&spade::compute::gen::kSpvVariants_airflow_mg_residual, "airflow_mg_residual", 64, 0x03fbdfe0d7116c8bull},
    {&spade::compute::gen::kSpvVariants_airflow_mg_residual, "airflow_mg_residual", 128, 0x914bb9ed47af6c4bull},
    {&spade::compute::gen::kSpvVariants_airflow_mg_restrict, "airflow_mg_restrict", 32, 0x54f2beb3a1646547ull},
    {&spade::compute::gen::kSpvVariants_airflow_mg_restrict, "airflow_mg_restrict", 64, 0x84622131b0346c67ull},
    {&spade::compute::gen::kSpvVariants_airflow_mg_restrict, "airflow_mg_restrict", 128, 0x6c5f8e7dd209f9a7ull},
    {&spade::compute::gen::kSpvVariants_airflow_mg_restrict_full, "airflow_mg_restrict_full", 32, 0x00a11104c37e9bf7ull},
    {&spade::compute::gen::kSpvVariants_airflow_mg_restrict_full, "airflow_mg_restrict_full", 64, 0xa5b3fb17a576f9d7ull},
    {&spade::compute::gen::kSpvVariants_airflow_mg_restrict_full, "airflow_mg_restrict_full", 128, 0x67e5fe2dbd526597ull},
    {&spade::compute::gen::kSpvVariants_airflow_mg_smooth, "airflow_mg_smooth", 32, 0x6a6e33dbb72cc146ull},
    {&spade::compute::gen::kSpvVariants_airflow_mg_smooth, "airflow_mg_smooth", 64, 0x913f4ff111594626ull},
    {&spade::compute::gen::kSpvVariants_airflow_mg_smooth, "airflow_mg_smooth", 128, 0xb9a34a12aa726ce6ull},
    {&spade::compute::gen::kSpvVariants_airflow_pack_f16, "airflow_pack_f16", 32, 0xa41872aecc00fa1cull},
    {&spade::compute::gen::kSpvVariants_airflow_pack_f16, "airflow_pack_f16", 64, 0x055d8b1c49fc9f7cull},
    {&spade::compute::gen::kSpvVariants_airflow_pack_f16, "airflow_pack_f16", 128, 0x64fd97dac1dbc23cull},
    {&spade::compute::gen::kSpvVariants_airflow_pcg_apply, "airflow_pcg_apply", 32, 0x29c5cb02ce829168ull},
    {&spade::compute::gen::kSpvVariants_airflow_pcg_apply, "airflow_pcg_apply", 64, 0x28676990d0a96ac8ull},
    {&spade::compute::gen::kSpvVariants_airflow_pcg_apply, "airflow_pcg_apply", 128, 0xa1f5292d24cc0a88ull},
    {&spade::compute::gen::kSpvVariants_airflow_pcg_measure, "airflow_pcg_measure", 32, 0xe14cad46cb47ff61ull},
    {&spade::compute::gen::kSpvVariants_airflow_pcg_measure, "airflow_pcg_measure", 64, 0x3177d25c77acd601ull},
    {&spade::compute::gen::kSpvVariants_airflow_pcg_measure, "airflow_pcg_measure", 128, 0x8495de017da401c1ull},
    {&spade::compute::gen::kSpvVariants_airflow_pcg_scalars, "airflow_pcg_scalars", 32, 0x458e0cd4e3d4164eull},
    {&spade::compute::gen::kSpvVariants_airflow_pcg_scalars, "airflow_pcg_scalars", 64, 0xe8a1ef29255fb8aeull},
    {&spade::compute::gen::kSpvVariants_airflow_pcg_scalars, "airflow_pcg_scalars", 128, 0xcad21fe21f38946eull},
    {&spade::compute::gen::kSpvVariants_airflow_pcg_update, "airflow_pcg_update", 32, 0x9c607db68b920c7full},
    {&spade::compute::gen::kSpvVariants_airflow_pcg_update, "airflow_pcg_update", 64, 0xfe16061920a3735full},
    {&spade::compute::gen::kSpvVariants_airflow_pcg_update, "airflow_pcg_update", 128, 0xb49f6a643b1c1b1full},
    {&spade::compute::gen::kSpvVariants_airflow_plan_a_measure, "airflow_plan_a_measure", 32, 0x1e7986226da66266ull},
    {&spade::compute::gen::kSpvVariants_airflow_plan_a_measure, "airflow_plan_a_measure", 64, 0x67eb4b87eadff906ull},
    {&spade::compute::gen::kSpvVariants_airflow_plan_a_measure, "airflow_plan_a_measure", 128, 0x2b480a4ac31c7446ull},
    {&spade::compute::gen::kSpvVariants_airflow_publish_fields, "airflow_publish_fields", 32, 0x166068f53adbbedbull},
    {&spade::compute::gen::kSpvVariants_airflow_publish_fields, "airflow_publish_fields", 64, 0xc3460a0efa17273bull},
    {&spade::compute::gen::kSpvVariants_airflow_publish_fields, "airflow_publish_fields", 128, 0xececff851ded63fbull},
    {&spade::compute::gen::kSpvVariants_airflow_region_mean, "airflow_region_mean", 32, 0x1ee84b6144653d83ull},
    {&spade::compute::gen::kSpvVariants_airflow_region_mean, "airflow_region_mean", 64, 0x6e20a10af7f70fe3ull},
    {&spade::compute::gen::kSpvVariants_airflow_region_mean, "airflow_region_mean", 128, 0x53a89b0cf30f1523ull},
    {&spade::compute::gen::kSpvVariants_airflow_remove_mean, "airflow_remove_mean", 32, 0x06648700235057b9ull},
    {&spade::compute::gen::kSpvVariants_airflow_remove_mean, "airflow_remove_mean", 64, 0x90adbd218f27bc59ull},
    {&spade::compute::gen::kSpvVariants_airflow_remove_mean, "airflow_remove_mean", 128, 0x550004dc2d60f819ull},
    {&spade::compute::gen::kSpvVariants_airflow_solid_divergence, "airflow_solid_divergence", 32, 0xddaf6cc8c27a8920ull},
    {&spade::compute::gen::kSpvVariants_airflow_solid_divergence, "airflow_solid_divergence", 64, 0xf4474fae409f5780ull},
    {&spade::compute::gen::kSpvVariants_airflow_solid_divergence, "airflow_solid_divergence", 128, 0x4cc12028dfacae40ull},
    {&spade::compute::gen::kSpvVariants_airflow_solid_gradient, "airflow_solid_gradient", 32, 0xd6ae23c2d405b139ull},
    {&spade::compute::gen::kSpvVariants_airflow_solid_gradient, "airflow_solid_gradient", 64, 0x393092ea87ddc599ull},
    {&spade::compute::gen::kSpvVariants_airflow_solid_gradient, "airflow_solid_gradient", 128, 0x8d1af5b4a55527d9ull},
    {&spade::compute::gen::kSpvVariants_airflow_solid_velocities, "airflow_solid_velocities", 32, 0xb5f42e4fa823b654ull},
    {&spade::compute::gen::kSpvVariants_airflow_solid_velocities, "airflow_solid_velocities", 64, 0xb3a5c610301a3434ull},
    {&spade::compute::gen::kSpvVariants_airflow_solid_velocities, "airflow_solid_velocities", 128, 0x19916df0a462fc74ull},
    {&spade::compute::gen::kSpvVariants_airflow_tile_dot, "airflow_tile_dot", 32, 0x8638a62cbe9a1623ull},
    {&spade::compute::gen::kSpvVariants_airflow_tile_dot, "airflow_tile_dot", 64, 0x14cdb2352941c403ull},
    {&spade::compute::gen::kSpvVariants_airflow_tile_dot, "airflow_tile_dot", 128, 0xfdb398c56853d1c3ull},
    {&spade::compute::gen::kSpvVariants_airflow_tile_sum, "airflow_tile_sum", 32, 0xb04c9c3d640d7ddbull},
    {&spade::compute::gen::kSpvVariants_airflow_tile_sum, "airflow_tile_sum", 64, 0x43409d0b99f3e23bull},
    {&spade::compute::gen::kSpvVariants_airflow_tile_sum, "airflow_tile_sum", 128, 0x4b7d0b5b16fc397bull},
    {&spade::compute::gen::kSpvVariants_airflow_tile_sum_probe, "airflow_tile_sum_probe", 32, 0x2aff03562872d667ull},
    {&spade::compute::gen::kSpvVariants_airflow_tile_sum_probe, "airflow_tile_sum_probe", 64, 0xaeb76ba4cc08ab07ull},
    {&spade::compute::gen::kSpvVariants_airflow_tile_sum_probe, "airflow_tile_sum_probe", 128, 0x16540778f89ca247ull},
    {&spade::compute::gen::kSpvVariants_airflow_velocity_bc, "airflow_velocity_bc", 32, 0xac0879646e4dda97ull},
    {&spade::compute::gen::kSpvVariants_airflow_velocity_bc, "airflow_velocity_bc", 64, 0xf297575041486f77ull},
    {&spade::compute::gen::kSpvVariants_airflow_velocity_bc, "airflow_velocity_bc", 128, 0x6f5bea9b6a12cbb7ull},
    {&spade::compute::gen::kSpvVariants_airflow_viscous, "airflow_viscous", 32, 0x05ecc24794ebb3edull},
    {&spade::compute::gen::kSpvVariants_airflow_viscous, "airflow_viscous", 64, 0xd3d020721b22a8cdull},
    {&spade::compute::gen::kSpvVariants_airflow_viscous, "airflow_viscous", 128, 0xf8e92b6d34ea508dull},
    {&spade::compute::gen::kSpvVariants_airflow_wall_load, "airflow_wall_load", 32, 0x9e4bf20fc23c46ffull},
    {&spade::compute::gen::kSpvVariants_airflow_wall_load, "airflow_wall_load", 64, 0x10230fab97999b5full},
    {&spade::compute::gen::kSpvVariants_airflow_wall_load, "airflow_wall_load", 128, 0x15b20fe22b98011full},
    {&spade::compute::gen::kSpvVariants_collision_dynamic, "collision_dynamic", 32, 0xbd1da5878b1caefaull},
    {&spade::compute::gen::kSpvVariants_collision_dynamic, "collision_dynamic", 64, 0xf07482888c1f871aull},
    {&spade::compute::gen::kSpvVariants_collision_dynamic, "collision_dynamic", 128, 0xadb83a0c8319f7daull},
    {&spade::compute::gen::kSpvVariants_collision_fill, "collision_fill", 32, 0x9aceadca50428517ull},
    {&spade::compute::gen::kSpvVariants_collision_fill, "collision_fill", 64, 0x7af901440b624f77ull},
    {&spade::compute::gen::kSpvVariants_collision_fill, "collision_fill", 128, 0x4306a3902009a637ull},
    {&spade::compute::gen::kSpvVariants_collision_gather, "collision_gather", 32, 0x8205c2fe40a555a9ull},
    {&spade::compute::gen::kSpvVariants_collision_gather, "collision_gather", 64, 0x83337051144d1789ull},
    {&spade::compute::gen::kSpvVariants_collision_gather, "collision_gather", 128, 0x5de15b88c07548c9ull},
    {&spade::compute::gen::kSpvVariants_collision_static, "collision_static", 32, 0x80d826b29bf9082bull},
    {&spade::compute::gen::kSpvVariants_collision_static, "collision_static", 64, 0x751ba3b2ee93974bull},
    {&spade::compute::gen::kSpvVariants_collision_static, "collision_static", 128, 0xd6f8322cafa36d0bull},
    {&spade::compute::gen::kSpvVariants_denorm_probe, "denorm_probe", 64, 0xaf6ac072a7fbe415ull},
    {&spade::compute::gen::kSpvVariants_field_dryden, "field_dryden", 32, 0x9f0ca32fc1d3ef2dull},
    {&spade::compute::gen::kSpvVariants_field_dryden, "field_dryden", 64, 0xb1f12ab614c2750dull},
    {&spade::compute::gen::kSpvVariants_field_dryden, "field_dryden", 128, 0xa38b15e89b0dfbcdull},
    {&spade::compute::gen::kSpvVariants_field_environment, "field_environment", 32, 0xefd98018e80698c3ull},
    {&spade::compute::gen::kSpvVariants_field_environment, "field_environment", 64, 0x617be169520c9ae3ull},
    {&spade::compute::gen::kSpvVariants_field_environment, "field_environment", 128, 0x78af66f5235aafa3ull},
    {&spade::compute::gen::kSpvVariants_forces_drag, "forces_drag", 32, 0x1cbbf78c7059e636ull},
    {&spade::compute::gen::kSpvVariants_forces_drag, "forces_drag", 64, 0xb13208aec6771d16ull},
    {&spade::compute::gen::kSpvVariants_forces_drag, "forces_drag", 128, 0x12582c4cfbfa0cd6ull},
    {&spade::compute::gen::kSpvVariants_fp32_math_probe, "fp32_math_probe", 64, 0x7925132bb5e42de4ull},
    {&spade::compute::gen::kSpvVariants_grid_build, "grid_build", 32, 0x3ce2333ef41af3d8ull},
    {&spade::compute::gen::kSpvVariants_grid_build, "grid_build", 64, 0xa889155131a3a678ull},
    {&spade::compute::gen::kSpvVariants_grid_build, "grid_build", 128, 0x344bcbc08a984f38ull},
    {&spade::compute::gen::kSpvVariants_grid_sort, "grid_sort", 32, 0x0e9ed654dfac6c8full},
    {&spade::compute::gen::kSpvVariants_grid_sort, "grid_sort", 64, 0x0b72807c27e5df6full},
    {&spade::compute::gen::kSpvVariants_grid_sort, "grid_sort", 128, 0xa042c91f148b64afull},
    {&spade::compute::gen::kSpvVariants_integrate, "integrate", 32, 0x562223c68ebebc2cull},
    {&spade::compute::gen::kSpvVariants_integrate, "integrate", 64, 0xa26153ed87a8008cull},
    {&spade::compute::gen::kSpvVariants_integrate, "integrate", 128, 0xbebb10dfce48734cull},
    {&spade::compute::gen::kSpvVariants_medium_update, "medium_update", 32, 0x82d16e2aefbf1341ull},
    {&spade::compute::gen::kSpvVariants_medium_update, "medium_update", 64, 0x7067f59a10344761ull},
    {&spade::compute::gen::kSpvVariants_medium_update, "medium_update", 128, 0x863eed079e901521ull},
    {&spade::compute::gen::kSpvVariants_rotors, "rotors", 32, 0x62534b101fb50efeull},
    {&spade::compute::gen::kSpvVariants_rotors, "rotors", 64, 0x658a27ec5492cb9eull},
    {&spade::compute::gen::kSpvVariants_rotors, "rotors", 128, 0xc02c55568f305a5eull},
    {&spade::compute::gen::kSpvVariants_sensor_gnss, "sensor_gnss", 32, 0xc7f9886454abcd4cull},
    {&spade::compute::gen::kSpvVariants_sensor_gnss, "sensor_gnss", 64, 0x3636b00ac7904aacull},
    {&spade::compute::gen::kSpvVariants_sensor_gnss, "sensor_gnss", 128, 0x4d28703a3e6d7e6cull},
    {&spade::compute::gen::kSpvVariants_sensor_imu, "sensor_imu", 32, 0xbfbaf8d29ab48b72ull},
    {&spade::compute::gen::kSpvVariants_sensor_imu, "sensor_imu", 64, 0xe8f2a32e92f40c52ull},
    {&spade::compute::gen::kSpvVariants_sensor_imu, "sensor_imu", 128, 0xb702a922284ca512ull},
};

}  // namespace

TEST(SpirvPin, EveryExistingKernelVariantIsByteIdentical) {
    for (const Row& row : kRows) {
        const spade::compute::SpirvVariant* found = nullptr;
        for (std::size_t i = 0; i < row.set->count; ++i) {
            if (row.set->variants[i].workgroup_size == row.workgroup_size) found = &row.set->variants[i];
        }
        ASSERT_NE(found, nullptr) << row.name << " has no variant compiled for " << row.workgroup_size;
        EXPECT_EQ(fnv1a64_words(found->words, found->word_count), row.hash)
            << row.name << " at local size " << row.workgroup_size << " changed bytes";
    }
}

// The table is the whole set: a variant of a pinned kernel that the table does not name is a new compile this pin
// has not seen, and says so.
TEST(SpirvPin, TheTableNamesEveryVariantOfEveryPinnedKernel) {
    for (const Row& row : kRows) {
        std::size_t named = 0;
        for (const Row& other : kRows) {
            if (other.set == row.set) ++named;
        }
        EXPECT_EQ(named, row.set->count) << row.name;
    }
}
