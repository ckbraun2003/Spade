// ---------------------------------------------------------------------------
// test_slang_layouts.cpp -- the D9 pipeline's HOST-SIDE checks. No device, no
// Vulkan call, no GTEST_SKIP: everything here runs on a CI runner with no GPU,
// which is why none of these suites is named Gpu* (tests/AppendSpadeLabels.cmake
// gives the "gpu" ctest label to Gpu*.* by name prefix, and these must not
// carry it -- they are not skippable).
//
// THREE THINGS ARE UNDER TEST, and the first one is unusual enough to state
// plainly:
//
//   1. THAT THIS FILE COMPILES AT ALL. It includes layout_check.gen.hpp, the
//      generated header whose entire content is static_asserts comparing every
//      C++ state-row field offset against the offset slangc REPORTED for the
//      corresponding field of engine/shaders/shared/layouts.slang. If a field
//      moved on either side, this translation unit does not compile and the
//      build fails naming the struct and the field. The runtime assertions
//      below cannot express that; they exist to catch the one thing a
//      static_assert cannot -- a generated header that checked NOTHING.
//
//   2. THE BINDING REGISTRY COVERS THE BOUND WALK EXACTLY. bindings.gen.hpp
//      must carry a binding for each of the ELEVEN bound arrays and for the
//      FIVE `.slot_to_world` siblings kernels need, with no duplicate index and
//      no silent addition -- and no binding for any array listed as
//      deliberately unbound (that list is empty as of the GPU-sensor leg).
//
//      WHAT THIS CANNOT SEE, SAID PLAINLY. kBoundArrays below is
//      hardcoded, on purpose (a test reading its expectations out of the
//      artifact under test would assert nothing). That independence is exactly
//      why nothing here notices an array REGISTERED in sim/simulation.cpp: the
//      guard on that side is StateMirror::upload()'s runtime refusal, which is
//      Vulkan-only and GTEST_SKIP()-gated, so on a device-less box an
//      array_shapes() drift is caught by nothing at all.
//
//   3. THE SPIR-V FLOAT-CONTROLS GATE. SlangSpirv.FloatControlsPinned scans
//      the embedded SPIR-V of every compiled kernel against
//      engine/testing/spirv_scan.hpp's policy: NoContraction on every
//      contractable float op, no sum-of-products opcode, and (for modules held
//      to the exact profile) no OpFDiv and no sqrt.
// ---------------------------------------------------------------------------

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <iterator>
#include <span>
#include <string>
#include <utility>
#include <vector>

#include "bindings.gen.hpp"
#include "compute/backend.hpp"
#include "compute/layout_check.hpp"
#include "compute/spirv_variants.hpp"
#include "compute/vulkan/probe_runner.hpp"
#include "testing/spirv_scan.hpp"
#include "kernel_manifest.gen.hpp"
#include "layout_check.gen.hpp"
#include "collision_dynamic.spv.gen.hpp"
#include "collision_fill.spv.gen.hpp"
#include "collision_gather.spv.gen.hpp"
#include "collision_static.spv.gen.hpp"
#include "field_dryden.spv.gen.hpp"
#include "field_environment.spv.gen.hpp"
#include "forces_drag.spv.gen.hpp"
#include "fp32_math_probe.spv.gen.hpp"
#if defined(SPADE_MEASURE_UNPINNED_DENORMS)
#include "denorm_probe.spv.gen.hpp"  // M1's probe, built only in the measurement tree
#endif
#include "grid_build.spv.gen.hpp"
#include "grid_sort.spv.gen.hpp"
#include "integrate.spv.gen.hpp"
#include "medium_update.spv.gen.hpp"
#include "rotors.spv.gen.hpp"
#include "sensor_gnss.spv.gen.hpp"
#include "sensor_imu.spv.gen.hpp"

namespace {

namespace gen = spade::compute::gen;

// -------------------------------------------------------------------------
// The ELEVEN registered arrays that are BOUND, in registered-walk order
// (sim/simulation.cpp's register_array calls; replay_config pinned at walk
// index 16 by test_determinism.cpp, with the GNSS pair appended below it).
// Spelled out here rather than derived from bindings.gen.hpp, deliberately:
// this list is the INDEPENDENT statement of what the registry must cover, and
// a test that read its expectations out of the artifact under test would
// assert nothing.
// -------------------------------------------------------------------------
constexpr const char* kBoundArrays[] = {
    "world_params", "bodies",      "body_generation", "drag_bodies", "dryden",
    "imu_sensors",  "imu_ring",    "rotors",          "replay_config",
    "gnss_sensors", "gnss_ring",
};

// REGISTERED AND DELIBERATELY UNBOUND -- EMPTY AS OF THE GPU-SENSOR LEG, and
// kept rather than deleted because the mechanism is still live and the next
// registered array that no kernel reads belongs here.
//
// gnss_sensors and gnss_ring were its only two members, for exactly one leg:
// while nothing read them, binding them would have cost a descriptor slot and
// two Slang mirrors under the strict layout gate for buffers no kernel could
// read. sensor_gnss.slang changed that, so they moved to kBoundArrays above.
//
// THE MOVE WAS FORCED BY A TEST RATHER THAN NOTICED BY A READER.
// DeliberatelyUnboundArraysHaveNoBinding asserts in the NEGATIVE that every
// name here has no binding, so adding one fails loudly -- which is the whole
// point of writing an exemption as a two-way claim: AN EXEMPTION NOTHING CHECKS
// IS NOT AN EXEMPTION, IT IS A CLAIM.
// std::array rather than a C array because a zero-length C array is ill-formed;
// this is the one shape that can legally hold "none, today".
constexpr std::array<const char*, 0> kUnboundArrays{};

constexpr std::size_t kRegisteredArrayCount = 11;
static_assert(std::size(kBoundArrays) + std::size(kUnboundArrays) == kRegisteredArrayCount);

// One row of the registry as the generated header presents it. `name` is the
// Slang parameter name (the `.` of a registered array's sibling spelled `_`).
struct RegistryEntry {
    const char* name;
    uint32_t binding;
};

// EVERY constant bindings.gen.hpp is expected to define, named one by one. If
// the generator emitted one fewer, the corresponding line below fails to
// compile; if it emitted one more, kBindingCount_state moves and
// BindingRegistryHasNoUnlistedEntries fails. Both directions are covered,
// which is what "no silent omission" has to mean.
const RegistryEntry kExpectedRegistry[] = {
    // the nine original registered arrays
    {"world_params", gen::kBinding_world_params},
    {"bodies", gen::kBinding_bodies},
    {"body_generation", gen::kBinding_body_generation},
    {"drag_bodies", gen::kBinding_drag_bodies},
    {"dryden", gen::kBinding_dryden},
    {"imu_sensors", gen::kBinding_imu_sensors},
    {"imu_ring", gen::kBinding_imu_ring},
    {"rotors", gen::kBinding_rotors},
    {"replay_config", gen::kBinding_replay_config},
    // The second sensor kind (bindings.slang section E), appended at 23..24
    // rather than inserted in walk order so that no existing hand-authored
    // [[vk::binding(n)]] had to be renumbered.
    {"gnss_sensors", gen::kBinding_gnss_sensors},
    {"gnss_ring", gen::kBinding_gnss_ring},
    // the .slot_to_world siblings a kernel dispatched over a global slot space
    // needs -- FIVE of them now (bindings.slang section B records why the rest
    // are deliberately absent)
    {"bodies_slot_to_world", gen::kBinding_bodies_slot_to_world},
    {"drag_bodies_slot_to_world", gen::kBinding_drag_bodies_slot_to_world},
    {"imu_sensors_slot_to_world", gen::kBinding_imu_sensors_slot_to_world},
    {"rotors_slot_to_world", gen::kBinding_rotors_slot_to_world},
    // The GNSS slot->world sibling, bound on section B's rule: sensor_gnss
    // dispatches over the GNSS slot space and reads the map on its second line.
    // gnss_ring's sibling is not bound -- nothing dispatches over a ring.
    {"gnss_sensors_slot_to_world", gen::kBinding_gnss_sensors_slot_to_world},
    // the derived (non-registered) buffers -- backend-internal storage the
    // host owns, uploaded from configuration rather than walked out of the
    // ArenaSet. bindings.slang sections C and C' record why each exists.
    {"dryden_params", gen::kBinding_dryden_params},
    // S6 Task 6: the flattened per-world SDF programs static_contact.resolve walks,
    // the per-STEP tick exchange (host write + the kernel's witness), and the
    // PER-WORLD contact material (per world, not per run: the golden corpus's
    // `bounce` is a four-rung restitution ladder).
    {"sdf_nodes", gen::kBinding_sdf_nodes},
    {"sdf_transforms", gen::kBinding_sdf_transforms},
    {"sdf_world_ranges", gen::kBinding_sdf_world_ranges},
    {"step_params", gen::kBinding_step_params},
    {"step_witness", gen::kBinding_step_witness},
    {"contact_params", gen::kBinding_contact_params},
    // S6 Task 6b (checkpoint-1 ruling): the PER-WORLD broad-phase grid
    // config, closing the last per-batch config record (PassParams no longer
    // carries a ContactParams/GridParams pair at all -- see
    // layouts.slang's PassParams doc comment) and giving T7's
    // CollisionDynamic kernel a per-world buffer to read instead of reviving
    // it.
    {"grid_params", gen::kBinding_grid_params},
    // S6 Task 7: the dynamic broad phase's KEY ARRAY -- the device's analogue
    // of physics/grid.hpp's GridScratch, and the second device-WRITTEN buffer
    // in the set that is not registered state (step_witness is the other). It
    // is neither uploaded nor read back on the step path: the build, the sort
    // stages and the sweep all live inside one dynamic_contact.resolve pass.
    {"grid_entries", gen::kBinding_grid_entries},
    // The dynamic_contact.resolve GATHER's start-of-iteration shadow of pos, vel,
    // mass and the effective proxy radius -- one row per grid ENTRY, the
    // device analogue of GridScratch::snapshot. DERIVED for the same reason
    // grid_entries is: written by a dispatch inside the pass, never uploaded,
    // never read back, and dead the moment the pass ends.
    //
    // IT IS A BUFFER RATHER THAN A BARRIER. A gather thread reads all of its
    // partners and writes only itself; with threads mapped to entries another
    // workgroup may already have written a partner, and Vulkan has no
    // device-wide barrier inside a dispatch to separate the reads from the
    // writes. Shadowing the two fields this pass mutates is what makes the
    // iteration Jacobi rather than a race.
    {"body_snapshot", gen::kBinding_body_snapshot},
    // Module-API stage 3: every world's field sample row (layouts.slang's
    // FieldSampleRow) -- gravity, density and wind, written once per substep
    // by the two field-sample kernels and read by rotors, drag and integrate.
    // DERIVED for grid_entries' reason: written by a dispatch, read inside the
    // same substep, never uploaded, read back only by the field_samples()
    // diagnostic.
    {"field_samples", gen::kBinding_field_samples},
};

// How many entries of kExpectedRegistry are DERIVED (not part of the
// registered walk and not one of its bound siblings). Named rather than
// spelled as a literal in the count assertion below, so the assertion reads as
// the sentence it is meant to be: the BOUND arrays + five siblings + the
// derived buffers, and nothing else.
constexpr std::size_t kDerivedBufferCount = 11;

}  // namespace

// ===========================================================================
// 1. The generated layout check
// ===========================================================================

// The generated header asserted SOMETHING, and specifically one layout per
// mirrored Slang struct. Thirteen as of S6 Task 5: RngStream, BodyState,
// WorldParams, DragBodyRow, ContactParams, GridParams, RotorRow,
// ImuSensorRow, ImuSample, DrydenState, DrydenParams, ReplayConfig, and
// PassParams -- Task 5 gave the last a C++ counterpart
// (compute/vulkan/step_recorder.hpp's spade::compute::PassParams) and the
// matching @cpp directives, closing the exemption D9 (Task 3) originally
// carried for it (that struct's own header comment, layouts.slang, has the
// detail).
//
// EIGHTEEN AS OF S6 TASK 6, which added five: SdfNodeRow, SdfTransformRow and
// SdfWorldRange (the device image of every world's static SDF program, which
// static_contact.resolve walks -- compute/sdf_program.hpp), plus StepParams and
// StepWitness (the per-step tick exchange, whose device half Task 5's review
// deferred to this task -- compute/step_params.hpp).
//
// NINETEEN AS OF S6 TASK 7, which added one: GridEntryRow -- the device image
// of the dynamic broad phase's sorted key (compute/grid_entry.hpp). It mirrors
// that header's own struct rather than physics::GridEntry, for the reason
// stated there (the CPU record nests a GridCell and is scratch, not state).
//
// TWENTY WITH THE JACOBI GATHER, which added one: GatherBodyRow -- the
// start-of-iteration shadow of pos, vel, mass and the effective proxy radius
// that the gather's per-pair math reads, mirroring physics::GatherBody
// (physics/grid.hpp). Unlike GridEntryRow it mirrors the PHYSICS struct
// directly rather than a compute-side twin, because there is no CPU/device
// difference to absorb: both sides hold the same four fields in the same
// std430 row, and the CPU copy exists for the same reason the device one does.
//
// AND THE MIRROR EARNED ITS KEEP IMMEDIATELY. The offsets were predicted
// correctly and the ALIGNMENT was not: glm::vec3 aligns to 4, std430 requires
// 16 for any struct holding a vector, and the size was already 32 -- so every
// field would have read correctly and an ARRAY of them would have been strided
// wrong, which is the kind of defect that surfaces as physics rather than as a
// crash. The generated assert named it in one line. A layout claim nobody can
// check is exactly what this count exists to make impossible.
//
// WITHOUT THIS TEST a generator bug that emitted an empty layout_check.gen.hpp
// would produce a green build: zero static_asserts all pass. The count is the
// only thing that distinguishes "checked everything" from "checked nothing".
TEST(SlangLayouts, GeneratedHeaderChecksEveryMirroredStruct) {
    // TWENTY-THREE WITH THE FIELD SAMPLE ROW (module-API stage 3):
    // FieldSampleRow, mirroring physics::FieldSampleRow (physics/field_row.hpp),
    // alignas(16) on the C++ side for GatherBodyRow's reason above.
    EXPECT_EQ(gen::kMirroredStructCount, std::size_t{23});
    EXPECT_EQ(spade::compute::layout_check_mirrored_struct_count(), gen::kMirroredStructCount);
}

// ===========================================================================
// 2. The binding registry
// ===========================================================================

TEST(SlangLayouts, BindingRegistryCoversEveryRegisteredArray) {
    for (const char* array : kBoundArrays) {
        const auto found = std::find_if(
            std::begin(kExpectedRegistry), std::end(kExpectedRegistry),
            [array](const RegistryEntry& entry) { return std::string(entry.name) == array; });
        EXPECT_NE(found, std::end(kExpectedRegistry))
            << "registered array '" << array << "' has no binding in bindings.gen.hpp";
    }
}

// The registry holds the nine arrays, the four bound siblings, and the nine
// derived buffers -- and NOTHING else. kBindingCount_state is generated from
// the reflection record, so a binding added to bindings.slang without a
// corresponding decision recorded here moves this number and fails.
TEST(SlangLayouts, BindingRegistryHasNoUnlistedEntries) {
    EXPECT_EQ(std::size_t{gen::kBindingCount_state}, std::size(kExpectedRegistry));
    // FIVE bound siblings now, not four: sensor_gnss.slang dispatches over the
    // GNSS slot space, so gnss_sensors.slot_to_world joins on section B's rule.
    EXPECT_EQ(std::size_t{gen::kBindingCount_state},
              kRegisteredArrayCount - std::size(kUnboundArrays) + 5u + kDerivedBufferCount);
}

// AN EXEMPTION NOTHING CHECKS IS NOT AN EXEMPTION, IT IS A CLAIM. kUnboundArrays
// says two registered arrays deliberately have no binding; without this test
// that sentence could quietly stop being true in the direction that spends a
// descriptor slot on a buffer the list says is absent. Asserted in the negative
// so the exemption is bidirectional exactly like the coverage it replaces.
TEST(SlangLayouts, DeliberatelyUnboundArraysHaveNoBinding) {
    // INERT WHILE THE LIST IS EMPTY, AND IT SAYS SO RATHER THAN REPORTING GREEN.
    // The GPU-sensor leg bound this guard's last two members, so today it has
    // nothing to check -- and a loop over an empty list is exactly the "filter
    // that matches nothing" this tree treats as a false green. Skipping makes
    // the inertness VISIBLE in the run log instead of indistinguishable from a
    // guard that ran and found nothing wrong.
    //
    // It is kept rather than deleted because the mechanism is live: the next
    // registered array with no kernel reader re-populates kUnboundArrays and
    // this test starts guarding again, in both directions, with no other edit.
    if (kUnboundArrays.empty()) {
        GTEST_SKIP() << "no registered array is currently exempt from binding -- this guard is "
                        "inert by construction, not passing";
    }
    for (const char* array : kUnboundArrays) {
        const auto found = std::find_if(
            std::begin(kExpectedRegistry), std::end(kExpectedRegistry),
            [array](const RegistryEntry& entry) { return std::string(entry.name) == array; });
        EXPECT_EQ(found, std::end(kExpectedRegistry))
            << "'" << array
            << "' is listed as deliberately unbound but now has a binding. If a kernel really does "
               "read it, move it to kBoundArrays and name that kernel in bindings.slang section A.";
    }
}

// Two buffers sharing a binding index is the failure mode a hand-numbered
// registry actually has: the descriptor write for one silently overwrites the
// other and the kernel reads the wrong array. The indices come back from
// slangc reflection, so this catches a duplicate authored in bindings.slang.
TEST(SlangLayouts, BindingIndicesAreDistinctAndDense) {
    std::vector<uint32_t> indices;
    for (const RegistryEntry& entry : kExpectedRegistry) indices.push_back(entry.binding);
    std::sort(indices.begin(), indices.end());
    EXPECT_EQ(std::adjacent_find(indices.begin(), indices.end()), indices.end())
        << "two bindings share an index";

    // Dense from 0: a gap would mean a descriptor-set layout with a hole,
    // which is legal Vulkan but always a mistake here -- every binding in this
    // set is created by the same buffer table.
    for (std::size_t i = 0; i < indices.size(); ++i) {
        EXPECT_EQ(indices[i], static_cast<uint32_t>(i)) << "binding index gap at " << i;
    }
}

// The push-constant block must fit the 128 bytes Vulkan guarantees on every
// device (maxPushConstantsSize minimum), or the pipeline layout fails to
// create on hardware nobody tested on.
TEST(SlangLayouts, PushConstantBlockFitsTheGuaranteedMinimum) {
    EXPECT_EQ(gen::kSet_state, 0u);
    EXPECT_EQ(gen::kPushConstantOffset, 0u);
    EXPECT_GT(gen::kPushConstantSize, 0u);
    EXPECT_LE(gen::kPushConstantSize, 128u);
}

// ===========================================================================
// 3. The SPIR-V float-controls gate
// ===========================================================================

namespace {

// Every compiled kernel, with the profile it is held to: fp32_math (Task 4)
// under `exact`, every physics kernel under `parity`.
//
// THE NO_OP PROFILE IS GONE, WITH ITS ONLY INSTANCE (S6 Task 8). Through Task 7
// pipeline_smoke -- the byte-preserving stub every un-ported schedule slot
// dispatched -- was checked here under a third profile written for it. Task 8
// ports the last two passes, so no slot binds a stub, the two INERT slots
// record no dispatch at all (compute/vulkan/step_recorder.hpp's slot table),
// and the kernel, its CMake entry, this row and the profile are all deleted
// together. Keeping a profile with no module to check would have left an
// untested branch of scan_spirv() behind; see this task's report, which flags
// the retirement for review.
// ---------------------------------------------------------------------------
// A ROW IS A KERNEL'S WHOLE COMPILED FAMILY AS OF S6 TASK 9b, not one module.
//
// Making BackendDesc::workgroup_size live means each of the nine schedule
// kernels is compiled once per size in compute/backend.hpp's
// kSupportedWorkgroupSizes -- three real, separately-compiled SPIR-V modules
// that the backend picks between at pipeline creation. Every one of them runs
// on the device, so every one of them owes the same float-controls policy, and
// the ONLY safe way to guarantee that is for this table to hold the compiled
// SET and the loops below to walk it.
//
// WHY NOT 27 HAND-WRITTEN ROWS. Because the next size added to
// SPADE_SLANG_WORKGROUP_SIZES would compile nine more modules that this table
// would silently not scan -- a gate that quietly stops covering what it was
// written to cover is worse than no gate. Walking `variants.all()` means
// coverage follows the build by construction; EveryCompiledVariantIsScanned
// below states the resulting arithmetic out loud so a drop in coverage is a
// failure rather than a smaller number nobody reads.
// ---------------------------------------------------------------------------
struct SpirvModule {
    const spade::compute::SpirvVariantSet& variants;
    spade::testing::SpirvProfile profile;

    // -------------------------------------------------------------------
    // `integer_only` -- S6 Task 7, and it changes NO POLICY. The module is
    // still held to PARITY (P1 + P2 + P3) in full; what this flag changes is
    // the NON-VACUITY instrument the loop below uses to prove the embedded
    // module is a real one.
    //
    // WHY IT IS NEEDED. Parity's default non-vacuity check is "at least one
    // contractable float op", which is the right instrument for a physics
    // kernel: a kernel that lost its arithmetic, or an embedding that produced
    // an empty array, would otherwise satisfy P1 by having nothing to check.
    // grid_sort.slang has NO floating-point arithmetic BY CONSTRUCTION -- a
    // sort is integer comparison and a swap, and the keys' cell coordinates
    // were floored to integers by grid_build.slang -- so demanding one would
    // force the kernel to contain fake work purely to satisfy the checker.
    // That is exactly the trap engine/testing/spirv_scan.hpp's NO_OP note
    // records ("a profile that requires some would force the kernel to contain
    // fake work just to satisfy the checker"), reached from the other
    // direction.
    //
    // WHY NOT JUST USE THE NO_OP PROFILE FOR IT. NO_OP means "a byte-
    // preserving stub", and it DROPS rule P3 (the fp32 denormal pin) on the
    // argument that a kernel with zero arithmetic has no denormal hazard.
    // grid_sort is not a stub -- it is a real ported stage of a real pass --
    // and it does declare DenormPreserve (measured: the flag emits the
    // execution mode even for an arithmetic-free module), so demoting it to
    // NO_OP would silently stop checking a property it actually has. Keeping
    // PARITY and swapping only the non-vacuity instrument asserts strictly
    // more.
    //
    // WHAT REPLACES IT: the same two substitutes the NO_OP branch already
    // uses -- a real OpStore (so an empty or truncated embedding cannot pass)
    // and the "main" entry-point name -- PLUS the demand that this module
    // contains ZERO contractable ops, which for a sort is itself a real
    // assertion: floating-point arithmetic appearing in the comparator would
    // mean the key order had stopped being the CPU's integer one.
    // -------------------------------------------------------------------
    bool integer_only = false;
};

const SpirvModule kSpirvModules[] = {
    // fp32_math_probe (S6 Task 4) carries engine/shaders/fp32_math.slang
    // whole -- log32/exp32/sin32/cos32 and the correctly-rounded div32 -- so
    // scanning it IS scanning the ported kernels. EXACT, as the global
    // constraint requires of this module specifically: NoContraction on every
    // contractable op, no sum-of-products opcode, no OpFDiv (the one division
    // in log32 is respelled as integer long division; see that file's division
    // audit) and no sqrt (there is none to respell).
    {gen::kSpvVariants_fp32_math_probe, spade::testing::SpirvProfile::exact},
    // S6 Task 6's three ported kernels, under PARITY -- P1 (NoContraction on
    // every contractable op), P2 (no sum-of-products opcode) and P3 (fp32
    // denormals preserved), but NOT exact's E1/E2. That is the global
    // constraint's own split, not a relaxation: "downstream kernels MAY use
    // div/sqrt -- that is why CPU<->GPU is banded rather than bit-identical",
    // and each kernel's header lists every one of its div and sqrt sites
    // against the CPU expression it mirrors.
    //
    // collision_static carries engine/shaders/sdf_eval.slang whole (the port
    // of world/sdf.cpp), the same way fp32_math_probe carries fp32_math.slang
    // -- a Slang module reaches SPIR-V only through a kernel that imports it,
    // so scanning this module IS scanning the SDF evaluator. It also carries
    // fp32_math itself, transitively, through the heightfield's sin32.
    {gen::kSpvVariants_forces_drag, spade::testing::SpirvProfile::parity},
    {gen::kSpvVariants_collision_static, spade::testing::SpirvProfile::parity},
    {gen::kSpvVariants_integrate, spade::testing::SpirvProfile::parity},
    // S6 Task 7's three, the dynamic_contact.resolve chain, all under PARITY for
    // wave A's reasons. Two of them are worth a word each:
    //
    //   grid_build carries the only floating-point arithmetic in the broad
    //   phase -- the three divisions and three floors of grid_cell_of()
    //   (grid.cpp:306-327) -- so P1's non-vacuity has something real to bite
    //   on and P3's denormal pin genuinely matters (a flushed pos/cell_size
    //   would put a body in the wrong cell).
    //
    //   grid_sort contains NO float arithmetic at all: a sort is integer
    //   comparison and a swap. It is held to PARITY anyway rather than to
    //   NO_OP -- see SpirvModule::integer_only above for the full reasoning
    //   and for what replaces the non-vacuity instrument. Its own parity
    //   content is key_less() reproducing grid_entry_less(), which no SPIR-V
    //   rule can check: tests/test_gpu_parity.cpp's GpuGridSort cases are what
    //   check that, against std::sort itself.
    {gen::kSpvVariants_grid_build, spade::testing::SpirvProfile::parity},
    {gen::kSpvVariants_grid_sort, spade::testing::SpirvProfile::parity, /*integer_only=*/true},
    {gen::kSpvVariants_collision_dynamic, spade::testing::SpirvProfile::parity},
    // The Jacobi gather and its fill dispatch. Same PARITY profile as the
    // sweep it mirrors, because it performs the same per-pair arithmetic in
    // the same op order -- gather_apply_for_a IS resolve_pair with the
    // b-side writes deleted, so anything the profile forbids one of them is
    // forbidden the other. Its own module rather than two more entry points
    // in collision_dynamic: the variant scan below requires exactly one
    // differing word between two workgroup sizes, and three entry points
    // would make that three.
    // collision_fill is PARITY with integer_only, for grid_sort's reason
    // reached from a different direction. It is a COPY: it reads pos, mass,
    // vel and the effective radius off a body and writes them into the
    // shadow, and effective_proxy_radius is a select rather than arithmetic.
    // So it contains NO contractable float op, and parity's default
    // non-vacuity instrument -- "at least one" -- would fail on it.
    //
    // ZERO IS THE POSITIVE ASSERTION HERE, NOT AN EXEMPTION. Float arithmetic
    // appearing in this kernel would mean it had stopped being a copy and
    // started computing something, which is exactly the defect that would make
    // the shadow not a shadow of the START-of-iteration state. Demanding one
    // contractable op would force fake work in to satisfy the checker, which
    // is the trap spirv_scan.hpp's NO_OP note names. Policy is unchanged: P1,
    // P2 and P3 all still apply in full.
    {gen::kSpvVariants_collision_fill, spade::testing::SpirvProfile::parity, /*integer_only=*/true},
    {gen::kSpvVariants_collision_gather, spade::testing::SpirvProfile::parity},
    // S6 Task 8's three, completing the schedule, all under PARITY for wave A's
    // reasons. What each one CARRIES is the part worth naming, since a module
    // reaches SPIR-V only through a kernel that imports it and scanning the
    // kernel IS scanning everything it pulled in:
    //
    //   medium_update  carries dryden.slang (the port of world/medium.cpp) and,
    //                  through it, rng.slang and u64.slang. So this row is what
    //                  holds the Dryden coefficients, the Box-Muller
    //                  construction AND the 64-bit emulation to P1's
    //                  NoContraction rule and P4's no-Int64 rule at once --
    //                  which is exactly the check that would have caught the
    //                  capability defect Task 8's pre-flight found.
    //   rotors         carries the momentum-theory inflow curve, the
    //                  Cheeseman-Bennett ground factor and the RPM lag, plus
    //                  sdf_eval and dryden transitively.
    //   sensor_imu     carries rng.slang and u64.slang; its own float content is
    //                  the mount rotation, the bias walk and the noise sum.
    {gen::kSpvVariants_medium_update, spade::testing::SpirvProfile::parity},
    {gen::kSpvVariants_rotors, spade::testing::SpirvProfile::parity},
    {gen::kSpvVariants_sensor_imu, spade::testing::SpirvProfile::parity},
    //   sensor_gnss    the gnss.synthesize pass. PARITY like
    //                  sensor_imu, and for a sharper reason: its Gauss-Markov
    //                  coefficients are precomputed on the CPU, so this module
    //                  contains no exp and no OpFDiv at all -- the bias advance
    //                  is two multiplies and an add. The only banded operation
    //                  it reaches is rng.slang's Box-Muller sqrt, through the
    //                  nine draws, exactly as sensor_imu reaches it through
    //                  twelve.
    {gen::kSpvVariants_sensor_gnss, spade::testing::SpirvProfile::parity},
    //   field_environment  environment.sample: copies gravity and density into
    //                      the field row. NO FLOAT ARITHMETIC BY CONSTRUCTION
    //                      -- loads and stores only -- so it takes grid_sort's
    //                      integer_only instrument: still PARITY, its
    //                      non-vacuity proved by its stores, and any float
    //                      arithmetic that enters it later fails the gate.
    //   field_dryden       dryden.sample: mean wind plus dryden_turbulence(),
    //                      the multiplies and adds medium_update's row feeds.
    //                      It reaches no sqrt and no OpFDiv.
    {gen::kSpvVariants_field_environment, spade::testing::SpirvProfile::parity, /*integer_only=*/true},
    {gen::kSpvVariants_field_dryden, spade::testing::SpirvProfile::parity},
#if defined(SPADE_MEASURE_UNPINNED_DENORMS)
    // M1's probe: parity, like the physics kernels whose ops it measures. The
    // profile's NoContraction check is what lets the probe tell a driver that
    // fuses a mul + add from a module that asked for it.
    {gen::kSpvVariants_denorm_probe, spade::testing::SpirvProfile::parity},
#endif
};

// Rule P3's expectation for THIS build. The measurement build
// (SPADE_MEASURE_UNPINNED_DENORMS, a definition on spade_tests only; see the
// top-level CMakeLists.txt) compiles every kernel without the fp32 denormal
// pin, so there P3 inverts; everywhere else it is the pin.
#if defined(SPADE_MEASURE_UNPINNED_DENORMS)
constexpr spade::testing::DenormPolicy kDenormPolicy = spade::testing::DenormPolicy::unpinned;
#else
constexpr spade::testing::DenormPolicy kDenormPolicy = spade::testing::DenormPolicy::preserve;
#endif

}  // namespace

TEST(SlangSpirv, FloatControlsPinned) {
    ASSERT_GT(std::size(kSpirvModules), std::size_t{0}) << "no SPIR-V modules to check";

    for (const SpirvModule& entry : kSpirvModules) {
        // EVERY compiled variant, not just the default one (S6 Task 9b). All
        // three are real modules that vkCreateShaderModule will be handed
        // depending on BackendDesc::workgroup_size, so a policy violation
        // present in only one of them is a policy violation that ships.
        ASSERT_GT(entry.variants.count, std::size_t{0})
            << entry.variants.name << ": no compiled variants -- did the embed step run?";

        for (const spade::compute::SpirvVariant& variant : entry.variants.all()) {
            const std::string module = std::string(entry.variants.name) + " (local size " +
                                       std::to_string(variant.workgroup_size) + ")";
            const spade::testing::SpirvScanResult scan =
                spade::testing::scan_spirv(variant.code(), entry.profile, kDenormPolicy);

            EXPECT_TRUE(scan.well_formed) << module << ": not a well-formed SPIR-V module";

            if (entry.integer_only) {
                // An INTEGER-ONLY parity module (S6 Task 7's grid_sort): still
                // PARITY, still P1/P2/P3/P4, but its non-vacuity is proved a
                // different way -- see SpirvModule::integer_only for why.
                EXPECT_EQ(scan.contractable_ops, std::size_t{0})
                    << module << ": declared integer-only but carries " << scan.contractable_ops
                    << " floating-point arithmetic ops -- float math has entered a module declared to have "
                    << "none (grid_sort's comparator, a fill or a copy kernel); either remove it or drop "
                    << "integer_only so PARITY's contractable-op instrument applies";
                EXPECT_GT(scan.store_count, std::size_t{0})
                    << module << ": no OpStore found -- the embedded module is empty or truncated, "
                    << "which is what the contractable-op count would otherwise have caught";
                EXPECT_EQ(scan.entry_point_name, "main")
                    << module << ": unexpected SPIR-V entry point name -- "
                    << "compute/vulkan/step_recorder.cpp's VkPipelineShaderStageCreateInfo::pName "
                    << "must match this exactly";
            } else {
                // The parity/exact gate must not pass vacuously either. A module
                // with no contractable float op satisfies rule P1 by having
                // nothing to check, so a kernel that lost its arithmetic (or an
                // embedding that produced an empty array) would sail through
                // without this.
                EXPECT_GT(scan.contractable_ops, std::size_t{0})
                    << module << ": no contractable float ops found -- the gate would pass "
                    << "vacuously; is the embedded module the right one?";
                EXPECT_EQ(scan.decorated_ops, scan.contractable_ops)
                    << module << ": " << (scan.contractable_ops - scan.decorated_ops)
                    << " of " << scan.contractable_ops
                    << " contractable ops lack NoContraction";
            }

            // Rule P3, asserted by name as well as through the findings loop
            // below: this one is a module-level property with exactly one way to be
            // wrong, and a named expectation reads better in a failure log than
            // "[P3] no OpExecutionMode ...".
            //
            // UNCONDITIONAL AS OF S6 TASK 8. It used to carry a `profile != no_op`
            // guard, because the stub kernel was the one module that legitimately
            // declared no DenormPreserve execution mode (a module with provably
            // zero float arithmetic has no flush hazard to guard against). That
            // kernel and that profile are both retired, so every module in the
            // table is now an arithmetic one and the guard would be a branch nothing
            // takes.
            //
            // Inverted in the measurement build (kDenormPolicy, above).
            if constexpr (kDenormPolicy == spade::testing::DenormPolicy::preserve) {
                EXPECT_TRUE(scan.denorm_preserve_fp32)
                    << module << ": fp32 denormals are not pinned to preserve; is the module "
                    << "compiled -denorm-mode-fp32 preserve? (cmake/SpadeSlang.cmake)";
            } else {
                EXPECT_FALSE(scan.denorm_preserve_fp32)
                    << module << ": P3-unpinned: DenormPreserve 32 present in the measurement "
                    << "build (SPADE_MEASURE_UNPINNED_DENORMS); it must compile without "
                    << "-denorm-mode-fp32 preserve (cmake/SpadeSlang.cmake)";
            }

            // Rule P4 (EVERY profile, S6 Task 8), named for the same reason P3 is
            // named above: one module-level property, one way to be wrong, and a
            // failure log that says which fact broke rather than quoting a string.
            //
            // WHY IT IS THE ONE RULE WITH A DEVICE MEASUREMENT BEHIND IT. Task 8's
            // plan-mandated probe read shaderInt64 on this program's correctness
            // device and got VK_FALSE (tests/test_compute_context.cpp's
            // GpuContext.Int64ProbeMatchesTheDevice prints it), so a module that
            // declares Int64 is undefined behaviour there rather than a reported
            // error. forces_drag and integrate BOTH declared it from S6 Task 6
            // through Task 7 -- layouts.slang mirrored WorldParams::seed as a
            // `uint64_t` and slangc declares the type for any struct that has one.
            // This assertion is what would have caught that on the build it landed.
            EXPECT_FALSE(scan.declares_int64)
                << module << ": declares OpCapability Int64. The correctness device reports "
                << "shaderInt64 == VK_FALSE, so this module cannot legally run there. Mirror 64-bit "
                << "fields as `uint2` (shaders/shared/layouts.slang) and do the arithmetic through "
                << "engine/shaders/u64.slang.";

            // Rule P5 (S6 Task 8 review, minor M8), named for the same reason P3
            // and P4 are: one property, one way to be wrong, and a failure log that
            // says which fact broke.
            //
            // THIS IS THE MACHINE ENFORCEMENT OF THE RULE THE WHOLE PORT RESTS ON.
            // The global constraint's GPU corollary -- transcendentals come from
            // engine/shaders/fp32_math.slang's ports and NEVER from the driver --
            // was obeyed by every kernel from wave A onward and checked by NOTHING
            // until now. A kernel writing `exp(x)` instead of `exp32(x)` compiles,
            // links, runs, and silently produces a vendor- and driver-version-
            // dependent answer that no tolerance band could bound. Wave C is where
            // the hazard went live: dryden.slang, rotors.slang and rng.slang call
            // four ports between them, each one keystroke from the intrinsic of the
            // same name.
            //
            // ZERO IS THE ONLY PASSING VALUE, and it is the measured one for all
            // ten modules -- the only GLSL.std.450 calls anywhere in this tree are
            // FAbs, Floor and Sqrt, and Sqrt is E2's business (allowed under
            // parity by the global constraint, which is why CPU<->GPU is banded).
            EXPECT_EQ(scan.transcendental_ext_inst_ops, std::size_t{0})
                << module << ": calls " << scan.transcendental_ext_inst_ops
                << " GLSL.std.450 exp/log/trig instruction(s) -- the DRIVER's transcendentals, which "
                << "Vulkan specifies to a relative tolerance rather than correctly rounding. Call "
                << "engine/shaders/fp32_math.slang's log32/exp32/sin32/cos32 instead; a function with "
                << "no port yet needs one written, not the intrinsic.";

            for (const spade::testing::SpirvFinding& finding : scan.findings) {
                ADD_FAILURE() << module << " [" << finding.rule << "] " << finding.message;
            }
        }
    }
}

// ===========================================================================
// 4. The probe kernel's host/device contract (S6 Task 4)
// ===========================================================================

namespace {

// Enough of a SPIR-V walk to answer three questions about a module's INTERFACE:
// what its entry point is called, and which (set, binding) pairs it declares.
// spirv_scan.hpp deliberately does not do this -- it is a float-CONTROLS
// policy checker, and interface reflection is a different job with a different
// audience.
struct SpirvInterface {
    std::string entry_point;
    std::vector<std::pair<uint32_t, uint32_t>> set_bindings;  // (set, binding), sorted
};

[[nodiscard]] std::string decode_literal_string(std::span<const uint32_t> words) {
    std::string out;
    for (const uint32_t packed : words) {
        for (int b = 0; b < 4; ++b) {
            const char c = static_cast<char>((packed >> (8 * b)) & 0xFFu);
            if (c == '\0') return out;
            out.push_back(c);
        }
    }
    return out;
}

[[nodiscard]] SpirvInterface scan_interface(std::span<const uint32_t> words) {
    constexpr uint32_t kOpEntryPoint = 15;
    constexpr uint32_t kOpDecorate = 71;
    constexpr uint32_t kDecorationBinding = 33;
    constexpr uint32_t kDecorationDescriptorSet = 34;

    SpirvInterface result;
    std::vector<std::pair<uint32_t, uint32_t>> set_of;      // (id, set)
    std::vector<std::pair<uint32_t, uint32_t>> binding_of;  // (id, binding)

    for (std::size_t i = 5; i < words.size();) {
        const uint32_t word_count = words[i] >> 16;
        const uint32_t opcode = words[i] & 0xFFFFu;
        if (word_count == 0 || i + word_count > words.size()) break;

        if (opcode == kOpEntryPoint && word_count >= 4) {
            // <execution model> <entry id> <name...>
            result.entry_point = decode_literal_string(words.subspan(i + 3, word_count - 3));
        }
        if (opcode == kOpDecorate && word_count >= 4) {
            if (words[i + 2] == kDecorationDescriptorSet) {
                set_of.emplace_back(words[i + 1], words[i + 3]);
            } else if (words[i + 2] == kDecorationBinding) {
                binding_of.emplace_back(words[i + 1], words[i + 3]);
            }
        }
        i += word_count;
    }

    for (const auto& [id, set] : set_of) {
        for (const auto& [binding_id, binding] : binding_of) {
            if (binding_id == id) result.set_bindings.emplace_back(set, binding);
        }
    }
    std::sort(result.set_bindings.begin(), result.set_bindings.end());
    return result;
}

}  // namespace

namespace {

// A real module's words with its `OpExecutionMode <entry> DenormPreserve 32`
// overwritten by OpNops (word count 1, opcode 0); unchanged if it has none.
std::vector<uint32_t> without_denorm_preserve(std::span<const uint32_t> code) {
    namespace st = spade::testing;
    std::vector<uint32_t> words(code.begin(), code.end());
    for (std::size_t i = 5; i < words.size();) {
        const uint32_t word_count = words[i] >> 16;
        if (word_count == 0 || i + word_count > words.size()) break;
        if ((words[i] & 0xFFFFu) == st::spv_op::kExecutionMode && word_count >= 4 &&
            words[i + 2] == st::kExecutionModeDenormPreserve && words[i + 3] == st::kFloatWidth32) {
            std::fill_n(words.begin() + static_cast<std::ptrdiff_t>(i), word_count, uint32_t{1} << 16);
        }
        i += word_count;
    }
    return words;
}

// The same module with exactly one such execution mode, right after its first
// OpEntryPoint (the scanner judges the declaration, not where it sits).
std::vector<uint32_t> with_denorm_preserve(std::span<const uint32_t> code) {
    namespace st = spade::testing;
    std::vector<uint32_t> words = without_denorm_preserve(code);
    for (std::size_t i = 5; i < words.size();) {
        const uint32_t word_count = words[i] >> 16;
        if (word_count == 0 || i + word_count > words.size()) break;
        if ((words[i] & 0xFFFFu) == st::spv_op::kEntryPoint && word_count >= 3) {
            const uint32_t mode[] = {(uint32_t{4} << 16) | st::spv_op::kExecutionMode, words[i + 2],
                                     st::kExecutionModeDenormPreserve, st::kFloatWidth32};
            words.insert(words.begin() + static_cast<std::ptrdiff_t>(i + word_count), std::begin(mode),
                         std::end(mode));
            break;
        }
        i += word_count;
    }
    return words;
}

std::size_t findings_named(const spade::testing::SpirvScanResult& scan, const std::string& rule) {
    return static_cast<std::size_t>(std::count_if(scan.findings.begin(), scan.findings.end(),
                                                  [&](const auto& f) { return f.rule == rule; }));
}

}  // namespace

// Rule P3 in both directions, under both policies, on one real module. The
// inverted rule (DenormPolicy::unpinned, the SPADE_MEASURE_UNPINNED_DENORMS
// build's) has to be provable from a default build, which never compiles an
// unpinned module, so the test makes both forms of the module itself: the
// mode present, and the mode overwritten. It holds in either build.
TEST(SlangSpirv, RuleP3InvertsUnderTheUnpinnedPolicy) {
    namespace st = spade::testing;
    const std::span<const uint32_t> code = kSpirvModules[0].variants.all().front().code();
    const std::vector<uint32_t> pinned = with_denorm_preserve(code);
    const std::vector<uint32_t> unpinned = without_denorm_preserve(code);
    const st::SpirvProfile profile = kSpirvModules[0].profile;

    const st::SpirvScanResult pinned_preserve = st::scan_spirv(pinned, profile, st::DenormPolicy::preserve);
    const st::SpirvScanResult unpinned_preserve = st::scan_spirv(unpinned, profile, st::DenormPolicy::preserve);
    const st::SpirvScanResult pinned_unpinned = st::scan_spirv(pinned, profile, st::DenormPolicy::unpinned);
    const st::SpirvScanResult unpinned_unpinned = st::scan_spirv(unpinned, profile, st::DenormPolicy::unpinned);

    for (const st::SpirvScanResult* scan : {&pinned_preserve, &unpinned_preserve, &pinned_unpinned, &unpinned_unpinned}) {
        EXPECT_TRUE(scan->well_formed);
    }
    EXPECT_TRUE(pinned_preserve.denorm_preserve_fp32);
    EXPECT_FALSE(unpinned_preserve.denorm_preserve_fp32);

    // preserve: the pin passes, its absence is P3.
    EXPECT_EQ(findings_named(pinned_preserve, "P3"), std::size_t{0});
    EXPECT_EQ(findings_named(unpinned_preserve, "P3"), std::size_t{1});
    // unpinned: the pin is P3-unpinned, its absence passes, and P3 never fires.
    EXPECT_EQ(findings_named(pinned_unpinned, "P3-unpinned"), std::size_t{1});
    EXPECT_EQ(findings_named(unpinned_unpinned, "P3-unpinned"), std::size_t{0});
    EXPECT_EQ(findings_named(pinned_unpinned, "P3") + findings_named(unpinned_unpinned, "P3"), std::size_t{0});
    // preserve never reports the inverted rule.
    EXPECT_EQ(findings_named(pinned_preserve, "P3-unpinned") + findings_named(unpinned_preserve, "P3-unpinned"),
              std::size_t{0});
}

// engine/compute/vulkan/probe_runner.hpp states a contract the probe KERNEL has
// to meet -- one descriptor set, two storage buffers at binding 0 (arguments)
// and binding 1 (results), and an entry point Vulkan can find by name. Nothing
// enforces it at compile time: the host writes descriptors by index and the
// kernel declares them by index, in two different languages, and a mismatch
// surfaces as a probe that reads garbage or a pipeline that fails to create at
// run time on one box.
//
// So it is read back out of the compiled module. This is a HOST-ONLY test (no
// Gpu prefix, no device, no skip): the SPIR-V is embedded at build time, so the
// contract is checkable on a CI runner with no GPU at all -- which is exactly
// where a kernel/host binding drift would otherwise go unnoticed until the next
// person with a device ran the suite.
TEST(SlangSpirv, Fp32MathProbeDeclaresTheRunnerBindings) {
    const SpirvInterface iface = scan_interface(
        std::span<const uint32_t>(gen::kSpv_fp32_math_probe, gen::kSpvWordCount_fp32_math_probe));

    // probe_runner.cpp passes "main" as VkPipelineShaderStageCreateInfo::pName.
    EXPECT_EQ(iface.entry_point, "main")
        << "probe_runner.cpp names this entry point when it creates the pipeline";

    ASSERT_EQ(iface.set_bindings.size(), std::size_t{2})
        << "the probe kernel must declare exactly the two buffers run_probe() binds";
    EXPECT_EQ(iface.set_bindings[0].first, spade::compute::kProbeSet);
    EXPECT_EQ(iface.set_bindings[0].second, spade::compute::kProbeArgBinding);
    EXPECT_EQ(iface.set_bindings[1].first, spade::compute::kProbeSet);
    EXPECT_EQ(iface.set_bindings[1].second, spade::compute::kProbeOutBinding);
}

// The embedded module is a real SPIR-V blob, not a truncated or byte-swapped
// one. Cheap, and it is what distinguishes "the embedding worked" from "the
// scanner found nothing to complain about in four bytes of nothing".
TEST(SlangSpirv, EmbeddedModulesCarryTheSpirvHeader) {
    for (const SpirvModule& entry : kSpirvModules) {
        for (const spade::compute::SpirvVariant& variant : entry.variants.all()) {
            const std::string module = std::string(entry.variants.name) + " (local size " +
                                       std::to_string(variant.workgroup_size) + ")";
            const std::span<const uint32_t> words = variant.code();

            ASSERT_GE(words.size(), std::size_t{5}) << module << ": shorter than a SPIR-V header";
            EXPECT_EQ(words[0], spade::testing::kSpirvMagic) << module << ": bad magic";
            const uint32_t major = (words[1] >> 16) & 0xFFu;
            const uint32_t minor = (words[1] >> 8) & 0xFFu;
            EXPECT_EQ(major, 1u) << module << ": unexpected SPIR-V major version";
            // Vulkan 1.3 accepts SPIR-V up to 1.6; the box's Iris Plus reports
            // Vulkan 1.3.215. cmake/SpadeSlang.cmake pins -profile glsl_450, which
            // yields 1.3 -- new enough for compute, old enough for any 1.1+ device.
            EXPECT_GE(minor, 3u) << module << ": SPIR-V older than the pinned profile emits";
            EXPECT_LE(minor, 6u) << module << ": SPIR-V newer than Vulkan 1.3 accepts";
        }
    }
}

// ===========================================================================
// THE SCANNER'S COVERAGE ARITHMETIC, STATED OUT LOUD (S6 Task 9b).
//
// FloatControlsPinned above walks `variants.all()`, so its coverage follows the
// build automatically -- which is exactly why the COUNT needs asserting
// somewhere. An automatic loop over an accidentally-empty or accidentally-
// halved table passes silently and reports nothing; this test is what turns a
// drop in coverage into a failure instead of a smaller number nobody reads.
//
// THE ARITHMETIC: nine schedule kernels, each compiled once per entry in
// compute/backend.hpp's kSupportedWorkgroupSizes, plus fp32_math_probe, which
// has no local-size knob to serve (probe_runner.cpp dispatches it outside the
// schedule) and is therefore a set of one.
//
// THE STANDING GAP THIS ARITHMETIC USED TO NOT CLOSE (S6 hygiene; T9b review
// M7; CLOSED at T11 fix-wave, review W7b -- XS option). Through S6 Task 10b,
// both a `kScheduleKernels` literal here AND `kSpirvModules[]` itself (this
// file, above) were HAND-MAINTAINED: this assert cross-checked one hand count
// against another, not against CMake's own authoritative list of what it
// actually compiled. A TENTH kernel added to engine/CMakeLists.txt's
// `spade_slang_kernel_variants(...)` calls, if its author forgot (or did not
// know) to add a matching row to `kSpirvModules[]`, would have escaped EVERY
// scanner in this file silently -- the failure mode was "nobody edited
// either number," which the old assert could not distinguish from "nobody
// added a tenth kernel at all."
//
// THE FIX: `kernel_manifest.gen.hpp`'s `gen::kCompiledSpirvKernelCount` is
// now CMake's own count -- SpadeSlang.cmake's `spade_slang_kernel()`/
// `spade_slang_kernel_variants()` each APPEND their `<name>` to a GLOBAL
// property as engine/CMakeLists.txt calls them, and
// `spade_slang_write_kernel_manifest()` (called once, after the last such
// call) writes the distinct-name count into this generated header. The
// assert below checks `kSpirvModules[]`'s size against THAT, not against a
// second hand-typed literal: a kernel added to the CMake side alone now
// moves this constant and fails the cross-check for real, rather than
// requiring two independent edits to stay in (accidental) agreement.
//
// COUNT ONLY -- not a NAME-BIJECTION check (the review's larger "S" option,
// which would additionally catch "wrong kernel in the table" rather than
// just "wrong count"). Ticketed for S7 if the stronger form is wanted.
// ===========================================================================
TEST(SlangSpirv, EveryCompiledVariantIsScanned) {
    const std::size_t sizes = spade::compute::kSupportedWorkgroupSizes.size();

    ASSERT_EQ(std::size(kSpirvModules), gen::kCompiledSpirvKernelCount)
        << "kSpirvModules[] (test_slang_layouts.cpp) has " << std::size(kSpirvModules)
        << " entries but CMake compiled " << gen::kCompiledSpirvKernelCount
        << " distinct kernels (kernel_manifest.gen.hpp) -- a kernel was added or "
        << "removed on one side without the other";

    // These two remain hand-typed: they check a NARROWER, separate invariant
    // below (that exactly kScheduleKernels of kCompiledSpirvKernelCount's
    // kernels are schedule kernels with the full workgroup-size family, and
    // that the scanner's total covers kScheduleKernels*sizes + 1 modules)
    // rather than the total-entry-count gap gen::kCompiledSpirvKernelCount
    // above now closes. Folding these into the generated manifest too is
    // exactly the review's larger "S" option.
    //
    // THE PROSE USED TO SAY "NINE" WHILE THE CONSTANT SAID 11, which is worth a
    // line because it is this file's own failure mode in miniature: the number
    // was maintained and the sentence describing it was not, so the sentence
    // quietly became the less trustworthy of the two. It is written relative to
    // the constant now, so it cannot drift again. 12 as of the GPU-sensor leg
    // (sensor_gnss joined the schedule); 14 with the two field-sample kernels
    // (module-API stage 3).
    constexpr std::size_t kScheduleKernels = 14;
#if defined(SPADE_MEASURE_UNPINNED_DENORMS)
    constexpr std::size_t kSingleVariantKernels = 2;  // fp32_math_probe, and M1's denorm_probe
#else
    constexpr std::size_t kSingleVariantKernels = 1;  // fp32_math_probe
#endif

    std::size_t scanned = 0;
    std::size_t multi_variant_kernels = 0;
    for (const SpirvModule& entry : kSpirvModules) {
        scanned += entry.variants.count;
        if (entry.variants.count > 1) {
            ++multi_variant_kernels;
            EXPECT_EQ(entry.variants.count, sizes)
                << entry.variants.name << ": compiled for " << entry.variants.count
                << " local sizes, but compute/backend.hpp admits " << sizes
                << " -- cmake/SpadeSlang.cmake's SPADE_SLANG_WORKGROUP_SIZES has drifted from "
                << "kSupportedWorkgroupSizes";

            // And it is compiled for exactly THOSE sizes, not merely for as
            // many of them: a set of {32, 32, 64} has the right cardinality and
            // would leave 128 uncompiled and unscanned.
            for (const uint32_t supported : spade::compute::kSupportedWorkgroupSizes) {
                EXPECT_NE(entry.variants.for_size(supported), nullptr)
                    << entry.variants.name << ": no variant compiled for supported local size "
                    << supported;
            }
        }

        // ---------------------------------------------------------------
        // ANY TWO VARIANTS OF ONE KERNEL DIFFER IN EXACTLY ONE 32-BIT WORD,
        // AND THAT WORD IS THE LocalSize x-OPERAND.
        //
        // COMPARED BY CONTENT, NOT BY POINTER (Task 9b review, minor M1). The
        // first spelling of this check was `EXPECT_NE(a.words, b.words)` --
        // ADDRESS inequality, which three separate arrays satisfy even if the
        // embed step wrote identical BYTES into all three. That would have left
        // the whole per-size mechanism proven only by a sha256 table in a task
        // report, i.e. by a human reading it once, rather than by CI.
        //
        // AND THE ASSERTION IS THE STRONG FORM RATHER THAN MERE INEQUALITY,
        // because the strong form is what makes this task's central claim a
        // BUILD-GATED property instead of a measurement. `[numthreads(64,1,1)]`
        // was respelled `[numthreads(SPADE_WG,1,1)]` across nine kernels on the
        // argument that it is a build parameterization and NOT an arithmetic
        // change; the evidence was that every wg64 module came out
        // byte-identical to its pre-change artifact. Demanding that the ONLY
        // difference between any two sizes is the local size itself says the
        // same thing from inside the suite, and keeps saying it: any future
        // edit that makes a kernel's CODE depend on SPADE_WG -- a groupshared
        // array, an unrolled fold, a lane-count branch -- lands here as a
        // multi-word diff, whether or not it changes any digest on this box.
        //
        // (Measured across all 28 modules: word 38 for eight kernels, word 32
        // for grid_sort, whose module header is shorter. The index is NOT
        // asserted -- only that there is exactly one and that it holds the two
        // sizes -- because where slangc puts the execution mode is slangc's
        // business, and pinning it would make this a toolchain-version test.)
        // ---------------------------------------------------------------
        const std::span<const spade::compute::SpirvVariant> variants = entry.variants.all();
        for (std::size_t i = 0; i < variants.size(); ++i) {
            for (std::size_t j = i + 1; j < variants.size(); ++j) {
                const spade::compute::SpirvVariant& a = variants[i];
                const spade::compute::SpirvVariant& b = variants[j];
                const std::string pair = std::string(entry.variants.name) + " (local sizes " +
                                         std::to_string(a.workgroup_size) + " vs " +
                                         std::to_string(b.workgroup_size) + ")";

                ASSERT_EQ(a.word_count, b.word_count)
                    << pair << ": different module LENGTHS. Only the local size may differ "
                    << "between two variants of one kernel; a length change means the SOURCE "
                    << "now depends on SPADE_WG.";

                std::vector<std::size_t> differing;
                for (std::size_t w = 0; w < a.word_count; ++w) {
                    if (a.words[w] != b.words[w]) differing.push_back(w);
                }

                // THIS ALSO ENFORCES ONE ENTRY POINT PER MODULE, and nothing
                // else says so. Every [shader("compute")] entry point emits its
                // own LocalSize declaration, so a two-entry-point module differs
                // in two words between variants and a three-entry-point module
                // in three -- for a reason that has nothing to do with
                // lane-dependent codegen. The Jacobi gather met this at 3, then
                // at 2, and split into collision_fill.slang and
                // collision_gather.slang rather than move this number.
                //
                // That is the resolution. A module carries one entry point.
                ASSERT_EQ(differing.size(), std::size_t{1})
                    << pair << ": differ in " << differing.size()
                    << " words, expected exactly 1 (the LocalSize x-operand). "
                    << (differing.empty()
                            ? "ZERO means the embed step wrote the same module twice -- the "
                              "per-size mechanism is not doing anything."
                            : "MORE THAN ONE means this kernel's compiled CODE now depends on "
                              "SPADE_WG, not just its declared local size -- which is exactly "
                              "the local-size-dependent construct the A7 invariance sweep "
                              "exists to forbid.");

                const std::size_t w = differing.front();
                EXPECT_EQ(a.words[w], a.workgroup_size)
                    << pair << ": the one differing word (index " << w << ") is " << a.words[w]
                    << ", not this variant's local size -- so the modules differ somewhere "
                    << "other than their LocalSize declaration.";
                EXPECT_EQ(b.words[w], b.workgroup_size)
                    << pair << ": the one differing word (index " << w << ") is " << b.words[w]
                    << ", not this variant's local size -- so the modules differ somewhere "
                    << "other than their LocalSize declaration.";
            }
        }
    }

    EXPECT_EQ(multi_variant_kernels, kScheduleKernels)
        << "every kernel the schedule dispatches must carry the full workgroup-size family";
    EXPECT_EQ(scanned, kScheduleKernels * sizes + kSingleVariantKernels)
        << "the float-controls gate scanned " << scanned << " modules; expected "
        << (kScheduleKernels * sizes + kSingleVariantKernels);

    std::printf("\n=== SPIR-V policy scan coverage ===\n");
    std::printf("    %zu schedule kernels x %zu local sizes + %zu single-variant = %zu modules\n",
                kScheduleKernels, sizes, kSingleVariantKernels, scanned);
}
