// The standard module set: today's engine as modules (plan stages 1-4). Every
// pass names the GPU recipe of its own CPU function (builtin_cpu_for below),
// and every module that owns state declares its arrays.
#include "sim/module.hpp"

#include "physics/forces.hpp"
#include "physics/schedule.hpp"
#include "sensors/gnss.hpp"
#include "sensors/imu.hpp"
#include "sensors/rings.hpp"
#include "sim/builtin_state.hpp"
#include "sim/simulation.hpp"  // the spawn records
#include "vehicles/rotor.hpp"
#include "world/medium.hpp"

namespace spade::modules {
namespace {

// THE BUILT-INS' STATE: the seven arrays create() registers from these
// declarations, with the extents its capacities follow (sim/simulation.cpp's
// registration block has the walk-order rules). Each ring holds kRingDepth
// samples per sensor, so sensor g owns ring rows [g * kRingDepth, (g + 1) *
// kRingDepth). The notes below came here from the hand-written registration
// calls these declarations replaced (stage 4).
constexpr ArrayDecl kDragArrays[] = {{.name = "drag_bodies",
                                      .elem_size = attached_row_size<physics::DragBodyRow>(),
                                      .extent = Extent::per_element,
                                      .spawn_size = sizeof(DragElementSpawn),
                                      .init = &builtin::init_drag_row,
                                      .validate = &builtin::validate_drag_spawn}};

// One Dryden filter row per world. It was the FIRST production registration of
// the turbulence state (Task 16 shipped the model and its tests before anything
// in the engine registered the array), and registering it is what makes
// "snapshot the world, restore it, resume the identical gust sequence" true
// rather than aspirational.
constexpr ArrayDecl kDrydenArrays[] = {{.name = "dryden", .elem_size = row_size<DrydenState>()}};

// The sensor table and its output rings (Task 19). REGISTERED, which is what
// makes "snapshot mid-flight, restore, and the IMU resumes the identical noise
// sequence AND the identical unread samples" true: the rows carry the bias
// random walk, the rate-divider phase, the rng stream and the ring write
// cursor; the ring array carries the samples themselves. None of it lives in a
// Simulation member, deliberately.
//
// The ring is DIRECT-INDEXED, never slot-allocated: a ring window's lifetime
// is its sensor's, so an independent alloc/free would be a second lifecycle to
// keep in step with the first: the despawn cascade zeroes a sensor's window
// when it frees the sensor (sim/module.hpp's ATTACHED ROWS).
constexpr ArrayDecl kImuArrays[] = {
    {.name = "imu_sensors",
     .elem_size = attached_row_size<sensors::ImuSensorRow>(),
     .extent = Extent::per_sensor,
     .spawn_size = sizeof(ImuSensorSpawn),
     .init = &builtin::init_imu_row,
     .validate = &builtin::validate_imu_spawn},
    {.name = "imu_ring",
     .elem_size = row_size<sensors::ImuSample>(),
     .extent = Extent::per_row,
     .owner = "imu_sensors",
     .depth = sensors::kRingDepth}};

// The rotor table (Task 18). REGISTERED, and that is NOT optional the way it
// arguably is for the parameter-only drag rows: a RotorRow carries `omega` --
// genuine dynamic state with its own time constant -- and `omega_cmd`, the
// command in force at the snapshot instant. A snapshot that missed them would
// restore a vehicle whose rotors are at the wrong speed, or spinning down
// toward zero, and whose next second of flight differs (vehicles/rotor.hpp
// says exactly this).
//
// SIZED BY THE FORCE-ELEMENT CAPACITY, which the drag table also uses. Rotors
// and drag bodies are both force elements (spec §3) and share ONE declared
// budget per world, so a world declaring N force elements can hold at most N
// rotors -- but the two arrays are separately allocated, which costs a world
// that never spawns a vehicle N unused RotorRows (80 bytes each; 320 bytes for
// the corpus's largest world). Accepted deliberately: the alternative is
// either a per-kind capacity in the world FILE (S5's format, for a distinction
// a world author should not have to predict) or a conditional registration,
// which would make the state layer's shape depend on its contents.
constexpr ArrayDecl kRotorArrays[] = {{.name = "rotors",
                                       .elem_size = attached_row_size<vehicles::RotorRow>(),
                                       .extent = Extent::per_element,
                                       .spawn_size = sizeof(builtin::RotorSpawn),
                                       .init = &builtin::init_rotor_row,
                                       .validate = &builtin::refuse_direct_rotor}};

// GNSS, the second sensor, and the first array appended below replay_config.
// Its ring has the IMU ring's shape: sensor g owns ring rows [g * kRingDepth,
// (g+1) * kRingDepth). kRingDepth lives in sensors/rings.hpp, not in imu.hpp --
// it is shared ring vocabulary rather than an IMU constant, so reusing it here
// is correct and not a copy.
//
// THAT REUSE IS A CHOICE AND NOT AN INHERITANCE. A GNSS fix arrives at roughly
// 5-10 Hz against the IMU's ~1 kHz, so 64 slots is ~6-12 s of history here
// against ~64 ms there. The depths happening to match is convenient, not
// principled; if the rates ever justify a separate kGnssRingDepth, that is a
// deliberate commit, not a drive-by.
constexpr ArrayDecl kGnssArrays[] = {
    {.name = "gnss_sensors",
     .elem_size = attached_row_size<sensors::GnssSensorRow>(),
     .extent = Extent::per_sensor,
     .spawn_size = sizeof(GnssSensorSpawn),
     .init = &builtin::init_gnss_row,
     .validate = &builtin::validate_gnss_spawn},
    {.name = "gnss_ring",
     .elem_size = row_size<sensors::GnssFix>(),
     .extent = Extent::per_row,
     .owner = "gnss_sensors",
     .depth = sensors::kRingDepth}};

// Dryden provides the wind field: the mean wind plus this substep's gust,
// sampled after the filter advances (the read of dryden.dryden orders it).
constexpr FieldDecl kDrydenFields[] = {{.name = "wind", .kind = FieldKind::vec3, .unit = "m/s"}};
constexpr QuantityAccess kDrydenAccess[] = {{"dryden.dryden", Access::write}};
constexpr QuantityAccess kDrydenSampleAccess[] = {
    {"dryden.dryden", Access::read}, {"world.params", Access::read}, {"field.wind", Access::write}};
constexpr PassDecl kDrydenPasses[] = {
    {.name = "advance", .phase = Phase::fields, .access = kDrydenAccess, .cpu = &physics::pass_medium_update,
     .gpu = compute::GpuRecipe::medium_update},
    {.name = "sample", .phase = Phase::fields, .access = kDrydenSampleAccess, .cpu = &physics::pass_dryden_sample,
     .gpu = compute::GpuRecipe::dryden_sample}};

// The environment provides gravity and density, copied from the world's params.
// Stateless; appended to the set, so no module's set index moves.
constexpr FieldDecl kEnvironmentFields[] = {{.name = "gravity", .kind = FieldKind::vec3, .unit = "m/s^2"},
                                            {.name = "density", .kind = FieldKind::scalar, .unit = "kg/m^3"}};
constexpr QuantityAccess kEnvironmentSampleAccess[] = {
    {"world.params", Access::read}, {"field.gravity", Access::write}, {"field.density", Access::write}};
constexpr PassDecl kEnvironmentPasses[] = {{.name = "sample",
                                            .phase = Phase::fields,
                                            .access = kEnvironmentSampleAccess,
                                            .cpu = &physics::pass_environment_sample,
                                            .gpu = compute::GpuRecipe::environment_sample}};

constexpr QuantityAccess kRotorAccess[] = {{"body.pose", Access::read},
                                           {"body.wrench", Access::accumulate},
                                           {"rotor.rotors", Access::write},
                                           {"field.density", Access::read},
                                           {"field.wind", Access::read}};
constexpr PassDecl kRotorPasses[] = {
    {.name = "forces", .phase = Phase::forces, .access = kRotorAccess, .cpu = &physics::pass_rotor_forces,
     .gpu = compute::GpuRecipe::rotors}};

// Rotors then drag is an fp32 accumulation order the goldens pin.
constexpr QuantityAccess kDragAccess[] = {{"body.pose", Access::read},
                                          {"body.wrench", Access::accumulate},
                                          {"drag.drag_bodies", Access::read},
                                          {"field.density", Access::read},
                                          {"field.wind", Access::read}};
constexpr std::string_view kDragAfter[] = {"rotor.forces"};
constexpr PassDecl kDragPasses[] = {{.name = "forces",
                                     .phase = Phase::forces,
                                     .access = kDragAccess,
                                     .after = kDragAfter,
                                     .cpu = &physics::pass_drag,
                                     .gpu = compute::GpuRecipe::drag}};

constexpr QuantityAccess kImuAccess[] = {{"body.pose", Access::read},
                                         {"body.specific_force", Access::read},
                                         {"imu.imu_sensors", Access::write},
                                         {"imu.imu_ring", Access::write}};
constexpr PassDecl kImuPasses[] = {
    {.name = "synthesize", .phase = Phase::sensors, .access = kImuAccess, .cpu = &physics::pass_sensor_imu,
     .gpu = compute::GpuRecipe::sensor_imu}};

constexpr QuantityAccess kGnssAccess[] = {
    {"body.pose", Access::read}, {"gnss.gnss_sensors", Access::write}, {"gnss.gnss_ring", Access::write}};
constexpr PassDecl kGnssPasses[] = {
    {.name = "synthesize", .phase = Phase::sensors, .access = kGnssAccess, .cpu = &physics::pass_sensor_gnss,
     .gpu = compute::GpuRecipe::sensor_gnss}};

// Kinematic behaviors write poses before any field is sampled (Q2), so they
// run first in Fields; force behaviors run after every built-in force. Both
// declare what they touch, so the compiler checks them like any other pass. A
// force behavior may overwrite force_acc (the drone stand sets -m*g), so it
// declares a write; being placed last orders it after the accumulators.
constexpr QuantityAccess kKinematicAccess[] = {{"body.pose", Access::write}};
constexpr QuantityAccess kForceBehaviorAccess[] = {{"body.wrench", Access::write}};
constexpr PassDecl kBehaviorPasses[] = {
    {.name = "kinematic", .phase = Phase::fields, .placement = Placement::first, .access = kKinematicAccess,
     .cpu = &physics::pass_behaviors_kinematic, .gpu = compute::GpuRecipe::behaviors_kinematic},
    {.name = "force", .phase = Phase::forces, .placement = Placement::last, .access = kForceBehaviorAccess,
     .cpu = &physics::pass_behaviors_force, .gpu = compute::GpuRecipe::behaviors_force},
};

constexpr QuantityAccess kContactAccess[] = {{"body.pose", Access::write}};
constexpr PassDecl kStaticPasses[] = {{.name = "resolve",
                                       .phase = Phase::constraints,
                                       .access = kContactAccess,
                                       .cpu = &physics::pass_collision_static,
                                       .gpu = compute::GpuRecipe::collision_static}};
// Both contact passes correct pos and vel: dynamic runs on static's result.
constexpr std::string_view kDynamicAfter[] = {"static_contact.resolve"};
constexpr PassDecl kDynamicPasses[] = {{.name = "resolve",
                                        .phase = Phase::constraints,
                                        .access = kContactAccess,
                                        .after = kDynamicAfter,
                                        .cpu = &physics::pass_collision_dynamic,
                                        .gpu = compute::GpuRecipe::collision_dynamic}};

constexpr QuantityAccess kIntegrateAccess[] = {{"body.pose", Access::write},
                                               {"body.wrench", Access::write},
                                               {"body.specific_force", Access::write},
                                               {"field.gravity", Access::read}};
constexpr PassDecl kIntegratePasses[] = {
    {.name = "integrate", .phase = Phase::integrate, .access = kIntegrateAccess, .cpu = &physics::pass_integrate,
     .gpu = compute::GpuRecipe::integrate}};

}  // namespace

ModuleSet standard_modules() {
    // Set order is today's registration order for the modules that own state,
    // then the stateless ones. That order is the golden walk: drag, dryden, imu
    // and rotor carry the legacy marker, so their arrays register before
    // replay_config, in this order; gnss registers after it.
    return {
        {.name = "drag", .passes = kDragPasses, .state = kDragArrays, .legacy_walk = true},
        {.name = "dryden", .passes = kDrydenPasses, .fields = kDrydenFields, .state = kDrydenArrays,
         .legacy_walk = true},
        {.name = "imu", .passes = kImuPasses, .state = kImuArrays, .legacy_walk = true},
        {.name = "rotor", .passes = kRotorPasses, .state = kRotorArrays, .legacy_walk = true},
        {.name = "gnss", .passes = kGnssPasses, .state = kGnssArrays},
        {.name = "behaviors", .passes = kBehaviorPasses},
        {.name = "static_contact", .passes = kStaticPasses},
        {.name = "dynamic_contact", .passes = kDynamicPasses},
        {.name = "integrate", .passes = kIntegratePasses},
        {.name = "environment", .passes = kEnvironmentPasses, .fields = kEnvironmentFields},
    };
}

PassFn builtin_cpu_for(compute::GpuRecipe recipe) noexcept {
    switch (recipe) {
        case compute::GpuRecipe::none: return nullptr;
        case compute::GpuRecipe::behaviors_kinematic: return &physics::pass_behaviors_kinematic;
        case compute::GpuRecipe::medium_update: return &physics::pass_medium_update;
        case compute::GpuRecipe::rotors: return &physics::pass_rotor_forces;
        case compute::GpuRecipe::drag: return &physics::pass_drag;
        case compute::GpuRecipe::behaviors_force: return &physics::pass_behaviors_force;
        case compute::GpuRecipe::collision_static: return &physics::pass_collision_static;
        case compute::GpuRecipe::collision_dynamic: return &physics::pass_collision_dynamic;
        case compute::GpuRecipe::integrate: return &physics::pass_integrate;
        case compute::GpuRecipe::sensor_imu: return &physics::pass_sensor_imu;
        case compute::GpuRecipe::sensor_gnss: return &physics::pass_sensor_gnss;
        case compute::GpuRecipe::environment_sample: return &physics::pass_environment_sample;
        case compute::GpuRecipe::dryden_sample: return &physics::pass_dryden_sample;
    }
    return nullptr;
}

}  // namespace spade::modules
