// The standard module set: today's engine as modules (plan stages 1-3). Every
// pass names the GPU recipe of its own CPU function (builtin_cpu_for below).
#include "sim/module.hpp"

#include "physics/schedule.hpp"

namespace spade::modules {
namespace {

// Dryden provides the wind field: the mean wind plus this substep's gust,
// sampled after the filter advances (the read of dryden.state orders it).
constexpr FieldDecl kDrydenFields[] = {{.name = "wind", .kind = FieldKind::vec3, .unit = "m/s"}};
constexpr QuantityAccess kDrydenAccess[] = {{"dryden.state", Access::write}};
constexpr QuantityAccess kDrydenSampleAccess[] = {
    {"dryden.state", Access::read}, {"world.params", Access::read}, {"field.wind", Access::write}};
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
                                           {"rotor.state", Access::write},
                                           {"field.density", Access::read},
                                           {"field.wind", Access::read}};
constexpr PassDecl kRotorPasses[] = {
    {.name = "forces", .phase = Phase::forces, .access = kRotorAccess, .cpu = &physics::pass_rotor_forces,
     .gpu = compute::GpuRecipe::rotors}};

// Rotors then drag is an fp32 accumulation order the goldens pin.
constexpr QuantityAccess kDragAccess[] = {{"body.pose", Access::read},
                                          {"body.wrench", Access::accumulate},
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
                                         {"imu.state", Access::write}};
constexpr PassDecl kImuPasses[] = {
    {.name = "synthesize", .phase = Phase::sensors, .access = kImuAccess, .cpu = &physics::pass_sensor_imu,
     .gpu = compute::GpuRecipe::sensor_imu}};

constexpr QuantityAccess kGnssAccess[] = {{"body.pose", Access::read}, {"gnss.state", Access::write}};
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
    // Set order is today's registration order for the modules that own state
    // (drag, dryden, imu, rotor, gnss: the golden walk, which stage 4 hands to
    // the modules), then the stateless ones.
    return {
        {.name = "drag", .passes = kDragPasses},
        {.name = "dryden", .passes = kDrydenPasses, .fields = kDrydenFields},
        {.name = "imu", .passes = kImuPasses},
        {.name = "rotor", .passes = kRotorPasses},
        {.name = "gnss", .passes = kGnssPasses},
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
