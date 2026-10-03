// ---------------------------------------------------------------------------
// bindings.slang -- THE BINDING REGISTRY. Binding assignments only: this file
// declares one shader parameter per device-visible buffer and nothing else. No
// struct is defined here (they are all in layouts.slang, imported below) and
// no function or entry point is declared here.
//
// WHAT CONSUMES IT. slangc reflection over THIS file (via
// cmake/SpadeSlang.cmake, `-no-codegen -reflection-json`) reports the set and
// binding index Slang actually assigned to each parameter, and
// tools/gen_layout_check.py turns those into bindings.gen.hpp:
//
//      namespace spade::compute::gen {
//          inline constexpr uint32_t kSet_state              = 0;
//          inline constexpr uint32_t kBinding_bodies         = 1;
//          ...
//      }
//
// T5's buffer table and every kernel dispatch read those constants, so the
// host and the shaders cannot disagree about which slot holds which array --
// there is exactly one authored place, this file, and the C++ side is
// generated from it.
//
// REFLECTION WITHOUT AN ENTRY POINT is why this file can stay declaration-only.
// slangc's `-no-codegen` mode type-checks and lays out a module without
// generating any code, and reports every global shader parameter in it even
// though no kernel references them (verified on slang v2026.14.1 -- see this
// task's report). The alternative, reflecting a KERNEL and reading its
// parameters, would have made the registry depend on which kernel happened to
// touch which buffer; an unused binding would vanish from the generated header
// the moment a kernel stopped reading it.
//
// ---------------------------------------------------------------------------
// SET LAYOUT
//
// ONE descriptor set, set 0 ("state"), holding every storage buffer, plus one
// push-constant block. One set rather than several because every kernel in
// S6 reads from the same pool of world-partitioned arrays: splitting them
// across sets would buy per-frequency update granularity that nothing in this
// engine has (the whole state pool is bound once and stepped in place) at the
// cost of a second layout to keep in sync.
//
// ---------------------------------------------------------------------------
// WHAT IS BOUND, AND WHY EACH ONE
//
// A. THE NINE BOUND REGISTERED ARRAYS (bindings 0..8), in registered-walk
//    order: world_params, bodies, body_generation, drag_bodies, dryden,
//    imu_sensors, imu_ring, rotors, replay_config. replay_config is bound even
//    though no kernel will read it, for completeness (layouts.slang's
//    ReplayConfig comment).
//
//    THE WALK HOLDS ELEVEN ARRAYS, NOT NINE. gnss_sensors and gnss_ring are
//    bound at 23..24 with the GNSS slot->world sibling at 25 -- see section E.
//
//    THEY WERE DELIBERATELY UNBOUND FOR EXACTLY ONE LEG, AND THE REASON IS
//    WORTH KEEPING. When the arena was registered no kernel read either array,
//    so binding them would have meant authoring Slang mirrors for GnssSensorRow
//    and GnssFix under the strict layout gate purely so a descriptor could
//    point at buffers nothing reads -- cost with no consumer. That held until
//    sensor_gnss.slang existed, which was the next leg. The sentence
//    "deliberately unbound" read as a settled end state when the honest form
//    was "unbound UNTIL THE SENSOR EMITS": A SENTENCE TRUE WHEN WRITTEN, PHRASED
//    SO NOTHING IN IT EXPIRES. What caught the reversal was not a reader but a
//    test -- test_slang_layouts.cpp asserted in the NEGATIVE that those arrays
//    had no binding, so adding one failed loudly and forced this paragraph to
//    be rewritten rather than quietly outgrown.
//
//    state_mirror.cpp's per-entry has_binding flag is still what makes
//    REGISTERED and BOUND separable, and five `.slot_to_world` siblings remain
//    unbound on section B's rule.
//
//    HOW THIS IS GUARDED, STATED HONESTLY -- AN EARLIER FORM OF THIS
//    PARAGRAPH CLAIMED MORE THAN IS TRUE. It said the registry "mirrors that
//    list EXACTLY -- test_slang_layouts.cpp asserts the correspondence in both
//    directions so neither an addition nor an omission can pass silently."
//    Only one of those directions exists:
//
//      SLANG SIDE, GUARDED. A binding added to or removed from this file moves
//      kBindingCount_state, which is generated from slangc reflection, and
//      BindingRegistryHasNoUnlistedEntries fails.
//
//      REGISTRY SIDE, NOT GUARDED BY ANY TEST IN THAT FILE. Its
//      kRegisteredArrays is a hardcoded list of string literals compared
//      against the test's own kExpectedRegistry; nothing there walks the live
//      StateRegistry, so an array registered in sim/simulation.cpp passes every
//      test in that file silently.
//
//    THE LIST MUST STAY HARDCODED -- its own comment is right that a test
//    reading its expectations out of the artifact under test would assert
//    nothing. The independence that makes it a good expectation is the same
//    property that makes it blind to the thing it names; both halves are true
//    at once, which is why the old sentence read as correct. The real guard on
//    the registry side is RUNTIME: StateMirror::upload() walks the live
//    registry and refuses an array it has no buffer for. WARNING: that path is
//    Vulkan-only and its tests are GTEST_SKIP()-gated on vulkan_available(),
//    and A SKIPPED TEST COUNTS AS PASSED -- so on a box with no device, a
//    registry-side addition is invisible: CPU suite fully green, backend broken
//    at runtime.
//
// B. FOUR .slot_to_world SIBLINGS (bindings 9..12). Every registered array has
//    one (state/arenas.hpp: register_array("bodies", ...) contributes both
//    "bodies" and "bodies.slot_to_world" to the walk), but only four carry
//    information a KERNEL needs, and binding a map that answers nothing would
//    be a descriptor slot spent on a second source of truth. The rule: a
//    sibling is bound iff a kernel dispatched over that array's GLOBAL slot
//    space must recover the owning world (to index WorldParams) or the slot's
//    liveness (kInvalidWorld == free).
//
//      BOUND (dispatch is per-slot over a slot-allocated array):
//        bodies.slot_to_world       -- Integrate, CollisionStatic and both
//                                      halves of ForceElements dispatch one
//                                      thread per body slot across all worlds.
//        imu_sensors.slot_to_world  -- SensorSynthesis dispatches per sensor
//                                      slot (kernels/sensor_imu.slang), which
//                                      is its own extent.
//
//      BOUND BUT NOT READ BY ANY KERNEL, as of S6 Task 8 -- RECORDED HERE
//      RATHER THAN LEFT FOR A READER TO DISCOVER, and flagged for review:
//        drag_bodies.slot_to_world  -- the rule above predicted "ForceElements
//        rotors.slot_to_world          dispatches per drag row / per rotor
//                                      row". It does not. Both force kernels
//                                      dispatch ONE THREAD PER BODY and fold
//                                      that body's elements serially, for the
//                                      race/determinism/workgroup-invariance
//                                      reasons forces_drag.slang's header sets
//                                      out at length -- so each thread already
//                                      knows its world from
//                                      bodies.slot_to_world and never needs an
//                                      element's own map.
//
//                                      They are LEFT BOUND rather than removed
//                                      because a binding index is an ABI: 10
//                                      and 12 sit in the middle of a dense
//                                      range and dropping them would renumber
//                                      every later buffer (through
//                                      grid_entries at 21) for two descriptors'
//                                      worth of saving. That is a decision for
//                                      the coordinator, not a drive-by, and it
//                                      is noted in the S6 Task 8 report.
//
//      NOT BOUND, and each for a stated reason rather than as an oversight:
//        world_params.slot_to_world -- capacity 1 per world, so global slot w
//                                      IS world w. The map is the identity and
//                                      a kernel already has the world index as
//                                      its dispatch coordinate.
//        dryden.slot_to_world       -- capacity 1 per world; identical to the
//                                      world_params case.
//        body_generation.slot_to_world -- body_generation is a DIRECT-INDEXED
//                                      sibling of bodies sharing its slot
//                                      space, and its own map is uniformly
//                                      "free" (sim/simulation.cpp says so at
//                                      the register_array call). bodies'
//                                      map is the answer; a second binding
//                                      would be a second source of truth.
//        imu_ring.slot_to_world     -- ring windows are direct-indexed off the
//                                      sensor slot and never slot-allocated,
//                                      so this map is uniformly "free" too.
//        replay_config.slot_to_world -- one row, not per-world, never read on
//                                      the device.
//
//      All five stay UNBOUND rather than being reserved at fixed indices: an
//      index reserved for a buffer nobody creates is a descriptor a validation
//      layer will complain about, and adding one later appends to the end of
//      the set without moving anything already there.
//
// C. DERIVED PER-WORLD PARAMS BUFFERS -- THREE AS OF S6 TASK 6b, NOT ONE:
//    dryden_params (binding 13), contact_params (binding 19) and grid_params
//    (binding 20), each a per-world configuration record no registered array
//    carries. dryden_params leads because it came first (S6 Task 5): the gust
//    port needs the scale lengths, sigmas and reference airspeed on the
//    device and WorldParams carries none of them. None of the three is a
//    registered array -- they are configuration, not evolving state -- and
//    device-side derived storage uploaded from the host is explicitly the
//    category the canonical global constraint permits: "S6 adds NO
//    register_array call" (compute/step_params.hpp, compute/vulkan/backend.hpp).
//    (This site said "registered state is what is frozen" -- a paraphrase that
//    lost its subject; see compute/vulkan/state_mirror.hpp. What makes these
//    three legal is that they are DERIVED, which is unchanged.) See
//    layouts.slang's DrydenParams/ContactParams/GridParams comments.
//
//    ALL THREE HAVE READERS AS OF S6 TASK 8, which closes the last of this
//    file's "staged ahead of its consumer" notes. dryden_params was staged at
//    Task 5 and is read by kernels/medium_update.slang (the advance) and by
//    kernels/field_dryden.slang (the gust, written to the field row since
//    module-API stage 3); contact_params has been read by CollisionStatic since Task 6 and by
//    CollisionDynamic since Task 7; grid_params was staged at Task 6b and is
//    read by grid_build.slang.
//
// D. ONE PUSH-CONSTANT BLOCK: PassParams (24 bytes as of S6 Task 6b, inside
//    the 128-byte guaranteed minimum), carrying only the per-dispatch
//    scalars. It carried ContactParams/GridParams by value through Task 6;
//    Task 6b's PassParams decision tree removed both fields (layouts.slang's
//    PassParams doc comment has the full ruling) once every kernel/
//    step_recorder.cpp grep came back with no reader.
//
// ---------------------------------------------------------------------------
// ACCESS QUALIFIERS. Everything a pass mutates is RWStructuredBuffer; the
// three read-only inputs (the .slot_to_world maps, dryden_params,
// replay_config) are StructuredBuffer. This is documentation with teeth: a
// kernel that tries to write a map the host owns will not compile.
//
// EXPLICIT INDICES, not compiler-assigned ones. Every parameter carries
// [[vk::binding(index, set)]] rather than letting Slang number them in
// declaration order, because these indices are an ABI the host's descriptor
// writes depend on: reordering two declarations in this file would otherwise
// silently swap two buffers. The generator reads back what Slang assigned, so
// a typo'd duplicate index shows up as a duplicate in bindings.gen.hpp and
// fails test_slang_layouts.cpp's distinctness assertion.
// ---------------------------------------------------------------------------

// THE @set DIRECTIVE. Reflection reports the NUMBER of a descriptor set, never
// a name, so the generated header's kSet_<name> constant needs one authored
// here. Same reasoning as layouts.slang's @cpp directives: the name belongs
// next to the declaration, not in a table inside the generator. The generator
// errors on a reflected parameter whose set has no directive.
//
// @set 0 state

import layouts;

// -- A. the nine registered arrays, in registered-walk order ----------------

[[vk::binding(0, 0)]] RWStructuredBuffer<WorldParams>  world_params;
[[vk::binding(1, 0)]] RWStructuredBuffer<BodyState>    bodies;
[[vk::binding(2, 0)]] RWStructuredBuffer<uint>         body_generation;
[[vk::binding(3, 0)]] RWStructuredBuffer<DragBodyRow>  drag_bodies;
[[vk::binding(4, 0)]] RWStructuredBuffer<DrydenState>  dryden;
[[vk::binding(5, 0)]] RWStructuredBuffer<ImuSensorRow> imu_sensors;
[[vk::binding(6, 0)]] RWStructuredBuffer<ImuSample>    imu_ring;
[[vk::binding(7, 0)]] RWStructuredBuffer<RotorRow>     rotors;
[[vk::binding(8, 0)]] RWStructuredBuffer<ReplayConfig> replay_config;

// -- B. the .slot_to_world siblings kernels need ----------------------------
//
// Named with an underscore where the registered array name has a dot
// ("bodies.slot_to_world" -> bodies_slot_to_world): Slang identifiers cannot
// carry a dot. The generated constants keep the Slang spelling
// (kBinding_bodies_slot_to_world) and test_slang_layouts.cpp is what pins the
// correspondence back to the registered name.

[[vk::binding(9,  0)]] StructuredBuffer<uint> bodies_slot_to_world;
[[vk::binding(10, 0)]] StructuredBuffer<uint> drag_bodies_slot_to_world;
[[vk::binding(11, 0)]] StructuredBuffer<uint> imu_sensors_slot_to_world;
[[vk::binding(12, 0)]] StructuredBuffer<uint> rotors_slot_to_world;

// -- C. derived per-world configuration -------------------------------------

[[vk::binding(13, 0)]] StructuredBuffer<DrydenParams> dryden_params;

// The static SDF program, per world, flattened (S6 Task 6). Same DERIVED,
// backend-internal category as dryden_params above and for the same reason:
// CollisionStatic needs the world's geometry on the device, and the geometry
// is CONFIGURATION (Simulation's per-world SdfProgram), not registered state,
// so it reaches the device through an upload rather than through the walk.
// Three buffers because a world set holds N independent programs -- see
// compute/sdf_program.hpp for why the split is nodes/transforms/ranges and why
// a node's `transform` index stays world-local.
[[vk::binding(14, 0)]] StructuredBuffer<SdfNodeRow>      sdf_nodes;
[[vk::binding(15, 0)]] StructuredBuffer<SdfTransformRow> sdf_transforms;
[[vk::binding(16, 0)]] StructuredBuffer<SdfWorldRange>   sdf_world_ranges;

// The PER-WORLD contact material (S6 Task 6). Same derived category again, and
// a buffer rather than a push-constant field for a reason the corpus makes
// concrete: tests/golden/scenarios/bounce.scenario.yaml is a restitution
// LADDER whose four worlds carry four DIFFERENT ContactParams, so a single
// per-dispatch record cannot serve them. CollisionStatic reads THIS, exactly
// as the CPU pass reads its own world's `w.contacts`. THROUGH S6 TASK 6,
// PassParams ALSO carried a `contact` field -- world 0's record, meaningful
// only to a future BATCHED CollisionDynamic form -- but Task 6b's PassParams
// decision tree removed it (no kernel ever read it); see this file's binding
// 20 comment and layouts.slang's PassParams doc comment for the ruling.
[[vk::binding(19, 0)]] StructuredBuffer<ContactParams> contact_params;

// The PER-WORLD broad-phase grid config (S6 Task 6b, checkpoint-1 ruling).
// Same derived category, same per-world reasoning as contact_params directly
// above: a world set may be heterogeneous, so a single per-dispatch record
// cannot serve every world's cell_size. Spelling mirrors contact_params/
// binding 19 verbatim, one binding index later.
//
// SANCTIONED STAGING, DEAD UNTIL T7 -- same adjudication as dryden_params
// (binding 13, "T8 consumer, harmless until then"): no kernel reads this
// buffer yet (CollisionDynamic, the one pass that will, is still the stub
// slot in step_recorder.hpp's table). It is added now, ahead of its
// consumer, because closing the per-world config record for every S6 Task 6
// buffer at once is this task's whole point -- see this file's binding 19
// comment for why CollisionStatic already reads contact_params per world
// rather than per batch, and grid_params exists so T7's CollisionDynamic can
// do the same instead of reviving the per-batch PassParams.grid this task
// just removed.
[[vk::binding(20, 0)]] StructuredBuffer<GridParams> grid_params;

// The DYNAMIC BROAD PHASE'S KEY ARRAY (S6 Task 7). READ-WRITE, and only the
// second device-WRITTEN buffer in this file that is not registered state (the
// first is step_witness below) -- the three CollisionDynamic dispatches
// build it, sort it in place, and then sweep it, all on the device, within one
// pass of one substep.
//
// NOT UPLOADED AND NOT READ BACK ON THE STEP PATH. It is the device's analogue
// of physics/grid.hpp's GridScratch: pure derived storage that is rebuilt from
// the body positions at the start of every CollisionDynamic pass and carries
// no meaning across passes, which is exactly why adding it needs no
// register_array call and does not touch the registered walk at all.
// StateMirror::read_grid_entries() copies it out for the GpuGridSort tests
// alone -- a diagnostic path, one submit, never called by step().
//
// SIZE IS A DISPATCH-SHAPE DECISION, not a physics one: compute/
// grid_entry.hpp's grid_domain_of() turns StepShape::batch_dynamic_collision
// into either ONE power-of-two segment covering every world's slots (the
// batched shape, D8's payoff) or world_count independently-sorted segments of
// next_pow2(body_capacity) each. Both produce the same physics; see that
// header for the argument and tests/test_gpu_parity.cpp's two shower cases for
// the measurement.
[[vk::binding(21, 0)]] RWStructuredBuffer<GridEntryRow> grid_entries;

// The CollisionDynamic GATHER's start-of-iteration shadow of pos/vel/mass/
// radius, one row per grid entry (layouts.slang's GatherBodyRow). Written by
// the fill dispatch, read-only to the gather itself, and dead between passes
// exactly as grid_entries is.
//
// IT IS A SEPARATE BUFFER BECAUSE THERE IS NO BARRIER TO PUT BETWEEN THE READS
// AND THE WRITES. A gather thread reads every one of its partners and writes
// only itself; with threads mapped to entries, another workgroup may already
// have written a partner, and Vulkan offers no device-wide barrier inside a
// dispatch. Shadowing the two fields this pass mutates is what makes the
// iteration a Jacobi one rather than a race.
[[vk::binding(22, 0)]] RWStructuredBuffer<GatherBodyRow> body_snapshot;

// Every world's field sample row (module-API stage 3; layouts.slang's
// FieldSampleRow), indexed by world. Written by field_environment and
// field_dryden in the Fields phase, read by rotors, forces_drag and integrate
// later in the same substep. DERIVED, like grid_entries: never uploaded,
// zero-filled at create, read back only by Simulation::field_samples() (a
// diagnostic). Binding 26 because 25 was the last in use.
[[vk::binding(26, 0)]] RWStructuredBuffer<FieldSampleRow> field_samples;

// ---------------------------------------------------------------------------
// E. THE SECOND SENSOR KIND (bindings 23..25) -- registered state, bound
// because a kernel now reads it.
//
// APPENDED AT 23..25 RATHER THAN INSERTED AT 9..10 BESIDE THE OTHER REGISTERED
// ARRAYS, AND THAT IS DELIBERATE. These indices are hand-authored
// [[vk::binding(n, 0)]] attributes, so putting the arrays in walk order would
// have renumbered fifteen existing bindings for cosmetic tidiness -- and this
// file already tolerates numeric order differing from file order (step_params
// and step_witness are 17/18, declared below grid_entries at 21). The C++ side
// never hardcodes a number: it reads gen::kBinding_* out of the reflection
// record, so appending costs nothing and renumbering would have risked a silent
// swap that only a parity failure would catch.
//
// THE SIBLING IS BOUND, AND SECTION B'S RULE IS WHY. "A sibling is bound iff a
// kernel dispatched over that array's GLOBAL slot space needs it" --
// sensor_gnss.slang dispatches per GNSS sensor slot and reads
// gnss_sensors_slot_to_world on its second line to find the owning world,
// exactly as sensor_imu.slang reads the IMU's at binding 11. gnss_ring's own
// sibling is NOT bound: nothing dispatches over the ring.
// ---------------------------------------------------------------------------
[[vk::binding(23, 0)]] RWStructuredBuffer<GnssSensorRow> gnss_sensors;
[[vk::binding(24, 0)]] RWStructuredBuffer<GnssFix>       gnss_ring;
[[vk::binding(25, 0)]] StructuredBuffer<uint>            gnss_sensors_slot_to_world;

// -- C'. the per-STEP exchange (S6 Task 5's finding C2, closed by Task 6) ----
//
// step_params is the ONE buffer whose contents change between two submits of
// the same recorded command buffer: the host writes the step's tick into it
// (HOST_COHERENT, persistently mapped) immediately before each vkQueueSubmit.
// Task 5 created and wrote it but deliberately left it unbound and unread --
// "wiring it into bindings.slang and proving a kernel can read it is Task 6's
// job, the first task with a real consumer". This is that wiring.
//
// step_witness is where the Integrate kernel PUBLISHES what it read, so the
// far side of the mechanism is observable from the host after a readback.
// READ-WRITE, unlike the rest of sections B and C: it is one of the two
// device-written buffers in this file that are not registered state (the
// other, added by S6 Task 7, is `grid_entries` at binding 21 -- see its own
// comment above).
[[vk::binding(17, 0)]] StructuredBuffer<StepParams>    step_params;
[[vk::binding(18, 0)]] RWStructuredBuffer<StepWitness> step_witness;

// -- D. the per-dispatch push constant --------------------------------------

[[vk::push_constant]] ConstantBuffer<PassParams> pass_params;
