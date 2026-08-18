// ---------------------------------------------------------------------------
// layouts.slang -- the engine's shared state PODs, authored once, in Slang.
// STRUCT DEFINITIONS ONLY: nothing in this file declares a buffer, a binding,
// an entry point or a function. Binding assignments live in bindings.slang
// next door; kernels live under ../kernels/.
//
// WHAT THIS FILE IS FOR (engine design spec D9, section 9 "Single-source
// layouts"). v1's defect was a nine-file struct duplication: the same rigid
// body was spelled out independently in C++ and in every shader that touched
// it, and the two drifted silently. From S6 the GPU-visible shape of every
// registered state row is authored HERE, slangc reflection reports the byte
// offsets Slang actually assigns, and a generated header
// (layout_check.gen.hpp, build tree only) turns those reported offsets into
// static_asserts against the C++ structs. A field reordered on either side is
// then a COMPILE ERROR naming the field, not a silent byte-image divergence
// that surfaces as a wrong trajectory.
//
// THE C++ SIDE REMAINS THE DECLARING SIDE, deliberately, and this is the one
// design point worth stating plainly. state/layout.hpp and its siblings still
// DECLARE the structs the CPU twin computes with; this file MIRRORS them, and
// the generated asserts are the proof that the mirror is exact. Generating the
// C++ headers FROM Slang instead was considered and rejected for S6: the CPU
// twin is the parity reference and its headers carry documentation, defaults,
// operator==, and constexpr helpers that no reflection record can express.
// "Single-source" here therefore means "one place where the two are checked
// against each other, mechanically, every build" -- which is what actually
// prevents the v1 defect.
//
// AUTHORING RULES, all of them load-bearing:
//
//   1. EVERY FIELD OF THE C++ STRUCT APPEARS HERE, INCLUDING EVERY RESERVED
//      AND PADDING LANE (_p0, _r0, _reserved0, ...). They are not decoration:
//      they are what puts each float3 on a 16-byte row, and the generated
//      asserts cover every one of them by name. A pad omitted here would
//      shift every later field and fail the build, which is the point.
//   2. FIELD ORDER IS THE CONTRACT. Do not reorder, do not "tidy".
//   3. float3 == glm::vec3 (12 bytes, std430 alignment 16); float4 ==
//      glm::quat. GLM's quat MEMORY order is x,y,z,w with glm 1.0.1 and no
//      GLM_FORCE_QUAT_DATA_WXYZ (its CONSTRUCTOR takes w,x,y,z -- the
//      coincidence the spec calls out); state/layout.hpp's header note and
//      test_state.cpp's StateLayout.GlmQuatMemoryOrderIsXyzw pin that at
//      runtime, since no static_assert can see into GLM's anonymous union.
//   4. NEW STRUCT => add the two @cpp directives above it (below) or the
//      generator FAILS the build rather than silently skipping it.
//   5. A 64-BIT INTEGER IS MIRRORED AS `uint2`, NEVER AS `uint64_t` (S6 Task
//      8). See the note directly below -- this is a portability rule with a
//      measurement behind it, not a style preference.
//
// ---------------------------------------------------------------------------
// WHY EVERY 64-BIT FIELD BELOW IS A `uint2` (S6 Task 8)
//
// THE MEASUREMENT. Task 8's plan-mandated probe read
// VkPhysicalDeviceFeatures::shaderInt64 on this program's CORRECTNESS DEVICE --
// the Intel Iris Plus every parity band in this engine is calibrated on -- and
// it is **VK_FALSE**. That device cannot legally execute a SPIR-V module that
// declares `OpCapability Int64`.
//
// WHAT THAT MEANT FOR THIS FILE. slangc declares the 64-bit integer TYPE for
// any struct that HAS a `uint64_t` field, whether or not a kernel reads it --
// so mirroring WorldParams::seed as `uint64_t` put OpCapability Int64 into
// forces_drag.spv and integrate.spv from S6 Task 6 onward, purely because those
// kernels subscript `world_params`. Measured on the committed artifacts, both
// carried it. That is a Vulkan VALID-USAGE VIOLATION on this device, and it
// shipped silently because this box has no validation layer installed.
//
// THE FIX IS FREE, WHICH IS WHY IT IS THE FIX. In std430 a `uint2` has size 8
// and alignment 8 -- exactly a `uint64_t`'s -- so replacing one with the other
// moves NO offset, changes NO struct size and changes NO alignment. The
// generated static_asserts prove it per field rather than leaving it to this
// paragraph: `sizeof(cpp::field) == 8` and `offsetof(cpp::field) == <same
// number as before>` still compare against the C++ side's real `uint64_t`.
//
// LANE ORDER IS `.x` = LOW WORD, `.y` = HIGH WORD, and that IS a
// little-endianness assumption -- stated rather than hidden. It is the same one
// the whole engine already makes and could not function without: snapshots are
// byte-wise memcpy of these very structs, and every golden digest is an FNV-1a
// fold over those bytes, so a big-endian host would fail the determinism corpus
// long before it noticed a lane order. (StepParams below is the one record that
// genuinely carries NO such assumption, because BOTH sides spell it as two
// separate `uint` words; see its note.)
//
// THE TRIPWIRE that keeps a future edit from reintroducing the violation is
// engine/testing/spirv_scan.hpp's rule P4: no compiled module may declare
// OpCapability Int64, in ANY profile. Adding a `uint64_t` field here fails the
// build, in the test, naming the module.
//
// THE @cpp DIRECTIVES. Each struct carries, immediately above it:
//
//      // @cpp-type   <fully qualified C++ type>
//      // @cpp-header <include path, relative to engine/>
//
// tools/gen_layout_check.py reads them to know which C++ type each mirrored
// struct must agree with and which header declares it. They live next to the
// struct rather than in a table inside the generator because "what this
// mirrors" is a property of the struct, and a table in another file is the
// second source of truth this whole exercise exists to abolish. The generator
// errors on a reflected struct with no directives AND on a directive naming a
// struct that reflection never saw.
//
// ALIGNMENT, AND WHY THE GENERATED ASSERTS DO NOT DEMAND EQUALITY OF IT. std430
// gives a struct the alignment of its largest member: 16 for anything with a
// float3/float4 in it, 8 for the uint64-carrying scalar records, and 4 for the
// all-scalar ones. Several C++ counterparts are declared `alignas(16)` anyway
// (ContactParams, GridParams), which OVER-aligns them relative to std430
// without changing their size or any offset. Over-alignment is harmless -- a
// buffer offset that satisfies 16 satisfies 4 -- so the generated header
// asserts alignof(C++) % <reflected alignment> == 0 (never under-aligned)
// while asserting sizeof and every offset EXACTLY.
// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------
// RngStream -- spade::rng::Stream (core/rng.hpp). Not a state row of its own:
// it is a MEMBER of ImuSensorRow and DrydenState, which is exactly why it must
// be mirrored -- a nested struct's own field offsets are part of its
// enclosing row's byte image.
//
// Two 8-byte rows: state | (cached_gauss, has_cached). `cached_gauss` and
// `has_cached` are MEMBERS rather than compiler-local state because the
// Box-Muller partner value is real, snapshot-carried randomness (core/rng.hpp's
// "NO HIDDEN STATE" note) -- the GPU port must carry it identically or a
// restored run draws a different sequence.
// ---------------------------------------------------------------------------
// @cpp-type   spade::rng::Stream
// @cpp-header core/rng.hpp
struct RngStream {
    // splitmix64 state; ALL of the entropy. `uint2` rather than `uint64_t`
    // (S6 Task 8) -- .x is the low word, .y the high word; see this file's
    // header note. engine/shaders/u64.slang is the exact 64-bit arithmetic
    // engine/shaders/rng.slang performs on this pair, and it is EXACT rather
    // than approximate, which is why the GPU's stream position is a
    // bit-identity claim and not a banded one.
    uint2 state;
    float cached_gauss;  // the unconsumed half of a Box-Muller pair
    uint  has_cached;    // 1 iff cached_gauss holds a live value
};

// ---------------------------------------------------------------------------
// BodyState -- one rigid body's authoritative state. Eight 16-byte rows; see
// state/layout.hpp for the frame conventions (pos/vel/force_acc world-frame;
// omega_body/inv_inertia_diag/torque_acc/specific_force body-frame) and for
// why the two accumulators deliberately do not share a frame.
// ---------------------------------------------------------------------------
// @cpp-type   spade::BodyState
// @cpp-header state/layout.hpp
struct BodyState {
    float3 pos;               // world-frame position, m
    // Per-body contact-proxy radius override, metres, >= 0 (D-S6-2). 0 is a
    // SENTINEL meaning "defer to this world's ContactParams::proxy_radius" --
    // see physics::effective_proxy_radius() in physics/contacts.hpp, the one
    // canonical C++ predicate this field's GPU-side readers mirror exactly
    // (collision_static.slang). Former std430 pad `_p0`; still keeps `orient`
    // on row 1, since it occupies exactly the same 4 bytes -- this is a
    // RENAME of existing storage, not a layout change.
    float  proxy_radius;
    float4 orient;            // body->world rotation, unit quaternion (x,y,z,w)
    float3 vel;               // world-frame linear velocity, m/s
    float  mass;              // kg (the row-2 slot the pad would occupy)
    float3 omega_body;        // body-frame angular velocity, rad/s
    float  _p1;               // std430 pad
    float3 inv_inertia_diag;  // body-frame 1/I diagonal, 1/(kg m^2)
    uint   flags;             // per-body predicate bits (none assigned yet)
    float3 force_acc;         // world-frame accumulated force, N
    float  _p2;               // std430 pad
    float3 torque_acc;        // BODY-frame accumulated torque, N m
    float  _p3;               // std430 pad
    float3 specific_force;    // body-frame specific force (IMU), m/s^2
    float  _p4;               // std430 pad
};

// ---------------------------------------------------------------------------
// WorldParams -- one world's parameter record, indexed by world id, so a
// single dispatch covering N worlds reads its own world's row.
//
// TAIL PADDING IS REAL AND INTENTIONAL: named fields end at byte 56 and the
// 16-byte base alignment rounds the stride to 64. Bytes 56..63 belong to no
// field on either side; ArenaSet zero-fills them, which is what makes them
// deterministic in a snapshot rather than heap garbage. Growth spends
// `_reserved0` first, then that tail, moving nothing.
// ---------------------------------------------------------------------------
// @cpp-type   spade::WorldParams
// @cpp-header state/layout.hpp
struct WorldParams {
    float3   gravity;        // world-frame gravity, m/s^2
    float    air_density;    // kg/m^3
    float3   wind;           // world-frame mean wind, m/s
    float    _p;             // std430 pad -- keeps row 2 aligned
    uint     body_capacity;  // fixed slots in this world's body partition
    uint     body_count;     // live bodies (mirror; ArenaSet is the authority)
    uint2    seed;           // this world's rng stream seed (uint2: header note)
    uint2    _reserved0;     // reserved for versioned growth; must stay 0
};

// ---------------------------------------------------------------------------
// DragBodyRow -- one drag element (physics/forces.hpp). Four 16-byte rows.
// `body_slot` is WORLD-LOCAL: it indexes the same world's `bodies` partition,
// not a global arena slot.
// ---------------------------------------------------------------------------
// @cpp-type   spade::physics::DragBodyRow
// @cpp-header physics/forces.hpp
struct DragBodyRow {
    uint   body_slot;      // world-local index into this world's bodies span
    uint   enabled;        // active-high; 0 == inert
    uint   mode;           // drag_mode::quadratic (0) or ::componentwise (1)
    float  area;           // m^2, reference area -- quadratic mode only
    float3 local_pos;      // r, BODY-frame offset from the body's COM, m
    float  _p0;            // std430 pad -- keeps `local_orient` on row 2
    float4 local_orient;   // local->body rotation (x,y,z,w)
    float3 coeffs;         // quadratic: coeffs.x is Cd; componentwise: per-axis, kg/m
    float  _p1;            // std430 pad
};

// ---------------------------------------------------------------------------
// ContactParams -- the static-collision material record (physics/contacts.hpp).
// Two 16-byte rows of scalars, no vectors, so std430 gives it alignment 4
// while the C++ struct is alignas(16); the size (32) and every offset agree
// exactly, which is what the generated asserts pin (see the alignment note in
// this file's header).
// ---------------------------------------------------------------------------
// @cpp-type   spade::physics::ContactParams
// @cpp-header physics/contacts.hpp
struct ContactParams {
    float restitution_e;   // 0 = perfectly inelastic, 1 = lossless bounce
    float friction_mu;     // Coulomb coefficient; one mu, no static/dynamic split
    float baumgarte_beta;  // positional-correction gain, fraction of excess depth
    float slop;            // penetration allowance, m
    float proxy_radius;    // sphere-proxy radius, m
    float _r0;             // reserved; must stay 0
    float _r1;
    float _r2;
};

// ---------------------------------------------------------------------------
// GridParams -- the dynamic broad phase's one knob (physics/grid.hpp). One
// 16-byte row; same alignment situation as ContactParams above.
// ---------------------------------------------------------------------------
// @cpp-type   spade::physics::GridParams
// @cpp-header physics/grid.hpp
struct GridParams {
    float cell_size;  // broad-phase cell edge, m; must be >= 2 * proxy_radius
    float _r0;        // reserved; must stay 0
    float _r1;
    float _r2;
};

// ---------------------------------------------------------------------------
// RotorRow -- one rotor element (vehicles/rotor.hpp). Five 16-byte rows.
// `omega`/`omega_cmd` are STATE (shaft speed and its command), which is why a
// rotor row rides every snapshot rather than being rebuilt from a descriptor.
// ---------------------------------------------------------------------------
// @cpp-type   spade::vehicles::RotorRow
// @cpp-header vehicles/rotor.hpp
struct RotorRow {
    uint   body_slot;     // world-local index into this world's bodies span
    uint   enabled;       // active-high; 0 == inert
    float  tau;           // RPM-lag time constant, s; <= 0 means no lag
    float  radius;        // rotor radius R, m
    float3 local_pos;     // r, BODY-frame hub offset from the body's COM, m
    float  spin_dir;      // +1 / -1 about the thrust axis; 0 = no reaction torque
    float4 local_orient;  // local->body rotation (x,y,z,w)
    float  omega;         // STATE: shaft speed, rad/s, non-negative
    float  omega_cmd;     // STATE: commanded shaft speed, rad/s, non-negative
    float  thrust_coeff;  // k_T in T_static = k_T * omega^2, N s^2
    float  torque_coeff;  // k_Q in Q = k_Q * omega^2, N m s^2, non-negative
    float  _r0;           // reserved for the gyroscopic/BEMT opt-ins; must stay 0
    float  _r1;
    float  _r2;
    float  _r3;
};

// ---------------------------------------------------------------------------
// ImuSensorRow -- one IMU's table row (sensors/imu.hpp). Eight 16-byte rows.
// EVERYTHING THE SENSOR NEEDS TO CONTINUE IS IN THE ROW -- the bias random
// walk, the phase counter, the rng stream and the ring write cursor -- so a
// snapshot carries it all and a restored run reproduces the sample sequence.
// The ring reference is implicit: sensor slot g owns ring slots
// [g * kRingDepth, (g+1) * kRingDepth).
// ---------------------------------------------------------------------------
// @cpp-type   spade::sensors::ImuSensorRow
// @cpp-header sensors/imu.hpp
struct ImuSensorRow {
    uint      body_slot;     // world-local index into this world's bodies span
    uint      kind;          // sensor_kind::; 0 == none == inert
    uint      rate_divider;  // emit one sample every N substeps; 1 == every substep
    uint      phase;         // substeps elapsed in the current sensor period
    float3    mount_pos;     // r, BODY-frame offset from the body's COM, m
    float     _p0;           // std430 pad -- keeps `mount_orient` on row 2
    float4    mount_orient;  // mount->body rotation (x,y,z,w)
    float     sigma_a;       // accel white noise, per-sample stddev, m/s^2
    float     sigma_g;       // gyro white noise, per-sample stddev, rad/s
    float     sigma_ba;      // accel bias random-walk step stddev, m/s^2 per sample
    float     sigma_bg;      // gyro bias random-walk step stddev, rad/s per sample
    float3    bias_a;        // accel bias state, MOUNT frame, m/s^2
    float     _p1;           // std430 pad
    float3    bias_g;        // gyro bias state, MOUNT frame, rad/s
    float     _p2;           // std430 pad
    RngStream noise;         // this sensor's own stream; ALL of its randomness
    uint2     last_index;    // newest sample index written; 0 == none yet (uint2: header note)
    uint2     _reserved0;    // reserved for versioned growth; must stay 0
};

// ---------------------------------------------------------------------------
// ImuSample -- one ring entry (sensors/imu.hpp). Three 16-byte rows.
// `index` is per-sensor monotonic starting at 1; `tick` is the step it was
// produced in.
// ---------------------------------------------------------------------------
// @cpp-type   spade::sensors::ImuSample
// @cpp-header sensors/imu.hpp
struct ImuSample {
    float3 accel;  // mount-frame specific force, m/s^2
    float  _p0;    // std430 pad -- keeps `gyro` on row 1
    float3 gyro;   // mount-frame angular velocity, rad/s
    float  _p1;    // std430 pad
    uint2  index;  // per-sensor monotonic sample index, starts at 1 (uint2: header note)
    uint2  tick;   // the step this sample was produced in
};

// ---------------------------------------------------------------------------
// DrydenState -- one world's gust-filter state (world/medium.hpp). Five 8-byte
// rows; NOT padded to 16-byte rows, because it carries no vector member --
// std430 alignment 8, matching the C++ struct exactly.
//
// u, v0/v1, w0/w1 are NORMALIZED filter states (unit stationary variance,
// dimensionless), not gust velocities: dryden_turbulence() applies sigma.
// ---------------------------------------------------------------------------
// @cpp-type   spade::DrydenState
// @cpp-header world/medium.hpp
struct DrydenState {
    RngStream stream;  // this world's gust stream; ALL of the randomness
    float     u;       // longitudinal, first-order state
    float     v0;      // lateral, second-order state 0
    float     v1;      // lateral, second-order state 1
    float     w0;      // vertical, second-order state 0
    float     w1;      // vertical, second-order state 1
    float     _p0;     // reserved; also what keeps every byte named
};

// ---------------------------------------------------------------------------
// DrydenParams -- one world's gust CONFIGURATION (world/medium.hpp): the scale
// lengths, the sigmas and the frozen-turbulence reference airspeed.
//
// WHY IT IS MIRRORED AND BOUND (bindings.slang binds it as a per-world storage
// buffer). The GPU Dryden port evaluates the same filter the CPU twin does,
// and every coefficient it needs comes from this record -- WorldParams carries
// gravity, mean wind and air density but NOT the gust parameters. It is not a
// registered state array (it is configuration, not evolving state), so the
// device copy is backend-internal derived storage, which is precisely the
// category S6 is allowed to add. Its layout is a real CPU<->GPU contract
// either way, so it is pinned here now rather than after the port is written.
// ---------------------------------------------------------------------------
// @cpp-type   spade::DrydenParams
// @cpp-header world/medium.hpp
struct DrydenParams {
    float scale_u;             // L_u, longitudinal scale length, m
    float scale_v;             // L_v, lateral scale length, m
    float scale_w;             // L_w, vertical scale length, m
    float sigma_u;             // longitudinal gust stddev, m/s
    float sigma_v;             // lateral gust stddev, m/s
    float sigma_w;             // vertical gust stddev, m/s
    float reference_airspeed;  // V, the frozen-turbulence airspeed, m/s
    float _reserved0;          // reserved for versioned growth; must stay 0
};

// ---------------------------------------------------------------------------
// ReplayConfig -- the last of the nine registered arrays (sim/simulation.hpp):
// the step decomposition and the world-set config hash a restore checks
// against. NO KERNEL READS IT, and it is mirrored and bound anyway: the
// binding registry's contract is that it covers the registered walk exactly,
// so an array present in the walk and absent from the registry would be the
// silent omission test_slang_layouts.cpp exists to catch.
// ---------------------------------------------------------------------------
// @cpp-type   spade::ReplayConfig
// @cpp-header sim/simulation.hpp
struct ReplayConfig {
    uint2 dt_ns;        // the STEP duration, ns (uint2: this file's header note)
    uint  substeps;     // substeps per step
    uint  _pad;         // keeps config_hash 8-byte aligned; must stay 0
    uint2 config_hash;  // config_hash(WorldSetDesc)
    uint2 _reserved0;   // reserved for versioned growth; must stay 0
};

// ---------------------------------------------------------------------------
// SdfNodeRow -- one node of a world's flattened, postfix SDF program
// (world/sdf.hpp's SdfNode; compute/sdf_program.hpp's device row). Two 16-byte
// rows.
//
// `transform` is WORLD-LOCAL and is uploaded unrebased -- the kernel adds its
// world's SdfWorldRange::transform_begin when it indexes `sdf_transforms`. See
// compute/sdf_program.hpp for why the upload keeps the host program's own
// bytes rather than rebasing them, and for why this mirrors SdfNodeRow rather
// than spade::SdfNode itself (glm::vec4's host alignment is 4; std430 asks for
// 16).
// ---------------------------------------------------------------------------
// @cpp-type   spade::compute::SdfNodeRow
// @cpp-header compute/sdf_program.hpp
struct SdfNodeRow {
    uint   kind;       // SdfPrim; meaningful only when op == SdfOp::none
    uint   op;         // SdfOp
    uint   transform;  // world-local index into this world's transform range
    uint   _pad;       // std430 pad -- keeps `params` on row 1
    float4 params;     // per-kind parameter pack; see world/sdf.hpp
};

// ---------------------------------------------------------------------------
// SdfTransformRow -- one node's PRE-INVERTED pose (world/sdf.hpp's
// SdfTransform), spelled as the four COLUMNS of glm's column-major mat4 rather
// than as a float4x4. Five 16-byte rows.
//
// WHY COLUMNS AND NOT float4x4, stated here because it looks like fussiness and
// is not. Slang's float4x4 and glm's mat4 do not agree on whether the first 16
// bytes are a row or a column, and the answer additionally depends on a
// compiler flag (-matrix-layout-*) -- so a float4x4 field would make the
// kernel's arithmetic silently depend on a build setting rather than on this
// file. Naming the columns removes the question. It costs nothing besides:
// spirv_scan.hpp's rule P2 forbids OpMatrixTimesVector outright (a matrix
// product's accumulation order is unspecified in SPIR-V), so the kernel has to
// spell the product out column by column in glm's own pinned order regardless
// of how the matrix is stored.
// ---------------------------------------------------------------------------
// @cpp-type   spade::compute::SdfTransformRow
// @cpp-header compute/sdf_program.hpp
struct SdfTransformRow {
    float4 col0;   // world_to_local[0]
    float4 col1;   // world_to_local[1]
    float4 col2;   // world_to_local[2]
    float4 col3;   // world_to_local[3]
    float  scale;  // uniform scale; d_world = scale * d_local
    float  _r0;    // reserved; must stay 0
    float  _r1;
    float  _r2;
};

// ---------------------------------------------------------------------------
// SdfWorldRange -- where world w's program sits inside the two concatenated
// SDF buffers. One 16-byte row, indexed by world id exactly like WorldParams.
// `node_count == 0` is an EMPTY program (a legal world with no geometry) and
// the kernel must answer it exactly as world/sdf.cpp's sample() does for an
// empty node list.
// ---------------------------------------------------------------------------
// @cpp-type   spade::compute::SdfWorldRange
// @cpp-header compute/sdf_program.hpp
struct SdfWorldRange {
    uint node_begin;       // first node of this world's program in `sdf_nodes`
    uint node_count;       // how many nodes it has; 0 == empty program
    uint transform_begin;  // first transform of this world's program
    uint _r0;              // reserved; must stay 0
};

// ---------------------------------------------------------------------------
// GridEntryRow -- ONE SORTED BROAD-PHASE KEY (S6 Task 7). Six 4-byte lanes,
// 24 bytes, std430 alignment 4 (no vector member), so this is one of the
// records whose C++ counterpart is NOT over-aligned and whose sizeof IS the
// array stride.
//
// It mirrors compute/grid_entry.hpp's GridEntryRow, NOT physics::GridEntry --
// see that header for why the device row is declared flat and separately (the
// CPU record nests a GridCell and is a std::vector element with no layout
// discipline of its own, because it is scratch rather than state).
//
// WHAT IT IS. physics/grid.cpp's build stage emits one of these per active,
// in-range body; the sort stage orders them by grid_entry_less()'s exact key
// (grid.cpp:329-335) -- (world, cell.z, cell.y, cell.x, slot) -- and the
// resolve stage sweeps them in that order. `world == 0xFFFFFFFF` is the
// SENTINEL that stands for the CPU's three kinds of skipped body plus the
// power-of-two padding a bitonic network needs; it sorts strictly after every
// live key because the world field is compared UNSIGNED.
//
// DERIVED, BACKEND-INTERNAL STORAGE, never registered state: nothing here is
// snapshot-walked, and the buffer is rebuilt from scratch by the first
// dispatch of every CollisionDynamic pass -- the same lifetime GridScratch has
// on the CPU.
// ---------------------------------------------------------------------------
// @cpp-type   spade::compute::GridEntryRow
// @cpp-header compute/grid_entry.hpp
struct GridEntryRow {
    uint world;   // owning world id, or 0xFFFFFFFF (the sentinel)
    int  cell_x;  // floor(pos.x / cell_size), SIGNED and unbounded (grid.hpp FIX 3)
    int  cell_y;
    int  cell_z;
    uint slot;    // GLOBAL body slot for a live entry; the entry's own index for a sentinel
    uint _r0;     // reserved; must stay 0
};

// ---------------------------------------------------------------------------
// StepParams -- the PER-STEP host write (S6 Task 5 review round 1, finding C2;
// consumed on the device for the first time by S6 Task 6). One 8-byte row, one
// element: the host rewrites it before every vkQueueSubmit, which is exactly
// what a push constant baked into a record-once command buffer cannot do.
//
// TWO uint LANES, NOT A uint64_t -- see compute/step_params.hpp's own note:
// a `uint64_t` field would make every kernel that reads it require
// OpCapability Int64, which is not a requirement worth imposing on the
// correctness device for a counter. The two lanes also carry no host-
// endianness assumption, because both sides read and write WORDS.
//
// S6 TASK 8 VINDICATED THAT CALL AS A MEASUREMENT, not merely as taste: the
// correctness device reports shaderInt64 == VK_FALSE, so a `uint64_t` here
// would have been undefined behaviour rather than an unwanted requirement.
// This record remains the only one in the file whose C++ counterpart is ALSO
// two words -- every other 64-bit mirror is a `uint2` over a real host
// `uint64_t` and therefore does assume little-endian lane order (header note).
// ---------------------------------------------------------------------------
// @cpp-type   spade::compute::StepParams
// @cpp-header compute/step_params.hpp
struct StepParams {
    uint tick_lo;  // low 32 bits of the step's tick
    uint tick_hi;  // high 32 bits
};

// ---------------------------------------------------------------------------
// StepWitness -- the one row the Integrate kernel WRITES BACK to prove it read
// StepParams (and PassParams) at all. One 16-byte row, one element,
// backend-internal derived storage that is never snapshotted and never
// digested. compute/step_params.hpp's own doc comment carries the full
// argument for why wave A publishes a witness instead of inventing a physical
// dependency on the tick it does not have.
// ---------------------------------------------------------------------------
// @cpp-type   spade::compute::StepWitness
// @cpp-header compute/step_params.hpp
struct StepWitness {
    uint tick_lo;  // low 32 bits of the tick the last Integrate dispatch read
    uint tick_hi;  // high 32 bits
    uint substep;  // PassParams::substep of the dispatch that wrote last
    uint _r0;      // reserved; must stay 0
};

// ---------------------------------------------------------------------------
// PassParams -- the per-dispatch scalars every kernel needs, delivered as a
// push constant (bindings.slang).
//
// MIRRORED AS OF S6 TASK 5. T5's compute/vulkan/step_recorder.hpp defines the
// C++ struct that writes into this push-constant range
// (spade::compute::PassParams), so the two @cpp directives below claim it
// exactly like every other mirror.
//
// NO CONTACT/GRID FIELDS AS OF S6 TASK 6b (checkpoint-1 ruling). T6 carried
// `ContactParams contact`/`GridParams grid` here as world 0's records, on the
// theory that a future batched CollisionDynamic kernel would read them as one
// per-dispatch material for the whole batch. Task 6b's PassParams decision
// tree found no kernel anywhere reads `pass_params.contact` or
// `pass_params.grid` (grepped across engine/shaders/** and
// step_recorder.cpp -- the only write was step_recorder.cpp copying world 0's
// records in, never a read back out), so the fields are removed under this
// program's own rule: kernels cannot read what does not exist. Per-world
// material now reaches the device exclusively through the two PER-WORLD
// buffers (`contact_params` binding 19, `grid_params` binding 20) -- see
// bindings.slang's section C for why per-world buffers, not a per-batch
// constant, are the right shape once worlds are allowed to disagree. T7's
// batched dynamic-collision kernels will define their own consumption path
// when they land, informed by this ruling rather than by reviving a per-batch
// single record.
// ---------------------------------------------------------------------------
// @cpp-type   spade::compute::PassParams
// @cpp-header compute/vulkan/step_recorder.hpp
struct PassParams {
    uint  world_count;       // worlds in this dispatch
    uint  body_capacity;     // per-world slots in the bodies partition
    uint  element_capacity;  // per-world slots in drag_bodies / rotors
    uint  sensor_capacity;   // per-world slots in imu_sensors
    float h;                 // substep duration, s
    uint  substep;           // index of this substep within the step

    // -- the CollisionDynamic key domain (S6 Task 7) ------------------------
    //
    // FIVE LANES, EVERY ONE SHAPE OR DISPATCH DATA -- which is what makes them
    // legal here where the removed `contact`/`grid` were not. The three
    // `grid_*` lanes describe how the key array is PARTITIONED (how many
    // entries, how big one independently-sorted segment is, how many body
    // slots map into one segment) and are computed from StepShape alone by
    // compute/grid_entry.hpp's grid_domain_of(); the two `sort_*` lanes name
    // WHICH STAGE of the bitonic network a particular dispatch is, and differ
    // from dispatch to dispatch inside the one recorded chain -- which is
    // exactly what a push constant baked at record time is for. NONE of them
    // is a physics parameter: cell_size still comes from `grid_params[world]`
    // and the material from `contact_params[world]`, per world, per the
    // checkpoint-1 ruling above.
    uint  grid_entry_count;    // N: entries in the whole key array
    uint  grid_segment;        // entries per independently-sorted segment (power of two, >= 1)
    uint  grid_segment_slots;  // body slots mapped into one segment
    uint  sort_k;              // bitonic stage: the block size being merged
    uint  sort_j;              // bitonic stage: the compare-exchange distance
};
