#pragma once

// ---------------------------------------------------------------------------
// camera_host.hpp -- 3.3: the host's door to the camera sensor kind, a C++ header surface (the lead's ruling: no flat C ABI; KAT links Spade
// statically from the installed prefix). Plain structs and Result<T>; installed with the other public headers.
//
// WHAT IT IS. A CameraHost wraps one Simulation and owns what a host needs beside it and that is not part of the registered state: a body's label and local
// keypoints, each rotor's blade azimuth, the per-camera due-frame cursors. It steps ONE TICK AT A TIME inside step(), polling every camera's due-frame ring and
// advancing the blade azimuths after each tick, so nothing is lost to a 64-deep ring however many ticks a host step spans, and every record carries its tick.
//
// WHY A HELPER AND NOT A SIMULATION MEMBER. psi lives outside the registered state (physics/blade_azimuth.hpp), so a save/restore of the Simulation brings omega
// back and not psi. The CameraHost therefore has save() and load() of its OWN state (the azimuths, the cursors, the labels and keypoints); a caller that snapshots the
// Simulation snapshots the CameraHost beside it, and a restored run's psi equals the uninterrupted run's (tested). No digest moves: nothing here is in the
// registered state.
// ---------------------------------------------------------------------------

#include <cstddef>
#include <cstdint>
#include <memory>
#include <span>
#include <vector>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include "core/error.hpp"
#include "sim/simulation.hpp"

namespace spade {

// A camera as a host states it: exposure in seconds and a rate in whole substeps (rate_hz is the host's to convert; a divider that is not a whole number of
// ticks is refused). The pose of a labelled object is at the exposure MIDPOINT: capture_time_ns below.
struct CameraSpec {
    glm::vec3 mount_pos_body_m{0.0f};
    glm::quat mount_orient{1.0f, 0.0f, 0.0f, 0.0f};  // camera -> body (v_body = q * v_cam); camera axes x right, y down, z forward
    uint32_t lens_kind = 0;                          // sensors::lens_kind::
    uint32_t width = 0;
    uint32_t height = 0;
    uint32_t channels = 1;                           // sensors::camera_channel:: bits
    float fx = 0.0f, fy = 0.0f, cx = 0.0f, cy = 0.0f;
    float k[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    double exposure_s = 0.0;
    uint32_t rate_divider = 1;                       // whole substeps
};

// What a per-env reset changes, in place: the mount orientation and, optionally, the lens. The camera, its ring and its frame sequence are kept.
struct CameraReset {
    glm::quat mount_orient{1.0f, 0.0f, 0.0f, 0.0f};
    bool set_lens = false;
    uint32_t lens_kind = 0;
    float fx = 0.0f, fy = 0.0f, cx = 0.0f, cy = 0.0f;
    float k[4] = {0.0f, 0.0f, 0.0f, 0.0f};
};

struct DueFrame {
    uint32_t camera = 0;           // the camera's index in this host, in the order added
    uint64_t frame_seq = 0;        // per camera, from 1; a gap is a dropped frame
    uint64_t tick = 0;             // the tick the frame is due in
    int64_t capture_time_ns = 0;   // the exposure midpoint on the sim clock
};

struct StepReport {
    uint64_t ticks = 0;
    std::vector<DueFrame> due;     // every frame that fell due in these ticks, in (tick, camera) order
};

// What the HOST knows about a rotor that the plant does not carry: the propeller's blade count (the part file's) and the motor's pole pairs (the motor's).
// The registered state holds neither, so the host that chose the vehicle hands them over once (set_rotor_info) and gets them back beside omega and psi, in
// one record for the consumer. 0 = not given. They feed vehicles::derive_frequencies (blade pass = blades x shaft, electrical = pole pairs x shaft).
struct RotorInfo {
    uint32_t blade_count = 0;      // 1 to 16, or 0 for not given
    uint32_t pole_pairs = 0;       // 0 for a brushed motor or not given, else up to 64
};

struct RotorReading {
    std::vector<float> omega;      // the plant's lagged shaft speed after the last tick, rad/s (RotorRow::omega, not the command)
    std::vector<float> psi;        // the blade azimuth after the last tick, rad in [0, 2 pi)
    std::vector<uint32_t> blade_count;  // as set by set_rotor_info (0: not given)
    std::vector<uint32_t> pole_pairs;
};

// The keypoint output record (20 bytes, KAT's wire shape). Declared now; Rendering's seam fills it (the visibility test and depth_m are theirs).
struct KeypointRecord {
    uint32_t gate_index;
    uint8_t corner;
    uint8_t visibility;  // COCO: 0 absent, 1 in frame but occluded, 2 visible
    uint8_t pad[2];
    float u;
    float v;
    float depth_m;
};
static_assert(sizeof(KeypointRecord) == 20);

class CameraHost {
public:
    [[nodiscard]] static Result<CameraHost> open(Simulation& sim);

    [[nodiscard]] Result<uint32_t> add_camera(BodyRef body, const CameraSpec& spec);
    [[nodiscard]] Result<void> reset_camera(uint32_t camera, const CameraReset& reset);

    [[nodiscard]] Result<void> set_label(BodyRef body, uint32_t label);
    [[nodiscard]] Result<void> set_keypoints(BodyRef body, std::span<const glm::vec3> local_points);
    [[nodiscard]] Result<uint32_t> label_of(BodyRef body) const;
    [[nodiscard]] Result<std::span<const glm::vec3>> keypoints_of(BodyRef body) const;

    // Steps `ticks` ticks. REFUSED, invalid_argument, "the simulation was restored without the camera host's state", when the Simulation's tick is not the tick
    // this host expects: the host records the Simulation's tick with every step and with every load(), so a Simulation restored (or otherwise moved) behind its back
    // is DETECTED, never a silent inconsistency (its psi, its due-frame cursors and its frame sequences would be those of another moment). The remedy is to
    // load() the host state saved beside the snapshot.
    [[nodiscard]] Result<StepReport> step(uint64_t ticks);

    [[nodiscard]] Result<RotorReading> rotors(const VehicleRef& vehicle) const;
    // The blade count and pole pairs are library data (the prop file's blades, the motor's pole pairs); a vehicle's ModelType carries neither, so the host
    // that chose the vehicle gives them once, one entry per rotor, refused naming the field for a count out of range or a wrong length.
    [[nodiscard]] Result<void> set_rotor_info(const VehicleRef& vehicle, std::span<const RotorInfo> info);

    // The host's own state, beside the Simulation's snapshot.
    [[nodiscard]] std::vector<std::byte> save() const;
    [[nodiscard]] Result<void> load(std::span<const std::byte> bytes);

    // An IMU or any sensor sample's time on the sim clock: the sample at tick n is the state at the END of step n.
    [[nodiscard]] int64_t sample_time_ns(uint64_t tick) const noexcept;

private:
    struct Impl;
    std::shared_ptr<Impl> impl_;
};

}  // namespace spade
