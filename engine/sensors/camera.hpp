#pragma once

// ---------------------------------------------------------------------------
// The camera sensor kind -- its ROW, its STAMP, its noise tag and its scheduling rule (Phase 3.1 of rendering/camera-sensor-design, sections 5, 7, 10, 11).
//
// WHAT IS HERE, AND WHAT IS NOT. A camera is a sensor row like the IMU's and the GNSS's: a mount on a body, a rate (divider and phase), a seeded noise stream
// and a tick-stamped output. The PIXELS are not here and are not Core's: they live in Rendering's output pool, and the stamp below is the fixed-size header
// written beside a pool slot. The row is only what the world has to hold for a camera to be a part of it, so a snapshot and a hash see a changed lens.
//
// DECIDED WITH RENDERING (Phase 3.1, 2026-11-19):
//   * lens_kind is a uint32 KIND (0 pinhole, 1 KB4; above is reserved and refused), never a table index. The intrinsics themselves (fx, fy, cx, cy, k1..k4)
//     are IN THE ROW, so a changed lens is a changed row; k1..k4 are zero for a pinhole.
//   * cull_mask is the camera's TLAS cull mask: eight bits, the bodies' slots it can see (bit 7 is every static, bits 0..6 a body's slot in its env). The
//     default is 0xFF with the carrying body's own bit cleared (self-visibility off). It is NOT a channel selector.
//   * mode is colour or grey. The CHANNEL SET (image, id, corners, depth later) is a separate field that waits on Rendering's Phase 1 list: `channels` is a
//     named field that must stay 0 until then, and a non-zero value is refused.
// ---------------------------------------------------------------------------

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <string>
#include <string_view>
#include <type_traits>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include "core/rng.hpp"
#include "sensors/kinds.hpp"
#include "sensors/rings.hpp"
#include "state/layout.hpp"

namespace spade::sensors {

// The rng domain tag a camera's noise stream is derived under (core/rng.hpp make_stream(world_seed, tag, local_slot)). PINNED: changing it re-seeds every camera
// and invalidates every recorded snapshot. It differs from the IMU's and the GNSS's.
inline constexpr std::string_view kCameraNoiseDomainTag = "sensor.camera";

namespace lens_kind {
inline constexpr uint32_t pinhole = 0u;
inline constexpr uint32_t kb4 = 1u;
inline constexpr uint32_t count = 2u;  // the first value no lens uses
}  // namespace lens_kind

namespace camera_mode {
inline constexpr uint32_t colour = 0u;
inline constexpr uint32_t grey = 1u;
inline constexpr uint32_t count = 2u;
}  // namespace camera_mode

// Flags in CameraStamp::flags (Rendering's wire shape): bit 0 valid, bit 1 the frame was dropped or late, bit 2 a camera-rate frame with the shutter blur on.
namespace stamp_flag {
inline constexpr uint32_t valid = 1u << 0;
inline constexpr uint32_t dropped_or_late = 1u << 1;
inline constexpr uint32_t blurred = 1u << 2;
inline constexpr uint32_t known = valid | dropped_or_late | blurred;  // the rest are reserved zero
}  // namespace stamp_flag

// The camera row. One element of the world-partitioned camera arena (the arena itself is a later commit). std430, 16-byte base alignment, every byte in a
// named field.
struct alignas(kStd430StructAlignment) CameraSensorRow {
    uint32_t body_slot;        // WORLD-LOCAL index into this world's bodies span
    uint32_t kind;             // sensor_kind::; 0 == none == inert (active-high liveness)
    uint32_t rate_divider;     // render one frame every N substeps; >= 1
    uint32_t phase;            // substeps elapsed in the current camera period
    glm::vec3 mount_pos;       // BODY-frame offset from the COM, m
    float shutter_fraction;    // exposure as a fraction of the camera period, in [0, 1]
    glm::quat mount_orient;    // mount -> body rotation, unit quaternion (the IMU's convention)
    uint32_t lens_kind;        // lens_kind::
    uint32_t width;            // pixels
    uint32_t height;           // pixels
    uint32_t mode;             // camera_mode::
    float fx;                  // intrinsics, pixels
    float fy;
    float cx;
    float cy;
    float k1;                  // KB4 distortion; zero for a pinhole
    float k2;
    float k3;
    float k4;
    uint32_t blur_samples;     // motion-blur sub-samples; 1 == off
    uint32_t aa_samples;       // antialiasing samples per pixel; 1 == off
    float exposure_gain;       // linear gain, > 0
    uint32_t cull_mask;        // the TLAS cull mask: bits 0..7 only
    uint32_t channels;         // the channel set: reserved, must stay 0 until Phase 1 names it
    uint32_t _p0;              // std430 pad
    rng::Stream noise;         // this camera's own stream; ALL of its randomness
    SampleIndex last_index;    // the newest frame's generation; 0 == none yet
    uint64_t _reserved0;       // reserved for versioned growth; must stay 0
    uint64_t _reserved1;       // ditto
};

static_assert(std::is_standard_layout_v<CameraSensorRow>, "CameraSensorRow must be standard-layout for offsetof to be meaningful");
static_assert(std::is_trivially_copyable_v<CameraSensorRow>, "CameraSensorRow must be memcpy-able: snapshots copy it byte-wise");
static_assert(std::is_trivially_destructible_v<CameraSensorRow>, "arena slots are never individually destroyed");
static_assert(alignof(CameraSensorRow) == 16, "std430 base alignment");
static_assert(sizeof(CameraSensorRow) == 160, "std430 array stride");

static_assert(offsetof(CameraSensorRow, body_slot) == 0);
static_assert(offsetof(CameraSensorRow, kind) == 4);
static_assert(offsetof(CameraSensorRow, rate_divider) == 8);
static_assert(offsetof(CameraSensorRow, phase) == 12);
static_assert(offsetof(CameraSensorRow, mount_pos) == 16);
static_assert(offsetof(CameraSensorRow, shutter_fraction) == 28);
static_assert(offsetof(CameraSensorRow, mount_orient) == 32);
static_assert(offsetof(CameraSensorRow, lens_kind) == 48);
static_assert(offsetof(CameraSensorRow, width) == 52);
static_assert(offsetof(CameraSensorRow, height) == 56);
static_assert(offsetof(CameraSensorRow, mode) == 60);
static_assert(offsetof(CameraSensorRow, fx) == 64);
static_assert(offsetof(CameraSensorRow, fy) == 68);
static_assert(offsetof(CameraSensorRow, cx) == 72);
static_assert(offsetof(CameraSensorRow, cy) == 76);
static_assert(offsetof(CameraSensorRow, k1) == 80);
static_assert(offsetof(CameraSensorRow, k2) == 84);
static_assert(offsetof(CameraSensorRow, k3) == 88);
static_assert(offsetof(CameraSensorRow, k4) == 92);
static_assert(offsetof(CameraSensorRow, blur_samples) == 96);
static_assert(offsetof(CameraSensorRow, aa_samples) == 100);
static_assert(offsetof(CameraSensorRow, exposure_gain) == 104);
static_assert(offsetof(CameraSensorRow, cull_mask) == 108);
static_assert(offsetof(CameraSensorRow, channels) == 112);
static_assert(offsetof(CameraSensorRow, _p0) == 116);
static_assert(offsetof(CameraSensorRow, noise) == 120);
static_assert(offsetof(CameraSensorRow, last_index) == 136);
static_assert(offsetof(CameraSensorRow, _reserved0) == 144);
static_assert(offsetof(CameraSensorRow, _reserved1) == 152);
static_assert(offsetof(CameraSensorRow, mount_pos) % 16 == 0);
static_assert(offsetof(CameraSensorRow, mount_orient) % 16 == 0);
static_assert(offsetof(CameraSensorRow, noise) % 8 == 0);
static_assert(offsetof(CameraSensorRow, last_index) % 8 == 0);

// The stamp: the fixed-size header Spade writes beside a pool slot (Rendering's wire shape, design section 7). std430, 96 bytes.
struct alignas(kStd430StructAlignment) CameraStamp {
    uint64_t tick;             // the physics tick the frame is rendered from
    uint32_t camera_index;
    uint32_t generation;       // per-camera frame counter, +1 each frame; a gap is a drop
    uint32_t dropped;          // frames skipped since the last one
    uint32_t lens_kind;
    uint32_t pool_slot;        // the output slot the pixels are in
    uint32_t flags;            // stamp_flag::
    glm::vec3 position;        // the camera's pose at capture, world frame
    float fx;                  // the intrinsics actually used
    float fy;
    float cx;
    float cy;
    float k1;
    float k2;
    float k3;
    float k4;
    float _p0;                 // std430 pad
    glm::quat orientation;     // camera -> world at capture
};

static_assert(std::is_standard_layout_v<CameraStamp>);
static_assert(std::is_trivially_copyable_v<CameraStamp>);
static_assert(alignof(CameraStamp) == 16);
static_assert(sizeof(CameraStamp) == 96);
static_assert(offsetof(CameraStamp, tick) == 0);
static_assert(offsetof(CameraStamp, camera_index) == 8);
static_assert(offsetof(CameraStamp, generation) == 12);
static_assert(offsetof(CameraStamp, dropped) == 16);
static_assert(offsetof(CameraStamp, lens_kind) == 20);
static_assert(offsetof(CameraStamp, pool_slot) == 24);
static_assert(offsetof(CameraStamp, flags) == 28);
static_assert(offsetof(CameraStamp, position) == 32);
static_assert(offsetof(CameraStamp, fx) == 44);
static_assert(offsetof(CameraStamp, fy) == 48);
static_assert(offsetof(CameraStamp, cx) == 52);
static_assert(offsetof(CameraStamp, cy) == 56);
static_assert(offsetof(CameraStamp, k1) == 60);
static_assert(offsetof(CameraStamp, k2) == 64);
static_assert(offsetof(CameraStamp, k3) == 68);
static_assert(offsetof(CameraStamp, k4) == 72);
static_assert(offsetof(CameraStamp, _p0) == 76);
static_assert(offsetof(CameraStamp, orientation) == 80);

// The camera's own noise stream, derived by its WORLD-LOCAL slot (not its global arena slot: batching invariance, as the IMU's and the GNSS's).
[[nodiscard]] constexpr rng::Stream camera_noise_stream(uint64_t world_seed, uint32_t local_slot) noexcept {
    return rng::make_stream(world_seed, kCameraNoiseDomainTag, local_slot);
}

// One substep of the camera's divider and phase: advances `row.phase` and returns whether a frame is due this substep. A divider of 1 (or 0, read as 1)
// is every substep; the phase wraps to 0 on the due substep. The capture tick is the substep's tick (the caller stamps it).
[[nodiscard]] bool camera_advance(CameraSensorRow& row) noexcept;

// Why a row is not a valid camera, or the empty string: the first rule it breaks, naming the field. A zero row (kind none) is valid: it is inert.
[[nodiscard]] std::string camera_row_issue(const CameraSensorRow& row);

}  // namespace spade::sensors
