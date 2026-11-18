// ---------------------------------------------------------------------------
// scene/library_place.hpp -- compose_placement (spade::scene; L88l, the library contract's section 4, "compose_placement").
//
// A pure function: for each item, the world pose of a part,
//
//     pose = parent . install_point . adjust . part_to_mount
//
// a product of poses in that order, in double, with the quaternions normalised and the result's `w >= 0` (a w within 1e-15 of 0 is taken as 0 and
// the first axis of x, y, z beyond 1e-15 is made positive, so a rotation has one spelling). It reads no library and no file.
//
//   parent          the part that carries the install point, in the drone's design frame; absent: the identity (the frame part)
//   install_point   the point's pose in the parent part's frame
//   adjust          the slot's adjustment: a position offset (metres) and three rotations in degrees, right-handed about the install
//                   point's axes (+X forward, +Y left, +Z up), composed as Rz(yaw) . Ry(pitch) . Rx(roll): intrinsic, so a positive
//                   pitch tips +X toward -Z. It acts in the install point's own frame, after the install point's pose. Absent: none
//   adjust_range    the frame part's allowed range for the adjustment, per axis; absent: unchecked. An absent adjust is a zero
//                   adjustment and IS checked against a range that is present (a range that excludes 0 refuses it)
//   part_to_mount   the placed part's own offset; absent: the identity
//
// A bad item does not hide the others: its result has no pose and its own refusal (`adjust_out_of_range`: `field` names the
// first axis outside its range in the order pos[0..2], yaw_deg, pitch_deg, roll_deg; `invalid_request`: a number that is not
// finite, a quaternion that is not a unit one to 2e-6 in its squared length, an adjust_range whose min exceeds its max). The
// results are in request order, each with its item's key. Deterministic: + - * / and sqrt only, the angles through
// sincos_deg_arith (exact at multiples of 90 degrees), so MSVC and gcc give the same bits.
// ---------------------------------------------------------------------------
#pragma once

#include <optional>
#include <string>
#include <vector>

#include "scene/library_set.hpp"
#include "scene/part_blocks.hpp"

namespace spade::scene {

struct PlacementItem {
    std::string key{};
    std::optional<SetPose> parent{};
    SetPose install_point{};
    std::optional<SetAdjust> adjust{};
    std::optional<AdjustRange> adjust_range{};
    std::optional<SetPose> part_to_mount{};
};

struct PlacedItem {
    std::string key{};
    std::optional<SetPose> pose{};              // absent when the item is refused
    std::optional<LibraryRefusal> refusal{};    // code and field set; the other members empty
};

[[nodiscard]] std::vector<PlacedItem> compose_placement(const std::vector<PlacementItem>& items);

}  // namespace spade::scene
