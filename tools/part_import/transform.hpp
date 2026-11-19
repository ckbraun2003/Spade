#pragma once

// ---------------------------------------------------------------------------
// part_import/transform.hpp -- the part frame and the unit guess (design note, sections 2 and 6).
//
// --origin X,Y,Z is the point of the file, in the file's own unit, that becomes the part's origin: the part frame is p' = scale * axes(p - origin).
// --axes is the part file's `to_part.axes`: an exact permutation of the source axes with signs, a PROPER rotation (determinant +1; a
// mirror reverses the winding and is refused). --scale multiplies every position once, after the unit, in double. Axes then scale,
// applied once to the metre positions, then make_mesh() again, so what is stored passed the gate in its final frame.
// ---------------------------------------------------------------------------

#include <array>
#include <memory>
#include <optional>
#include <string>
#include <string_view>

#include <glm/glm.hpp>

#include "report.hpp"
#include "world/mesh.hpp"

namespace spade::partimport {

// Part axis i is sign[i] times the source axis source[i] (0 x, 1 y, 2 z).
struct Axes {
    std::array<int, 3> source{0, 1, 2};
    std::array<int, 3> sign{1, 1, 1};
    [[nodiscard]] bool identity() const { return source == std::array<int, 3>{0, 1, 2} && sign == std::array<int, 3>{1, 1, 1}; }
    [[nodiscard]] std::string text() const;  // "x,y,z", "-y,x,z": the form parse_axes reads
    bool operator==(const Axes&) const = default;
};

// Three comma-separated items, each x, y or z (either case) with an optional leading '-', the three axes each used once; spaces
// around an item are ignored. Codes: axes_invalid, axes_mirror (a determinant of -1).
[[nodiscard]] Expected<Axes> parse_axes(std::string_view text);
// Three comma-separated finite numbers, the point of the file (in the file's own unit) that becomes the part's origin. Code: origin_invalid.
[[nodiscard]] Expected<glm::dvec3> parse_origin(std::string_view text);
// A finite number above zero. Code: scale_invalid.
[[nodiscard]] Expected<double> parse_scale(std::string_view text);

[[nodiscard]] glm::dvec3 apply_axes(const Axes& axes, const glm::dvec3& p);

// The mesh in the part frame: its positions through the axes and the scale, rebuilt by make_mesh() (unit metre). The same mesh
// (pointer) when the axes are the identity and the scale is 1. A refusal is the gate's kind (refusal_name()), with make_mesh()'s message.
[[nodiscard]] Expected<std::shared_ptr<const TriangleMesh>> transform_mesh(const std::shared_ptr<const TriangleMesh>& mesh,
                                                                          const Axes& axes, double scale,
                                                                          const glm::dvec3& origin_m = glm::dvec3(0.0));

// A unit for a file that carries none (STL, OBJ), from its bounding box's sides in the file's own numbers: millimetres when the
// longest side is 5 to 5000, metres when it is 0.005 to 5 (the lower bound inclusive, the upper exclusive), nothing outside both. A
// guess, reported with its basis and never applied.
struct UnitGuess {
    std::string unit;   // "mm" or "m"
    std::string basis;  // a sentence
};
[[nodiscard]] std::optional<UnitGuess> guess_unit(const glm::dvec3& box_sides);

}  // namespace spade::partimport
