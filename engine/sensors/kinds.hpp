#pragma once

#include <cstdint>

// ---------------------------------------------------------------------------
// sensors/kinds.hpp -- the sensor kind vocabulary, shared by every sensor row
// type and by the poll surface that discriminates between them.
//
// WHY THIS FILE EXISTS, AND IT IS NOT ORGANISATIONAL. Until 2026-09-21 this
// namespace lived inside sensors/imu.hpp, which made the tag that distinguishes
// sensor KINDS a member of one kind's header. A second kind could not name its
// own tag without including the first kind's row, its noise model and its
// std430 layout. THE VOCABULARY THAT SEPARATES TWO THINGS CANNOT LIVE INSIDE
// ONE OF THEM.
//
// That is the concrete half of a finding recorded in
// design-specs/spade/pivot-audit.md (EX-3, EX-4): the engine has a general
// component vocabulary in objects/ -- SensorComponent, ForceElementComponent --
// built ON TOP of sim/'s arenas and holding only handles into them, so it
// cannot generalise what it sits on. The arenas are the subject. This is the
// arenas' side of it.
//
// ---------------------------------------------------------------------------
// ADDING A KIND -- read this before adding a line below.
//
// A value here is a CLAIM THAT A ROW TYPE EXISTS. Do not add one ahead of its
// row: sensors/ already carries a live `CameraComponent` (objects/component.hpp,
// id 7) whose sample type has never been written, and a declaration with
// nothing behind it is worse than an absence because it reads as done. Add the
// row, the sample, the noise stream and the poll path, then add the tag.
//
// Each kind owns, and must not share:
//   * its ROW type      (std430, layout-pinned, every byte in a named field)
//   * its SAMPLE type   (ditto -- it is what a ring stores)
//   * its rng DOMAIN TAG (core/rng.hpp make_stream; PINNED -- changing one
//     re-seeds every sensor of that kind and invalidates every recorded
//     snapshot and every committed determinism digest)
//
// And the value itself is PINNED once a snapshot exists that contains it: the
// number is in the bytes of every row of that kind, in every blob and, from S6,
// in every Slang buffer. Renumbering is a snapshot-format break.
// ---------------------------------------------------------------------------

namespace spade::sensors {

// ---------------------------------------------------------------------------
// Sensor row `kind` values. A plain uint32_t rather than an enum class, for the
// same reason DragBodyRow::mode is one: the field's BYTES are what a snapshot
// blob and (from S6) a Slang buffer see.
//
// `none` == 0 is ALSO THE LIVENESS PREDICATE, active-high exactly like
// BodyState::flags and DragBodyRow::enabled: a zero-filled slot -- freed, or
// reserved but not yet initialized -- has kind == none and is skipped without
// any liveness lookup. That is why there is no separate `enabled` field.
//
// ⚠ `none` IS NOT AN ERROR ON THE POLL PATH. A row whose init is still queued
// has kind == none and last_index == 0, and polling a sensor you just added,
// before the next step, is a legal thing to do -- it reports "nothing yet".
// A validator that rejects `none` breaks that documented case. What a validator
// must reject is a kind that is neither `none` nor the one it was asked for,
// because that is a TYPE CONFUSION REACHED THROUGH A VALID REF: the slot is
// allocated, the partition agrees, and the bytes belong to a different row
// type.
// ---------------------------------------------------------------------------
namespace sensor_kind {

inline constexpr uint32_t none = 0u;
inline constexpr uint32_t imu = 1u;
inline constexpr uint32_t gnss = 2u;

// The first value no kind uses. Not a kind: a bound, for a switch's default and
// for the "is this tag one we know" predicate. It moves when a kind is added,
// which is deliberate -- a count that absorbs an addition silently is a count
// nobody has to classify.
inline constexpr uint32_t count = 3u;

// Is `tag` a value this build knows about? A tag outside the vocabulary is a
// row from a newer build, a corrupted slot, or a hand-built ref -- never
// something to interpret.
[[nodiscard]] constexpr bool is_known(uint32_t tag) noexcept { return tag < count; }

// Is `tag` a live sensor of some kind? `none` is a reserved-but-uninitialised
// or freed slot, which is legal and inert rather than wrong.
[[nodiscard]] constexpr bool is_live(uint32_t tag) noexcept {
    return tag != none && is_known(tag);
}

// May a poll for `wanted` read a row tagged `actual`?
//
// TRUE for an exact match and for `none` (queued init, see above). FALSE for
// any other known kind and for any unknown tag.
//
// ⛔ AS OF THIS COMMIT NOTHING CALLS THIS YET, AND SAYING SO IS THE POINT. The
// predicate is written and unit-tested; the wiring is leg 2, where it goes into
// Simulation::validate_imu_ref() (sim/simulation.cpp), which today checks the
// world index, the partition and the allocation and NEVER READS `row.kind`.
// Until a GNSS arena exists no row can carry a conflicting tag, so wiring it
// now would add a production branch that no test could reach -- and an
// unreachable guard is the blind kind this tree has been bitten by before.
//
// That is also why this comment states the gap instead of describing the
// finished state: THIS FILE'S OWN CONTRACT, TWENTY LINES UP, WARNS THAT A
// DECLARATION WITH NOTHING BEHIND IT READS AS DONE. It would be a poor file
// that broke its own rule in its own prose.
//
// WHAT IT IS FOR, once wired: before a second kind existed, `kind` was written
// on every row and read by nothing on the poll path, so it was a liveness flag
// wearing a tag's name. A poll that reaches a row of the wrong kind is a TYPE
// CONFUSION THROUGH A VALID REF -- the slot is allocated, the partition agrees,
// and the bytes belong to another row type.
[[nodiscard]] constexpr bool poll_permits(uint32_t actual, uint32_t wanted) noexcept {
    return actual == none || actual == wanted;
}

}  // namespace sensor_kind

}  // namespace spade::sensors
