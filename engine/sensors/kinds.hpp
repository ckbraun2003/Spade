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
// ⛔ NOT WIRED. NOTHING IN THE ENGINE CALLS THIS, AND SAYING SO IS THE POINT.
// The predicate is written and unit-tested; no production path reads it.
// Simulation::validate_imu_ref() (sim/simulation.cpp) checks the world index,
// the partition and the allocation, and NEVER READS `row.kind`. Read this as a
// report would say it: THIS CHECK IS NOT RUNNING. A predicate with no callers
// cannot produce a false green, but it READS AS A CHECK THAT IS RUNNING, and
// that is a claim -- an exemption nothing checks is not an exemption.
//
// THE CONDITION IT GUARDS IS "ONE ARENA HOLDING TWO KINDS", AND THAT CONDITION
// DOES NOT EXIST. gnss_sensors and imu_sensors are SEPARATE arenas with
// separate ArrayIds and different row layouts, so a row reached through
// imu_id_ can only ever be tagged `imu` or `none`. Nothing short of memory
// corruption can put a conflicting tag there.
//
// ⛔ AND THE PRECONDITION THAT USED TO BE WRITTEN HERE WAS A PROXY. It said:
// "until a GNSS arena exists no row can carry a conflicting tag". The GNSS
// arena shipped at 7627c34d and NO ROW CAN CARRY A CONFLICTING TAG ANYWAY,
// because that sentence named an OBSERVABLE EVENT rather than the PROPERTY it
// stood for. The proxy was satisfiable two ways and the arena was built the way
// that does not satisfy it -- then the original sentence was carried into a leg
// plan as though it had been discharged.
//
//   A PRECONDITION WRITTEN AS A PROXY IS DISCHARGED BY THE PROXY, NOT BY THE
//   CONDITION, AND NOTHING IN THE SENTENCE MARKS THE DIFFERENCE.
//
// The corrected precondition is the one above: ONE ARENA HOLDING TWO KINDS.
// Recorded rather than quietly fixed, because this predicate shipped at
// 5daee4f5 with a contract naming a leg that has since been ruled the wrong
// remedy, and a reversal should be visible where the thing reversed lives.
//
// WHAT THE REAL HAZARD IS, AND WHERE IT IS ACTUALLY CLOSED. Both sensor arenas
// are sized `sensor_capacity` with independently allocated slots, so a slot
// that is valid in one is STRUCTURALLY valid in the other: allocated, partition
// agreeing, bytes belonging to another row type. That is a type confusion
// through a valid ref, and it is real. It is closed AT COMPILE TIME by giving
// GNSS a DISTINCT ref type, so the compiler refuses poll_imu(gnss_ref). A
// compile error beats a runtime error, and it leaves no branch to go
// unreachable.
//
// SO WHAT IS `kind` DOING NOW? Being a liveness flag wearing a tag's name --
// in BOTH rows, since 7627c34d. Adding the second arena DOUBLED the number of
// places that is true and created ZERO sites where the tag discriminates. That
// is honest and it is not urgent; it is written down so the next author does
// not read `kind` as a type discriminator and build on it.
//
// WHAT WOULD MAKE THIS LIVE. A design where ONE arena holds rows of more than
// one kind -- which would need one row type, and GnssSensorRow is not
// ImuSensorRow. If that day comes, this predicate is ready and its unit tests
// in tests/test_gnss.cpp already pin the semantics. Until then it is kept
// rather than deleted because retiring it loses the analysis and the next
// author re-derives it badly.
[[nodiscard]] constexpr bool poll_permits(uint32_t actual, uint32_t wanted) noexcept {
    return actual == none || actual == wanted;
}

}  // namespace sensor_kind

}  // namespace spade::sensors
