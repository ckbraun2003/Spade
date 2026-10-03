#pragma once

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <span>
#include <string>
#include <string_view>
#include <type_traits>
#include <vector>

#include "core/error.hpp"
#include "core/time.hpp"
#include "state/registry.hpp"

// ---------------------------------------------------------------------------
// Snapshots -- the engine's replay/determinism contract (engine design spec §4
// "Snapshot (P8) and readback": "Snapshot = registry walk -> versioned blob").
//
// THE BLOB IS A REGISTRY WALK AND NOTHING ELSE. save() visits every array the
// StateRegistry knows about, in walk order, and writes each one's shape and
// bytes. It has no list of engine arrays of its own, so it cannot fall behind
// one: registration is the only way to obtain arena storage (arenas.hpp), so
// "authoritative state the snapshot missed" is not expressible. State that
// does not live in an arena -- rng stream state, for one -- joins the same
// walk by registering itself, and is carried automatically, with no
// snapshot-side special case.
//
// WHAT IS *NOT* IN THE BLOB. Derived state: the per-world free lists and live
// counts an ArenaSet maintains. Those are a pure function of the slot_to_world
// map, which IS in the walk, and ArenaSet::resync_from_slot_to_world()
// performs the derivation. That is why restore() comes in two flavours below
// and why the ArenaSet one is the one callers want -- a restore that loads
// arena bytes without resyncing produces a set whose NEXT allocation diverges
// from the uninterrupted run, silently.
//
// FORMAT (version 2). Little of it is negotiable; all of it is versioned.
//
//   SnapshotHeader                       40 bytes
//   for each registered array, in walk order:
//       SnapshotSectionHeader            24 bytes
//       name                             name_length bytes, not NUL-terminated
//       payload                          byte_length bytes, the array's bytes
//
// Sections are tightly packed -- no alignment padding between or inside them,
// so a payload can start at any offset. Every read of blob bytes therefore
// goes through memcpy; nothing is reinterpret_cast out of a blob.
//
// BYTE ORDER IS HOST ORDER, deliberately. A snapshot is a same-host replay
// artifact (record a run, resume it, diff the digest), not an interchange
// format: the payloads are raw std430 state images whose float and struct
// layouts are already host/ABI facts, so byte-swapping the scalars in the
// header would buy nothing. `version` exists so that the day a cross-host
// format is actually needed, it can be version 2 rather than a silent
// reinterpretation of version 1 blobs.
//
// TRUST BOUNDARY. Registry descriptors are trusted: they point at memory the
// registrant owns and their extents describe that memory (StateRegistry
// enforces the parts it can). Blobs are NOT trusted: every section is
// bounds-checked against the bytes that actually remain, no length declared
// inside a blob is believed without that check, and no allocation is sized
// from a blob-declared count. A malformed blob is an error, never an overread.
// ---------------------------------------------------------------------------

namespace spade {

// Only referenced by the restore()/save() overloads below; callers that have
// an ArenaSet to pass necessarily include state/arenas.hpp already.
class ArenaSet;

// ---------------------------------------------------------------------------
// FNV-1a, 64-bit. Chosen for being short enough to re-implement from this
// header in a debugger or a Python tool, not for cryptographic strength
// (nothing here defends against a crafted collision; restore() re-checks every
// field the schema hash covers anyway).
//
// THE TREE'S SEEDED FOLD, with three users: schema_hash() below, the
// determinism digest (engine/testing/replay.hpp's state_digest/world_digest)
// and the world-set config identity (sim/world_set.hpp's config_hash). All
// three fold THIS function deliberately -- a second implementation of FNV would
// be a second answer to "did these bytes change". The one near-duplicate in the
// tree is core/rng.hpp's fnv1a64(string_view): same construction, same
// constants, but no seed parameter, so it cannot chain a fold; it exists to
// derive rng domain tags and nothing folds with both.
//
// It has outgrown "lives next to its only caller", then, and the honest home is
// a core/hash.hpp. Left here rather than moved as a drive-by: every consumer
// above already includes this header for other reasons, and relocating a
// function whose output is baked into four committed golden digests deserves
// its own commit rather than a footnote in someone else's.
// ---------------------------------------------------------------------------
inline constexpr uint64_t kFnv1a64Offset = 0xcbf29ce484222325ULL;
inline constexpr uint64_t kFnv1a64Prime = 0x00000100000001b3ULL;

[[nodiscard]] constexpr uint64_t fnv1a64(std::span<const std::byte> bytes,
                                         uint64_t seed = kFnv1a64Offset) noexcept {
    uint64_t hash = seed;
    for (const std::byte byte : bytes) {
        hash ^= static_cast<uint64_t>(std::to_integer<unsigned char>(byte));
        hash *= kFnv1a64Prime;
    }
    return hash;
}

// 'SPSN', spelled so that a little-endian hex dump of a blob's first four
// bytes reads "SPSN" left to right.
inline constexpr uint32_t kSnapshotMagic =
    uint32_t{'S'} | (uint32_t{'P'} << 8) | (uint32_t{'S'} << 16) | (uint32_t{'N'} << 24);

// Bumped whenever the byte format changes in any way a version-1 reader would
// misread -- which includes adding a header field, since the header is fixed
// size. There is deliberately no migration path (YAGNI): a version this build
// does not know is rejected, not converted.
//
// Version 2 (2026-10-02, module-API plan stage 1) added
// SnapshotHeader::configuration_identity. A version-1 blob is refused.
inline constexpr uint32_t kSnapshotVersion = 2;

// Refusal ceiling for a whole blob, in bytes. Its job is not to be a policy
// (1 TiB is far past anything the engine will snapshot) but to make the size
// arithmetic in save() and read_file() provably overflow-free, and to stop a
// corrupt file size from being turned into an allocation request.
inline constexpr uint64_t kMaxSnapshotBytes = uint64_t{1} << 40;

// ---------------------------------------------------------------------------
// The fixed 40-byte blob header. Read and written by memcpy of its object
// representation, so the static_asserts below are the format -- an implicit
// padding byte here would be a silent format change.
//
// `world_count` and `array_count` are the "world-set shape" summary: enough
// for a tool (or a human with a hex editor) to tell two blobs apart without
// parsing the section table. They are NOT the authority on shape -- the
// per-section extents are, and restore() checks those field by field. The
// header's copies are checked too, so a hand-edited header is still rejected.
// ---------------------------------------------------------------------------
struct SnapshotHeader {
    uint32_t magic;        // kSnapshotMagic
    uint32_t version;      // kSnapshotVersion
    uint64_t schema_hash;  // schema_hash() of the registry that produced this
    uint64_t tick;         // Tick::value at which the snapshot was taken
    uint32_t world_count;  // world_set_size() of that registry
    uint32_t array_count;  // number of sections that follow
    // v2: the configuration that produced the state -- a Simulation's compiled
    // module schedule identity (sim/module.hpp). 0 for a bare registry save.
    // The state layer stores it; Simulation::restore() refuses a mismatch (L2).
    uint64_t configuration_identity;
};

static_assert(std::is_standard_layout_v<SnapshotHeader>);
static_assert(std::is_trivially_copyable_v<SnapshotHeader>);
static_assert(sizeof(SnapshotHeader) == 40, "snapshot header is a fixed 40-byte prefix");
static_assert(offsetof(SnapshotHeader, magic) == 0);
static_assert(offsetof(SnapshotHeader, version) == 4);
static_assert(offsetof(SnapshotHeader, schema_hash) == 8);
static_assert(offsetof(SnapshotHeader, tick) == 16);
static_assert(offsetof(SnapshotHeader, world_count) == 24);
static_assert(offsetof(SnapshotHeader, array_count) == 28);
static_assert(offsetof(SnapshotHeader, configuration_identity) == 32);
static_assert(sizeof(SnapshotHeader::magic) + sizeof(SnapshotHeader::version) +
                      sizeof(SnapshotHeader::schema_hash) + sizeof(SnapshotHeader::tick) +
                      sizeof(SnapshotHeader::world_count) + sizeof(SnapshotHeader::array_count) +
                      sizeof(SnapshotHeader::configuration_identity) ==
                  sizeof(SnapshotHeader),
              "SnapshotHeader has implicit padding: the byte image would not match the field list");

// ---------------------------------------------------------------------------
// One section's fixed 24-byte prefix, followed by `name_length` name bytes and
// then `byte_length` payload bytes.
//
// `byte_length` is redundant -- it must equal elem_size * world_count *
// capacity_per_world -- and that redundancy is the point: restore() checks the
// identity, so a blob whose declared length disagrees with its declared shape
// is rejected instead of being read at one of the two lengths.
// ---------------------------------------------------------------------------
struct SnapshotSectionHeader {
    uint32_t name_length;
    uint32_t elem_size;
    uint32_t world_count;
    uint32_t capacity_per_world;
    uint64_t byte_length;
};

static_assert(std::is_standard_layout_v<SnapshotSectionHeader>);
static_assert(std::is_trivially_copyable_v<SnapshotSectionHeader>);
static_assert(sizeof(SnapshotSectionHeader) == 24, "snapshot section header is a fixed 24-byte prefix");
static_assert(offsetof(SnapshotSectionHeader, name_length) == 0);
static_assert(offsetof(SnapshotSectionHeader, elem_size) == 4);
static_assert(offsetof(SnapshotSectionHeader, world_count) == 8);
static_assert(offsetof(SnapshotSectionHeader, capacity_per_world) == 12);
static_assert(offsetof(SnapshotSectionHeader, byte_length) == 16);
static_assert(sizeof(SnapshotSectionHeader::name_length) + sizeof(SnapshotSectionHeader::elem_size) +
                      sizeof(SnapshotSectionHeader::world_count) +
                      sizeof(SnapshotSectionHeader::capacity_per_world) +
                      sizeof(SnapshotSectionHeader::byte_length) ==
                  sizeof(SnapshotSectionHeader),
              "SnapshotSectionHeader has implicit padding: the byte image would not match the field list");

// ---------------------------------------------------------------------------
// A snapshot, as bytes.
//
// The class invariant is exactly one thing: the buffer is at least
// sizeof(SnapshotHeader) bytes long, its magic is ours, and its version is one
// this build understands. That is what makes the accessors below total
// functions rather than Result-returning ones, and it is ALL the invariant
// promises -- the section table is validated by restore(), against the target
// registry, because "is this blob well formed" and "does this blob fit this
// state" are the same question and answering it twice would be answering it
// differently.
//
// Bytes are immutable after construction (no mutating accessor exists), so the
// header the accessors read is always the header from_bytes() validated. A
// caller that wants to tamper -- every negative test does -- copies bytes()
// out, edits, and comes back through from_bytes().
//
// The one state that does not satisfy the invariant is a MOVED-FROM blob (its
// vector is validly empty). Rather than leave that as undefined behaviour
// waiting in an accessor, header() checks and reports a zeroed header there,
// so a moved-from blob is inert -- restore() rejects it, nothing overreads.
// ---------------------------------------------------------------------------
class SnapshotBlob {
public:
    ~SnapshotBlob() = default;
    SnapshotBlob(const SnapshotBlob&) = default;
    SnapshotBlob& operator=(const SnapshotBlob&) = default;
    SnapshotBlob(SnapshotBlob&&) = default;
    SnapshotBlob& operator=(SnapshotBlob&&) = default;

    // There is no default constructor on purpose: an empty buffer is not a
    // snapshot, and every blob in the program has come through one of the
    // three doors below.

    // Adopts raw bytes -- from read_file(), from save(), from a test that just
    // corrupted a copy. Fails with io_error if the buffer is too short to hold
    // a header or the magic is wrong (both mean "these are not our bytes"),
    // and with schema_mismatch if the version is not kSnapshotVersion (they
    // are our bytes, in a format this build does not speak).
    [[nodiscard]] static Result<SnapshotBlob> from_bytes(std::vector<std::byte> bytes);

    // Whole-file binary read, then from_bytes(). io_error on any filesystem
    // failure -- missing, unreadable, a directory, a short read, or a file
    // larger than kMaxSnapshotBytes (a size that big is treated as corruption
    // rather than as an allocation request).
    [[nodiscard]] static Result<SnapshotBlob> read_file(const std::filesystem::path& path);

    // Whole-file binary write, truncating an existing file. io_error on any
    // filesystem failure, including a failure that only surfaces at close.
    [[nodiscard]] Result<void> write_file(const std::filesystem::path& path) const;

    // The validated header, by value (it is 40 bytes and read out of the
    // buffer on each call, so it can never drift from the bytes).
    [[nodiscard]] SnapshotHeader header() const noexcept;

    [[nodiscard]] uint32_t version() const noexcept { return header().version; }
    [[nodiscard]] uint64_t schema_hash() const noexcept { return header().schema_hash; }
    [[nodiscard]] Tick tick() const noexcept { return Tick{header().tick}; }
    [[nodiscard]] uint32_t world_count() const noexcept { return header().world_count; }
    [[nodiscard]] uint32_t array_count() const noexcept { return header().array_count; }
    [[nodiscard]] uint64_t configuration_identity() const noexcept { return header().configuration_identity; }

    [[nodiscard]] std::span<const std::byte> bytes() const noexcept { return bytes_; }
    [[nodiscard]] std::size_t size() const noexcept { return bytes_.size(); }

private:
    explicit SnapshotBlob(std::vector<std::byte> bytes) noexcept : bytes_(std::move(bytes)) {}

    std::vector<std::byte> bytes_;
};

// ---------------------------------------------------------------------------
// The schema identity of a registry: FNV-1a 64 over, in walk order, each
// array's name (length-prefixed, so no two name/field boundaries can be
// confused), element size, world count and per-world capacity -- prefixed by
// the array count.
//
// It covers shape only, never values, and never the walk POSITION of an array
// as a number: two registries with the same arrays in the same order hash the
// same, and any difference in names, element sizes, extents or array count
// changes the hash. Restoring a blob into a registry with a different hash is
// schema_mismatch.
// ---------------------------------------------------------------------------
[[nodiscard]] uint64_t schema_hash(const StateRegistry& registry) noexcept;

// Worlds in the world set, as the walk sees it: the largest world_count over
// the registered arrays (0 for an empty registry). Every array an ArenaSet
// registers carries that set's world_count, so for an ArenaSet this is
// exactly ArenaSet::world_count(); a non-arena registration is free to carry
// fewer worlds (a single-world global, say) without changing the answer.
[[nodiscard]] uint32_t world_set_size(const StateRegistry& registry) noexcept;

// ---------------------------------------------------------------------------
// save -- walk the registry, write the blob. `tick` is recorded verbatim in
// the header; the state layer has no clock of its own to read it from (and
// nothing under engine/ reads a wall clock at all).
//
// Fails with capacity_exceeded if the registry is too large to describe in the
// format (more than 2^32-1 arrays, a name longer than 2^32-1 bytes, an array
// whose world_count * capacity_per_world overflows a uint32, or a total above
// kMaxSnapshotBytes), and with internal if a descriptor violates the registry's
// own invariants (a null pointer for a non-empty array). It cannot fail for
// any reason to do with the VALUES in the arrays -- bytes are opaque here.
// ---------------------------------------------------------------------------
[[nodiscard]] Result<SnapshotBlob> save(const StateRegistry& registry, Tick tick,
                                        uint64_t configuration_identity = 0);

// Convenience for the common caller: an ArenaSet's registry is its whole
// state. Saving needs no arena cooperation at all (nothing derived is
// written), which is exactly the asymmetry with restore() below.
[[nodiscard]] Result<SnapshotBlob> save(const ArenaSet& arenas, Tick tick, uint64_t configuration_identity = 0);

// ---------------------------------------------------------------------------
// restore, byte level -- overwrite each registered array with the blob's
// bytes for it.
//
// TAKES A CONST REGISTRY, and that is not a slip: the registry object is not
// modified, the memory it points at is. RegisteredArray::data is a mutable
// pointer inside a const descriptor for precisely this reason (see
// registry.hpp), and it is also what lets this be called with
// ArenaSet::registry(), which is const.
//
// DOES NOT REBUILD DERIVED STATE. On an ArenaSet's registry this leaves the
// free lists and live counts describing the state the set was in BEFORE the
// call, which is a silent divergence on the next allocation. Use the ArenaSet
// overload unless you are restoring a registry that owns no arenas.
//
// ALL OR NOTHING. Every check -- schema hash, header shape, the whole section
// table, every declared length against the bytes that remain, and every
// section against its registry entry -- happens before the first byte is
// written. On any error, not one byte of the target has changed.
//
// Errors: schema_mismatch when the blob describes a different set of arrays
// than the target registry has (different hash, count, names, element sizes or
// extents), io_error when the blob itself does not parse (truncated, trailing
// bytes, a declared length that does not fit or does not match its shape),
// internal when the target registry violates its own invariants.
// ---------------------------------------------------------------------------
[[nodiscard]] Result<void> restore(const StateRegistry& registry, const SnapshotBlob& blob);

// ---------------------------------------------------------------------------
// restore, whole arena set -- the byte-level restore above, plus the
// derivation that makes it complete: every arena's free lists and live counts
// are rebuilt from its just-restored slot_to_world map. THIS is the one that
// satisfies "resume from a snapshot reproduces the uninterrupted run".
//
// Also all-or-nothing. The extra failure mode the resync could introduce -- a
// blob whose slot_to_world payload names a world that does not own that slot
// -- is checked on the BLOB's bytes before anything is written, so the resync
// itself cannot fail here and no error path can leave a half-restored set.
// That content-level corruption is reported as io_error (the blob is bad);
// the same inconsistency found on a live arena by
// ArenaSet::resync_from_slot_to_world() is invalid_argument (the caller
// corrupted their own state) -- different reporters, different fault.
// ---------------------------------------------------------------------------
[[nodiscard]] Result<void> restore(ArenaSet& arenas, const SnapshotBlob& blob);

// ---------------------------------------------------------------------------
// One section of a blob, located by name, WITHOUT restoring anything.
//
// `payload` is a VIEW INTO the blob's own buffer -- valid for as long as that
// SnapshotBlob is alive and unmoved, never owned. Read it with memcpy like
// everything else that comes out of a blob: sections are tightly packed, so a
// payload can start at any offset and nothing here is legal to
// reinterpret_cast.
// ---------------------------------------------------------------------------
struct BlobSection {
    uint32_t elem_size = 0;
    uint32_t world_count = 0;
    uint32_t capacity_per_world = 0;
    std::span<const std::byte> payload{};
};

// ---------------------------------------------------------------------------
// find_section -- "what does this blob say about the array called `name`?",
// answered before, and independently of, any decision to restore it.
//
// IT EXISTS SO THERE IS EXACTLY ONE BLOB PARSER. Simulation::restore() has to
// read one section's bytes (the `replay_config` row) BEFORE it hands the blob
// to restore(), and the alternative -- a second, sim-side walk over the section
// table -- would duplicate this file's whole trust boundary in a layer that has
// no business owning it. This runs the SAME bounds-checked parse restore() runs
// and hands back one section's extents and bytes.
//
// STILL NAME-AGNOSTIC. The caller supplies the name; nothing in the state layer
// knows what any array MEANS, which is the property the header opens with ("the
// blob is a registry walk and nothing else").
//
// Errors: io_error when the blob does not parse (exactly the errors restore()
// would report for it), not_found when it parses and holds no section by that
// name. Note that finding a section proves nothing about whether the blob FITS
// a given registry -- that is restore()'s question, checked there.
// ---------------------------------------------------------------------------
[[nodiscard]] Result<BlobSection> find_section(const SnapshotBlob& blob, std::string_view name);

}  // namespace spade
