#include "state/snapshot.hpp"

#include <algorithm>
#include <cstring>
#include <fstream>
#include <limits>
#include <new>
#include <string_view>
#include <system_error>
#include <utility>

#include "core/path_text.hpp"
#include "state/arenas.hpp"

namespace spade {

static_assert(sizeof(std::size_t) >= 8,
              "the blob size arithmetic below assumes a 64-bit target (declared lengths are uint64)");

namespace {

// ---------------------------------------------------------------------------
// Error spelling. Every message is prefixed so a log line says which layer
// produced it, and every one names the array or offset at fault -- a snapshot
// failure that says only "corrupt" is a failure you debug with a hex editor.
// ---------------------------------------------------------------------------

Error io_err(std::string what) { return Error{Code::io_error, "snapshot: " + std::move(what)}; }
Error schema_err(std::string what) { return Error{Code::schema_mismatch, "snapshot: " + std::move(what)}; }
Error capacity_err(std::string what) { return Error{Code::capacity_exceeded, "snapshot: " + std::move(what)}; }
Error internal_err(std::string what) { return Error{Code::internal, "snapshot: " + std::move(what)}; }


// ---------------------------------------------------------------------------
// Byte-level helpers. Everything that touches blob bytes goes through memcpy:
// sections are tightly packed, so a payload -- or a section header -- can
// start at any alignment, and reinterpret_cast'ing a misaligned uint64 out of
// the buffer would be undefined behaviour on a good day and a fault on a bad
// one.
// ---------------------------------------------------------------------------

template <class T>
void append_pod(std::vector<std::byte>& out, const T& value) {
    static_assert(std::is_trivially_copyable_v<T>);
    std::byte raw[sizeof(T)];
    std::memcpy(raw, &value, sizeof(T));
    out.insert(out.end(), raw, raw + sizeof(T));
}

template <class T>
T read_pod(const std::byte* from) noexcept {
    static_assert(std::is_trivially_copyable_v<T>);
    T value{};
    std::memcpy(&value, from, sizeof(T));
    return value;
}

uint64_t mix_u32(uint64_t hash, uint32_t value) noexcept {
    std::byte raw[sizeof(value)];
    std::memcpy(raw, &value, sizeof(value));
    return fnv1a64(std::span<const std::byte>(raw, sizeof(raw)), hash);
}

uint64_t mix_chars(uint64_t hash, std::string_view text) noexcept {
    return fnv1a64(std::as_bytes(std::span<const char>(text.data(), text.size())), hash);
}

// The declared payload length a section of this shape must have. Kept in one
// place so save() and restore() cannot disagree about it by a byte.
//
// Both multiplications are done in uint64 from uint32 inputs, so neither can
// overflow -- but the ELEMENT COUNT is also required to fit in a uint32,
// because that is the type RegisteredArray::element_count() returns and slot
// indices are uint32 engine-wide. save() refuses to write a shape that does
// not fit; restore() refuses to read one.
uint64_t payload_bytes_for(uint32_t elem_size, uint32_t world_count, uint32_t capacity_per_world) noexcept {
    return uint64_t{elem_size} * (uint64_t{world_count} * capacity_per_world);
}

bool element_count_fits_u32(uint32_t world_count, uint32_t capacity_per_world) noexcept {
    return uint64_t{world_count} * capacity_per_world <= std::numeric_limits<uint32_t>::max();
}

// ---------------------------------------------------------------------------
// A parsed section: a name and a payload that have both been proven to lie
// inside the blob. Views into the blob's buffer, valid for as long as the
// SnapshotBlob they were parsed from -- restore() consumes them within one
// call and never hands them out.
// ---------------------------------------------------------------------------
struct Section {
    std::string_view name;
    uint32_t elem_size = 0;
    uint32_t world_count = 0;
    uint32_t capacity_per_world = 0;
    const std::byte* payload = nullptr;
    std::size_t payload_bytes = 0;
};

// Bounds-checked parse of the whole section table. Believes nothing the blob
// declares: every length is checked against the bytes that actually remain
// BEFORE it is used, and the section count is checked against the smallest
// possible section before it is used to size a vector.
Result<std::vector<Section>> parse_sections(const SnapshotBlob& blob) {
    const std::span<const std::byte> bytes = blob.bytes();
    const std::size_t end = bytes.size();
    // Guaranteed by SnapshotBlob's invariant; restated because everything
    // below subtracts from `end`.
    if (end < sizeof(SnapshotHeader)) {
        return std::unexpected(io_err("blob is shorter than its header"));
    }
    std::size_t offset = sizeof(SnapshotHeader);

    const uint32_t declared = blob.array_count();
    // Never size an allocation from a blob-declared count. Each section costs
    // at least a section header, so this bound is exact and cheap; without it
    // a four-byte edit turns a 2 KiB file into a 100 GiB reserve().
    const std::size_t max_sections = (end - offset) / sizeof(SnapshotSectionHeader);
    if (declared > max_sections) {
        return std::unexpected(io_err("header declares " + std::to_string(declared) + " arrays but the blob has room for " +
                                      std::to_string(max_sections)));
    }

    std::vector<Section> sections;
    // Capped: `declared` is already bounded by the blob's own length, but a
    // large blob would still let a corrupt count amplify into a reservation
    // several times its size. A modest floor gets the same benefit for every
    // real registry and lets growth handle the rest.
    sections.reserve(std::min<std::size_t>(declared, 1024));

    for (uint32_t i = 0; i < declared; ++i) {
        const std::string at = " (section " + std::to_string(i) + ")";

        if (end - offset < sizeof(SnapshotSectionHeader)) {
            return std::unexpected(io_err("truncated section header" + at));
        }
        const auto head = read_pod<SnapshotSectionHeader>(bytes.data() + offset);
        offset += sizeof(SnapshotSectionHeader);

        if (end - offset < head.name_length) {
            return std::unexpected(io_err("truncated section name" + at));
        }
        Section section;
        section.name = std::string_view(reinterpret_cast<const char*>(bytes.data() + offset), head.name_length);
        offset += head.name_length;

        // A zero element size cannot come from a registry (StateRegistry
        // rejects it), so a blob claiming one is corrupt rather than merely
        // mismatched.
        if (head.elem_size == 0) {
            return std::unexpected(io_err("section declares a zero element size" + at));
        }
        if (!element_count_fits_u32(head.world_count, head.capacity_per_world)) {
            return std::unexpected(io_err("section element count overflows a uint32" + at));
        }
        if (head.byte_length != payload_bytes_for(head.elem_size, head.world_count, head.capacity_per_world)) {
            return std::unexpected(io_err("section declares a length that disagrees with its shape" + at));
        }
        if (static_cast<uint64_t>(end - offset) < head.byte_length) {
            return std::unexpected(io_err("truncated section payload" + at));
        }

        section.elem_size = head.elem_size;
        section.world_count = head.world_count;
        section.capacity_per_world = head.capacity_per_world;
        section.payload = bytes.data() + offset;
        section.payload_bytes = static_cast<std::size_t>(head.byte_length);
        offset += section.payload_bytes;

        sections.push_back(section);
    }

    // Strict: a blob we wrote ends exactly here. Trailing bytes mean the
    // buffer is not (only) a snapshot, which is a corruption we would rather
    // report than ignore.
    if (offset != end) {
        return std::unexpected(io_err("blob has " + std::to_string(end - offset) + " trailing bytes after the last section"));
    }
    return sections;
}

// Field-by-field check that the parsed sections describe exactly the target
// registry's arrays, in walk order. The schema hash has already agreed by the
// time this runs -- this is the check that makes the hash an optimisation
// rather than a load-bearing assumption. A hash is never trusted for
// correctness.
Result<void> match_registry(const StateRegistry& registry, const std::vector<Section>& sections) {
    if (registry.size() != sections.size()) {
        return std::unexpected(schema_err("blob has " + std::to_string(sections.size()) + " arrays, registry has " +
                                          std::to_string(registry.size())));
    }

    std::size_t index = 0;
    std::string failure;
    registry.for_each_array([&](const RegisteredArray& array) {
        if (!failure.empty()) return;  // for_each_array cannot break; latch the first failure
        const Section& section = sections[index++];
        if (array.name != section.name) {
            failure = "array " + std::to_string(index - 1) + ": blob names '" + std::string(section.name) +
                      "', registry names '" + array.name + "'";
        } else if (array.elem_size != section.elem_size) {
            failure = "array '" + array.name + "': element size " + std::to_string(section.elem_size) + " in the blob, " +
                      std::to_string(array.elem_size) + " in the registry";
        } else if (array.world_count != section.world_count || array.capacity_per_world != section.capacity_per_world) {
            failure = "array '" + array.name + "': shape " + std::to_string(section.world_count) + "x" +
                      std::to_string(section.capacity_per_world) + " in the blob, " + std::to_string(array.world_count) +
                      "x" + std::to_string(array.capacity_per_world) + " in the registry";
        }
    });
    if (!failure.empty()) {
        return std::unexpected(schema_err(std::move(failure)));
    }

    // The registry's own invariant (a non-empty array has storage). Checked
    // here rather than trusted, because the alternative is a memcpy to
    // nullptr in apply_sections(), which must be unable to fail.
    index = 0;
    std::string broken;
    registry.for_each_array([&](const RegisteredArray& array) {
        if (!broken.empty()) return;
        const Section& section = sections[index++];
        if (array.data == nullptr && section.payload_bytes != 0) {
            broken = "array '" + array.name + "' is registered with no storage but is not empty";
        }
        if (array.byte_size() != section.payload_bytes) {
            broken = "array '" + array.name + "': byte size disagrees with its own extents";
        }
    });
    if (!broken.empty()) {
        return std::unexpected(internal_err(std::move(broken)));
    }
    return {};
}

// Everything that must hold before a single byte is written: the blob speaks
// this version, describes this schema, and parses. Returns the matched
// sections so the caller can apply_sections() them without re-parsing.
Result<std::vector<Section>> validate_against(const StateRegistry& registry, const SnapshotBlob& blob) {
    // SnapshotBlob::from_bytes() already enforced this; re-checked so restore()
    // is correct on its own terms rather than by reference to how the blob was
    // built.
    if (blob.version() != kSnapshotVersion) {
        return std::unexpected(schema_err("blob is format version " + std::to_string(blob.version()) + ", this build reads " +
                                          std::to_string(kSnapshotVersion)));
    }
    if (blob.schema_hash() != schema_hash(registry)) {
        return std::unexpected(schema_err("schema hash mismatch: blob describes a different set of arrays"));
    }
    if (blob.world_count() != world_set_size(registry)) {
        return std::unexpected(schema_err("world-set shape mismatch: blob has " + std::to_string(blob.world_count()) +
                                          " worlds, registry has " + std::to_string(world_set_size(registry))));
    }

    Result<std::vector<Section>> sections = parse_sections(blob);
    if (!sections) return sections;
    if (Result<void> matched = match_registry(registry, *sections); !matched) {
        return std::unexpected(matched.error());
    }
    return sections;
}

// The write pass. Reached only after validate_against() (and, for an arena
// set, the slot-map check) has passed, which is what lets it be infallible:
// every destination is non-null-or-empty and every length has been proven to
// be both the registry's and the blob's.
//
// NAMED apply_sections(), NOT apply() -- a plain `apply` in this anonymous
// namespace is an ADL trap: at the call sites below, unqualified lookup on
// an argument list of (const StateRegistry&, const std::vector<Section>&)
// also considers std::apply (std::vector's associated namespace is std),
// and on libstdc++/gcc overload resolution can bind to THAT candidate --
// gcc-13 hard-errors instantiating std::tuple_size_v<std::vector<Section>>
// inside std::apply's own signature before it ever gets to comparing
// argument types, so the failure looks like a std::apply misuse from the
// call site, not a same-name local function. MSVC's stdlib does not trigger
// the same trap, which is why this compiled clean here before spade-linux's
// first-ever gcc run caught it. A name std cannot possibly own removes the
// trap at its root instead of parenthesizing every call site to suppress
// ADL ((apply)(...)) and leaving the same footgun for the next helper
// someone names apply/visit/swap/size/data/begin/end in this file.
void apply_sections(const StateRegistry& registry, const std::vector<Section>& sections) noexcept {
    std::size_t index = 0;
    registry.for_each_array([&](const RegisteredArray& array) {
        const Section& section = sections[index++];
        if (section.payload_bytes != 0) {
            std::memcpy(array.data, section.payload, section.payload_bytes);
        }
    });
}

// The one content-level check the arena overload needs before writing: a
// slot_to_world payload must name, for each slot, either "free" or the world
// whose partition contains that slot. It is the same predicate
// ArenaSet::resync_from_slot_to_world() applies to a live arena -- run here,
// on the blob, so that the resync after apply_sections() cannot fail and the
// restore stays all-or-nothing.
Result<void> check_slot_map(const Section& section) {
    const uint32_t count = section.world_count * section.capacity_per_world;  // proven to fit a uint32 by parse
    // capacity_per_world == 0 implies count == 0, so the division below is
    // never reached with a zero divisor.
    for (uint32_t slot = 0; slot < count; ++slot) {
        const auto world = read_pod<uint32_t>(section.payload + std::size_t{slot} * sizeof(uint32_t));
        if (world == kInvalidWorld) continue;
        if (world >= section.world_count || slot / section.capacity_per_world != world) {
            return std::unexpected(io_err("array '" + std::string(section.name) + "': slot " + std::to_string(slot) +
                                          " is mapped to world " + std::to_string(world) + ", which does not own it"));
        }
    }
    return {};
}

}  // namespace

// ---------------------------------------------------------------------------
// SnapshotBlob
// ---------------------------------------------------------------------------

Result<SnapshotBlob> SnapshotBlob::from_bytes(std::vector<std::byte> bytes) {
    if (bytes.size() < sizeof(SnapshotHeader)) {
        return std::unexpected(io_err("buffer of " + std::to_string(bytes.size()) + " bytes is too short to hold a " +
                                      std::to_string(sizeof(SnapshotHeader)) + "-byte header"));
    }
    const auto head = read_pod<SnapshotHeader>(bytes.data());
    if (head.magic != kSnapshotMagic) {
        return std::unexpected(io_err("bad magic: not a snapshot blob"));
    }
    if (head.version != kSnapshotVersion) {
        return std::unexpected(schema_err("blob is format version " + std::to_string(head.version) +
                                          ", this build reads " + std::to_string(kSnapshotVersion)));
    }
    return SnapshotBlob(std::move(bytes));
}

SnapshotHeader SnapshotBlob::header() const noexcept {
    // from_bytes() established that bytes_ is at least a header long, and no
    // public operation can shorten it -- with exactly one exception: a
    // MOVED-FROM blob, whose vector is validly empty. Reading 40 bytes out of
    // that would be an overread reachable through the public API, so the size
    // is re-checked here and a moved-from blob reports a zeroed header. Every
    // consumer then fails cleanly (magic 0 is not ours, version 0 is not one
    // we read) instead of running off the end.
    if (bytes_.size() < sizeof(SnapshotHeader)) return SnapshotHeader{};
    return read_pod<SnapshotHeader>(bytes_.data());
}

Result<void> SnapshotBlob::write_file(const std::filesystem::path& path) const {
    std::ofstream out(path, std::ios::binary | std::ios::trunc);
    if (!out) {
        return std::unexpected(io_err("cannot open '" + path_text(path) + "' for writing"));
    }
    if (!bytes_.empty()) {
        out.write(reinterpret_cast<const char*>(bytes_.data()), static_cast<std::streamsize>(bytes_.size()));
    }
    // Close explicitly and re-check: a buffered write failure (a full disk,
    // most of the time) surfaces at flush, not at write().
    out.close();
    if (!out) {
        return std::unexpected(io_err("failed writing '" + path_text(path) + "'"));
    }
    return {};
}

Result<SnapshotBlob> SnapshotBlob::read_file(const std::filesystem::path& path) {
    std::error_code ec;
    const std::uintmax_t size = std::filesystem::file_size(path, ec);
    if (ec) {
        return std::unexpected(io_err("cannot size '" + path_text(path) + "': " + ec.message()));
    }
    if (static_cast<uint64_t>(size) > kMaxSnapshotBytes) {
        return std::unexpected(io_err("'" + path_text(path) + "' is " + std::to_string(size) +
                                      " bytes, past the snapshot ceiling"));
    }

    std::ifstream in(path, std::ios::binary);
    if (!in) {
        return std::unexpected(io_err("cannot open '" + path_text(path) + "' for reading"));
    }

    std::vector<std::byte> bytes;
    try {
        bytes.resize(static_cast<std::size_t>(size));
    } catch (const std::bad_alloc&) {
        return std::unexpected(io_err("cannot allocate " + std::to_string(size) + " bytes for '" + path_text(path) + "'"));
    }
    if (size != 0) {
        in.read(reinterpret_cast<char*>(bytes.data()), static_cast<std::streamsize>(size));
        if (!in || static_cast<std::uintmax_t>(in.gcount()) != size) {
            return std::unexpected(io_err("short read on '" + path_text(path) + "'"));
        }
    }
    return from_bytes(std::move(bytes));
}

// ---------------------------------------------------------------------------
// Schema identity
// ---------------------------------------------------------------------------

uint64_t schema_hash(const StateRegistry& registry) noexcept {
    uint64_t hash = kFnv1a64Offset;
    hash = mix_u32(hash, static_cast<uint32_t>(registry.size()));
    registry.for_each_array([&hash](const RegisteredArray& array) {
        // Length-prefixed name: without it, ("ab", "c") and ("a", "bc") could
        // in principle hash alike.
        hash = mix_u32(hash, static_cast<uint32_t>(array.name.size()));
        hash = mix_chars(hash, array.name);
        hash = mix_u32(hash, array.elem_size);
        hash = mix_u32(hash, array.world_count);
        hash = mix_u32(hash, array.capacity_per_world);
    });
    return hash;
}

uint32_t world_set_size(const StateRegistry& registry) noexcept {
    uint32_t worlds = 0;
    registry.for_each_array([&worlds](const RegisteredArray& array) { worlds = std::max(worlds, array.world_count); });
    return worlds;
}

// ---------------------------------------------------------------------------
// save
// ---------------------------------------------------------------------------

Result<SnapshotBlob> save(const StateRegistry& registry, Tick tick, uint64_t configuration_identity,
                          uint64_t model_registry_identity) {
    if (registry.size() > std::numeric_limits<uint32_t>::max()) {
        return std::unexpected(capacity_err("registry has more arrays than the format can describe"));
    }

    // Pass 1: total size, with every per-array limit checked. Nothing is
    // written until the whole size is known, so the buffer is allocated once
    // and a registry the format cannot describe fails before any work.
    uint64_t total = sizeof(SnapshotHeader);
    std::string refusal;
    registry.for_each_array([&](const RegisteredArray& array) {
        if (!refusal.empty()) return;
        if (array.name.size() > std::numeric_limits<uint32_t>::max()) {
            refusal = "array name is longer than the format's length field";
            return;
        }
        if (!element_count_fits_u32(array.world_count, array.capacity_per_world)) {
            refusal = "array '" + array.name + "': world_count * capacity_per_world overflows a uint32";
            return;
        }
        const uint64_t section = uint64_t{sizeof(SnapshotSectionHeader)} + array.name.size() +
                                 payload_bytes_for(array.elem_size, array.world_count, array.capacity_per_world);
        if (section > kMaxSnapshotBytes - total) {  // total <= kMaxSnapshotBytes always, so this cannot underflow
            refusal = "total snapshot size exceeds the " + std::to_string(kMaxSnapshotBytes) + "-byte ceiling";
            return;
        }
        total += section;
    });
    if (!refusal.empty()) {
        return std::unexpected(capacity_err(std::move(refusal)));
    }

    // The registry's own invariant, checked before it is dereferenced.
    std::string broken;
    registry.for_each_array([&broken](const RegisteredArray& array) {
        if (broken.empty() && array.data == nullptr && array.byte_size() != 0) {
            broken = "array '" + array.name + "' is registered with no storage but is not empty";
        }
    });
    if (!broken.empty()) {
        return std::unexpected(internal_err(std::move(broken)));
    }

    // Pass 2: write. Walk order is registration order, which is deterministic;
    // there is no container in this path whose iteration order depends on a
    // hash, a pointer or anything else that varies between runs.
    std::vector<std::byte> out;
    // The one big allocation, made once and sized exactly, so the appends
    // below cannot reallocate. Caught rather than propagated: no exception
    // crosses an engine module boundary, so an allocation failure becomes
    // capacity_exceeded -- the same discipline as arenas.cpp's nothrow arena
    // allocation.
    try {
        out.reserve(static_cast<std::size_t>(total));
    } catch (const std::bad_alloc&) {
        return std::unexpected(capacity_err("cannot allocate " + std::to_string(total) + " bytes for the blob"));
    }

    const SnapshotHeader head{
        .magic = kSnapshotMagic,
        .version = kSnapshotVersion,
        .schema_hash = schema_hash(registry),
        .tick = tick.value,
        .world_count = world_set_size(registry),
        .array_count = static_cast<uint32_t>(registry.size()),
        .configuration_identity = configuration_identity,
        .model_registry_identity = model_registry_identity,
    };
    append_pod(out, head);

    registry.for_each_array([&out](const RegisteredArray& array) {
        const std::size_t payload = array.byte_size();
        const SnapshotSectionHeader section{
            .name_length = static_cast<uint32_t>(array.name.size()),
            .elem_size = array.elem_size,
            .world_count = array.world_count,
            .capacity_per_world = array.capacity_per_world,
            .byte_length = payload,
        };
        append_pod(out, section);
        const auto* name_bytes = reinterpret_cast<const std::byte*>(array.name.data());
        out.insert(out.end(), name_bytes, name_bytes + array.name.size());
        if (payload != 0) {
            out.insert(out.end(), array.data, array.data + payload);
        }
    });

    if (out.size() != total) {
        return std::unexpected(internal_err("wrote " + std::to_string(out.size()) + " bytes, sized " + std::to_string(total)));
    }
    // Through the same door as every other blob: whatever save() produces is
    // exactly what a reader will accept, or save() is broken and says so here.
    return SnapshotBlob::from_bytes(std::move(out));
}

Result<SnapshotBlob> save(const ArenaSet& arenas, Tick tick, uint64_t configuration_identity,
                          uint64_t model_registry_identity) {
    return save(arenas.registry(), tick, configuration_identity, model_registry_identity);
}

// ---------------------------------------------------------------------------
// restore
// ---------------------------------------------------------------------------

Result<void> restore(const StateRegistry& registry, const SnapshotBlob& blob) {
    Result<std::vector<Section>> sections = validate_against(registry, blob);
    if (!sections) return std::unexpected(sections.error());
    apply_sections(registry, *sections);
    return {};
}

Result<void> restore(ArenaSet& arenas, const SnapshotBlob& blob) {
    const StateRegistry& registry = arenas.registry();

    Result<std::vector<Section>> sections = validate_against(registry, blob);
    if (!sections) return std::unexpected(sections.error());

    // Which sections are slot_to_world maps is not a guess: ArenaSet::
    // registry() is const, so nothing but ArenaSet::register_array() can add
    // to this registry, and that call contributes exactly two consecutive
    // entries per arena -- the elements, then the map. So section 2i+1 is
    // arena i's map, for every i. If that ever stops being true this check
    // fails loudly here instead of quietly skipping the validation below.
    const std::size_t expected = 2u * arenas.array_count();
    if (sections->size() != expected) {
        return std::unexpected(internal_err("arena set registers " + std::to_string(expected) +
                                            " arrays but its registry walk has " + std::to_string(sections->size()) +
                                            "; the elements/slot_to_world pairing this restore relies on has changed"));
    }
    for (std::size_t i = 1; i < sections->size(); i += 2) {
        const Section& map = (*sections)[i];
        if (!map.name.ends_with(kSlotToWorldSuffix) || map.elem_size != sizeof(uint32_t)) {
            return std::unexpected(internal_err("expected a slot_to_world map at walk position " + std::to_string(i) +
                                                ", found '" + std::string(map.name) + "'"));
        }
        if (Result<void> ok = check_slot_map(map); !ok) return ok;
    }

    apply_sections(registry, *sections);

    // The derivation that the walk deliberately does not carry. Cannot fail:
    // every index is in range by construction and every map was validated
    // above, which are resync's only two failure modes -- but a silent
    // divergence on the next spawn is exactly what this whole task exists to
    // prevent, so the impossible case is reported rather than assumed.
    for (uint32_t i = 0; i < arenas.array_count(); ++i) {
        if (Result<void> ok = arenas.resync_from_slot_to_world(ArrayIndex{i}); !ok) {
            return std::unexpected(internal_err("resync of array " + std::to_string(i) +
                                                " failed after a validated restore: " + ok.error().context));
        }
    }
    return {};
}

// ---------------------------------------------------------------------------
// find_section
// ---------------------------------------------------------------------------

Result<BlobSection> find_section(const SnapshotBlob& blob, std::string_view name) {
    // THE WHOLE TABLE IS PARSED, not scanned to the first hit and abandoned.
    // parse_sections() is what proves every declared length lies inside the
    // buffer, and a section that happens to sit before a truncated one is not a
    // section this function should hand out -- a caller reading it would be
    // reading from a blob restore() is about to reject anyway.
    Result<std::vector<Section>> sections = parse_sections(blob);
    if (!sections) return std::unexpected(sections.error());

    for (const Section& section : *sections) {
        if (section.name != name) continue;
        return BlobSection{
            .elem_size = section.elem_size,
            .world_count = section.world_count,
            .capacity_per_world = section.capacity_per_world,
            .payload = std::span<const std::byte>(section.payload, section.payload_bytes),
        };
    }
    // FIRST match would be ambiguous if a blob could carry two sections of one
    // name -- it cannot: a blob is written from a StateRegistry, which rejects
    // duplicate names. A hand-crafted blob that carries two is rejected by
    // restore()'s name-by-name match against the registry, so nothing that
    // reaches a caller of this function can have been ambiguous.
    return std::unexpected(Error{Code::not_found, "blob has no section named '" + std::string(name) + "'"});
}

}  // namespace spade
