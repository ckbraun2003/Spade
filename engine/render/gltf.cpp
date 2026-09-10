#include "render/gltf.hpp"

#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <iterator>
#include <optional>
#include <span>
#include <string>
#include <utility>
#include <vector>

#include <glm/geometric.hpp>
#include <glm/vec3.hpp>
#include <glm/vec4.hpp>

#include <nlohmann/json.hpp>

namespace spade::render {
namespace {

using Json = nlohmann::json;

[[nodiscard]] Error invalid(std::string context) {
    return Error{Code::invalid_argument, std::move(context)};
}
[[nodiscard]] Error io(std::string context) {
    return Error{Code::io_error, std::move(context)};
}

// ---------------------------------------------------------------------------
// glTF 2.0 accessor componentType codes this loader actually accepts (spec
// section 3.6.2.2 defines six; POSITION is FLOAT-only and indices are
// UNSIGNED_BYTE/SHORT/INT-only per the spec itself, so the signed BYTE/SHORT
// codes never appear in a value this loader validates through -- they are
// deliberately not modeled here at all, rather than kept as unreachable
// vocabulary).
// ---------------------------------------------------------------------------
constexpr int kUnsignedByte = 5121;
constexpr int kUnsignedShort = 5123;
constexpr int kUnsignedInt = 5125;
constexpr int kFloatComponent = 5126;

// PRECONDITION: `component_type` is one of the four constants above -- every
// caller validates that before computing a size with it.
[[nodiscard]] std::size_t component_byte_size(int component_type) {
    switch (component_type) {
        case kUnsignedByte:
            return 1;
        case kUnsignedShort:
            return 2;
        case kUnsignedInt:
        case kFloatComponent:
            return 4;
        default:
            return 0;  // unreachable given the precondition above
    }
}

// SCALAR/VEC2/VEC3/VEC4 only -- MAT2/MAT3/MAT4 never appear in this loader's
// two accessor roles (POSITION, indices) and are deliberately not modeled.
[[nodiscard]] std::size_t accessor_type_arity(const std::string& type) {
    if (type == "SCALAR") return 1;
    if (type == "VEC2") return 2;
    if (type == "VEC3") return 3;
    if (type == "VEC4") return 4;
    return 0;
}

// ---------------------------------------------------------------------------
// A minimal, self-contained base64 decoder (RFC 4648) -- no external
// dependency, pure integer arithmetic (no libm, deterministic). Whitespace is
// tolerated and skipped; '=' padding is simply ignored rather than position-
// validated, which is enough for this loader's one real input (glTF's own
// `data:application/octet-stream;base64,...` URIs, which never carry
// whitespace in practice) without inventing a stricter RFC checker no
// producer here needs.
// ---------------------------------------------------------------------------

[[nodiscard]] int base64_value(char c) {
    if (c >= 'A' && c <= 'Z') return c - 'A';
    if (c >= 'a' && c <= 'z') return c - 'a' + 26;
    if (c >= '0' && c <= '9') return c - '0' + 52;
    if (c == '+') return 62;
    if (c == '/') return 63;
    return -1;
}

[[nodiscard]] Result<std::vector<uint8_t>> decode_base64(std::string_view text) {
    std::vector<uint8_t> out;
    out.reserve(text.size() / 4 * 3 + 3);
    uint32_t buffer = 0;
    int bits = 0;
    for (char c : text) {
        if (c == '=' || c == '\r' || c == '\n' || c == ' ' || c == '\t') {
            continue;
        }
        const int value = base64_value(c);
        if (value < 0) {
            return std::unexpected(invalid("gltf: buffer data URI contains an invalid base64 character"));
        }
        buffer = (buffer << 6) | static_cast<uint32_t>(value);
        bits += 6;
        if (bits >= 8) {
            bits -= 8;
            out.push_back(static_cast<uint8_t>((buffer >> bits) & 0xFFu));
        }
    }
    return out;
}

constexpr std::string_view kDataUriPrefix = "data:application/octet-stream;base64,";

// Decodes buffers[index]'s raw bytes -- an embedded base64 data URI or a
// relative external file resolved against `base_dir` (parse_gltf's own
// parameter; load_gltf() passes path.parent_path()). No support for a
// buffer with no `uri` at all: that shape is an embedded GLB binary chunk,
// which this loader's ".glb is out of scope" boundary (gltf.hpp) excludes.
[[nodiscard]] Result<std::vector<uint8_t>> load_buffer_bytes(const Json& buffer, std::size_t index,
                                                              const std::filesystem::path& base_dir) {
    if (!buffer.is_object() || !buffer.contains("uri") || !buffer["uri"].is_string()) {
        return std::unexpected(invalid("gltf: buffers[" + std::to_string(index) +
                                        "] has no string 'uri' (embedded GLB binary chunks are not supported)"));
    }
    const std::string uri = buffer["uri"].get<std::string>();
    std::vector<uint8_t> bytes;
    if (uri.starts_with("data:")) {
        if (!uri.starts_with(kDataUriPrefix)) {
            return std::unexpected(invalid("gltf: buffers[" + std::to_string(index) +
                                            "] uses an unsupported data URI (only '" +
                                            std::string(kDataUriPrefix) + "...' is supported)"));
        }
        Result<std::vector<uint8_t>> decoded =
            decode_base64(std::string_view(uri).substr(kDataUriPrefix.size()));
        if (!decoded) {
            return std::unexpected(decoded.error());
        }
        bytes = std::move(*decoded);
    } else {
        const std::filesystem::path bin_path = base_dir / uri;
        std::ifstream in(bin_path, std::ios::binary);
        if (!in) {
            return std::unexpected(io("gltf: cannot open external buffer file '" + bin_path.string() + "'"));
        }
        bytes.assign(std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>());
        if (in.bad()) {
            return std::unexpected(io("gltf: failed reading external buffer file '" + bin_path.string() + "'"));
        }
    }
    if (buffer.contains("byteLength") && buffer["byteLength"].is_number_unsigned()) {
        const std::size_t declared = buffer["byteLength"].get<std::size_t>();
        if (declared != bytes.size()) {
            return std::unexpected(invalid("gltf: buffers[" + std::to_string(index) + "] declares byteLength " +
                                            std::to_string(declared) + " but decoded to " +
                                            std::to_string(bytes.size()) + " bytes"));
        }
    }
    return bytes;
}

// ---------------------------------------------------------------------------
// Accessor decoding. An accessor names a bufferView, a componentType/type
// (which together fix each element's byte size) and a count; this loader
// requires tightly-packed data (no interleaving) and no sparse override --
// neither of which tools/gen_meshes.py ever emits, and both of which would
// otherwise be silently misread rather than rejected.
// ---------------------------------------------------------------------------

struct AccessorHeader {
    std::size_t buffer_view = 0;
    std::size_t byte_offset = 0;  // the accessor's own byteOffset within its bufferView
    int component_type = 0;
    std::size_t count = 0;
    std::size_t arity = 0;  // 1 (SCALAR) or 3 (VEC3) -- the only two this loader reads
};

// Decodes buffers[] LAZILY, on first actual use, and caches the result for
// any later accessor sharing the same buffer. NOT an eager up-front decode of
// every declared buffer: this loader only ever reads meshes[0]'s primitives,
// and a well-formed multi-mesh file may declare a buffer that belongs to some
// OTHER mesh this loader never touches -- eagerly resolving (and requiring
// the presence of) every buffer in the document would reject a perfectly
// loadable file over data this loader has no business needing. It also
// naturally orders "is this accessor/bufferView shape even valid" checks
// BEFORE any file I/O or base64 decode for a buffer that shape never ends up
// needing.
class BufferCache {
   public:
    BufferCache(const Json& buffers, const std::filesystem::path& base_dir)
        : buffers_(buffers), base_dir_(base_dir), decoded_(buffers.size()) {}

    [[nodiscard]] Result<std::span<const uint8_t>> bytes(std::size_t index) {
        if (!decoded_[index].has_value()) {
            Result<std::vector<uint8_t>> loaded = load_buffer_bytes(buffers_[index], index, base_dir_);
            if (!loaded) {
                return std::unexpected(loaded.error());
            }
            decoded_[index] = std::move(*loaded);
        }
        return std::span<const uint8_t>(*decoded_[index]);
    }

   private:
    const Json& buffers_;
    const std::filesystem::path& base_dir_;
    std::vector<std::optional<std::vector<uint8_t>>> decoded_;  // sized once, never resized -- spans stay valid
};

[[nodiscard]] Result<AccessorHeader> read_accessor_header(const Json& accessors, std::size_t accessor_index,
                                                           std::string_view attribute_name) {
    if (accessor_index >= accessors.size()) {
        return std::unexpected(invalid("gltf: " + std::string(attribute_name) + " accessor index " +
                                        std::to_string(accessor_index) + " is out of range (accessors has " +
                                        std::to_string(accessors.size()) + " entries)"));
    }
    const Json& a = accessors[accessor_index];
    if (!a.is_object()) {
        return std::unexpected(invalid("gltf: accessors[" + std::to_string(accessor_index) + "] is not an object"));
    }
    if (a.contains("sparse")) {
        return std::unexpected(invalid("gltf: accessors[" + std::to_string(accessor_index) +
                                        "] is sparse, which this loader does not support"));
    }
    if (!a.contains("bufferView") || !a["bufferView"].is_number_unsigned()) {
        return std::unexpected(
            invalid("gltf: accessors[" + std::to_string(accessor_index) +
                    "] has no unsigned 'bufferView' (bufferView-less/zero-filled accessors are not supported)"));
    }
    if (!a.contains("componentType") || !a["componentType"].is_number_integer()) {
        return std::unexpected(
            invalid("gltf: accessors[" + std::to_string(accessor_index) + "] has no integer 'componentType'"));
    }
    if (!a.contains("count") || !a["count"].is_number_unsigned()) {
        return std::unexpected(
            invalid("gltf: accessors[" + std::to_string(accessor_index) + "] has no unsigned 'count'"));
    }
    if (!a.contains("type") || !a["type"].is_string()) {
        return std::unexpected(
            invalid("gltf: accessors[" + std::to_string(accessor_index) + "] has no string 'type'"));
    }
    const std::size_t arity = accessor_type_arity(a["type"].get<std::string>());
    if (arity == 0) {
        return std::unexpected(invalid("gltf: accessors[" + std::to_string(accessor_index) +
                                        "] has an unsupported 'type' (only SCALAR/VEC2/VEC3/VEC4 are read)"));
    }

    AccessorHeader header;
    header.buffer_view = a["bufferView"].get<std::size_t>();
    header.byte_offset =
        (a.contains("byteOffset") && a["byteOffset"].is_number_unsigned()) ? a["byteOffset"].get<std::size_t>() : 0;
    header.component_type = a["componentType"].get<int>();
    header.count = a["count"].get<std::size_t>();
    header.arity = arity;
    return header;
}

// Resolves an accessor header to the raw bytes it names, checking every
// index/offset/length along the way: bufferView index, its buffer index, the
// bufferView's own range against its buffer, and the accessor's range
// against its bufferView. Rejects an interleaved bufferView (a `byteStride`
// other than this accessor's own natural element size) rather than silently
// reading the wrong bytes.
[[nodiscard]] Result<std::span<const uint8_t>> accessor_byte_span(const AccessorHeader& header,
                                                                   const Json& buffer_views, std::size_t buffer_count,
                                                                   BufferCache& buffers,
                                                                   std::string_view attribute_name) {
    if (header.buffer_view >= buffer_views.size()) {
        return std::unexpected(invalid("gltf: " + std::string(attribute_name) + "'s bufferView index " +
                                        std::to_string(header.buffer_view) + " is out of range"));
    }
    const Json& view = buffer_views[header.buffer_view];
    if (!view.is_object() || !view.contains("buffer") || !view["buffer"].is_number_unsigned()) {
        return std::unexpected(
            invalid("gltf: bufferViews[" + std::to_string(header.buffer_view) + "] has no unsigned 'buffer'"));
    }
    const std::size_t buffer_index = view["buffer"].get<std::size_t>();
    if (buffer_index >= buffer_count) {
        return std::unexpected(invalid("gltf: bufferViews[" + std::to_string(header.buffer_view) +
                                        "]'s buffer index " + std::to_string(buffer_index) + " is out of range"));
    }
    if (!view.contains("byteLength") || !view["byteLength"].is_number_unsigned()) {
        return std::unexpected(
            invalid("gltf: bufferViews[" + std::to_string(header.buffer_view) + "] has no unsigned 'byteLength'"));
    }
    const std::size_t view_offset =
        (view.contains("byteOffset") && view["byteOffset"].is_number_unsigned()) ? view["byteOffset"].get<std::size_t>()
                                                                                  : 0;
    const std::size_t view_length = view["byteLength"].get<std::size_t>();

    const std::size_t element_size = component_byte_size(header.component_type) * header.arity;
    if (view.contains("byteStride") && view["byteStride"].is_number_unsigned() &&
        view["byteStride"].get<std::size_t>() != element_size) {
        return std::unexpected(invalid("gltf: bufferViews[" + std::to_string(header.buffer_view) +
                                        "] is interleaved (byteStride), which this loader does not support"));
    }

    const std::size_t needed = element_size * header.count;
    if (header.byte_offset > view_length || needed > view_length - header.byte_offset) {
        return std::unexpected(invalid("gltf: " + std::string(attribute_name) +
                                        "'s accessor runs past the end of its bufferView"));
    }

    // Only NOW -- every index/shape check above already passed -- does this
    // function touch the buffer's actual bytes (decoding/reading it on first
    // use, via BufferCache).
    Result<std::span<const uint8_t>> buffer_bytes = buffers.bytes(buffer_index);
    if (!buffer_bytes) {
        return std::unexpected(buffer_bytes.error());
    }
    if (view_offset > buffer_bytes->size() || view_length > buffer_bytes->size() - view_offset) {
        return std::unexpected(
            invalid("gltf: bufferViews[" + std::to_string(header.buffer_view) + "] runs past the end of its buffer"));
    }
    const std::size_t absolute_offset = view_offset + header.byte_offset;
    return buffer_bytes->subspan(absolute_offset, needed);
}

// Reads a VEC3/FLOAT accessor's bytes (POSITION or, when present, NORMAL)
// into a vec3 array, rejecting any non-finite component -- "positions
// finite" is part of this loader's own contract with a valid MeshData, not
// just a happy-path property of well-formed files, and the same holds for a
// file's own NORMAL data once this loader has decided to trust it.
[[nodiscard]] Result<std::vector<glm::vec3>> read_vec3_float_accessor(const AccessorHeader& header,
                                                                       std::span<const uint8_t> bytes,
                                                                       std::string_view attribute_name) {
    std::vector<glm::vec3> out(header.count);
    for (std::size_t i = 0; i < header.count; ++i) {
        float components[3];
        std::memcpy(components, bytes.data() + i * 3 * sizeof(float), 3 * sizeof(float));
        out[i] = glm::vec3(components[0], components[1], components[2]);
        if (!std::isfinite(out[i].x) || !std::isfinite(out[i].y) || !std::isfinite(out[i].z)) {
            return std::unexpected(invalid("gltf: " + std::string(attribute_name) +
                                            " accessor contains a non-finite value at vertex " + std::to_string(i)));
        }
    }
    return out;
}

// Reads a SCALAR/(UNSIGNED_BYTE|UNSIGNED_SHORT|UNSIGNED_INT) accessor's bytes
// into 32-bit indices. The component type is already validated by the caller
// (one of the three cases below) before this is reached.
[[nodiscard]] std::vector<uint32_t> read_scalar_index_accessor(const AccessorHeader& header,
                                                                std::span<const uint8_t> bytes) {
    std::vector<uint32_t> out(header.count);
    for (std::size_t i = 0; i < header.count; ++i) {
        switch (header.component_type) {
            case kUnsignedByte:
                out[i] = static_cast<uint32_t>(bytes[i]);
                break;
            case kUnsignedShort: {
                uint16_t value;
                std::memcpy(&value, bytes.data() + i * sizeof(uint16_t), sizeof(uint16_t));
                out[i] = static_cast<uint32_t>(value);
                break;
            }
            case kUnsignedInt: {
                uint32_t value;
                std::memcpy(&value, bytes.data() + i * sizeof(uint32_t), sizeof(uint32_t));
                out[i] = value;
                break;
            }
            default:
                break;  // unreachable -- caller validated component_type first
        }
    }
    return out;
}

// ---------------------------------------------------------------------------
// Top-level document parsing.
// ---------------------------------------------------------------------------

[[nodiscard]] Result<MeshData> parse_gltf_impl(std::string_view json_text, const std::filesystem::path& base_dir) {
    // Json::parse throws nlohmann::json::exception (parse_error) on bad
    // syntax -- caught by parse_gltf()'s own try/catch below, which is why
    // this function is free to call it unguarded.
    const Json root = Json::parse(std::string(json_text));

    if (!root.is_object()) {
        return std::unexpected(invalid("gltf: top-level document must be a JSON object"));
    }
    if (!root.contains("meshes") || !root["meshes"].is_array()) {
        return std::unexpected(invalid("gltf: missing 'meshes' array"));
    }
    const Json& meshes = root["meshes"];
    if (meshes.empty()) {
        return std::unexpected(invalid("gltf: 'meshes' is empty"));
    }
    // Only the first mesh is read -- see gltf.hpp's own "MULTIPLE meshes[]"
    // note. Every shipped file (tools/gen_meshes.py) carries exactly one.
    const Json& mesh = meshes[0];
    if (!mesh.is_object() || !mesh.contains("primitives") || !mesh["primitives"].is_array() ||
        mesh["primitives"].empty()) {
        return std::unexpected(invalid("gltf: meshes[0] has no non-empty 'primitives' array"));
    }
    const Json& primitives = mesh["primitives"];

    if (!root.contains("accessors") || !root["accessors"].is_array()) {
        return std::unexpected(invalid("gltf: missing 'accessors' array"));
    }
    if (!root.contains("bufferViews") || !root["bufferViews"].is_array()) {
        return std::unexpected(invalid("gltf: missing 'bufferViews' array"));
    }
    if (!root.contains("buffers") || !root["buffers"].is_array()) {
        return std::unexpected(invalid("gltf: missing 'buffers' array"));
    }
    const Json& accessors = root["accessors"];
    const Json& buffer_views = root["bufferViews"];
    const Json& buffers = root["buffers"];
    const std::size_t buffer_count = buffers.size();
    BufferCache buffer_cache(buffers, base_dir);  // decodes lazily -- see its own comment

    MeshData mesh_data;
    // Submesh order follows primitives[]'s own array index (SR-11) -- never a
    // map/unordered-container iteration order. `primitives` is a JSON ARRAY
    // (nlohmann::json's array_t preserves insertion/file order unlike its
    // object_t), and this loop reads it by increasing index, so this is
    // structural, not merely observed.
    for (std::size_t p = 0; p < primitives.size(); ++p) {
        const Json& prim = primitives[p];
        if (!prim.is_object()) {
            return std::unexpected(invalid("gltf: primitives[" + std::to_string(p) + "] is not an object"));
        }
        if (prim.contains("mode")) {
            if (!prim["mode"].is_number_integer()) {
                return std::unexpected(
                    invalid("gltf: primitives[" + std::to_string(p) + "]'s 'mode' must be an integer"));
            }
            if (prim["mode"].get<int>() != 4) {
                return std::unexpected(invalid("gltf: primitives[" + std::to_string(p) + "] has mode " +
                                                std::to_string(prim["mode"].get<int>()) +
                                                " -- only TRIANGLES (mode 4) is supported"));
            }
        }
        if (!prim.contains("attributes") || !prim["attributes"].is_object() ||
            !prim["attributes"].contains("POSITION") || !prim["attributes"]["POSITION"].is_number_unsigned()) {
            return std::unexpected(invalid("gltf: primitives[" + std::to_string(p) +
                                            "] is missing an unsigned 'attributes.POSITION'"));
        }
        const std::size_t position_accessor_index = prim["attributes"]["POSITION"].get<std::size_t>();

        Result<AccessorHeader> position_header = read_accessor_header(accessors, position_accessor_index, "POSITION");
        if (!position_header) {
            return std::unexpected(position_header.error());
        }
        if (position_header->arity != 3 || position_header->component_type != kFloatComponent) {
            return std::unexpected(invalid(
                "gltf: POSITION accessor has an unsupported component type or type (must be VEC3 of FLOAT)"));
        }
        Result<std::span<const uint8_t>> position_bytes =
            accessor_byte_span(*position_header, buffer_views, buffer_count, buffer_cache, "POSITION");
        if (!position_bytes) {
            return std::unexpected(position_bytes.error());
        }
        Result<std::vector<glm::vec3>> positions =
            read_vec3_float_accessor(*position_header, *position_bytes, "POSITION");
        if (!positions) {
            return std::unexpected(positions.error());
        }

        if (!prim.contains("indices") || !prim["indices"].is_number_unsigned()) {
            return std::unexpected(
                invalid("gltf: primitives[" + std::to_string(p) + "] is missing an unsigned 'indices'"));
        }
        const std::size_t indices_accessor_index = prim["indices"].get<std::size_t>();
        Result<AccessorHeader> index_header = read_accessor_header(accessors, indices_accessor_index, "indices");
        if (!index_header) {
            return std::unexpected(index_header.error());
        }
        if (index_header->arity != 1 || (index_header->component_type != kUnsignedByte &&
                                          index_header->component_type != kUnsignedShort &&
                                          index_header->component_type != kUnsignedInt)) {
            return std::unexpected(invalid(
                "gltf: indices accessor has an unsupported component type or type (must be SCALAR of "
                "UNSIGNED_BYTE/UNSIGNED_SHORT/UNSIGNED_INT)"));
        }
        if (index_header->count % 3 != 0) {
            return std::unexpected(
                invalid("gltf: primitives[" + std::to_string(p) + "]'s index count is not a multiple of 3"));
        }
        Result<std::span<const uint8_t>> index_bytes =
            accessor_byte_span(*index_header, buffer_views, buffer_count, buffer_cache, "indices");
        if (!index_bytes) {
            return std::unexpected(index_bytes.error());
        }
        const std::vector<uint32_t> raw_indices = read_scalar_index_accessor(*index_header, *index_bytes);

        for (uint32_t idx : raw_indices) {
            if (idx >= positions->size()) {
                return std::unexpected(invalid("gltf: primitives[" + std::to_string(p) +
                                                "] has an index referencing vertex " + std::to_string(idx) +
                                                ", past POSITION's " + std::to_string(positions->size()) +
                                                " vertices"));
            }
        }

        // The primitive's raw material index, copied VERBATIM into
        // MeshData::submesh_material -- this loader never reads the file's
        // own `materials[]` array (gltf.hpp's "OUT OF SCOPE" note); resolving
        // this index against real material data, if that ever happens, is a
        // later task's job. Absent -> 0, matching MeshData's own "material
        // index 0 is always a valid default" convention (scene.hpp).
        std::size_t material_index = 0;
        if (prim.contains("material")) {
            if (!prim["material"].is_number_unsigned()) {
                return std::unexpected(
                    invalid("gltf: primitives[" + std::to_string(p) + "]'s 'material' must be a non-negative integer"));
            }
            material_index = prim["material"].get<std::size_t>();
        }

        // NORMAL, READ WHEN PRESENT (fix round, review "Important" finding):
        // task-R4-brief.md states flat generation as a CONDITION ("when
        // absent" / "when the file has none"), not an unconditional rule --
        // see gltf.hpp's own corrected header comment. A present NORMAL is
        // read through the identical VEC3/FLOAT accessor path already built
        // for POSITION, and must be PARALLEL to it (same count) -- glTF's own
        // requirement for per-vertex attributes on one primitive.
        std::optional<std::vector<glm::vec3>> file_normals;
        if (prim["attributes"].contains("NORMAL")) {
            if (!prim["attributes"]["NORMAL"].is_number_unsigned()) {
                return std::unexpected(invalid("gltf: primitives[" + std::to_string(p) +
                                                "]'s 'attributes.NORMAL' must be an unsigned integer"));
            }
            const std::size_t normal_accessor_index = prim["attributes"]["NORMAL"].get<std::size_t>();
            Result<AccessorHeader> normal_header = read_accessor_header(accessors, normal_accessor_index, "NORMAL");
            if (!normal_header) {
                return std::unexpected(normal_header.error());
            }
            if (normal_header->arity != 3 || normal_header->component_type != kFloatComponent) {
                return std::unexpected(invalid(
                    "gltf: NORMAL accessor has an unsupported component type or type (must be VEC3 of FLOAT)"));
            }
            if (normal_header->count != positions->size()) {
                return std::unexpected(invalid(
                    "gltf: primitives[" + std::to_string(p) + "]'s NORMAL accessor count " +
                    std::to_string(normal_header->count) + " does not match POSITION's vertex count " +
                    std::to_string(positions->size())));
            }
            Result<std::span<const uint8_t>> normal_bytes =
                accessor_byte_span(*normal_header, buffer_views, buffer_count, buffer_cache, "NORMAL");
            if (!normal_bytes) {
                return std::unexpected(normal_bytes.error());
            }
            Result<std::vector<glm::vec3>> read_normals =
                read_vec3_float_accessor(*normal_header, *normal_bytes, "NORMAL");
            if (!read_normals) {
                return std::unexpected(read_normals.error());
            }
            file_normals = std::move(*read_normals);
        }

        // Positions/indices (and, above, NORMAL) are validated in full BEFORE
        // any of this primitive's data is appended to mesh_data, so a
        // failure partway through a later primitive never leaves an earlier
        // one's contribution as the only partial state -- the whole function
        // returns an error and mesh_data, wherever it got to, is simply
        // discarded (never returned to the caller).
        const uint32_t submesh_first = static_cast<uint32_t>(mesh_data.indices.size());
        if (file_normals.has_value()) {
            // SHARED-VERTEX PATH (NORMAL present): positions/normals/indices
            // are appended UNCHANGED from the file's own accessors, only
            // offset into this MeshData's running vertex/index space -- no
            // duplication, exactly like the source accessors themselves.
            const uint32_t vertex_offset = static_cast<uint32_t>(mesh_data.positions.size());
            mesh_data.positions.insert(mesh_data.positions.end(), positions->begin(), positions->end());
            mesh_data.normals.insert(mesh_data.normals.end(), file_normals->begin(), file_normals->end());
            for (uint32_t idx : raw_indices) {
                mesh_data.indices.push_back(idx + vertex_offset);
            }
        } else {
            // FLAT (PER-FACE) NORMAL PATH (NORMAL absent): every triangle
            // gets three brand-new, unshared vertices so MeshData::normals's
            // own contract ("flat meshes duplicate vertices", scene.hpp)
            // holds exactly.
            for (std::size_t t = 0; t + 3 <= raw_indices.size(); t += 3) {
                const glm::vec3& p0 = (*positions)[raw_indices[t]];
                const glm::vec3& p1 = (*positions)[raw_indices[t + 1]];
                const glm::vec3& p2 = (*positions)[raw_indices[t + 2]];
                const glm::vec3 normal = glm::normalize(glm::cross(p1 - p0, p2 - p0));
                if (!std::isfinite(normal.x) || !std::isfinite(normal.y) || !std::isfinite(normal.z)) {
                    return std::unexpected(invalid("gltf: primitives[" + std::to_string(p) +
                                                    "] has a degenerate (zero-area) triangle at index " +
                                                    std::to_string(t)));
                }
                const uint32_t base = static_cast<uint32_t>(mesh_data.positions.size());
                mesh_data.positions.push_back(p0);
                mesh_data.positions.push_back(p1);
                mesh_data.positions.push_back(p2);
                mesh_data.normals.push_back(normal);
                mesh_data.normals.push_back(normal);
                mesh_data.normals.push_back(normal);
                mesh_data.indices.push_back(base);
                mesh_data.indices.push_back(base + 1);
                mesh_data.indices.push_back(base + 2);
            }
        }
        const uint32_t submesh_count = static_cast<uint32_t>(mesh_data.indices.size()) - submesh_first;

        // ALWAYS explicit, never the tessellate.cpp-style empty/implicit
        // shorthand (SR-11) -- gltf.hpp's own header comment: this loader is
        // the first PRODUCER of the non-empty submesh case, even for a
        // single primitive.
        mesh_data.submesh_first_index.push_back(submesh_first);
        mesh_data.submesh_index_count.push_back(submesh_count);
        mesh_data.submesh_material.push_back(static_cast<uint32_t>(material_index));
    }

    // The file's OWN material palette (MeshData::source_materials). This is
    // the "later task" gltf.hpp's OUT-OF-SCOPE note deferred: until it existed,
    // a primitive's material index was carried through unresolved and then
    // applied by raster_cpu to the SCENE's palette, so an authored mesh drew
    // in whatever colours the surrounding world happened to have at those
    // indices.
    //
    // Deliberately narrow, and still not a PBR loader: baseColorFactor only.
    // metallic/roughness/textures/alphaMode stay out of scope, because
    // render::Material carries no field for them -- reading a value nothing
    // can render would be the same defect in the other direction.
    //
    // A file with no `materials[]` leaves this EMPTY, which preserves the
    // original meaning of submesh_material exactly (scene-palette indices) for
    // every producer and every existing fixture.
    if (root.contains("materials") && root["materials"].is_array()) {
        const Json& mats = root["materials"];
        mesh_data.source_materials.reserve(mats.size());
        for (std::size_t m = 0; m < mats.size(); ++m) {
            Material out{};  // defaults stand for anything the file omits
            const Json& mat = mats[m];
            if (mat.is_object() && mat.contains("pbrMetallicRoughness") &&
                mat["pbrMetallicRoughness"].is_object()) {
                const Json& pbr = mat["pbrMetallicRoughness"];
                if (pbr.contains("baseColorFactor") && pbr["baseColorFactor"].is_array() &&
                    pbr["baseColorFactor"].size() == 4) {
                    const Json& c = pbr["baseColorFactor"];
                    for (std::size_t k = 0; k < 4; ++k) {
                        if (!c[k].is_number()) {
                            return std::unexpected(invalid(
                                "gltf: materials[" + std::to_string(m) +
                                "].pbrMetallicRoughness.baseColorFactor must be four numbers"));
                        }
                    }
                    out.base_color = glm::vec4(c[0].get<float>(), c[1].get<float>(),
                                                c[2].get<float>(), c[3].get<float>());
                }
            }
            mesh_data.source_materials.push_back(out);
        }
    }

    return mesh_data;
}

}  // namespace

Result<MeshData> parse_gltf(std::string_view json_text, const std::filesystem::path& base_dir) {
    try {
        return parse_gltf_impl(json_text, base_dir);
    } catch (const nlohmann::json::exception& e) {
        // nlohmann's OWN expected exception family -- bad JSON syntax, or a
        // defensive check above missed a type mismatch a raw .get<T>() then
        // threw on. Either way, the cause is the FILE, not this loader.
        return std::unexpected(invalid(std::string("gltf: ") + e.what()));
    } catch (const std::exception& e) {
        // Anything else (e.g. std::bad_alloc) is not a shape this parser
        // itself recognizes as "a broken file" -- reported distinctly, same
        // split world_file.cpp's own world_from_yaml() makes between
        // YAML::Exception and a generic std::exception.
        return std::unexpected(Error{Code::internal, std::string("gltf: unexpected failure: ") + e.what()});
    }
}

Result<MeshData> load_gltf(const std::filesystem::path& path) {
    std::string text;
    try {
        std::ifstream in(path, std::ios::binary);
        if (!in) {
            return std::unexpected(io("gltf: cannot open '" + path.string() + "'"));
        }
        text.assign(std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>());
        if (in.bad()) {
            return std::unexpected(io("gltf: failed reading '" + path.string() + "'"));
        }
    } catch (const std::exception& e) {
        return std::unexpected(io("gltf: failed reading '" + path.string() + "': " + e.what()));
    }
    return parse_gltf(text, path.parent_path());
}

}  // namespace spade::render
