// world/mesh_import/gltf.cpp -- the glTF 2.0 geometry reader. See
// world/mesh_import.hpp for the contract, the refusals and their order.
//
// One Reader per call. It parses the container (GLB chunks, or JSON text),
// checks the asset and extensionsRequired, walks the default scene depth
// first, and builds one object per mesh node: every primitive's kinds and
// counts first, then the object's capacity, then the data, then the weld and
// make_mesh(). Buffers load lazily, once, when an accessor first reads one.
// Every number in the file is read byte by byte, little-endian; the
// transform is + - * / in double, each sum left to right.
#include "world/mesh_import.hpp"

#include <array>
#include <bit>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <expected>
#include <fstream>
#include <limits>
#include <new>
#include <optional>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include <nlohmann/json.hpp>

#include "core/path_text.hpp"
#include "world/mesh_import/weld.hpp"

namespace spade {
namespace {

using Json = nlohmann::json;

constexpr uint32_t kGlbMagic = 0x46546C67u;   // "glTF"
constexpr uint32_t kChunkJson = 0x4E4F534Au;  // "JSON"
constexpr uint32_t kChunkBin = 0x004E4942u;   // "BIN\0"

constexpr uint64_t kUnsignedByte = 5121;
constexpr uint64_t kUnsignedShort = 5123;
constexpr uint64_t kUnsignedInt = 5125;
constexpr uint64_t kFloat = 5126;
constexpr uint64_t kTriangles = 4;

[[nodiscard]] uint32_t le32(const std::byte* p) noexcept {
    return std::to_integer<uint32_t>(p[0]) | (std::to_integer<uint32_t>(p[1]) << 8) |
           (std::to_integer<uint32_t>(p[2]) << 16) | (std::to_integer<uint32_t>(p[3]) << 24);
}

[[nodiscard]] std::string num(uint64_t v) {
    return std::to_string(v);
}

[[nodiscard]] std::string_view mode_name(uint64_t mode) noexcept {
    switch (mode) {
        case 0: return "POINTS";
        case 1: return "LINES";
        case 2: return "LINE_LOOP";
        case 3: return "LINE_STRIP";
        case 4: return "TRIANGLES";
        case 5: return "TRIANGLE_STRIP";
        case 6: return "TRIANGLE_FAN";
        default: return {};
    }
}

[[nodiscard]] uint64_t saturating_add(uint64_t a, uint64_t b) noexcept {
    return a > std::numeric_limits<uint64_t>::max() - b ? std::numeric_limits<uint64_t>::max() : a + b;
}

// Whether `count` elements of `element` bytes, `stride` apart from `offset`,
// lie inside `length` bytes. count >= 1 and stride >= element >= 1.
[[nodiscard]] bool fits(uint64_t offset, uint64_t count, uint64_t stride, uint64_t element, uint64_t length) noexcept {
    if (offset > length || element > length - offset) return false;
    return count - 1 <= (length - offset - element) / stride;
}

// The value of a node's `name` (or a mesh's): empty when absent or not text.
[[nodiscard]] std::string name_of(const Json& object) {
    const auto it = object.find("name");
    return it != object.end() && it->is_string() ? it->get<std::string>() : std::string();
}

// --- URIs ---------------------------------------------------------------------

[[nodiscard]] int sextet(char c) noexcept {
    if (c >= 'A' && c <= 'Z') return c - 'A';
    if (c >= 'a' && c <= 'z') return c - 'a' + 26;
    if (c >= '0' && c <= '9') return c - '0' + 52;
    if (c == '+') return 62;
    if (c == '/') return 63;
    return -1;
}

// RFC 4648 base64, with or without its padding.
[[nodiscard]] std::optional<std::vector<std::byte>> decode_base64(std::string_view s) {
    for (int pad = 0; pad < 2 && !s.empty() && s.back() == '='; ++pad) s.remove_suffix(1);
    if (s.size() % 4 == 1) return std::nullopt;
    std::vector<std::byte> out;
    out.reserve(s.size() / 4 * 3 + 2);
    uint32_t acc = 0;
    int bits = 0;
    for (const char c : s) {
        const int v = sextet(c);
        if (v < 0) return std::nullopt;
        acc = (acc << 6) | static_cast<uint32_t>(v);
        bits += 6;
        if (bits >= 8) {
            bits -= 8;
            out.push_back(static_cast<std::byte>((acc >> bits) & 0xFFu));
            acc &= (1u << bits) - 1u;
        }
    }
    return out;
}

[[nodiscard]] int hex_digit(char c) noexcept {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return -1;
}

// RFC 3986 percent-decoding; the result is UTF-8 bytes.
[[nodiscard]] std::optional<std::string> percent_decode(std::string_view s) {
    std::string out;
    for (std::size_t i = 0; i < s.size(); ++i) {
        if (s[i] != '%') {
            out += s[i];
            continue;
        }
        if (i + 2 >= s.size()) return std::nullopt;
        const int hi = hex_digit(s[i + 1]);
        const int lo = hex_digit(s[i + 2]);
        if (hi < 0 || lo < 0) return std::nullopt;
        out += static_cast<char>((hi << 4) | lo);
        i += 2;
    }
    return out;
}

// The URI's scheme ("https" in "https://..."), or empty for a relative
// reference: a scheme is letters, digits, + - and ., starting with a letter,
// before the first ':' that comes before any '/', '?' or '#'.
[[nodiscard]] std::string_view uri_scheme(std::string_view uri) noexcept {
    const std::size_t end = uri.find_first_of(":/?#");
    if (end == std::string_view::npos || end == 0 || uri[end] != ':') return {};
    const auto alpha = [](char c) { return (c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z'); };
    if (!alpha(uri[0])) return {};
    for (std::size_t i = 1; i < end; ++i) {
        const char c = uri[i];
        if (!alpha(c) && !(c >= '0' && c <= '9') && c != '+' && c != '-' && c != '.') return {};
    }
    return uri.substr(0, end);
}

[[nodiscard]] std::optional<std::vector<std::byte>> read_file(const std::filesystem::path& path) {
    std::ifstream in(path, std::ios::binary);
    if (!in) return std::nullopt;
    in.seekg(0, std::ios::end);
    const std::streamoff size = in.tellg();
    if (size < 0) return std::nullopt;
    in.seekg(0, std::ios::beg);
    std::vector<std::byte> out(static_cast<std::size_t>(size));
    if (size > 0 && !in.read(reinterpret_cast<char*>(out.data()), static_cast<std::streamsize>(size))) {
        return std::nullopt;
    }
    return out;
}

// --- transforms -----------------------------------------------------------------

// The top three rows of a 4x4 affine matrix, row-major, applied as m * (p, 1).
using Affine = std::array<std::array<double, 4>, 3>;

constexpr Affine kIdentity{{{1.0, 0.0, 0.0, 0.0}, {0.0, 1.0, 0.0, 0.0}, {0.0, 0.0, 1.0, 0.0}}};

// a * b.
[[nodiscard]] Affine compose(const Affine& a, const Affine& b) noexcept {
    Affine r{};
    for (std::size_t i = 0; i < 3; ++i) {
        for (std::size_t j = 0; j < 4; ++j) {
            double s = a[i][0] * b[0][j] + a[i][1] * b[1][j] + a[i][2] * b[2][j];
            if (j == 3) s = s + a[i][3];
            r[i][j] = s;
        }
    }
    return r;
}

// m * (p, 1). Not "apply": std::apply is found by ADL through std::array.
[[nodiscard]] glm::dvec3 transform_point(const Affine& m, const glm::dvec3& p) noexcept {
    return {m[0][0] * p.x + m[0][1] * p.y + m[0][2] * p.z + m[0][3],
            m[1][0] * p.x + m[1][1] * p.y + m[1][2] * p.z + m[1][3],
            m[2][0] * p.x + m[2][1] * p.y + m[2][2] * p.z + m[2][3]};
}

[[nodiscard]] double determinant(const Affine& m) noexcept {
    return m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1]) - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0]) +
           m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
}

// --- the reader -------------------------------------------------------------------

class Reader {
public:
    Reader(std::string file, std::filesystem::path base_dir) : file_(std::move(file)), base_dir_(std::move(base_dir)) {}

    [[nodiscard]] Result<std::vector<MeshObject>> read(std::span<const std::byte> bytes);

private:
    struct Accessor {
        std::string where;  // "...: mesh 0 primitive 1 indices accessor 2"
        uint64_t index = 0;
        uint64_t count = 0;
        uint64_t component_type = 0;
        std::optional<uint64_t> view;
        uint64_t byte_offset = 0;
    };
    struct Data {
        const std::byte* base = nullptr;  // null: no bufferView, every element is zero
        uint64_t stride = 0;
    };
    struct Primitive {
        Accessor position;
        std::optional<Accessor> indices;
    };
    struct Buffer {
        bool loaded = false;
        std::vector<std::byte> owned;
        std::span<const std::byte> bytes;
    };

    [[nodiscard]] std::unexpected<Error> fail(Code code, std::string_view detail) const {
        std::string context = file_;
        context += ": ";
        context += detail;
        return std::unexpected(Error{code, std::move(context)});
    }

    [[nodiscard]] static std::string at(const std::string& where, std::string_view what) {
        return where.empty() ? std::string(what) : where + ": " + std::string(what);
    }

    [[nodiscard]] Result<std::optional<uint64_t>> opt_uint(const Json& object, const char* key,
                                                           const std::string& where) const {
        const auto it = object.find(key);
        if (it == object.end()) return std::optional<uint64_t>{};
        if (!it->is_number_unsigned()) {
            return fail(Code::invalid_argument, at(where, std::string("'") + key + "' is not a non-negative integer"));
        }
        return std::optional<uint64_t>{it->get<uint64_t>()};
    }

    [[nodiscard]] Result<uint64_t> need_uint(const Json& object, const char* key, const std::string& where) const {
        auto v = opt_uint(object, key, where);
        if (!v) return std::unexpected(v.error());
        if (!*v) return fail(Code::invalid_argument, at(where, std::string("'") + key + "' is missing"));
        return **v;
    }

    // root[array][index], which must exist and be an object.
    [[nodiscard]] Result<const Json*> element(const char* array, const char* noun, uint64_t index,
                                              const std::string& where) const {
        const auto it = root_.find(array);
        const uint64_t size = it != root_.end() && it->is_array() ? it->size() : 0;
        if (index >= size) {
            return fail(Code::invalid_argument, at(where, std::string(noun) + " " + num(index) + " does not exist (" +
                                                              num(size) + " " + array + ")"));
        }
        const Json& e = (*it)[static_cast<std::size_t>(index)];
        if (!e.is_object())
            return fail(Code::invalid_argument, at(where, std::string(noun) + " " + num(index) + " is not an object"));
        return &e;
    }

    [[nodiscard]] Result<std::span<const std::byte>> split_glb(std::span<const std::byte> bytes);
    [[nodiscard]] Result<void> check_asset() const;
    [[nodiscard]] Result<std::vector<uint64_t>> node_list(const Json& object, const char* key, const std::string& where,
                                                          uint64_t node_count) const;
    [[nodiscard]] Result<Affine> local_transform(const Json& node, uint64_t index) const;
    [[nodiscard]] Result<MeshObject> object(const Json& node, uint64_t node_index, uint64_t mesh_index,
                                            const Affine& world);
    [[nodiscard]] Result<Accessor> describe(uint64_t index, const std::string& role_where, bool position) const;
    [[nodiscard]] Result<Data> data(const Accessor& a, uint64_t element_size);
    [[nodiscard]] Result<std::pair<std::span<const std::byte>, uint64_t>> buffer(uint64_t index,
                                                                                 const std::string& where);

    std::string file_;
    std::filesystem::path base_dir_;
    Json root_;
    bool glb_ = false;
    std::optional<std::span<const std::byte>> bin_;
    std::vector<Buffer> buffers_;
};

Result<std::span<const std::byte>> Reader::split_glb(std::span<const std::byte> bytes) {
    if (bytes.size() < 12)
        return fail(Code::invalid_argument, "GLB: " + num(bytes.size()) + " bytes, short of its 12-byte header");
    const uint32_t version = le32(bytes.data() + 4);
    if (version != 2) return fail(Code::unavailable, "GLB version " + num(version) + "; this build reads version 2");
    const uint32_t length = le32(bytes.data() + 8);
    if (length != bytes.size()) {
        return fail(Code::invalid_argument,
                    "GLB: the header says " + num(length) + " bytes, but there are " + num(bytes.size()));
    }
    if (bytes.size() < 20 || le32(bytes.data() + 16) != kChunkJson) {
        return fail(Code::invalid_argument, "GLB: the first chunk is not JSON");
    }
    const uint64_t json_length = le32(bytes.data() + 12);
    if (json_length > bytes.size() - 20) {
        return fail(Code::invalid_argument, "GLB: the JSON chunk (" + num(json_length) + " bytes) runs past the file");
    }
    const std::span<const std::byte> json = bytes.subspan(20, static_cast<std::size_t>(json_length));
    const std::size_t next = 20 + static_cast<std::size_t>(json_length);
    if (bytes.size() - next >= 8) {
        const uint64_t chunk_length = le32(bytes.data() + next);
        if (chunk_length > bytes.size() - next - 8) {
            return fail(Code::invalid_argument, "GLB: chunk 1 (" + num(chunk_length) + " bytes) runs past the file");
        }
        if (le32(bytes.data() + next + 4) == kChunkBin) {
            bin_ = bytes.subspan(next + 8, static_cast<std::size_t>(chunk_length));
        }
    }
    return json;
}

Result<void> Reader::check_asset() const {
    const auto asset = root_.find("asset");
    if (asset == root_.end() || !asset->is_object()) return fail(Code::invalid_argument, "no 'asset' object");
    const auto version = asset->find("version");
    if (version == asset->end() || !version->is_string()) {
        return fail(Code::invalid_argument, "asset: 'version' is missing or not text");
    }
    const std::string v = version->get<std::string>();
    if (!v.starts_with("2.")) return fail(Code::unavailable, "glTF version '" + v + "'; this build reads 2.x");
    const auto min_version = asset->find("minVersion");
    if (min_version != asset->end()) {
        if (!min_version->is_string()) return fail(Code::invalid_argument, "asset: 'minVersion' is not text");
        const std::string m = min_version->get<std::string>();
        if (m != "2.0") return fail(Code::unavailable, "minVersion '" + m + "'; this build reads glTF 2.0");
    }
    return {};
}

Result<std::vector<uint64_t>> Reader::node_list(const Json& object, const char* key, const std::string& where,
                                                uint64_t node_count) const {
    std::vector<uint64_t> out;
    const auto it = object.find(key);
    if (it == object.end()) return out;
    if (!it->is_array()) return fail(Code::invalid_argument, at(where, std::string("'") + key + "' is not an array"));
    for (const Json& n : *it) {
        if (!n.is_number_unsigned()) {
            return fail(Code::invalid_argument,
                        at(where, std::string("'") + key + "' holds an entry that is not a node index"));
        }
        const uint64_t i = n.get<uint64_t>();
        if (i >= node_count) {
            return fail(Code::invalid_argument,
                        at(where, "node " + num(i) + " does not exist (" + num(node_count) + " nodes)"));
        }
        out.push_back(i);
    }
    return out;
}

Result<Affine> Reader::local_transform(const Json& node, uint64_t index) const {
    const std::string where = "node " + num(index);
    // `key` as exactly `n` finite numbers, into `out`; absent leaves `out`.
    const auto numbers = [&](const char* key, std::size_t n, double* out) -> Result<bool> {
        const auto it = node.find(key);
        if (it == node.end()) return false;
        if (!it->is_array() || it->size() != n) {
            return fail(Code::invalid_argument, where + ": '" + key + "' is not " + num(n) + " numbers");
        }
        for (std::size_t i = 0; i < n; ++i) {
            const Json& e = (*it)[i];
            if (!e.is_number() || !std::isfinite(e.get<double>())) {
                return fail(Code::invalid_argument, where + ": '" + key + "' is not " + num(n) + " finite numbers");
            }
            out[i] = e.get<double>();
        }
        return true;
    };

    std::array<double, 16> m{};
    auto has_matrix = numbers("matrix", 16, m.data());
    if (!has_matrix) return std::unexpected(has_matrix.error());
    std::array<double, 3> t{0.0, 0.0, 0.0};
    std::array<double, 4> q{0.0, 0.0, 0.0, 1.0};
    std::array<double, 3> s{1.0, 1.0, 1.0};
    auto has_t = numbers("translation", 3, t.data());
    if (!has_t) return std::unexpected(has_t.error());
    auto has_r = numbers("rotation", 4, q.data());
    if (!has_r) return std::unexpected(has_r.error());
    auto has_s = numbers("scale", 3, s.data());
    if (!has_s) return std::unexpected(has_s.error());

    if (*has_matrix) {
        if (*has_t || *has_r || *has_s) {
            return fail(Code::invalid_argument, where + " has both a matrix and translation, rotation or scale");
        }
        // Column-major: element (row, col) is m[col * 4 + row].
        if (m[3] != 0.0 || m[7] != 0.0 || m[11] != 0.0 || m[15] != 1.0) {
            return fail(Code::invalid_argument, where + ": the matrix's last row is not 0 0 0 1");
        }
        Affine a{};
        for (std::size_t row = 0; row < 3; ++row) {
            for (std::size_t col = 0; col < 4; ++col) a[row][col] = m[col * 4 + row];
        }
        return a;
    }

    // T * R * S. R from q / |q|: s2 = 2 / |q|^2, the standard expansion.
    const double x = q[0];
    const double y = q[1];
    const double z = q[2];
    const double w = q[3];
    const double n = x * x + y * y + z * z + w * w;
    if (!(n > 0.0)) return fail(Code::invalid_argument, where + ": the rotation quaternion is zero");
    const double s2 = 2.0 / n;
    const double r[3][3] = {
        {1.0 - s2 * (y * y + z * z), s2 * (x * y - z * w), s2 * (x * z + y * w)},
        {s2 * (x * y + z * w), 1.0 - s2 * (x * x + z * z), s2 * (y * z - x * w)},
        {s2 * (x * z - y * w), s2 * (y * z + x * w), 1.0 - s2 * (x * x + y * y)},
    };
    Affine a{};
    for (std::size_t row = 0; row < 3; ++row) {
        for (std::size_t col = 0; col < 3; ++col) a[row][col] = r[row][col] * s[col];
        a[row][3] = t[row];
    }
    return a;
}

Result<std::vector<MeshObject>> Reader::read(std::span<const std::byte> bytes) {
    std::span<const std::byte> json = bytes;
    if (bytes.size() >= 4 && le32(bytes.data()) == kGlbMagic) {
        glb_ = true;
        auto chunk = split_glb(bytes);
        if (!chunk) return std::unexpected(chunk.error());
        json = *chunk;
    }
    const char* text = reinterpret_cast<const char*>(json.data());
    root_ = Json::parse(text, text + json.size(), nullptr, false);
    if (root_.is_discarded()) return fail(Code::invalid_argument, "not JSON");
    if (!root_.is_object()) return fail(Code::invalid_argument, "the document is not a JSON object");
    if (auto ok = check_asset(); !ok) return std::unexpected(ok.error());

    if (const auto it = root_.find("extensionsRequired"); it != root_.end()) {
        if (!it->is_array()) return fail(Code::invalid_argument, "'extensionsRequired' is not an array");
        if (!it->empty()) {
            std::string names;
            for (const Json& e : *it) {
                if (!e.is_string())
                    return fail(Code::invalid_argument, "'extensionsRequired' holds an entry that is not text");
                names += names.empty() ? "'" : ", '";
                names += e.get<std::string>();
                names += "'";
            }
            return fail(Code::unavailable,
                        std::string(it->size() == 1 ? "requires the extension " : "requires the extensions ") + names +
                            ", which this build does not read (export without it)");
        }
    }

    if (const auto it = root_.find("buffers"); it != root_.end()) {
        if (!it->is_array()) return fail(Code::invalid_argument, "'buffers' is not an array");
        buffers_.resize(it->size());
    }
    if (const auto it = root_.find("nodes"); it != root_.end() && !it->is_array()) {
        return fail(Code::invalid_argument, "'nodes' is not an array");
    }
    const auto nodes_it = root_.find("nodes");
    const uint64_t node_count = nodes_it != root_.end() ? nodes_it->size() : 0;

    // The default scene: `scene`, else scenes[0].
    auto scene_field = opt_uint(root_, "scene", "");
    if (!scene_field) return std::unexpected(scene_field.error());
    const auto scenes_it = root_.find("scenes");
    if (!scene_field->has_value() && (scenes_it == root_.end() || !scenes_it->is_array() || scenes_it->empty())) {
        return fail(Code::invalid_argument, "no scene to read ('scenes' is missing or empty)");
    }
    const uint64_t scene_index = scene_field->value_or(0);
    auto scene = element("scenes", "scene", scene_index, "");
    if (!scene) return std::unexpected(scene.error());
    auto roots = node_list(**scene, "nodes", "scene " + num(scene_index), node_count);
    if (!roots) return std::unexpected(roots.error());

    // Depth first, pre-order: roots in order, each node's children in order.
    struct Pending {
        uint64_t node;
        Affine parent;
    };
    std::vector<Pending> stack;
    for (auto r = roots->rbegin(); r != roots->rend(); ++r) stack.push_back({*r, kIdentity});
    std::vector<uint8_t> seen(static_cast<std::size_t>(node_count), 0);
    std::vector<MeshObject> objects;
    while (!stack.empty()) {
        const Pending p = stack.back();
        stack.pop_back();
        const std::string where = "node " + num(p.node);
        if (seen[static_cast<std::size_t>(p.node)] != 0) {
            return fail(Code::invalid_argument,
                        where + " is reached twice (a node has one parent, and the graph no cycle)");
        }
        seen[static_cast<std::size_t>(p.node)] = 1;
        const Json& node = (*nodes_it)[static_cast<std::size_t>(p.node)];
        if (!node.is_object()) return fail(Code::invalid_argument, where + " is not an object");
        auto children = node_list(node, "children", where, node_count);
        if (!children) return std::unexpected(children.error());
        auto local = local_transform(node, p.node);
        if (!local) return std::unexpected(local.error());
        const Affine world = compose(p.parent, *local);
        auto mesh = opt_uint(node, "mesh", where);
        if (!mesh) return std::unexpected(mesh.error());
        if (mesh->has_value()) {
            auto o = object(node, p.node, **mesh, world);
            if (!o) return std::unexpected(o.error());
            objects.push_back(std::move(*o));
        }
        for (auto c = children->rbegin(); c != children->rend(); ++c) stack.push_back({*c, world});
    }
    if (objects.empty()) return fail(Code::invalid_argument, "no node in scene " + num(scene_index) + " holds a mesh");
    return objects;
}

Result<MeshObject> Reader::object(const Json& node, uint64_t node_index, uint64_t mesh_index, const Affine& world) {
    auto mesh_json = element("meshes", "mesh", mesh_index, "node " + num(node_index));
    if (!mesh_json) return std::unexpected(mesh_json.error());
    const Json& mesh = **mesh_json;
    std::string name = name_of(node);
    if (name.empty()) name = name_of(mesh);
    if (name.empty()) name = "node[" + num(node_index) + "]";
    const std::string prefix = "object '" + name + "'";

    if (node.contains("skin")) {
        return fail(Code::unavailable,
                    prefix + ": node " + num(node_index) + " is skinned (skin); this build reads static meshes only");
    }
    if (const auto ext = node.find("extensions");
        ext != node.end() && ext->is_object() && ext->contains("EXT_mesh_gpu_instancing")) {
        return fail(Code::unavailable, prefix + ": node " + num(node_index) +
                                           " uses EXT_mesh_gpu_instancing, which this build does not read");
    }
    const auto prims = mesh.find("primitives");
    if (prims == mesh.end() || !prims->is_array() || prims->empty()) {
        return fail(Code::invalid_argument, prefix + ": mesh " + num(mesh_index) + " has no primitives");
    }

    // Pass 1: each primitive's mode, targets and accessor kinds; the counts.
    std::vector<Primitive> primitives;
    uint64_t triangles = 0;
    uint64_t vertices = 0;
    for (std::size_t k = 0; k < prims->size(); ++k) {
        const std::string where = prefix + ": mesh " + num(mesh_index) + " primitive " + num(k);
        const Json& prim = (*prims)[k];
        if (!prim.is_object()) return fail(Code::invalid_argument, where + " is not an object");
        auto mode = opt_uint(prim, "mode", where);
        if (!mode) return std::unexpected(mode.error());
        const uint64_t m = mode->value_or(kTriangles);
        if (m != kTriangles) {
            if (mode_name(m).empty())
                return fail(Code::invalid_argument, where + ": mode " + num(m) + " is not a glTF mode");
            return fail(Code::unavailable, where + ": mode " + num(m) + " (" + std::string(mode_name(m)) +
                                               "); this build reads TRIANGLES (mode 4) only (export triangles)");
        }
        if (const auto targets = prim.find("targets");
            targets != prim.end() && (!targets->is_array() || !targets->empty())) {
            return fail(Code::unavailable, where + " has morph targets, which this build does not read");
        }
        const auto attributes = prim.find("attributes");
        if (attributes == prim.end() || !attributes->is_object()) {
            return fail(Code::invalid_argument, where + ": 'attributes' is missing or not an object");
        }
        auto position_index = need_uint(*attributes, "POSITION", where);
        if (!position_index) return std::unexpected(position_index.error());
        auto position = describe(*position_index, where + " POSITION", true);
        if (!position) return std::unexpected(position.error());
        auto indices_index = opt_uint(prim, "indices", where);
        if (!indices_index) return std::unexpected(indices_index.error());
        std::optional<Accessor> indices;
        if (indices_index->has_value()) {
            auto a = describe(**indices_index, where + " indices", false);
            if (!a) return std::unexpected(a.error());
            indices = std::move(*a);
        }
        const uint64_t n = indices ? indices->count : position->count;
        if (n % 3 != 0) {
            return fail(
                Code::invalid_argument,
                where + ": " + num(n) + (indices ? " indices" : " vertices and no indices") + ", not a multiple of 3");
        }
        triangles = saturating_add(triangles, n / 3);
        vertices = saturating_add(vertices, position->count);
        primitives.push_back(Primitive{std::move(*position), std::move(indices)});
    }
    if (triangles > kMaxMeshTriangles) {
        return fail(Code::capacity_exceeded, prefix + ": " + num(triangles) + " triangles, more than the " +
                                                 num(kMaxMeshTriangles) + " a mesh may hold");
    }
    if (vertices > std::numeric_limits<uint32_t>::max()) {
        return fail(Code::capacity_exceeded, prefix + ": " + num(vertices) + " vertices, more than the " +
                                                 num(std::numeric_limits<uint32_t>::max()) + " an object may index");
    }

    // Pass 2: the data, transformed into metres in the object's frame.
    std::vector<glm::dvec3> positions;
    std::vector<uint32_t> indices;
    for (const Primitive& prim : primitives) {
        auto pd = data(prim.position, 12);
        if (!pd) return std::unexpected(pd.error());
        std::optional<Data> id;
        uint64_t index_size = 0;
        if (prim.indices) {
            index_size = prim.indices->component_type == kUnsignedByte    ? 1
                         : prim.indices->component_type == kUnsignedShort ? 2
                                                                          : 4;
            auto d = data(*prim.indices, index_size);
            if (!d) return std::unexpected(d.error());
            id = *d;
        }
        const uint32_t base = static_cast<uint32_t>(positions.size());
        const std::size_t count = static_cast<std::size_t>(prim.position.count);
        positions.reserve(positions.size() + count);
        for (std::size_t v = 0; v < count; ++v) {
            glm::dvec3 p(0.0);
            if (pd->base != nullptr) {
                const std::byte* e = pd->base + v * pd->stride;
                p = glm::dvec3(static_cast<double>(std::bit_cast<float>(le32(e))),
                               static_cast<double>(std::bit_cast<float>(le32(e + 4))),
                               static_cast<double>(std::bit_cast<float>(le32(e + 8))));
            }
            positions.push_back(transform_point(world, p));
        }
        if (!prim.indices) {
            for (std::size_t v = 0; v < count; ++v) indices.push_back(base + static_cast<uint32_t>(v));
            continue;
        }
        const std::size_t n = static_cast<std::size_t>(prim.indices->count);
        indices.reserve(indices.size() + n);
        for (std::size_t k = 0; k < n; ++k) {
            uint32_t i = 0;
            if (id->base != nullptr) {
                const std::byte* e = id->base + k * id->stride;
                i = index_size == 1   ? std::to_integer<uint32_t>(e[0])
                    : index_size == 2 ? std::to_integer<uint32_t>(e[0]) | (std::to_integer<uint32_t>(e[1]) << 8)
                                      : le32(e);
            }
            if (i >= prim.position.count) {
                return fail(Code::invalid_argument, prim.indices->where + ": index " + num(i) + " (at " + num(k) +
                                                        ") is out of range: POSITION accessor " +
                                                        num(prim.position.index) + " has " + num(prim.position.count) +
                                                        " vertices");
            }
            indices.push_back(base + i);
        }
    }

    // A mirroring transform turns every triangle inside out; reverse them.
    if (determinant(world) < 0.0) {
        for (std::size_t t = 0; t + 2 < indices.size(); t += 3) std::swap(indices[t + 1], indices[t + 2]);
    }

    const MeshInput input = mesh_import::weld_exact(positions, indices, MeshUnit::metre);
    auto built = make_mesh(input, file_ + ": " + prefix);
    if (!built) return std::unexpected(built.error());
    return MeshObject{std::move(name), std::move(*built)};
}

Result<Reader::Accessor> Reader::describe(uint64_t index, const std::string& role_where, bool position) const {
    auto json = element("accessors", "accessor", index, role_where);
    if (!json) return std::unexpected(json.error());
    const Json& a = **json;
    Accessor out;
    out.where = role_where + " accessor " + num(index);
    out.index = index;
    if (a.contains("sparse")) {
        return fail(Code::unavailable, out.where + " is sparse; this build does not read sparse accessors");
    }
    auto count = need_uint(a, "count", out.where);
    if (!count) return std::unexpected(count.error());
    if (*count == 0) return fail(Code::invalid_argument, out.where + ": 'count' is 0");
    out.count = *count;
    auto type = need_uint(a, "componentType", out.where);
    if (!type) return std::unexpected(type.error());
    out.component_type = *type;
    const auto shape = a.find("type");
    if (shape == a.end() || !shape->is_string()) {
        return fail(Code::invalid_argument, out.where + ": 'type' is missing or not text");
    }
    const std::string s = shape->get<std::string>();
    const auto norm = a.find("normalized");
    if (norm != a.end() && !norm->is_boolean()) {
        return fail(Code::invalid_argument, out.where + ": 'normalized' is not true or false");
    }
    const bool normalized = norm != a.end() && norm->get<bool>();
    const std::string kind = s + " of componentType " + num(out.component_type) + (normalized ? ", normalized" : "");
    if (position) {
        if (s != "VEC3" || out.component_type != kFloat || normalized) {
            return fail(Code::unavailable, out.where + " is " + kind +
                                               "; this build reads POSITION as VEC3 FLOAT (no KHR_mesh_quantization)");
        }
    } else if (s != "SCALAR" || normalized ||
               (out.component_type != kUnsignedByte && out.component_type != kUnsignedShort &&
                out.component_type != kUnsignedInt)) {
        return fail(
            Code::invalid_argument,
            out.where + " is " + kind + "; glTF indices are SCALAR UNSIGNED_BYTE, UNSIGNED_SHORT or UNSIGNED_INT");
    }
    auto view = opt_uint(a, "bufferView", out.where);
    if (!view) return std::unexpected(view.error());
    out.view = *view;
    auto offset = opt_uint(a, "byteOffset", out.where);
    if (!offset) return std::unexpected(offset.error());
    out.byte_offset = offset->value_or(0);
    return out;
}

Result<Reader::Data> Reader::data(const Accessor& a, uint64_t element_size) {
    if (!a.view) return Data{nullptr, element_size};
    auto json = element("bufferViews", "bufferView", *a.view, a.where);
    if (!json) return std::unexpected(json.error());
    const Json& v = **json;
    const std::string where = a.where + ": bufferView " + num(*a.view);
    auto buffer_index = need_uint(v, "buffer", where);
    if (!buffer_index) return std::unexpected(buffer_index.error());
    auto view_offset = opt_uint(v, "byteOffset", where);
    if (!view_offset) return std::unexpected(view_offset.error());
    auto view_length = need_uint(v, "byteLength", where);
    if (!view_length) return std::unexpected(view_length.error());
    auto stride = opt_uint(v, "byteStride", where);
    if (!stride) return std::unexpected(stride.error());
    const uint64_t step = stride->value_or(element_size);
    if (step < element_size) {
        return fail(Code::invalid_argument,
                    where + ": byteStride " + num(step) + " is less than an element's " + num(element_size) + " bytes");
    }
    if (!fits(a.byte_offset, a.count, step, element_size, *view_length)) {
        return fail(Code::invalid_argument, a.where + ": " + num(a.count) + " elements from byteOffset " +
                                                num(a.byte_offset) + ", " + num(step) +
                                                " bytes apart, run past bufferView " + num(*a.view) + "'s byteLength " +
                                                num(*view_length));
    }
    auto buf = buffer(*buffer_index, where);
    if (!buf) return std::unexpected(buf.error());
    const uint64_t declared = buf->second;
    const uint64_t offset = view_offset->value_or(0);
    if (offset > declared || *view_length > declared - offset) {
        return fail(Code::invalid_argument, where + ": " + num(*view_length) + " bytes from byteOffset " + num(offset) +
                                                " run past buffer " + num(*buffer_index) + "'s byteLength " +
                                                num(declared));
    }
    return Data{buf->first.data() + offset + a.byte_offset, step};
}

Result<std::pair<std::span<const std::byte>, uint64_t>> Reader::buffer(uint64_t index, const std::string& where) {
    auto json = element("buffers", "buffer", index, where);
    if (!json) return std::unexpected(json.error());
    const Json& b = **json;
    const std::string bwhere = where + ": buffer " + num(index);
    auto declared = need_uint(b, "byteLength", bwhere);
    if (!declared) return std::unexpected(declared.error());
    Buffer& slot = buffers_[static_cast<std::size_t>(index)];
    if (slot.loaded) return std::pair{slot.bytes, *declared};

    const auto uri = b.find("uri");
    if (uri == b.end()) {
        if (!glb_ || index != 0 || !bin_) {
            return fail(Code::invalid_argument,
                        bwhere + " has no uri" + (glb_ && index == 0 ? ", and the GLB has no BIN chunk" : ""));
        }
        if (bin_->size() < *declared || bin_->size() - *declared > 3) {
            return fail(Code::invalid_argument, bwhere + ": the GLB's BIN chunk holds " + num(bin_->size()) +
                                                    " bytes, but its byteLength is " + num(*declared));
        }
        slot.bytes = *bin_;
        slot.loaded = true;
        return std::pair{slot.bytes, *declared};
    }
    if (!uri->is_string()) return fail(Code::invalid_argument, bwhere + ": 'uri' is not text");
    const std::string u = uri->get<std::string>();
    if (u.starts_with("data:")) {
        const std::size_t comma = u.find(',');
        if (comma == std::string::npos || !std::string_view(u).substr(0, comma).ends_with(";base64")) {
            return fail(Code::invalid_argument, bwhere + ": a data: URI that is not base64");
        }
        auto decoded = decode_base64(std::string_view(u).substr(comma + 1));
        if (!decoded) return fail(Code::invalid_argument, bwhere + ": the data: URI's base64 is malformed");
        slot.owned = std::move(*decoded);
    } else if (const std::string_view scheme = uri_scheme(u); !scheme.empty()) {
        return fail(Code::unavailable, bwhere + ": the URI scheme '" + std::string(scheme) +
                                           "' is not read (only data: URIs and relative files)");
    } else {
        if (base_dir_.empty()) {
            return fail(
                Code::invalid_argument,
                bwhere + " names the file '" + u + "', and no base directory was given (use read_gltf, or a base_dir)");
        }
        const auto decoded = percent_decode(u);
        if (!decoded) return fail(Code::invalid_argument, bwhere + ": the URI '" + u + "' has a malformed %-escape");
        std::u8string utf8;
        for (const char c : *decoded) utf8 += static_cast<char8_t>(c);
        const std::filesystem::path path = base_dir_ / std::filesystem::path(utf8);
        auto bytes = read_file(path);
        if (!bytes) return fail(Code::io_error, bwhere + ": cannot read '" + path_text(path) + "'");
        slot.owned = std::move(*bytes);
    }
    if (slot.owned.size() != *declared) {
        return fail(Code::invalid_argument,
                    bwhere + ": " + num(slot.owned.size()) + " bytes, but its byteLength is " + num(*declared));
    }
    slot.bytes = slot.owned;
    slot.loaded = true;
    return std::pair{slot.bytes, *declared};
}

// Runs a Reader, turning the exceptions a JSON library or an allocation can
// throw into Results: none crosses a Spade API.
[[nodiscard]] Result<std::vector<MeshObject>> run(const std::string& file, const std::filesystem::path& base_dir,
                                                  std::span<const std::byte> bytes) {
    try {
        Reader reader(file, base_dir);
        return reader.read(bytes);
    } catch (const nlohmann::json::exception& e) {
        return std::unexpected(Error{Code::invalid_argument, file + ": " + e.what()});
    } catch (const std::bad_alloc&) {
        return std::unexpected(Error{Code::capacity_exceeded, file + ": out of memory reading it"});
    } catch (const std::exception& e) {
        return std::unexpected(Error{Code::internal, file + ": unexpected failure: " + e.what()});
    }
}

}  // namespace

Result<std::vector<MeshObject>> read_gltf(const std::filesystem::path& path) {
    const std::string file = path_text(path);
    std::optional<std::vector<std::byte>> bytes;
    try {
        bytes = read_file(path);
    } catch (const std::exception& e) {
        return std::unexpected(Error{Code::io_error, file + ": cannot read it: " + e.what()});
    }
    if (!bytes) return std::unexpected(Error{Code::io_error, file + ": cannot read it"});
    const std::filesystem::path parent = path.parent_path();
    return run(file, parent.empty() ? std::filesystem::path(".") : parent, *bytes);
}

Result<std::vector<MeshObject>> read_gltf_bytes(std::span<const std::byte> bytes,
                                                const std::filesystem::path& base_dir) {
    return run("(glTF bytes)", base_dir, bytes);
}

}  // namespace spade
