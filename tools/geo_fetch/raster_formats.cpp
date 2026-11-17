#include "raster_formats.hpp"

#include <bit>
#include <cstddef>
#include <string>

namespace spade::geofetch {

namespace {

[[nodiscard]] std::unexpected<Error> bad(const std::string& why) {
    return std::unexpected(Error{Code::schema_mismatch, why});
}

// Bounds-checked reads from the TIFF's bytes, in its byte order.
struct Reader {
    std::string_view bytes;
    bool big = false;

    [[nodiscard]] bool has(std::size_t at, std::size_t n) const { return at <= bytes.size() && n <= bytes.size() - at; }
    [[nodiscard]] uint16_t u16(std::size_t at) const {
        const auto b0 = static_cast<uint8_t>(bytes[at]);
        const auto b1 = static_cast<uint8_t>(bytes[at + 1]);
        return big ? static_cast<uint16_t>((b0 << 8) | b1) : static_cast<uint16_t>((b1 << 8) | b0);
    }
    [[nodiscard]] uint32_t u32(std::size_t at) const {
        uint32_t v = 0;
        for (std::size_t i = 0; i < 4; ++i) {
            const auto b = static_cast<uint8_t>(bytes[at + (big ? i : 3 - i)]);
            v = (v << 8) | b;
        }
        return v;
    }
};

struct Entry {
    uint16_t type = 0;
    uint32_t count = 0;
    std::size_t value_at = 0;  // where the values start: the entry's own value field, or the offset it holds
};

// The values of one tag as unsigned integers (SHORT or LONG), or false when the tag is another type or runs off the data.
[[nodiscard]] bool values_of(const Reader& r, const Entry& e, std::vector<uint32_t>& out) {
    if (e.type != 3 && e.type != 4) return false;
    const std::size_t size = e.type == 3 ? 2 : 4;
    if (!r.has(e.value_at, static_cast<std::size_t>(e.count) * size)) return false;
    out.clear();
    for (uint32_t i = 0; i < e.count; ++i) {
        const std::size_t at = e.value_at + i * size;
        out.push_back(e.type == 3 ? r.u16(at) : r.u32(at));
    }
    return true;
}

}  // namespace

Result<TiffGrid> read_tiff_f32(std::string_view bytes) {
    if (bytes.size() < 8 || !((bytes[0] == 'I' && bytes[1] == 'I') || (bytes[0] == 'M' && bytes[1] == 'M'))) {
        return bad("3dep: the answer is not a TIFF");
    }
    Reader r{bytes, bytes[0] == 'M'};
    if (r.u16(2) != 42) return bad("3dep: the TIFF is not a classic TIFF (BigTIFF is not supported)");
    const std::size_t ifd = r.u32(4);
    if (!r.has(ifd, 2)) return bad("3dep: the TIFF's directory is past the end of the data");
    const std::size_t entries = r.u16(ifd);
    if (!r.has(ifd + 2, entries * 12)) return bad("3dep: the TIFF's directory is cut short");

    const auto find = [&](uint16_t tag, Entry& out) {
        for (std::size_t i = 0; i < entries; ++i) {
            const std::size_t at = ifd + 2 + i * 12;
            if (r.u16(at) != tag) continue;
            out.type = r.u16(at + 2);
            out.count = r.u32(at + 4);
            const std::size_t size = out.type == 3 ? 2 : 4;
            out.value_at = static_cast<std::size_t>(out.count) * size <= 4 ? at + 8 : r.u32(at + 8);
            return true;
        }
        return false;
    };
    // One number of a tag, or `fallback` when the tag is absent; false in `ok` when it is present but unreadable.
    bool ok = true;
    const auto number = [&](uint16_t tag, uint32_t fallback) {
        Entry e;
        if (!find(tag, e)) return fallback;
        std::vector<uint32_t> v;
        if (!values_of(r, e, v) || v.empty()) {
            ok = false;
            return fallback;
        }
        return v[0];
    };

    const uint32_t width = number(256, 0);
    const uint32_t height = number(257, 0);
    const uint32_t bits = number(258, 1);
    const uint32_t compression = number(259, 1);
    const uint32_t bands = number(277, 1);
    const uint32_t format = number(339, 1);
    const uint32_t rows_per_strip = number(278, height);
    if (!ok || width == 0 || height == 0) return bad("3dep: the TIFF has no usable width and height");
    if (compression != 1) {
        return bad("3dep: the TIFF is compressed (code " + std::to_string(compression) + "); only uncompressed is read");
    }
    if (bands != 1) return bad("3dep: the TIFF has " + std::to_string(bands) + " bands, one is expected");
    if (bits != 32 || format != 3) {
        return bad("3dep: the TIFF is not float32 (bits " + std::to_string(bits) + ", sample format " + std::to_string(format) + ")");
    }
    if (static_cast<uint64_t>(width) * height > 16u * 1024u * 1024u) return bad("3dep: the TIFF is larger than 16 million samples");

    TiffGrid g;
    g.width = width;
    g.height = height;

    // Tiles (tags 322 to 325): what an ArcGIS ImageServer sends. Row-major, each tile tile_w x tile_h samples in full (the edge tiles
    // are padded past the image), so each one is cropped to the image as it is copied.
    Entry probe;
    if (find(322, probe) || find(323, probe) || find(324, probe) || find(325, probe)) {
        const uint32_t tile_w = number(322, 0);
        const uint32_t tile_h = number(323, 0);
        constexpr uint32_t kMaxTileSide = 4096;
        if (!ok || tile_w == 0 || tile_h == 0 || tile_w > kMaxTileSide || tile_h > kMaxTileSide) {
            return bad("3dep: the TIFF's tile size is missing or not usable (tile " + std::to_string(tile_w) + " x " + std::to_string(tile_h) + ")");
        }
        const uint64_t across = (static_cast<uint64_t>(width) + tile_w - 1) / tile_w;
        const uint64_t down = (static_cast<uint64_t>(height) + tile_h - 1) / tile_h;
        Entry to, tc;
        std::vector<uint32_t> tile_offsets, tile_counts;
        if (!find(324, to) || !values_of(r, to, tile_offsets) || !find(325, tc) || !values_of(r, tc, tile_counts)) {
            return bad("3dep: the TIFF has no readable tile offsets and byte counts");
        }
        if (tile_offsets.size() != across * down || tile_counts.size() != across * down) {
            return bad("3dep: the TIFF has " + std::to_string(tile_offsets.size()) + " tiles, " + std::to_string(across * down) + " expected for " +
                       std::to_string(width) + " x " + std::to_string(height) + " in tiles of " + std::to_string(tile_w) + " x " + std::to_string(tile_h));
        }
        const uint64_t tile_bytes = static_cast<uint64_t>(tile_w) * tile_h * 4;
        g.samples.assign(static_cast<std::size_t>(width) * height, 0.0f);
        for (uint64_t t = 0; t < across * down; ++t) {
            if (tile_counts[t] < tile_bytes || !r.has(tile_offsets[t], tile_bytes)) {
                return bad("3dep: a tile of the TIFF is short or past the end of the data");
            }
            const uint64_t x0 = (t % across) * tile_w;
            const uint64_t y0 = (t / across) * tile_h;
            for (uint64_t row = 0; row < tile_h && y0 + row < height; ++row) {
                for (uint64_t col = 0; col < tile_w && x0 + col < width; ++col) {
                    g.samples[(y0 + row) * width + x0 + col] =
                        std::bit_cast<float>(r.u32(tile_offsets[t] + 4 * (row * tile_w + col)));
                }
            }
        }
        return g;
    }

    Entry eo, ec;
    std::vector<uint32_t> offsets, counts;
    if (!find(273, eo) || !values_of(r, eo, offsets) || !find(279, ec) || !values_of(r, ec, counts) || offsets.empty() ||
        offsets.size() != counts.size()) {
        return bad("3dep: the TIFF has no readable strip offsets and byte counts");
    }
    (void)rows_per_strip;  // the byte counts say how much each strip holds

    g.samples.reserve(static_cast<std::size_t>(width) * height);
    for (std::size_t s = 0; s < offsets.size(); ++s) {
        if (counts[s] % 4 != 0 || !r.has(offsets[s], counts[s])) return bad("3dep: a strip of the TIFF is past the end of the data");
        for (std::size_t i = 0; i < counts[s] / 4; ++i) g.samples.push_back(std::bit_cast<float>(r.u32(offsets[s] + 4 * i)));
    }
    if (g.samples.size() != static_cast<std::size_t>(width) * height) {
        return bad("3dep: the TIFF's strips hold " + std::to_string(g.samples.size()) + " samples, " +
                   std::to_string(static_cast<std::size_t>(width) * height) + " expected");
    }
    return g;
}

Result<PngHeader> read_png_header(std::string_view bytes) {
    static constexpr std::string_view kSignature("\x89PNG\r\n\x1a\n", 8);
    if (bytes.size() < 8 + 8 + 13 || bytes.substr(0, 8) != kSignature) return bad("naip: the answer is not a PNG");
    const auto be32 = [&](std::size_t at) {
        uint32_t v = 0;
        for (std::size_t i = 0; i < 4; ++i) v = (v << 8) | static_cast<uint8_t>(bytes[at + i]);
        return v;
    };
    if (be32(8) != 13 || bytes.substr(12, 4) != "IHDR") return bad("naip: the PNG's first chunk is not its IHDR");
    PngHeader h;
    h.width = be32(16);
    h.height = be32(20);
    h.bit_depth = static_cast<uint8_t>(bytes[24]);
    h.color_type = static_cast<uint8_t>(bytes[25]);
    return h;
}

}  // namespace spade::geofetch
