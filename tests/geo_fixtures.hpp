#pragma once

// Synthetic fixtures for the geo map fetch's tests (L85, SP-5): TIFF and PNG bytes built by hand, and a fake network that
// serves them. No test touches the real network.

#include <algorithm>
#include <bit>
#include <cstdint>
#include <functional>
#include <map>
#include <string>
#include <utility>
#include <vector>

#include "../tools/geo_fetch/http_client.hpp"

namespace spade::geofetchtest {

using spade::geofetch::HttpClient;
using spade::geofetch::HttpError;
using spade::geofetch::HttpProgress;
using spade::geofetch::HttpResponse;

struct TiffOptions {
    bool big_endian = false;
    uint16_t compression = 1;
    uint16_t bits = 32;
    uint16_t sample_format = 3;  // IEEE float
    uint16_t samples_per_pixel = 1;
    uint32_t rows_per_strip = 0;  // 0: one strip
    bool tiled = false;
    bool truncate = false;
};

inline void put16(std::string& s, uint16_t v, bool be) {
    if (be) {
        s.push_back(static_cast<char>(v >> 8));
        s.push_back(static_cast<char>(v & 0xff));
    } else {
        s.push_back(static_cast<char>(v & 0xff));
        s.push_back(static_cast<char>(v >> 8));
    }
}

inline void put32(std::string& s, uint32_t v, bool be) {
    if (be) {
        for (int i = 3; i >= 0; --i) s.push_back(static_cast<char>((v >> (8 * i)) & 0xff));
    } else {
        for (int i = 0; i < 4; ++i) s.push_back(static_cast<char>((v >> (8 * i)) & 0xff));
    }
}

// A TIFF of `w * h` float32 samples: header, the strips' bytes, the strip offset and count arrays, then the IFD.
inline std::string make_tiff(uint32_t w, uint32_t h, const std::vector<float>& samples, const TiffOptions& o = {}) {
    const bool be = o.big_endian;
    std::string s = be ? "MM" : "II";
    put16(s, 42, be);
    put32(s, 0, be);  // the IFD offset, patched below
    const uint32_t rps = o.rows_per_strip == 0 ? h : o.rows_per_strip;
    std::vector<uint32_t> offsets, counts;
    for (uint32_t row = 0; row < h; row += rps) {
        const uint32_t rows = std::min(rps, h - row);
        offsets.push_back(static_cast<uint32_t>(s.size()));
        counts.push_back(rows * w * 4);
        for (uint32_t i = 0; i < rows * w; ++i) put32(s, std::bit_cast<uint32_t>(samples[row * w + i]), be);
    }
    const uint32_t offsets_at = static_cast<uint32_t>(s.size());
    for (uint32_t v : offsets) put32(s, v, be);
    const uint32_t counts_at = static_cast<uint32_t>(s.size());
    for (uint32_t v : counts) put32(s, v, be);
    const uint32_t ifd_at = static_cast<uint32_t>(s.size());
    struct Entry {
        uint16_t tag, type;
        uint32_t count, value;  // a SHORT value sits in the first two bytes of the value field
    };
    const uint32_t n = static_cast<uint32_t>(offsets.size());
    std::vector<Entry> e = {{256, 4, 1, w},
                            {257, 4, 1, h},
                            {258, 3, 1, o.bits},
                            {259, 3, 1, o.compression},
                            {262, 3, 1, 1},
                            {273, 4, n, n == 1 ? offsets[0] : offsets_at},
                            {277, 3, 1, o.samples_per_pixel},
                            {278, 4, 1, rps},
                            {279, 4, n, n == 1 ? counts[0] : counts_at},
                            {339, 3, 1, o.sample_format}};
    if (o.tiled) e.push_back({322, 4, 1, 256});
    put16(s, static_cast<uint16_t>(e.size()), be);
    for (const Entry& x : e) {
        put16(s, x.tag, be);
        put16(s, x.type, be);
        put32(s, x.count, be);
        if (x.type == 3) {
            put16(s, static_cast<uint16_t>(x.value), be);
            put16(s, 0, be);
        } else {
            put32(s, x.value, be);
        }
    }
    put32(s, 0, be);  // no next IFD
    for (int i = 0; i < 4; ++i) s[4 + i] = static_cast<char>(be ? (ifd_at >> (8 * (3 - i))) & 0xff : (ifd_at >> (8 * i)) & 0xff);
    if (o.truncate) s.resize(s.size() / 2);
    return s;
}

// The PNG signature and an IHDR chunk (its CRC is not checked by the tool), then `payload` as stand-in image data.
inline std::string make_png(uint32_t w, uint32_t h, int depth, int color, const std::string& payload = {}) {
    std::string s = "\x89PNG\r\n\x1a\n";
    const auto be32 = [&](uint32_t v) {
        for (int i = 3; i >= 0; --i) s.push_back(static_cast<char>((v >> (8 * i)) & 0xff));
    };
    be32(13);
    s += "IHDR";
    be32(w);
    be32(h);
    s.push_back(static_cast<char>(depth));
    s.push_back(static_cast<char>(color));
    s.push_back(0);
    s.push_back(0);
    s.push_back(0);
    be32(0);
    s += payload;
    return s;
}

// What the fake serves, and the request that matches it (side 40 m, a 10 m grid cell and a 20 m pixel: a 4 x 4 grid and a
// 2 x 2 image).
struct GeoFixture {
    std::string geoid = R"({"geoidModel":"GEOID18","latitude":40.005,"longitude":-112.995,"geoidHeight":-23.4})";
    std::string tiff;  // 4 x 4, height 1000 + 2 * column, so the middle of the grid is 1003 m
    std::string png;   // 2 x 2 RGB
    GeoFixture() {
        std::vector<float> h(16);
        for (uint32_t r = 0; r < 4; ++r) {
            for (uint32_t c = 0; c < 4; ++c) h[r * 4 + c] = 1000.0f + 2.0f * static_cast<float>(c);
        }
        tiff = make_tiff(4, 4, h);
        png = make_png(2, 2, 8, 2, "stand-in image data");
    }
};

// A fake network: `handler(url, n)` answers the n-th (from 0) request to that url's host. Every request is logged.
class FakeHttp : public HttpClient {
  public:
    using Handler = std::function<HttpResponse(const std::string& url, int nth_for_host)>;
    explicit FakeHttp(Handler h) : handler_(std::move(h)) {}

    // Serves `f`'s good answers by host.
    static Handler serve(const GeoFixture& f) {
        return [f](const std::string& url, int) {
            HttpResponse r;
            if (url.find("geodesy.noaa.gov") != std::string::npos) {
                r.body = f.geoid;
            } else if (url.find("elevation.nationalmap.gov") != std::string::npos) {
                r.body = f.tiff;
            } else if (url.find("imagery.nationalmap.gov") != std::string::npos) {
                r.body = f.png;
            } else {
                r.status = 404;
            }
            return r;
        };
    }

    HttpResponse get(const std::string& url, int, const HttpProgress& progress) override {
        urls.push_back(url);
        const std::string host = url.substr(0, url.find('/', 8));
        const int n = counts_[host]++;
        HttpResponse r = handler_(url, n);
        if (progress && r.error == HttpError::none && !r.body.empty()) {
            progress(r.body.size() / 2, r.body.size());
            progress(r.body.size(), r.body.size());
        }
        return r;
    }

    std::vector<std::string> urls;

  private:
    Handler handler_;
    std::map<std::string, int> counts_;
};

}  // namespace spade::geofetchtest
