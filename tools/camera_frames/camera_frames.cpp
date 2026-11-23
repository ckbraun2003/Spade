#include "camera_frames.hpp"

#include <algorithm>
#include <charconv>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <system_error>

#include "render/camera_shade.hpp"

namespace camera_frames {

namespace {

constexpr double kPi = 3.14159265358979323846;

[[nodiscard]] spade::Error refusal(const std::string& why) { return spade::Error{spade::Code::invalid_argument, why}; }

[[nodiscard]] bool number(const std::string& text, double& out) {
    const char* first = text.data();
    const char* last = first + text.size();
    const auto r = std::from_chars(first, last, out);
    return r.ec == std::errc{} && r.ptr == last && std::isfinite(out);
}

[[nodiscard]] bool numbers(const std::string& text, std::size_t count, double* out) {
    std::size_t begin = 0;
    for (std::size_t i = 0; i < count; ++i) {
        const std::size_t end = i + 1 < count ? text.find(',', begin) : text.size();
        if (end == std::string::npos || !number(text.substr(begin, end - begin), out[i])) return false;
        begin = end + 1;
    }
    return true;
}

[[nodiscard]] bool dimension(const std::string& text, uint32_t& out) {
    uint32_t v = 0;
    const char* first = text.data();
    const auto r = std::from_chars(first, first + text.size(), v);
    if (r.ec != std::errc{} || r.ptr != first + text.size() || v == 0 || v > 16384) return false;
    out = v;
    return true;
}

// the colour of an instance id: hue steps of the golden ratio, so neighbouring ids are far apart, at one saturation and value
[[nodiscard]] std::array<uint8_t, 3> instance_colour(uint16_t id) {
    if (id == 0) return {0, 0, 0};
    const double hue = std::fmod(static_cast<double>(id) * 0.6180339887498949, 1.0) * 6.0;
    const double s = 0.75, v = 0.95;
    const int sector = static_cast<int>(hue);
    const double f = hue - sector;
    const double p = v * (1.0 - s), q = v * (1.0 - s * f), t = v * (1.0 - s * (1.0 - f));
    double rgb[3] = {v, t, p};
    switch (sector % 6) {
        case 0: rgb[0] = v; rgb[1] = t; rgb[2] = p; break;
        case 1: rgb[0] = q; rgb[1] = v; rgb[2] = p; break;
        case 2: rgb[0] = p; rgb[1] = v; rgb[2] = t; break;
        case 3: rgb[0] = p; rgb[1] = q; rgb[2] = v; break;
        case 4: rgb[0] = t; rgb[1] = p; rgb[2] = v; break;
        default: rgb[0] = v; rgb[1] = p; rgb[2] = q; break;
    }
    return {static_cast<uint8_t>(std::lround(rgb[0] * 255.0)), static_cast<uint8_t>(std::lround(rgb[1] * 255.0)), static_cast<uint8_t>(std::lround(rgb[2] * 255.0))};
}

void put32(std::vector<uint8_t>& v, uint32_t x) {
    for (int s = 24; s >= 0; s -= 8) v.push_back(static_cast<uint8_t>(x >> s));
}

[[nodiscard]] uint32_t crc32(const uint8_t* p, std::size_t n) {
    uint32_t crc = 0xFFFFFFFFu;
    for (std::size_t i = 0; i < n; ++i) {
        crc ^= p[i];
        for (int k = 0; k < 8; ++k) crc = (crc & 1u) ? (crc >> 1) ^ 0xEDB88320u : crc >> 1;
    }
    return ~crc;
}

void chunk(std::vector<uint8_t>& out, const char type[4], const std::vector<uint8_t>& data) {
    put32(out, static_cast<uint32_t>(data.size()));
    std::vector<uint8_t> body(type, type + 4);
    body.insert(body.end(), data.begin(), data.end());
    out.insert(out.end(), body.begin(), body.end());
    put32(out, crc32(body.data(), body.size()));
}

[[nodiscard]] spade::Result<spade::render::LensSpec> checked(spade::render::LensSpec lens, const char* what) {
    if (const std::string issue = spade::render::lens_issue(lens); !issue.empty()) return std::unexpected(refusal(std::string(what) + ": " + issue));
    return lens;
}

}  // namespace

spade::Result<Options> parse_args(std::span<const std::string> args) {
    Options o;
    bool have_world = false;
    for (std::size_t i = 0; i < args.size(); ++i) {
        const std::string& a = args[i];
        const auto value = [&]() -> const std::string* { return i + 1 < args.size() ? &args[++i] : nullptr; };
        const auto vec3 = [&](glm::dvec3& into) -> bool {
            const std::string* v = value();
            double x[3];
            if (v == nullptr || !numbers(*v, 3, x)) return false;
            into = glm::dvec3(x[0], x[1], x[2]);
            return true;
        };
        if (a == "--world") {
            const std::string* v = value();
            if (v == nullptr || v->empty()) return std::unexpected(refusal("--world needs a file"));
            o.world = *v;
            have_world = true;
        } else if (a == "--camera") {
            if (!vec3(o.position)) return std::unexpected(refusal("--camera needs X,Y,Z (three numbers)"));
        } else if (a == "--look-at") {
            if (!vec3(o.target)) return std::unexpected(refusal("--look-at needs X,Y,Z (three numbers)"));
        } else if (a == "--up") {
            if (!vec3(o.up)) return std::unexpected(refusal("--up needs X,Y,Z (three numbers)"));
        } else if (a == "--size") {
            const std::string* v = value();
            const std::size_t x = v != nullptr ? v->find('x') : std::string::npos;
            if (v == nullptr || x == std::string::npos || !dimension(v->substr(0, x), o.width) || !dimension(v->substr(x + 1), o.height)) {
                return std::unexpected(refusal("--size needs WxH, each from 1 to 16384 (for example 640x480)"));
            }
        } else if (a == "--pinhole-fov" || a == "--fisheye-fov") {
            const std::string* v = value();
            double deg = 0.0;
            if (v == nullptr || !number(*v, deg)) return std::unexpected(refusal(a + " needs a number of degrees"));
            (a == "--pinhole-fov" ? o.pinhole_fov_deg : o.fisheye_fov_deg) = deg;
        } else if (a == "--k") {
            const std::string* v = value();
            if (v == nullptr || !numbers(*v, 4, o.k.data())) return std::unexpected(refusal("--k needs K1,K2,K3,K4 (four numbers)"));
        } else if (a == "--out") {
            const std::string* v = value();
            if (v == nullptr || v->empty()) return std::unexpected(refusal("--out needs a folder"));
            o.out_dir = *v;
        } else if (a == "--name") {
            const std::string* v = value();
            if (v == nullptr || v->empty() || v->find_first_of("/\\") != std::string::npos) return std::unexpected(refusal("--name needs a file stem with no folder in it"));
            o.stem = *v;
        } else if (a == "--instances") {
            o.instances = true;
        } else {
            return std::unexpected(refusal("unknown option '" + a + "'"));
        }
    }
    if (!have_world) return std::unexpected(refusal("--world is required: the world file to draw"));
    return o;
}

spade::Result<glm::dmat3> look_at(const glm::dvec3& position, const glm::dvec3& target, const glm::dvec3& up) {
    const glm::dvec3 aim = target - position;
    if (!(glm::length(aim) > 1e-12)) return std::unexpected(refusal("look_at: the target is the camera's own position"));
    if (!(glm::length(up) > 1e-12)) return std::unexpected(refusal("look_at: up has no length"));
    const glm::dvec3 forward = glm::normalize(aim);
    glm::dvec3 right = glm::cross(-up, forward);  // x = y x z, with y the world's down
    if (!(glm::length(right) > 1e-9)) return std::unexpected(refusal("look_at: up is parallel to the aim, so the camera's right is not defined"));
    right = glm::normalize(right);
    const glm::dvec3 down = glm::cross(forward, right);  // y = z x x
    return glm::dmat3(right, down, forward);
}

spade::Result<spade::render::LensSpec> pinhole_lens(uint32_t width, uint32_t height, double horizontal_fov_deg) {
    if (width == 0 || height == 0) return std::unexpected(refusal("pinhole: the picture has no size"));
    if (!(horizontal_fov_deg > 0.0 && horizontal_fov_deg < 180.0)) return std::unexpected(refusal("pinhole: the field of view must be above 0 and under 180 degrees"));
    spade::render::LensSpec lens;
    lens.kind = spade::render::kLensPinhole;
    lens.width = width;
    lens.height = height;
    lens.fx = lens.fy = (0.5 * width) / std::tan(0.5 * horizontal_fov_deg * kPi / 180.0);
    lens.cx = 0.5 * width;
    lens.cy = 0.5 * height;
    return checked(lens, "pinhole");
}

spade::Result<spade::render::LensSpec> fisheye_lens(uint32_t width, uint32_t height, double diagonal_fov_deg, const std::array<double, 4>& k) {
    if (width == 0 || height == 0) return std::unexpected(refusal("fisheye: the picture has no size"));
    if (!(diagonal_fov_deg > 0.0 && diagonal_fov_deg < 180.0)) return std::unexpected(refusal("fisheye: the diagonal field of view must be above 0 and under 180 degrees"));
    const double theta = 0.5 * diagonal_fov_deg * kPi / 180.0;
    const double t2 = theta * theta;
    const double theta_d = theta * (1.0 + t2 * (k[0] + t2 * (k[1] + t2 * (k[2] + t2 * k[3]))));
    if (!(theta_d > 0.0)) return std::unexpected(refusal("fisheye: the distortion k1..k4 puts the corner at no image radius above 0"));
    spade::render::LensSpec lens;
    lens.kind = spade::render::kLensKb4;
    lens.width = width;
    lens.height = height;
    lens.cx = 0.5 * width;
    lens.cy = 0.5 * height;
    const double corner = std::sqrt(lens.cx * lens.cx + lens.cy * lens.cy);
    lens.fx = lens.fy = corner / theta_d;
    for (int i = 0; i < 4; ++i) lens.k[i] = k[static_cast<std::size_t>(i)];
    return checked(lens, "fisheye");
}

spade::Result<Views> render_views(const spade::render::RenderScene& scene, const spade::render::RayScene& rays, const Options& options, std::span<const uint16_t> label_ids) {
    const auto pin = pinhole_lens(options.width, options.height, options.pinhole_fov_deg);
    if (!pin) return std::unexpected(pin.error());
    const auto fish = fisheye_lens(options.width, options.height, options.fisheye_fov_deg, options.k);
    if (!fish) return std::unexpected(fish.error());
    const auto frame = look_at(options.position, options.target, options.up);
    if (!frame) return std::unexpected(frame.error());
    std::vector<uint16_t> ids(label_ids.begin(), label_ids.end());
    if (ids.empty()) {
        for (std::size_t i = 0; i < scene.labels.size(); ++i) ids.push_back(static_cast<uint16_t>(i + 1));
    } else if (ids.size() != scene.labels.size()) {
        return std::unexpected(refusal("label_ids has " + std::to_string(ids.size()) + " entries and the scene has " + std::to_string(scene.labels.size()) + " labels"));
    }
    Views views;
    for (const bool fisheye : {false, true}) {
        spade::render::CameraFrameDesc desc;
        desc.lens = fisheye ? *fish : *pin;
        desc.position = options.position;
        desc.camera_to_world = *frame;
        desc.channels = spade::render::kChannelImage | spade::render::kChannelInstanceId;
        desc.label_ids = ids;
        auto out = spade::render::render_camera_frame(scene, rays, desc);
        if (!out) return std::unexpected(refusal(std::string(fisheye ? "fisheye: " : "pinhole: ") + out.error().context));
        (fisheye ? views.fisheye : views.pinhole) = std::move(*out);
    }
    return views;
}

std::vector<uint8_t> interleave_rgb(std::span<const uint8_t> planar, uint32_t width, uint32_t height, uint32_t channels) {
    const std::size_t pixels = std::size_t{width} * height;
    if ((channels != 1 && channels != 3) || planar.size() != pixels * channels) return {};
    std::vector<uint8_t> rgb(pixels * 3);
    for (std::size_t i = 0; i < pixels; ++i) {
        for (std::size_t c = 0; c < 3; ++c) rgb[3 * i + c] = planar[(channels == 1 ? 0 : c) * pixels + i];
    }
    return rgb;
}

std::vector<uint8_t> instance_rgb(std::span<const uint16_t> ids) {
    std::vector<uint8_t> rgb;
    rgb.reserve(ids.size() * 3);
    for (const uint16_t id : ids) {
        const auto c = instance_colour(id);
        rgb.insert(rgb.end(), c.begin(), c.end());
    }
    return rgb;
}

std::vector<uint8_t> encode_png_rgb(uint32_t width, uint32_t height, std::span<const uint8_t> rgb) {
    if (width == 0 || height == 0 || rgb.size() != std::size_t{width} * height * 3) return {};
    std::vector<uint8_t> raw;
    raw.reserve(std::size_t{height} * (1 + std::size_t{width} * 3));
    for (uint32_t y = 0; y < height; ++y) {
        raw.push_back(0);  // filter: none
        raw.insert(raw.end(), rgb.begin() + static_cast<std::ptrdiff_t>(std::size_t{y} * width * 3), rgb.begin() + static_cast<std::ptrdiff_t>(std::size_t{y + 1} * width * 3));
    }
    std::vector<uint8_t> z = {0x78, 0x01};  // zlib: deflate, 32 KiB window, no dictionary, check bits for 0x7801
    for (std::size_t at = 0; at < raw.size() || at == 0;) {
        const std::size_t n = std::min<std::size_t>(65535, raw.size() - at);
        const bool last = at + n >= raw.size();
        z.push_back(last ? 1 : 0);  // BFINAL, BTYPE 00 (stored)
        z.push_back(static_cast<uint8_t>(n & 255));
        z.push_back(static_cast<uint8_t>(n >> 8));
        z.push_back(static_cast<uint8_t>(~n & 255));
        z.push_back(static_cast<uint8_t>((~n >> 8) & 255));
        z.insert(z.end(), raw.begin() + static_cast<std::ptrdiff_t>(at), raw.begin() + static_cast<std::ptrdiff_t>(at + n));
        at += n;
        if (last) break;
    }
    uint32_t a = 1, b = 0;
    for (const uint8_t byte : raw) {
        a = (a + byte) % 65521u;
        b = (b + a) % 65521u;
    }
    put32(z, (b << 16) | a);

    std::vector<uint8_t> png = {0x89, 'P', 'N', 'G', 0x0D, 0x0A, 0x1A, 0x0A};
    std::vector<uint8_t> ihdr;
    put32(ihdr, width);
    put32(ihdr, height);
    ihdr.insert(ihdr.end(), {8, 2, 0, 0, 0});  // 8 bits, truecolour, deflate, no filter method, no interlace
    chunk(png, "IHDR", ihdr);
    chunk(png, "IDAT", z);
    chunk(png, "IEND", {});
    return png;
}

spade::Result<std::vector<std::string>> write_views(const Options& options, const Views& views) {
    namespace fs = std::filesystem;
    std::error_code ec;
    fs::create_directories(options.out_dir, ec);
    if (ec) return std::unexpected(refusal("cannot make the folder " + options.out_dir + ": " + ec.message()));
    std::vector<std::string> written;
    const auto put = [&](const std::string& name, const std::vector<uint8_t>& png) -> bool {
        if (png.empty()) return false;
        const fs::path path = fs::path(options.out_dir) / name;
        std::ofstream out(path, std::ios::binary);
        out.write(reinterpret_cast<const char*>(png.data()), static_cast<std::streamsize>(png.size()));
        if (!out) return false;
        written.push_back(path.string());
        return true;
    };
    for (const bool instances : {false, true}) {
        if (instances && !options.instances) break;
        for (const bool fisheye : {false, true}) {
            const spade::render::CameraFrame& f = fisheye ? views.fisheye : views.pinhole;
            const std::string name = options.stem + (fisheye ? "_fisheye" : "_pinhole") + (instances ? "_instances.png" : ".png");
            std::vector<uint8_t> rgb;
            if (instances) {
                rgb = instance_rgb(f.instance_id);
            } else {
                const std::size_t pixels = std::size_t{options.width} * options.height;
                rgb = interleave_rgb(f.image, options.width, options.height, pixels != 0 ? static_cast<uint32_t>(f.image.size() / pixels) : 0u);
            }
            if (!put(name, encode_png_rgb(options.width, options.height, rgb))) return std::unexpected(refusal("cannot write " + name + " in " + options.out_dir));
        }
    }
    return written;
}

}  // namespace camera_frames
