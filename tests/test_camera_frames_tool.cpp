// tools/camera_frames (O150, video 4): the engine camera's CPU reference frames of one pose, the pinhole and the KB4 fisheye, written as PNGs. What is pinned: the pose's
// look-at in the engine's camera frame (x right, y down, z forward), the two lenses' own geometry (the pinhole's horizontal field of view, the fisheye's diagonal one),
// the PNG bytes (signature, chunk CRCs, the zlib stored blocks and their Adler-32 read back to the pixels), the instance colours, the flags' refusals, and an end-to-end
// frame of a wall through both lenses.

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <set>
#include <string>
#include <vector>

#include "../tools/camera_frames/camera_frames.hpp"
#include "render/camera_lens.hpp"
#include "render/ray_scene.hpp"
#include "render/scene.hpp"
#include "unique_temp.hpp"

namespace cf = camera_frames;
namespace rd = spade::render;
namespace fs = std::filesystem;

namespace {

[[nodiscard]] std::vector<std::string> words(std::initializer_list<const char*> list) {
    std::vector<std::string> out;
    for (const char* w : list) out.emplace_back(w);
    return out;
}

[[nodiscard]] uint32_t be32(const std::vector<uint8_t>& v, std::size_t at) {
    return (uint32_t{v[at]} << 24) | (uint32_t{v[at + 1]} << 16) | (uint32_t{v[at + 2]} << 8) | uint32_t{v[at + 3]};
}

[[nodiscard]] uint32_t crc32_of(const uint8_t* p, std::size_t n) {
    uint32_t crc = 0xFFFFFFFFu;
    for (std::size_t i = 0; i < n; ++i) {
        crc ^= p[i];
        for (int k = 0; k < 8; ++k) crc = (crc & 1u) ? (crc >> 1) ^ 0xEDB88320u : crc >> 1;
    }
    return ~crc;
}

// The pixels a PNG of this encoder holds: checks the signature and every chunk's CRC, reads the zlib stream's stored blocks and its Adler-32, strips each row's filter byte.
struct Decoded {
    uint32_t width = 0, height = 0;
    uint8_t depth = 0, colour = 0;
    std::vector<uint8_t> rgb;
    bool ok = false;
    std::string why;
};

[[nodiscard]] Decoded decode(const std::vector<uint8_t>& png) {
    Decoded d;
    static const uint8_t sig[8] = {0x89, 'P', 'N', 'G', 0x0D, 0x0A, 0x1A, 0x0A};
    if (png.size() < 8 + 12 || !std::equal(sig, sig + 8, png.begin())) {
        d.why = "no PNG signature";
        return d;
    }
    std::vector<uint8_t> idat;
    std::size_t at = 8;
    std::string last;
    while (at + 12 <= png.size()) {
        const uint32_t len = be32(png, at);
        if (at + 12 + len > png.size()) {
            d.why = "a chunk runs past the end";
            return d;
        }
        const std::string type(png.begin() + static_cast<std::ptrdiff_t>(at + 4), png.begin() + static_cast<std::ptrdiff_t>(at + 8));
        if (crc32_of(png.data() + at + 4, 4 + len) != be32(png, at + 8 + len)) {
            d.why = "bad CRC in " + type;
            return d;
        }
        if (type == "IHDR") {
            d.width = be32(png, at + 8);
            d.height = be32(png, at + 12);
            d.depth = png[at + 16];
            d.colour = png[at + 17];
        } else if (type == "IDAT") {
            idat.insert(idat.end(), png.begin() + static_cast<std::ptrdiff_t>(at + 8), png.begin() + static_cast<std::ptrdiff_t>(at + 8 + len));
        }
        last = type;
        at += 12 + len;
    }
    if (last != "IEND" || at != png.size()) {
        d.why = "does not end with IEND";
        return d;
    }
    if (idat.size() < 2 + 4 || (idat[0] & 0x0F) != 8 || ((uint32_t{idat[0]} << 8 | idat[1]) % 31u) != 0u) {
        d.why = "bad zlib header";
        return d;
    }
    std::vector<uint8_t> raw;
    std::size_t z = 2;
    for (bool final = false; !final;) {
        if (z + 5 > idat.size()) {
            d.why = "a stored block runs past the end";
            return d;
        }
        const uint8_t head = idat[z];
        final = (head & 1u) != 0u;
        if (((head >> 1) & 3u) != 0u) {
            d.why = "a block that is not stored";
            return d;
        }
        const uint32_t len = idat[z + 1] | (uint32_t{idat[z + 2]} << 8);
        const uint32_t nlen = idat[z + 3] | (uint32_t{idat[z + 4]} << 8);
        if ((len ^ 0xFFFFu) != nlen || z + 5 + len + 4 > idat.size()) {
            d.why = "a bad stored block length";
            return d;
        }
        raw.insert(raw.end(), idat.begin() + static_cast<std::ptrdiff_t>(z + 5), idat.begin() + static_cast<std::ptrdiff_t>(z + 5 + len));
        z += 5 + len;
    }
    uint32_t a = 1, b = 0;
    for (const uint8_t byte : raw) {
        a = (a + byte) % 65521u;
        b = (b + a) % 65521u;
    }
    if (z + 4 != idat.size() || be32(idat, z) != ((b << 16) | a)) {
        d.why = "bad Adler-32";
        return d;
    }
    if (raw.size() != std::size_t{d.height} * (1u + 3u * d.width)) {
        d.why = "the raw size is not height x (1 + 3 x width)";
        return d;
    }
    for (uint32_t y = 0; y < d.height; ++y) {
        const std::size_t row = std::size_t{y} * (1u + 3u * d.width);
        if (raw[row] != 0) {
            d.why = "a row with a filter";
            return d;
        }
        d.rgb.insert(d.rgb.end(), raw.begin() + static_cast<std::ptrdiff_t>(row + 1), raw.begin() + static_cast<std::ptrdiff_t>(row + 1 + 3u * d.width));
    }
    d.ok = true;
    return d;
}

// A quad in the plane z = `z`, |x| <= half, |y| <= half, facing -z, one item with the given label.
void add_wall(rd::RenderScene& scene, float z, float half, const std::string& label) {
    rd::MeshData m;
    m.positions = {{-half, -half, z}, {half, -half, z}, {half, half, z}, {-half, half, z}};
    m.normals = {{0, 0, -1}, {0, 0, -1}, {0, 0, -1}, {0, 0, -1}};
    m.indices = {0, 2, 1, 0, 3, 2};
    m.submesh_first_index = {0};
    m.submesh_index_count = {6};
    m.submesh_material = {0};
    const uint32_t mesh = static_cast<uint32_t>(scene.meshes.size());
    scene.meshes.push_back(m);
    rd::DrawItem item;
    item.mesh_index = mesh;
    scene.labels.push_back(label);
    item.object_label = static_cast<uint32_t>(scene.labels.size() - 1);
    scene.statics.push_back(item);
}

}  // namespace

// The pose: the camera looks along target - position, with the world's up above it. The engine's camera frame is x right, y down, z forward, so the columns are right, down,
// forward and the matrix is a rotation. Looking down -x with +y up is the frame the engine's own camera tests use.
TEST(CameraFramesTool, LookAtGivesTheEnginesCameraFrameAsARotation) {
    const auto m = cf::look_at({3.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 1.0, 0.0});
    ASSERT_TRUE(m.has_value()) << (m ? "" : m.error().context);
    const glm::dvec3 want_x(0.0, 0.0, -1.0), want_y(0.0, -1.0, 0.0), want_z(-1.0, 0.0, 0.0);
    for (int i = 0; i < 3; ++i) {
        EXPECT_NEAR((*m)[0][i], want_x[i], 1e-12) << "right, column 0, component " << i;
        EXPECT_NEAR((*m)[1][i], want_y[i], 1e-12) << "down, column 1, component " << i;
        EXPECT_NEAR((*m)[2][i], want_z[i], 1e-12) << "forward, column 2, component " << i;
    }
    // a slanted look: still a rotation (orthonormal, determinant +1), forward along the aim
    const auto s = cf::look_at({1.0, 2.0, -3.0}, {4.0, 0.5, 2.0}, {0.0, 1.0, 0.0});
    ASSERT_TRUE(s.has_value());
    EXPECT_NEAR(glm::determinant(*s), 1.0, 1e-12);
    EXPECT_NEAR(glm::length((*s)[2] - glm::normalize(glm::dvec3(3.0, -1.5, 5.0))), 0.0, 1e-12);
    EXPECT_NEAR(glm::dot((*s)[0], (*s)[1]), 0.0, 1e-12);
    EXPECT_GT(glm::dot(-(*s)[1], glm::dvec3(0.0, 1.0, 0.0)), 0.0) << "the image's up (minus y) leans toward the world's up";
}

TEST(CameraFramesTool, LookAtRefusesAnAimItCannotMake) {
    const auto none = cf::look_at({1.0, 1.0, 1.0}, {1.0, 1.0, 1.0}, {0.0, 1.0, 0.0});
    ASSERT_FALSE(none.has_value());
    EXPECT_NE(none.error().context.find("target"), std::string::npos) << none.error().context;
    const auto straight_up = cf::look_at({0.0, 0.0, 0.0}, {0.0, 5.0, 0.0}, {0.0, 1.0, 0.0});
    ASSERT_FALSE(straight_up.has_value());
    EXPECT_NE(straight_up.error().context.find("up"), std::string::npos) << straight_up.error().context;
    const auto no_up = cf::look_at({0.0, 0.0, 0.0}, {0.0, 0.0, 1.0}, {0.0, 0.0, 0.0});
    ASSERT_FALSE(no_up.has_value());
    EXPECT_NE(no_up.error().context.find("up"), std::string::npos) << no_up.error().context;
}

// The pinhole's field of view is the horizontal one: fx = (w/2) / tan(fov/2), the centre in the middle, square pixels.
TEST(CameraFramesTool, ThePinholesHorizontalFieldOfViewIsItsImageWidth) {
    const auto lens = cf::pinhole_lens(640, 480, 90.0);
    ASSERT_TRUE(lens.has_value()) << (lens ? "" : lens.error().context);
    EXPECT_EQ(lens->kind, rd::kLensPinhole);
    EXPECT_DOUBLE_EQ(lens->fx, 320.0);
    EXPECT_DOUBLE_EQ(lens->fy, 320.0);
    EXPECT_DOUBLE_EQ(lens->cx, 320.0);
    EXPECT_DOUBLE_EQ(lens->cy, 240.0);
    EXPECT_TRUE(rd::lens_issue(*lens).empty());
    // the left edge (pixel 0, the row through the centre) is 45 degrees off the axis
    const glm::dvec3 edge = rd::lens_ray(*lens, 0.0, 240.0);
    EXPECT_NEAR(edge.x / edge.z, -1.0, 1e-12);
    EXPECT_FALSE(cf::pinhole_lens(640, 480, 0.0).has_value());
    EXPECT_FALSE(cf::pinhole_lens(640, 480, 180.0).has_value());
    EXPECT_FALSE(cf::pinhole_lens(0, 480, 90.0).has_value());
}

// The fisheye's is the DIAGONAL: with k all 0 (equidistant, r = f theta), the image corner is half the diagonal field of view off the axis, and the engine accepts it.
TEST(CameraFramesTool, TheFisheyesDiagonalFieldOfViewIsItsCornerAngle) {
    const auto lens = cf::fisheye_lens(640, 480, 170.0, {0.0, 0.0, 0.0, 0.0});
    ASSERT_TRUE(lens.has_value()) << (lens ? "" : lens.error().context);
    EXPECT_EQ(lens->kind, rd::kLensKb4);
    EXPECT_TRUE(rd::lens_issue(*lens).empty()) << rd::lens_issue(*lens);
    EXPECT_DOUBLE_EQ(lens->cx, 320.0);
    EXPECT_DOUBLE_EQ(lens->cy, 240.0);
    const glm::dvec3 corner = rd::lens_ray(*lens, 0.0, 0.0);
    const double off_axis_deg = std::acos(corner.z) * 180.0 / 3.14159265358979323846;
    EXPECT_NEAR(off_axis_deg, 85.0, 1e-3) << "half of 170 degrees";
    const glm::dvec3 axis = rd::lens_ray(*lens, 320.0, 240.0);
    EXPECT_NEAR(axis.z, 1.0, 1e-12);
    // a distortion that turns the lens over is the engine's refusal, said by name; a field of view of 180 or more is refused here
    EXPECT_FALSE(cf::fisheye_lens(640, 480, 180.0, {0.0, 0.0, 0.0, 0.0}).has_value());
    EXPECT_FALSE(cf::fisheye_lens(640, 480, 0.0, {0.0, 0.0, 0.0, 0.0}).has_value());
    const auto turned = cf::fisheye_lens(640, 480, 170.0, {-5.0, 0.0, 0.0, 0.0});
    ASSERT_FALSE(turned.has_value());
    EXPECT_NE(turned.error().context.find("k1"), std::string::npos) << turned.error().context;
}

// The PNG: the signature, a CRC on every chunk, the zlib stream's stored blocks and its Adler-32; read back, every pixel is the one written. A picture larger than one
// stored block (65,535 bytes) crosses a block boundary.
TEST(CameraFramesTool, ThePngReadsBackToTheSamePixels) {
    std::vector<uint8_t> tiny = {255, 0, 0, 0, 255, 0, 0, 0, 255, 10, 20, 30};
    const Decoded small = decode(cf::encode_png_rgb(2, 2, tiny));
    ASSERT_TRUE(small.ok) << small.why;
    EXPECT_EQ(small.width, 2u);
    EXPECT_EQ(small.height, 2u);
    EXPECT_EQ(small.depth, 8);
    EXPECT_EQ(small.colour, 2) << "truecolour";
    EXPECT_EQ(small.rgb, tiny);

    const uint32_t w = 200, h = 150;  // 90,150 raw bytes: two stored blocks
    std::vector<uint8_t> big(std::size_t{w} * h * 3u);
    for (std::size_t i = 0; i < big.size(); ++i) big[i] = static_cast<uint8_t>((i * 7u + i / 251u) & 255u);
    const Decoded round = decode(cf::encode_png_rgb(w, h, big));
    ASSERT_TRUE(round.ok) << round.why;
    EXPECT_EQ(round.rgb, big);
    EXPECT_TRUE(cf::encode_png_rgb(2, 2, std::vector<uint8_t>(5, 0)).empty()) << "a pixel buffer of the wrong size is refused with no file";
}

TEST(CameraFramesTool, ThePlanarImageIsInterleavedAndAGreyOneIsSpreadToThree) {
    const std::vector<uint8_t> planar = {1, 2, 3, 4,  10, 20, 30, 40,  100, 110, 120, 130};  // 3 x (2 x 2)
    const std::vector<uint8_t> rgb = cf::interleave_rgb(planar, 2, 2, 3);
    EXPECT_EQ(rgb, (std::vector<uint8_t>{1, 10, 100, 2, 20, 110, 3, 30, 120, 4, 40, 130}));
    const std::vector<uint8_t> grey = cf::interleave_rgb(std::vector<uint8_t>{5, 6, 7, 8}, 2, 2, 1);
    EXPECT_EQ(grey, (std::vector<uint8_t>{5, 5, 5, 6, 6, 6, 7, 7, 7, 8, 8, 8}));
    EXPECT_TRUE(cf::interleave_rgb(planar, 2, 2, 4).empty()) << "a channel count that is not 1 or 3";
    EXPECT_TRUE(cf::interleave_rgb(std::vector<uint8_t>(5, 0), 2, 2, 3).empty()) << "a buffer of the wrong size";
}

// A false colour for each instance id: 0 black, one id one colour, a different colour for each of the first hundred ids, and none of them black.
TEST(CameraFramesTool, EachInstanceIdHasItsOwnColourAndZeroIsBlack) {
    std::vector<uint16_t> ids = {0, 1, 2, 1, 0};
    const std::vector<uint8_t> rgb = cf::instance_rgb(ids);
    ASSERT_EQ(rgb.size(), 15u);
    EXPECT_EQ(std::vector<uint8_t>(rgb.begin(), rgb.begin() + 3), (std::vector<uint8_t>{0, 0, 0}));
    EXPECT_EQ(std::vector<uint8_t>(rgb.begin() + 3, rgb.begin() + 6), std::vector<uint8_t>(rgb.begin() + 9, rgb.begin() + 12)) << "one id, one colour";
    EXPECT_NE(std::vector<uint8_t>(rgb.begin() + 3, rgb.begin() + 6), std::vector<uint8_t>(rgb.begin() + 6, rgb.begin() + 9));
    std::vector<uint16_t> many;
    for (uint16_t i = 0; i < 100; ++i) many.push_back(i);
    const std::vector<uint8_t> all = cf::instance_rgb(many);
    std::set<uint32_t> seen;  // each colour packed into one integer (a set of vectors trips gcc-13 -Werror=stringop-overread)
    for (std::size_t i = 0; i < many.size(); ++i) seen.insert(uint32_t{all[3 * i]} << 16 | uint32_t{all[3 * i + 1]} << 8 | uint32_t{all[3 * i + 2]});
    EXPECT_EQ(seen.size(), 100u) << "the first hundred ids are a hundred colours";
    for (std::size_t i = 1; i < many.size(); ++i) EXPECT_FALSE(all[3 * i] == 0 && all[3 * i + 1] == 0 && all[3 * i + 2] == 0) << "id " << i << " is black";
}

TEST(CameraFramesTool, TheFlagsMakeTheOptionsAndARefusalNamesItsFlag) {
    const auto o = cf::parse_args(words({"--world", "w.yaml", "--camera", "1,2,3", "--look-at", "0,1,0", "--up", "0,1,0", "--size", "320x240", "--pinhole-fov", "80", "--fisheye-fov",
                                         "160", "--k", "0.01,0,0,0", "--out", "outdir", "--name", "shot", "--instances"}));
    ASSERT_TRUE(o.has_value()) << (o ? "" : o.error().context);
    EXPECT_EQ(o->world, "w.yaml");
    EXPECT_EQ(o->position, glm::dvec3(1.0, 2.0, 3.0));
    EXPECT_EQ(o->target, glm::dvec3(0.0, 1.0, 0.0));
    EXPECT_EQ(o->width, 320u);
    EXPECT_EQ(o->height, 240u);
    EXPECT_DOUBLE_EQ(o->pinhole_fov_deg, 80.0);
    EXPECT_DOUBLE_EQ(o->fisheye_fov_deg, 160.0);
    EXPECT_DOUBLE_EQ(o->k[0], 0.01);
    EXPECT_EQ(o->out_dir, "outdir");
    EXPECT_EQ(o->stem, "shot");
    EXPECT_TRUE(o->instances);

    const auto bad = [](std::initializer_list<const char*> list, const char* flag) {
        const auto r = cf::parse_args(words(list));
        ASSERT_FALSE(r.has_value()) << flag;
        EXPECT_NE(r.error().context.find(flag), std::string::npos) << r.error().context;
    };
    bad({"--camera", "1,2,3"}, "--world");
    bad({"--world", "w.yaml", "--bogus"}, "--bogus");
    bad({"--world", "w.yaml", "--size", "320"}, "--size");
    bad({"--world", "w.yaml", "--size", "0x10"}, "--size");
    bad({"--world", "w.yaml", "--camera", "1,2"}, "--camera");
    bad({"--world", "w.yaml", "--camera", "1,2,x"}, "--camera");
    bad({"--world", "w.yaml", "--k", "1,2,3"}, "--k");
    bad({"--world", "w.yaml", "--pinhole-fov"}, "--pinhole-fov");
    bad({"--world"}, "--world");
}

// End to end: a wall of half-width 1 at 5 m straight ahead, label id 7. Both lenses' middle pixel is the axis ray, so it is the same colour and the wall's id; the wall covers
// more of the pinhole picture (the pinhole's fx is 320 against the fisheye's 270) and a corner of either sees past it. The same pose and the same scene give the same bytes.
TEST(CameraFramesTool, ABothLensViewOfAWallHasTheWallInTheMiddleOfEach) {
    rd::RenderScene scene;
    scene.materials.resize(1);
    scene.materials[0].base_color = glm::vec4(0.5f, 0.25f, 0.125f, 1.0f);
    scene.lighting.sun_direction = glm::vec3(0.0f, 0.0f, 1.0f);
    scene.lighting.sun_color = glm::vec3(1.0f);
    scene.lighting.sun_intensity = 0.6f;
    scene.lighting.ambient_color = glm::vec3(0.1f);
    add_wall(scene, 5.0f, 1.0f, "gate");
    const spade::Result<rd::RayScene> rays = rd::RayScene::build(scene);
    ASSERT_TRUE(rays.has_value());

    cf::Options o;
    o.position = {0.0, 0.0, 0.0};
    o.target = {0.0, 0.0, 1.0};
    const std::vector<uint16_t> ids = {7};
    const auto views = cf::render_views(scene, *rays, o, ids);
    ASSERT_TRUE(views.has_value()) << (views ? "" : views.error().context);
    const uint32_t w = o.width, h = o.height;
    ASSERT_EQ(views->pinhole.image.size(), 3u * w * h);
    ASSERT_EQ(views->fisheye.image.size(), 3u * w * h);
    ASSERT_EQ(views->pinhole.instance_id.size(), std::size_t{w} * h);
    const std::size_t mid = std::size_t{h / 2} * w + w / 2;
    EXPECT_EQ(views->pinhole.instance_id[mid], 7u);
    EXPECT_EQ(views->fisheye.instance_id[mid], 7u);
    for (uint32_t c = 0; c < 3; ++c) EXPECT_EQ(views->pinhole.image[c * w * h + mid], views->fisheye.image[c * w * h + mid]) << "channel " << c;
    EXPECT_EQ(views->pinhole.instance_id[0], 0u);
    EXPECT_EQ(views->fisheye.instance_id[0], 0u);
    const auto wall = [](const rd::CameraFrame& f) { return std::count(f.instance_id.begin(), f.instance_id.end(), uint16_t{7}); };
    EXPECT_GT(wall(views->fisheye), 0);
    EXPECT_GT(wall(views->pinhole), wall(views->fisheye));

    const auto again = cf::render_views(scene, *rays, o, ids);
    ASSERT_TRUE(again.has_value());
    EXPECT_EQ(again->pinhole.image, views->pinhole.image) << "a pure function of the pose and the scene";
    EXPECT_EQ(again->fisheye.image, views->fisheye.image);

    // an empty table means the scene's own label index + 1; a table that does not cover the labels is the engine's refusal, passed up
    EXPECT_FALSE(cf::render_views(scene, *rays, o, std::vector<uint16_t>{7, 8}).has_value()) << "two ids for one label";
    const auto own = cf::render_views(scene, *rays, o, std::vector<uint16_t>{});
    ASSERT_TRUE(own.has_value()) << (own ? "" : own.error().context);
    EXPECT_EQ(own->pinhole.instance_id[mid], 1u) << "label index 0 + 1";
}

TEST(CameraFramesTool, TheFilesAreWrittenByTheirNames) {
    const spade::test::TempDir temp("camera_frames_test");  // a folder no other run shares; removed when the test ends
    const fs::path dir = temp.path() / "nested";
    cf::Options o;
    o.width = 4;
    o.height = 3;
    o.out_dir = dir.string();
    o.stem = "shot";
    cf::Views v;
    for (rd::CameraFrame* f : {&v.pinhole, &v.fisheye}) {
        f->image.assign(3u * 4u * 3u, 90);
        f->instance_id.assign(4u * 3u, 2);
    }
    const auto plain = cf::write_views(o, v);
    ASSERT_TRUE(plain.has_value()) << (plain ? "" : plain.error().context);
    ASSERT_EQ(plain->size(), 2u);
    EXPECT_EQ(fs::path((*plain)[0]).filename().string(), "shot_pinhole.png");
    EXPECT_EQ(fs::path((*plain)[1]).filename().string(), "shot_fisheye.png");
    for (const std::string& p : *plain) {
        std::ifstream in(p, std::ios::binary);
        const std::vector<uint8_t> bytes((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
        const Decoded d = decode(bytes);
        ASSERT_TRUE(d.ok) << p << ": " << d.why;
        EXPECT_EQ(d.width, 4u);
        EXPECT_EQ(d.height, 3u);
    }
    o.instances = true;
    const auto with = cf::write_views(o, v);
    ASSERT_TRUE(with.has_value());
    ASSERT_EQ(with->size(), 4u);
    EXPECT_EQ(fs::path((*with)[2]).filename().string(), "shot_pinhole_instances.png");
    EXPECT_EQ(fs::path((*with)[3]).filename().string(), "shot_fisheye_instances.png");
}
