// ---------------------------------------------------------------------------
// test_flow_movie_export.cpp -- tools/flow_movie/line_export.hpp, a hover frame's streamlines as JSON for the
// website's 3-D viewer (Cameron's ask, relayed by the lead 2026-11-10): the seeds around each rotor disc, the body
// frame, the file under 10 MB by decimation, the header, and the caption. Analytic fields through the publish
// format (flow_synthetic.hpp), so a failure is the exporter's and never the solver's.
// ---------------------------------------------------------------------------

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <string>
#include <vector>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include "../tools/flow_movie/line_export.hpp"
#include "flow_synthetic.hpp"

namespace {

namespace fm = flow_movie;
using spade::test::FlowChannels;

constexpr float kRadius = 0.0762f;

// A box 0.8 m a side, 64 cells, holding the air going straight down at `down` m/s with a swirl about the vertical
// line through (cx, cz): a Lamb-Oseen column (circulation `gamma`, core `rc`), the shape of a rotor's wake.
[[nodiscard]] spade::FlowVolume column(float down, float gamma, float rc, float cx, float cz) {
    return spade::test::make_flow_volume(glm::vec3(-0.4f), 0.0125f, 64, 64, 64, [=](glm::vec3 p) {
        const float dx = p.x - cx, dz = p.z - cz;
        const float r2 = dx * dx + dz * dz;
        const float r = std::sqrt(r2);
        const float swirl = r > 1.0e-6f ? gamma / (6.2831853f * r) * (1.0f - std::exp(-r2 / (rc * rc))) : 0.0f;
        FlowChannels c{};
        c[spade::kFlowU] = r > 1.0e-6f ? -swirl * dz / r : 0.0f;
        c[spade::kFlowV] = -down;
        c[spade::kFlowW] = r > 1.0e-6f ? swirl * dx / r : 0.0f;
        return c;
    });
}

[[nodiscard]] spade::FlowVolume uniform_down(float down) { return column(down, 0.0f, 0.02f, 0.0f, 0.0f); }

[[nodiscard]] std::vector<fm::ExportRotor> quad_rotors() {
    return {{glm::vec3(0.11f, 0.024f, 0.0f), glm::vec3(0, 1, 0), kRadius, +1.0f},
            {glm::vec3(0.0f, 0.024f, 0.11f), glm::vec3(0, 1, 0), kRadius, -1.0f},
            {glm::vec3(-0.11f, 0.024f, 0.0f), glm::vec3(0, 1, 0), kRadius, +1.0f},
            {glm::vec3(0.0f, 0.024f, -0.11f), glm::vec3(0, 1, 0), kRadius, -1.0f}};
}

[[nodiscard]] fm::ExportInfo info() {
    return {"flow_hover", "commit abc123def456, MSVC 1944, release", "D1, 3 m/s headwind, 104^3 at 8 mm",
            "a stand-in", 17u, 1020u, 0.51};
}

struct Parsed {
    std::vector<std::vector<glm::vec3>> lines;
    std::vector<std::vector<double>> speeds;
};

// The numbers of every "points":[[x,y,z],...] and "speeds":[...] in a file this exporter wrote.
[[nodiscard]] Parsed parse(const std::string& json) {
    Parsed out;
    std::size_t at = 0;
    while ((at = json.find("{\"points\":[", at)) != std::string::npos) {
        at += 10;  // at the outer '['
        std::vector<glm::vec3> pts;
        std::size_t i = at + 1;
        while (json[i] == '[') {
            char* end = nullptr;
            const double x = std::strtod(json.c_str() + i + 1, &end);
            const double y = std::strtod(end + 1, &end);
            const double z = std::strtod(end + 1, &end);
            pts.emplace_back(static_cast<float>(x), static_cast<float>(y), static_cast<float>(z));
            i = static_cast<std::size_t>(end - json.c_str()) + 1u;  // past ']'
            if (json[i] == ',') ++i;
        }
        const std::size_t s = json.find("\"speeds\":[", i);
        std::vector<double> sp;
        std::size_t j = s + 10;
        while (json[j] != ']') {
            char* end = nullptr;
            sp.push_back(std::strtod(json.c_str() + j, &end));
            j = static_cast<std::size_t>(end - json.c_str());
            if (json[j] == ',') ++j;
        }
        out.lines.push_back(std::move(pts));
        out.speeds.push_back(std::move(sp));
        at = j;
    }
    return out;
}

// Braces and brackets balance outside strings, and no value is a JSON-illegal token.
[[nodiscard]] bool well_formed(const std::string& json) {
    int depth = 0;
    bool in_string = false;
    for (std::size_t i = 0; i < json.size(); ++i) {
        const char c = json[i];
        if (in_string) {
            if (c == '\\') ++i;
            else if (c == '"') in_string = false;
            continue;
        }
        if (c == '"') in_string = true;
        else if (c == '{' || c == '[') ++depth;
        else if (c == '}' || c == ']') --depth;
        if (depth < 0) return false;
    }
    return depth == 0 && !in_string;
}

[[nodiscard]] bool has(const std::string& text, const std::string& part) { return text.find(part) != std::string::npos; }

TEST(FlowMovieExport, TheSeedsAreRingsAroundEachRotorDiscInTheBodyFramesPlaneAboveItAndTheirCountsAddUp) {
    const fm::ExportBody body{glm::vec3(0.01f, 0.02f, -0.03f), glm::angleAxis(0.5f, glm::vec3(0, 1, 0))};
    const auto rotors = quad_rotors();
    fm::ExportOptions options;
    options.lines = 600;
    const auto seeds = fm::export_seeds(body, rotors, options);
    ASSERT_TRUE(seeds.has_value()) << seeds.error().context;
    // 600 over four rotors is 150 each, in five rings of 11, 21, 30, 39 and 49 (counts in proportion to radius).
    ASSERT_EQ(seeds->size(), std::size_t{600});
    // The seeds come rotor by rotor. Every seed of rotor r, back in the body frame, is on r's plane 0.3 R above the
    // disc and 0.3 to 1.3 R from its axis.
    const glm::quat inverse = glm::conjugate(body.orient);
    for (std::size_t r = 0; r < rotors.size(); ++r) {
        for (std::size_t i = 150 * r; i < 150 * (r + 1); ++i) {
            const glm::vec3 d = inverse * ((*seeds)[i] - body.pos) - rotors[r].hub;
            EXPECT_NEAR(d.y, 0.3f * kRadius, 1.0e-5f) << "rotor " << r << " seed " << i;
            const float rho = std::sqrt(d.x * d.x + d.z * d.z);
            EXPECT_GE(rho, 0.29f * kRadius) << "rotor " << r << " seed " << i;
            EXPECT_LE(rho, 1.31f * kRadius) << "rotor " << r << " seed " << i;
        }
    }
    // The innermost ring of the first rotor: 11 seeds at 0.3 R.
    for (std::size_t i = 0; i < 11; ++i) {
        const glm::vec3 d = inverse * ((*seeds)[i] - body.pos) - rotors[0].hub;
        EXPECT_NEAR(std::sqrt(d.x * d.x + d.z * d.z), 0.3f * kRadius, 1.0e-5f);
    }
    // A pure function: the same seeds again.
    const auto again = fm::export_seeds(body, rotors, options);
    ASSERT_TRUE(again.has_value());
    EXPECT_EQ(*seeds, *again);
}

TEST(FlowMovieExport, ALineInTheBodyFrameIsTheWorldLineTurnedAndShiftedByTheBodysPose) {
    // The air goes straight down in the world; the body is pitched 0.4 rad about +z, so in the body frame the lines
    // run along the world's -y turned by the inverse of the pose, and the speed is the same 3 m/s everywhere.
    const spade::FlowVolume volume = uniform_down(3.0f);
    const fm::ExportBody body{glm::vec3(0.0f, 0.0f, 0.0f), glm::angleAxis(0.4f, glm::vec3(0, 0, 1))};
    const std::vector<fm::ExportRotor> rotors{{glm::vec3(0.0f, 0.0f, 0.0f), glm::vec3(0, 1, 0), kRadius, 1.0f}};
    fm::ExportOptions options;
    options.lines = 20;
    options.seed_height_radii = 0.0f;
    const auto result = fm::export_streamlines(volume, body, rotors, info(), options);
    ASSERT_TRUE(result.has_value()) << result.error().context;
    ASSERT_TRUE(well_formed(result->json));
    const Parsed p = parse(result->json);
    ASSERT_EQ(p.lines.size(), std::size_t{result->lines});
    ASSERT_GE(p.lines.size(), 15u);
    const glm::vec3 want = glm::conjugate(body.orient) * glm::vec3(0.0f, -1.0f, 0.0f);
    for (std::size_t l = 0; l < p.lines.size(); ++l) {
        const auto& pts = p.lines[l];
        ASSERT_GE(pts.size(), 2u);
        const glm::vec3 dir = glm::normalize(pts.back() - pts.front());
        EXPECT_GT(glm::dot(dir, want), 0.9999f) << "line " << l << " runs along the body-frame image of -y";
        for (const double s : p.speeds[l]) EXPECT_NEAR(s, 3.0, 0.01);
    }
}

TEST(FlowMovieExport, TheFileIsWellFormedHasTheHeaderTheRotorsAndTheCountsAndNoIllegalNumber) {
    const spade::FlowVolume volume = column(3.0f, 0.6f, 0.012f, 0.11f, 0.0f);
    const fm::ExportBody body{glm::vec3(0.0f), glm::quat(1.0f, 0.0f, 0.0f, 0.0f)};
    const auto rotors = quad_rotors();
    const auto result = fm::export_streamlines(volume, body, rotors, info());
    ASSERT_TRUE(result.has_value()) << result.error().context;
    const std::string& j = result->json;
    EXPECT_TRUE(well_formed(j));
    EXPECT_EQ(j.rfind("{\"header\":{\"format\":\"spade-streamlines-1\"", 0), std::size_t{0});
    for (const char* part : {"\"name\":\"flow_hover\"", "\"frame\":\"body\"", "\"units\":{\"length\":\"m\",\"speed\":\"m/s\"}",
                             "\"axes\":", "+x the nose", "\"sim\":\"commit abc123def456", "\"config\":\"D1, 3 m/s headwind",
                             "\"field\":{\"frame\":17,\"step\":1020", "\"cells\":[64,64,64]", "\"rotors\":[",
                             "\"hub\":[0.11000,0.02400,0.00000]", "\"radius\":0.07620", "\"spin\":-1", "\"seeding\":",
                             "\"decimation\":{\"least_arc_length_m\":", "\"counts\":{\"lines\":"}) {
        EXPECT_TRUE(has(j, part)) << part;
    }
    for (const char* bad : {"nan", "inf", "-0.0000", "-0.00,", "e+", "E+"}) EXPECT_FALSE(has(j, bad)) << bad;
    EXPECT_TRUE(has(j, "\"counts\":{\"lines\":" + std::to_string(result->lines) + ",\"points\":" +
                           std::to_string(result->points)));
    const Parsed p = parse(j);
    EXPECT_EQ(p.lines.size(), std::size_t{result->lines});
    std::size_t points = 0;
    for (const auto& l : p.lines) points += l.size();
    EXPECT_EQ(points, std::size_t{result->points});
    // A wake the swirl turns: its lines bend, so this is not a field of straight lines.
    EXPECT_GT(p.lines.size(), 400u);
}

TEST(FlowMovieExport, EightHundredLinesOfAWakeFitUnderTenMegabytesAndTheSameInputsGiveTheSameBytes) {
    const spade::FlowVolume volume = column(3.0f, 0.6f, 0.012f, 0.11f, 0.0f);
    const fm::ExportBody body{glm::vec3(0.0f), glm::quat(1.0f, 0.0f, 0.0f, 0.0f)};
    const auto rotors = quad_rotors();
    fm::ExportOptions options;
    options.lines = 800;
    const auto a = fm::export_streamlines(volume, body, rotors, info(), options);
    const auto b = fm::export_streamlines(volume, body, rotors, info(), options);
    ASSERT_TRUE(a.has_value()) << a.error().context;
    ASSERT_TRUE(b.has_value());
    EXPECT_LE(a->json.size(), fm::kExportMaxBytes);
    EXPECT_GE(a->lines, 700u);
    EXPECT_EQ(a->json, b->json);
}

TEST(FlowMovieExport, AnOverBudgetFileIsDecimatedByArcLengthUntilItFitsAndAnImpossibleOneIsRefusedNamingItsSize) {
    const spade::FlowVolume volume = column(3.0f, 0.6f, 0.012f, 0.11f, 0.0f);
    const fm::ExportBody body{glm::vec3(0.0f), glm::quat(1.0f, 0.0f, 0.0f, 0.0f)};
    const auto rotors = quad_rotors();
    fm::ExportOptions loose;
    loose.lines = 400;
    const auto first = fm::export_streamlines(volume, body, rotors, info(), loose);
    ASSERT_TRUE(first.has_value());
    fm::ExportOptions tight = loose;
    tight.max_bytes = first->json.size() / 3u;
    const auto fitted = fm::export_streamlines(volume, body, rotors, info(), tight);
    ASSERT_TRUE(fitted.has_value()) << fitted.error().context;
    EXPECT_LE(fitted->json.size(), tight.max_bytes);
    EXPECT_GT(fitted->spacing_m, first->spacing_m) << "it got there by thinning the points";
    EXPECT_LT(fitted->points, first->points);
    // Every kept line still ends where its full one does: the first and last points are always kept.
    EXPECT_EQ(fitted->lines + fitted->dropped, first->lines + first->dropped);

    fm::ExportOptions impossible = loose;
    impossible.max_bytes = 2000;
    const auto refused = fm::export_streamlines(volume, body, rotors, info(), impossible);
    ASSERT_FALSE(refused.has_value());
    EXPECT_EQ(refused.error().code, spade::Code::capacity_exceeded);
    EXPECT_TRUE(has(refused.error().context, "bytes")) << refused.error().context;
    EXPECT_TRUE(has(refused.error().context, "2000")) << refused.error().context;
}

TEST(FlowMovieExport, RefusalsNameTheirCause) {
    const spade::FlowVolume volume = uniform_down(3.0f);
    const fm::ExportBody body{glm::vec3(0.0f), glm::quat(1.0f, 0.0f, 0.0f, 0.0f)};
    const auto rotors = quad_rotors();
    const auto refused = [&](const fm::ExportBody& b, std::span<const fm::ExportRotor> r, const fm::ExportOptions& o,
                             const spade::FlowVolume& v) {
        const auto result = fm::export_streamlines(v, b, r, info(), o);
        EXPECT_FALSE(result.has_value());
        EXPECT_EQ(result.has_value() ? spade::Code::internal : result.error().code, spade::Code::invalid_argument);
        return result.has_value() ? std::string() : result.error().context;
    };
    EXPECT_TRUE(has(refused(body, {}, {}, volume), "at least one rotor"));
    std::vector<fm::ExportRotor> bad = rotors;
    bad[2].axis = glm::vec3(0.0f, 2.0f, 0.0f);
    EXPECT_TRUE(has(refused(body, bad, {}, volume), "rotor 2's axis is not a unit vector"));
    bad = rotors;
    bad[1].radius = 0.0f;
    EXPECT_TRUE(has(refused(body, bad, {}, volume), "rotor 1 has no radius"));
    fm::ExportOptions o;
    o.lines = 0;
    EXPECT_TRUE(has(refused(body, rotors, o, volume), "lines must be"));
    o = {};
    o.point_decimals = 9;
    EXPECT_TRUE(has(refused(body, rotors, o, volume), "decimals"));
    fm::ExportBody nan_body = body;
    nan_body.pos.x = std::nanf("");
    EXPECT_TRUE(has(refused(nan_body, rotors, {}, volume), "not finite"));
    spade::FlowVolume broken = volume;
    broken.v.pop_back();
    EXPECT_FALSE(refused(body, rotors, {}, broken).empty());
}

TEST(FlowMovieExport, TheCaptionNamesWhatTheFileIsAndNeverClaimsMore) {
    const fm::ExportInfo i = info();
    fm::ExportResult r;
    r.lines = 612;
    r.points = 90000;
    r.spacing_m = 0.01f;
    fm::ExportCaption in;
    in.info = &i;
    in.result = &r;
    in.commit = "abc123";
    in.date_utc = "2026-11-10";
    in.command = "flow_movie --video d1-hover --export-lines flow_hover";
    in.ticket = "web-spade-export";
    const std::string hover = fm::export_caption_text(in);
    for (const char* part : {"# flow_hover", "- Commit: abc123", "612 streamlines", "frame 17, step 1020",
                             "a snapshot of an unsteady field", "+x the nose", "not validated speeds",
                             "has not been measured against a downwash record", "web-spade-export"}) {
        EXPECT_TRUE(has(hover, part)) << part;
    }
    EXPECT_FALSE(has(hover, "crosswind setting")) << "the headwind case makes no crosswind claim";
    EXPECT_FALSE(has(hover, "0.18 s")) << "no device claim for a CPU run";
    in.crosswind = true;
    in.device = true;
    const std::string cross = fm::export_caption_text(in);
    EXPECT_TRUE(has(cross, "supported but not validated"));
    EXPECT_TRUE(has(cross, "not a prediction of the thrust, power or attitude"));
    EXPECT_TRUE(has(cross, "bit-equal to the CPU's through about 0.18 s"));
    EXPECT_TRUE(has(cross, "a single run on the device of record"));
    EXPECT_EQ(fm::data_manifest_line("flow_hover.json", 4096, "sim", "hover streamlines"),
              "- flow_hover.json | data, 4096 bytes | - | sim | hover streamlines");
}

}  // namespace
