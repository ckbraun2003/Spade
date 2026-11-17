#include "geo_services.hpp"

#include <charconv>
#include <cmath>
#include <format>
#include <system_error>

namespace spade::geofetch {

namespace {

[[nodiscard]] std::string bbox_text(const Bbox& a) {
    return std::format("{:.9f},{:.9f},{:.9f},{:.9f}", a.west_deg, a.south_deg, a.east_deg, a.north_deg);
}

[[nodiscard]] std::unexpected<Error> bad(const std::string& why) {
    return std::unexpected(Error{Code::schema_mismatch, "ngs: " + why});
}

// The position just after `"key"` and its colon in `body`, or npos.
[[nodiscard]] std::size_t value_after_key(std::string_view body, std::string_view key) {
    const std::string quoted = "\"" + std::string(key) + "\"";
    const std::size_t at = body.find(quoted);
    if (at == std::string_view::npos) return std::string_view::npos;
    std::size_t i = at + quoted.size();
    while (i < body.size() && (body[i] == ' ' || body[i] == '\t' || body[i] == '\r' || body[i] == '\n')) ++i;
    if (i >= body.size() || body[i] != ':') return std::string_view::npos;
    ++i;
    while (i < body.size() && (body[i] == ' ' || body[i] == '\t' || body[i] == '\r' || body[i] == '\n')) ++i;
    return i;
}

}  // namespace

std::string elevation_url(const Bbox& area, uint32_t cols, uint32_t rows) {
    return std::format(
        "https://elevation.nationalmap.gov/arcgis/rest/services/3DEPElevation/ImageServer/exportImage"
        "?bbox={}&bboxSR=4326&imageSR=4326&size={},{}"
        "&format=tiff&pixelType=F32&interpolation=RSP_BilinearInterpolation&noData=-9999&f=image",
        bbox_text(area), cols, rows);
}

std::string image_url(const Bbox& area, uint32_t cols, uint32_t rows) {
    return std::format(
        "https://imagery.nationalmap.gov/arcgis/rest/services/USGSNAIPImagery/ImageServer/exportImage"
        "?bbox={}&bboxSR=4326&imageSR=4326&size={},{}"
        "&format=png&interpolation=RSP_BilinearInterpolation&bandIds=0,1,2&f=image",
        bbox_text(area), cols, rows);
}

// https://geodesy.noaa.gov/web_services/geoid.shtml: `model` is a numeric ID (14 is GEOID18), not the year, and longitude is negative west.
// A request with no model is answered with another model on that page's own sample, so the model is sent and the answer is checked.
std::string geoid_url(double lat_deg, double lon_deg) {
    return std::format("https://geodesy.noaa.gov/api/geoid/ght?lat={:.9f}&lon={:.9f}&model=14", lat_deg, lon_deg);
}

Result<GeoidAnswer> parse_geoid_json(std::string_view body) {
    const std::size_t at = value_after_key(body, "geoidHeight");
    if (at == std::string_view::npos) return bad("the answer has no \"geoidHeight\"");
    GeoidAnswer out;
    const char* first = body.data() + at;
    const char* last = body.data() + body.size();
    const std::from_chars_result r = std::from_chars(first, last, out.height_m);
    if (r.ec == std::errc::result_out_of_range) return bad("\"geoidHeight\" is out of range");
    if (r.ec != std::errc() || r.ptr == first) return bad("\"geoidHeight\" is not a number");
    if (!std::isfinite(out.height_m)) return bad("\"geoidHeight\" is not finite");
    const std::size_t m = value_after_key(body, "geoidModel");
    if (m == std::string_view::npos) return bad("the answer has no \"geoidModel\"; GEOID18 is required");
    if (m >= body.size() || body[m] != '"') return bad("\"geoidModel\" is not a string");
    const std::size_t end = body.find('"', m + 1);
    if (end == std::string_view::npos) return bad("\"geoidModel\" is not closed");
    const std::string model(body.substr(m + 1, end - m - 1));
    if (model != "GEOID18") return bad("the model is " + model + ", GEOID18 is required");
    out.model = model;
    return out;
}

}  // namespace spade::geofetch
