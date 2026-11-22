#include "camera_frames.hpp"

namespace camera_frames {

namespace {
[[nodiscard]] spade::Error stub() { return spade::Error{spade::Code::invalid_argument, "camera_frames: not written yet"}; }
}  // namespace

spade::Result<Options> parse_args(std::span<const std::string>) { return std::unexpected(stub()); }
spade::Result<glm::dmat3> look_at(const glm::dvec3&, const glm::dvec3&, const glm::dvec3&) { return std::unexpected(stub()); }
spade::Result<spade::render::LensSpec> pinhole_lens(uint32_t, uint32_t, double) { return std::unexpected(stub()); }
spade::Result<spade::render::LensSpec> fisheye_lens(uint32_t, uint32_t, double, const std::array<double, 4>&) { return std::unexpected(stub()); }
spade::Result<Views> render_views(const spade::render::RenderScene&, const spade::render::RayScene&, const Options&, std::span<const uint16_t>) {
    return std::unexpected(stub());
}
std::vector<uint8_t> interleave_rgb(std::span<const uint8_t>, uint32_t, uint32_t, uint32_t) { return {}; }
std::vector<uint8_t> instance_rgb(std::span<const uint16_t>) { return {}; }
std::vector<uint8_t> encode_png_rgb(uint32_t, uint32_t, std::span<const uint8_t>) { return {}; }
spade::Result<std::vector<std::string>> write_views(const Options&, const Views&) { return std::unexpected(stub()); }

}  // namespace camera_frames
