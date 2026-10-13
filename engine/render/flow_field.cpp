#include "render/flow_field.hpp"

#include <cmath>
#include <cstdint>
#include <limits>
#include <string>

#include "render/flow_sample.hpp"

namespace spade::render {

FieldPalette field_palette_of(ColourMapId id) {
    const auto& table = colour_map_table(id);
    FieldPalette palette;
    palette.stops.reserve(table.size());
    for (const glm::vec3& c : table) {
        palette.stops.push_back(c * 255.0f);
    }
    return palette;
}

Result<FieldLayer> slice_flow_volume(const FlowVolume& volume, FlowQuantity quantity, const FieldLayer& geometry) {
    const auto refuse = [](const std::string& why) {
        return std::unexpected(Error{Code::invalid_argument, "slice_flow_volume: " + why});
    };
    if (Result<void> valid = validate_flow_volume(volume); !valid) {
        return refuse(valid.error().context);
    }
    const uint64_t cells = uint64_t{geometry.cells_u} * geometry.cells_v;
    if (cells > (uint64_t{1} << 24)) {
        return refuse(std::to_string(geometry.cells_u) + " x " + std::to_string(geometry.cells_v) +
                      " cells is past 2^24");
    }
    FieldLayer out = geometry;
    out.values.assign(static_cast<std::size_t>(cells), std::numeric_limits<float>::quiet_NaN());
    const FlowSampler sampler = make_flow_sampler(volume);
    for (uint32_t j = 0; j < out.cells_v; ++j) {
        for (uint32_t i = 0; i < out.cells_u; ++i) {
            const FlowValue v = sample_quantity(sampler, quantity, field_cell_center(out, i, j));
            if (!v.no_data) {
                out.values[static_cast<std::size_t>(j) * out.cells_u + i] = v.value;  // no-data stays NaN
            }
        }
    }
    if (Result<void> valid = validate_field_layer(out); !valid) {
        return refuse(valid.error().context);
    }
    return out;
}

Result<void> validate_flow_surface_colouring(const FlowSurfaceColouring& colouring) {
    const auto refuse = [](const std::string& why) {
        return std::unexpected(Error{Code::invalid_argument, "flow surface colouring: " + why});
    };
    if (colouring.volume == nullptr) {
        return refuse("it has no volume");
    }
    if (Result<void> valid = validate_flow_volume(*colouring.volume); !valid) {
        return refuse(valid.error().context);
    }
    if (Result<void> valid = validate_transfer_function(colouring.colours); !valid) {
        return refuse(valid.error().context);
    }
    if (!(std::isfinite(colouring.offset_cells) && colouring.offset_cells >= 0.0f &&
          colouring.offset_cells <= kMaxSurfaceOffsetCells)) {
        return refuse("its offset of " + std::to_string(colouring.offset_cells) + " cells is not finite in [0, " +
                      std::to_string(kMaxSurfaceOffsetCells) + "]");
    }
    if (colouring.extrapolate && !(colouring.offset_cells > 0.0f)) {
        return refuse("it extrapolates with no offset to extrapolate from");
    }
    return {};
}

Result<TransferFunction> cp_colours(float q_ref, float p_ref, float cp_min, float cp_max, ColourMapId map) {
    const auto refuse = [](const std::string& why) {
        return std::unexpected(Error{Code::invalid_argument, "C_p colours: " + why});
    };
    if (!(std::isfinite(q_ref) && q_ref > 0.0f)) {
        return refuse("q_ref " + std::to_string(q_ref) + " Pa is not finite and > 0");
    }
    if (!std::isfinite(p_ref)) {
        return refuse("p_ref is not finite");
    }
    if (!(std::isfinite(cp_min) && std::isfinite(cp_max) && cp_min < cp_max)) {
        return refuse("the C_p range is not finite with min < max");
    }
    TransferFunction tf;
    tf.quantity = FlowQuantity::pressure;
    tf.colours = map;
    tf.range_min = p_ref + cp_min * q_ref;
    tf.range_max = p_ref + cp_max * q_ref;
    if (!(std::isfinite(tf.range_min) && std::isfinite(tf.range_max) && tf.range_min < tf.range_max)) {
        return refuse("its pressure range collapses in float: q_ref " + std::to_string(q_ref) +
                      " Pa is too small beside p_ref " + std::to_string(p_ref) + " Pa");
    }
    return tf;
}

}  // namespace spade::render
