#include "physics/airflow/mac_grid.hpp"

#include <cmath>
#include <string>

namespace spade::physics::airflow {

namespace {
constexpr const char* kSideNames[6] = {"x-", "x+", "y-", "y+", "z-", "z+"};
}  // namespace

Result<void> check_shape(const GridShape& shape, const DomainBc& bc) {
    if (shape.nx == 0 || shape.ny == 0 || shape.nz == 0) {
        return std::unexpected(Error{Code::invalid_argument, "airflow grid: every dimension must be at least 1"});
    }
    if (!(shape.dx > 0.0f) || !std::isfinite(shape.dx)) {
        return std::unexpected(Error{Code::invalid_argument, "airflow grid: dx must be positive and finite"});
    }
    // The cell count, multiplied with a check: three uint32 sizes can wrap 64
    // bits (2^31 per axis is 2^93), so the first product is bounded before
    // the second; past this every total of the shape fits 64 bits.
    constexpr uint64_t kMaxCells = uint64_t{1} << 31;
    if (const uint64_t xy = uint64_t{shape.nx} * shape.ny; xy > kMaxCells || xy * shape.nz > kMaxCells) {
        return std::unexpected(Error{Code::capacity_exceeded, "airflow grid: " + std::to_string(shape.nx) + " x " +
                                                                  std::to_string(shape.ny) + " x " +
                                                                  std::to_string(shape.nz) + " is more than 2^31 cells"});
    }
    if (const uint64_t bytes = grid_bytes(shape); bytes > kGridByteCap) {
        return std::unexpected(Error{Code::capacity_exceeded, "airflow grid: " + std::to_string(bytes) +
                                                                  " bytes of faces and cells, past the cap of " +
                                                                  std::to_string(kGridByteCap)});
    }
    const uint32_t size[3] = {shape.nx, shape.ny, shape.nz};
    for (uint32_t axis = 0; axis < 3; ++axis) {
        const bool lo = bc.face[2 * axis] == FaceBc::periodic;
        const bool hi = bc.face[2 * axis + 1] == FaceBc::periodic;
        if (lo != hi) {
            return std::unexpected(Error{Code::invalid_argument,
                                         std::string("airflow grid: face ") + kSideNames[2 * axis + (lo ? 1 : 0)] +
                                             " must be periodic like its opposite face"});
        }
        if (lo && size[axis] != 1 && size[axis] % 2 != 0) {
            return std::unexpected(Error{Code::invalid_argument,
                                         std::string("airflow grid: periodic axis ") + "xyz"[axis] +
                                             " must have an even size or size 1 (red-black colouring)"});
        }
    }
    return {};
}

Result<MacGrid> make_grid(const GridShape& shape, const DomainBc& bc) {
    if (Result<void> ok = check_shape(shape, bc); !ok) return std::unexpected(ok.error());
    MacGrid g;
    g.shape = shape;
    g.u.assign(std::size_t{shape.nx + 1u} * shape.ny * shape.nz, 0.0f);
    g.v.assign(std::size_t{shape.nx} * (shape.ny + 1u) * shape.nz, 0.0f);
    g.w.assign(std::size_t{shape.nx} * shape.ny * (shape.nz + 1u), 0.0f);
    g.p.assign(std::size_t{shape.nx} * shape.ny * shape.nz, 0.0f);
    return g;
}

}  // namespace spade::physics::airflow
