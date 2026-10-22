#include "physics/airflow/regions.hpp"

#include <algorithm>
#include <cmath>

#include "physics/airflow/reduce.hpp"

namespace spade::physics::airflow {

namespace {

// The face of cell (i, j, k) on side `side` (2 axis + 0 low, 1 high): its
// component and index.
struct FaceRef {
    uint32_t c;
    std::size_t idx;
};

[[nodiscard]] FaceRef face_of(const GridShape& s, uint32_t i, uint32_t j, uint32_t k, uint32_t side) noexcept {
    const uint32_t a = side / 2u;
    const uint32_t up = side % 2u;
    switch (a) {
        case 0: return {0u, u_index(s, i + up, j, k)};
        case 1: return {1u, v_index(s, i, j + up, k)};
        default: return {2u, w_index(s, i, j, k + up)};
    }
}

}  // namespace

uint64_t regions_bytes(const GridShape& s) noexcept {
    const uint64_t cells = uint64_t{s.nx} * s.ny * s.nz;
    const uint64_t tile = uint64_t{kReduceTile} * kReduceTile * kReduceTile;
    return cells * (sizeof(uint32_t) + sizeof(uint8_t) + sizeof(uint32_t)) + tile * sizeof(uint32_t);
}

void label_regions(FluidRegions& r, const GridShape& s, std::array<bool, 3> periodic, const SolidField& f) {
    const std::size_t cells = std::size_t{s.nx} * s.ny * s.nz;
    r.label.assign(cells, kNoRegion);
    r.solid.assign(cells, 1u);
    r.cells.clear();
    r.first.clear();
    r.count = 0;
    const uint32_t n[3] = {s.nx, s.ny, s.nz};
    const auto fluid = [&](uint32_t i, uint32_t j, uint32_t k) {
        for (uint32_t side = 0; side < 6; ++side) {
            const FaceRef fr = face_of(s, i, j, k, side);
            if (f.theta[fr.c][fr.idx] > 0.0f) return true;
        }
        return false;
    };
    std::vector<uint32_t> queue;
    queue.reserve(cells);
    for (uint32_t k = 0; k < s.nz; ++k) {
        for (uint32_t j = 0; j < s.ny; ++j) {
            for (uint32_t i = 0; i < s.nx; ++i) {
                const std::size_t start = cell_index(s, i, j, k);
                if (r.label[start] != kNoRegion || !fluid(i, j, k)) continue;
                const uint32_t id = r.count++;
                r.cells.push_back(0u);
                r.first.push_back(static_cast<uint32_t>(start));
                queue.clear();
                queue.push_back(static_cast<uint32_t>(start));
                r.label[start] = id;
                for (std::size_t head = 0; head < queue.size(); ++head) {
                    const uint32_t c = queue[head];
                    ++r.cells[id];
                    r.solid[c] = 0u;
                    const uint32_t ci = c % s.nx;
                    const uint32_t cj = (c / s.nx) % s.ny;
                    const uint32_t ck = c / (s.nx * s.ny);
                    const uint32_t q[3] = {ci, cj, ck};
                    for (uint32_t side = 0; side < 6; ++side) {
                        const uint32_t a = side / 2u;
                        const bool up = side % 2u == 1u;
                        const FaceRef fr = face_of(s, ci, cj, ck, side);
                        if (!(f.theta[fr.c][fr.idx] > 0.0f)) continue;
                        uint32_t nb[3] = {ci, cj, ck};
                        if (up ? q[a] + 1u < n[a] : q[a] > 0u) {
                            nb[a] = up ? q[a] + 1u : q[a] - 1u;
                        } else if (periodic[a] && n[a] > 1u) {
                            nb[a] = up ? 0u : n[a] - 1u;
                        } else {
                            continue;  // a domain face: no neighbour
                        }
                        const std::size_t to = cell_index(s, nb[0], nb[1], nb[2]);
                        if (r.label[to] != kNoRegion) continue;
                        r.label[to] = id;
                        queue.push_back(static_cast<uint32_t>(to));
                    }
                }
            }
        }
    }
    r.closed.assign(r.count, 1u);
    r.faces.assign(r.count, 0u);
    r.flux.assign(r.count, 0.0);
    r.magnitude.assign(r.count, 0.0);
    r.total.assign(r.count, 0.0f);
    r.block.assign(r.count, 0.0f);
    r.in_block.assign(r.count, 0u);
    r.tile.assign(r.count, 0.0f);
    r.mark.assign(r.count, 0u);
    r.touched.assign(std::size_t{kReduceTile} * kReduceTile * kReduceTile, 0u);
}

RegionFault check_regions(FluidRegions& r, const GridShape& s, const DomainBc& bc, const SolidField& f) noexcept {
    std::fill(r.closed.begin(), r.closed.end(), uint8_t{1});
    std::fill(r.faces.begin(), r.faces.end(), uint8_t{0});
    std::fill(r.flux.begin(), r.flux.end(), 0.0);
    std::fill(r.magnitude.begin(), r.magnitude.end(), 0.0);
    const uint32_t n[3] = {s.nx, s.ny, s.nz};
    const double dx2 = static_cast<double>(s.dx) * static_cast<double>(s.dx);
    for (uint32_t side = 0; side < 6; ++side) {
        const FaceKind kind = face_kind(bc, static_cast<Side>(side));
        if (kind == FaceKind::periodic) continue;
        const uint32_t a = side / 2u;
        const bool up = side % 2u == 1u;
        const uint32_t t1 = a == 0 ? 1u : 0u;
        const uint32_t t2 = a == 2 ? 1u : 2u;
        const double normal = kind == FaceKind::fixed_velocity ? fixed_normal_velocity(bc, static_cast<Side>(side)) : 0.0;
        for (uint32_t v = 0; v < n[t2]; ++v) {
            for (uint32_t u = 0; u < n[t1]; ++u) {
                uint32_t q[3];
                q[a] = up ? n[a] - 1u : 0u;
                q[t1] = u;
                q[t2] = v;
                const FaceRef fr = face_of(s, q[0], q[1], q[2], side);
                const float theta = f.theta[fr.c][fr.idx];
                if (!(theta > 0.0f)) continue;
                const uint32_t id = r.label[cell_index(s, q[0], q[1], q[2])];
                if (id == kNoRegion) continue;
                if (kind == FaceKind::pressure_outlet) {
                    r.closed[id] = 0u;
                    continue;
                }
                if (normal == 0.0) continue;
                const double outward = (up ? normal : -normal) * static_cast<double>(theta) * dx2;
                r.flux[id] += outward;
                r.magnitude[id] += std::fabs(outward);
                r.faces[id] = static_cast<uint8_t>(r.faces[id] | (1u << side));
            }
        }
    }
    for (uint32_t id = 0; id < r.count; ++id) {
        if (r.closed[id] != 0u && std::fabs(r.flux[id]) > 1.0e-6 * r.magnitude[id]) {
            return RegionFault{id, r.first[id], r.faces[id]};
        }
    }
    return {};
}

void remove_region_means(std::span<float> x, FluidRegions& r, const GridShape& s) noexcept {
    std::fill(r.total.begin(), r.total.end(), 0.0f);
    std::fill(r.block.begin(), r.block.end(), 0.0f);
    std::fill(r.in_block.begin(), r.in_block.end(), uint8_t{0});
    // The block's sums into the totals, for each region the block met.
    const auto end_block = [&r]() noexcept {
        for (uint32_t id = 0; id < r.count; ++id) {
            if (r.in_block[id] == 0u) continue;
            r.total[id] += r.block[id];
            r.block[id] = 0.0f;
            r.in_block[id] = 0u;
        }
    };
    uint32_t tiles_in_block = 0;
    for (uint32_t tk = 0; tk < s.nz; tk += kReduceTile) {
        const uint32_t k_end = std::min(tk + kReduceTile, s.nz);
        for (uint32_t tj = 0; tj < s.ny; tj += kReduceTile) {
            const uint32_t j_end = std::min(tj + kReduceTile, s.ny);
            for (uint32_t ti = 0; ti < s.nx; ti += kReduceTile) {
                const uint32_t i_end = std::min(ti + kReduceTile, s.nx);
                std::size_t met = 0;
                for (uint32_t k = tk; k < k_end; ++k) {
                    for (uint32_t j = tj; j < j_end; ++j) {
                        for (uint32_t i = ti; i < i_end; ++i) {
                            const std::size_t c = cell_index(s, i, j, k);
                            const uint32_t id = r.label[c];
                            if (id == kNoRegion || r.closed[id] == 0u) continue;
                            if (r.mark[id] == 0u) {
                                r.mark[id] = 1u;
                                r.touched[met++] = id;
                            }
                            r.tile[id] += x[c];
                        }
                    }
                }
                for (std::size_t m = 0; m < met; ++m) {
                    const uint32_t id = r.touched[m];
                    r.block[id] += r.tile[id];
                    r.in_block[id] = 1u;
                    r.tile[id] = 0.0f;
                    r.mark[id] = 0u;
                }
                if (++tiles_in_block == kReduceBlockTiles) {
                    end_block();
                    tiles_in_block = 0;
                }
            }
        }
    }
    end_block();  // the last block, holding the rest
    for (uint32_t id = 0; id < r.count; ++id) {
        if (r.closed[id] == 0u) continue;
        r.total[id] *= static_cast<float>(1.0 / static_cast<double>(r.cells[id]));  // the host's reciprocal
    }
    for (std::size_t c = 0; c < x.size(); ++c) {
        const uint32_t id = r.label[c];
        if (id == kNoRegion || r.closed[id] == 0u) continue;
        x[c] -= r.total[id];
    }
}

std::string describe_regions(const FluidRegions& r) {
    uint32_t closed = 0;
    for (const uint8_t c : r.closed) closed += c != 0u ? 1u : 0u;
    return "fluid regions: " + std::to_string(r.count) + " (" + std::to_string(r.count - closed) +
           " with an outlet, " + std::to_string(closed) + " closed)";
}

}  // namespace spade::physics::airflow
