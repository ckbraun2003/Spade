#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <new>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include <glm/glm.hpp>

#include "core/error.hpp"

// ---------------------------------------------------------------------------
// THE AIRFLOW SOLVER'S GRID (physics/plans/2026-10-05-airflow-design.md §1.5,
// §2). A MAC (staggered) grid of cubic cells: each velocity component lives on
// the faces normal to its axis, the pressure at the cell centres. A domain is
// [0, nx dx] x [0, ny dx] x [0, nz dx], in metres, in the grid's own frame.
//
// The six domain faces are indexed 2 * axis + (0 for the minus face, 1 for the
// plus face) everywhere in this library.
// ---------------------------------------------------------------------------
namespace spade::physics::airflow {

enum Side : uint32_t { kXMinus = 0, kXPlus = 1, kYMinus = 2, kYPlus = 3, kZMinus = 4, kZPlus = 5 };

enum class FaceBc : uint8_t {
    open,      // the far field: an inflow where the ambient enters, otherwise a pressure outlet (p = 0)
    wall,      // no slip, at the face's wall velocity (a moving lid is a wall with a tangential velocity)
    periodic,  // wraps to the opposite face; both faces of an axis say so, or neither
};

struct DomainBc {
    std::array<FaceBc, 6> face{FaceBc::open, FaceBc::open, FaceBc::open,
                               FaceBc::open, FaceBc::open, FaceBc::open};
    std::array<glm::vec3, 6> wall_velocity{};  // per face, read where face[s] == wall, m/s
    glm::vec3 ambient{0.0f};                   // the far-field velocity at open faces, m/s
};

struct GridShape {
    uint32_t nx = 0;
    uint32_t ny = 0;
    uint32_t nz = 0;
    float dx = 0.0f;  // the cell size, metres
};

struct MacGrid {
    GridShape shape{};
    std::vector<float> u;  // x-faces, (nx + 1) ny nz, at (i dx, (j + 1/2) dx, (k + 1/2) dx)
    std::vector<float> v;  // y-faces, nx (ny + 1) nz
    std::vector<float> w;  // z-faces, nx ny (nz + 1)
    std::vector<float> p;  // cells, nx ny nz: gauge pressure, Pa
};

[[nodiscard]] inline std::size_t u_index(const GridShape& g, uint32_t i, uint32_t j, uint32_t k) noexcept {
    return i + std::size_t{g.nx + 1u} * (j + std::size_t{g.ny} * k);
}
[[nodiscard]] inline std::size_t v_index(const GridShape& g, uint32_t i, uint32_t j, uint32_t k) noexcept {
    return i + std::size_t{g.nx} * (j + std::size_t{g.ny + 1u} * k);
}
[[nodiscard]] inline std::size_t w_index(const GridShape& g, uint32_t i, uint32_t j, uint32_t k) noexcept {
    return i + std::size_t{g.nx} * (j + std::size_t{g.ny} * k);
}
[[nodiscard]] inline std::size_t cell_index(const GridShape& g, uint32_t i, uint32_t j, uint32_t k) noexcept {
    return i + std::size_t{g.nx} * (j + std::size_t{g.ny} * k);
}

// The k planes a component's samples span (k outermost in its array).
[[nodiscard]] inline uint32_t sample_planes(const GridShape& s, uint32_t component) noexcept {
    return component == 2 ? s.nz + 1u : s.nz;
}

// Every sample of velocity component `component` (0 u, 1 v, 2 w) in its k
// planes [k_begin, k_end), in array order (k outermost, i innermost), with its
// index and position: on its own axis at index * dx, on the other two at
// (index + 1/2) dx. The CPU step's threads take planes (worker_pool.hpp).
template <class F>
void for_each_sample_in(const GridShape& s, uint32_t component, uint32_t k_begin, uint32_t k_end, F&& f) {
    const uint32_t cx = component == 0 ? s.nx + 1u : s.nx;
    const uint32_t cy = component == 1 ? s.ny + 1u : s.ny;
    const float ox = component == 0 ? 0.0f : 0.5f;
    const float oy = component == 1 ? 0.0f : 0.5f;
    const float oz = component == 2 ? 0.0f : 0.5f;
    std::size_t idx = std::size_t{cx} * cy * k_begin;
    for (uint32_t k = k_begin; k < k_end; ++k) {
        for (uint32_t j = 0; j < cy; ++j) {
            for (uint32_t i = 0; i < cx; ++i, ++idx) {
                f(idx, glm::vec3((static_cast<float>(i) + ox) * s.dx, (static_cast<float>(j) + oy) * s.dx,
                                 (static_cast<float>(k) + oz) * s.dx));
            }
        }
    }
}

// Every sample of the component, every plane.
template <class F>
void for_each_sample(const GridShape& s, uint32_t component, F&& f) {
    for_each_sample_in(s, component, 0u, sample_planes(s, component), std::forward<F>(f));
}

// Per axis, whether the domain wraps there (make_grid refuses an unpaired
// periodic face, so the minus face decides).
[[nodiscard]] inline std::array<bool, 3> periodic_axes(const DomainBc& bc) noexcept {
    return {bc.face[kXMinus] == FaceBc::periodic, bc.face[kYMinus] == FaceBc::periodic,
            bc.face[kZMinus] == FaceBc::periodic};
}

// THE HOST'S STATED GRID CAP: 16 GiB. A POLICY number, not a measurement (Oversight's nod, O193), changed only by the lead or Cameron. It is what a
// CPU run and a case's validation hold a grid (with its scratch and solids) to, and it protects one thing: a case that cannot be run is refused BEFORE
// any work starts, by name, instead of failing on an allocation in the middle of a run. It is the same on every machine and every run, never the
// machine's free memory (which moves with whatever else is running), so a case file validates the same everywhere and a test that asserts a refusal
// never flakes. A run may only LOWER it (host_grid_cap, check_grid's user_cap). A grid that passes it whose memory the machine cannot give fails at
// allocation, by name (assign_zeroed: the grid and the bytes), never a crash. The device has its own, separate limit: the heap's budget less a reserve
// (compute/device_limits.hpp, vram_cap), with no fixed ceiling. The cell-count check below (2^31) is unchanged: it protects the 32-bit index.
inline constexpr uint64_t kGridByteCap = uint64_t{16} << 30;

// The cap a run is held to: the stated one, or `user_cap` when that is non-zero and lower. A run may only lower it.
[[nodiscard]] inline uint64_t host_grid_cap(uint64_t user_cap) noexcept {
    return (user_cap != 0 && user_cap < kGridByteCap) ? user_cap : kGridByteCap;
}

// The refusal for an allocation that failed (or, under ScopedAllocationLimit, would have): capacity_exceeded naming the stage, with the grid's shape, and the bytes.
namespace detail {
inline thread_local uint64_t g_allocation_limit = 0;  // the test seam: 0 is off
[[nodiscard]] inline Error out_of_memory(std::string_view stage, uint64_t bytes) {
    return Error{Code::capacity_exceeded, std::string(stage) + ": out of memory allocating " + std::to_string(bytes) + " bytes"};
}
[[nodiscard]] inline std::string shape_text(const GridShape& s) {
    return std::to_string(s.nx) + " x " + std::to_string(s.ny) + " x " + std::to_string(s.nz);
}
}  // namespace detail

// TEST SEAM: while one is alive on this thread, allocating() acts as a machine that refuses any allocation over `limit` bytes, so a test FORCES the
// failure at every entry point and never relies on memory running out. Production code never makes one.
class ScopedAllocationLimit {
public:
    explicit ScopedAllocationLimit(uint64_t limit) noexcept : previous_(detail::g_allocation_limit) { detail::g_allocation_limit = limit; }
    ~ScopedAllocationLimit() { detail::g_allocation_limit = previous_; }
    ScopedAllocationLimit(const ScopedAllocationLimit&) = delete;
    ScopedAllocationLimit& operator=(const ScopedAllocationLimit&) = delete;

private:
    uint64_t previous_;
};

// THE GUARD (the lead's ruling, O193: no CPU path may crash): every Result-returning function that allocates runs its body in this. A std::bad_alloc or
// std::length_error becomes capacity_exceeded naming `stage` (say what is being built and the grid's shape) and the `bytes` it was building; any other
// exception goes through (a bug is not an out-of-memory).
template <class T, class F>
[[nodiscard]] Result<T> allocating(std::string_view stage, uint64_t bytes, F&& body) {
    if (detail::g_allocation_limit != 0 && bytes > detail::g_allocation_limit) return std::unexpected(detail::out_of_memory(stage, bytes));
    try {
        return std::forward<F>(body)();
    } catch (const std::bad_alloc&) {
        return std::unexpected(detail::out_of_memory(stage, bytes));
    } catch (const std::length_error&) {
        return std::unexpected(detail::out_of_memory(stage, bytes));
    }
}

// Fills `v` with `count` zeroes. A failed allocation is a refusal by allocating(), naming `what` (the grid and the array) and the bytes, and leaves `v` as it
// was: never a throw out of a function that promises a Result (L6). A template over the vector so a test can force the failure with an allocator that
// refuses past a limit.
template <class Vec>
[[nodiscard]] Result<void> assign_zeroed(Vec& v, std::size_t count, std::string_view what) {
    return allocating<void>(what, uint64_t{count} * sizeof(typename Vec::value_type), [&]() -> Result<void> {
        v.assign(count, typename Vec::value_type{});
        return {};
    });
}

// The bytes of a grid's u, v, w and p, in 64 bits, for a shape check_shape()
// takes (past its cell cap the products can wrap).
[[nodiscard]] inline uint64_t grid_bytes(const GridShape& s) noexcept {
    const uint64_t nx = s.nx;
    const uint64_t ny = s.ny;
    const uint64_t nz = s.nz;
    const uint64_t floats = (nx + 1u) * ny * nz + nx * (ny + 1u) * nz + nx * ny * (nz + 1u) + nx * ny * nz;
    return floats * sizeof(float);
}

// Whether make_grid() takes this shape and these faces, with no allocation
// (but a refusal's message). Refused (invalid_argument, naming what): a zero
// dimension; dx not positive and finite; a periodic face whose opposite face
// is not periodic; a periodic axis of odd size other than 1, since red-black
// colouring needs an even ring. capacity_exceeded past 2^31 cells (naming
// the sizes; the count is checked, so no uint32 triple wraps it), or past
// kGridByteCap bytes (naming both). check_grid() (fluid_step.hpp) adds the
// domain and the step's scratch.
[[nodiscard]] Result<void> check_shape(const GridShape& shape, const DomainBc& bc);

// A grid at rest: every face and cell 0. check_shape()'s refusals, before
// anything is allocated.
[[nodiscard]] Result<MacGrid> make_grid(const GridShape& shape, const DomainBc& bc);

}  // namespace spade::physics::airflow
