// The share of the kinetic energy of a block of velocities above a wavenumber: the grid-scale energy measure (ke_hi) the
// subgrid viscosity's film tests (test_airflow_sgs.cpp) and the web box's smoothness measurement (test_gpu_airflow_step.cpp)
// both use. Moved here unchanged from test_airflow_sgs.cpp so the two read the same number.
#pragma once

#include <array>
#include <cmath>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <numbers>
#include <vector>

namespace spade::test {

// The share of the kinetic energy of a block of cell-centre velocities (z, y, x, 3) above k = pi / (2 dx): Hann
// window on each axis, a direct DFT per axis (the block is about 56 x 64 x 56).
[[nodiscard]] inline double ke_share_above(const std::vector<float>& vel, uint32_t nz, uint32_t ny, uint32_t nx,
                                           double dx) {
    const std::size_t total = std::size_t{nz} * ny * nx;
    const auto hann = [](uint32_t n, uint32_t i) {
        return 0.5 - 0.5 * std::cos(2.0 * std::numbers::pi * i / static_cast<double>(n - 1u));
    };
    const auto freq = [&](uint32_t n, uint32_t m) {  // numpy's fftfreq, times 2 pi
        const int64_t mm =
            m < (n + 1u) / 2u ? static_cast<int64_t>(m) : static_cast<int64_t>(m) - static_cast<int64_t>(n);
        return 2.0 * std::numbers::pi * static_cast<double>(mm) / (static_cast<double>(n) * dx);
    };
    const auto table = [](uint32_t n) {
        std::vector<std::complex<double>> t(std::size_t{n} * n);
        for (uint32_t m = 0; m < n; ++m) {
            for (uint32_t i = 0; i < n; ++i) {
                const double a = -2.0 * std::numbers::pi * static_cast<double>(m) * i / static_cast<double>(n);
                t[std::size_t{m} * n + i] = {std::cos(a), std::sin(a)};
            }
        }
        return t;
    };
    const std::array<std::vector<std::complex<double>>, 3> tab = {table(nz), table(ny), table(nx)};
    const uint32_t dims[3] = {nz, ny, nx};
    std::vector<double> energy(total, 0.0);
    std::vector<std::complex<double>> a(total);
    std::vector<std::complex<double>> b(total);
    for (int c = 0; c < 3; ++c) {
        for (uint32_t z = 0; z < nz; ++z) {
            for (uint32_t y = 0; y < ny; ++y) {
                for (uint32_t x = 0; x < nx; ++x) {
                    const std::size_t at = (std::size_t{z} * ny + y) * nx + x;
                    a[at] = static_cast<double>(vel[at * 3u + static_cast<std::size_t>(c)]) * hann(nz, z) *
                            hann(ny, y) * hann(nx, x);
                }
            }
        }
        for (int axis = 0; axis < 3; ++axis) {
            const uint32_t n = dims[axis];
            const std::size_t stride = axis == 0 ? std::size_t{ny} * nx : (axis == 1 ? std::size_t{nx} : 1u);
            for (std::size_t base = 0; base < total; ++base) {
                if ((base / stride) % n != 0u) continue;  // the first sample of each line
                for (uint32_t m = 0; m < n; ++m) {
                    std::complex<double> acc(0.0, 0.0);
                    for (uint32_t i = 0; i < n; ++i) acc += tab[axis][std::size_t{m} * n + i] * a[base + i * stride];
                    b[base + m * stride] = acc;
                }
            }
            a.swap(b);
        }
        for (std::size_t at = 0; at < total; ++at) energy[at] += std::norm(a[at]);
    }
    double sum = 0.0;
    double high = 0.0;
    const double k_cut = std::numbers::pi / (2.0 * dx);
    for (uint32_t z = 0; z < nz; ++z) {
        const double kz = freq(nz, z);
        for (uint32_t y = 0; y < ny; ++y) {
            const double ky = freq(ny, y);
            for (uint32_t x = 0; x < nx; ++x) {
                const double kx = freq(nx, x);
                const std::size_t at = (std::size_t{z} * ny + y) * nx + x;
                sum += energy[at];
                if (std::sqrt(kx * kx + ky * ky + kz * kz) > k_cut) high += energy[at];
            }
        }
    }
    return high / sum;
}

}  // namespace spade::test
