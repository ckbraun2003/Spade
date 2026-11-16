#include "spike/airflow/line_chain.hpp"

#include <algorithm>
#include <cmath>

#include "core/detail/flush_subnormals.hpp"

namespace spade::airflow_gpu {

namespace af = spade::physics::airflow;

Result<LineRecords> line_records(std::span<const af::LineRotor> rotors, const af::GridShape& shape, float rho, float h,
                                 uint32_t n_sub) {
    for (const af::LineRotor& rotor : rotors) {
        if (Result<void> ok = af::check_line(rotor, shape, rho, h, n_sub); !ok) return std::unexpected(ok.error());
    }
    // The CPU computes these taps, positions and products under spread_line()'s flush scope: so does the host.
    const spade::math::ScopedFlushSubnormals flush;
    LineRecords out;
    out.per_rotor.reserve(rotors.size());
    const float dx3 = shape.dx * shape.dx * shape.dx;
    const float share = 1.0f / static_cast<float>(n_sub);
    const float to_dv = h / (rho * dx3);
    for (const af::LineRotor& rotor : rotors) {
        const std::vector<af::LineElement> elements = af::line_elements(rotor, shape.dx, h, n_sub);
        out.per_rotor.push_back(static_cast<uint32_t>(elements.size()));
        const std::size_t first = out.elems.size();
        out.elems.resize(first + elements.size() * kLineStride, 0.0f);
        for (std::size_t e = 0; e < elements.size(); ++e) {
            const af::LineElement& el = elements[e];
            float* r = out.elems.data() + first + e * kLineStride;
            for (uint32_t a = 0; a < 3; ++a) {
                r[a] = rotor.axis[static_cast<glm::length_t>(a)];
                r[3 + a] = el.motion[static_cast<glm::length_t>(a)];
            }
            r[6] = el.chord;
            r[7] = el.sin_beta;
            r[8] = el.cos_beta;
            r[9] = el.r;
            r[10] = el.dr * share;
            r[11] = rotor.omega;
            r[12] = to_dv;
            r[13] = rho;
            for (uint32_t c = 0; c < 3; ++c) {
                const af::LineStencil st = af::line_stencil(shape, c, el.pos, rotor.eps_cells);
                float* comp = r + kLineHeaderWords + c * kLineComponentWords;
                for (uint32_t a = 0; a < 3; ++a) {
                    const af::LineAxisTaps& ax = st.ax[a];
                    comp[a] = static_cast<float>(ax.lo);
                    comp[3 + a] = static_cast<float>(ax.n);
                    for (int t = 0; t < ax.n; ++t) comp[7 + kLineTaps * a + static_cast<uint32_t>(t)] = ax.w[t];
                }
                comp[6] = st.sum;
            }
        }
    }
    return out;
}

std::vector<float> polar_records(const af::PolarTable& polar) {
    const std::size_t n = polar.key.size();
    std::vector<float> out;
    out.reserve(1 + 3 * n);
    out.push_back(static_cast<float>(n));
    out.insert(out.end(), polar.key.begin(), polar.key.end());
    out.insert(out.end(), polar.cl.begin(), polar.cl.end());
    out.insert(out.end(), polar.cd.begin(), polar.cd.end());
    return out;
}

std::vector<af::LineReport> line_reports(const LineRecords& records, std::span<const af::LineRotor> rotors,
                                         std::span<const float> force) {
    std::vector<af::LineReport> out;
    std::size_t element = 0;
    for (std::size_t k = 0; k < rotors.size() && k < records.per_rotor.size(); ++k) {
        double thrust = 0.0;
        double torque = 0.0;
        for (uint32_t i = 0; i < records.per_rotor[k]; ++i, ++element) {
            const float* r = records.elems.data() + element * kLineStride;
            const float* f = force.data() + element * kLineForceStride;
            const float drs = r[10];
            thrust += static_cast<double>(f[3]) * static_cast<double>(drs);
            torque += static_cast<double>(f[4]) * static_cast<double>(r[9]) * static_cast<double>(drs);
        }
        af::LineReport rep;
        rep.thrust = static_cast<float>(thrust);
        rep.torque = static_cast<float>(torque);
        rep.power = static_cast<float>(torque * static_cast<double>(rotors[k].omega));
        out.push_back(rep);
    }
    return out;
}

}  // namespace spade::airflow_gpu
