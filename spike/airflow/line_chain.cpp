#include "spike/airflow/line_chain.hpp"

#include <algorithm>
#include <cmath>
#include <string>

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
    out.tip.assign(kTipHeaderWords, 0.0f);
    const af::LineTipParams tip_params;
    for (const af::LineRotor& rotor : rotors) {
        const std::vector<af::LineElement> elements = af::line_elements(rotor, shape.dx, h, n_sub);
        out.per_rotor.push_back(static_cast<uint32_t>(elements.size()));
        const std::size_t first = out.elems.size();
        std::size_t tip_per = 0;  // this rotor's blade size if it is tip-corrected (its groups' first number below)
        const uint32_t tip_first_group = static_cast<uint32_t>((out.tip.size() - kTipHeaderWords) / kTipStride);
        if (rotor.tip_correction && rotor.omega > 0.0f && !elements.empty()) {
            const af::LineTipTables tb = af::line_tip_tables(rotor, elements, shape.dx, tip_params);
            if (tb.per == 0 || tb.per > kTipMaxPer || tb.th.size() > kTipMaxSegments + 1 ||
                tb.reach > static_cast<int>(kTipMaxReach)) {
                return std::unexpected(Error{Code::invalid_argument,
                                             "line: the tip correction takes at most " + std::to_string(kTipMaxPer) +
                                                 " elements a blade, " + std::to_string(kTipMaxSegments) +
                                                 " chords and a span filter " + std::to_string(kTipMaxReach) +
                                                 " elements wide (this rotor: " + std::to_string(tb.per) + ", " +
                                                 std::to_string(tb.th.size() - 1) + ", " +
                                                 std::to_string(tb.reach) + ")"});
            }
            tip_per = tb.per;
            const uint32_t base_element = static_cast<uint32_t>(first / kLineStride);
            for (std::size_t g0 = 0; g0 + tb.per <= elements.size(); g0 += tb.per) {
                const af::LineElement& e0 = elements[g0];
                const std::size_t at = out.tip.size();
                out.tip.resize(at + kTipStride, 0.0f);
                float* g = out.tip.data() + at;
                const glm::vec3 d = (e0.pos - rotor.centre) / e0.r;
                g[0] = static_cast<float>(base_element + g0);
                g[1] = static_cast<float>(tb.per);
                g[2] = static_cast<float>(tip_params.iterations);
                g[3] = static_cast<float>(tb.th.size() - 1);
                g[4] = static_cast<float>(tb.reach);
                g[5] = rotor.spin;
                g[6] = e0.dr;
                g[7] = e0.r - 0.5f * e0.dr;
                g[8] = tb.eps_m;
                g[9] = tb.kern_sum;
                g[10] = tb.relax;
                g[11] = rotor.omega;
                g[12] = rho;
                for (uint32_t a = 0; a < 3; ++a) {
                    const glm::length_t i = static_cast<glm::length_t>(a);
                    g[16 + a] = rotor.centre[i];
                    g[19 + a] = d[i];
                    g[22 + a] = e0.motion[i];
                    g[25 + a] = rotor.axis[i];
                }
                for (std::size_t j = 0; j < tb.th.size(); ++j) {
                    g[32 + j] = tb.th[j];
                    g[64 + j] = tb.ct[j];
                    g[96 + j] = tb.st[j];
                }
                for (std::size_t m = 0; m < tb.kern.size(); ++m) g[128 + m] = tb.kern[m];
            }
        }
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
        // Each element of a whole blade knows its group (word 14: the group's number + 1) and its place in it (word 15).
        for (std::size_t e = 0; tip_per > 0 && e / tip_per < (elements.size() / tip_per); ++e) {
            float* r = out.elems.data() + first + e * kLineStride;
            r[14] = static_cast<float>(tip_first_group + e / tip_per + 1u);
            r[15] = static_cast<float>(e % tip_per);
        }
    }
    out.tip[0] = static_cast<float>((out.tip.size() - kTipHeaderWords) / kTipStride);
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
