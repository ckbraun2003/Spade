// ---------------------------------------------------------------------------
// test_gpu_fpv_trim.cpp -- the FPV film's hover trim (O158, L-S31): ONE controller run on the film box that finds the rotor speed at which the four lines
// plus the air's force on the frame carry the weight. Test code only (the controller is tests/fpv_trim.hpp, tested offline against fake plants).
//
//   GpuFpvTrim.HoverTrimOnTheFilmBox   opt-in: SPADE_FPV_TRIM_OUT=<file> (GPU, about 40 to 100 minutes; a gpu-class job, never in the gate)
//   GpuFpvTrim.TheObservingWrapperChangesNoBit   opt-in: SPADE_FPV_TRIM_WRAPPER=1 (GPU, about 2 minutes)
//
// THE RUN. LiveRun::start() on the OpenFrame-5F kit model in the film's box kD1FpvGrid (252 x 237 x 284 at 1.25 mm, held by d1_fpv_hold), with the rotors as
// lines and the frame as solids, exactly as the film builds it. The fluid maker is gpu_live_fluid()'s own work behind a thin LiveFluid that forwards every
// call unchanged and ALSO reads the device's wall_load() (the pressure force on the frame) and the lines' thrust. The control is the DUTY: each window
// the controller's next speed (fpv_trim::next_omega) becomes a throttle by bisection on the chain's own steady map (rotor_omega_for_duty), so the plant the
// controller sees includes the chain's spin-up lag, the film's own. The share is (the lines' thrust + the frame's air force, upward) over the weight.
//
// WHAT IT PRINTS: every window's commanded speed and share, the outcome (settled with its throttle, duty and speed, or UNSETTLED), the device memory in use
// sampled each window (its peak is the run's own measured VRAM, for the next ticket's --vram-gb) and the wall time. SPADE_FPV_TRIM_OUT receives the same
// lines as text, each written as it is said.
//
// THE WRAPPER CHECK. The wall_load read-back is a read, not a step: the second test runs 60 steps at the showcase trim through the plain gpu_live_fluid()
// maker and through the wrapper (reading wall_load every 25 steps) and asserts every step's line thrust is the same BITS. (wall_load itself is not in
// the plain path, so the comparison is on what the wrapper must not change.)
// ---------------------------------------------------------------------------

#include <gtest/gtest.h>

#include <algorithm>
#include <bit>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <memory>
#include <string>
#include <vector>

#include "analysis/detail/hover.hpp"
#include "analysis/live_run.hpp"
#include "compute/backend.hpp"
#include "compute/vulkan/context.hpp"
#include "d1_case.hpp"
#include "flow_still.hpp"
#include "fpv_trim.hpp"
#include "gpu_skip.hpp"
#include "openframe5f_model.hpp"
#include "phantom_case.hpp"
#include "physics/airflow/fluid_step.hpp"
#include "scene/part_file.hpp"
#include "scene/prop_line.hpp"
#include "spike/airflow/live_fluid.hpp"
#include "vehicles/propulsion_steady.hpp"

namespace {

namespace an = spade::analysis;
namespace af = spade::physics::airflow;
namespace ag = spade::airflow_gpu;
namespace ft = spade::test::fpv_trim;
namespace of = spade::testing::openframe5f;
using flow_still::env_cstr;

// LiveRun's fluid on the device (gpu_live_fluid()'s work), forwarding everything and exposing the device's wall_load().
class ObservedFluid final : public an::LiveFluid {
  public:
    explicit ObservedFluid(std::unique_ptr<ag::GpuLiveFluid> inner, ag::GpuFluid* device) noexcept : inner_(std::move(inner)), device_(device) {}
    [[nodiscard]] spade::Result<af::FluidHealth> step(std::span<const af::ActuatorDisc> discs) override { return inner_->step(discs); }
    [[nodiscard]] spade::Result<af::FluidHealth> step(std::span<const af::ActuatorDisc> discs, std::span<const af::LineRotor> lines) override {
        return inner_->step(discs, lines);
    }
    [[nodiscard]] spade::Result<void> publish(const af::PublishInput& in, spade::FlowVolume& out) override { return inner_->publish(in, out); }
    [[nodiscard]] spade::Result<void> read(af::MacGrid& g) const override { return inner_->read(g); }
    [[nodiscard]] double line_thrust() const override { return inner_->line_thrust(); }
    [[nodiscard]] spade::Result<af::WallLoad> wall_load() { return device_->wall_load(glm::vec3(0.0f)); }

  private:
    std::unique_ptr<ag::GpuLiveFluid> inner_;
    ag::GpuFluid* device_;  // owned by inner_
};

// The maker: GpuFluid::create() from the run's start, as gpu_live_fluid() does, with the device kept for wall_load(). `seen` receives the wrapper.
[[nodiscard]] an::LiveFluidMaker observed_maker(spade::compute::VulkanContext& ctx, ObservedFluid*& seen) {
    return [&ctx, &seen](const an::LiveFluidStart& s) -> spade::Result<std::unique_ptr<an::LiveFluid>> {
        auto config = s.config;
        config.sgs_cs = 0.17f;  // the film's subgrid constant, as flow_movie hands gpu_live_fluid
        std::optional<ag::LineSetup> line;
        if (s.lines != nullptr) {
            line = ag::LineSetup{.polar = &s.lines->polar, .elements = s.line_elements, .n_sub = s.lines->n_sub, .tip_correction = true,
                                 .binned = true, .rotors = s.line_rotors};
        }
        const ag::DyeSetup dye{af::DyeRelease{.rate = 0.0f}, af::TipRelease{.gain = 1.5f}};
        spade::Result<std::unique_ptr<ag::GpuFluid>> fluid =
            ag::GpuFluid::create(ctx, s.grid, s.domain, config, s.solids, 64u, line ? &*line : nullptr, &dye);
        if (!fluid) return std::unexpected(fluid.error());
        ag::GpuFluid* device = fluid->get();
        auto inner = std::make_unique<ag::GpuLiveFluid>(std::move(*fluid));
        auto wrapper = std::make_unique<ObservedFluid>(std::move(inner), device);
        seen = wrapper.get();
        return wrapper;
    };
}

struct Case {
    spade::testing::PhantomCase made;
    an::LiveLines lines;
};

[[nodiscard]] spade::Result<Case> film_case() {
    const af::GridShape grid = spade::testing::kD1FpvGrid;
    constexpr float kStepS = 25.0e-6f;
    const auto probe = spade::testing::still_air_case(of::data_dir() / of::kModelFile, "openframe5f", grid, kStepS, glm::vec3(0.0f));
    if (!probe) return std::unexpected(probe.error());
    float hub_mean = 0.0f;
    for (const auto& r : probe->model.rotors) hub_mean += r.local_pos.y / static_cast<float>(probe->model.rotors.size());
    hub_mean += probe->model.com_offset.y;  // the held point is the design origin
    auto made = spade::testing::still_air_case(of::data_dir() / of::kModelFile, "openframe5f", grid, kStepS, spade::testing::d1_fpv_hold(hub_mean));
    if (!made) return std::unexpected(made.error());
    std::ifstream in(of::data_dir() / of::kPropFile, std::ios::binary);
    const std::string text((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
    const auto part = spade::scene::prop_from_yaml(text);
    if (!part) return std::unexpected(part.error());
    const auto prop = spade::scene::line_propeller(*part);
    if (!prop) return std::unexpected(prop.error());
    Case c{std::move(*made), {}};
    c.lines.blade = prop->blade;
    c.lines.polar = prop->polar;
    c.lines.n_sub = 8;
    c.lines.kernel_cells = 0.0f;
    c.lines.fit_kernel_cells = prop->fit_kernel_cells;  // the declared kernel (2.97 cells, polar C), as flow_movie's lines.hpp does
    return c;
}

TEST(GpuFpvTrim, TheObservingWrapperChangesNoBit) {
    if (env_cstr("SPADE_FPV_TRIM_WRAPPER") == nullptr) GTEST_SKIP() << "opt-in: SPADE_FPV_TRIM_WRAPPER=1 (GPU, about 2 minutes)";
    if (const auto why = spade::testing::vulkan_skip_reason()) GTEST_SKIP() << *why;
    auto c = film_case();
    ASSERT_TRUE(c.has_value()) << c.error().context;
    auto ctx = spade::compute::VulkanContext::create(spade::compute::BackendDesc{.kind = spade::compute::BackendKind::vulkan});
    ASSERT_TRUE(ctx.has_value()) << ctx.error().context;
    constexpr int kSteps = 60;
    const auto usage_mib = [&]() { return static_cast<double>((*ctx)->memory_budget().usage) / (1024.0 * 1024.0); };
    const double baseline_mib = usage_mib();
    double peak_mib = baseline_mib;
    const an::Controls armed{true, c->made.trim.hover, 0.0f, 0.0f, 0.0f};
    std::vector<uint64_t> plain_bits;
    {
        const ag::DyeSetup dye{af::DyeRelease{.rate = 0.0f}, af::TipRelease{.gain = 1.5f}};
        auto run = an::LiveRun::start(c->made.resolved, c->made.model, ag::gpu_live_fluid(**ctx, 64u, 0.17f, dye), &c->lines);
        ASSERT_TRUE(run.has_value()) << run.error().context;
        auto t10 = std::chrono::steady_clock::now();
        for (int i = 0; i < kSteps; ++i) {
            ASSERT_TRUE(run->step(armed).has_value());
            plain_bits.push_back(std::bit_cast<uint64_t>(run->line_thrust()));
            if (i + 1 == 10) t10 = std::chrono::steady_clock::now();
        }
        const double seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - t10).count();
        std::printf("[ fpv trim ] plain path: %.4f s a step over steps 11 to %d (%.2f s)\n", seconds / (kSteps - 10), kSteps, seconds);
    }
    std::vector<uint64_t> wrapped_bits;
    {
        ObservedFluid* seen = nullptr;
        auto run = an::LiveRun::start(c->made.resolved, c->made.model, observed_maker(**ctx, seen), &c->lines);
        ASSERT_TRUE(run.has_value()) << run.error().context;
        ASSERT_NE(seen, nullptr);
        peak_mib = std::max(peak_mib, usage_mib());
        for (int i = 0; i < kSteps; ++i) {
            ASSERT_TRUE(run->step(armed).has_value());
            wrapped_bits.push_back(std::bit_cast<uint64_t>(run->line_thrust()));
            peak_mib = std::max(peak_mib, usage_mib());
            if ((i + 1) % 25 == 0) {
                const auto wl = seen->wall_load();
                ASSERT_TRUE(wl.has_value()) << wl.error().context;
                std::printf("[ fpv trim ] step %d: frame force (%.5f, %.5f, %.5f) N\n", i + 1, static_cast<double>(wl->force.x),
                            static_cast<double>(wl->force.y), static_cast<double>(wl->force.z));
            }
        }
    }
    std::printf("[ fpv trim ] device memory in use: %.0f MiB before, peak %.0f MiB (%.3f GiB) over the wrapped run's start and %d steps; the run's own cost %.3f GiB\n",
                baseline_mib, peak_mib, peak_mib / 1024.0, kSteps, (peak_mib - baseline_mib) / 1024.0);
    for (std::size_t i = 0; i < wrapped_bits.size(); ++i) {
        std::printf("[ fpv trim ] line thrust bits, step %zu: 0x%016llx\n", i + 1, static_cast<unsigned long long>(wrapped_bits[i]));
    }
    ASSERT_EQ(plain_bits.size(), wrapped_bits.size());
    for (std::size_t i = 0; i < plain_bits.size(); ++i) {
        EXPECT_EQ(plain_bits[i], wrapped_bits[i]) << "step " << i + 1 << ": the wall_load read-back changed the lines' thrust";
    }
}

TEST(GpuFpvTrim, HoverTrimOnTheFilmBox) {
    const char* out_path = env_cstr("SPADE_FPV_TRIM_OUT");
    if (out_path == nullptr) GTEST_SKIP() << "opt-in: SPADE_FPV_TRIM_OUT=<file> (GPU, 40 to 100 minutes)";
    if (const auto why = spade::testing::vulkan_skip_reason()) GTEST_SKIP() << *why;
    auto c = film_case();
    ASSERT_TRUE(c.has_value()) << c.error().context;
    const an::AnalysisCase& spec = c->made.resolved.spec;
    ASSERT_TRUE(c->made.model.propulsion.has_value());
    const auto& chain = *c->made.model.propulsion;
    const double idle = static_cast<double>(spec.power.idle.value_or(an::kDefaultIdleDuty));
    const double soc = static_cast<double>(spec.power.state_of_charge.value_or(an::kDefaultStateOfCharge));
    const double density = static_cast<double>(spec.fluid.density);
    const double weight = an::detail::world_weight(c->made.model);
    ASSERT_GT(weight, 0.0);

    const auto omega_of_duty = [&](double duty) {
        const auto cmd = spade::vehicles::rotor_omega_for_duty(chain, duty, density, soc);
        return cmd ? static_cast<double>(cmd->omega_cmd) : std::numeric_limits<double>::quiet_NaN();
    };
    // the duty whose steady commanded speed is omega (the map is monotone in duty): bisection to 1e-10 of the duty range
    const auto throttle_for = [&](double omega) {
        double lo = idle;
        double hi = 1.0;
        for (int i = 0; i < 80; ++i) {
            const double mid = 0.5 * (lo + hi);
            (omega_of_duty(mid) < omega ? lo : hi) = mid;
        }
        return static_cast<float>(((0.5 * (lo + hi)) - idle) / (1.0 - idle));
    };

    auto ctx = spade::compute::VulkanContext::create(spade::compute::BackendDesc{.kind = spade::compute::BackendKind::vulkan});
    ASSERT_TRUE(ctx.has_value()) << ctx.error().context;
    const double baseline_mib = static_cast<double>((*ctx)->memory_budget().usage) / (1024.0 * 1024.0);
    ObservedFluid* seen = nullptr;
    auto run = an::LiveRun::start(c->made.resolved, c->made.model, observed_maker(**ctx, seen), &c->lines);
    ASSERT_TRUE(run.has_value()) << run.error().context;
    ASSERT_NE(seen, nullptr);

    std::ofstream out(out_path, std::ios::binary | std::ios::trunc);
    ASSERT_TRUE(out.good()) << "cannot write SPADE_FPV_TRIM_OUT " << out_path;
    const auto say = [&](const std::string& line) {
        std::printf("[ fpv trim ] %s\n", line.c_str());
        std::fflush(stdout);
        out << line << '\n';
        out.flush();  // every line on disk as it is said: a run that stops an hour in keeps its windows
    };
    char buf[512];
    std::snprintf(buf, sizeof buf, "weight %.5f N; gain %.2f, windows of %zu steps, floor %zu windows, cap %zu steps; device memory %.0f MiB before the run's start",
                  weight, ft::Config{}.gain, ft::Config{}.window_steps, ft::Config{}.rule.min_windows, ft::Config{}.cap_steps, baseline_mib);
    say(buf);

    double peak_mib = 0.0;
    uint64_t step_count = 0;
    const auto t0 = std::chrono::steady_clock::now();
    bool step_failed = false;
    std::string step_error;
    const auto plant = [&](double omega, std::size_t steps) -> double {
        const an::Controls controls{true, throttle_for(omega), 0.0f, 0.0f, 0.0f};
        double thrust_sum = 0.0;
        double force_sum = 0.0;
        std::size_t force_n = 0;
        for (std::size_t i = 0; i < steps; ++i) {
            const auto s = run->step(controls);
            if (!s) {
                step_failed = true;
                step_error = s.error().context;
                return std::numeric_limits<double>::quiet_NaN();
            }
            ++step_count;
            thrust_sum += run->line_thrust();
            if ((i + 1) % 25 == 0) {
                const auto wl = seen->wall_load();
                if (!wl) {
                    step_failed = true;
                    step_error = wl.error().context;
                    return std::numeric_limits<double>::quiet_NaN();
                }
                force_sum += static_cast<double>(wl->force.y);
                ++force_n;
            }
        }
        peak_mib = std::max(peak_mib, static_cast<double>((*ctx)->memory_budget().usage) / (1024.0 * 1024.0));
        const double thrust = thrust_sum / static_cast<double>(steps);
        const double force = force_n != 0 ? force_sum / static_cast<double>(force_n) : 0.0;
        const double share = (thrust + force) / weight;
        std::snprintf(buf, sizeof buf, "window %llu steps: omega_cmd %.3f rad/s (%.1f rpm), throttle %.5f, lines %.5f N, frame %.5f N, share %.5f; wall %.0f s",
                      static_cast<unsigned long long>(step_count), omega, omega * 60.0 / (2.0 * 3.14159265358979), static_cast<double>(controls.throttle),
                      thrust, force, share, std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count());
        say(buf);
        return share;
    };

    // start at the showcase trim's speed (the chain's hover for the weight over four, no frame force)
    const double omega0 = omega_of_duty(idle + (1.0 - idle) * static_cast<double>(c->made.trim.hover));
    ASSERT_TRUE(std::isfinite(omega0));
    const ft::Result result = ft::run(plant, omega0);
    ASSERT_FALSE(step_failed) << step_error;
    const double device_peak = peak_mib;
    if (result.outcome == ft::Outcome::settled) {
        const float throttle = throttle_for(result.omega);
        std::snprintf(buf, sizeof buf, "SETTLED after %zu steps (%zu windows): omega_cmd %.3f rad/s (%.1f rpm), throttle %.5f, duty %.5f, share %.5f",
                      result.steps, result.windows.size(), result.omega, result.omega * 60.0 / (2.0 * 3.14159265358979), static_cast<double>(throttle),
                      idle + (1.0 - idle) * static_cast<double>(throttle), result.windows.back().share);
    } else {
        std::snprintf(buf, sizeof buf, "UNSETTLED after %zu steps (%zu windows, cap %zu): NOT to be used for the film; the last window's omega_cmd %.3f rad/s, share %.5f",
                      result.steps, result.windows.size(), ft::Config{}.cap_steps, result.windows.empty() ? 0.0 : result.windows.back().omega,
                      result.windows.empty() ? 0.0 : result.windows.back().share);
    }
    say(buf);
    std::snprintf(buf, sizeof buf, "device memory in use: baseline %.0f MiB, peak over the windows %.0f MiB (%.3f GiB); wall %.0f s", baseline_mib, device_peak,
                  device_peak / 1024.0, std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count());
    say(buf);
    EXPECT_NE(result.outcome, ft::Outcome::refused) << "the plant gave a non-finite share";
}

}  // namespace
