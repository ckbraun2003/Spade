// Manual-only benchmark harness for the CPU rasterizer's fill cost --
// base-package phase, workstream W6c (MV3's never-pinned measured throughput
// table), 2026-09-08.
//
// Same policy as bench_core.cpp / bench_sim.cpp: spade_bench is a standalone
// executable, NOT registered with ctest, run by hand. Correctness of
// render() is covered by spade/tests/test_render_raster.cpp; this file is for
// "how fast".
//
// ---------------------------------------------------------------------------
// WHY THIS FILE EXISTS, AND WHAT DECISION IT SERVES
// ---------------------------------------------------------------------------
// The editor proposes raising its render-scale cap to device pixels (MV3 /
// ruling BP11: "HD scope is sharp only"). That multiplies the rendered pixel
// count. The editor realm measured its own half and found the PRESENT path
// (Qt's blit) tracks TARGET pixels, which the cap raise does not change:
// present cost came back neutral at 1.03x. So the entire cost of the change
// lands HERE, on the render side, and this file is the number the decision
// turns on.
//
// THE RATIO THAT MATTERS IS 1.68x, NOT 2.25x. MV3 s.8 priced exactly one
// 1892x1066 surface (2,016,872 px). The proposal renders 2256x1504
// (3,393,024 px). 3393024/2016872 = 1.68. The 2.25x figure that circulated is
// proposed-over-CURRENT (1504x1003), a true number against a denominator the
// budget question never asked about.
//
// ---------------------------------------------------------------------------
// EVERY FAMILY HERE CALLS ->UseRealTime(). THIS IS THE POINT, NOT A DETAIL.
// ---------------------------------------------------------------------------
// google-benchmark defaults to CPU time, and no pre-existing family in this
// directory calls UseRealTime(). baselines.json records what that costs:
// BM_StepQuadWorldsGpu/64 clears its floor by 9.84x on cpu_time and 1.014x on
// real_time -- the same run, the same benchmark, a ~10x difference in
// apparent headroom -- and BM_StepPlainBodiesGpu/1000 measured cpu_time as
// exactly 0, reporting items_per_second as the JSON value Infinity.
//
// A frame budget is a WALL-CLOCK question. "Does the frame fit in 16.6 ms"
// is not asking how much CPU time the thread was charged; it is asking
// whether the frame finished. So every family below is real-time based, and
// any row of this file's output pinned into MV3 must say so ON THE ROW --
// not once in a preamble, because preambles get skimmed and a row reading
// 1.014x beside a row reading 9.84x for the same benchmark is the entire
// lesson.
//
// ---------------------------------------------------------------------------
// WHAT WOULD MAKE THESE NUMBERS WRONG
// ---------------------------------------------------------------------------
// Stated here because a pinned figure with no falsification condition is not
// a measurement, it is a memory -- and MV3's table has sat unpinned since
// sign-off precisely because a bare number could not be checked later.
//
//   * A DIFFERENT BOX. These are one machine's numbers. baselines.json's own
//     _meta says of this box's GPU that it is "not a proxy for a more capable
//     device a reader might assume"; the same caution applies to its CPU.
//   * A DIFFERENT BUILD. Release vs Debug moves rasterizer cost by more than
//     any ratio measured here. spade_fp_strict is on this target, so the fp
//     model is pinned, but the optimiser level is not pinned BY this file.
//   * A SCENE CHANGE. Fill cost is per-pixel; TRIANGLE cost is not. The
//     ground-plane family below is deliberately near-empty geometry, so it
//     measures the per-pixel floor a resolution change actually multiplies.
//     A scene with real geometry does not scale the same way, and reading
//     these rows as "the cost of any frame at this resolution" is the one
//     misuse most likely to happen.
//   * NOTHING IN CI READS THIS. baselines.json's policy is manual comparison
//     by a human. No check will contradict a stale number here.
//
// ---------------------------------------------------------------------------
// HOW TO REGENERATE -- MV3 CITES THIS FILE, NOT A NUMBER
// ---------------------------------------------------------------------------
// The viewport & cameras spec's measured-throughput obligation is discharged by
// pinning the METHOD rather than a figure. That is deliberate: MV3's table sat
// unpinned from sign-off (2026-08-19) precisely because a bare number is
// unfalsifiable later, and nothing in CI reads this directory's results -- so a
// digit embedded in a spec today would repeat the original mistake with fresher
// numbers. What is pinned is the conclusion, the basis, what would falsify it,
// and this command:
//
//   spade_bench --benchmark_filter=BM_Render \
//               --benchmark_min_time=1s \
//               --benchmark_repetitions=3
//
// The min-time floor is not optional. Without it a hand-rolled loop on this box
// produced a 2.4x run-to-run spread; the editor realm hit exactly that and the
// floor collapsed it to 1.08-1.27x. 1s is ~64x the ~15.6 ms Windows scheduler
// tick, so no single preempted sample can dominate -- the same reasoning
// baselines.json's `spread_policy` states for the sim families.
//
// READ real_time AND cpu_time TOGETHER, ALWAYS. On single-threaded CPU work
// they should be close. When they are not, the thread is being descheduled and
// the numbers are silently rescaled rather than corrupted -- they still look
// entirely plausible. The first run of this file measured 1775 ms / 2916 ms for
// the two decision cases with real_time ~3x cpu_time, because other sessions on
// this shared box were building. The clean re-run was 4x faster with real ~ cpu.
// THAT GAP IS THE INSTRUMENT REPORTING ITS OWN TRUSTWORTHINESS; there is no
// other signal that the box was loaded.
//
// WHAT THIS BOX MEASURED, 2026-09-08 (a reference point to re-derive against,
// NOT a pin -- Release, spade/build-host, mean of 3, real_time):
//
//   BackgroundFill  1892x1066  401.4 ms  CV  9.8%   |  2256x1504  659.3 ms  CV 2.8%
//   GroundPlane     1892x1066  396.2 ms  CV  7.0%   |  2256x1504  722.2 ms  CV 5.9%
//
//   ratio priced -> proposed: 1.64x (fill) to 1.82x (ground), against a 1.683x
//   pixel ratio. Per-pixel rate 4.7-5.3 M px/s across a 15x pixel range.
//
// BM_RenderSlots, 2026-09-09, AFTER slots shipped: STILL UNRESOLVED, and this
// entry exists so the next person does not spend the afternoon re-deriving a
// number that is not there. Two runs, same box, ~20 minutes apart, both
// Release/spade-build-host, both with the min-time floor this file requires:
//
//   run A (min_time=1s, reps=3): slots 1 -> 4 mean 36.4 ms -> 65.0 ms  = 1.79x
//   run B (min_time=2s, reps=5): slots 1 -> 4 mean 22.8 ms -> 75.9 ms  = 3.33x
//
// THE RATIO ITSELF DISAGREES BY 1.9x BETWEEN RUNS, so it cannot be pinned --
// and the ratio was supposed to be the durable quantity here, the way it is
// for the resolution rows above. Run A was also NON-MONOTONIC (slots=3 mean
// 42.7 ms, FASTER than slots=2 at 56.2 ms, which is impossible for work that
// only grows) with slots=2 at CV 38.7% and a 2.3x spread across its own three
// repetitions. Run B was better behaved (CV 12.8% / 14.7%, mean and median
// within 7%) but its absolute slots=1 figure is 1.6x run A's, which is the
// box, not the code.
//
// The box was under memory pressure both times (~1.5 GB free of 7.6, several
// sessions live). real_time and cpu_time stayed CLOSE throughout -- 1.13-1.17x
// -- so the descheduling signature this file's header describes did NOT fire.
// That is worth recording precisely because it is the honest failure of a
// check I trusted: real~cpu says the thread was not preempted while running,
// and says nothing about cache and memory-bandwidth contention from other
// processes, which is what a RAM-starved box actually does to a fill-bound
// rasterizer. A CLEAN real/cpu ratio IS NOT A CERTIFICATE THAT THE BOX WAS
// QUIET. The within-case spread is the check that caught it here.
//
// WHAT IS SAFE TO USE ANYWAY, and it is enough to design a UI against:
// TREAT N SLOTS AS COSTING N x ONE SLOT. That is a conservative UPPER bound,
// not a measurement -- both runs came in well under it (1.79x and 3.33x for a
// 4x pixel increase), and sublinearity is what the implementation predicts:
// dronesim/spade/host.cpp hoists buildLiveBodyPoses/buildLiveBodyMeshIndices/
// update_dynamics OUT of the per-slot loop, so those run once per
// kathost_render however many slots are live, and every extra slot re-renders
// a scene already hot in cache. Both runs agree on the SIGN of that effect
// even where they disagree on its size. What they do not support is any
// specific number, and W7 must not be handed one.
//
// TO RESOLVE IT: re-run on a quiet box (nothing else building, >4 GB free),
// reps>=5, and require BOTH monotonicity in slot count AND agreement between
// two independent runs before recording a figure. Neither condition held here.
//
// BM_RenderSlots is NOT part of that: it came back at up to 57% CV with slots=3
// measuring slower than slots=4, which is impossible for monotonically
// increasing work. It is noise and is recorded as unresolved rather than pinned.
//
// ---------------------------------------------------------------------------
// THE THREE SWEEPS
// ---------------------------------------------------------------------------
//   1. BM_RenderBackgroundFill/<case>  -- an EMPTY scene: sky gradient only,
//      no meshes, no statics, no overlays. This is the pure per-pixel write
//      floor, and it is the deliberate mirror of the editor realm's own
//      fillRect control on the present side, so the two halves have a
//      comparable floor rather than two unrelated baselines.
//
//   2. BM_RenderGroundPlane/<case>  -- one ground plane, the shape the
//      base-package world actually has (content/worlds/base-ground.world.yaml
//      is one `plane` SDF node and nothing else). This is the row the 1.68x
//      question is answered from.
//
//      Both sweep the FOUR resolutions the decision actually contains, not
//      round numbers: 640x360 (MV3 s.3's secondary-slot size), 1504x1003
//      (what this box renders today at cap 1.0), 1892x1066 (what MV3 s.8
//      priced), and 2256x1504 (the device-pixel proposal).
//
//   3. BM_RenderSlots/<n>  -- n independent targets rendered per iteration at
//      the secondary-slot resolution, n in {1,2,3,4}.
//
//      UPDATED 2026-09-09, W6: CAMERA SLOTS NOW EXIST, and this family is no
//      longer the loose proxy the paragraph here used to describe. G-5 was
//      ruled (option 3), and dronesim/spade/host.cpp's kathost_render now
//      holds kMaxCameraSlots camera/pool pairs and renders every configured
//      one. Crucially it hoists ALL the shared work out of the per-slot loop
//      -- buildLiveBodyPoses, buildLiveBodyMeshIndices and update_dynamics
//      each run ONCE per kathost_render however many slots are live -- so
//      what actually repeats per slot is exactly one spade::render::render()
//      call, which is exactly what this loop does.
//
//      What the host adds on top, per slot, and why it is not measured here:
//      a FramePool::acquireWritableSlot (a scan of >=2 refcounts), a
//      std::vector::resize that is a no-op after the first frame at a given
//      size, and a publishRendered that copies seven scalars. All three are
//      O(1)-ish bookkeeping against a full rasterization of 230k pixels, so
//      this remains a lower bound -- but a TIGHT one, for a stated reason,
//      rather than a lower bound of unknown looseness.
//
//      Still not measured here, and still the honest gap: this benchmark
//      links spade::render directly and cannot link kat_host_spade (HS2
//      rule 2 -- no kat concept inside spade/), so cross-slot effects that
//      only exist at the host level (cache pressure from N live pixel
//      buffers, pool growth under concurrent acquires) are outside its
//      reach by construction, not by omission.
// ---------------------------------------------------------------------------

#include <benchmark/benchmark.h>

#include <cstdint>
#include <vector>

#include <glm/glm.hpp>

#include "render/raster_cpu.hpp"
#include "render/scene.hpp"
#include "render/target.hpp"

namespace {

using spade::render::Camera;
using spade::render::DrawMode;
using spade::render::RenderOptions;
using spade::render::RenderScene;
using spade::render::RenderTarget;

// The four resolutions the cap decision actually contains. Index order is the
// benchmark Arg() value, so a row reads back to a named case rather than to a
// bare integer.
struct ResCase {
    const char* label;
    uint32_t width;
    uint32_t height;
};

constexpr ResCase kResCases[] = {
    {"640x360_secondary_slot", 640, 360},     // MV3 s.3's secondary-slot size
    {"1504x1003_current_cap1", 1504, 1003},   // what this box renders today
    {"1892x1066_mv3_priced", 1892, 1066},     // the surface MV3 s.8 priced
    {"2256x1504_device_px", 2256, 1504},      // the device-pixel proposal
};
constexpr int kResCaseCount = static_cast<int>(sizeof(kResCases) / sizeof(kResCases[0]));

[[nodiscard]] RenderTarget make_target(std::vector<uint8_t>& storage, uint32_t width, uint32_t height) {
    storage.assign(static_cast<size_t>(width) * height * 4u, 0u);
    RenderTarget target;
    target.pixels = std::span<uint8_t>(storage.data(), storage.size());
    target.width = width;
    target.height = height;
    target.stride = width * 4u;
    target.format = spade::render::PixelFormat::bgrx8;
    return target;
}

// A camera looking at the origin from a typical third-person distance -- the
// same framing content/scenes/base-ground.kscene's "default" bookmark uses,
// so the benchmark renders roughly what a viewer actually sees.
[[nodiscard]] Camera bench_camera() {
    Camera camera;
    camera.position = glm::vec3(0.0f, 2.0f, 4.0f);
    camera.orientation = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
    return camera;
}

[[nodiscard]] RenderOptions bench_options() {
    RenderOptions options;
    options.mode = DrawMode::shaded;
    options.shadows = false;   // shadow cost is a scene property, not a pixel one
    options.overlays = false;  // overlays draw fixed-size chrome, which would
                               // dilute a per-pixel sweep with a constant term
    return options;
}

// Empty scene: no meshes, no statics, no dynamics, no ground planes. render()
// still writes every pixel (the sky gradient), which is exactly the floor we
// want -- this is the render-side analogue of a fillRect.
[[nodiscard]] RenderScene empty_scene() {
    RenderScene scene;
    scene.materials = {spade::render::Material{}};
    scene.bounds = spade::render::Aabb{.min = glm::vec3(-5.0f), .max = glm::vec3(5.0f)};
    return scene;
}

// One infinite ground plane -- the base-package world's actual shape.
[[nodiscard]] RenderScene ground_plane_scene() {
    RenderScene scene = empty_scene();
    spade::render::GroundPlane ground;
    ground.normal = glm::vec3(0.0f, 1.0f, 0.0f);
    ground.offset = 0.0f;
    ground.material = 0;
    scene.ground_planes.push_back(ground);
    return scene;
}

void run_fill(benchmark::State& state, const RenderScene& scene, uint32_t width, uint32_t height) {
    const Camera camera = bench_camera();
    const RenderOptions options = bench_options();
    std::vector<uint8_t> storage;
    RenderTarget target = make_target(storage, width, height);
    std::vector<float> shadow_scratch;  // reused across frames, per render()'s contract

    for (auto _ : state) {
        const auto result = spade::render::render(scene, camera, options, target, &shadow_scratch);
        if (!result) {
            state.SkipWithError("render() failed");
            return;
        }
        benchmark::DoNotOptimize(target.pixels.data());
        benchmark::ClobberMemory();
    }

    // Pixels per second is the unit this whole question is asked in, and the
    // pixel count is stated as a counter so a reader never has to recover it
    // from the case label.
    const double pixels = static_cast<double>(width) * static_cast<double>(height);
    state.counters["pixels"] = pixels;
    state.counters["pixels_per_second"] =
        benchmark::Counter(pixels * static_cast<double>(state.iterations()), benchmark::Counter::kIsRate);
}

void BM_RenderBackgroundFill(benchmark::State& state) {
    const ResCase& c = kResCases[state.range(0)];
    state.SetLabel(c.label);
    run_fill(state, empty_scene(), c.width, c.height);
}

void BM_RenderGroundPlane(benchmark::State& state) {
    const ResCase& c = kResCases[state.range(0)];
    state.SetLabel(c.label);
    run_fill(state, ground_plane_scene(), c.width, c.height);
}

// n independent targets per iteration -- see this file's header for why this
// is a LOWER BOUND on multi-slot cost, not multi-slot cost.
void BM_RenderSlots(benchmark::State& state) {
    const int slots = static_cast<int>(state.range(0));
    const uint32_t width = kResCases[0].width, height = kResCases[0].height;  // secondary-slot size
    // W6: the label no longer says "no slot abi exists", because one does.
    // It still says PROXY -- see the family's comment in this file's header
    // for exactly which per-slot costs sit outside a spade-only benchmark.
    state.SetLabel("640x360_per_slot__engine_only_excludes_host_bookkeeping");

    const RenderScene scene = ground_plane_scene();
    const Camera camera = bench_camera();
    const RenderOptions options = bench_options();

    std::vector<std::vector<uint8_t>> storage(static_cast<size_t>(slots));
    std::vector<RenderTarget> targets;
    targets.reserve(static_cast<size_t>(slots));
    for (int i = 0; i < slots; ++i) {
        targets.push_back(make_target(storage[static_cast<size_t>(i)], width, height));
    }
    std::vector<float> shadow_scratch;

    for (auto _ : state) {
        for (int i = 0; i < slots; ++i) {
            const auto result = spade::render::render(scene, camera, options, targets[static_cast<size_t>(i)],
                                                      &shadow_scratch);
            if (!result) {
                state.SkipWithError("render() failed");
                return;
            }
            benchmark::DoNotOptimize(targets[static_cast<size_t>(i)].pixels.data());
        }
        benchmark::ClobberMemory();
    }

    const double pixels = static_cast<double>(width) * static_cast<double>(height) * static_cast<double>(slots);
    state.counters["pixels_per_frame"] = pixels;
    state.counters["pixels_per_second"] =
        benchmark::Counter(pixels * static_cast<double>(state.iterations()), benchmark::Counter::kIsRate);
}

}  // namespace

// ->UseRealTime() on every family: see this file's header. A frame budget is a
// wall-clock question, and cpu_time answers a different one.
BENCHMARK(BM_RenderBackgroundFill)->DenseRange(0, kResCaseCount - 1, 1)->UseRealTime();
BENCHMARK(BM_RenderGroundPlane)->DenseRange(0, kResCaseCount - 1, 1)->UseRealTime();
BENCHMARK(BM_RenderSlots)->DenseRange(1, 4, 1)->UseRealTime();

// NO BENCHMARK_MAIN() HERE -- same reason bench_sim.cpp states: spade_bench is
// one executable built from several benchmark TUs, and benchmark::benchmark_main
// supplies main() once for all of them.
