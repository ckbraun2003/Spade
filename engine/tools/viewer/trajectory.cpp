// ---------------------------------------------------------------------------
// trajectory.cpp -- see trajectory.hpp.
//
// THE FILE, line by line (text, LF, one record per line). It is a CPU golden
// (TD-1): everything in it is a pure function of the scene and the engine, so
// two runs at one commit match byte for byte.
//
//   # <comment>
//   scene <name>
//   backend cpu
//   dt_ns <n>
//   substeps <n>
//   ticks <n>
//   worlds <n>
//   world <w> seed <seed>                    one per world
//   bodies <n> vehicles <n>
//   checkpoint <tick> <chain> <state> <world 0> ...
//                                            every 25 ticks, and the last;
//                                            16-hex-digit digests
//                                            (testing/replay.hpp). <chain>
//                                            folds the state digest of every
//                                            tick up to and including <tick>
//   vehicle <tick> <i> <x> <y> <z>           every 25 ticks, each vehicle, m
//   centroid <tick> <w> <x> <y> <z>          every 250 ticks, and the last,
//                                            each world's mean body position, m
//
// The running chain catches a divergence at ANY tick, and it makes every
// prefix checkable: a check that stops at tick N compares every checkpoint up
// to N. The vehicle and centroid lines let a reader check the motion against
// the scene's recorded characterisation (scenes.cpp).
//
// Step timings and memory are NOT in the file, because they vary per run. They
// go to stdout as "perf <name> <value>" lines. The clock is read around
// step(1) only, to time it; nothing stepped reads it (L1).
// ---------------------------------------------------------------------------
#include "trajectory.hpp"

#include <algorithm>
#include <chrono>
#include <cstdarg>
#include <cstddef>
#include <cstdio>
#include <exception>
#include <fstream>
#include <span>
#include <stdexcept>
#include <vector>

#if defined(_WIN32)
#ifndef NOMINMAX
#define NOMINMAX
#endif
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>
#include <psapi.h>
#endif

#include <glm/glm.hpp>

#include "setup.hpp"
#include "state/snapshot.hpp"
#include "testing/replay.hpp"

namespace spade::viewer {
namespace {

constexpr uint64_t kCheckpointTicks = 25;
constexpr uint64_t kCentroidTicks = 250;

// Ticks left out of the step-time statistics: the first steps pay one-time
// costs (allocation, cold caches) that are not the scene's steady cost.
constexpr uint64_t kTimingWarmupTicks = 100;

// printf into a string. The file is built in memory and written once, so a
// run that fails part way leaves no partial file behind.
void appendf(std::string& out, const char* format, ...) {
    char line[512];
    std::va_list args;
    va_start(args, format);
    const int n = std::vsnprintf(line, sizeof(line), format, args);
    va_end(args);
    if (n < 0 || static_cast<std::size_t>(n) >= sizeof(line)) {
        throw std::runtime_error("a trajectory line did not fit in 512 bytes");
    }
    out.append(line, static_cast<std::size_t>(n));
}

struct ProcessMemory {
    uint64_t private_bytes = 0;
    uint64_t working_set = 0;
    uint64_t peak_working_set = 0;
};

// Both metrics, because v1 reports PrivateUsage and the sandbox the working
// set; a baseline that named only one could not be compared with either.
ProcessMemory process_memory() {
    ProcessMemory m;
#if defined(_WIN32)
    PROCESS_MEMORY_COUNTERS_EX c{};
    c.cb = sizeof(c);
    if (GetProcessMemoryInfo(GetCurrentProcess(), reinterpret_cast<PROCESS_MEMORY_COUNTERS*>(&c),
                             sizeof(c))) {
        m.private_bytes = c.PrivateUsage;
        m.working_set = c.WorkingSetSize;
        m.peak_working_set = c.PeakWorkingSetSize;
    }
#endif
    return m;
}

uint64_t fold_digest(uint64_t chain, uint64_t digest) {
    return spade::fnv1a64(std::as_bytes(std::span<const uint64_t, 1>(&digest, 1)), chain);
}

// The value at fraction `q` of the sorted samples (nearest rank).
long long quantile(std::vector<long long> samples, double q) {
    if (samples.empty()) {
        return 0;
    }
    const std::size_t rank = static_cast<std::size_t>(q * static_cast<double>(samples.size() - 1));
    std::nth_element(samples.begin(), samples.begin() + static_cast<std::ptrdiff_t>(rank), samples.end());
    return samples[rank];
}

void write_checkpoint(std::string& out, const spade::Simulation& sim, uint32_t world_count, uint64_t chain) {
    appendf(out, "checkpoint %llu %016llx %016llx", static_cast<unsigned long long>(sim.tick().value),
            static_cast<unsigned long long>(chain), static_cast<unsigned long long>(spade::testing::state_digest(sim)));
    for (uint32_t w = 0; w < world_count; ++w) {
        appendf(out, " %016llx", static_cast<unsigned long long>(spade::testing::world_digest(sim, w)));
    }
    out += '\n';
}

void write_vehicles(std::string& out, const spade::Simulation& sim, const std::vector<spade::VehicleRef>& vehicles) {
    for (std::size_t i = 0; i < vehicles.size(); ++i) {
        const spade::Result<const spade::BodyState*> body = sim.body(vehicles[i].body);
        if (!body) {
            throw std::runtime_error("vehicle " + std::to_string(i) + " has no body: " + body.error().context);
        }
        const glm::vec3 p = (*body)->pos;
        appendf(out, "vehicle %llu %zu %.9g %.9g %.9g\n", static_cast<unsigned long long>(sim.tick().value),
                     i, static_cast<double>(p.x), static_cast<double>(p.y), static_cast<double>(p.z));
    }
}

void write_centroids(std::string& out, const spade::Simulation& sim, uint32_t world_count) {
    for (uint32_t w = 0; w < world_count; ++w) {
        const spade::Result<std::span<const spade::BodyState>> bodies = sim.world_bodies(w);
        if (!bodies) {
            throw std::runtime_error("world_bodies(" + std::to_string(w) + ") failed: " + bodies.error().context);
        }
        glm::vec3 sum(0.0f);
        for (const spade::BodyState& b : *bodies) {
            sum += b.pos;
        }
        const glm::vec3 c = bodies->empty() ? sum : sum / static_cast<float>(bodies->size());
        appendf(out, "centroid %llu %u %.9g %.9g %.9g\n", static_cast<unsigned long long>(sim.tick().value), w,
                     static_cast<double>(c.x), static_cast<double>(c.y), static_cast<double>(c.z));
    }
}

}  // namespace

int write_trajectory(const Scene& scene, uint64_t ticks, const std::string& path) {
    std::string out;
    try {
        SceneRun run = make_simulation(scene, spade::compute::BackendDesc{});
        spade::Simulation& sim = run.sim;
        const uint32_t world_count = sim.layout().world_count;
        const ProcessMemory after_setup = process_memory();

        appendf(out, "# spade_viewer trajectory (INT-4, SL14b): a CPU golden. Format: engine/tools/viewer/trajectory.cpp.\n");
        appendf(out, "scene %s\nbackend cpu\ndt_ns %llu\nsubsteps %u\nticks %llu\nworlds %u\n", scene.name.c_str(),
                     static_cast<unsigned long long>(kStepDtNs), kSubsteps, static_cast<unsigned long long>(ticks),
                     world_count);
        for (std::size_t w = 0; w < scene.worlds.worlds.size(); ++w) {
            appendf(out, "world %zu seed %llu\n", w,
                         static_cast<unsigned long long>(scene.worlds.worlds[w].seed));
        }
        appendf(out, "bodies %zu vehicles %zu\n", scene.bodies.size(), scene.vehicles.size());

        uint64_t chain = spade::kFnv1a64Offset;
        std::vector<long long> step_ns;
        step_ns.reserve(static_cast<std::size_t>(ticks));
        for (uint64_t t = 0;; ++t) {
            if (sim.tick().value != t) {
                throw std::runtime_error("the simulation tick " + std::to_string(sim.tick().value) +
                                         " does not match the loop tick " + std::to_string(t));
            }
            chain = fold_digest(chain, spade::testing::state_digest(sim));
            if (t % kCheckpointTicks == 0 || t == ticks) {
                write_checkpoint(out, sim, world_count, chain);
                write_vehicles(out, sim, run.vehicle_refs);
            }
            if (t % kCentroidTicks == 0 || t == ticks) {
                write_centroids(out, sim, world_count);
            }
            if (t == ticks) {
                break;
            }

            // The same order as the window loop (bridge.cpp step_and_sync()):
            // the hook sees the tick about to run, then exactly one step.
            if (scene.command_hook != nullptr) {
                scene.command_hook(sim, sim.tick().value, run.vehicle_refs);
            }
            const auto t0 = std::chrono::steady_clock::now();
            const spade::Result<void> stepped = sim.step(1);
            const auto t1 = std::chrono::steady_clock::now();
            if (!stepped) {
                throw std::runtime_error("Simulation::step failed at tick " + std::to_string(t) + ": " +
                                         stepped.error().context);
            }
            if (t >= kTimingWarmupTicks) {
                step_ns.push_back(std::chrono::duration_cast<std::chrono::nanoseconds>(t1 - t0).count());
            }
        }
        // Per-run numbers go to stdout, so the golden file stays byte-stable.
        const ProcessMemory at_end = process_memory();
        std::printf("perf step_ns_median %lld\n", quantile(step_ns, 0.5));
        std::printf("perf step_ns_p90 %lld\n", quantile(step_ns, 0.9));
        std::printf("perf step_samples %zu\n", step_ns.size());
        std::printf("perf private_bytes_after_setup %llu\n", static_cast<unsigned long long>(after_setup.private_bytes));
        std::printf("perf working_set_after_setup %llu\n", static_cast<unsigned long long>(after_setup.working_set));
        std::printf("perf private_bytes_end %llu\n", static_cast<unsigned long long>(at_end.private_bytes));
        std::printf("perf working_set_end %llu\n", static_cast<unsigned long long>(at_end.working_set));
        std::printf("perf peak_working_set %llu\n", static_cast<unsigned long long>(at_end.peak_working_set));
    } catch (const std::exception& e) {
        std::fprintf(stderr,
                     "spade_viewer: the trajectory of scene '%s' failed: %s. Nothing was written to '%s'; "
                     "fix the cause and run again.\n",
                     scene.name.c_str(), e.what(), path.c_str());
        return 1;
    }

    std::ofstream file(path, std::ios::binary | std::ios::trunc);
    file.write(out.data(), static_cast<std::streamsize>(out.size()));
    file.close();
    if (!file) {
        std::fprintf(stderr,
                     "spade_viewer: cannot write '%s'. Check that its directory exists, is writable and has "
                     "free space, then run again.\n",
                     path.c_str());
        return 1;
    }
    return 0;
}

}  // namespace spade::viewer
