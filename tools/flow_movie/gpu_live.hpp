// flow_movie --gpu (plan B, B7's G2; the lead's approval, 2026-10-27): a
// LiveRun film's fluid stepped on the device by GpuFluid
// (spike/airflow/step_chain.hpp) instead of LiveRun's fluid_step().
//
// INTERIM: NO PARTS, HELD AT HOVER. GpuFluid takes no solids until Physics'
// B5.6 hooks (G3), so a run model with parts in the flow is refused, and the
// film's title and card say "no parts". The drone is held, and its rotors are
// held at their commanded speed as actuator discs, formed as B7's acceptance
// harness forms them (tests/test_gpu_airflow_d1_acceptance.cpp): LiveRun
// started on the case and stepped once at the program's controls, for its
// rotor rows and held body, then vehicles::rotor_discs() with each row's
// speed its command. A program whose controls change is refused, since the
// discs never do.
//
// The rest is LiveRun's: its FluidConfig without solids (the case's rho, nu
// and step; plan A's n_v and sweeps), its grid from rest, its frames at
// kFramesPerSecond of simulated time, the grid's minimum corner at -dx n / 2,
// each frame's log (the held body, then the rotors' speeds) and run.txt's
// lines (the step, then the label).
//
// Each published frame is checked on the host before it is cached
// (frame_check.hpp): a non-finite one, or one whose fastest cell is past ten
// times the case's expected top speed, stops the film, naming its frame and
// step; a blown-up flow never becomes a film (the lead, 2026-10-27). No save:
// the film runs in minutes, and a cache left unfinished starts again.
#pragma once

#include <cstdint>
#include <filesystem>
#include <string>

#include "cache.hpp"
#include "core/error.hpp"
#include "live_case.hpp"

namespace flow_movie {

// Simulates `frames` frames of `lc` into `cache`, marked `mark`; `stem`
// names the film in its errors and progress lines. Refused, before the device
// is touched, as above and as LiveRun::start() and GpuFluid::create() refuse.
[[nodiscard]] spade::Result<void> simulate_gpu(const LiveCase& lc, uint32_t frames, const std::string& stem,
                                               const std::filesystem::path& cache, const CacheMark& mark);

}  // namespace flow_movie
