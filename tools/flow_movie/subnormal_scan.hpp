// D1's part of the subnormal scan (Cameron's "measure first", 2026-10-22,
// through the lead; C8's part is tests/test_airflow_subnormal_scan.cpp): does
// the CPU solver meet a subnormal float flying D1? The GPU flushes them, so a
// GPU twin can equal the CPU bit for bit only if none ever changes a result.
//
// Two LiveRuns side by side on this thread. LiveRun and the solver under it
// start no thread, so this thread's MXCSR sees every operation they perform:
//   - the default run, with x86 MXCSR's sticky flags read around each step():
//     DE, some operation had a subnormal operand; UE, some result was tiny
//     and inexact;
//   - a twin with MXCSR's FTZ and DAZ set, which flush as the GPU does.
// Both start from rest, or both restore one save (a film's live.save from
// inside D1's window). After every step both are saved and the saves compared
// byte for byte: the first step at which the twin's state differs in any bit,
// and by how much in the grid. The default run's grid (u, v, w, p, read from
// its save) is scanned for subnormals. LiveRun keeps the pressure solve's
// scratch to itself, so only the flags see that. Nothing is cached; the two
// saves under `work_dir` are overwritten each step.
#pragma once

#include <cstdint>
#include <filesystem>

#include "live_case.hpp"

namespace flow_movie {

// Prints a line a step, then the table. `from_save` empty: both runs start
// from rest. 0 done; 1 failed, named on stderr; 77 without an MXCSR (not x86).
[[nodiscard]] int subnormal_scan(const LiveCase& lc, uint32_t steps, const std::filesystem::path& from_save,
                                 const std::filesystem::path& work_dir);

// What raises them (the lead: is it the body's update or the fluid?). One
// run, from rest or the save, with MXCSR's DE and UE exceptions unmasked in
// step(): a vectored handler records the first instruction of each kind a
// step faults on and the calls under it (unwound from the fault), masks that
// kind and lets the instruction run again, masked, as the default run does.
// Each address is named from flow_movie.map beside the exe (the link's
// /MAP): the function and its object file. Nothing is saved. 0 done; 1
// failed, named on stderr; 77 off Windows x64.
[[nodiscard]] int subnormal_locate(const LiveCase& lc, uint32_t steps, const std::filesystem::path& from_save);

}  // namespace flow_movie
