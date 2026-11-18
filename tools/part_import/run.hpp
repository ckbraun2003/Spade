#pragma once

// ---------------------------------------------------------------------------
// part_import/run.hpp -- spade_part_import, in-process (L88; design note). The program is main.cpp over this: everything the library
// leaves to its caller (the environment, where the exe lives, the cancel flag) is in RunEnv, so the tests drive the whole tool on a
// temporary store.
//
// Standard output is JSON lines and nothing else: {"event":"stage",...} lines, then ONE {"event":"result",...} line last, whatever
// happened. Standard error holds the usage text for a usage problem and nothing else. The exit code is the result's `exit`.
// ---------------------------------------------------------------------------

#include <chrono>
#include <filesystem>
#include <functional>
#include <iosfwd>
#include <string>
#include <vector>

namespace spade::partimport {

// The version of the mesh readers' conversion: a part of every cache key, changed when what a reader writes into a sidecar changes.
inline constexpr const char* kMeshConverterVersion = "mesh-1";
// The same for a STEP import: the converter route (step_convert.py's --emit-3mf and the 3MF reader). OpenCASCADE's own version is not in the
// key (it is not known before the converter runs); it is in the converter's answer.
inline constexpr const char* kStepConverterVersion = "step-1";

struct RunEnv {
    std::function<const char*(const char* name)> getenv;  // SPADE_PART_STORE, SPADE_STEP_PYTHON, SPADE_STEP_CONVERT
    std::filesystem::path exe_dir;                        // beside the exe: step_convert.py and .venv-step
    std::function<bool()> cancelled;                      // polled between stages; may be empty
    std::string token = "0";                              // unique per process (a temporary directory's name)
    std::chrono::seconds stale_after = std::chrono::hours(1);  // a leftover temporary this old is removed at the start of an import
};

[[nodiscard]] int run_part_import(const std::vector<std::string>& args, const RunEnv& env, std::ostream& out, std::ostream& err);

}  // namespace spade::partimport
