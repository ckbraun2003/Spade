// spade_geo_fetch (L85e, SP-5): the program around run_cli(). Everything the library leaves to its caller is chosen here: the real
// network (WinHTTP on Windows, "no HTTP layer on this platform yet" elsewhere), the real clock, the process environment and a
// temporary-name token. The box is left to fetch_map, which asks Core's geodesy (area.hpp). Nothing here is tested by a unit test: the
// program's behaviour is run_cli()'s, tested on a fake network; tests/CMakeLists.txt runs the built program for its usage and its
// git-work-tree refusal, and scripts/package-test.ps1 runs the installed one.

#include <chrono>
#include <cstdlib>
#include <format>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "cli.hpp"

#if defined(_WIN32)
#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>
#else
#include <unistd.h>
#endif

namespace {

[[nodiscard]] std::string utc_now() {
    const auto now = std::chrono::floor<std::chrono::seconds>(std::chrono::system_clock::now());
    return std::format("{:%Y-%m-%dT%H:%M:%SZ}", now);
}

[[nodiscard]] std::string process_token() {
#if defined(_WIN32)
    return std::to_string(static_cast<unsigned long>(GetCurrentProcessId()));
#else
    return std::to_string(static_cast<long>(getpid()));
#endif
}

#if defined(_MSC_VER)
#pragma warning(push)
#pragma warning(disable : 4996)  // std::getenv: the process environment is read once, on one thread, before anything else runs
#endif
[[nodiscard]] const char* environment(const char* name) { return std::getenv(name); }
#if defined(_MSC_VER)
#pragma warning(pop)
#endif

}  // namespace

int main(int argc, char** argv) {
    using namespace spade::geofetch;
    const std::vector<std::string> args(argv + 1, argv + argc);
    std::unique_ptr<HttpClient> http = make_platform_http_client();
    CliEnv env{FetchEnv{*http, nullptr, utc_now, [](int ms) { std::this_thread::sleep_for(std::chrono::milliseconds(ms)); }, nullptr,
                        process_token()},
               environment,
#if defined(_WIN32)
               spade::MapPlatform::windows
#else
               spade::MapPlatform::posix
#endif
    };
    std::cout << std::unitbuf;  // a line at a time, so an editor reading the progress through a pipe sees it as it happens
    return run_cli(args, env, std::cout, std::cerr);
}
