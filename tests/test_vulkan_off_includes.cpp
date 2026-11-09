// test_vulkan_off_includes.cpp -- no file a SPADE_VULKAN=OFF target compiles includes a generated header unguarded (CPU source
// scan, in the manner of BackendStub's and the BitPortability scans).
//
// WHY. bindings.gen.hpp, layout_check.gen.hpp and the <kernel>.spv.gen.hpp headers are generated only in a SPADE_VULKAN=ON
// configure (engine/CMakeLists.txt's Vulkan gate). A TU that spade_sim and the other Vulkan-free libraries compile, or a header
// such a TU reaches, that includes one unguarded builds on every Windows and Vulkan-ON leg and fails only at the consumer-OFF
// leg: stage 7 batch C did exactly that with compute::gen::kBinding_airflow_config in sim/module_schedule.cpp (gate 26's leg,
// build-docker/3b042067530d), the same class as gate 24's missing stub. This test turns the next one into a failure on any build,
// naming the file and line.
//
// WHAT IS SCANNED. Every .cpp and .hpp under engine/ except what only a Vulkan build compiles (kVulkanOnly below: the whole of
// compute/vulkan/ bar its OFF stub, compute/layout_check.cpp, render/vulkan/). A generated-header include is GUARDED when an
// enclosing `#if` names __has_include (the pattern sim/standard_modules.cpp uses). The preprocessor conditions are tracked, so
// a guard several lines up counts and one that already closed does not.
#include <gtest/gtest.h>

#include <cstddef>
#include <filesystem>
#include <fstream>
#include <regex>
#include <string>
#include <vector>

namespace {

namespace fs = std::filesystem;

// Paths (relative to engine/, forward slashes) only a SPADE_VULKAN=ON build compiles. A directory ends in '/'.
const std::vector<std::string> kVulkanOnly = {"compute/vulkan/", "compute/layout_check.cpp", "render/vulkan/"};
// ... but the OFF fallback of the backend lives in compute/vulkan/ and IS compiled with Vulkan off.
const std::vector<std::string> kOffFiles = {"compute/vulkan/backend_stub.cpp"};

[[nodiscard]] bool vulkan_only(const std::string& relative) {
    for (const std::string& off : kOffFiles) {
        if (relative == off) return false;
    }
    for (const std::string& prefix : kVulkanOnly) {
        if (prefix.back() == '/' ? relative.rfind(prefix, 0) == 0 : relative == prefix) return true;
    }
    return false;
}

struct Finding {
    std::string file;
    std::size_t line = 0;
    std::string text;
};

// The unguarded generated-header includes of one file; `guarded` counts the guarded ones (non-vacuity).
void scan(const fs::path& path, const std::string& relative, std::vector<Finding>& unguarded, std::size_t& guarded) {
    static const std::regex kInclude(R"(^\s*#\s*include\s+"[A-Za-z0-9_.]*\.gen\.hpp")");
    static const std::regex kIf(R"(^\s*#\s*(if|ifdef|ifndef)\b)");
    static const std::regex kEndif(R"(^\s*#\s*endif\b)");
    std::ifstream in(path);
    std::vector<bool> stack;  // one entry per open #if: does its condition name __has_include?
    std::string line;
    std::size_t number = 0;
    while (std::getline(in, line)) {
        ++number;
        if (!line.empty() && line.back() == '\r') line.pop_back();
        if (std::regex_search(line, kIf)) {
            stack.push_back(line.find("__has_include") != std::string::npos);
        } else if (std::regex_search(line, kEndif)) {
            if (!stack.empty()) stack.pop_back();
        } else if (std::regex_search(line, kInclude)) {
            bool covered = false;
            for (const bool g : stack) covered = covered || g;
            if (covered) {
                ++guarded;
            } else {
                unguarded.push_back({relative, number, line});
            }
        }
    }
}

}  // namespace

TEST(VulkanOff, NoFileAnOffTargetCompilesIncludesAGeneratedHeaderUnguarded) {
    const fs::path engine = fs::path(SPADE_ENGINE_DIR);
    ASSERT_TRUE(fs::exists(engine / "sim" / "module_schedule.cpp")) << "the source tree was not found at " << engine.string();
    for (const std::string& v : kVulkanOnly) {
        EXPECT_TRUE(fs::exists(engine / v)) << "kVulkanOnly names '" << v << "', which does not exist: renamed?";
    }
    for (const std::string& v : kOffFiles) EXPECT_TRUE(fs::exists(engine / v)) << "kOffFiles names '" << v << "'";

    std::vector<Finding> unguarded;
    std::size_t guarded = 0;
    std::size_t scanned = 0;
    for (const fs::directory_entry& entry : fs::recursive_directory_iterator(engine)) {
        if (!entry.is_regular_file()) continue;
        const std::string ext = entry.path().extension().string();
        if (ext != ".cpp" && ext != ".hpp") continue;
        const std::string relative = fs::relative(entry.path(), engine).generic_string();
        if (vulkan_only(relative)) continue;
        ++scanned;
        scan(entry.path(), relative, unguarded, guarded);
    }
    ASSERT_GT(scanned, 100u) << "the scan found almost nothing under engine/: has its layout changed?";
    EXPECT_GE(guarded, 1u) << "no guarded include was recognised (sim/standard_modules.cpp has some): the guard scan is broken";
    for (const Finding& f : unguarded) {
        ADD_FAILURE() << "engine/" << f.file << ":" << f.line << ": " << f.text
                      << "\n  a SPADE_VULKAN=OFF build has no generated headers: guard it with #if __has_include(...), or take "
                         "the number from compute/set_one_numbers.hpp (Vulkan-free) and tie it to the generated constant "
                         "there under __has_include";
    }
}
