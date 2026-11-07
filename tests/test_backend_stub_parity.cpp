// test_backend_stub_parity.cpp -- every VulkanBackend member function that engine/compute/vulkan/backend.cpp defines is also
// defined in backend_stub.cpp, the SPADE_VULKAN=OFF fallback (CPU source scan, in the manner of the BitPortability scans).
//
// WHY. sim/simulation.cpp calls every VulkanBackend method unconditionally (the cpu/vulkan routing is a runtime branch), so a
// SPADE_VULKAN=OFF build links only if the stub defines each one. A method added to backend.cpp without its stub twin builds and
// passes on every Windows and Vulkan-ON leg and fails only at the consumer-OFF link: it happened with device_report() (ae58a66d)
// and again with memory_report() (stage 7 batch A). This test turns the next one into a failure on any build.
//
// WHAT COUNTS AS A DEFINITION. A line of backend.cpp that starts in column 0 (a return type, `static`, ...) and names
// `VulkanBackend::<name>(`. Calls inside bodies are indented; a definition that begins mid-line would need the scan widened.
#include <gtest/gtest.h>

#include <cstddef>
#include <filesystem>
#include <fstream>
#include <regex>
#include <set>
#include <string>

namespace {

namespace fs = std::filesystem;

[[nodiscard]] std::set<std::string> defined_members(const fs::path& file) {
    std::set<std::string> out;
    std::ifstream in(file);
    static const std::regex kDefinition(R"(^[A-Za-z_].*\bVulkanBackend::(~?[A-Za-z_]\w*)\s*\()");
    std::string line;
    while (std::getline(in, line)) {
        if (!line.empty() && line.back() == '\r') line.pop_back();
        if (line.rfind("//", 0) == 0) continue;
        std::smatch m;
        if (std::regex_search(line, m, kDefinition)) out.insert(m[1].str());
    }
    return out;
}

}  // namespace

TEST(BackendStub, EveryMemberTheRealBackendDefinesIsDefinedByTheStubToo) {
    const fs::path dir = fs::path(SPADE_ENGINE_DIR) / "compute" / "vulkan";
    ASSERT_TRUE(fs::exists(dir / "backend.cpp")) << "the source tree was not found at " << dir.string();
    ASSERT_TRUE(fs::exists(dir / "backend_stub.cpp")) << "the source tree was not found at " << dir.string();

    const std::set<std::string> real = defined_members(dir / "backend.cpp");
    const std::set<std::string> stub = defined_members(dir / "backend_stub.cpp");
    ASSERT_GT(real.size(), 10u) << "the scan found almost nothing in backend.cpp: has its layout changed?";
    ASSERT_TRUE(real.contains("create") && stub.contains("create"));

    for (const std::string& name : real) {
        EXPECT_TRUE(stub.contains(name)) << "VulkanBackend::" << name
                                         << " is defined in backend.cpp but not in backend_stub.cpp: a SPADE_VULKAN=OFF build "
                                            "will not link (add a stub, and a line to its header comment)";
    }
}
