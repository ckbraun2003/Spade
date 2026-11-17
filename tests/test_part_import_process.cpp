#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <string>
#include <thread>
#include <vector>

#include "../tools/part_import/process.hpp"

// The child-process layer (part_import/process.hpp; L88 step 4): output captured whole, exit codes, lines as they come, a kill on cancel,
// a program that cannot start. The children are python one-liners; a machine without python skips with the reason named.

namespace {

using namespace spade::partimport;
using std::chrono::steady_clock;

[[nodiscard]] bool have_python() {
    ProcessOptions o;
    o.argv = {"python", "-c", "pass"};
    const ProcessResult r = run_process(o);
    return r.started && r.exit == 0;
}

#define NEED_PYTHON()                                                                                  \
    if (!have_python()) {                                                                              \
        GTEST_SKIP() << "no python on PATH: the process layer is tested with python one-liners";       \
    }                                                                                                  \
    static_assert(true)

[[nodiscard]] ProcessOptions py(const std::string& code) {
    ProcessOptions o;
    o.argv = {"python", "-c", code};
    return o;
}

TEST(PartImportProcess, ABadProgramDoesNotStartAndSaysWhy) {
    ProcessOptions o;
    o.argv = {"spade-no-such-program-xyz"};
    const ProcessResult r = run_process(o);
    EXPECT_FALSE(r.started);
    EXPECT_FALSE(r.start_error.empty());
    ProcessOptions none;
    const ProcessResult e = run_process(none);
    EXPECT_FALSE(e.started);
    EXPECT_FALSE(e.start_error.empty());
}

TEST(PartImportProcess, OutputAndErrorAreCapturedWholeAndTheExitCodeIsTheChilds) {
    NEED_PYTHON();
    const ProcessResult r = run_process(py("import sys; sys.stdout.write('a\\nb'); sys.stderr.write('oops'); sys.exit(4)"));
    ASSERT_TRUE(r.started) << r.start_error;
    EXPECT_FALSE(r.killed);
    EXPECT_EQ(r.exit, 4);
    EXPECT_EQ(r.out, "a\nb");
    EXPECT_EQ(r.err, "oops");
}

TEST(PartImportProcess, ALargeOutputOnBothPipesDoesNotBlockTheChild) {
    NEED_PYTHON();
    const ProcessResult r =
        run_process(py("import sys\nfor i in range(20000):\n    sys.stdout.write('x'*50+'\\n'); sys.stderr.write('y'*50+'\\n')"));
    ASSERT_TRUE(r.started) << r.start_error;
    EXPECT_EQ(r.exit, 0);
    EXPECT_EQ(r.out.size(), 20000u * 51u);
    EXPECT_EQ(r.err.size(), 20000u * 51u);
}

TEST(PartImportProcess, EachStdoutLineIsHandedOverWhole) {
    NEED_PYTHON();
    std::vector<std::string> lines;
    ProcessOptions o = py("import sys\nfor i in range(3):\n    print('line', i, flush=True)\nsys.stdout.write('tail')");
    o.on_stdout_line = [&](std::string_view l) { lines.emplace_back(l); };
    const ProcessResult r = run_process(o);
    ASSERT_TRUE(r.started) << r.start_error;
    EXPECT_EQ(lines, (std::vector<std::string>{"line 0", "line 1", "line 2", "tail"}));
    EXPECT_EQ(r.out, "line 0\nline 1\nline 2\ntail");
}

TEST(PartImportProcess, ArgumentsWithSpacesAndQuotesArriveAsGiven) {
    NEED_PYTHON();
    ProcessOptions o;
    o.argv = {"python", "-c", "import sys; print('|'.join(sys.argv[1:]))", "a b", "say \"hi\"", "c:\\dir with space\\", ""};
    const ProcessResult r = run_process(o);
    ASSERT_TRUE(r.started) << r.start_error;
    EXPECT_EQ(r.out, "a b|say \"hi\"|c:\\dir with space\\|\n");
}

TEST(PartImportProcess, ACancelKillsTheChildPromptly) {
    NEED_PYTHON();
    std::atomic<bool> cancel{false};
    std::thread later([&] {
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
        cancel = true;
    });
    ProcessOptions o = py("import time; time.sleep(60)");
    o.cancelled = [&] { return cancel.load(); };
    const auto t0 = steady_clock::now();
    const ProcessResult r = run_process(o);
    later.join();
    ASSERT_TRUE(r.started) << r.start_error;
    EXPECT_TRUE(r.killed);
    EXPECT_LT(steady_clock::now() - t0, std::chrono::seconds(20));
}

}  // namespace
