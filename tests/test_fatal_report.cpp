// A test that dies without failing says why (test-docs/02-build-and-gate.md,
// "A test that dies without failing"; the lead, 2026-10-13).
//
// GoogleTest turns off Windows Error Reporting for its process and clears
// abort()'s report flag, and ctest prints no exit code. So a test ended by an
// escaped exception, or by a crash on a thread other than the test's, left
// nothing in LastTest.log after its "[ RUN ]". This environment installs three
// reporters. Each writes one line to stderr, which ctest keeps, and changes
// nothing else: the process still dies as it would have, with the same code.
//   - std::terminate: the escaping exception's type and what(), then abort().
//   - Windows, the unhandled-exception filter: the code in hex, the address,
//     the thread and the module; then the filter that was there before (the
//     CRT's, which turns an uncaught C++ exception into std::terminate).
//   - Windows, SIGABRT: MSVC keeps a terminate handler per thread, so a
//     terminate on any thread but the main one reaches abort() unreported.
//     abort() raises SIGABRT, whose handler is the same on every thread.
// They stay installed after the last test, so a crash in teardown is
// reported too.

#include <gtest/gtest.h>

#include <csignal>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <exception>
#include <stdexcept>
#include <thread>
#include <typeinfo>

#if defined(_WIN32)
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#endif

namespace {

[[noreturn]] void report_terminate() noexcept {
    std::fputs("[spade_tests] fatal: ", stderr);
    if (const std::exception_ptr current = std::current_exception()) {
        try {
            std::rethrow_exception(current);
        } catch (const std::exception& e) {
            std::fprintf(stderr, "std::terminate, uncaught %s: %s\n", typeid(e).name(), e.what());
        } catch (...) {
            std::fputs("std::terminate, uncaught exception not derived from std::exception\n", stderr);
        }
    } else {
        std::fputs("std::terminate with no exception in flight\n", stderr);
    }
    std::fflush(stderr);
    std::abort();
}

#if defined(_WIN32)

LPTOP_LEVEL_EXCEPTION_FILTER g_previous_filter = nullptr;

// No CRT from here on: the crash may have left the heap or a stream's lock
// unusable, so lines are built in a stack buffer and written by the OS.
void write_stderr(const char* text, std::size_t size) noexcept {
    DWORD written = 0;
    WriteFile(GetStdHandle(STD_ERROR_HANDLE), text, static_cast<DWORD>(size), &written, nullptr);
}

struct Line {
    char text[512];
    std::size_t size = 0;

    Line& add(const char* s) noexcept {
        while (*s != '\0' && size + 1 < sizeof text) text[size++] = *s++;
        return *this;
    }
    Line& hex(std::uint64_t value, int digits) noexcept {
        add("0x");
        for (int shift = (digits - 1) * 4; shift >= 0 && size + 1 < sizeof text; shift -= 4) {
            text[size++] = "0123456789ABCDEF"[(value >> shift) & 0xFu];
        }
        return *this;
    }
    Line& dec(std::uint64_t value) noexcept {
        char digits[20];
        int n = 0;
        do {
            digits[n++] = static_cast<char>('0' + value % 10u);
            value /= 10u;
        } while (value != 0u);
        while (n > 0 && size + 1 < sizeof text) text[size++] = digits[--n];
        return *this;
    }
    void write() const noexcept { write_stderr(text, size); }
};

LONG WINAPI report_unhandled(EXCEPTION_POINTERS* info) {
    const EXCEPTION_RECORD& record = *info->ExceptionRecord;
    Line()
        .add("[spade_tests] fatal: unhandled exception ")
        .hex(record.ExceptionCode, 8)
        .add(" at ")
        .hex(reinterpret_cast<std::uintptr_t>(record.ExceptionAddress), 16)
        .add(" on thread ")
        .dec(GetCurrentThreadId())
        .add("\n")
        .write();
    // The module second: finding it takes the loader lock, which the crash
    // may hold, and the line above is out by then.
    HMODULE module = nullptr;
    char path[MAX_PATH];
    if (GetModuleHandleExA(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS | GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT,
                           static_cast<LPCSTR>(record.ExceptionAddress), &module) != 0 &&
        GetModuleFileNameA(module, path, MAX_PATH) != 0) {
        Line().add("[spade_tests] fatal:   in ").add(path).add("\n").write();
    }
    return g_previous_filter != nullptr ? g_previous_filter(info) : EXCEPTION_CONTINUE_SEARCH;
}

void report_abort(int) {
    static constexpr char kLine[] = "[spade_tests] fatal: abort() called; the process exits with code 3\n";
    write_stderr(kLine, sizeof kLine - 1);
}

#endif

class FatalReport final : public ::testing::Environment {
public:
    void SetUp() override {
        std::set_terminate(&report_terminate);
#if defined(_WIN32)
        g_previous_filter = SetUnhandledExceptionFilter(&report_unhandled);
        std::signal(SIGABRT, &report_abort);
#endif
    }
};

[[maybe_unused]] ::testing::Environment* const kFatalReport =
    ::testing::AddGlobalTestEnvironment(new FatalReport);

}  // namespace

TEST(FatalReport, ItsReportersAreInstalledForEveryTest) {
    EXPECT_EQ(std::get_terminate(), &report_terminate);
#if defined(_WIN32)
    const LPTOP_LEVEL_EXCEPTION_FILTER filter = SetUnhandledExceptionFilter(nullptr);
    SetUnhandledExceptionFilter(filter);
    EXPECT_EQ(filter, &report_unhandled);
    const auto abort_handler = std::signal(SIGABRT, SIG_DFL);
    std::signal(SIGABRT, abort_handler);
    EXPECT_EQ(abort_handler, &report_abort);
#endif
}

// Each death test runs its statement in a child process and checks how it
// died: the exit status a test would have had without the reporters, and the
// reporter's line in its stderr (GoogleTest's simple regex on Windows, so no
// groups or classes).
#if defined(_WIN32)
// abort() under GoogleTest: its report flag cleared, it ends in _exit(3).
#define SPADE_ABORTED ::testing::ExitedWithCode(3)
#else
#define SPADE_ABORTED ::testing::KilledBySignal(SIGABRT)
#endif

TEST(FatalReportDeathTest, AnEscapedExceptionIsNamedWithItsMessage) {
    EXPECT_EXIT(
        {
            try {
                throw std::runtime_error("the fatal-report probe");
            } catch (...) {
                std::terminate();
            }
        },
        SPADE_ABORTED, "fatal: std::terminate, uncaught .*runtime_error: the fatal-report probe");
}

#if defined(_WIN32)

TEST(FatalReportDeathTest, ACrashOnAnotherThreadIsNamedByItsCode) {
    EXPECT_EXIT(
        {
            std::thread worker([] {
                volatile int* volatile nowhere = nullptr;
                *nowhere = 1;
            });
            worker.join();
        },
        ::testing::ExitedWithCode(static_cast<int>(EXCEPTION_ACCESS_VIOLATION)),
        "fatal: unhandled exception 0xC0000005 at 0x.* on thread ");
}

TEST(FatalReportDeathTest, AnAbortOnAnotherThreadIsReported) {
    EXPECT_EXIT(
        {
            std::thread worker([] { std::abort(); });
            worker.join();
        },
        SPADE_ABORTED, "fatal: abort.. called");
}

#endif

#undef SPADE_ABORTED
