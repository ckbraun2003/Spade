// L73a: the launcher's decisions (sandbox/launcher_logic.hpp) and the failure message a tester reads when the window
// will not open (sandbox/app_failure.hpp).

#include <gtest/gtest.h>

#include <string>
#include <vector>

#include "../sandbox/app_about.hpp"
#include "../sandbox/app_failure.hpp"
#include "../sandbox/launcher_logic.hpp"

namespace {

namespace launcher = spade::sandbox::launcher;
namespace app = spade::sandbox::app;

TEST(Launcher, ALogIsNamedByItsTimeSoTheOldestSortsFirst) {
    EXPECT_EQ(launcher::log_file_name(2026, 11, 3, 4, 5, 6), "spade-20261103-040506.log");
    EXPECT_LT(launcher::log_file_name(2026, 11, 9, 23, 59, 59), launcher::log_file_name(2026, 11, 10, 0, 0, 0));
    EXPECT_LT(launcher::log_file_name(2026, 12, 1, 0, 0, 0), launcher::log_file_name(2027, 1, 1, 0, 0, 0));
}

// The last few runs are kept: the oldest go, and nothing else in the folder is ever named.
TEST(Launcher, TheOldestLogsAreDeletedAndAStrangerIsNeverNamed) {
    const std::vector<std::string> names = {"spade-20261105-100000.log", "notes.txt",
                                            "spade-20261101-100000.log", "spade-20261103-100000.log",
                                            "spade-20261104-100000.log", "spade-20261102-100000.log",
                                            "spade-20261106-100000.log", "spade-20261107-100000.log.bak"};
    const std::vector<std::string> gone = launcher::logs_to_delete(names, 4);
    EXPECT_EQ(gone, (std::vector<std::string>{"spade-20261101-100000.log", "spade-20261102-100000.log"}))
        << "6 logs, 4 kept: the 2 oldest, oldest first";
    EXPECT_TRUE(launcher::logs_to_delete(names, 6).empty()) << "nothing over";
    EXPECT_TRUE(launcher::logs_to_delete(names, 100).empty());
    EXPECT_TRUE(launcher::logs_to_delete({}, 4).empty());
    EXPECT_EQ(launcher::kKeepLogs, 5u);
}

// The arguments go to the sandbox as they came: the shortcut for the drone box is SpadeEditor.exe --window.
TEST(Launcher, TheArgumentsAreTheCommandLineWithoutTheProgramName) {
    using launcher::arguments_after_program;
    EXPECT_EQ(arguments_after_program<std::string>("\"C:\\Program Files\\Spade\\SpadeEditor.exe\" --window --scene drone"),
              "--window --scene drone");
    EXPECT_EQ(arguments_after_program<std::string>("SpadeEditor.exe --window"), "--window");
    EXPECT_EQ(arguments_after_program<std::string>("SpadeEditor.exe"), "");
    EXPECT_EQ(arguments_after_program<std::string>("\"C:\\Program Files\\Spade\\SpadeEditor.exe\""), "");
    EXPECT_EQ(arguments_after_program<std::string>("\"C:\\Spade\\SpadeEditor.exe\"   --editor \"my scene.yaml\""),
              "--editor \"my scene.yaml\"")
        << "the blanks after the name go; the rest is verbatim, a quoted argument and its quotes included";
    EXPECT_EQ(arguments_after_program<std::wstring>(L"\"C:\\Program Files\\Spade\\SpadeEditor.exe\" --window"),
              L"--window");
}

TEST(AppFailure, ANoOpenGlFailureSaysWhatIsNeededAndWhatToDo) {
    const app::FailureText t = app::window_failure_text(app::WindowFailure::no_opengl, "no context");
    EXPECT_EQ(t.title, "Spade Editor");
    EXPECT_NE(t.body.find("OpenGL 4.3"), std::string::npos) << t.body;
    EXPECT_NE(t.body.find("graphics driver"), std::string::npos) << t.body;
}

// Every message names where the log is (a tester can send it) and ends with the technical detail; with no detail
// there is no empty "Detail" line.
TEST(AppFailure, EveryFailureNamesTheLogAndCarriesTheDetail) {
    for (const app::WindowFailure kind :
         {app::WindowFailure::no_display, app::WindowFailure::no_opengl, app::WindowFailure::other}) {
        const app::FailureText t = app::window_failure_text(kind, "GLFW said: boom");
        EXPECT_EQ(t.title, "Spade Editor");
        EXPECT_NE(t.body.find("%LOCALAPPDATA%\\Spade\\logs"), std::string::npos) << t.body;
        EXPECT_NE(t.body.find("GLFW said: boom"), std::string::npos) << t.body;
        EXPECT_EQ(t.body.find("OpenGL 4.3") != std::string::npos, kind == app::WindowFailure::no_opengl)
            << "only the OpenGL failure blames OpenGL: " << t.body;
        EXPECT_EQ(app::window_failure_text(kind, "").body.find("Detail"), std::string::npos);
    }
}

// --version and Help > About say the same version (L73a item 2), and About carries the licence and the pointer to the
// third-party notices.
TEST(AppAbout, TheVersionLineAndTheAboutTextCarryTheSameVersion) {
    EXPECT_EQ(app::version_line("0.1.0+abc1234"), "Spade Editor 0.1.0+abc1234");
    const std::string about = app::about_text("0.1.0+abc1234");
    EXPECT_NE(about.find("Spade Editor 0.1.0+abc1234"), std::string::npos) << about;
    EXPECT_NE(about.find("Apache License, Version 2.0"), std::string::npos) << about;
    EXPECT_NE(about.find("THIRD_PARTY_NOTICES"), std::string::npos) << about;
}

}  // namespace
