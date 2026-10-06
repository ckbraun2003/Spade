# spade_sandbox -- the engine's reference application (SL10-SL13, Plan C).
#
# SL2b: this target may include only headers the engine INSTALLS and may link
# only spade:: targets, plus -- from task C2 -- the windowing vendor targets.
# That is why the engine link list below is aliases and nothing else: no
# spade_render, no relative include into engine/, no vendor target beyond the
# two the window genuinely needs.
#
# ⚠⚠ THE WINDOWING DEPENDENCIES ARE LINKED ONLY WHEN THERE IS A BACKEND, AND
# THAT IS A LINK-CORRECTNESS DECISION RATHER THAN A SIZE ONE. ImGui's OpenGL3
# backend references GL symbols at LINK time even when no GL call of ours is
# reached. On an image with no GL (measured 2026-09-18 on a Linux CI image --
# GL_HEADERS_PRESENT=no, X11=no, EGL=no) GLFW is built with no backend, so
# linking imgui there would fail the link for a window nobody asked it to open.
#
# With SPADE_GLFW_HAS_BACKEND=0 this target links neither, compiles
# gl_target_sink.cpp with its GL half guarded out, and the sandbox REFUSES at
# runtime with the cause named. A missing capability refuses, never degrades.
#
# ⭐ gl_target_sink.cpp is in the source list UNCONDITIONALLY. Its conversion,
# its frame budget and its refusal path are logic, and they keep their Linux
# compile coverage; only the code that touches GLFW/GL/ImGui sits behind the
# guard. That is the mitigation plan-c-sandbox.md names for this task.

add_executable(spade_sandbox
    main.cpp
    gl_target_sink.cpp
    # Each scene's setup and its one frame function, shared by the window,
    # the builder's --smoke and the live smoke.
    scene_sessions.cpp
    # The built-in scene files (assets/scenes/) run in the window: the live
    # smoke's scene main functions.
    scene_file_session.cpp
    # --live-smoke: the recorded tour (scripts/live-smoke.ps1). Its verdict
    # machinery is display-free in live_smoke.hpp and tested in spade_tests.
    live_tour.cpp
)

target_link_libraries(spade_sandbox PRIVATE
    spade::core
    spade::world
    spade::render
    # The drone sim box steps a Simulation (drone_sim.hpp) -- the first scene
    # in the sandbox that does.
    spade::sim
    # The built-in scene files run through compose_file() and instantiate()
    # (scene_file_session.cpp).
    spade::scene
)

if(SPADE_RENDER_GL)
    # The PRIMARY render path. Linked only when the option is on; with it off
    # the sandbox still builds and runs on raster_cpu, which is exactly what
    # "the cpu is the fallback" has to mean for a build with no GL at all.
    target_link_libraries(spade_sandbox PRIVATE spade::render_gl)
    target_compile_definitions(spade_sandbox PRIVATE SPADE_SANDBOX_HAS_GPU=1)
else()
    target_compile_definitions(spade_sandbox PRIVATE SPADE_SANDBOX_HAS_GPU=0)
endif()

target_compile_features(spade_sandbox PRIVATE cxx_std_23)

if(WIN32)
    # GetProcessMemoryInfo, for the memory figure the user asked to sit beside
    # the fps. Linked OUTSIDE the backend guard on purpose: working_set_bytes()
    # is declared unconditionally, so it must link in a no-backend build too --
    # a symbol that resolves on only one configure path fails at somebody
    # else's link rather than at mine.
    target_link_libraries(spade_sandbox PRIVATE psapi)
endif()

if(SPADE_GLFW_HAS_BACKEND)
    target_compile_definitions(spade_sandbox PRIVATE
        SPADE_SANDBOX_HAS_GL=1
        # glfw3.h:241 pulls <GL/gl.h> without this, which is what broke the
        # no-GL-headers probe until the macro was added. Kept here as well as
        # on the imgui target because THIS target includes glfw3.h itself, and
        # a definition that arrives only transitively is one refactor from
        # vanishing.
        GLFW_INCLUDE_NONE
    )
    target_link_libraries(spade_sandbox PRIVATE glfw imgui)

    if(WIN32)
        target_compile_definitions(spade_sandbox PRIVATE
            # TARGET-scoped, never directory-scoped: windows.h's unguarded
            # min/max macros shadow GLM's quaternion_exponential.inl, which is
            # the same defect CMakeLists.txt already records against v1.
            NOMINMAX
            WIN32_LEAN_AND_MEAN
        )
        # GL 1.1 -- glGenTextures/glBindTexture/glTexImage2D/glTexParameteri and
        # the GL_RGBA/GL_LINEAR constants -- is what this target calls, and
        # opengl32 is where Windows exports it. Everything above 1.1 is ImGui's
        # problem and ImGui loads it itself.
        target_link_libraries(spade_sandbox PRIVATE opengl32)
    else()
        find_package(OpenGL QUIET)
        if(OpenGL_FOUND)
            target_link_libraries(spade_sandbox PRIVATE OpenGL::GL)
        endif()
    endif()
else()
    target_compile_definitions(spade_sandbox PRIVATE SPADE_SANDBOX_HAS_GL=0)
    message(STATUS
        "spade_sandbox: no windowing backend (SPADE_GLFW_HAS_BACKEND=0). "
        "The target builds and --headless works; the window refuses at runtime with the cause.")
endif()

# The UI font (the editor design's §13; assets/fonts/README.md), copied beside
# the executable, where the window loads it from whatever the working
# directory. Its OFL licence travels with it, as the licence requires.
set(SPADE_SANDBOX_FONTS
    "${CMAKE_CURRENT_SOURCE_DIR}/../assets/fonts/Inter-Regular.ttf"
    "${CMAKE_CURRENT_SOURCE_DIR}/../assets/fonts/Inter-SemiBold.ttf"
    "${CMAKE_CURRENT_SOURCE_DIR}/../assets/fonts/OFL.txt")
add_custom_command(TARGET spade_sandbox POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E make_directory "$<TARGET_FILE_DIR:spade_sandbox>/fonts"
    COMMAND ${CMAKE_COMMAND} -E copy_if_different ${SPADE_SANDBOX_FONTS} "$<TARGET_FILE_DIR:spade_sandbox>/fonts/"
    VERBATIM)

# NOT installed, and that is deliberate: SL2a gives Spade a release identity of
# eleven LIBRARY modules. The sandbox is a consumer of that surface, not part
# of it -- installing it would put a developer tool in a package whose whole
# claim is "a domain-neutral engine an out-of-tree consumer links against".
