# Packaging.cmake -- the Windows installer for "Spade Editor" (L73a; release design D90 to D95). Included from the END of the root
# CMakeLists.txt, so CPack sees every target.
#
# WHAT IT MAKES. One CPack component, `editor`: spade_sandbox.exe (the editor), Interface's WIN32 launcher SpadeEditor.exe when its target
# exists, spade_geo_fetch.exe (the map fetch the editor runs; L85e), assets/{library,scenes,worlds,fonts}, the licence and notices, the release docs, the user guide's HTML and PDF, and the MSVC
# runtime app-local. Nothing else: no tests, no flow_movie, no SDK headers or libraries.
#
# EVERY RULE IS EXCLUDE_FROM_ALL. `cmake --install` with no component is the SDK install (the eleven library modules; sandbox/CMakeLists.txt
# keeps the sandbox out of it on purpose), so the editor's files only install when the `editor` component is named: `cmake --install
# <build> --component editor --prefix <dir>`, which is what CPack and scripts/package.ps1 do.
#
# THE MSVC RUNTIME IS APP-LOCAL, NOT /MT. InstallRequiredSystemLibraries finds the redistributable DLLs of the toolchain that built the
# exe, and they go beside it (vcruntime140*.dll, msvcp140*.dll, the UCRT) so a machine without the Visual C++ Redistributable runs it.
# A static /MT would change the CRT of every target and vendor library in the build for one installer; this is install-time only.
#
# THE PACKAGE'S INPUTS FROM OUTSIDE THE TREE are the user guide's build (Test/Docs' docs/user-guide/build.ps1): the ENVIRONMENT variables
# SPADE_USER_GUIDE_DIR (its site\ folder) and SPADE_USER_GUIDE_PDF (its PDF), read when the component is INSTALLED, not at configure, so
# a new guide needs no reconfigure. scripts/package.ps1 sets both. Without them the install warns and carries no guide; package.ps1 refuses that.
if(NOT WIN32 OR NOT SPADE_BUILD_SANDBOX)
    return()
endif()

file(READ "${PROJECT_SOURCE_DIR}/packaging/common/VERSION" _spade_version_text)
string(REGEX MATCH "^[ \t\r\n]*([0-9]+\\.[0-9]+\\.[0-9]+)" _matched "${_spade_version_text}")
if(NOT _matched)
    message(FATAL_ERROR "packaging/common/VERSION must begin with MAJOR.MINOR.PATCH")
endif()
set(SPADE_EDITOR_VERSION "${CMAKE_MATCH_1}")

# ---- the install rules, component editor ----------------------------------------------------------------------------------------------
set(_editor_install EXCLUDE_FROM_ALL COMPONENT editor)

install(TARGETS spade_sandbox RUNTIME DESTINATION . ${_editor_install})
set(_editor_exe "spade_sandbox")
# What scripts/package.ps1 builds: the editor's executables, and nothing of the SDK or the tests.
add_custom_target(spade_editor_files DEPENDS spade_sandbox)
# The map fetch tool (tools/geo_fetch; L85e): the editor runs it beside itself. A console program, so a tester can also run it by hand.
if(TARGET spade_geo_fetch)
    add_dependencies(spade_editor_files spade_geo_fetch)
    install(TARGETS spade_geo_fetch RUNTIME DESTINATION . ${_editor_install})
endif()
# The part import tool (tools/part_import; L88): the editor runs it beside itself. step_convert.py ships beside it, the STEP route's
# script (python and OpenCASCADE are the user's own install: spade_part_import --probe says how; they are never bundled).
if(TARGET spade_part_import)
    add_dependencies(spade_editor_files spade_part_import)
    install(TARGETS spade_part_import RUNTIME DESTINATION . ${_editor_install})
    install(FILES "${PROJECT_SOURCE_DIR}/tools/step_convert/step_convert.py" DESTINATION . ${_editor_install})
endif()
if(TARGET spade_editor_launcher)
    add_dependencies(spade_editor_files spade_editor_launcher)
    # Interface's WIN32 launcher (OUTPUT_NAME SpadeEditor): no console, output to %LOCALAPPDATA%\Spade\logs, arguments passed through.
    install(TARGETS spade_editor_launcher RUNTIME DESTINATION . ${_editor_install})
    set(_editor_exe "SpadeEditor")
endif()

# The asset root is <install>/assets; the editor finds it from its own exe (Interface). README.md files are repository notes.
install(DIRECTORY
    "${PROJECT_SOURCE_DIR}/assets/library"
    "${PROJECT_SOURCE_DIR}/assets/scenes"
    "${PROJECT_SOURCE_DIR}/assets/worlds"
    "${PROJECT_SOURCE_DIR}/assets/fonts"
    "${PROJECT_SOURCE_DIR}/assets/models"
    DESTINATION assets ${_editor_install}
    PATTERN "README.md" EXCLUDE)

install(FILES
    "${PROJECT_SOURCE_DIR}/LICENSE"
    "${PROJECT_SOURCE_DIR}/THIRD_PARTY_NOTICES.md"
    "${PROJECT_SOURCE_DIR}/packaging/common/VERSION"
    "${PROJECT_SOURCE_DIR}/packaging/common/INSTALL.md"
    "${PROJECT_SOURCE_DIR}/packaging/common/RELEASE_NOTES.md"
    "${PROJECT_SOURCE_DIR}/packaging/common/SYSTEM_REQUIREMENTS.md"
    DESTINATION . ${_editor_install})

install(CODE [[
    set(_guide "$ENV{SPADE_USER_GUIDE_DIR}")
    set(_pdf "$ENV{SPADE_USER_GUIDE_PDF}")
    set(_dst "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/docs/user-guide")
    if(_guide AND EXISTS "${_guide}/index.html")
        file(INSTALL DESTINATION "${_dst}" TYPE DIRECTORY FILES "${_guide}/")
        if(_pdf AND EXISTS "${_pdf}")
            file(INSTALL DESTINATION "${_dst}" TYPE FILE FILES "${_pdf}")
        endif()
    else()
        message(WARNING "packaging: SPADE_USER_GUIDE_DIR is unset or has no index.html; installing WITHOUT the user guide")
    endif()
]] ${_editor_install})

# The runtime, app-local. The module's own install() is skipped so its files go through our EXCLUDE_FROM_ALL rule and the editor component.
set(CMAKE_INSTALL_SYSTEM_RUNTIME_LIBS_SKIP TRUE)
set(CMAKE_INSTALL_UCRT_LIBRARIES TRUE)
include(InstallRequiredSystemLibraries)
if(CMAKE_INSTALL_SYSTEM_RUNTIME_LIBS)
    install(FILES ${CMAKE_INSTALL_SYSTEM_RUNTIME_LIBS} DESTINATION . ${_editor_install})
else()
    message(WARNING "packaging: InstallRequiredSystemLibraries found no MSVC runtime; the installer would not run on a machine without the Visual C++ Redistributable")
endif()

# ---- CPack: the Inno Setup installer -----------------------------------------------------------------------------------------------------
set(CPACK_GENERATOR "INNOSETUP")
set(CPACK_PACKAGE_NAME "Spade Editor")
set(CPACK_PACKAGE_VENDOR "Spade")
set(CPACK_PACKAGE_VERSION "${SPADE_EDITOR_VERSION}")
set(CPACK_PACKAGE_INSTALL_DIRECTORY "Spade Editor")
# scripts/package.ps1 overrides this with the version and the SHA: Spade-Editor-<version>-<sha7>-setup.
set(CPACK_PACKAGE_FILE_NAME "Spade-Editor-${SPADE_EDITOR_VERSION}-setup")
set(CPACK_RESOURCE_FILE_LICENSE "${PROJECT_SOURCE_DIR}/LICENSE")
set(CPACK_COMPONENTS_ALL editor)
# One component, always installed: Inno's generator has only a "custom" type, so an unflagged component is unticked in a silent install
# (/VERYSILENT installed no files). REQUIRED gives it Inno's "fixed" flag.
set(CPACK_COMPONENT_EDITOR_REQUIRED TRUE)

set(CPACK_INNOSETUP_ARCHITECTURE "x64")
set(CPACK_INNOSETUP_USE_MODERN_WIZARD ON)
# Per user, no administrator prompt: PrivilegesRequired=lowest installs under %LOCALAPPDATA%\Programs and registers the uninstaller under HKCU.
set(CPACK_INNOSETUP_SETUP_PrivilegesRequired "lowest")
# The files sit at the install root, not under bin/.
set(CPACK_INNOSETUP_EXECUTABLES_DIRECTORY ".")
set(CPACK_PACKAGE_EXECUTABLES "${_editor_exe}" "Spade Editor")
set(CPACK_CREATE_DESKTOP_LINKS "${_editor_exe}")

# The two Start-menu shortcuts CPack's executable list cannot express (arguments, a document): Spade Drone Box and Spade User Guide.
set(SPADE_EDITOR_EXE_NAME "${_editor_exe}")
configure_file("${PROJECT_SOURCE_DIR}/packaging/windows/shortcuts.iss.in"
    "${CMAKE_BINARY_DIR}/packaging/shortcuts.iss" @ONLY)
set(CPACK_INNOSETUP_EXTRA_SCRIPTS "${CMAKE_BINARY_DIR}/packaging/shortcuts.iss")

# CPACK_INNOSETUP_EXECUTABLE (ISCC.exe) is passed by scripts/package.ps1, which finds it.
include(CPack)
