# SpadePython.cmake -- which Python the build runs its generators with (L75).
#
# embed_spirv.py and gen_layout_check.py (engine/shaders/tools/, standard library only) write every *.spv.gen.hpp and the layout
# check. They give the same bytes under any Python 3: the 62 commands of a release tree were run under the Microsoft Store's
# 3.12 and under MSYS2's 3.14 and all 63 files they write compare equal. Which interpreter runs them therefore changes no output.
#
# What it does change is whether the build runs at all on some machines: the Store's Python is an app-execution alias under
# C:\Program Files\WindowsApps, and starting it from a build tool sometimes fails with "Access is denied". So the build WARNS
# when CMake picked that one, and names the fix. It never refuses: the warning is information, not a gate.
#
# scripts/build.ps1 passes -DPython3_EXECUTABLE=<a non-Store Python> when it configures a tree (scripts/find-python.ps1 finds one).

# Sets <out_var> to a warning text when <python_exe> is a Microsoft Store Python (a path through WindowsApps), else to "".
function(spade_python_warning python_exe out_var)
    set(_text "")
    if("${python_exe}" MATCHES "[Ww][Ii][Nn][Dd][Oo][Ww][Ss][Aa][Pp][Pp][Ss]")
        string(CONCAT _text
            "The Python for the shader generators is a Microsoft Store Python:\n  ${python_exe}\n"
            "Its app-execution alias can fail a build with 'Access is denied'. The generated files are byte-identical under any "
            "Python 3, so pick another: reconfigure with -DPython3_EXECUTABLE=<a python.exe outside WindowsApps>, or build with "
            "scripts/build.ps1, which does that for a new tree.")
    endif()
    set(${out_var} "${_text}" PARENT_SCOPE)
endfunction()

# find_package(Python3) for the generators, and the warning above when it chose the Store's.
macro(spade_find_python)
    find_package(Python3 REQUIRED COMPONENTS Interpreter)
    spade_python_warning("${Python3_EXECUTABLE}" _spade_python_warning)
    # SpadeSlang.cmake is included from more than one directory: say it once per configure.
    get_property(_spade_python_warned GLOBAL PROPERTY SPADE_PYTHON_WARNED)
    if(_spade_python_warning AND NOT _spade_python_warned)
        message(WARNING "${_spade_python_warning}")
        set_property(GLOBAL PROPERTY SPADE_PYTHON_WARNED TRUE)
    endif()
endmacro()
