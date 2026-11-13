# SpadePython.StoreWarning (tests/CMakeLists.txt): cmake/SpadePython.cmake's warning about a Microsoft Store Python.
# That interpreter's alias fails some builds with "Access is denied" (the Store's app-execution alias), so the build names
# it and says how to pick another; any other interpreter gets no warning. The warning is a text returned in a variable,
# so this script checks it without a configure.
#
#   cmake -DPYTHON_CHOICE_SCRIPT=<cmake/SpadePython.cmake> -P python_choice_test.cmake
cmake_minimum_required(VERSION 3.28)

if(NOT EXISTS "${PYTHON_CHOICE_SCRIPT}")
    message(FATAL_ERROR "SpadePython: no script at ${PYTHON_CHOICE_SCRIPT}")
endif()
include("${PYTHON_CHOICE_SCRIPT}")

set(_failed FALSE)
function(expect_warning path)
    spade_python_warning("${path}" _text)
    if("${_text}" STREQUAL "")
        message(SEND_ERROR "SpadePython: no warning for the Store interpreter ${path}")
        set(_failed TRUE PARENT_SCOPE)
        return()
    endif()
    foreach(needle "${path}" "-DPython3_EXECUTABLE")
        string(FIND "${_text}" "${needle}" _at)
        if(_at EQUAL -1)
            message(SEND_ERROR "SpadePython: the warning for ${path} does not contain '${needle}':\n${_text}")
            set(_failed TRUE PARENT_SCOPE)
        endif()
    endforeach()
endfunction()
function(expect_quiet path)
    spade_python_warning("${path}" _text)
    if(NOT "${_text}" STREQUAL "")
        message(SEND_ERROR "SpadePython: a warning for ${path}, which is not a Store interpreter:\n${_text}")
        set(_failed TRUE PARENT_SCOPE)
    endif()
endfunction()

# The interpreter CMake picked on the machine this was written on, the Store's alias, and the case and slash variants of it.
expect_warning("C:/Program Files/WindowsApps/PythonSoftwareFoundation.Python.3.12_3.12.2800.0_x64__qbz5n2kfra8p0/python3.12.exe")
expect_warning("C:\\Program Files\\WindowsApps\\PythonSoftwareFoundation.Python.3.12_3.12.2800.0_x64__qbz5n2kfra8p0\\python3.12.exe")
expect_warning("C:/Users/Someone/AppData/Local/Microsoft/WindowsApps/python.exe")
expect_warning("c:/program files/windowsapps/pythonsoftwarefoundation.python.3.12_x64__q/python3.12.exe")

# Interpreters that are not the Store's: no warning.
expect_quiet("C:/msys64/ucrt64/bin/python.exe")
expect_quiet("C:/Python312/python.exe")
expect_quiet("/usr/bin/python3")
expect_quiet("")

if(_failed)
    message(FATAL_ERROR "SpadePython: see the errors above")
endif()
message(STATUS "SpadePython: Store interpreters warn, others do not")
