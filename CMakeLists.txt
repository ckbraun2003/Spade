cmake_minimum_required(VERSION 3.15)
project(Spade VERSION 0.1.0 LANGUAGES CXX C)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_compile_definitions(GLM_ENABLE_EXPERIMENTAL)

include(FetchContent)

# Function to help with target aliases for consistency
function(create_target_alias target alias)
    if(TARGET ${target} AND NOT TARGET ${alias})
        add_library(${alias} ALIAS ${target})
    endif()
endfunction()

# Output directories
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

# Add vendor dependencies before our modules
add_subdirectory(vendor)

# Add source and examples
add_subdirectory(src)
add_subdirectory(examples)

# Copy Assets to Binary Directory
file(COPY ${CMAKE_SOURCE_DIR}/assets DESTINATION ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})
