add_library(Spade STATIC)

# Define source and include directories for Spade
target_include_directories(Spade PUBLIC 
    ${CMAKE_CURRENT_SOURCE_DIR}/../include
)

# Collect sources (globbing for simplicity in early stage, though explicit is better for larger projects)
file(GLOB_RECURSE SPADE_SOURCES CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp" 
    "${CMAKE_CURRENT_SOURCE_DIR}/*.c"
)
target_sources(Spade PRIVATE ${SPADE_SOURCES})

# Link dependencies
target_link_libraries(Spade PUBLIC 
    glfw
    glm
    imgui
    glad
)

# Copy shaders to build directory
add_custom_command(TARGET Spade POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory
    ${CMAKE_SOURCE_DIR}/shaders
    ${CMAKE_BINARY_DIR}/bin/shaders
)
