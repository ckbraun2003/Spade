set(TARGET_NAME Benchmark)

add_executable(${TARGET_NAME} main.cpp)

target_link_libraries(${TARGET_NAME} PRIVATE Spade)

# Copy assets if needed (nothing specific yet)
