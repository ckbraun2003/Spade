# Third-party notices

Spade itself is licensed under the Apache License 2.0 (`LICENSE`). Each component below keeps its own licence.

Spade uses the components below. All but one are fetched by CMake at configure time, at the pins in `vendor/CMakeLists.txt`, and are not committed to this repository. The exception is the Inter font, vendored in `assets/fonts/` by Interface's UI-1. Each licence was read from the component's own upstream at its pinned version on 2026-10-05: from the fetched source's licence file, or from the upstream repository at that tag.

| Component | Version or pin | Licence | Upstream | Used by |
|---|---|---|---|---|
| GLFW | `3.4` | zlib | <https://github.com/glfw/glfw> | the sandbox window (`spade_sandbox`); the GL tests (`spade_tests`) |
| GLM | `1.0.1` | MIT (dual-licensed with the Happy Bunny License; Spade takes MIT) | <https://github.com/g-truc/glm> | engine maths, header-only (`spade_core`, `spade_render_gl`) |
| Dear ImGui | `v1.91.5-docking` | MIT | <https://github.com/ocornut/imgui> | the sandbox UI (`spade_sandbox`) |
| glad (libigl-glad) | commit `651a425101365aa6e8504988ef9bb363d066c5ee` (glad 0.1.34, GL 4.1 core) | generated code: public domain, WTFPL or CC0, at the user's choice. glad's README notes that the Apache-2.0 licence of the GL specification it was generated from may also apply. `include/KHR/khrplatform.h`: Khronos' MIT-style licence, in its header | <https://github.com/libigl/libigl-glad>; generator <https://github.com/Dav1dde/glad> | its header for GL types (`spade_render_gl`); its loader for the GL tests and the consumer smoke test |
| GoogleTest | `v1.15.2` | BSD-3-Clause | <https://github.com/google/googletest> | tests only (`spade_tests`) |
| Google Benchmark | `v1.9.1` | Apache-2.0 | <https://github.com/google/benchmark> | benchmarks only (`spade_bench`) |
| yaml-cpp | `0.8.0` | MIT | <https://github.com/jbeder/yaml-cpp> | world and scene files (`spade_world`, `spade_scene`); tests |
| nlohmann/json | `v3.12.0` | MIT (the headers Spade includes; the repository holds other licences for tooling and test files) | <https://github.com/nlohmann/json> | header-only (`spade_objects`, `spade_render`); tests |
| Vulkan-Headers | `vulkan-sdk-1.4.357.0` | Apache-2.0 OR MIT (per file; the `vulkan/` headers Spade includes carry both) | <https://github.com/KhronosGroup/Vulkan-Headers> | the Vulkan backend (`spade_compute`); tests |
| volk | `vulkan-sdk-1.4.357.0` | MIT | <https://github.com/zeux/volk> | the Vulkan loader, compiled into `spade_compute` |
| Slang | `v2026.14.1` prebuilt release (Windows and Linux archives, pinned by SHA-256 in `vendor/CMakeLists.txt`) | Apache-2.0 WITH LLVM-exception. The archive carries further licences for bundled parts in its `LICENSES/` folder | <https://github.com/shader-slang/slang> | build-time only: `slangc` compiles the kernels to SPIR-V, which is embedded in `spade_compute`. No Slang binary is linked or installed |
| Inter | `4.1` (`Inter-Regular.ttf`, `Inter-SemiBold.ttf`, unchanged from the release's `extras/ttf/`) | SIL Open Font License 1.1, copyright 2016 The Inter Project Authors. The licence is `assets/fonts/OFL.txt`, which stays beside the font files wherever they are copied | <https://github.com/rsms/inter> | the sandbox and editor UI font (vendored by Interface's UI-1) |

## Notes

- **What ends up in Spade's installed libraries:**
  - volk's source is compiled into `spade_compute`, which is installed.
  - Header code is compiled into Spade's own objects: GLM, nlohmann/json, Vulkan-Headers and yaml-cpp's headers, and glad's header for the GL types (`spade_render_gl`, which links no loader).
  - A binary distribution of Spade must therefore carry those licence notices.
  - GLFW, Dear ImGui and Inter are in the sandbox, which is built but not installed. GoogleTest, Google Benchmark and the glad loader are used by tests and benchmarks only.
- **The consumer smoke test** (`tests/consumer/`) fetches GLM, yaml-cpp and glad itself, at the same pins as above.
- **The Docker leg's image** (`scripts/docker-leg.Dockerfile`) installs Ubuntu packages to build and test. Nothing from it is distributed.
- **Each licence's full text** is in the fetched source tree (`build-ninja/<preset>/_deps/<name>-src/`) and at the upstream links above. Inter's is in this repository.
