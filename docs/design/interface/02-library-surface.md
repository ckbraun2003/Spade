# The library surface

What Spade installs, how a consumer links it, and the rule that keeps Spade's own tools honest about it. What exists is in `07-status.md`.

## What a consumer gets (`SL2`)

Spade installs as a CMake package. Each engine module is a static library with a `spade::` alias, installed with `install(TARGETS … EXPORT spadeTargets)`, its headers installed per module directory (`include/<module>/`), and a generated `spadeConfig.cmake` (`cmake/spadeConfig.cmake.in`) that re-creates the third-party targets the export cannot carry (glm, yaml-cpp) before including the export. A consumer writes:

```cmake
find_package(spade CONFIG REQUIRED)
target_link_libraries(app PRIVATE spade::sim spade::render)
```

`tests/consumer/` is the worked out-of-tree example: it authors a world, saves it as a world file, reloads it, builds a world set from the reloaded copy and steps a Simulation, so it proves the file format and the package rather than the in-memory builder. It is load-bearing: it is how a link gap the in-tree tests cannot see (a static archive extracts only the members it needs) gets caught.

**Not shipped:** the sandbox (a developer tool, not a library module; `sandbox/CMakeLists.txt`), `engine/testing/` (test support), and the v1 engine. Packaging beyond CMake install — a registry, binaries, an ABI promise — is out of scope (`SL2a`).

## Release identity (`SL2a`)

Spade has its own changelog (`CHANGELOG.md`), a README that a reader who has never heard of any consumer can build, install and consume from, and a versioning policy for `project(Spade VERSION …)`. Today's version is `0.2.0`. Consumers are described in `../consumers.md` and nowhere else.

## The sandbox uses only the public surface (`SL2b`)

`spade_sandbox` includes only headers the engine installs and links only `spade::` targets, plus the two windowing vendor targets the window needs (GLFW, ImGui) and nothing else. It never reaches into an engine directory by relative path.

This is the most useful constraint Interface has: the sandbox, and the editor after it, is Spade's largest consumer, so the rule makes it a continuous proof that the public API is enough. When the editor needs something the API does not expose, that is a **finding about the library**, reported to Core, not a reason to widen an include path. The same holds for templates (`engine D4`'s template half): a template is built on the public API and never linked into the core.

The rule is kept by review today. Its guard — a check over the sandbox's include graph that fails on a seeded reach into an engine internal — is owed (`SL18` obligation 6, Test/Docs's; `07-status.md`).

## Templates and examples

A template is a ready-made assembly of objects, components and parameters, built on the public API (`../01-engine-model.md`). The quadrotor is the first: today `make_quadrotor()` and `hover_command()` are engine code in `vehicles/quadrotor.*` (Physics), and the drone sim box's controller and mixer (`sandbox/drone_sim.hpp`) are scene code written as template material. Moving the quadrotor out of the core into a template is future work, after Core's module API exists. Example content such as `quad_hover` and `gate` belongs to the quadrotor template.
