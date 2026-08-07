# Spade Engine

Spade is a high-performance, hybrid CPU/GPU simulation engine built on a Data-Oriented **Entity-Component-System (ECS)** architecture. It leverages Compute Shaders for massive parallel physics simulations and Instanced Rendering to display thousands of objects efficiently.

> **Status (2026-08):** Spade now lives inside the Kat monorepo (`spade/`, subtree-merged with full history) as the engine beneath Kat's sim stack. Its v2 direction — Vulkan, fixed-step deterministic stepping, a CPU reference twin for every authoritative GPU pass, headless/offscreen operation, seeded RNG, snapshots — is pinned in `design-specs/kat-spade-simstack-foundation.html` (local, untracked). The engine design session that details the technicals happens against that spec; the gap register there is the authoritative list of what changes. This README documents the engine **as it is today**.

## Core Architecture

Spade moves away from traditional OOP hierarchies in favor of direct backing data structures that map 1:1 to GPU buffers.

-   **Hybrid ECS**:
    -   **CPU**: Manages high-level logic, inputs, and component pools (`Universe`, `ComponentPool`, `Entity`).
    -   **GPU**: Executes heavy physics and collision logic via **Compute Shaders** and **SSBOs** (Shader Storage Buffer Objects).
-   **Instanced Rendering**: All entities sharing a mesh are rendered in a single draw call using `glDrawElementsInstanced`.
-   **Spatial Hashing Collision**: Implements a GPU-based **Sorted Grid** algorithm (Bitonic Sort) to achieve `O(N)` average-case complexity for collisions, allowing for tens of thousands of interacting particles.
-   **SPH Fluids**: Smoothed-particle hydrodynamics (density + force passes) sharing the same spatial grid.

## Getting Started

### Prerequisites

*   C++20 (`CMAKE_CXX_STANDARD 20`)
*   CMake 3.15+
*   OpenGL **4.3 core** support (required for Compute Shaders and SSBOs)
*   Windows (uses `windows.h`/`psapi.h` for process stats; platform unlock is a v2 item)

Dependencies are **fetched at configure time** via CMake `FetchContent` (see `vendor/CMakeLists.txt` — nothing is vendored in-tree):

| Dependency | Pin | Role |
|---|---|---|
| GLFW | tag `3.4` | Windowing / input / GL context |
| GLM | tag `1.0.1` | Mathematics |
| GLAD | `libigl/libigl-glad` @ master | OpenGL loading (4 compute/image entry points hand-loaded on top) |
| Dear ImGui | `docking` branch | Linked but not yet used in the engine |

### Building and Running

```bash
mkdir build
cd build
cmake ..
cmake --build .
```

Run the sandbox example (binaries land in `build/bin/`):

```bash
./bin/Sandbox
```

## Usage

`examples/sandbox/main.cpp` is the canonical, always-accurate example. The shape of a Spade program:

```cpp
#include <Spade/Spade.hpp>

using namespace Spade;

Engine engine;
Universe universe;

int main() {
    // 1. Entities: camera + an instanced particle system
    //    (see examples/sandbox/main.cpp for full component setup)

    // 2. Window + GL context (required before any buffer load)
    engine.SetupEngineWindow(1280, 720, "Spade Sandbox");

    // 3. Upload data to GPU SSBOs
    engine.LoadInstanceBuffers(universe);   // Transforms, Motions, Materials
    engine.LoadCameraBuffers(universe);     // Camera uniforms
    engine.LoadCollisionBuffers(universe);  // Bounding data

    float bounds   = 20.0f;  // World half-extent
    float cellSize = 1.0f;   // Grid cell size (> largest particle diameter)
    int   substeps = 4;

    // 4. Main loop
    while (engine.IsRunning()) {
        engine.ProcessInput(universe);

        float stepTime = engine.GetDeltaTime() / substeps;
        for (int i = 0; i < substeps; ++i) {
            engine.EnableGravity(-9.8f);
            engine.EnableGridCollision(bounds, cellSize);
            engine.EnableMotion(stepTime);
        }

        engine.RenderColor();
        engine.DrawScene(universe);  // also updates frame statistics
    }
    return 0;
}
```

## API Reference

### `Spade::Engine`

#### Setup & buffer loading
*   `SetupEngineWindow(width, height, title)` — creates the GLFW window and GL 4.3 context. Must precede all buffer loads.
*   `LoadInstanceBuffers(Universe&)` — flattens and uploads `MeshComponent` instance vectors (`Transform`, `Motion`, `Material`) to GPU SSBOs. Call after spawning entities.
*   `LoadCollisionBuffers(Universe&)` — uploads `BoundingComponent` data.
*   `LoadCameraBuffers(Universe&)` — uploads active camera data.
*   `LoadFluidBuffers(Universe&)` — uploads `FluidComponent` data (SPH).
*   `LoadGridBuffers()` — allocates the spatial-hash grid buffers.

#### Physics pipeline (GPU compute dispatches)
*   `EnableGravity(float gravity)` — applies constant acceleration to all instances with motion.
*   `EnableMotion(float deltaTime)` — integrates velocity → position (semi-implicit Euler), then clears acceleration.
*   `EnableGridCollision(float bounds, float cellSize)` — the **Spatial Hashing** pipeline: grid build → bitonic sort → offsets → reorder → solve → scatter. `bounds` is the half-extent of the simulation box; `cellSize` must exceed the largest object diameter.
*   `EnableBruteForceCollision(float bounds)` — legacy O(N²) collision.
*   `EnableSPHFluid(float globalBounds, float cellSize)` — SPH density + force passes over the shared grid.
*   `EnableBruteForceNewtonianGravity(float G)` — **stub**: shader exists, C++ body is empty.

#### Rendering & input
*   `RenderColor()` / `RenderVelocity()` / `RenderWireframe()` — select the active fragment path.
*   `RenderShader(name, fragmentShaderFile, geometryShaderFile = "")` — bind a custom shader program.
*   `DrawScene(Universe&, clearColor = {0,0,0,1})` — instanced draw calls for all meshes; also updates frame statistics (delta time, FPS, memory).
*   `ProcessInput(Universe&)` — updates entities carrying an `InputComponent`.
*   `IsRunning()` · `GetTime()` · `GetDeltaTime()` · `GetFPS()` · `GetMemory()` · `IsKeyPressed(key)` · `IsPlaying()` · `IsMouseButtonPressed(button)` · `GetMousePosition()` · `SetMouseCursorMode()`.

### Components (`Spade/Core/Components.hpp`)

*   **MeshComponent** — holds the instance vectors (`instanceTransforms`, `instanceMotions`, `instanceMaterials`) plus the GL mesh objects.
*   **BoundingComponent** — physical properties (`size`, `friction`, `bounciness`) shared by all instances of the entity.
*   **FluidComponent** — SPH material parameters.
*   **CameraComponent** — FOV and clip planes.
*   **InputComponent** — keybindings and movement speed.

### Data structures (`Spade/Core/Primitives.hpp`)

Manually padded to match GLSL `std430` alignment; POD and memcpy-able by design.

-   `Transform`: `vec3` pos, `quat` rot, `vec3` scale.
-   `Motion`: `vec3` vel, `float` mass, `vec3` accel.
-   `Material`: `vec4` color, `float` metallic/roughness/emission.
