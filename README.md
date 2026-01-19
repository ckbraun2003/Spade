# Spade Engine

Spade is a high-performance, hybrid CPU/GPU simulation engine built on a Data-Oriented **Entity-Component-System (ECS)** architecture. It leverages Compute Shaders for massive parallel physics simulations and Instanced Rendering to display thousands of objects efficiently.

## Core Architecture

Spade moves away from traditional OOP hierarchies in favor of direct backing data structures that map 1:1 to GPU buffers.

-   **Hybrid ECS**: 
    -   **CPU**: Manages high-level logic, inputs, and component pools (`Universe`, `Component`).
    -   **GPU**: Executes heavy physics and collision logic via **Compute Shaders** and **SSBOs** (Shader Storage Buffer Objects).
-   **Instanced Rendering**: All entities sharing a mesh are rendered in a single draw call using `glDrawElementsInstanced`.
-   **Spatial Hashing Collision**: Implements a GPU-based **Sorted Grid** algorithm (Bitonic Sort) to achieve `O(N)` average-case complexity for collisions, allowing for tens of thousands of interacting particles.

## Getting Started

### Prerequisites

*   C++17 or later
*   CMake 3.10+
*   OpenGL 4.6 support (Required for Compute Shaders and SSBOs)
*   Dependencies (Vendored):
    -   GLFW (Windowing)
    -   GLAD (OpenGL Loading)
    -   GLM (Mathematics)

### Building and Running

1.  Clone the repository.
2.  Configure with CMake:
    ```bash
    mkdir build
    cd build
    cmake ..
    ```
3.  Build:
    ```bash
    cmake --build .
    ```
4.  Run the sandbox example:
    ```bash
    ./examples/sandbox/SpadeSandbox
    ```

## Usage Example

The following example demonstrates how to set up a massive particle simulation with the Spade engine.

```cpp
#include <Spade/Spade.hpp>
#include <iostream>

using namespace Spade;

Engine engine;
Universe universe;

int main() {
    // 1. Create a Camera Entity
    EntityID camID = universe.CreateEntityID();
    Entity camera(camID, &universe);
    
    // Configure Camera
    camera.AddComponent<TransformComponent>()->transform.position = {0.0, 5.0, 15.0};
    camera.AddComponent<CameraComponent>()->fov = 60.0;
    
    // Add Input for flying around
    auto* input = camera.AddComponent<InputComponent>();
    input->speed = 10.0f;
    input->bindings[GLFW_KEY_W] = MoveForward;
    input->bindings[GLFW_KEY_S] = MoveBackward;
    input->bindings[GLFW_KEY_A] = MoveLeft;
    input->bindings[GLFW_KEY_D] = MoveRight;

    // 2. Create the Particle System Entity
    // In Spade, one Entity can hold thousands of instances of a mesh.
    EntityID particlesID = universe.CreateEntityID();
    Entity particles(particlesID, &universe);

    // Add Collision Bounds (Shared by all instances in this mesh)
    auto* boundsComp = particles.AddComponent<BoundingComponent>();
    boundsComp->bound.size = 0.5f;        // Diameter
    boundsComp->bound.isSphere = true;
    boundsComp->bound.bounciness = 0.8f;
    boundsComp->bound.friction = 0.5f;

    // Add Mesh and Instances
    auto* meshComp = particles.AddComponent<MeshComponent>();
    meshComp->mesh = GenerateSphere(0.25f, 16, 16); // Radius = size/2

    // Spawn 1000 Particles
    for (int i = 0; i < 1000; ++i) {
        Transform t;
        t.position = { (float)(rand()%10 - 5), (float)(rand()%10 + 5), (float)(rand()%10 - 5) };
        t.rotation = { 1.0, 0.0, 0.0, 0.0 };
        t.scale = { 1.0, 1.0, 1.0 };
        
        Motion m;
        m.velocity = { 0.0, 0.0, 0.0 };
        m.mass = 1.0;
        m.acceleration = { 0.0, 0.0, 0.0 };
        
        Material mat;
        mat.color = { (float)rand()/RAND_MAX, (float)rand()/RAND_MAX, (float)rand()/RAND_MAX, 1.0 };

        meshComp->instanceTransforms.push_back(t);
        meshComp->instanceMotions.push_back(m);
        meshComp->instanceMaterials.push_back(mat);
    }

    // 3. Setup Window
    engine.SetupEngineWindow(1280, 720, "Spade Sandbox");

    // 4. Upload Data to GPU (SSBOs)
    engine.LoadInstanceBuffers(universe); // Upload Transforms, Motions, Materials
    engine.LoadCameraBuffers(universe);   // Upload Camera Uniforms
    engine.LoadCollisionBuffers(universe); // Upload Bounding Data

    // Simulation Parameters
    float bounds = 20.0f;    // World size
    float cellSize = 1.0f;   // Grid cell size (should be > largest particle)
    int substeps = 4;        // Physics accuracy

    // 5. Main Loop
    while (engine.IsRunning()) {
        engine.UpdateStatistics(); // Calculate DeltaTime/FPS
        std::cout << "FPS: " << engine.GetFPS() << std::endl;

        engine.ProcessInput(universe);

        float dt = engine.GetDeltaTime();
        float stepTime = dt / substeps;

        // Physics Sub-stepping
        for(int i=0; i<substeps; ++i) {
            engine.EnableGravity(9.8f, stepTime);
            engine.EnableMotion(stepTime);
            
            // Choose ONE collision system:
            // engine.EnableCollision(bounds, stepTime); // Brute Force (Slow, O(N^2))
            engine.EnableGridCollision(bounds, cellSize, stepTime); // Sorted Grid (Fast, O(N))
        }

        engine.DrawScene(universe);
    }

    return 0;
}
```

## API Reference

### `Spade::Engine`

The main controller for the simulation.

#### Setup & Buffer Loading
*   `SetupEngineWindow(width, height, title)`: Creates the GLFW window and context.
*   `LoadInstanceBuffers(Universe&)`: Flattens and uploads `MeshComponent` instance vectors (`Transform`, `Motion`, `Material`) to GPU SSBOs. Call this after spawning entities.
*   `LoadCollisionBuffers(Universe&)`: Uploads `BoundingComponent` data.
*   `LoadCameraBuffers(Universe&)`: Uploads active camera data.

#### Physics pipeline
*   `EnableGravity(gravity, deltaTime)`: Applies downward acceleration to all instances with motion.
*   `EnableMotion(deltaTime)`: Integrates Velocity -> Position.
*   `EnableGridCollision(globalBounds, cellSize, deltaTime)`: Runs the **Spatial Hashing** pipeline.
    *   `globalBounds`: Half-extent of the simulation box (e.g., 20.0 = -20 to +20).
    *   `cellSize`: Size of grid cells. Must be larger than the largest object diameter.
*   `EnableCollision(globalBounds, deltaTime)`: Runs the legacy Brute Force collision (O(N^2)).

#### Rendering & Input
*   `ProcessInput(Universe&)`: Updates entities with `InputComponent`.
*   `DrawScene(Universe&)`: Performs the instanced draw calls for all meshes.
*   `IsRunning()`: Checks window close flag.
*   `GetFPS()`: Live Frames Per Second.
*   `GetDeltaTime()`: Time elapsed since last frame (capped for stability).

### Components (`Spade/Core/Components.hpp`)

*   **MeshComponent**: Holds the **vectors** of instances (`instanceTransforms`, `instanceMotions`, `instanceMaterials`).
*   **BoundingComponent**: Defines physical properties (`size`, `friction`, `bounciness`) shared by all instances of the entity.
*   **CameraComponent**: Defines FOV and planes.
*   **InputComponent**: Defines keybindings and movement speed.

### Data Structures (`Spade/Core/Primitives.hpp`)

Data structures are manually padded to match GLSL `std430` alignment.

-   `Transform`: `vec3` pos, `quat` rot, `vec3` scale.
-   `Motion`: `vec3` vel, `float` mass, `vec3` accel.
-   `Material`: `vec4` color, `float` metallic/roughness/emission.
