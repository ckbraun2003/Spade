# Spade Engine

Spade is a high-performance C++ simulation engine built on an Entity-Component-System (ECS) architecture. It is designed for building complex simulations with efficient rendering and physics.

## System Overview

The engine moves away from traditional Object-Oriented Programming (OOP) inheritance hierarchies in favor of a Data-Oriented Design (DOD). This is achieved through an **ECS (Entity Component System)** architecture:

*   **Universe**: The container for all data. It manages entities and component pools.
*   **Entity**: A simple ID that represents an object in the world. It has no data itself, only an ID.
*   **Component**: Pure data structures (structs) that hold state (e.g., Position, Velocity, Mesh).
*   **Systems**: Logic that operates on entities with specific components (e.g., `UpdateMotion` operates on entities with `Transform` and `Motion`).

## Getting Started

### Prerequisites

*   C++17 or later
*   CMake
*   OpenGL 4.6 support
*   Dependencies (Usually managed via vendor/submodules):
    *   GLFW
    *   GLAD
    *   GLM

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

Here is a basic example of how to set up a simulation with a camera and a moving cube:

```cpp
#include <Spade/Spade.hpp>

using namespace Spade;

Engine engine;
Universe universe;

int main() {
    // 1. Create a Camera Entity
    EntityID camID = universe.CreateEntityID();
    Entity camera(camID, &universe);
    
    // Add components to camera
    camera.AddComponent<TransformComponent>()->transform.position = {0.0, 1.0, 5.0};
    camera.AddComponent<CameraComponent>();
    
    // basic WASD input
    auto* input = camera.AddComponent<InputComponent>();
    input->bindings[GLFW_KEY_W] = MoveForward;
    input->bindings[GLFW_KEY_S] = MoveBackward;

    // 2. Create a Cube Entity
    EntityID cubeID = universe.CreateEntityID();
    Entity cube(cubeID, &universe);

    // Position and Scale
    cube.AddComponent<TransformComponent>();
    
    // Physics properties
    auto* motion = cube.AddComponent<MotionComponent>();
    motion->motion.velocity = {0.0f, 0.0f, 0.0f};
    motion->motion.mass = 1.0;

    // Visual Mesh
    auto* mesh = cube.AddComponent<MeshComponent>();
    mesh->mesh = GenerateCube(1.0);

    // CRITICAL: You must add at least one instance for the mesh to render
    Transform instanceTransform;
    instanceTransform.position = {0.0, 0.0, 0.0}; // Offset from entity if needed
    instanceTransform.rotation = {1.0, 0.0, 0.0, 0.0};
    instanceTransform.scale = {1.0, 1.0, 1.0};
    
    Motion instanceMotion; // Needed if using physics on instances
    instanceMotion.velocity = {0.0f, 0.0f, 0.0f};

    mesh->instanceTransforms.push_back(instanceTransform);
    mesh->instanceMotions.push_back(instanceMotion);

    // 3. Setup Window
    engine.SetupEngineWindow(1920, 1080, "My Simulation");

    // 4. Upload initial data to GPU
    engine.LoadMeshBuffers(universe);
    engine.LoadCameraBuffer(universe);

    // 5. Main Loop
    while (engine.IsRunning()) {
        engine.ProcessInput(universe); // Handle keyboard/mouse
        engine.UpdateMotion(universe); // Physics step
        engine.DrawScene(universe);    // Render
    }

    return 0;
}
```

## API Reference

### Engine Class
The `Engine` class manages the core application loop, windowing, and systems execution.

*   `SetupEngineWindow(width, height, title)`: Initializes the window and OpenGL context.
*   `IsRunning()`: Returns true if the window is open and the engine is running.
*   `ProcessInput(Universe&)`: Handles user input and updates entities with `InputComponent`.
*   `UpdateMotion(Universe&)`: Applies physics (velocity, acceleration) to entities with `TransformComponent` and `MotionComponent`.
*   `DrawScene(Universe&)`: Renders all active entities with `MeshComponent` using the active `CameraComponent`.
*   `LoadMeshBuffers(Universe&)`: Uploads static mesh data to the GPU (call before loop).
*   `LoadCameraBuffer(Universe&)`: Uploads initial camera data (call before loop).
*   `GetFPS()`, `GetMemory()`: Returns current performance stats.

### Universe & Entity
*   `Universe`: Owning container. Call `CreateEntityID()` to get a new logical object.
*   `Entity`: Wrapper around an ID and Universe pointer for easier API access.
    *   `AddComponent<T>()`: Adds component `T` to the entity. Returns pointer to the new component.
    *   `GetComponent<T>()`: Returns pointer to existing component or `nullptr`.
    *   `HasComponent<T>()`: Returns boolean.

### Components
Components are pure data structs found in `Spade/Core/Components.hpp`.

| Component | Description | key Members |
| :--- | :--- | :--- |
| **TransformComponent** | World position/rotation | `transform.position`, `transform.rotation`, `transform.scale` |
| **MotionComponent** | Physics properties | `motion.velocity`, `motion.acceleration`, `motion.mass` |
| **MeshComponent** | Visual geometry | `mesh` (Vertex data), instances support |
| **CameraComponent** | Viewing parameters | `fov`, `nearPlane`, `farPlane` |
| **InputComponent** | Input mapping | `bindings` (Map Key -> Action), `speed` |
| **MaterialComponent** | Rendering material | `material.color`, `material.roughness`, `material.metallic` |
| **BoundingComponent** | Collision bounds | `sphere`, `boundingBox` |

### Primitives
Helper functions in `Spade/Core/Primitives.hpp` to generate standard meshes.
*   `GenerateCube(size)`
*   `GenerateSphere(radius, sectors, stacks)`
*   `GenerateQuad(size)`
