# Spade Engine

**Spade** is a high-performance C++ Simulation and Rendering Engine designed for "Interstellar" quality visualizations and real-time path tracing.

## Features

*   **Hybrid Rendering**: Supports both traditional Rasterization (OpenGL) and Wavefront Path Tracing (Compute Shaders).
*   **Entity Component System (ECS)**: Flexible architecture for managing game objects using `Universe`, `Entity`, and `Components`.
*   **Real-Time Path Tracing**:
    *   Full Global Illumination (GI) with multi-bounce lighting.
    *   PBR Materials (Roughness, Metallic, Emission).
    *   Temporal Accumulation for noise-free images.
    *   Dynamic Resolution Scaling.
*   **Resource Management**: Centralized `ResourceManager` for Meshes, Shaders, and Materials.
*   **Input System**: Easy access to Keyboard and Mouse state.

---

## 1. Getting Started

### Initialization
```cpp
#include <Spade/Core/Engine.hpp>

int main() {
    Spade::Engine engine;
    
    // 1. Setup Window
    engine.SetupEngineWindow(1280, 720, "My Sim");
    
    // 2. Initialize Core Systems (OpenGL, etc.)
    engine.Initialize();

    // 3. Main Loop
    while (engine.IsRunning()) {
        engine.Update(0.016f); // Update Logic
        
        // Choose your Renderer:
        // engine.RenderFrame(); // Rasterizer
        // OR
        // engine.GetRendering().RenderPathTracing(engine.GetUniverse(), cameraEntity);
    }
    return 0;
}
```

---

## 2. World Building (ECS)

Spade uses a data-driven ECS. You don't create "Objects", you create **Entities** and attach **Components**.

### Creating an Entity
```cpp
using namespace Spade;
Entity myObject = engine.CreateEmptyEntity("MyObject");
```

### Transform
Every entity usually needs a transform (position, rotation, scale).
```cpp
auto* t = myObject.AddComponent<Transform>();
t->position = {0.0f, 5.0f, 0.0f};
t->scale = {2.0f, 2.0f, 2.0f};
```

### Adding Geometry (Mesh)
To render an object, it needs a `MeshComponent`.
```cpp
auto* mesh = myObject.AddComponent<MeshComponent>();
mesh->meshID = ResourceManager::Get().GetMeshID("Sphere"); 
// Note: "Sphere", "Cube", "Plane" are built-in primitives.
```

### Materials
Spade supports two material types depending on the renderer.

**For Ray Tracing:**
```cpp
auto* mat = myObject.AddComponent<RayTracingMaterial>();
mat->color = {1.0f, 0.5f, 0.0f, 1.0f}; // Orange
mat->emission = 0.0f; // Set > 0 for Light Sources
mat->roughness = 0.2f; // Shiny
mat->metallic = 1.0f; // Metal
```

**For Rasterization:**
```cpp
auto* mat = myObject.AddComponent<MaterialComponent>();
mat->colorOverride = {1.0f, 0.0f, 0.0f, 1.0f};
mat->emission = 0.0f;
```

---

## 3. The Camera

You must create a Camera entity to view the world.
```cpp
Entity camera = engine.CreateCameraEntity("MainCamera");
auto* cam = camera.GetComponent<Camera>();
cam->fov = 60.0f;

// Move it back
camera.GetComponent<Transform>()->position = {0.0f, 0.0f, 10.0f};

// Optional: Add Input Control (WASD + Mouse Look)
camera.AddComponent<CameraInputComponent>();
```

---

## 4. Ray Tracing System

The Path Tracer is the core visual feature of Spade.

### Basic Rendering
Call `RenderPathTracing` in your loop instead of `RenderFrame`.
```cpp
engine.GetRendering().RenderPathTracing(engine.GetUniverse(), camera);
```

### Configuration
You can configure the quality and resolution dynamically:

**1. Bounce Depth** (Light Bounces)
Controls how many times light reflects. Higher = more realistic but slower.
```cpp
engine.GetRendering().SetBounceDepth(5); // Default is 5
```

**2. Resolution** (Dynamic Scaling)
Set the internal rendering resolution. It will automatically upscale to the window size.
```cpp
// Render at 1920x1080 regardless of window size
engine.GetRendering().SetRenderResolution(1920, 1080); 
```

**3. Temporal Accumulation** (Noise Reduction)
The engine **automatically accumulates** samples when the camera is static.
- **Moving**: Image resets / becomes noisy (Real-time feedback).
- **Static**: Image clears noise and converges to ground truth.

---

## 5. Input API

Access input via the Engine instance.
```cpp
if (engine.IsKeyPressed(GLFW_KEY_SPACE)) {
    // Jump
}

glm::vec2 mouse = engine.GetMousePosition();
```

---

## 6. Resource Manager

Use `Spade::ResourceManager` to load or create assets.

### Loading/Creating Meshes
The engine provides primitives ("Cube", "Sphere", "Plane"). To create custom meshes:
```cpp
ResourceID myMeshID = ResourceManager::Get().CreateMesh("CustomShape");
ResourceManager::Get().GetResource<MeshResource>(myMeshID)->Upload(vertices, indices);
```

---

## 7. Example: Full Ray Tracing Scene

```cpp
#include <Spade/Core/Engine.hpp>
#include <Spade/Core/Components.hpp>

int main() {
    Spade::Engine engine;
    engine.SetupEngineWindow(1280, 720, "Spade RT");
    engine.Initialize();

    auto& rm = Spade::ResourceManager::Get();

    // 1. Create Sun (Light Source)
    auto sun = engine.CreateEmptyEntity("Sun");
    sun.AddComponent<Transform>()->position = {0, 10, 0};
    sun.AddComponent<Transform>()->scale = {3, 3, 3};
    sun.AddComponent<MeshComponent>()->meshID = rm.GetMeshID("Sphere");
    auto* sunMat = sun.AddComponent<RayTracingMaterial>();
    sunMat->color = {1, 1, 0.8, 1};
    sunMat->emission = 10.0f; // BRIGHT!

    // 2. Create Floor
    auto floor = engine.CreateEmptyEntity("Floor");
    floor.AddComponent<Transform>()->position = {0, -2, 0};
    floor.AddComponent<Transform>()->scale = {20, 1, 20};
    floor.AddComponent<MeshComponent>()->meshID = rm.GetMeshID("Cube");
    auto* floorMat = floor.AddComponent<RayTracingMaterial>();
    floorMat->color = {0.5, 0.5, 0.5, 1};
    floorMat->roughness = 0.1f; // Reflection

    // 3. Create Camera
    auto cam = engine.CreateCameraEntity("Camera");
    cam.GetComponent<Transform>()->position = {0, 2, 10};
    cam.AddComponent<CameraInputComponent>();

    // RT Settings
    engine.GetRendering().SetRenderResolution(1280, 720);
    engine.GetRendering().SetBounceDepth(4);

    while(engine.IsRunning()) {
        engine.Update(0.016f);
        engine.GetRendering().RenderPathTracing(engine.GetUniverse(), cam);
    }
    return 0;
}
```
