# Spade Engine

**Spade** is a high-performance, Data-Oriented game engine written in C++20 and OpenGL 4.3+. It utilizes a strict Entity-Component-System (ECS) architecture and Shader Storage Buffer Objects (SSBOs) for efficient, batched rendering.

## Key Features
- **Strict ECS Architecture**: Data is separated from logic. Entities are IDs, Components are POD structs, Systems are functions.
- **SSBO Batched Rendering**: Objects are drawn in large batches, minimizing draw calls and state changes.
- **Data-Oriented Design**: Cache-friendly data layout for maximum performance.
- **Built-in Systems**: Camera, Interaction (Drag & Drop), and Physics integration.

---

## 1. Getting Started

### Prerequisites
- **CMake** 3.15+
- **C++20** Compiler (MSVC, GCC, Clang)
- **OpenGL 4.3+** compatible GPU

### Installation
1. Clone the repository:
   ```bash
   git clone https://github.com/your-repo/Spade.git
   ```
2. Build with CMake:
   ```bash
   mkdir build
   cd build
   cmake ..
   cmake --build . --config Release
   ```

### Running Examples
- **Sandbox**: interactable scene with draggable objects.
  - Controls: `WASD` to move, `Right-Click + Mouse` to look, `Left-Click` to drag objects.
- **Benchmark**: Performance test generating 1024 animated cubes.

---

## 2. Architecture

### The Universe
The `Universe` is the ECS container. It manages:
- **Entities**: Simple unique IDs (Handles).
- **Component Pools**: Contiguous arrays of component data.

### Entities & Components
To add a component, you interact with the Entity handle:
```cpp
Entity entity = engine.CreateEmptyEntity("MyObject");
entity.AddComponent<MeshComponent>();
```

### Systems
Systems are functions that iterate over Component Pools. They are located in the `Engine` class or external functions.
- **`UpdateCameraSystem(dt)`**: Handles Camera Input & Movement.
- **`UpdateInteractionSystem(dt)`**: Handles Mouse Picking & Object Dragging.
- **`UpdateScripts(dt)`**: Executes logic for `NativeScriptComponent`.

---

## 3. Components Reference

### `Transform`
Defines position, rotation, and scale.
```cpp
struct Transform {
    glm::vec3 position;
    glm::quat rotation; // Quaternion
    glm::vec3 scale;
    
    glm::mat4 GetMatrix() const; // Returns Model Matrix
    glm::vec3 GetForward() const; // Returns Forward vector
};
```

### `MeshComponent`
References a Mesh Resource to render.
```cpp
struct MeshComponent {
    ResourceID meshID;
};
```

### `MaterialComponent`
References a Material Resource and optional overrides.
```cpp
struct MaterialComponent {
    ResourceID materialID;
    glm::vec4 colorOverride;
};
```

### `Camera`
Defines projection parameters.
```cpp
struct Camera {
    float fov;
    float nearPlane, farPlane;
};
```

### `CameraInputComponent`
Enables `UpdateCameraSystem` to control this entity.
```cpp
struct CameraInputComponent {
    float speed;
    float sensitivity;
    bool isActive;
};
```

### `NativeScriptComponent`
Allows attaching ad-hoc logic via std::function.
```cpp
struct NativeScriptComponent {
    std::function<void(Entity, float)> OnUpdate;
};
```

---

## 4. API Reference

### Engine API
```cpp
void Initialize();
void IsRunning();
void Update(); // Poll Events
void RenderFrame(); // Swap Buffers

// Systems
void UpdateCameraSystem(float deltaTime);
void UpdateInteractionSystem(float deltaTime);

// Input
bool IsKeyPressed(int key);
bool IsMouseButtonPressed(int button);
glm::vec2 GetMousePosition();
void SetMouseCursorMode(bool trapped);
```

### Resource Manager
```cpp
ResourceManager& rm = ResourceManager::Get();
ResourceID meshID = rm.GetMeshID("Sphere");
ResourceID matID = rm.GetMaterialID("DefaultMaterial");
```

---

## 5. Usage Example
```cpp
#include <Spade/Spade.hpp>

using namespace Spade;

int main() {
    Engine engine;
    engine.SetupEngineWindow(1280, 720, "My Game");
    engine.Initialize();

    // Create Camera
    Entity cam = engine.CreateCameraEntity("MainCamera");
    cam.GetComponent<Transform>()->position = {0, 2, 5};

    // Create Object
    Entity sphere = engine.CreateEmptyEntity("Sphere");
    sphere.AddComponent<Transform>()->position = {0, 0, 0};
    sphere.AddComponent<MeshComponent>()->meshID = ResourceManager::Get().GetMeshID("Sphere");
    sphere.AddComponent<MaterialComponent>()->materialID = ResourceManager::Get().GetMaterialID("DefaultMaterial");

    // Game Loop
    float lastTime = 0.0f;
    while (engine.IsRunning()) {
        float time = engine.GetTime();
        float dt = time - lastTime;
        lastTime = time;

        // Systems
        engine.UpdateCameraSystem(dt);
        engine.UpdateInteractionSystem(dt);
        engine.GetPhysics().Update(engine.GetUniverse(), dt);

        // Render
        engine.GetRendering().Render(engine.GetUniverse(), cam);
        engine.RenderFrame();
        engine.Update();
    }
}
```
