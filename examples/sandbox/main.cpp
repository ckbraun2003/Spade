#include <iostream>
#include <vector>
#include <chrono>
#include <cmath>
#include <cstdlib>

// Add GLM extensions
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/constants.hpp>
#include <glm/gtx/quaternion.hpp>

#include <Spade/Spade.hpp>

using namespace Spade;

Engine engine;
Universe universe;

int main() {

  // Create Camera
  EntityID cameraID = universe.CreateEntityID();
  Entity camera = Entity(cameraID, &universe);
  camera.AddComponent<TransformComponent>();

  camera.AddComponent<CameraComponent>();
  camera.GetComponent<CameraComponent>()->fov = 90.0;
  camera.GetComponent<CameraComponent>()->nearPlane = 0.01;
  camera.GetComponent<CameraComponent>()->farPlane = 1000;

  camera.AddComponent<InputComponent>();
  camera.GetComponent<InputComponent>()->speed = 10.0f;
  camera.GetComponent<InputComponent>()->bindings[GLFW_KEY_SPACE] = MoveUp;
  camera.GetComponent<InputComponent>()->bindings[GLFW_KEY_LEFT_SHIFT] = MoveDown;
  camera.GetComponent<InputComponent>()->bindings[GLFW_KEY_W] = MoveForward;
  camera.GetComponent<InputComponent>()->bindings[GLFW_KEY_S] = MoveBackward;
  camera.GetComponent<InputComponent>()->bindings[GLFW_KEY_A] = MoveLeft;
  camera.GetComponent<InputComponent>()->bindings[GLFW_KEY_D] = MoveRight;

  // Create Particles
  EntityID meshID = universe.CreateEntityID();
  Entity mesh = Entity(meshID, &universe);

  mesh.AddComponent<TransformComponent>();
  mesh.GetComponent<TransformComponent>()->transform.position = {0.0f, 0.0f, 0.0f};
  mesh.GetComponent<TransformComponent>()->transform.rotation = {1.0, 0.0, 0.0, 0.0};
  mesh.GetComponent<TransformComponent>()->transform.scale = {1.0, 1.0, 1.0};

  mesh.AddComponent<BoundingComponent>();
  mesh.GetComponent<BoundingComponent>()->bound.size = 0.2;
  mesh.GetComponent<BoundingComponent>()->bound.isSphere = true;
  mesh.GetComponent<BoundingComponent>()->bound.bounciness = 0.0;
  mesh.GetComponent<BoundingComponent>()->bound.friction = 0.0;
  mesh.GetComponent<BoundingComponent>()->bound.active = true;

  mesh.AddComponent<MeshComponent>();
  mesh.GetComponent<MeshComponent>()->mesh = GenerateSphere(0.1, 16, 16);
  //mesh.GetComponent<MeshComponent>()->mesh = GenerateCube(1.0);

  mesh.AddComponent<FluidComponent>();
  mesh.GetComponent<FluidComponent>()->fluidMaterial.restDensity = 1;
  mesh.GetComponent<FluidComponent>()->fluidMaterial.viscosity = 0.05;
  mesh.GetComponent<FluidComponent>()->fluidMaterial.stiffness = 500;
  mesh.GetComponent<FluidComponent>()->fluidMaterial.active = true;

  // // Create Sphere
  // EntityID sphereID = universe.CreateEntityID();
  // Entity sphere = Entity(sphereID, &universe);
  //
  // sphere.AddComponent<TransformComponent>();
  // sphere.GetComponent<TransformComponent>()->transform.position = {0.0f, 0.0f, 0.0f};
  // sphere.GetComponent<TransformComponent>()->transform.rotation = {1.0, 0.0, 0.0, 0.0};
  // sphere.GetComponent<TransformComponent>()->transform.scale = {1.0, 1.0, 1.0};
  //
  // sphere.AddComponent<BoundingComponent>();
  // sphere.GetComponent<BoundingComponent>()->bound.size = 1.0;
  // sphere.GetComponent<BoundingComponent>()->bound.isSphere = true;
  // sphere.GetComponent<BoundingComponent>()->bound.bounciness = 0.0;
  // sphere.GetComponent<BoundingComponent>()->bound.friction = 0.0;
  // sphere.GetComponent<BoundingComponent>()->bound.active = true;
  //
  // sphere.AddComponent<MeshComponent>();
  // sphere.GetComponent<MeshComponent>()->mesh = GenerateSphere(0.5, 16, 16);
  // //mesh.GetComponent<MeshComponent>()->mesh = GenerateCube(1.0);
  //
  // sphere.AddComponent<FluidComponent>();
  // sphere.GetComponent<FluidComponent>()->fluidMaterial.restDensity = 1;
  // sphere.GetComponent<FluidComponent>()->fluidMaterial.viscosity = 0.05;
  // sphere.GetComponent<FluidComponent>()->fluidMaterial.stiffness = 500;
  // sphere.GetComponent<FluidComponent>()->fluidMaterial.active = false;
  //
  // {
  //   Transform transform;
  //   transform.position = {0.0, 0.0, 0.0};
  //   transform.rotation = {1.0, 0.0, 0.0, 0.0};
  //   transform.scale = {1.0, 1.0, 1.0};
  //
  //   Motion motion;
  //   motion.velocity = {0.0, 0.0, 0.0};
  //   motion.mass = 1.0;
  //   motion.acceleration = {0.0f, 0.0f, 0.0f};
  //
  //   Material material;
  //   material.color = {0.0f, 0.2f, 0.7f, 1.0f};
  //
  //   sphere.GetComponent<MeshComponent>()->instanceTransforms.push_back(transform);
  //   sphere.GetComponent<MeshComponent>()->instanceMotions.push_back(motion);
  //   sphere.GetComponent<MeshComponent>()->instanceMaterials.push_back(material);
  // }

  // Spawn Spheres in a Grid
  int gridSize = 30;
  float spacing = 0.25;
  float startOffset = -((gridSize - 1) * spacing) * 0.5f;

  for (int x = 0; x < gridSize; ++x) {
    for (int y = 0; y < gridSize; ++y) {
      for (int z = 0; z < gridSize; ++z) {
        Transform transform;
        transform.position = {
          startOffset + x * spacing,
          startOffset + y * spacing,
          startOffset + z * spacing
        };
        transform.rotation = {1.0, 0.0, 0.0, 0.0};
        transform.scale = {1.0, 1.0, 1.0};

        Motion motion;
        // Random velocity to create collisions
        motion.velocity = {
          (float)(rand() % 100 - 50) / 100.0f,
          (float)(rand() % 100 - 50) / 100.0f,
          (float)(rand() % 100 - 50) / 100.0f
        };
        //motion.velocity = {0.0, 0.0, 0.0};
        motion.mass = 0.01;
        motion.acceleration = {0.0f, 0.0f, 0.0f};

        Material material;
        material.color = {
          (float)rand() / (float)RAND_MAX,
          (float)rand() / (float)RAND_MAX,
          (float)rand() / (float)RAND_MAX,
          1.0};
        //material.color = {0.0f, 0.2f, 0.7f, 1.0f};

        mesh.GetComponent<MeshComponent>()->instanceTransforms.push_back(transform);
        mesh.GetComponent<MeshComponent>()->instanceMotions.push_back(motion);
        mesh.GetComponent<MeshComponent>()->instanceMaterials.push_back(material);
      }
    }
  }

  unsigned int substeps = 10;
  float bounds = float(gridSize) * spacing;
  float cellSize = 0.25f;

  camera.GetComponent<TransformComponent>()->transform.position = {0.0, -(bounds * 0.5), bounds};

  // Setup Window
  engine.SetupEngineWindow(1920, 1080, "Spade");

  engine.LoadInstanceBuffers(universe);
  engine.LoadCameraBuffers(universe);
  engine.LoadCollisionBuffers(universe);
  engine.LoadFluidBuffers(universe);

  // Begin Engine Loop
  while (engine.IsRunning()) {
    // FPS / MEMORY counter
    std::cout << "FPS: " << engine.GetFPS() << " | Mem: " << engine.GetMemory() << " MB" << std::endl;

    // Process Input
    engine.ProcessInput(universe);

    float deltaTime = engine.GetDeltaTime();
    float substepTime = deltaTime / (float)substeps;

    // Update Motion
    for (int i = 0; i < substeps; ++i) {
      engine.EnableGravity(10.0f, substepTime);

      //engine.EnableCollision(bounds, substepTime);
      engine.EnableGridCollision(bounds, cellSize, substepTime);

      engine.EnableSPHFluid(bounds, cellSize, substepTime);

      engine.EnableMotion(substepTime);
    }

    // Draw meshes
    engine.DrawScene(universe, "assets/shaders/[FRAGMENT]Color.frag");

  }

  return 0;

};
