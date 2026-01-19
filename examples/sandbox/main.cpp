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
  camera.GetComponent<TransformComponent>()->transform.position = {0.0, -1.0, 5.0};

  camera.AddComponent<CameraComponent>();
  camera.GetComponent<CameraComponent>()->fov = 90.0;
  camera.GetComponent<CameraComponent>()->nearPlane = 0.01;
  camera.GetComponent<CameraComponent>()->farPlane = 100;

  camera.AddComponent<InputComponent>();
  camera.GetComponent<InputComponent>()->speed = 5.0f;
  camera.GetComponent<InputComponent>()->bindings[GLFW_KEY_SPACE] = MoveUp;
  camera.GetComponent<InputComponent>()->bindings[GLFW_KEY_LEFT_SHIFT] = MoveDown;
  camera.GetComponent<InputComponent>()->bindings[GLFW_KEY_W] = MoveForward;
  camera.GetComponent<InputComponent>()->bindings[GLFW_KEY_S] = MoveBackward;
  camera.GetComponent<InputComponent>()->bindings[GLFW_KEY_A] = MoveLeft;
  camera.GetComponent<InputComponent>()->bindings[GLFW_KEY_D] = MoveRight;

  // Create Cube
  EntityID meshID = universe.CreateEntityID();
  Entity mesh = Entity(meshID, &universe);

  // mesh.AddComponent<TransformComponent>();
  // mesh.GetComponent<TransformComponent>()->transform.position = {0.0f, 0.0f, 0.0f};
  // mesh.GetComponent<TransformComponent>()->transform.rotation = {1.0, 0.0, 0.0, 0.0};
  // mesh.GetComponent<TransformComponent>()->transform.scale = {1.0, 1.0, 1.0};
  //
  // mesh.AddComponent<MotionComponent>();
  // mesh.GetComponent<MotionComponent>()->motion.velocity = {0.0f, 0.0f, 0.0f};
  // mesh.GetComponent<MotionComponent>()->motion.acceleration = {0.0f, 0.0f, 0.0f};
  // mesh.GetComponent<MotionComponent>()->motion.mass = 1.0;

  mesh.AddComponent<BoundingComponent>();
  mesh.GetComponent<BoundingComponent>()->bound.size = 0.2;
  mesh.GetComponent<BoundingComponent>()->bound.isSphere = true;
  mesh.GetComponent<BoundingComponent>()->bound.bounciness = 0.5;
  mesh.GetComponent<BoundingComponent>()->bound.friction = 0.5;

  mesh.AddComponent<MeshComponent>();
  mesh.GetComponent<MeshComponent>()->mesh = GenerateSphere(0.1, 16, 16);
  //mesh.GetComponent<MeshComponent>()->mesh = GenerateCube(1.0);

  // EntityID meshID2 = universe.CreateEntityID();
  // Entity mesh2 = Entity(meshID2, &universe);
  //
  // mesh2.AddComponent<BoundingComponent>();
  // mesh2.GetComponent<BoundingComponent>()->bound.size = 1.0;
  // mesh2.GetComponent<BoundingComponent>()->bound.isSphere = true;
  // mesh2.GetComponent<BoundingComponent>()->bound.bounciness = 0.3;
  // mesh2.GetComponent<BoundingComponent>()->bound.friction = 0.7;
  //
  // mesh2.AddComponent<MeshComponent>();
  // mesh2.GetComponent<MeshComponent>()->mesh = GenerateSphere(0.5, 16, 16);
  //mesh2.GetComponent<MeshComponent>()->mesh = GenerateCube(1.0);

  // Spawn 1000 Spheres in a Grid (10x10x10)
  int gridSize = 10;
  float spacing = 0.2;
  float startOffset = -((gridSize - 1) * spacing) / 2.0f;

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
          (float)(rand() % 100 - 50) / 10.0f,
          (float)(rand() % 100 - 50) / 10.0f,
          (float)(rand() % 100 - 50) / 10.0f
        };
        //motion.velocity = {0.0, 0.0, 0.0};
        motion.mass = 1.0;
        motion.acceleration = {0.0f, 0.0f, 0.0f};

        Material material;
        material.color = {
          (float)rand() / (float)RAND_MAX,
          (float)rand() / (float)RAND_MAX,
          (float)rand() / (float)RAND_MAX, 1.0};

        mesh.GetComponent<MeshComponent>()->instanceTransforms.push_back(transform);
        mesh.GetComponent<MeshComponent>()->instanceMotions.push_back(motion);
        mesh.GetComponent<MeshComponent>()->instanceMaterials.push_back(material);
      }
    }
  }

  // gridSize = 2;
  // spacing = 1.0;
  // startOffset = -((gridSize - 1) * spacing) / 2.0f;
  //
  // for (int x = 0; x < gridSize; ++x) {
  //   for (int y = 0; y < gridSize; ++y) {
  //     for (int z = 0; z < gridSize; ++z) {
  //       Transform transform;
  //       transform.position = {
  //         startOffset + x * spacing,
  //         startOffset + y * spacing,
  //         startOffset + z * spacing
  //       };
  //       transform.rotation = {1.0, 0.0, 0.0, 0.0};
  //       transform.scale = {1.0, 1.0, 1.0};
  //
  //       Motion motion;
  //       // Random velocity to create collisions
  //       motion.velocity = {
  //         (float)(rand() % 100 - 50) / 10.0f,
  //         (float)(rand() % 100 - 50) / 10.0f,
  //         (float)(rand() % 100 - 50) / 10.0f
  //       };
  //       //motion.velocity = {0.0, 0.0, 0.0};
  //       motion.mass = 2.0;
  //       motion.acceleration = {0.0f, 0.0f, 0.0f};
  //
  //       Material material;
  //       material.color = {
  //         (float)rand() / (float)RAND_MAX,
  //         (float)rand() / (float)RAND_MAX,
  //         (float)rand() / (float)RAND_MAX, 1.0};
  //
  //       mesh2.GetComponent<MeshComponent>()->instanceTransforms.push_back(transform);
  //       mesh2.GetComponent<MeshComponent>()->instanceMotions.push_back(motion);
  //       mesh2.GetComponent<MeshComponent>()->instanceMaterials.push_back(material);
  //     }
  //   }
  // }

  // Setup Window
  engine.SetupEngineWindow(1920, 1080, "Spade");

  engine.LoadInstanceBuffers(universe);
  engine.LoadCameraBuffers(universe);
  engine.LoadCollisionBuffers(universe);

  unsigned int substeps = 1;
  float bounds = 10.0f;
  float cellSize = 0.25f;

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
      engine.EnableMotion(substepTime);
      //engine.EnableCollision(bounds, substepTime);
      engine.EnableGridCollision(bounds, cellSize, substepTime);
    }

    // Draw meshes
    engine.DrawScene(universe);

  }

  return 0;

};
