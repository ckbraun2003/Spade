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
  camera.GetComponent<TransformComponent>()->transform.position = {0.0, 1.0, 10.0};

  camera.AddComponent<CameraComponent>();

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

  mesh.AddComponent<TransformComponent>();
  mesh.GetComponent<TransformComponent>()->transform.position = {0.0f, 0.0f, 0.0f};
  mesh.GetComponent<TransformComponent>()->transform.rotation = {1.0, 0.0, 0.0, 0.0};
  mesh.GetComponent<TransformComponent>()->transform.scale = {1.0, 1.0, 1.0};

  mesh.AddComponent<MotionComponent>();
  mesh.GetComponent<MotionComponent>()->motion.velocity = {0.0f, 0.0f, 0.0f};
  mesh.GetComponent<MotionComponent>()->motion.acceleration = {0.0f, 0.0f, 0.0f};
  mesh.GetComponent<MotionComponent>()->motion.mass = 1.0;

  mesh.AddComponent<BoundingComponent>();
  mesh.GetComponent<BoundingComponent>()->sphere.radius = 0.1;

  mesh.AddComponent<MeshComponent>();
  mesh.GetComponent<MeshComponent>()->mesh = GenerateSphere(0.1, 16, 16);

  // Spawn 100 Spheres in a Grid (10x10)
  int gridSize = 10;
  float spacing = 0.25f;
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
          (float)(rand() % 100 - 50) / 1000.0f,
          (float)(rand() % 100 - 50) / 1000.0f,
          (float)(rand() % 100 - 50) / 1000.0f
        };
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

  // Setup Window
  engine.SetupEngineWindow(1920, 1080, "Spade");
  engine.SetGlobalBounds(5.0f);

  engine.LoadInstanceBuffers(universe);
  engine.LoadCameraBuffer(universe);

  // Begin Engine Loop
  while (engine.IsRunning()) {
    // FPS / MEMORY counter
    std::cout << "FPS: " << engine.GetFPS() << " | Mem: " << engine.GetMemory() << " MB" << std::endl;


    // Process Input
    engine.ProcessInput(universe);

    // Update Motion
    //engine.EnableGravity(0.1f);
    engine.EnableMotion();

    // Draw meshes
    engine.DrawScene(universe);

  }

  return 0;

};
