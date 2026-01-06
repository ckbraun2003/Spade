#include <iostream>
#include <vector>
#include <chrono>
#include <cmath>

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
  camera.GetComponent<TransformComponent>()->transform.position = {0.0, 1.0, 5.0};

  camera.AddComponent<CameraComponent>();

  // Create Cube
  EntityID cubeID = universe.CreateEntityID();
  Entity cube = Entity(cubeID, &universe);

  cube.AddComponent<TransformComponent>();
  cube.GetComponent<TransformComponent>()->transform.position = {0.0f, 0.0f, 0.0f};
  cube.GetComponent<TransformComponent>()->transform.rotation = {1.0, 0.0, 0.0, 0.0};
  cube.GetComponent<TransformComponent>()->transform.scale = {1.0, 1.0, 1.0};

  cube.AddComponent<MotionComponent>();
  cube.GetComponent<MotionComponent>()->motion.velocity = {0.0f, 0.0f, 0.0f};
  cube.GetComponent<MotionComponent>()->motion.acceleration = {0.0f, 0.0f, 0.0f};
  cube.GetComponent<MotionComponent>()->motion.mass = 1.0;

  cube.AddComponent<MeshComponent>();
  cube.GetComponent<MeshComponent>()->mesh = GenerateCube(1.0);

  Transform transform;
  transform.position = {0.0, 0.0, 0.0};
  transform.rotation = {1.0, 0.0, 0.0, 0.0};
  transform.scale = {1.0, 1.0, 1.0};

  cube.GetComponent<MeshComponent>()->instanceTransforms.push_back(transform);

  // Setup Window
  engine.SetupEngineWindow(1920, 1080, "Spade");

  engine.LoadMeshBuffers(universe);
  engine.LoadMeshBuffers(universe);
  engine.LoadCameraBuffer(universe);

  // Begin Engine Loop
  while (engine.IsRunning()) {
    // FPS / MEMORY counter
    std::cout << "FPS: " << engine.GetFPS() << " | Mem: " << engine.GetMemory() << " MB" << std::endl;

    // Draw meshes
    engine.DrawScene(universe);

  }

  return 0;

};
