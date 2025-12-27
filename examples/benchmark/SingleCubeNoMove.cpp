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
  camera.GetComponent<TransformComponent>()->transform.position = {0.0, 1.0f, 5.0f};

  camera.AddComponent<CameraComponent>();

  // Create Cube
  EntityID cubeID = universe.CreateEntityID();
  Entity cube = Entity(cubeID, &universe);
  cube.AddComponent<TransformComponent>();
  cube.GetComponent<TransformComponent>()->transform.position = {0.0, 0.0, 0.0};
  cube.GetComponent<TransformComponent>()->transform.rotation = {1.0, 0.0, 0.0, 0.0};
  cube.GetComponent<TransformComponent>()->transform.scale = {1.0, 1.0, 1.0};

  cube.AddComponent<MeshComponent>();
  cube.GetComponent<MeshComponent>()->mesh = GenerateCube(1.0);

  // Setup Window
  engine.SetupEngineWindow(1920, 1080, "Spade");
  engine.InitializeUniverse(universe);

  // Begin Engine Loop
  while (engine.IsRunning()) {
    // FPS / MEMORY counter
    std::cout << "FPS: " << engine.GetFPS() << " | Mem: " << engine.GetMemory() << " MB" << std::endl;


    engine.DrawScene(universe);
  }

  return 0;

};