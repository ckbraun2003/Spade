#include <iostream>
#include <chrono>

#include <Spade/Spade.hpp>

using namespace Spade;

Engine engine;
Universe universe;

int main() {

  // Setup Window
  engine.SetupEngineWindow(1920, 1080, "Spade");

  // Create Camera
  EntityID cameraID = universe.CreateEntityID();
  Entity camera = Entity(cameraID, &universe);
  camera.AddComponent<TransformComponent>();
  camera.GetComponent<TransformComponent>()->transform.position = {0.0, 1.0, 5.0};

  camera.AddComponent<CameraComponent>();

  camera.AddComponent<InputComponent>();
  camera.GetComponent<InputComponent>()->bindings[GLFW_KEY_SPACE] = MoveUp;
  camera.GetComponent<InputComponent>()->bindings[GLFW_KEY_LEFT_SHIFT] = MoveDown;
  camera.GetComponent<InputComponent>()->bindings[GLFW_KEY_W] = MoveForward;
  camera.GetComponent<InputComponent>()->bindings[GLFW_KEY_S] = MoveBackward;
  camera.GetComponent<InputComponent>()->bindings[GLFW_KEY_A] = MoveLeft;
  camera.GetComponent<InputComponent>()->bindings[GLFW_KEY_D] = MoveRight;

  // Create Cube
  EntityID cubeID = universe.CreateEntityID();
  Entity cube = Entity(cubeID, &universe);
  cube.AddComponent<TransformComponent>();
  cube.GetComponent<TransformComponent>()->transform.position = {0.0, 0.0, 0.0};
  cube.GetComponent<TransformComponent>()->transform.rotation = {1.0, 0.0, 0.0, 0.0};
  cube.GetComponent<TransformComponent>()->transform.scale = {1.0, 1.0, 1.0};

  cube.AddComponent<MeshComponent>();
  cube.GetComponent<MeshComponent>()->mesh = GenerateCube(1.0);

  cube.AddComponent<MaterialComponent>();
  cube.GetComponent<MaterialComponent>()->material.color = {1.0, 0.0, 1.0, 0.0};


  // Begin Engine Loop
  while (engine.IsRunning()) {

    // Process Input
    engine.ProcessInput(universe);

    // Initialize Universe
    engine.InitializeUniverse(universe);

    // Draw meshes
    engine.DrawScene(universe);

  }

  return 0;

};
