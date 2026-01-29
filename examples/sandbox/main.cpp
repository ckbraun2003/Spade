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
  EntityID planetID = universe.CreateEntityID();
  Entity planets = Entity(planetID, &universe);

  planets.AddComponent<TransformComponent>();
  planets.GetComponent<TransformComponent>()->transform.position = {0.0f, 0.0f, 0.0f};
  planets.GetComponent<TransformComponent>()->transform.rotation = {1.0, 0.0, 0.0, 0.0};
  planets.GetComponent<TransformComponent>()->transform.scale = {1.0, 1.0, 1.0};

  planets.AddComponent<BoundingComponent>();
  planets.GetComponent<BoundingComponent>()->bound.size = 0.2;
  planets.GetComponent<BoundingComponent>()->bound.isSphere = true;
  planets.GetComponent<BoundingComponent>()->bound.bounciness = 0.0;
  planets.GetComponent<BoundingComponent>()->bound.friction = 0.0;
  planets.GetComponent<BoundingComponent>()->bound.active = true;

  planets.AddComponent<FluidComponent>();
  planets.GetComponent<FluidComponent>()->fluidMaterial.restDensity = 1.0;
  planets.GetComponent<FluidComponent>()->fluidMaterial.viscosity = 0.5;
  planets.GetComponent<FluidComponent>()->fluidMaterial.stiffness = 500.0;
  planets.GetComponent<FluidComponent>()->fluidMaterial.active = true;

  planets.AddComponent<MeshComponent>();
  planets.GetComponent<MeshComponent>()->mesh = GenerateSphere(0.1, 16, 16);
  planets.GetComponent<MeshComponent>()->SpawnInstancesInCube(10.0, {3.0, 1.0, -3.0}, 50000);
  planets.GetComponent<MeshComponent>()->SetMass(0.01);
  planets.GetComponent<MeshComponent>()->RandomizeVelocity();
  planets.GetComponent<MeshComponent>()->RandomizeColor();


  unsigned int substeps = 10;
  float bounds = 10.0;

  camera.GetComponent<TransformComponent>()->transform.position = {0.0, -(bounds * 0.5), bounds};

  // Setup Window
  engine.SetupEngineWindow(1920, 1080, "Spade");

  engine.LoadInstanceBuffers(universe);
  engine.LoadCameraBuffers(universe);
  engine.LoadCollisionBuffers(universe);
  engine.LoadFluidBuffers(universe);
  engine.LoadGridBuffers();

  // Begin Engine Loop
  while (engine.IsRunning()) {
    // FPS / MEMORY counter
    std::cout << "FPS: " << engine.GetFPS() << " | Mem: " << engine.GetMemory() << " MB" << std::endl;

    // Process Input
    engine.ProcessInput(universe);

    if (engine.IsPlaying()) {
      float deltaTime = engine.GetDeltaTime();
      float substepTime = deltaTime / (float)substeps;

      // Update Motion
      for (int i = 0; i < substeps; ++i) {
        engine.EnableGravity(10.0);

        engine.EnableSPHFluid(bounds, 0.25);
        engine.EnableGridCollision(bounds, 0.25);

        engine.EnableMotion(substepTime);
      }
    }

    // Draw meshes
    engine.RenderColor();
    engine.DrawScene(universe);

  }

  return 0;

};
