#include <iostream>
#include <vector>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <string_view>

// Add GLM extensions
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/constants.hpp>
#include <glm/gtx/quaternion.hpp>

#include <Spade/Spade.hpp>

using namespace Spade;

Engine engine;
Universe universe;

// ---------------------------------------------------------------------------
// Task 14b amendment A3: the v1 renders stay runnable as visual regression
// references while v2 (spade/engine/) progresses. This file used to be ONE
// hardcoded scene (50,000-sphere SPH fluid + grid collision); it is now three,
// selected by argv:
//
//   fluid    (no-arg default) EXACTLY today's prior behavior -- every Engine
//            call below in the `fluid` branch is the SAME call, with the SAME
//            literal arguments, in the SAME order, as before this file was
//            parameterized. Nothing about that path may drift.
//   spheres  the same large-count spawn, grid collision only (no
//            FluidComponent, no EnableSPHFluid) -- so the collision render is
//            regression-visible independent of the SPH solver. A modest
//            bounciness on the bound (vs. fluid's 0.0) makes the settling
//            pile visibly jostle instead of clumping dead-still.
//   cubes    the same large-count spawn and grid collision, but a cube mesh
//            (GenerateCube, Primitives.hpp) instead of a sphere -- v1 has no
//            true box collision proxy (see the isSphere note below), so this
//            exercises the render path with a different mesh over the SAME
//            sphere-proxy physics as `spheres`.
//
// v1 is frozen (global-constraints.md): every symbol used below already
// existed in Spade's public headers before this task.
// ---------------------------------------------------------------------------

namespace {

enum class DemoScene { Fluid, Spheres, Cubes };

void PrintUsage(const char* programName) {
  std::cerr << "usage: " << programName << " [fluid|spheres|cubes]\n"
            << "available scenes:\n"
            << "  fluid    (default) 50,000-sphere SPH fluid + grid collision\n"
            << "  spheres  50,000-sphere grid collision only, no SPH fluid\n"
            << "  cubes    50,000 cube-mesh instances, grid collision\n";
}

}  // namespace

int main(int argc, char** argv) {
  const char* programName = (argc > 0) ? argv[0] : "Sandbox";

  DemoScene scene = DemoScene::Fluid;
  if (argc == 2) {
    const std::string_view arg(argv[1]);
    if (arg == "fluid") {
      scene = DemoScene::Fluid;
    } else if (arg == "spheres") {
      scene = DemoScene::Spheres;
    } else if (arg == "cubes") {
      scene = DemoScene::Cubes;
    } else {
      std::cerr << "error: unknown scene '" << arg << "'\n";
      PrintUsage(programName);
      return 2;
    }
  } else if (argc > 2) {
    std::cerr << "error: expected at most one argument (the scene name)\n";
    PrintUsage(programName);
    return 2;
  }

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
  // fluid keeps bounciness 0.0 (today's exact value); spheres/cubes get a
  // modest bounciness so the settling pile visibly jostles instead of
  // clumping dead-still once EnableSPHFluid is no longer there to keep it
  // moving. NOTE: v1's grid-collision compute shader
  // ([SYSTEM]GridCollision.comp) resolves every body as a SPHERE regardless
  // of this `isSphere` flag -- it is carried in the Bound struct for
  // std430 layout parity but never branched on -- so `cubes` below renders a
  // cube mesh over the same sphere-proxy physics as `spheres`, which is the
  // "sphere-proxy bounds are fine if that is what v1 does" case the brief
  // anticipates.
  planets.GetComponent<BoundingComponent>()->bound.bounciness = (scene == DemoScene::Fluid) ? 0.0f : 0.3f;
  planets.GetComponent<BoundingComponent>()->bound.friction = 0.0;
  planets.GetComponent<BoundingComponent>()->bound.active = true;

  if (scene == DemoScene::Fluid) {
    planets.AddComponent<FluidComponent>();
    planets.GetComponent<FluidComponent>()->fluidMaterial.restDensity = 1.0;
    planets.GetComponent<FluidComponent>()->fluidMaterial.viscosity = 0.5;
    planets.GetComponent<FluidComponent>()->fluidMaterial.stiffness = 500.0;
    planets.GetComponent<FluidComponent>()->fluidMaterial.active = true;
  }

  // Largest round instance count that stays interactive on this box for all
  // three scenes (see task-14b-report.md for the measured FPS this was
  // chosen from). fluid's count is unchanged from before this task.
  constexpr int kFluidCount = 50000;
  constexpr int kSpheresCount = 50000;
  constexpr int kCubesCount = 50000;

  planets.AddComponent<MeshComponent>();
  int instanceCount = kFluidCount;
  if (scene == DemoScene::Cubes) {
    // Edge length 0.2 (half-extent 0.1) matches the sphere-proxy's radius
    // (bound.size 0.2 == diameter 0.2, radius 0.1) so the rendered cube
    // roughly fills its own collision volume, same as the sphere mesh does
    // for `fluid`/`spheres` below.
    planets.GetComponent<MeshComponent>()->mesh = GenerateCube(0.2f);
    instanceCount = kCubesCount;
  } else {
    planets.GetComponent<MeshComponent>()->mesh = GenerateSphere(0.1, 16, 16);
    instanceCount = (scene == DemoScene::Fluid) ? kFluidCount : kSpheresCount;
  }
  planets.GetComponent<MeshComponent>()->SpawnInstancesInCube(10.0, {3.0, 1.0, -3.0}, instanceCount);
  planets.GetComponent<MeshComponent>()->SetMass(0.01);
  planets.GetComponent<MeshComponent>()->RandomizeVelocity();
  planets.GetComponent<MeshComponent>()->RandomizeColor();


  unsigned int substeps = 10;
  float bounds = 10.0;

  camera.GetComponent<TransformComponent>()->transform.position = {0.0, -(bounds * 0.5), bounds};

  // Setup Window
  const char* windowTitle = "Spade";
  if (scene == DemoScene::Spheres) windowTitle = "Spade - spheres";
  if (scene == DemoScene::Cubes) windowTitle = "Spade - cubes";
  engine.SetupEngineWindow(1920, 1080, windowTitle);

  engine.LoadInstanceBuffers(universe);
  engine.LoadCameraBuffers(universe);
  engine.LoadCollisionBuffers(universe);
  if (scene == DemoScene::Fluid) {
    // LoadFluidBuffers walks every MeshComponent-bearing entity and reads its
    // FluidComponent unconditionally (Engine.cpp) -- an entity with no
    // FluidComponent makes that a null dereference, not a graceful skip. Only
    // called when `planets` actually has one (the `fluid` branch above).
    engine.LoadFluidBuffers(universe);
  }
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

        if (scene == DemoScene::Fluid) {
          engine.EnableSPHFluid(bounds, 0.25);
        }
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
