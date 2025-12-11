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

int main() {
    try {
        Engine engine;
        engine.SetupEngineWindow(1280, 720, "Spade Engine - SSBO Renderer");
        std::cout << "Initializing Engine..." << std::endl;
        engine.Initialize();
        std::cout << "Engine Initialized." << std::endl;

        ResourceManager& rm = ResourceManager::Get();
        Universe& universe = engine.GetUniverse();

        // --- Create Camera ---
        Entity camera = engine.CreateCameraEntity("MainCamera");
        camera.AddComponent<Camera>(); 
        // Disable default input
        if(auto* input = camera.GetComponent<CameraInputComponent>()) {
            input->isActive = false;
        }
        camera.GetComponent<Transform>()->position = {0.0f, 10.0f, 15.0f};

        // --- Create Entities ---
        std::cout << "Creating Entities..." << std::endl;

        std::vector<Entity> animatedCubes;

        // 1. Cube Grid
        int gridSize = 50;
        float spacing = 2.0f;
        float offset = (gridSize * spacing) / 2.0f;

        for (int x = 0; x < gridSize; ++x) {
            for (int z = 0; z < gridSize; ++z) {
                Entity ent = engine.CreateEmptyEntity("Cube_Grid");
                ent.GetComponent<Transform>()->position = {
                    (x * spacing) - offset, 
                    0.0f, 
                    (z * spacing) - offset
                };
                
                auto* mesh = ent.AddComponent<MeshComponent>();
                mesh->meshID = rm.GetMeshID("Cube");
                
                auto* mat = ent.AddComponent<MaterialComponent>();
                mat->materialID = rm.GetMaterialID("DefaultMaterial");
                // Gradient colors
                float r = (float)x / gridSize;
                float g = (float)z / gridSize;
                mat->colorOverride = {r, g, 1.0f - r, 1.0f}; 
                
                animatedCubes.push_back(ent);
            }
        }
        
        // 2. Floor Quad
        {
            Entity ent = engine.CreateEmptyEntity("Floor");
            ent.GetComponent<Transform>()->position = {0.0f, -5.0f, 0.0f};
            ent.GetComponent<Transform>()->scale = {50.0f, 50.0f, 1.0f};
            ent.GetComponent<Transform>()->rotation = glm::quat(glm::vec3(-glm::half_pi<float>(), 0.0f, 0.0f)); 

            auto* mesh = ent.AddComponent<MeshComponent>();
            mesh->meshID = rm.GetMeshID("Quad"); 
            
            std::cout << "Created Floor." << std::endl;
        }

        std::cout << "Entities properties set." << std::endl;

        // --- Main Loop ---
        float lastTime = 0.0f;
        int frames = 0;
        float fpsTimer = 0.0f;
        
        std::cout << "Entering Main Loop..." << std::endl;
        while (engine.IsRunning()) {
            float time = engine.GetTime();
            float deltaTime = time - lastTime;
            lastTime = time;

            // Update FPS
            frames++;
            fpsTimer += deltaTime;
            if (fpsTimer >= 1.0f) {
                std::cout << "FPS: " << frames << " (" << (1000.0f / frames) << " ms)" << std::endl;
                frames = 0;
                fpsTimer = 0.0f;
            }

            // 1. Orbital Camera Logic
            float radius = 50.0f;
            float camSpeed = 0.3f;
            float camX = sin(time * camSpeed) * radius;
            float camZ = cos(time * camSpeed) * radius;
            Transform* camTrans = camera.GetComponent<Transform>();
            camTrans->position = glm::vec3(camX, 15.0f, camZ);
            
            // Look at center
            glm::vec3 target = glm::vec3(0.0f, 0.0f, 0.0f);
            glm::mat4 view = glm::lookAt(camTrans->position, target, glm::vec3(0.0f, 1.0f, 0.0f));
            camTrans->rotation = glm::quat_cast(glm::inverse(view));

            // 2. Wave Physics (Animation)
            for (auto& ent : animatedCubes) {
                Transform* t = ent.GetComponent<Transform>();
                // Wave based on position
                float waveHeight = 2.0f;
                float waveSpeed = 2.0f;
                float frequency = 0.5f;
                float y = sin(time * waveSpeed + t->position.x * frequency + t->position.z * frequency) * waveHeight;
                t->position.y = y;
            }

            // --- Engine Updates ---
            engine.UpdateScripts(deltaTime);
            engine.UpdateInteractionSystem(deltaTime);
            // engine.UpdateCameraSystem(deltaTime); 

            engine.GetPhysics().Update(universe, deltaTime);
            
            // Render
            glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

            engine.GetRendering().Render(universe, camera);

            engine.RenderFrame();
            engine.Update();
        }

    } catch (const std::exception& e) {
        std::cerr << "EXCEPTION: " << e.what() << std::endl;
        // Keep console open
        int x; std::cin >> x; 
        return 1;
    }
    return 0;
}
