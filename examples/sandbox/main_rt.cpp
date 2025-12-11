#include <iostream>
#include <vector>
#include <chrono>
#include <cmath>

#include <Spade/Spade.hpp>
#include <glm/gtc/constants.hpp>

using namespace Spade;

int main() {
    try {
// ...
// ... (Top of file remains)
        Engine engine;
        engine.SetupEngineWindow(1280, 720, "Spade Engine - SSBO Renderer");
        std::cout << "Initializing Engine..." << std::endl;
        engine.Initialize();
        std::cout << "Engine Initialized." << std::endl;

        ResourceManager& rm = ResourceManager::Get();
        Universe& universe = engine.GetUniverse();

        // --- Create Camera ---
        Entity camera = engine.CreateCameraEntity("MainCamera");
        camera.AddComponent<Camera>(); // Default FOV 45
        camera.GetComponent<Transform>()->position = {0.0f, 2.0f, 5.0f};

        // --- Create Entities ---
        // --- Create Entities ---
        std::cout << "Creating Entities (Ray Tracing Grid)..." << std::endl;

        // 1. Cube Grid
        int gridSize = 5;
        float spacing = 2.0f;
        float offset = (gridSize * spacing) / 2.0f;

        for (int x = 0; x < gridSize; ++x) {
            for (int z = 0; z < gridSize; ++z) {
                Entity ent = engine.CreateEmptyEntity("Sphere_Grid");
                ent.GetComponent<Transform>()->position = {
                    (x * spacing) - offset, 
                    0.0f, 
                    (z * spacing) - offset
                };
                
                auto* mesh = ent.AddComponent<MeshComponent>();
                mesh->meshID = rm.GetMeshID("Sphere");
                
                // RAY TRACING MATERIAL
                auto* mat = ent.AddComponent<RayTracingMaterial>();
                // Gradient colors
                float r = (float)x / gridSize;
                float g = (float)z / gridSize;
                mat->color = {r, g, 1.0f - r, 1.0f};
                mat->roughness = 0.0f;
                mat->metallic = 1.0f;
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
            
            // RAY TRACING MATERIAL
            auto* mat = ent.AddComponent<RayTracingMaterial>();
            mat->color = {0.2f, 0.2f, 0.2f, 1.0f};
            mat->roughness = 0.5f;
            
            std::cout << "Created Floor." << std::endl;
        }

        // 3. Sun (Light Source)
        {
            Entity ent = engine.CreateEmptyEntity("Sun");
            ent.GetComponent<Transform>()->position = {0.0f, 20.0f, 0.0f};
            ent.GetComponent<Transform>()->scale = {10.0f, 10.0f, 10.0f};
            
            auto* mesh = ent.AddComponent<MeshComponent>();
            mesh->meshID = rm.GetMeshID("Sphere");
            
            auto* mat = ent.AddComponent<RayTracingMaterial>();
            mat->color = {1.0f, 1.0f, 0.9f, 1.0f};
            mat->emission = 10.0f; // Very bright light source
            mat->roughness = 1.0f;
            
            std::cout << "Created Sun." << std::endl;
        }

        std::cout << "Entities properties set." << std::endl;

        // --- Main Loop ---
        float lastTime = 0.0f;
        
        std::cout << "Entering Main Loop..." << std::endl;
        while (engine.IsRunning()) {
            float time = engine.GetTime();
            float deltaTime = time - lastTime;
            lastTime = time;

            // --- Input Handling ---
            
            // --- Engine Logic (Systems) ---
            engine.Update(deltaTime);
            
            // --- Rendering ---
            glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

            // Path Tracing Render
            engine.GetRendering().RenderPathTracing(universe, camera);

            engine.RenderFrame();
        }

    } catch (const std::exception& e) {
        std::cerr << "EXCEPTION: " << e.what() << std::endl;
        // Keep console open
        int x; std::cin >> x; 
        return 1;
    }
    return 0;
}
