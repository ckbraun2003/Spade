#include <iostream>
#include <vector>
#include <chrono>
#include <cmath>

#include <Spade/Spade.hpp>

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
        std::cout << "Creating Entities..." << std::endl;
        // 1. Sphere
        {
            Entity ent = engine.CreateEmptyEntity("Sphere");
            ent.GetComponent<Transform>()->position = {-2.0f, 0.0f, 0.0f};
            
            auto* mesh = ent.AddComponent<MeshComponent>();
            mesh->meshID = rm.GetMeshID("Sphere");
            
            auto* mat = ent.AddComponent<MaterialComponent>();
            mat->materialID = rm.GetMaterialID("DefaultMaterial");
            mat->colorOverride = {1.0f, 0.2f, 0.2f, 1.0f}; // Red
            
            std::cout << "Created Sphere." << std::endl;
        }

        // 2. Cube
        {
            Entity ent = engine.CreateEmptyEntity("Cube");
            ent.GetComponent<Transform>()->position = {2.0f, 0.0f, 0.0f};
            
            auto* mesh = ent.AddComponent<MeshComponent>();
            mesh->meshID = rm.GetMeshID("Cube");
            
            auto* mat = ent.AddComponent<MaterialComponent>();
            mat->materialID = rm.GetMaterialID("DefaultMaterial");
            mat->colorOverride = {0.2f, 1.0f, 0.2f, 1.0f}; // Green
            
            std::cout << "Created Cube." << std::endl;
        }
        
        // 3. Floor Quad
        {
            Entity ent = engine.CreateEmptyEntity("Floor");
            ent.GetComponent<Transform>()->position = {0.0f, -1.0f, 0.0f};
            ent.GetComponent<Transform>()->scale = {10.0f, 10.0f, 1.0f};
            
            // Assign Quad mesh if it exists
            auto* mesh = ent.AddComponent<MeshComponent>();
            // Since we added Quad generation, get its ID
            // Assuming "Quad" was created in InitializeResources (I added it).
            mesh->meshID = rm.GetMeshID("Quad"); 
            
            std::cout << "Created Floor." << std::endl;
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
            
            // 1. Update Scripts
            engine.UpdateScripts(deltaTime);

            // 2. Interaction (Selecting/Dragging)
            engine.UpdateInteractionSystem(deltaTime);
            engine.UpdateCameraSystem(deltaTime);

            // Update Physics
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
