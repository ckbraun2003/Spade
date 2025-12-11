#include "Spade/Systems/CameraSystem.hpp"
#include "Spade/Core/Components.hpp"
#include <iostream>
#include <glm/gtc/quaternion.hpp>

namespace Spade {

    // Helper input checks
    static bool IsKeyPressed(GLFWwindow* window, int key) {
        return glfwGetKey(window, key) == GLFW_PRESS;
    }

    void CameraSystem::Update(Universe& universe, GLFWwindow* window, float deltaTime) {
        auto& inputPool = universe.GetPool<CameraInputComponent>();
        auto& transPool = universe.GetPool<Transform>();

        // Iterate all input components
        for(size_t i = 0; i < inputPool.m_Data.size(); ++i) {
            
            // Forward compatible lookup
            EntityID id = inputPool.m_IndexToEntity[i];

            CameraInputComponent& input = inputPool.m_Data[i];
            Transform* t = transPool.Get(id);
            
            if (!t) continue;

            // Logic copied from Engine.cpp
            if (input.isActive) {
                glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
                
                double x, y;
                glfwGetCursorPos(window, &x, &y);
                glm::vec2 mouse = {(float)x, (float)y};
                
                if (input.firstMouse) {
                    input.lastMouse = mouse;
                    input.firstMouse = false;
                }
                
                float xoff = mouse.x - input.lastMouse.x;
                float yoff = input.lastMouse.y - mouse.y;
                input.lastMouse = mouse;

                glm::quat yaw = glm::angleAxis(glm::radians(-xoff * input.sensitivity), glm::vec3(0, 1, 0));
                glm::quat pitch = glm::angleAxis(glm::radians(yoff * input.sensitivity), glm::vec3(1, 0, 0));
                
                t->rotation = glm::normalize(yaw * t->rotation * pitch); 

                float velocity = input.speed * deltaTime;
                glm::vec3 fwd = t->GetForward();
                glm::vec3 right = t->GetRight();
                glm::vec3 up = glm::vec3(0, 1, 0); 
                
                if (IsKeyPressed(window, GLFW_KEY_W)) t->position += fwd * velocity;
                if (IsKeyPressed(window, GLFW_KEY_S)) t->position -= fwd * velocity;
                if (IsKeyPressed(window, GLFW_KEY_A)) t->position -= right * velocity;
                if (IsKeyPressed(window, GLFW_KEY_D)) t->position += right * velocity;
                if (IsKeyPressed(window, GLFW_KEY_Q)) t->position -= up * velocity;
                if (IsKeyPressed(window, GLFW_KEY_E)) t->position += up * velocity;

                if (IsKeyPressed(window, GLFW_KEY_M)) {
                    input.isActive = false;
                    glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_NORMAL);
                    input.firstMouse = true; 
                }

            } else {
                if (IsKeyPressed(window, GLFW_KEY_N)) { 
                    input.isActive = true;
                    input.firstMouse = true;
                }
            }
        }
    }

}
