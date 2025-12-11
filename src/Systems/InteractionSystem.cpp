#include "Spade/Systems/InteractionSystem.hpp"
#include "Spade/Core/Components.hpp"
#include "Spade/Core/Physics.hpp"

namespace Spade {

    static EntityID g_SelectedEntity = INVALID_ENTITY_ID;
    static bool g_IsDragging = false;

    void InteractionSystem::Update(Universe& universe, GLFWwindow* window, float deltaTime) {
        auto& camPool = universe.GetPool<Camera>();
        if (camPool.m_Data.empty()) return;
        
        // Assume first camera is main
        EntityID camID = camPool.m_IndexToEntity[0];
        // Need simpler Get helper
        Transform* camTrans = universe.GetPool<Transform>().Get(camID);
        Camera* cam = &camPool.m_Data[0];
        
        if (!camTrans || !cam) return;

        bool cursorLocked = (glfwGetInputMode(window, GLFW_CURSOR) == GLFW_CURSOR_DISABLED);
        
        if (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_1) == GLFW_PRESS) {
             
             Ray ray;
             if(cursorLocked) {
                 ray.origin = camTrans->position;
                 ray.direction = camTrans->GetForward();
             } else {
                 double mx, my;
                 glfwGetCursorPos(window, &mx, &my);
                 int width, height;
                 glfwGetWindowSize(window, &width, &height);

                 float x = (2.0f * (float)mx) / width - 1.0f;
                 float y = 1.0f - (2.0f * (float)my) / height;

                 glm::mat4 proj = glm::perspective(glm::radians(cam->fov), (float)width/height, cam->nearPlane, cam->farPlane);
                 glm::mat4 view = glm::inverse(camTrans->GetMatrix());

                 glm::vec4 ray_clip = glm::vec4(x, y, -1.0, 1.0);
                 glm::vec4 ray_eye = glm::inverse(proj) * ray_clip;
                 ray_eye = glm::vec4(ray_eye.x, ray_eye.y, -1.0, 0.0);
                 glm::vec3 ray_wor = glm::vec3(glm::inverse(view) * ray_eye);
                 ray.origin = camTrans->position;
                 ray.direction = glm::normalize(ray_wor);
             }
             
             if (!g_IsDragging) {
                 float closestT = 10000.0f;
                 EntityID closestID = INVALID_ENTITY_ID;

                 auto& transformPool = universe.GetPool<Transform>();
                 // Iterate all transforms
                 for(size_t i=0; i<transformPool.m_Data.size(); ++i) {
                     EntityID id = transformPool.m_IndexToEntity[i];
                     if (id == camID) continue;
                     
                     Transform* t = &transformPool.m_Data[i];
                     BoundingSphere sphere{t->position, 1.5f}; // Fixed size picker
                     
                     float dist;
                     if (RayIntersectSphere(ray, sphere, dist)) {
                         if (dist < closestT) {
                             closestT = dist;
                             closestID = id;
                         }
                     }
                 }
                 
                 if (closestID != INVALID_ENTITY_ID) {
                     g_SelectedEntity = closestID;
                     g_IsDragging = true;
                 }
             } else {
                 if (g_SelectedEntity != INVALID_ENTITY_ID) {
                     Transform* t = universe.GetPool<Transform>().Get(g_SelectedEntity);
                     if (t) {
                         glm::vec3 planeNormal = -camTrans->GetForward();
                         Plane dragPlane{t->position, planeNormal}; 

                         float dist;
                         if (RayIntersectPlane(ray, dragPlane, dist)) {
                             glm::vec3 target = ray.origin + ray.direction * dist;
                             t->position = target;
                         }
                     }
                 }
             }
        } else {
            g_IsDragging = false;
        }
    }

}
