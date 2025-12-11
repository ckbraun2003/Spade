#include "Spade/Core/Engine.hpp"

#include <iostream>
#include <Spade/Core/Physics.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>

namespace Spade {

  Engine::Engine() : m_GLFWwindow(nullptr) {}

  Engine::~Engine() {
      ResourceManager::Get().Shutdown();
      glfwTerminate();
  }

  void Engine::SetupEngineWindow(int width, int height, const std::string& title) {
    m_Window = Window(width, height, title);
  }

  void Engine::Initialize() {
      SetupGLFWWindow();
      SetupGLADWindow();
  }

  bool Engine::IsRunning() const {
      return !glfwWindowShouldClose(m_GLFWwindow);
  }

  void Engine::Update() {
      glfwPollEvents();
  }

  void Engine::RenderFrame() {
      glfwSwapBuffers(m_GLFWwindow);
  }

  float Engine::GetTime() const {
      return (float)glfwGetTime();
  }

  bool Engine::IsKeyPressed(int key) const {
      return glfwGetKey(m_GLFWwindow, key) == GLFW_PRESS;
  }

  bool Engine::IsMouseButtonPressed(int button) const {
      return glfwGetMouseButton(m_GLFWwindow, button) == GLFW_PRESS;
  }

  glm::vec2 Engine::GetMousePosition() const {
      double x, y;
      glfwGetCursorPos(m_GLFWwindow, &x, &y);
      return { (float)x, (float)y };
  }

  void Engine::SetMouseCursorMode(bool trapped) {
      if (trapped) {
          glfwSetInputMode(m_GLFWwindow, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
      } else {
          glfwSetInputMode(m_GLFWwindow, GLFW_CURSOR, GLFW_CURSOR_NORMAL);
      }
  }

  void Engine::UpdateScripts(float deltaTime) {
      auto& scriptPool = m_Universe.GetPool<NativeScriptComponent>();
      for(size_t i = 0; i < scriptPool.m_Data.size(); ++i) {
          if (scriptPool.m_Data[i].OnUpdate) {
             EntityID id = scriptPool.m_IndexToEntity[i];
             scriptPool.m_Data[i].OnUpdate(Entity(id, &m_Universe), deltaTime);
          }
      }
  }

  void Engine::UpdateCameraSystem(float deltaTime) {
      auto& inputPool = m_Universe.GetPool<CameraInputComponent>();
      auto& transPool = m_Universe.GetPool<Transform>();

      for(size_t i = 0; i < inputPool.m_Data.size(); ++i) {
          EntityID id = inputPool.m_IndexToEntity[i];
          CameraInputComponent& input = inputPool.m_Data[i];
          Transform* t = transPool.Get(id);
          
          if (!t) continue;

          // Mouse Look Logic (FPS Style - Always Active if Component is Active)
          if (input.isActive) {
               SetMouseCursorMode(true); // Lock Cursor
               
               glm::vec2 mouse = GetMousePosition();
               
               if (input.firstMouse) {
                   input.lastMouse = mouse;
                   input.firstMouse = false;
               }
               
               float xoff = mouse.x - input.lastMouse.x;
               float yoff = input.lastMouse.y - mouse.y;
               input.lastMouse = mouse;

               glm::quat yaw = glm::angleAxis(glm::radians(-xoff * input.sensitivity), glm::vec3(0, 1, 0));

               glm::quat pitch = glm::angleAxis(glm::radians(yoff * input.sensitivity), glm::vec3(1, 0, 0));
               
               // Global Yaw, Local Pitch
               t->rotation = glm::normalize(yaw * t->rotation * pitch); 

               // Movement
               float velocity = input.speed * deltaTime;
               glm::vec3 fwd = t->GetForward();
               glm::vec3 right = t->GetRight();
               glm::vec3 up = glm::vec3(0, 1, 0); 
               
               if (IsKeyPressed(GLFW_KEY_W)) t->position += fwd * velocity;
               if (IsKeyPressed(GLFW_KEY_S)) t->position -= fwd * velocity;
               if (IsKeyPressed(GLFW_KEY_A)) t->position -= right * velocity;
               if (IsKeyPressed(GLFW_KEY_D)) t->position += right * velocity;
               if (IsKeyPressed(GLFW_KEY_Q)) t->position -= up * velocity;
               if (IsKeyPressed(GLFW_KEY_E)) t->position += up * velocity;

               // Unlock cursor on Escape
               if (IsKeyPressed(GLFW_KEY_M)) {
                   input.isActive = false;
                   SetMouseCursorMode(false);
                   input.firstMouse = true; // Reset so it doesn't jump on re-entry
               }

          } else {
              // Re-enable if clicked
              if (IsKeyPressed(GLFW_KEY_N)) { // Keep N as per user change
                  input.isActive = true;
                  input.firstMouse = true;
              }
          }
      }
  }

  // Simple state for interaction system (static or member?)
  // Ideally Engine members but for now static to keep implementation contained.
  // Warning: Not thread safe, but we are single threaded.
  static EntityID g_SelectedEntity = INVALID_ENTITY_ID;
  static bool g_IsDragging = false;
  static float g_DragDist = 0.0f; 

  void Engine::UpdateInteractionSystem(float deltaTime) {
      auto& camPool = m_Universe.GetPool<Camera>();
      if (camPool.m_Data.empty()) return;
      
      EntityID camID = camPool.m_IndexToEntity[0];
      Entity camEnt(camID, &m_Universe);
      Transform* camTrans = camEnt.GetComponent<Transform>();
      Camera* cam = camEnt.GetComponent<Camera>();

      if (!camTrans || !cam) return;

      // Picking Logic
      // Center Screen Ray (Locked Cursor) or Mouse Ray (Unlocked)
      
      // Determine Cursor Mode
      bool cursorLocked = (glfwGetInputMode(m_GLFWwindow, GLFW_CURSOR) == GLFW_CURSOR_DISABLED);
      
      // We trigger interaction on Click
      if (IsMouseButtonPressed(GLFW_MOUSE_BUTTON_1)) {
           
           Ray ray;
           if(cursorLocked) {
               // Center of screen
               ray.origin = camTrans->position;
               ray.direction = camTrans->GetForward();
           } else {
               // Normal Mouse Picking
               glm::vec2 mouse = GetMousePosition();
               int width = 1280, height = 720; // TODO: Window
               float x = (2.0f * mouse.x) / width - 1.0f;
               float y = 1.0f - (2.0f * mouse.y) / height;

               glm::mat4 proj = glm::perspective(glm::radians(cam->fov), (float)width/height, cam->nearPlane, cam->farPlane);
               glm::mat4 view = glm::inverse(camTrans->GetMatrix());

               glm::vec4 ray_clip = glm::vec4(x, y, -1.0, 1.0);
               glm::vec4 ray_eye = glm::inverse(proj) * ray_clip;
               ray_eye = glm::vec4(ray_eye.x, ray_eye.y, -1.0, 0.0);
               glm::vec3 ray_wor = glm::vec3(glm::inverse(view) * ray_eye);
               ray.origin = camTrans->position;
               ray.direction = glm::normalize(ray_wor);
           }
           
           // Interaction Logic (Same as before)
           if (!g_IsDragging) {
               // Find Closest
               float closestT = 10000.0f;
               EntityID closestID = INVALID_ENTITY_ID;

               auto& transformPool = m_Universe.GetPool<Transform>();
               for(auto& pair : transformPool.m_EntityToIndex) {
                   EntityID id = pair.first;
                   if (id == camID) continue;
                   
                   Transform* t = &transformPool.m_Data[pair.second];
                   BoundingSphere sphere{t->position, 1.5f};
                   
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
               // Dragging logic...
               if (g_SelectedEntity != INVALID_ENTITY_ID) {
                   Transform* t = m_Universe.GetPool<Transform>().Get(g_SelectedEntity);
                   if (t) {
                       // Move object to specific distance in front of camera?
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

  Entity Engine::CreateEntity(const std::string& name) {

    EntityID id = m_Universe.CreateEntityID();
    Entity entity(id, &m_Universe);

    entity.AddComponent<Transform>();
    
    return entity;

  }

  Entity Engine::CreateEmptyEntity(const std::string &name) {

    return CreateEntity(name);

  }

  Entity Engine::CreateCameraEntity(const std::string &name) {

    Entity entity = CreateEntity(name);
    entity.AddComponent<Camera>();
    entity.AddComponent<CameraInputComponent>();
    return entity;

  }


  void Engine::SetupGLFWWindow() {

    glfwInit();
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    // Creates the GLFW Window with a Width, Height, Title, etc
    m_GLFWwindow = glfwCreateWindow(m_Window.width, m_Window.height, m_Window.title.c_str(), nullptr, nullptr);

    // Error handling
    if (m_GLFWwindow == nullptr)
    {
      glfwTerminate();
      throw EngineException("Failed to create GLFW window");
    }

    // Makes the window the current context for the thread
    glfwMakeContextCurrent(m_GLFWwindow);

  }

  void Engine::SetupGLADWindow() {

    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
    {
      throw EngineException("Failed to initialize GLAD");
    }

    // Tells OpenGL size of rendering window
    glViewport(0, 0, m_Window.width, m_Window.height);
    glEnable(GL_DEPTH_TEST);

    // Initialize Standard Resources now that GL is ready
    m_Rendering.InitializeResources();
  }

  Engine::EngineException::EngineException(const std::string &message) : runtime_error(message) {}
}
