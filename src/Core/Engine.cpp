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
  
  void Engine::Update(float deltaTime) {
       glfwPollEvents();
       
       m_ScriptSystem.Update(m_Universe, deltaTime);
       m_InteractionSystem.Update(m_Universe, m_GLFWwindow, deltaTime);
       m_CameraSystem.Update(m_Universe, m_GLFWwindow, deltaTime);
       m_Physics.Update(m_Universe, deltaTime);
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

 // Old systems removed

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
