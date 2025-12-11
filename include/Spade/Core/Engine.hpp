#pragma once

#include <string>
#include <stdexcept>
#include <tuple>
#include <memory>
#include <glm/glm.hpp>

#include <glad/glad.h>
#include <GLFW/glfw3.h>

#include "Spade/Core/Primitives.hpp"
#include "Spade/Core/Objects.hpp"
#include "Spade/Core/Components.hpp"
#include "Spade/Core/Rendering.hpp"
#include "Spade/Core/Physics.hpp"

#include "Spade/Systems/ScriptSystem.hpp"
#include "Spade/Systems/CameraSystem.hpp"
#include "Spade/Systems/InteractionSystem.hpp"

namespace Spade {

  class Engine
  {
  public:

    Engine();
    ~Engine();

    void SetupEngineWindow(int width, int height, const std::string& title);
    void Initialize();

    // Game Loop API
    bool IsRunning() const;
    void Update(float deltaTime);
    void RenderFrame();
    float GetTime() const;

    // Systems
    // Decoupled into their own classes


    // Input API
    bool IsKeyPressed(int key) const;
    bool IsMouseButtonPressed(int button) const;
    glm::vec2 GetMousePosition() const;
    void SetMouseCursorMode(bool trapped);

    Entity CreateEmptyEntity(const std::string& name);
    Entity CreateCameraEntity(const std::string& name);

    Universe& GetUniverse() { return m_Universe; }
    Rendering& GetRendering() { return m_Rendering; }
    Physics& GetPhysics() { return m_Physics; }
    
    GLFWwindow* GetGLFWWindow() { return m_GLFWwindow; }

  private:

    class EngineException : public std::runtime_error
    {
    public:
       EngineException(const std::string& message);
    };

    void SetupGLFWWindow();
    void SetupGLADWindow();

    Entity CreateEntity(const std::string& name);

    struct Window {
      int width;
      int height;
      std::string title;
      Window(int w, int h, std::string t) : width(w), height(h), title(t) {}
      Window() : width(800), height(600), title("Engine") {}
    };

    Window m_Window;
    GLFWwindow* m_GLFWwindow;

    Universe m_Universe;
    Rendering m_Rendering;
    Physics m_Physics;
    
    // Logic Systems
    ScriptSystem m_ScriptSystem;
    CameraSystem m_CameraSystem;
    InteractionSystem m_InteractionSystem;

  };

}