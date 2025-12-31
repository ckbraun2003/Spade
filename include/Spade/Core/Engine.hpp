#pragma once

#include <string>
#include <stdexcept>
#include <tuple>
#include <memory>

#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <windows.h>
#include <psapi.h>

#include "Spade/Core/Primitives.hpp"
#include "Spade/Core/Objects.hpp"
#include "Spade/Core/Components.hpp"
#include "Spade/Core/Resources.hpp"

namespace Spade {

  class Engine
  {
  public:

    ~Engine();

    void LoadCameraBuffer(Universe& universe);
    void LoadMeshBuffers(Universe& universe);

    // Physics
    void UpdatePhysics(Universe& universe);

    // Draw
    void DrawScene(Universe& universe);

    // Input Handling
    void ProcessInput(Universe &universe);


    // Engine API Functions
    [[nodiscard]] bool IsRunning() const;
    [[nodiscard]] float GetTime() const;
    [[nodiscard]] float GetFPS() const { return m_FPS; }
    [[nodiscard]] float GetMemory() const { return m_Memory; }
    [[nodiscard]] bool IsKeyPressed(int key) const;
    [[nodiscard]] bool IsMouseButtonPressed(int button) const;
    [[nodiscard]] glm::vec2 GetMousePosition() const;
    void SetMouseCursorMode(bool trapped);
    void SetupEngineWindow(int width, int height, const std::string& title);

  private:

    void SetupGLFWandGLADWindow(const int& width, const int& height, const std::string& title);

    void SaveRenderToFile(const std::string& fileName);
    void UpdateStatistics();

    bool IsKeyPressed(int key);

    // Window Variables
    std::string m_WindowTitle;
    glm::vec2 m_WindowSize = {800.0, 600.0};
    GLFWwindow* m_GLFWwindow = nullptr;

    // Frame Statistics
    float m_LastTime = 0.0f;
    float m_CurrentTime = 0.0f;
    float m_DeltaTime = 0.0f;
    float m_FPSTimer = 0.0f;
    float m_Memory = 0.0f;
    unsigned int m_FrameCounter = 0;

    float m_FPS = 0.0f;
    unsigned int m_TotalFrames = 0;

    // Camera (--Cache--)
    CameraComponent m_ActiveCamera;

    // Shader
    ProgramID m_ShaderProgram = 0;
    ProgramID m_ComputeProgram = 0;
    BufferID m_UBO_Camera = 0;

    BufferID m_SSBO_EntityTransforms = 0;
    BufferID m_SSBO_InstanceTransforms = 0;

    BufferID m_SSBO_EntityMotions = 0;
    BufferID m_SSBO_InstanceMotions = 0;

    BufferID m_UBO_EntityTransformIndex = 0;
    BufferID m_UBO_InstanceTransformStartIndex = 0;

    class EngineException : public std::runtime_error
    {
    public:
      explicit EngineException(const std::string& message);
    };

  };

}