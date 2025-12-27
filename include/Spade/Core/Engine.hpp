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

    // Draw
    void InitializeUniverse(Universe& universe);
    void DrawScene(Universe& universe);

    // Inputs
    void ProcessInput(Universe &universe);


    // Engine API Functions
    bool IsRunning() const;
    float GetTime() const;
    float GetFPS() const { return m_FPS; }
    float GetMemory() const { return m_Memory; }
    bool IsKeyPressed(int key) const;
    bool IsMouseButtonPressed(int button) const;
    glm::vec2 GetMousePosition() const;
    void SetMouseCursorMode(bool trapped);
    void SetupEngineWindow(int width, int height, const std::string& title);

  private:

    void SetupGLFWandGLADWindow(const int& width, const int& height, const std::string& title);

    void SetupStorageCache(Universe& universe);
    void SetupCameraCache(Universe& universe);
    void SetupShaderStorageBuffers();
    void ClearBufferCache() { m_Spheres.clear(); m_Triangles.clear(); m_BoundingBoxes.clear(); m_RenderTables.clear(); m_Transforms.clear(); m_Materials.clear(); }

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

    bool m_IsRunning = true;
    bool m_IsDirty = true;

    // Camera (--Cache--)
    CameraComponent* m_ActiveCamera = nullptr;

    // Shader Storage Buffers (--Cache--)
    std::vector<RenderTable> m_RenderTables;
    std::vector<Transform> m_Transforms;
    std::vector<Material> m_Materials;
    std::vector<Sphere> m_Spheres;
    std::vector<Triangle> m_Triangles;
    std::vector<BoundingBox> m_BoundingBoxes;

    // IDS
    ProgramID m_ShaderProgram = 0;
    BufferID m_SSBO_Triangles = 0;
    BufferID m_SSBO_Spheres = 0;
    BufferID m_SSBO_BoundingBoxes = 0;
    BufferID m_SSBO_Materials = 0;
    BufferID m_SSBO_Transforms = 0;
    BufferID m_SSBO_RenderTables = 0;

    class EngineException : public std::runtime_error
    {
    public:
      explicit EngineException(const std::string& message);
    };

  };

}