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

    void LoadCameraBuffers(Universe& universe);
    void LoadInstanceBuffers(Universe& universe);
    void LoadCollisionBuffers(Universe& universe);
    void LoadFluidBuffers(Universe& universe);

    // Physics Systems
    void EnableGravity(float gravity, float deltaTime);
    void EnableMotion(float deltaTime);

    void EnableBruteForceCollision(float bounds, float deltaTime);
    void EnableGridCollision(float bounds, float cellSize, float deltaTime);

    void EnableSPHFluid(float globalBounds, float cellSize, float deltaTime);

    void EnableBruteForceNewtonianGravity(float gravityConstant, float deltaTime);

    // Render Systems

    // Main functions
    void DrawScene(Universe& universe, const std::string& fragmentShaderFile);

    // Input Handling
    void ProcessInput(Universe &universe);


    // Engine API Functions
    [[nodiscard]] bool IsRunning() const;
    [[nodiscard]] float GetTime() const;
    [[nodiscard]] float GetDeltaTime() const { return m_DeltaTime; }
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

    void BuildGrid(float globalBounds, float cellSize);

    // Window Variables
    std::string m_WindowTitle;
    glm::vec2 m_WindowSize = {800.0, 600.0};
    GLFWwindow* m_GLFWwindow = nullptr;

    // Frame Statistics
    float m_LastTime = 0.0f;
    float m_CurrentTime = 0.0f;
    float m_DeltaTime = 0.016f;
    float m_FPSTimer = 0.0f;
    float m_Memory = 0.0f;
    unsigned int m_FrameCounter = 0;

    float m_FPS = 0.0f;
    unsigned int m_TotalFrames = 0;

    // --Cache--
    std::vector<Transform> m_InstanceTransforms;
    std::vector<Motion> m_InstanceMotions;
    std::vector<Material> m_InstanceMaterials;
    std::vector<unsigned int> m_InstanceToEntityIndex;

    CameraComponent m_ActiveCamera{};

    // Shader
    ProgramID m_ShaderProgram = 0;
    ProgramID m_MotionProgram = 0;
    ProgramID m_CollisionProgram = 0;
    ProgramID m_GravityProgram = 0;
    
    ProgramID m_GridClearProgram = 0;
    ProgramID m_GridBuildProgram = 0;
    ProgramID m_GridCollisionProgram = 0;

    // Fluid Programs
    ProgramID m_FluidDensityProgram = 0;
    ProgramID m_FluidForceProgram = 0;

    ProgramID m_NewtonianGravityProgram = 0;
    
    // Sorted Grid Programs
    ProgramID m_BitonicSortProgram = 0;
    ProgramID m_GridOffsetsProgram = 0;
    ProgramID m_GridReorderProgram = 0;
    ProgramID m_GridScatterProgram = 0;

    // Buffers
    BufferID m_SSBO_InstanceTransforms = 0;
    BufferID m_SSBO_InstanceMotions = 0;
    BufferID m_SSBO_InstanceMaterials = 0;
    BufferID m_SSBO_InstanceToEntityIndex = 0;

    BufferID m_SSBO_EntityBounds = 0;
    BufferID m_SSBO_EntityFluidMaterials = 0;
    
    // Grid Buffers
    BufferID m_SSBO_GridHead = 0;
    BufferID m_SSBO_GridPairs = 0;
    BufferID m_SSBO_SortedTransforms = 0;
    BufferID m_SSBO_SortedMotions = 0;
    

    BufferID m_UBO_Camera = 0;


    class EngineException : public std::runtime_error
    {
    public:
      explicit EngineException(const std::string& message);
    };

  };

}
