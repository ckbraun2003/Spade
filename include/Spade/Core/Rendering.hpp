#pragma once

#include <string>
#include <stdexcept>

#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>

#include "Spade/Core/Objects.hpp"
#include <vector>

namespace Spade {

  class Rendering
  {
  public:

    Rendering();

    // Initialize standard resources (quad, cube, etc)
    void InitializeResources();

    void Render(Universe& universe, Entity camera);
    
    // Ray Tracing
    void RenderPathTracing(Universe& universe, Entity camera);
    void BuildPathTracingScene(Universe& universe);
    void MarkSceneDirty() { m_SceneDirty = true; }
    
    void SetRenderResolution(int width, int height) { m_RenderResolution = glm::ivec2(width, height); }
    glm::ivec2 GetRenderResolution() const { return m_RenderResolution; }

    void SetBounceDepth(int bounces) { m_MaxBounces = bounces; }
    void SetRaysPerFrame(int rays) { m_RaysPerFrame = rays; } // Runs pipeline N times per frame

  private:

    class RenderException : public std::runtime_error
    {
    public:
      RenderException(const std::string& message);
    };

    void SetupBuffers();

    struct PerFrameUniforms {
        glm::mat4 view;
        glm::mat4 projection;
        glm::mat4 viewInverse;
        glm::mat4 projInverse;
    };

    struct ObjectData {
        glm::mat4 model;
        glm::vec4 color; 
        float emission;
        float padding[3];
    };

    unsigned int m_UBO;      // Camera/Frame data (Binding 0)
    unsigned int m_SSBO;     // Object data (Binding 1)
    
    // Temp buffer for each frame
    std::vector<ObjectData> m_ObjectBufferCPU;
    
    bool m_SceneDirty = true;
    glm::ivec2 m_RenderResolution = {1920, 1080};
    int m_MaxBounces = 2;
    int m_RaysPerFrame = 1;
    unsigned int m_FrameCount = 0;
    
    struct PathTracingContext {
        unsigned int sphereBuffer = 0;
        unsigned int triangleBuffer = 0;
        unsigned int materialBuffer = 0;
    } m_PTContext;

  };

}