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
    };

    struct ObjectData {
        glm::mat4 model;
        glm::vec4 color; 
        // padding if necessary (std430 aligns vec4 to 16 bytes, mat4 is 64. Struct is 80 bytes. 16-byte alignment ok.)
    };

    unsigned int m_UBO;      // Camera/Frame data (Binding 0)
    unsigned int m_SSBO;     // Object data (Binding 1)
    
    // Temp buffer for each frame
    std::vector<ObjectData> m_ObjectBufferCPU;

  };

}