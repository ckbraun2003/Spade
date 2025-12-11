#pragma once

#include <string>
#include <vector>

#include <glm/glm.hpp>

namespace Spade {

  class Object {
  public:

    Object(const std::string& name = "");

    std::string m_Name;

  };


  struct Vertex {
    glm::vec3 position;
    glm::vec3 normal;
    glm::vec2 texCoords;
  };

  struct MeshData {
      std::vector<Vertex> vertices;
      std::vector<unsigned int> indices;
  };

  // Primitive Generators
  MeshData GenerateSphere(float radius = 1.0f, int sectors = 36, int stacks = 18);
  MeshData GenerateCube(float size = 1.0f);
  MeshData GenerateQuad(float size = 1.0f);

}