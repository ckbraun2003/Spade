#pragma once

#include <cmath>
#include <vector>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

/*
==========================================
GLSL BUFFER ALIGNMENT & PADDING REFERENCE
==========================================

------------------------------------------
std140  (GLSL 140+ / UBOs)
------------------------------------------
Scalar alignment:
  float / int / uint / bool : 4 bytes

Vector alignment:
  vec2  : 8 bytes
  vec3  : 16 bytes   (padded like vec4)
  vec4  : 16 bytes

Size rules:
  - vec3 occupies 16 bytes (last 4 bytes padding)
  - Arrays: each element rounded up to 16 bytes
  - Struct members are aligned individually
  - Struct size is rounded up to a multiple of 16 bytes

Example:
  vec3 a;   // offset 0, size 16
  float b;  // offset 16
  // total struct size = 32 bytes

------------------------------------------
std430  (GLSL 430+ / SSBOs)
------------------------------------------
Scalar alignment:
  float / int / uint / bool : 4 bytes

Vector alignment:
  vec2  : 8 bytes
  vec3  : 16 bytes   (still aligned to 16!)
  vec4  : 16 bytes

Size rules:
  - vec3 size is 12 bytes, but alignment is 16
  - Arrays are tightly packed (no forced 16-byte stride)
  - Struct size is NOT rounded to 16 bytes
  - Members only aligned to their base alignment

Example:
  vec3 a;   // offset 0, size 12
  float b;  // offset 12
  // total struct size = 16 bytes

------------------------------------------
IMPORTANT NOTES
------------------------------------------
- vec3 ALWAYS has 16-byte alignment (std140 + std430)
- std140 pads aggressively (UBO-friendly, cache-safe)
- std430 packs tightly (SSBO-friendly, more efficient)
- Always mirror padding explicitly in C/C++ structs

==========================================
*/


namespace Spade {

  struct Vertex {
    glm::vec3 position = {0.0, 0.0, 0.0};
    glm::vec3 normal = {0.0, 0.0, 0.0};
    glm::vec2 texCoords = {0.0, 0.0};
  };

  struct Mesh {
    std::vector<Vertex> vertices;
    std::vector<unsigned int> indices;
  };

  struct BoundingBox {
    unsigned int startTriangleIndex;
    unsigned int size;
    glm::vec3 center;
    glm::vec3 minimum;
    glm::vec3 maximum;
  };

  struct Material {
    glm::vec4 color = {1.0, 1.0, 1.0, 1.0};
    float emission = 0.0;
    float roughness = 0.0;
    float metallic = 0.0;
    float padding;
  };

  struct Triangle {
    Vertex vertexA;
    Vertex vertexB;
    Vertex vertexC;

  };

  struct Sphere {
    float radius;
  };

  struct Transform {
    glm::vec3 position = {0.0, 0.0, 0.0};
    float padding;
    glm::quat rotation = {1.0, 0.0, 0.0, 0.0};
    glm::vec3 scale = {1.0, 1.0, 1.0};
    float padding2;
  };

  struct Motion {
    glm::vec3 velocity = {0.0, 0.0, 0.0};
    float mass = 1;
    glm::vec3 acceleration = {0.0, 0.0, 0.0};
    float padding;
  };

  struct Camera {
    glm::mat4 view;
    glm::mat4 projection;
    glm::mat4 viewInverse;
    glm::mat4 projInverse;
  };

  Mesh GenerateQuad(float size);
  Mesh GenerateCube(float size);
  Mesh GenerateSphere(float radius, int sectors, int stacks);

}