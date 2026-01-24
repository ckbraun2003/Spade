#pragma once

#include <memory>
#include <string>
#include <vector>
#include <random>
#include <cmath>
#include <functional>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/quaternion.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include "Spade/Core/Resources.hpp"
#include "Spade/Core/Primitives.hpp"
#include "Spade/Core/Enums.hpp"

namespace Spade {

  struct TransformComponent {
    // Primitve Data
    Transform transform;

    // Component Data
    glm::mat4 model = glm::mat4(1.0f);
    
    // Helper functions
    glm::mat4 GetModel() const;
    glm::vec3 GetForward() const;
    glm::vec3 GetRight() const;
    glm::vec3 GetUp() const;
  };

  struct MotionComponent {
    Motion motion;
  };

  struct MeshComponent {
    Mesh mesh;

    std::vector<Transform> instanceTransforms;
    std::vector<Motion> instanceMotions;
    std::vector<Material> instanceMaterials;

    unsigned int instanceStartIndex = 0;

    BufferID VAO = 0;
    BufferID VBO = 0;
    BufferID EBO = 0;

    ~MeshComponent();

    void SpawnInstancesInSphere(float radius, glm::vec3 center, int count);
    void SpawnInstancesInCube(float size, glm::vec3 center, int count);
    void SetVelocity(glm::vec3 velocity);
    void SetColor(glm::vec4 color);
    void SetMass(float mass);

    void RandomizeVelocity();
    void RandomizeColor();
  };

  struct BoundingComponent {
    Bound bound;
  };

  struct MaterialComponent {
    Material material;
  };

  struct FluidComponent {
    FluidMaterial fluidMaterial;
  };

  struct CameraComponent {
    Camera camera;

    float fov = 90.0f;
    float nearPlane = 0.1f;
    float farPlane = 1000.0f;

    bool isActive = true;
  };

  struct InputComponent {
    glm::vec3 front = glm::vec3(0.0f, 0.0f, -1.0f);
    glm::vec3 up = glm::vec3(0.0f, 1.0f,  0.0f);
    float speed = 1.0f;

    std::unordered_map<int, Input> bindings;

  };

}