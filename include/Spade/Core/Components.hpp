#pragma once

#include <memory>
#include <string>
#include <vector>
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

  struct MeshComponent {
    Mesh mesh;

    BufferID VAO = 0;
    BufferID VBO = 0;
    BufferID EBO = 0;

    ~MeshComponent();
  };

  struct BoundingComponent {
    Sphere sphere;
    BoundingBox boundingBox;

    bool isSphere = false;
  };

  struct MaterialComponent {
    Material material;
  };

  struct CameraComponent {
    Camera camera;

    float fov = 45.0f;
    float nearPlane = 0.1f;
    float farPlane = 1000.0f;

    bool isActive = true;

    BufferID m_UBO = 0;

    ~CameraComponent();
  };

  struct MovementComponent {
    Movement movement;

  };

  struct InputComponent {
    glm::vec3 front = glm::vec3(0.0f, 0.0f, -1.0f);
    glm::vec3 up = glm::vec3(0.0f, 1.0f,  0.0f);
    float speed = 1.0f;

    std::unordered_map<int, Input> bindings;

  };

}