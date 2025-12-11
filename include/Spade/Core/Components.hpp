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

namespace Spade {

  struct Transform {
    glm::vec3 position = {0.0f, 0.0f, 0.0f};
    glm::quat rotation = glm::quat(1.0f, 0.0f, 0.0f, 0.0f); // Identity
    glm::vec3 scale = {1.0f, 1.0f, 1.0f};
    
    // Matrix generation helper
    glm::mat4 GetMatrix() const {
        glm::mat4 mat = glm::mat4(1.0f);
        mat = glm::translate(mat, position);
        mat *= glm::toMat4(rotation);
        mat = glm::scale(mat, scale);
        return mat;
    }

    glm::vec3 GetForward() const {
        return glm::rotate(rotation, glm::vec3(0.0f, 0.0f, -1.0f));
    }
    
    glm::vec3 GetRight() const {
        return glm::rotate(rotation, glm::vec3(1.0f, 0.0f, 0.0f));
    }
    
    glm::vec3 GetUp() const {
        return glm::rotate(rotation, glm::vec3(0.0f, 1.0f, 0.0f));
    }
  };

  struct MeshComponent {
    ResourceID meshID = INVALID_RESOURCE_ID;
  };

  struct MaterialComponent {
      ResourceID materialID = INVALID_RESOURCE_ID;
      glm::vec4 colorOverride = {1.0f, 1.0f, 1.0f, 1.0f}; 
  };

  struct Camera {
      float fov = 45.0f;
      float nearPlane = 0.1f;
      float farPlane = 100.0f;
  };

  struct CameraInputComponent {
      float speed = 5.0f;
      float sensitivity = 0.1f;
      bool isActive = true;
      glm::vec2 lastMouse = {0.0f, 0.0f};
      bool firstMouse = true;
  };

  struct NativeScriptComponent {
      std::function<void(Entity, float)> OnUpdate;
  };

}