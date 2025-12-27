#include "Spade/Core/Components.hpp"

namespace Spade {

  glm::mat4 TransformComponent::GetModel() const {
      glm::mat4 mat = model;
      mat = glm::translate(mat, transform.position);
      mat *= glm::toMat4(transform.rotation);
      mat = glm::scale(mat, transform.scale);
      return mat;
  }

  glm::vec3 TransformComponent::GetForward() const {
    return glm::rotate(transform.rotation, glm::vec3(0.0f, 0.0f, -1.0f));
  }

  glm::vec3 TransformComponent::GetRight() const {
    return glm::rotate(transform.rotation, glm::vec3(1.0f, 0.0f, 0.0f));
  }

  glm::vec3 TransformComponent::GetUp() const {
    return glm::rotate(transform.rotation, glm::vec3(0.0f, 1.0f, 0.0f));
  }

  MeshComponent::~MeshComponent() {
    if (VAO) glDeleteVertexArrays(1, &VAO);
    if (VBO) glDeleteBuffers(1, &VBO);
    if (EBO) glDeleteBuffers(1, &EBO);
  }

  CameraComponent::~CameraComponent() {
    if (m_UBO) glDeleteBuffers(1, &m_UBO);
  }

  void BasicMovement(Transform& transform, Movement& movement, float deltaTime) {

    transform.position += movement.velocity * deltaTime;

  }

}