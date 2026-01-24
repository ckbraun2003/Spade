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

  void MeshComponent::SpawnInstancesInSphere(float radius, glm::vec3 center, int count) {
    // Setup Random Number Generator
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<float> dis(0.0f, 1.0f);
    std::uniform_real_distribution<float> disFull(-1.0f, 1.0f);

    // Constants
    const float PI_2 = 6.28318530718f; // 2 * Pi

    for (int i = 0; i < count; ++i) {
      Transform transform;
      Motion motion;
      Material material;

      // 1. RANDOMIZE DIRECTION
      // We pick a random height (z) and a random angle around the vertical axis (theta).
      // This naturally distributes points evenly on a sphere's surface (Archimedes' Hat-Box Theorem).
      float z = disFull(gen); // Random value between -1 and 1
      float theta = dis(gen) * PI_2; // Random angle 0 to 2*Pi

      // Calculate the horizontal radius at this z-height
      // derived from: sin(phi) = sqrt(1 - cos^2(phi))
      float horizontal_dist = std::sqrt(1.0f - z * z);

      float x = horizontal_dist * std::cos(theta);
      float y = horizontal_dist * std::sin(theta);

      // 2. RANDOMIZE DISTANCE FROM CENTER (Radius)
      // We assume the point is currently on the *surface* of a unit sphere (radius 1).
      // We now scale it. We must use Cube Root (cbrt) so points don't cluster in the center.
      float u = dis(gen);
      float scale = radius * std::cbrt(u);

      // 3. APPLY SCALE
      transform.position = {
        center.x + (x * scale),
        center.y + (y * scale),
        center.z + (z * scale)};

      instanceTransforms.push_back(transform);
      instanceMotions.push_back(motion);
      instanceMaterials.push_back(material);
    }
  }

  void MeshComponent::SpawnInstancesInCube(float size, glm::vec3 center, int count) {
    // 1. Calculate dimensions
    // We estimate how many points per axis we need to fit the totalCount.
    // We use ceil() to ensure we have enough room for all particles.
    int particlesPerSide = std::ceil(std::cbrt(count));

    // 2. Calculate the spacing (Step size) between particles
    // We divide the length by (count - 1) so the grid touches the exact edges of the cube.
    float step = size / (float)(particlesPerSide - 1);

    // 3. Determine the starting corner (Bottom-Left-Back relative to center)
    float halfSize = size * 0.5f;
    glm::vec3 startPos = {
      center.x - halfSize,
      center.y - halfSize,
      center.z - halfSize
  };

    // 4. Loop through the Grid
    for (int z = 0; z < particlesPerSide; ++z) {
      for (int y = 0; y < particlesPerSide; ++y) {
        for (int x = 0; x < particlesPerSide; ++x) {
          Transform transform;
          Motion motion;
          Material material;

          // Stop immediately if we have reached the requested count
          // (This happens if totalCount isn't a perfect cube number)
          if (instanceTransforms.size() >= count) break;

          glm::vec3 p;
          p.x = startPos.x + (x * step);
          p.y = startPos.y + (y * step);
          p.z = startPos.z + (z * step);

          transform.position = p;

          instanceTransforms.push_back(transform);
          instanceMotions.push_back(motion);
          instanceMaterials.push_back(material);
        }
      }
    }
  }


  void MeshComponent::SetColor(glm::vec4 color) {
    for (int i = 0; i < instanceTransforms.size(); ++i) {
      instanceMaterials[i].color = color;
    }
  }

  void MeshComponent::SetVelocity(glm::vec3 velocity) {
    for (int i = 0; i < instanceTransforms.size(); ++i) {
      instanceMotions[i].velocity = velocity;
    }
  }

  void MeshComponent::SetMass(float mass) {
    for (int i = 0; i < instanceTransforms.size(); ++i) {
      instanceMotions[i].mass = mass;
    }
  }

  void MeshComponent::RandomizeColor() {
    // Setup Random Number Generator
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<float> dis(-1.0f, 1.0f);

    for (int i = 0; i < instanceTransforms.size(); ++i) {
      instanceMaterials[i].color = {
        dis(gen),
        dis(gen),
        dis(gen),
        1.0f
      };
    }
  }

  void MeshComponent::RandomizeVelocity() {
    // Setup Random Number Generator
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<float> dis(-1.0f, 1.0f);

    for (int i = 0; i < instanceTransforms.size(); ++i) {
      instanceMotions[i].velocity = {
        dis(gen),
        dis(gen),
        dis(gen)
      };
    }
  }


}