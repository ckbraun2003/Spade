#include "Spade/Core/Physics.hpp"
#include "Spade/Core/Components.hpp"
#include <iostream>

namespace Spade {

  Physics::Physics() {}

  void Physics::Update(Universe& universe, float deltaTime) {
      // Basic Physics Loop
      // Ideally we iterate rigid bodies, but for now let's just say we iterate transforms 
      // or particles if we had a ParticleComponent.
      
      // As a placeholder for "Physics", let's just verify we can iterate.
      // In a real engine we'd look for RigidBodyComponent.
      
      // Example: Iterate Transforms (just to show system logic)
      /* 
      auto& pool = universe.GetPool<Transform>();
      for(auto& t : pool.m_Data) {
          // Apply velocity if we had it
      }
      */
  }
  
  PhysicsException::PhysicsException(const std::string& message) : runtime_error(message) {}

}