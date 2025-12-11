#pragma once

#include "Spade/Core/Objects.hpp"

#include <string>
#include <vector>
#include <stdexcept>

#include <glad/glad.h>

#include <glm/glm.hpp>
#include <cmath>

namespace Spade {

    struct Ray {
        glm::vec3 origin;
        glm::vec3 direction;
    };

    struct BoundingSphere {
        glm::vec3 center;
        float radius;
    };

    struct BoundingBox {
        glm::vec3 min;
        glm::vec3 max;
    };

    struct Plane {
        glm::vec3 point;
        glm::vec3 normal;
    };

    // Intersection Helpers
    inline bool RayIntersectSphere(const Ray& ray, const BoundingSphere& sphere, float& t) {
        glm::vec3 oc = ray.origin - sphere.center;
        float a = glm::dot(ray.direction, ray.direction);
        float b = 2.0f * glm::dot(oc, ray.direction);
        float c = glm::dot(oc, oc) - sphere.radius * sphere.radius;
        float discriminant = b * b - 4 * a * c;

        if (discriminant < 0) {
            return false;
        } else {
            t = (-b - sqrt(discriminant)) / (2.0f * a);
            return t > 0;
        }
    }

    inline bool RayIntersectBox(const Ray& ray, const BoundingBox& box, float& t) {
        float tmin = (box.min.x - ray.origin.x) / ray.direction.x;
        float tmax = (box.max.x - ray.origin.x) / ray.direction.x;

        if (tmin > tmax) std::swap(tmin, tmax);

        float tymin = (box.min.y - ray.origin.y) / ray.direction.y;
        float tymax = (box.max.y - ray.origin.y) / ray.direction.y;

        if (tymin > tymax) std::swap(tymin, tymax);

        if ((tmin > tymax) || (tymin > tmax)) return false;

        if (tymin > tmin) tmin = tymin;
        if (tymax < tmax) tmax = tymax;

        float tzmin = (box.min.z - ray.origin.z) / ray.direction.z;
        float tzmax = (box.max.z - ray.origin.z) / ray.direction.z;

        if (tzmin > tzmax) std::swap(tzmin, tzmax);

        if ((tmin > tzmax) || (tzmin > tmax)) return false;

        if (tzmin > tmin) tmin = tzmin;
        if (tzmax < tmax) tmax = tzmax;

        t = tmin;
        return t > 0;
    }

    inline bool RayIntersectPlane(const Ray& ray, const Plane& plane, float& t) {
        float denom = glm::dot(plane.normal, ray.direction);
        if (abs(denom) > 1e-6) {
            glm::vec3 p0l0 = plane.point - ray.origin;
            t = glm::dot(p0l0, plane.normal) / denom;
            return (t >= 0);
        }
        return false;
    }

  class Physics
  {
  public:

    Physics();

    void Update(Universe& universe, float deltaTime);

  };

  class PhysicsException : public std::runtime_error
  {
  public:

    PhysicsException(const std::string& message);

  };

}