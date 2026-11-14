// RED STUB of the tour's chapter 4 physics (see tests/test_sandbox_bodies.cpp): the interface, with nothing behind it.
// The GREEN commit fills it in.

#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include "compute/backend.hpp"
#include "core/error.hpp"
#include "physics/rigid_contact.hpp"
#include "sim/simulation.hpp"

namespace spade::sandbox::bodies {

enum class Kind { sphere, box, capsule, hull };

struct Piece {
    Kind kind = Kind::box;
    std::string name;
    glm::vec3 start{0.0f};
    glm::quat tilt{1.0f, 0.0f, 0.0f, 0.0f};
    glm::vec3 vel{0.0f};
    float mass = 1.0f;
    glm::vec3 inv_inertia{1.0f};
    physics::ShapeDesc shape;
    float bound = 0.0f;  // a sphere about the piece's centre that holds all of it, m
};

struct Verdict {
    bool ok = false;
    std::string detail;
};

[[nodiscard]] inline std::vector<Piece> row() { return {}; }

class BodiesSim {
  public:
    [[nodiscard]] static Result<BodiesSim> create(compute::BackendKind = compute::BackendKind::cpu) { return BodiesSim{}; }
    [[nodiscard]] Result<void> step(uint32_t) { return {}; }
    [[nodiscard]] Result<void> advance(float) { return {}; }
    [[nodiscard]] uint64_t steps() const noexcept { return 0; }
    [[nodiscard]] const std::vector<Piece>& pieces() const noexcept { return pieces_; }
    [[nodiscard]] BodyState state(std::size_t) const { return {}; }
    [[nodiscard]] Verdict verdict(std::size_t) const { return {}; }
    [[nodiscard]] std::vector<std::byte> bytes() const { return {}; }

  private:
    std::vector<Piece> pieces_;
};

}  // namespace spade::sandbox::bodies
