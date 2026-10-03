#pragma once

// physics/sampled_medium.hpp -- the medium as a world's stored field samples
// (module-API plan, stage 3). The density and wind providers wrote this
// substep's values into the world's sample row in the Fields phase; this view
// hands them to a reader that takes a Medium, so the model functions
// (apply_rotors, apply_drag) are unchanged.
//
// sample() is a copy, not arithmetic: the stored values are the same operations
// on the same inputs as DrydenMedium::sample(), so a reader sees the same bits.
// Every provider today is position-independent, so `pos` is not consulted. A
// position-dependent provider (SPH) needs point-indexed sampling, not this.

#include <span>

#include <glm/vec3.hpp>

#include "physics/field_row.hpp"
#include "state/layout.hpp"
#include "world/medium.hpp"

namespace spade::physics {

class SampledMedium final : public Medium {
public:
    explicit SampledMedium(std::span<const float> row) noexcept : row_(row) {}

    [[nodiscard]] MediumSample sample(const WorldParams&, glm::vec3) const override {
        return MediumSample{row_[kFieldDensityOffset],
                            glm::vec3(row_[kFieldWindOffset + 0], row_[kFieldWindOffset + 1], row_[kFieldWindOffset + 2])};
    }

private:
    std::span<const float> row_;
};

}  // namespace spade::physics
