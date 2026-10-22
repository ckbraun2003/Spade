// The hover a model's propulsion chain holds it at (INT-11): what resolve()'s
// idle check and showcase_trim() both compare against, in LiveRun's world
// (y-up, under Environment{}'s gravity). Shared inside spade::analysis.

#pragma once

#include <string>

#include "core/error.hpp"
#include "vehicles/model_type.hpp"
#include "vehicles/propulsion_steady.hpp"

namespace spade::analysis::detail {

// The model's weight in LiveRun's world, N.
[[nodiscard]] double world_weight(const vehicles::ModelType& model);

// "density 1.225 kg/m3, state of charge 1", for a refusal.
[[nodiscard]] std::string air_text(double density, double soc);

// The steady point at which each of the model's rotors carries its share of
// the weight on `chain`; refused, naming the model and the air, when the
// chain can't reach it. The model has rotors.
[[nodiscard]] Result<vehicles::SteadyPoint> hover_point(const vehicles::ModelType& model,
                                                        const vehicles::PropulsionChain& chain, double density,
                                                        double soc);

}  // namespace spade::analysis::detail
