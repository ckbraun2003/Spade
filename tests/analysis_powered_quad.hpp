// The powered quad the LiveRun tests fly: the reference quad compiled from
// its airframe, without its parts (a test grid is far too coarse for them),
// with its own propulsion chain on the model (INT-11). It is the chain the
// airframe compile fitted the rotors with, so the throttle map and the rotors
// agree. Its cases carry its power: a flight controller's 5 % idle and the
// airframe's state of charge.

#pragma once

#include <gtest/gtest.h>

#include "analysis/case.hpp"
#include "analysis/live_run.hpp"
#include "reference_airframe.hpp"
#include "vehicles/airframe_compile.hpp"
#include "vehicles/model_type.hpp"
#include "vehicles/propulsion_steady.hpp"

namespace spade::analysis::test {

struct PoweredQuad {
    vehicles::ModelType model;  // with its chain (ModelType::propulsion), without its parts
    Power power;                // what its cases carry
};

[[nodiscard]] inline PoweredQuad powered_quad() {
    const vehicles::AirframeSpec spec = vehicles::fixtures::reference_quad();
    const auto compiled = vehicles::compile_airframe(spec);
    EXPECT_TRUE(compiled.has_value()) << (compiled ? "" : compiled.error().context);
    const auto chain = vehicles::airframe_propulsion_chain(spec);
    EXPECT_TRUE(chain.has_value()) << (chain ? "" : chain.error().context);
    PoweredQuad p;
    if (compiled) p.model = compiled->model;
    p.model.parts.clear();
    if (chain) p.model.propulsion = *chain;
    p.power = Power{.idle = 0.05f, .state_of_charge = static_cast<float>(spec.state_of_charge)};
    return p;
}

// `c` carrying the powered quad's power.
[[nodiscard]] inline AnalysisCase with_power(AnalysisCase c, const PoweredQuad& p) {
    c.power = p.power;
    return c;
}

// The speed the chain commands for an armed throttle, as LiveRun must.
[[nodiscard]] inline float commanded_omega(const PoweredQuad& p, float throttle, double density) {
    const auto duty = vehicles::armed_duty(true, throttle, static_cast<double>(p.power.idle.value_or(0.0f)));
    EXPECT_TRUE(duty.has_value());
    EXPECT_TRUE(p.model.propulsion.has_value());
    if (!p.model.propulsion) return 0.0f;
    const auto cmd = vehicles::rotor_omega_for_duty(*p.model.propulsion, duty.value_or(0.0), density,
                                                    static_cast<double>(p.power.state_of_charge.value_or(1.0f)));
    EXPECT_TRUE(cmd.has_value()) << (cmd ? "" : cmd.error().context);
    return cmd ? cmd->omega_cmd : 0.0f;
}

}  // namespace spade::analysis::test
