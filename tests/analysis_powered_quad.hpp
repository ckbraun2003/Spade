// The powered quad the LiveRun tests fly: the reference quad compiled from
// its airframe, without its parts (the solids join with P2, and a test grid
// is far too coarse for them), with its own propulsion chain. It is the chain
// the airframe compile fitted the rotors with, so the throttle map and the
// rotors agree. Idle is a flight controller's 5 %.

#pragma once

#include <gtest/gtest.h>

#include "analysis/live_run.hpp"
#include "reference_airframe.hpp"
#include "vehicles/airframe_compile.hpp"
#include "vehicles/model_type.hpp"
#include "vehicles/propulsion_steady.hpp"

namespace spade::analysis::test {

struct PoweredQuad {
    vehicles::ModelType model;
    LivePropulsion propulsion;
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
    if (chain) p.propulsion.chain = *chain;
    p.propulsion.idle = 0.05;
    p.propulsion.soc = spec.state_of_charge;
    return p;
}

// The speed the chain commands for an armed throttle, as LiveRun must.
[[nodiscard]] inline float commanded_omega(const PoweredQuad& p, float throttle, double density) {
    const auto duty = vehicles::armed_duty(true, throttle, p.propulsion.idle);
    EXPECT_TRUE(duty.has_value());
    const auto cmd = vehicles::rotor_omega_for_duty(p.propulsion.chain, duty.value_or(0.0), density, p.propulsion.soc);
    EXPECT_TRUE(cmd.has_value()) << (cmd ? "" : cmd.error().context);
    return cmd ? cmd->omega_cmd : 0.0f;
}

}  // namespace spade::analysis::test
