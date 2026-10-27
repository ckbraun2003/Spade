// ---------------------------------------------------------------------------
// airflow_pcg_scalars.slang (plan B, tasks B5.1b and B5.4) -- THE CG'S
// QUOTIENTS AND DECISIONS on the GPU, from airflow_scalars to airflow_scalars
// (slots: airflow_pcg.slang), one thread, each the expression or branch of
// pcg.cpp's solve_pcg_to(), its CPU twin and the authority:
//   op 0  alpha    = rz / dq                                   (pcg.cpp:118)
//   op 1  beta     = rz_next / rz                              (pcg.cpp:126)
//   op 2  residual = b_max > 0 ? r_max / b_max : 0             (pcg.cpp:57)
// and B5.4's, which run the solve's branches in pcg.cpp's order (the plan's
// B5, "the device decides"; the host only reads the decision word):
//   op 3  start    the solve's scalars at their start (:73-80): not stopped,
//                  0 iterations, least 0, best +inf, nothing measured
//   op 4  step     the exact-zero stop, then alpha (:112-118): an exact zero
//                  in dq or rz stops the solve, exact; else measured is
//                  cleared and alpha = rz / dq
//   op 5  next     beta (:126), rz <- rz_next (:128), the iteration counted
//                  (the loop's ++, :81)
//   op 6  gate     the measure's gauge on a solve already stopped and
//                  measured: each closed region's mean set to +0.0 between
//                  airflow_region_mean's means and its subtract, so x - 0.0
//                  leaves x as it is (the CPU does not measure again)
//   op 7  decide   the measure past the floor and its decisions (:83-109):
//                  the residual (op 2's), measured; the mark; least and
//                  since; the cap; the stall and its one restart; best <- x
//                  (keep_best, for airflow_pcg_update). On a solve an exact
//                  zero stopped before the floor's measure, it is the
//                  finish's measure (:130) and decides nothing.
//   op 8  finish   the stop (:131-137): residual_at; an exact zero above the
//                  mark is stalled; above the mark with a better x measured,
//                  that x (take_best, for airflow_pcg_update), its residual
//                  and its iteration; capped = above the mark, not stalled
//                  (fluid_step.cpp's health).
// Every op from 4 to 7 returns at once while the stop word is set, except
// that op 7 takes the finish's measure; op 6 acts only then.
//
// EVERY DIVISION IS fp32_math's div32_ftz: divss's bits under the CPU step's
// FTZ|DAZ, NaN and infinity included, where the device's own OpFDiv is not
// correctly rounded (the 3060 Ti gives 0x3EAAB556 for 0x3F800800 / 3.0, the
// host 0x3EAAB555). The compares are exact, and the one multiply, kStallGain
// x least (:86), is a correctly rounded OpFMul. So the quotients and every
// decision are the CPU's from the CPU's bits, and the module is scanned under
// the exact profile.
//
// THE CONSTANTS are the pcg block's (airflow_params_layout.slang, word 28):
// the mark (kResidualAnnounce), kStallGain, kStallIterations, the floor and
// the cap; the closed-region count for op 6.
//
// SET 1, bindings.slang section F: reads and writes airflow_scalars (21);
// op 6 writes airflow_region_table's means (19); reads airflow_params only
// through the module imports.
//
// A MISUSE SHOWS AS NaN: an op past 8 writes a NaN residual, and so does a
// finish with nothing measured.
// ---------------------------------------------------------------------------

import bindings;
import airflow_params_layout;
import airflow_pcg;
import fp32_math;

// The airflow push convention: 16 bytes, the count in word 2.
struct PcgScalarsPush {
    uint op;  // 0 alpha ... 8 finish
    uint unused0;
    uint count;  // 1: one thread
    uint unused1;
};

[[vk::push_constant]] ConstantBuffer<PcgScalarsPush> scalars_push;

static const uint kOpAlpha = 0u;
static const uint kOpBeta = 1u;
static const uint kOpResidual = 2u;
static const uint kOpStart = 3u;
static const uint kOpStep = 4u;
static const uint kOpNext = 5u;
static const uint kOpGate = 6u;
static const uint kOpDecide = 7u;
static const uint kOpFinish = 8u;

float slot(uint s) { return airflow_scalars[s]; }

void set_slot(uint s, float v) { airflow_scalars[s] = v; }

void set_flag(uint s, bool on) { airflow_scalars[s] = on ? 1.0f : 0.0f; }

// A count slot's value: a whole float, exact below 2^24.
uint count_of(uint s) { return uint(airflow_scalars[s]); }

// pcg.cpp:57.
float residual_quotient() {
    const float r_max = slot(kSlotRMax);
    const float b_max = slot(kSlotBMax);
    return b_max > 0.0f ? div32_ftz(r_max, b_max) : 0.0f;
}

void start() {
    set_flag(kSlotStop, false);
    set_flag(kSlotRestart, false);
    set_slot(kSlotIterations, 0.0f);
    set_flag(kSlotStalled, false);
    set_slot(kSlotResidual, 0.0f);
    set_slot(kSlotResidualAt, 0.0f);
    set_slot(kSlotLeast, 0.0f);
    set_slot(kSlotSince, 0.0f);
    set_flag(kSlotRestarted, false);
    set_slot(kSlotBest, asfloat(0x7F800000u));  // +inf
    set_slot(kSlotBestAt, 0.0f);
    set_flag(kSlotMeasured, false);
    set_flag(kSlotExact, false);
    set_flag(kSlotKeepBest, false);
    set_flag(kSlotTakeBest, false);
    set_flag(kSlotCapped, false);
}

void step() {
    if (pcg_stopped()) { return; }
    const float rz = slot(kSlotRz);
    const float dq = slot(kSlotDq);
    if (dq == 0.0f || rz == 0.0f) {  // nothing left to reduce: solved, or (above the mark) stalled
        set_flag(kSlotExact, true);
        set_flag(kSlotStop, true);
        return;
    }
    set_flag(kSlotMeasured, false);
    set_slot(kSlotAlpha, div32_ftz(rz, dq));
}

void next() {
    if (pcg_stopped()) { return; }
    const float rz_next = slot(kSlotRzNext);
    set_slot(kSlotBeta, div32_ftz(rz_next, slot(kSlotRz)));
    set_slot(kSlotRz, rz_next);
    set_slot(kSlotIterations, float(count_of(kSlotIterations) + 1u));
}

void gate() {
    if (!pcg_stopped() || !slot_set(kSlotMeasured)) { return; }
    const uint closed = pcg_closed_regions();
    for (uint j = 0u; j < closed; ++j) { set_region_mean(j, 0.0f); }
}

void decide() {
    set_flag(kSlotRestart, false);
    set_flag(kSlotKeepBest, false);
    if (pcg_stopped()) {
        // An exact zero stopped the iterations. Inside the floor nothing was
        // measured: this is the finish's measure (:130). Past it, the
        // iteration's own measure stands.
        if (!slot_set(kSlotMeasured)) {
            set_slot(kSlotResidual, residual_quotient());
            set_flag(kSlotMeasured, true);
        }
        return;
    }
    const float residual = residual_quotient();
    set_slot(kSlotResidual, residual);
    set_flag(kSlotMeasured, true);
    if (residual <= pcg_residual_announce()) {
        set_flag(kSlotStop, true);
        return;
    }
    const uint iterations = count_of(kSlotIterations);
    const uint floor = pcg_floor();
    if (iterations == floor || residual < pcg_stall_gain() * slot(kSlotLeast)) {
        set_slot(kSlotLeast, residual);
        set_slot(kSlotSince, 0.0f);
    } else {
        set_slot(kSlotSince, float(count_of(kSlotSince) + 1u));
    }
    const uint last = floor > pcg_cap() ? floor : pcg_cap();
    if (iterations >= last) {  // the cap: announced as capped, not stalled
        set_flag(kSlotStop, true);
        return;
    }
    const uint stall = pcg_stall_iterations();
    if (count_of(kSlotSince) >= stall && slot_set(kSlotRestarted)) {
        set_flag(kSlotStalled, true);
        set_flag(kSlotStop, true);
        return;
    }
    if (residual < slot(kSlotBest)) {
        set_slot(kSlotBest, residual);
        set_slot(kSlotBestAt, float(iterations));
        set_flag(kSlotKeepBest, true);
    }
    if (count_of(kSlotSince) >= stall) {  // the first stall: start again once from the true residual
        set_flag(kSlotRestarted, true);
        set_slot(kSlotSince, 0.0f);
        set_flag(kSlotRestart, true);
    }
}

void finish() {
    if (!slot_set(kSlotMeasured)) {  // the measure chain always measures at a stop
        set_slot(kSlotResidual, asfloat(0x7FC00000u));
        return;
    }
    const float mark = pcg_residual_announce();
    const float residual = slot(kSlotResidual);
    set_slot(kSlotResidualAt, slot(kSlotIterations));
    if (slot_set(kSlotExact) && residual > mark) { set_flag(kSlotStalled, true); }
    set_flag(kSlotTakeBest, false);
    const float best = slot(kSlotBest);
    if (residual > mark && best < residual) {  // stopped above the mark: return the least measured
        set_flag(kSlotTakeBest, true);
        set_slot(kSlotResidual, best);
        set_slot(kSlotResidualAt, slot(kSlotBestAt));
    }
    set_flag(kSlotCapped, slot(kSlotResidual) > mark && !slot_set(kSlotStalled));
}

[shader("compute")]
[numthreads(SPADE_WG, 1, 1)]
void airflow_pcg_scalars(uint3 tid : SV_DispatchThreadID) {
    if (tid.x != 0u || tid.x >= scalars_push.count) { return; }
    const uint op = scalars_push.op;
    if (op == kOpAlpha) {
        airflow_scalars[kSlotAlpha] = div32_ftz(airflow_scalars[kSlotRz], airflow_scalars[kSlotDq]);
        return;
    }
    if (op == kOpBeta) {
        airflow_scalars[kSlotBeta] = div32_ftz(airflow_scalars[kSlotRzNext], airflow_scalars[kSlotRz]);
        return;
    }
    if (op == kOpResidual) {
        airflow_scalars[kSlotResidual] = residual_quotient();
        return;
    }
    if (op == kOpStart) {
        start();
        return;
    }
    if (op == kOpStep) {
        step();
        return;
    }
    if (op == kOpNext) {
        next();
        return;
    }
    if (op == kOpGate) {
        gate();
        return;
    }
    if (op == kOpDecide) {
        decide();
        return;
    }
    if (op == kOpFinish) {
        finish();
        return;
    }
    airflow_scalars[kSlotResidual] = asfloat(0x7FC00000u);
}
