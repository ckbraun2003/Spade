#include <gtest/gtest.h>

#include <cinttypes>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <span>
#include <string>
#include <vector>

#include "compute/backend.hpp"
#include "compute/vulkan/context.hpp"
#include "compute/vulkan/probe_runner.hpp"

#include "denorm_probe.spv.gen.hpp"

// ===========================================================================
// A STANDING DEVICE RECORD (banded-parity plan T6). It began as M1 of the
// NVIDIA fp32-denormal measurement
// (docs/design/core/plans/2026-10-04-nvidia-denorm-measurement-plan.md), in
// the measurement build; it now runs in every build, where no kernel requests
// an fp32 denormal mode (P3), so its table is the device's own default, in
// every gate log. It asserts only its harness rules: every case explained by
// an answer, the control cases exact, no contraction. What the device does
// with subnormals is printed, never asserted: any device's default passes.
//
// For each fp32 op class the physics kernels use, the probe kernel
// (engine/shaders/kernels/denorm_probe.slang) runs every case of an operand
// table built around the subnormal boundary. The host computes four answers
// per case, in software:
//   preserved        IEEE 754 with subnormals kept: the CPU twin;
//   inputs flushed   every subnormal operand read as a zero of its sign;
//   outputs flushed  every subnormal result of every step written as a zero
//                    of its sign (the multiply-add's product included);
//   both             the two together;
// and, for the multiply-add, the FUSED answer std::fma gives. A case is
// classed by the first answer the device's bits match, and each class gets a
// verdict for its inputs and for its outputs.
//
// A MEASUREMENT THAT CAN STILL FAIL (agreed with Core). The table always
// prints, and a flush is a finding, not a failure. The test fails only when
// nothing explains what the device did, because then the harness or the probe
// is wrong:
//   (a) a case matches no answer ("other", printed with its bits);
//   (b) a CONTROL case differs on an exact class. A control case has normal,
//       non-zero operands and four answers that agree, so no flush can change
//       it. Only division and square root may differ there, as accuracy;
//   (c) a class ran no cases;
//   (d) a multiply-add matches the fused answer: the driver contracted a
//       NoContraction mul + add, a parity finding in its own right.
// ===========================================================================

namespace {

using spade::compute::BackendDesc;
using spade::compute::BackendKind;
using spade::compute::run_probe;
using spade::compute::VulkanContext;
using spade::compute::vulkan_available;

[[nodiscard]] float float_of(uint32_t u) {
    float x = 0.0f;
    std::memcpy(&x, &u, sizeof(x));
    return x;
}

[[nodiscard]] uint32_t bits_of(float x) {
    uint32_t u = 0;
    std::memcpy(&u, &x, sizeof(u));
    return u;
}

[[nodiscard]] bool subnormal(float x) { return std::fpclassify(x) == FP_SUBNORMAL; }

// A subnormal replaced by a zero of its sign; anything else unchanged.
[[nodiscard]] float flush(float x) { return subnormal(x) ? std::copysign(0.0f, x) : x; }

// The probe's modes, numbered as engine/shaders/kernels/denorm_probe.slang
// numbers them.
enum class Op : uint32_t {
    pass = 0,
    add = 1,
    sub = 2,
    mul = 3,
    abs = 4,
    neg = 5,
    min = 6,
    max = 7,
    mul_add = 8,
    floor = 9,
    floor_int = 10,
    compare = 11,
    clamp = 12,
    sqrt = 13,
    div = 14,
};

struct OpClass {
    Op op;
    const char* name;
    int arity;
    bool exact;  // false only where the device may differ in accuracy
};

constexpr OpClass kClasses[] = {
    {Op::pass, "pass-through", 1, true}, {Op::add, "add", 2, true},
    {Op::sub, "sub", 2, true},           {Op::mul, "mul", 2, true},
    {Op::abs, "abs", 1, true},           {Op::neg, "neg", 1, true},
    {Op::min, "min", 2, true},           {Op::max, "max", 2, true},
    {Op::mul_add, "mul-then-add", 3, true}, {Op::floor, "floor", 1, true},
    {Op::floor_int, "floor->int", 1, true}, {Op::compare, "compares", 2, true},
    {Op::clamp, "clamp", 3, true},       {Op::sqrt, "sqrt", 1, false},
    {Op::div, "div", 2, false},
};

// Every subnormal shape (the smallest, low and high mantissas, the largest),
// the normals on either side of the boundary, normals whose products,
// differences and quotients land subnormal, ordinary values and zero, each
// with both signs.
[[nodiscard]] std::vector<float> operand_table() {
    const uint32_t magnitudes[] = {
        0x00000001u,  // 2^-149, the smallest subnormal
        0x00000002u,
        0x00012345u,
        0x00400000u,  // 2^-127
        0x007FFFFFu,  // the largest subnormal
        0x00800000u,  // 2^-126, the smallest normal
        0x00C00000u,  // 1.5 * 2^-126: minus 2^-126 lands subnormal
        0x01000000u,  // 2^-125
        0x1C800000u,  // 2^-70: squared lands subnormal
        0x20000000u,  // 2^-63
        0x3F800000u,  // 1
        0x3F800800u,  // 1 + 2^-12: its square rounds, so fused and unfused differ
        0x40400000u,  // 3
        0x4B000000u,  // 2^23
        0x00000000u,  // zero
    };
    std::vector<float> out;
    for (const uint32_t m : magnitudes) {
        out.push_back(float_of(m));
        out.push_back(float_of(m | 0x80000000u));
    }
    return out;
}

// One op, IEEE 754 single precision. `flush_steps` flushes the result of
// every arithmetic step, which is what a flushing device does to the
// multiply-add's product as well as to its sum. min, max and clamp follow
// GLSL.std.450's FMin, FMax and FClamp exactly; the results of floor->int
// and the compares are small integers, returned as exact floats, as the
// probe returns them.
[[nodiscard]] float evaluate(Op op, float a, float b, float c, bool flush_steps) {
    const auto step = [flush_steps](float x) { return flush_steps ? flush(x) : x; };
    switch (op) {
        case Op::pass:
            return a;
        case Op::add:
            return step(a + b);
        case Op::sub:
            return step(a - b);
        case Op::mul:
            return step(a * b);
        case Op::abs:
            return std::fabs(a);
        case Op::neg:
            return -a;
        case Op::min:
            return b < a ? b : a;
        case Op::max:
            return a < b ? b : a;
        case Op::mul_add: {
            // volatile: the product is rounded on its own, never fused.
            volatile float product = step(a * b);
            return step(product + c);
        }
        case Op::floor:
            return std::floor(a);
        case Op::floor_int:
            return static_cast<float>(static_cast<int32_t>(std::floor(a)));
        case Op::compare: {
            uint32_t bits = 0;
            if (a < 0.0f) bits |= 1u;
            if (a > 0.0f) bits |= 2u;
            if (a == 0.0f) bits |= 4u;
            if (a <= 0.0f) bits |= 8u;
            if (a >= 0.0f) bits |= 16u;
            if (a < b) bits |= 32u;
            return static_cast<float>(bits);
        }
        case Op::clamp: {
            const float lifted = a < b ? b : a;
            return c < lifted ? c : lifted;
        }
        case Op::sqrt:
            return step(std::sqrt(a));
        case Op::div:
            return step(a / b);
    }
    return std::nanf("");
}

struct Answers {
    float preserved = 0.0f;
    float inputs_flushed = 0.0f;
    float outputs_flushed = 0.0f;
    float both = 0.0f;
    float fused = 0.0f;          // mul-then-add only
    float fused_flushed = 0.0f;  // mul-then-add only: fused, on a flushing device
};

[[nodiscard]] Answers answers(Op op, float a, float b, float c) {
    Answers r;
    r.preserved = evaluate(op, a, b, c, false);
    r.inputs_flushed = evaluate(op, flush(a), flush(b), flush(c), false);
    r.outputs_flushed = evaluate(op, a, b, c, true);
    r.both = evaluate(op, flush(a), flush(b), flush(c), true);
    if (op == Op::mul_add) {
        r.fused = std::fma(a, b, c);
        r.fused_flushed = flush(std::fma(flush(a), flush(b), flush(c)));
    }
    return r;
}

// Bit equality, with two allowances SPIR-V makes: any NaN matches any NaN,
// and for min, max and clamp, whose zero sign GLSL.std.450 leaves open, any
// zero matches any zero.
[[nodiscard]] bool same(Op op, float device, float host) {
    if (std::isnan(device) && std::isnan(host)) return true;
    const bool zero_sign_open = op == Op::min || op == Op::max || op == Op::clamp;
    if (zero_sign_open && device == 0.0f && host == 0.0f) return true;
    return bits_of(device) == bits_of(host);
}

// The distance in units in the last place, for the accuracy allowance.
[[nodiscard]] int64_t ulp_distance(float x, float y) {
    const auto ordered = [](float v) {
        const int64_t u = bits_of(v);
        return (u & 0x80000000) != 0 ? -(u & 0x7FFFFFFF) : u;
    };
    return std::llabs(ordered(x) - ordered(y));
}

constexpr int64_t kAccuracyUlps = 4;  // division and square root only
constexpr int kMaxPrinted = 12;       // "other" and control cases printed per class

struct Tally {
    int cases = 0;
    int control = 0;
    int kept = 0;
    int in_flushed = 0;
    int out_flushed = 0;
    int either_flushed = 0;  // inputs and outputs flushed give the same bits
    int both_flushed = 0;
    int contracted = 0;
    int accuracy = 0;
    int other = 0;
    int control_differs = 0;
    // Evidence per side. A case is evidence that a side was kept only when
    // the device matches an answer that keeps it and no answer that flushes
    // it, and the reverse for lost: a case several answers explain decides
    // nothing.
    int inputs_kept = 0;
    int inputs_lost = 0;
    int outputs_kept = 0;
    int outputs_lost = 0;
};

[[nodiscard]] const char* verdict(int kept, int lost) {
    if (kept > 0 && lost > 0) return "MIXED";
    if (lost > 0) return "FLUSHED";
    if (kept > 0) return "preserved";
    return "not distinguished";
}

[[nodiscard]] std::span<const std::byte> probe_spirv() {
    return std::as_bytes(std::span<const uint32_t>(spade::compute::gen::kSpv_denorm_probe,
                                                   spade::compute::gen::kSpvWordCount_denorm_probe));
}

void print_device(VulkanContext& ctx) {
    VkPhysicalDeviceFloatControlsProperties controls{};
    controls.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_FLOAT_CONTROLS_PROPERTIES;
    VkPhysicalDeviceDriverProperties driver{};
    driver.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_DRIVER_PROPERTIES;
    driver.pNext = &controls;
    VkPhysicalDeviceProperties2 properties{};
    properties.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_PROPERTIES_2;
    properties.pNext = &driver;
    vkGetPhysicalDeviceProperties2(ctx.physical_device(), &properties);
    std::printf("[GpuDenormProbe] device=%s  driver=%s (%s)  driverVersion=0x%08X\n",
                properties.properties.deviceName, driver.driverName, driver.driverInfo,
                properties.properties.driverVersion);
    std::printf("[GpuDenormProbe] float controls: denormPreserveF32=%u denormFlushToZeroF32=%u "
                "denormBehaviorIndependence=%u; kernels compiled with no fp32 denormal mode\n",
                static_cast<unsigned>(controls.shaderDenormPreserveFloat32),
                static_cast<unsigned>(controls.shaderDenormFlushToZeroFloat32),
                static_cast<unsigned>(controls.denormBehaviorIndependence));
}

}  // namespace

TEST(GpuDenormProbe, EveryFp32OpClassIsExplainedByAFlushVariant) {
    // The host twin is only a twin if the host keeps subnormals itself.
    volatile float smallest = float_of(0x00000001u);
    volatile float one = 1.0f;
    ASSERT_EQ(bits_of(smallest * one), 0x00000001u) << "the host flushes subnormal results";
    volatile float half_min = float_of(0x00400000u);
    ASSERT_EQ(bits_of(half_min + half_min), 0x00800000u) << "the host flushes subnormal inputs";

    if (!vulkan_available()) GTEST_SKIP();
    BackendDesc desc{.kind = BackendKind::vulkan};
    auto created = VulkanContext::create(desc);
    ASSERT_TRUE(created.has_value()) << created.error().context;
    VulkanContext& ctx = **created;
    print_device(ctx);

    const std::vector<float> table = operand_table();
    std::printf("[GpuDenormProbe] %-13s %6s %7s %6s %6s %7s %6s %5s %10s %8s %5s | %-17s %-17s\n",
                "class", "cases", "control", "kept", "in-ftz", "out-ftz", "either", "both",
                "contracted", "accuracy", "other", "inputs", "outputs");

    for (const OpClass& cls : kClasses) {
        // The cases: every operand for a unary op, every pair for a binary
        // one, every triple for a ternary one (clamp's bounds ordered).
        std::vector<float> args = {float_of(static_cast<uint32_t>(cls.op))};
        const std::size_t n = table.size();
        const std::size_t span_b = cls.arity >= 2 ? n : 1;
        const std::size_t span_c = cls.arity >= 3 ? n : 1;
        for (std::size_t i = 0; i < n; ++i) {
            for (std::size_t j = 0; j < span_b; ++j) {
                for (std::size_t k = 0; k < span_c; ++k) {
                    const float a = table[i];
                    const float b = cls.arity >= 2 ? table[j] : 0.0f;
                    const float c = cls.arity >= 3 ? table[k] : 0.0f;
                    if (cls.op == Op::clamp && c < b) continue;
                    args.push_back(a);
                    args.push_back(b);
                    args.push_back(c);
                }
            }
        }
        const std::size_t cases = (args.size() - 1) / 3;
        std::vector<float> out(cases);
        if (cases > 0) {
            const auto ran = run_probe(ctx, probe_spirv(), args, out);
            ASSERT_TRUE(ran.has_value()) << cls.name << ": " << ran.error().context;
        }

        Tally t;
        int printed = 0;
        for (std::size_t i = 0; i < cases; ++i) {
            const float a = args[1 + 3 * i];
            const float b = args[2 + 3 * i];
            const float c = args[3 + 3 * i];
            const float device = out[i];
            const Answers r = answers(cls.op, a, b, c);
            ++t.cases;

            const bool operands_normal = std::fpclassify(a) == FP_NORMAL &&
                                         (cls.arity < 2 || std::fpclassify(b) == FP_NORMAL) &&
                                         (cls.arity < 3 || std::fpclassify(c) == FP_NORMAL);
            const bool answers_agree = same(cls.op, r.inputs_flushed, r.preserved) &&
                                       same(cls.op, r.outputs_flushed, r.preserved) &&
                                       same(cls.op, r.both, r.preserved);
            const bool control = operands_normal && answers_agree;
            if (control) ++t.control;

            const bool kept = same(cls.op, device, r.preserved);
            const bool in = same(cls.op, device, r.inputs_flushed);
            const bool outf = same(cls.op, device, r.outputs_flushed);
            const bool both = same(cls.op, device, r.both);

            const bool inputs_kept_explains = kept || outf;
            const bool inputs_lost_explains = in || both;
            const bool outputs_kept_explains = kept || in;
            const bool outputs_lost_explains = outf || both;
            if (inputs_kept_explains && !inputs_lost_explains) ++t.inputs_kept;
            if (inputs_lost_explains && !inputs_kept_explains) ++t.inputs_lost;
            if (outputs_kept_explains && !outputs_lost_explains) ++t.outputs_kept;
            if (outputs_lost_explains && !outputs_kept_explains) ++t.outputs_lost;

            const char* what = nullptr;
            if (kept) {
                ++t.kept;
            } else if (in && outf) {
                ++t.either_flushed;
            } else if (in) {
                ++t.in_flushed;
            } else if (outf) {
                ++t.out_flushed;
            } else if (both) {
                ++t.both_flushed;
            } else if (cls.op == Op::mul_add && (same(cls.op, device, r.fused) ||
                                                  same(cls.op, device, r.fused_flushed))) {
                ++t.contracted;
                what = "CONTRACTED (matches std::fma)";
            } else if (!cls.exact && !std::isnan(device) &&
                       (ulp_distance(device, r.preserved) <= kAccuracyUlps ||
                        ulp_distance(device, r.inputs_flushed) <= kAccuracyUlps ||
                        ulp_distance(device, r.outputs_flushed) <= kAccuracyUlps ||
                        ulp_distance(device, r.both) <= kAccuracyUlps)) {
                ++t.accuracy;
                what = control ? "ACCURACY on normal operands" : "ACCURACY";
            } else {
                ++t.other;
                what = "OTHER (matches no answer)";
            }
            if (control && !kept && cls.exact) {
                ++t.control_differs;
                if (what == nullptr) what = "CONTROL CASE DIFFERS";
            }
            if (what != nullptr && printed < kMaxPrinted) {
                ++printed;
                std::printf("[GpuDenormProbe]   %s %s: a=0x%08" PRIX32 " b=0x%08" PRIX32 " c=0x%08" PRIX32
                            "  device=0x%08" PRIX32 "  preserved=0x%08" PRIX32 " in=0x%08" PRIX32
                            " out=0x%08" PRIX32 " both=0x%08" PRIX32 "\n",
                            cls.name, what, bits_of(a), bits_of(b), bits_of(c), bits_of(device),
                            bits_of(r.preserved), bits_of(r.inputs_flushed), bits_of(r.outputs_flushed),
                            bits_of(r.both));
            }
        }

        std::printf("[GpuDenormProbe] %-13s %6d %7d %6d %6d %7d %6d %5d %10d %8d %5d | %-17s %-17s\n",
                    cls.name, t.cases, t.control, t.kept, t.in_flushed, t.out_flushed, t.either_flushed,
                    t.both_flushed, t.contracted, t.accuracy, t.other,
                    verdict(t.inputs_kept, t.inputs_lost), verdict(t.outputs_kept, t.outputs_lost));
        std::fflush(stdout);

        EXPECT_GT(t.cases, 0) << cls.name << ": no cases ran (c)";
        EXPECT_EQ(t.other, 0) << cls.name << ": cases no answer explains (a)";
        EXPECT_EQ(t.control_differs, 0) << cls.name << ": control cases differ on an exact class (b)";
        EXPECT_EQ(t.contracted, 0) << cls.name << ": the driver contracted a NoContraction mul + add (d)";
    }
}

// ===========================================================================
// THE EDGE AT 2^-126 (the FTZ plan, docs/design/physics/plans/
// 2026-10-22-airflow-ftz-plan.md, Task 4; decision D8). A multiply is the one
// op that can round across the smallest normal: a quotient or square root
// never does, and a sum below 2^-126 is exact. Three flush rules part there:
//   before rounding    the exact product below 2^-126 flushes;
//   after rounding     the product rounded to 24 bits with an unbounded
//                      exponent flushes if that is below 2^-126 (x86 MXCSR's
//                      FTZ: the CPU twin's scope);
//   flush of gradual   the IEEE (gradual-underflow) result flushes if it is
//                      subnormal.
// "upper" (2^-126 - 2^-172) separates before rounding from the other two;
// "lower" (just past half an ulp of the 2^-127 binade below 2^-126)
// separates after rounding from flush of gradual. Each row runs with every
// sign pair through the probe's mul mode. The device must give what the CPU
// twin's scope gives: the after-rounding answer, the bits the box's CPU gave
// under FTZ+DAZ (measured 2026-10-22, the plan's facts F4; AirflowFtz pins
// the host's own bits inside the scope on the same rows). Every answer is
// computed here from the exact product, which a double holds (24 x 24 bits),
// scaled by powers of two only: no libm. A device that fits another rule
// fails, naming it (D8: the divergence goes to the lead). Measured 2026-10-22
// on the RTX 3060 Ti, driver 572.83: after rounding, every row and sign.
// Mutation: the expected bits the host's IEEE product (no flush): the lower,
// daz and 2^-70 rows go red.
// ===========================================================================
namespace {

struct EdgeAnswers {
    float gradual = 0.0f;           // IEEE 754, subnormals kept, operands included
    float before = 0.0f;            // tininess before rounding
    float after = 0.0f;             // tininess after rounding: the CPU scope
    float flush_of_gradual = 0.0f;  // the gradual result, flushed
};

// The three flush rules read a subnormal operand as a zero of its sign (the
// standing table's finding: the device flushes mul's inputs); the gradual
// answer keeps it, as the IEEE host does.
[[nodiscard]] EdgeAnswers edge_answers(float a, float b) {
    const double exact = static_cast<double>(flush(a)) * static_cast<double>(flush(b));  // exact: 48 bits
    const float signed_zero = (std::signbit(a) != std::signbit(b)) ? -0.0f : 0.0f;
    EdgeAnswers r;
    r.gradual = static_cast<float>(static_cast<double>(a) * static_cast<double>(b));  // one rounding of an exact value
    const float flushed_gradual = static_cast<float>(exact);
    const bool tiny_exact = std::fabs(exact) < 0x1p-126;
    r.before = tiny_exact ? signed_zero : static_cast<float>(exact);
    // 24 bits with an unbounded exponent: scaled into float's normal range,
    // rounded there by the conversion, scaled back exactly.
    const double rounded = static_cast<double>(static_cast<float>(exact * 0x1p64)) * 0x1p-64;
    r.after = std::fabs(rounded) < 0x1p-126 ? signed_zero : static_cast<float>(rounded);
    r.flush_of_gradual = subnormal(flushed_gradual) ? signed_zero : flushed_gradual;
    return r;
}

}  // namespace

TEST(GpuDenormProbe, AProductJustBelowTheSmallestNormalFlushesAsTheCpuScopeDoes) {
    struct Row {
        const char* name;
        uint32_t a;
        uint32_t b;
    };
    const Row rows[] = {
        {"upper", 0x1FFFFFFEu, 0x20000001u},          // 2^-126 - 2^-172
        {"lower", 0x1FFFF448u, 0x200005DCu},          // in [2^-126 - 2^-150, 2^-126 - 2^-151)
        {"exact", 0x20000000u, 0x20000000u},          // 2^-126
        {"daz", 0x00400000u, 0x3F800000u},            // 2^-127 x 1: a subnormal operand
        {"2^-70 squared", 0x1C800000u, 0x1C800000u},  // 2^-140
    };
    // The host is IEEE here (the standing test asserts it), so its own
    // product is the gradual answer: a check on the model.
    for (const Row& row : rows) {
        volatile float a = float_of(row.a);
        volatile float b = float_of(row.b);
        ASSERT_EQ(bits_of(a * b), bits_of(edge_answers(float_of(row.a), float_of(row.b)).gradual))
            << row.name << ": the model's gradual answer is not the host's IEEE product";
    }

    if (!vulkan_available()) GTEST_SKIP() << "no Vulkan device: the edge is a device measurement";
    BackendDesc desc{.kind = BackendKind::vulkan};
    auto created = VulkanContext::create(desc);
    ASSERT_TRUE(created.has_value()) << created.error().context;
    VulkanContext& ctx = **created;
    print_device(ctx);

    std::vector<float> args = {float_of(static_cast<uint32_t>(Op::mul))};
    struct Case {
        const char* name;
        float a;
        float b;
    };
    std::vector<Case> cases;
    for (const Row& row : rows) {
        for (const uint32_t sa : {0u, 0x80000000u}) {
            for (const uint32_t sb : {0u, 0x80000000u}) {
                cases.push_back({row.name, float_of(row.a | sa), float_of(row.b | sb)});
                args.push_back(cases.back().a);
                args.push_back(cases.back().b);
                args.push_back(0.0f);
            }
        }
    }
    std::vector<float> out(cases.size());
    const auto ran = run_probe(ctx, probe_spirv(), args, out);
    ASSERT_TRUE(ran.has_value()) << ran.error().context;

    int discriminating = 0;
    for (std::size_t i = 0; i < cases.size(); ++i) {
        const Case& c = cases[i];
        const EdgeAnswers r = edge_answers(c.a, c.b);
        const uint32_t device = bits_of(out[i]);
        std::string fits;
        const auto fit = [&](const char* rule, float answer) {
            if (device == bits_of(answer)) fits += fits.empty() ? rule : std::string(", ") + rule;
        };
        fit("before rounding", r.before);
        fit("after rounding", r.after);
        fit("flush of gradual", r.flush_of_gradual);
        fit("gradual (no flush)", r.gradual);
        if (fits.empty()) fits = "NONE";
        if (bits_of(r.after) != bits_of(r.before) || bits_of(r.after) != bits_of(r.flush_of_gradual)) ++discriminating;
        std::printf("[GpuDenormProbe] edge %-14s a=0x%08" PRIX32 " b=0x%08" PRIX32 "  device=0x%08" PRIX32
                    "  before=0x%08" PRIX32 " after=0x%08" PRIX32 " flush-of-gradual=0x%08" PRIX32
                    " gradual=0x%08" PRIX32 "  fits: %s\n",
                    c.name, bits_of(c.a), bits_of(c.b), device, bits_of(r.before), bits_of(r.after),
                    bits_of(r.flush_of_gradual), bits_of(r.gradual), fits.c_str());
        EXPECT_EQ(device, bits_of(r.after)) << c.name << ": the device does not flush as the CPU scope does (after "
                                                         "rounding); it fits: "
                                            << fits;
    }
    std::fflush(stdout);
    EXPECT_GT(discriminating, 0) << "no row separates the rules";
}
