// SPIR-V rules E1/E2 as the user restated them on 2026-10-22
// (docs/design/test-docs/00-decisions.md; 01-verification, "SPIR-V scanner").
// An fp32_math module may hold an OpFDiv or a GLSL.std.450 InverseSqrt only as
// the estimate inside an exact operation, whose integer correction is proven
// for any estimate within the Vulkan bound. The scan's half is reach: an
// estimate may pass through FMul/FAdd/FSub/FNegate, Floor/Trunc/Round and
// OpCopyObject, and may leave only through a conversion to an integer. Any
// other use fails, and a bare Sqrt fails anywhere. The correction's exactness
// is the other half, Core's injected-error battery, not tested here.
//
// The modules are synthetic and built word by word, because the scanner reads
// a word stream. They hold what the rule looks at and no more: no capability,
// no entry point, and ids used where the scanner reads uses, not types. So they
// are valid only as far as the scanner reads them, never for a driver.

#include <gtest/gtest.h>

#include <cstdint>
#include <functional>
#include <string>
#include <vector>

#include "testing/spirv_scan.hpp"

namespace st = spade::testing;

namespace {

// The opcodes these modules use, from the SPIR-V specification.
namespace op {
constexpr uint32_t kUndef = 1;
constexpr uint32_t kExtInstImport = 11;
constexpr uint32_t kExtInst = 12;
constexpr uint32_t kTypeVoid = 19;
constexpr uint32_t kTypeBool = 20;
constexpr uint32_t kTypeInt = 21;
constexpr uint32_t kTypeFloat = 22;
constexpr uint32_t kTypeFunction = 33;
constexpr uint32_t kFunction = 54;
constexpr uint32_t kFunctionEnd = 56;
constexpr uint32_t kFunctionCall = 57;
constexpr uint32_t kStore = 62;
constexpr uint32_t kDecorate = 71;
constexpr uint32_t kCompositeExtract = 81;
constexpr uint32_t kCopyObject = 83;
constexpr uint32_t kConvertFToU = 109;
constexpr uint32_t kConvertFToS = 110;
constexpr uint32_t kConvertUToF = 112;
constexpr uint32_t kBitcast = 124;
constexpr uint32_t kFNegate = 127;
constexpr uint32_t kFAdd = 129;
constexpr uint32_t kFMul = 133;
constexpr uint32_t kFDiv = 136;
constexpr uint32_t kSelect = 169;
constexpr uint32_t kFOrdLessThan = 184;
constexpr uint32_t kPhi = 245;
constexpr uint32_t kLabel = 248;
constexpr uint32_t kReturn = 253;
constexpr uint32_t kReturnValue = 254;
}  // namespace op

// GLSL.std.450's extended-instruction numbers.
namespace glsl {
constexpr uint32_t kRound = 1;
constexpr uint32_t kTrunc = 3;
constexpr uint32_t kFAbs = 4;
constexpr uint32_t kFloor = 8;
constexpr uint32_t kSqrt = 31;
constexpr uint32_t kInverseSqrt = 32;
}  // namespace glsl

// The ids every module declares: the import, the types, one function, and
// three undefined inputs (two floats and an unsigned).
constexpr uint32_t kGlsl = 1;
constexpr uint32_t kFloat = 2;
constexpr uint32_t kUint = 3;
constexpr uint32_t kInt = 4;
constexpr uint32_t kVoid = 5;
constexpr uint32_t kBool = 6;
constexpr uint32_t kFnType = 7;
constexpr uint32_t kFn = 8;
constexpr uint32_t kEntry = 9;
constexpr uint32_t kA = 10;
constexpr uint32_t kB = 11;
constexpr uint32_t kN = 12;

class Module {
public:
    Module() {
        declare(op::kExtInstImport, {kGlsl, 0x4C534C47u, 0x6474732Eu, 0x3035342Eu, 0u});  // "GLSL.std.450"
        declare(op::kTypeFloat, {kFloat, 32});
        declare(op::kTypeInt, {kUint, 32, 0});
        declare(op::kTypeInt, {kInt, 32, 1});
        declare(op::kTypeVoid, {kVoid});
        declare(op::kTypeBool, {kBool});
        declare(op::kTypeFunction, {kFnType, kVoid});
        use(op::kFunction, {kVoid, kFn, 0, kFnType});
        use(op::kLabel, {kEntry});
        use(op::kUndef, {kFloat, kA});
        use(op::kUndef, {kFloat, kB});
        use(op::kUndef, {kUint, kN});
    }

    // An instruction with a result: its new id. A contractable op is decorated
    // NoContraction, as P1 asks, so a module that keeps E1/E2 scans clean.
    uint32_t value(uint32_t opcode, uint32_t type, std::vector<uint32_t> operands) {
        const uint32_t id = next_++;
        operands.insert(operands.begin(), {type, id});
        use(opcode, std::move(operands));
        if (st::is_contractable_op(opcode)) declare(op::kDecorate, {id, st::kDecorationNoContraction});
        return id;
    }

    uint32_t glsl_call(uint32_t instruction, std::vector<uint32_t> operands) {
        operands.insert(operands.begin(), {kGlsl, instruction});
        return value(op::kExtInst, kFloat, std::move(operands));
    }

    // An instruction with no result.
    void use(uint32_t opcode, std::vector<uint32_t> operands) { emit(body_, opcode, operands); }

    [[nodiscard]] std::vector<uint32_t> words() const {
        std::vector<uint32_t> out{st::kSpirvMagic, 0x00010300u, 0u, next_, 0u};
        out.insert(out.end(), declarations_.begin(), declarations_.end());
        out.insert(out.end(), body_.begin(), body_.end());
        emit(out, op::kReturn, {});
        emit(out, op::kFunctionEnd, {});
        return out;
    }

private:
    void declare(uint32_t opcode, const std::vector<uint32_t>& operands) { emit(declarations_, opcode, operands); }

    static void emit(std::vector<uint32_t>& to, uint32_t opcode, const std::vector<uint32_t>& operands) {
        to.push_back((static_cast<uint32_t>(operands.size() + 1) << 16) | opcode);
        to.insert(to.end(), operands.begin(), operands.end());
    }

    std::vector<uint32_t> declarations_;
    std::vector<uint32_t> body_;
    uint32_t next_ = 20;
};

std::size_t findings_named(const st::SpirvScanResult& scan, const std::string& rule) {
    std::size_t n = 0;
    for (const st::SpirvFinding& f : scan.findings) n += f.rule == rule ? 1 : 0;
    return n;
}

std::string all_findings(const st::SpirvScanResult& scan) {
    std::string text;
    for (const st::SpirvFinding& f : scan.findings) text += "\n  " + f.rule + ": " + f.message;
    return text;
}

}  // namespace

// Rendering's exact division (rendering/exact-div-sqrt f71252b), in the shape
// Core read from its source (2026-10-22). The estimate y = a / b is scaled into
// the quotient's integer, and scaled against the integer remainder, then
// floored, to pick the rounding. Every path from y ends in an integer.
TEST(SpirvScanEstimates, PassesTheExactDivisionsEstimate) {
    Module m;
    const uint32_t y = m.value(op::kFDiv, kFloat, {kA, kB});
    const uint32_t ay = m.value(op::kFMul, kFloat, {kA, y});
    const uint32_t q = m.value(op::kFMul, kFloat, {ay, kB});
    m.value(op::kConvertFToU, kUint, {q});
    const uint32_t r = m.value(op::kConvertUToF, kFloat, {kN});  // the integer remainder: no estimate in it
    const uint32_t ry = m.value(op::kFMul, kFloat, {r, y});
    const uint32_t scaled = m.value(op::kFMul, kFloat, {ry, kA});
    const uint32_t half_up = m.value(op::kFAdd, kFloat, {scaled, kB});
    m.value(op::kConvertFToS, kInt, {m.glsl_call(glsl::kFloor, {half_up})});

    const st::SpirvScanResult scan = st::scan_spirv(m.words(), st::SpirvProfile::exact);
    EXPECT_TRUE(scan.clean()) << all_findings(scan);
    EXPECT_EQ(scan.estimate_ops, 1u);
}

// The exact sqrt's shape: r = InverseSqrt(x), x r scaled and truncated into the
// root's integer. A second path carries r through OpCopyObject, a negation and
// a rounding into a bitcast to an integer.
TEST(SpirvScanEstimates, PassesTheExactSqrtsEstimate) {
    Module m;
    const uint32_t r = m.glsl_call(glsl::kInverseSqrt, {kA});
    const uint32_t xr = m.value(op::kFMul, kFloat, {kA, r});
    const uint32_t scaled = m.value(op::kFMul, kFloat, {xr, kB});
    m.value(op::kConvertFToU, kUint, {m.glsl_call(glsl::kTrunc, {scaled})});
    const uint32_t copy = m.value(op::kCopyObject, kFloat, {r});
    const uint32_t negated = m.value(op::kFNegate, kFloat, {copy});
    m.value(op::kBitcast, kInt, {m.glsl_call(glsl::kRound, {negated})});

    const st::SpirvScanResult scan = st::scan_spirv(m.words(), st::SpirvProfile::exact);
    EXPECT_TRUE(scan.clean()) << all_findings(scan);
    EXPECT_EQ(scan.estimate_ops, 1u);
}

// Every other use fails, as E1 for a division's estimate and E2 for an inverse
// sqrt's, with exactly one finding: each module has one wrong use.
TEST(SpirvScanEstimates, FailsAnEstimateThatReachesAnythingButAnInteger) {
    struct Case {
        const char* what;
        const char* rule;
        std::function<void(Module&)> build;
    };
    const auto divide = [](Module& m) { return m.value(op::kFDiv, kFloat, {kA, kB}); };
    const Case cases[] = {
        {"stored", "E1", [&](Module& m) { m.use(op::kStore, {kN, divide(m)}); }},
        {"returned", "E1", [&](Module& m) { m.use(op::kReturnValue, {divide(m)}); }},
        {"compared after a float step", "E1",
         [&](Module& m) {
             const uint32_t t = m.value(op::kFMul, kFloat, {divide(m), kA});
             m.value(op::kFOrdLessThan, kBool, {t, kB});
         }},
        {"selected", "E1", [&](Module& m) { m.value(op::kSelect, kFloat, {kN, divide(m), kA}); }},
        {"joined by an OpPhi", "E1", [&](Module& m) { m.value(op::kPhi, kFloat, {divide(m), kEntry, kA, kEntry}); }},
        {"passed to a call", "E1", [&](Module& m) { m.value(op::kFunctionCall, kFloat, {kFn, divide(m)}); }},
        {"bitcast to a float", "E1", [&](Module& m) { m.value(op::kBitcast, kFloat, {divide(m)}); }},
        {"through FAbs, which is not on the list", "E1",
         [&](Module& m) { m.value(op::kConvertFToU, kUint, {m.glsl_call(glsl::kFAbs, {divide(m)})}); }},
        {"an inverse sqrt stored after a float step", "E2",
         [&](Module& m) {
             const uint32_t r = m.glsl_call(glsl::kInverseSqrt, {kA});
             m.use(op::kStore, {kN, m.value(op::kFMul, kFloat, {r, kA})});
         }},
        {"a bare Sqrt, even converted", "E2",
         [&](Module& m) { m.value(op::kConvertFToU, kUint, {m.glsl_call(glsl::kSqrt, {kA})}); }},
    };
    for (const Case& c : cases) {
        Module m;
        c.build(m);
        const st::SpirvScanResult scan = st::scan_spirv(m.words(), st::SpirvProfile::exact);
        EXPECT_EQ(findings_named(scan, c.rule), 1u) << c.what << all_findings(scan);
        EXPECT_EQ(scan.findings.size(), 1u) << c.what << all_findings(scan);
    }
}

// The finding names the use and where its estimate came from.
TEST(SpirvScanEstimates, NamesTheUseAndItsEstimate) {
    Module m;
    m.use(op::kStore, {kN, m.value(op::kFDiv, kFloat, {kA, kB})});
    const st::SpirvScanResult scan = st::scan_spirv(m.words(), st::SpirvProfile::exact);
    ASSERT_EQ(scan.findings.size(), 1u) << all_findings(scan);
    EXPECT_NE(scan.findings[0].message.find("OpStore"), std::string::npos) << scan.findings[0].message;
    EXPECT_NE(scan.findings[0].message.find("OpFDiv at word"), std::string::npos) << scan.findings[0].message;
}

// A literal operand whose value happens to equal an estimate's id is not a use
// of it: here an OpCompositeExtract index.
TEST(SpirvScanEstimates, ReadsALiteralAsALiteral) {
    Module m;
    const uint32_t y = m.value(op::kFDiv, kFloat, {kA, kB});
    m.value(op::kConvertFToU, kUint, {y});
    m.value(op::kCompositeExtract, kFloat, {kA, y});
    const st::SpirvScanResult scan = st::scan_spirv(m.words(), st::SpirvProfile::exact);
    EXPECT_TRUE(scan.clean()) << all_findings(scan);
}

// Under parity, division and roots stay allowed: a banded kernel may use them
// (the global constraint behind the two profiles).
TEST(SpirvScanEstimates, LeavesParityKernelsTheirDivisionAndRoots) {
    Module m;
    m.use(op::kStore, {kN, m.value(op::kFDiv, kFloat, {kA, kB})});
    m.use(op::kStore, {kN, m.glsl_call(glsl::kInverseSqrt, {kA})});
    m.use(op::kStore, {kN, m.glsl_call(glsl::kSqrt, {kA})});
    const st::SpirvScanResult scan = st::scan_spirv(m.words(), st::SpirvProfile::parity);
    EXPECT_TRUE(scan.clean()) << all_findings(scan);
}
