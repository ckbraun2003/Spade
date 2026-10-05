#pragma once

#include <cstddef>
#include <cstdint>
#include <span>
#include <string>
#include <vector>

// ---------------------------------------------------------------------------
// spirv_scan.hpp -- a minimal SPIR-V word scanner, and the float-controls
// POLICY the D9 gate enforces with it. HEADER-ONLY TEST SUPPORT: it lives in
// engine/testing/ alongside replay.hpp and scenario_file.hpp, is compiled into
// spade_tests only, and is never installed.
//
// WHY A SCANNER IN-TREE RATHER THAN spirv-dis. The pinned slang prebuilt ships
// slangc/slangd/slangi and the runtime DLLs -- it does NOT ship spirv-dis, and
// nothing else in this tree depends on the Vulkan SDK being installed (the
// whole point of vendoring Vulkan-Headers and volk). Adding an SDK dependency
// so a test could grep disassembly TEXT would also make the check a string
// match against a tool's output format. The binary form is simpler and more
// stable: SPIR-V is a stream of 32-bit words, each instruction prefixed by
// (word_count << 16 | opcode), and everything below needs nothing more than
// that.
//
// ===========================================================================
// THE POLICY, and why each rule exists. TWO profiles (a third, NO_OP, was
// retired with its only module by S6 Task 8 -- see below).
//
// Profile PARITY -- every kernel whose arithmetic feeds registered state or a
// digest:
//
//   P1. EVERY CONTRACTABLE FLOAT OP CARRIES NoContraction. Vulkan lets an
//       implementation fuse a*b+c into a single-rounding FMA unless the result
//       is decorated NoContraction. The CPU twin is compiled /fp:precise (MSVC)
//       + -ffp-contract=off (gcc/clang, engine/CMakeLists.txt's
//       spade_fp_strict), i.e. two roundings, so an undecorated GPU multiply-
//       add is a last-bit divergence waiting for a driver that fuses.
//       cmake/SpadeSlang.cmake compiles every kernel -fp-mode precise, which
//       is what puts the decoration on; this rule is the tripwire that fires
//       if that flag is ever dropped.
//
//   P3. NO FP32 DENORMAL MODE IS REQUESTED. No module declares a
//       `DenormPreserve` or `DenormFlushToZero` execution mode, for any width,
//       or the matching capability (SPV_KHR_float_controls; the four values
//       below). Every device then runs the kernels legally, with its own
//       default.
//       - Why: requesting a mode the device does not support
//         (VkPhysicalDeviceFloatControlsProperties) is a Vulkan valid-usage
//         violation, i.e. undefined behaviour, not a reported error. The RTX
//         3060 Ti supports neither mode.
//       - What it costs: subnormal magnitudes only. Both devices on record
//         flush by default (the Iris Plus, recorded in S6 Task 4, and the
//         3060 Ti, measured in 2026-10-04-nvidia-denorm-report.md), and
//         fp32_math's subnormal paths are integer forms that do not depend on
//         the mode (the banded-parity plan's T1). The CPU twin preserves, and
//         the difference is banded, never assumed away (TD-14).
//       - The tripwire: if a flag, or a future slangc default, puts a request
//         back, the gate fails before any device runs it.
//       - Checked in BOTH profiles.
//       - History: until 2026-10-05, P3 REQUIRED `DenormPreserve 32`, and
//         SpadeSlang.cmake compiled every kernel -denorm-mode-fp32 preserve.
//         The user's ruling of 2026-10-05 ended bit-exact CPU<->GPU parity, and
//         the rule was amended to this (docs/design/core/plans/2026-10-05-
//         banded-parity-plan.md; test-docs/00-decisions.md).
//
//   P4. NO `OpCapability Int64` (S6 Task 8, and it is a MEASUREMENT rather
//       than a precaution). The Intel Iris Plus, the device of record when
//       this rule was written, reports VkPhysicalDeviceFeatures::shaderInt64
//       == VK_FALSE (Task 8's plan-mandated probe;
//       tests/test_compute_context.cpp's GpuContext.Int64ProbeMatchesTheDevice
//       prints it for whatever device runs it). A SPIR-V module may only
//       declare a capability whose feature the logical device enabled, so a
//       module declaring Int64 is UNDEFINED BEHAVIOUR on such a device, not a
//       reported error -- the same shape of hazard P3 exists for.
//
//       IT CAUGHT A REAL, SHIPPED VIOLATION. forces_drag.spv and integrate.spv
//       carried OpCapability Int64 from S6 Task 6 through Task 7, because
//       shaders/shared/layouts.slang mirrored WorldParams::seed as a
//       `uint64_t` and slangc declares the 64-bit TYPE for any struct that has
//       one, read or not. Task 8 respelled every 64-bit mirror as a `uint2`
//       (identical std430 size, alignment and offsets -- the generated
//       static_asserts prove it per field) and wrote engine/shaders/u64.slang
//       so rng.slang can still do EXACT 64-bit arithmetic on the pair. This
//       rule is what stops the next `uint64_t` from reintroducing it.
//
//       CHECKED IN EVERY PROFILE, like P2 and unlike P1/P3: the hazard is a
//       property of the module's declaration section, not of what kind of
//       arithmetic the kernel does, so a stub is no more entitled to it than a
//       physics kernel.
//
//   P2. NO SUM-OF-PRODUCTS OPCODE (OpDot, OpVectorTimesMatrix,
//       OpMatrixTimesVector, OpMatrixTimesMatrix). Two independent reasons,
//       either sufficient. First, SPIR-V does not specify the ORDER in which
//       these accumulate their products, and fp32 addition is not associative
//       -- so even a perfectly non-fusing implementation may not match the CPU
//       twin's pinned summation order. Second, measured on slang v2026.14.1
//       (this task's report has the transcript): -fp-mode precise decorates
//       OpFAdd, OpFSub, OpFMul, OpFNegate, OpFRem and OpVectorTimesScalar, but
//       does NOT decorate OpDot -- so even the contraction inside a dot product
//       is unpinnable. The sanctioned spelling is the sum written out
//       componentwise in the CPU twin's order.
//
//   P5. NO GLSL.std.450 TRANSCENDENTAL. The machine enforcement of the global
//       constraint's GPU corollary -- "kernels never call GPU-vendor
//       transcendental intrinsics on parity paths -- they call the Slang ports
//       of log32/exp32/sin32/cos32" -- which was a rule every kernel obeyed and
//       NOTHING CHECKED until S6 Task 8 (review minor M8). It closes a
//       PRE-EXISTING gap rather than imposing a new requirement: wave A already
//       depended on it (integrate.slang's exp-map calls sin32/cos32), and wave
//       C is merely the first wave where a slip would have been easy -- dryden,
//       rotors and rng call four ports between them, and `exp`, `log`, `sin`
//       and `cos` are each one keystroke from the intrinsic of the same name.
//
//       WHY IT MATTERS, since the ports are what a reader sees in the source: a
//       Slang kernel that writes `exp(x)` compiles, links and runs. It just
//       calls the DRIVER's exponential, which Vulkan specifies to a relative
//       tolerance rather than correctly rounding, and whose answer therefore
//       varies by vendor and by driver version. That is not a bounded error
//       like OpFDiv's <= 2.5 ulp -- it is an unmeasurable one, and it would put
//       every digest downstream of it at the mercy of a driver update. The
//       whole point of the fp32_math ports is that they are the SAME
//       arithmetic on both sides, proven bit-identical by
//       tests/test_gpu_fp32_math.cpp.
//
//       THE FORBIDDEN SET IS THE WHOLE EXP/LOG/TRIG FAMILY, not only the four
//       functions the ports replace: Sin, Cos, Tan, the six inverse and
//       hyperbolic forms, Atan2, Pow, Exp, Log, Exp2 and Log2. Forbidding only
//       Exp/Log/Sin/Cos would leave the hole exactly where a future author
//       would fall into it -- `exp2`, `pow` and `tan` are the same class of
//       vendor-defined function, and a kernel that needs one needs a PORT of
//       it, not the intrinsic. The widening cost nothing: measured across all
//       ten compiled modules, the ONLY GLSL.std.450 calls anywhere in this tree
//       are FAbs, Floor and Sqrt.
//
//       SQRT AND INVERSESQRT ARE DELIBERATELY NOT IN THE SET. They are E1/E2's
//       business, exact-profile only, because the global constraint explicitly
//       allows downstream kernels div and sqrt -- that allowance is precisely
//       why CPU<->GPU is banded rather than bit-identical. Banning sqrt under
//       parity would forbid `bounce`'s friction step, sdf_eval's five length
//       sites and Box-Muller's radius, i.e. most of the port.
//
//       COUNTED IN EVERY PROFILE, reported as a finding under parity and exact
//       alike (exact is a superset of parity), and asserted BY NAME in
//       tests/test_slang_layouts.cpp so a failure log names the rule rather
//       than quoting a string.
//
// (P1 through P5 are PARITY. P3 was added by S6 Task 4, P4 and P5 by Task 8;
// P3 and P4 came from a device measurement and P5 from a review finding -- see
// their text above. P3 was amended on 2026-10-05, from "DenormPreserve 32
// required" to "no denormal mode requested", under the user's ruling that
// ended bit-exact CPU<->GPU parity.)
//
// Profile EXACT -- PARITY, plus the two rules the global constraints impose on
// the fp32_math module (Task 4) and on anything else that must be
// correctly-rounded rather than merely non-fusing:
//
//   E1. NO OpFDiv. Vulkan's division is <= 2.5 ulp, NOT correctly rounded.
//       Respell as a multiply by a precomputed, correctly-rounded inverse
//       constant, documented at the site.
//   E2. NO GLSL.std.450 Sqrt OR InverseSqrt. Same reason, same bound.
//
// DOWNSTREAM KERNELS MAY USE div/sqrt (global constraint: "downstream kernels
// MAY use div/sqrt -- that is why CPU<->GPU is banded rather than
// bit-identical"), which is exactly why E1/E2 are a separate profile rather
// than part of PARITY.
//
// PROFILE NO_OP -- RETIRED (S6 Task 8), AND THE RETIREMENT IS RECORDED RATHER
// THAN SILENT.
//
// It existed for ONE module: pipeline_smoke, the byte-preserving stub every
// un-ported schedule slot dispatched from S6 Task 5 through Task 7. Its rules
// were N1 (zero floating-point arithmetic ops -- every opcode P1 would demand
// NoContraction on, forbidden outright) and N2 (zero extended-instruction-set
// calls), and its reason for existing was that PARITY's non-vacuity expectation
// ("at least one contractable op, and every one of them decorated") is the
// WRONG SHAPE of assertion for a kernel that must contain none: it would force
// the stub to carry fake work purely to satisfy the checker.
//
// Task 8 ports the last two passes (MediumUpdate and SensorSynthesis), so no
// schedule slot binds a stub any more -- the two INERT slots record no dispatch
// at all -- and pipeline_smoke is deleted. A profile with no module to check
// would be an untested branch of scan_spirv() kept against a future need that
// S6 does not have, so it is deleted with its instance. THE ARGUMENT SURVIVES
// WHERE IT IS STILL LOAD-BEARING: SpirvModule::integer_only in
// tests/test_slang_layouts.cpp reaches the same "swap the non-vacuity
// instrument, keep every rule" conclusion from the other direction, for
// grid_sort -- a real kernel that genuinely has no float arithmetic. If a
// future task needs a stub again, that flag is the pattern to extend, and this
// note is the record of what was tried first.
//
// ===========================================================================

namespace spade::testing {

// SPIR-V opcodes this scanner recognises. Values from the SPIR-V 1.6
// specification's "Instructions" table; only the ones the policy names are
// listed, because an opcode nobody checks is an opcode nobody should be able
// to mis-transcribe.
namespace spv_op {
inline constexpr uint32_t kCapability = 17;
inline constexpr uint32_t kExtInstImport = 11;
inline constexpr uint32_t kExtInst = 12;
inline constexpr uint32_t kEntryPoint = 15;
inline constexpr uint32_t kExecutionMode = 16;
inline constexpr uint32_t kStore = 62;
inline constexpr uint32_t kDecorate = 71;
inline constexpr uint32_t kFNegate = 127;
inline constexpr uint32_t kFAdd = 129;
inline constexpr uint32_t kFSub = 131;
inline constexpr uint32_t kFMul = 133;
inline constexpr uint32_t kFDiv = 136;
inline constexpr uint32_t kFRem = 140;
inline constexpr uint32_t kFMod = 141;
inline constexpr uint32_t kVectorTimesScalar = 142;
inline constexpr uint32_t kMatrixTimesScalar = 143;
inline constexpr uint32_t kVectorTimesMatrix = 144;
inline constexpr uint32_t kMatrixTimesVector = 145;
inline constexpr uint32_t kMatrixTimesMatrix = 146;
inline constexpr uint32_t kOuterProduct = 147;
inline constexpr uint32_t kDot = 148;
}  // namespace spv_op

// SPIR-V Decoration numbers. NoContraction is 42 -- worth stating, because it
// sits in a run of neighbours (FPFastMathMode 40, LinkageAttributes 41,
// InputAttachmentIndex 43, Alignment 44) that an off-by-one would silently
// mistake it for, producing a check that always passes.
inline constexpr uint32_t kDecorationNoContraction = 42;

// GLSL.std.450 extended-instruction numbers for the two roots E2 forbids.
inline constexpr uint32_t kGlslStd450Sqrt = 31;
inline constexpr uint32_t kGlslStd450InverseSqrt = 32;

// ---------------------------------------------------------------------------
// The GLSL.std.450 transcendentals rule P5 forbids, by their extended-
// instruction numbers (GLSL.std.450 specification, "Instructions"). Listed
// individually with their numbers visible rather than as a range, because the
// range 13..30 also CONTAINS nothing else -- but a reader checking this against
// the spec should be able to do it name by name, and a future addition
// (Determinant is 33, Sqrt is 31) must be a deliberate edit rather than a
// bound that quietly grew.
//
// NOT LISTED, AND EACH FOR A REASON: Sqrt/InverseSqrt (31/32) are E2's, allowed
// under parity by the global constraint; FAbs (4), Floor (8), FMin/FMax
// (37/40) and FClamp (43) are not transcendentals at all -- they are exact
// operations on the value's bits or a selection between two of them, and the
// kernels use FAbs and Floor today.
// ---------------------------------------------------------------------------
[[nodiscard]] inline bool is_forbidden_transcendental(uint32_t glsl_std450_instruction) noexcept {
    switch (glsl_std450_instruction) {
        case 13:  // Sin
        case 14:  // Cos
        case 15:  // Tan
        case 16:  // Asin
        case 17:  // Acos
        case 18:  // Atan
        case 19:  // Sinh
        case 20:  // Cosh
        case 21:  // Tanh
        case 22:  // Asinh
        case 23:  // Acosh
        case 24:  // Atanh
        case 25:  // Atan2
        case 26:  // Pow
        case 27:  // Exp
        case 28:  // Log
        case 29:  // Exp2
        case 30:  // Log2
            return true;
        default:
            return false;
    }
}

[[nodiscard]] inline const char* transcendental_name(uint32_t glsl_std450_instruction) noexcept {
    switch (glsl_std450_instruction) {
        case 13: return "Sin";
        case 14: return "Cos";
        case 15: return "Tan";
        case 16: return "Asin";
        case 17: return "Acos";
        case 18: return "Atan";
        case 19: return "Sinh";
        case 20: return "Cosh";
        case 21: return "Tanh";
        case 22: return "Asinh";
        case 23: return "Acosh";
        case 24: return "Atanh";
        case 25: return "Atan2";
        case 26: return "Pow";
        case 27: return "Exp";
        case 28: return "Log";
        case 29: return "Exp2";
        case 30: return "Log2";
        default: return "<transcendental>";
    }
}

// The denormal modes and their capabilities (SPV_KHR_float_controls; core from
// SPIR-V 1.4), the four values rule P3 refuses. An execution mode's single
// literal operand is the FLOAT WIDTH it applies to; P3 refuses every width.
// Confirmed on 2026-10-05 two ways: Khronos SPIRV-Headers
// (include/spirv/unified1/spirv.hpp) gives these values, and the pinned slangc
// v2026.14.1 emits exactly them for -denorm-mode-fp32 preserve and ftz, naming
// them in its spirv-asm output.
inline constexpr uint32_t kExecutionModeDenormPreserve = 4459;
inline constexpr uint32_t kExecutionModeDenormFlushToZero = 4460;
inline constexpr uint32_t kCapabilityDenormPreserve = 4464;
inline constexpr uint32_t kCapabilityDenormFlushToZero = 4465;
inline constexpr uint32_t kFloatWidth32 = 32;

// SPIR-V Capability Int64 (rule P4). Worth stating that it sits in a run of
// neighbours an off-by-one would silently mistake it for -- Float64 is 10,
// Int64Atomics is 12 -- so a mis-transcription would produce a rule that never
// fires. Cross-checked against the compiled artifacts: before Task 8's fix,
// scanning forces_drag.spv's declaration section for this exact value found it,
// and scanning collision_static.spv (which does not touch world_params) did
// not. That asymmetry is what confirms both the number and the diagnosis.
inline constexpr uint32_t kCapabilityInt64 = 11;

inline constexpr uint32_t kSpirvMagic = 0x07230203u;

// One finding: a human-readable statement of a rule that was broken. The scan
// returns every finding rather than the first, so a failing test names all of
// them at once instead of forcing one round trip per violation.
struct SpirvFinding {
    std::string rule;     // "P1", "P2", "P3", "P4", "P5", "E1", "E2"
    std::string message;  // what was found, with the offending word index
};

enum class SpirvProfile {
    parity,  // P1 + P2 + P3 + P4
    exact,   // P1 + P2 + P3 + P4 + E1 + E2
    // no_op RETIRED (S6 Task 8) with its only instance; see this file's header.
};

// True for the opcodes an implementation may contract and that slangc DOES
// decorate under -fp-mode precise. OpDot and the matrix products are absent on
// purpose: rule P2 forbids them outright rather than asking for a decoration
// slangc will not emit.
[[nodiscard]] inline bool is_contractable_op(uint32_t opcode) noexcept {
    switch (opcode) {
        case spv_op::kFNegate:
        case spv_op::kFAdd:
        case spv_op::kFSub:
        case spv_op::kFMul:
        case spv_op::kFDiv:
        case spv_op::kFRem:
        case spv_op::kFMod:
        case spv_op::kVectorTimesScalar:
        case spv_op::kMatrixTimesScalar:
        case spv_op::kOuterProduct:
            return true;
        default:
            return false;
    }
}

[[nodiscard]] inline const char* op_name(uint32_t opcode) noexcept {
    switch (opcode) {
        case spv_op::kFNegate: return "OpFNegate";
        case spv_op::kFAdd: return "OpFAdd";
        case spv_op::kFSub: return "OpFSub";
        case spv_op::kFMul: return "OpFMul";
        case spv_op::kFDiv: return "OpFDiv";
        case spv_op::kFRem: return "OpFRem";
        case spv_op::kFMod: return "OpFMod";
        case spv_op::kVectorTimesScalar: return "OpVectorTimesScalar";
        case spv_op::kMatrixTimesScalar: return "OpMatrixTimesScalar";
        case spv_op::kVectorTimesMatrix: return "OpVectorTimesMatrix";
        case spv_op::kMatrixTimesVector: return "OpMatrixTimesVector";
        case spv_op::kMatrixTimesMatrix: return "OpMatrixTimesMatrix";
        case spv_op::kOuterProduct: return "OpOuterProduct";
        case spv_op::kDot: return "OpDot";
        default: return "<op>";
    }
}

// What the scan measured, so a passing test can also assert it looked at
// something. A module with no float arithmetic at all would satisfy every
// rule vacuously; `contractable_ops` is what makes that distinguishable.
struct SpirvScanResult {
    bool well_formed = false;         // magic word present, whole words, no truncated instruction
    uint32_t version_major = 0;
    uint32_t version_minor = 0;
    std::size_t contractable_ops = 0;  // ops subject to rule P1
    std::size_t decorated_ops = 0;     // of those, how many carry NoContraction (parity/exact only)
    // Rule P3 (EVERY profile): each denormal execution mode or capability the
    // module declares, as "<what> at word <i>". Empty is the only passing value.
    std::vector<std::string> denorm_requests;
    bool declares_int64 = false;        // rule P4 (EVERY profile): OpCapability Int64 present
    // Rule P5: GLSL.std.450 exp/log/trig calls. COUNTED in every profile so a
    // caller can assert zero by name; reported as a finding under parity and
    // exact. Expected to be 0 for every module this engine compiles -- the
    // fp32_math Slang ports are what a kernel calls instead.
    std::size_t transcendental_ext_inst_ops = 0;
    std::size_t ext_inst_ops = 0;      // OpExtInst calls into any imported set -- counted, not forbidden

    // Populated for every profile, cheap and unconditional -- not every caller
    // needs them (test_slang_layouts.cpp's integer_only branch is today's user),
    // but they are facts about the module a future profile might also want
    // without this scanner growing a second pass to get them.
    std::string entry_point_name;  // OpEntryPoint's own literal name (NOT the Slang source function name)
    std::size_t store_count = 0;   // OpStore instructions -- what proves a load-then-store round trip
                                    // was not optimized away entirely (round 2's own verification ask)

    std::vector<SpirvFinding> findings;

    [[nodiscard]] bool clean() const noexcept { return well_formed && findings.empty(); }
    [[nodiscard]] bool requests_denorm_mode() const noexcept { return !denorm_requests.empty(); }
};

// ---------------------------------------------------------------------------
// Scans one module. TWO PASSES, and the reason is structural rather than
// stylistic: SPIR-V requires all OpDecorate instructions to appear in the
// module's decoration section, BEFORE the function bodies whose results they
// decorate -- but relying on that ordering to check "is this result decorated"
// in a single pass would make the checker depend on a layout rule instead of
// on the fact being checked. Pass 1 collects the decorated result ids and the
// GLSL.std.450 import id; pass 2 judges the instructions.
// ---------------------------------------------------------------------------
[[nodiscard]] inline SpirvScanResult scan_spirv(std::span<const uint32_t> words,
                                                SpirvProfile profile) {
    SpirvScanResult result;

    // Header: magic, version, generator, bound, schema -- five words minimum.
    if (words.size() < 5 || words[0] != kSpirvMagic) {
        result.findings.push_back({"format", "not a SPIR-V module (bad magic or too short)"});
        return result;
    }
    result.version_major = (words[1] >> 16) & 0xFFu;
    result.version_minor = (words[1] >> 8) & 0xFFu;

    // Both are looked up by linear search below. Deliberately: a module has a
    // handful of extended-instruction imports and, at these module sizes, a
    // few hundred decorations -- a set or a sort would cost more code than the
    // scan it accelerates, and this runs once per test.
    std::vector<uint32_t> no_contraction;
    std::vector<uint32_t> glsl_std450_ids;
    // Every OpExtInstImport id, regardless of name -- deliberately broader than
    // glsl_std450_ids (E2's, specific to the two forbidden roots), so
    // `ext_inst_ops` counts calls into ANY extended-instruction set rather than
    // only the one this pipeline happens to import today.
    std::vector<uint32_t> all_ext_inst_import_ids;

    // -- pass 1: decorations, extended-instruction-set imports, entry point --
    for (std::size_t i = 5; i < words.size();) {
        const uint32_t word_count = words[i] >> 16;
        const uint32_t opcode = words[i] & 0xFFFFu;
        if (word_count == 0 || i + word_count > words.size()) {
            result.findings.push_back({"format", "truncated or zero-length instruction"});
            return result;
        }
        if (opcode == spv_op::kDecorate && word_count >= 3 &&
            words[i + 2] == kDecorationNoContraction) {
            no_contraction.push_back(words[i + 1]);
        }
        // P4 (EVERY profile), collected in pass 1 for the same structural
        // reason P3 is: OpCapability lives in the module's declaration
        // section, ahead of every function body, so judging it in pass 2 would
        // be judging instruction order rather than the declaration. One
        // operand: the capability value.
        if (opcode == spv_op::kCapability && word_count >= 2 && words[i + 1] == kCapabilityInt64) {
            result.declares_int64 = true;
        }
        // P3, collected in pass 1 with the decorations: OpExecutionMode and
        // OpCapability sit in the module's own header section, before any
        // function body, so a pass-2 check would be a check about instruction
        // order rather than about the request. Execution mode operands:
        // <entry point id> <mode> <width>; every width counts.
        if (opcode == spv_op::kExecutionMode && word_count >= 3 &&
            (words[i + 2] == kExecutionModeDenormPreserve ||
             words[i + 2] == kExecutionModeDenormFlushToZero)) {
            std::string what = words[i + 2] == kExecutionModeDenormPreserve
                                   ? "OpExecutionMode DenormPreserve"
                                   : "OpExecutionMode DenormFlushToZero";
            if (word_count >= 4) what += " " + std::to_string(words[i + 3]);
            result.denorm_requests.push_back(what + " at word " + std::to_string(i));
        }
        if (opcode == spv_op::kCapability && word_count >= 2 &&
            (words[i + 1] == kCapabilityDenormPreserve || words[i + 1] == kCapabilityDenormFlushToZero)) {
            result.denorm_requests.push_back(
                std::string(words[i + 1] == kCapabilityDenormPreserve ? "OpCapability DenormPreserve"
                                                                      : "OpCapability DenormFlushToZero") +
                " at word " + std::to_string(i));
        }
        if (opcode == spv_op::kExtInstImport && word_count >= 3) {
            all_ext_inst_import_ids.push_back(words[i + 1]);
            // Operands: result id, then the set name as packed, NUL-terminated
            // UTF-8 words. "GLSL.std.450" is 12 bytes + NUL = 13 -> 4 words.
            std::string name;
            for (uint32_t w = 2; w < word_count; ++w) {
                const uint32_t packed = words[i + w];
                for (int b = 0; b < 4; ++b) {
                    const char c = static_cast<char>((packed >> (8 * b)) & 0xFFu);
                    if (c == '\0') break;
                    name.push_back(c);
                }
            }
            if (name == "GLSL.std.450") glsl_std450_ids.push_back(words[i + 1]);
        }
        if (opcode == spv_op::kEntryPoint && word_count >= 3 && result.entry_point_name.empty()) {
            // Operands: execution model, entry point <id>, then the name as
            // packed, NUL-terminated UTF-8 words (identical packing to the
            // import name above). A module may declare more than one entry
            // point in general; this scanner only ever inspects single-
            // entry-point compute kernels, so the first one found is kept.
            for (uint32_t w = 3; w < word_count; ++w) {
                const uint32_t packed = words[i + w];
                bool done = false;
                for (int b = 0; b < 4; ++b) {
                    const char c = static_cast<char>((packed >> (8 * b)) & 0xFFu);
                    if (c == '\0') { done = true; break; }
                    result.entry_point_name.push_back(c);
                }
                if (done) break;
            }
        }
        i += word_count;
    }

    const auto is_decorated = [&no_contraction](uint32_t id) {
        for (const uint32_t decorated : no_contraction) {
            if (decorated == id) return true;
        }
        return false;
    };
    const auto is_glsl_set = [&glsl_std450_ids](uint32_t id) {
        for (const uint32_t set_id : glsl_std450_ids) {
            if (set_id == id) return true;
        }
        return false;
    };
    const auto is_ext_inst_set = [&all_ext_inst_import_ids](uint32_t id) {
        for (const uint32_t set_id : all_ext_inst_import_ids) {
            if (set_id == id) return true;
        }
        return false;
    };

    // -- pass 2: judge ------------------------------------------------------
    for (std::size_t i = 5; i < words.size();) {
        const uint32_t word_count = words[i] >> 16;
        const uint32_t opcode = words[i] & 0xFFFFu;

        // P2 / E1: opcodes forbidden outright, before any decoration question.
        if (opcode == spv_op::kDot || opcode == spv_op::kVectorTimesMatrix ||
            opcode == spv_op::kMatrixTimesVector || opcode == spv_op::kMatrixTimesMatrix) {
            result.findings.push_back(
                {"P2", std::string(op_name(opcode)) +
                           " at word " + std::to_string(i) +
                           ": sum-of-products opcodes have unspecified accumulation order and "
                           "slangc does not decorate them NoContraction; spell the sum out "
                           "componentwise in the CPU twin's pinned order"});
        }
        if (profile == SpirvProfile::exact && opcode == spv_op::kFDiv) {
            result.findings.push_back(
                {"E1", "OpFDiv at word " + std::to_string(i) +
                           ": Vulkan division is <= 2.5 ulp, not correctly rounded; multiply by a "
                           "precomputed correctly-rounded inverse instead"});
        }
        // P5 (parity AND exact): a GLSL.std.450 exp/log/trig call is the
        // DRIVER's transcendental, specified to a relative tolerance rather
        // than correctly rounded -- an unbounded, vendor- and
        // driver-version-dependent difference from the CPU twin, not a banded
        // one. Counted unconditionally so a test can assert zero by name.
        if (opcode == spv_op::kExtInst && word_count >= 5 && is_glsl_set(words[i + 3]) &&
            is_forbidden_transcendental(words[i + 4])) {
            ++result.transcendental_ext_inst_ops;
            result.findings.push_back(
                {"P5", std::string("GLSL.std.450 ") + transcendental_name(words[i + 4]) +
                           " at word " + std::to_string(i) +
                           ": that is the DRIVER's transcendental, specified to a relative "
                           "tolerance and different per vendor and per driver version. Call the "
                           "engine's own port instead -- engine/shaders/fp32_math.slang's "
                           "log32/exp32/sin32/cos32, which tests/test_gpu_fp32_math.cpp proves "
                           "bit-identical to the host; a function with no port yet needs one "
                           "written, not the intrinsic"});
        }

        if (profile == SpirvProfile::exact && opcode == spv_op::kExtInst && word_count >= 5 &&
            is_glsl_set(words[i + 3])) {
            const uint32_t instruction = words[i + 4];
            if (instruction == kGlslStd450Sqrt || instruction == kGlslStd450InverseSqrt) {
                result.findings.push_back(
                    {"E2", std::string("GLSL.std.450 ") +
                               (instruction == kGlslStd450Sqrt ? "Sqrt" : "InverseSqrt") +
                               " at word " + std::to_string(i) +
                               ": <= 2.5 ulp, not correctly rounded; respell without it"});
            }
        }

        // OpExtInst into ANY imported set, counted for the record. No rule
        // forbids it any more: N2 did, and retired with the no_op profile
        // (S6 Task 8). The COUNT stays because it is a cheap, unconditional
        // fact about a module -- E2's specifically-named roots are checked
        // above, and a future profile that wants the broader statement has the
        // number already.
        if (opcode == spv_op::kExtInst && word_count >= 5 && is_ext_inst_set(words[i + 3])) {
            ++result.ext_inst_ops;
        }

        // P1: everything an implementation may fuse must say it must not.
        if (is_contractable_op(opcode) && word_count >= 3) {
            ++result.contractable_ops;
            const uint32_t result_id = words[i + 2];  // <result-type> <result-id> ...
            if (is_decorated(result_id)) {
                ++result.decorated_ops;
            } else {
                result.findings.push_back(
                    {"P1", std::string(op_name(opcode)) + " (result %" +
                               std::to_string(result_id) + ") at word " + std::to_string(i) +
                               " carries no NoContraction decoration; is the module compiled "
                               "-fp-mode precise?"});
            }
        }

        if (opcode == spv_op::kStore) {
            ++result.store_count;
        }

        i += word_count;
    }

    // P4. Reported once for the module, because a capability declaration IS a
    // module-level property -- the same shape of check as P3 below, and for the
    // same structural reason.
    if (result.declares_int64) {
        result.findings.push_back(
            {"P4",
             "`OpCapability Int64`: a device that reports shaderInt64 == VK_FALSE (the Iris "
             "Plus did) makes a module declaring it undefined behaviour there "
             "rather than a reported error. Mirror 64-bit fields as `uint2` "
             "(shaders/shared/layouts.slang) and do the arithmetic through "
             "engine/shaders/u64.slang, which is exact."});
    }

    // P3: no denormal mode requested. Reported once for the module, listing
    // every request, because the request IS a module-level property.
    if (result.requests_denorm_mode()) {
        std::string list;
        for (const std::string& request : result.denorm_requests) {
            list += (list.empty() ? "" : "; ") + request;
        }
        result.findings.push_back(
            {"P3",
             "a denormal mode is requested (" + list + "). A device that does not support the "
             "mode runs this module as undefined behaviour; no kernel may request one. Is a "
             "-denorm-mode flag back in cmake/SpadeSlang.cmake, or did a slangc update change "
             "its default?"});
    }

    result.well_formed = true;
    return result;
}

}  // namespace spade::testing
