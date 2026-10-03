#pragma once

#include <cstddef>
#include <cstdint>
#include <string_view>
#include <type_traits>

#include <glm/vec3.hpp>

#include "core/rng.hpp"
#include "state/layout.hpp"

// ---------------------------------------------------------------------------
// Medium -- the one seam every aerodynamic force reads the air through
// (engine design D6: "Per-world density/temperature + uniform wind + Dryden
// turbulence (seeded), behind one Medium interface P7 fields later
// implement").
//
// v0 was deliberately the degenerate case: ConstantMedium returns the
// per-world constants already carried in WorldParams and ignores position.
// The point of shipping it first was the SEAM, not the physics -- rotor aero,
// drag, and the sensor models were written against `const Medium&` and gained
// Dryden turbulence (and, behind P7, real flow fields) without touching a
// single call site. DrydenMedium, at the bottom of this header, is the first
// stateful implementation and the proof that the seam held.
//
// WHY WorldParams AND NOT WorldDesc/Environment. Environment (world/builder.hpp)
// is the AUTHORING record -- what a world file says. WorldParams
// (state/layout.hpp) is the RUNTIME row -- what the per-world param buffer
// holds, indexed by world id, and what a dispatch covering N worlds actually
// reads. Sampling is a per-step, per-world operation, so it takes the runtime
// row. That also keeps the medium out of the business of world loading.
//
// DETERMINISM. Everything a Medium samples must be a pure function of
// (WorldParams, position, and the sampler's own registered state). A stateful
// implementation -- Dryden is the first one -- keeps its filter state in a
// registered state array and draws exclusively from a spade::rng::Stream
// derived from WorldParams::seed with its own domain tag (core/rng.hpp).
// Reading a clock, or holding a hidden generator, breaks replay.
//
// VIRTUAL DISPATCH, AND ITS LIMIT. `sample` is virtual because this is a
// per-PASS boundary: a pass resolves the medium once and then runs its inner
// loop, so the indirect call is amortized over every body rather than paid
// per body. That is affordable now and is NOT a precedent -- S6 mirrors these
// passes onto the GPU, where a vtable does not exist. Future Medium
// implementations must therefore stay expressible as DATA (parameters in the
// per-world buffer plus a small kind tag) that a shader can branch on; an
// implementation whose behaviour lives in arbitrary host code will have no
// GPU twin, and P1/P2 parity is not optional.
// ---------------------------------------------------------------------------

namespace spade {

// What the air is doing at one point. fp32, SI, world frame -- the same
// conventions as WorldParams itself.
struct MediumSample {
    float density = 0.0f;      // kg/m^3
    glm::vec3 wind{0.0f};      // world-frame air velocity, m/s
};

class Medium {
public:
    Medium() = default;
    virtual ~Medium() = default;

protected:
    // Rule of five, spelled out. A Medium owns nothing, so these are all
    // defaulted -- but a polymorphic base with a user-declared destructor gets
    // its copy/move operations deprecated-but-generated, and "deprecated but
    // generated" is exactly how a slicing bug reaches a release. Protected
    // rather than public: copying through a base reference would slice, so
    // only derived classes (which know their own type) may use them.
    Medium(const Medium&) = default;
    Medium& operator=(const Medium&) = default;
    Medium(Medium&&) = default;
    Medium& operator=(Medium&&) = default;

public:
    // The air at `pos` in the world described by `params`.
    //
    // No Result<>: there is no failure mode. A medium that cannot describe a
    // point does not exist -- worlds are unbounded and the fallback is always
    // the per-world constants. Keeping this total is what lets it sit in an
    // inner loop.
    [[nodiscard]] virtual MediumSample sample(const WorldParams& params, glm::vec3 pos) const = 0;
};

// ---------------------------------------------------------------------------
// ConstantMedium (v0) -- uniform density and wind, straight out of
// WorldParams. Position-independent by definition.
//
// Stateless, and that is a feature: it holds no per-world data of its own, so
// ONE instance serves every world in a world set (the WorldParams row it is
// handed selects the world). It contributes nothing to the snapshot walk
// because it has nothing to contribute.
// ---------------------------------------------------------------------------
class ConstantMedium final : public Medium {
public:
    [[nodiscard]] MediumSample sample(const WorldParams& params, glm::vec3 pos) const override;
};

// ===========================================================================
// Dryden turbulence -- MIL-F-8785C low-altitude form (engine design D6:
// "Per-world density/temperature + uniform wind + Dryden turbulence
// (seeded), behind one Medium interface").
//
// ---------------------------------------------------------------------------
// 1. THE MODEL, STATED AS AUTOCORRELATIONS RATHER THAN PSD CONSTANTS
//
// The standard quotes Dryden as a pair of spatial power spectra plus a table
// of shaping filters. Both are written against an IMPLICIT convention for the
// driving noise -- one-sided vs two-sided spectrum, cycles vs radians, unit
// "power" vs unit intensity -- and the single most common way to get a Dryden
// implementation wrong is to paste the standard's continuous-time gain
// constants (the sqrt(2L/(pi V)) family) onto a discrete filter under a
// different convention. The result has exactly the right SHAPE and a variance
// off by a factor of pi, 2pi, or 2, which no spectrum plot makes obvious.
//
// So nothing here is copied from a gain table. The model is pinned by its
// autocorrelation functions, which are convention-free, and every constant
// below is derived from them:
//
//     longitudinal (u):   R_u(t) = sigma_u^2 exp(-|t|/tau_u)
//     transverse (v, w):  R(t)   = sigma^2  exp(-|t|/tau) (1 - |t|/(2 tau))
//
// Those are precisely Dryden's spatial correlation functions
// f(xi) = exp(-xi/L) and g(xi) = exp(-xi/L)(1 - xi/(2L)), carried into time by
// FROZEN TURBULENCE: a vehicle flying at reference airspeed V through a field
// that is static in space sees a spatial length L as a time constant
//
//     tau = L / V.
//
// They are also what the standard's shaping filters produce --
// H_u(s) proportional to 1/(1 + tau_u s) and
// H_{v,w}(s) proportional to (1 + sqrt(3) tau s)/(1 + tau s)^2 -- once their
// gains are fixed BY the variance instead of quoted from a table. Section 3
// does exactly that fixing.
//
// REFERENCE AIRSPEED, HONESTLY. V is a configured constant
// (DrydenParams::reference_airspeed, default 5 m/s), not the vehicle's actual
// airspeed. For a hovering or slowly-translating multirotor there is no
// meaningful "the" airspeed, and 5 m/s is a stand-in for the low-speed regime
// the engine targets first. Two consequences, stated rather than hidden:
// (a) tau is then a property of the WORLD, not of any vehicle, which is what
// lets one filter row serve every body in the world; (b) at true hover the
// frozen-turbulence mapping degenerates (a stationary vehicle should see the
// field's own temporal decorrelation, not L/V), so at low V this is an
// approximation whose knob is V itself -- lowering V lengthens the gust
// correlation time, raising it shortens it. A per-body airspeed-driven
// mapping and a spatially correlated field are both S8 roadmap, not v1.
//
// SCALE LENGTHS. Defaults L_u = L_v = 200 m, L_w = 50 m. These are not
// arbitrary: MIL-F-8785C's low-altitude relations L_w = h and
// L_u = L_v = h / (0.177 + 0.000823 h)^1.2 (h in FEET) evaluated at
// h = 50 m = 164.04 ft give L_w = 50 m and L_u = 202 m. The defaults are that
// altitude's values rounded to 200 m -- a 1% difference, well inside the
// model's own fidelity. Altitude-varying scale lengths are not v1.
//
// ---------------------------------------------------------------------------
// 2. STATE-SPACE REALIZATION, AND WHY THE STATE IS NORMALIZED
//
// The transverse channel is second order, so it needs two states. Writing
// H(s) = K (1 + sqrt(3) tau s) / (1 + tau s)^2 = K a^2 (sqrt(3) s / a + 1) /
// (s + a)^2 with a = 1/tau, the CONTROLLABLE CANONICAL form is
//
//     xdot = A x + B n,   A = [[0, 1], [-a^2, -2a]],  B = [0, 1]^T
//     y    = C x,         C = [K a^2, K sqrt(3) a]
//
// with n unit-intensity white noise. Solving the continuous Lyapunov equation
// A P + P A^T + B B^T = 0 gives the stationary state covariance
//
//     P = diag(1/(4 a^3), 1/(4 a))   (the off-diagonal term is exactly zero)
//
// so Var(y) = C P C^T = K^2 / tau, and demanding Var(y) = sigma^2 fixes
// K = sigma sqrt(tau). That is the gain derivation the standard's table
// encodes; doing it here is what makes the convention question moot.
//
// THE STATE STORED IN DrydenState IS NOT x -- it is z = P^{-1/2} x, i.e. x
// rescaled so its stationary covariance is the IDENTITY. Three things fall
// out, and together they are the reason for the change of variables:
//
//   * the output collapses to  y = sigma (z0 + sqrt(3) z1) / 2, whose
//     variance is sigma^2 (1/4 + 3/4) = sigma^2 BY INSPECTION;
//   * the discrete transition matrix becomes dimensionless and depends on
//     nothing but the step ratio theta = h/tau (section 3);
//   * initialization at the stationary distribution is just "two independent
//     standard normals", with no dependence on tau or sigma -- so there is no
//     spin-up transient to burn off and no state to rescale when the
//     turbulence level or the scale lengths change mid-run.
//
// The longitudinal channel is normalized the same way: DrydenState::u holds a
// UNIT-variance Ornstein-Uhlenbeck process and the output is sigma_u * u.
//
// ---------------------------------------------------------------------------
// 3. DISCRETIZATION: EXACT, NOT EULER
//
// Both channels are discretized EXACTLY (matched to the continuous process at
// the sample instants) rather than by an explicit Euler / Gauss-Markov update
// of the "u += (-u + sigma sqrt(2 tau/h) w) h/tau" kind. Reasons, in order of
// weight:
//
//   1. Unconditional stability. The engine's substep h is a schedule
//      decision and tau = L/V is a config decision; nothing couples them.
//      Explicit Euler on the u channel is stable only for h < 2 tau and its
//      variance is wrong by O(h/tau) even when stable. The exact form has no
//      such domain: it is correct for EVERY theta in [0, inf), and as
//      theta -> inf it degrades gracefully to white noise of variance
//      sigma^2 rather than to oscillation or blow-up.
//   2. The autocorrelation is right by construction, so the "time constant
//      is L/V" test measures the model rather than the discretization error.
//   3. It is not more expensive: the coefficients depend only on theta, and
//      a step is the same handful of multiply-adds either way.
//
// Longitudinal, with phi = exp(-theta):
//
//     u <- phi u + sqrt(1 - phi^2) n
//
// Transverse: z <- Phi z + L n, where Phi = exp(A h) expressed in the
// normalized coordinates of section 2. A has a DOUBLE eigenvalue at -a, so
// (A + aI) is nilpotent and the matrix exponential is exact in closed form;
// after the P^{-1/2} similarity every tau cancels and
//
//     Phi = exp(-theta) [[1 + theta,  theta], [-theta, 1 - theta]].
//
// THE PROCESS-NOISE COVARIANCE IS NOT INTEGRATED -- IT IS IDENTIFIED.
// Exact discretization preserves the stationary covariance, which in these
// coordinates is the identity, so the discrete Lyapunov identity
// I = Phi I Phi^T + Q collapses to
//
//     Q = I - Phi Phi^T
//
// and L is its lower Cholesky factor. This is the whole variance-normalization
// argument, and it is worth being explicit about what it buys: the stationary
// variance of the DISCRETE recursion is exactly 1 per state, for every theta,
// as an algebraic identity rather than as a limit -- no continuous-time
// constant is ever evaluated at a discrete step size. The same identity
// applied to the scalar channel gives 1 - phi^2, which is why that coefficient
// is spelled that way above.
//
// NUMERICAL CONDITIONING (the part that is easy to get silently wrong).
// Q's entries are
//
//     Q00 = 1 - e^{-2t}(1 + 2t + 2t^2) = (4/3)t^3 - 2t^4 + ...
//     Q01 = 2 t^2 e^{-2t}
//     Q11 = 1 - e^{-2t}(1 - 2t + 2t^2) = 4t - 8t^2 + ...
//
// Q00 is O(theta^3) -- the "position-like" state of a smooth process barely
// moves in one step -- so at a realistic 200 Hz substep with tau = 40 s
// (theta = 1.25e-4) its true value is 2.6e-12 while the expression
// 1 - (something within 3e-12 of 1) evaluates in fp32 to exactly ZERO. That
// would make sqrt(Q00) zero, the Cholesky's L10 = Q01/L00 an infinity, and the
// whole filter NaN within one step. medium.cpp therefore evaluates Q00 and
// Q11 from documented Taylor series below a pinned threshold and from the
// closed form above it, and test_dryden.cpp checks both branches against a
// cancellation-free double-precision reference across a theta sweep spanning
// 1e-6 to 32. No fp64 appears in the engine; the series is the fp32 answer.
//
// STABILITY DOMAIN, STATED. theta in [0, kDrydenMaxStepRatio]. Phi's
// eigenvalues are exp(-theta) (double) and phi = exp(-theta), both in (0, 1]
// for every theta >= 0, so the recursion is a contraction at every step size
// -- there is no CFL-like condition to violate. theta = 0 (h = 0, or V = 0,
// or a non-finite input) yields Phi = I and Q = 0: the field freezes, which is
// the right degenerate answer. Above kDrydenMaxStepRatio the step ratio is
// clamped, purely to keep 2 t^2 e^{-2t} from evaluating as 0 * inf; by then
// exp(-theta) < 1e-13 and the process is already white noise to fp32.
//
// ---------------------------------------------------------------------------
// 4. WHERE THE STATE LIVES, AND THE CALL CONTRACT
//
// DrydenMedium OWNS NOTHING. One DrydenState row per world lives in a
// registered state array (ArenaSet::register_array<DrydenState>), so it is
// snapshot-covered by construction and a replay resumes the identical gust
// sequence. The medium is a per-world VIEW over one such row plus its
// parameters; sample() reads the CURRENT filter output and never advances it.
//
// THE SCHEDULE CONTRACT, spelled out because it is not enforceable by the
// type system: dryden_advance() must be called EXACTLY ONCE PER SUBSTEP for
// each world, in the dryden.advance pass -- in the Fields phase, so before
// any reader (sim/standard_modules.cpp; only placement-first kinematic
// behaviors run ahead of it, and they touch no medium state)
// -- and therefore BEFORE the dryden.sample pass writes sample()'s
// expression into the world's wind field, which every force element then
// reads (module-API stage 3). Calling it twice per substep halves the correlation time and
// inflates the number of gust draws; calling it zero times freezes the gust
// for that substep. Both are silent, so the pass that owns the schedule owns
// this invariant.
//
// DETERMINISM. Every draw comes from the row's own rng::Stream, derived
// once by dryden_init() as make_stream(WorldParams::seed, "dryden", 0). There
// is no other random source, no clock read, and no state outside the row --
// which is what makes "snapshot the row, restore it, resume identically" a
// property of the layout rather than a thing to be tested for and hoped.
// Exactly five gaussians are drawn per substep in a pinned order,
// UNCONDITIONALLY -- including at turbulence level none, where they are
// multiplied by sigma = 0. Skipping the draws would make the stream position
// depend on the turbulence level, so two runs of the same scenario at
// different levels would diverge in every OTHER stochastic system too.
//
// POSITION INDEPENDENCE. sample() ignores `pos`, exactly as ConstantMedium
// does. This is a FROZEN-FIELD POINT MODEL: one gust vector per world per
// substep, seen identically by every body in that world. Real Dryden fields
// are spatially correlated (bodies metres apart see correlated but not equal
// gusts, and a single body sees a gust gradient across its rotor disc); that
// is the S8 spatial-field roadmap item, and until then the honest statement
// is that this model has no length scale in the spatial sense at all.
// ===========================================================================

// sqrt(3), fp32. Spelled as a constant so the transverse output's numerator
// is not recomputed (and possibly re-rounded differently) per call site.
inline constexpr float kSqrt3 = 1.73205080756887729353f;

// The rng domain tag every Dryden stream is derived under (core/rng.hpp's
// derivation axis "WHICH SYSTEM"). Pinned: changing it re-seeds every gust
// sequence in the determinism corpus.
inline constexpr std::string_view kDrydenDomainTag = "dryden";

// Upper clamp on the step ratio theta = h/tau. See the stability note above:
// this is anti-overflow hygiene, not a stability condition. exp(-32) ~ 1.3e-14,
// so at the clamp the filter is already white noise as far as fp32 can tell.
inline constexpr float kDrydenMaxStepRatio = 32.0f;

// ---------------------------------------------------------------------------
// Turbulence intensity, as the four discrete levels the standard's users
// actually speak in. Level 0 means "no turbulence" and is required to produce
// EXACTLY zero perturbation (not a small one).
//
// THE MAPPING, so nobody has to reverse-engineer the numbers in medium.cpp.
// MIL-F-8785C parameterizes low-altitude intensity by W_20, the mean wind
// speed at 20 ft, and sets
//
//     sigma_w = 0.1 W_20,
//     sigma_u = sigma_v = sigma_w / (0.177 + 0.000823 h)^0.4   (h in feet).
//
// The conventional light/moderate/severe values of W_20 are 15/30/45 knots.
// Evaluated at the same h = 50 m = 164.04 ft that the default scale lengths
// come from, the bracket is 0.312007 and the ratio sigma_u/sigma_w is
// 1.593436, giving (sigma_u = sigma_v, sigma_w) in m/s:
//
//     none      (0, 0)
//     light     (1.2296, 0.7717)
//     moderate  (2.4592, 1.5433)
//     severe    (3.6888, 2.3150)
//
// These are per-axis standard deviations of the gust velocity, so "severe" is
// a ~3.7 m/s 1-sigma horizontal gust -- genuinely severe for a small
// multirotor, which is the point. sigma scales linearly in W_20, so the three
// non-zero rows are exact multiples of the first.
// ---------------------------------------------------------------------------
enum class TurbulenceLevel : uint32_t {
    none = 0,
    light = 1,
    moderate = 2,
    severe = 3,
};

// ---------------------------------------------------------------------------
// DrydenParams -- the per-world turbulence configuration. PARAMETERS, not
// state: nothing here changes as the sim runs, nothing here is snapshotted by
// this task, and a step is a pure function of (state, params, h).
//
// Deliberately holds sigma and L DIRECTLY rather than a TurbulenceLevel: the
// level is an authoring convenience (dryden_params() below is its factory),
// while the runtime record is the physical quantities, so a world file can
// specify either without the runtime growing a mode. Which per-world buffer
// this record eventually lives in is the world-loading task's call; it is not
// folded into WorldParams because that layout is a std430 wire contract with
// eight reserved bytes, and this is 32.
//
// UNITS: scale_* metres, sigma_* m/s, reference_airspeed m/s.
// ---------------------------------------------------------------------------
struct DrydenParams {
    float scale_u = 200.0f;           // L_u, longitudinal scale length, m
    float scale_v = 200.0f;           // L_v, lateral scale length, m
    float scale_w = 50.0f;            // L_w, vertical scale length, m
    float sigma_u = 0.0f;             // longitudinal gust stddev, m/s
    float sigma_v = 0.0f;             // lateral gust stddev, m/s
    float sigma_w = 0.0f;             // vertical gust stddev, m/s
    float reference_airspeed = 5.0f;  // V, the frozen-turbulence airspeed, m/s
    float _reserved0 = 0.0f;          // reserved for versioned growth; must stay 0

    friend constexpr bool operator==(const DrydenParams&, const DrydenParams&) noexcept = default;
};

static_assert(std::is_standard_layout_v<DrydenParams>);
static_assert(std::is_trivially_copyable_v<DrydenParams>);
static_assert(sizeof(DrydenParams) == 32, "DrydenParams layout is a config contract");
static_assert(alignof(DrydenParams) == 4);
static_assert(offsetof(DrydenParams, scale_u) == 0);
static_assert(offsetof(DrydenParams, scale_v) == 4);
static_assert(offsetof(DrydenParams, scale_w) == 8);
static_assert(offsetof(DrydenParams, sigma_u) == 12);
static_assert(offsetof(DrydenParams, sigma_v) == 16);
static_assert(offsetof(DrydenParams, sigma_w) == 20);
static_assert(offsetof(DrydenParams, reference_airspeed) == 24);
static_assert(offsetof(DrydenParams, _reserved0) == 28);
static_assert(8 * sizeof(float) == sizeof(DrydenParams),
              "DrydenParams has implicit padding: every byte must belong to a named field");

// The default scale lengths and reference airspeed, with the level's sigmas.
// TurbulenceLevel::none yields sigma == 0 on all three axes, which is what
// makes "level none produces a bit-for-bit ConstantMedium" true rather than
// approximately true.
[[nodiscard]] DrydenParams dryden_params(TurbulenceLevel level) noexcept;

// ---------------------------------------------------------------------------
// DrydenState -- one world's filter state. THE row that lives in a registered
// array; there is no other Dryden state anywhere.
//
// A POD by construction (trivially copyable, standard layout, no implicit
// padding) so a snapshot carries it as raw bytes, exactly like rng::Stream --
// whose layout note this struct follows, including the 8-byte alignment: it is
// not (yet) an std430-mirrored array element, so it is packed dense rather
// than padded to 16-byte rows. The static_asserts below are the enforcement.
//
// WHY IT IS DECLARED HERE AND NOT IN state/layout.hpp. Layout single-sourcing
// puts SHARED PODs -- ones more than one module reads -- in layout.hpp. This
// row has exactly one reader, the Dryden medium, in the same way rng::Stream
// lives in core/rng.hpp next to its only consumer. It carries the same
// size/offset/no-implicit-padding assert discipline, which is what the
// single-sourcing rule is actually protecting; when S6's Slang generator takes
// over, this struct emigrates to the generated header with its asserts intact.
//
// Layout, five 8-byte rows:
//   row 0-1  stream (state | cached_gauss, has_cached)
//   row 2    u  | v0
//   row 3    v1 | w0
//   row 4    w1 | _p0
//
// u, v0/v1 and w0/w1 are NORMALIZED filter states (unit stationary variance,
// dimensionless) -- see section 2 of the header note. They are not gust
// velocities; dryden_turbulence() applies sigma and produces those.
// ---------------------------------------------------------------------------
struct DrydenState {
    rng::Stream stream;  // this world's gust stream; ALL of the randomness
    float u = 0.0f;      // longitudinal, first-order state
    float v0 = 0.0f;     // lateral, second-order state 0
    float v1 = 0.0f;     // lateral, second-order state 1
    float w0 = 0.0f;     // vertical, second-order state 0
    float w1 = 0.0f;     // vertical, second-order state 1
    float _p0 = 0.0f;    // reserved; also what keeps every byte named

    friend constexpr bool operator==(const DrydenState&, const DrydenState&) noexcept = default;
};

static_assert(std::is_standard_layout_v<DrydenState>, "DrydenState must be standard-layout for offsetof to be meaningful");
static_assert(std::is_trivially_copyable_v<DrydenState>, "DrydenState must be memcpy-able: snapshots copy it byte-wise");
static_assert(std::is_trivially_destructible_v<DrydenState>, "registered array slots are never individually destroyed");
static_assert(sizeof(DrydenState) == 40, "DrydenState layout is a snapshot contract");
static_assert(alignof(DrydenState) == 8, "DrydenState layout is a snapshot contract");
static_assert(offsetof(DrydenState, stream) == 0);
static_assert(offsetof(DrydenState, u) == 16);
static_assert(offsetof(DrydenState, v0) == 20);
static_assert(offsetof(DrydenState, v1) == 24);
static_assert(offsetof(DrydenState, w0) == 28);
static_assert(offsetof(DrydenState, w1) == 32);
static_assert(offsetof(DrydenState, _p0) == 36);
static_assert(sizeof(rng::Stream) + 6 * sizeof(float) == sizeof(DrydenState),
              "DrydenState has implicit padding: every byte must belong to a named field");

// ---------------------------------------------------------------------------
// Seeds `state` from the world's rng root and places the filters ON their
// stationary distribution -- five standard normals, in the pinned order
// u, v0, v1, w0, w1, straight into the five normalized states.
//
// Starting stationary rather than at zero is not cosmetic. A zero start is a
// gust-free world for the first few tau (up to 40 s at the default V), which
// is long enough that every short episode would be measuring the spin-up
// transient instead of the model; and it would force every statistical test to
// burn in. In the normalized coordinates the stationary distribution is
// N(0, I), so "start stationary" costs exactly five draws and depends on
// neither sigma nor tau -- which is the third dividend of section 2's change
// of variables.
//
// Uses WorldParams::seed (the world's documented rng seed root) with the
// "dryden" domain tag and index 0: one gust process per world.
// ---------------------------------------------------------------------------
void dryden_init(DrydenState& state, const WorldParams& world) noexcept;

// ---------------------------------------------------------------------------
// The discrete coefficients, exposed because they are the reviewable part.
//
// These are a pure function of the step ratio theta = h/tau and nothing else
// (that is what section 2's normalization bought), so they are testable in
// isolation against a high-precision reference at step ratios no statistical
// test could ever reach -- theta = 1e-6 needs ~1e10 substeps to say anything
// about a variance, and about a microsecond to say everything about a
// coefficient. test_dryden.cpp does exactly that, which is the only practical
// way to defend the conditioning claims in section 3.
// ---------------------------------------------------------------------------
struct DrydenFirstOrderCoeffs {
    float phi = 1.0f;  // exp(-theta)
    float l = 0.0f;    // sqrt(1 - phi^2), the exact one-step noise gain
};

struct DrydenSecondOrderCoeffs {
    // Phi = exp(A h) in normalized coordinates, row-major.
    float phi00 = 1.0f;
    float phi01 = 0.0f;
    float phi10 = 0.0f;
    float phi11 = 1.0f;
    // Lower Cholesky factor of Q = I - Phi Phi^T.
    float l00 = 0.0f;
    float l10 = 0.0f;
    float l11 = 0.0f;
};

// Both are TOTAL: `theta` is clamped to [0, kDrydenMaxStepRatio] on the way
// in, and a negative or non-finite argument is read as 0 (a frozen field).
[[nodiscard]] DrydenFirstOrderCoeffs dryden_first_order_coeffs(float theta) noexcept;
[[nodiscard]] DrydenSecondOrderCoeffs dryden_second_order_coeffs(float theta) noexcept;

// The step ratio theta = h / tau = h V / L, clamped to [0, kDrydenMaxStepRatio]
// and mapped to 0 for any non-positive or non-finite input (see the stability
// note). Exposed so a caller can ask "what step ratio am I actually running
// at?" without re-deriving the guards.
[[nodiscard]] float dryden_step_ratio(float h, float reference_airspeed, float scale_length) noexcept;

// ---------------------------------------------------------------------------
// One substep of the three filters. THE dryden.advance pass for this medium.
//
// Call exactly once per substep per world, BEFORE any force element samples
// the medium -- see the schedule contract in section 4 of the header note.
// `h` is the effective substep dt (dt / substeps), the same quantity
// physics::integrate_bodies() takes; this function does not own a substep loop
// any more than the integrator does.
//
// Draws exactly five gaussians from `state.stream`, in the pinned order
// n_u, n_v0, n_v1, n_w0, n_w1, unconditionally (see the determinism note).
// Reads no clock, touches no global, allocates nothing, and cannot fail:
// degenerate inputs (h <= 0, V <= 0, L <= 0, non-finite anything) freeze the
// filters at theta = 0 rather than producing a NaN.
// ---------------------------------------------------------------------------
void dryden_advance(DrydenState& state, const DrydenParams& params, float h) noexcept;

// ---------------------------------------------------------------------------
// The current gust vector, world frame, m/s. Pure read -- no draws, no state
// change; calling it twice in a substep returns the same vector.
//
// AXIS MAPPING, and its honest caveat. The Dryden triad is defined in wind
// axes: u along the airspeed vector, v lateral, w vertical. This model has no
// airspeed vector (section 1), so the triad is mapped onto FIXED world axes in
// Spade's Y-up, metres, SI frame (world/builder.hpp):
//
//     u (longitudinal) -> world +X
//     v (lateral)      -> world +Z
//     w (vertical)     -> world +Y
//
// That is exact when the mean flight direction is +X and is otherwise a
// relabelling: it swaps which horizontal axis carries the longer 200 m scale
// length. Since L_u == L_v by default the horizontal pair is isotropic anyway,
// so the only physically live distinction -- vertical (50 m) versus horizontal
// (200 m) -- is preserved for ANY heading. The aeronautical sign convention
// (w positive down) is NOT preserved; the process is zero-mean and symmetric,
// so the sign of w is not observable.
// ---------------------------------------------------------------------------
[[nodiscard]] glm::vec3 dryden_turbulence(const DrydenState& state, const DrydenParams& params) noexcept;

// ---------------------------------------------------------------------------
// DrydenMedium -- WorldParams' density and mean wind, plus the current gust.
//
// A NON-OWNING PER-WORLD VIEW, and it has to be: Medium::sample() takes a
// WorldParams row and a position, with no world id, so the binding to "which
// world's filter row" has to live in the object. ConstantMedium can be a
// single instance shared by every world precisely because it is stateless;
// this one is two pointers, constructed for one world's read --
// Simulation::sample_medium() is the host's -- and it must not outlive the
// arena the row lives in. The step itself no longer builds one: the
// dryden.sample pass writes this expression into the world's field row
// (module-API stage 3).
//
// That is also exactly the shape the S6 GPU mirror wants (medium.hpp's
// "future implementations must stay expressible as DATA" note): the row is an
// element of a per-world buffer, and on the device the world id from the
// dispatch does what this pointer does on the host. Nothing about the
// behaviour lives in host-only code.
// ---------------------------------------------------------------------------
class DrydenMedium final : public Medium {
public:
    // `state` and `params` must outlive this view. `state` is normally an
    // element of the registered DrydenState array.
    DrydenMedium(const DrydenState& state, const DrydenParams& params) noexcept
        : state_(&state), params_(&params) {}

    ~DrydenMedium() override = default;
    DrydenMedium(const DrydenMedium&) = default;
    DrydenMedium& operator=(const DrydenMedium&) = default;
    DrydenMedium(DrydenMedium&&) = default;
    DrydenMedium& operator=(DrydenMedium&&) = default;

    // Density verbatim from `params`; wind = params.wind + the current gust.
    // Position-independent (see section 4).
    [[nodiscard]] MediumSample sample(const WorldParams& params, glm::vec3 pos) const override;

    [[nodiscard]] const DrydenState& state() const noexcept { return *state_; }
    [[nodiscard]] const DrydenParams& params() const noexcept { return *params_; }

private:
    const DrydenState* state_;
    const DrydenParams* params_;
};

}  // namespace spade
