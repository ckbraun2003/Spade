#pragma once

// ---------------------------------------------------------------------------
// core/detail/flush_subnormals.hpp -- x86 MXCSR's subnormal flush (FTZ and
// DAZ) for a scope. INTERNAL, like the rest of core/detail/: not installed
// (engine/CMakeLists.txt), so <xmmintrin.h> never reaches a consumer.
//
// ONE HOME (TD-9). It began in tests/test_fp32_math.cpp, for the flush-mode
// test. Core's exact-division battery (tests/fp32_exact_battery.hpp: the
// divss/sqrtss reference) and Physics' CPU airflow step (the airflow FTZ plan,
// 2026-10-22, D3-D5) need the same scope, so it lives here, once. A caller
// includes this file; nobody copies the class.
//
// THE WORD INSIDE IS FIXED (D3): kFlushSubnormalsWord, not the caller's word
// with two bits added. A caller's rounding mode would otherwise change the
// bits, and an unmasked underflow would switch the flush off (FTZ acts only
// while underflow is masked). On exit the saved word is restored WHOLE (D4):
// the scope's sticky flags are discarded with it. No static state, so each
// thread's MXCSR is its own.
//
// THE WRAPPER RULE. Neither MSVC (/fp:precise) nor gcc (no -frounding-math)
// treats MXCSR as an input to float arithmetic: either may move a
// register-only float op across _mm_setcsr. So a guarded entry point is a thin
// wrapper -- the scope, then ONE call to a body marked SPADE_NOINLINE, passing
// its arguments through -- and the wrapper itself does no float work. Every
// float op then runs inside an opaque call, strictly between the two MXCSR
// writes.
//
// x86-64 ONLY (D5). A target without this flush cannot meet the bit-equal
// rule, so the build stops there rather than compiling a no-op silently.
// Whoever adds such a target decides its flush then.
// ---------------------------------------------------------------------------

#if !(defined(__x86_64__) || defined(_M_X64))
#error "core/detail/flush_subnormals.hpp is x86-64 only: without MXCSR's FTZ/DAZ no target meets the bit-equal rule"
#endif

#include <xmmintrin.h>  // _mm_getcsr, _mm_setcsr

// A body the compiler must not inline into its wrapper (the wrapper rule
// above). Spelled first in the declaration: SPADE_NOINLINE inline void f(...).
#if defined(_MSC_VER)
#define SPADE_NOINLINE __declspec(noinline)
#else
#define SPADE_NOINLINE [[gnu::noinline]]
#endif

namespace spade::math {

// The MXCSR word inside the scope: FTZ (bit 15) and DAZ (bit 6) set, round to
// nearest (bits 13-14 clear), all six exceptions masked (bits 7-12), no sticky
// flag (bits 0-5). The default word is 0x1F80, so from it this is 0x1F80 with
// FTZ and DAZ added.
inline constexpr unsigned int kFlushSubnormalsWord = 0x9FC0u;

// Sets MXCSR to kFlushSubnormalsWord for its lifetime, and restores the
// previous MXCSR on every exit.
class ScopedFlushSubnormals {
public:
    ScopedFlushSubnormals() noexcept : saved_(_mm_getcsr()) { _mm_setcsr(kFlushSubnormalsWord); }
    ~ScopedFlushSubnormals() { _mm_setcsr(saved_); }
    ScopedFlushSubnormals(const ScopedFlushSubnormals&) = delete;
    ScopedFlushSubnormals& operator=(const ScopedFlushSubnormals&) = delete;

private:
    unsigned int saved_;
};

}  // namespace spade::math
