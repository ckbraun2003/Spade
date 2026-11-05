// core/grade.hpp -- what a module claims about its implementation on one
// backend (module API stage 5, Task 3; spec section 8, TD-14, L3, L4).
//
// A grade is a claim, never a measurement: create() checks it against the
// backend and a requested minimum, and nothing here runs a kernel.
//   reference    the CPU implementation is the source of truth; its goldens are
//                CPU digests (TD-1, TD-12). Declared on the CPU only: the GPU is
//                never a golden source (L4).
//   banded       every quantity the module writes stays inside its pinned TD-14
//                band of the CPU reference (a band may be zero), measured on each
//                device of record. Declared on Vulkan, and only beside a CPU
//                `reference` to band against.
//   best_effort  it runs, with no golden and no band.
//   absent       there is no implementation on that backend: a declaration, never
//                a world's grade, and create() always refuses it there.
// The values are ordered, higher is stronger; a set's grade on a backend is its
// weakest module's (L3). Grades are not spelt into any identity.
//
// Dependency-free by design (stage 5 layering): nothing from sim/, objects/ or
// compute/ is included, so Rendering's renderer grades (RND-3) can use it too.
#pragma once

#include <cstdint>
#include <string_view>

namespace spade {

enum class Grade : uint8_t { absent = 0, best_effort = 1, banded = 2, reference = 3 };

// "absent", "best-effort", "banded", "reference"; "" for a value outside the enum.
[[nodiscard]] constexpr std::string_view grade_name(Grade g) noexcept {
    switch (g) {
        case Grade::absent: return "absent";
        case Grade::best_effort: return "best-effort";
        case Grade::banded: return "banded";
        case Grade::reference: return "reference";
    }
    return "";
}

}  // namespace spade
