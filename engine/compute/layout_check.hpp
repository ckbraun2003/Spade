#pragma once

#include <cstddef>

// ---------------------------------------------------------------------------
// The compute backend's D9 layout-check surface -- one function, and it is
// deliberately the only thing the check exposes.
//
// The check ITSELF is entirely compile-time: compute/layout_check.cpp includes
// the generated layout_check.gen.hpp, whose static_asserts compare every field
// offset and every struct size of the C++ state rows against the offsets
// slangc REPORTED for engine/shaders/shared/layouts.slang. Drift is a build
// failure of spade_compute; there is nothing to call at runtime and nothing a
// caller could do about it if there were.
//
// What a caller CAN usefully ask is how much was checked, which is what this
// function answers. A generator bug that emitted an empty or truncated header
// would produce a build that passes every assert it contains -- vacuously.
// tests/test_slang_layouts.cpp pins the count against the structs the registry
// is supposed to mirror, so "checked nothing, successfully" cannot pass.
//
// This header is part of spade_compute's installed surface (engine/
// CMakeLists.txt installs compute/*.hpp), and it names no Vulkan and no
// generated type, so including it costs a consumer nothing.
// ---------------------------------------------------------------------------

namespace spade::compute {

// Number of Slang structs whose layout the generated header asserts against a
// C++ counterpart. Constant for a given build; see the note above for why it
// is worth asking.
[[nodiscard]] std::size_t layout_check_mirrored_struct_count() noexcept;

}  // namespace spade::compute
