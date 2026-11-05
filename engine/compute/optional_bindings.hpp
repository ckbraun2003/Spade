// compute/optional_bindings.hpp -- the fixed bindings of optional modules (module API stage 6; plan decisions 7 and
// "What stage 6 unblocks"). Vulkan-free, so sim/ reads the switches and the mirror reads the names.
//
// THE PATTERN. A kernel binds statically, and the descriptor set is written once at create(), so a binding a kernel
// reads must hold SOMETHING in every Simulation, including one whose module set has no module owning it. A module that
// is optional (it is not in the standard set) therefore has a FIXED BINDING in bindings.slang's set 0:
//   * when the set holds the module's array, the state mirror binds that array's buffer there, as for any bound array;
//   * when it does not, the mirror binds a zeroed buffer of `dummy_bytes` there, and the recorder switches the kernel's
//     read of it OFF through a bit in PassParams::switches, so the kernel computes today's bits and never indexes the
//     dummy (it is one row, not one per body).
// The first user is the translation lock (`lock_rows`, binding 29, read by integrate.slang). Stage 7 adds a row per
// consumer: the per-point row readers, the part table's mirror, the per-rotor model row.
//
// A binding that is NOT in this table and is unbound still fails the mirror's census
// (GpuStateMirrorTest.DescriptorSetBindsEveryRegistryBinding), so a forgotten standard binding is still caught.
//
// This table names an ARRAY (a registered walk entry). An optional configuration table or scratch resolves by its
// declared binding number instead, which is a few lines in the mirror and belongs to the first one that needs it.
#pragma once

#include <cstdint>
#include <string_view>

namespace spade::compute {

// The bits of PassParams::switches (layouts.slang), one per optional binding.
inline constexpr uint32_t kSwitchLockRows = 1u << 0;

struct OptionalBinding {
    std::string_view name;     // the registered array that owns the binding when the set holds it
    uint32_t switch_bit;       // the PassParams::switches bit a kernel tests before it reads the binding
    uint32_t dummy_bytes;      // the zeroed buffer bound when the set holds no such array
};

inline constexpr OptionalBinding kOptionalBindings[] = {
    {"lock_rows", kSwitchLockRows, 16},  // physics::LockRow, one row
};

}  // namespace spade::compute
