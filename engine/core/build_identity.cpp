// The engine's build stamp (core/build_identity.hpp).
#include "core/build_identity.hpp"

// Generated in the build tree by cmake/stamp_identity.cmake before every build
// of spade_core; this file is its only reader.
#include "spade_build_stamp.hpp"

namespace spade {

BuildIdentity build_identity() noexcept { return BuildIdentity{SPADE_BUILD_COMMIT, SPADE_BUILD_DIRTY != 0}; }

}  // namespace spade
