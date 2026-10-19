#pragma once

// ---------------------------------------------------------------------------
// The engine's build stamp: the commit its sources were at, and whether their
// tracked files had uncommitted changes, both taken when the engine was BUILT
// (cmake/stamp_identity.cmake, run before every build of spade_core). Nothing
// asks git when the binary runs, so a host needs no git to name what it runs,
// and a later checkout never changes what a built binary says. Analysis runs
// are named by it (analysis/run.hpp), and so are flow_movie's cards.
//
// Where the commit comes from, first match wins:
//   - git, when the sources are the top level of a git work tree;
//   - cmake/archive_commit.txt, which `git archive` fills with the commit it
//     archives (export-subst, .gitattributes), so a build from an archive (the
//     Docker leg, a source tarball) names its commit too, never dirty;
//   - "unknown", never dirty: sources that are neither (a copy made by hand, or
//     a directory nested inside another repository, which must not borrow that
//     repository's commit) cannot say what they are.
// ---------------------------------------------------------------------------

#include <string_view>

namespace spade {

struct BuildIdentity {
    std::string_view commit;  // 40 lowercase hex, or "unknown"; static storage
    bool dirty = false;       // tracked files had uncommitted changes at build time
};

[[nodiscard]] BuildIdentity build_identity() noexcept;

}  // namespace spade
