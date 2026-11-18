// A fixture for scripts/gcc-check-selftest.sh. Not part of any target: nothing builds it, and no leg ever compiled it,
// so it is the "new file" case of `scripts/gcc-check.sh --compile`.
//
// It holds a warning only the optimiser finds: -Wmaybe-uninitialized. `gcc-check.sh` without --compile (-fsyntax-only)
// passes this file; the leg's own line (-O3 -Wall -Wextra -Wpedantic -Werror) fails it. The self-test expects both.
namespace {
int pick(int n) {
    int x;
    if (n > 3) {
        x = n;
    }
    return x;
}
}  // namespace

int o2_maybe_uninitialized(int n) { return pick(n); }
