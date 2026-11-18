// A fixture for scripts/gcc-check-selftest.sh: the control for o2_maybe_uninitialized.cpp. A new file with no warning,
// which `scripts/gcc-check.sh --compile` must pass, so the self-test cannot be satisfied by a check that fails everything.
int o2_clean(int n) { return n + 1; }
