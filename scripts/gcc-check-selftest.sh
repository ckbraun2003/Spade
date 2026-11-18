#!/usr/bin/env bash
# Self-test of `scripts/gcc-check.sh --compile` on a NEW file (one no leg has compiled): it must catch a warning only the
# optimiser finds, and must pass a clean new file. The fixtures are tests/gcc_check_fixtures/; nothing builds them.
#
#   scripts/gcc-check-selftest.sh          from the worktree root, in Git Bash; it needs an idle leg volume, as gcc-check.sh does
#
# Three runs of gcc-check.sh, about a minute: the fixture with --compile must FAIL on -Wmaybe-uninitialized and say it was
# compiled with a sibling's command line; the clean fixture with --compile must pass; the fixture with the plain check
# (-fsyntax-only) must pass, which is why --compile exists. GCC_CHECK_BUILD, GCC_CHECK_VOLUME pass through.
# Exit 0 when all three hold, 1 otherwise, 2 for usage.
set -u
cd "$(dirname "$0")/.." || exit 2
[ -d engine ] && [ -d tests/gcc_check_fixtures ] || { echo "gcc-check-selftest: run it from a worktree with tests/gcc_check_fixtures/" >&2; exit 2; }
fix=tests/gcc_check_fixtures
bad=0

out=$(scripts/gcc-check.sh --compile "$fix/o2_maybe_uninitialized.cpp" 2>&1); rc=$?
echo "$out" | grep -E '^gcc |maybe-uninitialized'
if [ "$rc" -eq 1 ] && echo "$out" | grep -q 'maybe-uninitialized' && echo "$out" | grep -q "^gcc FAIL: $fix/o2_maybe_uninitialized.cpp (compiled with .*command line"; then
    echo "selftest ok:   --compile catches the O2-only warning in a new file"
else
    echo "selftest FAIL: --compile did not catch the O2-only warning in a new file (exit $rc)"; bad=1
fi

out=$(scripts/gcc-check.sh --compile "$fix/o2_clean.cpp" 2>&1); rc=$?
echo "$out" | grep -E '^gcc '
if [ "$rc" -eq 0 ] && echo "$out" | grep -q "^gcc ok: *$fix/o2_clean.cpp (compiled with .*command line"; then
    echo "selftest ok:   --compile passes a clean new file"
else
    echo "selftest FAIL: --compile did not pass a clean new file (exit $rc)"; bad=1
fi

out=$(scripts/gcc-check.sh "$fix/o2_maybe_uninitialized.cpp" 2>&1); rc=$?
echo "$out" | grep -E '^gcc '
if [ "$rc" -eq 0 ]; then
    echo "selftest ok:   the plain check passes the fixture (the warning is optimiser-only)"
else
    echo "selftest FAIL: the plain check fails the fixture, so it is no longer an optimiser-only warning (exit $rc)"; bad=1
fi
exit $bad
