#!/usr/bin/env bash
# Self-test of `scripts/gcc-check.sh --compile` on a NEW file (one no leg has compiled): it must catch a warning only the
# optimiser finds, and must pass a clean new file. The fixtures are tests/gcc_check_fixtures/; nothing builds them.
#
#   scripts/gcc-check-selftest.sh          from the worktree root, in Git Bash; it needs an idle leg volume, as gcc-check.sh does
#
# Five runs of gcc-check.sh, about two minutes: the fixture with --compile must FAIL on -Wmaybe-uninitialized and say it was
# compiled with a sibling's command line; the clean fixture with --compile must pass; the fixture with the plain check
# (-fsyntax-only) must pass, which is why --compile exists. Then code under tools/fc_link, which includes its own headers as
# "fc_link/...": a header and a .cpp (fc_link_probe.*, copied to tools/fc_link/gcc_check_probe.* for the runs and removed after)
# must pass --compile and the plain check, so the tools/ include root cannot go missing. GCC_CHECK_BUILD, GCC_CHECK_VOLUME pass
# through. Exit 0 when all five hold, 1 otherwise, 2 for usage.
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
# Code under tools/fc_link (Interface's link, Physics' autopilot map) includes its headers as "fc_link/..." from the tools/ include root. The fixtures
# are copied in for the runs and removed after (never over a file that is there).
probe=tools/fc_link
p_hpp=$probe/gcc_check_probe.hpp
p_cpp=$probe/gcc_check_probe.cpp
if [ -e "$p_hpp" ] || [ -e "$p_cpp" ]; then
    echo "gcc-check-selftest: $p_hpp or $p_cpp exists; not overwriting it" >&2; exit 2
fi
made_dir=0
[ -d "$probe" ] || { mkdir -p "$probe" || exit 2; made_dir=1; }
cleanup() { rm -f "$p_hpp" "$p_cpp"; [ "$made_dir" -eq 0 ] || rmdir "$probe" 2>/dev/null; }
trap cleanup EXIT
cp "$fix/fc_link_probe.hpp" "$p_hpp" && cp "$fix/fc_link_probe.cpp" "$p_cpp" || exit 2

out=$(scripts/gcc-check.sh --compile "$p_hpp" "$p_cpp" 2>&1); rc=$?
echo "$out" | grep -E '^gcc |No such file'
if [ "$rc" -eq 0 ] && echo "$out" | grep -q "^gcc ok: *$p_hpp" && echo "$out" | grep -q "^gcc ok: *$p_cpp (compiled with .*command line"; then
    echo "selftest ok:   --compile checks a tools/fc_link header and .cpp"
else
    echo "selftest FAIL: --compile did not pass a tools/fc_link header and .cpp (exit $rc)"; bad=1
fi

out=$(scripts/gcc-check.sh "$p_hpp" "$p_cpp" 2>&1); rc=$?
echo "$out" | grep -E '^gcc |No such file'
if [ "$rc" -eq 0 ] && echo "$out" | grep -q "^gcc ok: *$p_hpp" && echo "$out" | grep -q "^gcc ok: *$p_cpp"; then
    echo "selftest ok:   the plain check passes a tools/fc_link header and .cpp"
else
    echo "selftest FAIL: the plain check did not pass a tools/fc_link header and .cpp (exit $rc)"; bad=1
fi
exit $bad
