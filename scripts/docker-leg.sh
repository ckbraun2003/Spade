#!/usr/bin/env bash
# The Docker leg's in-container steps (TD-11, TD-12). scripts\docker-leg.ps1
# runs this copy, taken from the archive of the commit under test, as
#
#   bash /out/docker-leg.sh <sync|all|configure|build|test|consumer> [--jobs N]
#
# with the spade-docker-leg volume at /leg and the host's
# build-docker\<short-sha>\ at /out:
#
#   /out/src.tar   the commit's files, LF (git -c core.autocrlf=false archive)
#   /leg/src       those files, synced so unchanged files keep their mtimes
#   /leg/commit    the commit's full hash
#   /leg/build     the Release tree, every option at its default except V1;
#                  its dependencies stay in /leg/build/_deps between runs
#   /leg/consumer  scripts/consumer-smoke.sh's work directory (Interface)
#
# A part that fails does not stop the parts that don't depend on it. The exit
# code is 0 only if every part of the step passed; summary.txt lists each part.
set -u

usage() {
    echo "usage: docker-leg.sh <sync|all|configure|build|test|consumer> [--jobs N]" >&2
    exit 2
}

[ $# -ge 1 ] || usage
step=$1
shift
jobs=1
while [ $# -gt 0 ]; do
    case $1 in
        --jobs) [ $# -ge 2 ] || usage; jobs=$2; shift 2 ;;
        *) usage ;;
    esac
done
case $step in sync|all|configure|build|test|consumer) ;; *) usage ;; esac
case $jobs in ''|*[!0-9]*) usage ;; esac

src=/leg/src
build=/leg/build
out=/out

# ---- sync: /out/src.tar -> /leg/src ----------------------------------------
# rsync without -t: a file whose content is unchanged is skipped and keeps its
# old mtime, so ninja rebuilds only what the new commit changed.
if [ "$step" = sync ]; then
    set -e
    rm -rf /leg/stage
    mkdir -p /leg/stage "$src"
    tar -xf "$out/src.tar" -C /leg/stage
    commit=$(cat "$out/commit")
    rsync -rl --checksum --delete /leg/stage/ "$src/"
    rm -rf /leg/stage
    printf '%s\n' "$commit" > /leg/commit
    echo "docker-leg: synced $commit into $src"
    exit 0
fi

exec > >(tee -a "$out/leg.log") 2>&1
tee_pid=$!

commit=$(cat /leg/commit 2>/dev/null || echo unknown)
summary=$out/summary.txt
declare -A status seconds
parts=()

# run_part NAME COMMAND... : runs COMMAND, records PASS or FAIL(exit N).
run_part() {
    local name=$1
    shift
    local t0=$SECONDS
    echo
    echo "docker-leg: ---- $name ----"
    "$@"
    local rc=$?
    seconds[$name]=$((SECONDS - t0))
    if [ $rc -eq 0 ]; then status[$name]=PASS; else status[$name]="FAIL (exit $rc)"; fi
    parts+=("$name")
    echo "docker-leg: $name: ${status[$name]} in ${seconds[$name]} s"
    return $rc
}

# block NAME BY : records that NAME could not run because BY failed.
block() {
    status[$1]="BLOCKED by $2"
    seconds[$1]=0
    parts+=("$1")
    echo "docker-leg: $1: not run, $2 failed"
}

passed() { [ "${status[$1]:-}" = PASS ]; }

do_configure() {
    cmake -S "$src" -B "$build" -G Ninja \
        -DCMAKE_BUILD_TYPE=Release \
        -DSPADE_BUILD_V1=OFF \
        -DFETCHCONTENT_UPDATES_DISCONNECTED=ON
}

# -k 0 keeps going past a failed translation unit, so one run lists every one.
do_build() {
    cmake --build "$build" --parallel "$jobs" -- -k 0 2>&1 | tee "$out/build.log"
    local rc=${PIPESTATUS[0]}
    # Repo-relative file:line for every error, for routing to the owning realm.
    grep -E "^($src/)?[^ :]+:[0-9]+:[0-9]+: (fatal )?error:" "$out/build.log" \
        | sed "s|^$src/||" | sort -u > "$out/build-errors.txt"
    grep -E '^FAILED: ' "$out/build.log" | sort -u >> "$out/build-errors.txt"
    return "$rc"
}

do_test() {
    ctest --test-dir "$build" -N -L spade | sed -n 's/^ *Test *#[0-9]*: //p' > "$out/tests.txt"
    gpu_excluded=$(ctest --test-dir "$build" -N -L gpu | sed -n 's/^Total Tests: //p')
    ctest --test-dir "$build" -L spade -LE gpu --output-on-failure --no-tests=error \
        --output-junit "$out/ctest-junit.xml" 2>&1 | tee "$out/ctest.log"
    return "${PIPESTATUS[0]}"
}

# The consumer half is Interface's recipe; a commit without it fails (TD-5).
do_consumer() {
    local mode=$1
    local smoke=$src/scripts/consumer-smoke.sh
    if [ ! -f "$smoke" ]; then
        echo "docker-leg: FAIL consumer: scripts/consumer-smoke.sh is not in $commit"
        return 1
    fi
    local args=(--vulkan "$mode" --work /leg/consumer --jobs "$jobs")
    [ "$mode" = ON ] && args+=(--from-build "$build")
    [ -d "$build/_deps" ] && args+=(--deps "$build/_deps")
    bash "$smoke" "${args[@]}"
}

echo "docker-leg: commit $commit, step $step, jobs $jobs"
echo "docker-leg: $(gcc-13 --version | head -n 1)"
echo "docker-leg: $(cmake --version | head -n 1)"
echo "docker-leg: memory limit $(cat /sys/fs/cgroup/memory.max 2>/dev/null || echo unknown), $(nproc) CPUs visible"
echo "docker-leg: started $(date -u +%Y-%m-%dT%H:%M:%SZ)"

gpu_excluded=
t_start=$SECONDS

if [ "$step" = all ] || [ "$step" = configure ]; then
    run_part configure do_configure
fi
if [ "$step" = all ] || [ "$step" = build ]; then
    if [ "$step" = all ] && ! passed configure; then block build configure
    else run_part build do_build; fi
fi
if [ "$step" = all ] || [ "$step" = test ]; then
    if [ "$step" = all ] && ! passed build; then block test build
    else run_part test do_test; fi
fi
if [ "$step" = all ] || [ "$step" = consumer ]; then
    if [ "$step" = all ] && ! passed build; then block "consumer ON" build
    else run_part "consumer ON" do_consumer ON; fi
    run_part "consumer OFF" do_consumer OFF
fi

# ---- summary ----------------------------------------------------------------
result=PASS
for p in "${parts[@]}"; do passed "$p" || result=FAIL; done
{
    echo "docker leg: $result"
    echo "commit      $commit"
    echo "step        $step, jobs $jobs, $((SECONDS - t_start)) s"
    echo "toolchain   $(gcc-13 --version | head -n 1); $(cmake --version | head -n 1)"
    echo "memory      limit $(cat /sys/fs/cgroup/memory.max 2>/dev/null || echo unknown), peak $(cat /sys/fs/cgroup/memory.peak 2>/dev/null || echo unknown) bytes"
    echo "disk        $(df -h /leg | awk 'NR==2 {print $4 " free in the VM, " $5 " used"}')"
    for p in "${parts[@]}"; do
        printf '%-13s %s, %s s\n' "$p" "${status[$p]}" "${seconds[$p]}"
    done
    if [ -s "$out/build-errors.txt" ] && ! passed build; then
        echo "build errors (build-errors.txt):"
        sed 's/^/  /' "$out/build-errors.txt"
    fi
    if [ -f "$out/ctest.log" ] && [ -n "${status[test]:-}" ]; then
        echo "tests       $(grep -E 'tests passed, [0-9]+ tests failed out of' "$out/ctest.log" | tail -n 1)"
        echo "gpu         ${gpu_excluded:-?} excluded with -LE gpu (TD-13)"
        echo "registered  $(wc -l < "$out/tests.txt") test names (tests.txt)"
        sed -n '/^The following tests did not run:/,/^$/p; /^The following tests FAILED:/,/^$/p' "$out/ctest.log"
    fi
} > "$summary"
echo
cat "$summary"

# Let tee drain before the container exits, or the log loses its tail.
exec >&- 2>&-
wait "$tee_pid"
[ "$result" = PASS ]
