#!/usr/bin/env bash
# The Docker leg's in-container steps (TD-11, TD-12). scripts\docker-leg.ps1
# runs the copy from the archive of the commit under test as
#
#   bash /out/docker-leg.sh <all|configure|build|test|consumer> [--jobs N]
#
# and its own copy (docker-leg-driver.sh) for the steps that arrange the
# volume, since an older commit's copy may predate them:
#
#   bash /out/docker-leg-driver.sh seed [--from VOLUME]
#   bash /out/docker-leg-driver.sh sync
#
# with the run's volume (spade-docker-leg-<run>) at /leg and the host's
# build-docker\<short-sha>\ at /out:
#
#   /out/src.tar   the commit's files, LF (git -c core.autocrlf=false archive)
#   /leg/src       those files, synced so unchanged files keep their mtimes
#   /leg/commit    the commit's full hash
#   /leg/previous  what /leg/commit held before the last sync
#   /leg/seed      how the volume began: empty, or a copy of which volume at
#                  which commit
#   /leg/build     the Release tree, every option at its default except V1;
#                  its dependencies stay in /leg/build/_deps between runs
#   /leg/consumer  scripts/consumer-smoke.sh's work directory (Interface)
#
# `seed` runs once, on a new volume. With --from it copies build, src, consumer
# and commit from the idle volume mounted at /seed, keeping every mtime: the
# outputs stay newer than the sources they were built from, and the sync that
# follows gives each file the new commit changed a fresh mtime, so ninja
# rebuilds exactly that. Without --from the volume starts empty.
#
# `consumer` configures and builds before its two calls: consumer ON installs
# from /leg/build, which must be the commit under test, not whatever commit
# last built there. `all` adds the test parts to that.
#
# A part that fails does not stop the parts that don't depend on it. The exit
# code is 0 only if every part of the step passed; summary.txt lists each part.
set -u

usage() {
    echo "usage: docker-leg.sh <sync|all|configure|build|test|consumer> [--jobs N]" >&2
    echo "       docker-leg.sh seed [--from VOLUME]" >&2
    exit 2
}

[ $# -ge 1 ] || usage
step=$1
shift
jobs=1
from=
while [ $# -gt 0 ]; do
    case $1 in
        --jobs) [ $# -ge 2 ] || usage; jobs=$2; shift 2 ;;
        --from) [ $# -ge 2 ] && [ "$step" = seed ] || usage; from=$2; shift 2 ;;
        *) usage ;;
    esac
done
case $step in seed|sync|all|configure|build|test|consumer) ;; *) usage ;; esac
case $jobs in ''|*[!0-9]*) usage ;; esac

# The roots are overridable only so the step logic can be exercised outside a
# container (with stand-in tools); the driver never sets them.
root=${DOCKER_LEG_ROOT:-/leg}
out=${DOCKER_LEG_OUT:-/out}
src=$root/src
build=$root/build
seed_root=${DOCKER_LEG_SEED:-/seed}

# ---- seed: /seed -> /leg, once, on a new volume -----------------------------
if [ "$step" = seed ]; then
    if [ -e "$src" ] || [ -e "$build" ]; then
        echo "docker-leg: $root already holds a tree; a volume is seeded only when new" >&2
        exit 1
    fi
    if [ -z "$from" ]; then
        origin="empty (no seed)"
    elif [ ! -f "$seed_root/build/CMakeCache.txt" ]; then
        origin="empty ($from had no configured build to copy)"
    else
        set -e
        for d in build src consumer commit; do
            if [ -e "$seed_root/$d" ]; then cp -a "$seed_root/$d" "$root/"; fi
        done
        set +e
        origin="a copy of $from at $(cat "$seed_root/commit" 2>/dev/null || echo 'an unrecorded commit')"
    fi
    printf '%s\n' "$origin" > "$root/seed"
    echo "docker-leg: the volume starts as $origin"
    exit 0
fi

# ---- sync: /out/src.tar -> /leg/src ----------------------------------------
# rsync without -t: a file whose content is unchanged is skipped and keeps its
# old mtime, so ninja rebuilds only what the new commit changed. A file it does
# write gets the clock's time, which must be later than every output in the
# build tree, or ninja keeps the stale object. A clock that stepped back (the
# WSL VM after the host sleeps) or a seed built under a later clock breaks
# that, so those files are then stamped one second past the newest output.
if [ "$step" = sync ]; then
    set -e
    rm -rf "$root/stage"
    mkdir -p "$root/stage" "$src"
    tar -xf "$out/src.tar" -C "$root/stage"
    commit=$(cat "$out/commit")
    rsync -rl --checksum --delete --out-format='%n' "$root/stage/" "$src/" > "$root/synced.txt"
    rm -rf "$root/stage"
    newest=
    if [ -d "$build" ]; then newest=$(find "$build" -type f -printf '%Ts\n' | sort -n | tail -n 1); fi
    now=$(date +%s)
    if [ "${newest:-0}" -ge "$now" ]; then
        stamped=0
        while IFS= read -r f; do
            if [ -f "$src/$f" ]; then touch -d "@$((newest + 1))" "$src/$f"; stamped=$((stamped + 1)); fi
        done < "$root/synced.txt"
        echo "docker-leg: the newest output in $build ($newest) is not older than the clock ($now);" \
             "stamped the $stamped file(s) this sync wrote at $((newest + 1))"
    fi
    if [ -f "$root/commit" ]; then cp "$root/commit" "$root/previous"; else echo nothing > "$root/previous"; fi
    printf '%s\n' "$commit" > "$root/commit"
    echo "docker-leg: synced $commit into $src"
    exit 0
fi

exec > >(tee -a "$out/leg.log") 2>&1
tee_pid=$!

commit=$(cat "$root/commit" 2>/dev/null || echo unknown)
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
    # The viewer-trajectory guard checks a short prefix in the gate; here it
    # checks every tick of every golden, on gcc (INT-4, TD-12).
    SPADE_FULL_VIEWER_TRAJECTORIES=1 \
    ctest --test-dir "$build" -L spade -LE gpu --output-on-failure --no-tests=error \
        --output-junit "$out/ctest-junit.xml" 2>&1 | tee "$out/ctest.log"
    return "${PIPESTATUS[0]}"
}

# Rendering's agreement matrix prints one `agreement: ...` line per case on
# every run (d and d_probe at %.17g, toolchain named), but ctest shows a
# passing test's output only with -V. So its cases are re-run verbosely to
# collect the lines for the band file's gcc-release attestation. A commit
# without the matrix has nothing to collect; one with it must give a line per
# registered case (TD-5). Whether the cases pass is the test part's business.
agreement_pattern='^Active/AgreementMatrix\.'
do_agreement() {
    local registered found
    registered=$(ctest --test-dir "$build" -N -R "$agreement_pattern" | sed -n 's/^Total Tests: //p')
    if [ "${registered:-0}" = 0 ]; then
        : > "$out/agreement.txt"
        agreement_note="no agreement matrix in this commit"
        echo "docker-leg: $agreement_note"
        return 0
    fi
    ctest --test-dir "$build" -R "$agreement_pattern" -V 2>&1 \
        | sed 's/^[0-9]*: //' | grep '^agreement: ' > "$out/agreement.txt"
    found=$(wc -l < "$out/agreement.txt")
    agreement_note="$found line(s) for $registered registered case(s) (agreement.txt)"
    echo "docker-leg: $agreement_note"
    [ "$found" -eq "$registered" ]
}

# The consumer half is Interface's recipe; a commit without it fails (TD-5).
do_consumer() {
    local mode=$1
    local smoke=$src/scripts/consumer-smoke.sh
    if [ ! -f "$smoke" ]; then
        echo "docker-leg: FAIL consumer: scripts/consumer-smoke.sh is not in $commit"
        return 1
    fi
    local args=(--vulkan "$mode" --work "$root/consumer" --jobs "$jobs")
    [ "$mode" = ON ] && args+=(--from-build "$build")
    [ -d "$build/_deps" ] && args+=(--deps "$build/_deps")
    # The SL2b sandbox stage, when this commit's recipe offers it: its --help
    # lists `--sandbox` (Interface keeps that line for this). An older commit
    # runs without it, and the summary says so either way.
    if bash "$smoke" --help 2>/dev/null | grep -q -- '--sandbox'; then
        args+=(--sandbox)
        sandbox_note="on (consumer-smoke.sh --sandbox)"
    else
        sandbox_note="not offered by this commit's consumer-smoke.sh"
    fi
    bash "$smoke" "${args[@]}" || return
    # The measurement build's install guard (SPADE_MEASURE_UNPINNED_DENORMS):
    # the installed spade_compute must carry no trace of it. Only the ON
    # prefix has spade_compute. A commit without the scanner says so.
    if [ "$mode" = ON ]; then
        local scan=$src/scripts/denorm-marker-scan.sh
        if [ ! -f "$scan" ]; then
            denorm_note="not in this commit (scripts/denorm-marker-scan.sh)"
        elif bash "$scan" "$root/consumer/prefix-vkon"; then
            denorm_note="clean: the consumer ON prefix's spade_compute and headers carry no marker"
        else
            denorm_note="FAIL: denorm-marker-scan.sh on the consumer ON prefix (leg.log)"
            return 1
        fi
    fi
}

echo "docker-leg: commit $commit, step $step, jobs $jobs"
echo "docker-leg: $(gcc-13 --version | head -n 1)"
echo "docker-leg: $(cmake --version | head -n 1)"
echo "docker-leg: memory limit $(cat /sys/fs/cgroup/memory.max 2>/dev/null || echo unknown), $(nproc) CPUs visible"
echo "docker-leg: started $(date -u +%Y-%m-%dT%H:%M:%SZ)"

gpu_excluded=
agreement_note=
sandbox_note=
denorm_note=
t_start=$SECONDS

# all and consumer chain configure -> build -> what needs a built tree.
chained=no
case $step in all|consumer) chained=yes ;; esac

if [ "$chained" = yes ] || [ "$step" = configure ]; then
    run_part configure do_configure
fi
if [ "$chained" = yes ] || [ "$step" = build ]; then
    if [ "$chained" = yes ] && ! passed configure; then block build configure
    else run_part build do_build; fi
fi
if [ "$step" = all ] || [ "$step" = test ]; then
    if [ "$step" = all ] && ! passed build; then block test build
    else run_part test do_test; fi
    if [ "$step" = all ] && ! passed build; then block agreement build
    else run_part agreement do_agreement; fi
fi
if [ "$chained" = yes ]; then
    if ! passed build; then block "consumer ON" build
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
    echo "disk        $(df -h "$root" | awk 'NR==2 {print $4 " free in the VM, " $5 " used"}')"
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
        echo "viewer      full-length trajectories (SPADE_FULL_VIEWER_TRAJECTORIES=1)"
        echo "registered  $(wc -l < "$out/tests.txt") test names (tests.txt)"
        echo "agreement   ${agreement_note:-not collected}"
        sed -n '/^The following tests did not run:/,/^$/p; /^The following tests FAILED:/,/^$/p' "$out/ctest.log"
    fi
    if [ -n "$sandbox_note" ]; then
        echo "sandbox     $sandbox_note"
    fi
    if [ -n "$denorm_note" ]; then
        echo "denorm      $denorm_note"
    fi
} > "$summary"
echo
cat "$summary"

# Let tee drain before the container exits, or the log loses its tail.
exec >&- 2>&-
wait "$tee_pid"
[ "$result" = PASS ]
