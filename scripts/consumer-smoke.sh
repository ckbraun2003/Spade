#!/usr/bin/env bash
# scripts/consumer-smoke.sh -- install Spade, then build and run tests/consumer
# against the installed prefix, for one SPADE_VULKAN mode.
#
# This is the consumer half of the self-run Docker leg (TD-11): Linux, gcc,
# CPU-only, release. Interface owns this recipe; Test/Docs owns the leg, the
# image and when it runs. The leg calls it once per mode:
#
#   scripts/consumer-smoke.sh --vulkan ON  --work /work [--from-build DIR]
#   scripts/consumer-smoke.sh --vulkan OFF --work /work
#
# WHY IT EXISTS
# -------------
# Every in-tree test links spade:: as build-tree targets and never reads
# spadeConfig.cmake or spadeTargets.cmake. tests/consumer is the one place a
# defect in the installed package shows up: a public header that includes an
# uninstalled header, an export that drops a link dependency, a config file
# that fails to find glm or yaml-cpp. Restructure defect 3 was exactly that (a
# SPADE_VULKAN=OFF install shipping sim/simulation.hpp without the compute/
# headers it includes), and it was found by hand. This script is how it stays
# found.
#
# WHAT IT DOES
# ------------
#   configure, build-libs  (skipped with --from-build) a release tree with v1,
#                          the sandbox and GL off, and SPADE_VULKAN as given;
#                          only the installed library targets are built
#   install                into a FRESH prefix
#   consumer-configure     a FRESH tests/consumer tree against that prefix,
#                          and a check that find_package(spade) resolved to it
#   consumer-build, consumer-run
#
# The prefix and the consumer tree are wiped every run: a stale header or
# archive left in either could make a missing install look present. The
# library tree is kept between runs (ninja is incremental), since nothing
# inside it can mask an install defect.
#
# Usage:
#   scripts/consumer-smoke.sh --vulkan ON|OFF --work DIR
#                             [--from-build DIR] [--deps DIR] [--jobs N]
#
#   --vulkan ON|OFF   The SPADE_VULKAN mode to install and consume.
#   --work DIR        Where every tree goes (created if missing). The source
#                     tree is never written to, so it may be mounted read-only.
#   --from-build DIR  Install from this already configured AND built tree
#                     instead of building one. Refused when its SPADE_VULKAN
#                     differs from --vulkan.
#   --deps DIR        A FetchContent _deps directory (e.g. <build>/_deps). Each
#                     <name>-src in it is reused through
#                     FETCHCONTENT_SOURCE_DIR_<NAME>; without it, dependencies
#                     are fetched over the network.
#   --jobs N          Build parallelism (default 1).
#
# Exit status: 0 pass; 1 a stage failed (the last line names it); 2 usage.
#
# Needs: bash, cmake >= 3.28, ninja, a C++23 compiler with <expected> (gcc 13+),
# git, and tar (CMake unpacks the slang archive with it).

set -uo pipefail

usage() {
    sed -n 's/^# \{0,1\}//; /^Usage:/,/^Exit status/p' "$0"
}

die_usage() {
    echo "consumer-smoke: $*" >&2
    usage >&2
    exit 2
}

vulkan=""
work=""
from_build=""
deps=""
jobs=1

while [ $# -gt 0 ]; do
    case "$1" in
        -h|--help) usage; exit 0 ;;
        --vulkan|--work|--from-build|--deps|--jobs)
            [ $# -ge 2 ] || die_usage "$1 needs a value"
            case "$1" in
                --vulkan) vulkan="$2" ;;
                --work) work="$2" ;;
                --from-build) from_build="$2" ;;
                --deps) deps="$2" ;;
                --jobs) jobs="$2" ;;
            esac
            shift 2 ;;
        *) die_usage "unknown argument: $1" ;;
    esac
done

case "$vulkan" in
    ON|OFF) ;;
    "") die_usage "--vulkan is required" ;;
    *) die_usage "--vulkan must be ON or OFF, not '$vulkan'" ;;
esac
[ -n "$work" ] || die_usage "--work is required"
case "$jobs" in
    ''|*[!0-9]*|0) die_usage "--jobs must be a positive integer, not '$jobs'" ;;
esac
if [ -n "$from_build" ] && [ ! -f "$from_build/CMakeCache.txt" ]; then
    die_usage "--from-build $from_build is not a configured build tree (no CMakeCache.txt)"
fi
if [ -n "$deps" ] && [ ! -d "$deps" ]; then
    die_usage "--deps $deps is not a directory"
fi

src="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
[ -f "$src/tests/consumer/CMakeLists.txt" ] || die_usage "cannot find tests/consumer under $src"

mkdir -p "$work" || die_usage "cannot create --work $work"
work="$(cd "$work" && pwd)"
[ -n "$from_build" ] && from_build="$(cd "$from_build" && pwd)"
[ -n "$deps" ] && deps="$(cd "$deps" && pwd)"

mode="$(echo "$vulkan" | tr 'A-Z' 'a-z')"
build="$work/build-vk$mode"
prefix="$work/prefix-vk$mode"
consumer="$work/consumer-vk$mode"

# One -DFETCHCONTENT_SOURCE_DIR_<NAME> per <name>-src in --deps. <NAME> is the
# FetchContent_Declare name upper-cased with hyphens kept (YAML-CPP,
# VULKAN-HEADERS), and _deps names its checkouts by the lower-cased declare
# name, so the directory name is all the mapping needs. A project that does not
# declare a given name ignores it (--no-warn-unused-cli keeps that quiet).
dep_args=()
if [ -n "$deps" ]; then
    for d in "$deps"/*-src; do
        [ -d "$d" ] || continue
        name="$(basename "$d")"
        name="${name%-src}"
        dep_args+=("-DFETCHCONTENT_SOURCE_DIR_$(echo "$name" | tr 'a-z' 'A-Z')=$d")
    done
fi

# stage NAME COMMAND... -- run one stage, time it, and stop the script with the
# stage named if it fails.
stage() {
    local name="$1"
    shift
    echo "consumer-smoke: $name vulkan=$vulkan ..."
    local t0=$SECONDS
    "$@"
    local rc=$?
    if [ $rc -ne 0 ]; then
        echo "consumer-smoke: FAIL $name (vulkan=$vulkan, exit $rc)"
        exit 1
    fi
    echo "consumer-smoke: $name vulkan=$vulkan ok, $((SECONDS - t0)) s"
}

# The targets `cmake --install` copies. The OFF list is every installed module
# (engine/CMakeLists.txt's install(TARGETS ...) rules); ON adds spade_compute.
# A module added to the install rules but not here makes the install stage fail
# on the missing archive, so this list cannot drift into a silent pass.
lib_targets=(spade_core spade_state spade_world spade_objects spade_physics
             spade_render spade_sim spade_vehicles)
[ "$vulkan" = ON ] && lib_targets+=(spade_compute)

from_build_mode() {
    sed -n 's/^SPADE_VULKAN:BOOL=//p' "$from_build/CMakeCache.txt"
}

echo "consumer-smoke: source $src"
echo "consumer-smoke: $(cmake --version | head -n 1)"
echo "consumer-smoke: compiler $("${CXX:-c++}" --version 2>/dev/null | head -n 1)"

if [ -n "$from_build" ]; then
    got="$(from_build_mode)"
    if [ "$got" != "$vulkan" ]; then
        echo "consumer-smoke: FAIL from-build (vulkan=$vulkan): $from_build has SPADE_VULKAN=${got:-unset}"
        exit 1
    fi
    install_from="$from_build"
else
    stage configure cmake -S "$src" -B "$build" -G Ninja --no-warn-unused-cli \
        -DCMAKE_BUILD_TYPE=Release \
        -DSPADE_VULKAN="$vulkan" \
        -DSPADE_BUILD_V1=OFF -DSPADE_BUILD_SANDBOX=OFF -DSPADE_RENDER_GL=OFF \
        ${dep_args[@]+"${dep_args[@]}"}
    stage build-libs cmake --build "$build" --parallel "$jobs" --target "${lib_targets[@]}"
    install_from="$build"
fi

rm -rf "$prefix" "$consumer" || { echo "consumer-smoke: FAIL clean (vulkan=$vulkan): cannot remove $prefix or $consumer"; exit 1; }
stage install cmake --install "$install_from" --prefix "$prefix"

stage consumer-configure cmake -S "$src/tests/consumer" -B "$consumer" -G Ninja --no-warn-unused-cli \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_PREFIX_PATH="$prefix" \
    ${dep_args[@]+"${dep_args[@]}"}

# find_package(spade) must have resolved to THIS prefix. A spade package found
# anywhere else (a system install in the image, a registry entry) would make the
# rest of this script test the wrong thing and pass.
spade_dir="$(sed -n 's/^spade_DIR:PATH=//p' "$consumer/CMakeCache.txt")"
case "$spade_dir" in
    "$prefix"/*) echo "consumer-smoke: find_package(spade) -> $spade_dir" ;;
    *)
        echo "consumer-smoke: FAIL consumer-configure (vulkan=$vulkan): spade_DIR=${spade_dir:-unset} is not under $prefix"
        exit 1 ;;
esac

stage consumer-build cmake --build "$consumer" --parallel "$jobs"
stage consumer-run "$consumer/spade_consumer_smoke"

echo "consumer-smoke: PASS vulkan=$vulkan"
