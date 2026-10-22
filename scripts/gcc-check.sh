#!/usr/bin/env bash
# The gcc check before review (docs/design/test-docs/02-build-and-gate.md).
# Compiles each named file with gcc-13 -fsyntax-only and the engine's warnings
# as errors, in the Docker leg's image, against a leg volume's dependency and
# generated headers. From the worktree root, in Git Bash:
#
#   scripts/gcc-check.sh tests/test_example.cpp engine/render/example.hpp
#
# It checks the files as they are on disk, uncommitted changes included; a
# header is checked through a one-line translation unit that includes it. The
# volume is mounted read-only, and it is the newest leg volume (one the driver
# made: label spade.leg) that holds a configured build and that no running
# container uses, so a check never reads a volume a leg is building and any
# number of checks can run beside legs. GCC_CHECK_VOLUME=<volume> picks one.
#
# Generated headers (the kernels' .spv.gen.hpp, bindings.gen.hpp) are read
# from the worktree's own build first, then the volume's: the volume's are its
# leg commit's, so a branch's new or changed kernel would otherwise be checked
# against old headers or none. The build is build-ninja/release when it exists;
# GCC_CHECK_BUILD=<dir under the worktree> names another, and an empty value
# uses the volume's alone. A .slang newer than the build's last generator
# output is named in a gcc note: build first.
#
# --compile (first argument): compile each named .cpp in full, with the leg
# volume's own Release command line (-O3), plus -Wall -Wextra -Wpedantic
# -Werror. -fsyntax-only never runs the optimiser, so it misses the warnings
# only optimisation finds (maybe-uninitialized, array-bounds); a leg finds
# them minutes later. This finds them in seconds. A header is still checked
# with -fsyntax-only; a .cpp the volume's build never compiled (a new file)
# fails with the reason. scripts/gcc_check_compile.py does the compiling.
#
#   scripts/gcc-check.sh --compile engine/render/flow_render.cpp
#
# Exit: 0 every file passed, 1 a file failed, 2 usage, or no usable volume.
set -u
prefix=spade-docker-leg
here=$(cd "$(dirname "$0")" && pwd -W)

mode=syntax
if [ "${1:-}" = "--compile" ]; then
    mode=compile
    shift
fi
[ $# -ge 1 ] || { echo "usage: scripts/gcc-check.sh [--compile] <file>..." >&2; exit 2; }
if [ ! -d engine ] || [ ! -d tests ]; then
    echo "gcc-check: run it from the worktree root (no engine/ or tests/ here)" >&2
    exit 2
fi
for f in "$@"; do
    [ -f "$f" ] || { echo "gcc-check: $f: no such file" >&2; exit 2; }
done
build=${GCC_CHECK_BUILD-build-ninja/release}
case $build in
    /*|?:*) echo "gcc-check: GCC_CHECK_BUILD=$build: name a directory under the worktree root" >&2; exit 2 ;;
esac
if [ -n "${GCC_CHECK_BUILD:-}" ] && [ ! -d "$build" ]; then
    echo "gcc-check: GCC_CHECK_BUILD=$build: no such directory" >&2
    exit 2
fi
[ -d "$build" ] || build=

image=$(docker images -q "$prefix" | head -n 1)
if [ -z "$image" ]; then
    echo "gcc-check: no $prefix image; build it with scripts\\docker-leg.ps1 -Step image" >&2
    exit 2
fi

# Leg volumes, newest first. Only the driver's (label spade.leg): mounting a
# volume name that does not exist makes Docker create it EMPTY and unlabelled,
# and such a volume must never be chosen.
leg_volumes() {
    docker volume ls -q --filter label=spade.leg \
      | xargs -r docker volume inspect --format '{{.CreatedAt}} {{.Name}}' \
      | sort -r | cut -d' ' -f2
}

volume=${GCC_CHECK_VOLUME:-}
if [ -z "$volume" ]; then
    for v in $(leg_volumes); do
        [ -z "$(docker ps -q --filter "volume=$v")" ] || continue
        # A volume whose leg never configured has no dependency headers.
        MSYS_NO_PATHCONV=1 docker run --rm -v "$v:/leg:ro" "$image" test -d /leg/build/_deps || continue
        volume=$v
        break
    done
fi
if [ -z "$volume" ]; then
    echo "gcc-check: no idle leg volume holds a configured build; run a leg (scripts\\docker-leg.ps1), or let one finish" >&2
    exit 2
fi

MSYS_NO_PATHCONV=1 docker run --rm --memory 3g \
  -v "$volume:/leg:ro" -v "$(pwd -W):/src:ro" -v "$here:/gcc-check:ro" "$image" bash -c '
  volume=$1; mode=$2; build=$3; shift 3
  if [ ! -d /leg/build/_deps ]; then
    echo "gcc-check: volume $volume has no configured build (/leg/build/_deps)"; exit 2
  fi
  echo "gcc-check: volume $volume, commit $(cat /leg/commit 2>/dev/null || echo unrecorded); $(g++-13 --version | head -n 1)"
  mkdir -p /tmp/s && for d in engine sandbox tests tools include src examples spike; do
    if [ -d /src/$d ]; then cp -r /src/$d /tmp/s; fi
  done; cd /tmp/s
  # Every generated-header directory the volume holds, with the worktree
  # build copy of it ahead.
  gen=(); mine=()
  for g in $(cd /leg/build && find . -maxdepth 4 -path ./_deps -prune -o -type d -path "*/generated/spade_slang" -print | sort); do
    g=${g#./}
    if [ -n "$build" ] && [ -d "/src/$build/$g" ]; then gen+=("-I/src/$build/$g"); mine+=("/src/$build/$g"); fi
    gen+=("-I/leg/build/$g")
  done
  if [ ${#mine[@]} -gt 0 ]; then
    echo "gcc-check: generated headers from $build first, then the volume"
    # The newest output of any kind: slangc rewrites its .spv on every run, but
    # both header generators leave an unchanged header untouched.
    newest=$(find "${mine[@]}" -maxdepth 1 -type f -printf "%T@ %p\n" | sort -n | tail -n 1 | cut -d" " -f2)
    stale=($(find /src/engine/shaders /src/spike -name "*.slang" -newer "$newest" 2>/dev/null | sed "s|^/src/||" | sort))
    if [ ${#stale[@]} -gt 0 ]; then
      echo "gcc note: ${#stale[@]} .slang file(s) changed since $build last generated its headers (${stale[*]:0:3}): build first, or this check reads stale ones"
    fi
  else
    echo "gcc-check: generated headers from the volume alone (its leg commit); GCC_CHECK_BUILD=<dir> names a worktree build"
  fi
  D=/leg/build/_deps; rc=0; compile=()
  for f in "$@"; do
    # --compile: a .cpp goes to the full compile below; a header stays here.
    if [ "$mode" = compile ] && [ "${f##*.}" = cpp ]; then compile+=("$f"); continue; fi
    tu=$f; case $f in *.hpp) printf "#include \"/tmp/s/%s\"\n" "$f" > /tmp/hdr.cpp; tu=/tmp/hdr.cpp ;; esac
    g++-13 -std=gnu++23 -fsyntax-only -Wall -Wextra -Wpedantic -Werror -ffp-contract=off \
      -DGLM_ENABLE_EXPERIMENTAL -DGLFW_INCLUDE_NONE -DYAML_CPP_STATIC_DEFINE \
      -DSPADE_ENGINE_DIR=\"/src/engine\" -DSPADE_GOLDEN_DIR=\"/src/tests/golden\" \
      -DSPADE_TESTS_DIR=\"/src/tests\" -DSPADE_TEST_OUTPUT_DIR=\"/tmp/test-output\" \
      -DSPADE_ASSETS_DIR=\"/src/assets\" \
      -Iengine -Isandbox -Iengine/tools/viewer -I. "${gen[@]}" \
      -I$D/glm-src -I$D/glad-src/include -I$D/glfw-src/include -I$D/imgui-src -I$D/imgui-src/backends \
      -I$D/yaml-cpp-src/include -I$D/nlohmann_json-src/include -I$D/volk-src -I$D/vulkan-headers-src/include \
      -isystem $D/googletest-src/googletest/include -isystem $D/benchmark-src/include \
      "$tu" && echo "gcc ok:   $f" || { echo "gcc FAIL: $f"; rc=1; }
  done
  if [ ${#compile[@]} -gt 0 ]; then
    GCC_CHECK_BUILD=$build python3 /gcc-check/gcc_check_compile.py "$volume" "${compile[@]}" || rc=1
  fi
  exit $rc' _ "$volume" "$mode" "$build" "$@"
