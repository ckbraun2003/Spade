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
# Exit: 0 every file passed, 1 a file failed, 2 usage, or no usable volume.
set -u
prefix=spade-docker-leg

[ $# -ge 1 ] || { echo "usage: scripts/gcc-check.sh <file>..." >&2; exit 2; }
if [ ! -d engine ] || [ ! -d tests ]; then
    echo "gcc-check: run it from the worktree root (no engine/ or tests/ here)" >&2
    exit 2
fi
for f in "$@"; do
    [ -f "$f" ] || { echo "gcc-check: $f: no such file" >&2; exit 2; }
done

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
  -v "$volume:/leg:ro" -v "$(pwd -W):/src:ro" "$image" bash -c '
  volume=$1; shift
  if [ ! -d /leg/build/_deps ]; then
    echo "gcc-check: volume $volume has no configured build (/leg/build/_deps)"; exit 2
  fi
  echo "gcc-check: volume $volume, commit $(cat /leg/commit 2>/dev/null || echo unrecorded); $(g++-13 --version | head -n 1)"
  mkdir -p /tmp/s && cp -r /src/engine /src/sandbox /src/tests /tmp/s && cd /tmp/s
  D=/leg/build/_deps; rc=0
  for f in "$@"; do
    tu=$f; case $f in *.hpp) printf "#include \"/tmp/s/%s\"\n" "$f" > /tmp/hdr.cpp; tu=/tmp/hdr.cpp ;; esac
    g++-13 -std=gnu++23 -fsyntax-only -Wall -Wextra -Wpedantic -Werror -ffp-contract=off \
      -DGLM_ENABLE_EXPERIMENTAL -DGLFW_INCLUDE_NONE -DYAML_CPP_STATIC_DEFINE \
      -DSPADE_ENGINE_DIR=\"/src/engine\" -DSPADE_GOLDEN_DIR=\"/src/tests/golden\" \
      -DSPADE_TESTS_DIR=\"/src/tests\" -DSPADE_TEST_OUTPUT_DIR=\"/tmp/test-output\" \
      -DSPADE_ASSETS_DIR=\"/src/assets\" \
      -Iengine -Isandbox -Iengine/tools/viewer -I/leg/build/engine/generated/spade_slang \
      -I$D/glm-src -I$D/glad-src/include -I$D/glfw-src/include -I$D/imgui-src -I$D/imgui-src/backends \
      -I$D/yaml-cpp-src/include -I$D/nlohmann_json-src/include -I$D/volk-src -I$D/vulkan-headers-src/include \
      -isystem $D/googletest-src/googletest/include -isystem $D/benchmark-src/include \
      "$tu" && echo "gcc ok:   $f" || { echo "gcc FAIL: $f"; rc=1; }
  done; exit $rc' _ "$volume" "$@"
