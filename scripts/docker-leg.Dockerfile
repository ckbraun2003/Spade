# The Docker leg's image (TD-11, TD-12): Linux, gcc, CPU-only. Built by
# scripts\docker-leg.ps1 from the commit under test and tagged by this file's
# content hash, so editing it builds a new image. It holds the toolchain only;
# the source and build trees live in the spade-docker-leg volume.
#
# gcc-13 is the second toolchain for goldens (TD-12). Noble's cmake is 3.28.3,
# the project's floor (CMakeLists.txt), so the leg proves the floor as well.
# python3 runs the shader-reflection steps (cmake/SpadeSlang.cmake). No X11
# packages: GLFW then builds without a window backend (vendor/CMakeLists.txt).
FROM ubuntu:24.04@sha256:a853f94d226358a79c740cfc7bce0c289748f3fe3488d921d038ccd752c61b60

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update \
 && apt-get install -y --no-install-recommends \
        gcc-13 g++-13 cmake ninja-build git python3 \
        ca-certificates rsync xz-utils \
 && rm -rf /var/lib/apt/lists/*

ENV CC=gcc-13 CXX=g++-13
WORKDIR /leg
