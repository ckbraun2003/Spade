# The PX4 SITL image for Spade's autopilot link (O140, O144). Built with the NETWORK OFF from a source tree fetched once:
#   docker build --network none -f tools/fc_link/px4/Dockerfile -t spade-px4-sitl:v1.17.0 <build-px4>
# where <build-px4> holds PX4-Autopilot (tag v1.17.0, d6f12ad1c4f70ad3230afd7d86e971421e02fef4, shallow, submodules) and overlay/ (the files below).
# The base image is PX4's own build image, pinned by digest (amd64 manifest of px4io/px4-dev:v1.17.0). PX4 is BSD-3-Clause; the SITL binary is run
# locally in a container, never distributed, never linked into Spade or Kat and never modified (its EPL-2.0 DDS/zenoh code too): see
# docs/design/interface/records/ for the run record notes.
FROM px4io/px4-dev@sha256:82e3ddcf4bf3fa4dbc0be6489c6de4158d0efbc8530182c691184302027335ce
RUN git config --global --add safe.directory '*'
COPY PX4-Autopilot /src/PX4-Autopilot
# our one board variant: PX4's SITL board without the Gazebo modules
COPY overlay/spade_sitl.px4board /src/PX4-Autopilot/boards/px4/sitl/spade.px4board
WORKDIR /src/PX4-Autopilot
RUN make px4_sitl_spade ARGS="-j6"
