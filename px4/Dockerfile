# PX4 SITL (v1.16.0, the version Pegasus PR #144 was tested with), built without any simulator:
# Pegasus Simulator in Isaac Sim is the simulator and talks to it over MAVLink (TCP 4560, lockstep).
#   docker build -t isaac-exploration/px4:v1.16.0 px4/
FROM ubuntu:22.04
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
        git ca-certificates sudo lsb-release tzdata python3 python3-pip \
    && rm -rf /var/lib/apt/lists/*
WORKDIR /opt
# Full clone: the version header script reads git tags (incl. the NuttX submodule's), which a shallow clone lacks.
RUN git clone --branch v1.16.0 --recursive https://github.com/PX4/PX4-Autopilot.git \
    && test "$(git -C PX4-Autopilot rev-parse HEAD)" = "6ea3539157ca358c70a515878b77077af7d4611d"
# PX4's own dependency script, minus the NuttX (hardware) toolchain and simulators.
RUN bash PX4-Autopilot/Tools/setup/ubuntu.sh --no-nuttx --no-sim-tools && rm -rf /var/lib/apt/lists/*
WORKDIR /opt/PX4-Autopilot
RUN make px4_sitl_default && rm -rf build/px4_sitl_default/external
COPY run_px4.sh /opt/run_px4.sh
ENTRYPOINT ["/opt/run_px4.sh"]
