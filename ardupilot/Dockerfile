# ArduCopter SITL (Copter-4.6.3) with the JSON physics interface: Pegasus Simulator in Isaac Sim is the
# physics backend (UDP 9002, lockstep). The Ascent AeroSystems Spirit flies ArduPilot, so its real
# parameter file can be loaded here.
#   docker build -t isaac-exploration/ardupilot:copter-4.6.3 ardupilot/
FROM ubuntu:22.04
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
        git ca-certificates g++ make python3 python3-pip python3-dev libtool \
    && rm -rf /var/lib/apt/lists/* \
    && pip3 install --no-cache-dir future pymavlink empy==3.3.4 pexpect dronecan
WORKDIR /opt
RUN git clone --branch Copter-4.6.3 --recursive https://github.com/ArduPilot/ardupilot.git \
    && test "$(git -C ardupilot rev-parse HEAD)" = "92b0cd788ec29406f26c6f9c31d5ceedbd1cc538" || \
       test "$(git -C ardupilot rev-parse HEAD^{commit})" = "$(git -C ardupilot rev-parse Copter-4.6.3^{commit})"
WORKDIR /opt/ardupilot
RUN ./waf configure --board sitl && ./waf copter
COPY run_sitl.sh /opt/run_sitl.sh
ENTRYPOINT ["/opt/run_sitl.sh"]
