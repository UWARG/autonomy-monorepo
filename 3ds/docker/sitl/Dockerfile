# Build a pinned, headless ArduCopter SITL binary for the integration test.
# The build context is the monorepo root because compose.sitl.yaml uses context: ..
FROM ubuntu:22.04 AS builder

ARG ARDUPILOT_REF=Copter-4.7.2
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    ccache \
    git \
    pkg-config \
    python3 \
    python3-dev \
    python3-lxml \
    python3-numpy \
    python3-pexpect \
    python3-pip \
    python3-serial \
    python3-setuptools \
    && rm -rf /var/lib/apt/lists/*

RUN git clone --depth 1 --branch "${ARDUPILOT_REF}" --recurse-submodules \
    --shallow-submodules https://github.com/ArduPilot/ardupilot.git /ardupilot

WORKDIR /ardupilot
RUN ./waf configure --board sitl && ./waf copter

FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
    libgcc-s1 \
    libstdc++6 \
    procps \
    && rm -rf /var/lib/apt/lists/*

COPY --from=builder /ardupilot/build/sitl/bin/arducopter /ardupilot/arducopter
COPY --from=builder /ardupilot/Tools/autotest/default_params/copter.parm /ardupilot/copter.parm
COPY 3ds/docker/sitl/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

EXPOSE 5762
ENTRYPOINT ["/entrypoint.sh"]
