FROM ubuntu:24.04

ENV DEBIAN_FRONTEND=noninteractive

COPY --from=ghcr.io/astral-sh/uv:0.11.17 /uv /uvx /bin/


RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    python3 \
    python3-venv \
    python3-pip \
    git \
    curl \
    ca-certificates \
    libgl1 \
    libglib2.0-0 \
    libgomp1 \
    libsm6 \
    libxext6 \
    libxrender1 \
    xvfb \
    apt-utils \
    sudo \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /app
WORKDIR /app/src

RUN useradd -m -s /bin/bash devuser && \
    echo "devuser ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers && \
    chown -R devuser:devuser /app

USER devuser
ENV USER=devuser
ENV HOME=/home/devuser
ENV LOGNAME=devuser

# Pin to the ArduPilot revision from the working 2026-07-07 run
# (sim_vehicle.log: AP: ArduCopter V4.8.0-dev (5190fb32)). Unpinned clone
# drifts on every cache-busting rebuild.
# Clone without --recurse-submodules so submodule SHAs match ARDUPILOT_REF,
# not whatever master has at clone time.
ARG ARDUPILOT_REF=5190fb3244d5c16cb197f91f8aedf752ff68fbdb
RUN git clone https://github.com/ArduPilot/ardupilot.git ardupilot && \
      cd ardupilot && \
      git checkout "${ARDUPILOT_REF}" && \
      git submodule update --init --recursive && \
      git log -1 --format='ArduPilot %H (%ci) %s' | tee /app/ardupilot_version.txt && \
      Tools/environment_install/install-prereqs-ubuntu.sh -y;

WORKDIR /app/src/ardupilot
RUN bash -lc 'source /home/devuser/venv-ardupilot/bin/activate && \
    ./waf configure --board sitl && \
    ./waf copter -j2'

WORKDIR /app

EXPOSE 5761/tcp


USER root
COPY . /app/
RUN chown -R devuser:devuser /app &&\
    chmod +x /app/docker/entrypoint_rerun.sh

USER devuser
RUN uv sync --locked

ENTRYPOINT ["/app/docker/entrypoint_rerun.sh"]