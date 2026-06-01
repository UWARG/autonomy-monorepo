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
    && rm -rf /var/lib/apt/lists/*

WORKDIR /app

COPY . /app

WORKDIR /app/src

RUN git clone --recurse-submodules https://github.com/ArduPilot/ardupilot.git

WORKDIR /app/src/ardupilot

RUN Tools/environment_install/install-prereqs-ubuntu.sh -y

RUN git submodule update --init --recursive

RUN . ~/.profile

WORKDIR /app

RUN uv sync --locked

RUN chmod +x /docker/entrypoint.sh

ENTRYPOINT ["/docker/entrypoint.sh"]

