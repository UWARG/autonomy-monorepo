FROM ros:humble-ros-base AS dev

ENV DEBIAN_FRONTEND=noninteractive

SHELL ["/bin/bash", "-lc"]

RUN apt-get update && apt-get install -y --no-install-recommends \
    geographiclib-tools \
    git \
    python3-colcon-common-extensions \
    python3-pip \
    python3-pytest \
    python3-rosdep \
    ros-humble-mavros \
    ros-humble-mavros-extras \
    ros-humble-py-trees \
    ros-humble-py-trees-ros \
    ros-humble-py-trees-ros-interfaces \
    ros-humble-rosbridge-suite \
    v4l-utils \
    wget \
    #for non-optimal
    ros-humble-v4l2-camera \
  && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y --no-install-recommends bzip2 curl \
  && mkdir -p /usr/share/GeographicLib \
  && cd /tmp \
  && curl -fL --retry 5 --retry-all-errors --retry-delay 3 \
       --connect-timeout 20 --max-time 180 \
       -o egm96-5.tar.bz2 \
       "https://sourceforge.net/projects/geographiclib/files/geoids-distrib/egm96-5.tar.bz2/download" \
  && tar -xjf egm96-5.tar.bz2 -C /usr/share/GeographicLib \
  && test -f /usr/share/GeographicLib/geoids/egm96-5.pgm \
  && rm -f egm96-5.tar.bz2 \
  && rm -rf /var/lib/apt/lists/* /var/cache/apt/archives/*

RUN if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then rosdep init; fi \
  && rosdep update --rosdistro humble

RUN pip install --upgrade pip packaging hatchling pymavlink

RUN echo "source /opt/ros/humble/setup.bash" >> /etc/bash.bashrc

WORKDIR /ros_ws

FROM dev AS prod

COPY camera/ /monorepo/camera/
COPY utils/ /monorepo/utils/
RUN chmod +x /monorepo/camera/scripts/setup_arducam_v4l2.sh \
  && pip install /monorepo/camera /monorepo/utils && pip install "numpy>=1.17.3,<1.25" sortedcontainers scipy

COPY airside/src/ src/

RUN source /opt/ros/humble/setup.bash \
  && rosdep install --from-paths src --ignore-src --rosdistro humble -y --skip-keys ament_python \
  && colcon build --symlink-install

COPY airside/docker/airside_entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]