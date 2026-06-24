FROM ros:humble-ros-base AS dev

ENV DEBIAN_FRONTEND=noninteractive

SHELL ["/bin/bash", "-lc"]

RUN apt-get update && apt-get install -y --no-install-recommends \
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
    ros-humble-depthai \
    rerun-sdk \
  && rm -rf /var/lib/apt/lists/*

RUN if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then rosdep init; fi \
  && rosdep update --rosdistro humble

RUN pip install --upgrade pip packaging hatchling

RUN echo "source /opt/ros/humble/setup.bash" >> /etc/bash.bashrc

WORKDIR /ros_ws

FROM dev AS prod

COPY camera/ /monorepo/camera/
COPY utils/ /monorepo/utils/
RUN pip install /monorepo/camera /monorepo/utils

COPY airside/src/ src/

RUN source /opt/ros/humble/setup.bash \
  && rosdep install --from-paths src --ignore-src --rosdistro humble -y --skip-keys ament_python \
  && colcon build --symlink-install

COPY airside/docker/airside_entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
