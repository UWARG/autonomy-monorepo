FROM ros:humble-ros-base AS dev

ENV DEBIAN_FRONTEND=noninteractive

SHELL ["/bin/bash", "-lc"]

ARG ROS_SNAPSHOT=2026-08-07
COPY airside/docker/ros-snapshots.asc /usr/share/keyrings/ros-snapshots.asc
RUN rm /etc/apt/sources.list.d/ros2.sources \
  && printf 'Types: deb\nURIs: http://snapshots.ros.org/humble/%s/ubuntu\nSuites: jammy\nComponents: main\nSigned-By: /usr/share/keyrings/ros-snapshots.asc\n' \
     "${ROS_SNAPSHOT}" > /etc/apt/sources.list.d/ros2-snapshot.sources

RUN apt-get update && apt-get install -y --no-install-recommends \
    geographiclib-tools \
    git \
    python3-colcon-common-extensions \
    python3-numpy \
    python3-pip \
    python3-pytest \
    python3-rosdep \
    python3-shapely \
    ros-humble-mavros=2.14.* \
    ros-humble-mavros-extras=2.14.* \
    ros-humble-py-trees \
    ros-humble-py-trees-ros \
    ros-humble-py-trees-ros-interfaces \
    ros-humble-rosbridge-suite \
    wget \
  && rm -f /etc/apt/sources.list.d/ros2-snapshot.list \
  && rm -rf /var/lib/apt/lists/*

RUN /opt/ros/humble/lib/mavros/install_geographiclib_datasets.sh

RUN if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then rosdep init; fi \
  && rosdep update --rosdistro humble

RUN pip install --upgrade pip packaging hatchling pymavlink

RUN echo "source /opt/ros/humble/setup.bash" >> /etc/bash.bashrc

WORKDIR /ros_ws

FROM dev AS prod

COPY camera/ /monorepo/camera/
COPY perception/ /monorepo/perception/
COPY utils/ /monorepo/utils/
RUN pip install /monorepo/camera /monorepo/utils /monorepo/perception

COPY airside/src/ src/

RUN source /opt/ros/humble/setup.bash \
  && rosdep install --from-paths src --ignore-src --rosdistro humble -y --skip-keys ament_python \
  && colcon build --symlink-install

COPY airside/docker/airside_entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
