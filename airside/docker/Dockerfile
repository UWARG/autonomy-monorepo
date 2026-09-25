FROM ros:humble-ros-base AS dev

ENV DEBIAN_FRONTEND=noninteractive
ARG ROS_HUMBLE_SNAPSHOT=2026-07-02

SHELL ["/bin/bash", "-lc"]

RUN gpg --batch --keyserver hkp://keyserver.ubuntu.com:80 \
      --recv-keys 4B63CF8FDE49746E98FA01DDAD19BAB3CBF125EA \
  && gpg --batch --export 4B63CF8FDE49746E98FA01DDAD19BAB3CBF125EA \
      > /usr/share/keyrings/ros-snapshots.gpg \
  && rm -f /etc/apt/sources.list.d/ros2.sources \
  && printf '%s\n' \
      "deb [arch=amd64 signed-by=/usr/share/keyrings/ros-snapshots.gpg] http://snapshots.ros.org/humble/${ROS_HUMBLE_SNAPSHOT}/ubuntu jammy main" \
      > /etc/apt/sources.list.d/ros2-snapshot.list

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
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-ros2bag \
    ros-humble-rosbag2-storage-default-plugins \
    ros-humble-rosbridge-suite \
    wget \
  && rm -rf /var/lib/apt/lists/*

RUN /opt/ros/humble/lib/mavros/install_geographiclib_datasets.sh

RUN if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then rosdep init; fi \
  && rosdep update --rosdistro humble

RUN pip install --upgrade pip packaging hatchling pymavlink

RUN echo "source /opt/ros/humble/setup.bash" >> /etc/bash.bashrc

WORKDIR /ros_ws

FROM dev AS prod

COPY camera/ /monorepo/camera/
COPY obstacle-avoidance/ /monorepo/obstacle-avoidance/
COPY utils/ /monorepo/utils/
RUN pip install /monorepo/camera /monorepo/obstacle-avoidance /monorepo/utils

COPY airside/src/ src/

RUN source /opt/ros/humble/setup.bash \
  && rosdep install --from-paths src --ignore-src --rosdistro humble -y --skip-keys ament_python \
  && colcon build --symlink-install

COPY airside/docker/airside_entrypoint.sh /entrypoint.sh
RUN sed -i 's/\r$//' /entrypoint.sh \
  && chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
