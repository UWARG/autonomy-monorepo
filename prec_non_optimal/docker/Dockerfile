FROM ros:humble-ros-base

ENV DEBIAN_FRONTEND=noninteractive

SHELL ["/bin/bash", "-lc"]

RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-colcon-common-extensions \
    python3-pip \
    python3-pytest \
    python3-rosdep \
    ros-humble-mavros \
    ros-humble-mavros-extras \
    ros-humble-cv-bridge \
    ros-humble-apriltag-ros \
    ros-humble-rqt-image-view \
    ros-humble-camera-calibration \
    ros-humble-usb-cam \
    ros-humble-image-proc \
    ros-humble-v4l2-camera \
    python3-scipy \
    ros-humble-py-trees \
    ros-humble-py-trees-ros \
    ros-humble-py-trees-ros-interfaces \
  && rm -rf /var/lib/apt/lists/*

RUN if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then rosdep init; fi \
  && rosdep update --rosdistro humble

COPY camera/ /monorepo/camera/
COPY utils/ /monorepo/utils/
RUN pip install numpy scipy sortedcontainers

WORKDIR /ros_ws

COPY airside_prec/src/ src/

RUN source /opt/ros/humble/setup.bash \
  && rosdep install --from-paths src --ignore-src --rosdistro humble -y --skip-keys ament_python \
  && colcon build --symlink-install

COPY airside_prec/docker/airside_entrypoint.sh /airside_entrypoint.sh
RUN chmod +x /airside_entrypoint.sh

RUN /opt/ros/humble/lib/mavros/install_geographiclib_datasets.sh


ENTRYPOINT ["/airside_entrypoint.sh"]
CMD ["bash"]
