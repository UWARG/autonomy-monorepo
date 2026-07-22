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
    ros-humble-foxglove-bridge \
    ros-humble-rosbag2-storage-mcap \
    wget \
  && rm -rf /var/lib/apt/lists/*

RUN /opt/ros/humble/lib/mavros/install_geographiclib_datasets.sh

# Retry rosdep update -- the rosdistro YAML fetch from githubusercontent is flaky.
RUN if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then rosdep init; fi \
  && rosdep update --rosdistro humble

RUN pip install --upgrade pip packaging hatchling pymavlink

RUN echo "source /opt/ros/humble/setup.bash" >> /etc/bash.bashrc

WORKDIR /ros_ws

FROM dev AS prod

COPY camera/ /monorepo/camera/
COPY utils/ /monorepo/utils/
COPY gnc/ /monorepo/gnc/
RUN pip install /monorepo/camera /monorepo/utils && pip install --timeout 120 --retries 10 --no-deps /monorepo/gnc  # gnc dep=utils (installed)

RUN apt-get update && apt-get install -y --no-install-recommends geographiclib-tools \
  && for i in 1 2 3 4 5; do geographiclib-get-geoids egm96-5 && break \
       || { echo "geoid download retry $i"; sleep 5; }; done \
  && rm -rf /var/lib/apt/lists/*

COPY airside/src/ src/

RUN source /opt/ros/humble/setup.bash \
  && rosdep install --from-paths src --ignore-src --rosdistro humble -y --skip-keys ament_python \
  && colcon build --symlink-install

COPY airside/docker/airside_entrypoint.sh /entrypoint.sh
RUN sed -i 's/\r$//' /entrypoint.sh && chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
