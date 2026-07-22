# Non-optimal / precision-landing image for Jetson Orin Nano
#
# Target hardware / software (from your board):
#   Jetson Orin Nano Developer Kit
#   JetPack 7.2 · L4T R39.2 · CUDA 13.2 · aarch64 · GPU arch sm_87
#
# Why this shape works:
#   - JetPack 7.2 host uses CUDA 13.2; prebuilt CUDA-12 Jetson images fail on this stack.
#   - ROS 2 Humble apt packages need Ubuntu 22.04 (not the host's 24.04).
#   - nvidia/cuda:13.2.0-*-ubuntu22.04 gives CUDA 13.2 + Jammy, so Humble apt + cv2.cuda
#     can coexist. GPU access at runtime comes from compose `runtime: nvidia`.
#
# Build ON the Jetson (arm64), not on WSL/x86:
#   cd airside && docker compose --profile non-optimal build
#
# Orin Nano is memory-tight; if the OpenCV build OOMs, rebuild with:
#   docker build --build-arg OPENCV_JOBS=2 -f airside/docker/Non_optimal.Dockerfile ...

ARG CUDA_DEVEL_IMAGE=nvidia/cuda:13.2.0-devel-ubuntu22.04
ARG CUDA_RUNTIME_IMAGE=nvidia/cuda:13.2.0-runtime-ubuntu22.04
ARG OPENCV_VERSION=4.10.0
ARG OPENCV_JOBS=4

# -----------------------------------------------------------------------------
# Stage 1: build OpenCV with CUDA (Orin = compute 8.7)
# -----------------------------------------------------------------------------
FROM ${CUDA_DEVEL_IMAGE} AS opencv-builder

ARG OPENCV_VERSION
ARG OPENCV_JOBS

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    pkg-config \
    python3-dev \
    python3-numpy \
    python3-pip \
    libjpeg-dev \
    libpng-dev \
    libtiff-dev \
    libavcodec-dev \
    libavformat-dev \
    libswscale-dev \
    libv4l-dev \
    libgtk-3-dev \
    libopenexr-dev \
    libtbb-dev \
    wget \
  && rm -rf /var/lib/apt/lists/*

WORKDIR /tmp
# CUDA needs opencv_contrib (cudev). Same version tag as opencv.
RUN git clone --depth 1 --branch ${OPENCV_VERSION} https://github.com/opencv/opencv.git \
  && git clone --depth 1 --branch ${OPENCV_VERSION} https://github.com/opencv/opencv_contrib.git

WORKDIR /tmp/opencv/build
# BFMatcher needs cudafeatures2d; skip cuDNN/DNN — processor does not use them.
RUN cmake -D CMAKE_BUILD_TYPE=RELEASE \
      -D CMAKE_INSTALL_PREFIX=/opt/opencv \
      -D OPENCV_EXTRA_MODULES_PATH=/tmp/opencv_contrib/modules \
      -D WITH_CUDA=ON \
      -D WITH_CUDNN=OFF \
      -D OPENCV_DNN_CUDA=OFF \
      -D CUDA_ARCH_BIN=8.7 \
      -D CUDA_ARCH_PTX=8.7 \
      -D CUDA_FAST_MATH=ON \
      -D WITH_CUBLAS=ON \
      -D ENABLE_FAST_MATH=ON \
      -D BUILD_opencv_python3=ON \
      -D BUILD_TESTS=OFF \
      -D BUILD_PERF_TESTS=OFF \
      -D BUILD_EXAMPLES=OFF \
      -D BUILD_opencv_apps=OFF \
      -D WITH_GTK=OFF \
      -D WITH_QT=OFF \
      -D PYTHON3_EXECUTABLE=/usr/bin/python3 \
      .. \
  && cmake --build . -j${OPENCV_JOBS} \
  && cmake --install .

# -----------------------------------------------------------------------------
# Stage 2: ROS Humble + CUDA runtime + airside workspace
# -----------------------------------------------------------------------------
FROM ${CUDA_RUNTIME_IMAGE}

ENV DEBIAN_FRONTEND=noninteractive \
    LANG=C.UTF-8 \
    ROS_DISTRO=humble \
    OpenCV_DIR=/opt/opencv/lib/cmake/opencv4 \
    LD_LIBRARY_PATH=/opt/opencv/lib:${LD_LIBRARY_PATH}

SHELL ["/bin/bash", "-lc"]

COPY --from=opencv-builder /opt/opencv /opt/opencv
RUN echo "/opt/opencv/lib" > /etc/ld.so.conf.d/opencv.conf && ldconfig \
  && CV2_PATH="$(find /opt/opencv -type d -name 'cv2' | head -n1)" \
  && CV2_PARENT="$(dirname "${CV2_PATH}")" \
  && echo "export PYTHONPATH=${CV2_PARENT}\${PYTHONPATH:+:\${PYTHONPATH}}" > /etc/profile.d/opencv-python.sh \
  && echo "${CV2_PARENT}" > /etc/opencv-python-path \
  && export PYTHONPATH="${CV2_PARENT}${PYTHONPATH:+:${PYTHONPATH}}" \
  && python3 -c "import cv2; print('OpenCV', cv2.__version__); assert hasattr(cv2, 'cuda'), 'cv2.cuda missing'"

# ROS 2 Humble apt repo (Ubuntu 22.04 / Jammy)
RUN apt-get update && apt-get install -y --no-install-recommends \
    curl \
    gnupg2 \
    lsb-release \
    ca-certificates \
  && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
       -o /usr/share/keyrings/ros-archive-keyring.gpg \
  && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
       > /etc/apt/sources.list.d/ros2.list \
  && apt-get update && apt-get install -y --no-install-recommends \
    python3-colcon-common-extensions \
    python3-pip \
    python3-pytest \
    python3-rosdep \
    python3-scipy \
    ros-humble-ros-base \
    ros-humble-mavros \
    ros-humble-mavros-extras \
    ros-humble-v4l2-camera \
    ros-humble-usb-cam \
    ros-humble-image-transport \
    ros-humble-py-trees \
    ros-humble-py-trees-ros \
    ros-humble-py-trees-ros-interfaces \
    ros-humble-diagnostic-updater \
    ros-humble-tf2-ros \
    geographiclib-tools \
    git \
    # Do NOT install ros-humble-cv-bridge / python3-opencv from apt — CPU OpenCV.
  && rm -rf /var/lib/apt/lists/*

RUN if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then rosdep init; fi \
  && rosdep update --rosdistro humble

RUN /opt/ros/humble/lib/mavros/install_geographiclib_datasets.sh

COPY camera/ /monorepo/camera/
COPY utils/ /monorepo/utils/
RUN pip3 install --no-cache-dir numpy scipy sortedcontainers \
  && pip3 install --no-cache-dir /monorepo/camera /monorepo/utils

WORKDIR /ros_ws

# Build cv_bridge against our CUDA OpenCV (not the apt CPU build).
RUN mkdir -p src \
  && git clone --depth 1 --branch humble \
       https://github.com/ros-perception/vision_opencv.git src/vision_opencv

COPY airside/src/ src/

RUN source /opt/ros/humble/setup.bash \
  && rosdep install --from-paths src --ignore-src --rosdistro humble -y \
       --skip-keys "ament_python libopencv-dev python3-opencv cv_bridge" \
  && colcon build --symlink-install \
       --cmake-args -DOpenCV_DIR=/opt/opencv/lib/cmake/opencv4

COPY airside/docker/airside_entrypoint.sh /airside_entrypoint.sh
RUN chmod +x /airside_entrypoint.sh

# Quick CUDA module check (device count needs --runtime nvidia at run time).
RUN export PYTHONPATH="$(cat /etc/opencv-python-path)${PYTHONPATH:+:${PYTHONPATH}}" \
  && python3 - <<'PY'
import cv2
print("OpenCV:", cv2.__version__)
print("CUDA module:", hasattr(cv2, "cuda"))
print("Build info snippet:\n", "\n".join(
    line for line in cv2.getBuildInformation().splitlines()
    if "NVIDIA CUDA" in line or "cuDNN" in line or "Python 3" in line
))
PY

ENTRYPOINT ["/airside_entrypoint.sh"]
CMD ["bash"]
