# Jetson / precision-landing image for Jetson Orin Nano
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
#   cd airside && docker compose --profile jetson build
#
# Orin Nano is memory- and disk-tight:
#   - OpenCV OOM: OPENCV_JOBS=2
#   - apt "not enough free space": free host disk first, then rebuild
#       df -h && docker system prune -af
#   docker build --build-arg OPENCV_JOBS=2 -f airside/docker/Jetson.Dockerfile ...

ARG CUDA_DEVEL_IMAGE=nvidia/cuda:13.2.0-devel-ubuntu22.04
ARG CUDA_RUNTIME_IMAGE=nvidia/cuda:13.2.0-runtime-ubuntu22.04
# CUDA 13.2 needs OpenCV 4.x tip (cudev/CCCL fixes). Tagged 4.10–4.13.0 fail to compile.
ARG OPENCV_VERSION=4.x
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
# CUDA needs opencv_contrib (cudev). Same ref as opencv (use 4.x for CUDA 13.2).
RUN git clone --depth 1 --branch ${OPENCV_VERSION} https://github.com/opencv/opencv.git \
  && git clone --depth 1 --branch ${OPENCV_VERSION} https://github.com/opencv/opencv_contrib.git

WORKDIR /tmp/opencv/build
# BFMatcher needs cudafeatures2d; skip cuDNN/DNN — processor does not use them.
# CUDA 13.2 CCCL/Thrust requires C++17 (OpenCV defaults are too old).
# cudacodec needs libcuda (CUDA_CUDA_LIBRARY) at configure time — unused; leave OFF.
RUN cmake -D CMAKE_BUILD_TYPE=RELEASE \
      -D CMAKE_INSTALL_PREFIX=/opt/opencv \
      -D OPENCV_EXTRA_MODULES_PATH=/tmp/opencv_contrib/modules \
      -D CMAKE_CXX_STANDARD=17 \
      -D CMAKE_CXX_STANDARD_REQUIRED=ON \
      -D CMAKE_CUDA_STANDARD=17 \
      -D CMAKE_CUDA_STANDARD_REQUIRED=ON \
      -D CUDA_NVCC_FLAGS="-std=c++17" \
      -D WITH_CUDA=ON \
      -D WITH_CUDNN=OFF \
      -D OPENCV_DNN_CUDA=OFF \
      -D BUILD_opencv_cudacodec=OFF \
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
# nvidia/cuda runtime image has no python3 yet — only wire linker/PYTHONPATH here.
RUN echo "/opt/opencv/lib" > /etc/ld.so.conf.d/opencv.conf && ldconfig \
  && CV2_PATH="$(find /opt/opencv -type d -name 'cv2' | head -n1)" \
  && test -n "${CV2_PATH}" \
  && CV2_PARENT="$(dirname "${CV2_PATH}")" \
  && echo "export PYTHONPATH=${CV2_PARENT}\${PYTHONPATH:+:\${PYTHONPATH}}" > /etc/profile.d/opencv-python.sh \
  && echo "${CV2_PARENT}" > /etc/opencv-python-path

# ROS 2 Humble apt repo (Ubuntu 22.04 / Jammy).
# Split installs + clear apt archives between batches — Orin Nano disk is tight
# (~400MB download / ~1.7GB install for ros-base+mavros in one shot often fails with
# "not enough free space in /var/cache/apt/archives/").
RUN apt-get update && apt-get install -y --no-install-recommends \
    curl \
    gnupg2 \
    lsb-release \
    ca-certificates \
    python3 \
    python3-numpy \
  && rm -rf /var/lib/apt/lists/* /var/cache/apt/archives/*

RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
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
    git \
  && rm -rf /var/lib/apt/lists/* /var/cache/apt/archives/*

RUN apt-get update && apt-get install -y --no-install-recommends \
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
    # Do NOT install ros-humble-cv-bridge / python3-opencv from apt — CPU OpenCV.
  && rm -rf /var/lib/apt/lists/* /var/cache/apt/archives/* \
  && export PYTHONPATH="$(cat /etc/opencv-python-path)${PYTHONPATH:+:${PYTHONPATH}}" \
  && python3 -c "import cv2; print('OpenCV', cv2.__version__); assert hasattr(cv2, 'cuda'), 'cv2.cuda missing'"

RUN if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then rosdep init; fi \
  && rosdep update --rosdistro humble

# MAVROS needs the egm96-5 geoid. install_geographiclib_datasets.sh hits flaky
# SourceForge mirrors and can hang/retry for 30+ minutes in Docker builds.
RUN apt-get update && apt-get install -y --no-install-recommends bzip2 \
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

# Ubuntu 22.04's pip is too old for modern hatchling (needs packaging.licenses).
RUN pip3 install --no-cache-dir --upgrade pip packaging hatchling

COPY camera/ /monorepo/camera/
COPY utils/ /monorepo/utils/
# OpenCV was built against apt NumPy 1.x; keep pip on NumPy 1.x too.
RUN pip3 install --no-cache-dir "numpy<2" sortedcontainers \
  && pip3 install --no-cache-dir /monorepo/camera /monorepo/utils \
  && pip3 install --no-cache-dir "numpy<2"

WORKDIR /ros_ws

# Build cv_bridge against our CUDA OpenCV (not the apt CPU build).
RUN mkdir -p src \
  && git clone --depth 1 --branch humble \
       https://github.com/ros-perception/vision_opencv.git src/vision_opencv

COPY airside/src/ src/

# vision_opencv needs a C++ toolchain to build cv_bridge. CUDA runtime image
# has none. libboost-python-dev comes from universe (already on this base's
# sources.list — do not add a duplicate universe entry).
#
# CUDA-built OpenCVConfig calls FindCUDA (needs nvcc). The runtime image has
# no compiler toolkit; cv_bridge only links prebuilt OpenCV libs, so skip that
# find rather than installing a multi‑GB cuda-nvcc package.
RUN source /opt/ros/humble/setup.bash \
  && apt-get update \
  && apt-get install -y --no-install-recommends \
       build-essential \
       cmake \
       libboost-python-dev \
  && rm -rf /var/lib/apt/lists/* /var/cache/apt/archives/* \
  && sed -i \
       -e 's/find_host_package([[:space:]]*CUDA/# find_host_package(CUDA/g' \
       -e 's/find_package([[:space:]]*CUDA/# find_package(CUDA/g' \
       /opt/opencv/lib/cmake/opencv4/OpenCVConfig.cmake \
  && rosdep install --from-paths src --ignore-src --rosdistro humble -y \
       --skip-keys "ament_python libopencv-dev python3-opencv cv_bridge python3-sortedcontainers" \
  && colcon build --symlink-install \
       --cmake-args -DOpenCV_DIR=/opt/opencv/lib/cmake/opencv4

COPY airside/docker/airside_entrypoint.sh /airside_entrypoint.sh
RUN chmod +x /airside_entrypoint.sh

# Quick CUDA module check (device count needs --runtime nvidia at run time).
RUN pip3 install --no-cache-dir "numpy<2" \
  && export PYTHONPATH="$(cat /etc/opencv-python-path)${PYTHONPATH:+:${PYTHONPATH}}" \
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
