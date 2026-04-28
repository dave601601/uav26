FROM ros:jazzy
# base: Ubuntu 24.04 (Noble) + ROS2 Jazzy (ros-base)
ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]

# locale
RUN apt update && apt install -y locales && \
    locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG=en_US.UTF-8

# OSRF repo (Gazebo Harmonic용)
RUN apt update && apt install -y curl gnupg lsb-release ca-certificates && \
    curl -sSL https://packages.osrfoundation.org/gazebo.gpg \
        -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
        > /etc/apt/sources.list.d/gazebo-stable.list

# ROS2 desktop + 빌드 툴
RUN apt update && apt install -y \
    ros-jazzy-desktop \
    python3-colcon-common-extensions \
    python3-vcstool \
    python3-argcomplete \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# RealSense ROS2 wrapper 의존성
RUN apt update && apt install -y \
    ros-jazzy-diagnostic-updater \
    ros-jazzy-diagnostic-aggregator \
    ros-jazzy-image-transport \
    ros-jazzy-image-transport-plugins \
    ros-jazzy-cv-bridge \
    ros-jazzy-camera-info-manager \
    ros-jazzy-image-publisher \
    ros-jazzy-librealsense2* \
    ros-jazzy-realsense2-camera-msgs \
    && rm -rf /var/lib/apt/lists/*

# Gazebo Harmonic + ROS2 bridge
RUN apt update && apt install -y \
    ros-jazzy-ros-gz \
    ros-jazzy-ros-gz-bridge \
    ros-jazzy-ros-gz-sim \
    ros-jazzy-ros-gz-image \
    ros-jazzy-rqt-image-view \
    ros-jazzy-rqt-graph \
    && rm -rf /var/lib/apt/lists/*

# rosdep update (init은 ros:jazzy에 이미 돼있음)
RUN rosdep update

# Python: opencv-contrib (aruco 포함) + trimesh + transforms3d
# numpy 는 <2 로 핀 — apt 의 scipy / trimesh 가 numpy 1.x ABI 로 빌드돼 있어
# numpy 2.x 를 깔면 깨짐 (ImportError: dtype size changed). opencv-contrib-python
# 4.13+ 는 numpy>=2 를 요구하므로 같이 <4.13 으로 핀 (4.12 까지 numpy 1.x 호환).
RUN pip install --break-system-packages --no-cache-dir --ignore-installed \
        'numpy<2' 'opencv-contrib-python<4.13' \
        trimesh transforms3d

# 자동 source
RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc

WORKDIR /workspace