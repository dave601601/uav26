FROM ghcr.io/px4/px4-dev:v1.16.2
# base: Ubuntu 24.04 (Noble) → ROS2 Jazzy
ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]

# locale (ROS2 필수)
RUN apt update && apt install -y locales && \
    locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG=en_US.UTF-8

# # ROS2 apt repo — 공식 ros2-apt-source 방식
# RUN apt update && apt install -y software-properties-common curl ca-certificates && \
#     add-apt-repository universe -y && \
#     ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \
#         | grep -F "tag_name" | awk -F\" '{print $4}') && \
#     curl -L -o /tmp/ros2-apt-source.deb \
#         "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb" && \
#     apt install -y /tmp/ros2-apt-source.deb && \
#     rm /tmp/ros2-apt-source.deb


# ROS2 apt repo (수동 설치 - GitHub API 안 거침)
RUN apt update && apt install -y software-properties-common curl gnupg lsb-release ca-certificates && \
    add-apt-repository universe -y && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
        -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
        > /etc/apt/sources.list.d/ros2.list
        
RUN apt update && apt install -y \
    ros-jazzy-desktop \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    python3-argcomplete \
    && rm -rf /var/lib/apt/lists/*

# rosdep
RUN rosdep init || true
RUN rosdep update

# 자동 source
RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc

WORKDIR /workspace