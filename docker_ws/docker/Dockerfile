# Start from official ROS 2 Humble desktop image
FROM osrf/ros:humble-desktop
# ----------------------------
# Environment variables
# ----------------------------
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble
ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8
ENV TZ=UTC
# ----------------------------
# Basic utilities + lsb-release
# ----------------------------
RUN apt-get update && apt-get install -y \
    wget \
    lsb-release \
    tmux \
    gedit \
    && rm -rf /var/lib/apt/lists/*
# ----------------------------
# Gazebo repository
# ----------------------------
RUN sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" > /etc/apt/sources.list.d/gazebo-stable.list' \
    && wget -qO - https://packages.osrfoundation.org/gazebo.key | apt-key add -
# ----------------------------
# Gazebo + ROS–Gazebo integration
# ----------------------------
RUN apt-get update && apt-get install -y \
    gazebo \
    libgazebo-dev \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-ros-gz \
    ros-humble-gazebo-ros2-control \
    ros-humble-py-binding-tools \
    ros-humble-pcl-ros \
    && rm -rf /var/lib/apt/lists/*
# ----------------------------
# Development tools
# ----------------------------
RUN apt-get update && apt-get install -y \
    build-essential \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    git \
    nano \
    && rosdep update --rosdistro $ROS_DISTRO \
    && rm -rf /var/lib/apt/lists/*
# ----------------------------
# X11 / Qt support for GUI apps
# ----------------------------
RUN apt-get update && apt-get install -y \
    x11-apps \
    libx11-xcb1 \
    libxcb-xfixes0 \
    libqt5x11extras5 \
    qtbase5-dev \
    && rm -rf /var/lib/apt/lists/*
# ----------------------------
# ROS 2 core tools
# ----------------------------
RUN apt-get update && apt-get install -y \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-xacro \
    ros-humble-rviz2 \
    ros-humble-joint-state-publisher \
    ros-humble-joint-state-publisher-gui \
    && rm -rf /var/lib/apt/lists/*
# ----------------------------
# MoveIt 2
# ----------------------------
RUN apt-get update && apt-get install -y \
    ros-humble-moveit \
    ros-humble-moveit-visual-tools \
    && rm -rf /var/lib/apt/lists/*
# ----------------------------
# Gazebo transport (needed by some plugins)
# ----------------------------
RUN apt-get update && apt-get install -y \
    libgz-transport11-dev \
    && rm -rf /var/lib/apt/lists/*
# ----------------------------
# Create non-root user
# ----------------------------
ARG USERNAME=rosdev
ARG USER_UID=1000
ARG USER_GID=$USER_UID
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME
# ----------------------------
# Create ROS 2 workspace
# ----------------------------
WORKDIR /home/$USERNAME/arm_ws
RUN mkdir -p src && chown -R $USERNAME:$USERNAME /home/$USERNAME
# ----------------------------
# Switch to non-root user
# ----------------------------
USER $USERNAME
WORKDIR /home/$USERNAME/arm_ws
# ----------------------------
# Source ROS + Gazebo on startup
# ----------------------------
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /home/$USERNAME/.bashrc \
    && echo "source /usr/share/gazebo-11/setup.sh" >> /home/$USERNAME/.bashrc
# ----------------------------
# Entrypoint
# ----------------------------
COPY docker/entrypoint.sh /entrypoint.sh
RUN sudo chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
