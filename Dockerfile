# Use the official ROS2 Humble base image
FROM osrf/ros:humble-desktop

# Set the environment variables
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8

# Update and install dependencies
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    libbullet-dev \
    python3-colcon-common-extensions \
    python3-pip \
    python3-vcstool \
    wget \
    && rm -rf /var/lib/apt/lists/*

# Create a workspace
RUN mkdir -p /home/akshat/ros2_ws/src
WORKDIR /home/akshat/ros2_ws

# Copy the ROS2 source code
COPY . /home/akshat/ros2_ws/src

# Install dependencies using rosdep
RUN rosdep update && \
    apt-get update && apt-get install -y \
    ros-humble-gazebo-msgs \
    ros-humble-gazebo-plugins \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-rtabmap-slam \
    ros-humble-rtabmap \
    ros-humble-nav2-bringup\
    && rosdep install --from-paths src --ignore-src -r -y

# Build the ROS2 workspace
RUN . /opt/ros/humble/setup.sh && \
    colcon build

# Source the setup files
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "source /home/akshat/ros2_ws/install/setup.bash" >> ~/.bashrc

# Default command
CMD ["bash"]
