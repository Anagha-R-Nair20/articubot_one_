# Base image for ARM architecture (Raspberry Pi 4)
FROM arm64v8/ros:iron-ros-base

# Set display environment variable for GUI support
ENV DISPLAY=:0

# Add the ROS 2 repository for Iron on ARM64
RUN apt-get update && apt-get install -y curl gnupg2 lsb-release
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | sudo apt-key add -
RUN sh -c 'echo "deb [arch=arm64] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'

# Install dependencies
RUN apt-get update && apt-get install -y \
   # gazebo11 \
   # libgazebo11-dev \
    ros-iron-desktop \
    git wget qtbase5-dev qtchooser qt5-qmake qtbase5-dev-tools \
    python3-rosdep \
    python3-vcstool \
    python3-serial\
    python3-colcon-common-extensions \
    x11-apps \
    mesa-utils \
    ros-iron-xacro \
    ros-iron-joint-state-publisher-gui \
    ros-iron-twist-mux \
    ros-iron-ros2-control \
    ros-iron-ros2-controllers \
    ros-iron-slam-toolbox \
    ros-iron-navigation2 \
    ros-iron-turtlebot3* \
    ros-iron-rmw-cyclonedds-cpp \
    ros-iron-rplidar-ros\
    libserial-dev\
    joystick jstest-gtk evtest
# Set up a non-root user for running ROS
ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Create the user and add to necessary groups
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && usermod -a -G dialout $USERNAME \
    && usermod -a -G rdma $USERNAME

# Allow the user to run sudo commands without a password
RUN apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

# Set up ROS 2 CycloneDDS configuration
ENV RMW_IMPLEMENTATION="rmw_cyclonedds_cpp"
WORKDIR /app
COPY ros_entrypoint.sh .
ENV CYCLONEDDS_URI="/app/cyclonedds.xml"
USER $USERNAME
COPY cyclonedds.xml /app

# Set up environment for ROS 2
RUN echo 'source /opt/ros/iron/setup.bash' >> /home/${USERNAME}/.bashrc
RUN echo 'source /app/ros2_ws/install/local_setup.bash' >> /home/${USERNAME}/.bashrc
RUN echo 'export TURTLEBOT3_MODEL=waffle' >> /home/${USERNAME}/.bashrc
RUN echo 'export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/iron/share/turtlebot3_gazebo/models' >> /home/${USERNAME}/.bashrc
RUN echo 'export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/code/ros2_ws/src/aws_worlds/aws-robomaker-small-warehouse-world/models' >> /home/${USERNAME}/.bashrc

# Set the entry point for the container
CMD ["bash"]

