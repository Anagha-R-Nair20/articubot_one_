# Base image
FROM osrf/ros:iron-desktop-full

# Set display environment variable for GUI support
ENV DISPLAY=:0

RUN apt-get update && apt install -y software-properties-common && \
    add-apt-repository universe

RUN apt update &&  apt install -y curl gnupg2 lsb-release
# RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key  -o /usr/share/keyrings/ros-archive-keyring.gpg 
# RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -sc)  main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install additional dependencies for GUI and Gazebo
RUN apt-get update && apt-get install -y \
    git wget qtbase5-dev qtchooser qt5-qmake qtbase5-dev-tools \
    python3-rosdep \
    python3-vcstool \
    python3-colcon-common-extensions \
    x11-apps \
    mesa-utils \
    ros-iron-desktop \
    ros-iron-xacro \
    ros-iron-joint-state-publisher-gui \
    ros-iron-gazebo-ros-pkgs \
    ros-iron-twist-mux \
    ros-iron-ros2-control ros-iron-ros2-controllers ros-iron-gazebo-ros2-control \
    ros-iron-slam-toolbox ros-iron-navigation2 ros-iron-nav2-bringup \
    ros-iron-turtlebot3* ros-iron-rmw-cyclonedds-cpp \
    joystick jstest-gtk evtest



#https://github.com/ros2/rosidl/issues/680#issuecomment-1123964009
RUN apt-get update && apt-get dist-upgrade -y

# Set up a user to avoid running as root
ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=$USER_UID
#adding for joystick support without sudo
#https://www.reddit.com/r/linuxquestions/comments/bh4ex1/is_adding_a_user_to_input_group_secure/
#https://stackoverflow.com/questions/29357095/linux-how-to-get-group-id-from-group-name-and-vice-versa


# ARG INPUT_GID=107          

# in this build with osrf/ros:iron-desktop-full there is no "input" group but instead there is an "rdma" group
# which seems to be its equivalent(?..no source) 
RUN groupadd --gid $USER_GID $USERNAME \
    # && groupadd --gid $INPUT_GID input \
    # && groupadd --gid $INPUT_GID dialout \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && usermod -a -G dialout  $USERNAME \
    && usermod -a -G rdma  $USERNAME                   
    # && useradd --uid $USER_UID --gid $INPUT_GID

# RUN groupadd --gid $INPUT_GID input 

RUN apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

#RUN wget https://s3.us-west-1.amazonaws.com/download.behaviortree.dev/groot2_linux_installer/Groot2-v1.5.0-linux-installer.run 
#RUN chmod 777 Groot2-v1.5.0-linux-installer.run 
#&& ./Groot2-v1.5.0-linux-installer.run
ENV RMW_IMPLEMENTATION="rmw_cyclonedds_cpp"
WORKDIR /app
COPY ros_entrypoint.sh .
ENV CYCLONEDDS_URI="/app/cyclonedds.xml"
USER $USERNAME
COPY cyclonedds.xml /app
RUN echo 'source  /opt/ros/iron/setup.bash' >> /home/${USERNAME}/.bashrc
RUN echo 'source  /app/ros2_ws/install/local_setup.bash' >> /home/${USERNAME}/.bashrc
RUN echo 'export TURTLEBOT3_MODEL=waffle' >> /home/${USERNAME}/.bashrc
RUN echo 'export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/iron/share/turtlebot3_gazebo/models' >> /home/${USERNAME}/.bashrc
RUN echo 'export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/code/ros2_ws/src/aws_worlds/aws-robomaker-small-warehouse-world/models' >> /home/${USERNAME}/.bashrc

# ENV TURTLEBOT3_MODEL=waffle  
# ENV GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/iron/share/turtlebot3_gazebo/models'
# Set the entry point for the container

# ENTRYPOINT [ "/app/ros_entrypoint.sh" ]
CMD ["bash"]