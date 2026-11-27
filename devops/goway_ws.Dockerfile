FROM ros:jazzy

USER root

# Install dependencies
RUN apt-get update && \
    apt-get install -y \
    git \
    build-essential \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-pip \
    libyaml-cpp-dev \
    libboost-dev \
    libeigen3-dev \
    ros-jazzy-rmw-cyclonedds-cpp \
    ros-jazzy-rosidl-generator-dds-idl \
    nano \
    openssh-server \
    net-tools \
    wireless-tools \
    usbutils \
    pciutils \
    htop \
    ros-jazzy-spacenav \
    iputils-ping \
    iproute2 \
    lsof \
    ros-jazzy-rosbridge-suite \
    ros-jazzy-librealsense2* \
    ros-jazzy-realsense2-* \
    ros-jazzy-navigation2 \
    ros-jazzy-nav2-bringup \
    libpcl-dev \
    libeigen3-dev \
    ros-jazzy-yasmin \
    ros-jazzy-yasmin-* \
    ros-jazzy-pointcloud-to-laserscan \
    ros-jazzy-spatio-temporal-voxel-layer \
    ros-jazzy-pcl-ros \
    ros-jazzy-pcl-conversions \
    ros-jazzy-slam-toolbox \
    cmake \
    g++ \
    libboost-all-dev \
    libfmt-dev \
    libcap2-bin && \
    getcap -r /usr /opt 2>/dev/null | cut -d' ' -f1 | sort -u | xargs -r -n1 setcap -r

# Setup ROS environment
SHELL ["/bin/bash", "-c"]
RUN rosdep update

# Rename existing UID-1000 'ubuntu' user to 'htxgrrt' instead of creating a new one
RUN usermod -l htxgrrt -d /home/htxgrrt -m ubuntu \
    && groupmod -n htxgrrt ubuntu

# Grant sudo privileges to 'htxgrrt' without password
RUN echo "htxgrrt ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

# Switch to user 'htxgrrt'
USER htxgrrt
WORKDIR /home/htxgrrt

# Create folders
RUN mkdir -p /home/htxgrrt/.htxgrrt/app/goaway_ws
WORKDIR /home/htxgrrt/.htxgrrt/app/goaway_ws

# Setup ROS environment for the user
RUN echo "source /opt/ros/jazzy/setup.bash" >> /home/htxgrrt/.bashrc
RUN echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> /home/htxgrrt/.bashrc
RUN echo "source /home/htxgrrt/.htxgrrt/app/goaway_ws/custom.env" >> /home/htxgrrt/.bashrc
RUN echo "PS1='\\[\\033[1;38;2;85;52;128m\\]\\u@\\h\\[\\033[00m\\]:\\[\\033[00;31m\\]\\w\\[\\033[00m\\]\\$ '" >> /home/htxgrrt/.bashrc

# Some shortcuts aliases for convenience
# check disk usage of directories
RUN echo "alias dir_du='du -h --max-depth=1 | sort -hr'" >> /home/htxgrrt/.bashrc
# build in Release mode, skipping tests and linters
RUN echo "alias build_all='colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF'" >> /home/htxgrrt/.bashrc
# same, but with tests and linters enabled
RUN echo "alias build_all_test='colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON'" >> /home/htxgrrt/.bashrc
# tar the install folder
RUN echo "alias tar_install='tar -cvzf install.tar.gz install/'" >> /home/htxgrrt/.bashrc

# build and install unitree_sdk2, required to build deploy, runtime not needed
RUN mkdir -p /home/htxgrrt/.htxgrrt/tmp && \
    cd /home/htxgrrt/.htxgrrt/tmp && \
    git clone --depth=1 https://github.com/unitreerobotics/unitree_sdk2.git && \
    cd unitree_sdk2 && \
    mkdir build && cd build && \
    cmake .. -DCMAKE_INSTALL_PREFIX=/opt/unitree_robotics -DBUILD_EXAMPLES=OFF && \
    sudo make install && \
    rm -rf /tmp/unitree_sdk2

# build and install LivoxSDK2, required to build fastlio, runtime not needed
RUN mkdir -p /home/htxgrrt/.htxgrrt/tmp && \
    git clone --depth=1 https://github.com/Livox-SDK/Livox-SDK2.git && \
    cd Livox-SDK2 && \
    mkdir build && cd build && \
    cmake .. && \
    make -j$(nproc) && \
    sudo make install && \
    rm -rf /tmp/Livox-SDK2

# Default entrypoint
ENTRYPOINT ["/bin/bash"]