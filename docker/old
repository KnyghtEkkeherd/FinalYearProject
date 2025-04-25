# === Stage 1: Base System Setup (Core Dependencies) ===
ARG ROS_DISTRO=jazzy
FROM ros:${ROS_DISTRO}-ros-base AS core

# Set environment variables
ENV PIP_BREAK_SYSTEM_PACKAGES=1 \
    DEBIAN_FRONTEND=noninteractive

# Use bash instead of sh
SHELL ["/bin/bash", "-c"]

# Install core system tools
RUN apt-get update -q && apt-get upgrade -yq && \
    apt-get install -yq --no-install-recommends \
        wget \
        curl \
        git \
        build-essential \
        python3-pip \
        udev \
        bash-completion && \
    rm -rf /var/lib/apt/lists/*

# Set up user
ARG USERNAME=ros
ARG USER_UID=1001
ARG USER_GID=$USER_UID

RUN groupadd --gid $USER_GID $USERNAME \
  && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
  && mkdir /home/$USERNAME/.config && chown $USER_UID:$USER_GID /home/$USERNAME/.config

# Set up sudo for user
RUN apt-get update \
  && apt-get install -y sudo \
  && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME\
  && chmod 0440 /etc/sudoers.d/$USERNAME \
  && rm -rf /var/lib/apt/lists/*

# Fix dma_heap device permissions and ensure the directory exists
# https://raspberrypi.stackexchange.com/questions/141106/how-to-fix-the-libcamera-error-could-not-open-any-dmaheap-device
RUN mkdir -p /etc/udev/rules.d \
  && echo 'SUBSYSTEM=="dma_heap", GROUP="video", MODE="0660"' > /etc/udev/rules.d/rspi.rules
#   && udevadm control --reload-rules && udevadm trigger

# Add user to video group
RUN usermod -aG video $USERNAME

# === Stage 2: ROS Dependencies ===
FROM core AS ros

# Set environment variables
ENV BUILD=/home/ros/build

# Install ROS 2 core dependencies
RUN apt-get update -q && \
    apt-get install -yq --no-install-recommends \
        gnupg2 \
        iputils-ping \
        usbutils \
        libserial-dev \
        python3-argcomplete \
        python3-colcon-common-extensions \
        python3-networkx \
        python3-pip \
        python3-rosdep \
        python3-vcstool \
    && apt-get install -yq --no-install-recommends \
        ros-${ROS_DISTRO}-joint-state-publisher-gui \
        ros-${ROS_DISTRO}-xacro \
        ros-${ROS_DISTRO}-demo-nodes-cpp \
        ros-${ROS_DISTRO}-demo-nodes-py \
        ros-${ROS_DISTRO}-rviz2 \
    && rm -rf /var/lib/apt/lists/*

# Install ROS 2 navigation dependencies
RUN apt-get update -q && \
    apt-get install -yq --no-install-recommends \
        ros-${ROS_DISTRO}-ros-gz \
        ros-${ROS_DISTRO}-rqt-reconfigure \
        ros-${ROS_DISTRO}-ros2-control \
        ros-${ROS_DISTRO}-ros2-controllers \
        ros-${ROS_DISTRO}-robot-localization \
        ros-${ROS_DISTRO}-pcl-ros \
        ros-${ROS_DISTRO}-ros2-controllers-test-nodes \
    && rm -rf /var/lib/apt/lists/*

# Install ROS 2 camera dependencies
RUN apt-get update -q && \
    apt-get install -yq --no-install-recommends \
        libopencv-dev \
        python3-opencv \
    && apt-get install -yq --no-install-recommends \
        ros-${ROS_DISTRO}-rclpy \
        ros-${ROS_DISTRO}-sensor-msgs \
    && rm -rf /var/lib/apt/lists/*

# Install Mesa for 3D
RUN apt-get update -q && \
    apt-get install -yq --no-install-recommends software-properties-common && \
    add-apt-repository ppa:kisak/kisak-mesa && \
    rm -rf /var/lib/apt/lists/*

# Prepare ROS workspace build directory
RUN mkdir -p $BUILD

# === Stage 2.1: Camera Setup ===
FROM ros AS camera

# Install libcamera dependencies
RUN apt-get update -q && apt-get install -yq --no-install-recommends \
    libboost-dev \
    libgnutls28-dev \
    openssl \
    libtiff5-dev \
    pybind11-dev \
    qtbase5-dev \
    libqt5core5a \
    libqt5gui5 \
    libqt5widgets5 \
    meson \
    cmake \
    python3-yaml \
    python3-ply \
    libglib2.0-dev \
    libepoxy-dev \
    libgstreamer-plugins-base1.0-dev && \
    python3 -m pip install jinja2 ply && \
    rm -rf /var/lib/apt/lists/*

# Clone and build libcamera
RUN git clone https://github.com/raspberrypi/libcamera.git && \
    cd libcamera && \
    meson setup build --buildtype=release -Dpipelines=rpi/vc4,rpi/pisp -Dipas=rpi/vc4,rpi/pisp -Dv4l2=true -Dgstreamer=enabled -Dtest=false -Dlc-compliance=disabled -Dcam=disabled -Dqcam=disabled -Ddocumentation=disabled -Dpycamera=enabled && \
    ninja -C build && \
    ninja -C build install && \
    cd .. && \
    rm -rf libcamera && \
    rm -rf /var/lib/apt/lists/*

# Install rpicam-apps dependencies
RUN apt-get update -q && apt-get install -yq --no-install-recommends \
    cmake \
    libboost-program-options-dev \
    libdrm-dev \
    libexif-dev \
    meson \
    ninja-build && \
    rm -rf /var/lib/apt/lists/*

# Clone and build rpicam-apps
RUN git clone https://github.com/raspberrypi/rpicam-apps.git && \
    cd rpicam-apps && \
    meson setup build -Denable_libav=enabled -Denable_drm=enabled -Denable_egl=enabled -Denable_qt=enabled -Denable_opencv=disabled -Denable_tflite=disabled -Denable_hailo=disabled && \
    meson compile -C build && \
    meson install -C build && \
    ldconfig && \
    cd .. && \
    rm -rf rpicam-apps && \
    rm -rf /var/lib/apt/lists/*

# Add dtoverlay=imx477 to /boot/firmware/config.txt
# RUN echo "dtoverlay=imx477" >> /boot/firmware/config.txt

# Camera node dependencies
RUN apt-get update -q && python3 -m pip install cv_bridge opencv-python && \
    rm -rf /var/lib/apt/lists/*

# === Stage 3: Source Code ===
FROM camera AS local

# Copy only necessary files for setup
COPY docker/custom_bashrc /home/ros/.bashrc
COPY docker/workspace.sh docker/entrypoint.sh $BUILD
COPY src $BUILD/src

# Make scripts executable
RUN chmod +x $BUILD/workspace.sh $BUILD/entrypoint.sh

# Set up ROS2 environment
WORKDIR /home/ros/FinalYearProject
RUN rosdep update

# Install workspace dependencies and build
RUN $BUILD/workspace.sh

# === Stage 4: Final Image ===
FROM local AS final

# Set container entrypoint
ENTRYPOINT ["/home/ros/build/entrypoint.sh"]

# Run container indefinitely for exec-ability
CMD ["/bin/bash", "-c", "tail -f /dev/null"]