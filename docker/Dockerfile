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
        bash-completion && \
    rm -rf /var/lib/apt/lists/*

# === Stage 2: ROS Dependencies ===
FROM core AS ros

# Set environment variables
ENV BUILD=/root/build

# Install additional ROS tools
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
        python3-vcstool && \
    apt-get install -yq --no-install-recommends \
        ros-${ROS_DISTRO}-joint-state-publisher-gui \
        ros-${ROS_DISTRO}-xacro \
        ros-${ROS_DISTRO}-demo-nodes-cpp \
        ros-${ROS_DISTRO}-demo-nodes-py \
        ros-${ROS_DISTRO}-rviz2 \
        ros-${ROS_DISTRO}-ros-gz \
        ros-${ROS_DISTRO}-rqt-reconfigure \
        ros-${ROS_DISTRO}-ros2-control \
        ros-${ROS_DISTRO}-ros2-controllers \
        ros-${ROS_DISTRO}-robot-localization \
        ros-${ROS_DISTRO}-pcl-ros \
        ros-${ROS_DISTRO}-ros2-controllers-test-nodes \
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

# === Stage 3: Source Code ===
FROM camera AS local

# Copy only necessary files for setup
COPY docker/custom_bashrc /root/.bashrc
COPY docker/workspace.sh docker/entrypoint.sh $BUILD
COPY src $BUILD/src

# Make scripts executable
RUN chmod +x $BUILD/workspace.sh $BUILD/entrypoint.sh

# Set up ROS2 environment
WORKDIR $BUILD
RUN rosdep update

# Install workspace dependencies and build
RUN $BUILD/workspace.sh

# === Stage 4: Final Image ===
FROM local AS final

# Set container entrypoint
ENTRYPOINT ["/root/build/entrypoint.sh"]

# Run container indefinitely for exec-ability
CMD ["/bin/bash", "-c", "tail -f /dev/null"]