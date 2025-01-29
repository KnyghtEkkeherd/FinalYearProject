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
ENV ROS_WS=/root/FinalYearProject \
    SHARED_ROS=/root/shared/

# Install additional ROS tools
RUN apt-get update -q && \
    apt-get install -yq --no-install-recommends \
        gnupg2 \
        iputils-ping \
        usbutils \
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
        ros-${ROS_DISTRO}-rqt-reconfigure && \
    rm -rf /var/lib/apt/lists/*

# Install Mesa for 3D
RUN apt-get update -q && \
    apt-get install -yq --no-install-recommends software-properties-common && \
    add-apt-repository ppa:kisak/kisak-mesa && \
    rm -rf /var/lib/apt/lists/*

# Prepare ROS workspace directory
RUN mkdir -p $ROS_WS

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
FROM camera AS src

# Copy only necessary files for module setup
COPY docker/workspace.sh /root/
COPY docker/entrypoint.sh /root/
COPY docker/custom_bashrc /root/.bashrc
COPY . $ROS_WS

# Make scripts executable
RUN chmod +x /root/workspace.sh /root/entrypoint.sh

# Set up ROS2 environment
RUN rosdep update

# Install workspace dependencies and build
WORKDIR /root
RUN ./workspace.sh

# === Stage 4: Final Image ===
FROM src AS final

# Set container entrypoint
WORKDIR /root
ENTRYPOINT ["/root/entrypoint.sh"]

# Run container indefinitely for exec-ability
CMD ["/bin/bash", "-c", "tail -f /dev/null"]