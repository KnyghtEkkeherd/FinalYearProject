# === Stage 1: Base System Setup ===
ARG ROS_DISTRO=jazzy
FROM ros:${ROS_DISTRO}-ros-base AS core

# Set environment variables
ENV PIP_BREAK_SYSTEM_PACKAGES=1 \
    DEBIAN_FRONTEND=noninteractive

# Install core system tools
RUN apt-get update -q && apt-get upgrade -yq && \
    apt-get install -yq --no-install-recommends \
        python3-pip \
        xacro \
    && rm -rf /var/lib/apt/lists/*

# Install pip packages
RUN apt-get update -q && python3 -m pip install cv_bridge opencv-python pyserial pyserial-asyncio numpy && \
    rm -rf /var/lib/apt/lists/*

# Use bash instead of sh
SHELL ["/bin/bash", "-c"]

# === Stage 2: Build ===
FROM core AS build

# Move to the build workspace
ARG BUILD_WS=/build_ws
WORKDIR ${BUILD_WS}

# Build colcon workspace and install dependencies
RUN mkdir -p ${BUILD_WS}/src
RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
 && apt-get update -y \
 && rosdep install --from-paths src --ignore-src --rosdistro ${ROS_DISTRO} -y \
 && colcon build --symlink-install

# Set up the entrypoint
COPY ./docker/entrypoint.sh /tmp/entrypoint.sh
RUN chmod +x /tmp/entrypoint.sh

# === Stage 3: Overlay (source code) ===
FROM build AS workspace

# Create an overlay workspace
ARG WORKSPACE=/fyp_ws
RUN mkdir -p ${WORKSPACE}/src
WORKDIR ${WORKSPACE}

# Copy the source code
COPY . ${WORKSPACE}

# Source the ROS 2 workspace
RUN source ${BUILD_WS}/install/setup.bash \
 && rosdep install --from-paths src --ignore-src --rosdistro ${ROS_DISTRO} -y \
 && colcon build --symlink-install

# Set up the entrypoint
COPY --from=build /tmp/entrypoint.sh /entrypoint.sh
ENTRYPOINT [ "/entrypoint.sh" ]

# === Stage 4: Development ===
FROM workspace AS dev

# Set up user
ARG USERNAME=ros
ARG USER_UID=1001
ARG USER_GID=${USER_UID}

# Give birth, linux style
RUN groupadd --gid ${USER_GID} ${USERNAME} \
  && useradd --uid ${USER_UID} --gid ${USER_GID} --create-home ${USERNAME}

# Add to sudoers and give home
RUN echo ${USERNAME} ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/${USERNAME} \
  && chmod 0440 /etc/sudoers.d/${USERNAME} \
  && mkdir -p /home/${USERNAME} \
  && chown -R ${USER_UID}:${USER_GID} /home/${USERNAME}

# Give user our new workspace
RUN chown -R ${USER_UID}:${USER_GID} ${WORKSPACE}

# Set to enter as user
USER ${USERNAME}
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /home/${USERNAME}/.bashrc