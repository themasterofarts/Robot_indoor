FROM osrf/ros:jazzy-desktop

ARG DEBIAN_FRONTEND=noninteractive
ARG USERNAME=vscode
ARG USER_UID=1000
ARG USER_GID=1000

SHELL ["/bin/bash", "-c"]

ENV ROS_DISTRO=jazzy
ENV ROS_WS=/workspaces/robot_indoor

RUN apt-get update && apt-get install -y --no-install-recommends \
    curl \
    gnupg \
    lsb-release \
    software-properties-common \
    && add-apt-repository universe \
    && curl -fsSL https://packages.osrfoundation.org/gazebo.gpg -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" > /etc/apt/sources.list.d/gazebo-stable.list \
    && apt-get update \
    && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    libgl1-mesa-dri \
    libgl1 \
    libglib2.0-0 \
    mesa-utils \
    pkg-config \
    python3-colcon-common-extensions \
    python3-pip \
    python3-requests \
    python3-rosdep \
    python3-vcstool \
    ros-dev-tools \
    ros-jazzy-joint-state-publisher \
    ros-jazzy-joint-state-publisher-gui \
    ros-jazzy-nav2-bringup \
    ros-jazzy-navigation2 \
    ros-jazzy-robot-state-publisher \
    ros-jazzy-ros-gz \
    ros-jazzy-ros-gz-bridge \
    ros-jazzy-ros-gz-image \
    ros-jazzy-ros-gz-sim \
    ros-jazzy-rviz2 \
    ros-jazzy-slam-toolbox \
    ros-jazzy-tf2-ros \
    ros-jazzy-xacro \
    sudo \
    xauth \
    && rm -rf /var/lib/apt/lists/*

RUN EXISTING_GROUP="$(getent group "${USER_GID}" | cut -d: -f1 || true)" \
    && if getent group "${USERNAME}" >/dev/null 2>&1; then \
      groupmod --gid "${USER_GID}" "${USERNAME}"; \
    elif [ -n "${EXISTING_GROUP}" ]; then \
      groupmod --new-name "${USERNAME}" "${EXISTING_GROUP}"; \
    else \
      groupadd --gid "${USER_GID}" "${USERNAME}"; \
    fi \
    && EXISTING_USER_BY_UID="$(getent passwd "${USER_UID}" | cut -d: -f1 || true)" \
    && if id -u "${USERNAME}" >/dev/null 2>&1; then \
      usermod --uid "${USER_UID}" --gid "${USER_GID}" --home "/home/${USERNAME}" --move-home --shell /bin/bash "${USERNAME}"; \
    elif [ -n "${EXISTING_USER_BY_UID}" ]; then \
      usermod --login "${USERNAME}" --uid "${USER_UID}" --gid "${USER_GID}" --home "/home/${USERNAME}" --move-home --shell /bin/bash "${EXISTING_USER_BY_UID}"; \
    else \
      useradd --uid "${USER_UID}" --gid "${USER_GID}" -m -s /bin/bash "${USERNAME}"; \
    fi \
    && echo "${USERNAME} ALL=(root) NOPASSWD:ALL" > "/etc/sudoers.d/${USERNAME}" \
    && chmod 0440 "/etc/sudoers.d/${USERNAME}" \
    && mkdir -p "${ROS_WS}" \
    && chown -R "${USER_UID}:${USER_GID}" "/home/${USERNAME}" "${ROS_WS}"

RUN rosdep init 2>/dev/null || true

RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> "/home/${USERNAME}/.bashrc" \
    && echo "if [ -f ${ROS_WS}/install/setup.bash ]; then source ${ROS_WS}/install/setup.bash; fi" >> "/home/${USERNAME}/.bashrc"

WORKDIR ${ROS_WS}
USER ${USERNAME}

CMD ["bash"]
