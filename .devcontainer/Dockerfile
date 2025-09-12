FROM ros:humble-ros-base
RUN apt-get update && apt-get install -y --no-install-recommends         build-essential cmake git python3-colcon-common-extensions         ros-humble-ros2-control ros-humble-ros2-controllers         ros-humble-realtime-tools ros-humble-pluginlib         ros-humble-rclcpp ros-humble-controller-interface         ros-humble-hardware-interface ros-humble-robot-localization         && rm -rf /var/lib/apt/lists/*
RUN useradd -ms /bin/bash vscode && usermod -aG sudo vscode && echo 'vscode ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers
USER vscode
WORKDIR /workspaces/ros2-adaptive-control
