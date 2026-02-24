# Dockerfile for Szuflada V2 -- ROS2 Jazzy rover workspace
# ========================================================
#
# Build:
#   docker build -t orionrover_img .
#
# Run (creates container named OrionRover):
#   docker run -it --privileged --name OrionRover \
#     --net=host \
#     --env="DISPLAY=$DISPLAY" \
#     --env="QT_X11_NO_MITSHM=1" \
#     -v /dev/bus/usb:/dev/bus/usb \
#     --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
#     orionrover_img
#
# Architecture:
#   mqtt_bridge (ONE MQTT connection) <-> all other nodes via ROS2 topics

# 1. Base image: ROS2 Jazzy
FROM docker.io/library/ros:jazzy-ros-base

# 2. Install ROS2 packages + Python dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-jazzy-desktop \
    ros-jazzy-depthimage-to-laserscan \
    python3-pip \
    python3-paho-mqtt \
    mosquitto \
    mosquitto-clients \
    && rm -rf /var/lib/apt/lists/*

# 3. Install Python packages
RUN pip3 install --break-system-packages \
    pyserial \
    paho-mqtt

# 4. Create workspace and copy source
RUN mkdir -p /root/rover_ws/src
COPY rover_autonomy /root/rover_ws/src/rover_autonomy

# 5. Build the workspace
WORKDIR /root/rover_ws
RUN /bin/bash -c "source /opt/ros/jazzy/setup.bash && colcon build --symlink-install" || true

# 6. Auto-source ROS2 and workspace on shell start
RUN echo 'source /opt/ros/jazzy/setup.bash' >> /root/.bashrc \
    && echo 'source /root/rover_ws/install/setup.bash 2>/dev/null || true' >> /root/.bashrc

CMD ["bash"]
