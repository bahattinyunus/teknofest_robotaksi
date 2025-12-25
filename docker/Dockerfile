FROM ros:humble-ros-base

# Install dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    libgl1-mesa-glx \
    && rm -rf /var/lib/apt/lists/*

# Install python requirements
COPY requirements.txt /tmp/
RUN pip3 install --no-cache-dir -r /tmp/requirements.txt

# Create workspace
WORKDIR /ros2_ws
COPY src /ros2_ws/src

# Build the workspace
RUN . /opt/ros/humble/setup.sh && colcon build

# Setup entrypoint
COPY docker/entrypoint.sh /
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
