# Use ROS2 Humble Desktop as base
FROM osrf/ros:humble-desktop

# Install system dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    git \
    vim \
    && rm -rf /var/lib/apt/lists/*


# Install Python packages needed for oracle tests
RUN apt-get update && apt-get install -y python3-pip
RUN pip3 install --no-cache-dir pytest numpy pyquaternion
RUN pip3 install pytest

# Create ROS2 workspace
RUN mkdir -p /ros2_ws/ros2_code
WORKDIR /ros2_ws

COPY ./ros2_code /ros2_ws/src/task_003_image_transport
COPY ./tests/test_oracle_image_transport.py \
     /ros2_ws/src/task_003_image_transport/test/test_oracle_image_transport.py
#for this task
RUN touch /ros2_ws/src/task_003_image_transport/__init__.py

	
# Source ROS2
SHELL ["/bin/bash", "-c"]
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN /bin/bash -c "source /opt/ros/humble/setup.bash"



# Set ROS2 workspace
WORKDIR /ros2_ws
RUN . /opt/ros/humble/setup.bash && colcon build

# Set entrypoint
ENTRYPOINT ["/bin/bash"]
