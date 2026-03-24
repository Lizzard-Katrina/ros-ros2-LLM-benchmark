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

COPY ./ros2_code /ros2_ws/src/task_002
#COPY ./CMakeLists.txt /ros2_ws/src/task_006
#COPY ./package.xml /ros2_ws/src/task_006
COPY ./tests \
     /ros2_ws/src/task_002/test
#for this task
RUN touch /ros2_ws/src/task_002/__init__.py

	
# Source ROS2
SHELL ["/bin/bash", "-c"]
RUN source /opt/ros/humble/setup.bash && \
    cd /ros2_ws && \
    colcon build --symlink-install



# Set ROS2 workspace
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "source /ros2_ws/install/setup.bash" >> ~/.bashrc

WORKDIR /ros2_ws


# Set entrypoint
#ENTRYPOINT ["/entrypoint.sh"]
# Default command
CMD ["/bin/bash"]
