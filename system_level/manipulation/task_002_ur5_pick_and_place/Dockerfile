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


# Create ROS2 workspace
RUN mkdir -p /ros2_ws/ros2_code
WORKDIR /ros2_ws

COPY ./ros2_code/controller.py /ros2_ws/src/motion_planning/controller.py
COPY ./ros2_code/motion_planning.py /ros2_ws/src/motion_planning/motion_planning.py
COPY ./tests/test_controller_oracle.py /ros2_ws/src/motion_planning/test/test_controller_oracle.py
COPY ./tests/test_motion_planning_oracle.py /ros2_ws/src/motion_planning/test/test_motion_planning_oracle.py
COPY ./pytest.ini /ros2_ws

RUN mkdir src/motion_planning/scripts\
    touch src/motion_planning/__init__.py && \
    touch src/motion_planning/scripts/__init__.py
	
# Source ROS2
SHELL ["/bin/bash", "-c"]
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN /bin/bash -c "source /opt/ros/humble/setup.bash"



# Set ROS2 workspace
WORKDIR /ros2_ws
RUN . /opt/ros/humble/setup.bash && colcon build

# Set entrypoint
ENTRYPOINT ["/bin/bash"]
