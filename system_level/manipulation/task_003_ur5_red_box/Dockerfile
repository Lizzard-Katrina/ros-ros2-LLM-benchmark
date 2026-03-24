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

COPY ./ros2_code/robotic_arm_arm_ikfast_moveit_plugin.cpp /ros2_ws/src/motion_planning/ros2_code/robotic_arm_arm_ikfast_moveit_plugin.cpp
COPY ./tests/CMakeLists.txt /ros2_ws/src/motion_planning/CMakeLists.txt
COPY ./tests/package.xml /ros2_ws/src/motion_planning/package.xml
COPY ./tests/test_ik_decision_loop.cpp /ros2_ws/src/motion_planning/test/test_ik_decision_loop.cpp
COPY ./tests/include /ros2_ws/src/motion_planning/include
COPY ./tests/include/kinematics/kinematics_query_options.h /ros2_ws/src/motion_planning/include/kinematics_base/kinematics_query_options.hpp

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
