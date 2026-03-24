# Use ROS 2 Humble base image (includes rclpy and core ROS 2 libraries)
FROM ros:humble-ros-base

# Set working directory inside the container
WORKDIR /app

# Copy local project files into the container
COPY . /app

# Install Python dependencies for testing
RUN apt-get update && apt-get install -y python3-pip \
    && pip3 install --no-cache-dir numpy \
    && rm -rf /var/lib/apt/lists/*

# Set PYTHONPATH so Python can find ros2_code
ENV PYTHONPATH=/app

# Default command: run pytest in the tests directory
#CMD ["python3", "-m", "pytest", "-v", "tests"]

CMD ["python3", "-m", "pytest", "-v", "--disable-warnings", "-p", "no:launch_testing_ros_pytest_entrypoint", "tests"]
