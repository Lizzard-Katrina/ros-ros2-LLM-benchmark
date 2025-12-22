FROM navigation_base:latest

SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install -y \
    ros-noetic-husky-gazebo \
    ros-noetic-husky-navigation \
    ros-noetic-husky-control \
    ros-noetic-husky-description \
    && rm -rf /var/lib/apt/lists/*

COPY ros1_code /catkin_ws/src/benchmark_husky_nav
COPY docker/setup.sh /setup.sh

RUN chmod +x /setup.sh

CMD ["/setup.sh"]
