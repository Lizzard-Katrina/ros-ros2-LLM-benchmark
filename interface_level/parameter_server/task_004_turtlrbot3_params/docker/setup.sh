#!/bin/bash
docker build -t ros1_turtlebot3_params .
docker run -it ros1_turtlebot3_params /bin/bash
