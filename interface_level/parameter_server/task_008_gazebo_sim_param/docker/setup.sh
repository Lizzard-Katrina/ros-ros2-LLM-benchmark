#!/bin/bash
docker build -t ros1_gazebo_param .
docker run -it ros1_gazebo_param /bin/bash
