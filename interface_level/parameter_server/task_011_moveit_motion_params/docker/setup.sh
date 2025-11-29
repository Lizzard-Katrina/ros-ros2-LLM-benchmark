#!/bin/bash
docker build -f Dockerfile -t moveit_motion_params ..
docker run -it sensor_fusion_params /bin/bash
