#!/bin/bash
docker build -t ros1_speed_test .
docker run -it ros1_speed_test /bin/bash
