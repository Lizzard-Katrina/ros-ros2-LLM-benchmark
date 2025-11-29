#!/bin/bash

docker build -f Dockerfile -t slam_mapping_params .
docker run -it --rm slam_mapping_params
