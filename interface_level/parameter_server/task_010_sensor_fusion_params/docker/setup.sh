#!/bin/bash
docker build -f Dockerfile -t sensor_fusion_params ..
docker run -it sensor_fusion_params /bin/bash
