#!/bin/bash
docker build -f Dockerfile -t fetch_arm_params .
docker run -it fetch_arm_params /bin/bash

