#!/bin/bash
docker build -t ros1_param_test .
docker run -it ros1_param_test /bin/bash
