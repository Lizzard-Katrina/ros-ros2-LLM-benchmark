#!/bin/bash
docker build -t ros1_dynamic_param_rqt .
docker run -it ros1_dynamic_param_rqt /bin/bash
