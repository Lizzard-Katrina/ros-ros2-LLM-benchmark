#!/bin/bash
docker build -t ros1_navigation_stack .
docker run -it ros1_navigation_stack /bin/bash
