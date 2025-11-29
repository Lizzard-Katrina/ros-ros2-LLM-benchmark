#!/bin/bash
docker build -t px4_flight_params -f docker/Dockerfile .
docker run -it --rm px4_flight_params
