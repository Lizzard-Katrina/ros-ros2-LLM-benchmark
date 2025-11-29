#!/bin/bash

# Absolute path of this script directory
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# Project root: one level above docker/
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"

# Build using PROJECT ROOT as context
docker build -t pr2_calib_params -f "$SCRIPT_DIR/Dockerfile" "$PROJECT_ROOT"

# Run the container
docker run -it --rm \
    pr2_calib_params \
    /bin/bash
