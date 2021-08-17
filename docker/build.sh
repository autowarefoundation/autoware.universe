#!/bin/bash

SCRIPT_DIR=$(readlink -f "$(dirname "$0")")

AUTOWARE_PROJ_ROOT="$SCRIPT_DIR/../"
DOCKER_BUILDKIT=1 docker build -t autoware:base --ssh default -f "$SCRIPT_DIR/base/Dockerfile" "$AUTOWARE_PROJ_ROOT"
DOCKER_BUILDKIT=1 docker build -t autoware:pre-built --ssh default -f "$SCRIPT_DIR/pre-built/Dockerfile" "$AUTOWARE_PROJ_ROOT"
