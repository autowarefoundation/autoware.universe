#!/bin/bash

SCRIPT_DIR=$(readlink -f "$(dirname "$0")")

AUTOWARE_PROJ_ROOT="$SCRIPT_DIR/../"
DOCKER_BUILDKIT=1 docker build -t autoware:base --ssh default -f "$SCRIPT_DIR/Dockerfile.base" "$AUTOWARE_PROJ_ROOT"
DOCKER_BUILDKIT=1 docker build -t autoware:pre-built --ssh default -f "$SCRIPT_DIR/Dockerfile.pre-built" "$AUTOWARE_PROJ_ROOT"
