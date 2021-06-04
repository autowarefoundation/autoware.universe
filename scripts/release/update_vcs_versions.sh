#!/usr/bin/env bash

SCRIPT_DIR=$(readlink -f "$(dirname "$0")")
source "$SCRIPT_DIR/common/helper_functions.sh"

update_vcs_versions
