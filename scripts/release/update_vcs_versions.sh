#!/usr/bin/env bash

SCRIPT_DIR=$(readlink -f "$(dirname "$0")")
source "$SCRIPT_DIR/common/helper_functions.sh"

# Move to workspace root
cd "$(get_workspace_root)" || exit 1

# Update repos file
update_vcs_versions
