#!/usr/bin/env bash

SCRIPT_DIR=$(cd $(dirname $0); pwd)
source $SCRIPT_DIR/helper_functions.sh

# Update vcs versions
echo -e "\e[36mUpdate vcs versions\e[m"
source $SCRIPT_DIR/update_vcs_versions.sh

# Commit autoware.proj
git checkout --detach --quiet HEAD
git add $(get_repos_file_path)
git commit -m "Update $(basename $(get_repos_file_path))"
