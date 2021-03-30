#!/usr/bin/env bash

SCRIPT_DIR=$(cd $(dirname $0); pwd)
source $SCRIPT_DIR/helper_functions.sh

# Install prerequisites
install_yq_if_not_installed

# Create src directory if not exist
if ! [ -d src ]; then
  mkdir -p src
fi

# Check workspace diff
echo -e "\e[36mCheck workspace diff\e[m"
if ! is_workspace_clean; then
  echo -e "\e[31mPlease clear your workspace diff.\e[m"
  show_workspace_diff
  exit 1
fi

# Update workspace
echo -e "\e[36mUpdate workspace\e[m"
if ! update_workspace; then
  echo -e "\n\e[31mFailed to update workspace.\e[m"
  exit 1
fi
