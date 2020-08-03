#!/usr/bin/env bash

SCRIPT_DIR=$(cd $(dirname $0); pwd)
source $SCRIPT_DIR/helper_functions.sh

# Define functions
function show_usage() {
  echo -e "Usage: release_candidate.sh [--push|--delete] autoware_version
    --push:
      Push branches or tags of autoware repositories. Please use this option when you can be sure.

    --delete:
      Delete branches or tags of autoware repositories. Please use this option when you mistook something.

    autoware_version:
      The version to be used for branches or tags.
      The valid pattern for release_candidate.sh is '^v([0-9]+)\.([0-9]+)$'.
      The valid pattern for release.sh is '^v([0-9]+)\.([0-9]+)\.([0-9]+)$'.

    Note: Using --push and --delete at the same time may cause unexpected behaviors."
}

function parse_args(){
  while [ "${1:-}" != "" ]; do
    case "$1" in
      "-h" | "--help")
        help=true
        ;;
      "--push")
        push=true
        ;;
      "--delete")
        delete=true
        ;;
      *)
        autoware_version="$1"
    esac
    shift
  done
}

# Install prerequisites
install_yq_if_not_installed

# Parse arguments
parse_args $@

if [ "$help" ]; then
  show_usage
  exit 0
fi

if [ "$push" ]; then
  read -p "You are going to push branches or tags. Do you really want to continue? [y/N] " answer

  case $answer in
    [yY]* )
      ;;
    * )
      echo -e "\e[33mCanceled \e[0m"
      exit 1
      ;;
  esac
fi

if [ "$delete" ]; then
  read -p "You are going to delete branches or tags. Do you really want to continue? [y/N] " answer

  case $answer in
    [yY]* )
      ;;
    * )
      echo -e "\e[33mCanceled \e[0m"
      exit 1
      ;;
  esac
fi

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
