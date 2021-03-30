#!/usr/bin/env bash

SCRIPT_DIR=$(cd $(dirname $0); pwd)
source $SCRIPT_DIR/helper_functions.sh

# Define functions
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
