#!/usr/bin/env bash

# Notice: This script supposes to be called from release scripts.

# Define functions
function parse_common_args(){
  args=()
  while [ "${1:-}" != "" ]; do
    case "$1" in
      "-h" | "--help")
        help=true
        ;;
      "-y" | "--yes")
        flag_yes=true
        ;;
      "--change-reference-repositories")
        flag_change_reference_repositories=true
        ;;
      "--push")
        flag_push=true
        ;;
      "--delete")
        flag_delete=true
        ;;
      *)
        args+=("$1")
        ;;
    esac
    shift
  done
}

# Parse arguments
parse_common_args "$@"

if [ "$help" ]; then
  show_usage
  exit 0
fi

if [ "$flag_yes" = "" ]; then
  if [ "$flag_change_reference_repositories" ]; then
    read -rp "You are going to change reference repositories. Are you sure to continue? [y/N] " answer

    case "$answer" in
      [yY]* )
        ;;
      * )
        echo -e "\e[33mCanceled.\e[m"
        exit 1
        ;;
    esac
  fi

  if [ "$flag_push" ]; then
    read -rp "You are going to push branches or tags. Are you sure to continue? [y/N] " answer

    case "$answer" in
      [yY]* )
        ;;
      * )
        echo -e "\e[33mCanceled.\e[m"
        exit 1
        ;;
    esac
  fi

  if [ "$flag_delete" ]; then
    read -rp "You are going to delete branches or tags. Are you sure to continue? [y/N] " answer

    case "$answer" in
      [yY]* )
        ;;
      * )
        echo -e "\e[33mCanceled.\e[m"
        exit 1
        ;;
    esac
  fi
fi
