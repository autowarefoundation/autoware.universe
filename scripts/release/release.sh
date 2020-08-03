#!/usr/bin/env bash

SCRIPT_DIR=$(cd $(dirname $0); pwd)
source $SCRIPT_DIR/release_common_pre.sh

# Check version
if ! is_valid_autoware_version $autoware_version; then
  echo -e "\e[31mPlease input a valid autoware version as the 1st argument\e[m"
  show_usage
  exit 1
fi

# Check if using rc branches
echo -e "\e[36mCheck if using rc branch\e[m"
if ! is_on_rc_branch; then
  echo -e "\e[31mPlease checkout rc branch\e[m"
  exit 1
fi

# Add tags to autoware repositories
echo -e "\e[36mAdd tags to autoware repositories\e[m"
for autoware_repository in $(get_autoware_repositories); do
  git_command="git --work-tree=$autoware_repository --git-dir=$autoware_repository/.git"

  $git_command tag $autoware_version

  if [ "$push" ]; then
    $git_command push origin $autoware_version
  fi

  if [ "$delete" ]; then
    $git_command tag -d $autoware_version > /dev/null
  fi
done

source $SCRIPT_DIR/release_common_post.sh
