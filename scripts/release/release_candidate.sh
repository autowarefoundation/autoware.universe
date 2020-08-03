#!/usr/bin/env bash

SCRIPT_DIR=$(cd $(dirname $0); pwd)
source $SCRIPT_DIR/release_common_pre.sh

# Check version
if ! is_valid_autoware_rc_version $autoware_version; then
  echo -e "\e[31mPlease input a valid autoware rc version as the 1st argument\e[m"
  show_usage
  exit 1
fi

# Create rc branches in autoware repositories
echo -e "\e[36mCreate rc branches in autoware repositories\e[m"
for autoware_repository in $(get_autoware_repositories); do
  git_command="git --work-tree=$autoware_repository --git-dir=$autoware_repository/.git"

  rc_branch=rc/$autoware_version
  $git_command checkout --quiet -b $rc_branch

  if [ "$push" ]; then
    $git_command push origin $rc_branch
  fi

  if [ "$delete" ]; then
    $git_command checkout --detach --quiet HEAD
    $git_command branch -d --quiet $rc_branch
  fi
done

source $SCRIPT_DIR/release_common_post.sh
