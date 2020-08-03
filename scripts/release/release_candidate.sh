#!/usr/bin/env bash

SCRIPT_DIR=$(cd $(dirname $0); pwd)
source $SCRIPT_DIR/helper_functions.sh

# Define functions
function create_rc_branch() {
  repository="$1"
  if [ "$repository" = "" ]; then
    echo -e "Please input a repository name as the 1st argument"
    return 1
  fi

  git_command="git --work-tree=$repository --git-dir=$repository/.git"

  rc_branch=rc/$autoware_version
  $git_command checkout --quiet -b $rc_branch

  if [ "$push" ]; then
    $git_command push origin $rc_branch
  fi

  if [ "$delete" ]; then
    $git_command checkout --detach --quiet HEAD
    $git_command branch -D --quiet $rc_branch
  fi
}

# Pre common tasks
source $SCRIPT_DIR/pre_common_tasks.sh

# Check version
if ! is_valid_autoware_rc_version $autoware_version; then
  echo -e "\e[31mPlease input a valid autoware rc version as the 1st argument\e[m"
  show_usage
  exit 1
fi

# Create rc branches in autoware repositories
echo -e "\e[36mCreate rc branches in autoware repositories\e[m"
for autoware_repository in $(get_autoware_repositories); do
  create_rc_branch $autoware_repository
done

# Pre common tasks
source $SCRIPT_DIR/post_common_tasks.sh

# Create rc branch in autoware.proj
echo -e "\e[36mCreate rc branch in autoware.proj\e[m"
create_rc_branch .
