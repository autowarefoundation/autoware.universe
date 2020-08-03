#!/usr/bin/env bash

SCRIPT_DIR=$(cd $(dirname $0); pwd)
source $SCRIPT_DIR/helper_functions.sh

# Define functions
function add_tag() {
  repository="$1"
  if [ "$repository" = "" ]; then
    echo -e "Please input a repository name as the 1st argument"
    return 1
  fi

  git_command="git --work-tree=$repository --git-dir=$repository/.git"

  $git_command tag $autoware_version

  if [ "$push" ]; then
    $git_command push origin $autoware_version
  fi

  if [ "$delete" ]; then
    $git_command tag -d $autoware_version > /dev/null
  fi
}

# Pre common tasks
source $SCRIPT_DIR/pre_common_tasks.sh

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
  add_tag $autoware_repository
done

# Post common tasks
source $SCRIPT_DIR/post_common_tasks.sh

# Add tag to autoware.proj
echo -e "\e[36mAdd tag to autoware.proj\e[m"
add_tag .
