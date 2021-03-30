#!/usr/bin/env bash

SCRIPT_DIR=$(cd $(dirname $0); pwd)
source $SCRIPT_DIR/helper_functions.sh

# Define functions
function show_usage() {
  echo -e "Usage: create_experiment_branches.sh [--push|--delete] experiment_branch_name
    --push:
      Push branches or tags of autoware repositories. Please use this option when you can be sure.

    --delete:
      Delete branches or tags of autoware repositories. Please use this option when you mistook something.

    experiment_branch_name:
      The version to be used for experiment branches.
      The valid pattern is '^[0-9a-zA-Z][0-9a-zA-Z-]+$'.

    Note: Using --push and --delete at the same time may cause unexpected behaviors."
}

function create_experiment_branch() {
  repository="$1"
  if [ "$repository" = "" ]; then
    echo -e "Please input a repository name as the 1st argument"
    return 1
  fi

  git_command="git --work-tree=$repository --git-dir=$repository/.git"

  experiment_branch=experiment/$autoware_version
  $git_command checkout --quiet -b $experiment_branch

  if [ "$push" ]; then
    $git_command push origin $experiment_branch
  fi

  if [ "$delete" ]; then
    $git_command checkout --detach --quiet HEAD
    $git_command branch -D --quiet $experiment_branch
  fi
}

# Parse arguments
source $SCRIPT_DIR/parse_args.sh

# Check version
if ! is_valid_autoware_experiment_version $autoware_version; then
  echo -e "\e[31mPlease input a valid autoware experiment version as the 1st argument\e[m"
  show_usage
  exit 1
fi

# Pre common tasks
source $SCRIPT_DIR/pre_common_tasks.sh

# Create experiment branches in autoware repositories
echo -e "\e[36mCreate experiment branches in autoware repositories\e[m"
for autoware_repository in $(get_autoware_repositories); do
  create_experiment_branch $autoware_repository
done

# Pre common tasks
source $SCRIPT_DIR/post_common_tasks.sh

# Create experiment branch in autoware.proj
echo -e "\e[36mCreate experiment branch in autoware.proj\e[m"
create_experiment_branch .
