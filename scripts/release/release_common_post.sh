#!/usr/bin/env bash

SCRIPT_DIR=$(cd $(dirname $0); pwd)
source $SCRIPT_DIR/helper_functions.sh

# Update vcs versions
echo -e "\e[36mUpdate vcs versions\e[m"
for repository in $(get_vcs_repositories); do
  repo_path="src/$repository"
  git_command="git --work-tree=$repo_path --git-dir=$repo_path/.git"

  branch_name=$($git_command rev-parse --abbrev-ref HEAD)
  commit_description=$($git_command describe --tags 2> /dev/null)
  commit_hash=$($git_command rev-parse HEAD)

  if is_sem_ver $commit_description; then
    new_version=$commit_description
  elif is_release_branch_name $branch_name; then
    new_version=$branch_name
  else
    new_version=$commit_hash
  fi

  echo "$repository: $new_version"
  update_version_in_repos $repository $new_version
done
