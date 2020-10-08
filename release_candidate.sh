#!/usr/bin/env bash

# Parameter
export repos_file="autoware.proj.repos"
export use_commit_description=true

# Install prerequisites
export PATH=/snap/bin:$PATH
if !(command -v yq > /dev/null 2>&1); then
  sudo snap install yq
fi

# Update repositories
vcs import src < "$repos_file"
if [ $? -ne 0 ]; then
  echo -e "\n\e[31mFailed to execute 'vcs import'.\e[m"
  exit 1
fi
vcs pull src

# Replace versions in .repos file
export repositories="$(yq r "$repos_file" repositories | ag "^[a-zA-Z]+" | sed "s/://g")"
for repository in $repositories; do
  export repo_path="src/$repository"
  export git_command="git --work-tree=$repo_path --git-dir=$repo_path/.git"

  export commit_hash=$($git_command rev-parse HEAD)

  if [ "$repository" == "autoware/pilot.auto" ] || [ "$repository" == "autoware/launcher" ]; then
    export new_version=$1
  else
    export new_version=$commit_hash
  fi

  echo "$repository: $new_version"
  yq w --inplace "$repos_file" "repositories.[$repository].version" "$new_version"

done