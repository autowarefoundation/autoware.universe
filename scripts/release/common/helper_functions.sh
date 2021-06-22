#!/usr/bin/env bash

function get_workspace_root() {
  git rev-parse --show-toplevel
}

function get_repos_file_path() {
  echo "$(get_workspace_root)/autoware.proj.repos"
}

function install_yq_if_not_installed() {
  # Add to PATH
  export PATH=/snap/bin:$PATH

  # Install
  if ! (command -v yq > /dev/null 2>&1); then
    sudo snap install yq
  fi

  # Update
  if ! (yq --version | grep "yq version 4" > /dev/null); then
    sudo snap refresh yq
  fi
}

function is_rc_branch_name() {
  if ! [[ "$1" =~ ^(rc)\/.*$ ]]; then
    return 1
  fi

  return 0
}

function is_experiment_branch_name() {
  if ! [[ "$1" =~ ^(experiment)\/.*$ ]]; then
    return 1
  fi

  return 0
}

function is_on_corresponding_rc_branch() {
  branch_name=$(git rev-parse --abbrev-ref HEAD)

  if ! [[ "$branch_name" =~ ^(rc)\/$1$ ]]; then
    return 1
  fi

  return 0
}

function is_sem_ver() {
  # Use prefix 'v' although it's not correct sem ver
  if ! [[ "$1" =~ ^v([0-9]+)\.([0-9]+)\.([0-9]+)$ ]]; then
    return 1
  fi

  return 0
}

function is_valid_experiment_name() {
  if ! [[ "$1" =~ ^[0-9a-zA-Z][0-9a-zA-Z-]+$ ]]; then
    return 1
  fi

  return 0
}

function is_valid_reference_rc_version() {
  if ! is_sem_ver "$1"; then
    return 1
  fi

  return 0
}

function is_valid_reference_release_version() {
  if ! is_sem_ver "$1"; then
    return 1
  fi

  return 0
}

function is_valid_product_rc_version() {
  # You can customize here in your fork repository
  if ! is_sem_ver "$1"; then
    return 1
  fi

  return 0
}

function is_valid_product_release_version() {
  # You can customize here in your fork repository
  if ! is_sem_ver "$1"; then
    return 1
  fi

  return 0
}

function is_workspace_clean() {
  # .proj
  git diff HEAD --quiet --exit-code || return 1

  # .repos
  vcs custom src --skip-empty --git --args diff HEAD --quiet --exit-code > /dev/null 2>&1 || return 1

  return 0
}

function show_workspace_diff() {
  # .proj
  echo -e "\n[diff of .proj]"
  git diff HEAD --name-only

  # .repos
  echo -e "\n[diff of .repos]"
  vcs custom src --skip-empty --git --args diff HEAD --name-only --exit-code
}

function get_meta_repository() {
  echo "."
}

function get_reference_repositories() {
  echo "src/autoware/autoware.iv"
}

function get_product_repositories() {
  echo "src/autoware/launcher"
}

function get_vcs_repositories() {
  yq e '.repositories' "$(get_repos_file_path)" | grep -E "^[a-zA-Z]+" | sed "s/:.*//g"
}

function update_workspace() {
  # .proj
  git pull --rebase # Ignore errors because no-remote-branch always makes an error

  # .repos
  vcs custom src --git --args remote update || return 1
  vcs import src < "$(get_repos_file_path)" || return 1
  vcs pull src # Ignore errors because tag/hash versions always make errors

  return 0
}

function update_version_in_repos() {
  repository="$1"
  if [ "$repository" = "" ]; then
    echo -e "Please input a repository name as the 1st argument."
    return 1
  fi

  version="$2"
  if [ "$version" = "" ]; then
    echo -e "Please input a version as the 2nd argument."
    return 1
  fi

  yq eval --inplace ".repositories.\"$repository\".version = \"$version\"" "$(get_repos_file_path)"

  return 0
}

function update_vcs_versions() {
  for repository in $(get_vcs_repositories); do
    repo_path="src/$repository"
    git_command="git --work-tree=$repo_path --git-dir=$repo_path/.git"

    branch_name=$($git_command rev-parse --abbrev-ref HEAD)
    commit_description=$($git_command describe --tags 2> /dev/null)
    commit_hash=$($git_command rev-parse HEAD)

    if is_rc_branch_name "$branch_name"; then
      new_version="$branch_name"
    elif is_experiment_branch_name "$branch_name"; then
      new_version="$branch_name"
    elif is_sem_ver "$branch_name"; then
      new_version="$branch_name"
    elif is_sem_ver "$commit_description"; then
      new_version="$commit_description"
    else
      new_version="$commit_hash"
    fi

    echo "$repository: $new_version"
    update_version_in_repos "$repository" "$new_version"
  done
}

function create_branch() {
  repository="$1"
  branch_name="$2"
  flag_push="$3"
  flag_delete="$4"

  git_command="git --work-tree=$repository --git-dir=$repository/.git"

  if [ "$flag_delete" ]; then
    echo -e "Delete branch \"$branch_name\" in \"$repository\"."
    $git_command checkout --detach --quiet HEAD
    $git_command branch -D --quiet "$branch_name"
    return 0
  fi

  echo -e "Create branch \"$branch_name\" in \"$repository\"."
  $git_command checkout --quiet -b "$branch_name" || exit 1

  if [ "$flag_push" ]; then
    echo -e "Push branch \"$branch_name\" to \"$repository\"."
    $git_command push origin "$branch_name"
  fi

  return 0
}

function create_tag() {
  repository="$1"
  version="$2"
  flag_push="$3"
  flag_delete="$4"

  git_command="git --work-tree=$repository --git-dir=$repository/.git"

  if [ "$flag_delete" ]; then
    echo -e "Delete tag \"$version\" in \"$repository\"."
    $git_command tag -d "$version" > /dev/null
    return 0
  fi

  echo -e "Create tag \"$version\" in \"$repository\"."
  $git_command checkout --detach --quiet HEAD
  $git_command tag -a "$version" -m "$version" || exit 1

  if [ "$flag_push" ]; then
    echo -e "Push tag \"$version\" to \"$repository\"."
    $git_command push origin "$version"
  fi

  return 0
}

function checkout_branch_or_tag() {
  repository="$1"
  branch_or_tag_name="$2"

  git_command="git --work-tree=$repository --git-dir=$repository/.git"

  echo -e "Checkout \"$branch_or_tag_name\" in \"$repository\"."
  $git_command checkout --quiet "$branch_or_tag_name" || exit 1

  return 0
}
