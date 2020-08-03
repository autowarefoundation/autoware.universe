function get_repos_file_path() {
  echo $(git rev-parse --show-toplevel)/"autoware.proj.repos"
}

function install_yq_if_not_installed() {
  PATH=/snap/bin:$PATH
  if !(command -v yq > /dev/null 2>&1); then
    sudo snap install yq
  fi
}

function is_release_branch_name() {
  if ! [[ "$1" =~ ^(rc|release)\/.*$ ]]; then
    return 1
  fi

  return 0
}


function is_sem_ver() {
  if ! [[ "$1" =~ ^v?([0-9]+)\.([0-9]+)\.([0-9]+)$ ]]; then
    return 1
  fi

  return 0
}

function is_valid_autoware_rc_version() {
  if ! [[ "$1" =~ ^v([0-9]+)\.([0-9]+)$ ]]; then
    return 1
  fi

  return 0
}

function is_valid_autoware_version() {
  if ! [[ "$1" =~ ^v([0-9]+)\.([0-9]+)\.([0-9]+)$ ]]; then
    return 1
  fi

  return 0
}

function is_workspace_clean() {
  # .proj
  git diff HEAD --quiet --exit-code || return 1

  # .repos
  vcs custom src --skip-empty --args diff HEAD --quiet --exit-code > /dev/null 2>&1 || return 1

  return 0
}

function show_workspace_diff() {
  # .proj
  echo -e "\n[diff of .proj]"
  git diff HEAD --name-only

  # .repos
  echo -e "\n[diff of .repos]"
  vcs custom src --skip-empty --args diff HEAD --name-only --exit-code
}

function get_autoware_repositories() {
  echo "." "src/autoware/autoware.iv" "src/autoware/launcher"
}

function get_vcs_repositories() {
  echo "$(yq r $(get_repos_file_path) repositories | grep -E "^[a-zA-Z]+" | sed "s/://g")"
}

function is_on_rc_branch() {
  branch_name=$(git rev-parse --abbrev-ref HEAD)

  if ! is_release_branch_name $branch_name; then
    return 1
  fi

  return 0
}

function update_workspace() {
  # .proj
  git pull --rebase || return 1

  # .repos
  vcs custom src --args remote update || return 1
  vcs import src < $(get_repos_file_path) || return 1
  vcs pull src # Ignore errors because tag/hash versions always make errors

  return 0
}

function update_version_in_repos() {
  repository="$1"
  if [ "$repository" = "" ]; then
    echo -e "Please input a repository name as the 1st argument"
    return 1
  fi

  version="$2"
  if [ "$version" = "" ]; then
    echo -e "Please input a version as the 2nd argument"
    return 1
  fi

  yq w --inplace $(get_repos_file_path) "repositories.[$repository].version" "$version"

  return 0
}
