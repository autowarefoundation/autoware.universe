#!/usr/bin/env bash

SCRIPT_DIR=$(readlink -f "$(dirname "$0")")
source "$SCRIPT_DIR/common/helper_functions.sh"

# Define functions
function delete_old_rc_branches() {
  repository="$1"
  git_command="git --work-tree=$repository --git-dir=$repository/.git"

  # Show message
  echo -e "\e[32mDelete old RC branches in '$($git_command rev-parse --show-toplevel)'.\e[m"

  # Iterate all branches
  for remote_branch_name in $($git_command branch -r); do
    branch_name=$(echo "$remote_branch_name" | sed "s/origin\///g")
    tag_name=$(echo "$branch_name" | sed "s/rc\///g")

    # Ignore non-RC branches
    if ! is_rc_branch_name "$branch_name"; then
      continue
    fi

    # Find the corresponding tag
    if ! $git_command rev-parse "$tag_name" >/dev/null 2>&1; then
      echo -e "\e[33mNo corresponding tag for $remote_branch_name was not found, skipping.\e[m"
      continue
    fi

    # Show confirmation
    $git_command show --no-patch --no-notes --pretty=format:'%h %s%n%ad %aN%d' "origin/$branch_name"
    printf "\e[31m"
    read -rp "Are you sure to delete origin/$branch_name? [y/N] " answer
    printf "\e[m"

    # Delete if the answer is "yes"
    case $answer in
      [yY]* )
        echo -e "\e[32mRun 'git push --delete origin $branch_name'.\e[m"
        $git_command push --delete origin "$branch_name"
        ;;
      * )
        echo -e "\e[32mSkipped.\e[m"
        continue
        ;;
    esac
  done
}

# Move to workspace root
cd "$(get_workspace_root)" || exit 1

# Delete old RC branches
for repository in $(get_meta_repository) $(get_reference_repositories) $(get_product_repositories); do
  delete_old_rc_branches "$repository"
done
