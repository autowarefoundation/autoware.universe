#!/usr/bin/env bash

SCRIPT_DIR=$(readlink -f "$(dirname "$0")")
source "$SCRIPT_DIR/common/helper_functions.sh"

# Define functions
function show_usage() {
  echo -e "Usage: create_experiment_branches.sh experiment_name
      [-h/--help] [-y/--yes] [--change-reference-repositories] [--delete]

    -h/--help:
      Show usage and exit.

    -y/--yes:
      Proceed without confirmation.

    --change-reference-repositories:
      Whether to create branches/tags in reference repositories.

    --delete:
      Whether to delete branches/tags. Please use this option when you mistook something.

    experiment_name:
      The version to be used for experiment branches.
      The valid pattern is '^v([0-9]+)\.([0-9]+)\.([0-9]+)$'.
    "
}

# Parse arguments
source "$SCRIPT_DIR/common/parse_common_args.sh"
experiment_name="${args[0]}"

# Check args
if ! is_valid_experiment_name "$experiment_name"; then
  echo -e "\e[31mPlease input a valid experiment branch name as the 1st argument.\e[m"
  show_usage
  exit 1
fi

# Run pre common tasks
source "$SCRIPT_DIR/common/pre_common_tasks.sh"

# Set branch prefix
branch_prefix="experiment/"

# Create branches in reference repositories
echo -e "\e[36mCreate branches in autoware repositories\e[m"
for reference_repository in $(get_reference_repositories); do
  create_branch "$reference_repository" "$branch_prefix$experiment_name" "$flag_delete"
done

# Create branches in product repositories
echo -e "\e[36mCreate branches in product repositories\e[m"
for product_repository in $(get_product_repositories); do
  create_branch "$product_repository" "$branch_prefix$experiment_name" "$flag_delete"
done

# Run post common tasks
if [ "$flag_delete" = "" ]; then
  source "$SCRIPT_DIR/common/post_common_tasks.sh"
fi

# Create branch in meta repository
create_branch "$(get_meta_repository)" "$branch_prefix$experiment_name" "$flag_delete"
