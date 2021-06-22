#!/usr/bin/env bash

SCRIPT_DIR=$(readlink -f "$(dirname "$0")")
source "$SCRIPT_DIR/common/helper_functions.sh"

# Define functions
function show_usage() {
  echo -e "Usage: create_reference_rc_branches.sh reference_version
      [-h/--help] [-y/--yes] [--change-reference-repositories] [--push|--delete]

    -h/--help:
      Show usage and exit.

    -y/--yes:
      Proceed without confirmation.

    --change-reference-repositories:
      Whether to create branches/tags in reference repositories.

    --push:
      Whether to push branches/tags. Please use this option when you can be sure.

    --delete:
      Whether to delete branches/tags. Please use this option when you mistook something.

    reference_version:
      The version to be used for reference RC branches.
      The valid pattern is '^v([0-9]+)\.([0-9]+)\.([0-9]+)$'.

    Note: Using --push and --delete at the same time may cause unexpected behaviors."
}

# Parse arguments
source "$SCRIPT_DIR/common/parse_common_args.sh"
reference_version="${args[0]}"
product_version="$reference_version" # Use reference version to product repositories as well

# Set default values
if [ "$flag_change_reference_repositories" = "" ]; then
  flag_change_reference_repositories=true
fi

# Check args
if ! is_valid_reference_rc_version "$reference_version"; then
  echo -e "\e[31mPlease input a valid reference RC version as the 1st argument.\e[m"
  show_usage
  exit 1
fi

# Run pre common tasks
source "$SCRIPT_DIR/common/pre_common_tasks.sh"

# Set branch prefix
branch_prefix="rc/"

# Create branches in reference repositories
echo -e "\e[36mCreate branches in autoware repositories\e[m"
for reference_repository in $(get_reference_repositories); do
  if [ "$flag_change_reference_repositories" ]; then
    create_branch "$reference_repository" "$branch_prefix$reference_version" "$flag_push" "$flag_delete"
  else
    checkout_branch_or_tag "$reference_repository" "$branch_prefix$reference_version"
  fi
done

# Create branches in product repositories
echo -e "\e[36mCreate branches in product repositories\e[m"
for product_repository in $(get_product_repositories); do
  create_branch "$product_repository" "$branch_prefix$product_version" "$flag_push" "$flag_delete"
done

# Run post common tasks
if [ "$flag_delete" = "" ]; then
  source "$SCRIPT_DIR/common/post_common_tasks.sh"
fi

# Create branch in meta repository
create_branch "$(get_meta_repository)" "$branch_prefix$product_version" "$flag_push" "$flag_delete"
