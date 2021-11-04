#!/bin/bash
# Search for packages that have been modified from the main branch.
# Usage: get_modified_package.sh <base_branch>

set -e

SCRIPT_DIR=$(readlink -f "$(dirname "$0")")
ROOT_DIR=$(readlink -f "$SCRIPT_DIR"/../)

# Parse arguments
args=()
while [ "${1:-}" != "" ]; do
  case "$1" in
    *)
      args+=("$1")
      ;;
  esac
  shift
done

base_branch="${args[0]}"

# Check args
if [ "$base_branch" = "" ]; then
  echo -e "\e[31mPlease input a valid base_branch as the 1st argument.\e[m"
  exit 1
fi

function find_package_dir() {
  [ "$1" == "" ] && return 1

  target_dir=$(dirname "$1")
  while true ; do
    parent_dir=$(dirname "$target_dir")

    # Exit if no parent found
    if [ "$parent_dir" = "$target_dir" ] ; then
      return 0
    fi

    # Output package name if package.xml found
    if [ -f "$target_dir/package.xml" ] ; then
      if [ ! -f "$target_dir/COLCON_IGNORE" ] ; then
        echo "$target_dir"
        return 0
      fi
    fi

    # Move to parent dir
    target_dir=$parent_dir
  done

  return 1
}

# Find modified files from base branch
modified_files=$(git diff --name-only "$base_branch"...HEAD)

# Find modified packages
modified_package_dirs=()
for modified_file in $modified_files; do
  modified_package_dir=$(find_package_dir "$ROOT_DIR/$modified_file")

  if [ "$modified_package_dir" != "" ] ; then
    modified_package_dirs+=("$modified_package_dir")
  fi
done

# Get package names from paths
if [ "${#modified_package_dirs[@]}" != "0" ] ; then
  modified_packages=$(colcon list --names-only --paths "${modified_package_dirs[@]}")
fi

# Output
# shellcheck disable=SC2086
echo ::set-output name=package_list::$modified_packages
