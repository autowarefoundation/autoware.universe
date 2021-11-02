#!/bin/bash
# Search for packages that have been modified from the main branch.

set -e

SCRIPT_DIR=$(cd $(dirname $0); pwd)
ROOT_DIR=$(readlink -f $SCRIPT_DIR/../../)

function get_package_name_from_xml() {
  [ "$1" == "" ] && return 1

  # Get <name> tag in package.xml
  sed -rz "s|.*<name>\s*([a-zA-Z_0-9]+)\s*</name>.*|\1|" < "$1"

  return 0
}

function find_package_from_file() {
  [ "$1" == "" ] && return 1

  target_dir=$(dirname $1)
  while true ; do
    parent_dir=$(dirname "$target_dir")

    # Exit if no parent found
    if [ "$parent_dir" = "$target_dir" ] ; then
      return 0
    fi

    # Output package name if package.xml found
    if [ -f "$target_dir/package.xml" ] ; then
      get_package_name_from_xml "$target_dir"/package.xml
      return 0
    fi

    # Move to parent dir
    target_dir=$parent_dir
  done

  return 1
}

# Find modified files after merging origin/main
merge_base=$(git merge-base HEAD origin/main)
modified_files=$(git diff --name-only "$merge_base")

# Find modified packages
for modified_file in $modified_files; do
  modified_package=$(find_package_from_file "$ROOT_DIR/$modified_file")

  if [ "$modified_package" != "" ] ; then
    modified_packages="$modified_packages $modified_package"
  fi
done

# Remove duplicates
# shellcheck disable=SC2086
unique_modified_packages=$(printf "%s\n" $modified_packages | sort | uniq)

# Output
# shellcheck disable=SC2086
echo ::set-output name=package_list::$unique_modified_packages
