#!/usr/bin/env bash

# Notice: This script supposes to be called from release scripts.

# Update vcs versions
echo -e "\e[36mUpdate vcs versions\e[m"
update_vcs_versions

# Commit autoware.proj
git checkout --detach --quiet HEAD
git add "$(get_repos_file_path)"
git commit -m "Update $(basename "$(get_repos_file_path)")"
