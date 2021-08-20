#!/bin/bash
# Create comment body for generate-docs.yml
# Usage: create_doc_comment.sh <base_url> <base_branch>

set -e

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

base_url="${args[0]}"
base_branch="${args[1]}"

# Check args
if [ "$base_url" = "" ]; then
  echo -e "\e[31mPlease input a valid base_url as the 1st argument.\e[m"
  exit 1
fi

if [ "$base_branch" = "" ]; then
  echo -e "\e[31mPlease input a valid base_branch as the 2nd argument.\e[m"
  exit 1
fi

# Find modified files from merge-base
modified_files=$(git diff --name-only "$base_branch"...HEAD)

# Find modified markdown files
markdown_paths=()
for modified_file in $modified_files; do
  if ! [[ $modified_file =~ \.md$ ]]; then
    continue
  fi

  markdown_paths+=("${modified_file//docs\//}")
done

# Create comment body
comment_lines=("Documentation URL: $base_url")
comment_lines+=("Modified files:")
for markdown_path in "${markdown_paths[@]}"; do
  if [[ $markdown_path =~ README\.md$ ]]; then
    url_path=${markdown_path/README.md/}
  else
    url_path=${markdown_path/.md/\/}
  fi

  comment_lines+=("- $base_url$url_path")
done

# Workaround for multiline strings
# https://trstringer.com/github-actions-multiline-strings/
comment_body=$(printf "%s\n" "${comment_lines[@]}")
comment_body="${comment_body//'%'/'%25'}"
comment_body="${comment_body//$'\n'/'%0A'}"
comment_body="${comment_body//$'\r'/'%0D'}"

# Output
echo ::set-output name=comment_body::"$(printf "%s\n" "${comment_body[@]}")"
