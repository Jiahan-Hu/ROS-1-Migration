#!/bin/bash

# Function to recursively delete .git folders
delete_git_folders() {
  local folder="$1"

  # Iterate through all items in the folder
  for item in "$folder"/*; do
    if [ -d "$item" ]; then
      if [ -d "$item/.git" ]; then
        echo "Deleting $item/.git"
        rm -rf "$item/.git"
      fi
      delete_git_folders "$item"  # Recursively delete .git folders in subfolders
    fi
  done
}

# Delete .git folder in the current directory
if [ -d ".git" ]; then
  echo "Deleting .git"
  rm -rf ".git"
fi

# Delete .git folders in the current directory and its subfolders
delete_git_folders .

echo "Deletion completed"

