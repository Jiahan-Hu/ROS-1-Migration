#!/bin/bash

# Find all folders containing .git files
git_folders=$(find . -type d -name ".git")

# Output the locations of the found .git folders
echo "The following folders contain .git files:"
echo "$git_folders"

# Ask the user if they want to delete these .git folders
read -p "Do you want to delete these .git folders? (y/n): " choice

if [ "$choice" == "y" ]; then
    # User chose to delete
    for folder in $git_folders; do
        echo "Deleting $folder"
        rm -rf "$folder"
    done
    echo ".git folders have been deleted"
elif [ "$choice" == "n" ]; then
    # User chose not to delete
    echo ".git folders have not been deleted"
else
    # Invalid input
    echo "Invalid choice, .git folders have not been deleted"
fi
