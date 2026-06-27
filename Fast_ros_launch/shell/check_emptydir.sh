#!/bin/bash

# 递归函数来检查文件夹是否为空
check_empty_folders() {
  local folder="$1"
  
  # 遍历文件夹中的所有内容
  for item in "$folder"/*; do
    if [ -d "$item" ]; then
      check_empty_folders "$item"  # 递归检查子文件夹
    fi
  done
  
  # 检查当前文件夹是否为空
  if [ ! "$(ls -A "$folder")" ]; then
    echo "empty folder: $folder"
  fi
}

# 检查当前文件夹
check_empty_folders .

echo "check completed"

