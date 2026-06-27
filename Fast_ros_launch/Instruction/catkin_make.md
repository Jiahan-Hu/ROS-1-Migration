# build ros package

```
# catkin make only one necessary package
catkin_make --only-pkg-with-deps visualization_header_msgs
# catkin make other packages 
catkin_make -DCATKIN_WHITELIST_PACKAGES=""
```

