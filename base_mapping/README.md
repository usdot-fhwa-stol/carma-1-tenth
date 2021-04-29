### F1/10 Base Mapping
F1/10 Mapping based on Hector SLAM

Run locally on robot
```
user@ros-robot: roslaunch base_mapping base_mapping.launch
```

Visualize on remote computer
```
user@ros-computer: roslaunch base_mapping visualize.launch
```

Once satisfied, save map on robot
```
user@ros-robot: roscd base_mapping/map
user@ros-robot: rosrun map_server map_saver -f base_map
```
