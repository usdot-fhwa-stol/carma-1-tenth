# CARMA 1tenth
![IMG_1187](https://user-images.githubusercontent.com/49401497/73483195-06a20880-436d-11ea-90c9-7af8032146c0.JPG)


# CARMA devel-1tenth Base Autonomy Package
This branch describes the use, configuration, and installation, for the base autonomy package for the 1Tenth scale vehicle.
The base package supports the following features:

## Features
- TeleOperation
- Mapping
- Localization
- 2D Holuyo LIDAR Perception
- Pure-Pursuit Trajecotry Following
- VESC Low Level Controller

## Hardware Requirements

This document assumes that your hardware build includes a [USB Hokuyo 2D LiDAR](https://www.hokuyo-aut.jp/search/index.php?cate01=1&cate02=1) and a Vedder ESC (a.k.a VESC) [versions 4](https://vesc-project.com/) and above connected to a Jetson XavierNX dev-board.

Verify that the LiDAR and the VESC are able to communicate with the Xavier by issuing the following command:

```shell
user@computer:~$ ls /dev/ttyACM*
```

If the terminals responds that there are two devices on the ACM bus, your hardware has been set-up properly. Carefully note the ACM key value for each device by plugging them in alternatively as this is will be used in the master launch file.

## Teleop Instructions

The following instructions will launch the `move_base` package on the vehicle and the remote computer when using a Logitech 310/710 gaming keypad. After the commands have been issued on the respective machines, use the left up-down axis for throttle, brakes and reverse, and the right sideways axis to control steering. The left and right bumper switches will enable disable autonomous driving respectively.

On vehicle:
```
user@ros-robot: roslaunch move_base move_base.launch
```

On remote computer:
```
user@ros-computer: roslaunch move_base remote_teleop.launch
```

## Mapping Instructions

The `base_mapping` package uses the ROS `hector_mapping` algorithm to create a 2D occupancy grid. Use the following instructions to generate a map of your local environment. Once the commands have been issued, use the joystick to drive your vehicle around the closed-loop area to generate the map and save it on your vehicle.

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

## Localization Instructions

Once you have generated the map of your local environment, you can use the `particle_filter` package to localize your vehicle within the same environment.The majority of parameters you might want to tweak are in the launch/localize.launch file. You may have to modify the `odom_topic` or `scan_topic` parameters to match your vehicle's setup. Use the following command to bring up the localizer on your vehicle.

```
user@ros-robot: roslaunch particle_filter localization.launch
```

## Path following instructions

The folder `pure_pursuit` contains the nodes necessary for a simple path follower. Move the contents of the folder to your master ROS package (generally under `catkin_ws/src`) and compile the workspace again. Drive the car around manually in the known environemt using the localizer and record the wayoints every 5cm in a `csv` file using the `(x_pos, y_pos)` format. Place the genertaed waypoints file in the `/path` folder within this package and point the `trajectory_name` argument in the `find_nearest_goal` node to the waypoints file. Once you have followed the above steps in sequence, issue the following command on your vehicle after launching the `move_base` and `particle_filter` nodes. When ready, use the joystick to place the vehicle in autonomous driving mode to enable the pure-pursuit package to follow the known trajectory.

```console
user@ros-computer: roslaunch <master_ROS_package_name> purepursuit_one_car.launch
```

## Attribution
The development team would like to acknowledge the people who have made direct contributions to the design and code in this repository. [CARMA Attribution](https://github.com/usdot-fhwa-stol/CARMAPlatform/blob/develop/ATTRIBUTION.md)

1. Varundev Suresh Babu, University of Virginia
2. Prof. Madhur Behl, University of Virginia

## License
By contributing to the Federal Highway Administration (FHWA) Connected Automated Research Mobility Applications (CARMA), you agree that your contributions will be licensed under its Apache License 2.0 license. [CARMA License](https://github.com/usdot-fhwa-stol/CARMAPlatform/blob/develop/docs/License.md)
## Contact
Please click on the CARMA logo below to visit the Federal Highway Adminstration(FHWA) CARMA website. For more information, contact CARMA@dot.gov. 
[![image](https://user-images.githubusercontent.com/49401497/73481729-7367d380-436a-11ea-9ba9-c343eb99da82.png)](https://highways.dot.gov/research/research-programs/operations/CARMA)

