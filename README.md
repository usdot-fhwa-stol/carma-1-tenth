<!---
# CARMA 1tenth
 ![IMG_1187](https://user-images.githubusercontent.com/49401497/73483195-06a20880-436d-11ea-90c9-7af8032146c0.JPG)
-->

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

This document assumes that your hardware build includes a USB Hokuyo 2D LiDAR and a Vedder ESC (a.k.a VESC) versions 4 and above connected to a Jetson XavierNX dev-board.

Verify that the LiDAR and the VESC are able to communicate with the Xavier by issuing the following command:

```shell
user@computer:~$ ls /dev/ttyACM* || dmesg | grep -i ACM
```

If the terminals responds that there are two devices on the ACM bus, your hardware has been set-up properly.

## Remote Machine Setup

## Teleop Instructions

## Mapping Instructions

## Localization Instructions

## Path following instructions

## Attribution
The development team would like to acknowledge the people who have made direct contributions to the design and code in this repository. [CARMA Attribution](https://github.com/usdot-fhwa-stol/CARMAPlatform/blob/develop/ATTRIBUTION.md)

1. Varundev SureshBabu, University of Virginia
2. Prof. Madhur Behl, University of Virginia

## License
By contributing to the Federal Highway Administration (FHWA) Connected Automated Research Mobility Applications (CARMA), you agree that your contributions will be licensed under its Apache License 2.0 license. [CARMA License](https://github.com/usdot-fhwa-stol/CARMAPlatform/blob/develop/docs/License.md)
## Contact
Please click on the CARMA logo below to visit the Federal Highway Adminstration(FHWA) CARMA website. For more information, contact CARMA@dot.gov. 
[![image](https://user-images.githubusercontent.com/49401497/73481729-7367d380-436a-11ea-9ba9-c343eb99da82.png)](https://highways.dot.gov/research/research-programs/operations/CARMA)

