# Package Name

## Overview

Create a local grid map based on stereo images.

**Keywords:** local, map, grid

### License

The source code is released under a [BSD 3-Clause license](https://github.com/AnthonyCvn/...).

**Author: Anthony Cavin**

The local_grid_map package has been tested under [ROS] kinetic and Ubuntu 16.04.

## Installation

Reference page to install ROS kinetic on Ubuntu:

[Robot Operating System (ROS)](http://wiki.ros.org/kinetic/Installation/Ubuntu)

#### Dependencies

- [Robot Operating System (ROS)](http://wiki.ros.org) (middleware for robotics),
- [image_transport](http://wiki.ros.org/image_transport) (C++ library for images transport),
- [grid_map](https://github.com/ethz-asl/grid_map) (C++ library with ROS interface to manage grid maps),
- [OpenCV](https://opencv.org/) (library for computer vision)

#### Building

To build from source, clone the latest version from this repository into your catkin workspace and compile the package using

	cd catkin_workspace/src
	git clone https://github.com/AnthonyCvn/local_grid_map.git
	cd ../
	catkin build local_grid_map

### Unit Tests

Run the unit tests with

	catkin run_tests local_grid_map

## Usage

Run the main node with

	roslaunch local_grid_map default.launch

## Launch files

* **default.launch:** Launch a default local grid map.
    
    Argument set 1

    - **`map_frame_id`** . Default: `/map0`.

    Argument set 2
	
     - **`image_topic`** left image topic (Theora compressed). Default: `/camera0/camera_capture/camera/image`.

    Argument set 4
	
     - **`publish_rate`** Rate of publishing the local grid map in Hz. Default: `10`.

## Nodes

### local_grid_map

Create a local grid map based on stereo images.

#### Subscribed Topics

* **`/camera0/camera_capture/camera/image/theora`** ([theora_image_transport/Packet])

	Image topic.

#### Published Topics

* **`/image`** ([sensor_msgs/Image])

	The images captured with the camera in raw format.

#### Services

* **`trigger`** ([camera_capture/SetCamera])

	Trig the local grid map publisher.

		rosservice call /local_grid_map/trigger


## Bugs & Feature Requests

Please report bugs and request features using the [Issue Tracker](https://github.com/AnthonyCvn/local_grid_map/issues).


[ROS]: http://www.ros.org
[rviz]: http://wiki.ros.org/rviz
[theora_image_transport/Packet]: http://docs.ros.org/api/theora_image_transport/html/msg/Packet.html
