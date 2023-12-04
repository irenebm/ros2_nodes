# ROS 2 Simulations


## ROSbot XL

### In Gazebo

#### Prerequisites

```shell
todo
```

#### Launch with mapping

```shell
ros2 launch ros2_simulations gazebo_rosbot_xl.launch.py
```

Now you can move around the environment with [Navigation](https://navigation.ros.org/) and create a map!
Click the ``Nav2 Goal`` button and choose a destination.

![rosbotxl-gazebo](https://github.com/irenebm/ros2_nodes/blob/iron/.figures/ros2_simulations/preview_gazebo_rosbot_xl.png)

#### Launch with localization

```shell
ros2 launch ros2_simulations gazebo_rosbot_xl.launch.py slam:=False
```

### In Webots

#### Prerequisites

```shell
git clone --recurse-submodules https://github.com/husarion/webots_ros2.git -b develop-husarion
```

#### Launch with mapping

```shell
ros2 launch ros2_simulations webots_rosbot_xl.launch.py
```

Now you can move around the environment with [Navigation](https://navigation.ros.org/) and create a map!
Click the ``Nav2 Goal`` button and choose a destination.

![rosbotxl-webots](https://github.com/irenebm/ros2_nodes/blob/iron/.figures/ros2_simulations/preview_webots_rosbot_xl.png)

#### Launch with localization

```shell
ros2 launch ros2_simulations webots_rosbot_xl.launch.py slam:=False
```
