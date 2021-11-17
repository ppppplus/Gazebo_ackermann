# Gazebo Test Field for Ackermann (Support Multiple Robots)

## Tutorials

```shell
roslaunch bringup akm_sim.launch (akm_sim_2robots.launch for 2 robots
```

```shell
roslaunch racecar_control keyboard_teleop.launch robot_name:=akm_* # move your robot!
```

改变模型位置：<http://gazebosim.org/tutorials?tut=ros_comm&cat=connect_ros>

```
rosservice call /gazebo/set_model_state	'{model_state: {model_name: AKM_*, pose: {position: {x: **, y: **, z: **}}, reference_frame: world}}'
```

在本功能包中可以通过

```shell
rosrun racecar_control set_model_state.py AKM_* <x> <y> <theta>
```

改变小车位置，传入的四个参数分别是小车名、相对世界坐标系的横纵坐标和 yaw 角（弧度制）

## Cmake Problems

What to do First: Install necessary packages according to README.md

### 1.

> CMake Error at /usr/local/share/cmake-3.19/Modules/FindQt4.cmake:1314 (message):
> Found unsuitable Qt version "" from NOTFOUND, this code requires Qt 4.x

```shell
sudo apt install cmake gcc g++ qt{4,5}-qmake libqt4-dev	
```

### 2.

> --   No package 'orocos-bfl' found

```shell
sudo apt-get install ros-melodic-bfl
```

### 3.

>Could not load controller 'left_rear_wheel_velocity_controller' because controller type 'effort_controllers/JointVelocityController' does not exist.

```shell
sudo apt-get install ros-melodic-joint-state-controller
sudo apt-get install ros-melodic-effort-controllers
sudo apt-get install ros-melodic-position-controllers
```

其余类似的 robot_state_publisher 也是如此

### 4.

>Could not find a package configuration file provided by "image_transport"
>with any of the following names:
>
>image_transportConfig.cmake
>image_transport-config.cmake

```shell
sudo apt-get install ros-melodic-compressed-image-transport
```

### 5.

> Could not find a package configuration file provided by "tf(eigen)_conversions"

```shell
sudo apt-get install ros-melodic-tf(eigen)-conversions
```




## Debug 

### 1. 

All nodes are in the set namespace, so there is no need to add "/" before the topic name.

 "/" means root directory and "~" means current namespace + node name. 

### 2.

namespace 存在时在 rviz 中显示 "Frame does not exist"，建议检查 tf_tree 中所有的 Frame_id 前是否有 namespace，若没有可以通过以下方式传入：

```shell
<node name="robot_state_publisher" pkg="robot_state_publisher" type="robot_state_publisher">
    <param name="tf_prefix" value="$(arg robot_name)"/>
</node>
```

